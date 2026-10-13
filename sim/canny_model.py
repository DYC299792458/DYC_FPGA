#!/usr/bin/env python3
"""
canny_model.py  Canny 金标准模型 (与 src/canny/canny_edge.v 定点算法逐位一致)

用法:
  python3 canny_model.py gen  <W> <H> <th_high> <th_low> <outdir>   生成测试图 + 期望结果
  python3 canny_model.py check <outdir>                             对比 RTL 仿真输出

测试图: 渐变背景 + 实心圆 + 矩形 + 三角形 + 细线 + 椒盐噪声, 覆盖 4 个梯度方向和弱边缘连接。
RTL 每级窗口输出相对输入平移 (1,1), 4 级共 (4,4); 比较时去掉 MARGIN 像素的边界(窗口钳位区)。
"""
import os
import random
import sys

MARGIN = 6


def clamp_get(img, w, h, x, y):
    x = min(max(x, 0), w - 1)
    y = min(max(y, 0), h - 1)
    return img[y][x]


def win(img, w, h, x, y):
    """3x3 窗口, 返回 p11..p33 (行优先)"""
    return [clamp_get(img, w, h, x + dx, y + dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1)]


def gauss(img, w, h):
    out = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            p = win(img, w, h, x, y)
            corner = p[0] + p[2] + p[6] + p[8]
            edge = p[1] + p[3] + p[5] + p[7]
            out[y][x] = (corner + 2 * edge + 4 * p[4] + 8) >> 4
    return out


def grad(img, w, h):
    mag = [[0] * w for _ in range(h)]
    dr = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            p11, p12, p13, p21, p22, p23, p31, p32, p33 = win(img, w, h, x, y)
            gx_p = p13 + 2 * p23 + p33
            gx_n = p11 + 2 * p21 + p31
            gy_p = p31 + 2 * p32 + p33
            gy_n = p11 + 2 * p12 + p13
            ax, ay = abs(gx_p - gx_n), abs(gy_p - gy_n)
            sx, sy = gx_p >= gx_n, gy_p >= gy_n
            mag[y][x] = ax + ay
            if ay * 256 <= ax * 106:
                d = 0
            elif ax * 256 < ay * 106:
                d = 2
            elif sx == sy:
                d = 1
            else:
                d = 3
            dr[y][x] = d
    return mag, dr


def nms(mag, dr, w, h, th_hi, th_lo):
    cls = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            p = win(mag, w, h, x, y)
            m = p[4]
            d = clamp_get(dr, w, h, x, y)
            na, nb = {0: (p[3], p[5]), 1: (p[0], p[8]), 2: (p[1], p[7]), 3: (p[2], p[6])}[d]
            keep = m > na and m >= nb
            if keep and m >= th_hi:
                cls[y][x] = 2
            elif keep and m >= th_lo:
                cls[y][x] = 1
    return cls


def hyst(cls, w, h):
    out = [[0] * w for _ in range(h)]
    for y in range(h):
        for x in range(w):
            p = win(cls, w, h, x, y)
            strong_nb = any(p[i] == 2 for i in (0, 1, 2, 3, 5, 6, 7, 8))
            out[y][x] = 1 if (p[4] == 2 or (p[4] == 1 and strong_nb)) else 0
    return out


def canny(img, w, h, th_hi, th_lo):
    g = gauss(img, w, h)
    m, d = grad(g, w, h)
    c = nms(m, d, w, h, th_hi, th_lo)
    return hyst(c, w, h)


def make_image(w, h, seed=7):
    rnd = random.Random(seed)
    img = [[(x * 3 + y * 2) // 4 + 20 for x in range(w)] for y in range(h)]
    cx, cy, r = w * 0.28, h * 0.5, min(w, h) * 0.22
    for y in range(h):
        for x in range(w):
            if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                img[y][x] = 200
            if w * 0.55 <= x <= w * 0.78 and h * 0.15 <= y <= h * 0.45:
                img[y][x] = 230
            # 三角形 (斜边覆盖 45/135 度方向)
            ty0, ty1 = h * 0.55, h * 0.9
            if ty0 <= y <= ty1:
                half = (y - ty0) / (ty1 - ty0) * w * 0.12
                if abs(x - w * 0.72) <= half:
                    img[y][x] = 40
            if x == int(w * 0.92):              # 细竖线
                img[y][x] = 160
            img[y][x] = min(255, max(0, img[y][x] + rnd.randint(-6, 6)))
    for _ in range(w * h // 60):                # 椒盐噪声
        img[rnd.randrange(h)][rnd.randrange(w)] = rnd.choice((0, 255))
    return img


def gen(w, h, th_hi, th_lo, outdir):
    os.makedirs(outdir, exist_ok=True)
    img = make_image(w, h)
    rnd = random.Random(99)
    side = [[rnd.randint(0, 1) for _ in range(w)] for _ in range(h)]
    with open(os.path.join(outdir, "canny_in.hex"), "w") as f:
        for y in range(h):
            for x in range(w):
                f.write("%03x\n" % ((side[y][x] << 8) | img[y][x]))
    exp = canny(img, w, h, th_hi, th_lo)
    with open(os.path.join(outdir, "canny_meta.txt"), "w") as f:
        f.write("%d %d %d %d\n" % (w, h, th_hi, th_lo))
    with open(os.path.join(outdir, "canny_exp.txt"), "w") as f:
        for y in range(h):
            f.write("".join(str(v) for v in exp[y]) + "\n")
    with open(os.path.join(outdir, "canny_side.txt"), "w") as f:
        for y in range(h):
            f.write("".join(str(v) for v in side[y]) + "\n")
    n_edge = sum(map(sum, exp))
    print("generated %dx%d image, th_high=%d th_low=%d, expected edge pixels=%d" % (w, h, th_hi, th_lo, n_edge))


def check(outdir):
    w, h, th_hi, th_lo = map(int, open(os.path.join(outdir, "canny_meta.txt")).read().split())
    exp = [list(map(int, l.strip())) for l in open(os.path.join(outdir, "canny_exp.txt"))]
    side = [list(map(int, l.strip())) for l in open(os.path.join(outdir, "canny_side.txt"))]
    rtl = [l.split() for l in open(os.path.join(outdir, "canny_rtl.txt")) if l.strip()]
    if len(rtl) != w * h:
        print(" [FAIL] RTL output pixel count %d, expect %d" % (len(rtl), w * h))
        return 1
    S = 4   # 4 级窗口, 每级平移 (1,1)
    e_err = s_err = n = n_edge = 0
    for y in range(MARGIN + S, h - MARGIN + S):
        for x in range(MARGIN + S, w - MARGIN + S):
            if y >= h or x >= w:
                continue
            e, s = map(int, rtl[y * w + x])
            n += 1
            n_edge += e
            if e != exp[y - S][x - S]:
                e_err += 1
                if e_err <= 8:
                    print("   edge mismatch at out(%d,%d): rtl=%d model=%d" % (x, y, e, exp[y - S][x - S]))
            if s != side[y - S][x - S]:
                s_err += 1
    print(" compared %d interior pixels (%d edge pixels): edge mismatch=%d, side-bit mismatch=%d"
          % (n, n_edge, e_err, s_err))
    if e_err == 0 and s_err == 0 and n_edge > 0:
        print(" RESULT: ALL CHECKS PASSED")
        return 0
    print(" RESULT: FAILED")
    return 1


if __name__ == "__main__":
    if sys.argv[1] == "gen":
        gen(int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), sys.argv[6])
    else:
        sys.exit(check(sys.argv[2]))
