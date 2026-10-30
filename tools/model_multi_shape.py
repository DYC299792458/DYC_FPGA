"""多目标识别器的几何特征参考模型，用于离线调阈值。"""
from math import inf


def classify(rows, min_edge=20, eps=2):
    pts = [(x, y) for y, xs in enumerate(rows) for x in xs]
    if not pts:
        return 0, {}
    minx = min(x for x, _ in pts); maxx = max(x for x, _ in pts)
    miny = min(y for _, y in pts); maxy = max(y for _, y in pts)
    spans = []
    lefts = []
    for xs in rows:
        if xs:
            lefts.append(min(xs)); spans.append(max(xs)-min(xs)+1)
    turns = 0; direction = 0; variation = 0
    for a, b in zip(spans, spans[1:]):
        variation += abs(b-a)
        new_dir = 1 if b > a+eps else (2 if a > b+eps else direction)
        if direction and new_dir != direction:
            turns += 1
        direction = new_dir
    fill = sum(spans)
    ratio = fill*256//((maxx-minx+1)*(maxy-miny+1))
    bw=maxx-minx+1; bh=maxy-miny+1
    left_shift=abs(lefts[0]-lefts[-1]); end_delta=abs(spans[0]-spans[-1])
    edge_count=len(pts)
    if edge_count < min_edge: shape=0
    elif turns >= 2: shape=5
    elif left_shift >= 3 and end_delta <= bw//4 and variation <= bh*2: shape=4
    elif ratio >= 220: shape=2
    elif ratio >= 155: shape=1
    elif ratio >= 75: shape=3
    else: shape=0
    return shape, dict(edge=edge_count, ratio=ratio, turns=turns,
                       variation=variation, left_shift=left_shift,
                       end_delta=end_delta, box=(minx,miny,maxx,maxy))


def make_rows(kind, lane, width=96, height=72):
    cx=lane*32+16
    rows=[]
    for y in range(height):
        xs=[]
        for x in range(lane*32,(lane+1)*32):
            dx=x-cx; dy=y-36; adx=abs(dx); ady=abs(dy); hit=False
            if kind==1:
                d2=dx*dx+dy*dy; hit=11*11 <= d2 <= 13*13
            elif kind==2:
                hit=adx<=11 and ady<=15 and (adx>=10 or ady>=14)
            elif kind==3 and 20<=y<=52:
                hw=12*(y-20)//32
                hit=x in (cx-hw,cx+hw) or (y>=51 and cx-hw<=x<=cx+hw)
            elif kind==4 and 20<=y<=52:
                left=cx-12+(y-20)//4; right=left+20
                hit=x in (left,right) or (y in (20,52) and left<=x<=right)
            elif kind==5 and 0<=y-18<=36:
                ay=y-18
                if ay<=7: hw=1+ay
                elif ay<=13: hw=20-ay
                elif ay<=18: hw=7+(ay-13)*2
                elif ay<=25: hw=17-(ay-18)
                else: hw=10-(ay-25)*8//11
                hit=x in (cx-hw,cx+hw)
            if hit: xs.append(x)
        rows.append(xs)
    return rows


if __name__ == "__main__":
    expected={1:1,2:2,3:3,4:4,5:5}
    for kind,want in expected.items():
        got,feat=classify(make_rows(kind,1))
        print(f"kind={kind} got={got} features={feat}")
        assert got==want, (kind,got,feat)
    print("ALL REFERENCE TESTS PASSED")
