$ErrorActionPreference = 'Stop'

# Replicates the geometry used by tb_shape_recognizer.v (128x96, full-frame ROI)
# so the expected classification in the testbench can be confirmed independently.
# Mirrors the Verilog functions exactly, including integer division.

$W = 128; $H = 96
$MIN_EDGE = 20
$RECT_Q = 230; $CIRC_Q = 168; $TRI_Q = 96

function Classify($ratio_q, $edge) {
  if ($edge -lt $MIN_EDGE) { return 'NONE (0)' }
  if ($ratio_q -ge $RECT_Q) { return 'RECT (2)' }
  if ($ratio_q -ge $CIRC_Q) { return 'CIRCLE (1)' }
  if ($ratio_q -ge $TRI_Q)  { return 'TRIANGLE (3)' }
  return 'NONE (0)'
}

function Run-Case($name, $kind) {
  $minx = 99999; $maxx = -1; $miny = 99999; $maxy = -1
  $edge = 0; $fill = 0

  for ($y = 0; $y -lt $H; $y++) {
    $first = -1; $last = -1
    for ($x = 0; $x -lt $W; $x++) {
      $dx = $x - 64
      $dy = $y - 48
      $on = $false

      if ($kind -eq 'circle') {
        $d2 = ($dx * $dx) + ($dy * $dy)
        if ($d2 -ge 900 -and $d2 -le 1156) { $on = $true }
      }
      elseif ($kind -eq 'rect') {
        $adx = [math]::Abs($dx); $ady = [math]::Abs($dy)
        if ($adx -le 32 -and $ady -le 24) {
          if (((32 - $adx) -lt 2) -or ((24 - $ady) -lt 2)) { $on = $true }
        }
      }
      elseif ($kind -eq 'tri') {
        $top = 48 - 24; $bot = 48 + 24
        if ($y -ge $top -and $y -le $bot) {
          # Verilog integer division truncates toward zero; y-top >= 0 so both agree
          $lx = 64 - [int][math]::Truncate((32 * ($y - $top)) / ($bot - $top))
          $rx = 64 + [int][math]::Truncate((32 * ($y - $top)) / ($bot - $top))
          if ((($x -ge ($lx - 2)) -and ($x -le ($lx + 2))) -or
              (($x -ge ($rx - 2)) -and ($x -le ($rx + 2)))) { $on = $true }
          if ((($bot - $y) -lt 2) -and ($x -ge ($lx - 2)) -and ($x -le ($rx + 2))) { $on = $true }
        }
      }

      if ($on) {
        $edge++
        if ($first -lt 0) { $first = $x }
        $last = $x
        if ($x -lt $minx) { $minx = $x }
        if ($x -gt $maxx) { $maxx = $x }
        if ($y -lt $miny) { $miny = $y }
        if ($y -gt $maxy) { $maxy = $y }
      }
    }
    if ($first -ge 0) { $fill += ($last - $first + 1) }
  }

  if ($maxx -lt 0) {
    Write-Output ("{0,-10} edge={1,-6} area={2,-7} box={3,-12} ratio_q={4,-5} -> {5}" -f $name, $edge, $fill, '-', '-', 'NONE (0)')
    return
  }
  $bw = $maxx - $minx + 1
  $bh = $maxy - $miny + 1
  $ratio_q = [int][math]::Floor(($fill * 256) / ($bw * $bh))
  Write-Output ("{0,-10} edge={1,-6} area={2,-7} box={3,-12} ratio_q={4,-5} -> {5}" -f `
      $name, $edge, $fill, "$bw x $bh", $ratio_q, (Classify $ratio_q $edge))
}

Run-Case 'circle'    'circle'
Run-Case 'rect'      'rect'
Run-Case 'triangle'  'tri'
Run-Case 'empty'     'none'
