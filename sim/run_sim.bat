@echo off
rem ====================================================================
rem  Sobel unit simulation (ModelSim)
rem  Compiles only the Sobel / key related RTL files, no Efinity IP model.
rem  NOTE: keep this file ASCII only - Chinese text inside a .bat breaks
rem        cmd.exe parsing under the GBK code page.
rem ====================================================================
pushd "%~dp0"

set SRC=..\src\Sobeledge_8d\hdl

vlib work
vlog -work work %SRC%\shift_regs.v %SRC%\extract_region_c_shiftram_xtap.v %SRC%\Sobeledge_matrix_3x3.v %SRC%\Sobeledge_proc.v ..\src\sobel_ctrl.v ..\src\median_3x3.v tb_Sobeledge_8d_proc.v tb_sobel_ctrl.v tb_median_3x3.v

if errorlevel 1 goto compile_error

echo.
echo ===== test 1/3 : Sobeledge_8d_proc (algorithm + threshold) =====
vsim -c -do "run -all; quit -f" work.tb_Sobeledge_8d_proc

echo.
echo ===== test 2/3 : sobel_ctrl (2 keys: threshold +/- and display mode) =====
vsim -c -do "run -all; quit -f" work.tb_sobel_ctrl

echo.
echo ===== test 3/3 : median_3x3 (3x3 median filter) =====
vsim -c -do "run -all; quit -f" work.tb_median_3x3
goto end

:compile_error
echo.
echo *** COMPILE FAILED, see the errors above ***

:end
popd
pause
