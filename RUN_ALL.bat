@echo off
echo ============================================================
echo  Gates FEAD + H6 Truck  –  MATLAB R2026a Auto-Runner
echo ============================================================
echo.

set MATLAB_EXE="C:\Program Files\MATLAB\R2026a\bin\matlab.exe"
set PROJECT_DIR=C:\Users\RICARDO\fead-matlab

echo Launching MATLAB R2026a with FEAD project...
echo Project folder: %PROJECT_DIR%
echo.
echo MATLAB will:
echo   1. Load all parameters   (FEAD_params)
echo   2. Run FEAD simulation   (run_fead_sim)   -> 6 plots
echo   3. Run Truck simulation  (run_truck_sim)  -> 9 plots
echo   4. Open Layout Editor    (layout_editor)  -> GUI
echo.
echo ============================================================

%MATLAB_EXE% -sd "%PROJECT_DIR%" -r "startup_fead; run_fead_sim; run_truck_sim; layout_editor"

pause
