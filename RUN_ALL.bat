@echo off
echo ================================================================
echo  FEAD Belt Drive Test Rig  -  MATLAB R2026a Suite
echo  H6 OEM Engine  .  Simscape / Simulink / App
echo ================================================================
echo.
echo What would you like to run?
echo.
echo  [1] FEAD_App        - Full interactive app + animation (RECOMMENDED)
echo  [2] run_fead_sim    - FEAD simulation plots only
echo  [3] run_truck_sim   - Full truck simulation plots
echo  [4] FEAD_PostProcess- Post-process test rig results
echo  [5] build_FEAD_model- Build FEAD_BeltDrive.slx (needs Simscape)
echo  [6] build_truck_model Build H6_Truck_System.slx (needs Simscape)
echo  [7] ALL             - Run everything in sequence
echo.
set /p choice="Enter choice [1-7]: "

set MATLAB="C:\Program Files\MATLAB\R2026a\bin\matlab.exe"
set DIR=C:\Users\RICARDO\fead-matlab

if "%choice%"=="1" (
    %MATLAB% -sd "%DIR%" -r "startup_fead; FEAD_App"
    goto end
)
if "%choice%"=="2" (
    %MATLAB% -sd "%DIR%" -r "startup_fead; run_fead_sim"
    goto end
)
if "%choice%"=="3" (
    %MATLAB% -sd "%DIR%" -r "startup_fead; run_truck_sim"
    goto end
)
if "%choice%"=="4" (
    %MATLAB% -sd "%DIR%" -r "startup_fead; FEAD_PostProcess"
    goto end
)
if "%choice%"=="5" (
    %MATLAB% -sd "%DIR%" -r "startup_fead; build_FEAD_model"
    goto end
)
if "%choice%"=="6" (
    %MATLAB% -sd "%DIR%" -r "startup_fead; build_truck_model"
    goto end
)
if "%choice%"=="7" (
    %MATLAB% -sd "%DIR%" -r "startup_fead; run_fead_sim; run_truck_sim; FEAD_PostProcess; FEAD_App"
    goto end
)

echo Invalid choice. Launching FEAD_App by default...
%MATLAB% -sd "%DIR%" -r "startup_fead; FEAD_App"

:end
pause
