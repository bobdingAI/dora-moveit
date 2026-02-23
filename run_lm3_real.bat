@echo off
echo ========================================
echo LM3 Real Robot Control Mode
echo ========================================
echo.
echo WARNING: This will control the PHYSICAL LM3 robot!
echo.
echo Please ensure:
echo   1. Robot is powered on
echo   2. Workspace is clear
echo   3. Emergency stop is accessible
echo   4. Robot IP is configured (default: 10.20.17.1)
echo.
set /p confirm="Type 'yes' to continue: "
if /i not "%confirm%"=="yes" (
    echo Cancelled.
    pause
    exit /b
)

echo.
echo Starting Dora daemon...
dora up

echo.
echo Building and starting LM3 real robot dataflow...
dora start config/dataflow_lm3_real.yml --attach

echo.
echo Robot control stopped.
pause
