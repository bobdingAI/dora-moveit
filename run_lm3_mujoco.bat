@echo off
echo ========================================
echo LM3 MuJoCo Simulation Mode
echo ========================================
echo.
echo Starting Dora daemon...
dora up

echo.
echo Building and starting LM3 simulation dataflow...
dora start config/dataflow_lm3_mujoco.yml --attach

echo.
echo Simulation stopped.
pause
