@echo off
echo ========================================
echo LM3 L-Master Simulation Mode
echo ========================================
echo.

REM Start Dora daemon
echo Starting Dora daemon...
dora up

REM Start dataflow
echo.
echo Building and starting LM3 L-Master dataflow...
dora start config/dataflow_lm3_lmaster.yml

echo.
echo Dataflow stopped.
echo.
echo To stop L-Master container, run: docker stop lebai-master
pause
