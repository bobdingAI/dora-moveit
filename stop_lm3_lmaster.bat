@echo off
echo Stopping L-Master dataflow...
dora destroy

echo Stopping L-Master container...
docker stop lebai-master

echo Done.
pause
