@echo off

set PORT=COM5

mode %PORT%:1200 > nul
timeout /T 1 > nul
bossac --port=%PORT% -U false -e -w -v -b MdkController.bin -R

pause
