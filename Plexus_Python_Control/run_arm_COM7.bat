@echo off
cd /d "%~dp0"
.venv\Scripts\python.exe -m plexus app --hardware --port COM7 --manual --web-port 8090 --gripper stock --session manual_COM7
pause
