@echo off
rem GRAEY_GUI.cmd: double-click to start the ground station on this laptop.
rem Runs robotx_graey_2026\api\gui\laptop\graey_app.py (standard library
rem only) with the repo's .venv python if there is one, else "python". It connects to the
rem GRAEY TCP bridge at graey.local (GRAEY_HOST or --graey overrides this).
rem Opens http://localhost:8152. Extra arguments go to the copied gcs_app CLI.
rem Close this window to stop it.
setlocal
set "ROOT=%~dp0..\.."
set "PY=python"
if exist "%ROOT%\.venv\Scripts\python.exe" set "PY=%ROOT%\.venv\Scripts\python.exe"
if not defined GRAEY_HOST set "GRAEY_HOST=graey.local"
set "CRSD_BOAT=%GRAEY_HOST%"
curl.exe -sf -o nul -m 1 http://127.0.0.1:8152/ && (start "" http://localhost:8152/ & exit /b 0)
title GRAEY GCS
"%PY%" "%ROOT%\robotx_graey_2026\api\gui\laptop\graey_app.py" %*
if errorlevel 1 pause
