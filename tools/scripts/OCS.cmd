@echo off
setlocal
rem Delegate to the unchanged sibling team OCS checkout.
set "ROOT=%~dp0..\.."
set "OCS_ROOT=%ROOT%\..\team-ocs"
if not exist "%OCS_ROOT%\OCS.cmd" set "OCS_ROOT=%ROOT%\..\rx26_ocs"
if not exist "%OCS_ROOT%\OCS.cmd" (
  echo Team OCS checkout not found beside the GRAEY repository.
  echo Clone https://github.com/InspirationRobotics/rx26_ocs.git into team-ocs.
  exit /b 2
)
if exist "%OCS_ROOT%\.venv\Scripts\python.exe" set "PATH=%OCS_ROOT%\.venv\Scripts;%PATH%"
if exist "%OCS_ROOT%\run\laptop.toml" (
  call "%OCS_ROOT%\OCS.cmd" --config run\laptop.toml %*
) else (
  call "%OCS_ROOT%\OCS.cmd" %*
)
exit /b %errorlevel%
