@ECHO off
SETLOCAL enabledelayedexpansion
COLOR B

REM CHECK ADMIN MODE
net session >NUL 2>&1
IF NOT %errorLevel% == 0 (
	REM REQUEST ADMIN MODE IF NEEDED
	if not "%1"=="am_admin" (powershell start -verb runas '%0' am_admin & exit /b)
)

REM SET GLOBALS
FOR /F "tokens=* USEBACKQ" %%F IN (`where cmake`) DO SET CMAKEPATH=%%F
CD /D %~dp0

SET ROOT=%cd%
SET AMRELPATH=%cd%
FOR /F "usebackq tokens=*" %%i IN (`tools\vswhere -latest -requires Microsoft.Component.MSBuild -find MSBuild\**\Bin\MSBuild.exe`) DO SET VSPATH=%%i

REM CHECK PATHS
IF NOT DEFINED VSPATH (
	COLOR 4
	ECHO "failed to locate msbuild tools."
	PAUSE
	EXIT 2
)

IF NOT DEFINED CMAKEPATH (
	COLOR 4
	echo "failed to locate cmake. Please set cmake path in environment variable."
	PAUSE
	EXIT 3
)

REM BUILD AMREL
echo compiling AMREL for Release x64...
"%VSPATH%" %AMRELPATH%\AMREL.vcxproj /t:build /p:Configuration="Release" /p:Platform="x64" /p:BuildInParallel=true /p:OutDir=%AMRELPATH%\binaries\AMREL\Release
IF NOT %errorLevel% == 0 ECHO failed to compile AMREL.

REM echo compiling AMREL for Debug x64...
REM "%VSPATH%" %ILSDPATH%\AMREL.vcxproj /t:build /p:Configuration="Debug" /p:Platform="x64" /p:BuildInParallel=true /p:OutDir=%AMRELPATH%\binaries\AMREL\Debug
REM IF NOT %errorLevel% == 0 ECHO failed to compile AMREL.

pause

#echo STARTING AMREL...
#cd %AMRELPATH%\resources
#..\binaries\AMREL\Release\AMREL.exe
#pause
exit 0
