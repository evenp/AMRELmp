@echo off
cd ..\
rmdir /S /Q .\deps\shapelib\build >nul
rmdir .\src\Libs /S /Q >nul
rmdir .\binaries /S /Q >nul
rmdir .\intermediate /S /Q >nul
del /f /s /q .\.vs 1>nul
rmdir /s /q .vs >nul
del AMREL.sln >nul
del AMREL.vcxproj >nul
del AMREL.vcxproj.filters >nul
del AMREL.vcxproj.user >nul
pause
