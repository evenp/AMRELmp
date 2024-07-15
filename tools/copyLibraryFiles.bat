@echo off

mkdir ..\src\Libs\ 2> NUL

REM COPYING STB
echo -------STB-------
rmdir ..\src\Libs\stbi\ /s /q 2> NUL
mkdir ..\src\Libs\stbi\ 2> NUL
copy ..\deps\stb\stb_image.h ..\src\Libs\stbi
copy ..\deps\stb\stb_image_write.h ..\src\Libs\stbi


REM COPYING SHAPELIB's DLL
echo -------SHAPELIB's DLL-------
mkdir ..\binaries 2> NUL
mkdir ..\binaries\ILSD\Release 2> NUL
mkdir ..\binaries\ILSD\Debug 2> NUL
copy ..\deps\shapelib\build\Debug\shp.dll ..\binaries\AMREL\Release
copy ..\deps\shapelib\build\Debug\shp.dll ..\binaries\AMREL\Debug




pause
