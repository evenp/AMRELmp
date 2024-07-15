@echo off
mkdir resources 2> Nul
mkdir resources\asc 2> Nul
mkdir resources\steps 2> Nul
mkdir resources\config 2> Nul
mkdir resources\exports 2> Nul
mkdir resources\tilesets 2> Nul
mkdir resources\nvm 2> Nul
mkdir resources\til 2> Nul
mkdir resources\til\top 2> Nul
mkdir resources\til\mid 2> Nul
mkdir resources\til\eco 2> Nul
mkdir resources\xyz 2> Nul
"tools/premake5.exe" --file=src/premakeFile.lua vs2019
pause
