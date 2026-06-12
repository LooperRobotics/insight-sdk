@echo off
setlocal enabledelayedexpansion

echo Building Insight9SDK...

if exist build (
    echo Removing old build directory...
    rmdir /s /q build
)

mkdir build
cd build

cmake .. -G "Visual Studio 18 2026" -A x64 -DUSE_SYSTEM_FFMPEG=OFF -DUSE_SYSTEM_HIDAPI=OFF

cmake --build . --config Release

cd ..
echo.
echo Build completed. Example executable is in build\Release\example.exe