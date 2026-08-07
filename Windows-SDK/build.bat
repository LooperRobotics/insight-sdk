@echo off
setlocal enabledelayedexpansion

echo ========================================
echo Building Insight9SDK
echo ========================================

echo Checking OpenCV directory...
if exist "third_party\opencv\build\x64\vc16\lib\opencv_world4120.lib" (
    echo Found OpenCV: opencv_world4120.lib
) else (
    echo OpenCV library not found!
    echo Please run download_opencv.bat first.
    pause
    exit /b 1
)

if exist build (
    echo Removing old build directory...
    rmdir /s /q build
)

mkdir build
cd build

echo.
echo Configuring CMake...
cmake .. -G "Visual Studio 18 2026" -A x64 -DUSE_SYSTEM_FFMPEG=OFF -DUSE_SYSTEM_HIDAPI=OFF

if errorlevel 1 (
    echo.
    echo [ERROR] CMake configuration failed!
    cd ..
    pause
    exit /b 1
)

echo.
echo Building Release...
cmake --build . --config Release

if errorlevel 1 (
    echo.
    echo [ERROR] Build failed!
    cd ..
    pause
    exit /b 1
)

cd ..
echo.
echo ========================================
echo Build completed successfully!
echo ========================================
echo Example executable: build\Release\example.exe
echo.
echo To run the example:
echo   cd build\Release
echo   example.exe
echo ========================================
pause