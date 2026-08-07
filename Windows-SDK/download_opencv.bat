@echo off
echo Downloading OpenCV for Windows...

set OPENCV_VERSION=4.12.0
set OPENCV_URL=https://github.com/opencv/opencv/releases/download/%OPENCV_VERSION%/opencv-%OPENCV_VERSION%-windows.exe
set OPENCV_EXE=opencv-%OPENCV_VERSION%-windows.exe
set OPENCV_DIR=third_party\

if exist "%OPENCV_DIR%" (
    echo OpenCV already exists in project.
    exit /b 0
)

echo Downloading OpenCV...
curl -L -o "%OPENCV_EXE%" "%OPENCV_URL%"

if errorlevel 1 (
    echo Download failed! Please download manually from:
    echo %OPENCV_URL%
    exit /b 1
)

echo Extracting OpenCV...
"%OPENCV_EXE%" -o"%OPENCV_DIR%" -y

if errorlevel 1 (
    echo Extraction failed!
    exit /b 1
)

del "%OPENCV_EXE%"

echo OpenCV downloaded and extracted to %OPENCV_DIR%