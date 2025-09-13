@echo off
REM Installation script for Raspberry Pi 4 (64-bit OS)
REM Copy this to your Pi 4 and run to install dependencies

echo Installing Python packages for ECSE Robot System on Pi 4...
python3 --version

REM Navigate to the wheel files directory
cd /d "%~dp0rpi_packages"

REM Install packages in dependency order
echo Installing setuptools...
pip3 install --no-index --find-links . setuptools-80.9.0-py3-none-any.whl

echo Installing wheel...
pip3 install --no-index --find-links . wheel-0.45.1-py3-none-any.whl

echo Installing numpy...
pip3 install --no-index --find-links . numpy-1.24.4-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl

echo Installing Pillow...
pip3 install --no-index --find-links . pillow-11.3.0-cp39-cp39-manylinux2014_aarch64.manylinux_2_17_aarch64.whl

echo Installing OpenCV...
pip3 install --no-index --find-links . opencv_python-4.12.0.88-cp37-abi3-manylinux2014_aarch64.manylinux_2_17_aarch64.whl

echo Installing pygame...
pip3 install --no-index --find-links . pygame-2.6.1-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl

echo Installing smbus2...
pip3 install --no-index --find-links . smbus2-0.5.0-py2.py3-none-any.whl

echo Installing lgpio...
pip3 install --no-index --find-links . lgpio-0.2.2.0-cp39-cp39-manylinux_2_34_aarch64.whl

echo.
echo Installation complete! Pi 4 setup ready.
echo To start the robot: python3 Software/integrated_robot_system.py
pause