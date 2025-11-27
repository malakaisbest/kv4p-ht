@echo off
REM Batch Script to Run the KV4P Radio Controller

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"

REM Define the path to the virtual environment activation script
set "VENV_ACTIVATE=%SCRIPT_DIR%venv\Scripts\activate.bat"

REM Check if the virtual environment exists
if exist "%VENV_ACTIVATE%" (
    echo "Activating Python virtual environment..."
    call "%VENV_ACTIVATE%"
) else (
    echo "WARNING: Virtual environment not found. Running with system Python."
    echo "For best results, please create a virtual environment by running 'python -m venv venv' first."
)

REM Run the application
echo "Starting the radio controller application..."
python -m app.app
