# PowerShell Script to Run the KV4P Radio Controller

# Get the directory where this script is located
$ScriptDir = $PSScriptRoot

# Define the path to the virtual environment activation script
$VenvActivate = Join-Path -Path $ScriptDir -ChildPath "venv\Scripts\Activate.ps1"

# Check if the virtual environment exists
if (Test-Path $VenvActivate) {
    Write-Host "Activating Python virtual environment..."
    . $VenvActivate
} else {
    Write-Warning "Virtual environment not found. Running with system Python."
    Write-Warning "For best results, please create a virtual environment by running 'python -m venv venv' first."
}

# Run the application
Write-Host "Starting the radio controller application..."
python -m app.app

# The virtual environment will be automatically deactivated when the terminal session ends.
