# KV4P Radio Desktop Controller

This application provides a graphical user interface (GUI) to control a KV4P radio transceiver from your desktop.

## Setup and Installation

To run this application, you need Python 3 and a few third-party packages. It is highly recommended to use a virtual environment to manage the project's dependencies.

### 1. Create a Virtual Environment

A virtual environment isolates the packages for this project from your system's global Python installation.

Navigate to the `desktop-app` directory in your terminal and run the following command to create a virtual environment named `venv`:

```sh
python -m venv venv
```

### 2. Activate the Virtual Environment

Before installing packages or running the app, you must activate the virtual environment.

**On Windows:**
```sh
.\\venv\\Scripts\\activate
```

**On macOS and Linux:**
```sh
source venv/bin/activate
```

Your terminal prompt should now show `(venv)` at the beginning, indicating that the virtual environment is active.

### 3. Install Required Packages

With the virtual environment active, install the necessary Python packages using `pip`:

```sh
pip install pyserial sounddevice numpy opuslib
```

## Running the Application

Once the setup is complete, you can run the application with the following command from within the `desktop-app` directory:

```sh
python -m app.app
```

The radio controller window should appear, allowing you to connect to your device and begin operating it.

## Troubleshooting

### Opus Decode/Encode Errors on Windows

If you encounter errors related to Opus decoding or encoding when running the application on Windows, it might be due to the `opus.dll` file not being found in the system's PATH.

**Solution:**

1.  Locate the `opus.dll` file inside `desktop-app` directory.
2.  Copy this `opus.dll` file to your `C:\Windows\System32` folder.

After copying the DLL, restart the application. This should resolve the Opus-related errors.
