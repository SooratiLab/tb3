#!/bin/bash
# filepath: /home/humble/aice2011/venv/create_env.sh

set -e  # Exit on any error

# Get the directory where this script is located (venv folder inside aice2011)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_FOLDER="$(dirname "$SCRIPT_DIR")"
PARENT_FOLDER="$(dirname "$BASE_FOLDER")"

# If BASE_FOLDER is /home and ~/marl exists, use that instead
if [[ "$BASE_FOLDER" == "/home" && -d "$HOME/marl" ]]; then
    BASE_FOLDER="$HOME/marl"
    PARENT_FOLDER="$HOME"
    echo "[INFO] Detected Docker/container environment, using ~/marl as base folder"
fi

echo "Script directory: $SCRIPT_DIR"
echo "Base folder: $BASE_FOLDER"
echo "Parent folder: $PARENT_FOLDER"
echo

# Function to get Python version
get_python_version() {
    local python_cmd="$1"
    if command -v "$python_cmd" &> /dev/null; then
        local version=$("$python_cmd" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
        echo "$version"
    else
        echo ""
    fi
}

# Function to check if Python version is >= 3.8
is_python_valid() {
    local version="$1"
    if [[ -n "$version" ]]; then
        local major=$(echo "$version" | cut -d. -f1)
        local minor=$(echo "$version" | cut -d. -f2)
        if [[ "$major" -ge 3 && "$minor" -ge 8 ]]; then
            return 0
        fi
    fi
    return 1
}

# Function to check if a package is installed
is_package_installed() {
    local package_name="$1"
    pip show "$package_name" &> /dev/null
    return $?
}

# Check if aice2011 environment exists in envs folder
ENVS_AICE2011_PATH="$HOME/envs/aice2011"

# Check if valid environment exists (has bin/activate)
if [[ -f "$ENVS_AICE2011_PATH/bin/activate" ]]; then
    echo "Found aice2011 environment at: $ENVS_AICE2011_PATH"
    VENV_PATH="$ENVS_AICE2011_PATH"
else
    # Need to create environment
    echo "aice2011 environment not found. Creating new environment..."
    # Use ~/envs/aice2011 for all other cases
    mkdir -p "$HOME/envs"
    VENV_PATH="$HOME/envs/aice2011"
    echo "Will create environment at: $VENV_PATH"
    
    # Find suitable Python version
    PYTHON_CMD=""
    PYTHON_VERSION=""
    
    # Try Python 3.8 first (default)
    for py_cmd in "python3.8" "python3.10" "python3.11" "python3.12" "python3" "python"; do
        version=$(get_python_version "$py_cmd")
        if is_python_valid "$version"; then
            PYTHON_CMD="$py_cmd"
            PYTHON_VERSION="$version"
            echo "Using Python $version ($py_cmd)"
            break
        fi
    done
    
    if [[ -z "$PYTHON_CMD" ]]; then
        echo "Error: No suitable Python version (>= 3.8) found!"
        exit 1
    fi
    
    # Create virtual environment
    echo "Creating virtual environment with $PYTHON_CMD..."
    "$PYTHON_CMD" -m venv "$VENV_PATH"
fi
echo

# Activate the environment
echo "Activating environment: $VENV_PATH"
source "$VENV_PATH/bin/activate"
echo

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip
echo

# Check if base folder has setup.py
SETUP_PY_PATH="$BASE_FOLDER/setup.py"
if [[ ! -f "$SETUP_PY_PATH" ]]; then
    echo "setup.py not found in base folder. Copying from aice2011..."
    cp "$SCRIPT_DIR/setup.py" "$SETUP_PY_PATH"
fi
echo

# Get the package name from the base folder name
BASE_FOLDER_NAME=$(basename "$BASE_FOLDER")
PACKAGE_NAME=$(echo "$BASE_FOLDER_NAME" | tr '[:upper:]' '[:lower:]')

# Install base folder in editable mode (check if already installed)
if is_package_installed "$PACKAGE_NAME"; then
    echo "Base folder package '$PACKAGE_NAME' is already installed. Skipping..."
else
    echo "Installing base folder in editable mode..."
    cd "$BASE_FOLDER"
    pip install -e .
fi
echo

# Add base folder to .pth file to ensure it's always in sys.path
echo "Ensuring base folder is in sys.path..."
SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])")
PTH_FILE="$SITE_PACKAGES/argos_base.pth"

if [[ -f "$PTH_FILE" ]]; then
    # Check if our path is already in the file
    if grep -q "^$BASE_FOLDER$" "$PTH_FILE"; then
        echo "Base folder already in sys.path via $PTH_FILE"
    else
        echo "Updating $PTH_FILE with base folder path"
        echo "$BASE_FOLDER" >> "$PTH_FILE"
    fi
else
    echo "Creating $PTH_FILE"
    echo "$BASE_FOLDER" > "$PTH_FILE"
fi
echo "Added $BASE_FOLDER to sys.path"
echo

# # Install requirements based on Python version
# cd "$SCRIPT_DIR"
# PYTHON_VERSION=$(python -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
# echo "Python version in environment: $PYTHON_VERSION"

# REQ_FILE=""
# if [[ -f "requirements/aice2011-python-$PYTHON_VERSION.txt" ]]; then
#     REQ_FILE="requirements/aice2011-python-$PYTHON_VERSION.txt"
# elif [[ -f "requirements/tb3-python-$PYTHON_VERSION.txt" ]]; then
#     REQ_FILE="requirements/tb3-python-$PYTHON_VERSION.txt"
#     echo "Warning: Using tb3 requirements for Python $PYTHON_VERSION"
# elif [[ -f "requirements/e-swarm-python-3.8.txt" ]]; then
#     REQ_FILE="requirements/e-swarm-python-3.8.txt"
#     echo "Warning: Using Python 3.8 requirements for Python $PYTHON_VERSION"
# else
#     echo "Error: No suitable requirements file found!"
#     exit 1
# fi

# echo "Installing requirements from: $REQ_FILE"
# pip install -r "$REQ_FILE"

echo "Environment setup complete!"
echo "Virtual environment path: $VENV_PATH"
echo "To activate manually: source $VENV_PATH/bin/activate"
echo

# Fix permissions to prevent Docker from locking files (if running as root in Docker)
# if [ "$EUID" -eq 0 ] && [[ -d "$VENV_PATH" ]]; then
#     echo "Fixing permissions on virtual environment..."
#     chmod -R 755 "$VENV_PATH" 2>/dev/null || true
#     # Try to match ownership with a mounted volume (if available)
#     if [[ -d "$BASE_FOLDER" ]]; then
#         OWNER_UID=$(stat -c '%u' "$BASE_FOLDER" 2>/dev/null || echo "")
#         OWNER_GID=$(stat -c '%g' "$BASE_FOLDER" 2>/dev/null || echo "")
#         if [[ -n "$OWNER_UID" && "$OWNER_UID" != "0" ]]; then
#             echo "Setting ownership to UID:GID $OWNER_UID:$OWNER_GID to match host user..."
#             chown -R "$OWNER_UID:$OWNER_GID" "$VENV_PATH" 2>/dev/null || true
#         fi
#     fi
#     echo "Permissions updated."
# fi
# echo

# Return to base folder
cd "$BASE_FOLDER"