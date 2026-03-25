#!/bin/bash
# ARGUS — Fix directory structure
# Run this from INSIDE the website/ folder where all files are flat
# Usage: cd website && bash setup_structure.sh

echo "Setting up ARGUS directory structure..."

# Create subdirectories
mkdir -p templates
mkdir -p static
mkdir -p firmware/argus_vehicle
mkdir -p models
mkdir -p calibration

# Move index.html into templates/
if [ -f "index.html" ]; then
    mv index.html templates/index.html
    echo "  ✓ index.html → templates/index.html"
else
    echo "  ⚠ index.html not found (already moved?)"
fi

# Move Arduino firmware into firmware/argus_vehicle/
if [ -f "argus_vehicle.ino" ]; then
    mv argus_vehicle.ino firmware/argus_vehicle/argus_vehicle.ino
    echo "  ✓ argus_vehicle.ino → firmware/argus_vehicle/argus_vehicle.ino"
else
    echo "  ⚠ argus_vehicle.ino not found (already moved?)"
fi

echo ""
echo "Done! Structure:"
echo ""
echo "  website/"
echo "  ├── app.py"
echo "  ├── config.py"
echo "  ├── cv_pipeline.py"
echo "  ├── pid_controller.py"
echo "  ├── comms.py"
echo "  ├── calibrate.py"
echo "  ├── requirements.txt"
echo "  ├── README.md"
echo "  ├── templates/"
echo "  │   └── index.html"
echo "  ├── static/"
echo "  ├── firmware/"
echo "  │   └── argus_vehicle/"
echo "  │       └── argus_vehicle.ino"
echo "  ├── models/"
echo "  └── calibration/"
echo ""
echo "Next steps:"
echo "  1. pip install -r requirements.txt --break-system-packages"
echo "  2. Edit config.py (ESP32_IP, CAMERA_INDEX)"
echo "  3. python3 calibrate.py --both"
echo "  4. python3 app.py"