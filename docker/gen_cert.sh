#!/bin/bash

# Usage info
if [ $# -lt 1 ]; then
  echo "Usage: $0 [HOST_IP]"
  echo "Example: $0 192.168.0.10"
  exit 1
fi

HOST_IP="$1"
VR_FOLDER="/root/ros2_ws/src/robotis_vuer/robotis_vuer"

set -e

echo "1. Installing mkcert root CA..."
mkcert -install

echo "2. Generating certificate (cert.pem, key.pem)"
mkcert -cert-file cert.pem -key-file key.pem "$HOST_IP" localhost 127.0.0.1

echo "3. Copying certificates to Robotis Vuer folder ($VR_FOLDER)"
mkdir -p "$VR_FOLDER"
cp cert.pem key.pem "$VR_FOLDER"

echo "Done! Certificates have been copied to $VR_FOLDER."
