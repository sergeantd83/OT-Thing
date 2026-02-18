#!/bin/bash

# Fix potential Windows line ending issues if edited on PC
sed -i 's/\r$//' "$0" 2>/dev/null

if ! [ -w /dev/ttyUSB0 ] && ! [ -w /dev/ttyACM0 ] && [ "$EUID" -ne 0 ]; then
    if ! groups $USER | grep -q "\bdialout\b"; then
        exec sudo bash "$0" "$@"
        exit $?
    fi
fi

if [ ! -f "./esptool" ]; then
    echo "Downloading esptool..."
    URL=$(curl -s https://api.github.com/repos/espressif/esptool/releases | \
          grep -oP '"browser_download_url": "\K[^"]*linux-amd64[^"]*' | head -n 1)

    if [ -z "$URL" ]; then
        echo "Error: Could not find esptool. Please download it manually."
        exit 1
    fi

    curl -L -s "$URL" | tar -xz --strip-components 1
    rm -f espefuse espsecure esp_rfc2217_server README.md LICENSE
    chmod +x esptool
fi

S3_PORT=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -n 1)

if [ -n "$S3_PORT" ]; then
    echo "OTGW32 detected on $S3_PORT"
    PORT_ARG="--port $S3_PORT"
else
    echo "OTGW32 not found."
    exit 1
fi

echo "Programming OTGW32"
./esptool $PORT_ARG --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect 0x0 otgw32.bin
read -p "Press Enter to exit script"
