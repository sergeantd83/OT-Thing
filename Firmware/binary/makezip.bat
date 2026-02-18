set mcd="%CD%"

cd \users\tim\.pio_workspaces\build\otgw32

esptool --chip esp32s3 merge-bin -o %mcd%\otgw32.bin 0 bootloader.bin 0x8000 partitions.bin 0xe000 C:\Users\Tim\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin 0x10000 firmware.bin

cd %mcd%

wsl sh -c "chmod +x flash.sh && zip otgw32.zip flash.bat flash.sh readme.txt otgw32.bin && rm -f otgw32.bin"
