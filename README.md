# Zigbee fan control with rotary encoder support

Requires ESP32-C6.

1. idf.py -p comN erase-flash
2. idf.py -p comN flash
3. Get the MAC address from console (the other port for ESP32-C6)
4. Create a manufacturing partition using [Zigbee Manufacturing Partition Generator Utility](https://github.com/espressif/esp-zigbee-sdk/blob/main/tools/mfg_tool/README.md).
5. Clone the zigbee sdk repo and remove the `from future.moves.itertools import zip_longest` from `tools/esp_zb_mfg_tool.py`.
6. Install dependencies in ESP-IDF shell: `python -m pip install -r requirements.txt` and `python -m pip install cryptography esp-idf-nvs-partition-gen crcmod`
7. Generate an installcode with Python: `python -c "import os;import crcmod.predefined;installcode=os.urandom(12);crc=crcmod.predefined.mkCrcFun('crc-16');print(installcode.hex() + hex(crc(installcode))[2:]);"`
8. Install dependencies for mgf tool: `python -m pip install esp_idf_nvs_partition_gen future`
9. Run: `python esp_zb_mfg_tool.py -i 7f91fbafb9ec53ee8097b3bb8fce -m 404ccafffe5627c4 -c 0x8000 -mn Espressif -mc 0x1337` (change first parameter to random string from previous command, second is MAC without colons)
10. Flash the binary to the `zb_fct` partition (at `0xf5000`): `esptool.py -p com4 write_flash 0xf5000 404ccafffe5627c4.bin`