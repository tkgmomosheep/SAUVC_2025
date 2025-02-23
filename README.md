# Build status
![Build](https://github.com/tkgmomosheep/SAUVC_2025/actions/workflows/build.yml/badge.svg)
![Release](https://github.com/tkgmomosheep/SAUVC_2025/actions/workflows/create-release.yml/badge.svg)

# Flasing
1. Download and unzip the lastet firmware from release
2. Go to https://espressif.github.io/esptool-js/
3. Press connect in Program, select the correct port
4. Set Flash Address according to the following table
| Flash Address | File                |
|---------------|---------------------|
| 0x0           | bootloader.bin      |
| 0x10000       | sauvc-idf-only.bin  |
| 0x8000        | partition-table.bin |