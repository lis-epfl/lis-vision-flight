# Software

We first describe how to set up the custom Jetson Nano carrier board. All of the instructions were tested on a host system with Ubuntu 18.04 LTS.

1. Install Nvidia SDK Manager which you can find here: https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html

1. Run the SDK Manager, choose Jetson Nano as Target Hardware and JetPack 4.4 as Target OS. Click next and select P3448-000 Jetson Nano [developer kit version] and continue with download and install. For now, skip the flashing of the Nano when prompted. JetPack will be installed on `~/nvidia/nvidia_sdk/JetPack_4.4_Linux_JETSON_NANO_DEVKIT/Linux_for_Tegra/`.

1. Create the folder `~/nvidia/nvidia_sdk/JetPack_4.4_Linux_JETSON_NANO_DEVKIT/Linux_for_Tegra/sources` and extract the [kernel sources](https://developer.nvidia.com/embedded/dlc/r32-3-1_Release_v1.0/Sources/T210/public_sources.tbz2) into it. Make sure to also extract all the containers inside the file you download.

1. Download and unpack [dts.zip](./carrier_board_nano/dts.zip) on your host machine. Copy the `tegra210-p3448-0002-p3449-0000-b00.dts` and `tegra210-porg-p3448-common.dtsi` to `~/nvidia/nvidia_sdk/JetPack_4.4_Linux_JETSON_NANO_DEVKIT/Linux_for_Tegra/sources/hardware/nvidia/platform/t210/porg/kernel-dts/`  folder (overwrite). Copy the two `...lis_nano...` files to `./porg-platforms` subfolder.

1. Download and extract the [Linaro GCC](http://releases.linaro.org/components/toolchain/binaries/7.3-2018.05/aarch64-linux-gnu/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz) to `~/opt/`.

1. Copy these 3 lines at the end of `~/.bashrc`:
    
    ``` bash
    export CROSS_COMPILE=$HOME/opt/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
    export LOCALVERSION=-tegra
    export TEGRA_KERNEL_OUT=$HOME/nvidia/kernel_img
    ```
    and then run `source ~/.bashrc`.

1.  Navigate to `~/nvidia/nvidia_sdk/JetPack_4.4_Linux_JETSON_NANO_DEVKIT/Linux_for_Tegra/sources/kernel/kernel-4.9` and run:
    ``` bash
    mkdir -p $TEGRA_KERNEL_OUT
    make ARCH=arm64 O=$TEGRA_KERNEL_OUT tegra_defconfig
    make ARCH=arm64 O=$TEGRA_KERNEL_OUT dtbs
    cp ~/nvidia/kernel_img/arch/arm64/boot/dts/tegra210-p3448-0002-p3449-0000-b00.dtb ~/nvidia/nvidia_sdk/JetPack_4.4_Linux_JETSON_NANO/Linux_for_Tegra/kernel/dtb/
    ```

1. Copy the script [l4t_create_default_user.sh](carrier_board_nano/l4t_create_default_user.sh) to `~/nvidia/nvidia_sdk/JetPack_4.4_Linux_JETSON_NANO_DEVKIT/Linux_for_Tegra/` and move to the folder in a shell. Run
    ``` bash
    sudo ./l4t_create_default_user.sh -u lis -p lislis
    ```
    replace `lis` and `lislis` with your username and password.

1. Put the board into the ForcedRecovery mode (press "RECOV" button and while holding it, reset with "RES" button). Plug a standard micro-USB cable into the Ethernet over USB port (see image below). Check that an "Nvidia" device is showing up after issuing of `lsusb` command.

1. Flash the Nano module with `sudo ./flash.sh jetson-nano-emmc mmcblk0p1` to flash it on the emmc or `./flash.sh jetson-nano-qspi-sd mmcblk0p1` to flash it to the SD card.