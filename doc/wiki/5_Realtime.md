## Setting up the real-time patch

### Prerequisites:
- at least 30GB free disk space.

### Downloading kernel source
1. check your current kernel version
   ```
   uname -r
   ```

2. Find and download the real-time patch with matching major and minor versions from [here](https://cdn.kernel.org/pub/linux/kernel/projects/rt/) (*patch-\<version\>-rt\<nr\>.patch.gz*)

3. Download the kernel with identical version as the real-time patch from [here](https://mirrors.edge.kernel.org/pub/linux/kernel/) (*linux-\<version\>.tar.gz*)

4. Create directory to build kernel
   ```
   mkdir ~/kernel
   cd ~/kernel
   ```
   - copy the downloaded archives to the `~/kernel` directory

5. Unpack archives
   ```
   tar -xzf linux-<version>.tar.gz
   gunzip patch-<version>-rt<nr>.patch.gz
   ```

6. Patch the kernel with the realtime patch:
   ```
   cd linux-<version>
   patch -p1 < ../patch-<version>-rt<nr>.patch
   ```

7. Install dependencies needed for building the kernel
   ```
   sudo apt install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf fakeroot debhelper
   ```

### Kernel configuration

#### Create configuration for building the kernel
   - The easiest is to use the configuration of the current installation:
      ```
      cp /boot/config-<version>-generic .config
      ```
   - Modify certificates for signature checking
      ```
      scripts/config --disable SYSTEM_REVOCATION_KEYS
      scripts/config --disable SYSTEM_REVOCATION_LIST
      scripts/config --set-str SYSTEM_TRUSTED_KEYS ""
      ```
   - Enable all new Ubuntu configurations:
      ```
      yes '' | make oldconfig
      ```

#### Real-time adaptations
   The kernel configuration must be modified to make it real-time-capable. Open GUI for modifications:
   ```
   make menuconfig
   ```

##### Enable fully preemptible kernel
   ```
   -> General Setup
      -> Preemption Model
         (X) Fully Preemptible Kernel (Real-Time)
   ```

##### Enable high resolution timer support
   ```
   -> General setup
   -> Timers subsystem
      [*] High Resolution Timer Support
   ```

##### Omit scheduling-clock ticks on isolated CPU-s
   ```
   -> General setup
   -> Timers subsystem
      -> Timer tick handling
      (X) Full dynticks system (tickless)
   ```

##### Enforce maximum CPU frequency
   ```
   ->  Power management and ACPI options
   -> CPU Frequency scaling
      -> CPU Frequency scaling
      -> Default CPUFreq governor
      (X) performance
   ```

Save (without modifying the name) and exit menuconfig.

### Build and install kernel

1. Build the kernel (which will take quite some time):
   - Below kernel version 6.3:
   ```
   make -j `getconf _NPROCESSORS_ONLN` deb-pkg
   ```
   - From kernel version 6.3:
   ```
   make -j `getconf _NPROCESSORS_ONLN` bindeb-pkg
   ```
   After successful completion, there should be 4 debian packages in the `~/kernel` directory

2. Install all kernel debian packages:
    ```
    sudo dpkg -i ../*.deb
    ```

3. Reboot
   - Now the real time kernel should be installed. Reboot the system.
   - At startup, choose the built kernel from the boot menu (*Advanced options*), or configure it to be default before restart.
   - Check new kernel version with `uname -a`
      - you should see `PREEMPT_RT` in the kernel version

## Configuration

After installing the real-time kernel, the setting of scheduling priorities must be enabled for your user:
- extend `/etc/security/limits.conf` with
```
username	 -	 rtprio		 98
```
- Restart the system

## Possible issues of building the kernel
### SSL error at signing
**Error**:
`SSL error:FFFFFFFF80000002:system library::No such file or directory`

**Solution**: disable ZSTD module compression and rebuild
   ```
   scripts/config --disable CONFIG_MODULE_COMPRESS_ZSTD
   scripts/config --enable CONFIG_MODULE_COMPRESS_NONE
   ```

### Segmentation fault while building a specific driver

**Error example**
```
CC [M]  drivers/net/wireless/mediatek/mt76/mt7921/mac.occ1:
internal compiler error: Segmentation fault
```
**Solution**: remove the problematic driver (mt7921 in example) from the kernel configuration file
