## Setting up the real-time patch


### Downloading kernel

Prerequisites:
- at least 30GB free disk space.

check your current kernel version
`uname -r`

Find the real-time patch with mathching major and minor versions here https://cdn.kernel.org/pub/linux/kernel/projects/rt/

Create directory to build kernel
`mkdir ~/kernel`
`cd ~/kernel`

Check latest stable version [here](https://wiki.linuxfoundation.org/realtime/start)

and download the appropriate patch from [here](https://cdn.kernel.org/pub/linux/kernel/projects/rt/) (patch-<version>-rt<nr>.patch.gz)

and the kernel with identical version from [here](https://mirrors.edge.kernel.org/pub/linux/kernel/) (linux-<version>.tar.gz)

copy the archives to the `~/kernel` directory and unpack with

`tar -xzf linux-<version>.tar.gz`
`gunzip patch-<version>-rt<nr>.patch.gz`

Switch into the unpacked linux directory 
`cd linux-<version>`

Patch the kernel with the realtime patch:
`patch -p1 < ../patch-<version>-rt<nr>.patch`

We simply want to use the config of our current installation, so we copy the current config with
`cp /boot/config-<version>-generic .config`

Install dependencies needed for building the kernel
`sudo apt install libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf fakeroot`

To enable all Ubuntu configurations, we simply use
`yes '' | make oldconfig`

### Kernel configuration

The kernel configuration must be modified to make it real-time-capable
`make menuconfig`

##### Enable CONFIG_PREEMPT_RT
 -> General Setup
  -> Preemption Model (Fully Preemptible Kernel (Real-Time))
   (X) Fully Preemptible Kernel (Real-Time)

##### Enable CONFIG_HIGH_RES_TIMERS
 -> General setup
  -> Timers subsystem
   [*] High Resolution Timer Support

##### Enable CONFIG_NO_HZ_FULL
 -> General setup
  -> Timers subsystem
   -> Timer tick handling (Full dynticks system (tickless))
    (X) Full dynticks system (tickless)

##### Set CPU_FREQ_DEFAULT_GOV_PERFORMANCE [=y]
 ->  Power management and ACPI options
  -> CPU Frequency scaling
   -> CPU Frequency scaling (CPU_FREQ [=y])
    -> Default CPUFreq governor (<choice> [=y])
     (X) performance
Save and exit menuconfig. Now we’re going to build the kernel which will take quite some time.

```
make -j `getconf _NPROCESSORS_ONLN` deb-pkg
```

After successfull completition, there should be 4 debian packages:
`ls ../*deb`

Then we install all kernel debian packages
`sudo dpkg -i ../*.deb`

Now the real time kernel should be installed. Reboot the system and check the new kernel version

`reboot`
`uname -a`

## Configuration

After installing the real-time kerenel, the setting of scheduling priorities must be enabled for your user:
- extend `/etc/security/limits.conf` with 
```username	 -	 rtprio		 98```
- restart