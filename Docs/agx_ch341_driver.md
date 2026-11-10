# AGX CH341 / UM982 driver installation

Installed on 2026-10-13 on AGX, kernel `6.8.12-1021-tegra`.
The kernel had `CONFIG_USB_SERIAL_CH341` disabled. The upstream Linux v6.8.12
CH341 driver was built against the installed, matching NVIDIA kernel headers.

Source: https://raw.githubusercontent.com/gregkh/linux/v6.8.12/drivers/usb/serial/ch341.c
Source SHA256: b286d49dd1f2a3a8f96188888b4df7869f65c4d9bead51d5291d50150b235951
Build directory: `/tmp/njord-ch341-6.8.12` (temporary).
Module: `/lib/modules/6.8.12-1021-tegra/updates/ch341.ko`.
`depmod -a` and `modprobe ch341` completed successfully. USB alias matching
provides automatic loading for this installed kernel on device enumeration.
Kernel updates require rebuilding against their matching headers.

The module is unsigned; this kernel permits unsigned modules and reports the
normal module-verification/taint message. No kernel or boot configuration was
replaced. Reboot was not required or tested.

The CH340 at USB connector `platform-3610000.usb-usb-0:4.1:1.0` now provides:
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` -> `/dev/ttyUSB1`.
The narrowly scoped rule `config/udev/70-njord-um982.rules` is installed at
`/etc/udev/rules.d/70-njord-um982.rules`; it grants ibo access only to this
vendor/product on this physical USB path. No dialout group membership was added.

The UM982 ROS driver is running separately from the existing vessel bringup,
with the original launch defaults. Log: `/tmp/njord-startup-check/gnss-running.log`.
UART sentences GNGGA, GPGST, and GPTHS were observed at the configured baudrate.
GGA reported fix quality 0, so no valid position was confirmed. The driver logs
malformed/invalid GPGST covariance and NTRIP hostname-resolution warnings;
these are separate from the now-working USB driver.
The GNSS input still needs a valid satellite fix and valid covariance before
location data can be used. No fabricated location or sensor data was published.

For future full bringup, the AGX launch already detects this stable by-id path
and enables UM982 when the device exists. The running vessel bringup was not
restarted during this installation.

Build recipe (for this exact kernel source revision):

```bash
mkdir -p /tmp/njord-ch341-6.8.12
curl -fL https://raw.githubusercontent.com/gregkh/linux/v6.8.12/drivers/usb/serial/ch341.c \
  -o /tmp/njord-ch341-6.8.12/ch341.c
printf 'obj-m += ch341.o\n' > /tmp/njord-ch341-6.8.12/Makefile
make -C /lib/modules/$(uname -r)/build M=/tmp/njord-ch341-6.8.12 modules
sudo install -D -m 644 /tmp/njord-ch341-6.8.12/ch341.ko \
  /lib/modules/$(uname -r)/updates/ch341.ko
sudo depmod -a
sudo modprobe ch341
```
