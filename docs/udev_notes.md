
##  Use udevadm to find difference between u2d2 devices

https://dev.to/enbis/how-udev-rules-can-help-us-to-recognize-a-usb-to-serial-device-over-dev-tty-interface-pbk


'/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.3/1-2.3:1.0/ttyUSB0/tty/ttyUSB0'
'/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/1-2.4:1.0/ttyUSB1/tty/ttyUSB1'

diff -u u0 u1
--- u0	2022-12-11 10:36:59.102523065 -0600
+++ u1	2022-12-11 10:37:10.298355554 -0600
@@ -2,13 +2,13 @@
 specified by a RUN key. It may show incorrect results, because
 some values may be different, or not available at a simulation run.
 
-DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.3/1-2.3:1.0/ttyUSB0/tty/ttyUSB0
-DEVNAME=/dev/ttyUSB0
+DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/1-2.4:1.0/ttyUSB1/tty/ttyUSB1
+DEVNAME=/dev/ttyUSB1
 MAJOR=188
-MINOR=0
+MINOR=1
 ACTION=add
 SUBSYSTEM=tty
-DEVLINKS=/dev/ttyPAPRAS /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT5WIQA8-if00-port0 /dev/serial/by-path/pci-0000:00:14.0-usb-0:2.3:1.0-port0
+DEVLINKS=/dev/serial/by-path/pci-0000:00:14.0-usb-0:2.4:1.0-port0 /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNSOW-if00-port0 /dev/ttyPAPRAS
 ID_BUS=usb
 ID_VENDOR_ID=0403
 ID_MODEL_ID=6014
@@ -21,16 +21,16 @@
 ID_MODEL=USB__-__Serial_Converter
 ID_MODEL_ENC=USB\x20\x3c-\x3e\x20Serial\x20Converter
 ID_REVISION=0900
-ID_SERIAL=FTDI_USB__-__Serial_Converter_FT5WIQA8
-ID_SERIAL_SHORT=FT5WIQA8
+ID_SERIAL=FTDI_USB__-__Serial_Converter_FT4NNSOW
+ID_SERIAL_SHORT=FT4NNSOW
 ID_TYPE=generic
 ID_USB_INTERFACES=:ffffff:
 ID_USB_INTERFACE_NUM=00
 ID_USB_DRIVER=ftdi_sio
 ID_MODEL_FROM_DATABASE=FT232H Single HS USB-UART/FIFO IC
 .ID_PORT=0
-ID_PATH=pci-0000:00:14.0-usb-0:2.3:1.0
-ID_PATH_TAG=pci-0000_00_14_0-usb-0_2_3_1_0
+ID_PATH=pci-0000:00:14.0-usb-0:2.4:1.0
+ID_PATH_TAG=pci-0000_00_14_0-usb-0_2_4_1_0
 ID_MM_CANDIDATE=1
 TAGS=:systemd:
-USEC_INITIALIZED=5504244
+USEC_INITIALIZED=1375622883
