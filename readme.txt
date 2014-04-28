
In this document, we introduce how to support rtk 8723AE/AU BT driver in Linux system.
Support kernel version 2.6.32~3.13.0

===========================================================================================================

1. Install

(1)	Change to the directory containing the source files.

	make
	sudo make install

(2)	Insert RTK8723AE/AU dongle

	Now RTK8723AE/AU can be recognized by the system and bluetooth function can be used.

===========================================================================================================

2. Uninstall

(1)	unplug RTK8723AE/AU dongle 

(2)	Uninstall	

	sudo make uninstall
 
===========================================================================================================

3. Install BlueZ

The default BlueZ in Ubuntu maybe out of date
New versions can be downloaded from http://www.bluez.org/   
If you do not want to install blueZ,skip this step

Here we take Ubuntu 11.10 (linux 3.0.0-12-generic) for example. 
Make sure system is connected to the internet.

(1)	Setting up a Linux build environment 

	Install dbus
	$sudo apt-get install libdbus-1-dev
	Install glib
	$sudo apt-get install libglib2.0-dev

(2)	Decompress and install BlueZ follow Readme file in blueZ-4.96 directory

	$sudo tar -zxvf bluez-4.96.tar.gz
	$sudo cd bluez-4.96
	$sudo ./configure --prefix=/usr --mandir=/usr/share/man --sysconfdir=/etc --localstatedir=/var --libexecdir=/lib
	$sudo make && make install
 
===========================================================================================================

4. Set Up A2DP Sink

(1) Enable a2dp sink support

	Edit /etc/bluetooth/audio.conf by adding this line underneath the [General] 
	section:
	# This section contains options which are not specific to any
	# particular interface
	[General]
	Enable=Source

	After making this change, you will need to restart bluetoothd by running
	$sudo service bluetooth restart

(2) Pair your source device and connect to a2dp sink at the source side
	Once you've done this, the source device should show up as an input device under pulseaudio. 
	Then you need to tell pulseaudio to route this audio input to your output (such as your speakers,
	or a bluetooth headset). You can do this from the commandline using the pactl command:

	$pactl load-module module-loopback source=$BTSOURCE sink=$SINK

	for example: 
	pactl load-module module-loopback source=bluez_source.14_DA_E9_2A_D7_57 sink=alsa_output.pci-0000_00_14.2.analog-stereo
	You can find your own values for source and sink with 
	$pacmd list-sources 
	$pacmd list-sinks

	Be careful however to remove this loopback connection before you drop this audio connection! 
	pulseaudio appears to re-connect the loopback connection to the next available audio input when
	the connection drops, so if for instance your laptop microphone is unmuted, you may get some
	very bad feedback. To drop this loopback connection when you're done, run:

	$pactl unload-module $(pactl list short modules | grep "loopback.*$BTSOURCE" | cut -f1)

	Again, replace "$BTSOURCE" with the name for the pulseaudio source that refers to your bluetooth device.

(3)	You can use BLuetooth management applications such as blueman 
	Search and Connect to your Audio source in blueman instead of step(2)

===========================================================================================================

5. Configure Remote Wakeup Function 

(1) Check your vendor and product id of bluetooth device with lsusb

	$lsusb
	Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
	Bus 002 Device 001: ID 1d6b:0001 Linux Foundation 1.1 root hub
	Bus 001 Device 002: ID 0bda:1724 Realtek Semiconductor Corp. 
	Bus 002 Device 002: ID 0e0f:0003 VMware, Inc. Virtual Mouse
	Bus 002 Device 003: ID 0e0f:0002 VMware, Inc. Virtual USB Hub
	Bus 002 Device 004: ID 0e0f:0008 VMware, Inc.

(2) Create a new rule in /etc/udev/rules.d/  with the name "90-bluetoothwakeup.rules" for example, by the way the name is not important,
	
	$sudo gedit /etc/udev/rules.d/90-bluetoothwakeup.rules
     
	insert the following codes in the file:
        SUBSYSTEM=="usb", ATTRS{idVendor}=="0bda", ATTRS{idProduct}=="1724" RUN+="/bin/sh -c 'echo enabled > /sys$env{DEVPATH}/../power/wakeup'"
 	
	In my case, the idVendor="0bda" and idProduct="1724". echo "enabled" or "disabled" to enable or disable bluetooth remote wakeup ability.
    	then save and close the file .

(3) Reinsert the RTK8723AE/AU dongle,now you can wakeup your suspend system by bluetooth device.

