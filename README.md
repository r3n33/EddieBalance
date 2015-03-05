# EddieBalance
Eddie the Balance Bot is a self blanacing robot based on the Intel Edison and Sparkfun Blocks.

This project now reflects my EddiePlus design which incorporates encoders and some new body stylings. The new 3D models and details including a builder's guide are available on thingiverse: www.thingiverse.com/thing:694969

EddieBalance is currently running the 05-15 Yocto image from Intel and requires the installation of libmraa (instructions to come in due time).

EddieBalance listens for data on two ports; one for gaining control and the other for sending commands. And Eddie returns data to the last IP received on the response port.

* Control port: UDP 4240
* Command port: UDP 4242
* Response port: UDP 4243

To remote control Eddie you must gain control with a "bind" process that involves:

* Sending "DISCOVER" to 255.255.255.255 on port 4240 - All Eddie's on the network will respond with "DISCOVER: [unique name]".

* Using the IP/name from the response send "BIND" to [ip address] on port 4240 - Eddie will respond with "BIND: OK".

* Now you can send command packets (currently listed in main.c) to UDP port 4242.

The bind process was a necessary evil to allow multiple Eddie robots on the same WiFi network. I've been unable to get the Edison's AccessPoint mode to work with the GPIO issue in motordriver.c and Bluetooth control is next on my list of features.

The build details for the original Eddie still remain on thingiverse (but I highly recommend building a balance bot with encoders): www.thingiverse.com/thing:655423

If you need help building your Eddie please contact me through thingiverse and I'll get an email back to you.
