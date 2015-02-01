# EddieBalance
Eddie the Balance Bot

Extras Directory

EddieStartup.service is a basic service example to have Eddie automatically run at startup. To use this you must ensure the file location in the script is correct for your system and follow these instructions:

Copy: EddieStartup.service to /lib/systemd/system/

Execute: systemctl daemon-reload

Execute: systemctl enable EddieStartup.service

When you boot the latest build of EddieBalance will run automatically. 

To debug or rebuild you can use the StopService script.
