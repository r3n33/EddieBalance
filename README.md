# EddieBalance
Eddie the Balance Bot is a self blanacing robot based on the Intel Edison and Sparkfun Blocks.

Currently the build details are on thingiverse: www.thingiverse.com/thing:655423

Please bear with me while I figure out github in my spare time.

Better instructions will be provided soon but for now...

EddieBalance requires libmraa to be installed. I don't have the exact steps documented for Yocto but I did derive a solution from https://learn.sparkfun.com/tutorials/installing-libmraa-on-ubilinux-for-edison

If you'd like to automatically run the Eddie software when your Edison turns on see the extras directory.

See the src/ directory for a quick ./build script and execute with ./EddieBalance

If you need help building your Eddie please contact me through thingiverse and I'll get an email back to you.

Please Note:

I am actively updating the code on this project and as I accomplish better stability and control I'll keep this updated. I'm currently working towards eliminating the motion compensation I recently introduced; Tuning a 'setpoint' PID controller to command the pitch PID controller; Improved R/C commands; Better pitch and roll accuracy; Improved timing with measurements; iOS application; Windows GUI for configuration and tuning.
