# Setup and installation

## Basic configuration

### Wi-Fi

An internet connection is needed to do most things, so we set it up first.

<https://huobur.medium.com/how-to-setup-wifi-on-raspberry-pi-4-with-ubuntu-20-04-lts-64-bit-arm-server-ceb02303e49b>

Basically, this says add your `wlan0` details to `/etc/netplan/50-cloud-init.yaml` by appendding the following:

```text
...
    wifis:
        wlan0:
            optional: true
            access-points:
                "MyWiFi":
                    password: "MyPassword"
            dhcp4: true
```

Substitute correct SSID and password. I used my phone hotspot to connect as Eduroam is a pain to set up for this.

After saving the modified file, reboot and check the `wlan0` IP address with `ip addr`.

### Locale

By default, the keyboard is US layout and the locale is `C.UTF-8`.  This drives me mad, so change the keyboard layout first.

<https://askubuntu.com/questions/434849/change-keyboard-layout-english-uk-on-command-line-to-english-us>

Edit `/etc/default/keyboard` to look like this:

```text
# KEYBOARD CONFIGURATION FILE

# Consult the keyboard(5) manual page.

XKBMODEL="pc105"
XKBLAYOUT="gb"
XKBVARIANT=""
XKBOPTIONS=""

BACKSPACE="guess"
```

Then reboot.

Now for the language.  Generate the correct locale using the command:

```bash
sudo dpkg-reconfigure locales
```

Select `en_GB.UTF-8` and follow the prompts to exit.  Reboot yet again.  Now `locale` shows `en_GB.UTF-8` for everything and the keyboard should be correct.

## Update kernel to 6.2

Install Andy's magic scripts to make using Git easier.

```bash
mkdir ~/git
cd ~/git
git clone https://github.com/andyblight/bash_scripts.git
cd bash_scripts
./install.sh ubuntu22.04lts
```

Open a new shell and when you are in a git repo, you will see the branch name in parentheses on the prompt.

Automatic updates just mess things up on a robot, so we disable them as follows:

Edit `/etc/apt/apt.conf.d/20auto-upgrades` and modify the two lines to look like this

```text
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0"
```

Do a full upgrade.

```bash
upgrade.sh
```

You'll probably need to reboot after this upgrade.

Now download this repo using:

```bash
cd ~/git
git clone https://github.com/RealRobotics/pi-rugby-ball.git
cd pi-rugby-ball
```

Install the 6.2 kernel using the following script:

```bash
./update_kernel.bash
```

This script adds the `lunar` release files, installs the 6.2 kernel, reverts the `source.list` file and then reboots. After the reboot, verify that the new kernel is installed as follows:

```bash
$ uname -a
Linux ball-desktop 6.2.0-1017-raspi #19-Ubuntu SMP PREEMPT Mon Nov 13 15:35:19 UTC 2023 aarch64 aarch64 aarch64 GNU/Linux
```

Finally, comment out the newly added lines in the file `/etc/apt/sources.list` and then do `sudo apt update` to refresh the index.  This allows us to update and install code from 22.04LTS but still use a much later kernel.

## Build `libcamera` and tools

Clone this repo and install the `libcamera` and related code:

```bash
cd ~/git
git clone https://github.com/RealRobotics/rpi-kb.git
cd rpi-kb/ubuntu/pi_camera
./install_arducam.bash
```

Now we need to tell the system to use the correct overlay for the camera.  Open the config  file using

```text
sudo nano /boot/firmware/config.txt
```

Find the line:

```text
camera_auto_detect=1
```

and change it to:

```text
camera_auto_detect=0
dtoverlay=imx708
```

Save and reboot.

Finally, test the camera using these commands:

```bash
# Display list of cameras.
cam --list
# Show 5 seconds of video on screen
rpicam-hello
```

If these two work, then you have successfully set up your Raspberry Pi.  Time for a nice cup of tea!

## Configure the environment

To set up the software for the rugby ball, run the following:

```bash
mkdir -p ~/git
git https://github.com/RealRobotics/pi-rugby-ball.git
cd ~/git/pi-rugby-ball/
install/install_rpi.bash
workspace/setup_ws.bash
```

Note: This install and build can take several minutes and can overheat a RPi4B.  Make sure a fan is fitted!

Subsequent builds can be done using the the script [build.bash](build.bash).

Once this is complete, run the project using:

```bash
cd ~/ball_ws
colcon build
ros2 launch pi_rugby_ball all.launch.py
```
