# Setup and installation

## Basic configuration

### Wi-Fi

An internet connection is needed to do most things, so we set it up first.

<https://huobur.medium.com/how-to-setup-wifi-on-raspberry-pi-4-with-ubuntu-20-04-lts-64-bit-arm-server-ceb02303e49b>

Basically, this says add your `wlan0` details to `/etc/netplan/50-cloud-init.yaml`.

I used my phone hotspot to connect as Eduroam is a pain to set up for this.

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

Now for the language.  Generate the `en_GB` locale.

```bash
sudo dpkg-reconfigure locales
```

Select `en_GB.UTF-8` and follow the prompts to exit.  Reboot yet again.  Now `locale` shows `en_GB.UTF-8` for everything.

## C

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
