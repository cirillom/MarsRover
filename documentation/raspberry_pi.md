# Raspberry Pi

This document provides instructions for setting up and using the Raspberry Pi for development and deployment.

We used a Raspberry Pi 4 Model B with 8GB RAM for our setup, but this should work with other models as well.

## Setup
1. Download and Install [Raspberry Pi Imager](https://www.raspberrypi.org/software/).
2. Before writing the SD card, you can configure advanced options (like enabling SSH, setting hostname, Wi-Fi credentials, etc.) by pressing `Ctrl + Shift + X` in the Raspberry Pi Imager.
3. Insert your microSD card into your computer and use the Raspberry Pi Imager to write Ubuntu Desktop 22.04.5 LTS (64-bit) image to the SD card.
4. Once the imaging process is complete, safely eject the SD card from your computer and insert it into the Raspberry Pi.
5. Connect a monitor using the micro HDMI port (the lab should have one), keyboard, and mouse to the Raspberry Pi.
6. Boot up the Raspberry Pi and follow the on-screen instructions to complete the initial setup. Ideally you should connect in a Wi-Fi network created by a computer hotspot.

## SSH Configuration
### Enabling SSH on Raspberry Pi
1. Install OpenSSH Server to enable remote access:
   ```bash
   sudo apt update
   sudo apt install openssh-server
   ```
2. Start and enable the SSH service:
   ```bash
   sudo systemctl start ssh
   sudo systemctl enable ssh
   ```
3. Enable UFW firewall and allow SSH connections:
   ```bash
   sudo ufw allow ssh
   sudo ufw enable
   ```
### Enabling SSH on Windows
Windows 10 and later versions come with a built-in SSH client. To enable it:
1. Open "Settings" and go to "Apps".
2. Click on "Optional Features".
3. Scroll down and look for "OpenSSH Client". If it's not installed, click on "Add a feature" and install it.
4. Once installed, you can use the Command Prompt or PowerShell to connect to your Raspberry Pi using SSH:
   ```bash
   ssh username@raspberry_pi_ip_address
   ```
Replace `username` with your Raspberry Pi username and `raspberry_pi_ip_address` with the IP address of your Raspberry Pi.

### Finding the Raspberry Pi's IP Address
To find the IP address of your Raspberry Pi, you can use one of the following methods:
1. On the Raspberry Pi, open a terminal and run:
   ```bash
   ip a | grep inet
   ```
However this method needs a monitor connected to the Pi, so if you want to avoid that, you should use create a hotspot on your computer and connect the Pi to it. Then you can find the IP address from your computer:
1. Go to the network settings on your computer.
2. Look for connected devices in the hotspot settings to find the Raspberry Pi's IP address.

