# USB Camera Streaming Setup

This guide explains how to configure a USB camera to stream video over a network using the `motion` package on Ubuntu.

## Prerequisites

Before setting up the stream, ensure you have the following prerequisites:
  - A USB camera connected to the device.
  - SSH access to the machine.
  - A computer on the same network to view the stream.

## Setting Up The Stream

1.  **Install the Motion package:**
    Update your package list and install the software.

    ```bash
    sudo apt update
    sudo apt install motion
    ```

2.  **Configure the Global Settings:**
    Open the global configuration file using the nano text editor.

    ```bash
    sudo nano ~/.motion/motion.conf
    ```

    Paste these settings below:
    ```text
    stream_port 8081
    stream_localhost off
    webcontrol_port 8080
    webcontrol_localhost off
    stream_maxrate 30
    framerate 30
    width 640
    height 480
    ```

    Press `Ctrl+O` to save and `Ctrl+X` to exit.

3.  **Identify and Clear Camera Locks:**
    Before starting, ensure no other process (like a crashed instance of VLC or a background script) is holding the camera, which causes "Device or resource busy" errors.

    Check for processes using the camera:

    ```bash
    sudo fuser /dev/video0
    ```

    If a number (PID) is returned, kill that process immediately:

    ```bash
    sudo kill -9 <PID>
    ```

4.  **Launch the Motion Service:**
    Start the software with root privileges to ensure it can access the global configuration file and the hardware.

    ```bash
    sudo motion
    ```

5.  **Connect to the Video Feed:**
    Open a web browser on your Windows computer and navigate to that IP address on port 8081:
    `http://<YOUR_IP_ADDRESS>:8081`

    Where `<YOUR_IP_ADDRESS>` is the IP address used to SSH into the Jetson machine.
    You should now see the live video feed.