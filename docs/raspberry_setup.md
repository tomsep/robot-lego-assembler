# Raspberry Pi 3 and PiCamera

## Installing operating system (Raspbian Stretch)

1. Download Raspbian Stretch (Desktop | Lite).
2. Use Etcher (or similar program) to flash the image to the SD-card.
3. To enable SSH add an empty file named `ssh.txt` to the root directory of the SD-card. (for headless setups)

## Configuring Raspberry (headless)

### 1. Wired connection

1. Connect Raspberry to a router using ethernet cable.
2. Using IP assigned to Raspberry SSH into it using PuTTY or similar program.
3. Log in to Raspberry using username: pi and password: raspberry

### 2. Wireless connection and enabling PiCamera

Enable WiFi and camera using configuration menu opened by running

```
sudo raspi-config
```



### 3. Python dependencies 

Note: **Requires internet connection**. Alternatively the user may download required dependencies using another machine and then copying the files to Raspberry's SD-card for installation. [Files can be downloaded from PyPI](https://pypi.org/).

Run 

```
pip install picamera pyyaml
```



### 4. Copy project source code to Raspberry

Using SSH one can send a folder by running on host machine

```
scp -r /path/to/source/code/ pi@192.168.1.70:~/code
```

Change paths and IP's to match your setup.

## 