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

Enable WiFi and camera (interfacing options) using configuration menu opened by running

```
sudo raspi-config
```



### 3. Python 2 dependencies 

All Python dependencies are listed in files *requirements_host.txt* and *requirements_raspi.txt*. Former is meant for the host PC and latter for the Raspberry. The project uses **Python 2**. Raspbian Stretch comes preinstalled with 2.7 and 3.5.

Install by running commands (~12min)

#### Raspberry

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python-pip
pip install -r requirements_raspi.txt
```

Note: **Requires internet connection**. Alternatively the user may download required dependencies using another machine and then copying the files to Raspberry's SD-card for installation. [Files can be downloaded from PyPI](https://pypi.org/).

Note: Using SSH one can send a folder by running the following command on host machine

```
scp -r /path/to/source/code/ pi@192.168.1.70:~/code
```

Change paths and IP's to match your setup.

#### Test the system

Start the camera server by running (in the project's root folder)

```
python start_camera.py
```

If no errors are are raised and the script prints "waiting for connection" or similar then the system works and you can exit the command (ctrl+c)  and move to the next step.



### Setup camera server

The camera server should automatically run when the system is booted. This is done by adding file *start_camera.py* to bootable paths. 

Alternatively you could run *start_camera.py* manually every time you wish to start the server.

Open file

```
sudo nano /etc/rc.local
```

Add following line above the *exit 0* line

```
sudo -H -u pi python /home/pi/robot-lego-assembler/start_camera.py &
```

It will run the script as user *pi* for which the python dependencies were installed. <u>If you are using another user replace "pi" with it.</u>

Make sure to change the path to correspond your setup. The path must be absolute. Do not forget the ampersand ("&") as otherwise the system will hang.



# Optional

## Cloud9 IDE

[Instructions](https://www.siaris.net/post/cloud9/) to install Cloud9 development environment.

