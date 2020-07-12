# Installation
## Installing with a Thumb Drive
TODO
## Installing with a Virtual Machine

### What is a Virtual Machine?

A Virtual Machine (VM) is like a computer inside of your computer. It is an environment that runs like a normal program on your machine, but it emulates a fully-functional operating system, just like if you bought another computer! It has its own virtual hard drive, memory, CPU, and devices that it creates using resources from your computer. 

We will be creating a virtual machine that we can use to run and develop software from during the course of the camp.

### Preconfiguration (for Windows)
Before we get started, if you have a Windows machine, you will have to first allow your computer to use its resources to create a virtual machine by enabling virtualization.

1. Restart your computer.

2. On a keyboard, repeatedly press F2 (or F10 for some machines) as your system is starting up, before the Windows logo appears. Note that wireless keyboards may not work for this.

3. Go to the Configuration/Advanced Settings page.

4. Find the Virtualization/Virtual/SVM option (the name may be different) and turn it on.

5. Save the changes and restart.

![Image of BIOS Menu](images/bios.png) 

{{ site.data.alerts.note }}
The officially supported virtual machine software for this camp is **VirtualBox**. If you would prefer to use **VMWare** (an alternate software) instead, there are installation instructions for trial versions after this section. Both will work for the purposes of this camp.
{{ site.data.alerts.end }}


### **Installing VirtualBox**
### Downloading
**Windows:**
1. Go to https://www.virtualbox.org/wiki/Downloads.

2. Under the section for the latest version (6.1.10 at the time of writing), click the download link for “Windows hosts”.

3. Open the installer.

4. Follow the instructions to install VirtualBox.

**Mac:**
1. Go to https://www.virtualbox.org/wiki/Downloads.

2. Under the section for the latest version (6.1.10 at the time of writing), click the download link for “OS X hosts”.

3. Open the installer and double-click the “VirtualBox.pkg” file.

4. Follow the instructions to install VirtualBox.

{{ site.data.alerts.important }}
The first time you install VirtualBox, you may see a pop-up saying something along the lines of “System software from Oracle America was blocked”. If you see this message:
* Open System Preferences.

* Select the Security and Privacy menu.

* Click “Allow”.

{{ site.data.alerts.end }}

### Creating the virtual machine
1. Start downloading the installer for Ubuntu 18.04.4 from https://releases.ubuntu.com/18.04.4/ubuntu-18.04.4-desktop-amd64.iso (this is a large file, and will take some time to download).

2. Open VirtualBox.

3. Click the “New” button to create a virtual machine.

4. In the window that appears:
    * Specify “Texas Robocamp 2020” for the name.

    * Leave the “Machine Folder” as-is.

    * Set the type to “Linux” and the version to “Ubuntu (**64-bit**)”. (Note that VirtualBox tries to predict these settings based on the name - you will need to correct them after you enter the name.)

    * Click Continue.

5. Set the slider for the amount of memory to half the maximum value (which should be the amount of RAM on your system) and click Continue.
    * If you have 32 Gigabytes / 32768 MB of RAM, this will be 16384 megabytes.

    * If you have 16 Gigabytes / 16384 MB of RAM, this will be 8192 megabytes.

    * If you have 8 Gigabytes / 8192 MB of RAM, this will be 4096 megabytes.

6. Select “Create a virtual hard disk now” and click Create.

7. Select “VDI” and click Continue.

8. Select “Fixed-size” and click Continue.

9. Set the size to 15 GB and click Create (leave the file name and location as-is).

10. Click the Settings button.

11. Under the System tab, go to the Processor section. Set the number of cores as high as the green line reaches. Leave the other settings alone.

12. Under the Display tab, go to the Display section. Set the Video Memory setting as high as possible. Leave the other settings alone.

13. Click OK.

Once you are done, your screen should look something like this.

![Image of VirtualBox Menu](images/virtualboxmenu.png) 

### Installing Ubuntu
1. Select the “Texas Robocamp 2020” virtual machine and click Start.

2. Click the folder icon. Click “Add” and select the Ubuntu ISO file you downloaded earlier.

3. Click Choose, then click Start.

4. Once you get to the screen that says “Try Ubuntu”/”Install Ubuntu”, click the Install button.

5. If you use a custom keyboard layout, select the layout you use. Otherwise, click Continue.

6. Select “Minimal installation”, “Download updates while installing”, and “Install third-party software for graphics”. Then click Continue.

7. Select “Erase disk and install Ubuntu”. Click “Install Now”. On the pop-up, select “Continue”.
    {{ site.data.alerts.note }}
    Don’t worry about the warning - this won’t affect the rest of your computer. The installer only has access to the virtual machine you created in VirtualBox.
    {{ site.data.alerts.end }}

8. Select your local time zone on the map and click Continue.

9. Fill in your name. Set the username to “robocamp” and the password to “robocamp2020” (without the quotes). Select “Log in automatically” and click Continue.

10. Once the install finishes, click Restart.

11. When the screen says “Remove the installation medium”, check that the circular CD icon in the bottom bar of the VirtualBox window is gray. If not, click it and click Remove. Once the icon is gray, press Enter.

12. On the main screen, click Next on each screen until you get to the “You’re ready to go!” screen. Click Done.

13. If you see a pop-up saying “Updated software has been released”, close the window - they will be automatically installed as part of the setup process.

Your screen should look something like this when your installation is complete.

![Image of Ubuntu Desktop](images/ubuntudesktop.png) 

### Installing the Guest Additions
1. Go to the VirtualBox menu bar. Under the “Devices” menu, click “Insert Guest Additions CD image”.

2. If you don’t see the menu, make sure you are still in the virtual machine window.
    {{ site.data.alerts.note }}
    The first time you do this, you will see a series of prompts to download it. Just click OK at each step.
    {{ site.data.alerts.end }}

3. A popup will appear in Ubuntu. Click Run and enter your password.

4. Once the “Press Return to close this window” message appears, press Enter.

5. Restart the virtual machine by clicking the arrow in the top-right corner and clicking the Power button. Select Restart in the popup.
    {{ site.data.alerts.tip }}
    Whenever you are done using VirtualBox, you should turn off the virtual machine through this menu. Instead of clicking Restart, you will click Power Off.
    {{ site.data.alerts.end }}

{{ site.data.alerts.important }}
Congrats! You have now successfully set up your VirtualBox system. Now move to the bottom of this page to install the Robocamp software.
{{ site.data.alerts.end }}


### **Installing VMWare** *(Alternative for VirtualBox)*
### Downloading
1. Start downloading the installer for Ubuntu 18.04.4 from https://releases.ubuntu.com/18.04.4/ubuntu-18.04.4-desktop-amd64.iso (this is a large file, and will take some time to download).
2. Download and run the VMWare Installer (Fusion for Mac, Workstation Player for Windows) and walk through the installation prompts on the screen
    * **Mac**: 
        * Go to https://www.vmware.com/products/fusion/fusion-evaluation.html and click "Try Fusion 11.5"

        * Open the installer and double click the icon to start installation

    * **Windows**: Download VMWare Workstation Player from https://www.vmware.com/products/fusion/fusion-evaluation.html and run the installer
3. Follow the instructions to install VMWare with defaults selected
    * **Mac**:
{{ site.data.alerts.important }}
The first time you install VMWare, you may see a pop-up saying something along 	the lines of “System software was blocked”. If you see this message:
        * Open System Preferences.
        * Select the Security and Privacy menu.
        * Click “Allow”.
{{ site.data.alerts.end }}
             * You will also be prompted to choose to Try VMWare Fusion or VMWare Fusion Pro, choose the first option

             * Continue with the rest of installation normally, with default options selected

             * At the end of installation, go back to System Preferences -> Security and Privacy -> Privacy -> Full Disk Access and check VMWare. 

        	   ![Image of Mac security](images/macsecurity.png) 
      * **Windows**:
          * Continue with default options selected until the “Custom Setup” screen

          * Check the option to include the Enhanced Keyboard Driver

		      ![Image of Custom setup screen](images/vmwaresetup.png) 

          * Continue with the rest of the setup normally (with default settings)

          * Restart your computer to finish installation

### Creating the virtual machine
1. Open up VMWare

2. Click on “Create a New Virtual Machine” (if the installation screen doesn’t come up automatically)

3. Find and add your downloaded Ubuntu ISO File (drag and drop for Mac)

![Image of Windows VMWare Wizard](images/vmwizardwindows.png) 

![Image of Mac VMWare Wizard](images/vmwizardmac.png)

4. Make sure easy install is checked

5. Set Username and Password
    * Username and Display Name: **robocamp**

    * Password: **robocamp2020**

6. Customize your settings
    * **Mac**:
        * Finish creating your VM and the option to configure settings will automatically come up upon launch

        * You can also press the wrench icon at the top of the VM Window to access settings
	 	![Image of Mac Settings](images/macsettings.png)

    * **Windows**:
        * Option to set Hard Disk Size will show up in creation process

        * Set Hard disk size to 15 GB

        * Select “Store virtual disk as a single file”
		
		 ![Image of Windows VMWare HD setup](images/windowshd.png)

        * Option to configure settings will pop up during VM creation process, you can do so by clicking “Customize Hardware”

		 ![Image of Windows settings](images/windowssettings.png)

7. **(Mac)** Go to Hard Disk and set 15 GB Hard Drive Space 
     * Go to advanced settings and select “Pre allocate disk space” and unselect “Split into multiple files”, then click apply

8. Go to Memory (or Processors & Memory for Mac) and set 4GB (4096MB) memory, or about half of your available RAM
    * **Mac**: When you select your RAM, move the slider to the center

    * **Windows**: Make sure the amount of RAM you set is below the blue mark labeled “Maximum recommended memory”

9. Go to Processors (or Processors & Memory for Mac) and set processor cores to 2 (or more if your computer has more available)

10. Go to Display and uncheck “Accelerate 3D Graphics” (unless you have a discrete GPU)

11. Close settings and launch VM (it should launch automatically)

12. If prompted to Download/Install VMWare Tools, press “Download and Install”

13. Wait for installation to complete automatically, then log in and press next on all the prompts for the “Welcome to Ubuntu” window
    * If prompted to install Software Updates, just close the window (we will install them anyway in the process)

{{ site.data.alerts.tip }}
When starting the VM, you may get a message *“Cannot connect virtual device because no corresponding device is available on the host."* This is because your machine doesn't have a CD/DVD drive. Press yes to bypass the message.
{{ site.data.alerts.end }}

Your screen should look something like this when your installation is complete.

![Image of Ubuntu Desktop](images/ubuntudesktop.png) 

{{ site.data.alerts.important }}
Congrats! You have now successfully set up your VMWare system. Now move to the bottom of this page to install the Robocamp software.
{{ site.data.alerts.end }}

## Setting up Robocamp Software
{{ site.data.alerts.note }}
Only complete this section after you have successfully downloaded and installed Ubuntu 18.04 from a thumb drive or virtual machine.
{{ site.data.alerts.end }}
### Installing ROS
Press Control+Alt/Option+T to open a terminal.
* Another way to open the terminal is to click on the icon with nine dots in the lower left-hand corner of the screen (3 rows x 3 columns of dots). After clicking on this icon, type “terminal” and click on the “terminal” icon.

Then enter these commands, entering the password (robocamp2020) when prompted.
{{ site.data.alerts.note }}
* By default, Ubuntu uses a separate clipboard from the rest of your computer. You can merge them by going to the VirtualBox menu, selecting “Devices” > “Shared Clipboard”, and selecting “Bidirectional”.
* If you prefer, you can navigate to this set of instructions using the web browser inside the virtual machine, and then copy and paste from there.
    * Copy from and paste into the terminal using Ctrl-Shift-C and Ctrl-Shift-V.
* Enter the next command when you see the green line with the $ at the end. Only copy one line at a time.
* You will need to enter the password (robocamp2020) when prompted. Nothing will appear on the screen when you do so, not even dots; this is normal.
{{ site.data.alerts.end }}

{{ site.data.alerts.terminal_commands }}
**Adding ROS Packages**
* `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
* `sudo add-apt-repository http://packages.ros.org/ros/ubuntu`

**Adding Gazebo Packages**
* `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743`
* `sudo add-apt-repository http://packages.osrfoundation.org/gazebo/ubuntu-stable`

**Adding Robocamp Packages**
* `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 4F96EF95D295866724CAEEDA0540E766C789458D`
* `sudo add-apt-repository https://texas-robocamp.github.io/packages-virtual`

**Updating and upgrading packages**
* `sudo apt update`
* `sudo apt upgrade -y`

**Installing software**
* `sudo apt install -y ros-melodic-texas-robocamp-full`
* `sudo snap install --classic code`

**Setting up dependencies for ROS**
* `sudo rosdep init`
* `rosdep update`
{{ site.data.alerts.terminal_commands_end }}

{{ site.data.alerts.note }}
Ubuntu includes Firefox by default. If you would prefer to use Google Chrome, also enter these commands:
		{{ site.data.alerts.terminal_commands }}
		* `wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb`
		* `sudo apt install -y ./google-chrome-stable_current_amd64.deb`
		{{ site.data.alerts.terminal_commands_end }}
{{ site.data.alerts.end }}

### Testing
Close the current terminal and open up a new one using Ctrl-Alt-T.
{{ site.data.alerts.terminal_commands }}
* `source /opt/ros/melodic/setup.bash`
* `roslaunch texas_robocamp test_world.launch`
{{ site.data.alerts.terminal_commands_end }}

You should see two windows appear. One will have the UT logo and our simulated robot in the middle.

![Image of a successful install](images/successful_install.png)

Open a new terminal by right clicking in the terminal window and clicking “open tab”.

{{ site.data.alerts.terminal_commands }}
* `source /opt/ros/melodic/setup.bash`
* `rosrun texas_robocamp teleop_texbot`
{{ site.data.alerts.terminal_commands_end }}

Use the keys listed on the screen to drive the robot around. (You must have this terminal selected for them to work.)
At the bottom of the simulator, you will see a bar with a bunch of numbers. Write down the number next to the FPS label - we may use this to check performance. (It will be changing; write down an approximate value.)

{{ site.data.alerts.tip }}
If your FPS is very low (below 10) or you would like to improve it, try resizing the window down. 
{{ site.data.alerts.end }}

If everything looks good, go back to the terminal and press Control+C in both windows to shut down the testing system and wait for all the windows to close completely. Once you see a “done” message, you can shut down the virtual machine:

* Click the arrow in the top-right corner of the screen.

* Click the power icon.

* Click Power Off.

{{ site.data.alerts.important }}
Email us to tell us that you have successfully installed your software for the camp (include your FPS!), or to report any problems. We can be reached at *texas.robocamp@gmail.com*. **Also make sure to cc *camp@cs.utexas.edu*, or else we will not be able to send a reply!**
{{ site.data.alerts.end }}



