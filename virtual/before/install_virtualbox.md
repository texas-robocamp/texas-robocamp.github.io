# Part 1: Installing VirtualBox
On Windows:
1. Set up your BIOS for virtualization (the actual steps may vary slightly for your computer):
    * Restart your computer.
    * On a keyboard, repeatedly press F10/F12 as your system is starting up (before the Windows logo appears). Note that wireless keyboards may not work for this.
    * Go to the Configuration/Advanced Settings page.
    * Find the Virtualization/Virtual/SVM option (the name may be different) and turn it on.
    * Save the changes and restart.
2. Go to https://www.virtualbox.org/wiki/Downloads.
3. Under the section for the latest version (6.1.10 at the time of writing), click the download link for “Windows hosts”.
4. Open the installer.
5. Follow the instructions to install VirtualBox.

On macOS:
1. Go to https://www.virtualbox.org/wiki/Downloads.
2. Under the section for the latest version (6.1.10 at the time of writing), click the download link for “OS X hosts”.
3. Open the installer and double-click the “Install VirtualBox” file. (TODO double-check the name)
4. Follow the instructions to install VirtualBox.

{{ site.data.alerts.important }}
The first time you install VirtualBox, you will see a pop-up saying something along the lines of “System software from Oracle America was blocked”. If you see this message:
* Open System Preferences.
* Select the Security menu.
* Click “Allow”.
{{ site.data.alerts.end }}

# Part 2: Creating the virtual machine
1. Start downloading the installer for Ubuntu 18.04.4 from https://releases.ubuntu.com/18.04.4/ubuntu-18.04.4-desktop-amd64.iso (this is a large file, and will take some time to download).
2. Open VirtualBox.
3. Click the “New” button to create a virtual machine.
4. In the window that appears:
    * Specify “Texas Robocamp 2020” for the name.
    * Leave the “Machine Folder” as-is.
    * Set the type to “Linux” and the version to “Ubuntu (**64-bit**)”. (Note that VirtualBox tries to predict these settings based on the name - you will need to correct them after you enter the name.)
    * Click Continue.
4. Set the slider for the amount of memory to half the maximum value (which should be the amount of RAM on your system) and click Continue.
    * If you have 32 Gigabytes / 32768 MB of RAM, this will be 16384 megabytes.
    * If you have 16 Gigabytes / 16384 MB of RAM, this will be 8192 megabytes.
    * If you have 8 Gigabytes / 8192 MB of RAM, this will be 4096 megabytes.
5. Select “Create a virtual hard disk now” and click Create.
6. Select “VDI” and click Continue.
7. Select “Fixed-size” and click Continue.
8. Set the size to 15 GB and click Create (leave the file name and location as-is).
9. Click the Settings button.
10. Under the System tab, go to the Processor section. Set the number of cores as high as the green line reaches. Leave the other settings alone.
11. Under the Display tab, go to the Display section. Set the Video Memory setting as high as possible. Leave the other settings alone.
12. Click OK.

# Part 3: Installing Ubuntu
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

# Part 4: Installing the Guest Additions
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

# Part 5: Installing ROS
Press Control+Alt/Option+T to open a terminal.
* Another way to open the terminal is to click on the icon with nine dots in the lower left-hand corner of the screen (3 rows x 3 columns of dots). After clicking on this icon, type “terminal” and click on the “terminal” icon.

Then enter these commands, entering the password (robocamp2020) when prompted.
{{ site.data.alerts.note }}
* By default, Ubuntu uses a separate clipboard from the rest of your computer. You can merge them by going to the VirtualBox menu, selecting “Devices” > “Shared Clipboard”, and selecting “Bidirectional”.
* If you prefer, you can navigate to this set of instructions using the web browser inside the virtual machine, and then copy and paste from there.
* Enter the next command when you see the green line with the $ at the end. Only copy one line at a time.
* You will need to enter the password (robocamp2020) when prompted. Nothing will appear on the screen when you do so, not even dots; this is normal.
{{ site.data.alerts.end }}

* `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`
* `sudo add-apt-repository http://packages.ros.org/ros/ubuntu`
* `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743`
* `sudo add-apt-repository http://packages.osrfoundation.org/gazebo/ubuntu-stable`
* `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 4F96EF95D295866724CAEEDA0540E766C789458D`
* `sudo add-apt-repository https://texas-robocamp.github.io/packages-virtual`
* `sudo apt update`
* `sudo apt upgrade -y`
* `sudo apt install -y ros-melodic-texas-robocamp-full`
* `sudo snap install --classic code`
* `sudo rosdep init`
* `rosdep update`

{{ site.data.alerts.note }}
Ubuntu includes Firefox by default. If you would prefer to use Google Chrome, also enter these commands:
* `wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb`
* `sudo apt install -y ./google-chrome-stable_current_amd64.deb`
{{ site.data.alerts.end }}

# Part 6: Testing your installation
In your terminal window, type these commands:

* `source /opt/ros/melodic/setup.bash`
* `roslaunch texas_robocamp test_world.launch`

You should see two windows appear. One will have the UT logo and our simulated robot in the middle.

![Image of a successful install](images/successful_install.png)

Open a new terminal by right clicking in the terminal window and clicking “open tab”. Type in this command to drive the robot:

* `source /opt/ros/melodic/setup.bash`
* `rosrun texas_robocamp teleop_texbot`

Use the keys listed on the screen to drive the robot around. (You must have this terminal selected for them to work.)
At the bottom of the simulator, you will see a bar with a bunch of numbers. Write down the number next to the FPS label - we may use this to check performance. (It will be changing; write down an approximate value.)

If everything looks good, go back to the terminal and press Control+C in both windows to shut down the testing system and wait for all the windows to close completely. Once you see a “done” message, you can shut down the virtual machine:

* Click the arrow in the top-right corner of the screen.
* Click the power icon.
* Click Power Off.


Email us to tell us that you have successfully installed your software for the camp, or to report any problems. We can be reached at texas.robocamp@gmail.com

