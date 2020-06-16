---
title: "Getting Started with Linux"
tags: [linux]
keywords: linux
last_updated: July 2, 2018
sidebar: virtual 
permalink: using_linux.html
---

## Introduction to ~~Linux~~ *should we be calling it Ubuntu? I hesitate to use the words interchangably because that would probably confuse campers*

~~The computers in this lab~~ Most robotics programming is done using **TODO** *motivate this better* the Linux operating system. Linux offers a variety of free easy-to-use tools for software development that will help us along our way in constructing and programming our robots.

### Logging In

1. ~~Enter the user name given to you, and hit Enter.~~ *They'll have this set up in their own way?*
2. On the next screen, first click on the gear and select "ubuntu" from the menu.
3. Type in the password assigned to you, hit Enter, and you should be taken to a desktop screen.
4. A box will pop up telling you about keyboard shortcuts. Click the X and move on to the interesting stuff.

### Getting to a Web Browser

By default, the Ubuntu logo will be displayed on the launcher to the left of the monitor in the bottom left-hand corner.

**TODO** *Random idea but we could totally change the icon to be Bevo or the little robocamp bot. It's apparently just /usr/share/icons/Adwaita/scalable/actions/view-app-grid-symbolic.svg*

1. Click on the grid icon.

    * This will open the "applications overview".

2. Type in either "google-chrome" or "firefox" depending on which browser you would like to use.

    * No, neither browser is better than the other for this camp. This is purely a matter of personal preference.

3. Go to <https://utcs-robotics-camp.github.io/> to get to this site. *They're already here, do we need to explain this to them?*


## Getting to a Terminal

1. Click on the applications overview button

2. Type in "terminal."

3. A terminal window will open.

## The Super Key

Much like how Windows has the start key and MacOS has the command key, Linux has what we call the `super` key. By default, this will be either your start or command key. Pressing `super` by itself will also open the applications overview.

## Changing Windows

* This is similar to operating systems such as Windows and Mac OS. Hit `alt`+`tab` or `super`+`tab` to cycle windows, or click on the icons on the launcher.

## Snapping Windows
The version of Linux we're using let's you easily maximize windows and snap them to half of the screen. To do so, simply press `super`+`arrow-key` 

- `Super`+`Up`: make the window fill the screen 

- `Super`+`Left`: Snap the window to the left half of the screen

- `Super`+`Right`: Snap the window to the right half of the screen

- `Super`+`Down`: Unsnap the window, returning it to its normal size

## Locking Frequently-Used Programs to the Launcher

1. Right-click on the application that you would like to lock to the launcher.

2. Click "Lock to Launcher."

## Basic Linux Commands

Here a few commands you will need during the course of this camp. Try them out.

Command | Example | What it does
------- | ------- | ------------
ls | `ls .` | Lists the contents of the current directory.
mkdir | `mkdir new` | Makes a directory.
cd | `cd new` | Enters a different directory.
pwd | `pwd` | Tells you what directory you are in. "Present Working Directory"

**TODO** *Since we're in charge of making the VM, it may be useful to customize the bashrc so that they can just see the cwd at all times*

When you first open the terminal you will see a command prompt. You will be in your home directory. If you type `pwd`, you will see the "path" to your home directory.

The path given to you by `pwd` is called an "absolute path." If you type an absolute path anywhere in the system, it will always refer to the same place. Absolute paths begin with "/".

A relative path is any path that does not begin with "/". Relative paths are stated relative to your present working directory. If you type "cd ex01" from your home directory, it will go to the directory "ex01" inside your home directory.

Here are a few shortcuts that you can use when forming paths.

 Shortcut | Meaning | Detail 
 ------- | ------- | ------
 ~     | Home directory.            | A space set aside for each user to store their files. 
 .     | The current directory.     |                                                       
 ..    | The parent directory.      |                                                       
 /     | The root directory.        | The very top of the computer's filesystem.            

Here's a quick exercise to try this all out.

```
mkdir linux_exercise

ls

cd linux_exercise

ls ..

cd ..

ls

mkdir linux_exercise/subdirectory

cd ~/linux_exercise/subdirectory

pwd

cd ~

```

This is basically a text-based version of double clicking on folders, but the command line is a powerful tool which you will use all week.

## Tab Completion

The command line has tab-completion. If you start typing the name of a file, directory, or command and hit tab, it will finish the name for you. If there is more than one match, all possible options are displayed.

## Text Editor

For the C++ tutorials, we will use a text editor called Sublime.

To open Sublime you can do one of two things:

* Open a terminal, type "sublime_text &", hit Enter. The & tells the computer to allow you to continue to use that terminal while the editor is running.

* Open the lens (the dots) as before, type in sublime_text, hit Enter. You do not need the & in the lens.

Using sublime and the command line, you will write and execute your tutorial programs.

*We should probably still have a recommended text editor - imho Atom is way better than sublime but idk which would be more user friendly. It might be worth spending some time exploring what's easiest, since we can install whatever we want onto the VM*

## ~~arduino IDE~~  

**TODO** *Maybe VSCode*

## Shutting Down

Before you leave the lab, make sure you save all your work and log out of the computer so that it's free for the next person to use it. Open the account menu in the top right-hand corner of your screen, click the shut down icon, and select shut down from the power options.

{% include note.html content="Please do not unplug the USB before your computer has fully shut down, as this may corrupt files on the USB. If you are experiencing technical difficulties, notify one of the camp staff."%}

## Next Step

**TODO** *This link isn't real*
Proceed to ["Introduction to the Robot"](/robot_introduction.html)
