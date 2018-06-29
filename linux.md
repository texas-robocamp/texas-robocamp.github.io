Getting Ready to Code

Before you can even start writing your first program, you'll have to complete a number of small tasks. The first one is logging in.

Logging In

The computers in this lab use the Linux operating system. So...

Welcome to Linux!

Enter the user name given to you, and hit Enter.
On the next screen, first click on the gear and select xfce from the menu.
Insert the password assigned to you, hit Enter, and you should be taken to a desktop screen.
If a dialog box appears asking you about what configuration to use, select the option with Default.
Linux Commands

An important part of using Linux is becoming familiar with the command line. The following exercise will get you started with a few commands you'll find useful in the coming week.

ls is the command to list the contents of the current directory. When you first open the terminal and have a command prompt, you will be in your home directory. The home directory is denoted by a ~. Try typing ls into the command line now and hitting Enter to see what the contents of your home directory are.
cd is the command to change directories. Choose one of the directories you saw listed when you entered the ls command and type cd <chosen_directory> into the command line (without angle brackets). Hit Enter.
(Note that the command line has the idea of tab-complete. If you start typing the name of a unique file, directory, or command and hit tab, it will finish the name for you. If there is more than one match, all possible options are displayed.)
When you're using a terminal to navigate the filesystem, you may want to go back up a directory level to the parent directory. Just as your home directory has the ~ alias, the parent directory of the directory you are currently in has the alias .. (yes, that is a dot dot). Type cd .. into the command line and hit Enter to return to the parent directory, which, in this case, is your home directory.
pwd will tell you what directory you are currently in. Since you started this little exercise in your home directory, typing pwd at the command line and hitting Enter will tell you that you ended in your home directory, though not with a simple tilde; pwd gives you the full, absolute path. Likely, your terminal has already been configured to display this information at the start of its command prompt, but if not, pwd is a good way to keep from getting lost. Also at any point, you may enter the command cd to return to the home directory.
The above is essentially a command-based version of what you're used to doing by double clicking on folders. The command line is capable of a great deal more---in fact, you will be using it to compile your tutorial program! But more on that later. First, you'll need to write some code.

Text Editor

In this lab, we'll be using a text editor called sublime to write our tutorial program in C++. We'll use a different environment for the project. To open sublime, open up a terminal window, type in sublime_text & and hit Enter. The & tells the computer to allow you to continue to use that terminal while the editor is running.

Using sublime and the command line, you will be able to write and execute your tutorial programs.

Logging Out

Before you leave the lab, make sure you save all your work and log out of the computer, so that it's free for the next person to use it. Open up the account menu in the top right-hand corner of your screen, and click to log out.

Also, please do not reboot the computer. If you are experiencing technical difficulties, resist the urge to try turning the machine off and on again, and instead notify one of the camp staff.
