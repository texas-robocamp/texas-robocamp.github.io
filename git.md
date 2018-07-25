---
title: "git & github"
tags: [git github]
keywords: Git
sidebar: tutorials
permalink: git.html
---

`git` is a distributed version control system.

What this allows you to do is <b>version control</b> your software.

We only recommend this for big teams that are far ahead 

Version control is simple.

As you write your software, you are likely to:

- Update what it does and create better versions of the same code.
- Write libraries where more than one user uses your code, and wants the latest version.
- Make mistakes and want to go back to an older version at some point.
- Want to distribute your code among your team.
  - Relatedly, want to keep your entire team in sync.
  - Relatedly, want to synchronize your work with the rest of your team.

`git` and github offer solutions to this.

## Setting up github.com

We can divide this exercise into setting up github and using git. We'll start by setting up github.

### Registering for github.com

We don't really need to explain this. Go to [github.com](github.com) and create an account, if you do not already have one or do not wish to use your existing account. If you do not wish to use your personal email address, sign up for a [gmail](gmail.com) account.

### Creating Your Organization

- Select one teammate to create your organization and repository.
- Select a name for your team.
- In the github dashboard (the front page when you go to github and are signed in) click on your name and open the drag-down menu.
  - At the bottom, you will have "Create organization."
  - Click on that.

This should bring you to a page that says "Sign up your team."

- Organization name: Make it Texas-Robocamp-\<Your Team Name\>
  - Where \<Your Team Name\> is the name of your team.
- Select a billing email that you can check.
- Pick the free tier of billing.
- Do *not* check "This account is owned by a business."
- Click "Create organization."

This will bring you to "Invite Members."

- Use this to invite your teammates, by using their usernames.

Once done, hit continue, bringing you to "Organization details."

- Just hit "skip this step" unless you want to answer the questions.

Once through this, you will get to a page with the name of your organization. You can explore this site and pick pictures to represent yourself, your team, and your teammates, or continue with "Create a new repository."

### Creating a Repository

- Click "Create a new repository."
- Pick a name for your repository. I suggest, "RobotProject."

You can only create public repositories unless you have an upgraded account, so make your repository public.

- Click "Initialize this repository with a README." It makes the next few steps easier.
- Click "Create repository."

### Setting up `ssh`

Your usage of `git` will be a lot faster and simpler if you use ssh to access it.

It is okay if the first command below fails. The rest are likely to work.

{{ site.data.alerts.terminal_commands }}
mkdir ~/.ssh
cd ~/.ssh
ssh-keygen
{{ site.data.alerts.terminal_commands_end }}

ssh-keygen will ask you a bunch of stuff.

For the first_prompt, it's asking you what to name your key.

- Name your key id_rsa_\<your_computer_name\>. The name of your computer is what comes before the `$` in the terminal. Put that in the place of \<your_computer_name\>.
- Don't pick a passphrase, so just hit `enter` to bypass this question both times.

{{ site.data.alerts.terminal_commands }}
cat id_rsa_\<your_computer_name\>.pub
{{ site.data.alerts.terminal_commands_end }}

This will dump some fairly odd looking text to your screen.

- Left-click on the terminal to drag over the text, then right click to get the option to copy it and copy it.

- Go back to the github.com website and nagivate to your settings.
  - You can find the settings by clicking on the picture at the top-right-hand corner of the screen and picking "settings" from the drop-down menu.
- Go to SSH and GPG keys
- Click "New SSH key"
  - The title of the key should be the name of the computer that you are at.
  - Paste the text that you copied above into the box.
  - Click "Add SSH key".

What you've done is configured github.com to let your computer in without a username or password by using public-key cryptography. This is not only very secure, but it allows you to access `git` without needing to type in a username and password each time.

## Using `git`
The basic functions that you will need in `git` are `clone`, `commit`, `pull`, `push`

### `clone`

When you first access a <b>repository</b>, you will `clone` it. What this does is sets up access to the repository while copying the files to your machine.

- Navigate to your repository. It will be at a URL similar to "https://github.com/Texas-RoboCamp-\<Your Team Name\>/RobotProject", except that the part that says \<Your Team Name\> will have the name of your team in it.
- There is a button that says "Clone or download"
  - Pick "Clone with SSH" and copy the text that it gives you.
  - Go to your terminal.
  - Go to your home directory (by typing `cd`).


{{ site.data.alerts.terminal_commands }}
git clone git@github.com:Texas-RoboCamp-\<Your Team Name\>/RobotProject.git
{{ site.data.alerts.terminal_commands_end }}

You can, of course, paste the URI that you copied from github.com after `git clone` to clone your repository.

### Working in `git`

Now, you will have an empty workspace on your machine. `cd` into it. Make directories for the exercises that you do.

While you're learning git, instead of working on things separately (which will force you to deal with <b>conflicts</b>), we would prefer if you simply program at one terminal as a group, and then `commit`, `push`, and `pull` as needed. With time, you will learn how `git` works, and then you'll be able to split up your work effectively.

{{ site.data.alerts.tip }}
<ul>
<li>If you work in the same file, you will cause conflicts.</li>
<li>When conflicts occur, you will have to figure out how to <b>merge</b>.</li>
<li>Instead, what we recommend is that you work on the basic version of each exercise together, and then split the work on parts where you can work entirely separately, while still all learning the concepts and having fun.</li>
<li>For instance, maybe part of your group wants to work on the LEDs and part wants to make the robot play a simple tune. Split those two things into projects, in separate directories, and while doing things like the line-following exercise as a group.</li>
</ul>
{{ site.data.alerts.end }}

### `commit`

When you are done working on your program for a while and want to <b>commit</b> your progress, you use the command `git commit` from the <b>top</b> of your repository (the directory that was created when you typed `git clone ..`.

First, you need to add the files that you've created.

- Type `git status`.

This will show you everything inside your workspace. For convenience, you can simply type: `git add -A`, which adds everything.

- Type `git add -A`.

Now you can commit. You have to say what you did when you commit. This enforces a good workflow where you know what you were doing every time you check in.

- Type `git commit -m "<message>"`
  - Where \<message\> is replaced with a brief description of what you've done.

### `pull`

Committing only created a record of what you've done on your computer. You want to share your progress with your team, so they can get your stuff to all of them. However, your team have also been working, and may have updated the repository. You need to make sure you're in sync. For this, we have `pull`.

- `git pull`

This will simply copy any updates which may be in the repository onto your machine.

You may have merged at this point. If you have, `git` will prompt you to commit. You can safely just save what they request (by hitting the escape key, then typing ":wq").

### `push`

After you commit, you will want to share your work with the rest of your team. This is accomplished by typing `git push` which will upload your changes to github.

- Type `git push`.

