# UT RoboMaster Software


### The Game

Robomaster at heart plays like a combination of an FPS shooter and League of Legends. Each team gets a set of robots, which they control through a GUI on a computer using WASD and a mouse. Each robot has an amount of health, ammo, and, depending on the game format, potentially a level-up system or respawn system. Operators can drive and control these robots to shoot at opponents' armor plates. Shooting armor plates is the primary way to deal damage to other robots and is a key factor in the game.

There are two primary formats in RMUL (our competition): a 1v1 format, where two standard robots battle to death, and a more complicated 3v3 format. The 1v1 format is straightforward: two standard robots on a playing field with some terrain shoot at each other until one runs out of health. 

The 3v3 format is more like League of Legends. There are three bots on the field: the standard, the hero, and the sentry robots. There is also a home base with a set amount of health that, when destroyed, ends the match. The standard and the hero are human-controllable, while the sentry is fully autonomous. The standard is meant to be designed for dueling, shooting smaller projectiles that deal less damage.

In contrast, the hero is designed to be a heavy hitter, intended primarily to deal extra damage to the opposing home base. Finally, the fully autonomous sentry is usually used to defend the home base against opposing robots. These robots must maneuver and kill the opponent's robots to reach the opponent's home base to win the game.

[Videos in drive](https://drive.google.com/drive/folders/16SahyUM0eKlh3Aw-9YJ7O1C8-M2Ie7DB?usp=drive_link)

## Software vs Firmware

This repository contains the code for the software sub-team for UT RoboMaster. The software sub-team is one of the two
software-focused teams in RoboMaster, the other being the firmware sub-team. The distinction between the two teams can
be roughly understood as firmware handles the direct control of the robot, while software runs on a separate board to
try to provide more information or autonomy to the robot and operators.

## Software Goals
Hopefully, if not already apparent, the sentry robot should become one of the primary focuses of software. As it is fully autonomous and cannot communicate with other bots whatsoever, it must be able to defend the base on its own using computer vision and similar algorithms. We want to provide inputs to the sentry to tell it when and who to shoot and potentially have it be able to navigate the field on its own to either evade attack or better defend the base.

Similar capabilities would be helpful for both standard and hero systems. Auto-aim in these systems could help deal with the other team's erratic movement. Some teams have adopted a strategy of perpetually spinning their robots to make their armor plates harder to hit. Odometry could also provide additional information to operators, in the form of a minimap.

In general, our current goals are:

- Auto-aim
- IMU odometry
- Sentry Strategy

####

# Git and Github

The GitHub flow is a lightweight workflow that allows you to experiment and collaborate on your projects easily, without the risk of losing your previous work.

### Repositories

A repository is where your project work happens--think of it as your project folder. It contains all of your project’s files and revision history.  You can work within a repository alone or invite others to collaborate with you on those files.

### Cloning 

When a repository is created with GitHub, it’s stored remotely in the ☁️. You can clone a repository to create a local copy on your computer and then use Git to sync the two. This makes it easier to fix issues, add or remove files, and push larger commits. You can also use the editing tool of your choice as opposed to the GitHub UI. Cloning a repository also pulls down all the repository data that GitHub has at that point in time, including all versions of every file and folder for the project! This can be helpful if you experiment with your project and then realize you liked a previous version more. 
To learn more about cloning, read ["Cloning a Repository"](https://docs.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository). 

### Committing and pushing
**Committing** and **pushing** are how you can add the changes you made on your local machine to the remote repository in GitHub. That way your instructor and/or teammates can see your latest work when you’re ready to share it. You can make a commit when you have made changes to your project that you want to “checkpoint.” You can also add a helpful **commit message** to remind yourself or your teammates what work you did (e.g. “Added a README with information about our project”).

Once you have a commit or multiple commits that you’re ready to add to your repository, you can use the push command to add those changes to your remote repository. Committing and pushing may feel new at first, but we promise you’ll get used to it 🙂

### Branches
You can use branches on GitHub to isolate work that you do not want merged into your final project just yet. Branches allow you to develop features, fix bugs, or safely experiment with new ideas in a contained area of your repository. Typically, you might create a new branch from the default branch of your repository—main. This makes a new working copy of your repository for you to experiment with. Once your new changes have been reviewed by a teammate, or you are satisfied with them, you can merge your changes into the default branch of your repository.
To learn more about branching, read ["About Branches"](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/about-branches).

### Forks
A fork is another way to copy a repository, but is usually used when you want to contribute to someone else’s project. Forking a repository allows you to freely experiment with changes without affecting the original project and is very popular when contributing to open source software projects!
To learn more about forking, read ["Fork a repo"](https://docs.github.com/en/github/getting-started-with-github/fork-a-repo)

### Pull requests
When working with branches, you can use a pull request to tell others about the changes you want to make and ask for their feedback. Once a pull request is opened, you can discuss and review the potential changes with collaborators and add more changes if need be. You can add specific people as reviewers of your pull request which shows you want their feedback on your changes! Once a pull request is ready-to-go, it can be merged into your main branch.
To learn more about pull requests, read ["About Pull Requests"](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/about-pull-requests). 

# Frameworks

The primary framework we use in software is ROS or Robot Operating System. This guide will explain how it works more in-depth soon, but for the moment, ROS can be understood as a framework for running various modules simultaneously and organizing their communication with each other.

In our repository, we also write ROS packages in C++, so this onboarding will cover C++.

Lastly, we will move toward using OpenCV in our repository as well. OpenCV is a library that supports a wide variety of common image processing and computer vision functions. We should look to use it, as it will likely be more efficient and standard than what already exists.

## ROS
For the sake of keeping this concise and not bloated, this wiki quotes a lot of information from the ROS2 Humble Wiki.

Over the next few tutorials, you will learn about a series of core ROS 2 concepts that make up what is referred to as the “ROS (2) graph”.

The ROS graph is a network of ROS 2 elements processing data together at the same time. It encompasses all executables and the connections between them if you were to map them all out and visualize them.

One of the ways that ROS organizes information is through executables called Nodes. Nodes are essentially C++ programs that run asynchronously, or at the same time.

Each node in ROS should be responsible for a single, modular purpose, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services, actions, or parameters.
A full robotic system is comprised of many nodes working in concert. In ROS 2, a single executable (C++ program, Python program, etc.) can contain one or more nodes.

To make this a complete system, however, every node needs to be able to communicate with other nodes. Otherwise, it would be fairly pointless if we had a node that could control our wheels, but no way to tell them when to move.

You will learn more about ROS nodes in the actual coding section of the onboarding.

## Repository Format

All of our ROS code is contained within the `src` folder. Subfolders within this generally represent ROS packages. There are cases where there can be nested folders and where the top-level folder is not a package. You will be able to tell if a folder is a ROS package based on whether it has a `package.xml` file. Generally, packages also have a `src` directory that holds the source code for that package as well. The later ROS tutorial will cover this in further detail.

The `docker` folder holds the file that generates the Docker container and some scripts that run inside of it. In most cases, this should not be modified, and any changes will likely require all members to rebuild their Docker containers.

The `scripts` folder holds a set of executable scripts that run a series of common tasks, including launching the Docker container and running the Realsense camera.

The `launch` folder holds the configuration for ROS launch setups.

The `models` folder should be kept empty on GitHub. Later, our parameters will be placed here so we can run our models.

# Framework Onboarding

## Terminal/Linux

If you haven't already, you should become familiar with Linux/Unix systems, specifically the terminal.

If you are unfamiliar with Linux, try checking out our [VexU team's Linux onboarding](https://ghost-vexu.notion.site/Linux-Tutorials-b02c774b0fd54e0bbeff4e763c8eab40). Not all of it is applicable, but it will be useful.

Here are some of the useful commands you should know: 
- `ls` - list content in the current directory
- `cd {path name}` - change the current working directory to path name
- `mkdir {directory name}` - make a new directory in the current directory
- `pwd` - prints the path to current directory

#### Git Commands
- `git clone {repository link}` - clones the given repository into your current directory
- `git add {file path}` - tells git to keep track of this file (tracks changes to the file. **You only need to do this command on new files that are added to the repository or files that have been changed**).
- `git add .` - tells git to keep track of all new and modified files
- `git commit -m "{commit message}"` - commits the files from the add to github with the given message
- `git push` - pushes all changes from the commit to the github 
- `git status` - prints a list of new and modified files. (It can be useful to use this command before adding files, to see which files to add or which files have been modified). Also tells you the status of the repository like if you need to pull or push.
- `git restore {file name}` - restores the file to the previous version
- `git restore .` - restores all modified files
- `git pull` - updates the repository with the latest pushes
- `git branch {name}` - creates a new branch with name
- `git branch -a` - Lists all the branches
- `git checkout {branch name}` - changes the current branch to branch name


## C++

For general C++ knowledge, check out [this guide](https://www.learncpp.com/), specifically sections 0.1-0.8, 1.1-1.x, 2.1-2.x, 3.1-3.5, 4.1-4.x. You can also go through different courses such as [CodeAcademy](https://www.codecademy.com/learn/learn-c-plus-plus).

You can write and test some code using [Programiz](https://www.programiz.com/cpp-programming/online-compiler/) or [Godbolt](https://godbolt.org/).

You should have a **solid understanding** of c++ as all of the code base for this club is written in c++.


# Installation

## Pre-requisites

For any platform, you will need [Git](https://git-scm.com/downloads), [Docker](https://www.docker.com/), and ideally [Visual Studio Code](https://code.visualstudio.com/download), but if you strongly prefer you can replace Visual Studio Code with an IDE of your preference.

### Windows

If you are on a Windows platform, keep in mind that you will not be able to use the built in command line for Windows, as it does not support shell scripts. Instead, you should either opt to use Git Bash, which comes installed with Git by default, or if you are more tech-savvy, feel free to attempt to use Windows Subsystem Linux (scripts are untested on WSL).

Additionally, when you clone this repo, try to make sure there are no spaces in the path to the repo, as that can potentially break some scripts.



## Cloning the Repo

### SSH Authentication Setup

If you have already setup git with SSH authentication you may skip this step.

Generate a SSH key in Terminal (MacOS, Linux) or Git Bash (Windows)

```
ssh-keygen -t ed25519 -C "your_email@example.com"
```

When prompted, just press enter until the command finishes. This will create an ssh key in the default location without a passkey. (You can choose a passkey if you would like, but it will ask you for the passkey every time you pull or push).

Copy the output of the following command:
```
cat ~/.ssh/id_ed25519.pub
```

Open [GitHub -> Settings -> Keys](https://github.com/settings/keys) in a web browser.

Click "New SSH Key" and paste the output you copied earlier into the box titled "Key."

Click "Add SSH Key" to finish.

### Cloning

Go to the directory that you want to clone the repo in (if on Windows, make sure it has no spaces). Then run:

```
git clone https://github.com/e-j-present/robomaster_cv.git
```

Once cloned, run:

```
cd robomaster_cv
```

Then, when finished, run:

```
git submodule update --init --recursive
```

## Dev Setup

Once the repo is cloned, open the repo in Visual Studio Code. Press `ctrl + grave` (grave is the button below the tilde, `~`) to open up the terminal pop-up.

If you are on Windows, make sure that it is running in Git Bash and not on command line. 

First, configure your environment by running
```
./configure.sh
```

This will set up the correct docker environment for you.

To enter the container, there are two methods, one involving VSC and one not. I recommend using VSC as it will enable syntax highlighting and smart language features.

### VSC

If you have VSC installed, install the Dev Containers extension and the Remote Development Extension pack.

Once installed, open the command palette (Ctrl + Shift + P or Cmd + Shift + P) and search for the option for "Reopen workspace in container". Wait for it to build and set up the environment, and you should be all set to go. **You must have your docker container running before this step**.

### Non-VSC

If you do not use VSC, run
```
./scripts/run_dev.sh
```

For every new terminal you wish to create, you should run this command in a new terminal.

# Development

## Building Packages

The build system that ROS2 uses is called `colcon`. The repo includes several aliases to help with building.

To build a specific package, make sure you are in the root directory of the project (by default `/robomaster_cv` within the dev container) then run `build package_name`. This is an alias to `colcon build --symlink-install --packages-up-to package_name`. 

Try this by running `build realsense2_camera` (should take ~3 min)

You can also run `build_all`, but this is likely broken in the current version of the repository. 

## Coding

All of our source code is located within the `src` directory. You can feel free to poke around the [packages](http://wiki.ros.org/Packages), or check out the [software wiki](https://github.com/ut-ras/robomaster_cv/wiki) which goes more in depth.
