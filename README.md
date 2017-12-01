# ArUco-based tag tracker with bounding boxes

# Installation Instructions

There are two approaches
1. Install docker and build with docker, which normalizes the environment
2. Install OpenCV natively and execute accordingly

The first option typically provides a more reliable execution environment.  
The docker build should work on any **x87** system running Ubuntu **16.04** (or
  any Linux distribution really).

## Docker Installation

Download [docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)
and install on your machine.  The convenience script option may be particularly
useful.  Rather than adding a repo, you can use
```bash
  curl -fsSL get.docker.com -o get-docker.sh
  sudo sh get-docker.sh
```
which downloads a convenience script to install docker.

Once docker is installed, build the tracker container.  
```bash
  cd path_to_tag_tracker/docker
  ./docker_build.sh name_of_container
```
The variable 'name_of_container' could be anything. For example,
```bash
  ./docker_build.sh tag_tracker
```
After the build process completes, you can test the container using an .mp4 file.
```bash
  ./docker_test.sh name_of_container absolute_path_to_video
```
The variable 'name_of_container' should be whatever you named the container
in the previous step.  For example,
```bash
  ./docker_test.sh tag_tracker /home/robotarium/out.mp4
```
For the tracking window to show up, you'll most likely have to enable .x11
connections from the container with something like
```bash
  sudo xhost +
```
