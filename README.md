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
useful.

Once docker is installed, build the tracker container.  
```bash
  cd path_to_tracker/docker
  ./docker_build name_of_container
```
For example,
```bash
  ./docker_build tag_tracker
```
After the build process completes, you can test the container using an .mp4 file.
```bash
  ./docker_test name_of_container absolute_path_to_video
```
For example
```bash
  ./docker_test tag_tracker /home/robotarium/out.mp4
```
