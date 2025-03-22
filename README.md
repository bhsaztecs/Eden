# Eden is an abstraction library that interfaces with KIPR's wombat.

See https://github.com/kipr/libwallaby

Eden is licensed under GNU AGPL.

see data/example.cpp for example code

## Prerequisites
git - version control, how to actually download this stuff
docker - compiles in a docker container
ssh - shell into the robot, always nice.
scp - copy files to and from the robot

<details>
<summary>Linux</summary>
<br>

## Installation:
```sh
git clone https://github.com/bhsaztecs/Eden
cd Eden
source data/shfuncs
initialize
```
## Compiling
```sh
compile [destination folder] [target]
# destination folder should be UserName/ProjectName,
# the full path name on the robot would be
# /home/kipr/Documents/KISS/UserName/ProjectName

# target is either "executable" or "library" at the moment
# target takes any function, but executable & library are
# the only ones tested
```
## Troubleshooting:

### I'm not seeing my project in the IDE
edit
"/home/kipr/Documents/KISS/[UserName]/[ProjectName]/project.manifest"
and "/home/kipr/Documents/KISS/(i forgot the path sorry)"

### SCP refused to copy my files
```sh

shell
cd /home/kipr/Documents/KISS/[UserName]/
sudo chmod 777 -R *
```
</details>


<details>
<summary>Windows</summary>
<br>
  
## Installation:
```powershell
git clone https://github.com/bhsaztecs/Eden
cd Eden
. ./data/psfuncs.ps1
Initialize
```
## Compiling
```powershell
Compile [destination folder] [target]
# destination folder should be UserName/ProjectName,
# the full path name on the robot would be
# /home/kipr/Documents/KISS/UserName/ProjectName

# target is either "executable" or "library" at the moment
# target takes any function, but executable & library are
# the only ones tested
```
## Troubleshooting:

### I'm not seeing my project in the IDE
edit
"/home/kipr/Documents/KISS/[UserName]/[ProjectName]/project.manifest"
and "/home/kipr/Documents/KISS/(i forgot the path sorry)"

### SCP refused to copy my files
```sh
Remote-Shell
# IN RASPBERRY PI
cd /home/kipr/Documents/KISS/[UserName]/
sudo chmod 777 -R *
```
</details>
