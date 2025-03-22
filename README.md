# Eden is an abstraction library that interfaces with KIPR's wombat.

See https://github.com/kipr/libwallaby

Eden is licensed under GNU AGPL.

see data/example.cpp for example code

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
```sh
git clone https://github.com/bhsaztecs/Eden
cd Eden
. .\data\psfuncs.ps1
Initialize
```
## Compiling
```sh
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

shell
cd /home/kipr/Documents/KISS/[UserName]/
sudo chmod 777 -R *
```
</details>
