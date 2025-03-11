# Robot Controller

Robot Controller is a versatile tool developed as part of multiple research projects focused on autonomous robotics. It serves as a crucial component during experiments, enabling efficient control and adaptation of robotic systems. Features and functionalities are continuously expanded based on the evolving requirements of new research projects.

## Features

* Navigation Joystick for multiple robots.
* External gamepad controller can be used.
* Multicameras live feed.
* SLAM map.
* Open Street Map.
* ROS logs.
* Multiple buttons for configuration.
* Operation times.
* Robot status and battery.
* Robot PC status (CPU, RAM, etc.)

## Export to executable

### Export with electron-packager (recommended)

npm install electron-packager -g
electron-packager . --platform=win32 --arch=x64 robot-controller
npm install (Inside build folder)

## Export with electron-forge

npm install --save-dev @electron-forge/cli
npx electron-forge import
npm run make
npm install (Inside build folder)

## Add to .msi installer package

Just if you want to have it as .msi installable package use electron-wix-msi:

npm install electron-wix-msi --save-dev
node build_installer.js

## Screen Captures

Main screen of Robot Controller:

![Robot Controller 1](../screen-captures/controller/1.png)

![Robot Controller 2](../screen-captures/controller/20.png)

![Robot Controller 3](../screen-captures/controller/21.png)

![Robot Controller 4](../screen-captures/controller/22.png)

![Robot Controller 5](../screen-captures/controller/23.png)

![Robot Controller 6](../screen-captures/controller/31.png)

![Robot Controller 7](../screen-captures/controller/32.png)

![Robot Controller 8](../screen-captures/controller/33.png)

Toggle buttons which activate actions (on the left), controller options (on the right):

<p align="center">
  <img src="../screen-captures/controller/2.png">
  <img src="../screen-captures/controller/3.png">
</p>

![4](../screen-captures/controller/4.png)

![5](../screen-captures/controller/5.png)

![6](../screen-captures/controller/6.png)

![7](../screen-captures/controller/7.png)

![8](../screen-captures/controller/8.png)

![9](../screen-captures/controller/9.png)

![10](../screen-captures/controller/10.png)

![11](../screen-captures/controller/11.png)

![12](../screen-captures/controller/12.png)

![13](../screen-captures/controller/13.png)

![14](../screen-captures/controller/14.png)

![15](../screen-captures/controller/15.png)

![16](../screen-captures/controller/16.png)

![17](../screen-captures/controller/17.png)

![18](../screen-captures/controller/18.png)

![19](../screen-captures/controller/19.png)

![34](../screen-captures/controller/34.png)