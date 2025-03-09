# Robot Contoller

Robot contoller has developed is  the context of multiple research projects for autonomous robots. During experiments, Robot Contoller is a very useful tool. I add something when it is requierred by a new research project.

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

Just if you want to have it as .msi installable packake use electron-wix-msi:

npm install electron-wix-msi --save-dev
node build_installer.js