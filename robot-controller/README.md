## Export with electron-packager (recommended)

npm install electron-packager -g
electron-packager . --platform=win32 --arch=x64 robot-controller
npm install (Inside build folder)

## Export with electron-forge

npm install --save-dev @electron-forge/cli
npx electron-forge import
npm run make
npm install (Inside build folder)

## Add to msi installer package

npm install electron-wix-msi --save-dev
node build_installer.js