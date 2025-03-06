const { MSICreator } = require('electron-wix-msi');
const path = require('path');

const APP_DIR = path.resolve(__dirname, './robot-controller-win32-x64');
const OUT_DIR = path.resolve(__dirname, './installer');
const ICON_DIR = path.resolve(__dirname, './assets/images/favicon.ico');
const ICON_B1 = path.resolve(__dirname, './assets/images/background.png');
const ICON_B2 = path.resolve(__dirname, './assets/images/banner.png');
const ICON_1 = path.resolve(__dirname, './assets/images/exclamation.ico');
const ICON_2 = path.resolve(__dirname, './assets/images/info.ico');
const ICON_3 = path.resolve(__dirname, './assets/images/new.ico');
const ICON_4 = path.resolve(__dirname, './assets/images/up.ico');

const msiCreator = new MSICreator({
    appDirectory: APP_DIR,
    outputDirectory: OUT_DIR,
    appIconPath: ICON_DIR,
    description: 'This is the robot controller application.',
    exe: 'robot-controller',
    name: 'Robot Controller',
    shortcutFolderName: 'Robot Controller',
    manufacturer: 'George Alexakis',
    version: '1.0.0',
    arch: 'x64',
    ui: {
        chooseDirectory: true,
        images: {
            background: ICON_B1,
            banner: ICON_B2,
            exclamationIcon: ICON_1,
            infoIcon: ICON_2,
            newIcon: ICON_3,
            upIcon: ICON_4
        }
    }
});

msiCreator.create().then(function () {
    msiCreator.compile();
});