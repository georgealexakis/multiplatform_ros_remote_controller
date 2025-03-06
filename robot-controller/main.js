const { app, BrowserWindow } = require('electron')
const electron = require('electron')
const path = require('path')
// Main application window
function createWindow() {
  const display = electron.screen.getPrimaryDisplay()
  const maxiSize = display.workAreaSize
  const newWindow = new BrowserWindow({
    width: maxiSize.width,
    height: maxiSize.height,
    minWidth: 1100,
    minHeight: 650,
    show: false,
    title: 'Robot Controller',
    backgroundColor: '#212121',
    icon: path.join(__dirname + '/assets/images/favicon.png'),
    webPreferences: {
      preload: path.join(__dirname, 'preload.js')
    }
  })

  newWindow.loadFile('index.html')
  newWindow.setMenu(null);
  return newWindow
}
// Loader window
function createLoaderWindow() {
  const newWindow = new BrowserWindow({
    width: 400,
    height: 400,
    frame: false,
    show: false,
    transparent: true,
    title: 'Robot Controller',
    icon: path.join(__dirname + '/assets/images/favicon.png'),
    webPreferences: {
      preload: path.join(__dirname, 'preload.js')
    }
  })
  newWindow.loadFile('loader.html')
  return newWindow
}
// App on ready
app.on('ready', () => {
  let mainWindow = null
  let loaderWindow = createLoaderWindow()
  // Show main window and hide loader
  loaderWindow.once('show', () => {
    mainWindow = new createWindow()
    mainWindow.webContents.once('dom-ready', () => {
      mainWindow.maximize()
      loaderWindow.hide()
      loaderWindow.close()
      mainWindow.show()
    })
  })
  loaderWindow.show()
})
// App on all windows closed
app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})