/**
 * @author Russell Toris - russell.toris@gmail.com
 * @author Modified by Georgios Alexakis - geosalexs@gmail.com
 */
var MJPEGCANVAS = MJPEGCANVAS || {
  REVISION: '0.5.0-SNAPSHOT'
};

/**
 * @author Russell Toris - russell.toris@gmail.com
 */

/**
 * A MultiStreamViewer can be used to stream multiple MJPEG topics into a canvas.
 *
 * Emits the following events:
 *   * 'warning' - emitted if the given topic is unavailable
 *   * 'change' - emitted with the topic name that the canvas was changed to
 *
 * @constructor
 * @param options - possible keys include:
 *   * divID - the ID of the HTML div to place the canvas in
 *   * width - the width of the canvas
 *   * height - the height of the canvas
 *   * host - the hostname of the MJPEG server
 *   * port (optional) - the port to connect to
 *   * quality (optional) - the quality of the stream (from 1-100)
 *   * topics - an array of topics to stream
 *   * labels (optional) - an array of labels associated with each topic
 *   * defaultStream (optional) - the index of the default stream to use
 */
MJPEGCANVAS.MultiStreamViewer = function (options) {
  var that = this;
  options = options || {};
  var divID = options.divID;
  var width = options.width;
  var height = options.height;
  var host = options.host;
  var port = options.port || 8080;
  var quality = options.quality;
  var topics = options.topics;
  var labels = options.labels;
  var defaultStream = options.defaultStream || 0;
  var currentTopic = topics[defaultStream];
  var backgroundColor = options.backgroundColor || '#444';

  // create an overlay canvas for the button
  var canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  var context = canvas.getContext('2d');

  // menu div
  var menu = document.createElement('div');
  menu.style.display = 'none';
  document.getElementsByTagName('body')[0].appendChild(menu);

  // button for the error
  var buttonHeight = 30;
  var button = new MJPEGCANVAS.Button({
    height: buttonHeight
  });
  var buttonWidth = button.width;

  // use a regular viewer internally
  var viewer = new MJPEGCANVAS.Viewer({
    divID: divID,
    width: width,
    height: height,
    host: host,
    port: port,
    quality: quality,
    topic: currentTopic,
    overlay: canvas,
    backgroundColor: backgroundColor
  });

  // catch the events
  viewer.on('warning', function (warning) {
    that.emit('warning', warning);
  });
  viewer.on('change', function (topic) {
    currentTopic = topic;
    that.emit('change', topic);
  });

  /**
  * Close the connection with the server
  */
  this.close = function () {
    // Stop viewer
    if (viewer !== null && viewer !== undefined && viewer.timer !== null && viewer.timer !== undefined) {
      clearInterval(viewer.timer);
      viewer = null;
    }
    // Remove camera selection menu
    menu.remove();
  }

  /**
   * Clear the button from the overlay canvas.
   */
  function clearButton() {
    if (buttonTimer) {
      window.clearInterval(buttonTimer);
      // clear the button
      canvas.width = canvas.width;
      hasButton = false;
    }
  }

  /**
   * Fades the stream by putting an overlay on it.
   */
  function fadeImage() {
    canvas.width = canvas.width;
    // create the white box
    context.globalAlpha = 0.44;
    context.beginPath();
    context.rect(0, 0, width, height);
    context.fillStyle = '#adb5bd';
    context.fill();
    context.globalAlpha = 1;
  }

  // add the event listener
  var buttonTimer = null;
  var menuOpen = false;
  var hasButton = false;
  viewer.canvas.addEventListener('mousemove', function (e) {
    clearButton();

    if (!menuOpen) {
      hasButton = true;
      // add the button
      button.redraw();
      // Set the button position on the top
      context.drawImage(button.canvas, 5, 5);

      // display the button for 500 milliseconds
      buttonTimer = setInterval(function () {
        // clear the overlay canvas
        clearButton();
      }, 500);
    }
  }, false);

  // add the click listener
  viewer.canvas.addEventListener('click', function (e) {
    // check if the button is there
    if (hasButton) {
      // determine the click position
      var offsetLeft = 0;
      var offsetTop = 0;
      var element = viewer.canvas;
      while (element && !isNaN(element.offsetLeft) && !isNaN(element.offsetTop)) {
        offsetLeft += element.offsetLeft - element.scrollLeft;
        offsetTop += element.offsetTop - element.scrollTop;
        element = element.offsetParent;
      }

      var x = e.pageX - offsetLeft;
      var y = e.pageY - offsetTop;

      // check if we are in the 'edit' button on the top left corner
      if (x < (buttonWidth + 5) && x > 5 && y < (buttonHeight + 15) && y > 5) {
        menuOpen = true;

        clearButton();

        // create the menu
        var form = document.createElement('form');
        form.setAttribute('class', 'col-3');
        var streamMenu = document.createElement('select');
        streamMenu.setAttribute('name', 'stream');
        streamMenu.setAttribute('class', 'form-select form-select-sm streamer');
        // add each option
        for (var i = 0; i < topics.length; i++) {
          var option = document.createElement('option');
          // check if this is the selected option
          if (topics[i] === currentTopic) {
            option.setAttribute('selected', 'selected');
          }
          option.setAttribute('value', topics[i]);
          // check for a label
          if (labels) {
            option.innerHTML = labels[i];
          } else {
            option.innerHTML += topics[i];
          }
          streamMenu.appendChild(option);
        }
        form.appendChild(streamMenu);
        menu.appendChild(form);

        // display the menu
        menu.style.position = 'absolute';
        menu.style.top = offsetTop + 'px';
        menu.style.left = offsetLeft + 'px';
        menu.style.width = width + 'px';
        menu.style.display = 'block';

        // Fade image on click
        fadeImage();

        streamMenu.addEventListener('click', function () {
          var topic = streamMenu.options[streamMenu.selectedIndex].value;
          // make sure it is a new stream
          if (topic !== currentTopic) {
            // close the menu
            menu.innerHTML = '';
            menu.style.display = 'none';
            menuOpen = false;
            canvas.width = canvas.width;
            viewer.changeStream(topic);
          }
        }, false);
      }
    }
  }, false);
};
MJPEGCANVAS.MultiStreamViewer.prototype.__proto__ = EventEmitter2.prototype;

/**
 * @author Russell Toris - russell.toris@gmail.com
 */

/**
 * A Viewer can be used to stream a single MJPEG topic into a canvas.
 *
 * Emits the following events:
 *   * 'warning' - emitted if the given topic is unavailable
 *   * 'change' - emitted with the topic name that the canvas was changed to
 *
 * @constructor
 * @param options - possible keys include:
 *   * divID - the ID of the HTML div to place the canvas in
 *   * width - the width of the canvas
 *   * height - the height of the canvas
 *   * host - the hostname of the MJPEG server
 *   * port (optional) - the port to connect to
 *   * quality (optional) - the quality of the stream (from 1-100)
 *   * topic - the topic to stream, like '/wide_stereo/left/image_color'
 *   * overlay (optional) - a canvas to overlay after the image is drawn
 *   * refreshRate (optional) - a refresh rate in Hz
 *   * interval (optional) - an interval time in milliseconds
 */
MJPEGCANVAS.Viewer = function (options) {
  var that = this;
  options = options || {};
  var divID = options.divID;
  this.width = options.width;
  this.height = options.height;
  this.host = options.host;
  this.port = options.port || 8080;
  this.quality = options.quality;
  this.refreshRate = options.refreshRate || 10;
  this.interval = options.interval || 30;
  this.invert = options.invert || false;
  this.backgroundColor = options.backgroundColor || '#444';

  var topic = options.topic;
  var overlay = options.overlay;

  // create no image initially
  this.image = new Image();

  // used if there was an error loading the stream
  var errorIcon = new MJPEGCANVAS.ErrorIcon();

  // create the canvas to render to
  this.canvas = document.createElement('canvas');
  this.canvas.width = this.width;
  this.canvas.height = this.height;
  this.canvas.style.background = this.backgroundColor;
  document.getElementById(divID).appendChild(this.canvas);
  var context = this.canvas.getContext('2d');

  var drawInterval = Math.max(1 / this.refreshRate * 1000, this.interval);
  /**
   * A function to draw the image onto the canvas.
   */
  function draw() {
    // clear the canvas
    that.canvas.width = that.canvas.width;

    // check if we have a valid image
    if (that.image.width * that.image.height > 0) {
      context.drawImage(that.image, 0, 0, that.width, that.height);
    } else {
      // Set icon size
      var newWidth = 80;
      var newHeight = 80;
      // Compute offsets to center the image
      var xOffset = newWidth < that.canvas.width ? ((that.canvas.width - newWidth) / 2) : 0;
      var yOffset = newHeight < that.canvas.height ? ((that.canvas.height - newHeight) / 2) : 0;
      // Draw error icon image
      context.drawImage(errorIcon.image, xOffset, yOffset, newWidth, newHeight);
      that.emit('warning', 'Invalid stream.');
    }

    // check for an overlay
    if (overlay) {
      context.drawImage(overlay, 0, 0);
    }

    // silly firefox...
    if (navigator.userAgent.toLowerCase().indexOf('firefox') > -1) {
      var aux = that.image.src.split('?killcache=');
      that.image.src = aux[0] + '?killcache=' + Math.random(42);
    }
  }

  // grab the initial stream
  this.changeStream(topic);

  // call draw with the given interval or rate
  this.timer = setInterval(draw, drawInterval);
};
MJPEGCANVAS.Viewer.prototype.__proto__ = EventEmitter2.prototype;

/**
 * Change the stream of this canvas to the given topic.
 *
 * @param topic - the topic to stream, like '/wide_stereo/left/image_color'
 */
MJPEGCANVAS.Viewer.prototype.changeStream = function (topic) {
  this.image = new Image();
  // create the image to hold the stream
  var src = 'http://' + this.host + ':' + this.port + '/stream?topic=' + topic;
  // Use only compressed topics
  src += '&type=ros_compressed';
  // add various options
  src += '&width=' + this.width;
  src += '&height=' + this.height;
  if (this.quality > 0) {
    src += '&quality=' + this.quality;
  }
  if (this.invert) {
    src += '&invert=' + this.invert;
  }
  this.image.src = src;
  // emit an event for the change
  this.emit('change', topic);
};

/**
 * @author Russell Toris - russell.toris@gmail.com
 */

/**
 * A button renders a button with text to an internal canvas. The width will scale to fit the text.
 *
 * @constructor
 * @param options - possible keys include:
 *   * text - the text to display on the button
 *   * height - the height of the button
 */
MJPEGCANVAS.Button = function (options) {
  options = options || {};
  this.height = options.height;
  // used to draw the text internally
  this.canvas = document.createElement('canvas');
  this.redraw();
};

/**
 * Redraw the button to the internal canvas.
 */
MJPEGCANVAS.Button.prototype.redraw = function () {
  var context = this.canvas.getContext('2d');

  // determine text size
  this.width = this.height;
  this.canvas.width = this.width;
  this.canvas.height = this.height;

  // create the image
  var image = new Image();
  // keep the base64 representation internally
  image.src = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGAAAABgCAQAAABIkb+zAAAEGklEQVR42u3cTUhUURTA8TOMaYbZxyKjdhEIaasnQrUoMzeuok1hZLSwTSgZBAnRgNWiFq0LaysIEiGCiyyiVVQUBhJBUUZEGbRJrZT4tyiM7r3vzXs2070nOrP0vOP7MTPvfrzzRqSEwTKaOMApLjPGU17+fH1gigeMco0CR9hKTkILqmmhwDizpIlP3OUSe1kRDqCdL2SPOa5ziFWaCQBfucEu3QSAx3RSqZsAbzlGhW4CTLJHOwFG2Bwi4TD7Oc0gz1IQZjkaHiFa/Ps69jHAdBHEMKvDIkRGTp7dXGUugfCKHSERImfeWk7yOpYwz8FwCFFsZiXdsR+ob/SGQogSc2vo52sM4kIYhKhodiP3YggDIRCiFNl5+lhwEs76J0Qp81t45yR0+SZEqfM38sgBWKDdLyHKkF/LHQdhhkafhIZM+csZcRAmqBItQRW3HYSLoidYyUPHwLZTE6GON44ZUq0mwnbH+Hy+nP+wUHSuX8hYscexWtjg7/QzE8gxblW44vP0sxM2MWMNavU+Tz874YR1/NDST7TMEbPn+sJa6qxXBBChw0rs0wXIMWEkPl/iLrcfgAiHrdRWXYAq3hupg6oAIpwzUj+S1wWot5KbVQFEeFKCK5FXwBkj+ZY2QJOR/JlqXYAKPhnp21QBRLhppHeWDzBJL1uooYYt9DJZIkC/kd5fHsA83b9fo8nTzXwJAB1/PJilunHa5jyyLXb7Nj2g2Ui/Xw5AT4blYVbAGnM0Lj1gMn6AJ1/8u1D+fYJicTzx6OPhAxK3EGkIH7CyyG7bf8D/j9C//iX+25dRPhoV1oQ8kLmK3s+2TPM7lXCVHTQqdIQ8mSvBBNXvdNp1Qp1GhZshL2hcJ7TN6uqs0AWo5rNRo0kVQIRbRo0z2gB9Ro0n2gDNVpV6XYC8NRqfUwVwDGbv43sSwgS02l2eugA5nlt9ITlFAMeVKHZOFCpgvTXVesEyRQARhqxaJ0RTUG/1C86wSRfhivUejAf4xFECYIPjKaQeXe/BecfycLsmQC2vLMIb6jQRdvLNIjxM3m0LjXDRcXG+ratfc8JBGGF5pioNPhuFG60WL4A7Wdodifjit9fZ1QT/iI0ZAPgldDknKu9oyQDwTDjrJCzQl6bJ5SfAM2EgZsJ4r3gb/CLAM+FC7PZtPzUpAZ4JvY6B7UdM0x3/AO1vAM+Egwm7yq85ydoUAM+EHY4Z0q+Y4yq7zS+2BfBMWM1wkZXgNAPsY10CwC9BhKOpfrPgGYOcZr+jyzMAwmbngznZwi9BhD3p77WESqjgGG9VE0SopJPHqgkiIuziRpobpwETRFjFIa4nPlgeOkFEhBXs5RJ3ra5Od8wyToGWJTStlh2SYytHKHCNUR4wxYfFn+h5yhiXOcUBmtx7rkuN70PAcoouKzccAAAAAElFTkSuQmCC';

  // draw the button
  context.drawImage(image, 0, 0, this.width, this.height);
};

/**
 * @author Russell Toris - russell.toris@gmail.com
 */

/**
* An ErrorIcon contains a image with an error/warning sign on it. The image data for this object
* is contained internally as a data URI string.
*
* @constructor
*/
MJPEGCANVAS.ErrorIcon = function () {
  // create the image
  this.image = new Image();
  // keep the base64 representation internally
  this.image.src = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGAAAABgCAQAAABIkb+zAAACSElEQVR42u2bvU4CQRCApRCxNNqRWPoYWBEKSfQtaDDhSS4GWt7BltD6AhcwVhgs1MbEn85GPwuNkWi8PXdnb2fd7wHIfGR3bnZ3Zm0tkUgkEglnsE2HY4ZMuGDJLQC3LLlgwpBjOmyHGXiDLhk5JuRkdGmEE/w+Y54oyxNj9qsOvU6PBTYs6FGvatEMuMEFNwy8LygOuMQllxz4C36XUyQ4ZddH+Ic8IMUDh7LBr3OCNCesS4W/xRk+OGNLIvwm5/jinKbr8Pe4widX7Ln99/2G/67QdLf251TB3MleoO5p6/68ne3LDA+J89ekahv+Ea+VCrxyZFc03FM19xYFhlDNU7pG+nvFGQp/qVTZtCuYv/2eXbG9WV5gYPenORWAQdnwN7gOSuCajXICPdtl61gAemXCr1ke1SUEFtTMBVr2icO5ALTMBcZBCozNE+hjkAKPhsmUrotvj4AAdM0EsmAFMjOBPFiB3CT8HV6CFXhhp1ig7ab+EhGAdrFAP2iBfrHAKGiBUbHANGiBabHALGiBWbHAMmiBZbHAXdACd8UCz0ELPP8DAfVLSP0mVp9GHX3IhJh6KyWEGHkr5oToeyunhWh7O9CIYHKgcXWkFCH3eKgXIfN4rSKC4bWK9ost9VeLMVzuar9eV//AEcETk/pHvgieWdU/dEfQahBBs4f6dpsIGp4iaDlT3/QXQdtlBI2vEbQeR9D8HUH7fQQDEFGMoEQwBPShoHsM67PM0DsI90VD7yjiyoLSOwy6IqJ1HDeRSCQSSnkDkLafSPJMgbsAAAAASUVORK5CYII=';
};
