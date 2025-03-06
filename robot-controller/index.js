// Initialize variables
var linearSpeed = 0.0; // Initial linear speed
var angularSpeed = 0.0; // Initial angular speed
var maxDistance = 70; // Set maximum distance of joystick path related to speed
var maxLinear = 1.0; // Initial max linear speed
var maxAngular = 1.0; // Initial max angular speed
var fixedVelocity = false; // Fixed crucial velocity with fixed ranges
var fixedVelocityValues = [157.0, 97.0, 127.0, 157.0, 97.0, 127.0]; // Fixed velocity variables
var connectStatus = false; // Initial connect variable
var connectTopicsStatus = false; // Keep connection status of topics
var ROS = null; // Connection with ROS
var seconds = 0; // Operation time
var viewerFull = null; // Full screen URDF viewer
var WIDTH_FULL = 0; // Full screen URDF viewer width
var HEIGHT_FULL = 0; // Full screen URDF viewer height
var viewer = null; // URDF viewer
var mainJoystick = null; // Main joystick
var statisticsChart = null; // Statistics chart
var navigationMethod = 0; // Jostick or keyboard navigation method
var keyboardSelection = 0; // Select arrows or A, S, W, D
// Initialize topics for publishers and subscribers
var ROSOUT = null;
var TOPICS = [];
for (var i = 0; i < 16; i++) {
    TOPICS.push(null);
}
// Live stream from cameras
var camerasViewer = null;
// 3D Model variables and fixed frame
var tfClient = null;
var urdfClient = null;
var tfClientFull = null;
var urdfClientFull = null;
var cloudClient = null;
// Init arrays of statistics
var cpuD1 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
var cpuD2 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
var cpuD3 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
var cpuD4 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
var memoryD = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
// Colors
const colorsChart = ['#3a2cb7', '#673ab7', '#b7673a', '#2cb73a', '#b70567'];
const colorMain = '#673ab7';
const colorViewer = '#e2e7ea';
// Initialize timers and counter
var joyTimer = null;
var keyTimer = null;
var timer = null;
var totalSeconds = 0;
// Other actions and ranges
var selectedAction = [0, 0, 0, 0];
var ranges = [0.0, 0.0, 0.0, 0.0];
// Access timer elements
var hoursLabel = document.getElementById('hours');
var minutesLabel = document.getElementById('minutes');
var secondsLabel = document.getElementById('seconds');
// Access DOM elements - IPs, Ports
var ipElement = document.getElementById('robotIP');
var portElement = document.getElementById('robotPort');
var streamIpElement = document.getElementById('streamIP');
var streamPortElement = document.getElementById('streamPort');
// Access DOM elements - Miscellaneous
var spinnerElement = document.getElementById('spinnerIcon');
var connectElement = document.getElementById('connectText');
var connectTopicsElement = document.getElementById('connectTopicsText');
var joyElement = document.getElementById('joy');
var mainElement = document.getElementById('mainBlock');
var resetElement = document.getElementById('reset');
var closePointCloudElement = document.getElementById('urdfExit');
var rangesTriggerElement = document.getElementById('rangesTrigger');
// Access DOM elements - Status bar
var statusElement = document.getElementById('status');
var connectionElement = document.getElementById('connection');
var batteryElement = document.getElementById('battery');
// Access DOM elements - Live feed, URDF model, Chart
var liveParentElement = document.getElementById('liveParent');
var liveFeedElement = document.getElementById('liveFeed');
var liveElement = document.getElementById('live');
var urdfParentElement = document.getElementById('urdfParent');
var urdfFullscreenElement = document.getElementById('urdfFullscreen');
var chartElement = document.getElementById('stChart');
// Access DOM elements - Logs
var mainLogsElement = document.getElementById('mainLogs');
var logsElement = document.getElementById('logs');
var fvelocitySetElement = document.getElementById('fvelocitySet');
// Access DOM elements - Keyboard settings
var navMethodElement = document.getElementById('navMethod');
var keyConsoleElement = document.getElementById('keyConsole');
var defaultControlElement = document.getElementById('defaultControl');
var arrowsControlElement = document.getElementById('arrowsControl');
// Access DOM elements - Speed controller
var linearSpeedElement = document.getElementById('linearSpeed');
var maxLinearElement = document.getElementById('maxLinear');
var angularSpeedElement = document.getElementById('angularSpeed');
var maxAngularElement = document.getElementById('maxAngular');
var angularElement = document.getElementById('angular');
var linearElement = document.getElementById('linear');
// Access DOM elements - Topics, Checkboxes of topics
var topicsElements = [];
var checkboxesElements = [];
for (var i = 0; i < 20; i++) {
    topicsElements[i] = document.getElementById('t' + i);
    checkboxesElements[i] = document.getElementById('ct' + i);
}
// Access DOM elements - Chart
var chartElements = [];
for (var i = 0; i < 6; i++) {
    chartElements[i] = document.getElementById('p' + i);
}
// Access DOM elements - Angles, Goals, Switches, Buttons, Settings
var anglesElements = [];
for (var i = 0; i < 4; i++) {
    anglesElements[i] = document.getElementById('a' + i);
}
// Navigation map
var NAV_WIDTH = 0;
var NAV_HEIGHT = 0;
var goalsElements = [];
var robotModel = 'images/eddie.png';
for (var i = 0; i < 9; i++) {
    goalsElements[i] = document.getElementById('g' + i);
    goalsElements[i].addEventListener('click', (event) => {
        var id = event.target.id;
        switch (parseInt(id[1])) {
            case 0:
                // Show map modal
                $('#navigationtModal').modal('show');
                break;
            case 1:
                // Display goals
                var body = document.getElementById('goalsModalBody');
                var goals = '';
                var deleteButtons = [];
                for (var i = 0; i < GOALS.length; i++) {
                    goals = goals
                        + '<tr><th scope="row">'
                        + (i + 1) + '</th><td>'
                        + GOALS[i].goalID
                        + '</td><td>'
                        + '<button type="button" id="' + i + '" class="btn btn-sm btn-danger">Delete</button>'
                        + '</td></tr>';
                }
                body.innerHTML = goals;
                $('#goalsInfoModal').modal('show');
                for (var i = 0; i < GOALS.length; i++) {
                    deleteButtons.push(document.getElementById('' + i));
                    deleteButtons[i].addEventListener('click', (event) => {
                        GOALS.splice(parseInt(event.target.id), 1);
                        refreshGoals();
                        $('#goalsInfoModal').modal('hide');
                    });
                }
                break;
            case 2:
                // Model
                if (robotModel.localeCompare('images/eddie.png') === 0) {
                    robotModel = 'images/monster.png';
                    displayLogs('', 'Robot model: monster\n');
                } else if (robotModel.localeCompare('images/monster.png') === 0) {
                    robotModel = 'images/arrow.png';
                    displayLogs('', 'Robot model: arrow\n');
                } else if (robotModel.localeCompare('images/arrow.png') === 0) {
                    robotModel = 'images/eddie.png';
                    displayLogs('', 'Robot model: eddie\n');
                }
                document.getElementById('navigationMapFeed').style.display = 'none';
                document.getElementById('navigationMapPreview').style.display = 'block';
                if ((Object.keys(NAV_MAP).length !== 0)) {
                    NAV_VIEWER.scene.removeAllChildren();
                    NAV_MAP = {};
                    NAV_VIEWER = {};
                    document.getElementById('navigationMapFeed').innerHTML = '';
                }
                navStatus = false;
                break;
            case 3:
                // Save goals
                break;
            case 4:
                cancelGoals();
                break;
            case 5:
                // Clear goals
                break;
            case 6:
                NAVIGATION = false;
                POSEESTIMATION = true;
                GOALSNAVIGATION = false;
                displayLogs('', 'Pose estimation mode\n');
                break;
            case 7:
                NAVIGATION = false;
                POSEESTIMATION = false;
                GOALSNAVIGATION = true;
                displayLogs('', 'Multiple navigation mode\n');
                break;
            case 8:
                NAVIGATION = true;
                POSEESTIMATION = false;
                GOALSNAVIGATION = false;
                displayLogs('', 'Simple navigation mode\n');
                break;
        }
    });
}
// Navigation modal
var navigationModal = document.getElementById('navigationtModal');
// On shown modal
navigationModal.addEventListener('shown.bs.modal', () => {
    NAV_WIDTH = $('#navigationMapCanvasBody').width();
    NAV_HEIGHT = $('#navigationMapCanvasBody').height();
    document.getElementById('navigationMapPreview').width = NAV_WIDTH;
    document.getElementById('navigationMapPreview').height = NAV_WIDTH;
    if (connectStatus) {
        STARTBUTTON.disabled = false;
        CLEANBUTTON.disabled = false;
        zoominButton.disabled = false;
        zoomoutButton.disabled = false;
        displayMap();
    } else {
        STARTBUTTON.disabled = true;
        CLEANBUTTON.disabled = true;
        zoominButton.disabled = true;
        zoomoutButton.disabled = true;
    }
});
// Global variable
var NAV_MAP = {};
var NAV_VIEWER = {};
var NAV_ZOOM = {};
// Disaply map
function displayMap() {
    document.getElementById('navigationMapFeed').style.display = 'block';
    document.getElementById('navigationMapFeed').style.height = NAV_WIDTH + 'px';
    document.getElementById('navigationMapPreview').style.display = 'none';
    if (Object.keys(NAV_VIEWER).length === 0) {
        // Create the main viewer
        NAV_VIEWER = new ROS2D.Viewer({
            divID: 'navigationMapFeed',
            width: NAV_WIDTH,
            height: NAV_WIDTH,
            background: '#7f7f7f'
        });
        NAV_ZOOM = new ROS2D.ZoomView({
            ros: ROS,
            rootObject: NAV_VIEWER.scene
        });
        // Setup the nav client with external map
        NAV_MAP = new NAV2D.OccupancyGridClientNav({
            ros: ROS,
            robotMarkerImage: robotModel,
            goalMarkerImage: 'images/marker.png',
            iconsScale: 0.4,
            rootObject: NAV_VIEWER.scene,
            viewer: NAV_VIEWER,
            serverName: topicsElements[18].value,
            withOrientation: true,
            continuous: true
        });
    }
}
var zoominButton = document.getElementById('n0');
var zoomoutButton = document.getElementById('n1');
zoominButton.addEventListener('click', () => {
    if ((NAV_ZOOM !== undefined) && (NAV_ZOOM !== null) && (Object.keys(NAV_ZOOM).length !== 0)) {
        NAV_ZOOM.startZoom(NAV_WIDTH / 2, NAV_WIDTH / 2);
        NAV_ZOOM.zoom(1.2);
    }
});
zoomoutButton.addEventListener('click', () => {
    if ((NAV_ZOOM !== undefined) && (NAV_ZOOM !== null) && (Object.keys(NAV_ZOOM).length !== 0)) {
        NAV_ZOOM.startZoom(NAV_WIDTH / 2, NAV_WIDTH / 2);
        NAV_ZOOM.zoom(0.8);
    }
});
// Close connection
function disconnectNavigation() {
    document.getElementById('navigationMapFeed').style.display = 'none';
    document.getElementById('navigationMapPreview').style.display = 'block';
    if ((Object.keys(NAV_MAP).length !== 0) && connectStatus) {
        NAV_VIEWER.scene.removeAllChildren();
        NAV_MAP = {};
        NAV_VIEWER = {};
        document.getElementById('navigationMapFeed').innerHTML = '';
    }
}
// Switches
var switchesElements = [];
for (var i = 0; i < 8; i++) {
    switchesElements[i] = document.getElementById('s' + i);
    // Also add listeners
    switchesElements[i].addEventListener('change', (event) => { changeSwitchState(event); });
}
var buttonsElements = [];
for (var i = 0; i < 9; i++) {
    buttonsElements[i] = document.getElementById('b' + i);
    // Also add listeners
    buttonsElements[i].addEventListener('click', (event) => {
        var id = event.target.id;
        // Retrieve ID
        if (id === '' || id === undefined) {
            id = event.target.parentNode.id;
            if (id === '' || id === undefined) {
                id = event.target.parentNode.parentNode.id;
            }
        }
        switch (parseInt(id[1])) {
            case 0:
                $('#gamepadModal').modal('show');
                break;
            case 1:
                clearLogs();
                break;
            case 2:
                keyboardSettings();
                break;
            case 3:
                openSettings();
                break;
            case 4:
                reconnect();
                break;
            case 5:
                changeRanges();
                break;
            case 6:
                enterFullscreen();
                break;
            case 7:
                saveSettingsToLocalStorage();
                break;
            case 8:
                clearLocalStorage();
                break;
        }
    });
}
var settingsButtonsElements = [];
for (var i = 0; i < 6; i++) {
    settingsButtonsElements[i] = document.getElementById('f' + i);
    // Also add listeners
    settingsButtonsElements[i].addEventListener('click', (event) => {
        var id = event.target.id;
        if (id[0] !== 'f') {
            id = event.target.parentNode.id;
        }
        switch (parseInt(id[1])) {
            case 0:
                pullTrigger();
                break;
            case 1:
                pullTriggerTopics();
                break;
            case 2:
                getTopics(0);
                break;
            case 3:
                clearMainLogs();
                break;
            case 4:
                loadSettings();
                break;
            case 5:
                help();
                break;
        }
    });
}
// Add listeners on window
window.addEventListener('load', () => {
    // Init everything
    init();
    // Init map
    initMap();
    // Hide loader after 1000 milliseconds just to init some stuff on background
    setTimeout(() => {
        document.getElementById('preloader').style.display = 'none';
    }, 1000);
});
var count = 0;
window.addEventListener('keydown', (event) => {
    // Remove first keydown delay
    if (count == 0) {
        keyTimer = setInterval(() => {
            keyboardTrigger(event);
        }, 25);
        count++;
    } else {
        keyboardTrigger(event);
        clearInterval(keyTimer);
    }
});
window.addEventListener('keyup', () => {
    // Clear first keydown timer
    clearInterval(keyTimer);
    count = 0;
    clearkeyboardTrigger();
});
window.addEventListener('resize', () => {
    adjustURDF();
    resetViewer();
});
// Add listeners on speed controller
linearSpeedElement.addEventListener('input', () => {
    modifyLinearSpeed();
});
angularSpeedElement.addEventListener('input', () => {
    modifyAngularSpeed();
});
fvelocitySetElement.addEventListener('click', () => {
    for (let i = 0; i < 6; i++) {
        fixedVelocityValues[i] = parseFloat(document.getElementById('fv' + i).value);
    }
    displayLogs('', 'Fixed velocities have set.\n');
});
// Add listener on reset
resetElement.addEventListener('click', () => {
    location.reload();
});
// Add listener on LOGS
var logsStatus = false;
document.getElementById('logsOn').addEventListener('click', () => {
    if (connectStatus && !logsStatus) {
        mainLogsElement.value = '';
        // Subscribe to rosout_agg for logs
        ROSOUT = new ROSLIB.Topic({
            ros: ROS,
            name: '/rosout_agg',
            messageType: 'rosgraph_msgs/Log'
        });
        // Receive command
        ROSOUT.subscribe((msg) => {
            var text = msg.name + '\n'
                + msg.msg + '\n'
                + msg.file + '\n'
                + msg.function + '\n'
                + msg.name + '\n'
                + (msg.topics).toString() + '\n';;
            displayLogs(text + '\n');
        });
        logsStatus = true;
    }
});
document.getElementById('logsOff').addEventListener('click', () => {
    if (connectStatus) {
        ROSOUT.unsubscribe();
        ROSOUT = null;
        logsStatus = false;
    }
});
// Add ranges button listener
rangesTriggerElement.addEventListener('click', () => {
    publishRanges();
});
// Point Cloud 2 fullscreen
urdfFullscreenElement.addEventListener('click', () => {
    // Retrieve user settings
    var pointCloudFrame = topicsElements[14].value;
    var pointCloudTopic = topicsElements[15].value;
    // Display URDF modal
    $('#urdfModal').modal('show');
    // Reopen urdf and point cloud client if exists
    if (viewerFull !== null && tfClientFull !== null) {
        if (checkboxesElements[14].checked) {
            // Fullscreen viewer
            tfClientFull = new ROSLIB.TFClient({
                ros: ROS,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: pointCloudFrame
            });
            if (checkboxesElements[15].checked && cloudClient === null) {
                // Point point cloud client
                cloudClient = new ROS3D.PointCloud2({
                    ros: ROS,
                    tfClient: tfClientFull,
                    rootObject: viewerFull.scene,
                    topic: pointCloudTopic,
                    material: { size: 0.09, color: 0x7CFC00 }
                });
            }
            // Setup the URDF client
            urdfClientFull = new ROS3D.UrdfClient({
                ros: ROS,
                tfClient: tfClientFull,
                rootObject: viewerFull.scene,
                loader: ROS3D.COLLADA_LOADER_2
            });
        } else {
            cleanFullViewer();
        }
    }
    // Create the main viewer
    if (viewerFull === null) {
        WIDTH_FULL = $('#urdfModal').width();
        HEIGHT_FULL = $('#urdfModal').height();
        viewerFull = new ROS3D.Viewer({
            divID: 'urdfFull',
            width: WIDTH_FULL,
            height: HEIGHT_FULL,
            antialias: true,
            background: colorViewer,
            cameraPose: { x: 0, y: 4, z: 2 }
        });
        // Add a 3D grid
        viewerFull.addObject(new ROS3D.Grid({
            color: colorMain,
            lineWidth: 0.5,
            cellSize: 0.5
        }));
        if (ROS && checkboxesElements[14].checked) {
            // Fullscreen viewer
            tfClientFull = new ROSLIB.TFClient({
                ros: ROS,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: pointCloudFrame
            });
            if (checkboxesElements[15].checked) {
                // Point cloud client
                cloudClient = new ROS3D.PointCloud2({
                    ros: ROS,
                    tfClient: tfClientFull,
                    rootObject: viewerFull.scene,
                    topic: pointCloudTopic,
                    material: { size: 0.09, color: 0x7CFC00 }
                });
            }
            // Setup the URDF client
            urdfClientFull = new ROS3D.UrdfClient({
                ros: ROS,
                tfClient: tfClientFull,
                rootObject: viewerFull.scene,
                loader: ROS3D.COLLADA_LOADER_2
            });
        }
    } else {
        // Check dimensions and resize
        var newWidth = WIDTH_FULL;
        var newHeight = HEIGHT_FULL;
        if (WIDTH_FULL !== $('#urdfModal').width()) {
            newWidth = $('#urdfModal').width();
        }
        if (HEIGHT_FULL !== $('#urdfModal').height()) {
            newHeight = $('#urdfModal').height();
        }
        viewerFull.resize(newWidth, newHeight);
    }
});
closePointCloudElement.addEventListener('click', () => {
    cleanFullViewer();
});
function cleanFullViewer() {
    // Remove URDF from full viewer
    if (urdfClientFull !== null && urdfClientFull !== undefined) {
        viewerFull.scene.remove(urdfClientFull.urdf);
        urdfClientFull = null;
    }
    // Remove point cloud
    if (cloudClient !== null && cloudClient !== undefined && cloudClient.points.sn !== null) {
        for (var j = cloudClient.points.sn.children.length; j > 0; j--) {
            cloudClient.points.sn.remove(cloudClient.points.sn.children[j - 1]);
            viewerFull.scene.remove(cloudClient.points.sn);
        }
        cloudClient = null;
    }
}
// Add listener on keyboard settings
defaultControlElement.addEventListener('change', () => {
    changeKeyboard(0);
});
arrowsControlElement.addEventListener('change', () => {
    changeKeyboard(1);
});
// Initiallize app on page loading
function init() {
    // Display main block
    mainElement.style.display = 'block';
    // Initiallize navigation joystick
    mainJoystick = nipplejs.create({
        zone: joyElement,
        mode: 'static',
        position: { left: '0', top: '0' },
        color: colorMain,
        size: 70
    });
    // Initiallize statistics chart
    statisticsChart = new Chart(chartElement, {
        type: 'line',
        data: {
            labels: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
            datasets: [{
                label: ['CPU 1'],
                backgroundColor: colorsChart[0],
                data: cpuD1,
                borderColor: 'transparent',
                fill: true,
                tension: 0.3,
                pointRadius: 0
            },
            {
                label: ['CPU 2'],
                backgroundColor: colorsChart[1],
                data: cpuD2,
                borderColor: 'transparent',
                fill: true,
                tension: 0.3,
                pointRadius: 0
            },
            {
                label: ['CPU 3'],
                backgroundColor: colorsChart[2],
                data: cpuD3,
                borderColor: 'transparent',
                fill: true,
                tension: 0.3,
                pointRadius: 0
            },
            {
                label: ['CPU 4'],
                backgroundColor: colorsChart[3],
                data: cpuD4,
                borderColor: 'transparent',
                fill: true,
                tension: 0.3,
                pointRadius: 0
            },
            {
                label: ['RAM'],
                backgroundColor: colorsChart[4],
                data: memoryD,
                borderColor: 'transparent',
                fill: true,
                tension: 0.3,
                pointRadius: 0
            }]
        },
        options: {
            animation: { duration: 0 },
            scales: {
                y: {
                    beginAtZero: true,
                    stepSize: 20,
                    max: 100,
                    ticks: {
                        font: { size: 8 },
                        callback: (value, _index, _values) => {
                            return value + '%';
                        }
                    }
                },
                x: {
                    ticks: { font: { size: 8 } }
                }
            },
            plugins: {
                legend: {
                    labels: {
                        font: { size: 7 },
                        boxWidth: 10
                    }
                },
                tooltip: {
                    bodyFont: { size: 8 },
                    titleFont: { size: 10 }
                }
            }
        }
    });
    // Set on Start, on Move and on End functionalities
    self.mainJoystick.on('start', () => {
        // Enable a function that runs continuously and send Twist messages
        joyTimer = setInterval(() => {
            if (connectStatus && navigationMethod === 0) {
                moveRobot(linearSpeed, angularSpeed);
            }
        }, 25); // Publish every 25 milliseconds every message
    });
    self.mainJoystick.on('move', (_event, nipple) => {
        if (fixedVelocity) {
            linit = Math.sin(nipple.angle.radian) * 2 * (nipple.distance / maxDistance);
            ainit = -Math.cos(nipple.angle.radian) * 2 * (nipple.distance / maxDistance);
            // Map values with user settings
            linearSpeed = mapValue(linit, -1, +1, fixedVelocityValues[1], fixedVelocityValues[0]);
            angularSpeed = mapValue(ainit, -1, +1, fixedVelocityValues[4], fixedVelocityValues[3]);
        } else {
            linearSpeed = Math.sin(nipple.angle.radian) * maxLinear * 2 * (nipple.distance / maxDistance);
            angularSpeed = -Math.cos(nipple.angle.radian) * maxAngular * 2 * (nipple.distance / maxDistance);
        }
    });
    self.mainJoystick.on('end', () => {
        // Stop timer if it is enabled and set linear speed to zero to stop robot
        if (joyTimer) {
            clearInterval(joyTimer);
        }
        if (fixedVelocity) {
            linearSpeed = fixedVelocityValues[2];
            angularSpeed = fixedVelocityValues[5];
        } else {
            linearSpeed = 0.0;
            angularSpeed = 0.0;
        }
        if (connectStatus && navigationMethod === 0) {
            moveRobot(linearSpeed, angularSpeed);
        }
    });
    // Width and height
    const WIDTH = urdfParentElement.clientWidth;
    const HEIGHT = urdfParentElement.clientHeight;
    // Create the main viewer
    viewer = new ROS3D.Viewer({
        divID: 'urdf',
        width: WIDTH,
        height: HEIGHT,
        antialias: true,
        background: colorViewer,
        cameraPose: { x: 0, y: 4, z: 2 }
    });
    // Add a 3D grid to the main viewer
    viewer.addObject(new ROS3D.Grid({
        color: colorMain,
        lineWidth: 0.5,
        cellSize: 0.5
    }));
}
// Map values
function mapValue(value, in_min, in_max, out_min, out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Pull the trigger for the connection
var connecting = false;
function pullTrigger() {
    if (!connectStatus && connecting === false) {
        spinnerElement.style.display = 'inline-block';
        connectElement.innerHTML = 'Connecting...';
        connecting = true;
        connect();
    } else if (connecting === true) {
        connecting = false;
        if (ROS !== undefined && ROS !== null) {
            ROS.close();
        }
    } else {
        spinnerElement.style.display = 'inline-block';
        connectElement.innerHTML = 'Disconnecting...';
        disconnectApp();
    }
}
// Connect to robot
function connect() {
    // Construct IP
    var IP = 'ws://' + ipElement.value + ':' + portElement.value;
    // ROS connection states
    ROS = new ROSLIB.Ros({
        url: IP
    });
    // Successful connection
    ROS.on('connection', () => {
        // Display status
        statusElement.innerHTML = 'Connected';
        // Display logs
        displayLogs('Connected successfully to IP: ' + IP + '\n' +
            'Now you can connect to the selected topics.\n', 'Connected successfully to IP: ' + IP + '\n' +
        'Now you can connect to the selected topics.\n');
        // Start operation timer
        timer = setInterval(startTimer, 1000);
        connectStatus = true;
        connecting = false;
        // Connect button modify
        spinnerElement.style.display = 'none';
        connectElement.innerHTML = 'Disconnect';
        // Enable buttons
        settingsButtonsElements[1].disabled = false;
        settingsButtonsElements[2].disabled = false;
        // Disable inputs
        disableEnableInputs(true);
    });
    // Error on connection
    ROS.on('error', () => {
        // Display status
        statusElement.innerHTML = 'Network error';
        connectionElement.innerHTML = 'No signal';
        // Display logs
        displayLogs('Connection error with IP: ' + IP + '\n', 'Connection error with IP: ' + IP + '\n');
        doOnErrorClose(timer);
    });
    // Closed connection
    ROS.on('close', () => {
        // Display status
        statusElement.innerHTML = 'Closed';
        connectionElement.innerHTML = 'No signal';
        // Display logs
        displayLogs('Connection closed with IP: ' + IP + '\n', 'Connection closed with IP: ' + IP + '\n');
        doOnErrorClose(timer);
    });
}
// Do actions on Error or on Close connection with ROS
function doOnErrorClose(timer) {
    // Ros logs
    logsStatus = false;
    // SLAM map close
    disconnectNavigation();
    // Stop timer
    if (timer) {
        clearInterval(timer);
    }
    totalSeconds = 0;
    secondsLabel.innerHTML = padding(0);
    minutesLabel.innerHTML = padding(0);
    hoursLabel.innerHTML = padding(0);
    connectStatus = false;
    connecting = false;
    connectTopicsStatus = false;
    // Restore buttons to disable
    settingsButtonsElements[1].disabled = true;
    settingsButtonsElements[2].disabled = true;
    // Connect button modify
    spinnerElement.style.display = 'none';
    connectElement.innerHTML = 'Connect';
    connectTopicsElement.innerHTML = 'Connect Topics';
    // Stop urdf client
    if (urdfClient !== null && urdfClient !== undefined) {
        viewer.scene.remove(urdfClient.urdf);
        urdfClient = null;
    }
    // Clean full screen viewer
    cleanFullViewer();
    // Close connection with web video server
    if (camerasViewer !== null && camerasViewer !== undefined) {
        camerasViewer.close();
        switchesElements[5].checked = false;
    }
    // Disable live feed
    liveFeedElement.style.display = 'block';
    liveElement.style.display = 'none';
    // Disable switches
    disableActionSwitches();
    // Enable inputs
    disableEnableInputs(false);
}
// Disable or enable user inputs to avoid error
function disableEnableInputs(state) {
    ipElement.disabled = state;
    portElement.disabled = state;
    streamIpElement.disabled = state;
    streamPortElement.disabled = state;
    for (var i = 0; i < topicsElements.length; i++) {
        topicsElements[i].disabled = state;
        checkboxesElements[i].disabled = state;
    }
}
// Pull the trigger for the topics connection
function pullTriggerTopics() {
    if (!connectTopicsStatus) {
        connectTopicsElement.innerHTML = 'Disconnect Topics';
        createPubSub();
        connectTopicsStatus = true;
    } else {
        connectTopicsElement.innerHTML = 'Connect Topics';
        deletePubSub();
        connectTopicsStatus = false;
    }
}
// Create publisher and subscribers
function createPubSub() {
    // Get user input topics
    var diagnosticsTopic = topicsElements[2].value;
    var statisticsTopic = topicsElements[3].value;
    var frame = topicsElements[4].value;
    var cpuTopic = topicsElements[5].value;
    var memoryTopic = topicsElements[6].value;
    var batteryTopic = topicsElements[7].value;
    var navigationTopic = topicsElements[8].value;
    var goalsTopic = topicsElements[9].value;
    var cancelGoalsTopic = topicsElements[10].value;
    var actionsTopic = topicsElements[11].value;
    var rangesTopic = topicsElements[12].value;
    var moveSomethingTopic = topicsElements[13].value;
    var temperaturesTopic = topicsElements[16].value;
    var gpsTopic = topicsElements[17].value;
    var gpsPointsTopic = topicsElements[19].value;
    // Display logs
    displayLogs('Connected to topics.\n', 'Connected to topics.\n');
    // Subscribers and publishers
    if (checkboxesElements[2].checked) {
        // Subscribe to receive diagnostics
        TOPICS[6] = new ROSLIB.Topic({
            ros: ROS,
            name: diagnosticsTopic,
            messageType: 'std_msgs/String'
        });
        // Receive command
        TOPICS[6].subscribe((msg) => {
            statusElement.innerHTML = msg.data;
        });
        displayLogs('Diagnostics topic connected.\n', 'Diagnostics topic connected.\n');
    }
    if (checkboxesElements[3].checked) {
        // Subscribe to receive statistics
        TOPICS[7] = new ROSLIB.Topic({
            ros: ROS,
            name: statisticsTopic,
            messageType: 'std_msgs/String'
        });
        // Receive command
        TOPICS[7].subscribe((msg) => {
            connectionElement.innerHTML = msg.data;
        });
        displayLogs('Statistics topic connected.\n', 'Statistics topic connected.\n');
    }
    if (checkboxesElements[8].checked) {
        // Publish topic to move robot
        TOPICS[0] = new ROSLIB.Topic({
            ros: ROS,
            name: navigationTopic,
            messageType: 'geometry_msgs/Twist'
        });
        // Enable movements for specific robot type
        TOPICS[13] = new ROSLIB.Topic({
            ros: ROS,
            name: '/cmd',
            messageType: 'std_msgs/UInt8'
        });
        // Arm wheels for specific robot type
        TOPICS[14] = new ROSLIB.Topic({
            ros: ROS,
            name: '/auto_nav',
            messageType: 'std_msgs/Bool'
        });
        displayLogs('Navigation topic connected.\n', 'Navigation topic connected.\n');
    }
    if (checkboxesElements[11].checked) {
        // Publish topic to move something
        TOPICS[1] = new ROSLIB.Topic({
            ros: ROS,
            name: moveSomethingTopic,
            messageType: 'geometry_msgs/Twist'
        });
        displayLogs('Move topic connected.\n', 'Move topic connected.\n');
    }
    if (checkboxesElements[12].checked) {
        // Publish actions topic to do something
        TOPICS[2] = new ROSLIB.Topic({
            ros: ROS,
            name: actionsTopic,
            messageType: 'std_msgs/UInt64MultiArray'
        });
        displayLogs('Actions topic connected.\n', 'Actions topic connected.\n');
    }
    if (checkboxesElements[13].checked) {
        // Publish ranges topic to do something
        TOPICS[3] = new ROSLIB.Topic({
            ros: ROS,
            name: rangesTopic,
            messageType: 'std_msgs/Float32MultiArray'
        });
        displayLogs('Ranges topic connected.\n', 'Ranges topic connected.\n');
    }
    if (checkboxesElements[9].checked) {
        // Publish to set new goal
        TOPICS[4] = new ROSLIB.Topic({
            ros: ROS,
            name: goalsTopic,
            messageType: 'geometry_msgs/PoseStamped'
        });
        displayLogs('Goals topic connected.\n', 'Goals topic connected.\n');
    }
    if (checkboxesElements[10].checked) {
        // Publish to cancel all goals
        TOPICS[5] = new ROSLIB.Topic({
            ros: ROS,
            name: cancelGoalsTopic,
            messageType: 'actionlib_msgs/GoalID'
        });
        displayLogs('Cancel goals topic connected.\n', 'Cancel goals topic connected.\n');
    }
    if (checkboxesElements[5].checked) {
        // Subscribe to receive CPU stats
        TOPICS[8] = new ROSLIB.Topic({
            ros: ROS,
            name: cpuTopic,
            messageType: 'std_msgs/UInt64MultiArray'
        });
        // Receive command
        TOPICS[8].subscribe((msg) => {
            visualizeStatsInChart(1, msg.data);
        });
        displayLogs('CPU topic connected.\n', 'CPU topic connected.\n');
    }
    if (checkboxesElements[6].checked) {
        // Subscribe to receive memory stats
        TOPICS[9] = new ROSLIB.Topic({
            ros: ROS,
            name: memoryTopic,
            messageType: 'std_msgs/UInt64'
        });
        // Receive command
        TOPICS[9].subscribe((msg) => {
            visualizeStatsInChart(2, msg.data);
        });
        displayLogs('Memory topic connected.\n', 'Memory topic connected.\n');
    }
    if (checkboxesElements[7].checked) {
        // Subscribe to receive battery stats
        TOPICS[10] = new ROSLIB.Topic({
            ros: ROS,
            name: batteryTopic,
            messageType: 'std_msgs/UInt64'
        });
        // Receive command
        TOPICS[10].subscribe((msg) => {
            battery.innerHTML = msg.data + '%';
        });
        displayLogs('Battery topic connected.\n', 'Battery topic connected.\n');
    }
    if (checkboxesElements[16].checked) {
        // Subscribe to receive battery stats
        TOPICS[11] = new ROSLIB.Topic({
            ros: ROS,
            name: temperaturesTopic,
            messageType: 'std_msgs/UInt64MultiArray'
        });
        // Receive command
        TOPICS[11].subscribe((msg) => {
            // Set CPU data
            chartElements[0].style.width = (msg.data[0] / 1000) + '%';
            chartElements[0].innerHTML = (msg.data[0] / 1000) + '°C';
            // Set GPU data
            chartElements[1].style.width = (msg.data[1] / 1000) + '%';
            chartElements[1].innerHTML = (msg.data[1] / 1000) + '°C';
        });
        displayLogs('Temperatures topic connected.\n', 'Temperatures topic connected.\n');
    }
    if (checkboxesElements[17].checked) {
        // Subscribe to GPS to receive coordinates
        TOPICS[12] = new ROSLIB.Topic({
            ros: ROS,
            name: gpsTopic,
            messageType: 'sensor_msgs/NavSatFix'
        });
        TOPICS[12].subscribe((msg) => {
            if (msg.latitude !== null && msg.longitude !== null && msg.latitude !== undefined && msg.longitude !== undefined) {
                marker.setLatLng({ lat: msg.latitude, lng: msg.longitude });
                LATITUDE_POINT = msg.latitude;
                LONGITUDE_POINT = msg.latitude;
                GPS_STATUS = true;
            } else {
                GPS_STATUS = false;
            }
        });
        displayLogs('GPS topic connected.\n', 'GPS topic connected.\n');
    }
    if (checkboxesElements[19].checked) {
        // Publish GPS points
        TOPICS[15] = new ROSLIB.Topic({
            ros: ROS,
            name: gpsPointsTopic,
            messageType: 'std_msgs/String'
        });
    }
    // Setup a client to listen to TFs
    if (checkboxesElements[4].checked) {
        tfClient = new ROSLIB.TFClient({
            ros: ROS,
            angularThres: 0.01,
            transThres: 0.01,
            rate: 10.0,
            fixedFrame: frame
        });
        // Setup the URDF client
        urdfClient = new ROS3D.UrdfClient({
            ros: ROS,
            tfClient: tfClient,
            rootObject: viewer.scene,
            loader: ROS3D.COLLADA_LOADER_2
        });
    }
}
// Disconnect topic
function deletePubSub() {
    for (var i = 0; i < 12; i++) {
        if (i < 6) {
            // Stop publishers
            TOPICS[i].unadvertise();
        } else {
            // Stop subscribers
            TOPICS[i].unsubscribe();
        }
    }
    // Stop urdf client
    if (urdfClient !== null && urdfClient !== undefined) {
        viewer.scene.remove(urdfClient.urdf);
        urdfClient = null;
    }
    // Clean full screen viewer
    cleanFullViewer();
    // Close connection with web video server
    if (camerasViewer !== null && camerasViewer !== undefined) {
        camerasViewer.close();
        switchesElements[5].checked = false;
    }
    // Diasble live feed
    liveFeedElement.style.display = 'block';
    liveElement.style.display = 'none';
    // Print to logs
    displayLogs('Topics are disconnected and connections are closed!\n', '');
}
// Disconnect from ROS
function disconnectApp() {
    if (ROS !== undefined && ROS !== null) {
        if (ROS.isConnected) {
            ROS.close();
        }
    }
}
// Reconnect
function reconnect() {
    $('#resetModal').modal('show');
}
// Open settings
function openSettings() {
    $('#connectModal').modal('show');
}
// Open ranges
function changeRanges() {
    $('#rangesModal').modal('show');
}
// Publish geometry_msgs/Twist messages to move robot
function moveRobot(linear, angular) {
    var twist = new ROSLIB.Message({
        linear: {
            x: linear,
            y: 0.0,
            z: 0.0
        },
        angular: {
            x: 0.0,
            y: 0.0,
            z: angular
        }
    });
    if (connectStatus) {
        TOPICS[0].publish(twist);
        linearElement.innerHTML = linear.toFixed(3);
        angularElement.innerHTML = angular.toFixed(3);
    }
}
// Set angle value
function setAngle(id) {
    console.log('Angle: ' + id);
}
// Send predefined goals
function sendGoal(id) {
    switch (parseInt(id)) {
        case 1:
            setGoal(0, 0, 0, 0);
            break;
        case 2:
            setGoal(0, 0, 0, 0);
            break;
        case 3:
            setGoal(0, 0, 0, 0);
            break;
        case 4:
            setGoal(0, 0, 0, 0);
            break;
        case 5:
            setGoal(0, 0, 0, 0);
            break;
        case 6:
            setGoal(0, 0, 0, 0);
            break;
        case 7:
            setGoal(0, 0, 0, 0);
            break;
        case 8:
            setGoal(0, 0, 0, 0);
            break;
        case 9:
            setGoal(0, 0, 0, 0);
    }
}
// Publish geometry_msgs/PoseStamped messages to set goal
function setGoal(px, py, oz, ow) {
    var goal = new ROSLIB.Message({
        header: {
            stamp: Date.now(),
            frame_id: 'map'
        },
        pose: {
            position: {
                x: px,
                y: py,
                z: 0.0
            },
            orientation: {
                x: 0.0,
                y: 0.0,
                z: oz,
                w: ow
            }
        }
    });
    if (connectStatus) {
        statusElement.innerHTML = 'Simple goal';
        TOPICS[4].publish(goal);
    }
}
// Cancel all goals
function cancelGoals() {
    var goal = new ROSLIB.Message({ id: '' });
    if (connectStatus) {
        statusElement.innerHTML = 'All goals canceled';
        TOPICS[5].publish(goal);
    }
}
// Clear main logs
function clearMainLogs() {
    mainLogsElement.value = '';
}
// Clear logs
function clearLogs() {
    logsElement.value = '';
}
// Display help on main log
function help() {
    displayLogs('Main Documentantion\n' +
        '* General *\n' +
        '    1. Set the Robot IP or localhost if ROS is running on the same machine.\n' +
        '    2. Set the Stream IP or localhost for the cameras live feed streaming.\n' +
        '    3. Set the topics and check the checkboxes for each.\n' +
        '    4. Connect to ROS and then connect to topics.\n' +
        '* Settings *\n' +
        '    1. When the connection is established you can connect and display topics.\n' +
        '    2. Live feed uses web video server and is connected to different IP.\n' +
        '    3. Save settings to keep them for the next execution. Just press <Restore> button.\n' +
        '    4. Frame is used for 3D model.\n' +
        '    4. P.C.2 Frame is used for 3D model of the full screen viewer.\n' +
        '* Topics *\n' +
        '    1. Diagnostics, Statistics, CPU Usage, Memory Usage, Battery Usage, P.C.2 and Temperatures topics are for subscribing.\n' +
        '    2. Navigation, Goals, Cancel Goals, Actions, Ranges and Movements topics are for publishing.\n' +
        '* Installation *\n' +
        '    1. $ sudo apt-get install ros-melodic-tf2-ros\n' +
        '    2. $ sudo apt-get install ros-melodic-map-server\n' +
        '    3. $ sudo apt-get install ros-melodic-usb-cam\n' +
        '    4. $ sudo apt-get install ros-melodic-web-video-server\n' +
        '    5. $ sudo apt-get install ros-melodic-rosbridge-server\n' +
        '    6. $ sudo apt-get install ros-melodic-joint-state-publisher-gui\n' +
        '    7. $ sudo apt-get install ros-melodic-octomap\n' +
        '    8. $ sudo apt-get install ros-melodic-octomap-server\n' +
        '    9. $ sudo apt-get install ros-melodic-octomap-rviz-plugins\n' +
        '    10. $ sudo apt-get install ros-melodic-octomap-mapping\n' +
        '    11. $ sudo apt-get install ros-melodic-octomap-ros\n' +
        '    12. $ sudo apt-get install ros-melodic-octomap-msgs\n' +
        '    13. $ sudo apt-get install ros-melodic-visualization-tutorials\n' +
        '* Developer *\n' +
        '    George Alexakis - geosalexs@gmail.com\n', '');
}
// Get published topics from ROS
function getTopics(position) {
    // Init topics client
    var topicsClient = new ROSLIB.Service({
        ros: ROS,
        name: '/rosapi/topics',
        serviceType: 'rosapi/Topics'
    });
    // Init service
    var request = new ROSLIB.ServiceRequest();
    // Print topics to logs
    topicsClient.callService(request, (result) => {
        const allTopics = result.topics;
        var txt = '';
        txt += 'Topics List:\n';
        for (var i = 0; i < allTopics.length; i++) {
            txt += '' + allTopics[i] + '\n';
        }
        if (position == 0) {
            displayLogs(txt, '');
        }
        if (position == 1) {
            displayLogs('', txt);
        }
    });
}
// Display stats in chart
function visualizeStatsInChart(component, value) {
    if (component === 1) {
        if (value[0]) {
            cpuD1.pop();
            cpuD1.unshift(value[0]);
            statisticsChart.data.datasets[0].data = cpuD1;
        }
        if (value[1]) {
            cpuD2.pop();
            cpuD2.unshift(value[1]);
            statisticsChart.data.datasets[1].data = cpuD2;
        }
        if (value[2]) {
            cpuD3.pop();
            cpuD3.unshift(value[2]);
            statisticsChart.data.datasets[2].data = cpuD3;
        }
        if (value[3]) {
            cpuD4.pop();
            cpuD4.unshift(value[3]);
            statisticsChart.data.datasets[3].data = cpuD4;
        }
    } else {
        memoryD.pop();
        memoryD.unshift(value);
        statisticsChart.data.datasets[4].data = memoryD;
    }
    statisticsChart.update();
}
// Test graphs
function tests() {
    testChart();
    testStats();
}
// Test chart
function testChart() {
    statisticsChart.data.datasets[0].data = [30, 20, 40, 0, 10, 20, 20, 5, 90, 10, 6];
    statisticsChart.data.datasets[1].data = [0, 10, 50, 70, 10, 0, 40, 90, 20, 30, 6];
    statisticsChart.data.datasets[2].data = [90, 60, 0, 90, 60, 70, 90, 10, 50, 70, 7];
    statisticsChart.data.datasets[3].data = [40, 60, 0, 90, 60, 20, 80, 10, 50, 70, 8];
    statisticsChart.data.datasets[4].data = [80, 60, 0, 90, 20, 70, 90, 10, 50, 50, 9];
    statisticsChart.update();
}
// Test stats
function testStats() {
    for (var i = 0; i < 6; i++) {
        // Random number from 0 to 100
        const randomNumber = Math.floor(Math.random() * 101);
        chartElements[i].style.width = randomNumber + '%';
        chartElements[i].innerHTML = randomNumber + '%';
    }
}
// Speed range sliders
function modifyLinearSpeed() {
    var value = parseInt(linearSpeedElement.value);
    if (value >= 0) {
        if (value === 0) {
            maxLinear = 1;
        } else {
            maxLinear = value + 1;
        }
    } else {
        maxLinear = 1 / (Math.abs(value) + 1);
    }
    maxLinearElement.innerHTML = maxLinear;
}
function modifyAngularSpeed() {
    var value = parseInt(angularSpeedElement.value);
    if (value >= 0) {
        if (value === 0) {
            maxAngular = 1;
        } else {
            maxAngular = value + 1;
        }
    } else {
        maxAngular = 1 / (Math.abs(value) + 1);
    }
    maxAngularElement.innerHTML = maxAngular;
}
// Live stream viewer
function connectCameras() {
    // Init arrays and variables
    var labels = ['Camera No.1', 'Camera No.2'];
    var topics = [];
    // Init and reset DOM element
    liveElement.innerHTML = '';
    // Get user values
    const IP = streamIpElement.value;
    const PORT = streamPortElement.value;
    if (topicsElements[0].value !== '') {
        topics.push(topicsElements[0].value);
    }
    if (topicsElements[1].value !== '') {
        topics.push(topicsElements[1].value);
    }
    // Run live feed
    if (topics.length !== 0 && IP !== '' && PORT !== '') {
        initCameraViewer(topics, labels, IP, PORT);
    } else {
        liveFeedElement.style.display = 'block';
        liveElement.style.display = 'none';
    }
}
// Create the main viewer
function initCameraViewer(topicsArray, labelsArray, IP, PORT) {
    // Hide preview and display feed
    liveFeedElement.style.display = 'none';
    liveElement.style.display = 'flex';
    // Width and height
    const WIDTH = liveParentElement.clientWidth;
    const HEIGHT = liveParentElement.clientHeight;
    // Init viewer
    camerasViewer = new MJPEGCANVAS.MultiStreamViewer({
        divID: 'live',
        host: IP,
        port: PORT,
        width: WIDTH,
        height: HEIGHT,
        topics: topicsArray,
        labels: labelsArray,
        quality: 1,
        backgroundColor: '#323232'
    });
}
// Open keyboard panel
function keyboardSettings() {
    $('#keyboardSettingsModal').modal('show');
}
// Do the actions triggered by the keyboard
function keyboardTrigger(event) {
    var key = event.key;
    linearSpeed = 0.0;
    angularSpeed = 0.0;
    keyConsoleElement.innerHTML = key;
    if (navigationMethod === 1) {
        if (keyboardSelection === 0) {
            switch (key) {
                case 'w':
                case 'W':
                    linearSpeed = 0.1;
                    break;
                case 'a':
                case 'A':
                    angularSpeed = 0.1;
                    break;
                case 's':
                case 'S':
                    linearSpeed = -0.1;
                    break;
                case 'd':
                case 'D':
                    angularSpeed = -0.1;
                    break;
                default:
                    linearSpeed = 0.0;
                    angularSpeed = 0.0;
            }
            moveRobot(maxLinear * linearSpeed, maxAngular * angularSpeed);
        } else if (keyboardSelection === 1) {
            switch (key) {
                case 'ArrowUp':
                    linearSpeed = 0.1;
                    break;
                case 'ArrowLeft':
                    angularSpeed = 0.1;
                    break;
                case 'ArrowDown':
                    linearSpeed = -0.1;
                    break;
                case 'ArrowRight':
                    angularSpeed = -0.1;
                    break;
                default:
                    linearSpeed = 0.0;
                    angularSpeed = 0.0;
            }
            moveRobot(maxLinear * linearSpeed, maxAngular * angularSpeed);
        } else {
            // Stop for no keyboard selection
            moveRobot(0, 0);
        }
    }
}
// Clear panel of keyboard settings
function clearkeyboardTrigger() {
    keyConsoleElement.innerHTML = '';
}
// Change keyboard settings
function changeKeyboard(option) {
    keyboardSelection = option;
}
// Switches
function changeSwitchState(event) {
    const srcChecked = event.srcElement.checked;
    const srcId = event.srcElement.id;
    if (srcChecked) {
        if (srcId === 's0') {
            doAction(0);
        } else if (srcId === 's1') {
            doAction(1);
        } else if (srcId === 's2') {
            doAction(2);
        } else if (srcId === 's3') {
            doAction(3);
        } else if (srcId === 's4') {
            navMethodElement.innerHTML = 'Keyboard';
            navigationMethod = 1;
        } else if (srcId === 's5') {
            connectCameras();
        } else if (srcId === 's6') {
            displayGPSMap(true);
        } else if (srcId === 's7') {
            fixedVelocity = true;
            $('#fvelocityModal').modal('show');
        }
    }
    if (!srcChecked) {
        if (srcId === 's0') {
            doAction(0);
        } else if (srcId === 's1') {
            doAction(1);
        } else if (srcId === 's2') {
            doAction(2);
        } else if (srcId === 's3') {
            doAction(3);
        } else if (srcId === 's4') {
            navMethodElement.innerHTML = 'Joystick';
            navigationMethod = 0;
        } else if (srcId === 's5') {
            // Close connection with web video server
            if (camerasViewer !== null && camerasViewer !== undefined) {
                camerasViewer.close();
            }
            liveFeedElement.style.display = 'block';
            liveElement.style.display = 'none';
        } else if (srcId === 's6') {
            displayGPSMap(false);
        } else if (srcId === 's7') {
            fixedVelocity = false;
        }
    }
}
// Save settings to local storage
function saveSettingsToLocalStorage() {
    // Change storage state
    localStorage.setItem('STORAGE_STATE', true);
    // Initialize data object
    var CONTROLLER_DATA = {};
    // Save main settings
    CONTROLLER_DATA.robotIP = ipElement.value;
    CONTROLLER_DATA.robotPort = portElement.value;
    CONTROLLER_DATA.streamIP = streamIpElement.value;
    CONTROLLER_DATA.streamPort = streamPortElement.value;
    CONTROLLER_DATA.keyboard = keyboardSelection;
    // Save topics
    for (var i = 0; i < topicsElements.length; i++) {
        CONTROLLER_DATA['t' + i] = topicsElements[i].value;
        CONTROLLER_DATA['ct' + i] = checkboxesElements[i].checked;
    }
    // Save ranges
    for (var i = 0; i < ranges.length; i++) {
        ranges[i] = parseFloat(document.getElementById('range' + (i + 1)).value);
        CONTROLLER_DATA['range' + (i + 1)] = ranges[i];
    }
    // Save fixed velocities
    for (var i = 0; i < fixedVelocityValues.length; i++) {
        CONTROLLER_DATA['fv' + i] = fixedVelocityValues[i];
    }
    // Put the object into storage
    localStorage.setItem('CONTROLLER_DATA', JSON.stringify(CONTROLLER_DATA));
    // Print to logs
    displayLogs('', 'Settings saved to local storage!\n');
}
// Load settings from local storage
function loadSettings() {
    if (!connectStatus) {
        if (localStorage.getItem('STORAGE_STATE') === 'true') {
            // Retrieve the object from local storage
            const JSON_DATA = localStorage.getItem('CONTROLLER_DATA');
            CONTROLLER_DATA = JSON.parse(JSON_DATA);
            // Get individual values
            ipElement.value = CONTROLLER_DATA.robotIP;
            portElement.value = CONTROLLER_DATA.robotPort;
            streamIpElement.value = CONTROLLER_DATA.streamIP;
            streamPortElement.value = CONTROLLER_DATA.streamPort;
            // Get topic list
            for (var i = 0; i < topicsElements.length; i++) {
                topicsElements[i].value = CONTROLLER_DATA['t' + i];
                checkboxesElements[i].checked = CONTROLLER_DATA['ct' + i];
            }
            // Get keyboard selection
            keyboardSelection = CONTROLLER_DATA.keyboard;
            if (keyboardSelection === 0) {
                defaultControlElement.checked = true;
            } else {
                arrowsControlElement.checked = true;
            }
            // Load ranges
            for (var i = 0; i < ranges.length; i++) {
                ranges[i] = CONTROLLER_DATA['range' + (i + 1)];
                document.getElementById('range' + (i + 1)).value = ranges[i];
            }
            // Load fixed velocities
            for (var i = 0; i < fixedVelocityValues.length; i++) {
                fixedVelocityValues[i] = CONTROLLER_DATA['fv' + i];
                document.getElementById('fv' + i).value = fixedVelocityValues[i];
            }
            // Print to logs
            displayLogs('Settings loaded from local storage!\n', '');
        } else {
            // Print to logs
            displayLogs('There is no saved data!\n', '');
        }
    } else {
        displayLogs('Disconnect to load settings and then reconnect!\n', '');
    }
}
// Clear local storage
function clearLocalStorage() {
    localStorage.clear();
    displayLogs('', 'Settings cleared!\n');
}
// Display messages to logs
function displayLogs(mainString, secondaryString) {
    if (mainString !== '') {
        var mainLogs = mainLogsElement;
        mainLogs.value += mainString;
        mainLogs.focus();
        mainLogs.setSelectionRange(mainLogs.value.length, mainLogs.value.length);
    }
    if (secondaryString !== '') {
        var logs = logsElement;
        logs.value += secondaryString;
        logs.focus();
        logs.setSelectionRange(logs.value.length, logs.value.length);
    }
}
// Fullscreen mode
function enterFullscreen() {
    if (document.fullscreenElement || document.webkitFullscreenElement || document.mozFullScreenElement || document.msFullscreenElement) {
        if (document.exitFullscreen) {
            document.exitFullscreen();
        } else if (document.msExitFullscreen) {
            document.msExitFullscreen();
        } else if (document.mozCancelFullScreen) {
            document.mozCancelFullScreen();
        } else if (document.webkitExitFullscreen) {
            document.webkitExitFullscreen();
        }
    } else {
        if (document.documentElement.requestFullscreen) {
            document.documentElement.requestFullscreen();
        } else if (document.documentElement.webkitRequestFullscreen) {
            document.documentElement.webkitRequestFullscreen();
        } else if (document.documentElement.mozRequestFullScreen) {
            document.documentElement.mozRequestFullScreen();
        } else if (document.documentElement.msRequestFullscreen) {
            document.documentElement.msRequestFullscreen();
        }
    }
}
// Resize 3D model viewer
function adjustURDF() {
    // Resize the main viewer
    viewer.resize(urdfParentElement.clientWidth, urdfParentElement.clientHeight);
}
// Reset cameras viewer to keep aspect ratio
function resetViewer() {
    if (camerasViewer !== null && camerasViewer !== undefined) {
        camerasViewer.close();
        switchesElements[5].checked = false;
    }
    liveFeedElement.style.display = 'block';
    liveElement.style.display = 'none';
}
// Start operation timer
function startTimer() {
    ++totalSeconds;
    const seconds = totalSeconds % 60;
    const minutes = Math.trunc((totalSeconds / 60) % 60);
    const hours = Math.trunc(totalSeconds / 3600);
    secondsLabel.innerHTML = padding(seconds);
    minutesLabel.innerHTML = padding(minutes);
    hoursLabel.innerHTML = padding(hours);
}
// Add padding one digit if not exist
function padding(value) {
    var valueString = value + '';
    if (valueString.length < 2) {
        return '0' + valueString;
    } else {
        return valueString;
    }
}
// Gamepad change linear and angular speed, gamepad.js connection
function changeSpeed(type, state) {
    if (type === 0) {
        var maxL = parseInt(maxLinear);
        if (state === 0) {
            if (maxL >= 1.0 && maxL < 20) {
                maxL += 1.0;
            }
            if (maxL > 20) {
                maxL = 20;
            }
            maxLinear = maxL;
            maxLinearElement.innerHTML = maxLinear;
            linearSpeedElement.value = maxLinear;
        }
        if (state === 1) {
            if (maxL > 1.0 && maxL <= 20) {
                maxL -= 1.0;
            }
            if (maxL < 1) {
                maxL = 1;
            }
            maxLinear = maxL;
            maxLinearElement.innerHTML = maxLinear;
            linearSpeedElement.value = maxLinear;
        }
    }
    if (type === 1) {
        var maxA = parseInt(maxAngular);
        if (state === 0) {
            if (maxA >= 1.0 && maxA < 20) {
                maxA += 1.0;
            }
            if (maxA > 20) {
                maxA = 20;
            }
            maxAngular = maxA;
            maxAngularElement.innerHTML = maxAngular;
            angularSpeedElement.value = maxAngular;
        }
        if (state === 1) {
            if (maxA > 1.0 && maxA <= 20) {
                maxA -= 1.0;
            }
            if (maxA < 1) {
                maxA = 1;
            }
            maxAngular = maxA;
            maxAngularElement.innerHTML = maxAngular;
            angularSpeedElement.value = maxAngular;
        }
    }
}
// Gamepad set linear and angular speed, gamepad.js connection
var setLinear = 0;
var setAngular = 0;
function setSpeed(direction, distance) {
    if (connectStatus) {
        if (direction === 0) {
            setLinear = maxLinear * (-distance.toFixed(2));
            moveRobot(setLinear, setAngular);
        } else if (direction === 1) {
            setAngular = maxAngular * (-distance.toFixed(2));
            moveRobot(setLinear, setAngular);
        } else if (direction === 2) {
            setLinear = 0;
            moveRobot(setLinear, moveAngular);
        } else if (direction === 3) {
            setAngular = 0;
            moveRobot(setLinear, setAngular);
        }
    }
}
// Do selected movement
var moveLinear = 0;
var moveAngular = 0;
function doMovement(direction, distance) {
    if (connectStatus) {
        if (direction === 0) {
            moveLinear = (-distance.toFixed(2));
            moveSomething(moveLinear, moveAngular);
        } else if (direction === 1) {
            moveAngular = (-distance.toFixed(2));
            moveSomething(moveLinear, moveAngular);
        } else if (direction === 2) {
            moveLinear = 0;
            moveSomething(moveLinear, moveAngular);
        } else if (direction === 3) {
            moveAngular = 0;
            moveSomething(moveLinear, moveAngular);
        }
    }
}
// Publish geometry_msgs/Twist messages to move something
function moveSomething(linear, angular) {
    var twist = new ROSLIB.Message({
        linear: {
            x: linear,
            y: 0.0,
            z: 0.0
        },
        angular: {
            x: 0.0,
            y: 0.0,
            z: angular
        }
    });
    if (connectStatus) {
        TOPICS[1].publish(twist);
    }
}
// Do selected action
function doAction(state) {
    if (connectStatus) {
        // ROS MultiArray construction
        var dimension = new ROSLIB.Message({ label: 'actions_length', size: 4, stride: 1 });
        var layout = new ROSLIB.Message({ dim: [dimension], data_offset: 0 });
        var data = new ROSLIB.Message({ layout: layout, data: selectedAction });
        for (var i = 0; i < selectedAction.length; i++) {
            if (state === i) {
                if (selectedAction[i] === 0) {
                    data.data[i] = 1;
                    TOPICS[2].publish(data);
                    selectedAction[i] = 1;
                    switchesElements[i].checked = true;
                } else {
                    data.data[i] = 0;
                    TOPICS[2].publish(data);
                    selectedAction[i] = 0;
                    switchesElements[i].checked = false;
                }
                break;
            }
        }
    } else {
        disableActionSwitches();
    }
}
function disableActionSwitches() {
    // Set false when is not connected
    selectedAction = [0, 0, 0, 0];
    switchesElements[0].checked = false;
    switchesElements[1].checked = false;
    switchesElements[2].checked = false;
    switchesElements[3].checked = false;
}
// Publish ranges
function publishRanges() {
    if (connectStatus) {
        // Retrive data
        for (var i = 0; i < 4; i++) {
            ranges[i] = parseFloat(document.getElementById('range' + (i + 1)).value);
        }
        // ROS MultiArray construction
        var dimension = new ROSLIB.Message({ label: 'ranges_length', size: 4, stride: 1 });
        var layout = new ROSLIB.Message({ dim: [dimension], data_offset: 0 });
        var data = new ROSLIB.Message({ layout: layout, data: ranges });
        TOPICS[3].publish(data);
        displayLogs('', 'Ranges published!\n');
    }
}
// Init OSM maps and multiple controls
var marker;
var map;
var currentZoom = 3;
var viewType = 0;
var distance = 0.0;
var fullMapElement = document.getElementById('fullMap');
var focusMapElement = document.getElementById('focusMap');
var zoomMapElement = document.getElementById('zoomMap');
var unzoomMapElement = document.getElementById('unzoomMap');
var startOSMnavElement = document.getElementById('startOSMnav');
var clearOSMpinsElement = document.getElementById('clearOSMpins');
var osmPointsElement = document.getElementById('osmPoints');
var savePointElement = document.getElementById('savePoint');
// OSM marker icon
var OSMIcon = L.icon({
    iconUrl: 'images/pin.png',
    iconSize: [40, 40],
    iconAnchor: [20, 40]
});
var OSMIconPoint = L.icon({
    iconUrl: 'images/point.png',
    iconSize: [20, 20],
    iconAnchor: [10, 10]
});
// Move marker
focusMapElement.addEventListener('click', () => {
    focusMap({ lat: 35.3175676785051, lng: 25.10197073221207 });
});
zoomMapElement.addEventListener('click', () => {
    currentZoom++;
    if (currentZoom > 6) {
        currentZoom = 19;
        map.setZoom(currentZoom);
        focusMap({ lat: 35.3175676785051, lng: 25.10197073221207 });
    } else {
        if (currentZoom > 19) {
            currentZoom = 19;
        }
        map.setZoom(currentZoom);
    }
});
unzoomMapElement.addEventListener('click', () => {
    currentZoom--;
    if (currentZoom === 18) {
        currentZoom = 6;
    }
    if (currentZoom < 3) {
        currentZoom = 3;
    }
    map.setZoom(currentZoom);
});
osmPointsElement.addEventListener('click', () => {
    viewPoints();
});
var fullMapStatus = false;
fullMapElement.addEventListener('click', () => {
    if (!fullMapStatus) {
        document.getElementById('map-c').style.width = '100%';
        document.getElementById('map-c').style.height = '100%';
        document.getElementById('map-c').style.opacity = '1';
        document.getElementById('map-c').style.borderRadius = '2px';
        document.getElementById('map').style.borderRadius = '2px';
        fullMapStatus = true;
    } else {
        document.getElementById('map-c').style.width = '50%';
        document.getElementById('map-c').style.height = '40%';
        document.getElementById('map-c').style.opacity = '0.5';
        document.getElementById('map-c').style.borderRadius = '10px 2px 2px 2px';
        document.getElementById('map').style.borderRadius = '10px 2px 2px 2px';
        fullMapStatus = false;
    }
    map.invalidateSize();
});
function displayGPSMap(status) {
    if (status) {
        document.getElementById('map-c').style.display = 'block';
    } else {
        document.getElementById('map-c').style.display = 'none';
    }
    map.invalidateSize();
}
// Save robot GPS points
var LATITUDE_POINT = null;
var LONGITUDE_POINT = null;
var GPS_STATUS = false;
var GPS_POINTS = [];
savePointElement.addEventListener('click', () => {
    if (GPS_STATUS && (LATITUDE_POINT !== null) && (LONGITUDE_POINT !== null)) {
        GPS_POINTS.push({ lat: LATITUDE_POINT, lng: LONGITUDE_POINT });
        addPolyline(LATITUDE_POINT, LONGITUDE_POINT);
        addPoint({ latLng: [LATITUDE_POINT, LONGITUDE_POINT] }, map);
        // Logs
        displayLogs('', 'GPS points saved\n');
    }
});
// Focus map to current position
function focusMap(latLng) {
    map.setView(latLng);
    marker.setLatLng(latLng);
}
function initMap() {
    // Map options
    var options = {
        selector: 'map',
        TILE_SRC: 'tiles-library/tiles/{z}/{x}/{y}.png',
        currentZoom: currentZoom,
        latLng: [35.3187198, 25.1002132],
        options: {
            minZoom: 3,
            maxZoom: 19,
            zoomControl: false,
            reuseTiles: true,
            unloadInvisibleTiles: true,
            scrollWheelZoom: false
        }
    };
    map = L.map(options.selector, { zoomControl: options.zoomControl, reuseTiles: options.reuseTiles, unloadInvisibleTiles: options.unloadInvisibleTiles, scrollWheelZoom: options.scrollWheelZoom }).setView(options.latLng, options.currentZoom);
    // Tiles and marker
    L.tileLayer(options.TILE_SRC, options.options).addTo(map);
    // Robot marker
    addMarker({ latLng: options.latLng }, map);
    // Robot path
    polyline = L.polyline(polylineCoordinates, { color: '#383838' }).addTo(map);
    // Click event listener on map
    map.on('click', (event) => {
        const coordinates = event.latlng;
        addPolyline(coordinates.lat, coordinates.lng);
        addPoint({ latLng: [coordinates.lat, coordinates.lng] }, map);
    });
}
var markers = [];
var polyline = {};
// Create a red polyline from an array of LatLng points
var polylineCoordinates = [];
// Add point
function addPoint(options, map) {
    var point = L.marker(options.latLng, { draggable: true, icon: OSMIconPoint }).addTo(map);
    // Listeners
    const id = markers.length;
    point.on('drag', () => {
        polylineCoordinates[id][0] = markers[id].getLatLng().lat;
        polylineCoordinates[id][1] = markers[id].getLatLng().lng;
        map.removeLayer(polyline);
        polyline = L.polyline(polylineCoordinates, { color: '#383838' }).addTo(map);
        // Measure distance
        measureDistance();
    });
    // Push to marker array
    markers.push(point);
    // Measure distance
    measureDistance();
}
// Measure distance
function measureDistance() {
    distance = 0.0;
    if (polylineCoordinates.length > 1) {
        for (var i = 0; i < polylineCoordinates.length - 1; i++) {
            distance = distance + L.latLng(polylineCoordinates[i][0], polylineCoordinates[i][1]).distanceTo(L.latLng(polylineCoordinates[i + 1][0], polylineCoordinates[i + 1][1]));
        }
    }
    document.getElementById('distanceText').innerHTML = distance.toFixed(2) + ' meters';
}
// Add marker
function addMarker(options, map) {
    marker = L.marker(options.latLng, { icon: OSMIcon }).addTo(map);
}
// Add polyline
function addPolyline(latitude, longitude) {
    polylineCoordinates.push([latitude, longitude]);
    polyline.setLatLngs(polylineCoordinates);
}
// Display GPS points
function viewPoints() {
    // Display goals
    var body = document.getElementById('pointsModalBody');
    var points = '';
    var deleteButtons = [];
    for (var i = 0; i < polylineCoordinates.length; i++) {
        points = points
            + '<tr><th scope="row">'
            + (i + 1) + '</th><td>'
            + polylineCoordinates[i][0] + ', ' + polylineCoordinates[i][1]
            + '</td><td>'
            + '<button type="button" id="' + i + '" class="btn btn-sm btn-danger">Delete</button>'
            + '</td></tr>';
    }
    body.innerHTML = points;
    $('#pointsInfoModal').modal('show');
    for (var i = 0; i < polylineCoordinates.length; i++) {
        deleteButtons.push(document.getElementById('' + i));
        deleteButtons[i].addEventListener('click', (event) => {
            $('#pointsInfoModal').modal('hide');
            map.removeLayer(polyline);
            for (var i = 0; i < markers.length; i++) {
                map.removeLayer(markers[i]);
            }
            // Make markers array empty
            markers = [];
            polylineCoordinates.splice(parseInt(event.target.id), 1);
            for (var i = 0; i < polylineCoordinates.length; i++) {
                addPoint({ latLng: [polylineCoordinates[i][0], polylineCoordinates[i][1]] }, map);
            }
            polyline = L.polyline(polylineCoordinates, { color: '#383838' }).addTo(map);
        });
    }
}
// Start GPS navigation
startOSMnavElement.addEventListener('click', () => {
    if (polylineCoordinates.length === 0) {
        statusElement.innerHTML = 'No points';
    } else {
        if (connectStatus) {
            var pointsData = '|';
            for (var i = 0; i < polylineCoordinates.length; i++) {
                pointsData = pointsData + String(polylineCoordinates[i][0]) + '&' + String(polylineCoordinates[i][1]) + '|';
            }
            var points = new ROSLIB.Message({
                data: pointsData
            });
            TOPICS[15].publish(points);
            displayLogs('', 'GPS points published\n');
        }
    }
});
// Clear points
clearOSMpinsElement.addEventListener('click', () => {
    for (var i = 0; i < markers.length; i++) {
        map.removeLayer(markers[i]);
    }
    // Clear distance
    distance = 0.0;
    document.getElementById('distanceText').innerHTML = distance.toFixed(2) + ' meters';
    // Make markers array empty
    markers = [];
    // Make coordinates array empty
    polylineCoordinates = [];
    // Robot path
    map.removeLayer(polyline);
    polyline = L.polyline(polylineCoordinates, { color: '#383838' }).addTo(map);
    displayLogs('', 'GPS points cleared\n');
    // ROS gps node handler cleaning
    if (connectStatus) {
        var points = new ROSLIB.Message({
            data: ''
        });
        TOPICS[15].publish(points);
    }
});
// Select buttons
var selectButtons = [];
var selectStates = [true, true, true, true, true, true];
for (var i = 0; i < 7; i++) {
    selectButtons[i] = document.getElementById('sl' + i);
    // Also add listeners
    selectButtons[i].addEventListener('click', (event) => {
        switch (event.target.id) {
            case 'sl0':
                for (var i = 0; i < 6; i++) {
                    selectStates[i] = true;
                }
                for (var i = 0; i < 20; i++) {
                    checkboxesElements[i].checked = true;
                }
                break;
            case 'sl1':
                if (selectStates[1]) {
                    selectStates[1] = false;
                    for (var i = 0; i < 5; i++) {
                        checkboxesElements[i].checked = false;
                    }
                } else {
                    selectStates[1] = true;
                    for (var i = 0; i < 5; i++) {
                        checkboxesElements[i].checked = true;
                    }
                }
                break;
            case 'sl2':
                if (selectStates[2]) {
                    selectStates[2] = false;
                    for (var i = 0; i < 3; i++) {
                        checkboxesElements[i + 5].checked = false;
                    }
                } else {
                    selectStates[2] = true;
                    for (var i = 0; i < 3; i++) {
                        checkboxesElements[i + 5].checked = true;
                    }
                }
                break;
            case 'sl3':
                if (selectStates[3]) {
                    selectStates[3] = false;
                    for (var i = 0; i < 3; i++) {
                        checkboxesElements[i + 8].checked = false;
                    }
                } else {
                    selectStates[3] = true;
                    for (var i = 0; i < 3; i++) {
                        checkboxesElements[i + 8].checked = true;
                    }
                }
                break;
            case 'sl4':
                if (selectStates[4]) {
                    selectStates[4] = false;
                    for (var i = 0; i < 3; i++) {
                        checkboxesElements[i + 11].checked = false;
                    }
                } else {
                    selectStates[4] = true;
                    for (var i = 0; i < 3; i++) {
                        checkboxesElements[i + 11].checked = true;
                    }
                }
                break;
            case 'sl5':
                if (selectStates[5]) {
                    selectStates[5] = false;
                    for (var i = 0; i < 6; i++) {
                        checkboxesElements[i + 14].checked = false;
                    }
                } else {
                    selectStates[5] = true;
                    for (var i = 0; i < 6; i++) {
                        checkboxesElements[i + 14].checked = true;
                    }
                }
                break;
            case 'sl6':
                for (var i = 0; i < 6; i++) {
                    selectStates[i] = false;
                }
                for (var i = 0; i < 20; i++) {
                    checkboxesElements[i].checked = false;
                }
                break;
        }
    });
}
// SOUP vehicle
var armWheelsButton = document.getElementById('armWheels');
var disarmWheelsButton = document.getElementById('disarmWheels');
// Arm
armWheelsButton.addEventListener('click', () => {
    if (connectStatus) {
        TOPICS[14].publish({ data: true });
        // Set a tiny delay for the next command
        setTimeout(() => { TOPICS[13].publish({ data: 4 }); }, 100);
    }
});
// Disarm
disarmWheelsButton.addEventListener('click', () => {
    if (connectStatus) {
        TOPICS[14].publish({ data: false });
        // Set a tiny delay for the next command
        setTimeout(() => { TOPICS[13].publish({ data: 4 }); }, 100);
    }
});
// Load robot configurations
var configArray = [];
var config = 0;
var robotConfigSelection = document.getElementById('robotConfigSelection');
robotConfigSelection.addEventListener('change', () => {
    var response = JSON.parse(localStorage.getItem(robotConfigSelection.value));
    var configName = Object.keys(response)[0];
    var configFile = response[configName];
    loadConfig(configName, configFile);
});
function initConfigSelection() {
    removeAll(robotConfigSelection);
    addOptions(robotConfigSelection);
}
// Remove options from select element
function removeAll(selectBox) {
    while (selectBox.options.length > 0) {
        selectBox.remove(0);
    }
    // Set init option
    var option = document.createElement('option');
    option.setAttribute('selected', '');
    option.text = 'Select robot configuration';
    selectBox.add(option);
}
// Add options from select element
function addOptions(selectBox) {
    var config = JSON.parse(localStorage.getItem('config-array'));
    if (config !== null && config !== undefined) {
        for (var i = 0; i < config.length; i++) {
            var item = JSON.parse(localStorage.getItem(config[i]));
            var configName = Object.keys(item)[0];
            // Add options
            var option = document.createElement('option');
            option.value = configName;
            option.text = configName;
            selectBox.add(option);
        }
    }
}
// Download config file
document.getElementById('downloadConfig').addEventListener('click', () => {
    displayLogs('Downloading config files...\n', '');
    fetch('https://ros-config-app.herokuapp.com/').then((response) => {
        if (response.ok) {
            return response.json();
        }
        throw new Error('Something went wrong');
    }).then((response) => {
        if (response.length !== undefined && response !== {}) {
            removeAll(robotConfigSelection);
            configArray = [];
            for (var i = 0; i < response.length; i++) {
                var configName = Object.keys(response[i])[0];
                configArray.push(configName);
                var configFile = response[i];
                saveConfigToLocalStorage(configName, configFile);
                // Add options
                var option = document.createElement('option');
                option.value = configName;
                option.text = configName;
                robotConfigSelection.add(option);
            }
            localStorage.setItem('config-array', JSON.stringify(configArray));
        } else {
            displayLogs('No config files on server\n', '');
        }
    }).catch(() => {
        displayLogs('Error while retrieving config file\n', '');
    });
});
// Save config to local storage
function saveConfigToLocalStorage(configName, configFile) {
    // Put the object into storage
    localStorage.setItem(configName, JSON.stringify(configFile));
    // Print to logs
    displayLogs(configName + ' data saved to local storage!\n', '');
}
// Load downloaded config files
function loadConfig(configName, configFile) {
    CONTROLLER_DATA = configFile;
    // Get individual values
    ipElement.value = CONTROLLER_DATA.robotIP;
    portElement.value = CONTROLLER_DATA.robotPort;
    streamIpElement.value = CONTROLLER_DATA.streamIP;
    streamPortElement.value = CONTROLLER_DATA.streamPort;
    // Get topic list
    for (var i = 0; i < topicsElements.length; i++) {
        topicsElements[i].value = CONTROLLER_DATA['t' + i];
        checkboxesElements[i].checked = CONTROLLER_DATA['ct' + i];
    }
    // Load ranges
    for (var i = 0; i < ranges.length; i++) {
        ranges[i] = CONTROLLER_DATA['range' + (i + 1)];
        document.getElementById('range' + (i + 1)).value = ranges[i];
    }
    // Load fixed velocities
    for (var i = 0; i < fixedVelocityValues.length; i++) {
        fixedVelocityValues[i] = CONTROLLER_DATA['fv' + i];
        document.getElementById('fv' + i).value = fixedVelocityValues[i];
    }
    // Print to logs
    displayLogs(configName + ' settings loaded from local storage!\n', '');
}