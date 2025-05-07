/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 */

var NAV2D = NAV2D || {
  REVISION: '0.3.0'
};

// Global variables
var NAVIGATION = true;
var POSEESTIMATION = false;
var GOALSNAVIGATION = false;
var GOALMARKERS = [];
var GOALS = [];
// Be sure to create the elements with IDs STARTBUTTON, CLEANBUTTON
var STARTBUTTON = document.getElementById('STARTBUTTON');
var CLEANBUTTON = document.getElementById('CLEANBUTTON');
var refreshGoals;

/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. If
 * withOrientation is set to true, the user can also specify the orientation of
 * the robot by clicking at the goal position and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 */
NAV2D.Navigator = function (options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var serverName = options.serverName || '/move_base';
  var actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  var withOrientation = options.withOrientation || false;
  var robotMarkerImage = options.robotMarkerImage;
  var goalMarkerImage = options.goalMarkerImage;
  var iconsScale = options.iconsScale;
  this.rootObject = options.rootObject || new createjs.Container();


  // Setup the actionlib client
  var actionClient = new ROSLIB.ActionClient({
    ros: ros,
    actionName: actionName,
    serverName: serverName
  });

  // Pose estimation
  function setRobotPose(pose) {
    var robotPose = new ROSLIB.Topic({
      ros: ros,
      name: '/initialpose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    });

    var poseHome = new ROSLIB.Message({ header: { frame_id: 'map' }, pose: { pose: { position: { x: pose.position.x, y: pose.position.y, z: 0.0 }, orientation: { z: pose.orientation.z, w: pose.orientation.w } }, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942] } });
    robotPose.publish(poseHome);
  }

  /**
   * Send a goal to the navigation stack with the given pose.
   *
   * @param pose - the goal pose
   */
  function sendGoal(pose) {
    // Create a goal
    var goal = new ROSLIB.Goal({
      actionClient: actionClient,
      goalMessage: {
        target_pose: {
          header: {
            frame_id: 'map'
          },
          pose: pose
        }
      }
    });
    goal.send();

    // create a marker for the goal
    var goalMarker = new ROS2D.NavigationImage({
      size: iconsScale,
      image: goalMarkerImage,
      pulse: false
    });
    goalMarker.x = pose.position.x;
    goalMarker.y = -pose.position.y;
    goalMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
    goalMarker.scaleX = 1.0 / stage.scaleX;
    goalMarker.scaleY = 1.0 / stage.scaleY;
    that.rootObject.addChild(goalMarker);

    goal.on('result', function () {
      that.rootObject.removeChild(goalMarker);
    });
  }

  // Save multiple goals
  function saveGoals(pose) {
    // Create a goal
    var goal = new ROSLIB.Goal({
      actionClient: actionClient,
      goalMessage: {
        target_pose: {
          header: {
            frame_id: 'map'
          },
          pose: pose
        }
      }
    });
    // Save goals
    GOALS.push(goal);

    // Create a marker for the goal
    var goalMarker = new ROS2D.NavigationImage({
      size: iconsScale,
      image: goalMarkerImage,
      pulse: false
    });
    goalMarker.x = pose.position.x;
    goalMarker.y = -pose.position.y;
    goalMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
    goalMarker.scaleX = 1.0 / stage.scaleX;
    goalMarker.scaleY = 1.0 / stage.scaleY;
    that.rootObject.addChild(goalMarker);
    GOALMARKERS.push(goalMarker)
  }

  // Refresh goals
  refreshGoals = function () {
    for (var i = 0; i < GOALMARKERS.length; i++) {
      that.rootObject.removeChild(GOALMARKERS[i]);
    }
    GOALMARKERS = [];
    for (var i = 0; i < GOALS.length; i++) {
      // Create a marker for the goal
      var goalMarker = new ROS2D.NavigationImage({
        size: iconsScale,
        image: goalMarkerImage,
        pulse: false
      });
      goalMarker.x = GOALS[i].goalMessage.goal.target_pose.pose.position.x;
      goalMarker.y = -GOALS[i].goalMessage.goal.target_pose.pose.position.y;
      goalMarker.rotation = stage.rosQuaternionToGlobalTheta(GOALS[i].goalMessage.goal.target_pose.pose.orientation);
      goalMarker.scaleX = 1.0 / stage.scaleX;
      goalMarker.scaleY = 1.0 / stage.scaleY;
      that.rootObject.addChild(goalMarker);
      GOALMARKERS.push(goalMarker)
    }
  }

  // Send multiple goals on by one
  function sendMultipleGoals() {
    if (GOALS.length !== 0 && GOALS !== undefined && GOALS !== null) {
      GOALS[0].send();
      moving = true;
      GOALS[0].on('result', function () {
        GOALS.shift();
        sendMultipleGoals();
      });
    } else {
      for (var i = 0; i < GOALMARKERS.length; i++) {
        that.rootObject.removeChild(GOALMARKERS[i]);
      }
      GOALS = [];
      GOALMARKERS = [];
    }
  }

  // Get a handle to the stage
  var stage;
  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }
  // Enable touch events
  createjs.Touch.enable(stage);
  // Marker for the robot (eddie)
  var robotMarker = new ROS2D.NavigationImage({
    size: iconsScale,
    image: robotMarkerImage,
    pulse: false
  });

  // Wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = true;

  // Setup a listener for the robot pose
  var poseListener = new ROSLIB.Topic({
    ros: ros,
    name: '/robot_pose',
    messageType: 'geometry_msgs/Pose',
    throttle_rate: 100
  });
  poseListener.subscribe(function (pose) {
    // Update the robots position on the map
    robotMarker.x = pose.position.x;
    robotMarker.y = -pose.position.y;
    if (!initScaleSet) {
      robotMarker.scaleX = 1.0 / stage.scaleX;
      robotMarker.scaleY = 1.0 / stage.scaleY;
      initScaleSet = true;
    }

    // Change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
    robotMarker.visible = true;
  });

  // Display path traces
  var globalPathTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/DWAPlannerROS/global_plan',
    messageType: 'nav_msgs/Path'
  });
  var localPathTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/DWAPlannerROS/local_plan',
    messageType: 'nav_msgs/Path'
  });
  var pathTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/NavfnROS/plan',
    messageType: 'nav_msgs/Path'
  });
  // Paths
  var globalPath = new ROS2D.PathShape({
    strokeSize: 0.05,
    strokeColor: createjs.Graphics.getRGB(255, 128, 0, 0.66)
  });
  var localPath = new ROS2D.PathShape({
    strokeSize: 0.05,
    strokeColor: createjs.Graphics.getRGB(255, 0, 0, 0.66)
  });
  var path = new ROS2D.PathShape({
    strokeSize: 0.05,
    strokeColor: createjs.Graphics.getRGB(0, 128, 0, 0.66)
  });
  // Add to canvas
  this.rootObject.addChild(globalPath);
  this.rootObject.addChild(localPath);
  this.rootObject.addChild(path);
  // Subscribe to topics
  globalPathTopic.subscribe((message) => {
    globalPath.setPath(message);
  });
  localPathTopic.subscribe((message) => {
    localPath.setPath(message);
  });
  pathTopic.subscribe((message) => {
    path.setPath(message);
  });

  if (withOrientation === false) {
    // Setup a double click listener (no orientation)
    this.rootObject.addEventListener('dblclick', function (event) {
      // Convert to ROS coordinates
      var coords = stage.globalToRos(event.stageX, event.stageY);
      var pose = new ROSLIB.Pose({
        position: new ROSLIB.Vector3(coords)
      });
      // Send the single goal
      if (NAVIGATION === true) {
        sendGoal(pose);
      }
      // Estimate robot pose
      if (POSEESTIMATION === true) {
        setRobotPose(pose);
      }
      // Multiple goals
      if (GOALSNAVIGATION === true) {
        saveGoals(pose);
      }
    });
  } else { // withOrientation === true
    // setup a click-and-point listener (with orientation)
    var position = null;
    var positionVec3 = null;
    var thetaRadians = 0;
    var thetaDegrees = 0;
    var orientationMarker = null;
    var mouseDown = false;
    var xDelta = 0;
    var yDelta = 0;

    var mouseEventHandler = function (event, mouseState) {
      if (mouseState === 'down') {
        // get position when mouse button is pressed down
        position = stage.globalToRos(event.stageX, event.stageY);
        positionVec3 = new ROSLIB.Vector3(position);
        mouseDown = true;
      }
      else if (mouseState === 'move') {
        // remove obsolete orientation marker
        that.rootObject.removeChild(orientationMarker);

        if (mouseDown === true) {
          // if mouse button is held down:
          // - get current mouse position
          // - calulate direction between stored <position> and current position
          // - place orientation marker
          var currentPos = stage.globalToRos(event.stageX, event.stageY);
          var currentPosVec3 = new ROSLIB.Vector3(currentPos);

          orientationMarker = new ROS2D.NavigationImage({
            size: iconsScale,
            image: goalMarkerImage,
            pulse: false
          });

          xDelta = currentPosVec3.x - positionVec3.x;
          yDelta = currentPosVec3.y - positionVec3.y;
          thetaRadians = Math.atan2(xDelta, yDelta);
          thetaDegrees = thetaRadians * (180.0 / Math.PI);

          if (thetaDegrees >= 0 && thetaDegrees <= 180) {
            thetaDegrees += 270;
          } else {
            thetaDegrees -= 90;
          }

          orientationMarker.x = positionVec3.x;
          orientationMarker.y = -positionVec3.y;
          orientationMarker.rotation = thetaDegrees;
          orientationMarker.scaleX = 1.0 / stage.scaleX;
          orientationMarker.scaleY = 1.0 / stage.scaleY;

          that.rootObject.addChild(orientationMarker);
        }
      } else if (mouseDown) { // mouseState === 'up'
        // if mouse button is released
        // - get current mouse position (goalPos)
        // - calulate direction between stored <position> and goal position
        // - set pose with orientation
        // - send goal
        mouseDown = false;

        var goalPos = stage.globalToRos(event.stageX, event.stageY);

        var goalPosVec3 = new ROSLIB.Vector3(goalPos);

        xDelta = goalPosVec3.x - positionVec3.x;
        yDelta = goalPosVec3.y - positionVec3.y;

        thetaRadians = Math.atan2(xDelta, yDelta);

        if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
          thetaRadians += (3 * Math.PI / 2);
        } else {
          thetaRadians -= (Math.PI / 2);
        }

        var qz = Math.sin(-thetaRadians / 2.0);
        var qw = Math.cos(-thetaRadians / 2.0);
        var orientation = new ROSLIB.Quaternion({ x: 0, y: 0, z: qz, w: qw });
        var pose = new ROSLIB.Pose({
          position: positionVec3,
          orientation: orientation
        });

        // Send the single goal
        if (NAVIGATION === true) {
          sendGoal(pose);
        }
        // Estimate robot pose
        if (POSEESTIMATION === true) {
          setRobotPose(pose);
        }
        // Multiple goals
        if (GOALSNAVIGATION === true) {
          saveGoals(pose);
        }
      }
    };
    // Start navigation button listener
    STARTBUTTON.addEventListener('click', function () {
      sendMultipleGoals();
    });
    // Clean goals button listener
    CLEANBUTTON.addEventListener('click', function () {
      for (var i = 0; i < GOALMARKERS.length; i++) {
        that.rootObject.removeChild(GOALMARKERS[i]);
      }
      GOALS = [];
      GOALMARKERS = [];

      // Publish to /move_base/cancel to cancel all goals
      var cancelTopic = new ROSLIB.Topic({
        ros: ROS,
        name: '/move_base/cancel',
        messageType: 'actionlib_msgs/GoalID'
      });
      cancelTopic.publish(new ROSLIB.Message({ id: '' }));
    });

    this.rootObject.addEventListener('stagemousedown', function (event) {
      mouseEventHandler(event, 'down');
    });

    this.rootObject.addEventListener('stagemousemove', function (event) {
      mouseEventHandler(event, 'move');
    });

    this.rootObject.addEventListener('stagemouseup', function (event) {
      mouseEventHandler(event, 'up');
    });
  }
};

/**
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * A OccupancyGridClientNav uses an OccupancyGridClient to create a map for use with a Navigator.
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * rootObject (optional) - the root object to add this marker to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 *   * viewer - the main viewer to render to
 */
NAV2D.OccupancyGridClientNav = function (options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  var topic = options.topic || '/map';
  // Icons and scale of icons, be sure to use same dimensions
  this.robotMarkerImage = options.robotMarkerImage || 'images/eddie.png';
  this.goalMarkerImage = options.goalMarkerImage || 'images/marker.png';
  this.iconsScale = options.iconsScale || 1.0;
  var continuous = options.continuous;
  this.serverName = options.serverName || '/move_base';
  this.actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.rootObject = options.rootObject || new createjs.Container();
  this.viewer = options.viewer;
  this.withOrientation = options.withOrientation || false;
  this.navigator = null;

  // Setup a client to get the map
  var client = new ROS2D.OccupancyGridClient({
    ros: this.ros,
    rootObject: this.rootObject,
    continuous: continuous,
    topic: topic
  });
  client.on('change', function () {
    that.navigator = new NAV2D.Navigator({
      ros: that.ros,
      serverName: that.serverName,
      actionName: that.actionName,
      rootObject: that.rootObject,
      withOrientation: that.withOrientation,
      robotMarkerImage: that.robotMarkerImage,
      goalMarkerImage: that.goalMarkerImage,
      iconsScale: that.iconsScale
    });

    // Scale the viewer to fit the map
    that.viewer.scaleToDimensions(client.currentGrid.width, client.currentGrid.height);
    that.viewer.shift(client.currentGrid.pose.position.x, client.currentGrid.pose.position.y);
  });
};
