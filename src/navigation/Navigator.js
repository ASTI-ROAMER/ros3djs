/**
 * @author Randel Capati - randelmc21@gmail.com
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. 
 * The navigator IS ALSO A RENDERABLE OBJECT3D.
 * Sort of ported from ros2d (nav2d) to ros3d.
 * This always uses pose with orientation.
 * Works by clicking at the goal position (on the OccupancyGridNav) and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * rootObject - the root object to add the click listeners to and render robot markers to (OccupancyGridClientNav)
 *   * navigatorFrameID - tf frame ID on which the goal will be sent
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * tfClient- the TF client   (not used for now)
 *   * color (optional) - color of the marker of the **sent** pose
 *   * intermediateColor (optional) - color of the marker while dragging it around / choosing which orientation to go
 * 
 *   * isActive - the internal state whether Navigator works or not on clicks, 
 */

 ROS3D.Navigator = function(options) {
  THREE.Object3D.call(this);
  var that = this;
  options = options || {};

  var ros = options.ros;
  this.tfClient = options.tfClient;
  this.rootObject = options.rootObject;
  this.navigatorFrameID = options.navigatorFrameID || 'map';    // this SHOULD ALWAYS BE tfclient's FIXED FRAME
  
  // the default options to be used by Navigator, update this with navOptions that is passed by the caller
  // we do this so we don't have to guard all the vars (e.g. var serverName = options.serverName || '/move_base';)
  var defaultNavOptions = { navServerName:      '/move_base',
                            navActionName:      'move_base_msgs/MoveBaseAction',
                            navInitState:       false,
                            color:              0xcc00ff,
                            intermediateColor:  0xEEACFF,};
  // Update/merge the defaultNavOptions with the given navOptions
  var navOptions = Object.assign({}, defaultNavOptions, options.navOptions);

  var serverName = navOptions.navServerName;   // we don't need to store serverName since it is encoded in this.actionClient
  var actionName = navOptions.navActionName;
  this.color = navOptions.color;
  this.intermediateColor = navOptions.intermediateColor;
  this.markerFrameID = navOptions.markerFrameID || this.navigatorFrameID;
  this.isActive = navOptions.navInitState;        // toggle this if you want navigation or not

  // initialize mouse and marker vars
  this.mouseDownPos = null;                       // roslib.Vector3 pos
  this.mouseDown = false;                         // if mousedown was previously detected
  this.currentGoal = null;                        // action goal message
  this.goalMarkerOptions = {color: this.color};
  this.goalMarker = null;

  // setup the actionlib client
  this.actionClient = new ROSLIB.ActionClient({
    ros : ros,
    actionName : actionName,
    serverName : serverName
  });

  
  // Since this is called by objects other than itself (addeventlistener on OGNav)
  this.mouseEventHandler = this.mouseEventHandlerUnbound.bind(this);
  

};

ROS3D.Navigator.prototype.__proto__ = THREE.Object3D.prototype;

ROS3D.Navigator.prototype.sendGoal = function(pose){
  // create a goal, use the message type move_base_msgs/MoveBaseAction
  var goalMessage = {
    target_pose : {
      header : {
        frame_id : this.navigatorFrameID,   // get the frame id of the Occupancy Grid, and use that frame when publishing the goal
      },
      pose : pose
    }
  };
  // Only add "priority_val" on roamer messages since move_base_msgs/MoveBaseAction has no priority,
  // will produce error if the message and message type fields do not match
  if (this.actionClient.actionName === 'roamer_msgs/MoveBaseAction'){
    goalMessage.priority_val = 1;   // Since prio=0 can't override previous prio=0 goals
  };

  var goal = new ROSLIB.Goal({
    actionClient : this.actionClient,
    goalMessage : goalMessage,
  });
  goal.send();
  console.log('nav: pose sent');
  
  this.currentGoal = goal;

  // update marker
  this.updateGoalMarker(pose.position, pose.orientation);

  
};




ROS3D.Navigator.prototype.updateGoalMarker = function(pos, orientation, c=this.color){
  // remove old marker first
  if (this.goalMarker !== null){
    this.remove(this.goalMarker);
  }

  this.goalMarkerOptions.origin  = new THREE.Vector3(pos.x, pos.y, pos.z);
  this.goalMarkerOptions.rot = new THREE.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  this.goalMarkerOptions.direction = new THREE.Vector3(1,0,0);
  this.goalMarkerOptions.direction.applyQuaternion(this.goalMarkerOptions.rot);
  this.goalMarkerOptions.material = new THREE.MeshBasicMaterial({color: c});

  this.goalMarker = new ROS3D.Arrow(this.goalMarkerOptions);
  this.add(this.goalMarker);

  this.rootObject.emit('change');
};


// calculate ORIENTATION between (ROSLIB.Vector3) point1 and point2
ROS3D.Navigator.prototype.calculateOrientation = function(p1, p2){
  var xDelta = p2.x - p1.x;
  var yDelta = p2.y - p1.y;
  // var zDelta = p2.z - p1.z;
  if (xDelta === 0.0 && yDelta === 0.0){
    return (new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw}));
    console.log('nav ori: same down and up point');

    // TODO: If the mouse has NOT BEEN DRAGGED, get the orientation between the current robot pose and clicked position
  }
  
  // calc orientation from mouseDownPos and mouseUpPos
  var thetaRadians  = Math.atan2(xDelta,yDelta);

  if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
    thetaRadians += (3 * Math.PI / 2);
  } else {
    thetaRadians -= (Math.PI/2);
  }

  var qz =  Math.sin(-thetaRadians/2.0);
  var qw =  Math.cos(-thetaRadians/2.0);

  return (new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw}));
};


ROS3D.Navigator.prototype.mouseEventHandlerUnbound = function(event3D){
  // only handle mouse events when active
  // 0: Accepted    , so either stop propagation or don't handle it
  // 1: Failed
  // 2: Continued
  // if (this.isActive && (event3D.domEvent.button === 0) && (event3D.domEvent.buttons === 1)){
  if (this.isActive){
    // var poi = event3D.intersection.point; 

    // check event3D.type (handled by MouseHandler), !!!NOT!!! event3D.domEvent.type
    // so that target will not be fallbackTarget (which is OrbitControl)
    switch(event3D.type){
      case 'mouseover':
        // accept a mouseover, as we need it in MouseHandler line 138
        // but only accept it if it is a left mouse click
        // we do this so that middle/wheel and right click events are not accepted, and instead caught by camera
        // if (event3D.domEvent.type === 'mousedown' && event3D.domEvent.button === 0 && event3D.domEvent.buttons === 1){
        if (event3D.domEvent.type === 'mousedown' && event3D.domEvent.button === 0){
          event3D.stopPropagation();
        }
        break;
        
      case 'mousedown':
        // only handle mousedown for left mouse button (main button)
        // https://developer.mozilla.org/en-US/docs/Web/API/MouseEvent/buttons
        // https://developer.mozilla.org/en-US/docs/Web/API/MouseEvent/button
        if ((event3D.domEvent.button === 0) && (event3D.domEvent.buttons === 1)){
          var poi = event3D.intersection.point; 
          console.log('nav: mouseDOWN');
          this.mouseDownPos = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
          this.mouseDown = true;
          // this.updateGoalMarker(this.mouseDownPos, this.defaultOri, this.intermediateColor)

          event3D.stopPropagation();
        } else {
          event3D.forceExitToFallbackTarget();
        }
        break;


      case 'mouseout':
      case 'mouseup':
        // mouse up for main button only (left button)
        if (this.mouseDown && (event3D.domEvent.button === 0)  ){
          // reset
          this.mouseDown = false;

          // RECALCULATE POI for mouse up since the current event3D.intersection.point is the mouse down location,
          // but we need the mouse UP position
          var poi = this.calculateCurrentPOI(event3D);

          // get pos on mouse up/out
          var mouseUpPos = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});

          var orientation = this.calculateOrientation(this.mouseDownPos, mouseUpPos);

  
          var pose = new ROSLIB.Pose({
            position :    this.mouseDownPos,
            orientation : orientation
          });

          // console.log('nav: mouseUP');
          // console.log('nav down: ' + this.mouseDownPos.x + ', ' + this.mouseDownPos.y + ', ' + this.mouseDownPos.z);
          // console.log('nav up: ' + mouseUpPos.x + ', ' + mouseUpPos.y + ', ' + mouseUpPos.z);
          // console.log('nav ori: ' + orientation.z + ', ' + orientation.w);
          
          // send the goal
          this.sendGoal(pose);

          this.mouseDownPos = null;       // reset
          event3D.stopPropagation();
        } 
        break;
        

      case 'mousemove':
        if (this.mouseDown){
          // RECALCULATE POI for mouse up since the current event3D.intersection.point is the mouse down location,
          // but we need the mouse UP position
          var poi = this.calculateCurrentPOI(event3D);

          // get pos on mouse up/out
          var mouseUpPos = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});

          var orientation = this.calculateOrientation(this.mouseDownPos, mouseUpPos);
          this.updateGoalMarker(this.mouseDownPos, orientation, this.intermediateColor);
        }
        break;

      default:
        event3D.forceExitToFallbackTarget();
        break;               // DO NOT DO event3D.continuePropagation!!!
    }
  } 
};

ROS3D.Navigator.prototype.calculateCurrentPOI = function(event3D){
  // RECALCULATE POI for mouse up since the current event3D.intersection.point is the mouse down location,
  // but we need the mouse UP position
  var poi = new THREE.Vector3();
  var mouseRaycaster = new THREE.Raycaster();
  mouseRaycaster.params.Line.threshold = 0.001;
  mouseRaycaster.setFromCamera(event3D.mousePos, event3D.camera);
  

  if(event3D.intersection.object.plane){
    // if there is a plane (there is for occupancyGrid/Nav)
    // https://discourse.threejs.org/t/raycaster-ray-intersecting-plane/2500
    mouseRaycaster.ray.intersectPlane(event3D.intersection.object.plane, poi);
  } else {
    // if object has no plane property

    // event3D.intersection.object is the OccupancyGridNav object which wast raycasted on previous mouse down
    // so recalculate intersection with that object
    var newIntersections = [];
    newIntersections = mouseRaycaster.intersectObject(event3D.intersection.object);

    if (newIntersections.length) {
      poi.copy(newIntersections[0].point);
    } else {
      poi.copy(event3D.intersection.point);       // revert to mouse down POI if it fails
    }
  }

  return poi;
};


ROS3D.Navigator.prototype.activate = function(event3D){
  this.isActive = true;
};

ROS3D.Navigator.prototype.deactivate = function(event3D){
  this.isActive = false;
};

ROS3D.Navigator.prototype.toggleActivation = function(event3D){
  this.isActive = !this.isActive;
  console.log('Navigator isActive: ' + this.isActive);
};
