/**
 * @author Randel Capati - randelmc21@gmail.com
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. 
 * Sort of ported from ros2d (nav2d) to ros3d.
 * This always uses pose with orientation.
 * Works by clicking at the goal position (on the OccupancyGridNav) and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * rootObject - the root object to add the click listeners to and render robot markers to (OccupancyGridClientNav)
 *   * occupancyGridFrameID - tf frame ID of current map, defaults to 'map'
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * tfClient (optional) - the TF client   (not used for now)
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
  this.rootObject = options.rootObject;
  this.occupancyGridFrameID = options.occupancyGridFrameID || 'map';
  var serverName = options.serverName || '/move_base';
  var actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  this.tfClient = options.tfClient || null;
  this.color = options.color || 0xcc00ff;
  this.intermediateColor = options.intermediateColor || 0x8f00b3;

  this.isActive = options.navigatorInitState || false;                           // toggle this if you want navigation or not

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
        frame_id : this.occupancyGridFrameID,   // get the frame id of the Occupancy Grid, and use that frame when publishing the goal
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
  this.updateGoalMarker(pose.position, pose.orientation)

  // remove old marker first
  // if (this.goalMarker !== null){
  //   this.remove(this.goalMarker);
  // }

  // this.goalMarkerOptions.origin  = new THREE.Vector3( pose.position.x, pose.position.y, pose.position.z);
  // this.goalMarkerOptions.rot = new THREE.Quaternion(pose.orientation.x, pose.orientation.y, 
  //   pose.orientation.z, pose.orientation.w);
  // this.goalMarkerOptions.direction = new THREE.Vector3(1,0,0);
  // this.goalMarkerOptions.direction.applyQuaternion(this.goalMarkerOptions.rot);
  // this.goalMarkerOptions.material = new THREE.MeshBasicMaterial({color: this.color});

  // this.goalMarker = new ROS3D.Arrow(this.goalMarkerOptions);
  // this.add(this.goalMarker);
  // // this.rootObject.forceUpdate();
  // // this.rootObject.sceneNode.add(this.goalMarker);
  // this.rootObject.emit('change');
};


ROS3D.Navigator.prototype.updateGoalMarker = function(pos, orientation, color){
  // remove old marker first
  if (this.goalMarker !== null){
    this.remove(this.goalMarker);
  }
  var c = color || this.color;

  this.goalMarkerOptions.origin  = new THREE.Vector3(pos.x, pos.y, pos.z);
  this.goalMarkerOptions.rot = new THREE.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  this.goalMarkerOptions.direction = new THREE.Vector3(1,0,0);
  this.goalMarkerOptions.direction.applyQuaternion(this.goalMarkerOptions.rot);
  this.goalMarkerOptions.material = new THREE.MeshBasicMaterial({color: c});

  this.goalMarker = new ROS3D.Arrow(this.goalMarkerOptions);
  this.add(this.goalMarker);

  this.rootObject.emit('change');
}


// calculate ORIENTATION between (ROSLIB.Vector3) point1 and point2
ROS3D.Navigator.prototype.calculateOrientation = function(p1, p2){
  var xDelta = p2.x - p1.x;
  var yDelta = p2.y - p1.y;
  // var zDelta = p2.z - p1.z;
  if (xDelta === 0.0 && yDelta === 0.0){
    console.log('nav ori: same down and up point');
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
}


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

          event3D.stopPropagation();
        } 
        break;

      // case 'dblclick':
      //   var coords = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
      //   var pose = new ROSLIB.Pose({
      //     position : new ROSLIB.Vector3(coords)
      //   });
      //   // send the goal
      //   this.sendGoal(pose);
      //   console.log('nav: mouseDBLCLICK');

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



          // var xDelta = mouseUpPos.x - this.mouseDownPos.x;
          // var yDelta = mouseUpPos.y - this.mouseDownPos.y;
          // // var zDelta = mouseUpPos.z - this.mouseDownPos.z;
          
          // // calc orientation from mouseDownPos and mouseUpPos
          // var thetaRadians  = Math.atan2(xDelta,yDelta);

          // if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
          //   thetaRadians += (3 * Math.PI / 2);
          // } else {
          //   thetaRadians -= (Math.PI/2);
          // }
  
          // var qz =  Math.sin(-thetaRadians/2.0);
          // var qw =  Math.cos(-thetaRadians/2.0);
  
          // var orientation = new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw});
  
          var pose = new ROSLIB.Pose({
            position :    this.mouseDownPos,
            orientation : orientation
          });

          console.log('nav: mouseUP');
          console.log('nav down: ' + this.mouseDownPos.x + ', ' + this.mouseDownPos.y + ', ' + this.mouseDownPos.z);
          console.log('nav up: ' + mouseUpPos.x + ', ' + mouseUpPos.y + ', ' + mouseUpPos.z);
          console.log('nav ori: ' + orientation.z + ', ' + orientation.w);
          
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
          this.updateGoalMarker(this.mouseDownPos, orientation, this.intermediateColor)
        }
        break;

      // default:
        // break;               // DO NOT DO event3D.continuePropagation!!!
    }
  } 
}

ROS3D.Navigator.prototype.calculateCurrentPOI = function(event3D){
  // RECALCULATE POI for mouse up since the current event3D.intersection.point is the mouse down location,
  // but we need the mouse UP position
  var poi;
  var mouseRaycaster = new THREE.Raycaster();
  mouseRaycaster.linePrecision = 0.001;
  mouseRaycaster.setFromCamera(event3D.mousePos, event3D.camera);
  
  // event3D.intersection.object is the OccupancyGridNav object which wast raycasted on previous mouse down
  // so recalculate intersection with that object
  var newIntersections = [];
  newIntersections = mouseRaycaster.intersectObject(event3D.intersection.object);

  if (newIntersections) {
    poi = newIntersections[0].point
  } else {
    poi = event3D.intersection.point;       // revert to mouse down POI if it fails
  }
  return poi
}


ROS3D.Navigator.prototype.activate = function(event3D){
  this.isActive = true;
}

ROS3D.Navigator.prototype.deactivate = function(event3D){
  this.isActive = false;
}

ROS3D.Navigator.prototype.toggleActivation = function(event3D){
  this.isActive = !this.isActive;
  console.log('Navigator isActive: ' + this.isActive);
}

