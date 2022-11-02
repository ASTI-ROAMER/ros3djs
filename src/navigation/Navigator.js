/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 * @author Raffaello Bonghi - raffaello.bonghi@officinerobotiche.it
 * @author Randel
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
 *   * tfClient (optional) - the TF client
 *   * robot_pose (optional) - the robot topic or TF to listen position
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 */

ROS3D.Navigator = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  this.tfClient = options.tfClient || null;
  var robot_pose = options.robot_pose || '/robot_pose';
  var serverName = options.serverName || '/move_base';
  var actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  var withOrientation = options.withOrientation || false;
  var use_image = options.image;
  this.rootObject = options.rootObject;
  this.occupancyGridFrameID = options.occupancyGridFrameID || 'map';

  this.goalMarker = null;
  this.isActive = true;
  
  var currentGoal;


  this.mouseDownPos = null;
  this.mouseDown = false;             // if mousedown was previously detected
  this.orientationMarker = null;
  


  // binding mouse handlers
  // this.onMouseDblClick = this.onMouseDblClickUnbound.bind(this);
  // this.onMouseDown = this.onMouseDownUnbound.bind(this);

  // setup the actionlib client
  this.actionClient = new ROSLIB.ActionClient({
    ros : ros,
    actionName : actionName,
    serverName : serverName
  });
  
  // Since this called by objects other than itself (addeventlistener on OGNav)
  this.mouseEventHandler = this.mouseEventHandlerUnbound.bind(this);
  

};


ROS3D.Navigator.prototype.__proto__ = THREE.Mesh.prototype;

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
          var poi;
          var mouseRaycaster = new THREE.Raycaster();
          mouseRaycaster.linePrecision = 0.001;
          mouseRaycaster.setFromCamera(event3D.mousePos, event3D.camera);
          
          // event3D.intersection.object is the OccupancyGridNav object which wast raycasted on previous mouse down
          // so recalculate intersection with that object
          var newIntersections = [];
          newIntersections = mouseRaycaster.intersectObject(event3D.intersection.object, true);

          if (newIntersections.length > 0) {
            poi = newIntersections[0].point
          } else {
            poi = event3D.intersection.point;       // revert to mouse down POI if it fails
          }

          // get pos on mouse up/out
          var mouseUpPos = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});

          var xDelta = mouseUpPos.x - this.mouseDownPos.x;
          var yDelta = mouseUpPos.y - this.mouseDownPos.y;
          // var zDelta = mouseUpPos.z - this.mouseDownPos.z;
          
          // calc orientation from mouseDownPos and mouseUpPos
          var thetaRadians  = Math.atan2(xDelta,yDelta);

          if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
            thetaRadians += (3 * Math.PI / 2);
          } else {
            thetaRadians -= (Math.PI/2);
          }
  
          var qz =  Math.sin(-thetaRadians/2.0);
          var qw =  Math.cos(-thetaRadians/2.0);
  
          var orientation = new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw});
  
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
        

      // case 'mousemove':
      //   break;

      // default:
        // break;               // DO NOT DO event3D.continuePropagation!!!
    }
  } 
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

