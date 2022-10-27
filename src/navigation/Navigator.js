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
  
  this.currentGoal = goal;
};



// ROS3D.Navigator.prototype.onMouseDownUnbound = function(e){
//   // convert to ROS coordinates

//   var poi = e.intersection.point; 
//   var coords = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
//   var pose = new ROSLIB.Pose({
//     position : new ROSLIB.Vector3(coords)
//   });
//   // send the goal
//   this.sendGoal(pose);
//   e.stopPropagation();
//   console.log('nav: mouseDOWN')

// };

// ROS3D.Navigator.prototype.onMouseDblClickUnbound = function(e){
//   // convert to ROS coordinates

//   var poi = e.intersection.point; 
//   var coords = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
//   var pose = new ROSLIB.Pose({
//     position : new ROSLIB.Vector3(coords)
//   });
//   // send the goal
//   this.sendGoal(pose);
//   e.stopPropagation();
//   console.log('nav: mouseDBLCLICK')

// };


ROS3D.Navigator.prototype.mouseEventHandlerUnbound = function(event3D){
  // only handle mouse events when active AND with left mouse button is pressed
  if (this.isActive && (event3D.domEvent.button === 0)){
    event3D.stopPropagation();
    switch(event3D.domEvent.type){
      case 'mousedown':
        var poi = event3D.intersection.point; 
        var coords = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
        var pose = new ROSLIB.Pose({
          position : new ROSLIB.Vector3(coords)
        });
        // send the goal
        this.sendGoal(pose);
        console.log('nav: mouseDOWN');
        break;

      case 'dblclick':
        var poi = event3D.intersection.point; 
        var coords = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
        var pose = new ROSLIB.Pose({
          position : new ROSLIB.Vector3(coords)
        });
        // send the goal
        this.sendGoal(pose);
        console.log('nav: mouseDBLCLICK');

      default:
        event3D.stopPropagation();      // If no one accepts, UNDO the stopPropagation()
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

