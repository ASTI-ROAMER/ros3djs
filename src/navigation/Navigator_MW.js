/**
 * @author Randel Capati - randelmc21@gmail.com
 *///Mitz - FOR MANAGED WAYPOINTS 


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
 *   * navigatorFrameID - tf frame ID on which the goal will be sent
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * tfClient- the TF client   (not used for now)
 *   * color (optional) - color of the marker of the **sent** pose
 *   * intermediateColor (optional) - color of the marker while dragging it around / choosing which orientation to go
 * 
 *   * isActive - the internal state whether Navigator works or not on clicks, 
 */

ROS3D.Navigator_MW = function(options) {
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
                            color:              0x476648,
                            intermediateColor:  0x8FB787,
                            defaultDirection:   new THREE.Vector3(1,0,0),};
  // Update/merge the defaultNavOptions with the given navOptions
  var navOptions = Object.assign({}, defaultNavOptions, options.navOptions);

  
  var serverName = navOptions.navServerName;   // we don't need to store serverName since it is encoded in this.actionClient
  var actionName = navOptions.navActionName;
  this.color = navOptions.color;
  this.intermediateColor = navOptions.intermediateColor;
  this.markerFrameID = navOptions.markerFrameID || this.navigatorFrameID;
  this.isActive = navOptions.navInitState;   // toggle this if you want navigation or not

  // initialize mouse and marker vars
  this.mouseDownPos = null;                       // roslib.Vector3 pos
  this.mouseDown = false;                         // if mousedown was previously detected
  this.currentGoal = null;                        // action goal message
  this.goalMarkerOptions = {color: this.color};

  this.latestMarker = null;
  // this.goalMarker = new THREE.Object3D();
  //MITZ
  this.goalList = new Array();                         //stores goal points

  
  // setup the actionlib client
  this.actionClient = new ROSLIB.ActionClient({
    ros : ros,
    actionName : actionName,
    serverName : serverName
  });
    
  // Since this is called by objects other than itself (addeventlistener on OGNav)
  this.mouseEventHandler = this.mouseEventHandlerUnbound.bind(this);
    
  
};

ROS3D.Navigator_MW.prototype.__proto__ = THREE.Object3D.prototype;


ROS3D.Navigator_MW.prototype.addPoseMarker = function(pos, ori={x:0.0, y:0.0, z:0.0, w:1.0}, c=this.color){
  this.goalMarkerOptions.origin  = new THREE.Vector3(pos.x, pos.y, pos.z);
  this.goalMarkerOptions.rot = new THREE.Quaternion(ori.x, ori.y, ori.z, ori.w);
  this.goalMarkerOptions.direction = new THREE.Vector3(1,0,0);
  this.goalMarkerOptions.direction.applyQuaternion(this.goalMarkerOptions.rot);
  this.goalMarkerOptions.material = new THREE.MeshBasicMaterial({color: c});

  var tempMarker = new ROS3D.NodePose(this.goalMarkerOptions);
  this.add(tempMarker);
  this.latestMarker = tempMarker;             // just so we know what the last marker was for easy access
}

// Applies an orientation (quaternion) as a direction to the latest marker or the given marker
ROS3D.Navigator_MW.prototype.updateMarkerOri = function(ori, marker=this.latestMarker, c=this.color){
  if (marker !== null){
    var rot = new THREE.Quaternion(ori.x, ori.y, ori.z, ori.w);
    var direction = new THREE.Vector3(1,0,0);
    direction.applyQuaternion(rot);
    marker.setDirection(direction);
    marker.setColor(c);
    this.rootObject.emit('change');
  }
}

// TODO: USE THIS TO ADD POINT CONNECTORS LATER
// - newPos defaults to last mouse down position!
// - oldPose defaults to LAST GOAL LIST ENTRY ***POSITION***
ROS3D.Navigator_MW.prototype.addConnectorMarker = function(newPos=this.mouseDownPos, oldPos=null, c=this.color){
  // if(this.goalList.length > 0){
  
  // If there is no given oldPos (null), use latest entry of goalList.
  if (oldPos === null){
    try {
      // If goalList is empty, getting its last entry's position will produce an error. oldPos will be undefined.
      oldPos = this.goalList.slice(-1)[0].position;
    }
    catch(e){
      // Since oldPos is non-existent, stop creating the connector.
      console.log('Nav_MW: goalList is empty. Will not create a connector.');
      return;
    }
  }
  // var oldPos = this.goalList.slice(-1)[0].position;       // get last pose in array 
  var connOptions = {};
  connOptions.p1 = new THREE.Vector3(oldPos.x, oldPos.y, oldPos.z);
  connOptions.p2 = new THREE.Vector3(newPos.x, newPos.y, newPos.z);
  connOptions.material = new THREE.MeshBasicMaterial({color: c});

  var connMarker = new ROS3D.NodePoseConnector(connOptions);
  
  // if p1 and p2 are too close, NodePoseConnector will have no geometry, so only add it to navigator iff there is geometry
  if(connMarker.geometry){
    this.add(connMarker);
  }
  // }
}

// ROS3D.Navigator_MW.prototype.calcDistance = function(p1, p2){
//   var dx = p2.x - p1.x;
//   var dy = p2.y - p1.y;
//   var dz = p2.z - p1.z;
//   return Math.sqrt(dx*dx + dy*dy + dz*dz);

// }
  
ROS3D.Navigator_MW.prototype.clearAllMarkers = function(){
  // redundant function, just for clarity
  this.clear();   // remember, Navigator is a THREE.Object3D, clearing its children will remove any markers
}


// RANDEL: Re-constructs all markers of all the waypoints.
ROS3D.Navigator_MW.prototype.updateAllMarkers = function(){
  this.clear();
  var old_pose = null;
  var pose = null;

  for (let i=0; i < this.goalList.length;  i++){
    old_pose = pose;
    pose = this.goalList[i];
    this.addPoseMarker(pose.position, pose.orientation);
    if (old_pose !== null){
      this.addConnectorMarker(pose.position, old_pose.position);
    }

  }
  // this.rootObject.emit('change');
}
  

//Mitz - update array
ROS3D.Navigator_MW.prototype.updateGoalList = function(pose){
  this.goalList.push(pose);
  this.rootObject.emit('navigationUpd');

  // console.log('index: ', this.goalList.length - 1);
  // console.log('x: ',pose.position.x);
  // console.log('y: ',pose.position.y);
  // console.log('z: ',pose.position.z);

}
//Mitz - delete pose on array
ROS3D.Navigator_MW.prototype.deletePose = function(index_no){
  var goalData = this.goalList.splice(index_no, 1)

  this.updateAllMarkers();                    // Brute-force update the markers
  this.rootObject.emit('navigationUpd');
  
}

// calculate ORIENTATION between (ROSLIB.Vector3) point1 and point2
ROS3D.Navigator_MW.prototype.calculateOrientation = function(p1, p2){
  var xDelta = p2.x - p1.x;
  var yDelta = p2.y - p1.y;
  // var zDelta = p2.z - p1.z;
  if (xDelta === 0.0 && yDelta === 0.0){
    return (new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw}));
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


ROS3D.Navigator_MW.prototype.clearGoalList = function(){
  this.goalList = new Array();
  this.rootObject.emit('navigationUpd');
  this.clear();         // clears markers
}


ROS3D.Navigator_MW.prototype.mouseEventHandlerUnbound = function(event3D){
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
        // DO NOT ACCEPT MOUSEOVER if you will not process that input further (dont handle it in the cases below.)
        //    This is because once the mouseover (prior the actual mouse input) is accepted, MouseHandler.notify()
        //    will only return 0 or 2. BUT to be able to give the mouse signals to orbit control, it HAS TO RETURN 1
        //    which WILL ONLY HAPPEN if the mouseover was not accepted in the first place.
        // So in mouseover case, list all possible mouse signals you want to further process.
        if ((event3D.domEvent.type === 'mousedown' && event3D.domEvent.button === 0 /*&& event3D.domEvent.buttons === 1*/)){
          event3D.stopPropagation();
        }
        break;
        
      case 'mousedown':
        // only handle mousedown for left mouse button (main button)
        // https://developer.mozilla.org/en-US/docs/Web/API/MouseEvent/buttons
        // https://developer.mozilla.org/en-US/docs/Web/API/MouseEvent/button
        if ((event3D.domEvent.button === 0) && (event3D.domEvent.buttons === 1)){
          var poi = event3D.intersection.point; 
          //console.log('nav: mouseDOWN');
          this.mouseDownPos = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
          this.mouseDown = true;
          
          var goal = this.mouseDownPos
          this.addPoseMarker(this.mouseDownPos, undefined, this.intermediateColor);        // ADD MARKER ON MOUSEDOWN, then update its orientation upon mouseup
          //store pose goal to a var to display
          
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
      
          // send the goal
          //this.sendGoal(pose);
          // this.displayPose(pose.position) // will convert location  to string
          // this.storeGoalPose(pose.position); //transfer pose.position to storegoal for action client
          this.addConnectorMarker(this.mouseDownPos);
          this.updateGoalList(pose);
          this.updateMarkerOri(orientation);  
          
          // console.log(pose.position);
          // this.updateAllMarkers();

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
          this.updateMarkerOri(orientation, this.latestMarker, this.intermediateColor);  
          // this.updateGoalMarker(this.mouseDownPos, orientation, this.intermediateColor)
          // this.updateAllMarkers();
          event3D.stopPropagation();
        }
        break;

      default:
        event3D.forceExitToFallbackTarget();
        break;               // DO NOT DO event3D.continuePropagation!!!
    }
  } 
}

ROS3D.Navigator_MW.prototype.calculateCurrentPOI = function(event3D){
  // RECALCULATE POI for mouse up since the current event3D.intersection.point is the mouse down location,
  // but we need the mouse UP position
  var poi;
  var mouseRaycaster = new THREE.Raycaster();
  mouseRaycaster.params.Line.threshold = 0.001;
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
  this.poiPose = true
  return poi;
  
}


ROS3D.Navigator_MW.prototype.activate = function(event3D){
  this.isActive = true;
  
}

ROS3D.Navigator_MW.prototype.deactivate = function(event3D){
  this.isActive = false;
  // this.clearGoalList();       // REMOVE THISSSSS, FOR DEBUGGING ONLY
}

ROS3D.Navigator_MW.prototype.toggleActivation = function(event3D){
  this.isActive = !this.isActive;

  // REMOVE THISSSSS, FOR DEBUGGING ONLY
  if (!this.isActive){
    // this.clearGoalList();
  }
}


  