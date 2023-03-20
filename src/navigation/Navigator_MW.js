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
                            scaleMultiplier:    1.0,
                            color:              0x476648,
                            intermediateColor:  0x8FB787,
                            highlightColor:     0xffd11a,
                            defaultDirection:   new THREE.Vector3(1,0,0),};
  // Update/merge the defaultNavOptions with the given navOptions
  var navOptions = Object.assign({}, defaultNavOptions, options.navOptions);

  
  var serverName = navOptions.navServerName;   // we don't need to store serverName since it is encoded in this.actionClient
  var actionName = navOptions.navActionName;
  this.scaleMultiplier = navOptions.scaleMultiplier;
  this.color = navOptions.color;
  this.intermediateColor = navOptions.intermediateColor;
  this.highlightColor = navOptions.highlightColor;
  this.markerFrameID = navOptions.markerFrameID || this.navigatorFrameID;
  this.isActive = navOptions.navInitState;   // toggle this if you want navigation or not

  // initialize mouse and marker vars
  this.mouseDownPos = null;                       // roslib.Vector3 pos
  this.mouseDown = false;                         // if mousedown was previously detected
  this.goalMarkerOptions = {color: this.color};
  this.latestMarker = null;
  this.nodeMarkerList = [];                       // each elemen is an object {node:<NodePose>, conn0:<NodePoseConnector>}

  // Proxy handler object
  this.arrayChangeHandler = {
    that: this,       // maintain a reference of the navigator object in this proxy handler
    get: function(target, prop, value) {
      const val = target[prop];
      // console.log('GETTING [' + prop);
      if (typeof val === 'function') {
        if (['pop', 'push', 'splice'].includes(prop)) {
          return function () {
            var args = Array.from(arguments) 
            console.log('===' + prop);
            console.log('Args: [' + args + ']')

            // Manipulate nodeMarkerList corresponding to the ROSLIB.Pose in goalList.
            switch(prop){
              case 'push': {
                // Push can have multiple arguments that are individually pushed to the array.
                // Create a marker for each pushed element on the goalList!!!
                // Assume one element per push.
                let newROSPose = args[0];
                let oldROSPose = that.goalList.at(-1); 
                let poseMarker = that.addPoseMarker(newROSPose.position, newROSPose.orientation);
                let connMarker = null;
                if(oldROSPose){
                  connMarker = that.addConnectorMarker(newROSPose.position, oldROSPose.position);
                }
                that.nodeMarkerList.push({'node': poseMarker, 'conn0': connMarker})
                // args.forEach((el) => {
                //   var poseMarker = that.addPoseMarker(el.position, el.orientation);
                //   var oldROSPose = that.goalList.at(-1);
                //   var connMarker = null;
                //   if(!oldROSPose){
                //     connMarker = that.addConnectorMarker(el.position, oldROSPose.position);
                //   }
                  
                //   that.nodeMarkerList.push({'node': poseMarker, 'conn0': connMarker});
                // });
                that.rootObject.emit('navigationUpd');
              } break;
              case 'splice': {
                // RANDEL: NOW IT IS FULL IMPLEMENTATION OF SPLICE
                // Sometimes, splice is called with undefined: arr.splice(undefined, 1). It parses the undefined as 0.
                let [start, deleteCount, ...items] = args;
                let nodeIndexBefore, nodeIndexAfter, oldROSPose, newROSPose, nodeAfterMarkerObj, connMarker;

                switch(args.length){
                  case 0:
                    break;      // Don't do anything when no args

                  case 1:
                    start = start || 0;       // arr.splice(undefined) == arr.splice(0, arr.length)
                    deleteCount = that.goalList.length - start;
                    // Deliberate fall-through

                  case 2:
                    // Sometimes, splice is called with undefined: arr.splice(undefined, 1). It parses the undefined as 0.
                    start = start || 0;
                    deleteCount = deleteCount || 0;
                    // Get undeleted poses (FROM YET-TO-BE-MODIFIED ARRAY) before and after the deleted items, 
                    //  so we can adjust their connectors
                    nodeIndexBefore = start - 1;
                    nodeIndexAfter = start + deleteCount;
                    oldROSPose = that.goalList[nodeIndexBefore];      // **** DONT USE Array.at() *****
                    newROSPose = that.goalList[nodeIndexAfter];
                    nodeAfterMarkerObj = that.nodeMarkerList.at(nodeIndexAfter);

                    // Remove all nodes and connectors from scene for the given indeces
                    for(let i=start, c=0; c<deleteCount; c++, i++){
                      try{
                        Object.values(that.nodeMarkerList[i]).forEach(marker => that.remove(marker))
                        // that.remove(that.nodeMarkerList[i].node);       // simple removal of node only
                      } catch(err){
                        console.log('No marker to delete.')
                      }
                    }
                    // Modify conn0 of the next node, if it exists.
                    if(nodeAfterMarkerObj){
                      try{
                        // Delete the old connector first.
                        that.remove(nodeAfterMarkerObj.conn0);
                        connMarker = null;
                        if(oldROSPose){
                          // Create the new connector, only if there is a previous node
                          connMarker = that.addConnectorMarker(newROSPose.position, oldROSPose.position);
                        }
                        nodeAfterMarkerObj.conn0 = connMarker;
                      } catch(err){
                        console.log('Somethingsomething...')
                      }
                    }
                    that.nodeMarkerList.splice(start, deleteCount);
                    break;
                  
                  default:    // 3 or above ( with items to add)
                    // Sometimes, splice is called with undefined: arr.splice(undefined, 1). It parses the undefined as 0.
                    start = start || 0;
                    deleteCount = deleteCount || 0;
                    // Get undeleted poses (FROM YET-TO-BE-MODIFIED ARRAY) before and after the deleted items, 
                    //  so we can adjust their connectors
                    nodeIndexBefore = start - 1;
                    nodeIndexAfter = start + deleteCount;
                    oldROSPose = that.goalList[nodeIndexBefore];      // **** DONT USE Array.at() *****
                    newROSPose = that.goalList[nodeIndexAfter];
                    nodeAfterMarkerObj = that.nodeMarkerList.at(nodeIndexAfter);

                    // Remove all nodes and connectors from scene for the given indeces
                    for(let i=start, c=0; c<deleteCount; c++, i++){
                      try{
                        Object.values(that.nodeMarkerList[i]).forEach(marker => that.remove(marker))
                        // that.remove(that.nodeMarkerList[i].node);       // simple removal of node only
                      } catch(err){
                        console.log('No marker to delete.')
                      }
                    }

                    // create node markers for each item, then put them into a list
                    let lastInsertedPose = null;
                    let addedPoseMarkers= [];
                    connMarker = null;
                    for(let k in items){
                      let poseMarker = that.addPoseMarker(items[k].position, items[k].orientation);
                      if(oldROSPose){
                        connMarker = that.addConnectorMarker(items[k].position, oldROSPose.position);
                      }
                      lastInsertedPose = {'node': poseMarker, 'conn0': connMarker};
                      addedPoseMarkers.push(lastInsertedPose);
                      // reset and update
                      connMarker = null;
                      oldROSPose = items[k];
                    }

                    // Modify conn0 of the next node, if it exists.
                    if(nodeAfterMarkerObj){
                      try{
                        // Delete the old connector first.
                        that.remove(nodeAfterMarkerObj.conn0);
                        connMarker = null;
                        if(oldROSPose){
                          // Create the new connector, only if there is a previous node
                          connMarker = that.addConnectorMarker(newROSPose.position, oldROSPose.position);
                        }
                        nodeAfterMarkerObj.conn0 = connMarker;
                      } catch(err){
                        console.log('Somethingsomething...')
                      }
                    }
                    that.nodeMarkerList.splice(start, deleteCount, ...addedPoseMarkers);
                    break;
                }
                that.rootObject.emit('navigationUpd');
              } break;
              case 'pop': {
                var tempNodeObj = that.nodeMarkerList.pop();
                try{
                  for(var marker in Object.values(tempNodeObj)){
                    that.remove(marker); 
                  }
                } catch(err){
                  console.log('No marker to delete.')
                }
                that.rootObject.emit('navigationUpd');
              } break;
              default:
                that.rootObject.emit('navigationUpd');
            }
            return Array.prototype[prop].apply(target, arguments);      // This line (the apply) mutates the goalList
          }
        }
        return val.bind(target);
      }
      return val;
    },
    

    // set: function(target, property, value) {
    //   // console.log('@@setting [' + property + '] for [' + target + '] with value [' + value + ']');
    //   target[property] = value;
    //   // you have to return true to accept the changes
    //   // console.log(property, 'mutated to', value)
    //   // console.log("prop is integer: " + Number.isInteger(Number(property)))
    //   if (Number.isInteger(Number(property))){
    //     var poseMarker = that.addPoseMarker(value.position, value.orientation);
    //     // Set value of the linked sister array, when mutating original goalList array
    //     // Handle the node marker list.
    //     this.pNodeMarkerList[property] = {'node': poseMarker};         // "this" here refers to arrayChangeHandler proxy object

    //   } else {
    //     this.pNodeMarkerList[property] = value;
    //   }
    //   this.that.rootObject.emit('navigationUpd');
    //   return true;
    // },
    

  };
  // RANDEL: The original goal list object, each elem is a ROSLIB.Pose object
  this._goalListOrig = new Array();                     
  // RANDEL: This is a proxy object of the actual/original goalList
  this.goalList = new Proxy(this._goalListOrig, this.arrayChangeHandler);   

  
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

// UNUSED/REDUNDANT ????
ROS3D.Navigator_MW.prototype.findNodeMarkerObj = function(node){
  // node should be ROS3D.NodePose
  var index = this.nodeMarkerList.findIndex((el) => el['node'] === node);
  if(~index){
    return {'index': index, 'nodeMarkerListElem': this.nodeMarkerList[index]};
  } else{
    return {'index': index, 'nodeMarkerListElem': null};
  }    
};



ROS3D.Navigator_MW.prototype.addPoseMarker = function(pos, ori={x:0.0, y:0.0, z:0.0, w:1.0}, c=this.color){
  this.goalMarkerOptions.origin  = new THREE.Vector3(pos.x, pos.y, pos.z);
  this.goalMarkerOptions.rot = new THREE.Quaternion(ori.x, ori.y, ori.z, ori.w);
  this.goalMarkerOptions.direction = new THREE.Vector3(1,0,0);
  this.goalMarkerOptions.direction.applyQuaternion(this.goalMarkerOptions.rot);
  this.goalMarkerOptions.material = new THREE.MeshBasicMaterial({color: c});
  this.goalMarkerOptions.scaleMultiplier = this.scaleMultiplier;

  var tempMarker = new ROS3D.NodePose(this.goalMarkerOptions);
  this.add(tempMarker);
  return tempMarker;
  // this.latestMarker = tempMarker;             // just so we know what the last marker was for easy access
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


ROS3D.Navigator_MW.prototype.addConnectorMarker = function(newPos, oldPos,  c=this.color){
  // Create a Connector marker from previous node in the list to the current one, newPos and oldPos are Vector3 (have x,y,z)
  if(!newPos && !oldPos){
    return;                   // newPos and oldPos are required
  }

  var connOptions = {};
  connOptions.p1 = new THREE.Vector3(oldPos.x, oldPos.y, oldPos.z);
  connOptions.p2 = new THREE.Vector3(newPos.x, newPos.y, newPos.z);
  connOptions.material = new THREE.MeshBasicMaterial({color: c});

  var connMarker = new ROS3D.NodePoseConnector(connOptions);
  
  // if p1 and p2 are too close, NodePoseConnector will have no geometry, so only add it to navigator iff there is geometry
  if(connMarker.geometry){
    this.add(connMarker);
    return connMarker;
  }
};
  

ROS3D.Navigator_MW.prototype.clearAllMarkers = function(){
  // redundant function, just for clarity
  this.clear();   // remember, Navigator is a THREE.Object3D, clearing its children will remove any markers
}


// RANDEL: Re-constructs all markers of all the waypoints.  ***UNUSED/REDUNDANT*****
ROS3D.Navigator_MW.prototype.updateAllMarkers = function(){
  // Temporarily store goalList
  var goalList = Array.from(this.goalList);
  this.clearGoalList();
  
  for (var pose in goalList){
    this.goalList.push(pose);
  }
  this.rootObject.emit('change');
};
  

//Mitz - update array
ROS3D.Navigator_MW.prototype.pushToGoalList = function(pose){
  this.goalList.push(pose);             // Mutating the PROXY goalList should emit a signal
  // this.rootObject.emit('navigationUpd');
}

//Mitz - delete pose on array
ROS3D.Navigator_MW.prototype.deletePose = function(index_no){
  this.goalList.splice(index_no, 1);
  // this.rootObject.emit('navigationUpd');
}

// calculate ORIENTATION between (ROSLIB.Vector3) point1 and point2
ROS3D.Navigator_MW.prototype.calculateOrientation = function(p1, p2){
  if ( p1 === void 0 || p2 === void 0) {
    console.log('One or both given required points was not given, returning default value.')
    return (new ROSLIB.Quaternion());
  }
  var xDelta = p2.x - p1.x;
  var yDelta = p2.y - p1.y;
  // var zDelta = p2.z - p1.z;
  if (xDelta === 0.0 && yDelta === 0.0){
    console.log('nav ori: same down and up point');
    return (new ROSLIB.Quaternion());
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
  this.goalList.splice(0, this.goalList.length);
  // this.rootObject.emit('navigationUpd');
  // this.clear();         // clears markers
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
          this.mouseDown = true;

          var poi = event3D.intersection.point; 
          //console.log('nav: mouseDOWN');

          // ADD MARKER ON MOUSEDOWN, then update its orientation upon mouseup
          this.mouseDownPos = new ROSLIB.Vector3({x: poi.x, y: poi.y, z: 0});
          this.latestMarker = this.addPoseMarker(this.mouseDownPos, undefined, this.intermediateColor);
          
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
          
          // create a new pose the replace the old pose
          var updatedROSPose = new ROSLIB.Pose({
            position :    this.mouseDownPos,
            orientation : orientation,          // just give the default orientation
          });

          // delete the temporary marker
          this.remove(this.latestMarker);
          this.latestMarker = null;

          // update goalList, this will auto create markers
          this.pushToGoalList(updatedROSPose);

          this.mouseDownPos = null;       // reset only after updating marker orientation
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


ROS3D.Navigator_MW.prototype.highlightNodeAtIndex = function(index){
  var nodeMarkerObj = this.nodeMarkerList[index];
  if(nodeMarkerObj){
    try{
      var nodeMarker = nodeMarkerObj['node'];
      nodeMarker.setColor(this.highlightColor);
      this.rootObject.emit('change');
    } catch(err){}
  }
}


ROS3D.Navigator_MW.prototype.unhighlightNodeAtIndex = function(index){
  var nodeMarkerObj = this.nodeMarkerList[index];
  if(nodeMarkerObj){
    try{
      var nodeMarker = nodeMarkerObj['node'];
      nodeMarker.setColor(this.color);
      this.rootObject.emit('change');
    } catch(err){}
  }
}

ROS3D.Navigator_MW.prototype.unhighlightAllNodes = function(){
  Object.values(this.nodeMarkerList).forEach(nodeMarkerObj =>{
    try{
      var nodeMarker = nodeMarkerObj['node'];
      nodeMarker.setColor(this.color);
    } catch(err){}
  })
  this.rootObject.emit('change');
}


ROS3D.Navigator_MW.prototype.moveNodeFromIndexTo = function(fromIndex=-1, toIndex=-1, count=1){
  // count is the number of elements to move, starting from the [fromIndex]
  if( (fromIndex >= 0) && (fromIndex < this.goalList.length) && (toIndex >= 0) && (toIndex < this.goalList.length) && (count >= 0)){
    const elemsArrayToMove = this.goalList.splice(fromIndex, count);    // get element to be moved, delete it from goalList

    this.goalList.splice(toIndex, 0, ...elemsArrayToMove);
    return true;
  }
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


  