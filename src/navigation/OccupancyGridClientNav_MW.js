/**
 * @fileOverview
 * @author Randel Capati - randelmc21@gmail
 */ //Mitz - FOR MANAGED WAYPOINTS 

/**
 * An extension of OccupancyGridClient, with a Navigator.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the marker
 *
 * @constructor
 * @param options - object with following keys:
 *    %%% from OccupancyGridClient %%%
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * tfClient (optional) - the TF client handle to use for a scene node
 *   * compression (optional) - message compression (default: 'cbor')
 *   * rootObject (optional) - the root object to add this marker to
 *   * offsetPose (optional) - offset pose of the grid visualization, e.g. for z-offset (ROSLIB.Pose type)
 *   * color (optional) - color of the visualized grid
 *   * opacity (optional) - opacity of the visualized grid (0.0 == fully transparent, 1.0 == opaque)
 * 
 *    %%% from extension %%%
 *   * viewer - ROS3D.Viewer, the viewer that called this
 *   * navServerName (optional) - navigation action server name, defaults to /move_base
 *   * navActionName (optional) - navigation action name, defaults to move_base_msgs/MoveBaseAction
 * 
 */
ROS3D.OccupancyGridClientNav_MW= function(options) {
  ROS3D.OccupancyGridClient.call(this, options);
  this.viewer = options.viewer || null;
  this.navServerName = options.navServerName || '/move_base';
  this.navActionName = options.navActionName || 'move_base_msgs/MoveBaseAction';
  this.navigatorInitState = options.navigatorInitState || false;
  
  // set up navigator and its encapsulating sceneNode
  this.navSceneNode = null;       // just place holders, so that we know these vars exists
  this.navigator = null;  
  this.setupNavigator();          // properly setup the navigator
  
  //mitz - poi display
  this.goalPose  = options.goalPose 
};
ROS3D.OccupancyGridClientNav_MW.prototype.__proto__ = ROS3D.OccupancyGridClient.prototype;

ROS3D.OccupancyGridClientNav_MW.prototype.setupNavigator = function(){
  if (this.tfClient){     // tfclient is required for the navigator
    // Instantiate the navigator.
    this.navigator = new ROS3D.Navigator_MW({
      ros: this.ros,
      tfClient: this.tfClient,
      rootObject: this,
      serverName: this.navServerName,
      actionName: this.navActionName,
      navigatorFrameID: this.tfClient.fixedFrame,      // this should be the same frame as the FIXED FRAME (from tfClient), instead of occupancyGrid frame!!!
      isActive: this.navigatorInitState,
      targetGoal: this.goalPose
   
    });
   
    // The Navigator goal message SHOULD be in the fixed frame BUT its marker (the arrow) SHOULD BE RENDERED IN THE MAP (OccupancyGridNav)!!!
    // Create a ROS3D.SceneNode in which the Navigator is encapsulated in.
    this.navSceneNode = new ROS3D.SceneNode({
      frameID : this.tfClient.fixedFrame,
      tfClient : this.tfClient,
      object : this.navigator,
      pose : this.offsetPose
    });

    // Add the navSceneNode to the viewer's scene (THREE.Scene)
    this.rootObject.add(this.navSceneNode);
  }
}


// Override OccupancyGridClient.processMessage
ROS3D.OccupancyGridClientNav_MW.prototype.processMessage = function(message){
  // check for an old map
  if (this.currentGrid) {
    // check if it there is a tf client
    if (this.tfClient) {
      // grid is of type ROS3D.SceneNode
      this.sceneNode.unsubscribeTf();
      this.sceneNode.remove(this.currentGrid);
    } else {
      this.rootObject.remove(this.currentGrid);
    }
    this.currentGrid.dispose();
  }

  var newGrid = new ROS3D.OccupancyGridNav({
    message : message,
    color : this.color,
    opacity : this.opacity,
    navigator: this.navigator
  });

  // check if we care about the scene
  if (this.tfClient) {
    this.currentGrid = newGrid;
    if (this.sceneNode === null) {
      this.sceneNode = new ROS3D.SceneNode({
        frameID : message.header.frame_id,
        tfClient : this.tfClient,
        object : newGrid,
        pose : this.offsetPose
      });
      // this.sceneNode.add(this.navigator);
      this.rootObject.add(this.sceneNode);
    } else {
      this.sceneNode.add(this.currentGrid);
      // this.sceneNode.add(this.navigator);
    }
  } else {
    this.sceneNode = this.currentGrid = newGrid;
    this.rootObject.add(this.currentGrid);
    // this.rootObject.add(this.navigator);
  }

  if (this.viewer){
    // add sceneNode to viewer.selectableObjects
    this.viewer.addObject(this.sceneNode, true);
  }

  this.emit('change');

  // check if we should unsubscribe
  if (!this.continuous) {
    this.rosTopic.unsubscribe(this.processMessage);
  }
};