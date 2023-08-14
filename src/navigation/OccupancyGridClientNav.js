/**
 * @fileOverview
 * @author Randel Capati - randelmc21@gmail
 */

/**
 * An extension of OccupancyGridClient, with a Navigator.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the marker
 *  * 'navigationUpd' - Navigation object internally updated/changed  (eg: goal list changed). Marker may not have changed.
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
ROS3D.OccupancyGridClientNav = function(options) {
  ROS3D.OccupancyGridClient.call(this, options);
  this.viewer = options.viewer || null;
  this.navType = options.navType || 'normal';
  
  this.navOptions = options.navOptions || {};
  
  // set up navigator and its encapsulating sceneNode
  this.navSceneNode = null;       // just place holders, so that we know these vars exists
  this.navigator = null;  
  this.setupNavigator();          // properly setup the navigator
  
};
ROS3D.OccupancyGridClientNav.prototype.__proto__ = ROS3D.OccupancyGridClient.prototype;

ROS3D.OccupancyGridClientNav.prototype.setupNavigator = function(){
  if (this.tfClient){     // tfclient is required for the navigator
    // Check what type of navigator

    var navArgs = { ros: this.ros,
                    tfClient: this.tfClient,
                    rootObject: this,
                    navigatorFrameID: this.tfClient.fixedFrame,      // this should be the same frame as the FIXED FRAME (from tfClient), instead of occupancyGrid frame!!!
                    navOptions: this.navOptions,};
    switch(this.navType){
      case 'waypoints':
      case 'waypoint':
      case 'wp':
        this.navigator = new ROS3D.Navigator_MW(navArgs);
        break;
      case 'navigator':
      case 'normal':
      default:
        this.navigator = new ROS3D.Navigator(navArgs);
    }



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
};


// Override OccupancyGridClient.processMessage
ROS3D.OccupancyGridClientNav.prototype.processMessage = function(message){
  if (this.currentGrid) {                       // current grid is not empty, so this is not the 1st run we process the msg
    // console.log('FULL MAP UPDATE');
    if(this.rosTopic.subscribeId){
      this.currentGrid.updateMap(message);
    }
    
  } else{                                       // we don't have a grid yet, this is our 1st run
    console.log('INITIALIZING grid map');
    // create a new grid, this new grid needs the navigator to delegete its event processing to the navigator.
    var newGrid = new ROS3D.OccupancyGridNav({
      message : message,
      color : this.color,
      opacity : this.opacity,
      navigator: this.navigator,
    });

    // check if we care about the scene
    if (this.tfClient) {
      // console.log('have tf');
      try{
        this.currentGrid.dispose();
      } catch(err) {
        // pass
      }
      this.currentGrid = newGrid;
      // Create a sceneNode container for our map if we don't have one yet (assume 1st run so we don't have it)
      if (this.sceneNode === null) {            // create a sceneNode for the occupancyGrid if we dont have one yet
        this.sceneNode = new ROS3D.SceneNode({
          frameID : message.header.frame_id,
          tfClient : this.tfClient,
          object : newGrid,
          pose : this.offsetPose
        });
        this.sceneNode.name = 'SceneNode_map';
        this.rootObject.add(this.sceneNode);        // Add the sceneNode container of our map to the viewer's scene
      } else {                                  // if we have a sceneNode
        this.sceneNode.add(this.currentGrid);       // NOT SURE OF THIS ELSE BLOCK!!!
      }
    } else {                                        // NOT SURE OF THIS ELSE BLOCK!!!
      this.sceneNode = this.currentGrid = newGrid;
      this.rootObject.add(this.currentGrid);
    }

    if (this.viewer){
      // add sceneNode to viewer.selectableObjects
      this.viewer.addObject(this.sceneNode, true);
    }
  }

  this.emit('change');

  // check if we should unsubscribe
  if (!this.continuous) {
    // this.rosTopic.unsubscribe(this.processMessage);
    this.unsubscribe();
  }
};