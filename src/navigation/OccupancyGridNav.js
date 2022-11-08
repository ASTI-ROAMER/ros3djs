/**
 * @fileOverview
 * @author Randel Capati - randelmc21@gmail
 */

/**
 * An OccupancyGridNav can convert a ROS occupancy grid message into a THREE object.
 * This is an extension of OccupancyGrid, with an additional click handler [navigator].
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * options - same options as OccupancyGrid, REFER TO IT for options
 *   * navigator (optional) - a ROS3D.Navigator object, this makes the robot move when you click on the map.
 */
ROS3D.OccupancyGridNav = function(options) {
  ROS3D.OccupancyGrid.call(this, options);
  // var message = options.message;              // the ros OccupancyGrid message that we need to construct the mesh
  this.navigator = options.navigator || null;
  this.excludeFromHighlight = true;           // RANDEL: this will exclude this mesh from ROS3D.Highlighter


  var eventNames = [ 'contextmenu', 'click', 'dblclick', 'mouseout', 'mousedown', 'mouseup',
      'mousemove', 'mousewheel', 'DOMMouseScroll', 'touchstart', 'touchend', 'touchcancel',
      'touchleave', 'touchmove', 'mouseover' ];     // mouseover needs to be here because of ROS3D.MouseHandler
  
  if (this.navigator){
    // Set the navigator's marker frame ID to be same as this Occupancy Grid
    // this.navigator.markerFrameID = message.header.frame_id;

    // Bind all mouse events to the event handler (Navigator), if it exists.
    for (var i=0; i < eventNames.length; i++){
      this.addEventListener(eventNames[i], this.navigator.mouseEventHandler);
    };
  }
};


ROS3D.OccupancyGridNav.prototype.__proto__ = ROS3D.OccupancyGrid.prototype;

