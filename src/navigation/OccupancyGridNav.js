/**
 * @fileOverview
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An OccupancyGridNav can convert a ROS occupancy grid message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * message - the occupancy grid message
 *   * color (optional) - color of the visualized grid
 *   * opacity (optional) - opacity of the visualized grid (0.0 == fully transparent, 1.0 == opaque)
 */
ROS3D.OccupancyGridNav = function(options) {
  ROS3D.OccupancyGrid.call(this, options);
  this.handler = options.handler || null;
  this.excludeFromHighlight = true;           // RANDEL: this will exclude this mesh from ROS3D.Highlighter

  var eventNames = [ 'contextmenu', 'click', 'dblclick', 'mouseout', 'mousedown', 'mouseup',
      'mousemove', 'mousewheel', 'DOMMouseScroll', 'touchstart', 'touchend', 'touchcancel',
      'touchleave', 'touchmove', 'mouseover' ];     // mouseover needs to be here because of ROS3D.MouseHandler
  
  
  if (this.handler){
    for (var i=0; i < eventNames.length; i++){
      // Bind all mouse events to the event handler
      this.addEventListener(eventNames[i], this.handler.mouseEventHandler);
    };

    // this.addEventListener('mouseover', this.handler.onMouseOver.bind(this));
    // this.addEventListener('dblclick', this.handler.onMouseDblClick);
    // this.addEventListener('mousedown', this.handler.onMouseDown);
  }
  
  // this.addEventListener('mouseover', this.onMouseOver.bind(this));
  
};


ROS3D.OccupancyGridNav.prototype.__proto__ = ROS3D.OccupancyGrid.prototype;

// for testing only RANDEL
// ROS3D.OccupancyGridNav.prototype.onMouseOver = function(e){
//   console.log(e);
// }
