/**
 * @fileOverview
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An occupancy grid client that listens to a given map topic.
 *
 * Emits the following events:
 *
 *  * 'change' - there was an update or change in the marker
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * ros - the ROSLIB.Ros connection handle
 *   * topic (optional) - the map topic to listen to
 *   * continuous (optional) - if the map should be continuously loaded (e.g., for SLAM)
 *   * tfClient (optional) - the TF client handle to use for a scene node
 *   * compression (optional) - message compression (default: 'cbor')
 *   * rootObject (optional) - the root object to add this marker to
 *   * offsetPose (optional) - offset pose of the grid visualization, e.g. for z-offset (ROSLIB.Pose type)
 *   * color (optional) - color of the visualized grid
 *   * opacity (optional) - opacity of the visualized grid (0.0 == fully transparent, 1.0 == opaque)
 */
ROS3D.OccupancyGridClientNav = function(options) {
  ROS3D.OccupancyGridClient.call(this, options);

};
ROS3D.OccupancyGridClientNav.prototype.__proto__ = ROS3D.OccupancyGridClient.prototype;
