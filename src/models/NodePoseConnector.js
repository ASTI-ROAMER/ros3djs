/**
 * @fileOverview
 * @author Randel Capati - randelmc21@gmail.com
 */

/**
 * A NodePoseConnector is a THREE object that can be used to display a straight link between two posistions (xyz).
 * It consists of an arrow head in the middle of p1 and p2, and shafts coming out of p1 and p2 as illustrated below:
 * p1 ------>------ p2
 * 
 * If distance between p1 and p2 is too short, this will not generate a geometry, thus is NOT RENDERABLE.
 * If combined shaft lengths is less than the arrowhead length, only arrow head will be shown.
 *
 * @constructor
 * @param options - object with following keys:
 *    
 *   * p1 - the given point1
 *   * p2 - the given point2
 *   * headLength (optional) - the head length of the arrow
 *   * headRadius (optional) - the head radius of the arrow
 *   * shaftRadius (optional) - the shaft radius of the shaft
 *   * offsetLength (optional) - distance from the given point on which the geometry will start, 
 *                                set to 0 if you want the shaft to start on exactly p1/p2
 *   * material (optional) - the material to use for this arrow
 */
ROS3D.NodePoseConnector = function(options) {
  options = options || {};

  var headLength = options.headLength || 0.1;
  var headRadius = options.headRadius || 0.05;
  var shaftRadius = options.shaftRadius || 0.01;
  var offsetLength = options.offsetLength || 0.22;     // length from point to the start of the actual mesh

  var material = options.material || new THREE.MeshBasicMaterial();

  var minP2PLength = headLength + (2 * offsetLength);

  // If shaft radius is smaller than arrow head radius, adjust shaft radius
  if (shaftRadius > headRadius){
    shaftRadius = headRadius;
  }

  this.p1 = options.p1;
  this.p2 = options.p2;
  // if (!this.p1.isVector3() && !this.p2.isVector3())


  var direction = new THREE.Vector3(0, 0, 0);
  direction.subVectors(this.p2, this.p1).normalize();
  

  // get distance from p1 to p2, point-to-point length
  this.p2pLength = this.p1.distanceTo(this.p2);

  // there are 2 shafts. shaftLength is the length of 1 shaft
  var shaftLength = ((this.p2pLength - headLength) * 0.5) - offsetLength;

  // if the distance between the 2 given points is less than the minimum (2*offset + arrowhead length), dont create mesh.
  if (this.p2pLength < minP2PLength){
    return;
  }


  // Create head on the middle of the connector
  var m = new THREE.Matrix4();
  var geometry = new THREE.CylinderGeometry(0, headRadius, headLength, 24, 1);
  m.setPosition(new THREE.Vector3(0, this.p2pLength * 0.5, 0));
  geometry.applyMatrix4(m);
  
  // Only create shaft when a combined shaft length is at least the same length as the arrowhead length
  if (shaftLength * 2 > headLength){
    var shaftGeometry1 = new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 12, 1);
    var shaftGeometry2 = shaftGeometry1.clone();
    m.setPosition(new THREE.Vector3(0, (shaftLength * 0.5) + offsetLength, 0));
    shaftGeometry1.applyMatrix4(m);
    m.setPosition(new THREE.Vector3(0, (shaftLength * 1.5) + offsetLength + headLength, 0));
    shaftGeometry2.applyMatrix4(m);
    
    // put the connector together
    geometry.merge(shaftGeometry1);
    geometry.merge(shaftGeometry2);
  }

  

  THREE.Mesh.call(this, geometry, material);

  this.position.copy(new THREE.Vector3(this.p1.x, this.p1.y, this.p1.z));
  this.setDirection(direction);
};
ROS3D.NodePoseConnector.prototype.__proto__ = THREE.Mesh.prototype;


// ROS3D.NodePoseConnector.prototype.calcDistance = function(p1=this.p1, p2=this.p2){
//   var dx = p2.x - p1.x;
//   var dy = p2.y - p1.y;
//   var dz = p2.z - p1.z;
//   return Math.sqrt(dx*dx + dy*dy + dz*dz);
// };


/**
 * Set the direction of this arrow to that of the given vector.
 *
 * @param direction - the direction to set this arrow
 */
ROS3D.NodePoseConnector.prototype.setDirection = function(direction) {
  var axis = new THREE.Vector3();
  if(direction.x === 0 && direction.z === 0){
    axis.set(1, 0, 0);
  } else {
    axis.set(0, 1, 0).cross(direction);
  }
  var radians = Math.acos(new THREE.Vector3(0, 1, 0).dot(direction.clone().normalize()));
  this.matrix = new THREE.Matrix4().makeRotationAxis(axis.normalize(), radians);
  this.rotation.setFromRotationMatrix(this.matrix, this.rotation.order);
};

/**
 * Set this arrow to be the given length.
 *
 * @param length - the new length of the arrow
 */
ROS3D.NodePoseConnector.prototype.setLength = function(length) {
  this.scale.set(length, length, length);
};

/**
 * Set the color of this arrow to the given hex value.
 *
 * @param hex - the hex value of the color to use
 */
ROS3D.NodePoseConnector.prototype.setColor = function(hex) {
  this.material.color.setHex(hex);
};

/*
 * Free memory of elements in this marker.
 */
ROS3D.NodePoseConnector.prototype.dispose = function() {
  if (this.geometry !== undefined) {
      this.geometry.dispose();
  }
  if (this.material !== undefined) {
      this.material.dispose();
  }
};
