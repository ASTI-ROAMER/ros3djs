/**
 * @fileOverview
 * @author Randel Capati - randelmc21@gmail.com
 */

/**
 * A NodePose is a THREE object that can be used to display a Node (plain sphere OR sphere+cone) (for position marking).
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * origin (optional) - the origin of the NodePose
 *   * direction (optional) - the direction vector of the Node
 *   * radius (optional) - the radius of the NodePose
 *   * arrowHeadHeight (optional) - length of the arrow WRT the center of the center of sphere
 *                                - SHOULD be higher than radius!
 *   * withArrowHead (optional) - defaults to true, if set to false it will only be a sphere
 *   * material (optional) - the material to use for this Node
 */
ROS3D.NodePose = function(options) {
  options = options || {};
  var origin = options.origin || new THREE.Vector3(0, 0, 0);
  var direction = options.direction || new THREE.Vector3(1, 0, 0);
  var radius = options.radius || 0.1;
  var arrowHeadHeight = options.arrowHeadHeight || 0.2;       // WRT center of sphere
  var material = options.material || new THREE.MeshBasicMaterial();
  var withArrowHead = options.arrowhead || true;              // teardrop shape when with arrowhead

  // create and merge geometry
  var geometry = new THREE.SphereGeometry(radius, 64, 32);
  var m = new THREE.Matrix4();
  m.setPosition(new THREE.Vector3(0, 0, 0));
  geometry.applyMatrix4(m);

  if (withArrowHead && (arrowHeadHeight > radius)){
    // calculate parameters for cone head
    var theta = Math.PI/2 - Math.acos(radius/arrowHeadHeight);
    var arrowRadius = radius * Math.cos(theta);
    var d = radius * Math.sin(theta);         // offset of base WRT center of sphere
    var h = arrowHeadHeight - d;              // cone height
    var mesh_h_offset = (h * 0.5) + d;        // since cone origin is not on the base, calculate correct y offset
    // create the head
    var coneGeometry = new THREE.CylinderGeometry(0, arrowRadius, h, 64, 1);
    m.setPosition(new THREE.Vector3(0, mesh_h_offset, 0));
    coneGeometry.applyMatrix4(m);

    // put the arrow together
    geometry.merge(coneGeometry);
  }

  THREE.Mesh.call(this, geometry, material);

  this.position.copy(origin);
  this.setDirection(direction);
};
ROS3D.NodePose.prototype.__proto__ = THREE.Mesh.prototype;

/**
 * Set the direction of this NodePose to that of the given vector.
 *
 * @param direction - the direction to set this NodePose
 */
ROS3D.NodePose.prototype.setDirection = function(direction) {
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
 * Set this sphere to be the given radius.
 *
 * @param radius - the new radius of the sphere
 */
ROS3D.NodePose.prototype.setRadius = function(radius) {
  this.scale.set(radius, radius, radius);
};

/**
 * Set the color of this sphere to the given hex value.
 *
 * @param hex - the hex value of the color to use
 */
ROS3D.NodePose.prototype.setColor = function(hex) {
  this.material.color.setHex(hex);
};

/*
 * Free memory of elements in this marker.
 */
ROS3D.NodePose.prototype.dispose = function() {
  if (this.geometry !== undefined) {
      this.geometry.dispose();
  }
  if (this.material !== undefined) {
      this.material.dispose();
  }
};
