/**
 * @fileOverview
 * @author Russell Toris - rctoris@wpi.edu
 */

/**
 * An OccupancyGrid can convert a ROS occupancy grid message into a THREE object.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *   * message - the occupancy grid message
 *   * color (optional) - color of the visualized grid
 *   * opacity (optional) - opacity of the visualized grid (0.0 == fully transparent, 1.0 == opaque)
 */
ROS3D.OccupancyGrid = function(options) {
  options = options || {};
  var message = options.message;
  var opacity = options.opacity || 0.7;
  var color = options.color || {r:255,g:255,b:255,a:255};
  var transform = options.transform || this.defaultTransform;   // Transforms cell (in grid) value to RGB

  // just create dummy
  var geom = new THREE.PlaneBufferGeometry(1, 1);
  var imageData = new Uint8Array(1 * 1 * 4);    // create 1x1 pixel image, dummy only
  var texture = new THREE.DataTexture(imageData, 1, 1, THREE.RGBAFormat);

  var material = new THREE.MeshBasicMaterial({
    map : texture,
    transparent : opacity < 1.0,
    opacity : opacity
  });
  material.side = THREE.DoubleSide;

  // create the mesh
  THREE.Mesh.call(this, geom, material);

  // update the texture (after the the super call and this are accessible)
  this.mapImageData = null;               // Edit this if you want the update has the same dimensions
  this.mapOrigin = null;
  this.mapWidth = null;
  this.mapHeight = null;

  this.color = color;
  this.material = material;
  this.texture = texture;
  this.mapColorTransform = transform;     // Transforms cell (in grid) value to RGB

  this.excludeFromHighlight = true;       // RANDEL: this will exclude this mesh from ROS3D.Highlighter

  // update map from message
  this.updateMap(message);

};


// Create update the texture and (geometry) of the grid from the msg
// This will either create a new texture or update the existing texture
// This is only for full map topics (nav_msgs/OccupancyGrid)
ROS3D.OccupancyGrid.prototype.updateMap = function(message, transform=this.mapColorTransform){
  var data = message.data;

  // create the geometry
  var info = message.info;
  var origin = info.origin;
  var width = info.width;
  var height = info.height;
  
  if(this.isNewGridSameAsCur(origin, width, height)){
    // console.log('FULL - SAME MAP UPDATE');
    // if the new map has the same dimensions, just update its imageData
    if(!this.mapImageData){
      console.error('Expecting to have mapImageData since we are updating it, but it is empty');
    }

    this.buildImageData(this.mapImageData, data, this.mapWidth, this.mapHeight);
    this.texture.needsUpdate = true;


  } else {
    // console.log('FULL - DIFFERENT MAP UPDATE');
    // update texture and geometry if new map has different dimensions
    // console.log('CREATE new texture for New map');

    this.mapOrigin = origin;
    this.mapWidth = width;
    this.mapHeight = height;

    this.texture.dispose();
    // RANDEL: hard to update geometry, so just create a new geometry
    var geom = new THREE.PlaneBufferGeometry(width, height);
    this.geometry = geom;
    this.quaternion.copy(new THREE.Quaternion(
      origin.orientation.x,
      origin.orientation.y,
      origin.orientation.z,
      origin.orientation.w
    ));
    this.position.x = (width * info.resolution) / 2 + origin.position.x;
    this.position.y = (height * info.resolution) / 2 + origin.position.y;
    this.position.z = origin.position.z;
    this.scale.x = info.resolution;
    this.scale.y = info.resolution;

    // create the color material
    var imageData = new Uint8Array(width * height * 4);
    this.buildImageData(imageData, data, this.mapWidth, this.mapHeight);

    this.mapImageData = imageData;
    // console.log(this.mapImageData);

    var texture = new THREE.DataTexture(imageData, width, height, THREE.RGBAFormat);
    // texture.flipY = true;
    texture.minFilter = THREE.NearestFilter;
    texture.magFilter = THREE.NearestFilter;
    texture.needsUpdate = true;

    this.texture = texture;
    this.material.map = texture;
  }
};

// This is only for partial update map topics (map_msgs/OccupancyGridUpdate, topics with suffix of "_updates")
ROS3D.OccupancyGrid.prototype.updatePartialMap = function(message, transform=this.mapColorTransform){
  //https://docs.ros.org/en/jade/api/rviz/html/c++/map__display_8cpp_source.html#l00492
  // Reject updates which have any out-of-bounds data.
  if(message.x < 0 || message.y < 0 ||
    this.mapWidth < message.x + message.width ||
    this.mapHeight < message.y + message.height){
      console.error('Update area outside of original map area.');
      return;
  }

  if(!this.mapImageData){
    console.error('Expecting to have mapImageData since we are updating it, but it is empty. Aborting update.');
    return;
  }

  var height = message.height;
  var width = message.width;
  var ux = message.x;
  var uy = message.y;

  this.buildImageData(this.mapImageData, message.data, message.width, message.height, message.x, message.y);
  this.texture.needsUpdate = true;


};

ROS3D.OccupancyGrid.prototype.buildImageData = function(imageData, data, width, height, x=0, y=0){
  // THIS ASSUMES that the data from the update is WITHIN THE BOUND OF imageData!!!!
  // Iterate over writable indeces of imageData, specified the starting positon (x,y) and the dimension of the data
  // - row and col are in imageData coordinates
  for ( var row = y; row < height+y; row++) {
    for ( var col = x; col < width+x; col++) {
      // Determine the index to be used for extractring values from data (data[0] corresponds to (x,y) data)
      var dataRow = row - y;
      var mapI = col + (dataRow * width);
      // val is a grayscale value, converted from uint8data (data)
      var val = this.transformMapData(this.getValue(mapI, dataRow, col, data), this.mapColorTransform);

      // determine the color, color is an array of 4 values (RGBA)
      var color = this.getColor(mapI, dataRow, col, val);

      // determine the index into the image data array
      var i = (col + (row * width)) * 4;

      // copy the color
      imageData.set(color, i);
    }
  }

};


ROS3D.OccupancyGrid.prototype.isNewGridSameAsCur = function(origin, width, height){
  if(this.mapOrigin !== null){
    if(this.mapOrigin.position.x === origin.position.x && 
        this.mapOrigin.position.y === origin.position.y &&
        this.mapOrigin.position.z === origin.position.z &&
        // this.mapOrigin.orientation.x === origin.orientation.x && 
        // this.mapOrigin.orientation.y === origin.orientation.y && 
        // this.mapOrigin.orientation.z === origin.orientation.z && 
        // this.mapOrigin.orientation.w === origin.orientation.w && 
        this.mapWidth === width && 
        this.mapHeight === height){
      return true;
    }
  } 
  return false;
};


ROS3D.OccupancyGrid.prototype.dispose = function() {
  this.material.dispose();
  this.texture.dispose();
};

/**
 * Returns the value for a given grid cell
 * @param {int} index the current index of the cell
 * @param {int} row the row of the cell
 * @param {int} col the column of the cell
 * @param {object} data the data buffer
 */
ROS3D.OccupancyGrid.prototype.getValue = function(index, row, col, data) {
  return data[index];
};

/**
 * Returns a color value given parameters of the position in the grid; the default implementation
 * scales the default color value by the grid value. Subclasses can extend this functionality
 * (e.g. lookup a color in a color map).
 * @param {int} index the current index of the cell
 * @param {int} row the row of the cell
 * @param {int} col the column of the cell
 * @param {float} value the value of the cell
 * @returns r,g,b,a array of values from 0 to 255 representing the color values for each channel
 */
ROS3D.OccupancyGrid.prototype.getColor = function(index, row, col, value) {
  return [
    (value * this.color.r) / 255,
    (value * this.color.g) / 255,
    (value * this.color.b) / 255,
    255
  ];
};


/**
 * RANDEL!!!
 * Transforms Occupancy map value [0, 100], to a grayscale map by using the transform <transform>.
 * @param {int} value occupancy value
 * @param {function} transform the function used for transformation
 * @returns r,g,b,a array of values from 0 to 255 representing the color values for each channel
 */
 ROS3D.OccupancyGrid.prototype.transformMapData = function(value, transform=this.defaultTransform) {
  return transform(value);
};

/**
 * RANDEL!!!
 * This is the legacy transform.
 * Transforms occupancy value to a grayscale value.
 * A value of 100 (100% occupied) is black, a value of 0 (certainly UNoccupied) is white.
 * Other values are set to 127 (gray).
 * @param {int} value the value of the cell. Value should be [0, 100]
 * @returns grayscale value from 0 to 255 representing the occupancy value.
 */
 ROS3D.OccupancyGrid.prototype.defaultTransform = function(value) {
  var val_trans = value;
  if (value === 100) {
    val_trans = 0;
  } else if (value === 0) {
    val_trans = 255;
  } else {
    val_trans = 127;
  }
  
  return val_trans;
};


ROS3D.OccupancyGrid.prototype.__proto__ = THREE.Mesh.prototype;
