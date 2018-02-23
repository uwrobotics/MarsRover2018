// Auto-generated. Do not edit!

// (in-package occupancy_grid.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let OccupancyGridHeader = require('./OccupancyGridHeader.js');
let GridDataDimension = require('./GridDataDimension.js');

//-----------------------------------------------------------

class OccupancyGrid {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.dataDimension = null;
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new OccupancyGridHeader();
      }
      if (initObj.hasOwnProperty('dataDimension')) {
        this.dataDimension = initObj.dataDimension
      }
      else {
        this.dataDimension = [];
      }
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OccupancyGrid
    // Serialize message field [header]
    bufferOffset = OccupancyGridHeader.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [dataDimension]
    // Serialize the length for message field [dataDimension]
    bufferOffset = _serializer.uint32(obj.dataDimension.length, buffer, bufferOffset);
    obj.dataDimension.forEach((val) => {
      bufferOffset = GridDataDimension.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [data]
    bufferOffset = _arraySerializer.float32(obj.data, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OccupancyGrid
    let len;
    let data = new OccupancyGrid(null);
    // Deserialize message field [header]
    data.header = OccupancyGridHeader.deserialize(buffer, bufferOffset);
    // Deserialize message field [dataDimension]
    // Deserialize array length for message field [dataDimension]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.dataDimension = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.dataDimension[i] = GridDataDimension.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [data]
    data.data = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.dataDimension.forEach((val) => {
      length += GridDataDimension.getMessageSize(val);
    });
    length += 4 * object.data.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'occupancy_grid/OccupancyGrid';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ce7444edcba848b954358db865f7fab7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    OccupancyGridHeader header
    
    GridDataDimension[] dataDimension
    float32[] data
    
    ================================================================================
    MSG: occupancy_grid/OccupancyGridHeader
    #Camera Info
    uint32 cameraZMax
    uint32 cameraXMax
    float32 cameraYOffset
    
    #Occupancy Grid Info
    float32 gridResolution
    uint32 gridCameraZ
    uint32 gridCameraX
    
    
    ================================================================================
    MSG: occupancy_grid/GridDataDimension
    string label
    uint32 size
    uint32 stride
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OccupancyGrid(null);
    if (msg.header !== undefined) {
      resolved.header = OccupancyGridHeader.Resolve(msg.header)
    }
    else {
      resolved.header = new OccupancyGridHeader()
    }

    if (msg.dataDimension !== undefined) {
      resolved.dataDimension = new Array(msg.dataDimension.length);
      for (let i = 0; i < resolved.dataDimension.length; ++i) {
        resolved.dataDimension[i] = GridDataDimension.Resolve(msg.dataDimension[i]);
      }
    }
    else {
      resolved.dataDimension = []
    }

    if (msg.data !== undefined) {
      resolved.data = msg.data;
    }
    else {
      resolved.data = []
    }

    return resolved;
    }
};

module.exports = OccupancyGrid;
