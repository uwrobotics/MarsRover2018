// Auto-generated. Do not edit!

// (in-package occupancy_grid.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class OccupancyGridHeader {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cameraZMax = null;
      this.cameraXMax = null;
      this.cameraYOffset = null;
      this.gridResolution = null;
      this.gridCameraZ = null;
      this.gridCameraX = null;
    }
    else {
      if (initObj.hasOwnProperty('cameraZMax')) {
        this.cameraZMax = initObj.cameraZMax
      }
      else {
        this.cameraZMax = 0;
      }
      if (initObj.hasOwnProperty('cameraXMax')) {
        this.cameraXMax = initObj.cameraXMax
      }
      else {
        this.cameraXMax = 0;
      }
      if (initObj.hasOwnProperty('cameraYOffset')) {
        this.cameraYOffset = initObj.cameraYOffset
      }
      else {
        this.cameraYOffset = 0.0;
      }
      if (initObj.hasOwnProperty('gridResolution')) {
        this.gridResolution = initObj.gridResolution
      }
      else {
        this.gridResolution = 0.0;
      }
      if (initObj.hasOwnProperty('gridCameraZ')) {
        this.gridCameraZ = initObj.gridCameraZ
      }
      else {
        this.gridCameraZ = 0;
      }
      if (initObj.hasOwnProperty('gridCameraX')) {
        this.gridCameraX = initObj.gridCameraX
      }
      else {
        this.gridCameraX = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OccupancyGridHeader
    // Serialize message field [cameraZMax]
    bufferOffset = _serializer.uint32(obj.cameraZMax, buffer, bufferOffset);
    // Serialize message field [cameraXMax]
    bufferOffset = _serializer.uint32(obj.cameraXMax, buffer, bufferOffset);
    // Serialize message field [cameraYOffset]
    bufferOffset = _serializer.float32(obj.cameraYOffset, buffer, bufferOffset);
    // Serialize message field [gridResolution]
    bufferOffset = _serializer.float32(obj.gridResolution, buffer, bufferOffset);
    // Serialize message field [gridCameraZ]
    bufferOffset = _serializer.uint32(obj.gridCameraZ, buffer, bufferOffset);
    // Serialize message field [gridCameraX]
    bufferOffset = _serializer.uint32(obj.gridCameraX, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OccupancyGridHeader
    let len;
    let data = new OccupancyGridHeader(null);
    // Deserialize message field [cameraZMax]
    data.cameraZMax = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cameraXMax]
    data.cameraXMax = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cameraYOffset]
    data.cameraYOffset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gridResolution]
    data.gridResolution = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gridCameraZ]
    data.gridCameraZ = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gridCameraX]
    data.gridCameraX = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'occupancy_grid/OccupancyGridHeader';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9f8b35825f257a4716d746612ebebd87';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Camera Info
    uint32 cameraZMax
    uint32 cameraXMax
    float32 cameraYOffset
    
    #Occupancy Grid Info
    float32 gridResolution
    uint32 gridCameraZ
    uint32 gridCameraX
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OccupancyGridHeader(null);
    if (msg.cameraZMax !== undefined) {
      resolved.cameraZMax = msg.cameraZMax;
    }
    else {
      resolved.cameraZMax = 0
    }

    if (msg.cameraXMax !== undefined) {
      resolved.cameraXMax = msg.cameraXMax;
    }
    else {
      resolved.cameraXMax = 0
    }

    if (msg.cameraYOffset !== undefined) {
      resolved.cameraYOffset = msg.cameraYOffset;
    }
    else {
      resolved.cameraYOffset = 0.0
    }

    if (msg.gridResolution !== undefined) {
      resolved.gridResolution = msg.gridResolution;
    }
    else {
      resolved.gridResolution = 0.0
    }

    if (msg.gridCameraZ !== undefined) {
      resolved.gridCameraZ = msg.gridCameraZ;
    }
    else {
      resolved.gridCameraZ = 0
    }

    if (msg.gridCameraX !== undefined) {
      resolved.gridCameraX = msg.gridCameraX;
    }
    else {
      resolved.gridCameraX = 0
    }

    return resolved;
    }
};

module.exports = OccupancyGridHeader;
