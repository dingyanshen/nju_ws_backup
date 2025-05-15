// Auto-generated. Do not edit!

// (in-package dobot.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GraspServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.shelf_z = null;
      this.pos_z = null;
      this.error_x = null;
      this.error_y = null;
    }
    else {
      if (initObj.hasOwnProperty('shelf_z')) {
        this.shelf_z = initObj.shelf_z
      }
      else {
        this.shelf_z = false;
      }
      if (initObj.hasOwnProperty('pos_z')) {
        this.pos_z = initObj.pos_z
      }
      else {
        this.pos_z = false;
      }
      if (initObj.hasOwnProperty('error_x')) {
        this.error_x = initObj.error_x
      }
      else {
        this.error_x = 0.0;
      }
      if (initObj.hasOwnProperty('error_y')) {
        this.error_y = initObj.error_y
      }
      else {
        this.error_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspServiceRequest
    // Serialize message field [shelf_z]
    bufferOffset = _serializer.bool(obj.shelf_z, buffer, bufferOffset);
    // Serialize message field [pos_z]
    bufferOffset = _serializer.bool(obj.pos_z, buffer, bufferOffset);
    // Serialize message field [error_x]
    bufferOffset = _serializer.float32(obj.error_x, buffer, bufferOffset);
    // Serialize message field [error_y]
    bufferOffset = _serializer.float32(obj.error_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspServiceRequest
    let len;
    let data = new GraspServiceRequest(null);
    // Deserialize message field [shelf_z]
    data.shelf_z = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pos_z]
    data.pos_z = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error_x]
    data.error_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [error_y]
    data.error_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dobot/GraspServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba764222789330d239513964a79bae26';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool shelf_z
    bool pos_z
    float32 error_x
    float32 error_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspServiceRequest(null);
    if (msg.shelf_z !== undefined) {
      resolved.shelf_z = msg.shelf_z;
    }
    else {
      resolved.shelf_z = false
    }

    if (msg.pos_z !== undefined) {
      resolved.pos_z = msg.pos_z;
    }
    else {
      resolved.pos_z = false
    }

    if (msg.error_x !== undefined) {
      resolved.error_x = msg.error_x;
    }
    else {
      resolved.error_x = 0.0
    }

    if (msg.error_y !== undefined) {
      resolved.error_y = msg.error_y;
    }
    else {
      resolved.error_y = 0.0
    }

    return resolved;
    }
};

class GraspServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspServiceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspServiceResponse
    let len;
    let data = new GraspServiceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dobot/GraspServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspServiceResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: GraspServiceRequest,
  Response: GraspServiceResponse,
  md5sum() { return '1e7660736e31416d5bfaa2f3f2cf2e32'; },
  datatype() { return 'dobot/GraspService'; }
};
