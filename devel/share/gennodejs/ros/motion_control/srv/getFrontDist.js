// Auto-generated. Do not edit!

// (in-package motion_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class getFrontDistRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getFrontDistRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getFrontDistRequest
    let len;
    let data = new getFrontDistRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getFrontDistRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getFrontDistRequest(null);
    return resolved;
    }
};

class getFrontDistResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frontDist = null;
    }
    else {
      if (initObj.hasOwnProperty('frontDist')) {
        this.frontDist = initObj.frontDist
      }
      else {
        this.frontDist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getFrontDistResponse
    // Serialize message field [frontDist]
    bufferOffset = _serializer.float32(obj.frontDist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getFrontDistResponse
    let len;
    let data = new getFrontDistResponse(null);
    // Deserialize message field [frontDist]
    data.frontDist = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getFrontDistResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e75451363e2baac9b8bfb0e12a9fdd0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 frontDist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getFrontDistResponse(null);
    if (msg.frontDist !== undefined) {
      resolved.frontDist = msg.frontDist;
    }
    else {
      resolved.frontDist = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: getFrontDistRequest,
  Response: getFrontDistResponse,
  md5sum() { return '1e75451363e2baac9b8bfb0e12a9fdd0'; },
  datatype() { return 'motion_control/getFrontDist'; }
};
