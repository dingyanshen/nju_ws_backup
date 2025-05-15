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

class getRightDistRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getRightDistRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getRightDistRequest
    let len;
    let data = new getRightDistRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getRightDistRequest';
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
    const resolved = new getRightDistRequest(null);
    return resolved;
    }
};

class getRightDistResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rightDist = null;
    }
    else {
      if (initObj.hasOwnProperty('rightDist')) {
        this.rightDist = initObj.rightDist
      }
      else {
        this.rightDist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getRightDistResponse
    // Serialize message field [rightDist]
    bufferOffset = _serializer.float32(obj.rightDist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getRightDistResponse
    let len;
    let data = new getRightDistResponse(null);
    // Deserialize message field [rightDist]
    data.rightDist = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getRightDistResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46d3dd3b27754b2db4e99d35f4cb12c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 rightDist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getRightDistResponse(null);
    if (msg.rightDist !== undefined) {
      resolved.rightDist = msg.rightDist;
    }
    else {
      resolved.rightDist = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: getRightDistRequest,
  Response: getRightDistResponse,
  md5sum() { return '46d3dd3b27754b2db4e99d35f4cb12c3'; },
  datatype() { return 'motion_control/getRightDist'; }
};
