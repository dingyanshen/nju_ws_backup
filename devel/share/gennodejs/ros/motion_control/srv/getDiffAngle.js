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

class getDiffAngleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getDiffAngleRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getDiffAngleRequest
    let len;
    let data = new getDiffAngleRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getDiffAngleRequest';
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
    const resolved = new getDiffAngleRequest(null);
    return resolved;
    }
};

class getDiffAngleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.diffAngle = null;
    }
    else {
      if (initObj.hasOwnProperty('diffAngle')) {
        this.diffAngle = initObj.diffAngle
      }
      else {
        this.diffAngle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getDiffAngleResponse
    // Serialize message field [diffAngle]
    bufferOffset = _serializer.float32(obj.diffAngle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getDiffAngleResponse
    let len;
    let data = new getDiffAngleResponse(null);
    // Deserialize message field [diffAngle]
    data.diffAngle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getDiffAngleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39a7032f716c41fca7c3cc6c27125bc3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 diffAngle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getDiffAngleResponse(null);
    if (msg.diffAngle !== undefined) {
      resolved.diffAngle = msg.diffAngle;
    }
    else {
      resolved.diffAngle = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: getDiffAngleRequest,
  Response: getDiffAngleResponse,
  md5sum() { return '39a7032f716c41fca7c3cc6c27125bc3'; },
  datatype() { return 'motion_control/getDiffAngle'; }
};
