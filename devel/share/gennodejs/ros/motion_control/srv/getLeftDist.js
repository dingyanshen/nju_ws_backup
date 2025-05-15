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

class getLeftDistRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getLeftDistRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getLeftDistRequest
    let len;
    let data = new getLeftDistRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getLeftDistRequest';
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
    const resolved = new getLeftDistRequest(null);
    return resolved;
    }
};

class getLeftDistResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.leftDist = null;
    }
    else {
      if (initObj.hasOwnProperty('leftDist')) {
        this.leftDist = initObj.leftDist
      }
      else {
        this.leftDist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getLeftDistResponse
    // Serialize message field [leftDist]
    bufferOffset = _serializer.float32(obj.leftDist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getLeftDistResponse
    let len;
    let data = new getLeftDistResponse(null);
    // Deserialize message field [leftDist]
    data.leftDist = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getLeftDistResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e0d27729a997215618b53389df19443';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 leftDist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getLeftDistResponse(null);
    if (msg.leftDist !== undefined) {
      resolved.leftDist = msg.leftDist;
    }
    else {
      resolved.leftDist = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: getLeftDistRequest,
  Response: getLeftDistResponse,
  md5sum() { return '4e0d27729a997215618b53389df19443'; },
  datatype() { return 'motion_control/getLeftDist'; }
};
