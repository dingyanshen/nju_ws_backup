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

class getRightFrontDistRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getRightFrontDistRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getRightFrontDistRequest
    let len;
    let data = new getRightFrontDistRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getRightFrontDistRequest';
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
    const resolved = new getRightFrontDistRequest(null);
    return resolved;
    }
};

class getRightFrontDistResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.rightFrontDist = null;
    }
    else {
      if (initObj.hasOwnProperty('rightFrontDist')) {
        this.rightFrontDist = initObj.rightFrontDist
      }
      else {
        this.rightFrontDist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getRightFrontDistResponse
    // Serialize message field [rightFrontDist]
    bufferOffset = _serializer.float32(obj.rightFrontDist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getRightFrontDistResponse
    let len;
    let data = new getRightFrontDistResponse(null);
    // Deserialize message field [rightFrontDist]
    data.rightFrontDist = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_control/getRightFrontDistResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b33671b8766d8b0db5bcef50e936b90c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 rightFrontDist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getRightFrontDistResponse(null);
    if (msg.rightFrontDist !== undefined) {
      resolved.rightFrontDist = msg.rightFrontDist;
    }
    else {
      resolved.rightFrontDist = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: getRightFrontDistRequest,
  Response: getRightFrontDistResponse,
  md5sum() { return 'b33671b8766d8b0db5bcef50e936b90c'; },
  datatype() { return 'motion_control/getRightFrontDist'; }
};
