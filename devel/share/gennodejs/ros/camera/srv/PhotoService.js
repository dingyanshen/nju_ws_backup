// Auto-generated. Do not edit!

// (in-package camera.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class PhotoServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.type = null;
    }
    else {
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PhotoServiceRequest
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PhotoServiceRequest
    let len;
    let data = new PhotoServiceRequest(null);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera/PhotoServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bda37decd5e3814bcc042f341d2e60a1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 type
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PhotoServiceRequest(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    return resolved;
    }
};

class PhotoServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.error_x = null;
      this.error_y = null;
    }
    else {
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
    // Serializes a message object of type PhotoServiceResponse
    // Serialize message field [error_x]
    bufferOffset = _serializer.float32(obj.error_x, buffer, bufferOffset);
    // Serialize message field [error_y]
    bufferOffset = _serializer.float32(obj.error_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PhotoServiceResponse
    let len;
    let data = new PhotoServiceResponse(null);
    // Deserialize message field [error_x]
    data.error_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [error_y]
    data.error_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera/PhotoServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49012d3a75867c44d7c8a631f08af3cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 error_x
    float32 error_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PhotoServiceResponse(null);
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

module.exports = {
  Request: PhotoServiceRequest,
  Response: PhotoServiceResponse,
  md5sum() { return '117e6535d65959983e49d94959e9d276'; },
  datatype() { return 'camera/PhotoService'; }
};
