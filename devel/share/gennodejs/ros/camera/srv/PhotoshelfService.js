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

class PhotoshelfServiceRequest {
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
    // Serializes a message object of type PhotoshelfServiceRequest
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PhotoshelfServiceRequest
    let len;
    let data = new PhotoshelfServiceRequest(null);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera/PhotoshelfServiceRequest';
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
    const resolved = new PhotoshelfServiceRequest(null);
    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    return resolved;
    }
};

class PhotoshelfServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.results = null;
      this.positions_z = null;
      this.positions_x = null;
    }
    else {
      if (initObj.hasOwnProperty('results')) {
        this.results = initObj.results
      }
      else {
        this.results = [];
      }
      if (initObj.hasOwnProperty('positions_z')) {
        this.positions_z = initObj.positions_z
      }
      else {
        this.positions_z = [];
      }
      if (initObj.hasOwnProperty('positions_x')) {
        this.positions_x = initObj.positions_x
      }
      else {
        this.positions_x = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PhotoshelfServiceResponse
    // Serialize message field [results]
    bufferOffset = _arraySerializer.int32(obj.results, buffer, bufferOffset, null);
    // Serialize message field [positions_z]
    bufferOffset = _arraySerializer.int32(obj.positions_z, buffer, bufferOffset, null);
    // Serialize message field [positions_x]
    bufferOffset = _arraySerializer.int32(obj.positions_x, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PhotoshelfServiceResponse
    let len;
    let data = new PhotoshelfServiceResponse(null);
    // Deserialize message field [results]
    data.results = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [positions_z]
    data.positions_z = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [positions_x]
    data.positions_x = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.results.length;
    length += 4 * object.positions_z.length;
    length += 4 * object.positions_x.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera/PhotoshelfServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '880c8ac3d7edb422a74a333fb36ec2cc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] results
    int32[] positions_z
    int32[] positions_x
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PhotoshelfServiceResponse(null);
    if (msg.results !== undefined) {
      resolved.results = msg.results;
    }
    else {
      resolved.results = []
    }

    if (msg.positions_z !== undefined) {
      resolved.positions_z = msg.positions_z;
    }
    else {
      resolved.positions_z = []
    }

    if (msg.positions_x !== undefined) {
      resolved.positions_x = msg.positions_x;
    }
    else {
      resolved.positions_x = []
    }

    return resolved;
    }
};

module.exports = {
  Request: PhotoshelfServiceRequest,
  Response: PhotoshelfServiceResponse,
  md5sum() { return 'b956cbec76bf70325b60df673b2d7722'; },
  datatype() { return 'camera/PhotoshelfService'; }
};
