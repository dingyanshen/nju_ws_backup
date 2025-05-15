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

class ThrowServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pos_z = null;
      this.mailbox_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('pos_z')) {
        this.pos_z = initObj.pos_z
      }
      else {
        this.pos_z = false;
      }
      if (initObj.hasOwnProperty('mailbox_pos')) {
        this.mailbox_pos = initObj.mailbox_pos
      }
      else {
        this.mailbox_pos = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ThrowServiceRequest
    // Serialize message field [pos_z]
    bufferOffset = _serializer.bool(obj.pos_z, buffer, bufferOffset);
    // Serialize message field [mailbox_pos]
    bufferOffset = _serializer.bool(obj.mailbox_pos, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ThrowServiceRequest
    let len;
    let data = new ThrowServiceRequest(null);
    // Deserialize message field [pos_z]
    data.pos_z = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mailbox_pos]
    data.mailbox_pos = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dobot/ThrowServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb345315c2037379bea93776fcff6049';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool pos_z
    bool mailbox_pos
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ThrowServiceRequest(null);
    if (msg.pos_z !== undefined) {
      resolved.pos_z = msg.pos_z;
    }
    else {
      resolved.pos_z = false
    }

    if (msg.mailbox_pos !== undefined) {
      resolved.mailbox_pos = msg.mailbox_pos;
    }
    else {
      resolved.mailbox_pos = false
    }

    return resolved;
    }
};

class ThrowServiceResponse {
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
    // Serializes a message object of type ThrowServiceResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ThrowServiceResponse
    let len;
    let data = new ThrowServiceResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dobot/ThrowServiceResponse';
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
    const resolved = new ThrowServiceResponse(null);
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
  Request: ThrowServiceRequest,
  Response: ThrowServiceResponse,
  md5sum() { return 'b5ca28a4c733f2992478f530e3af3cbb'; },
  datatype() { return 'dobot/ThrowService'; }
};
