// Auto-generated. Do not edit!

// (in-package mascot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Gain {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.k_p = null;
      this.k_d = null;
      this.k_i = null;
    }
    else {
      if (initObj.hasOwnProperty('k_p')) {
        this.k_p = initObj.k_p
      }
      else {
        this.k_p = 0.0;
      }
      if (initObj.hasOwnProperty('k_d')) {
        this.k_d = initObj.k_d
      }
      else {
        this.k_d = 0.0;
      }
      if (initObj.hasOwnProperty('k_i')) {
        this.k_i = initObj.k_i
      }
      else {
        this.k_i = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gain
    // Serialize message field [k_p]
    bufferOffset = _serializer.float32(obj.k_p, buffer, bufferOffset);
    // Serialize message field [k_d]
    bufferOffset = _serializer.float32(obj.k_d, buffer, bufferOffset);
    // Serialize message field [k_i]
    bufferOffset = _serializer.float32(obj.k_i, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gain
    let len;
    let data = new Gain(null);
    // Deserialize message field [k_p]
    data.k_p = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k_d]
    data.k_d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k_i]
    data.k_i = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mascot/Gain';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42218fccfcf87f05cc02314f682ea6ae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 k_p
    float32 k_d
    float32 k_i
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Gain(null);
    if (msg.k_p !== undefined) {
      resolved.k_p = msg.k_p;
    }
    else {
      resolved.k_p = 0.0
    }

    if (msg.k_d !== undefined) {
      resolved.k_d = msg.k_d;
    }
    else {
      resolved.k_d = 0.0
    }

    if (msg.k_i !== undefined) {
      resolved.k_i = msg.k_i;
    }
    else {
      resolved.k_i = 0.0
    }

    return resolved;
    }
};

module.exports = Gain;
