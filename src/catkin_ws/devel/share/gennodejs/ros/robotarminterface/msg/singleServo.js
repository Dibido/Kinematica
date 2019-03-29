// Auto-generated. Do not edit!

// (in-package robotarminterface.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class singleServo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servoId = null;
      this.position = null;
      this.time = null;
    }
    else {
      if (initObj.hasOwnProperty('servoId')) {
        this.servoId = initObj.servoId
      }
      else {
        this.servoId = 0;
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type singleServo
    // Serialize message field [servoId]
    bufferOffset = _serializer.uint32(obj.servoId, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.uint32(obj.position, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.uint32(obj.time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type singleServo
    let len;
    let data = new singleServo(null);
    // Deserialize message field [servoId]
    data.servoId = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotarminterface/singleServo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '39faf3391f2b3261e444ffb062992407';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 servoId
    uint32 position
    uint32 time
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new singleServo(null);
    if (msg.servoId !== undefined) {
      resolved.servoId = msg.servoId;
    }
    else {
      resolved.servoId = 0
    }

    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0
    }

    return resolved;
    }
};

module.exports = singleServo;
