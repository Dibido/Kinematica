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

class servoPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servoId = null;
      this.position = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type servoPosition
    // Serialize message field [servoId]
    bufferOffset = _serializer.uint32(obj.servoId, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = _serializer.int32(obj.position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type servoPosition
    let len;
    let data = new servoPosition(null);
    // Deserialize message field [servoId]
    data.servoId = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotarminterface/servoPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '21c24bd1e99f0c44d572dd36095ff06f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 servoId
    int32 position
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new servoPosition(null);
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

    return resolved;
    }
};

module.exports = servoPosition;
