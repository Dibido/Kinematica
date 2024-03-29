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

class stopServos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servoIds = null;
    }
    else {
      if (initObj.hasOwnProperty('servoIds')) {
        this.servoIds = initObj.servoIds
      }
      else {
        this.servoIds = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type stopServos
    // Serialize message field [servoIds]
    bufferOffset = _arraySerializer.uint32(obj.servoIds, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type stopServos
    let len;
    let data = new stopServos(null);
    // Deserialize message field [servoIds]
    data.servoIds = _arrayDeserializer.uint32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.servoIds.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robotarminterface/stopServos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '58c2b1e06ba6865d4582bb6764fd753f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32[] servoIds
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new stopServos(null);
    if (msg.servoIds !== undefined) {
      resolved.servoIds = msg.servoIds;
    }
    else {
      resolved.servoIds = []
    }

    return resolved;
    }
};

module.exports = stopServos;
