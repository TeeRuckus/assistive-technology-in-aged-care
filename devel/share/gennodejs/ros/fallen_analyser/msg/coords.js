// Auto-generated. Do not edit!

// (in-package fallen_analyser.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class coords {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.xCord = null;
      this.yCord = null;
      this.rightCam = null;
    }
    else {
      if (initObj.hasOwnProperty('xCord')) {
        this.xCord = initObj.xCord
      }
      else {
        this.xCord = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('yCord')) {
        this.yCord = initObj.yCord
      }
      else {
        this.yCord = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('rightCam')) {
        this.rightCam = initObj.rightCam
      }
      else {
        this.rightCam = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type coords
    // Serialize message field [xCord]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.xCord, buffer, bufferOffset);
    // Serialize message field [yCord]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.yCord, buffer, bufferOffset);
    // Serialize message field [rightCam]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.rightCam, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type coords
    let len;
    let data = new coords(null);
    // Deserialize message field [xCord]
    data.xCord = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [yCord]
    data.yCord = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [rightCam]
    data.rightCam = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fallen_analyser/coords';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06455b0f277279021e59afa6a4c4a32a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float32 xCord
    std_msgs/Float32 yCord
    std_msgs/Bool rightCam
    
    ================================================================================
    MSG: std_msgs/Float32
    float32 data
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new coords(null);
    if (msg.xCord !== undefined) {
      resolved.xCord = std_msgs.msg.Float32.Resolve(msg.xCord)
    }
    else {
      resolved.xCord = new std_msgs.msg.Float32()
    }

    if (msg.yCord !== undefined) {
      resolved.yCord = std_msgs.msg.Float32.Resolve(msg.yCord)
    }
    else {
      resolved.yCord = new std_msgs.msg.Float32()
    }

    if (msg.rightCam !== undefined) {
      resolved.rightCam = std_msgs.msg.Bool.Resolve(msg.rightCam)
    }
    else {
      resolved.rightCam = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = coords;
