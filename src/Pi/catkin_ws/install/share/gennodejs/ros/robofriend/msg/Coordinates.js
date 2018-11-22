// Auto-generated. Do not edit!

// (in-package robofriend.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Coordinates {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.y_top = null;
      this.right = null;
      this.bottom = null;
      this.x_left = null;
      this.face_name = null;
    }
    else {
      if (initObj.hasOwnProperty('y_top')) {
        this.y_top = initObj.y_top
      }
      else {
        this.y_top = 0;
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = 0;
      }
      if (initObj.hasOwnProperty('bottom')) {
        this.bottom = initObj.bottom
      }
      else {
        this.bottom = 0;
      }
      if (initObj.hasOwnProperty('x_left')) {
        this.x_left = initObj.x_left
      }
      else {
        this.x_left = 0;
      }
      if (initObj.hasOwnProperty('face_name')) {
        this.face_name = initObj.face_name
      }
      else {
        this.face_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Coordinates
    // Serialize message field [y_top]
    bufferOffset = _serializer.uint16(obj.y_top, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = _serializer.uint16(obj.right, buffer, bufferOffset);
    // Serialize message field [bottom]
    bufferOffset = _serializer.uint16(obj.bottom, buffer, bufferOffset);
    // Serialize message field [x_left]
    bufferOffset = _serializer.uint16(obj.x_left, buffer, bufferOffset);
    // Serialize message field [face_name]
    bufferOffset = _serializer.string(obj.face_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Coordinates
    let len;
    let data = new Coordinates(null);
    // Deserialize message field [y_top]
    data.y_top = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [bottom]
    data.bottom = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [x_left]
    data.x_left = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [face_name]
    data.face_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.face_name.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robofriend/Coordinates';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be7c1e838538f5e60d0dc085dffee546';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 y_top
    uint16 right
    uint16 bottom
    uint16 x_left
    string face_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Coordinates(null);
    if (msg.y_top !== undefined) {
      resolved.y_top = msg.y_top;
    }
    else {
      resolved.y_top = 0
    }

    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = 0
    }

    if (msg.bottom !== undefined) {
      resolved.bottom = msg.bottom;
    }
    else {
      resolved.bottom = 0
    }

    if (msg.x_left !== undefined) {
      resolved.x_left = msg.x_left;
    }
    else {
      resolved.x_left = 0
    }

    if (msg.face_name !== undefined) {
      resolved.face_name = msg.face_name;
    }
    else {
      resolved.face_name = ''
    }

    return resolved;
    }
};

module.exports = Coordinates;
