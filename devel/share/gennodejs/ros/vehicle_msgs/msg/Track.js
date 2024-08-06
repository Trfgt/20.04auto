// Auto-generated. Do not edit!

// (in-package vehicle_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TrackCone = require('./TrackCone.js');

//-----------------------------------------------------------

class Track {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cones = null;
    }
    else {
      if (initObj.hasOwnProperty('cones')) {
        this.cones = initObj.cones
      }
      else {
        this.cones = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Track
    // Serialize message field [cones]
    // Serialize the length for message field [cones]
    bufferOffset = _serializer.uint32(obj.cones.length, buffer, bufferOffset);
    obj.cones.forEach((val) => {
      bufferOffset = TrackCone.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Track
    let len;
    let data = new Track(null);
    // Deserialize message field [cones]
    // Deserialize array length for message field [cones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cones[i] = TrackCone.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.cones.forEach((val) => {
      length += TrackCone.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vehicle_msgs/Track';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1bd255a768d640e676ebd0a7279d0839';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    vehicle_msgs/TrackCone[] cones
    ================================================================================
    MSG: vehicle_msgs/TrackCone
    float64 x
    float64 y
    
    string type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Track(null);
    if (msg.cones !== undefined) {
      resolved.cones = new Array(msg.cones.length);
      for (let i = 0; i < resolved.cones.length; ++i) {
        resolved.cones[i] = TrackCone.Resolve(msg.cones[i]);
      }
    }
    else {
      resolved.cones = []
    }

    return resolved;
    }
};

module.exports = Track;
