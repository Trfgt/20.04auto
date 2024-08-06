// Auto-generated. Do not edit!

// (in-package vehicle_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta_l = null;
      this.theta_r = null;
      this.torque_fl = null;
      this.torque_fr = null;
      this.torque_rl = null;
      this.torque_rr = null;
    }
    else {
      if (initObj.hasOwnProperty('theta_l')) {
        this.theta_l = initObj.theta_l
      }
      else {
        this.theta_l = 0.0;
      }
      if (initObj.hasOwnProperty('theta_r')) {
        this.theta_r = initObj.theta_r
      }
      else {
        this.theta_r = 0.0;
      }
      if (initObj.hasOwnProperty('torque_fl')) {
        this.torque_fl = initObj.torque_fl
      }
      else {
        this.torque_fl = 0.0;
      }
      if (initObj.hasOwnProperty('torque_fr')) {
        this.torque_fr = initObj.torque_fr
      }
      else {
        this.torque_fr = 0.0;
      }
      if (initObj.hasOwnProperty('torque_rl')) {
        this.torque_rl = initObj.torque_rl
      }
      else {
        this.torque_rl = 0.0;
      }
      if (initObj.hasOwnProperty('torque_rr')) {
        this.torque_rr = initObj.torque_rr
      }
      else {
        this.torque_rr = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Command
    // Serialize message field [theta_l]
    bufferOffset = _serializer.float64(obj.theta_l, buffer, bufferOffset);
    // Serialize message field [theta_r]
    bufferOffset = _serializer.float64(obj.theta_r, buffer, bufferOffset);
    // Serialize message field [torque_fl]
    bufferOffset = _serializer.float64(obj.torque_fl, buffer, bufferOffset);
    // Serialize message field [torque_fr]
    bufferOffset = _serializer.float64(obj.torque_fr, buffer, bufferOffset);
    // Serialize message field [torque_rl]
    bufferOffset = _serializer.float64(obj.torque_rl, buffer, bufferOffset);
    // Serialize message field [torque_rr]
    bufferOffset = _serializer.float64(obj.torque_rr, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Command
    let len;
    let data = new Command(null);
    // Deserialize message field [theta_l]
    data.theta_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [theta_r]
    data.theta_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_fl]
    data.torque_fl = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_fr]
    data.torque_fr = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_rl]
    data.torque_rl = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_rr]
    data.torque_rr = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vehicle_msgs/Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f166b646303bdf36237e2c962a86181';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 theta_l
    float64 theta_r
    float64 torque_fl
    float64 torque_fr
    float64 torque_rl
    float64 torque_rr
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Command(null);
    if (msg.theta_l !== undefined) {
      resolved.theta_l = msg.theta_l;
    }
    else {
      resolved.theta_l = 0.0
    }

    if (msg.theta_r !== undefined) {
      resolved.theta_r = msg.theta_r;
    }
    else {
      resolved.theta_r = 0.0
    }

    if (msg.torque_fl !== undefined) {
      resolved.torque_fl = msg.torque_fl;
    }
    else {
      resolved.torque_fl = 0.0
    }

    if (msg.torque_fr !== undefined) {
      resolved.torque_fr = msg.torque_fr;
    }
    else {
      resolved.torque_fr = 0.0
    }

    if (msg.torque_rl !== undefined) {
      resolved.torque_rl = msg.torque_rl;
    }
    else {
      resolved.torque_rl = 0.0
    }

    if (msg.torque_rr !== undefined) {
      resolved.torque_rr = msg.torque_rr;
    }
    else {
      resolved.torque_rr = 0.0
    }

    return resolved;
    }
};

module.exports = Command;
