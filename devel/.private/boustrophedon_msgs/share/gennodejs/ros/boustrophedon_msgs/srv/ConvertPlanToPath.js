// Auto-generated. Do not edit!

// (in-package boustrophedon_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let StripingPlan = require('../msg/StripingPlan.js');

//-----------------------------------------------------------

let nav_msgs = _finder('nav_msgs');

//-----------------------------------------------------------

class ConvertPlanToPathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.plan = null;
    }
    else {
      if (initObj.hasOwnProperty('plan')) {
        this.plan = initObj.plan
      }
      else {
        this.plan = new StripingPlan();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConvertPlanToPathRequest
    // Serialize message field [plan]
    bufferOffset = StripingPlan.serialize(obj.plan, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConvertPlanToPathRequest
    let len;
    let data = new ConvertPlanToPathRequest(null);
    // Deserialize message field [plan]
    data.plan = StripingPlan.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += StripingPlan.getMessageSize(object.plan);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'boustrophedon_msgs/ConvertPlanToPathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f7a5dc2a7fd3a005db67fe266f849039';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    boustrophedon_msgs/StripingPlan plan
    
    ================================================================================
    MSG: boustrophedon_msgs/StripingPlan
    Header header
    StripingPoint[] points
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: boustrophedon_msgs/StripingPoint
    geometry_msgs/Point point
    uint8 type
    
    uint8 OUTLINE = 0
    uint8 STRIPE_START = 1
    uint8 STRIPE_END = 2
    uint8 STRIPE_INTERMEDIATE = 3
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConvertPlanToPathRequest(null);
    if (msg.plan !== undefined) {
      resolved.plan = StripingPlan.Resolve(msg.plan)
    }
    else {
      resolved.plan = new StripingPlan()
    }

    return resolved;
    }
};

class ConvertPlanToPathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path = null;
    }
    else {
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = new nav_msgs.msg.Path();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConvertPlanToPathResponse
    // Serialize message field [path]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.path, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConvertPlanToPathResponse
    let len;
    let data = new ConvertPlanToPathResponse(null);
    // Deserialize message field [path]
    data.path = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.path);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'boustrophedon_msgs/ConvertPlanToPathResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '58d6f138c7de7ef47c75d4b7e5df5472';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    nav_msgs/Path path
    
    
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConvertPlanToPathResponse(null);
    if (msg.path !== undefined) {
      resolved.path = nav_msgs.msg.Path.Resolve(msg.path)
    }
    else {
      resolved.path = new nav_msgs.msg.Path()
    }

    return resolved;
    }
};

module.exports = {
  Request: ConvertPlanToPathRequest,
  Response: ConvertPlanToPathResponse,
  md5sum() { return '3a9fa9a5fc158705104b513251035497'; },
  datatype() { return 'boustrophedon_msgs/ConvertPlanToPath'; }
};
