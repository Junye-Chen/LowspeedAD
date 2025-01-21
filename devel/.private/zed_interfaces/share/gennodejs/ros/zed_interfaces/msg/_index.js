
"use strict";

let Skeleton2D = require('./Skeleton2D.js');
let Skeleton3D = require('./Skeleton3D.js');
let Keypoint3D = require('./Keypoint3D.js');
let Object = require('./Object.js');
let PosTrackStatus = require('./PosTrackStatus.js');
let BoundingBox3D = require('./BoundingBox3D.js');
let ObjectsStamped = require('./ObjectsStamped.js');
let Keypoint2Df = require('./Keypoint2Df.js');
let RGBDSensors = require('./RGBDSensors.js');
let BoundingBox2Di = require('./BoundingBox2Di.js');
let Keypoint2Di = require('./Keypoint2Di.js');
let PlaneStamped = require('./PlaneStamped.js');
let BoundingBox2Df = require('./BoundingBox2Df.js');

module.exports = {
  Skeleton2D: Skeleton2D,
  Skeleton3D: Skeleton3D,
  Keypoint3D: Keypoint3D,
  Object: Object,
  PosTrackStatus: PosTrackStatus,
  BoundingBox3D: BoundingBox3D,
  ObjectsStamped: ObjectsStamped,
  Keypoint2Df: Keypoint2Df,
  RGBDSensors: RGBDSensors,
  BoundingBox2Di: BoundingBox2Di,
  Keypoint2Di: Keypoint2Di,
  PlaneStamped: PlaneStamped,
  BoundingBox2Df: BoundingBox2Df,
};
