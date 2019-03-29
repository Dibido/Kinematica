
"use strict";

let stopServos = require('./stopServos.js');
let allServo = require('./allServo.js');
let stopSingleServo = require('./stopSingleServo.js');
let stopAllServo = require('./stopAllServo.js');
let moveServos = require('./moveServos.js');
let singleServo = require('./singleServo.js');
let armInstruction = require('./armInstruction.js');
let servoPosition = require('./servoPosition.js');

module.exports = {
  stopServos: stopServos,
  allServo: allServo,
  stopSingleServo: stopSingleServo,
  stopAllServo: stopAllServo,
  moveServos: moveServos,
  singleServo: singleServo,
  armInstruction: armInstruction,
  servoPosition: servoPosition,
};
