const fs = require("fs");

const TIME_MS_FROM_DOWN_TO_UP = 15000;
const QUEUE_NEXT_TIMEOUT = 500;
const STATE_FILE = "/home/homebridge/home-desk.json";
const DEFAULT_STATE = {
  currentPosition: 0,
  targetPosition: 0,
  positionState: "idle",
};

const MQTTTHING_DUMMY_GET = "desk/mqttthing/getState";
const MQTTTHING_DUMMY_SET = "desk/mqttthing/setTargetPosition";

const MQTT_TOPIC_AVAILABILITY = "desk/availability";
const MQTT_TOPIC_CALLBACK = "desk/callback";

module.exports = {
  init: function ({ config, log, publish }) {
    function writeState() {
      return fs.writeFileSync(STATE_FILE, JSON.stringify(state));
    }

    function readState() {
      try {
        return JSON.parse(fs.readFileSync(STATE_FILE)) || DEFAULT_STATE;
      } catch (err) {
        return DEFAULT_STATE;
      }
    }

    function notifyState() {
      log("Desk", "notifyState", state);
      publish(MQTTTHING_DUMMY_GET, JSON.stringify(state));
      writeState();
    }

    function processQueue() {
      if (queue.length === 0) {
        setTimeout(processQueue, QUEUE_NEXT_TIMEOUT);
        return;
      }

      if (state.positionState !== "idle") {
        log("Desk: Waiting for Desk to be idle");
        setTimeout(processQueue, QUEUE_NEXT_TIMEOUT);
        return;
      }

      const targetPosition = Number(queue.shift());
      log("Processing new targetPosition to", targetPosition);

      const timeMs =
        (TIME_MS_FROM_DOWN_TO_UP *
          Math.abs(targetPosition - state.currentPosition)) /
        100;
      const direction = targetPosition > state.currentPosition ? "up" : "down";

      // Send the command to the desk
      publish(
        MQTT_TOPIC_CALLBACK,
        JSON.stringify({ command: "updown", direction, timeMs })
      );

      state.targetPosition = targetPosition;
      state.positionState = direction;
      notifyState();

      setTimeout(() => {
        state.positionState = "idle";
        state.currentPosition = state.targetPosition;
        notifyState();

        setImmediate(processQueue, 0);
      }, timeMs);
    }

    // Change config

    config.name = config.name || "Desk";

    config.type = "window";
    config.topics = {
      getOnline: MQTT_TOPIC_AVAILABILITY,
      setTargetPosition: MQTTTHING_DUMMY_SET,
      getTargetPosition: MQTTTHING_DUMMY_GET,
      getCurrentPosition: MQTTTHING_DUMMY_GET,
      getPositionState: MQTTTHING_DUMMY_GET,
    };

    const queue = [];
    const state = readState();
    log("Initial state", state);

    setImmediate(processQueue, 0);
    setImmediate(notifyState, 0);

    return {
      properties: {
        online: {
          encode(message) {
            return message;
          },
          decode(message) {
            return message === "online";
          },
        },
        targetPosition: {
          encode(message) {
            log("Request setTargetPosition", message);
            queue.push(message);
            // Do not send anything to the desk, as we have a queue that process commands
            return undefined;
          },
          decode(message) {
            return JSON.parse(message).targetPosition;
          },
        },
        currentPosition: {
          decode(message) {
            return JSON.parse(message).currentPosition;
          },
        },
        positionState: {
          decode(message) {
            return { idle: "STOPPED", up: "INCREASING", down: "DECREASING" }[
              JSON.parse(message).positionState
            ];
          },
        },
      },
    };
  },
};
