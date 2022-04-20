module.exports = {
  init: function ({ config, log, publish }) {
    // Change config
    config.type = "custom";
    config.minDryValue = config.minDryValue || 800;
    config.services = [
      {
        type: "humiditySensor",
        name: `${config.name} - Soil Moisture`,
        history: true,
        topics: {
          // getOnline: `${config.topicPrefix}/availability`,
          getCurrentRelativeHumidity: `${config.topicPrefix}/soilmoisture`,
        },
      },
      {
        type: "leakSensor",
        name: `${config.name} - Dry Sensor`,
        topics: {
          getLeakDetected: `${config.topicPrefix}/leak`,
        },
      },
    ];

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
        currentRelativeHumidity: {
          decode(message) {
            console.log(`[${config.name}] Got moisture value: ${message}`);
            // Value is between 0 and 1024
            const dryValue = parseInt(message, 10);
            if (dryValue > config.minDryValue) {
              publish(`${config.topicPrefix}/leak`, true);
            } else {
              publish(`${config.topicPrefix}/leak`, false);
            }
            const humidityValue = Math.floor(100 - (dryValue / 1024) * 100);
            return humidityValue;
          },
        },
        leakDetected: {
          decode(message) {
            return message;
          },
        },
      },
    };
  },
};
