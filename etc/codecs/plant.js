module.exports = {
  init: function ({ config, log, publish }) {
    // Change config
    config.type = "custom";
    config.minHumidityValue = config.minHumidityValue || 30;
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
            log(`[${config.name}] Got moisture value: ${message}`);

            // Value is between 0 and 1024
            const dryValue = parseInt(message, 10);
            const humidityValue = Math.floor(100 - (dryValue / 1024) * 100);

            publish(
              `${config.topicPrefix}/leak`,
              humidityValue < config.minHumidityValue
            );

            // Sleep for 2h
            setImmediate(function () {
              publish(
                `${config.topicPrefix}/callback`,
                JSON.stringify({ command: "deepsleep", seconds: 60 * 60 * 2 })
              );
            });

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
