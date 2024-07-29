const mqtt = require('mqtt');
const LOCATION_ID = "XVDiGl9Wwo";
const DEVICE_ID = "m6QAZiORvD7";
const TOPIC = `location/${LOCATION_ID}/device/${DEVICE_ID}/event/up`;
const TEAM = "Team 1"
const USER_NAME = "testing1"
const PASSWORD  = "xMJf66dhtUyCY3d8W"

let mqttConnectionDetails = {
    host:                       "mqtt.core.aql.com",
    port:                       8884,
    protocol:                   "mqtts",
    keepalive:                  10,
    clientId:                   TEAM,
    protocolId:                 "MQTT",
    protocolVersion:            4,
    clean:                      true,
    reconnectPeriod:            2000,
    connectTimeout:             2000,
    rejectUnauthorized:         false,
    username: USER_NAME,
    password: PASSWORD
}

// Function to generate random values for the payload
function generatePayload() {
    const latitude = Math.random() * (90 - (-90)) + (-90); // Random latitude between -90 and 90
    const longitude = Math.random() * (180 - (-180)) + (-180); // Random longitude between -180 and 180
    const temperature = Math.random() * (40 - (-20)) + (-20); // Random temperature between -20 and 40
    const humidity = Math.random() * (100); // Random humidity between 0 and 100
    const co2 = Math.random() * (1000 - 400) + 400; // Random CO2 between 400 and 1000
    const pm25 = Math.random() * (100); // Random PM2.5 between 0 and 100
    const pm10 = Math.random() * (100); // Random PM10 between 0 and 100

    // Return the payload object
    return {
        latitude: latitude,
        longitude: longitude,
        temperature: temperature,
        humidity: humidity,
        co2: co2,
        pm25: pm25,
        pm10: pm10
    };
}

// Connect to MQTT broker with authentication
const client = mqtt.connect(mqttConnectionDetails);

// When connected, publish the payload to the specified topic every minute
client.on('connect', () => {
    console.log('Connected to MQTT broker');

    // Initial upload upon connection
    const initialPayload = generatePayload();
    console.log("Initial Payload:", initialPayload);
    console.log('Uploading to topic:', TOPIC);
    client.publish(TOPIC, JSON.stringify(initialPayload), () => {
        console.log('Initial message published');
    });

   
    setInterval(() => {
        for (let i = 0; i < 1; i++) {
            const payload = generatePayload();
            console.log("Payload:", payload);
            console.log('Uploading to topic:', TOPIC);
            client.publish(TOPIC, JSON.stringify(payload), () => {
                console.log('Message published');
            });
        }
    }, 5000);
});

// Log errors
client.on('error', (error) => {
    console.error('MQTT error:', error);
});
