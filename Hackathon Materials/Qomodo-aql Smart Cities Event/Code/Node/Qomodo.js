const mqtt = require('mqtt');
const Chance = require('chance');
const chance = new Chance();

// Configuration
const brokerUrl = 'mqtt://gw-0.qomodo.io:8883';
const topic = 'events';

// User Details
const options = {
    username: "YOUR_USER_NAME",
    password: "YOUR_PASSORD",
    rejectUnauthorized: true
};
const teamNumber = 0;// REPLACE WITH YOUR_TEAM NUMBER

// Required Observables
const observables = [
    { src_ip: '152.32.138.247', instances: 12 },
    { src_ip: '45.159.209.228', instances: 1, dst_ip: '152.32.138.247' },
    { src_ip: '155.138.146.162', instances: 1, dst_port: '5555', filename: 'kv-mipsel', file_hash: '48299c2c568ce5f0d4f801b4aee0a6109b68613d2948ce4948334bbd7adc49eb' }
];

// Generate random IPs, port, bytes, etc.
const generateRandomEvent = () => ({
    device_id: chance.guid(),
    event_id: chance.guid(),
    event_timestamp: new Date().toISOString(),
    event_type: 'NetworkConnection',
    src_ip: chance.ip(),
    src_port: chance.integer({ min: 1024, max: 65535 }).toString(),
    dst_ip: chance.ip(),
    dst_port: chance.integer({ min: 1024, max: 65535 }).toString(),
    src_bytes: chance.integer({ min: 1, max: 10000 }).toString(),
    dst_bytes: chance.integer({ min: 1, max: 10000 }).toString(),
    domain_name: chance.domain(),
    team_number:teamNumber
});

// Generate specific observables
const generateObservableEvent = (observable, isContinuedHost = false) => ({
    device_id: isContinuedHost ? continuedHost : chance.guid(),
    event_id: chance.guid(),
    event_timestamp: new Date().toISOString(),
    event_type: 'NetworkConnection',
    src_ip: observable.src_ip,
    src_port: observable.src_port || chance.integer({ min: 1024, max: 65535 }).toString(),
    dst_ip: observable.dst_ip || chance.ip(),
    dst_port: observable.dst_port || chance.integer({ min: 1024, max: 65535 }).toString(),
    src_bytes: chance.integer({ min: 1, max: 10000 }).toString(),
    dst_bytes: chance.integer({ min: 1, max: 10000 }).toString(),
    domain_name: chance.domain(),
    filename: observable.filename,
    file_hash: observable.file_hash,
    team_number: teamNumber
});

let continuedHost = chance.guid(); // This will link events for the same sensor


// Function to publish events
const publishEvents = () => {
    const client = mqtt.connect(brokerUrl, options);

    client.on('connect', function () {
        console.log('Connected to MQTT broker');

        // Publish random events
        for (let i = 0; i < 100; i++) {
            let message;
            if (i < observables[0].instances) {
                message = generateObservableEvent(observables[0]);
            } else if (i === observables[0].instances) {
                message = generateObservableEvent(observables[1]);
            } else if (i === observables[0].instances + 1) {
                message = generateObservableEvent(observables[2], true);
            } else if (i === observables[0].instances + 2) {
                message = generateObservableEvent({
                    src_ip: '155.138.146.162',
                    filename: 'kv-mipsel',
                    file_hash: '48299c2c568ce5f0d4f801b4aee0a6109b68613d2948ce4948334bbd7adc49eb'
                }, true);
            } else {
                message = generateRandomEvent();
            }

            const messageStr = JSON.stringify(message);
            client.publish(topic, messageStr, { qos: 1 }, function (err) {
                if (err) {
                    console.error('Failed to publish event:', err);
                } else {
                    console.log('Published event:', message);
                }
            });
        }

        // Disconnect the client after publishing
        setTimeout(() => client.end(), 5000);
    });

    client.on('error', function (err) {
        console.error('Connection error:', err);
    });
};

// Publish events immediately
publishEvents();

// Set an interval to publish events every 2 minutes (120,000 milliseconds)
setInterval(publishEvents, 120000);