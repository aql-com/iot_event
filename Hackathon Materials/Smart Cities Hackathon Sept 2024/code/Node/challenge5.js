const axios = require('axios');

// Constants
const CORE_URL = 'https://api.core.aql.com/v1/';
const TOKEN = '11881|gLoVRXiq8KJ6lFMdlvtXNkjOzKMtrUI5veCqIVKIa56fb661';
const DEVICE_ID = 'LD795U6YL4g';

function toHex(value) {
    // Convert value to hexadecimal string
    if (typeof value === 'number') {
        return value.toString(16);
    } else if (typeof value === 'string') {
        return Buffer.from(value).toString('hex');
    }
    return value;
}

function generateReading() {
    /**
     * Generate a random reading.
     */
    const temperature = (Math.random() * 60 - 20).toFixed(1);  // Random temperature between -20 and 40
    const humidity = Math.floor(Math.random() * 101);  // Random humidity between 0 and 100
    const co2 = Math.floor(Math.random() * 701 + 300);  // Random CO2 level between 300 and 1000 ppm
    const pm25 = Math.floor(Math.random() * 51);  // Random PM2.5 level between 0 and 50 µg/m³
    const pm10 = Math.floor(Math.random() * 101);  // Random PM10 level between 0 and 100 µg/m³

    return {
            temperature: toHex(temperature),
            humidity: toHex(humidity),
            co2: toHex(co2),
            pm25: toHex(pm25),
            pm10: toHex(pm10)

    };
}

async function submitReading(payload) {
    const url = `${CORE_URL}devices/external-data-webhook/other/${DEVICE_ID}`;


    const headers = { 'Authorization': `Bearer ${TOKEN}`, 'Content-Type': 'application/json' };

    try {
        const response = await axios.post(url, payload, { headers });
        console.log('Reading submitted successfully:', payload);
    } catch (error) {
        console.error('Error submitting reading:', error);
    }
}

async function generateAndSubmitReadings() {
    /**
     * Generate and submit readings every 5 minutes.
     */
        // Immediately submit the first reading
    const firstReading = generateReading();
    await submitReading(firstReading);

    // Schedule subsequent readings every 5 minutes
    setInterval(async () => {
        const reading = generateReading();
        await submitReading(reading);
    }, 300000);  // 300000 milliseconds = 5 minutes
}

// Test the function
generateAndSubmitReadings();
