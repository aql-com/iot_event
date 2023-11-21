const fuelSensorID = 'lVjRS7PvmxO';
const waterSensorID = 'lkmWs7PvmxO';
const temperatureSensorID = 'KEnkUJMGPO8';
const lightSensorID = 'poQ0tDOBqmQ';

// Fuel Gauge
let fuelGaugeData = null;
let fuelReadingValues = [];
let fuelReadingLabels = [];
let fuelLevels = [];
let fuelPercentage = 0;

const fuelWrapper = document.querySelector('.fuel-gauge-wrapper');
const fuelTimeDD = document.getElementById('fuelTime');
const fuelLevelSpan = document.getElementById('fuel-level');
const rootCSS = document.querySelector(':root');

if (fuelWrapper) {

    const fuelTimeChange = () => {
        let fuelTime = fuelTimeDD.value;
        let fuelIndex = fuelReadingValues.findIndex(x => x == fuelTime);
        let fuelLevel = fuelLevels[fuelIndex];
        let fuelDegrees = Math.round((fuelLevels[fuelIndex] / 100) * 220);

        rootCSS.style.setProperty('--fuelLevel', fuelDegrees + 'deg');
        fuelLevelSpan.innerHTML = fuelLevel;
    }
    fuelTimeDD.addEventListener('change', fuelTimeChange);

    const fetchFuelGaugeData = () => {
        fetch('https://api.core.aql.com/v1/sensors/sensor-data/aggregate/ave', {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Authorization': 'Bearer 11371|4L6TMzBwVlykgcc9nASWJTNriPeysVJhOde57XIO',
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                'sensor_ids': [
                    fuelSensorID,
                ],
                'startDate': '2023-11-14 00:00:00',
                'endDate': '2023-11-14 23:59:59',
                'sampleInterval': 1,
            })
        }).then(response => {
            if (response.status === 200) {
                response.json().then(data => processFuelGaugeData(data));
            }
        });
    }

    const processFuelGaugeData = fuelGaugeData => {

        for (let i = 0; i < fuelGaugeData.length; i++) {
            let fuelReadingTime = fuelGaugeData[i].sensorReadingDate;
            let fuelLevel = fuelGaugeData[i].Average;
            let fuelReadingLabel = fuelGaugeData[i].sensorReadingDate.substring(11, 16);

            fuelReadingValues.push(fuelReadingTime);
            fuelReadingLabels.push(fuelReadingLabel);
            fuelLevels.push(fuelLevel);
        }

        for (let i = 0; i < fuelReadingValues.length; i++) {
            const option = document.createElement("option");
            option.text = fuelReadingLabels[i];
            option.value = fuelReadingValues[i];
            fuelTimeDD.append(option);
        }
        fuelTimeChange();
    }

    document.addEventListener('DOMContentLoaded', fetchFuelGaugeData(fuelSensorID));
}

// Water Gauge
let waterGaugeData = null;
let waterReadingValues = [];
let waterReadingLabels = [];
let waterLevels = [];
let waterPercentage = 0;

const waterWrapper = document.querySelector('.water-gauge-wrapper');
const waterTimeDD = document.getElementById('waterTime');
const waterLevelSpan = document.getElementById('water-level');

if (waterWrapper) {

    const waterTimeChange = () => {
        let waterTime = waterTimeDD.value;
        let waterIndex = waterReadingValues.findIndex(x => x == waterTime);
        let waterLevel = waterLevels[waterIndex];

        rootCSS.style.setProperty('--waterLevel', waterLevel + '%');
        waterLevelSpan.innerHTML = waterLevel;
    }
    waterTimeDD.addEventListener('change', waterTimeChange);

    const fetchwaterGaugeData = () => {
        fetch('https://api.core.aql.com/v1/sensors/sensor-data/aggregate/ave', {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Authorization': 'Bearer 11371|4L6TMzBwVlykgcc9nASWJTNriPeysVJhOde57XIO',
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                'sensor_ids': [
                    waterSensorID,
                ],
                'startDate': '2023-11-13 00:00:00',
                'endDate': '2023-11-13 23:59:59',
                'sampleInterval': 1,
            })
        }).then(response => {
            if (response.status === 200) {
                response.json().then(data => processwaterGaugeData(data));
            }
        });
    }

    const processwaterGaugeData = waterGaugeData => {

        for (let i = 0; i < waterGaugeData.length; i++) {
            let waterReadingTime = waterGaugeData[i].sensorReadingDate;
            let waterLevel = waterGaugeData[i].Average;
            let waterReadingLabel = waterGaugeData[i].sensorReadingDate.substring(11, 16);

            waterReadingValues.push(waterReadingTime);
            waterReadingLabels.push(waterReadingLabel);
            waterLevels.push(waterLevel);
        }

        for (let i = 0; i < waterReadingValues.length; i++) {
            const option = document.createElement("option");
            option.text = waterReadingLabels[i];
            option.value = waterReadingValues[i];
            waterTimeDD.append(option);
        }
        waterTimeChange();
    }

    document.addEventListener('DOMContentLoaded', fetchwaterGaugeData(waterSensorID));
}

// Temperature Gauge
let temperatureGaugeData = null;
let temperatureReadingValues = [];
let temperatureReadingLabels = [];
let temperatureLevels = [];
let temperaturePercentage = 0;

const temperatureWrapper = document.querySelector('.temperature-gauge-wrapper');
const temperatureTimeDD = document.getElementById('temperatureTime');
const temperatureLevelSpan = document.getElementById('temperature-level');

if (temperatureWrapper) {
    const temperatureTimeChange = () => {
        let temperatureTime = temperatureTimeDD.value;
        let temperatureIndex = temperatureReadingValues.findIndex(x => x == temperatureTime);
        let temperatureLevel = temperatureLevels[temperatureIndex];
        let temperatureMeter = temperatureLevels[temperatureIndex] * 2.5;

        rootCSS.style.setProperty('--temperatureLevel', temperatureMeter + '%');
        temperatureLevelSpan.innerHTML = temperatureLevel;
    }
    temperatureTimeDD.addEventListener("change", temperatureTimeChange);

    const fetchtemperatureGaugeData = () => {
        fetch('https://api.core.aql.com/v1/sensors/sensor-data/aggregate/ave', {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Authorization': 'Bearer 11371|4L6TMzBwVlykgcc9nASWJTNriPeysVJhOde57XIO',
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                'sensor_ids': [
                    temperatureSensorID,
                ],
                'startDate': '2023-11-13 00:00:00',
                'endDate': '2023-11-13 23:59:59',
                'sampleInterval': 1
            })
        }).then(response => {
            if (response.status === 200) {
                response.json().then(data => processtemperatureGaugeData(data));
            }
        });
    }

    const processtemperatureGaugeData = temperatureGaugeData => {

        for (let i = 0; i < temperatureGaugeData.length; i++) {
            let temperatureReadingTime = temperatureGaugeData[i].sensorReadingDate;
            let temperatureLevel = temperatureGaugeData[i].Average;
            let temperatureReadingLabel = temperatureGaugeData[i].sensorReadingDate.substring(11, 16);

            temperatureReadingValues.push(temperatureReadingTime);
            temperatureReadingLabels.push(temperatureReadingLabel);
            temperatureLevels.push(temperatureLevel);
        }

        for (let i = 0; i < temperatureReadingValues.length; i++) {
            const option = document.createElement("option");
            option.text = temperatureReadingLabels[i];
            option.value = temperatureReadingValues[i];
            temperatureTimeDD.append(option);
        }
        temperatureTimeChange();
    }

    document.addEventListener('DOMContentLoaded', fetchtemperatureGaugeData(temperatureSensorID));
}

// Light Gauge
let lightGaugeData = null;
let lightReadingValues = [];
let lightReadingLabels = [];
let lightLevels = [];
let lightPercentage = 0;

const lightWrapper = document.querySelector('.light-gauge-wrapper');
const lightTimeDD = document.getElementById('lightTime');
const lightLevelSpan = document.getElementById('light-level');

if (lightWrapper) {
    const lightTimeChange = () => {
        let lightTime = lightTimeDD.value;
        let lightIndex = lightReadingValues.findIndex(x => x == lightTime);
        let lightLevel = lightLevels[lightIndex];
        let lightMeter = lightLevels[lightIndex] / 12000;

        rootCSS.style.setProperty('--lightLevel', lightMeter);
        lightLevelSpan.innerHTML = lightLevel;
    }
    lightTimeDD.addEventListener("change", lightTimeChange);

    const fetchlightGaugeData = () => {

        fetch('https://api.core.aql.com/v1/sensors/sensor-data/aggregate/ave', {
            method: 'POST',
            headers: {
                'Accept': 'application/json',
                'Authorization': 'Bearer 11371|4L6TMzBwVlykgcc9nASWJTNriPeysVJhOde57XIO',
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                'sensor_ids': [
                    lightSensorID,
                ],
                'startDate': '2023-11-10 00:00:00',
                'endDate': '2023-11-10 23:59:59',
                'sampleInterval': 1,
            })
        }).then(response => {
            if (response.status === 200) {
                response.json().then(data => processlightGaugeData(data));
            }
        });
    }

    const processlightGaugeData = lightGaugeData => {

        for (let i = 0; i < lightGaugeData.length; i++) {
            let lightReadingTime = lightGaugeData[i].sensorReadingDate;
            let lightLevel = lightGaugeData[i].Average;
            let lightReadingLabel = lightGaugeData[i].sensorReadingDate.substring(11, 16);

            lightReadingValues.push(lightReadingTime);
            lightReadingLabels.push(lightReadingLabel);
            lightLevels.push(lightLevel);
        }

        for (let i = 0; i < lightReadingValues.length; i++) {
            const option = document.createElement("option");
            option.text = lightReadingLabels[i];
            option.value = lightReadingValues[i];
            lightTimeDD.append(option);
        }

        lightTimeChange();
    }

    document.addEventListener('DOMContentLoaded', fetchlightGaugeData(lightSensorID));
}