import * as THREEv140 from 'three-v140';

let granularitySelect = null;
let sensorData = {};
let sensorConfig = {
    'QKQrtxlygOB': {
        type: 'line',
        label: 'Water Content Over Time',
        borderColor: '#408FEF',
        backgroundColor: '#408FEF'
    },
    'DZMDfPJkwqj': {
        type: 'line',
        label: 'Temperature Over Time',
        borderColor: '#D1607A',
        backgroundColor: '#D1607A'
    },
    'poQ0tDOBqmQ': {
        type: 'bar',
        label: 'Light Level',
        borderColor: '#FFC300',
        backgroundColor: '#FFC300'
    },
    'lL7WU7PvmxO': {
        type: 'radar',
        label: 'Temperature Over Time',
        borderColor: '#50C878',
        backgroundColor: 'rgba(80, 200, 120, 0.5)'
    }
};

Date.prototype.yyyymmdd = function () {
    const mm = this.getMonth() + 1;
    const dd = this.getDate() - 1;

    return [this.getFullYear(),
    (mm > 9 ? '' : '0') + mm,
    (dd > 9 ? '' : '0') + dd
    ].join('-');
};

const date = new Date();
const yesterday = date.yyyymmdd();
const startDate = yesterday + ' 00:00:00';
const endDate = yesterday + ' 23:59:59';

const granularityChange = () => {
    fetchData();
}

const fetchData = () => {
    fetch('https://api.core.aql.com/v1/sensors/sensor-data/aggregate/ave', {
        method: 'POST',
        headers: {
            'Accept': 'application/json',
            'Authorization': 'Bearer 11371|4L6TMzBwVlykgcc9nASWJTNriPeysVJhOde57XIO',
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            'sensor_ids': Object.keys(sensorConfig),
            'startDate': startDate,
            'endDate': endDate,
            'granularity': granularitySelect.value
        })
    }).then(response => {
        if (response.status === 200) {
            response.json().then(data => processData(data));
        }
    });
}

const charts = {};

const processData = apiData => {

    sensorData = {};

    for (let i = 0; i < apiData.length; i++) {

        let sensorId = apiData[i]['sensor_id'];

        if (sensorId in sensorData == false) {
            sensorData[sensorId] = {
                labels: [],
                values: [],
            };
        }

        sensorData[sensorId].labels.push(apiData[i].sensorReadingDate.substring(11, 16));
        sensorData[sensorId].values.push(apiData[i].Average);
    }

    const chartWrapper = document.getElementById('js-chart-wrapper');
    const sensorDataKeys = Object.keys(sensorData);

    for (let i = 0; i < sensorDataKeys.length; i++) {

        const chartID = sensorDataKeys[i];
        let chart = charts[chartID];

        if (chart) {

            chart.chartJS.destroy();

        } else {

            charts[chartID] = {};
            chart = charts[chartID];

            const chartDiv = document.createElement('div');
            chartDiv.style.width = '45%';

            chart.chartCanvas = document.createElement('canvas');

            chartDiv.append(chart.chartCanvas);
            chartWrapper.append(chartDiv);
        }

        chart.chartJS = new Chart(chart.chartCanvas, {
            type: sensorConfig[chartID].type,
            data: {
                labels: sensorData[chartID].labels,
                datasets: [{
                    label: sensorConfig[chartID].label,
                    data: sensorData[chartID].values,
                    borderColor: sensorConfig[chartID].borderColor,
                    backgroundColor: sensorConfig[chartID].backgroundColor,
                }]
            }
        });

        charts[chartID] = chart;
    }
}

const domContentLoadedHandler = event => {

    granularitySelect = document.getElementById('js-granularity-select');
    initHeaderBackground();
    fetchData();
    granularitySelect.addEventListener('change', granularityChange);
}

const initHeaderBackground = () => {

    const canvasWrapper = document.getElementById('js-header-canvas');

    if (!canvasWrapper || window.innerWidth < 768) {
        return;
    }

    VANTA.NET({
        el: canvasWrapper,
        mouseControls: true,
        touchControls: false,
        gyroControls: false,
        scale: 1,
        color: '#39A3D1',
        backgroundColor: '#2869B0',
        points: 10,
        spacing: 15,
        maxDistance: 30,
        THREE: THREEv140,
        minHeight: 1000,
        minWidth: 1000,
        mouseCoeffX: .05,
        mouseCoeffY: .05
    });
};

document.addEventListener('DOMContentLoaded', domContentLoadedHandler);