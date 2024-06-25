// When the DOM content is loaded
document.addEventListener("DOMContentLoaded", function() {
    // API endpoint URL
    let sensorid = "Your Sensor ID";
    const apiUrl = `https://api.core.aql.com/v1/sensors/${sensorid}/sensor-data/latest`;

    // Bearer token
    const bearerToken = "Your Bearer  Token";

    // Fetch temperature data from the API
    fetch(apiUrl, {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${bearerToken}`
        }
    })
    .then(response => response.json())
    .then(data => {
        // Get temperature value from the API response
      
        const temperature = data.value;
    
        // Map temperature to thermometer height
        const levelHeight = mapTemperatureToHeight(temperature);

        // Update thermometer level based on temperature
        document.getElementById("level").style.height = `${levelHeight}px`;

       
    })
    .catch(error => console.error("Error fetching temperature:", error));

    // Fetch chart data
    getChartData(bearerToken, sensorid);
});

// Function to map temperature to thermometer height
function mapTemperatureToHeight(temperature) {
    const minTemperature = 0;
    const maxTemperature = 100;
    const minHeight = 0;
    const maxHeight = 270;

    const normalizedTemperature = Math.max(Math.min(temperature, maxTemperature), minTemperature);
    const heightPercentage = normalizedTemperature / maxTemperature;
    const levelHeight = maxHeight * heightPercentage;

    return levelHeight;
}

// Function to fetch chart data
const getChartData = (bearerToken, sensorid) => {
    const apiUrl = `https://api.core.aql.com/v1/sensors/${sensorid}/sensor-data/aggregate/ave`;
    const today = new Date().toISOString().slice(0, 10);
    const body = JSON.stringify({
        "startDate": `${today} 00:00:00`,
        "endDate": `${today} 23:59:59`,
        "granularity": "quarter_hour"
    });

    // Fetch chart data from the API
    fetch(apiUrl, {
        method: "POST",
        headers: {
            "Accept": "application/json",
            "Authorization":`Bearer ${bearerToken}`,
            "Content-Type": "application/json"
        },
        body: body
    })
    .then(response => {
        if (response.ok) {
            return response.json();
        } else {
            throw new Error("Failed to fetch chart data");
        }
    })
    .then(data => {
        processGraphData(data);     
    })
    .catch(error => console.error("Error fetching chart data:", error));
}


// Function to process graph data
const processGraphData = apidata => {
    console.log(apidata);

    if (Object.keys(apidata).length > 0) {
        let graphLabels = [];
        let graphValues = [];

        // Extract graph labels and values from the API response
        for (let i = 0; i < apidata.length; i++) {
            graphLabels.push(apidata[i].sensorReadingDate.substring(11, 16));
            graphValues.push(apidata[i].Average);
        }

        // Create a line chart using Chart.js
        new Chart(
            document.getElementById('chart-wrap'),
            {
                type: 'line',
                data: {
                    labels: graphLabels,
                    datasets: [
                        {
                            label: 'Temperature over time',
                            data: graphValues,
                            borderColor: '#fab400',
                            backgroundColor: '#fab400',
                        }
                    ]
                }
            }
        );
    }
}
