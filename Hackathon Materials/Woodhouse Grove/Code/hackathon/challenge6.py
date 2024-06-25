import requests
import random
import time

# Constants
CORE_URL = 'https://api.core.aql.com/v1/'
TOKEN = 'YOUR BEARER TOKEN'
DEVICE_ID = 'YOUR DEVICE ID'

def generate_reading():
    """
    Generate a random reading.
    """
    timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())  # Current timestamp in UTC format
    latitude = round(random.uniform(-90, 90), 4)  # Random latitude between -90 and 90
    longitude = round(random.uniform(-180, 180), 4)  # Random longitude between -180 and 180
    temperature = round(random.uniform(-20, 40), 1)  # Random temperature between -20 and 40
    humidity = random.randint(0, 100)  # Random humidity between 0 and 100
    co2 = random.randint(300, 1000)  # Random CO2 level between 300 and 1000 ppm
    pm25 = random.randint(0, 50)  # Random PM2.5 level between 0 and 50 µg/m³
    pm10 = random.randint(0, 100)  # Random PM10 level between 0 and 100 µg/m³

    return {"reading": {
        "timestamp": timestamp,
        "latitude": latitude,
        "longitude": longitude,
        "temperature": temperature,
        "humidity": humidity,
        "co2": co2,
        "pm25": pm25,
        "pm10": pm10
        }
    }

def submit_reading(payload):
    url = f"{CORE_URL}devices/{DEVICE_ID}/add-reading"
    headers = {'Authorization': f'Bearer {TOKEN}', 'Content-Type': 'application/json'}
    # response = requests.post(url, json=payload, headers=headers)
    # response.raise_for_status()
    try:
        response = requests.post(url, json=payload, headers=headers)
        response.raise_for_status()
        print("Reading submitted successfully:", payload)
    except requests.exceptions.RequestException as e:
        print("Error submitting reading:", e)

import requests
import random
import time

# Constants
CORE_URL = 'https://api.core.aql.com/v1/'
TOKEN = 'YOUR BEARER TOKEN'
DEVICE_ID = 'YOUR DEVICE ID'

def generate_reading():
    """ Generate a random reading. """
    timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())  # Current timestamp in UTC format
    latitude = round(random.uniform(-90, 90), 4)  # Random latitude between -90 and 90
    longitude = round(random.uniform(-180, 180), 4)  # Random longitude between -180 and 180
    temperature = round(random.uniform(-20, 40), 1)  # Random temperature between -20 and 40
    humidity = random.randint(0, 100)  # Random humidity between 0 and 100
    co2 = random.randint(300, 1000)  # Random CO2 level between 300 and 1000 ppm
    pm25 = random.randint(0, 50)  # Random PM2.5 level between 0 and 50 µg/m³
    pm10 = random.randint(0, 100)  # Random PM10 level between 0 and 100 µg/m³
    return {
        "reading": {
            "timestamp": timestamp,
            "latitude": latitude,
            "longitude": longitude,
            "temperature": temperature,
            "humidity": humidity,
            "co2": co2,
            "pm25": pm25,
            "pm10": pm10
        }
    }

def submit_reading(payload):
    url = f"{CORE_URL}devices/{DEVICE_ID}/add-reading"
    headers = {'Authorization': f'Bearer {TOKEN}', 'Content-Type': 'application/json'}
    try:
        response = requests.post(url, json=payload, headers=headers)
        response.raise_for_status()
        print("Reading submitted successfully:", payload)
    except requests.exceptions.RequestException as e:
        print("Error submitting reading:", e)

def generate_and_submit_readings():
    """ Generate and submit readings every 5 minutes. """
    while True:
        reading = generate_reading()
        submit_reading(reading)
        time.sleep(300)  # Sleep for 300 seconds (5 minutes)

# Test the function
if __name__ == "__main__":
    generate_and_submit_readings()
