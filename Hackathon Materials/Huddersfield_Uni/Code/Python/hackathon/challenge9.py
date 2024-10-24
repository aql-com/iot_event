import requests
import json

# Constants
CORE_URL = 'https://api.core.aql.com/v1/'
TOKEN = 'REPLACE_WITH_YOUR_BEARER_TOKEN'
SENSOR_IDS = [
    "REPLACE_WITH_SENSOR_ID",
    "REPLACE_WITH_SENSOR_ID"
]

def get_reading(sensor_id):
    url = f"{CORE_URL}sensors/{sensor_id}/sensor-data/latest"
    headers = {'Authorization': f'Bearer {TOKEN}'}
    try:
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.HTTPError as e:
            raise

# Get data from each station
for sensor_id in SENSOR_IDS:
    print("Getting data from", sensor_id)
    reading = get_reading(sensor_id);
    print(reading)

print("Challenge Complete")