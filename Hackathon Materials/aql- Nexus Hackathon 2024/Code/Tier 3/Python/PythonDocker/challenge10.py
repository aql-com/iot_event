import requests

# Constants
CORE_URL = "https://api.core.aql.com/v1/"
TOKEN = "YOUR BEARER TOKEN"
SENSOR_IDS = ["YOUR_SENSOR_ID", "YOUR_SENSOR_ID"]

def get_reading(sensor_id):
    url = f"{CORE_URL}sensors/{sensor_id}/sensor-data/latest"
    headers = {"Authorization": f"Bearer {TOKEN}"}
    try:
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        return response.json()
    except requests.exceptions.HTTPError as e:
        print(e)


# Get data from each station
for sensor_id in SENSOR_IDS:
    reading = get_reading(sensor_id)
    print(reading)

print("Challenge Complete")
