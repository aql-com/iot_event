import requests
from datetime import datetime as dt

# Constants
GOV_URL = 'http://environment.data.gov.uk/flood-monitoring'
CORE_URL = 'https://api.core.aql.com/v1/'
TOKEN = '11723|QHyGgawAlDK4w754kQ2QBvfkCdGqByDPY0dsgP7K57fe7f66'
LOCATION_ID = 'XVDiGl9Wwo'
DECODER_ID = 'BrWLtgQLr0l'
TEAM = '1'
STATION_IDS = [
    "F1902",
    "L1515",
    "F1903",
    "L1907",
    "L2009",
    "L1931",
    "L2208",
    "F2206",
    "L2205",
    "L2402",
    "L2806",
    "L2803",
    "L2411",
    "L2403"
]

def find_device(device_id, token):
    url = f"{CORE_URL}devices/from-identifier/{device_id}{TEAM}"
    headers = {'Authorization': f'Bearer {token}'}
    try:
        response = requests.get(url, headers=headers)
        response.raise_for_status()
        return response.json().get('id')
    except requests.exceptions.HTTPError as e:
        if e.response.status_code == 404:
            print(f"404 error received, couldn't find device: {device_id}")
        else:
            print(e)

def send_reading(device_id, token, payload):
    url = f"{CORE_URL}devices/{device_id}/add-reading"
    headers = {'Authorization': f'Bearer {token}', 'Content-Type': 'application/json'}
    response = requests.post(url, json=payload, headers=headers)
    response.raise_for_status()

def process_station(station_id):
    url = f"{GOV_URL}/id/stations/{station_id}/measures"
    response = requests.get(url)
    response.raise_for_status()
    json_resp = response.json()
    items = json_resp.get("items", [])
    for item in items:
        notation = item.get("notation")
        latest_reading = item.get("latestReading", {})
        if not (notation and latest_reading.get("value") and latest_reading.get("dateTime")):
            continue
        utc_dt = dt.strptime(latest_reading["dateTime"], "%Y-%m-%dT%H:%M:%SZ")
        data = {
            "reading": {
                "level": latest_reading["value"],
                "timestamp": int(utc_dt.timestamp() * 1000)
            }
        }
        print(data)
        print(notation, end=": ")
        print(latest_reading["value"], end="")
        print(item.get("unitName", ""))
        print()
        device_id = find_device(station_id, TOKEN)
        send_reading(device_id, TOKEN, data)

# Get data from each station
for station_id in STATION_IDS:
    print("Getting data from", station_id)
    process_station(station_id)

print("Challenge Complete")
