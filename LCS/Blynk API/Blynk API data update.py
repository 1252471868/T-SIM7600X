import requests
import time

# --- Configuration ---
BOX_NUM = 4
BOX_NAME = "box4"
PUMP_NUM = 1  # Which pump ESP32 to send VOC data to
if BOX_NUM == 1:
    BLYNK_AUTH_TOKEN = "iihKlmC4B_tYYOZZS68Fm9H8PUJX7Ed_" # Replace with your token
elif BOX_NUM == 2:
    BLYNK_AUTH_TOKEN = "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI" # Replace with your token
elif BOX_NUM == 3:
    BLYNK_AUTH_TOKEN = "tzqMA1jqbtyY2iCwSWi6u34KtkcQKZ0L" # Replace with your token
elif BOX_NUM == 4:
    BLYNK_AUTH_TOKEN = "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI" # Replace with your token

if PUMP_NUM == 1:
    pumpAuthToken = "jiH6wNgCtex-XP0jmEBz5iy2DEDvcrOc";
elif PUMP_NUM == 2:
    pumpAuthToken = "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI";
elif PUMP_NUM == 3:
    pumpAuthToken = "tzqMA1jqbtyY2iCwSWi6u34KtkcQKZ0L";
elif PUMP_NUM == 4:
    pumpAuthToken = "YsI-BpuhjNEWeTcLKNbNLIiY_k_ulSpI";
else:
    pumpAuthToken = "jiH6wNgCtex-XP0jmEBz5iy2DEDvcrOc"; # Default to pump 1



BLYNK_CLOUD_SERVER = "blynk.cloud"  # Or your regional server e.g., "fra1.blynk.cloud"
TOKEN_A = BLYNK_AUTH_TOKEN  # Auth token of the sending ESP32
VPIN_A_READ = "v9"                  # Virtual pin on ESP32-A to read from

TOKEN_B = pumpAuthToken     # Auth token of the receiving ESP32 (pump)
VPIN_B_WRITE = "v16"                 # Virtual pin on ESP32-B to write to

POLL_INTERVAL_SECONDS = 5            # How often to check for new data
# --- End Configuration ---

# Keep track of the last value sent to avoid redundant updates (optional)
last_value_sent_to_b = None

def get_data_from_esp32a():
    """Fetches data from ESP32-A's virtual pin via Blynk API."""
    url = f"https://{BLYNK_CLOUD_SERVER}/external/api/get?token={TOKEN_A}&{VPIN_A_READ}"
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()  # Raises an exception for bad status codes (4xx or 5xx)
        # Assuming the value is a direct number. If it's a JSON array like ["123.45"], parse it.
        # Blynk's /get API usually returns the raw value directly for single pin reads.
        value_str = response.text
        print(f"Read from ESP32-A ({VPIN_A_READ}): {value_str}")
        return float(value_str) # Or int(), or handle potential conversion errors
    except requests.exceptions.RequestException as e:
        print(f"Error reading from ESP32-A: {e}")
    except ValueError as e:
        print(f"Error converting ESP32-A value '{value_str}' to float: {e}")
    return None

def send_data_to_esp32b(value):
    """Sends data to ESP32-B's virtual pin via Blynk API."""
    global last_value_sent_to_b
    
    if value is None:
        return

    # Optional: Only send if the value has changed
    if value == last_value_sent_to_b:
        # print(f"Value {value} is the same as last sent. Skipping update to ESP32-B.")
        return

    url = f"https://{BLYNK_CLOUD_SERVER}/external/api/update?token={TOKEN_B}&{VPIN_B_WRITE}={value}"
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        print(f"Sent to ESP32-B ({VPIN_B_WRITE}): {value}. Response: {response.status_code}")
        last_value_sent_to_b = value # Update last sent value
    except requests.exceptions.RequestException as e:
        print(f"Error writing to ESP32-B: {e}")

if __name__ == "__main__":
    print("Starting Blynk Intermediary Service...")
    print(f"Polling ESP32-A ({TOKEN_A}/{VPIN_A_READ}) every {POLL_INTERVAL_SECONDS} seconds.")
    print(f"Relaying to ESP32-B ({TOKEN_B}/{VPIN_B_WRITE}).")
    
    while True:
        data_from_a = get_data_from_esp32a()
        if data_from_a is not None:
            send_data_to_esp32b(data_from_a)
        
        # Wait for the next poll interval
        time.sleep(POLL_INTERVAL_SECONDS)