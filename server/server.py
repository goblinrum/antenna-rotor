from flask import Flask, jsonify
import serial
import time
import requests

app = Flask(__name__)

# Setup the serial connection
ser = None

@app.route('/connect_com')
def connect_com():
    """
    only connect to the com port when ready
    """
    try: 
        global ser
        ser = serial.Serial('COM3', 9600, timeout=1)
        time.sleep(2)  # Give Arduino time to reset
        return jsonify({'message': 'Connected to COM4'})
    except serial.SerialException as e:
        return jsonify({'error': f'Error connecting to COM: {e}'}), 500

@app.route('/disconnect_com')
def disconnect_com():
    """
    Disconnect from the com port
    """
    try:
        global ser
        ser.close()
        return jsonify({'message': 'Disconnected from COM4'})
    except serial.SerialException as e:
        return jsonify({'error': f'Error disconnecting from COM: {e}'}), 500

@app.route('/get_sensor_data')
def get_sensor_data():
    """
    Get the current position from the Arduino
    Also get the raw stepper motors movement angle so the frontend can display it
    """
    try:
        # Request data from Arduino
        ser.write(b'get_coords\n')
        time.sleep(2)  # Give Arduino time to respond and decode from bytes to string
        data = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Parse the CSV data from Arduino
        coords = {
            'data': data,
        }
        return coords, 200
    except serial.SerialException as e:
        return jsonify({'error': f'Serial communication error: {e}'}), 500
    except ValueError as e:
        return jsonify({'error': f'Data parsing error: {e}'}), 500

@app.route('/get_iss_location')
def get_iss_location():
    """
    Get the current position of the ISS from Open Notify API
    """
    try:
        response = requests.get('https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=tle')
        data = response.text.splitlines()
        res = []
        # iterate through and find the ISS, then return that and the next two lines
        for i, line in enumerate(data):
            if line.startswith('ISS (ZARYA)'):
                res.append(data[i+1].strip())
                res.append(data[i+2].strip())
                break
        return res, 200
    except requests.exceptions.RequestException as e:
        return jsonify({'error': f'Error getting ISS location: {e}'}), 500
    
@app.route('/send_position_to_esp')
def send_iss_location_to_esp():
    """
    Get the current position of the ISS from Open Notify API
    and send it to the ESP32
    """
    res, status_code = get_iss_location()
    if status_code == 200:
        # send the 3 lines of data to the ESP32
        data = res
        print(res)
        send_data = ""
        for line in data:
            send_data += line + "\n"
        ser.write(send_data.encode('utf-8'))
        return jsonify({'message': 'Position sent to ESP32'})
    else:
        return jsonify({'error': 'Error getting ISS location'}), 500
    
## TODO: control panel code

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)
