from flask import Flask, jsonify
import serial
import time
import requests

app = Flask(__name__)

# Setup the serial connection to Arduino - replace '/dev/ttyACM0' with your actual COM port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # Wait for the connection to settle

@app.route('/get_sensor_data')
def get_sensor_data():
    """
    Get the current position from the Arduino
    Also get the raw stepper motors movement angle so the frontend can display it
    """
    try:
        # Request data from Arduino
        ser.write(b'get_coords\n')
        time.sleep(2)  # Give Arduino time to respond
        data = ser.readline().decode('utf-8').rstrip()
        
        # Parse the CSV data from Arduino
        lat, lng, motor1, motor2 = data.split(",")
        coords = {
            'latitude': float(lat),
            'longitude': float(lng),
            'motor1': float(motor1),
            'motor2': float(motor2)
        }
        return jsonify(coords)
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
        # Request data from Open Notify API
        response = requests.get('http://api.open-notify.org/iss-now.json')
        ## TODO: use sensor gps data to request from API
        data = response.json()
        coords = {
            'latitude': float(data['iss_position']['latitude']),
            'longitude': float(data['iss_position']['longitude'])
        }
        return jsonify(coords)
    except requests.exceptions.RequestException as e:
        return jsonify({'error': f'Open Notify API error: {e}'}), 500 
    
@app.route('/send_position_to_esp')
def send_iss_location_to_esp():
    """
    Get the current position of the ISS from Open Notify API
    and send it to the ESP32
    """
    res = get_iss_location()
    if res.status_code == 200:
        data = res.json()
        ser.write(f"set_coords,{data['latitude']},{data['longitude']}\n".encode('utf-8'))
        return jsonify({'message': 'Position sent to ESP32'})
    else:
        return jsonify({'error': 'Error getting ISS location'}), 500
    
## TODO: control panel code

if __name__ == '__main__':
    app.run(debug=True)
