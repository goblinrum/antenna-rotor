import math
import time
from datetime import datetime

import ephem
import requests
import serial
from flask import Flask, jsonify
from sgp4.api import WGS72, Satrec, accelerated, jday

app = Flask(__name__)

# Setup the serial connection
ser = None

# Temporary global coordinates
lat = 0
lon = 0
alt = 0


#################### COM PORT CONTROLS ####################
# These are used to connect to the ESP32 via serial communication
@app.route('/connect_com')
def connect_com():
    """
    only connect to the com port when ready
    """
    try: 
        global ser
        ser = serial.Serial('COM3', 9600, timeout=1)
        time.sleep(2)  # Give time to reset
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
    
#################### SENSOR DATA FLOWS ####################
# These are used to get and send data to the ESP32
@app.route('/get_sensor_data')
def get_sensor_data():
    """
    Get the current position from the esp32
    Also get the raw stepper motors movement angle so the frontend can display it
    """
    try:
        # Request data from esp32
        ser.write(b'get_coords\n')
        time.sleep(2)  # Give esp32 time to respond and decode from bytes to string
        data = ser.readline().decode('utf-8', errors='ignore').strip()
        
        # Parse the CSV data from esp32
        coords = {
            'data': data,
        }

        global lat, lon, alt
        lat, lon, alt = data.split(',')
        return coords, 200
    except serial.SerialException as e:
        return jsonify({'error': f'Serial communication error: {e}'}), 500
    except ValueError as e:
        return jsonify({'error': f'Data parsing error: {e}'}), 500
    
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
    
#################### FRONTEND API ####################
# These are API endpoints used to serve data to the frontend
@app.route('/calculate_iss_position')
def calculate_iss_position():
    """
    Calculate the position of the ISS using the TLE data and the SGP4 library.
    Uses the global coordinates to calculate the position.
    If the lat, lon, alt are 0, then use the current server ip to determine approximate geolocation
    Currently only uses ephem for position calculations.
    SGP4 is used for velocity and radius position calculations.
    """
    try:
        # get the current geolocation
        global lat, lon, alt
        if lat == 0 and lon == 0 and alt == 0:
            lat, lon = get_geolocation()
            alt = 0
        # use ephem to calculate the position
        observer = ephem.Observer()
        observer.date = ephem.now()
        observer.lat = lat
        observer.lon = lon
        observer.elevation = alt
        # convert the current time to julian date
        jd = ephem.julian_date(observer)
        fr = 0.0
        # get the TLE data
        tle, status_code = get_iss_location()
        if status_code == 200:
            tle_line1, tle_line2 = tle
            # sat is for SGP4 calculations
            # iss is for ephem calculations
            sat = Satrec.twoline2rv(tle_line1, tle_line2)
            iss = ephem.readtle('ISS (ZARYA)', tle_line1, tle_line2)
            # use ephem to calculate relative position
            iss.compute(observer)
            e, r, v = sat.sgp4(jd, fr)
            # e is error code, r is the position relative to the equator, v is the velocity
            # see https://pypi.org/project/sgp4/ for more info
            # return the position
            return jsonify({
                'iss_position': {
                    'azimuth': math.degrees(iss.az),
                    'elevation': math.degrees(iss.alt),
                    'longitude': math.degrees(iss.sublong),
                    'latitude': math.degrees(iss.sublat),
                },
                'iss_direction': {
                    'velocity': v,
                    'radius position': r,
                },
                'user_location': {
                    'latitude': float(lat),
                    'longitude': float(lon),
                    'altitude': float(alt),
                },
            }), 200
        else:
            return jsonify({'error': 'Error getting ISS location'}), 500
    except Exception as e:
        return jsonify({'error': f'Error calculating ISS position: {e}'}), 500


#################### HELPER FUNCTIONS ####################
# These are helper functions used by the API endpoints
def get_geolocation():
    """
    Get the geolocation of the server
    """
    try:
        response = requests.get('https://ipinfo.io/')
        data = response.json()
        return data['loc'].split(',')
    except requests.exceptions.RequestException as e:
        return jsonify({'error': f'Error getting geolocation: {e}'}), 500
    
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
       
    
#################### CONTROL PANEL ####################

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)
