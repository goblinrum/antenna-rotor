import math
import time
from datetime import datetime

import ephem
import requests
import serial
from flask import Flask, jsonify, request
from sgp4.api import WGS72, Satrec, accelerated, jday
from flask_cors import CORS, cross_origin

app = Flask(__name__)
cors = CORS(app, resources={r"/*": {"origins": "*"}})

# Setup the serial connection
ser = None

# Temporary global coordinates
lat = 0
lon = 0
alt = 0
azi = 0
ele = 0

tle = None
last_updated = None

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
        global lat, lon, alt
        lat, lon, alt, azi, ele = data.split(',')
        coords = {
            'latitude': float(lat),
            'longitude': float(lon),
            'altitude': float(alt),
            'azimuth': float(azi),
            'elevation': float(ele),
        }
        return coords, 200
    except serial.SerialException as e:
        return jsonify({'error': f'Serial communication error: {e}'}), 500
    except ValueError as e:
        return jsonify({'error': f'Data parsing error: {e}\n, actual data {data}'}), 500
    
@app.route('/send_position_to_esp')
def send_iss_location_to_esp():
    """
    Get the current position of the ISS from Open Notify API
    and send it to the ESP32
    """
    id = request.args.get('id')
    res, status_code = get_iss_location(id)
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
            satname, tle_line1, tle_line2, unixtime = tle
            # sat is for SGP4 calculations
            # iss is for ephem calculations
            sat = Satrec.twoline2rv(tle_line1, tle_line2)
            iss = ephem.readtle(satname, tle_line1, tle_line2)
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
    
@app.route('/predict_iss_position')
def predict_iss_position():
    """
    Predicts the position of the ISS given a start, stop, and step time.
    Uses the same logic as calculate_iss_position when determining geolocation.
    Args:
        start: the start time in unix time
        stop: the stop time in unix time
        step: the step time in seconds

    Returns:
        A list of positions of the ISS for each step time
    """
    # read the args
    start = int(request.args.get('start'))
    stop = int(request.args.get('stop'))
    step = int(request.args.get('step'))

    try:
        # get the current geolocation
        global lat, lon, alt
        if lat == 0 and lon == 0 and alt == 0:
            lat, lon = get_geolocation()
            alt = 0
        # use ephem to calculate the position
        observer = ephem.Observer()
        observer.lat = lat
        observer.lon = lon
        observer.elevation = alt
        # get the TLE data
        tle, status_code = get_iss_location()
        if status_code == 200:
            satname, tle_line1, tle_line2, unixtime = tle
            # sat is for SGP4 calculations
            # iss is for ephem calculations
            sat = Satrec.twoline2rv(tle_line1, tle_line2)
            iss = ephem.readtle(satname, tle_line1, tle_line2)
            # use ephem to calculate relative position
            res = []
            for i in range(start, stop, step):
                # convert start time to julian date
                converted_time = datetime.fromtimestamp(i)
                observer.date = converted_time
                jd = ephem.julian_date(observer)
                fr = 0.0
                e, r, v = sat.sgp4(jd, fr)
                iss.compute(observer)
                res.append({
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
                })
            return jsonify(res), 200
        else:
            return jsonify({'error': 'Error getting ISS location'}), 500
    except Exception as e:
        return jsonify({'error': f'Error predicting ISS position: {e}'}), 500

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

@app.route('/get_iss_location')    
def get_iss_location(id = 25544):
    """
    Get the current position of the ISS. Optional param called id passed in to pass to the API.
    """
    try:
        if id:
            response = requests.get(f'https://tle.ivanstanojevic.me/api/tle/{id}')
        else:
            response = requests.get('https://tle.ivanstanojevic.me/api/tle/25544')
        # iterate through and find the ISS, then return that and the next two lines
        res = []
        response = response.json()
        res.append(response['name'])
        res.append(response['line1'])
        res.append(response['line2'])
        res.append(str(int(datetime.now().timestamp())))
        return res, 200
    except requests.exceptions.RequestException as e:
        return jsonify({'error': f'Error getting ISS location: {e}'}), 500
       
    
#################### CONTROL PANEL ####################

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False, host='0.0.0.0', port=9001)
