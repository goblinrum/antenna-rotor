# Backend Server

The backend server is designed to handle some of the data flows for sending and receiving data from the ESP32 and then visualizing it through our web application. The server is built using Flask.

## Installation

To start the server, we recommend you use a virtual environment. To do this, run the following commands:

```bash
cd server # Navigate to the server directory
python3 -m venv venv # Create a virtual environment (might need to use python instead of python3)
source venv/bin/activate # Activate the virtual environment, on Windows use venv\Scripts\activate.bat
```

Once you have activated the virtual environment, you can install the dependencies using the following command:

```bash
pip install -r requirements.txt
```

## Running the Server

To run the server, you can use the following command:

```bash
python3 server.py
```

Your server should now be running on `localhost:5000`


## API Endpoints

### Frontend endpoints

All the following endpoints are useful for the visualization

#### GET /calculate_iss_position

Example Usage: `GET localhost:5000/calculate_iss_position`

Example data returned:

```json
{
  "iss_direction": {
    "radius position": [
      4134.259433906586,
      -842.3367619410968,
      5318.691187479321
    ],
    "velocity": [
      1.7966873414803193,
      7.446543563201282,
      -0.21602179880510036
    ]
  },
  "iss_position": {
    "azimuth": 313.5439866044507,
    "elevation": -25.906233040677826,
    "latitude": 51.575823932276805,
    "longitude": 152.69370100918908
  },
  "user_location": {
    "altitude": 0,
    "latitude": 37.8716,
    "longitude": -122.2728
  }
}
```

* `iss_direction` - The direction of the ISS calculated using SGP4.
  * `radius position` - The satellite position in kilometers from the center of the earth in the idiosyncratic True Equator Mean Equinox coordinate frame
  * `velocity` - The rate at which the position is changing, expressed in kilometers per second
* `iss_position` - The position of the ISS relative to the user's location
  * `azimuth` - The azimuth of the ISS relative to the user's location
  * `elevation` - The elevation of the ISS relative to the user's location
  * `latitude` - The latitude of the ISS relative to the user's location
  * `longitude` - The longitude of the ISS relative to the user's location
* `user_location` - The user's location
    * `altitude` - The altitude of the user
    * `latitude` - The latitude of the user
    * `longitude` - The longitude of the user

#### GET /predict_iss_position?start={start_time_epoch}&end={end_time_epoch}&step={step_size_seconds}

Example Usage: `GET http://localhost:5000/predict_iss_position?start=1701937618&stop=1701937698&step=10`

Example data returned (list of JSON objects with the same format as `/calculate_iss_position`): 
```json
[
  {
    "iss_direction": {
      "radius position": [
        6642.787529650961,
        350.13474505728414,
        1382.3819406969144
      ],
      "velocity": [
        0.9568378682240839,
        4.912324265934913,
        -5.802696783663077
      ]
    },
    "iss_position": {
      "azimuth": 112.43448966362624,
      "elevation": -18.913222406135727,
      "latitude": 11.73976927277929,
      "longitude": -79.25394977008122
    },
    "user_location": {
      "altitude": 0.0,
      "latitude": 37.8716,
      "longitude": -122.2728
    }
  },
  {
    "iss_direction": {
      "radius position": [
        6651.933119910364,
        399.2345711913688,
        1324.2681662056075
      ],
      "velocity": [
        0.8722555200781039,
        4.907556656864941,
        -5.819966009863101
      ]
    },
    "iss_position": {
      "azimuth": 112.60909661956845,
      "elevation": -19.279096856953092,
      "latitude": 11.239530486835788,
      "longitude": -78.87829619591969
    },
    "user_location": {
      "altitude": 0.0,
      "latitude": 37.8716,
      "longitude": -122.2728
    }
  }, ...
]
```

Calculations will run using the server's geolocation if the ESP32 is not connected or if no data is received from the ESP32.

### COM Port endpoints
TODO

### Sensor data endpoints
TODO

### Helper functions
TODO


## Testing
Currently, server functions use Pytest for testing. To run the tests, run the following command:

```bash
pytest
```

You need to make sure your `.env` file is set up correctly. You can use the `.env.example` file as a template.