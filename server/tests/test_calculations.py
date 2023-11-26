"""
Test the frontend endpoints using pytest
"""
import os, pytest, requests
from server import server
from dotenv import load_dotenv

load_dotenv()  # take environment variables from .env.

@pytest.fixture
def client():
    server.app.config['TESTING'] = True
    with server.app.test_client() as client:
        yield client

def test_calculate_iss_long_lat(client):
    """
    Test the calculate_iss_position endpoint for longitude and latitude
    Compare the results with http://api.open-notify.org/iss-now.json
    """
    res = client.get('/calculate_iss_position')

    # Check that the response is valid
    assert res.status_code == 200
    assert 'iss_position' in res.json

    # Now we want to cross validate iss location with http://api.open-notify.org/iss-now.json
    # Get the position from our app
    iss_position = res.json['iss_position']
    # Get the position from the API
    response = requests.get('http://api.open-notify.org/iss-now.json')
    data = response.json()
    # Check that the positions are within the same 0.5 degree
    assert abs(iss_position['latitude'] - float(data['iss_position']['latitude'])) < 0.5
    assert abs(iss_position['longitude'] - float(data['iss_position']['longitude'])) < 0.5

def test_calculate_iss_azi_ele(client):
    """
    Test the calculate_iss_position endpoint for azimuth and elevation
    Compare the results with https://www.n2yo.com/api/
    """
    res = client.get('/calculate_iss_position')

    # Check that the response is valid
    assert res.status_code == 200
    assert 'iss_position' in res.json

    # Now we want to cross validate iss location with http://api.open-notify.org/iss-now.json
    # Get the position from our app
    iss_position = res.json['iss_position']
    user_position = res.json['user_location']

    # Get the position from the n2yo API
    # use the .env file to store the api key
    api_key = os.environ.get('N2YO_API_KEY')
    sat_id = 25544 # ISS satellite ID
    seconds = 1
    # request to get future position by 1 second
    response = requests.get(f'https://api.n2yo.com/rest/v1/satellite/positions/{sat_id}/{user_position["latitude"]}/{user_position["longitude"]}/0/{seconds}/&apiKey={api_key}')
    data = response.json()

    # Check that the positions are within the same 0.5 degree
    assert abs(iss_position['azimuth'] - float(data['positions'][0]['azimuth'])) < 0.5
    assert abs(iss_position['elevation'] - float(data['positions'][0]['elevation'])) < 0.5
    