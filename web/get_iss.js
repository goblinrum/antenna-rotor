import axios from 'axios';

axios.get('http://wednesdays.ddns.net:9001/calculate_iss_position.ddns.net:9001/calculate_iss_position')
    .then(response => {
        // Check if the request was successful (status code 200)
        if (!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
        
        // Parse the response JSON
        return response.json();
    })
    .then(data => {
        const iss = new ISS();
        iss.set_elevation(data);
        iss.set_latitude(data);
        iss.set_longitude(data);
        iss.set_user_latitude(data);
        iss.set_user_longitude(data)
    })
    .catch(error => {
        // Handle errors during the fetch operation
        console.error('Fetch error:', error);
    });


class ISS {

    constructor() {
        this.elevation = null;
        this.latitude = null;
        this.longitude = null;
        this.user_latitude = null
        this.user_longitude = null;
    }

    set_elevation(data) {
        this.elevation = data.iss_position.elevation;
    }
    
    set_latitude(data) {
        this.latitude = data.iss_position.latitude;
    }
    
    set_longitude(data) {
        this.longitude = data.iss_position.longitude;
    }
    
    set_user_latitude(data) {
        this.user_latitude = data.user_location.latitude;
    }
    
    set_user_longitude(data) {
        this.user_longitude = data.user.location.longitude;
    }

}



async function getISSData() {
    try {
        const response = await get('http://wednesdays.ddns.net:9001/calculate_iss_position');
        
        if (!response.ok) {
            throw new Error(`HTTP error! Status: ${response.status}`);
        }
  
        const data = await response.json();
        console.log(response);

        const iss = new ISS();
        iss.set_elevation(data);
        iss.set_latitude(data);
        iss.set_longitude(data);
        iss.set_user_latitude(data);
        iss.set_user_longitude(data)

        return [iss.elevation, iss.latitude, iss.longitude, iss.user_latitude, iss.user_longitude];
    } 
    
    catch (error) {
        console.error('Fetch error:', error);
        throw error; // Re-throw the error for the caller to handle
    }
}

export { getISSData };