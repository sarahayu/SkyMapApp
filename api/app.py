from flask import Flask, jsonify, request
import skymap
import time

app = Flask(__name__)
app.config["DEBUG"] = True
app.config['JSONIFY_PRETTYPRINT_REGULAR'] = False

starmap = skymap.Starmap(min_mag = 5)


@app.route('/getmap', methods=['POST'])
def get_map():
    request_data = request.get_json()
    fov = request_data["fov"]
    mag = request_data["mag"]
    utc_offset = request_data["utc_offset"]
    lon = request_data["lon"]
    lat = request_data["lat"]
    az = request_data["az"]
    alt = request_data["alt"]
    roll = request_data["roll"]
    compact = request_data["compact"]
    map_data = skymap.get_stars(
        map_instance=starmap,
        fov=fov,
        mag=mag,
        utc=int(time.time()) + utc_offset,
        lon=lon,
        lat=lat,
        az=az,
        alt=alt,
        roll=roll,
        compact=compact)

    # compact representation is plain string, return that
    if compact:
        return map_data

    # if not compact, we have to jsonify resulting map object
    return jsonify(map_data)

# app.run()