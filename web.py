from flask import Flask, render_template, request, jsonify, session
import json

with open('sat_config_ex.json', 'r') as f:
    ex_data = json.load(f)

app = Flask(__name__)
app.secret_key = 'your-secret-key'

@app.route("/")
def index():
    satellites = ex_data.get('satellites', [])
    groundStations = ex_data.get('groundStations', [])
    return render_template("index.html", satellites=json.dumps(satellites), groundStations=json.dumps(groundStations))

@app.route("/satelliteBuilder")
def satelliteBuilder():
    return render_template("satelliteBuilder.html")


@app.route("/launch-sim", methods=['POST'])
def launch_sim():
    try:
        data = request.get_json()
        satellites = data.get('satellites', [])
        groundStations = data.get('groundStations', [])
        session['satellite_data'] = satellites
        session['groundStations_data'] = groundStations
        return jsonify({"status": "success", 
            "message": f"Received {len(satellites)} satellites, {len(groundStations)} groundStations"})
    
    except Exception as e:
        print(f"Error processing launch sim data: {e}")
        return jsonify({"status": "error", "message": str(e)}), 400


@app.route("/runModel")
def runModel():
    satellites = session.get('satellite_data', [])
    groundStations = session.get('groundStations_data', [])
    return render_template("runModel.html", satellites=json.dumps(satellites), groundStations=json.dumps(groundStations))


@app.route("/configModel")
def configModel():
    return render_template("configModel.html")


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)