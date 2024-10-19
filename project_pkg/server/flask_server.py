from flask import Flask, request, jsonify

app = Flask(__name__)

# Endpoint to receive the old coordinates and modify them
@app.route('/update_coordinates', methods=['POST'])
def update_coordinates():
    data = request.get_json()  # Take the coordinate from JSON

    x_coord = data.get('X')
    y_coord = data.get('Y')

    print(f"Recieved coordinates: X {x_coord}, Y {y_coord}")

    # Move 20 meters to the right
    meters_to_shift = 20.0
    new_x = x_coord + meters_to_shift 
    new_y = y_coord
    return jsonify({"new_x": new_x, "new_y": new_y})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
