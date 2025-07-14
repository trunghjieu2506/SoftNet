#!/usr/bin/env python3
# server_sdf.py
import os, tempfile, json
from flask import Flask, request, jsonify
import numpy as np

app = Flask(__name__)

def analyze_sdf(path_to_sdf):
    """
    Dummy routine: read the file and compute 3 example floats.
    Replace this with your real SDF → fp-value pipeline.
    """
    return 10

@app.route("/process_sdf", methods=["POST"])
def process_sdf():
    if "file" not in request.files:
        return "missing file part", 400

    f = request.files["file"]
    if f.filename == "":
        return "empty filename", 400

    # write to a temporary file
    with tempfile.TemporaryDirectory() as tmpdir:
        dst = os.path.join(tmpdir, f.filename)
        # f.save(dst)
        result = analyze_sdf(dst)
    return jsonify(result)  # Flask sets Content-Type: application/json

if __name__ == "__main__":
    # Make the server reachable on your LAN; change host/port if you like
    app.run(host="0.0.0.0", port=5000)
