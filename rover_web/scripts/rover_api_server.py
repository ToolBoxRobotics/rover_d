#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from flask import Flask, jsonify

app = Flask(__name__)

battery_status = "Unknown"

def battery_status_callback(msg):
    global battery_status
    battery_status = msg.data

rospy.init_node("rover_web_api")
rospy.Subscriber("/battery_status", String, battery_status_callback)

@app.route("/battery_status", methods=["GET"])
def get_battery_status():
    return jsonify({"battery_status": battery_status})

if __name__ == "__main__":
    rospy.spin()
    app.run(host="0.0.0.0", port=5000)
