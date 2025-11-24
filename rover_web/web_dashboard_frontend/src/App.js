import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";

const App = () => {
  const [ros, setRos] = useState(null);
  const [batteryStatus, setBatteryStatus] = useState("Unknown");

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: "ws://localhost:9090"
    });
    setRos(rosInstance);

    // Subscribe to battery status topic
    const batteryStatusListener = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/battery_status',
      messageType: 'std_msgs/String'
    });
    batteryStatusListener.subscribe((message) => {
      setBatteryStatus(message.data);
    });

    return () => {
      batteryStatusListener.unsubscribe();
    };
  }, []);

  return (
    <div>
      <h1>Rover Web Dashboard</h1>
      <h3>Battery Status: {batteryStatus}</h3>
    </div>
  );
};

export default App;
