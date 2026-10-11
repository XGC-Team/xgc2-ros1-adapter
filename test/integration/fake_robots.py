#!/usr/bin/env python3
"""The ROS side of some vehicles, for the type server integration tests.

Publishes the topics a robot profile reads, answers the services it calls and
records what the type server publishes to the vehicle, one JSON object per
line. Configuration (JSON file, first argument):

  publish   {topic: message type}   published at `rate` Hz
  record    {topic: message type}   subscribed; every message is logged
  services  {name: service type}    answered with success
  rate      publish rate in Hz
  service_delay  seconds every service call takes (default 0)
  log       path of the record file

Prints "ready" once everything is registered.
"""

import json
import signal
import sys
import threading
import time

import genpy
import roslib.message
import rospy


def message_class(type_name):
    cls = roslib.message.get_message_class(type_name)
    if cls is None:
        raise SystemExit("unknown message type " + type_name)
    return cls


def make_message(type_name):
    cls = message_class(type_name)
    message = cls()
    if hasattr(message, "orientation"):
        message.orientation.w = 1.0
    if hasattr(message, "pose") and hasattr(message.pose, "orientation"):
        message.pose.orientation.w = 1.0
    if type_name == "std_msgs/Float32":
        message.data = 25.0
    if type_name == "std_msgs/String":
        message.data = "idle"
    if type_name == "mavros_msgs/State":
        message.connected = True
        message.armed = False
        message.mode = "POSCTL"
    if type_name == "sensor_msgs/BatteryState":
        message.voltage = 16.0
        message.percentage = 0.8
    return message


def summarize(message):
    summary = {}
    for name in ("linear", "angular"):
        value = getattr(message, name, None)
        if value is not None:
            summary[name] = [value.x, value.y, value.z]
    for name in ("data", "velocity", "yaw_rate", "position", "coordinate_frame", "type_mask"):
        value = getattr(message, name, None)
        if value is None:
            continue
        summary[name] = [value.x, value.y, value.z] if hasattr(value, "x") else value
    return summary


def main():
    with open(sys.argv[1]) as stream:
        config = json.load(stream)
    rospy.init_node("xgc_fake_robots", anonymous=True, disable_signals=True)
    # Leave the master the clean way so the next test sees no stale nodes.
    signal.signal(signal.SIGTERM, lambda *_: rospy.signal_shutdown("terminated"))
    log_lock = threading.Lock()
    log = open(config["log"], "a", buffering=1)

    def record(topic):
        def callback(message):
            entry = {"t": time.time(), "topic": topic, "msg": summarize(message)}
            with log_lock:
                log.write(json.dumps(entry) + "\n")
        return callback

    publishers = []
    for topic, type_name in sorted(config["publish"].items()):
        publishers.append((rospy.Publisher(topic, message_class(type_name), queue_size=10),
                           make_message(type_name)))
    subscribers = [rospy.Subscriber(topic, message_class(type_name), record(topic), queue_size=100)
                   for topic, type_name in sorted(config["record"].items())]
    services = []
    for name, type_name in sorted(config["services"].items()):
        service_class = roslib.message.get_service_class(type_name)

        def handle(request, name=name, service_class=service_class):
            entry = {"t": time.time(), "topic": name,
                     "msg": {"request": str(request).strip().replace("\n", ", ")}}
            with log_lock:
                log.write(json.dumps(entry) + "\n")
            time.sleep(float(config.get("service_delay", 0)))
            response = service_class._response_class()
            for field in ("success", "mode_sent"):
                if hasattr(response, field):
                    setattr(response, field, True)
            if hasattr(response, "result"):
                response.result = 0
            return response

        services.append(rospy.Service(name, service_class, handle))

    print("ready", flush=True)
    period = 1.0 / float(config.get("rate", 20))
    next_time = time.time()
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        for publisher, message in publishers:
            if hasattr(message, "header"):
                message.header.stamp = stamp
            publisher.publish(message)
        next_time += period
        delay = next_time - time.time()
        if delay > 0:
            time.sleep(delay)
        else:
            next_time = time.time()
    for subscriber in subscribers:
        subscriber.unregister()


if __name__ == "__main__":
    main()
