#!/usr/bin/env python3
"""Run the real xgc-rt-host on a Core-generated lightweight plant manifest and
record the ROS graph the plant creates; compare two such graphs.

It starts a ROS master of its own (`--rosmaster`, on `--port`), renders the
host manifest (JSON from `xgc-lightweight-manifest`, the same projection the
station launcher renders) into TOML with the plugin paths, pins their digests,
runs the host for `--seconds` after the shared epoch, reads the master's
published topics with their types, subscribed topics and services, then stops
the host with SIGTERM. Nothing else runs: no Gazebo, SITL, MAVROS or controller.

  plant_graph_live.py run --manifest m.json --host xgc-rt-host \\
      --role lightweight_vehicle=liblightweight_vehicle.so \\
      --role ros_sim_edge=libros_sim_edge.so \\
      --rosmaster $ROS_PREFIX/bin/rosmaster --port 11411 --output-dir out
  plant_graph_live.py compare out-per-robot/graph.json out-batch/graph.json

`compare` exits 0 when both runs published exactly the same (topic, type) set,
subscribed the same topics and served the same services.
"""
import argparse
import hashlib
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client
from pathlib import Path

BARE = re.compile(r"^[A-Za-z0-9_-]+$")


def key(name):
    return name if BARE.match(name) else json.dumps(name)


def value(item, in_array=False):
    if isinstance(item, bool):
        return "true" if item else "false"
    if isinstance(item, int):
        return repr(float(item)) if in_array else str(item)
    if isinstance(item, float):
        return repr(item)
    if isinstance(item, str):
        return json.dumps(item)
    if isinstance(item, list):
        return "[" + ", ".join(value(v, True) for v in item) + "]"
    if isinstance(item, dict):
        return "{ " + ", ".join(key(k) + " = " + value(v) for k, v in item.items()) + " }"
    raise SystemExit("unsupported manifest value %r" % (item,))


def render(manifest):
    lines = []

    def entries(table):
        for name, item in table.items():
            lines.append(key(name) + " = " + value(item))
        lines.append("")

    for name in ("session", "transport", "audit"):
        lines.append("[" + name + "]")
        entries(manifest[name])
    for name in ("channel", "plugin"):
        for table in manifest[name]:
            lines.append("[[" + name + "]]")
            entries(table)
    return "\n".join(lines)


def run(args):
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    roles = dict(item.split("=", 1) for item in args.role)
    manifest = json.loads(Path(args.manifest).read_text())
    epoch = (time.time_ns() // 1000000 + 4000) * 1000000
    session = dict(manifest["session"], epoch_ns=epoch)
    audit = dict(manifest.get("audit", {}), dir=str(out / "audit"))
    (out / "audit").mkdir(exist_ok=True)
    plugins = []
    for plugin in manifest["plugin"]:
        plugin = dict(plugin)
        path = roles[plugin.pop("role")]
        entry = {"name": plugin.pop("name"), "path": path, "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest()}
        config = dict(plugin.pop("config"))
        if "model" in config:  # the plant plugins take the shared epoch
            config["epoch_ns"] = epoch
        entry.update(plugin)
        entry["config"] = config
        plugins.append(entry)
    (out / "node.toml").write_text(render(dict(session=session, transport=manifest["transport"], audit=audit,
                                               channel=manifest["channel"], plugin=plugins)))
    env = dict(os.environ, ROS_MASTER_URI="http://127.0.0.1:%d" % args.port, ROS_IP="127.0.0.1",
               ROS_HOME=str(out / "ros"), ROS_LOG_DIR=str(out / "ros-log"))
    master = subprocess.Popen([args.rosmaster, "--core", "-p", str(args.port)], env=env,
                              stdout=open(out / "master.log", "w"), stderr=subprocess.STDOUT)
    host = None
    try:
        for _ in range(100):
            try:
                socket.create_connection(("127.0.0.1", args.port), 0.2).close()
                break
            except OSError:
                time.sleep(0.1)
        host = subprocess.Popen([args.host, "--manifest", str(out / "node.toml")], env=env,
                                stdout=open(out / "host.out", "w"), stderr=subprocess.STDOUT)
        time.sleep(max(0.0, epoch / 1e9 - time.time()) + args.seconds)
        proxy = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
        published, subscribed, services = proxy.getSystemState("/plant_graph_live")[2]
        types = dict(proxy.getTopicTypes("/plant_graph_live")[2])
        graph = {
            "published": sorted((t, types.get(t)) for t, _ in published),
            "subscribed": sorted(t for t, _ in subscribed),
            "services": sorted(s for s, _ in services),
            "nodes": sorted({n for _, nodes in published for n in nodes}),
        }
        (out / "graph.json").write_text(json.dumps(graph, indent=1, sort_keys=True) + "\n")
    finally:
        if host is not None:
            host.send_signal(signal.SIGTERM)
            try:
                host.wait(30)
            except subprocess.TimeoutExpired:
                host.kill()
        master.terminate()
        master.wait(10)
    print("host exit %s; %d published topics, %d subscribed, %d services" %
          (host.returncode, len(graph["published"]), len(graph["subscribed"]), len(graph["services"])))
    return 0


def compare(args):
    first, second = (json.loads(Path(p).read_text()) for p in (args.first, args.second))
    status = 0
    for name in ("published", "subscribed", "services", "nodes"):
        a = {tuple(x) if isinstance(x, list) else x for x in first[name]}
        b = {tuple(x) if isinstance(x, list) else x for x in second[name]}
        if a != b:
            status = 1
            print("%s differ: only first %s; only second %s" % (name, sorted(a - b), sorted(b - a)))
        else:
            print("%s identical (%d)" % (name, len(a)))
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run")
    r.add_argument("--manifest", required=True)
    r.add_argument("--host", required=True)
    r.add_argument("--role", action="append", default=[], metavar="ROLE=PATH")
    r.add_argument("--rosmaster", required=True)
    r.add_argument("--port", type=int, default=11411)
    r.add_argument("--seconds", type=float, default=5.0)
    r.add_argument("--output-dir", required=True)
    c = sub.add_parser("compare")
    c.add_argument("first")
    c.add_argument("second")
    args = parser.parse_args()
    return run(args) if args.command == "run" else compare(args)


if __name__ == "__main__":
    sys.exit(main())
