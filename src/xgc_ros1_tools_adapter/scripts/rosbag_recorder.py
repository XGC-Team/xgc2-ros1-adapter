#!/usr/bin/python3
import os, stat, sys

# Planning and API previews execute this same recorder entry before any ROS
# import, archive access, or recorder process is created.
import json
import re


def resolve_recording_topics(document):
    parameters = document["parameters"]
    robots = document["robots"]["robots"] or []
    requested = set(parameters.get("slotIds") or [])
    all_slots = not requested
    selected = []
    for robot in robots:
        if all_slots or robot["id"] in requested:
            selected.append(robot)
            requested.discard(robot["id"])
    if requested:
        raise ValueError("ROS bag Experiment slots are outside the frozen Robot selection: " + ", ".join(sorted(requested)))
    if parameters.get("robotTopics") and not selected:
        raise ValueError("ROS bag robot topics require at least one robot in the frozen asset component")
    absolute = re.compile(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*")
    measurement_topic = re.compile(r"/[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*")
    topics = set(parameters.get("globalTopics") or [])
    for camera in parameters.get("cameras") or []:
        topics.update(camera["root"] + "/" + topic for topic in camera["topics"])
    for robot in selected:
        namespace = robot["namespace"].rstrip("/")
        if not absolute.fullmatch(namespace):
            raise ValueError("frozen robot %r has invalid ROS namespace" % robot["id"])
        topics.update(namespace + "/" + topic for topic in parameters.get("robotTopics") or [])
    if parameters.get("includeMotionCapture"):
        added = False
        mode, simulator = document["runMode"], document["simulator"]
        for robot in selected:
            if robot.get("extension") is not None and not any(robot.get(kind) for kind in ("px4", "scout", "mecanum")):
                continue
            if mode not in ("physical", "simulation", "hybrid"):
                raise ValueError("runMode %r is unsupported by Adapter localization" % mode)
            source = robot.get("hybridSource", "")
            if mode == "hybrid" and source not in ("physical", "simulation"):
                raise ValueError("hybridSource %r is unsupported" % source)
            profile = robot["profileId"]
            for kind in ("px4-multirotor", "scout-mini", "mecanum-ugv"):
                if profile in (kind + ".physical.vrpn", kind + ".gazebo.vrpn", kind + ".xsim.ros"):
                    if mode == "physical" or mode == "hybrid" and source == "physical":
                        profile = kind + ".physical.vrpn"
                    else:
                        profile = kind + (".xsim.ros" if simulator == "xsim" else ".gazebo.vrpn")
                    break
            runtime = dict(robot.get("runtimeParameters") or {})
            if robot.get("simulationPoseTopic"):
                runtime["localization_pose_topic"] = robot["simulationPoseTopic"]
            if robot.get("simulationTwistTopic"):
                runtime["localization_twist_topic"] = robot["simulationTwistTopic"]
            if profile in ("px4-multirotor.xsim.ros", "scout-mini.xsim.ros", "mecanum-ugv.xsim.ros"):
                pose = runtime.get("localization_pose_topic") or robot["namespace"].rstrip("/") + "/pose"
                twist = runtime.get("localization_twist_topic") or robot["namespace"].rstrip("/") + "/twist"
            else:
                root = "/vrpn_client_node"
                if profile in ("px4-multirotor.gazebo.vrpn", "scout-mini.gazebo.vrpn", "mecanum-ugv.gazebo.vrpn"):
                    body = robot["namespace"].lstrip("/")
                elif profile in ("px4-multirotor.physical.vrpn", "scout-mini.physical.vrpn", "mecanum-ugv.physical.vrpn", "px4.mocap-rotor.ros1.v1"):
                    body = runtime.get("mocap_rigid_body", "")
                    if "mocap_rigid_body" not in runtime:
                        # Preview carries the same frozen asset, before private
                        # delivery parameters are projected for a target.
                        for kind in ("px4", "scout", "mecanum"):
                            if robot.get(kind) is not None:
                                body = robot[kind].get("mocapRigidBodyName", "")
                                break
                else:
                    raise ValueError("Profile %r has no robot measurement projection" % profile)
                if not body or "/" in body:
                    raise ValueError("robot %s has no valid measurement rigid body" % robot["id"])
                if mode == "hybrid":
                    root += "_" + source
                pose, twist = root + "/" + body + "/pose", root + "/" + body + "/twist"
            if not measurement_topic.fullmatch(pose) or not measurement_topic.fullmatch(twist):
                raise ValueError("robot %s has invalid localization topics" % robot["id"])
            topics.update((pose, twist))
            added = True
        if not added:
            raise ValueError("robot measurement was requested but the selected asset scope declares no rigid bodies")
    for excluded in parameters.get("excludedTopics") or []:
        if excluded["topic"] in topics:
            raise ValueError("ROS bag topic %r is both recorded and excluded" % excluded["topic"])
    if not 1 <= len(topics) <= 512:
        raise ValueError("ROS bag resolved topic count must be between 1 and 512")
    for topic in topics:
        if len(topic.encode("utf-8")) > 1024 or not absolute.fullmatch(topic):
            raise ValueError("resolved ROS bag topic %r is invalid" % topic)
    if sum(len(topic.encode("utf-8")) for topic in topics) > 32768:
        raise ValueError("ROS bag resolved topics exceed 32768 bytes")
    return {"slotIds": [robot["id"] for robot in selected], "topics": sorted(topics)}


# Preparation belongs to this recorder, shared by preview and recording.
import copy
import hashlib
import math


def json_bytes(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")


def read_request():
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate field " + key)
            result[key] = value
        return result
    data = sys.stdin.buffer.read((16 << 20) + 1)
    if len(data) > 16 << 20:
        raise ValueError("frozen recorder request exceeds 16 MiB")
    document = json.loads(data, object_pairs_hook=unique)
    if not isinstance(document, dict) or set(document) != {"parameters", "robots", "context"}:
        raise ValueError("recorder request requires parameters, robots and context")
    if not all(isinstance(document[key], dict) for key in document):
        raise ValueError("recorder request fields must be objects")
    return document


def normalize_parameters(value, context, preview):
    p = copy.deepcopy(value)
    fields = {"worldBoundary", "recordAllTopics", "slotIds", "robotTopics", "globalTopics", "recordingProfile", "cameras", "expectedDurationMinutes", "estimatedVideoBitrateMbps", "capacitySafetyFactor", "includeMotionCapture", "excludedTopics", "sessionId", "experimentResourceId", "experimentCommitId", "localizationOffset", "calibrationFiles", "factParameters", "algorithmSidecarFiles", "splitSizeMiB", "maxSplits", "minFreeSpaceGiB", "compression", "repeatLatched", "rosInstallPath", "rosMasterUri", "rosIp", "rosHostname"}
    if set(p) - fields:
        raise ValueError("unknown recorder parameters: " + ", ".join(sorted(set(p) - fields)))
    for name in ("sessionId", "experimentResourceId", "experimentCommitId", "localizationOffset", "worldBoundary"):
        if name in context:
            if name in p and p[name] != context[name]:
                raise ValueError(name + " disagrees with the frozen Session context")
            p[name] = copy.deepcopy(context[name])
    for name in ("rosInstallPath", "rosMasterUri", "rosIp", "rosHostname", "compression", "recordingProfile", "sessionId", "experimentResourceId", "experimentCommitId"):
        if name in p:
            if not isinstance(p[name], str):
                raise ValueError(name + " must be a string")
            p[name] = p[name].strip()
    profile = (p.get("recordingProfile") or "general").lower()
    if profile not in ("general", "camera_scientific"):
        raise ValueError("recordingProfile must be general or camera_scientific")
    p["recordingProfile"] = profile
    scientific = profile == "camera_scientific"
    for name, default in (("splitSizeMiB", 5120 if scientific else 1024), ("maxSplits", 2 if scientific else 10), ("minFreeSpaceGiB", 2), ("expectedDurationMinutes", 60), ("estimatedVideoBitrateMbps", 24 if scientific else 0), ("capacitySafetyFactor", 1.25), ("compression", "none" if scientific else "lz4"), ("repeatLatched", True), ("recordAllTopics", False), ("includeMotionCapture", False), ("rosInstallPath", "/opt/ros/noetic"), ("rosMasterUri", "http://127.0.0.1:11311"), ("rosIp", ""), ("rosHostname", "")):
        if name not in p or name in ("splitSizeMiB", "maxSplits", "minFreeSpaceGiB", "expectedDurationMinutes", "estimatedVideoBitrateMbps", "capacitySafetyFactor", "compression", "rosInstallPath", "rosMasterUri") and p[name] in (0, ""):
            p[name] = default
    for name in ("recordAllTopics", "includeMotionCapture", "repeatLatched"):
        if not isinstance(p[name], bool):
            raise ValueError(name + " must be boolean")
    for name, low, high in (("splitSizeMiB", 64, 102400), ("maxSplits", 1, 1000), ("minFreeSpaceGiB", 1, 1024), ("expectedDurationMinutes", 1, 10080)):
        if type(p[name]) is not int or not low <= p[name] <= high:
            raise ValueError(name + " is outside its storage bound")
    for name, low, high in (("estimatedVideoBitrateMbps", 0, 10000), ("capacitySafetyFactor", 1, 4)):
        if type(p[name]) not in (int, float) or not math.isfinite(p[name]) or not low <= p[name] <= high:
            raise ValueError(name + " is outside its finite bound")
    p["compression"] = p["compression"].lower()
    if p["compression"] not in ("none", "lz4", "bz2") or scientific and p["compression"] != "none":
        raise ValueError("invalid compression for the recording profile")
    if not os.path.isabs(p["rosInstallPath"]) or os.path.normpath(p["rosInstallPath"]) != p["rosInstallPath"] or "\x00" in p["rosInstallPath"]:
        raise ValueError("rosInstallPath must be a clean absolute path")
    import urllib.parse
    uri = urllib.parse.urlsplit(p["rosMasterUri"])
    if uri.scheme not in ("http", "https") or not uri.hostname or uri.username or uri.password or uri.query or uri.fragment:
        raise ValueError("rosMasterUri must be an explicit ROS master URL")
    if p["rosIp"] and p["rosHostname"]:
        raise ValueError("ROS_IP and ROS_HOSTNAME are mutually exclusive")
    absolute = re.compile(r"/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*")
    relative = re.compile(r"[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*")
    def topics(name, values, pattern, maximum):
        if not isinstance(values, list) or len(values) > maximum or any(not isinstance(v, str) for v in values):
            raise ValueError(name + " exceeds its declared list bound")
        values = sorted(set(v.strip() for v in values))
        if any(len(v.encode("utf-8")) > 1024 or not pattern.fullmatch(v) for v in values):
            raise ValueError(name + " contains an invalid ROS graph name")
        return values
    p["robotTopics"] = topics("robotTopics", p.get("robotTopics", []), relative, 256)
    p["globalTopics"] = topics("globalTopics", p.get("globalTopics", ["/tf", "/tf_static", "/clock"]), absolute, 256)
    p["slotIds"] = topics("slotIds", p.get("slotIds", []), re.compile(r"[a-z][a-z0-9-]{0,63}"), 256)
    cameras = p.setdefault("cameras", [])
    if not isinstance(cameras, list) or len(cameras) > 32:
        raise ValueError("cameras exceeds its declared list bound")
    seen = set()
    for camera in cameras:
        if not isinstance(camera, dict) or set(camera) != {"root", "topics"}:
            raise ValueError("camera requires root and topics")
        camera["root"] = topics("camera root", [camera["root"]], absolute, 1)[0]
        if camera["root"] in seen:
            raise ValueError("duplicate camera root")
        seen.add(camera["root"])
        camera["topics"] = topics("camera topics", camera["topics"], relative, 32)
        if not camera["topics"]:
            raise ValueError("camera must explicitly name topics")
    p["cameras"] = sorted(cameras, key=lambda c: c["root"])
    if scientific != bool(cameras):
        raise ValueError("camera_scientific requires authored cameras; general uses globalTopics")
    if not p["robotTopics"] and not p["globalTopics"] and not p["includeMotionCapture"] and not scientific:
        raise ValueError("recorder requires explicit topic selection")
    if scientific and not preview and (not p.get("sessionId") or "localizationOffset" not in p):
        raise ValueError("scientific recording requires frozen sessionId and localizationOffset")
    for name, maximum in (("sessionId", 64), ("experimentResourceId", 128), ("experimentCommitId", 128)):
        if name in p and len(p[name].encode("utf-8")) > maximum:
            raise ValueError(name + " exceeds its identity bound")
    if "localizationOffset" in p:
        offset = p["localizationOffset"]
        if not isinstance(offset, dict) or set(offset) != {"x", "y", "z"} or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1000000 for v in offset.values()):
            raise ValueError("localizationOffset must contain finite XYZ meters within 1e6")
    boundary = p.get("worldBoundary")
    if boundary is not None and not isinstance(boundary, dict) or len(json_bytes(boundary)) > 65536:
        raise ValueError("worldBoundary must preserve the bounded frozen object or null")
    for name, extensions in (("calibrationFiles", (".yaml", ".yml")), ("algorithmSidecarFiles", (".mat", ".yaml", ".yml"))):
        files = p.setdefault(name, [])
        if not isinstance(files, list) or len(files) > 32:
            raise ValueError(name + " exceeds its declared bound")
        roles = set()
        for file in files:
            if not isinstance(file, dict) or set(file) != {"role", "path"}:
                raise ValueError(name + " requires role and path")
            role, path = file["role"].strip(), file["path"].strip()
            if not re.fullmatch(r"[a-z][a-z0-9-]{0,63}", role) or role in roles or not os.path.isabs(path) or os.path.normpath(path) != path or "\x00" in path or os.path.splitext(path)[1].lower() not in extensions:
                raise ValueError("invalid or duplicate sidecar declaration")
            roles.add(role)
            file.update(role=role, path=path)
    excluded = p.setdefault("excludedTopics", [])
    if not isinstance(excluded, list) or len(excluded) > 64:
        raise ValueError("excludedTopics exceeds its declared bound")
    seen = set()
    for excluded_topic in excluded:
        if not isinstance(excluded_topic, dict) or set(excluded_topic) != {"topic", "reason"}:
            raise ValueError("excluded topic requires topic and reason")
        topic = topics("excluded topic", [excluded_topic["topic"]], absolute, 1)[0]
        reason = excluded_topic["reason"].strip()
        if topic in seen or not reason or len(reason.encode("utf-8")) > 512:
            raise ValueError("duplicate excluded topic or invalid reason")
        excluded_topic.update(topic=topic, reason=reason)
        seen.add(topic)
    p["excludedTopics"] = sorted(excluded, key=lambda v: v["topic"])
    facts = p.setdefault("factParameters", [])
    if not isinstance(facts, list) or len(facts) > 64:
        raise ValueError("factParameters exceeds its declared bound")
    seen = set()
    for fact in facts:
        if not isinstance(fact, dict) or not {"name", "purpose"} <= set(fact) or set(fact) - {"name", "purpose", "requestedValue"}:
            raise ValueError("invalid fact parameter declaration")
        name = topics("fact parameter", [fact["name"]], absolute, 1)[0]
        purpose = fact["purpose"].strip()
        if name in seen or not purpose or len(purpose.encode("utf-8")) > 512 or "requestedValue" in fact and len(json_bytes(fact["requestedValue"])) > 65536:
            raise ValueError("invalid or duplicate fact parameter")
        seen.add(name)
    estimated = math.ceil(p["expectedDurationMinutes"] * 60 * p["estimatedVideoBitrateMbps"] * 1000000 / 8 * p["capacitySafetyFactor"])
    if not scientific and estimated > p["splitSizeMiB"] * 1048576 * p["maxSplits"]:
        raise ValueError("planned split capacity is smaller than the estimated session")
    return p, estimated


def prepare(document, preview=False):
    p, estimated = normalize_parameters(document["parameters"], document["context"], preview)
    robots, context = document["robots"], document["context"]
    for key in ("experimentResourceId", "experimentCommitId"):
        if p.get(key) and p[key] != robots.get(key):
            raise ValueError(key + " does not match the frozen robot selection")
    resolved = resolve_recording_topics({"parameters": p, "robots": robots, "runMode": context["runMode"], "simulator": (context.get("scene") or {}).get("simulator", "")})
    sidecar = []
    for robot in robots.get("robots") or []:
        item = {key: copy.deepcopy(robot.get(key)) for key in ("id", "kind", "namespace", "hybridSource", "robotAssetId", "robotAssetCommitId", "robotAssetDigest", "profileId", "initialPose", "visualization")}
        item["visualizationDigest"] = hashlib.sha256(json_bytes(item["visualization"])).hexdigest()
        for kind in ("px4", "scout", "mecanum"):
            if robot.get(kind) is not None:
                body = robot[kind].get("mocapRigidBodyName", "").strip()
                if body:
                    item["mocapRigidBodyName"] = body
                break
        sidecar.append(item)
    manifest = {"schemaVersion": 1, "recordNamingVersion": 1, "experimentName": context["experimentName"], "runMode": context["runMode"], "recordingProfile": p["recordingProfile"], "recordAllTopics": p["recordAllTopics"], "includeMotionCapture": p["includeMotionCapture"], "experimentResourceId": robots["experimentResourceId"], "experimentCommitId": robots["experimentCommitId"], "experimentDigest": robots["experimentDigest"], "robotSelectionDigest": robots["robotSelectionDigest"], "slotIds": resolved["slotIds"], "cameras": p["cameras"], "cameraTopicRoots": [c["root"] for c in p["cameras"]], "excludedTopics": p["excludedTopics"], "topics": resolved["topics"], "expectedDurationMinutes": p["expectedDurationMinutes"], "estimatedVideoBitrateMbps": p["estimatedVideoBitrateMbps"], "capacitySafetyFactor": p["capacitySafetyFactor"], "estimatedBytes": estimated, "robots": sidecar, "requestEvidence": "workflow-bound-request-not-producer-applied", "requestedParameters": document["parameters"], "factSources": {name: p[name] for name in ("calibrationFiles", "algorithmSidecarFiles")}, "timestampContract": {"imageTime": "xgc_camera_msgs/FrameTiming.source_time", "sceneTime": "ROS message header stamp or /clock", "alignment": "immutable sidecar; source bags are never rewritten"}}
    manifest["factSources"]["parameters"] = p["factParameters"]
    for key in ("sessionId", "localizationOffset", "worldBoundary"):
        if key in p:
            manifest[key] = p[key]
    if p["recordingProfile"] == "camera_scientific":
        manifest["scientificTotalLimitBytes"] = 10 * 1024 ** 3
    return p, resolved, manifest


def record_arguments(document):
    p, resolved, manifest = prepare(document)
    archive = json.loads(os.environ["XGC_RECORD_CONTEXT_JSON"])
    context = document["context"]
    if archive.get("schemaVersion") != 1 or archive.get("category") != "Data" or archive.get("extension") != "bag" or context.get("runId") != archive.get("workflowRunId") or context.get("rootRunId") != archive.get("rootRunId") or context.get("targetId") != archive.get("targetId"):
        raise ValueError("recorder archive does not belong to the executing frozen Run")
    if archive.get("experimentId") != manifest["experimentResourceId"] or archive.get("experimentCommitId") != manifest["experimentCommitId"] or archive.get("experimentDigest") != manifest["experimentDigest"] or archive.get("runMode") != manifest["runMode"]:
        raise ValueError("recorder archive disagrees with the frozen Experiment")
    os.environ["ROS_MASTER_URI"] = p["rosMasterUri"]
    for name, field in (("ROS_IP", "rosIp"), ("ROS_HOSTNAME", "rosHostname")):
        if p[field]: os.environ[name] = p[field]
        else: os.environ.pop(name, None)
    output = archive["outputPath"]
    if not output.endswith(".bag") or os.path.dirname(output) != archive["outputDirectory"] or archive["outputDirectory"] != os.path.join(archive["recordDirectory"], "Data"):
        raise ValueError("recorder archive paths disagree")
    manifest.update(recordingId=archive["recordId"], recordPlannedAt=archive["plannedAt"], runId=archive["workflowRunId"], rootRunId=archive["rootRunId"], invocationId=archive["invocationId"], parentRunId=archive.get("parentRunId", ""), definitionId=archive["definitionId"], definitionDigest=archive["definitionDigest"], nodeId=archive["nodeId"])
    return [sys.argv[0], p["rosInstallPath"], output[:-4], json.dumps(resolved["topics"]), str(p["splitSizeMiB"]), str(p["maxSplits"]), str(p["minFreeSpaceGiB"]), p["compression"], str(p["repeatLatched"]).lower(), str(p["expectedDurationMinutes"]), str(p["estimatedVideoBitrateMbps"]), str(p["capacitySafetyFactor"]), json_bytes(manifest).decode("utf-8")]


try:
    if sys.argv[1:] == ["--prepare"]:
        _, resolved, manifest = prepare(read_request(), preview=True)
        sys.stdout.buffer.write(json_bytes(dict(resolved, estimatedBytes=manifest["estimatedBytes"])))
        raise SystemExit(0)
    if sys.argv[1:] != ["--record"]:
        raise ValueError("usage: xgc_ros1_tools_adapter_rosbag_recorder --prepare|--record")
    sys.argv = record_arguments(read_request())
except (ValueError, TypeError, KeyError) as error:
    sys.stderr.write(str(error) + "\n")
    raise SystemExit(2)


def require_archive_path(path, playback=False):
    user_root = os.environ.get("XGC_USER_FILES_DIR", "")
    if not user_root or not os.path.isabs(user_root) or os.path.normpath(user_root) != user_root or user_root == "/":
        raise SystemExit("XGC_USER_FILES_DIR must select a clean absolute operator archive")
    archive = os.path.join(user_root, "Experiments")
    if not path or path.strip() != path or not os.path.isabs(path) or os.path.normpath(path) != path:
        raise SystemExit("ROS bag path must be a clean absolute path")
    if not playback and (path.endswith("/") or os.path.basename(path) in ("", ".", "..")):
        raise SystemExit("outputPrefix must end in a file name stem")
    inspected = path if playback else os.path.dirname(path)
    if os.path.commonpath([archive, inspected]) != archive:
        raise SystemExit("ROS bag path must be inside the operator Experiments archive")
    current = "/"
    for part in inspected.split("/")[1:]:
        current = os.path.join(current, part)
        try:
            info = os.lstat(current)
        except FileNotFoundError:
            break
        if stat.S_ISLNK(info.st_mode):
            raise SystemExit("ROS bag path component is a symlink")
        if not (playback and current == path and current != archive) and not stat.S_ISDIR(info.st_mode):
            raise SystemExit("ROS bag path component must be a directory")

require_archive_path(sys.argv[2])

import datetime
import json
import math
import os
import re
import stat
import shutil
import signal
import subprocess
import sys

# Record fact evidence is subordinate to this recorder's session-manifest.
# No current configuration is promoted to a producer application receipt.
import copy
import hashlib
import queue
import threading
import multiprocessing
import time


class RecordFacts:
    TOPIC = "/xgc/record_facts"
    MAX_BYTES = 1 << 20
    MAX_EVENTS = 4096
    MAX_TOTAL_BYTES = 64 << 20
    POLL_SECONDS = 1.0

    def __init__(self, manifest, directory):
        self.directory = directory
        self.declarations = manifest.get("factSources") or {}
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.messages = queue.Queue(maxsize=256)
        self.threads = []
        self.subscribers = []
        self.previous = {}
        self.producers = {}
        self.revision = 0
        self.persisted = -1
        self.dropped = 0
        self.stored_bytes = 0
        self.stored_hashes = set()
        self.stored_paths = {}
        self.rospy = None
        self.ros_process = None
        self.sealed = False
        self.state = {
            "schemaVersion": 2,
            "contract": "xgc.record-facts/v2",
            "observationStartedAt": self.clock(),
            "observationEndedAt": None,
            "status": "observing",
            "events": [],
            "coverage": {
                "applied": "producer-reported-only",
                "snapshots": "sampled-observations-not-application",
                "beforeObservation": "unknown",
                "afterLastApplication": "end-not-observed",
                "clockAlignment": "not-inferred",
                "unreportedSources": "unknown",
                "droppedEvents": 0,
            },
        }

    @staticmethod
    def clock():
        return {"unixTimeNs": str(time.time_ns()),
                "monotonicTimeNs": str(time.monotonic_ns())}

    @staticmethod
    def encoded(value):
        return json.dumps(value, sort_keys=True, separators=(",", ":"),
                          ensure_ascii=False, allow_nan=False).encode("utf-8")

    def enqueue(self, kind, source, payload, observed=None):
        if self.closed.is_set():
            return
        try:
            self.messages.put_nowait((kind, source, payload, observed or self.clock()))
        except queue.Full:
            with self.lock:
                self.dropped += 1
                self.revision += 1

    def store(self, data, label="fact", suffix=".json"):
        if len(data) > self.MAX_BYTES:
            raise ValueError("fact-payload-too-large")
        digest = hashlib.sha256(data).hexdigest()
        if digest not in self.stored_hashes and self.stored_bytes + len(data) > self.MAX_TOTAL_BYTES:
            raise ValueError("fact-byte-budget-exhausted")
        label = re.sub(r"[^a-zA-Z0-9_]+", "-", label).strip("-")[:64] or "fact"
        suffix = suffix if re.fullmatch(r"\.[a-zA-Z0-9]{1,8}", suffix) else ".data"
        relative = self.stored_paths.get(digest)
        if relative is None:
            relative = "record-facts/%s-revision-%06d%s" % (label, len(self.stored_hashes) + 1, suffix)
        root = os.path.join(self.directory, "record-facts")
        os.makedirs(root, exist_ok=True)
        if not stat.S_ISDIR(os.lstat(root).st_mode):
            raise ValueError("fact-directory-not-regular")
        path = os.path.join(self.directory, relative)
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        except FileExistsError:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            with os.fdopen(fd, "rb") as stream:
                if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                    raise ValueError("fact-blob-not-regular")
                existing = stream.read(self.MAX_BYTES + 1)
            if existing != data:
                raise ValueError("fact-blob-conflict")
        else:
            try:
                with os.fdopen(fd, "wb") as stream:
                    stream.write(data)
                    stream.flush()
                    os.fsync(stream.fileno())
                dfd = os.open(root, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
                try:
                    os.fsync(dfd)
                finally:
                    os.close(dfd)
            except BaseException:
                # A partial blob is never referenced by the manifest.
                os.unlink(path)
                raise
        if digest not in self.stored_hashes:
            self.stored_hashes.add(digest)
            self.stored_paths[digest] = relative
            self.stored_bytes += len(data)
        return {"relativePath": relative, "sha256": digest, "sizeBytes": len(data)}

    def append(self, event):
        with self.lock:
            if self.sealed:
                return None
            events = self.state["events"]
            if len(events) >= self.MAX_EVENTS:
                self.dropped += 1
                self.revision += 1
                return None
            event["ordinal"] = len(events)
            events.append(event)
            self.revision += 1
            return event

    def observation(self, kind, source, value, observed=None):
        data = self.encoded(value)
        key = (kind, source)
        digest = hashlib.sha256(data).hexdigest()
        if self.previous.get(key) == digest:
            return
        content = self.store(data, kind + "-" + source)
        entry = self.append({"kind": kind, "source": source,
            "evidence": "snapshot-observed", "observedAt": observed or self.clock(),
            "content": content, "validUntil": None,
            "validity": "observation-only; between-sample changes are unknown"})
        if entry is not None:
            self.previous[key] = digest

    def unavailable(self, kind, source, reason):
        key = ("unavailable", kind, source)
        if self.previous.get(key) == reason:
            return
        entry = self.append({"kind": kind, "source": source, "evidence": "unavailable",
                             "observedAt": self.clock(), "reason": reason})
        if entry is not None:
            self.previous[key] = reason
            self.previous.pop((kind, source), None)

    def file(self, declaration, kind):
        source = declaration["role"]
        path = declaration["path"]
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
            with os.fdopen(fd, "rb") as stream:
                before = os.fstat(stream.fileno())
                if not stat.S_ISREG(before.st_mode):
                    raise ValueError("not-regular")
                if before.st_size > self.MAX_BYTES:
                    raise ValueError("too-large")
                data = stream.read(self.MAX_BYTES + 1)
                after = os.fstat(stream.fileno())
            identity = lambda info: (info.st_dev, info.st_ino, info.st_size,
                                     info.st_mtime_ns, info.st_ctime_ns)
            if len(data) > self.MAX_BYTES:
                raise ValueError("too-large")
            if identity(before) != identity(after):
                raise ValueError("changed-during-read")
            digest = hashlib.sha256(data).hexdigest()
            if self.previous.get((kind, source)) == digest:
                return
            content = self.store(data, kind + "-" + source, os.path.splitext(path)[1])
            event = self.append({"kind": kind, "source": source,
                "evidence": "snapshot-observed", "observedAt": self.clock(),
                "content": content, "validUntil": None,
                "validity": "file-bytes-observed; consumer-load-unknown"})
            if event is not None:
                self.previous[(kind, source)] = digest
                self.previous.pop(("unavailable", kind, source), None)
        except (OSError, ValueError) as exc:
            self.unavailable(kind, source, str(exc) if isinstance(exc, ValueError) else type(exc).__name__)

    @staticmethod
    def validate_producer(value):
        if not isinstance(value, dict) or type(value.get("schemaVersion")) is not int or value["schemaVersion"] != 1:
            raise ValueError("unsupported-producer-schema")
        source = value.get("source")
        if not isinstance(source, dict) or any(not isinstance(source.get(key), str) or
                not source[key] or len(source[key]) > 512 for key in ("id", "instanceId", "kind")):
            raise ValueError("invalid-producer-identity")
        sequence = value.get("sequence")
        if type(sequence) is not int or sequence < 1 or sequence > 9007199254740991:
            raise ValueError("invalid-producer-sequence")
        if value.get("event") not in ("applied", "stopped") or value.get("evidence") != "producer-applied":
            raise ValueError("invalid-producer-evidence")
        clock = value.get("effectiveAt")
        if not isinstance(clock, dict) or any(not isinstance(clock.get(key), str) or
                not re.fullmatch(r"[0-9]{1,30}", clock[key]) for key in
                ("rosTimeNs", "unixTimeNs", "monotonicTimeNs")):
            raise ValueError("invalid-producer-clock")
        if not isinstance(value.get("values"), dict) or not isinstance(value.get("provenance"), dict):
            raise ValueError("missing-producer-values-or-provenance")
        RecordFacts.encoded(value)
        return (source["kind"], source["id"], source["instanceId"])

    def producer(self, text, observed):
        if not isinstance(text, str) or len(text.encode("utf-8")) > self.MAX_BYTES:
            raise ValueError("fact-payload-too-large")
        def reject_constant(_value):
            raise ValueError("non-finite-producer-json")
        def unique_object(pairs):
            result = {}
            for name, item in pairs:
                if name in result:
                    raise ValueError("duplicate-producer-key")
                result[name] = item
            return result
        value = json.loads(text, parse_constant=reject_constant, object_pairs_hook=unique_object)
        key = self.validate_producer(value)
        content = self.store(text.encode("utf-8"), "application-" + value["source"]["kind"])
        previous = self.producers.get(key)
        sequence = value["sequence"]
        continuity = "first-observed" if sequence == 1 else "prior-events-unobserved"
        if previous:
            if sequence == previous["sequence"] and content["sha256"] == previous["content"]["sha256"]:
                return
            if sequence <= previous["sequence"]:
                raise ValueError("producer-sequence-replay-or-conflict")
            continuity = "sequence-gap"
            before = previous["effectiveAt"]
            after = value["effectiveAt"]
            if sequence == previous["sequence"] + 1:
                continuity = "consecutive-producer-reports"
                if int(after["monotonicTimeNs"]) < int(before["monotonicTimeNs"]):
                    continuity = "producer-clock-regression"
                elif int(after["rosTimeNs"]) < int(before["rosTimeNs"]):
                    continuity = "ros-clock-regression"
        event = {"kind": "producer", "source": value["source"],
            "sequence": sequence, "event": value["event"], "evidence": "producer-applied",
            "effectiveAt": value["effectiveAt"], "observedAt": observed,
            "content": content, "continuity": continuity,
            "validity": {"from": value["effectiveAt"], "until": None,
                "basis": "producer-reported-transition", "endReason": "end-not-observed"}}
        with self.lock:
            event = self.append(event)
            if event is None:
                return
            if previous and previous["event"] == "applied":
                if continuity == "consecutive-producer-reports":
                    previous["validity"]["until"] = value["effectiveAt"]
                    previous["validity"]["endReason"] = value["event"]
                else:
                    previous["validity"]["endReason"] = continuity
            if value["event"] == "stopped":
                event["validity"] = None
            self.producers[key] = event

    def process_messages(self):
        # Bounded work per pass keeps files/parameters from being starved by a publisher.
        for _ in range(256):
            try:
                kind, source, value, observed = self.messages.get_nowait()
            except queue.Empty:
                break
            try:
                if kind == "producer":
                    self.producer(value, observed)
                elif kind == "unavailable":
                    self.unavailable("observer", source, value)
                else:
                    self.observation(kind, source, value, observed)
            except (ValueError, TypeError, KeyError, OSError, RecursionError) as exc:
                self.unavailable("observer", source, type(exc).__name__ + ":" + str(exc)[:200])

    def poll(self):
        for field, kind in (("calibrationFiles", "calibration-file"),
                            ("algorithmSidecarFiles", "algorithm-file")):
            for declaration in (self.declarations.get(field) or []):
                if self.closed.is_set():
                    return
                self.file(declaration, kind)
        declarations = (self.declarations.get("parameters") or [])
        if not declarations:
            return
        # Exactly authored names: never getParam('/') / getParamNames / scan HOME.
        import xmlrpc.client
        class Transport(xmlrpc.client.Transport):
            def make_connection(self, host):
                connection = super().make_connection(host)
                connection.timeout = 0.25
                return connection
        for declaration in declarations:
            if self.closed.is_set():
                return
            name = declaration["name"]
            try:
                with xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"], transport=Transport()) as master:
                    code, _message, value = master.getParam("/xgc_record_facts", name)
                if code != 1:
                    self.unavailable("parameter", name, "parameter-unavailable")
                    continue
                self.observation("parameter", name, value)
                self.previous.pop(("unavailable", "parameter", name), None)
            except Exception as exc:
                self.unavailable("parameter", name, type(exc).__name__)

    def run(self):
        next_poll = 0.0
        while not self.closed.is_set():
            self.process_messages()
            if time.monotonic() >= next_poll:
                try:
                    self.poll()
                except Exception as exc:
                    self.unavailable("observer", "snapshot-poll", type(exc).__name__)
                next_poll = time.monotonic() + self.POLL_SECONDS
            self.closed.wait(0.05)
        self.process_messages()

    def ros(self, roots):
        try:
            import rospy
            from std_msgs.msg import String
            from sensor_msgs.msg import CameraInfo
            self.rospy = rospy
            rospy.init_node("xgc_record_facts", anonymous=True, disable_signals=True)
            if self.closed.is_set():
                return
            self.subscribers.append(rospy.Subscriber(self.TOPIC, String,
                lambda message: self.enqueue("producer", self.TOPIC, message.data), queue_size=256))
            def camera(topic):
                def receive(message):
                    # CameraInfo is publisher-observed, not proof of consumer loading.
                    values = {"frameId": message.header.frame_id, "width": message.width,
                        "height": message.height, "distortionModel": message.distortion_model,
                        "D": list(message.D), "K": list(message.K), "R": list(message.R), "P": list(message.P),
                        "binningX": message.binning_x, "binningY": message.binning_y,
                        "roi": {"xOffset": message.roi.x_offset, "yOffset": message.roi.y_offset,
                            "height": message.roi.height, "width": message.roi.width,
                            "doRectify": message.roi.do_rectify}}
                    observed = self.clock()
                    observed["rosHeaderTimeNs"] = str(message.header.stamp.to_nsec())
                    self.enqueue("camera-info", topic, values, observed)
                return receive
            for root in roots:
                topic = root.rstrip("/") + "/camera_info"
                self.subscribers.append(rospy.Subscriber(topic, CameraInfo, camera(topic), queue_size=4))
            self.enqueue("observer", "ros-subscriptions", {"status": "subscribed", "factTopic": self.TOPIC,
                "cameraInfoTopics": [root.rstrip("/") + "/camera_info" for root in roots]})
        except Exception as exc:
            self.enqueue("unavailable", "ros-subscriptions", type(exc).__name__)

    def ros_worker(self, reader, writer, roots):
        # rospy registration can hold its topic lock while retrying a lost
        # master. Keep its shutdown hooks outside the recorder process.
        reader.close()
        def emit(kind, source, payload, observed=None):
            with self.lock:
                writer.send((kind, source, payload, observed or self.clock()))
        self.enqueue = emit
        def stop(_signum, _frame):
            if self.rospy is not None:
                self.rospy.signal_shutdown("recording finished")
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, stop)
        self.ros(roots)
        if self.rospy is not None:
            self.rospy.spin()
        writer.close()

    def read_ros(self, reader):
        try:
            while True:
                self.enqueue(*reader.recv())
        except (EOFError, OSError):
            pass
        finally:
            reader.close()

    def start(self, roots):
        context = multiprocessing.get_context("fork")
        reader, writer = context.Pipe(duplex=False)
        self.ros_process = context.Process(target=self.ros_worker,
            args=(reader, writer, roots), daemon=True)
        self.ros_process.start()
        writer.close()
        for target, args in ((self.run, ()), (self.read_ros, (reader,))):
            thread = threading.Thread(target=target, args=args, daemon=True)
            self.threads.append(thread)
            thread.start()

    def snapshot(self, document):
        with self.lock:
            if self.persisted == self.revision:
                return False
            self.state["coverage"]["droppedEvents"] = self.dropped
            document["recordFacts"] = copy.deepcopy(self.state)
            self.persisted = self.revision
            return True

    def finish(self, document):
        # Stop accepting callbacks first; a blocked ROS master or file does not
        # delay native recorder shutdown. Such coverage is explicitly unknown.
        self.closed.set()
        if self.ros_process is not None:
            self.ros_process.terminate()
            self.ros_process.join(timeout=0.3)
            if self.ros_process.is_alive():
                self.ros_process.kill()
                self.ros_process.join()
            self.ros_process.close()
        for thread in self.threads:
            thread.join(timeout=0.3)
        with self.lock:
            self.sealed = True
            self.state["status"] = "closed"
            self.state["observationEndedAt"] = self.clock()
            self.state["coverage"]["pendingAtClose"] = self.messages.qsize()
            self.state["coverage"]["observerStillStopping"] = any(t.is_alive() for t in self.threads)
            self.revision += 1
        self.snapshot(document)


(root, output_prefix, topics_json, split_size_text, max_splits_text,
 min_space_text, compression, repeat_latched, duration_text, bitrate_text,
 safety_text, manifest_json) = sys.argv[1:]
topics = json.loads(topics_json)
manifest = json.loads(manifest_json)
if not isinstance(topics, list) or not topics or not all(isinstance(topic, str) and topic.startswith("/") for topic in topics):
    raise SystemExit("trusted ROS bag topic list is invalid")
if not isinstance(manifest, dict) or manifest.get("schemaVersion") != 1 or manifest.get("topics") != topics:
    raise SystemExit("trusted ROS bag session manifest is invalid")
excluded_topics = manifest.get("excludedTopics") or []
if not isinstance(excluded_topics, list) or not all(
        isinstance(item, dict) and isinstance(item.get("topic"), str) and item["topic"].startswith("/")
        and isinstance(item.get("reason"), str) and item["reason"] for item in excluded_topics):
    raise SystemExit("trusted ROS bag excludedTopics are invalid")
excluded_names = [item["topic"] for item in excluded_topics]
if any(topic in topics for topic in excluded_names):
    raise SystemExit("trusted ROS bag topic list contains an excluded topic")

# Recording owns derived-stream selection. No workflow preset scan or frozen
# copy roster is needed: original scientific topics never match this namespace.
exclusion_pattern = ""
if manifest.get("recordAllTopics", False):
    derived_pattern = r"/xgc/display(?:/.*)?"
    explicit_copies = [topic for topic in topics if topic.startswith("/xgc/display/")]
    if explicit_copies:
        derived_pattern = "(?!(?:" + "|".join(re.escape(topic) for topic in explicit_copies) + ")$)" + derived_pattern
    exclusion_pattern = "^(?:" + "|".join([re.escape(topic) for topic in excluded_names] + [derived_pattern]) + ")$"
    if len(exclusion_pattern.encode("utf8")) > 96 * 1024:
        raise SystemExit("ROS bag exclusion pattern exceeds 96KiB")
    manifest["excludedTopicPatterns"] = [{"pattern": derived_pattern, "reason": "derived display copies; explicitly selected copies and scientific sources remain recorded"}]
naming_version = manifest.get("recordNamingVersion", 0)
if naming_version not in (0, 1) or isinstance(naming_version, bool):
    raise SystemExit("trusted ROS bag record naming version is invalid")
recording_id = manifest.get("recordingId")
if naming_version == 1 and (not isinstance(recording_id, str) or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,254}", recording_id) is None):
    raise SystemExit("trusted ROS bag record identity is invalid")
recording_profile = manifest.get("recordingProfile")
if recording_profile not in ("general", "camera_scientific"):
    raise SystemExit("trusted ROS bag recording profile is invalid")
if compression not in ("none", "lz4", "bz2"):
    raise SystemExit("trusted ROS bag compression is invalid")
if repeat_latched not in ("true", "false"):
    raise SystemExit("trusted ROS bag repeat-latched value is invalid")
try:
    split_size = int(split_size_text)
    max_splits = int(max_splits_text)
    min_space_gib = int(min_space_text)
    duration_minutes = int(duration_text)
    bitrate_mbps = float(bitrate_text)
    safety_factor = float(safety_text)
except ValueError as exc:
    raise SystemExit("trusted ROS bag capacity parameters are invalid") from exc
if split_size < 64 or max_splits < 1 or min_space_gib < 1 or duration_minutes < 1 or bitrate_mbps < 0 or safety_factor < 1:
    raise SystemExit("trusted ROS bag capacity parameters are out of range")
output_directory = os.path.dirname(output_prefix)
os.makedirs(output_directory, exist_ok=True)
expected_bytes = math.ceil(duration_minutes * 60 * bitrate_mbps * 1000000 / 8 * safety_factor)
planned_capacity_bytes = split_size * 1024 * 1024 * max_splits
stop_limit_bytes = planned_capacity_bytes
if recording_profile == "camera_scientific":
    limit = manifest.get("scientificTotalLimitBytes")
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 64 * 1024 * 1024:
        raise SystemExit("trusted ROS bag scientificTotalLimitBytes is invalid")
    stop_limit_bytes = limit
reserve_bytes = min_space_gib * 1024 * 1024 * 1024
free_bytes = shutil.disk_usage(output_directory).free
if manifest.get("estimatedBytes") != expected_bytes:
    raise SystemExit("trusted ROS bag capacity estimate does not match the session manifest")
if recording_profile != "camera_scientific" and expected_bytes > planned_capacity_bytes:
    raise SystemExit("ROS bag capacity preflight failed: planned split capacity is smaller than the estimated session")
needed_bytes = min(expected_bytes, stop_limit_bytes)
manifest_path = os.path.join(output_directory, "session-manifest.json")

def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat().replace("+00:00", "Z")

def write_manifest(document):
    encoded = (json.dumps(document, ensure_ascii=False, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    temporary = manifest_path + ".part." + str(os.getpid())
    try:
        with open(temporary, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, manifest_path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
    directory_fd = os.open(output_directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)

prefix_name = os.path.basename(output_prefix)
native_pattern = re.compile(re.escape(prefix_name) + r"_(0|[1-9][0-9]*)\.bag$")

def recorded_paths():
    # User-readable experiment names may contain glob metacharacters.
    paths = []
    for entry in os.scandir(output_directory):
        suffix = entry.name[len(prefix_name):] if entry.name.startswith(prefix_name) else ""
        if suffix[:1] in ("_", "-", ".") and (suffix.endswith(".bag") or suffix.endswith(".bag.active")):
            paths.append(entry.path)
    return sorted(paths)

def recorded_bytes():
    total = 0
    for path in recorded_paths():
        try:
            info = os.lstat(path)
            if stat.S_ISREG(info.st_mode):
                total += info.st_size
        except OSError:
            continue
    return total

def move_closed_without_replacement(source, destination):
    source_info = os.lstat(source)
    if not stat.S_ISREG(source_info.st_mode):
        raise RuntimeError("finalized bag is not a regular file")
    source_fd = os.open(source, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    try:
        opened = os.fstat(source_fd)
        if (opened.st_dev, opened.st_ino, opened.st_size) != (source_info.st_dev, source_info.st_ino, source_info.st_size):
            raise RuntimeError("closed bag changed before preservation")
        os.fsync(source_fd)
    finally:
        os.close(source_fd)
    try:
        os.link(source, destination, follow_symlinks=False)
    except FileExistsError:
        destination_info = os.lstat(destination)
        # Interrupted link+unlink resumes only when both names are this exact
        # inode. Foreign existing bytes are never replaced, even if identical.
        if not stat.S_ISREG(destination_info.st_mode) or (source_info.st_dev, source_info.st_ino) != (destination_info.st_dev, destination_info.st_ino):
            raise RuntimeError("readable bag destination already exists")
    directory_fd = os.open(output_directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(directory_fd)
        os.unlink(source)
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)

def finalize_archive_files(document):
    def issued_bag_name(index, count):
        # One closed volume is the record. A part suffix would imply another volume exists.
        if count == 1 and index == 0:
            return prefix_name + ".bag"
        return prefix_name + "-part" + str(index + 1).zfill(2) + ".bag"
    # Pin this writer's exact closed inodes before any rename. A readable name
    # appearing later cannot become an issued file merely by matching a pattern.
    document["archiveFinalizationState"] = "pending"
    document["bagFiles"] = []
    write_manifest(document)
    segments = document.get("archiveSegments")
    if segments is None:
        segments = []
        for path in recorded_paths():
            name = os.path.basename(path)
            if name.endswith(".bag.active"):
                continue
            match = native_pattern.fullmatch(name)
            if match is None:
                continue
            info = os.lstat(path)
            if not stat.S_ISREG(info.st_mode):
                raise RuntimeError("bag inventory contains an unexpected closed file")
            index = int(match.group(1))
            segments.append({"sourceName": name, "name": "", "index": index, "device": info.st_dev, "inode": info.st_ino, "sizeBytes": info.st_size})
        closed_count = len(segments)
        for segment in segments:
            segment["name"] = issued_bag_name(segment["index"], closed_count)
        document["archiveSegments"] = segments
    document["archiveFinalizationState"] = "pending"
    document["bagFiles"] = []
    write_manifest(document)
    try:
        by_index = {}
        closed_count = len(segments)
        for segment in segments:
            index = segment["index"]
            source_name = prefix_name + "_" + str(index) + ".bag"
            final_name = issued_bag_name(index, closed_count)
            if not isinstance(index, int) or isinstance(index, bool) or index < 0 or segment["sourceName"] != source_name or segment["name"] != final_name or index in by_index:
                raise RuntimeError("bag finalization identity is invalid")
            source = os.path.join(output_directory, source_name)
            destination = os.path.join(output_directory, final_name)
            expected = (segment["device"], segment["inode"], segment["sizeBytes"])
            if os.path.lexists(source):
                info = os.lstat(source)
                if not stat.S_ISREG(info.st_mode) or (info.st_dev, info.st_ino, info.st_size) != expected:
                    raise RuntimeError("closed bag changed before finalization")
                move_closed_without_replacement(source, destination)
            info = os.lstat(destination)
            if not stat.S_ISREG(info.st_mode) or (info.st_dev, info.st_ino, info.st_size) != expected:
                raise RuntimeError("finalized bag identity changed")
            by_index[index] = {"id": "bag." + recording_id + "." + str(index), "name": final_name, "state": "finalized", "sizeBytes": info.st_size}
        expected_names = {entry["name"] for entry in by_index.values()}
        for path in recorded_paths():
            name = os.path.basename(path)
            info = os.lstat(path)
            if not stat.S_ISREG(info.st_mode):
                raise RuntimeError("bag inventory contains a non-regular file")
            if name.endswith(".bag.active"):
                document["bagFiles"].append({"name": name, "state": "partial", "sizeBytes": info.st_size})
            elif name not in expected_names:
                raise RuntimeError("bag inventory contains an unowned closed file")
        document["bagFiles"].extend(by_index[index] for index in sorted(by_index))
        document["archiveFinalizationState"] = "finalized"
        document.pop("archiveFinalizationError", None)
    except Exception as exc:
        document["archiveFinalizationState"] = "failed"
        document["archiveFinalizationError"] = str(exc)
        write_manifest(document)
        raise
    write_manifest(document)

if naming_version == 1:
    if os.path.lexists(manifest_path):
        if not stat.S_ISREG(os.lstat(manifest_path).st_mode):
            raise SystemExit("existing ROS bag manifest is not a regular file")
        with open(manifest_path, "r", encoding="utf-8") as stream:
            existing = json.load(stream)
        if existing.get("recordingId") != recording_id or existing.get("recordNamingVersion") != 1:
            raise SystemExit("existing ROS bag manifest belongs to another record")
        if existing.get("recordingEndedAt") and existing.get("status") in ("stopped", "completed", "failed") and existing.get("archiveFinalizationState") in ("pending", "failed"):
            finalize_archive_files(existing)
            raise SystemExit(0 if existing.get("terminationSignal") is not None else existing.get("recorderExitCode", 1))
        raise SystemExit("ROS bag record has already started; existing bytes are retained")
    if recorded_paths():
        raise SystemExit("ROS bag record already contains files; existing bytes are retained")
if free_bytes <= reserve_bytes or needed_bytes > free_bytes - reserve_bytes:
    raise SystemExit("ROS bag capacity preflight failed: insufficient free space after the configured reserve")

facts = RecordFacts(manifest, output_directory)
facts.start(manifest.get("cameraTopicRoots") or [])
facts.snapshot(manifest)
manifest["recordingStartedAtEvidence"] = "recorder-launch-request"
manifest["firstBagObservedAt"] = None
manifest["firstBagObservedAtEvidence"] = "file-presence-observed-not-first-sample"
manifest["recordingStartedAt"] = utc_now()
manifest["status"] = "recording"
manifest["capacityPreflight"] = {
    "freeBytes": free_bytes,
    "minimumReserveBytes": reserve_bytes,
    "plannedCapacityBytes": planned_capacity_bytes,
    "estimatedBytes": expected_bytes,
    "splitPolicy": "preserve-all",
    "stopLimitBytes": stop_limit_bytes,
}
write_manifest(manifest)
# Wait on the native recorder itself; the Python CLI can exit on a signal
# before its recorder child has finalized and renamed the bag.
executable = os.path.join(root, "lib", "rosbag", "record")
arguments = [executable, "--split", "--size=" + str(split_size)]
arguments.append("--min-space=" + str(min_space_gib) + "G")
if compression != "none":
    arguments.append("--" + compression)
if repeat_latched == "true":
    arguments.append("--repeat-latched")
arguments.extend(["--output-name" if naming_version == 1 else "--output-prefix", output_prefix])
if manifest.get("recordAllTopics", False):
    arguments.append("--all")
    arguments.extend(["--exclude", exclusion_pattern])
else:
    arguments.extend(topics)
child = None
forwarded_signal = None
signal_delivered = False
capacity_limit_reached = False

def forward(signum, _frame):
    global forwarded_signal, signal_delivered
    forwarded_signal = signum
    if child is not None and child.poll() is None:
        try:
            child.send_signal(signum)
            signal_delivered = True
        except ProcessLookupError:
            pass

signal.signal(signal.SIGINT, forward)
signal.signal(signal.SIGTERM, forward)
exit_code = 1
try:
    child = subprocess.Popen(arguments)
    if forwarded_signal is not None and not signal_delivered and child.poll() is None:
        child.send_signal(forwarded_signal)
        signal_delivered = True
    while True:
        changed = facts.snapshot(manifest)
        if manifest["firstBagObservedAt"] is None and recorded_paths():
            manifest["firstBagObservedAt"] = facts.clock()
            changed = True
        if changed:
            write_manifest(manifest)
        if child.poll() is not None:
            exit_code = child.returncode
            break
        if not signal_delivered and recorded_bytes() >= stop_limit_bytes:
            try:
                child.send_signal(signal.SIGTERM)
                signal_delivered = True
                forwarded_signal = signal.SIGTERM
                capacity_limit_reached = True
            except ProcessLookupError:
                pass
        try:
            exit_code = child.wait(timeout=0.25)
            break
        except subprocess.TimeoutExpired:
            continue
finally:
    facts.finish(manifest)
    manifest["recordingEndedAt"] = utc_now()
    manifest["recorderExitCode"] = exit_code
    manifest["terminationSignal"] = forwarded_signal
    manifest["capacityLimitReached"] = capacity_limit_reached
    manifest["status"] = "stopped" if forwarded_signal is not None else ("completed" if exit_code == 0 else "failed")
    if naming_version == 1:
        finalize_archive_files(manifest)
    else:
        bag_files = []
        for path in recorded_paths():
            state = "partial" if path.endswith(".bag.active") else "finalized"
            try:
                info = os.stat(path)
            except OSError as exc:
                bag_files.append({"name": os.path.basename(path), "state": "unreadable", "inspectionError": type(exc).__name__})
                continue
            bag_files.append({"name": os.path.basename(path), "state": state, "sizeBytes": info.st_size})
        manifest["bagFiles"] = bag_files
        write_manifest(manifest)
raise SystemExit(0 if forwarded_signal is not None else exit_code)
