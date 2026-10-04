#!/usr/bin/env python3
"""Structural guard for the PX4 runtime's per-callback work.

The PX4 Adapter receives about nine MAVROS and mocap topics per robot at the
source rate (about 100 Hz each) and gates every semantic channel to at most 10
Hz. The generated contract is immutable for the life of a runtime, so a channel's
output rate, schema, freshness bound and tracker are resolved once, in
ChannelTable's constructor, and a callback indexes an array. These checks fail
if a callback or gate helper goes back to consulting the contract, a
string-keyed map or a string literal channel id. The behavioural equivalence of
the resolved table with the string-keyed implementation is asserted by
channel_table_test.cpp (an oracle replay) and runs under catkin run_tests.
"""

import re
import unittest
from pathlib import Path


PACKAGE = (
    Path(__file__).resolve().parents[1] / "src" / "xgc_px4_multirotor_ros1_adapter"
)
RUNTIME = PACKAGE / "src" / "robot_runtime.cpp"
TABLE = PACKAGE / "include" / "xgc_px4_multirotor_ros1_adapter" / "channel_table.hpp"

# Every RobotRuntime member that runs per ROS callback or per periodic tick.
PER_CALLBACK_MEMBERS = (
    "px4PoseCallback",
    "mocapPoseCallback",
    "mocapVelocityCallback",
    "mocapAccelerationCallback",
    "px4VelocityCallback",
    "imuCallback",
    "batteryCallback",
    "controllerStatusCallback",
    "mavrosStateCallback",
    "mavrosExtendedStateCallback",
    "localSetpointCallback",
    "attitudeSetpointCallback",
    "timesyncStatusCallback",
    "emitPositionErrorLocked",
    "emitPx4PeriodicLocked",
    "emitStreamHealthLocked",
    "shouldEmitLocked",
    "makeEnvelopeLocked",
    "recordSourceLocked",
    "recordStateSourceLocked",
    "recordOutputLocked",
    "countDroppedLocked",
)

FORBIDDEN_IN_PER_CALLBACK_MEMBERS = (
    (r"contract::", "generated contract lookup"),
    (r"\bprofile_id_\b", "profile id (contract lookup key)"),
    (r"\bsources_\b", "string-keyed source map"),
    (r"\blast_output_\b", "string-keyed last-output map"),
    (r"\bsequences_\b", "string-keyed sequence map"),
    (
        r"\b(?:channelEnabled|channelRequired|shouldEmitLocked|makeEnvelopeLocked|"
        r"recordSourceLocked|recordOutputLocked|recordStateSourceLocked|"
        r"countDroppedLocked)\(\s*\"",
        "string literal channel id passed to a gate helper",
    ),
)

# ChannelTable members that run per callback; only resolve() may read the
# generated contract.
TABLE_HOT_MEMBERS = (
    "recordSource",
    "recordOutput",
    "shouldEmit",
    "makeEnvelope",
    "droppedSource",
    "ensureSource",
)


def strip_comments_and_strings(source: str) -> str:
    """Blank out comments and string/char literals, keeping offsets."""
    out = []
    i = 0
    n = len(source)
    while i < n:
        two = source[i : i + 2]
        if two == "//":
            j = source.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif two == "/*":
            j = source.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", source[i:j]))
            i = j
        elif source[i] in "\"'":
            quote = source[i]
            j = i + 1
            while j < n and source[j] != quote:
                j += 2 if source[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(quote + " " * (j - i - 2) + quote if j - i >= 2 else quote)
            i = j
        else:
            out.append(source[i])
            i += 1
    return "".join(out)


def function_body(source: str, pattern: str) -> str:
    """Return the text of the first function whose signature matches pattern.

    The source is scanned with comments and literals blanked so braces inside
    them do not confuse the matching; the returned text is from the original
    source (literals intact) at the same offsets.
    """
    blanked = strip_comments_and_strings(source)
    match = re.search(pattern, blanked, re.MULTILINE)
    if match is None:
        raise AssertionError("function not found: " + pattern)
    brace = blanked.find("{", match.end())
    if brace < 0:
        raise AssertionError("no body for: " + pattern)
    depth = 0
    for index in range(brace, len(blanked)):
        if blanked[index] == "{":
            depth += 1
        elif blanked[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace : index + 1]
    raise AssertionError("unterminated body for: " + pattern)


class Px4RuntimeResolveOnceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = RUNTIME.read_text(encoding="utf-8")
        cls.table = TABLE.read_text(encoding="utf-8")

    def test_per_callback_members_use_the_resolved_table_only(self):
        for name in PER_CALLBACK_MEMBERS:
            with self.subTest(member=name):
                body = function_body(
                    self.runtime, r"^[^\n]*\bRobotRuntime::%s\s*\(" % re.escape(name)
                )
                # The callback is the code between its signature and closing
                # brace; the stale-bound helper is resolved, not looked up.
                for pattern, meaning in FORBIDDEN_IN_PER_CALLBACK_MEMBERS:
                    self.assertIsNone(
                        re.search(pattern, strip_keep_literals(body)),
                        "%s still uses a %s" % (name, meaning),
                    )

    def test_stale_policy_is_not_looked_up_by_string_each_period(self):
        body = function_body(
            self.runtime, r"^[^\n]*\bRobotRuntime::emitPx4PeriodicLocked\s*\("
        )
        self.assertNotIn("channelStaleAfterSeconds(profile_id_", body)
        self.assertIn("channelStaleAfterSeconds(channels_[ChannelId::kHealth])", body)

    def test_runtime_has_no_string_keyed_channel_state(self):
        header = (PACKAGE / "include" / "xgc_px4_multirotor_ros1_adapter" /
                  "robot_runtime.hpp").read_text(encoding="utf-8")
        for token in ("sources_", "last_output_", "sequences_"):
            self.assertNotIn(token, header)
            self.assertNotIn(token, self.runtime)
        self.assertIn("ChannelTable channels_;", header)

    def test_only_the_table_constructor_reads_the_contract(self):
        blanked = strip_comments_and_strings(self.table)
        self.assertEqual(1, len(re.findall(r"contract::channelMetadata\(", blanked)))
        self.assertEqual(1, len(re.findall(r"contract::messageMetadata\(", blanked)))
        resolve = strip_comments_and_strings(
            function_body(self.table, r"\bvoid resolve\s*\(")
        )
        self.assertIn("contract::channelMetadata(", resolve)
        self.assertIn("contract::messageMetadata(", resolve)
        for name in TABLE_HOT_MEMBERS:
            with self.subTest(member=name):
                body = function_body(self.table, r"\b%s\s*\(" % re.escape(name))
                self.assertNotIn("contract::", strip_comments_and_strings(body))

    def test_gate_compares_the_precomputed_interval(self):
        body = function_body(self.table, r"\bbool shouldEmit\s*\(")
        self.assertIn("channel->emit_interval_seconds", body)
        self.assertNotIn("1.0 /", body)

    def test_channel_ids_match_the_wire_names_in_the_profile(self):
        names = re.search(
            r"names\[kChannelCount\]\s*=\s*\{(.*?)\};", self.table, re.DOTALL
        )
        self.assertIsNotNone(names)
        listed = re.findall(r'"([^"]+)"', names.group(1))
        profile = (
            PACKAGE.parents[1] / "profiles" / "ros1" / "px4-multirotor-ros1-v9.yaml"
        ).read_text(encoding="utf-8")
        profile_ids = set(re.findall(r"^\s*-?\s*id:\s*(\S+)\s*$", profile, re.MULTILINE))
        enum = re.search(r"enum class ChannelId[^{]*\{(.*?)\};", self.table, re.DOTALL)
        self.assertIsNotNone(enum)
        enumerators = [
            item.strip().split("=")[0].strip()
            for item in enum.group(1).split(",")
            if item.strip()
        ]
        self.assertEqual("kCount", enumerators[-1])
        self.assertEqual(len(listed), len(enumerators) - 1)
        self.assertEqual(len(set(listed)), len(listed))
        for channel_id in listed:
            self.assertIn(channel_id, profile_ids)


def strip_keep_literals(source: str) -> str:
    """Blank comments only, so string literals stay visible to the checks."""
    out = []
    i = 0
    n = len(source)
    while i < n:
        two = source[i : i + 2]
        if two == "//":
            j = source.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif two == "/*":
            j = source.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", source[i:j]))
            i = j
        elif source[i] == '"':
            j = i + 1
            while j < n and source[j] != '"':
                j += 2 if source[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(source[i:j])
            i = j
        else:
            out.append(source[i])
            i += 1
    return "".join(out)


if __name__ == "__main__":
    unittest.main()
