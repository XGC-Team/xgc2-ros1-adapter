#!/usr/bin/env python3

import copy
import json
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
TOOLS = REPOSITORY_ROOT / "tools"
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import generate_contract_metadata as CONTRACT  # noqa: E402
import generate_profile_catalog as CATALOG  # noqa: E402
import test_generate_contract_metadata as FIXTURES  # noqa: E402

SCHEMA = FIXTURES.SCHEMA_PATH
PROFILES = REPOSITORY_ROOT / "profiles" / "ros1"
SCOUT_PROFILES = [
    PROFILES / "scout-mini-physical-vrpn.yaml",
    PROFILES / "scout-mini-xsim-ros.yaml",
    PROFILES / "scout-mini-gazebo-vrpn.yaml",
]
SCOUT_PROVIDER = "xgc2-scout-mini-ros1-adapter"


class ProfileCatalogTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.temp = Path(temporary.name)
        self.registry = self.temp / "registry.json"
        self.registry.write_text(
            json.dumps(FIXTURES.registry_document()), encoding="utf-8"
        )

    def catalog(self, files=SCOUT_PROFILES, provider=SCOUT_PROVIDER):
        return CATALOG.build_catalog(
            self.registry, [str(path) for path in files], SCHEMA, provider
        )

    def test_every_profile_is_listed_with_the_digest_compiled_into_the_server(self):
        catalog = self.catalog()
        self.assertEqual(catalog["schema"], "xgc.robot.adapter-profile-catalog/v4")
        self.assertEqual(
            [entry["profileId"] for entry in catalog["profiles"]],
            ["scout-mini.physical.vrpn", "scout-mini.xsim.ros", "scout-mini.gazebo.vrpn"],
        )
        _, messages = CONTRACT.load_registry(self.registry)
        profiles = CONTRACT.load_profiles(
            [str(path) for path in SCOUT_PROFILES], SCHEMA, messages
        )
        header = CONTRACT.generate(
            987654321, messages, profiles, SCOUT_PROVIDER, "fixture"
        )
        for entry in catalog["profiles"]:
            self.assertEqual(entry["providerDefinitionId"], SCOUT_PROVIDER)
            self.assertRegex(
                header,
                r'\{{"{}", "{}", '.format(
                    re.escape(entry["profileId"]), entry["profileDigest"]
                ),
            )

    def test_the_digest_binds_the_provider_identity(self):
        scout = self.catalog()
        other = self.catalog(provider="xgc2-other-ros1-adapter")
        self.assertNotEqual(
            scout["profiles"][0]["profileDigest"],
            other["profiles"][0]["profileDigest"],
        )

    def test_profiles_of_one_provider_offer_the_same_operations(self):
        profile = yaml.safe_load(SCOUT_PROFILES[1].read_text(encoding="utf-8"))
        profile["channels"] = [
            channel
            for channel in profile["channels"]
            if channel["id"] != "operation.motion-intent-release"
        ]
        changed = self.temp / "scout-mini-xsim-ros.yaml"
        changed.write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.catalog(files=[SCOUT_PROFILES[0], changed])

    def test_the_command_line_writes_a_catalog_the_verifier_accepts(self):
        output = self.temp / "catalog.json"
        arguments = [
            "--registry", str(self.registry),
            "--profile-schema", str(SCHEMA),
            "--definition-id", SCOUT_PROVIDER,
        ]
        for path in SCOUT_PROFILES:
            arguments += ["--profile-file", str(path)]
        subprocess.run(
            [sys.executable, str(TOOLS / "generate_profile_catalog.py"), *arguments,
             "--output", str(output)],
            check=True,
        )
        verify = [sys.executable, str(TOOLS / "verify_profile_catalog.py"), *arguments,
                  "--catalog", str(output)]
        self.assertEqual(subprocess.run(verify).returncode, 0)

        installed = json.loads(output.read_text(encoding="utf-8"))
        for name, mutate in (
            ("digest", lambda c: c["profiles"][0].update(profileDigest="0" * 64)),
            ("provider", lambda c: c["profiles"][1].update(providerDefinitionId="x")),
            ("order", lambda c: c["profiles"].reverse()),
            ("channel", lambda c: c["profiles"][2]["channels"].pop()),
            (
                "operation timeout",
                lambda c: c["profiles"][0]["semantics"]["operations"][0].update(
                    timeoutMillis=1
                ),
            ),
        ):
            with self.subTest(mutation=name):
                changed = copy.deepcopy(installed)
                mutate(changed)
                output.write_text(json.dumps(changed), encoding="utf-8")
                self.assertNotEqual(subprocess.run(verify).returncode, 0)


if __name__ == "__main__":
    unittest.main()
