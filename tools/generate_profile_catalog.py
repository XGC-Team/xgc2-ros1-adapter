#!/usr/bin/env python3
"""Generate the installed robot profile catalog of one robot type.

The catalog is what Core reads to offer a robot type's profiles: the public
contract (parameters, semantics, channels) with the digest that a member's
registration must carry. It is generated from the same validated profile
files and registry as the contract tables compiled into the type server, so
the two cannot disagree.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from generate_contract_metadata import (
    catalog_profile_body,
    load_profiles,
    load_registry,
    profile_contract_digest,
)

CATALOG_SCHEMA = "xgc.robot.adapter-profile-catalog/v4"


def build_catalog(
    registry: Path,
    profile_files: list[str],
    profile_schema: Path,
    definition_id: str,
) -> dict[str, Any]:
    _, messages = load_registry(registry)
    profiles = load_profiles(profile_files, profile_schema, messages)
    installed = []
    for source_profile in profiles.values():
        body = catalog_profile_body(source_profile, messages, definition_id)
        installed.append({"profileDigest": profile_contract_digest(body), **body})
    require_one_operation_contract(installed)
    return {"schema": CATALOG_SCHEMA, "profiles": installed}


def require_one_operation_contract(profiles: list[dict[str, Any]]) -> None:
    """All profiles of a robot type offer the same operations.

    A robot's operations are the type server's, not the profile's: the profile
    only chooses the measurement source. Different operation sets or timeouts
    between the physical, Gazebo and xsim profiles would be a mistake in one
    of them.
    """
    first = profiles[0]
    expected = {
        operation["id"]: operation["timeoutMillis"]
        for operation in first["semantics"]["operations"]
    }
    for profile in profiles[1:]:
        actual = {
            operation["id"]: operation["timeoutMillis"]
            for operation in profile["semantics"]["operations"]
        }
        if actual != expected:
            raise ValueError(
                "profile {} offers other operations than profile {}".format(
                    profile["profileId"], first["profileId"]
                )
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--profile-file", required=True, action="append")
    parser.add_argument("--profile-schema", required=True)
    parser.add_argument("--definition-id", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    catalog = build_catalog(
        Path(args.registry),
        args.profile_file,
        Path(args.profile_schema),
        args.definition_id,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
