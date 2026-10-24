#!/usr/bin/env python3
"""Check that an installed profile catalog is exactly what its profiles produce."""

import argparse
import json
from pathlib import Path

from generate_profile_catalog import build_catalog


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", required=True)
    parser.add_argument("--profile-file", required=True, action="append")
    parser.add_argument("--profile-schema", required=True)
    parser.add_argument("--definition-id", required=True)
    parser.add_argument("--catalog", required=True)
    args = parser.parse_args()
    expected = build_catalog(
        Path(args.registry),
        args.profile_file,
        Path(args.profile_schema),
        args.definition_id,
    )
    installed = json.loads(Path(args.catalog).read_text(encoding="utf-8"))
    if installed != expected:
        raise SystemExit(
            "{} does not match the canonical contract of its profiles".format(
                args.catalog
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
