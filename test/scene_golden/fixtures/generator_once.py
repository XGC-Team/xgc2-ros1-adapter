#!/usr/bin/env python3.8
"""The generation test's counter and fixed seeded result."""
import json
from pathlib import Path
import sys

counter = Path(sys.argv[1])
counter.write_text(counter.read_text() + 'x' if counter.exists() else 'x')
print(json.dumps({'seed': 7, 'scene': {'schema': 'xgc2.scene.v1', 'id': 'generated',
                                     'frame': 'world', 'obstacles': []}}))
