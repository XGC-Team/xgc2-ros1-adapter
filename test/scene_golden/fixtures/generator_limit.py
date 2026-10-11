#!/usr/bin/env python3.8
"""Emit the preparation test's output beyond its granted byte limit."""
import sys

sys.stdout.write('x' * 2000000)
