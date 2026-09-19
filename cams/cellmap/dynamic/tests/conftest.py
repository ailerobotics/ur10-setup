# SPDX-License-Identifier: Apache-2.0
"""Put the dynamic-layer package dir on sys.path so the offline tests import the
pure modules directly, exactly like the standalone `python3 test_*.py` path."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
