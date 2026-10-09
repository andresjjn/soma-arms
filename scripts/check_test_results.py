#!/usr/bin/env python3
"""Fail unless a pytest junit XML shows at least one test that ran.

colcon test lets two silent zeros through, and colcon test-result, which
fails on any failure or error, reports both as a pass:

- setup.py stops declaring pytest: colcon falls back to unittest, which
  finds none of these pytest tests and writes no XML ("0 tests", green,
  CI run 37964849489).
- pytest collects nothing: it exits 5 and colcon passes that. A module
  level importorskip of a module missing in the ROS container collapsed
  the whole collection to "1 skipped" (CI run 31451116966, 2026-08-11):
  launch_testing's collection hook imports every test module itself and
  lets the skip escape.

A test ran when it reached a verdict, pass or fail. Skipped and errored
ones did not, and neither did colcon's placeholder for a pytest that
died before writing its XML.

Usage:
  python3 scripts/check_test_results.py build/soma_driver/pytest.xml
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def count_tests_run(xml_path) -> int:
    """Tests that reached a verdict in a junit XML; 0 if there is none."""
    path = Path(xml_path)
    if not path.is_file():
        return 0
    return sum(int(s.get('tests', 0)) - int(s.get('skipped', 0))
               - int(s.get('errors', 0))
               for s in ET.parse(path).getroot().iter('testsuite'))


def main(xml_path: str) -> int:
    ran = count_tests_run(xml_path)
    if ran < 1:
        print(f'FAILED: no test ran ({xml_path}). Is pytest installed and '
              'declared in setup.py? Did a module level skip collapse the '
              'collection?')
        return 1
    print(f'{ran} tests ran ({xml_path})')
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print(__doc__)
        raise SystemExit(2)
    raise SystemExit(main(sys.argv[1]))
