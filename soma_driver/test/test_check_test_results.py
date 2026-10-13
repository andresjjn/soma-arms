"""The CI guard against a colcon test that ran nothing and passed.

Pure tests on the junit XML shapes that actually reached CI: none at all
(the unittest fallback of run 37964849489), a collection collapsed to
one skip (run 31451116966), colcon's placeholder for a pytest that died,
and the real run of 2026-10-09.
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))

import check_test_results  # noqa: E402

# pytest 6.2.5 in the ROS container, a module level importorskip escaping
# launch_testing's collection hook: one session level skip, nothing else.
COLLAPSED = (
    '<?xml version="1.0" encoding="utf-8"?><testsuites>'
    '<testsuite name="pytest" errors="0" failures="0" skipped="1" tests="1">'
    '<testcase classname="soma_driver" name="" time="0.000">'
    '<skipped message="collection skipped"/></testcase>'
    '</testsuite></testsuites>')

# Written by colcon before it starts pytest, overwritten by a real run.
COLCON_PLACEHOLDER = (
    '<?xml version="1.0" encoding="UTF-8"?>'
    '<testsuite name="soma_driver" tests="1" failures="0" time="0" errors="1"'
    ' skipped="0"><testcase classname="soma_driver"'
    ' name="pytest.missing_result" time="0"><failure message="The test'
    ' invocation failed without generating a result file."/></testcase>'
    '</testsuite>')

# Run 37976168647: 397 passed, 5 skipped, "Summary: 402 tests".
ROS_JOB = (
    '<?xml version="1.0" encoding="utf-8"?><testsuites>'
    '<testsuite name="pytest" errors="0" failures="0" skipped="5" tests="402"/>'
    '</testsuites>')


def _xml(tmp_path, text):
    path = tmp_path / 'pytest.xml'
    path.write_text(text)
    return path


def test_no_xml_is_the_unittest_fallback_and_nothing_ran(tmp_path):
    assert check_test_results.count_tests_run(tmp_path / 'pytest.xml') == 0


def test_a_collection_collapsed_to_one_skip_ran_nothing(tmp_path):
    assert check_test_results.count_tests_run(_xml(tmp_path, COLLAPSED)) == 0


def test_the_colcon_placeholder_of_a_dead_pytest_ran_nothing(tmp_path):
    path = _xml(tmp_path, COLCON_PLACEHOLDER)
    assert check_test_results.count_tests_run(path) == 0


def test_skipped_tests_do_not_count_as_run(tmp_path):
    assert check_test_results.count_tests_run(_xml(tmp_path, ROS_JOB)) == 397


def test_failures_ran_and_are_left_to_colcon_test_result(tmp_path):
    path = _xml(tmp_path, '<testsuites><testsuite tests="3" failures="1"'
                          ' errors="0" skipped="0"/></testsuites>')
    assert check_test_results.count_tests_run(path) == 3


def test_main_refuses_a_silent_zero_and_passes_a_real_run(tmp_path, capsys):
    assert check_test_results.main(str(_xml(tmp_path, COLLAPSED))) == 1
    assert 'FAILED: no test ran' in capsys.readouterr().out
    assert check_test_results.main(str(_xml(tmp_path, ROS_JOB))) == 0
    assert '397 tests ran' in capsys.readouterr().out
