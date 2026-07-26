"""dimensions.yaml is the source of truth; the xacro must agree with it.

Pure test (no ROS): parses the xacro property literals with a regex and
compares them against soma_description/config/dimensions.yaml. If someone
updates a measurement in one place and not the other, this goes red.

Also pins the torso soft-limit numbers to SERVO_MAP, so the wedging
protection (2026-07-22) cannot silently drift between model and driver.
"""
import re
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from soma_driver.servo_map import SERVO_MAP  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
DIMENSIONS = REPO / 'soma_description' / 'config' / 'dimensions.yaml'

PROP_RE = re.compile(
    r'<xacro:property\s+name="(\w+)"\s+value="([0-9][0-9.eE+-]*)"\s*/>')


def xacro_literals(path: Path) -> dict:
    """Numeric <xacro:property> literals of a file, by name."""
    return {name: float(val) for name, val in PROP_RE.findall(path.read_text())}


def load_sections():
    data = yaml.safe_load(DIMENSIONS.read_text())
    return {key: sec for key, sec in data.items() if isinstance(sec, dict) and 'file' in sec}


SECTIONS = load_sections()


class TestYamlAgainstXacro:
    @pytest.mark.parametrize('section', sorted(SECTIONS))
    def test_every_yaml_value_matches_the_xacro(self, section):
        sec = SECTIONS[section]
        props = xacro_literals(REPO / sec['file'])
        mismatches = []
        for name, entry in sec['dimensions'].items():
            if name not in props:
                mismatches.append(f'{name}: in yaml but not a literal in {sec["file"]}')
            elif props[name] != pytest.approx(entry['value']):
                mismatches.append(
                    f'{name}: yaml={entry["value"]} xacro={props[name]}')
        assert not mismatches, '\n'.join(mismatches)

    @pytest.mark.parametrize('section', sorted(SECTIONS))
    def test_every_calibratable_literal_is_in_the_yaml(self, section):
        """A new [calibrate] literal must come with provenance."""
        sec = SECTIONS[section]
        props = xacro_literals(REPO / sec['file'])
        missing = sorted(set(props) - set(sec['dimensions']))
        assert not missing, f'literals without provenance in dimensions.yaml: {missing}'

    @pytest.mark.parametrize('section', sorted(SECTIONS))
    def test_every_entry_has_provenance(self, section):
        for name, entry in SECTIONS[section]['dimensions'].items():
            assert entry.get('status') in ('estimated', 'measured', 'design'), name
            assert entry.get('source'), name
            assert entry.get('date'), name


class TestTorsoLimitsPinnedToDriver:
    """Model soft limits and driver soft limits are the same numbers."""

    def test_lift_margin_is_the_driver_lower_limit(self):
        dims = SECTIONS['torso']['dimensions']
        assert SERVO_MAP['torso_lift_joint'].lower == pytest.approx(
            dims['lift_margin']['value'])

    def test_stroke_minus_margin_is_the_driver_upper_limit(self):
        dims = SECTIONS['torso']['dimensions']
        assert SERVO_MAP['torso_lift_joint'].upper == pytest.approx(
            dims['lift_stroke']['value'] - dims['lift_margin']['value'])
