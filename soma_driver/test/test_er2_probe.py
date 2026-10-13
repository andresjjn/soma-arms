"""The ER 2 probe parses what the model says and never reaches the network here.

Pure tests. The SDK is imported only inside query(), which these tests
replace; a missing key stops main() before query() could run.
"""
import struct
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts'))

import er2_probe  # noqa: E402

PNG_HEADER = (b'\x89PNG\r\n\x1a\n' + struct.pack('>I', 13) + b'IHDR'
              + struct.pack('>II', 640, 480) + b'\x08\x02\x00\x00\x00' + b'\x00' * 4)
JPEG_SOF0 = (b'\xff\xd8' + b'\xff\xe0' + struct.pack('>H', 4) + b'\x00\x00'
             + b'\xff\xc0' + struct.pack('>H', 11) + b'\x08' + struct.pack('>HH', 400, 640)
             + b'\x03' + b'\x00' * 9)


def test_importing_the_probe_does_not_import_the_google_sdk():
    assert 'google' not in sys.modules
    assert 'google.genai' not in sys.modules


def test_default_model_is_the_only_callable_preview():
    assert er2_probe.DEFAULT_MODEL == 'gemini-robotics-er-2-preview'


def test_key_comes_from_the_environment_only():
    assert er2_probe.api_key_from_env({}) is None
    assert er2_probe.api_key_from_env({'GEMINI_API_KEY': ' '}) is None
    assert er2_probe.api_key_from_env({'GOOGLE_API_KEY': 'k2'}) == 'k2'
    assert er2_probe.api_key_from_env({'GEMINI_API_KEY': 'k1', 'GOOGLE_API_KEY': 'k2'}) == 'k1'


def test_prompt_names_every_label_and_the_convention():
    prompt = er2_probe.build_prompt(['wooden block', 'empty pocket'])
    assert 'wooden block' in prompt and 'empty pocket' in prompt
    assert '[y, x]' in prompt and '0 to 1000' in prompt
    assert 'no code fence' in prompt


@pytest.mark.parametrize('text', [
    '[{"point": [512, 300], "label": "block"}]',
    '```json\n[{"point": [512, 300], "label": "block"}]\n```',
    '  ```\n[{"point": [512, 300.0], "label": "block"}]```  ',
])
def test_parse_accepts_bare_and_fenced_lists(text):
    assert er2_probe.parse_points(text) == [{'point': [512.0, 300.0], 'label': 'block'}]


@pytest.mark.parametrize('text', [
    'I can see a block near the center.',
    '{"point": [1, 2], "label": "x"}',
    '[{"point": [1200, 300], "label": "block"}]',
    '[{"point": [300], "label": "block"}]',
    '[{"label": "block"}]',
    '[{"point": [1, 2], "label": 7}]',
])
def test_parse_rejects_prose_and_malformed_answers(text):
    with pytest.raises(ValueError):
        er2_probe.parse_points(text)


def test_empty_list_means_nothing_seen():
    assert er2_probe.parse_points('[]') == []


def test_to_pixels_swaps_y_x_into_x_y():
    assert er2_probe.to_pixels([500, 250], 640, 480) == (160.0, 240.0)


def test_image_size_reads_png_and_jpeg_headers():
    assert er2_probe.image_size(PNG_HEADER) == (640, 480)
    assert er2_probe.image_size(JPEG_SOF0) == (640, 400)
    assert er2_probe.image_size(b'not an image') is None


def test_mime_type_by_extension():
    assert er2_probe.mime_type('frame.JPG') == 'image/jpeg'
    assert er2_probe.mime_type('frame.png') == 'image/png'
    with pytest.raises(ValueError):
        er2_probe.mime_type('frame.bmp')


def test_main_without_a_key_stops_before_any_query(monkeypatch, capsys):
    for var in er2_probe.KEY_VARS:
        monkeypatch.delenv(var, raising=False)

    def no_network(*args, **kwargs):
        raise AssertionError('query must not run without a key')
    monkeypatch.setattr(er2_probe, 'query', no_network)
    assert er2_probe.main(['frame.jpg', 'block']) == 2
    assert 'GEMINI_API_KEY' in capsys.readouterr().err


def test_main_reports_points_pixels_latency_and_usage(monkeypatch, capsys):
    monkeypatch.setenv('GEMINI_API_KEY', 'test-key')
    seen = {}

    def fake_query(image_path, labels, model, key, thinking=None):
        seen.update(image=image_path, labels=labels, model=model, key=key, thinking=thinking)
        return {'model': model, 'text': '[{"point": [500, 250], "label": "block"}]',
                'latency_s': 2.61, 'size': (640, 480),
                'usage': {'prompt_tokens': 1120, 'answer_tokens': 30,
                          'thinking_tokens': 0, 'total_tokens': 1150}}
    monkeypatch.setattr(er2_probe, 'query', fake_query)
    assert er2_probe.main(['frame.jpg', 'block', '--thinking', 'low']) == 0
    out = capsys.readouterr().out
    assert seen == {'image': 'frame.jpg', 'labels': ['block'], 'key': 'test-key',
                    'model': 'gemini-robotics-er-2-preview', 'thinking': 'low'}
    assert 'latency  2.61 s' in out
    assert 'pixels (160, 240)' in out
    assert 'prompt_tokens 1120' in out


def test_main_json_keeps_a_parse_failure_visible(monkeypatch, capsys):
    monkeypatch.setenv('GOOGLE_API_KEY', 'k')
    monkeypatch.setattr(er2_probe, 'query', lambda *a, **k: {
        'model': 'm', 'text': 'no idea', 'latency_s': 1.0, 'size': None, 'usage': {}})
    assert er2_probe.main(['frame.png', 'block', '--json']) == 0
    import json
    report = json.loads(capsys.readouterr().out)
    assert report['points'] == [] and 'not JSON' in report['parse_error']


def test_no_labels_asks_with_googles_own_pointing_prompt():
    # Without labels the model finds and names things itself; the prompt is
    # the one of the robotics overview, so results compare with the docs.
    prompt = er2_probe.build_prompt([])
    assert prompt == er2_probe.OPEN_PROMPT
    assert 'no more than 10 items' in prompt and '[y, x]' in prompt and '0-1000' in prompt


def test_main_runs_without_labels_and_annotates(monkeypatch, capsys, tmp_path):
    monkeypatch.setenv('GEMINI_API_KEY', 'test-key')
    seen, drawn = {}, {}
    monkeypatch.setattr(er2_probe, 'query', lambda image_path, labels, model, key, thinking=None: (
        seen.update(labels=labels) or
        {'model': model, 'text': '[{"point": [500, 250], "label": "box"}]', 'latency_s': 1.0,
         'size': (640, 480), 'usage': {}}))
    monkeypatch.setattr(er2_probe, 'annotate',
                        lambda image, points, out: drawn.update(image=image, points=points, out=out))
    out_path = str(tmp_path / 'seen.jpg')
    assert er2_probe.main(['frame.jpg', '--annotate', out_path]) == 0
    assert seen['labels'] == []
    assert drawn['out'] == out_path and drawn['points'][0]['label'] == 'box'


def test_annotate_draws_on_a_copy_of_the_same_size(tmp_path):
    Image = pytest.importorskip('PIL.Image')
    src, out = tmp_path / 'frame.jpg', tmp_path / 'seen.jpg'
    Image.new('RGB', (640, 480), (40, 40, 40)).save(src)
    er2_probe.annotate(str(src), [{'point': [500, 250], 'label': 'box'}], str(out))
    marked = Image.open(out)
    assert marked.size == (640, 480)
    # The point lands at x = 250/1000*640 = 160, y = 500/1000*480 = 240.
    r, g, b = marked.convert('RGB').getpixel((160, 240))
    assert r > 150 and g < 120
