#!/usr/bin/env python3
"""One image, one question to Gemini Robotics ER 2: where are these things?

A standalone bench probe, no ROS. It exists to measure, before anything is
designed around the model, what one pointing call costs and how long it
takes on a real frame of this bench. The key comes from the environment
and from nowhere else; this repository never holds one.

    export GEMINI_API_KEY=...                 # an AI Studio auth key
    python3 scripts/er2_probe.py frame.jpg "wooden block" "empty pocket"
    python3 scripts/er2_probe.py frame.jpg block --thinking low --json

Prints the raw answer, the parsed points as normalized [y, x] in 0 to
1000 and in pixels, the latency, and the token usage. The model id is
`gemini-robotics-er-2-preview` unless SOMA_ER2_MODEL or --model says
otherwise: previews get retired (ER 1.5 lasted seven months, ER 1.6 four
and a half), so the id is configuration, not code.

The Google SDK (`pip install google-genai`) is imported only inside
query(), so this file imports and its parsers are tested where no SDK and
no network exist. The cloud is never in the safety path: this script
cannot publish a command, arm anything or reach the driver.
"""
import argparse
import json
import os
import re
import struct
import sys
import time

DEFAULT_MODEL = os.environ.get('SOMA_ER2_MODEL', 'gemini-robotics-er-2-preview')
KEY_VARS = ('GEMINI_API_KEY', 'GOOGLE_API_KEY')
THINKING_LEVELS = ('minimal', 'low', 'medium', 'high')
MIME_BY_EXT = {'.jpg': 'image/jpeg', '.jpeg': 'image/jpeg', '.png': 'image/png',
               '.webp': 'image/webp'}
_FENCE = re.compile(r'^\s*```[a-zA-Z]*\s*(.*?)\s*```\s*$', re.DOTALL)


def api_key_from_env(environ=None) -> str | None:
    environ = os.environ if environ is None else environ
    for var in KEY_VARS:
        value = environ.get(var, '').strip()
        if value:
            return value
    return None


def build_prompt(labels: list[str]) -> str:
    items = ', '.join(labels)
    return (f'Point to the following items in the image: {items}. '
            'Answer with a JSON list only, no code fence and no prose, one '
            'entry per item you can see: [{"point": [y, x], "label": "<item>"}]. '
            'Each point is [y, x] normalized to 0 to 1000 across the image. '
            'Omit items that are not visible.')


def parse_points(text: str) -> list[dict]:
    """The model's answer as [{'point': [y, x], 'label': str}, ...].

    Accepts a bare JSON list or one wrapped in a code fence. Raises
    ValueError on anything else, including points outside 0 to 1000.
    """
    body = text.strip()
    m = _FENCE.match(body)
    if m:
        body = m.group(1)
    try:
        data = json.loads(body)
    except json.JSONDecodeError as exc:
        raise ValueError(f'not JSON: {exc}') from exc
    if not isinstance(data, list):
        raise ValueError('expected a JSON list')
    points = []
    for entry in data:
        if not isinstance(entry, dict) or 'point' not in entry or 'label' not in entry:
            raise ValueError(f'entry without point and label: {entry!r}')
        point = entry['point']
        if (not isinstance(point, (list, tuple)) or len(point) != 2
                or not all(isinstance(v, (int, float)) for v in point)):
            raise ValueError(f'point must be [y, x]: {point!r}')
        if not all(0 <= v <= 1000 for v in point):
            raise ValueError(f'point outside 0 to 1000: {point!r}')
        if not isinstance(entry['label'], str):
            raise ValueError(f'label must be a string: {entry["label"]!r}')
        points.append({'point': [float(point[0]), float(point[1])],
                       'label': entry['label']})
    return points


def to_pixels(point, width: int, height: int) -> tuple[float, float]:
    """Normalized [y, x] to (x_px, y_px)."""
    return (point[1] / 1000.0 * width, point[0] / 1000.0 * height)


def image_size(data: bytes) -> tuple[int, int] | None:
    """(width, height) of a PNG or JPEG from its header, else None."""
    if data[:8] == b'\x89PNG\r\n\x1a\n' and data[12:16] == b'IHDR':
        w, h = struct.unpack('>II', data[16:24])
        return w, h
    if data[:2] == b'\xff\xd8':
        i = 2
        while i + 9 < len(data):
            if data[i] != 0xFF:
                return None
            marker = data[i + 1]
            if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
                i += 2
                continue
            length = struct.unpack('>H', data[i + 2:i + 4])[0]
            if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
                h, w = struct.unpack('>HH', data[i + 5:i + 9])
                return w, h
            i += 2 + length
    return None


def mime_type(path: str) -> str:
    ext = os.path.splitext(path)[1].lower()
    if ext not in MIME_BY_EXT:
        raise ValueError(f'unsupported image type {ext!r}; use jpg, png or webp')
    return MIME_BY_EXT[ext]


def query(image_path: str, labels: list[str], model: str, key: str,
          thinking: str | None = None) -> dict:
    """One generate_content call. Returns text, latency and token usage."""
    from google import genai                      # imported here on purpose
    from google.genai import types

    with open(image_path, 'rb') as f:
        data = f.read()
    config_kwargs = {'temperature': 0.5}
    if thinking:
        try:
            config_kwargs['thinking_config'] = types.ThinkingConfig(thinking_level=thinking)
        except TypeError:
            print(f'this SDK has no thinking_level; sending without it', file=sys.stderr)
    client = genai.Client(api_key=key)
    t0 = time.perf_counter()
    response = client.models.generate_content(
        model=model,
        contents=[types.Part.from_bytes(data=data, mime_type=mime_type(image_path)),
                  build_prompt(labels)],
        config=types.GenerateContentConfig(**config_kwargs))
    latency = time.perf_counter() - t0
    usage = getattr(response, 'usage_metadata', None)
    return {
        'model': model,
        'text': response.text or '',
        'latency_s': latency,
        'usage': {name: getattr(usage, attr, None) for name, attr in (
            ('prompt_tokens', 'prompt_token_count'),
            ('answer_tokens', 'candidates_token_count'),
            ('thinking_tokens', 'thoughts_token_count'),
            ('total_tokens', 'total_token_count'))},
        'size': image_size(data),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('image', help='jpg, png or webp frame of the bench')
    ap.add_argument('labels', nargs='+', help='what to point at')
    ap.add_argument('--model', default=DEFAULT_MODEL)
    ap.add_argument('--thinking', choices=THINKING_LEVELS, default=None,
                    help='thinking level; the docs suggest low for pointing')
    ap.add_argument('--json', action='store_true', help='machine readable output')
    args = ap.parse_args(argv)

    key = api_key_from_env()
    if not key:
        print('no API key: set ' + ' or '.join(KEY_VARS) + ' in the environment '
              '(never in a file of this repository)', file=sys.stderr)
        return 2

    result = query(args.image, args.labels, args.model, key, args.thinking)
    try:
        points = parse_points(result['text'])
        parse_error = None
    except ValueError as exc:
        points, parse_error = [], str(exc)
    for p in points:
        if result['size']:
            p['pixels'] = to_pixels(p['point'], *result['size'])

    if args.json:
        print(json.dumps({**result, 'points': points, 'parse_error': parse_error}, indent=2))
        return 0
    print(f"model    {result['model']}")
    print(f"latency  {result['latency_s']:.2f} s")
    print('usage    ' + ', '.join(f'{k} {v}' for k, v in result['usage'].items()))
    print(f"size     {result['size']}")
    print('answer   ' + result['text'].strip().replace('\n', ' '))
    if parse_error:
        print(f'parse    FAILED: {parse_error}')
    for p in points:
        px = p.get('pixels')
        where = f", pixels ({px[0]:.0f}, {px[1]:.0f})" if px else ''
        print(f"point    {p['label']}: [y {p['point'][0]:.0f}, x {p['point'][1]:.0f}]{where}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
