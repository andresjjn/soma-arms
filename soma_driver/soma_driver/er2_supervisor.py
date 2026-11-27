"""Gemini Robotics ER 2 supervisor core for SOMA (v0.5).

PURE module (no ROS, lazy google-genai import): runs under pytest on any
machine with zero network calls.

Architecture rule (CLAUDE.md / docs/plan.md):
  "The cloud reasoner proposes; the armed driver disposes."

This module has no ROS dependency, holds no arming service or backend
handle, and never modifies SERVO_MAP limits or bypasses the 50 Hz
minimum-jerk ramp. Every motion proposal from ER 2 passes through a
strict tool whitelist and is verified at 50 Hz by tending.py /
kinematics.py before a single step list is returned to the executor.
"""
import csv
import json
import math
import os
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Protocol

from .homography import (
    DEFAULT_MAX_LATERAL_ERROR_MM,
    HomographyCalibration,
    assign_lane,
    er2_point_to_pixel,
)
from .kinematics import SIDES, load_dimensions
from .primitives import HOME
from .tending import (
    DEFAULT_POCKETS_MM,
    DEFAULT_PICK_Z_MM,
    TendingStep,
    plan_pick_and_place,
    plan_tending_cycle,
    solve_with_tilt,
)

DEFAULT_MODEL = os.environ.get('SOMA_ER2_MODEL', 'gemini-robotics-er-2-preview')
DEFAULT_INPUT_USD_PER_M = 1.0
DEFAULT_OUTPUT_USD_PER_M = 5.0
_FENCE = re.compile(r'^\s*```[a-zA-Z]*\s*(.*?)\s*```\s*$', re.DOTALL)


class Er2ApiError(RuntimeError):
    """Raised when the cloud reasoner is unreachable, rate-limited, or invalid."""


@dataclass(frozen=True)
class Er2Response:
    """Normalized response from any Er2Backend implementation."""

    text: str
    latency_s: float
    prompt_tokens: int = 0
    answer_tokens: int = 0
    thinking_tokens: int = 0
    total_tokens: int = 0
    model: str = DEFAULT_MODEL


class Er2Backend(Protocol):
    """Thin adapter boundary for Gemini Robotics ER 2."""

    def generate(self, image_bytes: bytes, mime_type: str,
                 prompt: str, thinking: str | None = 'low') -> Er2Response:
        ...


class GeminiEr2Client:
    """Live Gemini Robotics ER 2 client (lazy-imports google-genai)."""

    def __init__(self, api_key: str | None = None,
                 model: str = DEFAULT_MODEL) -> None:
        key = (api_key if api_key is not None
               else os.environ.get('GEMINI_API_KEY') or os.environ.get('GOOGLE_API_KEY') or '').strip()
        if not key:
            raise ValueError('GEMINI_API_KEY or GOOGLE_API_KEY must be set in the environment')
        self.api_key = key
        self.model = model

    def generate(self, image_bytes: bytes, mime_type: str,
                 prompt: str, thinking: str | None = 'low') -> Er2Response:
        from google import genai
        from google.genai import types

        cfg_kwargs: dict = {'temperature': 0.2}
        if thinking:
            try:
                cfg_kwargs['thinking_config'] = types.ThinkingConfig(thinking_level=thinking)
            except TypeError:
                pass
        client = genai.Client(api_key=self.api_key)
        t0 = time.perf_counter()
        try:
            resp = client.models.generate_content(
                model=self.model,
                contents=[types.Part.from_bytes(data=image_bytes, mime_type=mime_type), prompt],
                config=types.GenerateContentConfig(**cfg_kwargs),
            )
        except Exception as exc:
            raise Er2ApiError(str(exc)) from exc
        latency = time.perf_counter() - t0
        usage = getattr(resp, 'usage_metadata', None)
        return Er2Response(
            text=resp.text or '',
            latency_s=latency,
            prompt_tokens=int(getattr(usage, 'prompt_token_count', 0) or 0),
            answer_tokens=int(getattr(usage, 'candidates_token_count', 0) or 0),
            thinking_tokens=int(getattr(usage, 'thoughts_token_count', 0) or 0),
            total_tokens=int(getattr(usage, 'total_token_count', 0) or 0),
            model=self.model,
        )


# ---- Strict tool whitelist (cloud proposes; driver disposes) ----

TOOLS: tuple[dict, ...] = (
    {
        'name': 'pick_and_place',
        'description': 'Move one block between two reachable x_mm positions on a single arm lane.',
        'parameters': {
            'type': 'OBJECT',
            'properties': {
                'side': {'type': 'STRING', 'enum': list(SIDES)},
                'src_x_mm': {'type': 'NUMBER'},
                'dst_x_mm': {'type': 'NUMBER'},
            },
            'required': ['side', 'src_x_mm', 'dst_x_mm'],
        },
    },
    {
        'name': 'run_tending_cycle',
        'description': 'Run one full INPUT -> MACHINE -> OUTPUT tending cycle on one arm lane.',
        'parameters': {
            'type': 'OBJECT',
            'properties': {
                'side': {'type': 'STRING', 'enum': list(SIDES)},
            },
            'required': ['side'],
        },
    },
    {
        'name': 'go_home',
        'description': 'Return one arm safely to its hanging home pose.',
        'parameters': {
            'type': 'OBJECT',
            'properties': {
                'side': {'type': 'STRING', 'enum': list(SIDES)},
            },
            'required': ['side'],
        },
    },
)


def _strip_fence(text: str) -> str:
    body = text.strip()
    m = _FENCE.match(body)
    return m.group(1).strip() if m else body


def parse_points(text: str) -> list[dict]:
    """Parse ER 2 pointing JSON: [{'point': [y, x], 'label': str}, ...]."""
    try:
        data = json.loads(_strip_fence(text))
    except json.JSONDecodeError as exc:
        raise ValueError(f'pointing response is not valid JSON: {exc}') from exc
    if not isinstance(data, list):
        raise ValueError('pointing response must be a JSON list')
    points = []
    for entry in data:
        if not isinstance(entry, dict) or 'point' not in entry or 'label' not in entry:
            raise ValueError(f'pointing entry missing point or label: {entry!r}')
        pt = entry['point']
        if (not isinstance(pt, (list, tuple)) or len(pt) != 2
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in pt)):
            raise ValueError(f'point must be [y, x] numbers: {pt!r}')
        y_val, x_val = float(pt[0]), float(pt[1])
        if not (math.isfinite(y_val) and math.isfinite(x_val)):
            raise ValueError(f'point must be finite: {pt!r}')
        if not (0.0 <= y_val <= 1000.0 and 0.0 <= x_val <= 1000.0):
            raise ValueError(f'point outside 0..1000: {pt!r}')
        if not isinstance(entry['label'], str):
            raise ValueError(f'label must be a string: {entry["label"]!r}')
        points.append({'point': [y_val, x_val], 'label': entry['label']})
    return points


@dataclass(frozen=True)
class VerificationResult:
    success: bool
    part_id: str | None
    reason: str


def parse_verification(text: str) -> VerificationResult:
    """Parse ER 2 end-of-cycle verification JSON."""
    try:
        data = json.loads(_strip_fence(text))
    except json.JSONDecodeError as exc:
        raise ValueError(f'verification response is not valid JSON: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError('verification response must be a JSON object')
    success = data.get('success')
    if not isinstance(success, bool):
        raise ValueError(f'verification success must be a bool: {success!r}')
    part_id = data.get('part_id')
    if part_id is not None and not isinstance(part_id, str):
        raise ValueError(f'part_id must be a string or null: {part_id!r}')
    reason = data.get('reason', '')
    if not isinstance(reason, str):
        raise ValueError(f'reason must be a string: {reason!r}')
    return VerificationResult(success=success, part_id=part_id, reason=reason)


def execute_tool_call(name: str,
                      args: dict | None,
                      dims: dict[str, float] | None = None
                      ) -> tuple[dict, list[TendingStep]]:
    """Validate a whitelisted tool call and build its 50 Hz-verified steps.

    Returns (response_dict, steps). Never raises on invalid/unsafe calls;
    returns ({'ok': False, 'error': ...}, []) instead so the model receives
    a clean refusal.
    """
    payload = dict(args or {})
    if name == 'pick_and_place':
        if set(payload) != {'side', 'src_x_mm', 'dst_x_mm'}:
            return {'ok': False, 'error': 'expected exact keys: side, src_x_mm, dst_x_mm'}, []
        side = payload['side']
        src_x, dst_x = payload['src_x_mm'], payload['dst_x_mm']
        if side not in SIDES:
            return {'ok': False, 'error': f'invalid side {side!r}'}, []
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                   for v in (src_x, dst_x)):
            return {'ok': False, 'error': 'src_x_mm and dst_x_mm must be finite numbers'}, []
        try:
            steps = plan_pick_and_place(side, float(src_x), float(dst_x), dims=dims)
        except ValueError as exc:
            return {'ok': False, 'error': str(exc)}, []
        return {'ok': True, 'steps': len(steps)}, steps

    if name == 'run_tending_cycle':
        if set(payload) != {'side'}:
            return {'ok': False, 'error': 'expected exact key: side'}, []
        side = payload['side']
        if side not in SIDES:
            return {'ok': False, 'error': f'invalid side {side!r}'}, []
        try:
            steps = plan_tending_cycle(side, dims=dims)
        except ValueError as exc:
            return {'ok': False, 'error': str(exc)}, []
        return {'ok': True, 'steps': len(steps)}, steps

    if name == 'go_home':
        if set(payload) != {'side'}:
            return {'ok': False, 'error': 'expected exact key: side'}, []
        side = payload['side']
        if side not in SIDES:
            return {'ok': False, 'error': f'invalid side {side!r}'}, []
        prefix = f'{side}_arm_'
        home = {k: v for k, v in HOME.items() if k.startswith(prefix)}
        return {'ok': True, 'steps': 1}, [TendingStep('home', home, 1.5)]

    return {'ok': False, 'error': f'tool {name!r} is outside the whitelist'}, []


# ---- Telemetry & counters ----

@dataclass(frozen=True)
class CallMetrics:
    timestamp: str
    model: str
    operation: str
    latency_s: float
    prompt_tokens: int
    answer_tokens: int
    thinking_tokens: int
    total_tokens: int
    estimated_cost_usd: float
    error: str = ''


def estimate_cost_usd(prompt_tokens: int,
                      answer_tokens: int,
                      thinking_tokens: int = 0,
                      input_usd_per_m: float = DEFAULT_INPUT_USD_PER_M,
                      output_usd_per_m: float = DEFAULT_OUTPUT_USD_PER_M) -> float:
    out_tokens = answer_tokens + thinking_tokens
    return (prompt_tokens * input_usd_per_m + out_tokens * output_usd_per_m) / 1_000_000.0


@dataclass
class CycleCounters:
    verified: int = 0
    unverified: int = 0
    failed: int = 0
    recovered: int = 0

    @property
    def total_completed(self) -> int:
        return self.verified + self.unverified


@dataclass(frozen=True)
class LocalizedTarget:
    label: str
    point_yx: tuple[float, float]
    x_mm: float
    y_mm: float
    side: str | None
    lateral_offset_mm: float | None
    reachable: bool
    reason: str = ''


@dataclass(frozen=True)
class CycleOutcome:
    status: str                  # 'verified' | 'recovered' | 'failed' | 'unverified_fallback'
    side: str
    part_id: str | None = None
    reason: str = ''


class SupervisorSession:
    """Coordinates ER 2 pointing, verification, recovery, and fallback."""

    def __init__(self,
                 backend: Er2Backend,
                 calibration: HomographyCalibration,
                 dims: dict[str, float] | None = None,
                 input_usd_per_m: float = DEFAULT_INPUT_USD_PER_M,
                 output_usd_per_m: float = DEFAULT_OUTPUT_USD_PER_M) -> None:
        self.backend = backend
        self.cal = calibration
        self.dims = load_dimensions() if dims is None else dims
        self.input_usd_per_m = input_usd_per_m
        self.output_usd_per_m = output_usd_per_m
        self.counters = CycleCounters()
        self.usage_log: list[CallMetrics] = []
        self.seen_part_ids: list[str] = []

    def _record_usage(self, operation: str, resp: Er2Response, error: str = '') -> None:
        cost = estimate_cost_usd(
            resp.prompt_tokens, resp.answer_tokens, resp.thinking_tokens,
            self.input_usd_per_m, self.output_usd_per_m,
        )
        self.usage_log.append(CallMetrics(
            timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
            model=resp.model,
            operation=operation,
            latency_s=resp.latency_s,
            prompt_tokens=resp.prompt_tokens,
            answer_tokens=resp.answer_tokens,
            thinking_tokens=resp.thinking_tokens,
            total_tokens=resp.total_tokens,
            estimated_cost_usd=cost,
            error=error,
        ))

    def locate_part(self,
                    image_bytes: bytes,
                    mime_type: str = 'image/jpeg',
                    label: str = 'wooden block',
                    pick_z_mm: float = DEFAULT_PICK_Z_MM,
                    max_lateral_error_mm: float = DEFAULT_MAX_LATERAL_ERROR_MM
                    ) -> LocalizedTarget | None:
        """Ask ER 2 to point at `label` and check if an arm can reach it."""
        prompt = (
            f'Point to the {label} in the image. Answer with a JSON list only, '
            'no prose: [{"point": [y, x], "label": "<item>"}] with [y, x] in 0 to 1000. '
            'Return [] if not visible.'
        )
        resp = self.backend.generate(image_bytes, mime_type, prompt, thinking='low')
        try:
            pts = parse_points(resp.text)
        except ValueError as exc:
            self._record_usage('locate', resp, error=str(exc))
            raise Er2ApiError(str(exc)) from exc
        self._record_usage('locate', resp)
        if not pts:
            return None
        pt = pts[0]['point']
        u, v = er2_point_to_pixel(pt, *self.cal.image_size)
        x_mm, y_mm = self.cal.pixel_to_deck(u, v)
        try:
            side, offset_mm = assign_lane(
                y_mm, dims=self.dims, max_lateral_error_mm=max_lateral_error_mm)
        except ValueError as exc:
            return LocalizedTarget(
                label=pts[0]['label'],
                point_yx=(pt[0], pt[1]),
                x_mm=x_mm,
                y_mm=y_mm,
                side=None,
                lateral_offset_mm=None,
                reachable=False,
                reason=str(exc),
            )
        try:
            solve_with_tilt(x_mm, pick_z_mm, side, dims=self.dims)
        except ValueError as exc:
            return LocalizedTarget(
                label=pts[0]['label'],
                point_yx=(pt[0], pt[1]),
                x_mm=x_mm,
                y_mm=y_mm,
                side=side,
                lateral_offset_mm=offset_mm,
                reachable=False,
                reason=str(exc),
            )
        return LocalizedTarget(
            label=pts[0]['label'],
            point_yx=(pt[0], pt[1]),
            x_mm=x_mm,
            y_mm=y_mm,
            side=side,
            lateral_offset_mm=offset_mm,
            reachable=True,
        )

    def verify_cycle_end(self,
                         image_bytes: bytes,
                         expected_pocket: str = 'OUTPUT',
                         mime_type: str = 'image/jpeg') -> VerificationResult:
        """Ask ER 2 whether the block landed in `expected_pocket` and read its ID."""
        prompt = (
            f'Verify if the wooden block is cleanly seated in {expected_pocket} and read '
            'its engraved ID if visible. Answer with JSON only: '
            '{"success": true/false, "part_id": "<id or null>", "reason": "<short note>"}.'
        )
        resp = self.backend.generate(image_bytes, mime_type, prompt, thinking='low')
        try:
            result = parse_verification(resp.text)
        except ValueError as exc:
            self._record_usage('verify', resp, error=str(exc))
            raise Er2ApiError(str(exc)) from exc
        self._record_usage('verify', resp)
        return result

    def run_cycle(self,
                  side: str,
                  executor: Callable[[list[TendingStep]], None],
                  capture_frame: Callable[[], bytes],
                  input_x_mm: float | None = None,
                  machine_x_mm: float | None = None,
                  output_x_mm: float | None = None,
                  recover_on_drop: bool = True) -> CycleOutcome:
        """Execute one tending cycle, verify it with ER 2, and handle drops/fallbacks."""
        d_in, d_mach, d_out = DEFAULT_POCKETS_MM[side]
        in_x = d_in if input_x_mm is None else input_x_mm
        mach_x = d_mach if machine_x_mm is None else machine_x_mm
        out_x = d_out if output_x_mm is None else output_x_mm

        steps = plan_tending_cycle(side, in_x, mach_x, out_x, dims=self.dims)
        executor(steps)

        try:
            frame_after = capture_frame()
            verdict = self.verify_cycle_end(frame_after, expected_pocket='OUTPUT')
        except Er2ApiError as exc:
            self.counters.unverified += 1
            return CycleOutcome(
                status='unverified_fallback',
                side=side,
                reason=f'ER 2 unavailable, continuing on geometry: {exc}',
            )

        if verdict.success:
            self.counters.verified += 1
            if verdict.part_id:
                self.seen_part_ids.append(verdict.part_id)
            return CycleOutcome(
                status='verified',
                side=side,
                part_id=verdict.part_id,
                reason=verdict.reason,
            )

        if recover_on_drop:
            try:
                loc = self.locate_part(capture_frame(), label='wooden block')
            except Er2ApiError as exc:
                self.counters.failed += 1
                return CycleOutcome(status='failed', side=side, reason=str(exc))

            if loc is not None and loc.reachable and loc.side == side:
                recovery_steps = plan_pick_and_place(side, loc.x_mm, out_x, dims=self.dims)
                executor(recovery_steps)
                try:
                    verdict2 = self.verify_cycle_end(capture_frame(), expected_pocket='OUTPUT')
                except Er2ApiError as exc:
                    self.counters.unverified += 1
                    return CycleOutcome(status='unverified_fallback', side=side, reason=str(exc))
                if verdict2.success:
                    self.counters.verified += 1
                    self.counters.recovered += 1
                    if verdict2.part_id:
                        self.seen_part_ids.append(verdict2.part_id)
                    return CycleOutcome(
                        status='recovered',
                        side=side,
                        part_id=verdict2.part_id,
                        reason='dropped part re-localized and placed',
                    )
            elif loc is not None and not loc.reachable:
                self.counters.failed += 1
                return CycleOutcome(
                    status='failed',
                    side=side,
                    reason=f'part outside reachable lane: {loc.reason}',
                )

        self.counters.failed += 1
        return CycleOutcome(status='failed', side=side, reason=verdict.reason)

    def write_usage_csv(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow([
                'timestamp', 'model', 'operation', 'latency_s',
                'prompt_tokens', 'answer_tokens', 'thinking_tokens',
                'total_tokens', 'estimated_cost_usd', 'error',
            ])
            for m in self.usage_log:
                w.writerow([
                    m.timestamp, m.model, m.operation, f'{m.latency_s:.3f}',
                    m.prompt_tokens, m.answer_tokens, m.thinking_tokens,
                    m.total_tokens, f'{m.estimated_cost_usd:.6f}', m.error,
                ])
