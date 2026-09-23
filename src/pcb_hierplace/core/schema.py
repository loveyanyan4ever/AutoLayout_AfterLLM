"""Immutable design data. Coordinates: mm, X right, Y up, CCW degrees."""
from __future__ import annotations
from dataclasses import dataclass, asdict, field
from enum import StrEnum
from typing import Any
import hashlib
import json
import math
import time

Point = tuple[float, float]
Polygon = tuple[Point, ...]
BBox = tuple[float, float, float, float]


class Status(StrEnum):
    FEASIBLE = "FEASIBLE"
    INPUT_INVALID = "INPUT_INVALID"
    UNSUPPORTED_FEATURE = "UNSUPPORTED_FEATURE"
    CONSTRAINT_CONFLICT = "CONSTRAINT_CONFLICT"
    NO_FEASIBLE_FOUND = "NO_FEASIBLE_FOUND"
    MODEL_INFEASIBLE = "MODEL_INFEASIBLE"
    PROVEN_INFEASIBLE = "PROVEN_INFEASIBLE"
    NUMERICAL_FAILURE = "NUMERICAL_FAILURE"


class PlacementError(Exception):
    def __init__(self, code: str, message: str, status=Status.INPUT_INVALID, details=None):
        super().__init__(message)
        self.code, self.status, self.details = code, status, details or {}

    def as_dict(self):
        return {"status": str(self.status), "code": self.code,
                "message": str(self), "details": self.details}


@dataclass(frozen=True)
class Pose:
    ref: str
    x: float
    y: float
    angle: float = 0.
    layer: str = "top"


@dataclass(frozen=True)
class Pad:
    id: str
    number: str
    polygon: Polygon
    center: Point
    net: str = ""
    layer: str = "top"
    hole_mm: float = 0.
    geometry_source: str = "source_rectangle"
    hole_polygon: Polygon = ()
    hole_center: Point | None = None


@dataclass(frozen=True)
class Component:
    uuid: str
    ref: str
    footprint: str
    source_pose: Pose
    body: Polygon
    pads: tuple[Pad, ...]
    locked: bool = False
    body_source: str = "explicit_override"


@dataclass(frozen=True)
class Net:
    id: str
    pins: tuple[tuple[str, str], ...]  # reference, stable pad ID


@dataclass(frozen=True)
class DesignSnapshot:
    board_id: str
    source_path: str
    source_hash: str
    adapter: str
    source_unit: str
    origin: Point
    outline: Polygon
    components: tuple[Component, ...]
    nets: tuple[Net, ...]
    metadata: tuple[tuple[str, str], ...] = ()

    @property
    def outline_present(self):
        return bool(self.outline)

    @property
    def by_ref(self):
        return {c.ref: c for c in self.components}

    def source_state(self, board=None):
        return PlacementState(tuple(c.source_pose for c in self.components),
                              tuple(board or self.outline), tag="source")


@dataclass(frozen=True)
class Assignments:
    domains: tuple[tuple[str, str], ...]
    clusters: tuple[tuple[str, str], ...]
    logical_clusters: tuple[tuple[str, str], ...]
    changes: tuple[str, ...] = ()
    parent_clusters: tuple[tuple[str, str], ...] = ()


@dataclass(frozen=True)
class PlacementState:
    poses: tuple[Pose, ...]
    outline: Polygon
    regions: tuple[tuple[str, BBox], ...] = ()
    domain_regions: tuple[tuple[str, BBox], ...] = ()
    tag: str = ""
    choices: tuple[tuple[str, str], ...] = ()

    @property
    def by_ref(self):
        return {p.ref: p for p in self.poses}

    @property
    def digest(self):
        return digest(asdict(self))


@dataclass(frozen=True)
class Violation:
    rule: str
    objects: tuple[str, ...]
    message: str
    actual: float | None = None
    required: float | None = None


@dataclass(frozen=True)
class ValidationReport:
    violations: tuple[Violation, ...]
    min_copper_distance_mm: float | None = None
    creepage_status: str = "NOT_EVALUATED"
    safety_signoff: str = "NOT_PERFORMED"

    @property
    def model_feasible(self):
        return not self.violations

    def as_dict(self):
        return {**asdict(self), "model_feasible": self.model_feasible,
                "placement_rule_status": "PASSED" if self.model_feasible else "FAILED"}


@dataclass(frozen=True)
class CandidateRecord:
    state: PlacementState
    validation: ValidationReport
    metrics: dict
    objective_version: str = "exact-v1"
    priority_reference: tuple[tuple[str, float], ...] = ()


@dataclass
class StageResult:
    status: str
    best_feasible: CandidateRecord | None = None
    diagnostic_state: PlacementState | None = None
    metrics: dict = field(default_factory=dict)
    violations: list = field(default_factory=list)
    termination_reason: str = ""
    stats: dict = field(default_factory=dict)


@dataclass
class Budget:
    seconds: float
    start: float = field(default_factory=time.monotonic)

    @property
    def remaining(self):
        return max(0., self.seconds - (time.monotonic() - self.start))

    @property
    def expired(self):
        return self.remaining <= 0


def digest(data: Any) -> str:
    return hashlib.sha256(json.dumps(data, sort_keys=True, ensure_ascii=False,
                                    separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def file_hash(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def finite_number(value, field_name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise PlacementError("INVALID_NUMBER", f"{field_name} 必须是有限数值")
    return float(value)


def state_from_dict(data):
    return PlacementState(tuple(Pose(**p) for p in data["poses"]),
                          tuple(tuple(p) for p in data["outline"]),
                          tuple((k, tuple(v)) for k, v in data.get("regions", [])),
                          tuple((k, tuple(v)) for k, v in data.get("domain_regions", [])),
                          data.get("tag", ""), tuple(tuple(x) for x in data.get("choices", [])))
