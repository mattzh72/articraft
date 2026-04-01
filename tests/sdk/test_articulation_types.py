from __future__ import annotations

import pytest

from sdk import ArticulatedObject, MotionLimits, ValidationError
from sdk._core.v0._urdf_export import _articulation_element


def test_unknown_articulation_type() -> None:
    model = ArticulatedObject(name="planar_not_supported")
    base = model.part("base")
    slider = model.part("slider")

    with pytest.raises(ValidationError, match="Unknown articulation type"):
        model.articulation("base_to_slider", "planar", parent=base, child=slider)


def test_continuous_requires_motion_limits() -> None:
    model = ArticulatedObject(name="continuous_requires_limits")
    base = model.part("base")
    mast = model.part("mast")
    model.articulation("mast_slew", "continuous", parent=base, child=mast)

    with pytest.raises(
        ValidationError,
        match="must set motion_limits=MotionLimits\\(effort=..., velocity=...\\)",
    ):
        model.validate(strict=True)


def test_continuous_forbids_lower_upper_limits() -> None:
    model = ArticulatedObject(name="continuous_forbids_bounds")
    base = model.part("base")
    mast = model.part("mast")
    model.articulation(
        "mast_slew",
        "continuous",
        parent=base,
        child=mast,
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.0,
        ),
    )

    with pytest.raises(ValidationError, match="cannot include lower/upper limits"):
        model.validate(strict=True)


def test_continuous_accepts_effort_velocity_only() -> None:
    model = ArticulatedObject(name="continuous_valid")
    base = model.part("base")
    mast = model.part("mast")
    model.articulation(
        "mast_slew",
        "continuous",
        parent=base,
        child=mast,
        motion_limits=MotionLimits(effort=12.0, velocity=1.5),
    )
    model.validate(strict=True)


def test_floating_forbids_motion_limits() -> None:
    model = ArticulatedObject(name="floating_forbids_limits")
    base = model.part("base")
    payload = model.part("payload")
    model.articulation(
        "base_to_payload",
        "floating",
        parent=base,
        child=payload,
        motion_limits=MotionLimits(effort=1.0, velocity=1.0),
    )

    with pytest.raises(ValidationError, match="does not support motion limits"):
        model.validate(strict=True)


def test_urdf_export_normalizes_motion_axis() -> None:
    model = ArticulatedObject(name="normalized_axis_export")
    base = model.part("base")
    arm = model.part("arm")
    joint = model.articulation(
        "base_to_arm",
        "revolute",
        parent=base,
        child=arm,
        axis=(0.0, 0.0, 10.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-1.0, upper=1.0),
    )

    elem = _articulation_element(joint)
    axis_elem = elem.find("axis")
    assert axis_elem is not None
    assert axis_elem.attrib["xyz"] == "0 0 1"
