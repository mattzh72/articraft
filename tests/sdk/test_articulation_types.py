from __future__ import annotations

import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from sdk import ArticulatedObject, MotionLimits, ValidationError


def expect_validation_error(model: ArticulatedObject, message: str) -> None:
    try:
        model.validate(strict=True)
    except ValidationError as exc:
        assert message in str(exc)
    else:
        raise AssertionError(f"Expected ValidationError containing: {message}")


def test_unknown_articulation_type() -> None:
    model = ArticulatedObject(name="planar_not_supported")
    base = model.part("base")
    slider = model.part("slider")

    try:
        model.articulation("base_to_slider", "planar", parent=base, child=slider)
    except ValidationError as exc:
        assert "Unknown articulation type" in str(exc)
    else:
        raise AssertionError("Expected planar articulations to be rejected")


def test_continuous_requires_motion_limits() -> None:
    model = ArticulatedObject(name="continuous_requires_limits")
    base = model.part("base")
    mast = model.part("mast")
    model.articulation("mast_slew", "continuous", parent=base, child=mast)
    expect_validation_error(
        model,
        "must set motion_limits=MotionLimits(effort=..., velocity=...)",
    )


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
    expect_validation_error(model, "cannot include lower/upper limits")


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


def main() -> None:
    test_unknown_articulation_type()
    test_continuous_requires_motion_limits()
    test_continuous_forbids_lower_upper_limits()
    test_continuous_accepts_effort_velocity_only()


if __name__ == "__main__":
    main()
