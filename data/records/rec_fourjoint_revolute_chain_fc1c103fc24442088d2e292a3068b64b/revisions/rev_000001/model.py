from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


SEGMENT_LENGTHS = (0.70, 0.58, 0.48, 0.38)


def _add_hinge_fork(
    part,
    *,
    x: float,
    z: float,
    lower_name: str,
    upper_name: str,
    cap_name: str,
    metal: Material,
    accent: Material,
) -> None:
    """Add a C-shaped vertical-axis hinge fork with a clear middle slot."""

    # Lower and upper fork plates intentionally leave a middle gap for the
    # next segment's rotating knuckle and beam.
    part.visual(
        Box((0.16, 0.13, 0.014)),
        origin=Origin(xyz=(x, 0.0, z + 0.007)),
        material=metal,
        name=lower_name,
    )
    part.visual(
        Box((0.16, 0.13, 0.014)),
        origin=Origin(xyz=(x, 0.0, z + 0.053)),
        material=metal,
        name=upper_name,
    )
    # Rear web ties the two plates back into the rigid segment without entering
    # the rotating middle slot around the hinge pin line.
    part.visual(
        Box((0.045, 0.13, 0.060)),
        origin=Origin(xyz=(x - 0.072, 0.0, z + 0.030)),
        material=metal,
        name=f"{lower_name}_web",
    )
    part.visual(
        Box((0.11, 0.080, 0.007)),
        origin=Origin(xyz=(x, 0.0, z + 0.0615)),
        material=accent,
        name=cap_name,
    )


def _add_segment(
    part,
    *,
    length: float,
    metal: Material,
    hinge_metal: Material,
    cable: Material,
    accent: Material,
    with_distal_fork: bool,
    with_camera: bool = False,
) -> None:
    """Add one rigid arm segment in a frame whose origin is its proximal hinge."""

    # Rotating middle knuckle captured by the previous hinge fork.
    part.visual(
        Cylinder(radius=0.037, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=hinge_metal,
        name="proximal_boss",
    )

    beam_end = length - (0.075 if with_distal_fork else 0.035)
    beam_length = beam_end - 0.015
    part.visual(
        Box((beam_length, 0.046, 0.026)),
        origin=Origin(xyz=(0.015 + beam_length / 2.0, 0.0, 0.030)),
        material=metal,
        name="beam",
    )
    part.visual(
        Box((beam_length * 0.88, 0.014, 0.008)),
        origin=Origin(xyz=(0.030 + beam_length * 0.44, 0.0, 0.045)),
        material=cable,
        name="cable_strip",
    )

    if with_distal_fork:
        _add_hinge_fork(
            part,
            x=length,
            z=0.0,
            lower_name="distal_lower_plate",
            upper_name="distal_upper_plate",
            cap_name="distal_cap",
            metal=hinge_metal,
            accent=accent,
        )

    if with_camera:
        camera_black = cable
        # A short fixed inspection head makes the purpose clear while remaining
        # part of the last rigid segment rather than adding a fifth joint.
        part.visual(
            Box((0.11, 0.064, 0.036)),
            origin=Origin(xyz=(length - 0.012, 0.0, 0.030)),
            material=hinge_metal,
            name="sensor_neck",
        )
        part.visual(
            Box((0.145, 0.092, 0.076)),
            origin=Origin(xyz=(length + 0.065, 0.0, 0.030)),
            material=camera_black,
            name="camera_body",
        )
        part.visual(
            Cylinder(radius=0.034, length=0.040),
            origin=Origin(xyz=(length + 0.153, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name="lens_barrel",
        )
        part.visual(
            Cylinder(radius=0.025, length=0.010),
            origin=Origin(xyz=(length + 0.176, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=Material(name="blue_glass", rgba=(0.10, 0.22, 0.32, 1.0)),
            name="glass_face",
        )
        for idx, (y, z) in enumerate(((-0.032, 0.055), (0.032, 0.055), (-0.032, 0.005), (0.032, 0.005))):
            part.visual(
                Sphere(radius=0.008),
                origin=Origin(xyz=(length + 0.138, y, z)),
                material=accent,
                name=f"led_{idx}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_inspection_arm")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.006, 0.007, 1.0))
    safety_orange = model.material("safety_orange", rgba=(1.0, 0.42, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.42, 0.30, 0.050)),
        origin=Origin(xyz=(-0.075, 0.0, 0.025)),
        material=matte_black,
        name="floor_plate",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=matte_black,
        name="column",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=dark_steel,
        name="turntable_collar",
    )
    _add_hinge_fork(
        base,
        x=0.0,
        z=0.420,
        lower_name="lower_plate",
        upper_name="upper_plate",
        cap_name="hinge_cap",
        metal=dark_steel,
        accent=safety_orange,
    )

    segments = []
    for idx, length in enumerate(SEGMENT_LENGTHS):
        segment = model.part(f"segment_{idx}")
        _add_segment(
            segment,
            length=length,
            metal=aluminum,
            hinge_metal=dark_steel,
            cable=rubber,
            accent=safety_orange,
            with_distal_fork=idx < 3,
            with_camera=idx == 3,
        )
        segments.append(segment)

    hinge_limits = MotionLimits(effort=35.0, velocity=1.8, lower=-1.35, upper=1.35)
    model.articulation(
        "hinge_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=segments[0],
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    for idx in range(3):
        model.articulation(
            f"hinge_{idx + 1}",
            ArticulationType.REVOLUTE,
            parent=segments[idx],
            child=segments[idx + 1],
            origin=Origin(xyz=(SEGMENT_LENGTHS[idx], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=hinge_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    segments = [object_model.get_part(f"segment_{idx}") for idx in range(4)]
    hinges = [object_model.get_articulation(f"hinge_{idx}") for idx in range(4)]

    ctx.check(
        "four rigid segments",
        all(segment is not None for segment in segments) and len(segments) == 4,
        details="Expected exactly four named rigid segment links.",
    )
    ctx.check(
        "four hinge chain",
        all(hinge is not None for hinge in hinges)
        and [(hinge.parent, hinge.child) for hinge in hinges]
        == [
            ("base", "segment_0"),
            ("segment_0", "segment_1"),
            ("segment_1", "segment_2"),
            ("segment_2", "segment_3"),
        ],
        details="Expected one serial chain: base -> segment_0 -> segment_1 -> segment_2 -> segment_3.",
    )
    ctx.check(
        "all joints are simple hinges",
        all(
            hinge.articulation_type == ArticulationType.REVOLUTE
            and tuple(hinge.axis) == (0.0, 0.0, 1.0)
            and hinge.motion_limits is not None
            and hinge.motion_limits.lower == -1.35
            and hinge.motion_limits.upper == 1.35
            for hinge in hinges
        ),
        details="Each connection should be a limited revolute hinge about the vertical pin axis.",
    )

    # The middle knuckle of each child sits in the visible gap between the
    # parent's lower and upper hinge plates without relying on overlap.
    hinge_clearance_checks = [
        ("base", "segment_0", "lower_plate", "upper_plate"),
        ("segment_0", "segment_1", "distal_lower_plate", "distal_upper_plate"),
        ("segment_1", "segment_2", "distal_lower_plate", "distal_upper_plate"),
        ("segment_2", "segment_3", "distal_lower_plate", "distal_upper_plate"),
    ]
    for parent, child, lower, upper in hinge_clearance_checks:
        ctx.expect_gap(
            child,
            parent,
            axis="z",
            positive_elem="proximal_boss",
            negative_elem=lower,
            min_gap=0.001,
            max_gap=0.008,
            name=f"{child} clears lower hinge plate",
        )
        ctx.expect_gap(
            parent,
            child,
            axis="z",
            positive_elem=upper,
            negative_elem="proximal_boss",
            min_gap=0.001,
            max_gap=0.008,
            name=f"{child} clears upper hinge plate",
        )

    rest_tip = ctx.part_world_position(segments[3])
    with ctx.pose({"hinge_0": 0.50, "hinge_1": -0.70, "hinge_2": 0.60, "hinge_3": -0.40}):
        posed_tip = ctx.part_world_position(segments[3])
    ctx.check(
        "serial hinges steer the inspection end",
        rest_tip is not None
        and posed_tip is not None
        and abs(posed_tip[1] - rest_tip[1]) > 0.10,
        details=f"rest={rest_tip}, posed={posed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
