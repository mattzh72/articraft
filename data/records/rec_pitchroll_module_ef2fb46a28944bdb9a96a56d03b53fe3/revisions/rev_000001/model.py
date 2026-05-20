from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(x: float, y: float, z: float, cx: float, cy: float, cz: float):
    return cq.Workplane("XY").box(x, y, z).translate((cx, cy, cz))


def _cyl_x(length: float, radius: float, cx: float, cy: float, cz: float):
    return cq.Workplane("YZ").cylinder(length, radius).translate((cx, cy, cz))


def _cyl_y(length: float, radius: float, cx: float, cy: float, cz: float):
    return cq.Workplane("XZ").cylinder(length, radius).translate((cx, cy, cz))


def _make_side_support():
    base = _box(0.56, 0.82, 0.060, 0.0, 0.0, 0.030)
    side_a = _box(0.16, 0.075, 0.560, 0.0, 0.340, 0.310)
    side_b = _box(0.16, 0.075, 0.560, 0.0, -0.340, 0.310)

    # Raised bearing collars around the trunnion holes make the side support read
    # as a yoke rather than two plain plates.
    collar_a = _cyl_y(0.040, 0.078, 0.0, 0.292, 0.420)
    collar_b = _cyl_y(0.040, 0.078, 0.0, -0.292, 0.420)

    yoke = base.union(side_a).union(side_b).union(collar_a).union(collar_b)
    for y in (0.324, -0.324):
        yoke = yoke.cut(_cyl_y(0.180, 0.041, 0.0, y, 0.420))
    return yoke.clean()


def _make_pitch_frame():
    # A compact pitch frame: a roll-bearing annulus in the middle, a rectangular
    # cradle around it, and two short side trunnions captured by the yoke.
    bearing = _cyl_x(0.090, 0.105, 0.0, 0.0, 0.0).cut(_cyl_x(0.130, 0.054, 0.0, 0.0, 0.0))
    top_bar = _box(0.060, 0.430, 0.040, 0.0, 0.0, 0.135)
    bottom_bar = _box(0.060, 0.430, 0.040, 0.0, 0.0, -0.135)
    side_bar_a = _box(0.065, 0.045, 0.300, 0.0, 0.205, 0.0)
    side_bar_b = _box(0.065, 0.045, 0.300, 0.0, -0.205, 0.0)
    spoke_a = _box(0.060, 0.105, 0.045, 0.0, 0.145, 0.0)
    spoke_b = _box(0.060, 0.105, 0.045, 0.0, -0.145, 0.0)
    trunnion_a = _cyl_y(0.220, 0.043, 0.0, 0.300, 0.0)
    trunnion_b = _cyl_y(0.220, 0.043, 0.0, -0.300, 0.0)
    cap_a = _cyl_y(0.022, 0.043, 0.0, 0.415, 0.0)
    cap_b = _cyl_y(0.022, 0.043, 0.0, -0.415, 0.0)

    frame = (
        bearing.union(top_bar)
        .union(bottom_bar)
        .union(side_bar_a)
        .union(side_bar_b)
        .union(spoke_a)
        .union(spoke_b)
        .union(trunnion_a)
        .union(trunnion_b)
        .union(cap_a)
        .union(cap_b)
    )
    return frame.clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_spindle_pitch_roll_unit")

    painted_casting = model.material("matte_graphite_casting", rgba=(0.10, 0.11, 0.12, 1.0))
    bearing_steel = model.material("brushed_bearing_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    face_blue = model.material("anodized_front_face", rgba=(0.08, 0.17, 0.32, 1.0))
    index_yellow = model.material("painted_index_mark", rgba=(1.0, 0.78, 0.10, 1.0))

    support = model.part("side_support")
    support.visual(
        mesh_from_cadquery(_make_side_support(), "side_support_yoke", tolerance=0.0007),
        material=painted_casting,
        name="yoke",
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_make_pitch_frame(), "pitch_frame_cradle", tolerance=0.0007),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bearing_steel,
        name="cradle",
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=0.056, length=0.300),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )
    roll_spindle.visual(
        Cylinder(radius=0.083, length=0.036),
        origin=Origin(xyz=(0.203, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=face_blue,
        name="front_face",
    )
    roll_spindle.visual(
        Box((0.010, 0.016, 0.116)),
        origin=Origin(xyz=(0.224, 0.0, 0.0)),
        material=index_yellow,
        name="face_index",
    )

    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=support,
        child=pitch_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("side_support")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")
    pitch = object_model.get_articulation("pitch")
    roll = object_model.get_articulation("roll")

    ctx.allow_overlap(
        support,
        pitch_frame,
        elem_a="yoke",
        elem_b="cradle",
        reason=(
            "The trunnion stubs are intentionally modeled as a tiny hidden "
            "press-fit inside the yoke bearing holes so the side-supported "
            "pitch axis has a real captured support path."
        ),
    )
    ctx.allow_overlap(
        pitch_frame,
        roll_spindle,
        elem_a="cradle",
        elem_b="shaft",
        reason=(
            "The roll shaft is intentionally given a tiny hidden bearing "
            "interference inside the pitch-frame bore to represent a captured "
            "spindle fit."
        ),
    )

    ctx.check(
        "pitch joint uses side trunnion axis",
        tuple(round(v, 6) for v in pitch.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pitch.axis}",
    )
    ctx.check(
        "roll joint uses spindle axis",
        tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll.axis}",
    )
    ctx.expect_within(
        roll_spindle,
        pitch_frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="cradle",
        margin=0.002,
        name="spindle shaft is centered in the pitch frame bearing",
    )
    ctx.expect_overlap(
        roll_spindle,
        pitch_frame,
        axes="x",
        elem_a="shaft",
        elem_b="cradle",
        min_overlap=0.075,
        name="spindle passes through the short roll bearing",
    )
    ctx.expect_within(
        pitch_frame,
        support,
        axes="yz",
        inner_elem="cradle",
        outer_elem="yoke",
        margin=0.060,
        name="pitch frame sits between the side supports",
    )
    ctx.expect_overlap(
        pitch_frame,
        support,
        axes="y",
        elem_a="cradle",
        elem_b="yoke",
        min_overlap=0.090,
        name="trunnion stubs enter both yoke bearing sides",
    )

    rest_face = ctx.part_element_world_aabb(roll_spindle, elem="front_face")
    with ctx.pose({pitch: 0.50}):
        pitched_face = ctx.part_element_world_aabb(roll_spindle, elem="front_face")

    rest_center_z = (rest_face[0][2] + rest_face[1][2]) / 2.0 if rest_face else None
    pitched_center_z = (pitched_face[0][2] + pitched_face[1][2]) / 2.0 if pitched_face else None
    ctx.check(
        "pitch motion tilts the spindle face",
        rest_center_z is not None
        and pitched_center_z is not None
        and pitched_center_z < rest_center_z - 0.040,
        details=f"rest_z={rest_center_z}, pitched_z={pitched_center_z}",
    )

    rest_index = ctx.part_element_world_aabb(roll_spindle, elem="face_index")
    with ctx.pose({roll: math.pi / 2.0}):
        rolled_index = ctx.part_element_world_aabb(roll_spindle, elem="face_index")

    def _span(aabb, idx):
        return (aabb[1][idx] - aabb[0][idx]) if aabb else None

    ctx.check(
        "roll motion rotates the face index mark",
        rest_index is not None
        and rolled_index is not None
        and _span(rolled_index, 1) > _span(rest_index, 1) + 0.060
        and _span(rolled_index, 2) < _span(rest_index, 2) - 0.060,
        details=f"rest={rest_index}, rolled={rolled_index}",
    )

    return ctx.report()


object_model = build_object_model()
