from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


ROLL_AXIS_Z = 0.092
PITCH_AXIS_Z = 0.044


def _pedestal_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").circle(0.052).extrude(0.012)
    column = cq.Workplane("XY").circle(0.016).extrude(0.054).translate((0.0, 0.0, 0.012))
    beam = cq.Workplane("XY").box(0.082, 0.022, 0.014).translate((0.0, 0.0, 0.073))
    left_cheek = cq.Workplane("XY").box(0.012, 0.022, 0.060).translate((-0.051, 0.0, 0.092))
    right_cheek = cq.Workplane("XY").box(0.012, 0.022, 0.060).translate((0.051, 0.0, 0.092))

    return foot.union(column).union(beam).union(left_cheek).union(right_cheek)


def _outer_gimbal_shape() -> cq.Workplane:
    roll_shaft = cq.Workplane("YZ").circle(0.006).extrude(0.090).translate((-0.045, 0.0, 0.0))
    left_plate = cq.Workplane("XY").box(0.010, 0.012, 0.088).translate((0.0, -0.032, 0.042))
    right_plate = cq.Workplane("XY").box(0.010, 0.012, 0.088).translate((0.0, 0.032, 0.042))
    top_bridge = cq.Workplane("XY").box(0.010, 0.076, 0.010).translate((0.0, 0.0, 0.084))

    return roll_shaft.union(left_plate).union(right_plate).union(top_bridge)


def _inner_cradle_shape() -> cq.Workplane:
    pitch_shaft = cq.Workplane("XZ").circle(0.0055).extrude(0.052, both=True)
    spine = cq.Workplane("XY").box(0.010, 0.010, 0.040).translate((-0.008, 0.0, 0.0))
    top_arm = cq.Workplane("XY").box(0.024, 0.010, 0.008).translate((0.002, 0.0, 0.016))
    bottom_arm = cq.Workplane("XY").box(0.024, 0.010, 0.008).translate((0.002, 0.0, -0.016))
    center_web = cq.Workplane("XY").box(0.010, 0.010, 0.016).translate((0.002, 0.0, 0.0))
    neck = cq.Workplane("XY").box(0.010, 0.010, 0.012).translate((0.014, 0.0, 0.0))
    flange = cq.Workplane("YZ").circle(0.014).extrude(0.004).translate((0.016, 0.0, 0.0))

    return pitch_shaft.union(spine).union(top_arm).union(bottom_arm).union(center_web).union(neck).union(flange)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_instrument_head")

    model.material("pedestal_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("ring_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("cradle_graphite", rgba=(0.16, 0.17, 0.19, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="pedestal_gray",
        name="foot",
    )
    pedestal.visual(
        Cylinder(radius=0.016, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material="pedestal_gray",
        name="column",
    )
    pedestal.visual(
        Box((0.086, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material="pedestal_gray",
        name="beam",
    )
    pedestal.visual(
        Box((0.012, 0.022, 0.060)),
        origin=Origin(xyz=(-0.047, 0.0, 0.092)),
        material="pedestal_gray",
        name="left_cheek",
    )
    pedestal.visual(
        Box((0.012, 0.022, 0.060)),
        origin=Origin(xyz=(0.047, 0.0, 0.092)),
        material="pedestal_gray",
        name="right_cheek",
    )

    outer_gimbal = model.part("outer_gimbal")
    outer_gimbal.visual(
        Cylinder(radius=0.006, length=0.082),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="ring_silver",
        name="roll_shaft",
    )
    outer_gimbal.visual(
        Box((0.010, 0.012, 0.072)),
        origin=Origin(xyz=(0.0, -0.032, 0.036)),
        material="ring_silver",
        name="left_plate",
    )
    outer_gimbal.visual(
        Box((0.010, 0.012, 0.072)),
        origin=Origin(xyz=(0.0, 0.032, 0.036)),
        material="ring_silver",
        name="right_plate",
    )
    outer_gimbal.visual(
        Box((0.030, 0.076, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material="ring_silver",
        name="top_bridge",
    )
    outer_gimbal.visual(
        Box((0.006, 0.010, 0.078)),
        origin=Origin(xyz=(-0.012, 0.0, 0.039)),
        material="ring_silver",
        name="left_spine",
    )
    outer_gimbal.visual(
        Box((0.016, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.032, PITCH_AXIS_Z)),
        material="ring_silver",
        name="rear_lug",
    )
    outer_gimbal.visual(
        Box((0.016, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.032, PITCH_AXIS_Z)),
        material="ring_silver",
        name="front_lug",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.0055, length=0.054),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="cradle_graphite",
        name="pitch_shaft",
    )
    inner_cradle.visual(
        Box((0.010, 0.010, 0.040)),
        origin=Origin(xyz=(-0.008, 0.0, -0.004)),
        material="cradle_graphite",
        name="spine",
    )
    inner_cradle.visual(
        Box((0.024, 0.010, 0.008)),
        origin=Origin(xyz=(0.002, 0.0, 0.016)),
        material="cradle_graphite",
        name="top_arm",
    )
    inner_cradle.visual(
        Box((0.024, 0.010, 0.008)),
        origin=Origin(xyz=(0.002, 0.0, -0.016)),
        material="cradle_graphite",
        name="bottom_arm",
    )
    inner_cradle.visual(
        Box((0.010, 0.010, 0.016)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material="cradle_graphite",
        name="center_web",
    )
    inner_cradle.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material="cradle_graphite",
        name="neck",
    )
    inner_cradle.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="cradle_graphite",
        name="flange",
    )

    model.articulation(
        "pedestal_to_outer_roll",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=outer_gimbal,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.2, lower=-1.3, upper=1.3),
    )
    model.articulation(
        "outer_to_inner_pitch",
        ArticulationType.REVOLUTE,
        parent=outer_gimbal,
        child=inner_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        # The flange projects along local +X, so -Y pitches it upward toward +Z.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=-0.6, upper=0.9),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    outer_gimbal = object_model.get_part("outer_gimbal")
    inner_cradle = object_model.get_part("inner_cradle")
    roll = object_model.get_articulation("pedestal_to_outer_roll")
    pitch = object_model.get_articulation("outer_to_inner_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(outer_gimbal, pedestal, name="outer_gimbal_supported_by_pedestal")
    ctx.expect_contact(inner_cradle, outer_gimbal, name="inner_cradle_supported_by_outer_gimbal")

    ctx.check(
        "roll_axis_is_lateral",
        tuple(roll.axis) == (1.0, 0.0, 0.0),
        details=f"expected roll axis (1, 0, 0), got {roll.axis}",
    )
    ctx.check(
        "pitch_axis_is_perpendicular",
        tuple(pitch.axis) == (0.0, -1.0, 0.0),
        details=f"expected pitch axis (0, -1, 0), got {pitch.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
