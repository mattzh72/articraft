from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


BASE_SIZE = (0.42, 0.42, 0.05)
COLUMN_RADIUS = 0.045
COLUMN_HEIGHT = 0.62
THRUST_RADIUS = 0.085
THRUST_HEIGHT = 0.02

HUB_RADIUS = 0.09
HUB_HEIGHT = 0.06
ARM_BEAM_LENGTH = 0.72
ARM_BEAM_WIDTH = 0.09
ARM_BEAM_HEIGHT = 0.06

CARRIAGE_LENGTH = 0.16
CARRIAGE_WIDTH = 0.14
CARRIAGE_BASE_HEIGHT = 0.055
HEAD_PLATE_THICKNESS = 0.014
HEAD_PLATE_WIDTH = 0.12
HEAD_PLATE_HEIGHT = 0.14

ARM_SWING_LIMIT = 2.4
CARRIAGE_ORIGIN_X = 0.26
CARRIAGE_LOWER = -0.06
CARRIAGE_UPPER = 0.28


def _make_arm_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(HUB_RADIUS).extrude(HUB_HEIGHT)

    beam = (
        cq.Workplane("XY")
        .box(ARM_BEAM_LENGTH, ARM_BEAM_WIDTH, ARM_BEAM_HEIGHT)
        .translate((HUB_RADIUS + ARM_BEAM_LENGTH / 2.0, 0.0, ARM_BEAM_HEIGHT / 2.0))
    )

    rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.03, 0.0),
                (0.16, 0.0),
                (0.31, 0.045),
                (0.06, 0.045),
            ]
        )
        .close()
        .extrude(0.018, both=True)
    )

    return hub.union(beam).union(rib)


def _make_carriage_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BASE_HEIGHT)
        .translate((0.0, 0.0, CARRIAGE_BASE_HEIGHT / 2.0))
    )

    rear_block = (
        cq.Workplane("XY")
        .box(0.05, 0.10, 0.10)
        .translate((-0.04, 0.0, 0.105))
    )

    head_plate_x = (CARRIAGE_LENGTH / 2.0) - (HEAD_PLATE_THICKNESS / 2.0)
    head_plate_z = CARRIAGE_BASE_HEIGHT + (HEAD_PLATE_HEIGHT / 2.0)

    head_plate = (
        cq.Workplane("XY")
        .box(HEAD_PLATE_THICKNESS, HEAD_PLATE_WIDTH, HEAD_PLATE_HEIGHT)
        .translate((head_plate_x, 0.0, head_plate_z))
    )

    tool_mount = (
        cq.Workplane("YZ")
        .circle(0.022)
        .extrude(0.018)
        .translate((CARRIAGE_LENGTH / 2.0, 0.0, head_plate_z))
    )

    return base.union(rear_block).union(head_plate).union(tool_mount)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_positioning_arm")

    base_color = model.material("base_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    column_color = model.material("column_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    arm_color = model.material("arm_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    carriage_color = model.material("carriage_blue", rgba=(0.16, 0.34, 0.62, 1.0))

    base = model.part("base_column")
    base.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] / 2.0)),
        material=base_color,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] + (COLUMN_HEIGHT / 2.0))),
        material=column_color,
        name="column_shaft",
    )
    base.visual(
        Cylinder(radius=THRUST_RADIUS, length=THRUST_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_SIZE[2] + COLUMN_HEIGHT + (THRUST_HEIGHT / 2.0),
            )
        ),
        material=column_color,
        name="thrust_bearing",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_make_arm_shape(), "arm_assembly"),
        origin=Origin(),
        material=arm_color,
        name="arm_assembly",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage_assembly"),
        origin=Origin(),
        material=carriage_color,
        name="carriage_assembly",
    )

    model.articulation(
        "column_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] + COLUMN_HEIGHT + THRUST_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=-ARM_SWING_LIMIT,
            upper=ARM_SWING_LIMIT,
        ),
    )

    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_ORIGIN_X, 0.0, ARM_BEAM_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.35,
            lower=CARRIAGE_LOWER,
            upper=CARRIAGE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_column")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")

    base_plate = base.get_visual("base_plate")
    thrust_bearing = base.get_visual("thrust_bearing")
    arm_visual = arm.get_visual("arm_assembly")
    carriage_visual = carriage.get_visual("carriage_assembly")

    arm_joint = object_model.get_articulation("column_to_arm")
    carriage_joint = object_model.get_articulation("arm_to_carriage")

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

    ctx.check(
        "arm_joint_axis_is_vertical",
        tuple(arm_joint.axis) == (0.0, 0.0, 1.0),
        f"axis={arm_joint.axis}",
    )
    ctx.check(
        "carriage_joint_axis_follows_arm",
        tuple(carriage_joint.axis) == (1.0, 0.0, 0.0),
        f"axis={carriage_joint.axis}",
    )

    arm_limits = arm_joint.motion_limits
    carriage_limits = carriage_joint.motion_limits
    ctx.check(
        "arm_swing_limits_realistic",
        arm_limits is not None
        and arm_limits.lower is not None
        and arm_limits.upper is not None
        and isclose(arm_limits.lower, -ARM_SWING_LIMIT, abs_tol=1e-9)
        and isclose(arm_limits.upper, ARM_SWING_LIMIT, abs_tol=1e-9),
        f"limits={arm_limits}",
    )
    ctx.check(
        "carriage_travel_limits_realistic",
        carriage_limits is not None
        and carriage_limits.lower is not None
        and carriage_limits.upper is not None
        and isclose(carriage_limits.lower, CARRIAGE_LOWER, abs_tol=1e-9)
        and isclose(carriage_limits.upper, CARRIAGE_UPPER, abs_tol=1e-9),
        f"limits={carriage_limits}",
    )

    base_aabb = ctx.part_world_aabb(base)
    arm_aabb = ctx.part_world_aabb(arm)
    carriage_aabb = ctx.part_world_aabb(carriage)
    if base_aabb is not None:
        base_height = base_aabb[1][2] - base_aabb[0][2]
        ctx.check(
            "column_height_realistic",
            0.68 <= base_height <= 0.72,
            f"height={base_height:.3f}",
        )
    if arm_aabb is not None:
        arm_span = arm_aabb[1][0] - arm_aabb[0][0]
        ctx.check(
            "arm_span_realistic",
            0.88 <= arm_span <= 0.92,
            f"span={arm_span:.3f}",
        )
    if carriage_aabb is not None:
        carriage_height = carriage_aabb[1][2] - carriage_aabb[0][2]
        ctx.check(
            "carriage_head_height_realistic",
            0.18 <= carriage_height <= 0.21,
            f"height={carriage_height:.3f}",
        )

    with ctx.pose({arm_joint: 0.0, carriage_joint: 0.0}):
        ctx.expect_contact(
            arm,
            base,
            elem_a=arm_visual,
            elem_b=thrust_bearing,
            name="arm_contacts_thrust_bearing_at_rest",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem=arm_visual,
            negative_elem=thrust_bearing,
            max_gap=0.0005,
            max_penetration=0.0,
            name="arm_seated_on_column_at_rest",
        )
        ctx.expect_overlap(
            arm,
            base,
            axes="xy",
            elem_a=arm_visual,
            elem_b=thrust_bearing,
            min_overlap=0.16,
            name="arm_hub_centered_over_column",
        )
        ctx.expect_contact(
            carriage,
            arm,
            elem_a=carriage_visual,
            elem_b=arm_visual,
            name="carriage_contacts_arm_at_rest",
        )
        ctx.expect_gap(
            carriage,
            arm,
            axis="z",
            positive_elem=carriage_visual,
            negative_elem=arm_visual,
            max_gap=0.0005,
            max_penetration=0.0,
            name="carriage_seated_on_arm_at_rest",
        )
        ctx.expect_overlap(
            carriage,
            arm,
            axes="xy",
            elem_a=carriage_visual,
            elem_b=arm_visual,
            min_overlap=0.09,
            name="carriage_overlaps_arm_rail_at_rest",
        )
        ctx.expect_gap(
            arm,
            base,
            axis="z",
            positive_elem=arm_visual,
            negative_elem=base_plate,
            min_gap=0.63,
            name="arm_clear_of_base_plate",
        )

    if arm_limits is not None and arm_limits.lower is not None and arm_limits.upper is not None:
        with ctx.pose({arm_joint: arm_limits.lower, carriage_joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="arm_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="arm_lower_no_floating")
            ctx.expect_contact(
                arm,
                base,
                elem_a=arm_visual,
                elem_b=thrust_bearing,
                name="arm_lower_contact_maintained",
            )
        with ctx.pose({arm_joint: arm_limits.upper, carriage_joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="arm_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="arm_upper_no_floating")
            ctx.expect_contact(
                arm,
                base,
                elem_a=arm_visual,
                elem_b=thrust_bearing,
                name="arm_upper_contact_maintained",
            )

    if carriage_limits is not None and carriage_limits.lower is not None and carriage_limits.upper is not None:
        with ctx.pose({arm_joint: 1.1, carriage_joint: carriage_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_lower_no_floating")
            ctx.expect_contact(
                carriage,
                arm,
                elem_a=carriage_visual,
                elem_b=arm_visual,
                name="carriage_lower_contact_maintained",
            )
        with ctx.pose({arm_joint: -1.1, carriage_joint: carriage_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_upper_no_floating")
            ctx.expect_contact(
                carriage,
                arm,
                elem_a=carriage_visual,
                elem_b=arm_visual,
                name="carriage_upper_contact_maintained",
            )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="articulation_pose_sweep_no_overlaps",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
