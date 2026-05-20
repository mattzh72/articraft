from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot, pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.18
BASE_HEIGHT = 0.03
COLUMN_RADIUS = 0.08
COLUMN_HEIGHT = 0.12
COLLAR_RADIUS = 0.11
COLLAR_HEIGHT = 0.03
PEDESTAL_HEIGHT = BASE_HEIGHT + COLUMN_HEIGHT + COLLAR_HEIGHT

TURNTABLE_RADIUS = 0.105
TURNTABLE_HEIGHT = 0.032
BEAM_LENGTH = 0.29
BEAM_WIDTH = 0.146
BEAM_HEIGHT = 0.09
BEAM_CENTER_X = 0.225
BEAM_CENTER_Z = 0.068

SLIDER_LENGTH = 0.32
SLIDER_WIDTH = 0.088
SLIDER_HEIGHT = 0.058
SLIDER_JOINT_X = 0.08
SLIDER_JOINT_Z = 0.07
MAX_EXTENSION = 0.16

SPINDLE_MOUNT_X = SLIDER_LENGTH
SPINDLE_LENGTH = 0.113


def _make_pedestal_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    column = (
        cq.Workplane("XY")
        .circle(COLUMN_RADIUS)
        .extrude(COLUMN_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    collar = (
        cq.Workplane("XY")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT + COLUMN_HEIGHT))
    )
    return base.union(column).union(collar)


def _make_yaw_carriage_shape() -> cq.Workplane:
    turntable = cq.Workplane("XY").circle(TURNTABLE_RADIUS).extrude(TURNTABLE_HEIGHT)

    hub = (
        cq.Workplane("XY")
        .box(0.085, 0.11, 0.04)
        .translate((0.04, 0.0, 0.052))
    )

    left_wall = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH, 0.014, BEAM_HEIGHT)
        .translate((BEAM_CENTER_X, 0.066, BEAM_CENTER_Z))
    )
    right_wall = (
        cq.Workplane("XY")
        .box(BEAM_LENGTH, 0.014, BEAM_HEIGHT)
        .translate((BEAM_CENTER_X, -0.066, BEAM_CENTER_Z))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.07, 0.118, 0.012)
        .translate((0.085, 0.0, 0.112))
    )
    left_lip = (
        cq.Workplane("XY")
        .box(0.22, 0.018, 0.008)
        .translate((0.23, 0.041, 0.11))
    )
    right_lip = (
        cq.Workplane("XY")
        .box(0.22, 0.018, 0.008)
        .translate((0.23, -0.041, 0.11))
    )
    nose_ring = (
        cq.Workplane("YZ")
        .circle(0.028)
        .extrude(0.012)
        .translate((0.37, 0.0, 0.068))
    )

    return (
        turntable.union(hub)
        .union(left_wall)
        .union(right_wall)
        .union(rear_bridge)
        .union(left_lip)
        .union(right_lip)
        .union(nose_ring)
    )


def _make_slider_stage_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(SLIDER_LENGTH, SLIDER_WIDTH, SLIDER_HEIGHT)
        .translate((SLIDER_LENGTH / 2.0, 0.0, 0.0))
    )
    top_relief = (
        cq.Workplane("XY")
        .box(0.16, 0.036, 0.012)
        .translate((0.13, 0.0, 0.023))
    )
    front_flange = (
        cq.Workplane("YZ")
        .circle(0.034)
        .extrude(0.012)
        .translate((SLIDER_LENGTH - 0.012, 0.0, 0.0))
    )
    top_cap = (
        cq.Workplane("XY")
        .box(0.16, 0.05, 0.010)
        .translate((0.17, 0.0, 0.024))
    )
    return body.cut(top_relief).union(front_flange).union(top_cap)


def _make_spindle_body_shape() -> cq.Workplane:
    rear_flange = cq.Workplane("YZ").circle(0.042).extrude(0.018)
    barrel = (
        cq.Workplane("YZ")
        .circle(0.033)
        .extrude(0.05)
        .translate((0.018, 0.0, 0.0))
    )
    nose = (
        cq.Workplane("YZ")
        .circle(0.026)
        .workplane(offset=0.027)
        .circle(0.010)
        .loft()
        .translate((0.068, 0.0, 0.0))
    )
    collet = (
        cq.Workplane("YZ")
        .circle(0.008)
        .extrude(0.018)
        .translate((0.095, 0.0, 0.0))
    )
    return rear_flange.union(barrel).union(nose).union(collet)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yaw_telescoping_spindle_arm")

    dark_base = model.material("dark_base", rgba=(0.16, 0.17, 0.19, 1.0))
    arm_graphite = model.material("arm_graphite", rgba=(0.33, 0.36, 0.40, 1.0))
    slider_aluminum = model.material("slider_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    accent_orange = model.material("accent_orange", rgba=(0.80, 0.47, 0.18, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_make_pedestal_shape(), "pedestal_shell"),
        material=dark_base,
        name="pedestal_shell",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=PEDESTAL_HEIGHT),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT / 2.0)),
    )

    yaw_carriage = model.part("yaw_carriage")
    yaw_carriage.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, TURNTABLE_HEIGHT / 2.0)),
        material=arm_graphite,
        name="turntable_disc",
    )
    yaw_carriage.visual(
        Box((0.085, 0.12, 0.04)),
        origin=Origin(xyz=(0.04, 0.0, 0.052)),
        material=arm_graphite,
        name="rear_hub",
    )
    yaw_carriage.visual(
        Box((BEAM_LENGTH, 0.014, BEAM_HEIGHT)),
        origin=Origin(xyz=(BEAM_CENTER_X, 0.067, BEAM_CENTER_Z)),
        material=arm_graphite,
        name="left_beam_wall",
    )
    yaw_carriage.visual(
        Box((BEAM_LENGTH, 0.014, BEAM_HEIGHT)),
        origin=Origin(xyz=(BEAM_CENTER_X, -0.067, BEAM_CENTER_Z)),
        material=arm_graphite,
        name="right_beam_wall",
    )
    yaw_carriage.visual(
        Box((0.07, 0.134, 0.012)),
        origin=Origin(xyz=(0.085, 0.0, 0.112)),
        material=arm_graphite,
        name="rear_bridge",
    )
    yaw_carriage.visual(
        Box((0.24, 0.146, 0.010)),
        origin=Origin(xyz=(0.24, 0.0, 0.110)),
        material=arm_graphite,
        name="beam_roof",
    )
    yaw_carriage.inertial = Inertial.from_geometry(
        Box((0.50, 0.22, 0.12)),
        mass=8.5,
        origin=Origin(xyz=(0.20, 0.0, 0.06)),
    )

    slider_stage = model.part("slider_stage")
    slider_stage.visual(
        Box((SLIDER_LENGTH, 0.08, SLIDER_HEIGHT)),
        origin=Origin(xyz=(SLIDER_LENGTH / 2.0, 0.0, 0.0)),
        material=slider_aluminum,
        name="slider_body",
    )
    slider_stage.visual(
        Box((0.22, 0.050, 0.010)),
        origin=Origin(xyz=(0.18, 0.0, 0.024)),
        material=slider_aluminum,
        name="top_cover",
    )
    slider_stage.visual(
        Box((0.24, 0.009, 0.036)),
        origin=Origin(xyz=(0.17, 0.0435, 0.0)),
        material=accent_orange,
        name="left_guide",
    )
    slider_stage.visual(
        Box((0.24, 0.009, 0.036)),
        origin=Origin(xyz=(0.17, -0.0435, 0.0)),
        material=accent_orange,
        name="right_guide",
    )
    slider_stage.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(SLIDER_LENGTH - 0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=slider_aluminum,
        name="front_flange",
    )
    slider_stage.inertial = Inertial.from_geometry(
        Box((SLIDER_LENGTH, 0.12, 0.08)),
        mass=3.0,
        origin=Origin(xyz=(SLIDER_LENGTH / 2.0, 0.0, 0.0)),
    )

    spindle_head = model.part("spindle_head")
    spindle_head.visual(
        mesh_from_cadquery(_make_spindle_body_shape(), "spindle_body"),
        material=spindle_steel,
        name="spindle_body",
    )
    spindle_head.visual(
        Box((0.016, 0.014, 0.016)),
        origin=Origin(xyz=(0.086, 0.0, 0.022)),
        material=accent_orange,
        name="spindle_index",
    )
    spindle_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=SPINDLE_LENGTH),
        mass=1.2,
        origin=Origin(xyz=(SPINDLE_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_carriage,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "yaw_to_slider",
        ArticulationType.PRISMATIC,
        parent=yaw_carriage,
        child=slider_stage,
        origin=Origin(xyz=(SLIDER_JOINT_X, 0.0, SLIDER_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.35,
            lower=0.0,
            upper=MAX_EXTENSION,
        ),
    )
    model.articulation(
        "slider_to_spindle",
        ArticulationType.REVOLUTE,
        parent=slider_stage,
        child=spindle_head,
        origin=Origin(xyz=(SPINDLE_MOUNT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=10.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_carriage = object_model.get_part("yaw_carriage")
    slider_stage = object_model.get_part("slider_stage")
    spindle_head = object_model.get_part("spindle_head")

    yaw_joint = object_model.get_articulation("pedestal_to_yaw")
    extension_joint = object_model.get_articulation("yaw_to_slider")
    spindle_joint = object_model.get_articulation("slider_to_spindle")

    spindle_index = spindle_head.get_visual("spindle_index")

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
        "articulation axes match yaw-slide-spindle stack",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and extension_joint.axis == (1.0, 0.0, 0.0)
        and spindle_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"yaw={yaw_joint.axis}, extension={extension_joint.axis}, "
            f"spindle={spindle_joint.axis}"
        ),
    )

    with ctx.pose({yaw_joint: 0.0, extension_joint: 0.0, spindle_joint: 0.0}):
        ctx.expect_gap(
            yaw_carriage,
            pedestal,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="turntable seats on pedestal bearing collar",
        )
        ctx.expect_overlap(
            yaw_carriage,
            pedestal,
            axes="xy",
            min_overlap=0.18,
            name="turntable footprint stays centered over pedestal",
        )
        ctx.expect_within(
            slider_stage,
            yaw_carriage,
            axes="yz",
            margin=0.0,
            name="slider cross-section stays inside beam envelope",
        )
        ctx.expect_gap(
            spindle_head,
            slider_stage,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            name="spindle head seats against slider nose mount",
        )
        ctx.expect_overlap(
            spindle_head,
            slider_stage,
            axes="yz",
            min_overlap=0.055,
            name="spindle stays coaxial with slider centerline",
        )

    with ctx.pose({yaw_joint: 0.0, extension_joint: 0.0, spindle_joint: 0.0}):
        slider_retracted = ctx.part_world_position(slider_stage)
    with ctx.pose({yaw_joint: 0.0, extension_joint: MAX_EXTENSION, spindle_joint: 0.0}):
        slider_extended = ctx.part_world_position(slider_stage)

    slide_dx = slider_extended[0] - slider_retracted[0]
    slide_dy = slider_extended[1] - slider_retracted[1]
    slide_dz = slider_extended[2] - slider_retracted[2]
    ctx.check(
        "prismatic stage extends straight along beam axis",
        abs(slide_dx - MAX_EXTENSION) < 1e-4
        and abs(slide_dy) < 1e-6
        and abs(slide_dz) < 1e-6,
        details=(
            f"translation delta = ({slide_dx:.6f}, {slide_dy:.6f}, {slide_dz:.6f})"
        ),
    )

    with ctx.pose({yaw_joint: 0.0, extension_joint: 0.08, spindle_joint: 0.0}):
        yaw_pose_a = ctx.part_world_position(slider_stage)
    with ctx.pose({yaw_joint: 1.0, extension_joint: 0.08, spindle_joint: 0.0}):
        yaw_pose_b = ctx.part_world_position(slider_stage)

    radius_a = hypot(yaw_pose_a[0], yaw_pose_a[1])
    radius_b = hypot(yaw_pose_b[0], yaw_pose_b[1])
    ctx.check(
        "yaw stage swings beam about vertical axis",
        abs(radius_a - radius_b) < 1e-4
        and abs(yaw_pose_a[2] - yaw_pose_b[2]) < 1e-6
        and abs(yaw_pose_b[1]) > 0.05,
        details=(
            f"pose_a={yaw_pose_a}, pose_b={yaw_pose_b}, "
            f"radii=({radius_a:.6f}, {radius_b:.6f})"
        ),
    )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    with ctx.pose({yaw_joint: 0.0, extension_joint: 0.12, spindle_joint: 0.0}):
        spindle_index_aabb_0 = ctx.part_element_world_aabb(
            spindle_head,
            elem=spindle_index,
        )
    with ctx.pose({yaw_joint: 0.0, extension_joint: 0.12, spindle_joint: pi / 2.0}):
        spindle_index_aabb_1 = ctx.part_element_world_aabb(
            spindle_head,
            elem=spindle_index,
        )

    spindle_index_center_0 = aabb_center(spindle_index_aabb_0)
    spindle_index_center_1 = aabb_center(spindle_index_aabb_1)
    ctx.check(
        "spindle nose visibly rotates around tool axis",
        abs(spindle_index_center_0[0] - spindle_index_center_1[0]) < 0.002
        and abs(spindle_index_center_0[1] - spindle_index_center_1[1]) > 0.012
        and abs(spindle_index_center_0[2] - spindle_index_center_1[2]) > 0.012,
        details=(
            f"index centers = {spindle_index_center_0} -> {spindle_index_center_1}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
