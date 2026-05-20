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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.14
BASE_THICK = 0.02
BASE_PEDESTAL_RADIUS = 0.058
BASE_PEDESTAL_HEIGHT = 0.014
BASE_AXIS_Z = BASE_THICK + BASE_PEDESTAL_HEIGHT
BASE_BOLT_CIRCLE = 0.102
BASE_MOUNT_HOLE_RADIUS = 0.006

LOWER_FLANGE_RADIUS = 0.066
LOWER_FLANGE_HEIGHT = 0.008
LOWER_HUB_RADIUS = 0.053
LOWER_HUB_HEIGHT = 0.03

ARM_WIDTH = 0.024
UPPER_AXIS_OFFSET = 0.145
UPPER_AXIS_Z = 0.155
TOP_COLLAR_RADIUS = 0.04
TOP_COLLAR_HEIGHT = 0.03
TOP_SEAT_RADIUS = 0.052
TOP_SEAT_HEIGHT = 0.008

UPPER_STAGE_COLLAR_RADIUS = 0.026
UPPER_STAGE_COLLAR_HEIGHT = 0.022
UPPER_STAGE_PLATE_RADIUS = 0.055
UPPER_STAGE_PLATE_HEIGHT = 0.016


def _base_disk_shape():
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICK)
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICK)
        .circle(BASE_PEDESTAL_RADIUS)
        .extrude(BASE_PEDESTAL_HEIGHT)
    )
    holes = (
        cq.Workplane("XY")
        .polarArray(BASE_BOLT_CIRCLE, 0.0, 360.0, 4)
        .circle(BASE_MOUNT_HOLE_RADIUS)
        .extrude(BASE_THICK + 0.002)
    )
    return base.union(pedestal).cut(holes)


def _support_arm_shape():
    lower_flange = cq.Workplane("XY").circle(LOWER_FLANGE_RADIUS).extrude(LOWER_FLANGE_HEIGHT)
    lower_hub = cq.Workplane("XY").circle(LOWER_HUB_RADIUS).extrude(LOWER_HUB_HEIGHT)

    outer_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, 0.02),
                (0.055, 0.03),
                (0.095, 0.07),
                (0.128, 0.118),
                (0.145, 0.148),
                (0.112, 0.148),
                (0.09, 0.114),
                (0.06, 0.071),
                (0.028, 0.036),
            ]
        )
        .close()
        .extrude(ARM_WIDTH / 2.0, both=True)
    )

    inner_window = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.05, 0.048),
                (0.076, 0.06),
                (0.102, 0.098),
                (0.112, 0.121),
                (0.1, 0.129),
                (0.083, 0.103),
                (0.061, 0.07),
            ]
        )
        .close()
        .extrude(ARM_WIDTH, both=True)
    )
    web = outer_web.cut(inner_window)

    top_collar = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_AXIS_Z - TOP_COLLAR_HEIGHT)
        .center(UPPER_AXIS_OFFSET, 0.0)
        .circle(TOP_COLLAR_RADIUS)
        .extrude(TOP_COLLAR_HEIGHT)
    )
    top_seat = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_AXIS_Z - TOP_SEAT_HEIGHT)
        .center(UPPER_AXIS_OFFSET, 0.0)
        .circle(TOP_SEAT_RADIUS)
        .extrude(TOP_SEAT_HEIGHT)
    )

    lower_bore = cq.Workplane("XY").circle(0.018).extrude(LOWER_HUB_HEIGHT + 0.002)
    upper_bore = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_AXIS_Z - TOP_COLLAR_HEIGHT - 0.001)
        .center(UPPER_AXIS_OFFSET, 0.0)
        .circle(0.016)
        .extrude(TOP_COLLAR_HEIGHT + 0.003)
    )

    return (
        lower_flange.union(lower_hub)
        .union(web)
        .union(top_collar)
        .union(top_seat)
        .cut(lower_bore)
        .cut(upper_bore)
    )


def _upper_stage_shape():
    stage = cq.Workplane("XY").circle(UPPER_STAGE_COLLAR_RADIUS).extrude(UPPER_STAGE_COLLAR_HEIGHT)
    stage = (
        stage.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(UPPER_STAGE_PLATE_RADIUS)
        .extrude(UPPER_STAGE_PLATE_HEIGHT)
    )

    center_bore = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(UPPER_STAGE_COLLAR_HEIGHT + UPPER_STAGE_PLATE_HEIGHT + 0.002)
    )
    pockets = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_COLLAR_HEIGHT - 0.001)
        .polarArray(0.029, 0.0, 360.0, 3)
        .circle(0.007)
        .extrude(UPPER_STAGE_PLATE_HEIGHT + 0.003)
    )
    return stage.cut(center_bore).cut(pockets)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_offset_axis_rotary_assembly")

    model.material("base_dark", rgba=(0.16, 0.18, 0.21, 1.0))
    model.material("arm_blue", rgba=(0.29, 0.44, 0.64, 1.0))
    model.material("stage_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("fixture_dark", rgba=(0.24, 0.26, 0.28, 1.0))

    base_disk = model.part("base_disk")
    base_disk.visual(
        mesh_from_cadquery(_base_disk_shape(), "base_disk"),
        material="base_dark",
        name="base_body",
    )
    base_disk.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_AXIS_Z),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, BASE_AXIS_Z / 2.0)),
    )

    support_arm = model.part("support_arm")
    support_arm.visual(
        mesh_from_cadquery(_support_arm_shape(), "support_arm"),
        material="arm_blue",
        name="arm_body",
    )
    support_arm.inertial = Inertial.from_geometry(
        Box((UPPER_AXIS_OFFSET + 0.05, ARM_WIDTH, UPPER_AXIS_Z)),
        mass=1.35,
        origin=Origin(xyz=(0.08, 0.0, UPPER_AXIS_Z / 2.0)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_shape(), "upper_stage"),
        material="stage_silver",
        name="stage_body",
    )
    upper_stage.visual(
        Box((0.05, 0.034, 0.01)),
        origin=Origin(xyz=(0.046, 0.0, 0.033)),
        material="fixture_dark",
        name="tool_tab",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Box((0.11, 0.08, 0.04)),
        mass=0.42,
        origin=Origin(xyz=(0.025, 0.0, 0.02)),
    )

    model.articulation(
        "base_to_support",
        ArticulationType.REVOLUTE,
        parent=base_disk,
        child=support_arm,
        origin=Origin(xyz=(0.0, 0.0, BASE_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.9, upper=1.9, effort=22.0, velocity=1.8),
    )
    model.articulation(
        "support_to_upper",
        ArticulationType.REVOLUTE,
        parent=support_arm,
        child=upper_stage,
        origin=Origin(xyz=(UPPER_AXIS_OFFSET, 0.0, UPPER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.4, upper=2.4, effort=10.0, velocity=3.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_disk = object_model.get_part("base_disk")
    support_arm = object_model.get_part("support_arm")
    upper_stage = object_model.get_part("upper_stage")
    base_to_support = object_model.get_articulation("base_to_support")
    support_to_upper = object_model.get_articulation("support_to_upper")

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
        "both_joint_axes_vertical",
        all(
            isclose(val, ref, abs_tol=1e-9)
            for joint_vals, refs in (
                (base_to_support.axis, (0.0, 0.0, 1.0)),
                (support_to_upper.axis, (0.0, 0.0, 1.0)),
            )
            for val, ref in zip(joint_vals, refs)
        ),
        details=f"axes={base_to_support.axis}, {support_to_upper.axis}",
    )
    ctx.check(
        "upper_axis_is_laterally_offset",
        support_to_upper.origin.xyz[0] > 0.12
        and abs(support_to_upper.origin.xyz[1]) < 1e-9
        and support_to_upper.origin.xyz[2] > 0.14,
        details=f"upper joint origin={support_to_upper.origin.xyz}",
    )

    with ctx.pose({base_to_support: 0.0, support_to_upper: 0.0}):
        ctx.expect_contact(
            support_arm,
            base_disk,
            contact_tol=8e-4,
            name="support_arm_seats_on_base_disk",
        )
        ctx.expect_contact(
            upper_stage,
            support_arm,
            contact_tol=8e-4,
            name="upper_stage_seats_on_support_arm",
        )
        ctx.expect_origin_distance(
            upper_stage,
            support_arm,
            axes="xy",
            min_dist=0.135,
            max_dist=0.155,
            name="rotary_axes_have_visible_xy_offset",
        )
        ctx.expect_gap(
            upper_stage,
            base_disk,
            axis="z",
            min_gap=0.12,
            name="upper_stage_clears_grounded_base",
        )

    with ctx.pose({base_to_support: 0.65, support_to_upper: 0.0}):
        upper_pos = ctx.part_world_position(upper_stage)
        ctx.check(
            "base_rotation_sweeps_offset_stage_sideways",
            upper_pos is not None and upper_pos[1] > 0.075,
            details=f"upper_stage world position at q=0.65 is {upper_pos}",
        )

    with ctx.pose({base_to_support: 0.0, support_to_upper: 0.0}):
        rest_tool_center = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="tool_tab"))
        rest_stage_origin = ctx.part_world_position(upper_stage)

    with ctx.pose({base_to_support: 0.0, support_to_upper: 0.9}):
        turned_tool_center = _aabb_center(ctx.part_element_world_aabb(upper_stage, elem="tool_tab"))
        turned_stage_origin = ctx.part_world_position(upper_stage)
        ctx.check(
            "upper_stage_joint_visibly_rotates_tool_tab",
            rest_tool_center is not None
            and rest_stage_origin is not None
            and turned_tool_center is not None
            and turned_stage_origin is not None
            and rest_tool_center[0] > rest_stage_origin[0] + 0.025
            and turned_tool_center[1] > turned_stage_origin[1] + 0.025,
            details=(
                f"rest_tool_center={rest_tool_center}, rest_stage_origin={rest_stage_origin}, "
                f"turned_tool_center={turned_tool_center}, turned_stage_origin={turned_stage_origin}"
            ),
        )

    with ctx.pose({base_to_support: 0.9, support_to_upper: -1.1}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_representative_offset_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
