from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 0.44
RAIL_WIDTH = 0.11
RAIL_HEIGHT = 0.02

PEDESTAL_CENTER_X = 0.14
PEDESTAL_FOOT_LENGTH = 0.08
PEDESTAL_FOOT_WIDTH = 0.095
PEDESTAL_FOOT_HEIGHT = 0.018
PEDESTAL_CHEEK_LENGTH = 0.052
PEDESTAL_CHEEK_THICKNESS = 0.012
PEDESTAL_CHEEK_HEIGHT = 0.07
PEDESTAL_CHEEK_OFFSET_Y = 0.031
PEDESTAL_HOUSING_LENGTH = PEDESTAL_FOOT_LENGTH
PEDESTAL_HOUSING_RADIUS = 0.033
PEDESTAL_RAIL_EMBED = 0.001

SHAFT_AXIS_Z = 0.105
SHAFT_RADIUS = 0.016
BEARING_BORE_RADIUS = 0.018
SHAFT_LENGTH = 0.42
COLLAR_RADIUS = 0.023
COLLAR_THICKNESS = 0.006
HUB_RADIUS = 0.042
HUB_LENGTH = 0.055
FLANGE_RADIUS = 0.056
FLANGE_THICKNESS = 0.012
PEDestal_OUTBOARD_FACE_X = PEDESTAL_CENTER_X + PEDESTAL_FOOT_LENGTH / 2.0
SHAFT_CYLINDER_RPY = (0.0, math.pi / 2.0, 0.0)


def _build_base_rail() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)
        .edges("|Z")
        .fillet(0.008)
    )


def _build_pedestal(center_x: float) -> cq.Workplane:
    foot_center_z = RAIL_HEIGHT / 2.0 + PEDESTAL_FOOT_HEIGHT / 2.0 - PEDESTAL_RAIL_EMBED / 2.0
    cheek_bottom_z = foot_center_z + PEDESTAL_FOOT_HEIGHT / 2.0 - 0.004
    cheek_center_z = cheek_bottom_z + PEDESTAL_CHEEK_HEIGHT / 2.0

    foot = (
        cq.Workplane("XY")
        .box(PEDESTAL_FOOT_LENGTH, PEDESTAL_FOOT_WIDTH, PEDESTAL_FOOT_HEIGHT)
        .translate((center_x, 0.0, foot_center_z))
    )

    cheek_left = (
        cq.Workplane("XY")
        .box(PEDESTAL_CHEEK_LENGTH, PEDESTAL_CHEEK_THICKNESS, PEDESTAL_CHEEK_HEIGHT)
        .translate((center_x, PEDESTAL_CHEEK_OFFSET_Y, cheek_center_z))
    )
    cheek_right = (
        cq.Workplane("XY")
        .box(PEDESTAL_CHEEK_LENGTH, PEDESTAL_CHEEK_THICKNESS, PEDESTAL_CHEEK_HEIGHT)
        .translate((center_x, -PEDESTAL_CHEEK_OFFSET_Y, cheek_center_z))
    )

    housing = (
        cq.Workplane("YZ")
        .circle(PEDESTAL_HOUSING_RADIUS)
        .extrude(PEDESTAL_HOUSING_LENGTH)
        .translate((center_x - PEDESTAL_HOUSING_LENGTH / 2.0, 0.0, SHAFT_AXIS_Z))
    )

    bore = (
        cq.Workplane("YZ")
        .circle(BEARING_BORE_RADIUS)
        .extrude(PEDESTAL_HOUSING_LENGTH + 0.01)
        .translate((center_x - (PEDESTAL_HOUSING_LENGTH + 0.01) / 2.0, 0.0, SHAFT_AXIS_Z))
    )

    return foot.union(cheek_left).union(cheek_right).union(housing).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_rotary_shaft_module")

    model.material("frame_gray", rgba=(0.28, 0.31, 0.34, 1.0))
    model.material("shaft_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("hub_orange", rgba=(0.84, 0.42, 0.13, 1.0))

    support_frame = model.part(
        "support_frame",
        inertial=None,
    )
    support_frame.visual(
        mesh_from_cadquery(_build_base_rail(), "base_rail"),
        material="frame_gray",
        name="base_rail",
    )
    support_frame.visual(
        mesh_from_cadquery(_build_pedestal(-PEDESTAL_CENTER_X), "left_pedestal"),
        material="frame_gray",
        name="left_pedestal",
    )
    support_frame.visual(
        mesh_from_cadquery(_build_pedestal(PEDESTAL_CENTER_X), "right_pedestal"),
        material="frame_gray",
        name="right_pedestal",
    )
    support_frame.inertial = None

    shaft = model.part(
        "shaft",
        inertial=None,
    )
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        material="shaft_steel",
        origin=Origin(rpy=SHAFT_CYLINDER_RPY),
        name="shaft_core",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_THICKNESS),
        material="shaft_steel",
        origin=Origin(
            xyz=(-(PEDestal_OUTBOARD_FACE_X + COLLAR_THICKNESS / 2.0), 0.0, 0.0),
            rpy=SHAFT_CYLINDER_RPY,
        ),
        name="left_retaining_collar",
    )
    shaft.visual(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_THICKNESS),
        material="shaft_steel",
        origin=Origin(
            xyz=((PEDestal_OUTBOARD_FACE_X + COLLAR_THICKNESS / 2.0), 0.0, 0.0),
            rpy=SHAFT_CYLINDER_RPY,
        ),
        name="right_retaining_collar",
    )
    shaft.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_LENGTH),
        material="hub_orange",
        origin=Origin(rpy=SHAFT_CYLINDER_RPY),
        name="center_hub",
    )
    shaft.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_THICKNESS),
        material="shaft_steel",
        origin=Origin(
            xyz=(SHAFT_LENGTH / 2.0 - FLANGE_THICKNESS / 2.0, 0.0, 0.0),
            rpy=SHAFT_CYLINDER_RPY,
        ),
        name="end_flange",
    )

    model.articulation(
        "support_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=support_frame,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=15.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    shaft = object_model.get_part("shaft")
    shaft_joint = object_model.get_articulation("support_to_shaft")
    rail = support_frame.get_visual("base_rail")
    left_pedestal = support_frame.get_visual("left_pedestal")
    right_pedestal = support_frame.get_visual("right_pedestal")
    shaft_core = shaft.get_visual("shaft_core")
    left_collar = shaft.get_visual("left_retaining_collar")
    right_collar = shaft.get_visual("right_retaining_collar")
    hub = shaft.get_visual("center_hub")
    flange = shaft.get_visual("end_flange")

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
        "parts_present",
        support_frame is not None and shaft is not None,
        "Support frame or shaft part missing.",
    )
    ctx.check(
        "shaft_joint_is_continuous_about_x",
        shaft_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(float(v) for v in shaft_joint.axis) == (1.0, 0.0, 0.0)
        and shaft_joint.motion_limits is not None
        and shaft_joint.motion_limits.lower is None
        and shaft_joint.motion_limits.upper is None,
        f"Joint fields were type={shaft_joint.articulation_type}, axis={shaft_joint.axis}, limits={shaft_joint.motion_limits}.",
    )
    ctx.expect_origin_gap(
        shaft,
        support_frame,
        axis="z",
        min_gap=0.095,
        max_gap=0.115,
        name="shaft_axis_height_above_frame_origin",
    )
    ctx.expect_overlap(
        shaft,
        support_frame,
        axes="x",
        min_overlap=0.38,
        name="shaft_span_overlaps_support_span",
    )

    left_aabb = ctx.part_element_world_aabb(support_frame, elem=left_pedestal)
    right_aabb = ctx.part_element_world_aabb(support_frame, elem=right_pedestal)
    rail_aabb = ctx.part_element_world_aabb(support_frame, elem=rail)
    shaft_core_aabb = ctx.part_element_world_aabb(shaft, elem=shaft_core)
    hub_aabb = ctx.part_element_world_aabb(shaft, elem=hub)
    flange_aabb = ctx.part_element_world_aabb(shaft, elem=flange)

    def _center_x(aabb):
        return (aabb[0][0] + aabb[1][0]) / 2.0

    ctx.check(
        "hub_centered_between_pedestals",
        left_aabb is not None
        and right_aabb is not None
        and hub_aabb is not None
        and _center_x(left_aabb) < _center_x(hub_aabb) < _center_x(right_aabb)
        and abs(_center_x(hub_aabb)) <= 0.005,
        "Center hub is not visually centered between the two bearing pedestals.",
    )
    ctx.check(
        "flange_outboard_of_right_pedestal",
        right_aabb is not None
        and flange_aabb is not None
        and flange_aabb[0][0] >= right_aabb[1][0] + 0.01,
        "End flange should sit clearly outboard of the right pedestal.",
    )
    ctx.check(
        "shaft_runs_through_both_supports",
        left_aabb is not None
        and right_aabb is not None
        and shaft_core_aabb is not None
        and shaft_core_aabb[0][0] < left_aabb[0][0]
        and shaft_core_aabb[1][0] > right_aabb[1][0],
        "The straight shaft should pass fully through both supported bearing locations.",
    )
    ctx.check(
        "pedestals_mounted_on_rail",
        rail_aabb is not None
        and left_aabb is not None
        and right_aabb is not None
        and left_aabb[0][2] <= rail_aabb[1][2] + 0.001
        and right_aabb[0][2] <= rail_aabb[1][2] + 0.001,
        "Pedestals should sit directly on and connect into the shared base rail.",
    )
    ctx.expect_contact(
        shaft,
        support_frame,
        elem_a=left_collar,
        elem_b=left_pedestal,
        name="left_collar_contacts_left_pedestal",
    )
    ctx.expect_contact(
        shaft,
        support_frame,
        elem_a=right_collar,
        elem_b=right_pedestal,
        name="right_collar_contacts_right_pedestal",
    )

    with ctx.pose({shaft_joint: 1.3}):
        spun_pos = ctx.part_world_position(shaft)
    rest_pos = ctx.part_world_position(shaft)
    ctx.check(
        "continuous_spin_keeps_shaft_axis_fixed",
        spun_pos is not None
        and rest_pos is not None
        and max(abs(spun_pos[i] - rest_pos[i]) for i in range(3)) <= 1e-6,
        "Spinning the continuous joint should rotate about the supported shaft axis without translating the shaft part.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
