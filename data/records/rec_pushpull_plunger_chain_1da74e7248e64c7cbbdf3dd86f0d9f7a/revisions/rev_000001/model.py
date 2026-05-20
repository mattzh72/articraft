from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.090
BASE_WIDTH = 0.070
BASE_THICK = 0.012

BACK_PLATE_LENGTH = 0.018
BACK_PLATE_WIDTH = 0.050
BACK_PLATE_HEIGHT = 0.085
BACK_PLATE_X = -0.034

LOWER_SPINE_LENGTH = 0.060
LOWER_SPINE_WIDTH = 0.040
LOWER_SPINE_HEIGHT = 0.018
LOWER_SPINE_X = -0.006
LOWER_SPINE_Z = 0.020

RAIL_LENGTH = 0.110
RAIL_WIDTH = 0.010
RAIL_HEIGHT = 0.024
RAIL_X = 0.048
RAIL_Z = 0.050
RAIL_Y = 0.020

CHEEK_LENGTH = 0.020
CHEEK_WIDTH = 0.012
CHEEK_HEIGHT = 0.044
CHEEK_X = 0.098
CHEEK_Y = 0.016
CHEEK_Z = 0.060

GUSSET_LENGTH = 0.020
GUSSET_WIDTH = 0.012
GUSSET_HEIGHT = 0.028
GUSSET_X = 0.004
GUSSET_Y = 0.020
GUSSET_Z = 0.038

SLEEVE_Z = 0.055
SLEEVE_START_X = -0.025
SLEEVE_LENGTH = 0.070
SLEEVE_FRONT_X = SLEEVE_START_X + SLEEVE_LENGTH
SLEEVE_OUTER_R = 0.013
SLEEVE_INNER_R = 0.0085
SLEEVE_FLANGE_R = 0.015
SLEEVE_FLANGE_LEN = 0.006

PLUNGER_SHAFT_R = 0.0065
PLUNGER_TIP_R = 0.0075
PLUNGER_SHAFT_START = -0.060
PLUNGER_SHAFT_LEN = 0.080
PLUNGER_TIP_LEN = 0.020
PLUNGER_TIP_CENTER_X = PLUNGER_SHAFT_START + PLUNGER_SHAFT_LEN + (PLUNGER_TIP_LEN / 2.0)
PLUNGER_COLLAR_R = 0.011
PLUNGER_COLLAR_LEN = 0.004
PLUNGER_COLLAR_CENTER_X = PLUNGER_COLLAR_LEN / 2.0

PIVOT_X = 0.095
PIVOT_Z = 0.055
LEVER_THICK = 0.014
LEVER_HUB_R = 0.011
LEVER_LENGTH = 0.058
LEVER_ROOT_REAR_X = -0.010
LEVER_ROOT_LOW_Z = -0.013
LEVER_ROOT_HIGH_Z = 0.012
PIVOT_TRUNNION_R = 0.005
PIVOT_TRUNNION_LEN = 0.003
PIVOT_TRUNNION_CENTER_Y = 0.0085

SLIDE_UPPER = 0.028
LEVER_UPPER = 0.55


def _frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICK).translate((0.0, 0.0, BASE_THICK / 2.0))
    back_plate = cq.Workplane("XY").box(BACK_PLATE_LENGTH, BACK_PLATE_WIDTH, BACK_PLATE_HEIGHT).translate(
        (BACK_PLATE_X, 0.0, BACK_PLATE_HEIGHT / 2.0)
    )
    lower_spine = cq.Workplane("XY").box(LOWER_SPINE_LENGTH, LOWER_SPINE_WIDTH, LOWER_SPINE_HEIGHT).translate(
        (LOWER_SPINE_X, 0.0, LOWER_SPINE_Z)
    )
    rail_left = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT).translate((RAIL_X, RAIL_Y, RAIL_Z))
    rail_right = cq.Workplane("XY").box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT).translate((RAIL_X, -RAIL_Y, RAIL_Z))
    cheek_left = cq.Workplane("XY").box(CHEEK_LENGTH, CHEEK_WIDTH, CHEEK_HEIGHT).translate((CHEEK_X, CHEEK_Y, CHEEK_Z))
    cheek_right = cq.Workplane("XY").box(CHEEK_LENGTH, CHEEK_WIDTH, CHEEK_HEIGHT).translate((CHEEK_X, -CHEEK_Y, CHEEK_Z))
    gusset_left = cq.Workplane("XY").box(GUSSET_LENGTH, GUSSET_WIDTH, GUSSET_HEIGHT).translate(
        (GUSSET_X, GUSSET_Y, GUSSET_Z)
    )
    gusset_right = cq.Workplane("XY").box(GUSSET_LENGTH, GUSSET_WIDTH, GUSSET_HEIGHT).translate(
        (GUSSET_X, -GUSSET_Y, GUSSET_Z)
    )
    top_bridge = cq.Workplane("XY").box(0.022, 0.050, 0.012).translate((-0.018, 0.0, 0.078))

    frame = (
        base.union(back_plate)
        .union(lower_spine)
        .union(rail_left)
        .union(rail_right)
        .union(cheek_left)
        .union(cheek_right)
        .union(gusset_left)
        .union(gusset_right)
        .union(top_bridge)
    )
    return frame


def _sleeve_shape() -> cq.Workplane:
    sleeve = (
        cq.Workplane("YZ", origin=(SLEEVE_START_X, 0.0, SLEEVE_Z))
        .circle(SLEEVE_OUTER_R)
        .circle(SLEEVE_INNER_R)
        .extrude(SLEEVE_LENGTH)
    )
    front_flange = (
        cq.Workplane("YZ", origin=(SLEEVE_FRONT_X - SLEEVE_FLANGE_LEN, 0.0, SLEEVE_Z))
        .circle(SLEEVE_FLANGE_R)
        .circle(SLEEVE_OUTER_R)
        .extrude(SLEEVE_FLANGE_LEN)
    )
    rear_flange = (
        cq.Workplane("YZ", origin=(SLEEVE_START_X, 0.0, SLEEVE_Z))
        .circle(0.014)
        .circle(SLEEVE_OUTER_R)
        .extrude(0.005)
    )
    return sleeve.union(front_flange).union(rear_flange)


def _plunger_shaft_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(PLUNGER_SHAFT_START, 0.0, 0.0))
        .circle(PLUNGER_SHAFT_R)
        .extrude(PLUNGER_SHAFT_LEN)
    )


def _lever_shape() -> cq.Workplane:
    profile = (
        cq.Workplane("XZ")
        .moveTo(LEVER_ROOT_REAR_X, LEVER_ROOT_LOW_Z)
        .lineTo(LEVER_ROOT_REAR_X, LEVER_ROOT_HIGH_Z)
        .lineTo(-0.002, 0.016)
        .lineTo(0.020, 0.014)
        .lineTo(0.048, 0.010)
        .lineTo(LEVER_LENGTH, 0.004)
        .lineTo(LEVER_LENGTH, -0.004)
        .lineTo(0.042, -0.006)
        .lineTo(0.014, -0.008)
        .lineTo(-0.002, -0.010)
        .close()
        .extrude(LEVER_THICK)
        .translate((0.0, -LEVER_THICK / 2.0, 0.0))
    )
    hub = (
        cq.Workplane("XZ")
        .circle(LEVER_HUB_R)
        .extrude(LEVER_THICK)
        .translate((0.0, -LEVER_THICK / 2.0, 0.0))
    )
    return profile.union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_plunger_chain")

    model.material("frame_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("sleeve_zinc", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("plunger_black", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("lever_enamel", rgba=(0.56, 0.22, 0.16, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_frame_shape(), "support_frame"),
        material="frame_steel",
        name="frame",
    )
    support_frame.visual(
        mesh_from_cadquery(_sleeve_shape(), "guide_sleeve"),
        material="sleeve_zinc",
        name="guide_sleeve",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((0.170, 0.070, 0.090)),
        mass=1.6,
        origin=Origin(xyz=(0.030, 0.0, 0.045)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        mesh_from_cadquery(_plunger_shaft_shape(), "plunger_shaft"),
        material="plunger_black",
        name="shaft",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_TIP_R, length=PLUNGER_TIP_LEN),
        origin=Origin(xyz=(PLUNGER_TIP_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="plunger_black",
        name="tip",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_COLLAR_R, length=PLUNGER_COLLAR_LEN),
        origin=Origin(xyz=(PLUNGER_COLLAR_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="plunger_black",
        name="guide_collar",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=PLUNGER_TIP_R, length=0.100),
        mass=0.18,
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_lever_shape(), "lever"),
        origin=Origin(xyz=(0.0, 0.014, 0.0)),
        material="lever_enamel",
        name="lever_body",
    )
    lever.visual(
        Cylinder(radius=PIVOT_TRUNNION_R, length=PIVOT_TRUNNION_LEN),
        origin=Origin(xyz=(0.0, PIVOT_TRUNNION_CENTER_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="lever_enamel",
        name="pivot_left",
    )
    lever.visual(
        Cylinder(radius=PIVOT_TRUNNION_R, length=PIVOT_TRUNNION_LEN),
        origin=Origin(xyz=(0.0, -PIVOT_TRUNNION_CENTER_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="lever_enamel",
        name="pivot_right",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.070, LEVER_THICK, 0.035)),
        mass=0.12,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
    )

    model.articulation(
        "sleeve_to_plunger",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=plunger,
        origin=Origin(xyz=(SLEEVE_FRONT_X, 0.0, SLEEVE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_UPPER, effort=120.0, velocity=0.18),
    )
    model.articulation(
        "frame_to_lever",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=lever,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LEVER_UPPER, effort=18.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("lever")
    slide = object_model.get_articulation("sleeve_to_plunger")
    lever_hinge = object_model.get_articulation("frame_to_lever")

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

    ctx.expect_contact(
        plunger,
        lever,
        elem_a="tip",
        elem_b="lever_body",
        contact_tol=0.0008,
        name="plunger tip seats against the lever root",
    )
    ctx.expect_within(
        plunger,
        support_frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="guide_sleeve",
        margin=0.001,
        name="plunger shaft stays centered inside the guide sleeve",
    )

    rest_plunger_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_within(
            plunger,
            support_frame,
            axes="yz",
            inner_elem="shaft",
            outer_elem="guide_sleeve",
            margin=0.001,
            name="extended plunger shaft remains guided by the sleeve",
        )
        ctx.expect_overlap(
            plunger,
            support_frame,
            axes="x",
            elem_a="shaft",
            elem_b="guide_sleeve",
            min_overlap=0.020,
            name="plunger retains insertion at full extension",
        )
        extended_plunger_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger extends forward along the sleeve axis",
        rest_plunger_pos is not None
        and extended_plunger_pos is not None
        and extended_plunger_pos[0] > rest_plunger_pos[0] + 0.020,
        details=f"rest={rest_plunger_pos}, extended={extended_plunger_pos}",
    )

    rest_lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")
    with ctx.pose({lever_hinge: LEVER_UPPER}):
        raised_lever_aabb = ctx.part_element_world_aabb(lever, elem="lever_body")

    ctx.check(
        "lever nose swings upward from the forked frame",
        rest_lever_aabb is not None
        and raised_lever_aabb is not None
        and raised_lever_aabb[1][2] > rest_lever_aabb[1][2] + 0.020,
        details=f"rest={rest_lever_aabb}, raised={raised_lever_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
