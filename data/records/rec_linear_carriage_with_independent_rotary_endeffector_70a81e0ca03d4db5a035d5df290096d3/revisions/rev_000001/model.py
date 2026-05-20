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


BASE_L = 0.420
BASE_W = 0.120
BASE_T = 0.018
PEDESTAL_L = 0.386
PEDESTAL_W = 0.072
PEDESTAL_H = 0.012
GUIDE_L = 0.360
GUIDE_W = 0.052
GUIDE_H = 0.022
GUIDE_CENTER_Z = BASE_T + PEDESTAL_H + (GUIDE_H / 2.0)

CARR_L = 0.118
CARR_W = 0.104
CARR_BOTTOM_Z = -0.004
CARR_TOP_Z = 0.072
CARR_H = CARR_TOP_Z - CARR_BOTTOM_Z
CARR_CENTER_Y = 0.0
SIDE_BLOCK_W = 0.014
SIDE_BLOCK_CENTER_Y = 0.024
PAD_W = 0.012
PAD_CENTER_Y = 0.018
FRONT_CHEEK_W = 0.012
FRONT_CHEEK_CENTER_X = 0.044
CARR_FRONT_Y = 0.058
TOOL_AXIS_Y = 0.060
TOOL_AXIS_Z = 0.070
JOURNAL_R = 0.016
JOURNAL_L = 0.036
TOOL_HOUSING_X = 0.060
TOOL_HOUSING_Y = 0.032
TOOL_HOUSING_Z = 0.036
NOSE_FLANGE_R = 0.014
NOSE_BODY_R = 0.010
NOSE_TIP_R = 0.006
NOSE_FLANGE_L = 0.006
NOSE_BODY_L = 0.016
NOSE_TIP_L = 0.010
PIVOT_HUB_R = 0.008
PIVOT_HUB_L = 0.040
UPPER_FLANGE_Z = 0.022
LOWER_FLANGE_Z = -0.022
NECK_Y = 0.015
NECK_L = 0.030
MID_Y = 0.035
MID_L = 0.020
HOUSING_Y = 0.060
HOUSING_L = 0.040
NOSE_MOUNT_Y = 0.075
NOSE_MOUNT_L = 0.014
NOSE_FLANGE_CENTER_Y = 0.082
NOSE_BODY_CENTER_Y = 0.093
NOSE_TIP_CENTER_Y = 0.105
SLIDE_TRAVEL = 0.105
ROTARY_LIMIT = 0.38


def _rail_base_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))
        .translate((0.0, 0.0, 0.0))
    )
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rarray(BASE_L - 0.080, BASE_W - 0.050, 2, 2)
        .cboreHole(0.009, 0.016, 0.005)
    )

    pedestal = (
        cq.Workplane("XY")
        .box(PEDESTAL_L, PEDESTAL_W, PEDESTAL_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )

    side_ribs = (
        cq.Workplane("XY")
        .box(0.070, 0.016, 0.018, centered=(True, True, False))
        .translate((-(BASE_L / 2.0) + 0.052, 0.0, BASE_T))
        .union(
            cq.Workplane("XY")
            .box(0.070, 0.016, 0.018, centered=(True, True, False))
            .translate(((BASE_L / 2.0) - 0.052, 0.0, BASE_T))
        )
    )

    return base.union(pedestal).union(side_ribs)


def _guide_bar_shape() -> cq.Workplane:
    guide = cq.Workplane("XY").box(
        GUIDE_L, GUIDE_W, GUIDE_H, centered=(True, True, False)
    )
    guide = guide.edges("|X").fillet(0.003)
    return guide.translate((0.0, 0.0, BASE_T + PEDESTAL_H))


def _carriage_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(CARR_L, 0.016, 0.052, centered=(True, True, True))
        .translate((0.0, -0.030, 0.036))
        .union(
            cq.Workplane("XY")
            .box(CARR_L, 0.016, 0.052, centered=(True, True, True))
            .translate((0.0, 0.030, 0.036))
        )
        .union(
            cq.Workplane("XY")
            .box(CARR_L * 0.92, 0.014, 0.010, centered=(True, True, True))
            .translate((0.0, -0.016, 0.016))
        )
        .union(
            cq.Workplane("XY")
            .box(CARR_L * 0.92, 0.014, 0.010, centered=(True, True, True))
            .translate((0.0, 0.016, 0.016))
        )
        .union(
            cq.Workplane("XY")
            .box(0.088, 0.048, 0.056, centered=(True, True, True))
            .translate((0.0, -0.004, 0.070))
        )
        .union(
            cq.Workplane("XY")
            .box(CARR_L, 0.058, 0.004, centered=(True, True, True))
            .translate((0.0, 0.010, 0.094))
        )
        .union(
            cq.Workplane("XY")
            .box(CARR_L, 0.058, 0.004, centered=(True, True, True))
            .translate((0.0, 0.010, 0.046))
        )
        .union(
            cq.Workplane("XY")
            .box(FRONT_CHEEK_W, 0.022, 0.050, centered=(True, True, True))
            .translate((-FRONT_CHEEK_CENTER_X, TOOL_AXIS_Y, TOOL_AXIS_Z))
        )
        .union(
            cq.Workplane("XY")
            .box(FRONT_CHEEK_W, 0.022, 0.050, centered=(True, True, True))
            .translate((FRONT_CHEEK_CENTER_X, TOOL_AXIS_Y, TOOL_AXIS_Z))
        )
        .union(
            cq.Workplane("XY")
            .box(0.086, 0.010, 0.006, centered=(True, True, True))
            .translate((0.0, 0.052, 0.094))
        )
        .union(
            cq.Workplane("XY")
            .box(0.086, 0.010, 0.006, centered=(True, True, True))
            .translate((0.0, 0.052, 0.046))
        )
    )
    front_window = (
        cq.Workplane("XY")
        .box(0.092, 0.028, 0.040, centered=(True, True, True))
        .translate((0.0, TOOL_AXIS_Y, TOOL_AXIS_Z))
    )
    pivot_bore = (
        cq.Workplane("XY")
        .circle(0.009)
        .extrude(0.080, both=True)
        .translate((0.0, TOOL_AXIS_Y, TOOL_AXIS_Z))
    )
    return body.cut(front_window).cut(pivot_bore)


def _tool_housing_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(HUB_R)
        .extrude(HUB_L, both=True)
        .union(
            cq.Workplane("XY")
            .circle(0.016)
            .extrude(0.004)
            .translate((0.0, 0.0, UPPER_FLANGE_Z))
        )
        .union(
            cq.Workplane("XY")
            .circle(0.016)
            .extrude(0.004)
            .translate((0.0, 0.0, LOWER_FLANGE_Z))
        )
        .union(
            cq.Workplane("XY")
            .box(0.024, 0.006, 0.034, centered=(True, True, True))
            .translate((0.0, -0.019, 0.0))
        )
        .union(
            cq.Workplane("XY")
            .box(0.018, 0.028, 0.018, centered=(True, True, True))
            .translate((0.0, 0.010, 0.0))
        )
        .union(
            cq.Workplane("XZ")
            .circle(FORWARD_BODY_R)
            .extrude(FORWARD_BODY_L)
            .translate((0.0, FORWARD_BODY_CENTER_Y, 0.0))
        )
        .union(
            cq.Workplane("XY")
            .box(0.026, 0.018, 0.020, centered=(True, True, True))
            .translate((0.0, 0.046, 0.0))
        )
        .union(
            cq.Workplane("XY")
            .box(0.018, 0.012, 0.010, centered=(True, True, True))
            .translate((0.0, 0.038, 0.015))
        )
        .union(
            cq.Workplane("XZ")
            .circle(NOSE_FLANGE_R)
            .extrude(NOSE_FLANGE_L)
            .translate((0.0, NOSE_FLANGE_CENTER_Y, 0.0))
        )
        .union(
            cq.Workplane("XZ")
            .circle(NOSE_BODY_R)
            .extrude(NOSE_BODY_L)
            .translate((0.0, NOSE_BODY_CENTER_Y, 0.0))
        )
        .union(
            cq.Workplane("XZ")
            .circle(NOSE_TIP_R)
            .extrude(NOSE_TIP_L)
            .translate((0.0, NOSE_TIP_CENTER_Y, 0.0))
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_transfer_axis")

    model.material("rail_graphite", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("guide_steel", rgba=(0.70, 0.73, 0.77, 1.0))
    model.material("machined_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("guard_black", rgba=(0.11, 0.12, 0.14, 1.0))
    model.material("cartridge_black", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("tool_steel", rgba=(0.61, 0.63, 0.67, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(_rail_base_shape(), "rail_base"),
        material="rail_graphite",
        name="rail_base",
    )
    rail.visual(
        Box((GUIDE_L, GUIDE_W, GUIDE_H)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        material="guide_steel",
        name="guide_bar",
    )
    rail.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + PEDESTAL_H + GUIDE_H)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + PEDESTAL_H + GUIDE_H) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_body_shape(), "carriage_body"),
        material="machined_aluminum",
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARR_L, CARR_W, CARR_H + 0.008)),
        mass=2.1,
        origin=Origin(
            xyz=(0.0, CARR_CENTER_Y, CARR_BOTTOM_Z + ((CARR_H + 0.008) / 2.0))
        ),
    )

    tool = model.part("tool_cartridge")
    tool.visual(
        Cylinder(radius=PIVOT_HUB_R, length=PIVOT_HUB_L),
        origin=Origin(),
        material="tool_steel",
        name="pivot_hub",
    )
    tool.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, UPPER_FLANGE_Z)),
        material="tool_steel",
        name="upper_flange",
    )
    tool.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, LOWER_FLANGE_Z)),
        material="tool_steel",
        name="lower_flange",
    )
    tool.visual(
        Box((0.020, NECK_L, 0.018)),
        origin=Origin(xyz=(0.0, NECK_Y, 0.0)),
        material="cartridge_black",
        name="neck_block",
    )
    tool.visual(
        Box((0.024, MID_L, 0.022)),
        origin=Origin(xyz=(0.0, MID_Y, 0.0)),
        material="cartridge_black",
        name="mid_block",
    )
    tool.visual(
        Box((TOOL_HOUSING_X, HOUSING_L, TOOL_HOUSING_Z)),
        origin=Origin(xyz=(0.0, HOUSING_Y, 0.0)),
        material="cartridge_black",
        name="tool_body",
    )
    tool.visual(
        Box((0.024, NOSE_MOUNT_L, 0.020)),
        origin=Origin(xyz=(0.0, NOSE_MOUNT_Y, 0.0)),
        material="cartridge_black",
        name="nose_mount",
    )
    tool.visual(
        Cylinder(radius=NOSE_FLANGE_R, length=NOSE_FLANGE_L),
        origin=Origin(
            xyz=(0.0, NOSE_FLANGE_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
        name="nose_flange",
    )
    tool.visual(
        Cylinder(radius=NOSE_BODY_R, length=NOSE_BODY_L),
        origin=Origin(
            xyz=(0.0, NOSE_BODY_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
        name="nose_body",
    )
    tool.visual(
        Cylinder(radius=NOSE_TIP_R, length=NOSE_TIP_L),
        origin=Origin(
            xyz=(0.0, NOSE_TIP_CENTER_Y, 0.0),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material="tool_steel",
        name="nose_tip",
    )
    tool.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.050)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=1200.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "carriage_to_tool",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tool,
        origin=Origin(xyz=(0.0, TOOL_AXIS_Y, TOOL_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-ROTARY_LIMIT,
            upper=ROTARY_LIMIT,
            effort=35.0,
            velocity=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    tool = object_model.get_part("tool_cartridge")
    slide = object_model.get_articulation("rail_to_carriage")
    rotary = object_model.get_articulation("carriage_to_tool")
    carriage_body = carriage.get_visual("carriage_body")
    guide_bar = rail.get_visual("guide_bar")
    upper_flange = tool.get_visual("upper_flange")
    lower_flange = tool.get_visual("lower_flange")
    tool_body = tool.get_visual("tool_body")
    nose_tip = tool.get_visual("nose_tip")

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
        "slide_axis_aligned_to_rail",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic axis (1,0,0), got {slide.axis}",
    )
    ctx.check(
        "rotary_axis_vertical",
        tuple(rotary.axis) == (0.0, 0.0, 1.0),
        f"expected rotary axis (0,0,1), got {rotary.axis}",
    )

    with ctx.pose({slide: 0.0, rotary: 0.0}):
        ctx.expect_contact(
            carriage,
            rail,
            elem_a=carriage_body,
            elem_b=guide_bar,
            name="carriage_supported_on_guide_bar",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="z",
            positive_elem=carriage_body,
            negative_elem="rail_base",
            min_gap=0.010,
            name="carriage_body_clears_rail_base",
        )
        ctx.expect_contact(
            tool,
            carriage,
            elem_a=upper_flange,
            elem_b=carriage_body,
            name="upper_rotary_flange_supported_by_carriage",
        )
        ctx.expect_contact(
            tool,
            carriage,
            elem_a=lower_flange,
            elem_b=carriage_body,
            name="lower_rotary_flange_supported_by_carriage",
        )
        ctx.expect_gap(
            tool,
            carriage,
            axis="y",
            positive_elem=nose_tip,
            negative_elem=carriage_body,
            min_gap=0.010,
            max_gap=0.090,
            name="nose_tip_projects_clear_of_carriage_face",
        )

    with ctx.pose({slide: slide.motion_limits.lower, rotary: 0.0}):
        ctx.expect_contact(
            carriage,
            rail,
            elem_a=carriage_body,
            elem_b=guide_bar,
            name="carriage_remains_supported_at_negative_travel_limit",
        )

    with ctx.pose({slide: slide.motion_limits.upper, rotary: 0.0}):
        ctx.expect_contact(
            carriage,
            rail,
            elem_a=carriage_body,
            elem_b=guide_bar,
            name="carriage_remains_supported_at_positive_travel_limit",
        )

    with ctx.pose({rotary: rotary.motion_limits.lower}):
        ctx.expect_contact(
            tool,
            carriage,
            elem_a=upper_flange,
            elem_b=carriage_body,
            name="upper_flange_supported_at_negative_rotation_limit",
        )
        ctx.expect_contact(
            tool,
            carriage,
            elem_a=lower_flange,
            elem_b=carriage_body,
            name="lower_flange_supported_at_negative_rotation_limit",
        )

    with ctx.pose({rotary: rotary.motion_limits.upper}):
        ctx.expect_contact(
            tool,
            carriage,
            elem_a=upper_flange,
            elem_b=carriage_body,
            name="upper_flange_supported_at_positive_rotation_limit",
        )
        ctx.expect_contact(
            tool,
            carriage,
            elem_a=lower_flange,
            elem_b=carriage_body,
            name="lower_flange_supported_at_positive_rotation_limit",
        )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=28,
        name="articulation_paths_clear_through_sampled_motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
