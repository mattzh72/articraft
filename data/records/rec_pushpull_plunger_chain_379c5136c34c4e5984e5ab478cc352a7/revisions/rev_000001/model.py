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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.418
BASE_W = 0.160
BASE_T = 0.018
BASE_SIDE_H = 0.080
BASE_FRONT_H = 0.084
SIDE_WALL_T = 0.014
GUIDE_Z = 0.047
GUIDE_CAVITY_X0 = 0.055
GUIDE_CAVITY_X1 = 0.368
FRONT_BODY_X0 = 0.300

STAGE1_L = 0.220
STAGE1_W = 0.098
STAGE1_H = 0.056
STAGE1_REAR_T = 0.018
STAGE1_FRONT_T = 0.014

STAGE2_L = 0.155
STAGE2_W = 0.072
STAGE2_H = 0.036
STAGE2_REAR_T = 0.014
STAGE2_FRONT_T = 0.012

GUIDE_BAR_L = 0.066
GUIDE_BAR_W = 0.052
GUIDE_BAR_H = 0.022
COLLAR_L = 0.006
COLLAR_R = 0.012
ROD_R = 0.008
ROD_L = 0.244
NOSE_L = 0.022
NOSE_TIP_R = 0.003
PLUNGER_FORWARD = COLLAR_L + ROD_L + NOSE_L

BASE_TO_STAGE1_X = GUIDE_CAVITY_X0
STAGE1_TO_STAGE2_X = STAGE1_REAR_T
STAGE2_TO_PLUNGER_X = STAGE2_REAR_T + GUIDE_BAR_L

STAGE1_TRAVEL = 0.080
STAGE2_TRAVEL = 0.055
STAGE3_TRAVEL = 0.074


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    *,
    name: str | None = None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_x_cylinder(
    part,
    radius: float,
    length: float,
    x_center: float,
    material: str,
    *,
    name: str | None = None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _base_frame_shape():
    body = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(False, True, False))

    side_length = GUIDE_CAVITY_X1 - 0.035
    for y_pos in (-(BASE_W - SIDE_WALL_T) / 2.0, (BASE_W - SIDE_WALL_T) / 2.0):
        body = body.union(
            cq.Workplane("XY", origin=(0.035, y_pos, BASE_T)).box(
                side_length,
                SIDE_WALL_T,
                BASE_SIDE_H - BASE_T,
                centered=(False, True, False),
            )
        )

    body = body.union(
        cq.Workplane("XY", origin=(0.012, 0.0, BASE_T)).box(
            0.030,
            0.130,
            0.056,
            centered=(False, True, False),
        )
    )

    body = body.union(
        cq.Workplane("XY", origin=(FRONT_BODY_X0, 0.0, 0.0)).box(
            BASE_L - FRONT_BODY_X0,
            0.116,
            BASE_FRONT_H,
            centered=(False, True, False),
        )
    )

    for y_pos in (-0.046, 0.046):
        body = body.union(
            cq.Workplane("XY", origin=(0.090, y_pos, BASE_SIDE_H)).box(
                0.170,
                0.018,
                0.006,
                centered=(False, True, False),
            )
        )

    body = body.union(
        cq.Workplane("XY", origin=(0.378, 0.0, GUIDE_Z)).box(
            0.018,
            0.072,
            0.046,
            centered=(False, True, True),
        )
    )

    body = body.cut(
        cq.Workplane("XY", origin=(GUIDE_CAVITY_X0, 0.0, GUIDE_Z)).box(
            GUIDE_CAVITY_X1 - GUIDE_CAVITY_X0,
            STAGE1_W,
            STAGE1_H,
            centered=(False, True, True),
        )
    )

    body = body.cut(
        cq.Workplane("YZ", origin=(GUIDE_CAVITY_X1 - 0.001, 0.0, GUIDE_Z))
        .circle(ROD_R + 0.0025)
        .extrude(BASE_L - GUIDE_CAVITY_X1 + 0.002)
    )
    return body


def _outer_sleeve_shape():
    shell = cq.Workplane("XY").box(STAGE1_L, STAGE1_W, STAGE1_H, centered=(False, True, True))
    shell = shell.cut(
        cq.Workplane("XY", origin=(STAGE1_REAR_T, 0.0, 0.0)).box(
            STAGE1_L - STAGE1_REAR_T - STAGE1_FRONT_T,
            STAGE2_W,
            STAGE2_H,
            centered=(False, True, True),
        )
    )
    shell = shell.cut(
        cq.Workplane("XY", origin=(0.034, 0.0, 0.004)).box(
            0.150,
            0.050,
            STAGE1_H,
            centered=(False, True, False),
        )
    )
    shell = shell.cut(
        cq.Workplane("XY", origin=(STAGE1_L - STAGE1_FRONT_T - 0.001, 0.0, 0.0)).box(
            STAGE1_FRONT_T + 0.002,
            STAGE2_W + 0.004,
            STAGE2_H + 0.004,
            centered=(False, True, True),
        )
    )
    shell = shell.union(
        cq.Workplane("XY", origin=(STAGE1_L - 0.008, 0.0, 0.0)).box(
            0.008,
            0.086,
            0.046,
            centered=(False, True, True),
        )
    )
    shell = shell.cut(
        cq.Workplane("XY", origin=(STAGE1_L - 0.009, 0.0, 0.0)).box(
            0.010,
            STAGE2_W + 0.008,
            STAGE2_H + 0.006,
            centered=(False, True, True),
        )
    )
    return shell


def _inner_sleeve_shape():
    shell = cq.Workplane("XY").box(STAGE2_L, STAGE2_W, STAGE2_H, centered=(False, True, True))
    shell = shell.cut(
        cq.Workplane("XY", origin=(STAGE2_REAR_T, 0.0, 0.0)).box(
            STAGE2_L - STAGE2_REAR_T - STAGE2_FRONT_T,
            GUIDE_BAR_W,
            GUIDE_BAR_H,
            centered=(False, True, True),
        )
    )
    shell = shell.cut(
        cq.Workplane("XY", origin=(0.022, 0.0, 0.003)).box(
            0.095,
            0.034,
            STAGE2_H,
            centered=(False, True, False),
        )
    )
    shell = shell.union(
        cq.Workplane("XY", origin=(STAGE2_L - 0.008, 0.0, 0.0)).box(
            0.008,
            0.060,
            0.028,
            centered=(False, True, True),
        )
    )
    shell = shell.cut(
        cq.Workplane("YZ", origin=(STAGE2_L - STAGE2_FRONT_T - 0.001, 0.0, 0.0))
        .circle(ROD_R + 0.0015)
        .extrude(STAGE2_FRONT_T + 0.003)
    )
    return shell


def _plunger_shape():
    guide_bar = cq.Workplane("XY", origin=(-GUIDE_BAR_L, 0.0, 0.0)).box(
        GUIDE_BAR_L,
        GUIDE_BAR_W,
        GUIDE_BAR_H,
        centered=(False, True, True),
    )
    collar = cq.Workplane("YZ", origin=(0.0, 0.0, 0.0)).circle(COLLAR_R).extrude(COLLAR_L)
    rod = cq.Workplane("YZ", origin=(COLLAR_L, 0.0, 0.0)).circle(ROD_R).extrude(ROD_L)
    nose = (
        cq.Workplane("YZ", origin=(COLLAR_L + ROD_L, 0.0, 0.0))
        .circle(ROD_R)
        .workplane(offset=NOSE_L)
        .circle(NOSE_TIP_R)
        .loft()
    )
    return guide_bar.union(collar).union(rod).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_ejector_module")

    model.material("frame_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("satin_steel", rgba=(0.61, 0.64, 0.68, 1.0))
    model.material("polished_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("dark_oxide", rgba=(0.15, 0.16, 0.17, 1.0))

    base = model.part("base_frame")
    _add_box(base, (BASE_L, BASE_W, BASE_T), (BASE_L / 2.0, 0.0, BASE_T / 2.0), "frame_black", name="body")
    _add_box(base, (0.030, 0.130, 0.044), (0.015, 0.0, BASE_T + 0.022), "frame_black", name="rear_bridge")
    for side, y_pos in (("left", -0.061), ("right", 0.061)):
        _add_box(
            base,
            (0.315, 0.020, 0.062),
            (0.035 + 0.1575, y_pos, BASE_T + 0.031),
            "frame_black",
            name=f"{side}_wall",
        )
        _add_box(
            base,
            (0.170, 0.020, 0.006),
            (0.090 + 0.085, y_pos, 0.083),
            "dark_oxide",
            name=f"{side}_retainer",
        )
    _add_box(base, (0.112, 0.018, 0.066), (0.300 + 0.056, -0.040, BASE_T + 0.033), "frame_black", name="front_left_cheek")
    _add_box(base, (0.112, 0.018, 0.066), (0.300 + 0.056, 0.040, BASE_T + 0.033), "frame_black", name="front_right_cheek")
    _add_box(base, (0.112, 0.098, 0.024), (0.300 + 0.056, 0.0, 0.072), "frame_black", name="front_top_bridge")
    _add_box(base, (0.004, STAGE1_W, STAGE1_H), (GUIDE_CAVITY_X1 - 0.002, 0.0, GUIDE_Z), "dark_oxide", name="cavity_stop")
    _add_box(base, (0.004, 0.036, 0.028), (0.414, 0.0, GUIDE_Z), "dark_oxide", name="guide_face")

    stage1 = model.part("outer_sleeve")
    _add_box(stage1, (STAGE1_L, STAGE1_W, 0.008), (STAGE1_L / 2.0, 0.0, -0.024), "machined_steel", name="body")
    _add_box(stage1, (STAGE1_L, 0.012, 0.048), (STAGE1_L / 2.0, -0.043, 0.004), "machined_steel", name="left_wall")
    _add_box(stage1, (STAGE1_L, 0.012, 0.048), (STAGE1_L / 2.0, 0.043, 0.004), "machined_steel", name="right_wall")
    _add_box(stage1, (STAGE1_REAR_T, 0.074, 0.048), (STAGE1_REAR_T / 2.0, 0.0, 0.004), "dark_oxide", name="rear_plate")
    _add_box(stage1, (STAGE1_FRONT_T, 0.078, 0.010), (STAGE1_L - STAGE1_FRONT_T / 2.0, 0.0, 0.023), "dark_oxide", name="front_lip")

    stage2 = model.part("inner_sleeve")
    _add_box(stage2, (STAGE2_L, STAGE2_W, 0.006), (STAGE2_L / 2.0, 0.0, -0.015), "satin_steel", name="body")
    _add_box(stage2, (STAGE2_L, 0.008, 0.030), (STAGE2_L / 2.0, -0.032, 0.003), "satin_steel", name="left_wall")
    _add_box(stage2, (STAGE2_L, 0.008, 0.030), (STAGE2_L / 2.0, 0.032, 0.003), "satin_steel", name="right_wall")
    _add_box(stage2, (STAGE2_REAR_T, 0.056, 0.030), (STAGE2_REAR_T / 2.0, 0.0, 0.003), "dark_oxide", name="rear_plate")
    _add_box(stage2, (STAGE2_FRONT_T, 0.056, 0.008), (STAGE2_L - STAGE2_FRONT_T / 2.0, 0.0, 0.014), "dark_oxide", name="front_lip")
    _add_box(stage2, (STAGE2_FRONT_T, 0.008, 0.024), (STAGE2_L - STAGE2_FRONT_T / 2.0, -0.032, 0.003), "dark_oxide", name="front_left_cheek")
    _add_box(stage2, (STAGE2_FRONT_T, 0.008, 0.024), (STAGE2_L - STAGE2_FRONT_T / 2.0, 0.032, 0.003), "dark_oxide", name="front_right_cheek")

    plunger = model.part("plunger")
    _add_box(plunger, (GUIDE_BAR_L, GUIDE_BAR_W, GUIDE_BAR_H), (-GUIDE_BAR_L / 2.0, 0.0, 0.0), "dark_oxide", name="guide_bar")
    _add_x_cylinder(plunger, COLLAR_R, COLLAR_L, COLLAR_L / 2.0, "polished_steel", name="collar")
    _add_x_cylinder(plunger, ROD_R, ROD_L, COLLAR_L + ROD_L / 2.0, "polished_steel", name="rod")
    _add_x_cylinder(plunger, 0.005, 0.016, COLLAR_L + ROD_L + 0.008, "polished_steel", name="nose_shank")
    _add_x_cylinder(plunger, NOSE_TIP_R, 0.006, COLLAR_L + ROD_L + 0.019, "polished_steel", name="nose_tip")

    model.articulation(
        "base_to_outer_sleeve",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(BASE_TO_STAGE1_X, 0.0, GUIDE_Z - 0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.18, lower=0.0, upper=STAGE1_TRAVEL),
    )
    model.articulation(
        "outer_to_inner_sleeve",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(STAGE1_TO_STAGE2_X, 0.0, -0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.22, lower=0.0, upper=STAGE2_TRAVEL),
    )
    model.articulation(
        "inner_sleeve_to_plunger",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=plunger,
        origin=Origin(xyz=(STAGE2_TO_PLUNGER_X, 0.0, -0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.30, lower=0.0, upper=STAGE3_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    stage1 = object_model.get_part("outer_sleeve")
    stage2 = object_model.get_part("inner_sleeve")
    plunger = object_model.get_part("plunger")
    joint1 = object_model.get_articulation("base_to_outer_sleeve")
    joint2 = object_model.get_articulation("outer_to_inner_sleeve")
    joint3 = object_model.get_articulation("inner_sleeve_to_plunger")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    joints_ok = True
    for joint, upper in (
        (joint1, STAGE1_TRAVEL),
        (joint2, STAGE2_TRAVEL),
        (joint3, STAGE3_TRAVEL),
    ):
        limits = joint.motion_limits
        joints_ok = joints_ok and joint.axis == (1.0, 0.0, 0.0)
        joints_ok = joints_ok and limits is not None
        joints_ok = joints_ok and limits.lower == 0.0 and limits.upper == upper
    ctx.check(
        "serial_prismatic_axis_stack",
        joints_ok,
        "All sliding members should form a serial +X prismatic stack with explicit stroke limits.",
    )

    with ctx.pose({joint1: 0.0, joint2: 0.0, joint3: 0.0}):
        ctx.expect_contact(base, stage1, name="base_supports_outer_sleeve_at_rest")
        ctx.expect_contact(stage1, stage2, name="outer_sleeve_supports_inner_sleeve_at_rest")
        ctx.expect_contact(stage2, plunger, elem_b="guide_bar", name="inner_sleeve_supports_plunger_at_rest")
        ctx.expect_overlap(base, stage1, axes="x", min_overlap=0.21, name="outer_sleeve_overlaps_base_at_rest")
        ctx.expect_overlap(stage1, stage2, axes="x", min_overlap=0.15, name="inner_sleeve_overlaps_outer_at_rest")
        ctx.expect_overlap(
            stage2,
            plunger,
            axes="x",
            min_overlap=0.060,
            elem_b="guide_bar",
            name="plunger_guide_bar_overlaps_inner_at_rest",
        )
        ctx.expect_within(stage1, base, axes="yz", margin=0.0, name="outer_sleeve_stays_in_base_section")
        ctx.expect_within(stage2, stage1, axes="yz", margin=0.0, name="inner_sleeve_stays_in_outer_section")
        ctx.expect_within(
            plunger,
            stage2,
            axes="yz",
            margin=0.0,
            inner_elem="guide_bar",
            name="plunger_guide_bar_stays_in_inner_section",
        )
        ctx.expect_gap(
            plunger,
            base,
            axis="x",
            positive_elem="nose_tip",
            negative_elem="guide_face",
            min_gap=0.003,
            max_gap=0.010,
            name="plunger_nose_emerges_from_front_guide_at_rest",
        )

    with ctx.pose({joint1: STAGE1_TRAVEL, joint2: STAGE2_TRAVEL, joint3: STAGE3_TRAVEL}):
        ctx.expect_contact(base, stage1, name="base_supports_outer_sleeve_at_full_stroke")
        ctx.expect_contact(stage1, stage2, name="outer_sleeve_supports_inner_sleeve_at_full_stroke")
        ctx.expect_contact(stage2, plunger, elem_b="guide_bar", name="inner_sleeve_supports_plunger_at_full_stroke")
        ctx.expect_overlap(base, stage1, axes="x", min_overlap=0.21, name="outer_sleeve_keeps_base_overlap")
        ctx.expect_overlap(stage1, stage2, axes="x", min_overlap=0.14, name="inner_sleeve_keeps_outer_overlap")
        ctx.expect_overlap(
            stage2,
            plunger,
            axes="x",
            min_overlap=0.060,
            elem_b="guide_bar",
            name="plunger_guide_bar_keeps_inner_overlap",
        )
        ctx.expect_gap(
            base,
            stage1,
            axis="x",
            positive_elem="cavity_stop",
            negative_elem="front_lip",
            min_gap=0.008,
            max_gap=0.015,
            name="outer_sleeve_stops_before_front_guide_shoulder",
        )
        ctx.expect_gap(
            plunger,
            base,
            axis="x",
            positive_elem="nose_tip",
            negative_elem="guide_face",
            min_gap=0.075,
            name="plunger_nose_extends_cleanly_at_full_stroke",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
