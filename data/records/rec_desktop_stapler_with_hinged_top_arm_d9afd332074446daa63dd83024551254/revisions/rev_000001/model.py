from __future__ import annotations

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

BASE_L = 0.145
BASE_W = 0.046
BASE_T = 0.010

HINGE_X = -0.052
HINGE_Z = 0.0255

TOP_W = 0.031
TOP_OUTER_L = 0.126
TOP_INNER_W = 0.022
TOP_TRAVEL = 0.90

TRAY_TRAVEL = 0.052


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _make_base_plate() -> cq.Workplane:
    return _box((BASE_L, BASE_W, BASE_T), (0.0, 0.0, BASE_T / 2.0))


def _make_rear_support() -> cq.Workplane:
    cheek_len = 0.018
    cheek_w = 0.007
    cheek_h = 0.019
    cheek_z = BASE_T + (cheek_h / 2.0) - 0.001
    cheek_y = 0.0195

    left_cheek = _box((cheek_len, cheek_w, cheek_h), (HINGE_X, -cheek_y, cheek_z))
    right_cheek = _box((cheek_len, cheek_w, cheek_h), (HINGE_X, cheek_y, cheek_z))
    rear_bridge = _box((0.010, 0.028, 0.014), (HINGE_X - 0.008, 0.0, 0.015))
    return left_cheek.union(right_cheek).union(rear_bridge)


def _make_top_shell() -> cq.Workplane:
    rear_cap = _box((0.050, 0.024, 0.017), (0.017, 0.0, 0.006))
    rear_cap = rear_cap.rotate((0.017, 0.0, 0.000), (0.017, 1.0, 0.000), 11.0)
    mid_cap = _box((0.064, 0.030, 0.013), (0.056, 0.0, 0.005))
    mid_cap = mid_cap.rotate((0.056, 0.0, -0.001), (0.056, 1.0, -0.001), 4.0)
    nose_cap = _box((0.030, 0.026, 0.008), (0.106, 0.0, 0.000))
    nose_cap = nose_cap.rotate((0.106, 0.0, -0.006), (0.106, 1.0, -0.006), -8.0)
    left_wall = _box((0.102, 0.004, 0.011), (0.056, -0.0125, -0.006))
    right_wall = _box((0.102, 0.004, 0.011), (0.056, 0.0125, -0.006))
    left_rail = _box((0.082, 0.003, 0.002), (0.058, -0.0100, -0.007))
    right_rail = _box((0.082, 0.003, 0.002), (0.058, 0.0100, -0.007))

    return rear_cap.union(mid_cap).union(nose_cap).union(left_wall).union(right_wall).union(left_rail).union(right_rail)


def _make_tray() -> cq.Workplane:
    tray_outer = _box((0.106, 0.017, 0.006), (-0.035, 0.0, 0.0))
    tray_cavity = _box((0.094, 0.011, 0.0045), (-0.041, 0.0, 0.0010))
    front_tab = _box((0.010, 0.016, 0.005), (0.013, 0.0, -0.0005))
    nose_lip = _box((0.004, 0.017, 0.0025), (0.020, 0.0, -0.002))
    return tray_outer.cut(tray_cavity).union(front_tab).union(nose_lip)


def _make_top_skin() -> cq.Workplane:
    spine = (
        cq.Workplane("XY")
        .box(0.106, 0.026, 0.006)
        .translate((0.050, 0.0, 0.013))
        .rotate((0.050, 0.0, 0.013), (0.050, 1.0, 0.013), 4.0)
    )
    nose = (
        cq.Workplane("XY")
        .box(0.030, 0.024, 0.006)
        .translate((0.106, 0.0, 0.009))
        .rotate((0.106, 0.0, 0.009), (0.106, 1.0, 0.009), -10.0)
    )
    thumb_pad = (
        cq.Workplane("XY")
        .box(0.034, 0.020, 0.004)
        .translate((0.090, 0.0, 0.015))
    )
    return spine.union(nose).union(thumb_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    model.material("base_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("arm_black", rgba=(0.14, 0.14, 0.16, 1.0))
    model.material("zinc", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_plate(), "base_plate"), material="base_black", name="base_plate")
    base.visual(mesh_from_cadquery(_make_rear_support(), "rear_support"), material="base_black", name="rear_support")
    base.visual(
        Box((0.018, 0.010, 0.0014)),
        origin=Origin(xyz=(0.014, 0.0, BASE_T + 0.0007)),
        material="steel",
        name="anvil",
    )

    top_arm = model.part("top_arm")
    top_arm.visual(
        Box((0.050, 0.024, 0.017)),
        origin=Origin(xyz=(0.017, 0.0, 0.011), rpy=(0.0, 0.19, 0.0)),
        material="arm_black",
        name="rear_cover",
    )
    top_arm.visual(
        Box((0.064, 0.030, 0.013)),
        origin=Origin(xyz=(0.056, 0.0, 0.0070), rpy=(0.0, 0.07, 0.0)),
        material="arm_black",
        name="mid_cover",
    )
    top_arm.visual(
        Box((0.030, 0.026, 0.008)),
        origin=Origin(xyz=(0.106, 0.0, 0.0050), rpy=(0.0, -0.14, 0.0)),
        material="arm_black",
        name="nose_cover",
    )
    top_arm.visual(
        mesh_from_cadquery(_make_top_skin(), "top_shell"),
        material="arm_black",
        name="top_shell",
    )
    top_arm.visual(
        Box((0.102, 0.004, 0.011)),
        origin=Origin(xyz=(0.056, -0.0125, -0.006)),
        material="arm_black",
        name="left_wall",
    )
    top_arm.visual(
        Box((0.102, 0.004, 0.011)),
        origin=Origin(xyz=(0.056, 0.0125, -0.006)),
        material="arm_black",
        name="right_wall",
    )
    top_arm.visual(
        Box((0.082, 0.003, 0.002)),
        origin=Origin(xyz=(0.058, -0.0100, -0.007)),
        material="steel",
        name="left_rail",
    )
    top_arm.visual(
        Box((0.082, 0.003, 0.002)),
        origin=Origin(xyz=(0.058, 0.0100, -0.007)),
        material="steel",
        name="right_rail",
    )
    top_arm.visual(
        Box((0.016, 0.012, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.000)),
        material="arm_black",
        name="hinge_web",
    )
    top_arm.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="zinc",
        name="hinge_barrel",
    )

    tray = model.part("staple_tray")
    tray.visual(mesh_from_cadquery(_make_tray(), "staple_tray"), material="zinc", name="tray_body")

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top_arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TOP_TRAVEL, effort=12.0, velocity=2.0),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=top_arm,
        child=tray,
        origin=Origin(xyz=(0.108, 0.0, -0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TRAY_TRAVEL, effort=8.0, velocity=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_arm = object_model.get_part("top_arm")
    tray = object_model.get_part("staple_tray")
    arm_hinge = object_model.get_articulation("arm_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    with ctx.pose({arm_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            top_arm,
            base,
            axis="z",
            min_gap=0.001,
            max_gap=0.010,
            elem_a="left_wall",
            elem_b="base_plate",
            name="closed arm hovers just above the weighted base",
        )
        ctx.expect_overlap(
            top_arm,
            base,
            axes="xy",
            min_overlap=0.020,
            elem_a="mid_cover",
            elem_b="base_plate",
            name="arm footprint stays centered over the base",
        )
        ctx.expect_within(
            tray,
            top_arm,
            axes="y",
            margin=0.0005,
            elem_a="tray_body",
            elem_b="mid_cover",
            name="retracted staple tray stays laterally within the upper body",
        )
        ctx.expect_gap(
            top_arm,
            tray,
            axis="z",
            min_gap=0.0005,
            max_gap=0.006,
            positive_elem="mid_cover",
            negative_elem="tray_body",
            name="retracted staple tray runs just below the arm cover",
        )
        ctx.expect_gap(
            tray,
            top_arm,
            axis="y",
            min_gap=0.001,
            max_gap=0.005,
            positive_elem="tray_body",
            negative_elem="left_wall",
            name="tray clears the left guide wall",
        )
        ctx.expect_gap(
            top_arm,
            tray,
            axis="y",
            min_gap=0.001,
            max_gap=0.005,
            positive_elem="right_wall",
            negative_elem="tray_body",
            name="tray clears the right guide wall",
        )
        ctx.expect_overlap(
            tray,
            top_arm,
            axes="x",
            min_overlap=0.030,
            elem_a="tray_body",
            elem_b="left_wall",
            name="retracted tray remains deeply inserted in the top arm",
        )

    if arm_hinge.motion_limits is not None and arm_hinge.motion_limits.upper is not None:
        rest_shell = ctx.part_element_world_aabb(top_arm, elem="mid_cover")
        with ctx.pose({arm_hinge: arm_hinge.motion_limits.upper, tray_slide: 0.0}):
            open_shell = ctx.part_element_world_aabb(top_arm, elem="mid_cover")
        ctx.check(
            "top arm opens upward from the rear hinge",
            rest_shell is not None
            and open_shell is not None
            and open_shell[1][2] > rest_shell[1][2] + 0.035,
            details=f"rest={rest_shell}, open={open_shell}",
        )

    if tray_slide.motion_limits is not None and tray_slide.motion_limits.upper is not None:
        rest_pos = ctx.part_world_position(tray)
        with ctx.pose({arm_hinge: 0.0, tray_slide: tray_slide.motion_limits.upper}):
            extended_pos = ctx.part_world_position(tray)
            ctx.expect_within(
                tray,
                top_arm,
                axes="y",
                margin=0.0005,
                elem_a="tray_body",
                elem_b="mid_cover",
                name="extended tray stays laterally within the upper body",
            )
            ctx.expect_gap(
                top_arm,
                tray,
                axis="z",
                min_gap=0.0005,
                max_gap=0.006,
                positive_elem="mid_cover",
                negative_elem="tray_body",
                name="extended tray still runs below the arm cover",
            )
            ctx.expect_overlap(
                tray,
                top_arm,
                axes="x",
                min_overlap=0.028,
                elem_a="tray_body",
                elem_b="left_wall",
                name="extended tray still retains magazine insertion",
            )
        ctx.check(
            "tray slides forward out of the nose",
            rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.040,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
