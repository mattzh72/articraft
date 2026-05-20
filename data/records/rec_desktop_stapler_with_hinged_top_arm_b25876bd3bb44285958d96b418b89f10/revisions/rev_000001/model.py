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


def _base_deck_shape() -> cq.Workplane:
    profile = [
        (0.000, -0.006),
        (0.000, -0.036),
        (0.018, -0.038),
        (0.112, -0.034),
        (0.149, -0.030),
        (0.158, -0.023),
        (0.154, -0.014),
        (0.120, -0.007),
        (0.040, -0.004),
        (0.000, -0.005),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.028, both=True)

def _arm_shell_shape() -> cq.Workplane:
    profile = [
        (0.000, 0.003),
        (0.000, 0.016),
        (0.020, 0.018),
        (0.102, 0.017),
        (0.136, 0.015),
        (0.148, 0.010),
        (0.148, 0.004),
        (0.130, 0.002),
        (0.026, 0.001),
        (0.000, 0.002),
    ]
    shell = cq.Workplane("XZ").polyline(profile).close().extrude(0.013, both=True)
    cavity = cq.Workplane("XY").box(0.116, 0.018, 0.010).translate((0.080, 0.0, 0.0065))
    bridge = cq.Workplane("XY").box(0.018, 0.018, 0.006).translate((0.007, 0.0, 0.003))
    return shell.cut(cavity).union(bridge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_stapler")

    base_dark = model.material("base_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    arm_black = model.material("arm_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_deck_shape(), "base_deck"),
        material=base_dark,
        name="base_deck",
    )
    for idx, y in enumerate((-0.0185, 0.0185)):
        base.visual(
            Box((0.012, 0.006, 0.024)),
            origin=Origin(xyz=(0.006, y, 0.0)),
            material=steel,
            name=f"hinge_tower_{idx}",
        )
    base.visual(
        Box((0.020, 0.022, 0.006)),
        origin=Origin(xyz=(0.138, 0.0, -0.0190)),
        material=steel,
        name="anvil_plate",
    )
    base.visual(
        Box((0.030, 0.014, 0.014)),
        origin=Origin(xyz=(0.124, 0.0, -0.0260)),
        material=base_dark,
        name="anvil_mount",
    )
    for idx, y in enumerate((-0.016, 0.016)):
        base.visual(
            Box((0.040, 0.010, 0.004)),
            origin=Origin(xyz=(0.050 + idx * 0.055, y, -0.0355)),
            material=rubber,
            name=f"foot_pad_{idx}",
        )

    magazine = model.part("magazine")
    magazine.visual(
        Box((0.130, 0.029, 0.007)),
        origin=Origin(xyz=(0.075, 0.0, 0.0165)),
        material=steel,
        name="magazine_shell",
    )
    magazine.visual(
        Box((0.140, 0.004, 0.012)),
        origin=Origin(xyz=(0.072, -0.0125, 0.007)),
        material=steel,
        name="magazine_side_0",
    )
    magazine.visual(
        Box((0.140, 0.004, 0.012)),
        origin=Origin(xyz=(0.072, 0.0125, 0.007)),
        material=steel,
        name="magazine_side_1",
    )
    magazine.visual(
        Box((0.014, 0.029, 0.008)),
        origin=Origin(xyz=(0.005, 0.0, 0.005)),
        material=steel,
        name="magazine_web",
    )
    magazine.visual(
        Box((0.012, 0.029, 0.004)),
        origin=Origin(xyz=(0.138, 0.0, 0.015)),
        material=steel,
        name="magazine_nose",
    )
    magazine.visual(
        Box((0.012, 0.003, 0.010)),
        origin=Origin(xyz=(0.006, -0.0155, 0.0145)),
        material=steel,
        name="arm_bracket_0",
    )
    magazine.visual(
        Box((0.012, 0.003, 0.010)),
        origin=Origin(xyz=(0.006, 0.0155, 0.0145)),
        material=steel,
        name="arm_bracket_1",
    )
    magazine.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="magazine_barrel",
    )

    arm = model.part("top_arm")
    arm.visual(
        mesh_from_cadquery(_arm_shell_shape(), "top_arm_shell"),
        material=arm_black,
        name="arm_shell",
    )
    arm.visual(
        Cylinder(radius=0.0035, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="arm_barrel",
    )
    arm.visual(
        Box((0.084, 0.018, 0.002)),
        origin=Origin(xyz=(0.090, 0.0, 0.018)),
        material=rubber,
        name="grip_pad",
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.108, 0.018, 0.0120)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0070)),
        material=steel,
        name="tray_member",
    )
    tray.visual(
        Box((0.016, 0.022, 0.011)),
        origin=Origin(xyz=(0.008, 0.0, 0.0055)),
        material=arm_black,
        name="tray_handle",
    )

    model.articulation(
        "base_to_magazine",
        ArticulationType.REVOLUTE,
        parent=base,
        child=magazine,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.0, lower=0.0, upper=1.22),
    )
    model.articulation(
        "magazine_to_top_arm",
        ArticulationType.REVOLUTE,
        parent=magazine,
        child=arm,
        origin=Origin(xyz=(0.006, 0.0, 0.021)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0, lower=0.0, upper=1.00),
    )
    model.articulation(
        "magazine_to_tray",
        ArticulationType.PRISMATIC,
        parent=magazine,
        child=tray,
        origin=Origin(xyz=(0.128, 0.0, 0.0005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.12, lower=0.0, upper=0.048),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def elem_bounds(part_name: str, elem_name: str):
        return ctx.part_element_world_aabb(part_name, elem=elem_name)

    base = object_model.get_part("base")
    magazine = object_model.get_part("magazine")
    arm = object_model.get_part("top_arm")
    tray = object_model.get_part("tray")

    magazine_hinge = object_model.get_articulation("base_to_magazine")
    arm_hinge = object_model.get_articulation("magazine_to_top_arm")
    tray_slide = object_model.get_articulation("magazine_to_tray")

    with ctx.pose({magazine_hinge: 0.0, arm_hinge: 0.0, tray_slide: 0.0}):
        ctx.expect_gap(
            magazine,
            base,
            axis="z",
            positive_elem="magazine_side_0",
            negative_elem="base_deck",
            min_gap=0.002,
            max_gap=0.015,
            name="closed magazine sits just above the heavy base",
        )
        ctx.expect_overlap(
            magazine,
            base,
            axes="xy",
            elem_a="magazine_shell",
            elem_b="base_deck",
            min_overlap=0.020,
            name="closed magazine remains centered over the base deck",
        )
        ctx.expect_gap(
            arm,
            magazine,
            axis="z",
            positive_elem="arm_shell",
            negative_elem="magazine_shell",
            min_gap=0.0003,
            max_gap=0.008,
            name="top arm rests just above the staple magazine",
        )
        ctx.expect_within(
            tray,
            magazine,
            axes="yz",
            inner_elem="tray_member",
            margin=0.0015,
            name="loading tray stays centered inside the magazine channel",
        )
        ctx.expect_overlap(
            tray,
            magazine,
            axes="x",
            elem_a="tray_member",
            min_overlap=0.090,
            name="collapsed tray remains deeply inserted in the magazine",
        )

    mag_rest_bounds = elem_bounds("magazine", "magazine_shell")
    with ctx.pose({magazine_hinge: 1.05}):
        mag_open_bounds = elem_bounds("magazine", "magazine_shell")
    ctx.check(
        "magazine rotates upward from the rear hinge tower",
        mag_rest_bounds is not None
        and mag_open_bounds is not None
        and mag_open_bounds[1][2] > mag_rest_bounds[1][2] + 0.060,
        details=f"rest={mag_rest_bounds}, open={mag_open_bounds}",
    )

    arm_rest_bounds = elem_bounds("top_arm", "grip_pad")
    with ctx.pose({arm_hinge: 0.90}):
        arm_open_bounds = elem_bounds("top_arm", "grip_pad")
        ctx.expect_gap(
            arm,
            magazine,
            axis="z",
            positive_elem="grip_pad",
            negative_elem="magazine_shell",
            min_gap=0.015,
            name="opened top arm lifts clearly away from the magazine",
        )
    ctx.check(
        "top arm opens upward on its own rear hinge",
        arm_rest_bounds is not None
        and arm_open_bounds is not None
        and arm_open_bounds[1][2] > arm_rest_bounds[1][2] + 0.070,
        details=f"rest={arm_rest_bounds}, open={arm_open_bounds}",
    )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.048}):
        tray_open = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            magazine,
            axes="yz",
            inner_elem="tray_member",
            margin=0.0015,
            name="extended tray stays laterally guided in the magazine",
        )
        ctx.expect_overlap(
            tray,
            magazine,
            axes="x",
            elem_a="tray_member",
            min_overlap=0.070,
            name="extended tray retains insertion in the magazine body",
        )
    ctx.check(
        "tray slides forward out of the front loading opening",
        tray_rest is not None and tray_open is not None and tray_open[0] > tray_rest[0] + 0.035,
        details=f"rest={tray_rest}, extended={tray_open}",
    )

    return ctx.report()


object_model = build_object_model()
