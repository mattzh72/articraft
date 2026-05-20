from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_espresso_machine")

    body_metal = model.material("body_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.12, 0.12, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.30, 0.24, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_metal,
        name="base_plinth",
    )
    body.visual(
        Box((0.30, 0.025, 0.20)),
        origin=Origin(xyz=(0.0, 0.1075, 0.13)),
        material=body_metal,
        name="side_wall_0",
    )
    body.visual(
        Box((0.30, 0.025, 0.20)),
        origin=Origin(xyz=(0.0, -0.1075, 0.13)),
        material=body_metal,
        name="side_wall_1",
    )
    body.visual(
        Box((0.025, 0.19, 0.265)),
        origin=Origin(xyz=(-0.1375, 0.0, 0.1625)),
        material=body_metal,
        name="rear_wall",
    )
    body.visual(
        Box((0.145, 0.24, 0.11)),
        origin=Origin(xyz=(0.0675, 0.0, 0.27)),
        material=body_metal,
        name="upper_case",
    )
    body.visual(
        Box((0.060, 0.130, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, 0.176)),
        material=dark_metal,
        name="cup_platform",
    )
    body.visual(
        Box((0.110, 0.030, 0.012)),
        origin=Origin(xyz=(0.050, 0.080, 0.176)),
        material=dark_metal,
        name="cup_platform_side_0",
    )
    body.visual(
        Box((0.110, 0.030, 0.012)),
        origin=Origin(xyz=(0.050, -0.080, 0.176)),
        material=dark_metal,
        name="cup_platform_side_1",
    )
    body.visual(
        Box((0.13, 0.22, 0.015)),
        origin=Origin(xyz=(-0.075, 0.0, 0.2875)),
        material=black_plastic,
        name="reservoir_rim",
    )
    body.visual(
        Box((0.13, 0.031, 0.018)),
        origin=Origin(xyz=(0.085, 0.0915, 0.106)),
        material=black_plastic,
        name="tray_rail_0",
    )
    body.visual(
        Box((0.13, 0.031, 0.018)),
        origin=Origin(xyz=(0.085, -0.0915, 0.106)),
        material=black_plastic,
        name="tray_rail_1",
    )
    body.visual(
        Box((0.058, 0.082, 0.030)),
        origin=Origin(xyz=(0.106, 0.0, 0.225)),
        material=dark_metal,
        name="group_block",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.108, 0.0, 0.201)),
        material=dark_metal,
        name="group_head",
    )
    body.visual(
        Box((0.055, 0.018, 0.080)),
        origin=Origin(xyz=(0.060, 0.113, 0.215)),
        material=dark_metal,
        name="side_support",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.30, 0.24, 0.34)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.13, 0.20, 0.012)),
        origin=Origin(xyz=(0.065, 0.0, 0.006)),
        material=black_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.024, 0.055, 0.010)),
        origin=Origin(xyz=(0.118, 0.0, 0.010)),
        material=black_plastic,
        name="lid_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.13, 0.20, 0.016)),
        mass=0.25,
        origin=Origin(xyz=(0.065, 0.0, 0.008)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.140, 0.0, 0.295)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=dark_metal,
        name="basket",
    )
    portafilter.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_metal,
        name="collar",
    )
    portafilter.visual(
        Box((0.030, 0.018, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, -0.012)),
        material=dark_metal,
        name="neck",
    )
    portafilter.visual(
        Box((0.105, 0.022, 0.020)),
        origin=Origin(xyz=(0.0975, 0.0, -0.014)),
        material=black_plastic,
        name="handle",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.18, 0.07, 0.04)),
        mass=0.55,
        origin=Origin(xyz=(0.05, 0.0, -0.012)),
    )

    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.108, 0.0, 0.191)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.55,
        ),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=dark_metal,
        name="pivot",
    )
    steam_wand.visual(
        Box((0.050, 0.010, 0.010)),
        origin=Origin(xyz=(0.025, 0.0, -0.011)),
        material=dark_metal,
        name="arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.135),
        origin=Origin(xyz=(0.050, 0.0, -0.0825)),
        material=dark_metal,
        name="tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0035, length=0.020),
        origin=Origin(
            xyz=(0.060, 0.0, -0.145),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_metal,
        name="tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.08, 0.03, 0.17)),
        mass=0.18,
        origin=Origin(xyz=(0.04, 0.0, -0.08)),
    )

    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.055, 0.128, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.10,
            upper=0.65,
        ),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.120, 0.140, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.003)),
        material=dark_metal,
        name="tray_floor",
    )
    drip_tray.visual(
        Box((0.120, 0.012, 0.011)),
        origin=Origin(xyz=(0.060, 0.070, 0.0085)),
        material=dark_metal,
        name="tray_side_0",
    )
    drip_tray.visual(
        Box((0.120, 0.012, 0.011)),
        origin=Origin(xyz=(0.060, -0.070, 0.0085)),
        material=dark_metal,
        name="tray_side_1",
    )
    drip_tray.visual(
        Box((0.012, 0.140, 0.018)),
        origin=Origin(xyz=(0.126, 0.0, 0.009)),
        material=black_plastic,
        name="front_lip",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.14, 0.16, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.065, 0.0, 0.009)),
    )

    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.018, 0.0, 0.099)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.055,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    drip_tray = object_model.get_part("drip_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.010,
        max_penetration=0.001,
        positive_elem="lid_panel",
        negative_elem="reservoir_rim",
        name="lid sits nearly flush on the reservoir rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.12,
        elem_a="lid_panel",
        elem_b="reservoir_rim",
        name="lid covers the rear reservoir opening",
    )

    with ctx.pose({lid_hinge: math.radians(85.0)}):
        opened_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    closed_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid swings upward to expose the reservoir",
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][2] > closed_panel[1][2] + 0.10,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem="group_head",
        negative_elem="basket",
        name="portafilter seats directly under the group head",
    )
    ctx.expect_overlap(
        body,
        portafilter,
        axes="xy",
        min_overlap=0.05,
        elem_a="group_head",
        elem_b="basket",
        name="portafilter stays centered on the brew axis",
    )

    closed_handle = ctx.part_element_world_aabb(portafilter, elem="handle")
    with ctx.pose({portafilter_joint: 0.45}):
        turned_handle = ctx.part_element_world_aabb(portafilter, elem="handle")

    ctx.check(
        "portafilter rotates about the vertical brew axis",
        closed_handle is not None
        and turned_handle is not None
        and (turned_handle[0][1] + turned_handle[1][1]) / 2.0
        > (closed_handle[0][1] + closed_handle[1][1]) / 2.0 + 0.025,
        details=f"closed={closed_handle}, turned={turned_handle}",
    )

    ctx.expect_gap(
        steam_wand,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.015,
        positive_elem="tube",
        negative_elem="side_support",
        name="steam wand hangs just outside its side support",
    )

    closed_tube = ctx.part_element_world_aabb(steam_wand, elem="tube")
    with ctx.pose({wand_joint: 0.60}):
        swung_tube = ctx.part_element_world_aabb(steam_wand, elem="tube")

    ctx.check(
        "steam wand swings outward on its side pivot",
        closed_tube is not None
        and swung_tube is not None
        and (swung_tube[0][1] + swung_tube[1][1]) / 2.0
        > (closed_tube[0][1] + closed_tube[1][1]) / 2.0 + 0.020,
        details=f"closed={closed_tube}, swung={swung_tube}",
    )

    ctx.expect_gap(
        body,
        drip_tray,
        axis="z",
        min_gap=0.045,
        max_gap=0.075,
        positive_elem="cup_platform",
        negative_elem="front_lip",
        name="drip tray sits below the cup platform",
    )
    ctx.expect_gap(
        body,
        drip_tray,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="tray_rail_0",
        negative_elem="tray_side_0",
        name="positive-side tray guide captures the sliding tray",
    )
    ctx.expect_gap(
        drip_tray,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="tray_side_1",
        negative_elem="tray_rail_1",
        name="negative-side tray guide captures the sliding tray",
    )

    closed_lip = ctx.part_element_world_aabb(drip_tray, elem="front_lip")
    with ctx.pose({tray_joint: 0.055}):
        extended_lip = ctx.part_element_world_aabb(drip_tray, elem="front_lip")

    ctx.check(
        "drip tray slides forward along the base guide",
        closed_lip is not None
        and extended_lip is not None
        and extended_lip[1][0] > closed_lip[1][0] + 0.045,
        details=f"closed={closed_lip}, extended={extended_lip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
