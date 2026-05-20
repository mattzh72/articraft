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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="espresso_machine")

    brushed = model.material("brushed", rgba=(0.67, 0.69, 0.71, 1.0))
    dark = model.material("dark", rgba=(0.16, 0.17, 0.18, 1.0))
    black = model.material("black", rgba=(0.07, 0.07, 0.08, 1.0))
    smoke = model.material("smoke", rgba=(0.34, 0.36, 0.39, 0.38))
    tray_steel = model.material("tray_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.36, 0.28, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=dark,
        name="base_plinth",
    )
    body.visual(
        Box((0.20, 0.28, 0.25)),
        origin=Origin(xyz=(-0.10, 0.0, 0.145)),
        material=brushed,
        name="rear_housing",
    )
    body.visual(
        Box((0.22, 0.28, 0.08)),
        origin=Origin(xyz=(-0.01, 0.0, 0.31)),
        material=brushed,
        name="top_housing",
    )
    body.visual(
        Box((0.10, 0.26, 0.12)),
        origin=Origin(xyz=(0.14, 0.0, 0.25)),
        material=brushed,
        name="front_bridge",
    )
    body.visual(
        Box((0.18, 0.03, 0.17)),
        origin=Origin(xyz=(0.09, -0.125, 0.105)),
        material=brushed,
        name="left_cheek",
    )
    body.visual(
        Box((0.18, 0.03, 0.17)),
        origin=Origin(xyz=(0.09, 0.125, 0.105)),
        material=brushed,
        name="right_cheek",
    )
    body.visual(
        Box((0.15, 0.03, 0.12)),
        origin=Origin(xyz=(0.10, -0.125, 0.25)),
        material=brushed,
        name="left_upper_cheek",
    )
    body.visual(
        Box((0.15, 0.03, 0.12)),
        origin=Origin(xyz=(0.10, 0.125, 0.25)),
        material=brushed,
        name="right_upper_cheek",
    )
    body.visual(
        Box((0.03, 0.16, 0.04)),
        origin=Origin(xyz=(0.005, 0.0, 0.05)),
        material=dark,
        name="tray_stop",
    )
    body.visual(
        Box((0.03, 0.16, 0.01)),
        origin=Origin(xyz=(0.005, 0.0, 0.025)),
        material=dark,
        name="tray_stop_rib",
    )
    body.visual(
        Box((0.10, 0.012, 0.016)),
        origin=Origin(xyz=(0.08, -0.103, 0.036)),
        material=dark,
        name="left_guide",
    )
    body.visual(
        Box((0.10, 0.012, 0.008)),
        origin=Origin(xyz=(0.08, -0.103, 0.024)),
        material=dark,
        name="left_guide_rib",
    )
    body.visual(
        Box((0.10, 0.012, 0.016)),
        origin=Origin(xyz=(0.08, 0.103, 0.036)),
        material=dark,
        name="right_guide",
    )
    body.visual(
        Box((0.10, 0.012, 0.008)),
        origin=Origin(xyz=(0.08, 0.103, 0.024)),
        material=dark,
        name="right_guide_rib",
    )
    body.visual(
        Box((0.13, 0.12, 0.07)),
        origin=Origin(xyz=(0.01, 0.075, 0.385)),
        material=dark,
        name="grinder_base",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.06),
        origin=Origin(xyz=(0.205, 0.0, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="group_head",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.212, 0.0, 0.208)),
        material=black,
        name="group_collar",
    )
    body.visual(
        Box((0.03, 0.04, 0.055)),
        origin=Origin(xyz=(0.175, 0.075, 0.182)),
        material=dark,
        name="grinder_chute",
    )
    body.visual(
        Box((0.04, 0.022, 0.06)),
        origin=Origin(xyz=(0.12, 0.151, 0.245)),
        material=dark,
        name="wand_support",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0.13, 0.160, 0.245)),
        material=dark,
        name="wand_pivot_housing",
    )
    body.visual(
        Box((0.014, 0.13, 0.09)),
        origin=Origin(xyz=(0.186, -0.030, 0.304)),
        material=dark,
        name="control_fascia",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.195, -0.075, 0.286), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="brew_knob",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.192, -0.035, 0.322), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="button_0",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.192, -0.005, 0.322), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="button_1",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(xyz=(0.192, 0.025, 0.322), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="button_2",
    )
    cup_warmer = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.16, 0.18),
            0.003,
            slot_size=(0.020, 0.004),
            pitch=(0.028, 0.016),
            frame=0.012,
            corner_radius=0.006,
            stagger=True,
        ),
        "cup_warmer",
    )
    body.visual(
        cup_warmer,
        origin=Origin(xyz=(-0.045, -0.055, 0.3515)),
        material=tray_steel,
        name="cup_warmer",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.40, 0.30, 0.44)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    hopper = model.part("hopper")
    hopper.visual(
        Box((0.126, 0.116, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=black,
        name="collar",
    )
    hopper.visual(
        Box((0.008, 0.122, 0.11)),
        origin=Origin(xyz=(0.056, 0.0, 0.067)),
        material=smoke,
        name="front_wall",
    )
    hopper.visual(
        Box((0.008, 0.122, 0.11)),
        origin=Origin(xyz=(-0.056, 0.0, 0.067)),
        material=smoke,
        name="rear_wall",
    )
    hopper.visual(
        Box((0.120, 0.008, 0.11)),
        origin=Origin(xyz=(0.0, 0.053, 0.067)),
        material=smoke,
        name="right_wall",
    )
    hopper.visual(
        Box((0.120, 0.008, 0.11)),
        origin=Origin(xyz=(0.0, -0.053, 0.067)),
        material=smoke,
        name="left_wall",
    )
    hopper.inertial = Inertial.from_geometry(
        Box((0.13, 0.12, 0.13)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.118, 0.122, 0.006)),
        origin=Origin(xyz=(0.059, 0.0, 0.0038)),
        material=smoke,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.112),
        origin=Origin(xyz=(0.005, 0.0, 0.0005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.018, 0.040, 0.012)),
        origin=Origin(xyz=(0.112, 0.0, 0.010)),
        material=black,
        name="finger_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.13, 0.13, 0.014)),
        mass=0.08,
        origin=Origin(xyz=(0.062, 0.0, 0.007)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.031, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=tray_steel,
        name="flange",
    )
    portafilter.visual(
        Box((0.014, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.020, 0.002)),
        material=tray_steel,
        name="lug_0",
    )
    portafilter.visual(
        Box((0.014, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, 0.002)),
        material=tray_steel,
        name="lug_1",
    )
    portafilter.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=tray_steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.018, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=tray_steel,
        name="spout_bridge",
    )
    portafilter.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(-0.004, -0.004, -0.068)),
        material=tray_steel,
        name="spout_0",
    )
    portafilter.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.004, 0.004, -0.068)),
        material=tray_steel,
        name="spout_1",
    )
    portafilter.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.040, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="handle_neck",
    )
    portafilter.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(0.115, 0.0, -0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="handle_grip",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.20, 0.08, 0.09)),
        mass=0.45,
        origin=Origin(xyz=(0.09, 0.0, -0.028)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=dark,
        name="pivot_collar",
    )
    wand_geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.040),
            (0.015, -0.004, -0.062),
            (0.045, -0.010, -0.132),
            (0.085, -0.018, -0.202),
        ],
        radius=0.0045,
        samples_per_segment=16,
        radial_segments=18,
    )
    steam_wand.visual(
        mesh_from_geometry(wand_geom, "steam_wand"),
        material=tray_steel,
        name="wand_tube",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.09, 0.05, 0.22)),
        mass=0.15,
        origin=Origin(xyz=(0.04, -0.01, -0.10)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.130, 0.200, 0.006)),
        origin=Origin(xyz=(-0.055, 0.0, 0.003)),
        material=dark,
        name="tray_bottom",
    )
    drip_tray.visual(
        Box((0.116, 0.008, 0.024)),
        origin=Origin(xyz=(-0.053, 0.096, 0.015)),
        material=dark,
        name="right_wall",
    )
    drip_tray.visual(
        Box((0.116, 0.008, 0.024)),
        origin=Origin(xyz=(-0.053, -0.096, 0.015)),
        material=dark,
        name="left_wall",
    )
    drip_tray.visual(
        Box((0.012, 0.184, 0.024)),
        origin=Origin(xyz=(-0.114, 0.0, 0.015)),
        material=dark,
        name="rear_wall",
    )
    drip_tray.visual(
        Box((0.024, 0.218, 0.040)),
        origin=Origin(xyz=(0.011, 0.0, 0.020)),
        material=brushed,
        name="front_face",
    )
    tray_grate = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.172, 0.192),
            0.003,
            slot_size=(0.020, 0.005),
            pitch=(0.028, 0.015),
            frame=0.010,
            corner_radius=0.004,
            stagger=True,
        ),
        "drip_tray_grate",
    )
    drip_tray.visual(
        tray_grate,
        origin=Origin(xyz=(-0.055, 0.0, 0.0255)),
        material=tray_steel,
        name="grate",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.05)),
        mass=0.70,
        origin=Origin(xyz=(-0.060, 0.0, 0.020)),
    )

    model.articulation(
        "body_to_hopper",
        ArticulationType.FIXED,
        parent=body,
        child=hopper,
        origin=Origin(xyz=(0.01, 0.075, 0.42)),
    )
    model.articulation(
        "hopper_to_lid",
        ArticulationType.REVOLUTE,
        parent=hopper,
        child=lid,
        origin=Origin(xyz=(-0.056, 0.0, 0.122)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.225, 0.0, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.80,
            upper=0.35,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.13, 0.171, 0.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.2,
            lower=-1.30,
            upper=0.85,
        ),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.15, 0.0, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.16,
            lower=0.0,
            upper=0.065,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lid = object_model.get_part("lid")
    hopper = object_model.get_part("hopper")
    steam_wand = object_model.get_part("steam_wand")
    portafilter = object_model.get_part("portafilter")
    drip_tray = object_model.get_part("drip_tray")
    lid_hinge = object_model.get_articulation("hopper_to_lid")
    wand_pivot = object_model.get_articulation("body_to_steam_wand")
    filter_joint = object_model.get_articulation("body_to_portafilter")
    tray_slide = object_model.get_articulation("body_to_drip_tray")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            hopper,
            axis="z",
            min_gap=0.0005,
            max_gap=0.010,
            positive_elem="lid_panel",
            name="lid closes over the hopper opening",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    closed_lid_center = _center_from_aabb(closed_lid_aabb)
    open_lid_center = _center_from_aabb(open_lid_aabb)
    ctx.check(
        "lid opens upward",
        closed_lid_center is not None
        and open_lid_center is not None
        and open_lid_center[2] > closed_lid_center[2] + 0.03,
        details=f"closed={closed_lid_center}, open={open_lid_center}",
    )

    tray_rest = ctx.part_world_position(drip_tray)
    ctx.expect_within(
        drip_tray,
        "body",
        axes="yz",
        margin=0.02,
        name="drip tray stays centered in the lower bay",
    )
    with ctx.pose({tray_slide: 0.065}):
        tray_extended = ctx.part_world_position(drip_tray)
        ctx.expect_within(
            drip_tray,
            "body",
            axes="yz",
            margin=0.02,
            name="extended drip tray stays aligned with the body guides",
        )
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None and tray_extended is not None and tray_extended[0] > tray_rest[0] + 0.05,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    handle_rest = _center_from_aabb(ctx.part_element_world_aabb(portafilter, elem="handle_grip"))
    with ctx.pose({filter_joint: -0.70}):
        handle_unlocked = _center_from_aabb(ctx.part_element_world_aabb(portafilter, elem="handle_grip"))
    ctx.check(
        "portafilter handle swings around the brew axis",
        handle_rest is not None
        and handle_unlocked is not None
        and handle_unlocked[1] < handle_rest[1] - 0.05,
        details=f"rest={handle_rest}, unlocked={handle_unlocked}",
    )

    wand_rest = _center_from_aabb(ctx.part_world_aabb(steam_wand))
    with ctx.pose({wand_pivot: 0.80}):
        wand_swung = _center_from_aabb(ctx.part_world_aabb(steam_wand))
    ctx.check(
        "steam wand rotates outward on its side pivot",
        wand_rest is not None
        and wand_swung is not None
        and wand_swung[1] > wand_rest[1] + 0.025,
        details=f"rest={wand_rest}, swung={wand_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
