from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

BODY_DEPTH = 0.42
BODY_WIDTH = 0.32
BODY_HEIGHT = 0.60
TRAY_TRAVEL = 0.09
PORTAFILTER_SWING = math.radians(28.0)
WAND_SWING = math.radians(42.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_group_espresso_machine")

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.82, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.18, 0.19, 0.21, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - 0.06)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((0.26, BODY_WIDTH, 0.48)),
        origin=Origin(xyz=(-0.08, 0.0, 0.24)),
        material=stainless,
        name="rear_shell",
    )
    body.visual(
        Box((0.16, 0.04, 0.36)),
        origin=Origin(xyz=(0.11, -0.14, 0.18)),
        material=stainless,
        name="left_column",
    )
    body.visual(
        Box((0.16, 0.04, 0.36)),
        origin=Origin(xyz=(0.11, 0.14, 0.18)),
        material=stainless,
        name="right_column",
    )
    body.visual(
        Box((0.08, BODY_WIDTH, 0.02)),
        origin=Origin(xyz=(0.17, 0.0, 0.01)),
        material=dark_panel,
        name="base_plinth",
    )
    body.visual(
        Box((0.18, 0.26, 0.02)),
        origin=Origin(xyz=(0.10, 0.0, 0.09)),
        material=dark_panel,
        name="drain_deck",
    )
    body.visual(
        Box((0.04, 0.28, 0.24)),
        origin=Origin(xyz=(0.19, 0.0, 0.38)),
        material=dark_panel,
        name="front_fascia",
    )
    body.visual(
        Box((0.065, 0.11, 0.055)),
        origin=Origin(xyz=(0.205, 0.0, 0.39)),
        material=dark_panel,
        name="group_block",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.045),
        origin=Origin(
            xyz=(0.245, 0.0, 0.375),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black_plastic,
        name="group_face",
    )
    body.visual(
        Box((0.028, 0.055, 0.033)),
        origin=Origin(xyz=(0.232, 0.0, 0.3485)),
        material=black_plastic,
        name="group_mount",
    )
    body.visual(
        Box((0.10, 0.24, 0.01)),
        origin=Origin(xyz=(0.155, 0.0, 0.455)),
        material=black_plastic,
        name="control_panel",
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.24, 0.24, 0.004)),
        origin=Origin(xyz=(0.12, 0.0, 0.002)),
        material=black_plastic,
        name="tray_floor",
    )
    drip_tray.visual(
        Box((0.24, 0.006, 0.028)),
        origin=Origin(xyz=(0.12, -0.117, 0.014)),
        material=black_plastic,
        name="tray_side_0",
    )
    drip_tray.visual(
        Box((0.24, 0.006, 0.028)),
        origin=Origin(xyz=(0.12, 0.117, 0.014)),
        material=black_plastic,
        name="tray_side_1",
    )
    drip_tray.visual(
        Box((0.012, 0.24, 0.03)),
        origin=Origin(xyz=(0.006, 0.0, 0.015)),
        material=black_plastic,
        name="tray_rear_lip",
    )
    drip_tray.visual(
        Box((0.012, 0.24, 0.03)),
        origin=Origin(xyz=(0.234, 0.0, 0.015)),
        material=black_plastic,
        name="tray_front_lip",
    )
    for index, offset_y in enumerate((-0.072, -0.036, 0.0, 0.036, 0.072)):
        drip_tray.visual(
            Box((0.216, 0.014, 0.006)),
            origin=Origin(xyz=(0.12, offset_y, 0.029)),
            material=stainless,
            name=f"grate_bar_{index}",
        )

    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.05, 0.0, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.20,
            lower=0.0,
            upper=TRAY_TRAVEL,
        ),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.032, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=black_plastic,
        name="basket",
    )
    portafilter.visual(
        Box((0.022, 0.028, 0.016)),
        origin=Origin(xyz=(0.006, 0.0, -0.054)),
        material=black_plastic,
        name="spout_block",
    )
    portafilter.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.002, -0.008, -0.067)),
        material=black_plastic,
        name="spout_0",
    )
    portafilter.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.002, 0.008, -0.067)),
        material=black_plastic,
        name="spout_1",
    )
    portafilter.visual(
        Box((0.07, 0.022, 0.018)),
        origin=Origin(xyz=(0.043, 0.0, -0.042)),
        material=black_plastic,
        name="handle_stem",
    )
    portafilter.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(
            xyz=(0.145, 0.0, -0.048),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=knob_black,
        name="grip",
    )

    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.245, 0.0, 0.332)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-PORTAFILTER_SWING,
            upper=PORTAFILTER_SWING,
        ),
    )

    def add_wand(
        part_name: str,
        joint_name: str,
        *,
        y_pos: float,
        z_pos: float,
        main_length: float,
        tip_length: float,
    ) -> None:
        wand = model.part(part_name)
        wand.visual(
            Cylinder(radius=0.008, length=0.014),
            origin=Origin(
                xyz=(0.007, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black_plastic,
            name="pivot",
        )
        wand.visual(
            Sphere(radius=0.008),
            origin=Origin(xyz=(0.016, 0.0, 0.0)),
            material=black_plastic,
            name="elbow",
        )
        wand.visual(
            Cylinder(radius=0.005, length=main_length),
            origin=Origin(xyz=(0.028, 0.0, -main_length / 2.0)),
            material=stainless,
            name="tube",
        )
        wand.visual(
            Cylinder(radius=0.0038, length=tip_length),
            origin=Origin(
                xyz=(0.040, 0.0, -main_length + 0.01),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=stainless,
            name="tip",
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=wand,
            origin=Origin(xyz=(0.21, y_pos, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.2,
                lower=-WAND_SWING,
                upper=WAND_SWING,
            ),
        )

    add_wand(
        "steam_wand",
        "body_to_steam_wand",
        y_pos=-0.115,
        z_pos=0.405,
        main_length=0.19,
        tip_length=0.026,
    )
    add_wand(
        "water_wand",
        "body_to_water_wand",
        y_pos=0.115,
        z_pos=0.398,
        main_length=0.16,
        tip_length=0.022,
    )

    def add_valve_wheel(part_name: str, joint_name: str, *, y_pos: float) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(0.007, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black_plastic,
            name="shaft",
        )
        wheel.visual(
            Cylinder(radius=0.011, length=0.018),
            origin=Origin(
                xyz=(0.016, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=knob_black,
            name="hub",
        )
        wheel.visual(
            Box((0.006, 0.060, 0.010)),
            origin=Origin(xyz=(0.020, 0.0, 0.0)),
            material=knob_black,
            name="spoke_y",
        )
        wheel.visual(
            Box((0.006, 0.010, 0.060)),
            origin=Origin(xyz=(0.020, 0.0, 0.0)),
            material=knob_black,
            name="spoke_z",
        )
        wheel.visual(
            Sphere(radius=0.006),
            origin=Origin(xyz=(0.021, 0.0, 0.032)),
            material=knob_black,
            name="rim_grip",
        )
        wheel.visual(
            Sphere(radius=0.006),
            origin=Origin(xyz=(0.021, 0.0, -0.032)),
            material=knob_black,
            name="rim_grip_1",
        )
        wheel.visual(
            Sphere(radius=0.006),
            origin=Origin(xyz=(0.021, 0.032, 0.0)),
            material=knob_black,
            name="rim_grip_2",
        )
        wheel.visual(
            Sphere(radius=0.006),
            origin=Origin(xyz=(0.021, -0.032, 0.0)),
            material=knob_black,
            name="rim_grip_3",
        )

        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(0.21, y_pos, 0.455)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=8.0,
            ),
        )

    add_valve_wheel("valve_wheel_0", "body_to_valve_wheel_0", y_pos=-0.080)
    add_valve_wheel("valve_wheel_1", "body_to_valve_wheel_1", y_pos=0.080)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drip_tray = object_model.get_part("drip_tray")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    water_wand = object_model.get_part("water_wand")
    valve_wheel_0 = object_model.get_part("valve_wheel_0")
    tray_joint = object_model.get_articulation("body_to_drip_tray")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    water_joint = object_model.get_articulation("body_to_water_wand")
    wheel_joint_0 = object_model.get_articulation("body_to_valve_wheel_0")
    wheel_joint_1 = object_model.get_articulation("body_to_valve_wheel_1")

    ctx.expect_within(
        drip_tray,
        body,
        axes="y",
        inner_elem="tray_floor",
        outer_elem="drain_deck",
        margin=0.02,
        name="drip tray stays centered within the machine width",
    )
    ctx.expect_gap(
        body,
        drip_tray,
        axis="z",
        positive_elem="drain_deck",
        negative_elem="tray_front_lip",
        min_gap=0.01,
        max_gap=0.08,
        name="drip tray sits below the drain deck",
    )
    ctx.expect_overlap(
        drip_tray,
        body,
        axes="x",
        elem_a="tray_floor",
        elem_b="drain_deck",
        min_overlap=0.05,
        name="drip tray remains inserted at rest",
    )

    tray_rest = ctx.part_world_position(drip_tray)
    with ctx.pose({tray_joint: TRAY_TRAVEL}):
        ctx.expect_overlap(
            drip_tray,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="drain_deck",
            min_overlap=0.02,
            name="drip tray retains insertion when extended",
        )
        tray_extended = ctx.part_world_position(drip_tray)
    ctx.check(
        "drip tray slides forward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.06,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    ctx.expect_gap(
        body,
        portafilter,
        axis="z",
        positive_elem="group_mount",
        negative_elem="basket",
        max_gap=0.002,
        max_penetration=0.001,
        name="portafilter sits directly beneath the group head",
    )
    grip_rest = ctx.part_element_world_aabb(portafilter, elem="grip")
    with ctx.pose({portafilter_joint: PORTAFILTER_SWING}):
        grip_swung = ctx.part_element_world_aabb(portafilter, elem="grip")
    ctx.check(
        "portafilter rotates about the vertical brew axis",
        grip_rest is not None
        and grip_swung is not None
        and float(grip_swung[1][1]) > float(grip_rest[1][1]) + 0.08,
        details=f"rest={grip_rest}, swung={grip_swung}",
    )

    steam_tip_rest = ctx.part_element_world_aabb(steam_wand, elem="tip")
    water_tip_rest = ctx.part_element_world_aabb(water_wand, elem="tip")
    with ctx.pose({steam_joint: WAND_SWING, water_joint: WAND_SWING}):
        steam_tip_swung = ctx.part_element_world_aabb(steam_wand, elem="tip")
        water_tip_swung = ctx.part_element_world_aabb(water_wand, elem="tip")
    ctx.check(
        "steam wand pivots outward from its support",
        steam_tip_rest is not None
        and steam_tip_swung is not None
        and float(steam_tip_swung[1][0]) > float(steam_tip_rest[1][0]) + 0.05,
        details=f"rest={steam_tip_rest}, swung={steam_tip_swung}",
    )
    ctx.check(
        "hot water wand pivots outward from its support",
        water_tip_rest is not None
        and water_tip_swung is not None
        and float(water_tip_swung[1][0]) > float(water_tip_rest[1][0]) + 0.04,
        details=f"rest={water_tip_rest}, swung={water_tip_swung}",
    )

    rim_grip_rest = ctx.part_element_world_aabb(valve_wheel_0, elem="rim_grip")
    with ctx.pose({wheel_joint_0: math.pi / 2.0}):
        rim_grip_rotated = ctx.part_element_world_aabb(valve_wheel_0, elem="rim_grip")
    ctx.check(
        "front valve wheel rotates about its front-facing shaft",
        rim_grip_rest is not None
        and rim_grip_rotated is not None
        and abs(float(rim_grip_rotated[0][1]) - float(rim_grip_rest[0][1])) > 0.02,
        details=f"rest={rim_grip_rest}, rotated={rim_grip_rotated}",
    )
    ctx.check(
        "both valve wheels use continuous rotary joints",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"wheel_0={wheel_joint_0.articulation_type}, "
            f"wheel_1={wheel_joint_1.articulation_type}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
