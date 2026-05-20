from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_remote_station")

    armor = model.material("matte_olive_armor", rgba=(0.25, 0.28, 0.22, 1.0))
    dark = model.material("flat_black", rgba=(0.02, 0.02, 0.018, 1.0))
    gunmetal = model.material("parked_gunmetal", rgba=(0.09, 0.095, 0.09, 1.0))
    glass = model.material("coated_optic_glass", rgba=(0.05, 0.12, 0.17, 0.85))
    bolt = model.material("worn_bolt_heads", rgba=(0.12, 0.12, 0.105, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.34, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark,
        name="floor_disk",
    )
    base.visual(
        Cylinder(radius=0.28, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=armor,
        name="stationary_bearing",
    )
    for index, (x, y) in enumerate(
        ((0.24, 0.0), (-0.24, 0.0), (0.0, 0.24), (0.0, -0.24))
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.074)),
            material=bolt,
            name=f"bolt_{index}",
        )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.255, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=armor,
        name="rotating_drum",
    )
    pedestal.visual(
        Cylinder(radius=0.305, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark,
        name="yaw_bearing_ring",
    )
    pedestal.visual(
        Box((0.13, 0.040, 0.180)),
        origin=Origin(xyz=(0.0, 0.180, 0.260)),
        material=armor,
        name="yoke_cheek_0",
    )
    pedestal.visual(
        Box((0.13, 0.040, 0.180)),
        origin=Origin(xyz=(0.0, -0.180, 0.260)),
        material=armor,
        name="yoke_cheek_1",
    )
    pedestal.visual(
        Box((0.18, 0.300, 0.045)),
        origin=Origin(xyz=(-0.035, 0.0, 0.185)),
        material=armor,
        name="yoke_root_bridge",
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Cylinder(radius=0.035, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="pitch_trunnion",
    )
    tilt_frame.visual(
        Box((0.46, 0.150, 0.060)),
        origin=Origin(xyz=(0.145, 0.0, 0.000)),
        material=armor,
        name="cradle_beam",
    )
    tilt_frame.visual(
        Box((0.325, 0.110, 0.110)),
        origin=Origin(xyz=(0.250, 0.0, 0.083)),
        material=gunmetal,
        name="receiver",
    )
    tilt_frame.visual(
        Box((0.180, 0.090, 0.120)),
        origin=Origin(xyz=(0.245, -0.100, 0.062)),
        material=dark,
        name="ammo_box",
    )
    tilt_frame.visual(
        Cylinder(radius=0.020, length=0.705),
        origin=Origin(xyz=(0.765, 0.0, 0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel",
    )
    tilt_frame.visual(
        Cylinder(radius=0.042, length=0.270),
        origin=Origin(xyz=(0.535, 0.0, 0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="barrel_shroud",
    )
    tilt_frame.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(1.140, 0.0, 0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="flash_hider",
    )
    tilt_frame.visual(
        Box((0.165, 0.080, 0.075)),
        origin=Origin(xyz=(0.185, 0.050, 0.170)),
        material=armor,
        name="optic_mast",
    )
    tilt_frame.visual(
        Box((0.240, 0.140, 0.120)),
        origin=Origin(xyz=(0.200, 0.090, 0.245)),
        material=armor,
        name="optics_block",
    )
    tilt_frame.visual(
        Cylinder(radius=0.041, length=0.012),
        origin=Origin(xyz=(0.326, 0.090, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="main_lens",
    )
    tilt_frame.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.325, 0.135, 0.210), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="range_lens",
    )
    tilt_frame.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.335, 0.025, 0.290)),
        material=dark,
        name="hinge_ear_0",
    )
    tilt_frame.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.335, 0.155, 0.290)),
        material=dark,
        name="hinge_ear_1",
    )

    shield_flap = model.part("shield_flap")
    shield_flap.visual(
        Cylinder(radius=0.009, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    shield_flap.visual(
        Box((0.014, 0.130, 0.130)),
        origin=Origin(xyz=(0.015, 0.0, -0.065)),
        material=armor,
        name="armor_plate",
    )
    shield_flap.visual(
        Box((0.009, 0.105, 0.020)),
        origin=Origin(xyz=(0.025, 0.0, -0.062)),
        material=dark,
        name="pressed_rib",
    )

    model.articulation(
        "base_to_pedestal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pedestal_to_tilt_frame",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=tilt_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.9, lower=-0.35, upper=0.75),
    )
    model.articulation(
        "tilt_frame_to_shield_flap",
        ArticulationType.REVOLUTE,
        parent=tilt_frame,
        child=shield_flap,
        origin=Origin(xyz=(0.345, 0.090, 0.290)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pedestal = object_model.get_part("pedestal")
    tilt_frame = object_model.get_part("tilt_frame")
    shield_flap = object_model.get_part("shield_flap")
    yaw = object_model.get_articulation("base_to_pedestal")
    pitch = object_model.get_articulation("pedestal_to_tilt_frame")
    flap = object_model.get_articulation("tilt_frame_to_shield_flap")

    def _aabb_center(bounds):
        if bounds is None:
            return None
        return tuple((bounds[0][i] + bounds[1][i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        pedestal,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="rotating pedestal sits on fixed bearing",
    )
    ctx.expect_overlap(
        pedestal,
        base,
        axes="xy",
        min_overlap=0.20,
        name="round pedestal is centered on the base",
    )
    ctx.expect_contact(
        tilt_frame,
        pedestal,
        elem_a="pitch_trunnion",
        elem_b="yoke_cheek_0",
        name="pitch trunnion is supported by one yoke cheek",
    )
    ctx.expect_contact(
        tilt_frame,
        pedestal,
        elem_a="pitch_trunnion",
        elem_b="yoke_cheek_1",
        name="pitch trunnion is supported by the opposite yoke cheek",
    )
    ctx.expect_contact(
        shield_flap,
        tilt_frame,
        elem_a="hinge_barrel",
        elem_b="hinge_ear_0",
        name="shield hinge is captured at one ear",
    )
    ctx.expect_contact(
        shield_flap,
        tilt_frame,
        elem_a="hinge_barrel",
        elem_b="hinge_ear_1",
        name="shield hinge is captured at the opposite ear",
    )

    rest_flash = ctx.part_element_world_aabb(tilt_frame, elem="flash_hider")
    rest_plate = ctx.part_element_world_aabb(shield_flap, elem="armor_plate")
    rest_flash_center = _aabb_center(rest_flash)

    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_flash = ctx.part_element_world_aabb(tilt_frame, elem="flash_hider")
    yawed_flash_center = _aabb_center(yawed_flash)
    ctx.check(
        "yaw joint sweeps the weapon around the vertical axis",
        rest_flash_center is not None
        and yawed_flash_center is not None
        and yawed_flash_center[1] > rest_flash_center[1] + 1.0
        and abs(yawed_flash_center[0]) < 0.08,
        details=f"rest={rest_flash_center}, yawed={yawed_flash_center}",
    )

    with ctx.pose({pitch: 0.75}):
        raised_flash = ctx.part_element_world_aabb(tilt_frame, elem="flash_hider")
    ctx.check(
        "pitch joint elevates the gun barrel",
        rest_flash is not None
        and raised_flash is not None
        and raised_flash[1][2] > rest_flash[1][2] + 0.55,
        details=f"rest={rest_flash}, raised={raised_flash}",
    )

    with ctx.pose({flap: 1.55}):
        folded_plate = ctx.part_element_world_aabb(shield_flap, elem="armor_plate")
    ctx.check(
        "front shield flap folds upward and forward",
        rest_plate is not None
        and folded_plate is not None
        and folded_plate[1][0] > rest_plate[1][0] + 0.08
        and folded_plate[0][2] > rest_plate[0][2] + 0.10,
        details=f"rest={rest_plate}, folded={folded_plate}",
    )

    return ctx.report()


object_model = build_object_model()
