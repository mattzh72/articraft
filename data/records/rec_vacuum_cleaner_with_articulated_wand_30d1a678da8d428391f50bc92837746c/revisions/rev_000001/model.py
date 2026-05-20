from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_vacuum")

    safety_yellow = model.material("powder_coated_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    dark_housing = model.material("dark_composite", rgba=(0.08, 0.09, 0.09, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    gasket_blue = model.material("service_blue_gasket", rgba=(0.05, 0.25, 0.55, 1.0))
    warning_red = model.material("red_switch", rgba=(0.75, 0.04, 0.02, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.78, 0.42, 0.08)),
        origin=Origin(xyz=(-0.10, 0.0, 0.05)),
        material=dark_housing,
        name="skid_chassis",
    )
    body.visual(
        Cylinder(radius=0.19, length=0.58),
        origin=Origin(xyz=(-0.12, 0.0, 0.27), rpy=(0.0, pi / 2.0, 0.0)),
        material=safety_yellow,
        name="tank_shell",
    )
    body.visual(
        Box((0.54, 0.26, 0.08)),
        origin=Origin(xyz=(-0.12, 0.0, 0.11)),
        material=dark_housing,
        name="tank_cradle",
    )
    body.visual(
        Box((0.48, 0.32, 0.045)),
        origin=Origin(xyz=(-0.20, 0.0, 0.515)),
        material=dark_housing,
        name="service_deck",
    )
    body.visual(
        Box((0.07, 0.26, 0.090)),
        origin=Origin(xyz=(-0.20, 0.0, 0.475)),
        material=dark_housing,
        name="deck_rib",
    )
    body.visual(
        Cylinder(radius=0.075, length=0.12),
        origin=Origin(xyz=(0.23, 0.0, 0.32), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_housing,
        name="intake_collar",
    )
    body.visual(
        Cylinder(radius=0.095, length=0.055),
        origin=Origin(xyz=(-0.32, 0.235, 0.095), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_0",
    )
    body.visual(
        Cylinder(radius=0.095, length=0.055),
        origin=Origin(xyz=(-0.32, -0.235, 0.095), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.56),
        origin=Origin(xyz=(-0.32, 0.0, 0.095), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="wheel_axle",
    )
    body.visual(
        Box((0.62, 0.08, 0.025)),
        origin=Origin(xyz=(-0.08, 0.19, 0.105)),
        material=steel,
        name="side_rail_0",
    )
    body.visual(
        Box((0.62, 0.08, 0.025)),
        origin=Origin(xyz=(-0.08, -0.19, 0.105)),
        material=steel,
        name="side_rail_1",
    )
    body.visual(
        Box((0.16, 0.20, 0.03)),
        origin=Origin(xyz=(-0.43, 0.0, 0.5225)),
        material=steel,
        name="door_hinge_mount",
    )
    body.visual(
        Box((0.10, 0.08, 0.025)),
        origin=Origin(xyz=(0.03, 0.0, 0.5375)),
        material=dark_housing,
        name="switch_bezel",
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((0.37, 0.28, 0.020)),
        origin=Origin(xyz=(0.185, 0.0, 0.010)),
        material=safety_yellow,
        name="door_panel",
    )
    filter_door.visual(
        Box((0.33, 0.235, 0.006)),
        origin=Origin(xyz=(0.19, 0.0, 0.004)),
        material=gasket_blue,
        name="seal_gasket",
    )
    filter_door.visual(
        Cylinder(radius=0.016, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.078, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=warning_red,
        name="rocker_cap",
    )
    power_switch.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rocker_pin",
    )

    for index, y in enumerate((-0.085, 0.085)):
        latch = model.part(f"latch_{index}")
        latch.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=steel,
            name="latch_boss",
        )
        latch.visual(
            Box((0.082, 0.024, 0.012)),
            origin=Origin(xyz=(0.041, 0.0, 0.018)),
            material=dark_housing,
            name="latch_paddle",
        )
        model.articulation(
            f"filter_door_to_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=filter_door,
            child=latch,
            origin=Origin(xyz=(0.25, y, 0.020)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.70, upper=0.70),
        )

    wand_base = model.part("wand_base")
    wand_base.visual(
        Cylinder(radius=0.040, length=0.23),
        origin=Origin(xyz=(0.115, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_housing,
        name="base_tube",
    )
    wand_base.visual(
        Cylinder(radius=0.058, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="base_collar",
    )
    wand_base.visual(
        Cylinder(radius=0.053, length=0.040),
        origin=Origin(xyz=(0.215, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="elbow_collar",
    )
    wand_base.visual(
        Box((0.13, 0.055, 0.045)),
        origin=Origin(xyz=(0.110, 0.0, 0.055)),
        material=dark_housing,
        name="service_grip",
    )

    wand_mid = model.part("wand_mid")
    wand_mid.visual(
        Cylinder(radius=0.033, length=0.46),
        origin=Origin(xyz=(0.230, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="outer_tube",
    )
    wand_mid.visual(
        Cylinder(radius=0.047, length=0.050),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="mid_collar",
    )
    wand_mid.visual(
        Cylinder(radius=0.047, length=0.140),
        origin=Origin(xyz=(0.520, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_housing,
        name="outer_sleeve",
    )
    wand_mid.visual(
        Box((0.11, 0.050, 0.036)),
        origin=Origin(xyz=(0.515, 0.0, 0.050)),
        material=rubber,
        name="lock_clamp",
    )

    wand_tip = model.part("wand_tip")
    wand_tip.visual(
        Cylinder(radius=0.025, length=0.895),
        origin=Origin(xyz=(0.1225, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="inner_tube",
    )
    wand_tip.visual(
        Box((0.065, 0.026, 0.018)),
        origin=Origin(xyz=(0.080, 0.0, 0.032)),
        material=warning_red,
        name="lock_button",
    )
    wand_tip.visual(
        Cylinder(radius=0.040, length=0.060),
        origin=Origin(xyz=(0.595, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tip_end_collar",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Box((0.50, 0.20, 0.075)),
        origin=Origin(xyz=(0.285, 0.0, -0.069)),
        material=dark_housing,
        name="nozzle_housing",
    )
    floor_nozzle.visual(
        Box((0.46, 0.145, 0.014)),
        origin=Origin(xyz=(0.295, 0.0, -0.113)),
        material=steel,
        name="wear_plate",
    )
    floor_nozzle.visual(
        Box((0.038, 0.225, 0.055)),
        origin=Origin(xyz=(0.550, 0.0, -0.052)),
        material=rubber,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.110, 0.018, 0.120)),
        origin=Origin(xyz=(-0.015, 0.092, -0.019)),
        material=steel,
        name="yoke_cheek_0",
    )
    floor_nozzle.visual(
        Box((0.110, 0.018, 0.120)),
        origin=Origin(xyz=(-0.015, -0.092, -0.019)),
        material=steel,
        name="yoke_cheek_1",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.017, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="nozzle_pin",
    )
    floor_nozzle.visual(
        Box((0.40, 0.030, 0.018)),
        origin=Origin(xyz=(0.295, 0.0, -0.119)),
        material=rubber,
        name="brush_strip",
    )

    model.articulation(
        "body_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=filter_door,
        origin=Origin(xyz=(-0.42, 0.0, 0.5375)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(0.03, 0.0, 0.5500)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-0.25, upper=0.25),
    )
    model.articulation(
        "body_to_wand_base",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand_base,
        origin=Origin(xyz=(0.29, 0.0, 0.32)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.6, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "wand_base_to_wand_mid",
        ArticulationType.REVOLUTE,
        parent=wand_base,
        child=wand_mid,
        origin=Origin(xyz=(0.24, 0.0, 0.0), rpy=(0.0, 0.16, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.25, upper=0.55),
    )
    model.articulation(
        "wand_mid_to_wand_tip",
        ArticulationType.PRISMATIC,
        parent=wand_mid,
        child=wand_tip,
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.22),
    )
    model.articulation(
        "wand_tip_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=wand_tip,
        child=floor_nozzle,
        origin=Origin(xyz=(0.625, 0.0, 0.0), rpy=(0.0, -0.16, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.5, lower=-0.55, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    filter_door = object_model.get_part("filter_door")
    wand_base = object_model.get_part("wand_base")
    wand_mid = object_model.get_part("wand_mid")
    wand_tip = object_model.get_part("wand_tip")
    floor_nozzle = object_model.get_part("floor_nozzle")

    door_hinge = object_model.get_articulation("body_to_filter_door")
    wand_yaw = object_model.get_articulation("body_to_wand_base")
    wand_elbow = object_model.get_articulation("wand_base_to_wand_mid")
    wand_slide = object_model.get_articulation("wand_mid_to_wand_tip")
    nozzle_pitch = object_model.get_articulation("wand_tip_to_floor_nozzle")

    ctx.allow_overlap(
        wand_mid,
        wand_tip,
        elem_a="outer_tube",
        elem_b="inner_tube",
        reason="The inner wand is intentionally represented as a retained telescoping member inside the outer tube proxy.",
    )
    ctx.allow_overlap(
        wand_mid,
        wand_tip,
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        reason="The service collar captures the sliding wand tip with deliberate retained insertion.",
    )
    ctx.allow_overlap(
        wand_tip,
        floor_nozzle,
        elem_a="tip_end_collar",
        elem_b="nozzle_pin",
        reason="The nozzle pitch pin is intentionally captured through the wand end collar.",
    )

    ctx.expect_gap(
        wand_base,
        body,
        axis="x",
        max_gap=0.002,
        max_penetration=0.000001,
        positive_elem="base_collar",
        negative_elem="intake_collar",
        name="wand collar seats on body intake",
    )
    ctx.expect_overlap(
        wand_mid,
        wand_tip,
        axes="x",
        elem_a="outer_sleeve",
        elem_b="inner_tube",
        min_overlap=0.09,
        name="collapsed wand retains sleeve insertion",
    )
    ctx.expect_overlap(
        wand_tip,
        floor_nozzle,
        axes="y",
        elem_a="tip_end_collar",
        elem_b="nozzle_pin",
        min_overlap=0.07,
        name="nozzle pin spans the wand collar",
    )
    ctx.expect_contact(
        filter_door,
        body,
        elem_a="door_panel",
        elem_b="service_deck",
        contact_tol=0.001,
        name="filter door rests on service deck",
    )

    closed_door_aabb = ctx.part_world_aabb(filter_door)
    with ctx.pose({door_hinge: 1.0}):
        open_door_aabb = ctx.part_world_aabb(filter_door)
    ctx.check(
        "filter door opens upward for maintenance",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] > closed_door_aabb[1][2] + 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_tip = ctx.part_world_position(wand_tip)
    with ctx.pose({wand_slide: 0.22}):
        extended_tip = ctx.part_world_position(wand_tip)
        ctx.expect_overlap(
            wand_mid,
            wand_tip,
            axes="x",
            elem_a="outer_sleeve",
            elem_b="inner_tube",
            min_overlap=0.06,
            name="extended wand remains captured",
        )
    ctx.check(
        "wand telescopes forward",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.15,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    with ctx.pose({wand_yaw: 0.45, wand_elbow: 0.35, nozzle_pitch: -0.35}):
        ctx.expect_overlap(
            wand_tip,
            floor_nozzle,
            axes="y",
            elem_a="tip_end_collar",
            elem_b="nozzle_pin",
            min_overlap=0.05,
            name="nozzle pivot stays pinned while articulated",
        )

    return ctx.report()


object_model = build_object_model()
