from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_windshield_wiper_assembly")

    paint = model.material("black_powder_coat", rgba=(0.025, 0.027, 0.024, 1.0))
    satin = model.material("satin_black_arms", rgba=(0.010, 0.011, 0.010, 1.0))
    zinc = model.material("zinc_plated_fasteners", rgba=(0.62, 0.63, 0.58, 1.0))
    rubber = model.material("molded_black_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    plastic = model.material("dark_molded_plastic", rgba=(0.055, 0.060, 0.060, 1.0))
    grease = model.material("dark_greased_bushing", rgba=(0.015, 0.015, 0.012, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.46, 0.165, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=paint,
        name="base_plate",
    )
    frame.visual(
        Box((1.34, 0.052, 0.028)),
        origin=Origin(xyz=(0.0, 0.012, 0.038)),
        material=paint,
        name="raised_channel",
    )
    frame.visual(
        Box((1.16, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.044)),
        material=paint,
        name="front_reinforcing_rib",
    )
    frame.visual(
        Box((1.16, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.060, 0.044)),
        material=paint,
        name="rear_reinforcing_rib",
    )
    frame.visual(
        Box((0.30, 0.110, 0.050)),
        origin=Origin(xyz=(0.0, -0.105, 0.053)),
        material=paint,
        name="motor_saddle",
    )
    frame.visual(
        Cylinder(radius=0.084, length=0.046),
        origin=Origin(xyz=(0.0, -0.105, 0.084)),
        material=plastic,
        name="gearbox_cover",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.225),
        origin=Origin(xyz=(0.0, -0.235, 0.056), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="motor_can",
    )
    frame.visual(
        Box((0.082, 0.046, 0.052)),
        origin=Origin(xyz=(0.0, -0.362, 0.056)),
        material=plastic,
        name="sealed_connector",
    )
    frame.visual(
        Box((0.026, 0.080, 0.024)),
        origin=Origin(xyz=(0.0, -0.318, 0.048)),
        material=plastic,
        name="connector_boot",
    )

    pivot_specs = (
        (-0.48, "pivot_flange_0", "pivot_tower_0", "bearing_retainer_0", "pivot_box_rib_0", "tower_side_gusset_0"),
        (0.48, "pivot_flange_1", "pivot_tower_1", "bearing_retainer_1", "pivot_box_rib_1", "tower_side_gusset_1"),
    )
    for x, flange_name, tower_name, retainer_name, rib_name, gusset_name in pivot_specs:
        frame.visual(
            Cylinder(radius=0.075, length=0.014),
            origin=Origin(xyz=(x, 0.012, 0.031)),
            material=paint,
            name=flange_name,
        )
        frame.visual(
            Cylinder(radius=0.048, length=0.063),
            origin=Origin(xyz=(x, 0.012, 0.0685)),
            material=grease,
            name=tower_name,
        )
        frame.visual(
            Cylinder(radius=0.060, length=0.008),
            origin=Origin(xyz=(x, 0.012, 0.1035)),
            material=zinc,
            name=retainer_name,
        )
        frame.visual(
            Box((0.230, 0.036, 0.026)),
            origin=Origin(xyz=(x * 0.82, 0.012, 0.052)),
            material=paint,
            name=rib_name,
        )
        frame.visual(
            Box((0.030, 0.126, 0.032)),
            origin=Origin(xyz=(x, 0.000, 0.050)),
            material=paint,
            name=gusset_name,
        )

    bolt_positions = [
        (-0.66, 0.060),
        (-0.66, -0.060),
        (-0.52, 0.062),
        (-0.44, -0.062),
        (-0.13, -0.062),
        (0.13, -0.062),
        (0.44, -0.062),
        (0.52, 0.062),
        (0.66, 0.060),
        (0.66, -0.060),
        (-0.055, -0.145),
        (0.055, -0.145),
    ]
    for index, (x, y) in enumerate(bolt_positions):
        frame.visual(
            Cylinder(radius=0.012, length=0.007),
            origin=Origin(xyz=(x, y, 0.0275)),
            material=zinc,
            name=f"bolt_head_{index}",
        )

    motor_crank = model.part("motor_crank")
    motor_crank.visual(
        Cylinder(radius=0.050, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=zinc,
        name="crank_disc",
    )
    motor_crank.visual(
        Box((0.126, 0.026, 0.012)),
        origin=Origin(xyz=(0.047, 0.0, 0.008)),
        material=zinc,
        name="offset_crank_arm",
    )
    motor_crank.visual(
        Cylinder(radius=0.014, length=0.038),
        origin=Origin(xyz=(0.090, 0.0, 0.020)),
        material=zinc,
        name="crank_pin",
    )
    motor_crank.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.090, 0.0, 0.041)),
        material=zinc,
        name="crank_pin_nut",
    )

    drive_link = model.part("drive_link")
    drive_link.visual(
        Box((0.510, 0.020, 0.010)),
        origin=Origin(xyz=(-0.285, 0.0, 0.0)),
        material=zinc,
        name="link_bar",
    )
    drive_link.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="eye_motor",
    )
    drive_link.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=grease,
        name="eye_motor_bushing",
    )
    drive_link.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(-0.570, 0.0, 0.0)),
        material=zinc,
        name="eye_spindle",
    )
    drive_link.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(-0.570, 0.0, 0.001)),
        material=grease,
        name="eye_spindle_bushing",
    )
    drive_link.visual(
        Box((0.390, 0.009, 0.017)),
        origin=Origin(xyz=(-0.285, 0.0, 0.010)),
        material=paint,
        name="pressed_center_ridge",
    )

    sync_link = model.part("sync_link")
    sync_link.visual(
        Box((0.900, 0.022, 0.012)),
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        material=zinc,
        name="tie_bar",
    )
    sync_link.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="eye_spindle_0",
    )
    sync_link.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(0.960, 0.0, 0.0)),
        material=zinc,
        name="eye_spindle_1",
    )
    sync_link.visual(
        Box((0.820, 0.010, 0.018)),
        origin=Origin(xyz=(0.480, 0.0, 0.010)),
        material=paint,
        name="tie_bar_rolled_edge",
    )

    def add_wiper_part(name: str) -> object:
        part = model.part(name)
        part.visual(
            Cylinder(radius=0.036, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=zinc,
            name="hub_collar",
        )
        part.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.033)),
            material=zinc,
            name="spindle_nut",
        )
        part.visual(
            Cylinder(radius=0.020, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, 0.047)),
            material=paint,
            name="arm_riser",
        )
        part.visual(
            Box((0.016, 0.410, 0.012)),
            origin=Origin(xyz=(-0.018, 0.230, 0.066)),
            material=satin,
            name="arm_strap_0",
        )
        part.visual(
            Box((0.016, 0.410, 0.012)),
            origin=Origin(xyz=(0.018, 0.230, 0.066)),
            material=satin,
            name="arm_strap_1",
        )
        part.visual(
            Box((0.065, 0.052, 0.016)),
            origin=Origin(xyz=(0.0, 0.030, 0.066)),
            material=satin,
            name="root_yoke",
        )
        part.visual(
            Box((0.070, 0.052, 0.018)),
            origin=Origin(xyz=(0.0, 0.430, 0.064)),
            material=satin,
            name="blade_saddle",
        )
        part.visual(
            Box((0.560, 0.024, 0.014)),
            origin=Origin(xyz=(0.0, 0.485, 0.062)),
            material=satin,
            name="blade_carrier",
        )
        part.visual(
            Box((0.590, 0.018, 0.032)),
            origin=Origin(xyz=(0.0, 0.503, 0.0395)),
            material=rubber,
            name="rubber_blade",
        )
        part.visual(
            Box((0.500, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.466, 0.075)),
            material=zinc,
            name="stainless_flex_strip",
        )
        for clip_index, x in enumerate((-0.220, -0.110, 0.0, 0.110, 0.220)):
            part.visual(
                Box((0.034, 0.040, 0.020)),
                origin=Origin(xyz=(x, 0.475, 0.066)),
                material=satin,
                name=f"blade_clip_{clip_index}",
            )
        part.visual(
            Box((0.030, 0.145, 0.014)),
            origin=Origin(xyz=(0.0, -0.058, 0.011)),
            material=zinc,
            name="rear_bellcrank",
        )
        part.visual(
            Cylinder(radius=0.014, length=0.090),
            origin=Origin(xyz=(0.0, -0.117, 0.049)),
            material=zinc,
            name="rear_pin",
        )
        part.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.0, -0.117, 0.097)),
            material=zinc,
            name="rear_pin_nut",
        )
        return part

    spindle_0 = add_wiper_part("spindle_0")
    spindle_1 = add_wiper_part("spindle_1")

    drive = model.articulation(
        "frame_to_motor_crank",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=motor_crank,
        origin=Origin(xyz=(0.0, -0.105, 0.112)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22.0, velocity=4.0, lower=-1.30, upper=1.30),
        motion_properties=MotionProperties(damping=0.04, friction=0.03),
    )
    model.articulation(
        "motor_crank_to_drive_link",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(0.090, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-0.55, upper=0.55),
        motion_properties=MotionProperties(damping=0.02, friction=0.02),
    )
    model.articulation(
        "frame_to_spindle_0",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=spindle_0,
        origin=Origin(xyz=(-0.480, 0.012, 0.107)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.6, lower=-0.56, upper=0.56),
        motion_properties=MotionProperties(damping=0.08, friction=0.05),
        mimic=Mimic(joint=drive.name, multiplier=0.43, offset=0.0),
    )
    model.articulation(
        "frame_to_spindle_1",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=spindle_1,
        origin=Origin(xyz=(0.480, 0.012, 0.107)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=2.6, lower=-0.56, upper=0.56),
        motion_properties=MotionProperties(damping=0.08, friction=0.05),
        mimic=Mimic(joint=drive.name, multiplier=0.43, offset=0.0),
    )
    model.articulation(
        "spindle_0_to_sync_link",
        ArticulationType.REVOLUTE,
        parent=spindle_0,
        child=sync_link,
        origin=Origin(xyz=(0.0, -0.117, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=4.0, lower=-0.35, upper=0.35),
        motion_properties=MotionProperties(damping=0.03, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    motor_crank = object_model.get_part("motor_crank")
    drive_link = object_model.get_part("drive_link")
    sync_link = object_model.get_part("sync_link")
    spindle_0 = object_model.get_part("spindle_0")
    spindle_1 = object_model.get_part("spindle_1")
    drive = object_model.get_articulation("frame_to_motor_crank")

    ctx.allow_overlap(
        motor_crank,
        drive_link,
        elem_a="crank_pin",
        elem_b="eye_motor_bushing",
        reason="The drive-link eye intentionally wraps the eccentric crank pin with a bushed captured joint.",
    )
    ctx.allow_overlap(
        drive_link,
        motor_crank,
        elem_a="eye_motor",
        elem_b="crank_pin",
        reason="The solid rod-end proxy surrounds the crank pin where a real clevis eye would have a bore.",
    )
    ctx.expect_overlap(
        motor_crank,
        drive_link,
        axes="xy",
        elem_a="crank_pin",
        elem_b="eye_motor_bushing",
        min_overlap=0.020,
        name="drive link eye is centered on crank pin",
    )
    ctx.expect_overlap(
        drive_link,
        motor_crank,
        axes="xy",
        elem_a="eye_motor",
        elem_b="crank_pin",
        min_overlap=0.020,
        name="outer crank eye surrounds crank pin",
    )
    ctx.allow_overlap(
        spindle_0,
        drive_link,
        elem_a="rear_pin",
        elem_b="eye_spindle_bushing",
        reason="The far drive-link eye is captured over the spindle bellcrank pin at the serviceable rod end.",
    )
    ctx.allow_overlap(
        drive_link,
        spindle_0,
        elem_a="eye_spindle",
        elem_b="rear_pin",
        reason="The drive-link rod-end eye is intentionally modeled as a solid bushed eye around the bellcrank pin.",
    )
    ctx.expect_overlap(
        spindle_0,
        drive_link,
        axes="xy",
        elem_a="rear_pin",
        elem_b="eye_spindle_bushing",
        min_overlap=0.020,
        name="drive link reaches spindle bellcrank pin",
    )
    ctx.expect_overlap(
        drive_link,
        spindle_0,
        axes="xy",
        elem_a="eye_spindle",
        elem_b="rear_pin",
        min_overlap=0.020,
        name="outer drive eye surrounds bellcrank pin",
    )
    ctx.allow_overlap(
        spindle_0,
        sync_link,
        elem_a="rear_pin",
        elem_b="eye_spindle_0",
        reason="The synchronizing tie bar is intentionally stacked on the first spindle bellcrank pin.",
    )
    ctx.allow_overlap(
        spindle_1,
        sync_link,
        elem_a="rear_pin",
        elem_b="eye_spindle_1",
        reason="The synchronizing tie bar is intentionally stacked on the second spindle bellcrank pin.",
    )
    ctx.expect_overlap(
        spindle_0,
        sync_link,
        axes="xy",
        elem_a="rear_pin",
        elem_b="eye_spindle_0",
        min_overlap=0.020,
        name="tie bar first eye is on the first spindle pin",
    )
    ctx.expect_overlap(
        spindle_1,
        sync_link,
        axes="xy",
        elem_a="rear_pin",
        elem_b="eye_spindle_1",
        min_overlap=0.020,
        name="tie bar second eye is on the second spindle pin",
    )

    ctx.expect_gap(
        spindle_0,
        frame,
        axis="z",
        positive_elem="hub_collar",
        negative_elem="bearing_retainer_0",
        max_gap=0.002,
        max_penetration=0.001,
        name="first rotating hub seats on bearing retainer",
    )
    ctx.expect_gap(
        spindle_1,
        frame,
        axis="z",
        positive_elem="hub_collar",
        negative_elem="bearing_retainer_1",
        max_gap=0.002,
        max_penetration=0.001,
        name="second rotating hub seats on bearing retainer",
    )
    ctx.expect_overlap(
        spindle_0,
        frame,
        axes="xy",
        elem_a="hub_collar",
        elem_b="pivot_tower_0",
        min_overlap=0.040,
        name="first spindle is centered over its pivot tower",
    )
    ctx.expect_overlap(
        spindle_1,
        frame,
        axes="xy",
        elem_a="hub_collar",
        elem_b="pivot_tower_1",
        min_overlap=0.040,
        name="second spindle is centered over its pivot tower",
    )
    ctx.expect_gap(
        motor_crank,
        frame,
        axis="z",
        positive_elem="crank_disc",
        negative_elem="gearbox_cover",
        max_gap=0.006,
        max_penetration=0.0,
        name="crank disc rides above gearbox cover",
    )

    rest_aabb = ctx.part_element_world_aabb(spindle_0, elem="rubber_blade")
    with ctx.pose({drive: 1.0}):
        swept_aabb = ctx.part_element_world_aabb(spindle_0, elem="rubber_blade")
    if rest_aabb is not None and swept_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
        swept_center_x = (swept_aabb[0][0] + swept_aabb[1][0]) * 0.5
        moved = abs(swept_center_x - rest_center_x) > 0.080
    else:
        moved = False
    ctx.check(
        "motor drive visibly sweeps blade carrier",
        moved,
        details=f"rest_aabb={rest_aabb}, swept_aabb={swept_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
