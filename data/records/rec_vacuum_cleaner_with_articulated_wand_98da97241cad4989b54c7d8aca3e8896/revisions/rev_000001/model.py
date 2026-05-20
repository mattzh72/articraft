from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_articulated_vacuum")

    shell = Material("warm_gray_injection_molded_abs", rgba=(0.72, 0.73, 0.70, 1.0))
    dark = Material("dark_graphite_plastic", rgba=(0.08, 0.09, 0.09, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    accent = Material("teal_snap_collar_plastic", rgba=(0.00, 0.42, 0.48, 1.0))
    tube_mat = Material("brushed_aluminum_tube", rgba=(0.78, 0.80, 0.78, 1.0))
    metal = Material("zinc_plated_pivot_pins", rgba=(0.58, 0.60, 0.57, 1.0))
    cup_mat = Material("smoky_translucent_dust_cup", rgba=(0.34, 0.44, 0.46, 0.45))

    y_axis = (math.pi / 2.0, 0.0, 0.0)
    x_axis = (0.0, math.pi / 2.0, 0.0)

    body = model.part("body")
    body.visual(
        mesh_from_geometry(CapsuleGeometry(radius=0.095, length=0.255), "motor_pod"),
        origin=Origin(xyz=(-0.295, 0.0, 0.105), rpy=x_axis),
        material=shell,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.070, length=0.175),
        origin=Origin(xyz=(-0.112, 0.0, -0.070)),
        material=cup_mat,
        name="dust_cup",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(-0.112, 0.0, 0.025)),
        material=dark,
        name="dust_cup_lid",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.060),
        origin=Origin(xyz=(-0.070, 0.0, 0.0), rpy=x_axis),
        material=accent,
        name="outlet_collar",
    )
    for side, y in enumerate((-0.054, 0.054)):
        body.visual(
            Box((0.080, 0.018, 0.086)),
            origin=Origin(xyz=(-0.013, y, 0.0)),
            material=accent,
            name=f"body_fork_cheek_{side}",
        )
    body.visual(
        Cylinder(radius=0.0065, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis),
        material=metal,
        name="body_pivot_pin",
    )
    for side, y in enumerate((-0.074, 0.074)):
        body.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=y_axis),
            material=dark,
            name=f"body_pivot_cap_{side}",
        )
    body.visual(
        Box((0.220, 0.045, 0.045)),
        origin=Origin(xyz=(-0.260, 0.0, 0.250)),
        material=dark,
        name="handle_grip",
    )
    body.visual(
        Box((0.045, 0.040, 0.180)),
        origin=Origin(xyz=(-0.370, 0.0, 0.170)),
        material=dark,
        name="rear_handle_strut",
    )
    body.visual(
        Box((0.040, 0.040, 0.150)),
        origin=Origin(xyz=(-0.155, 0.0, 0.185)),
        material=dark,
        name="front_handle_strut",
    )
    body.visual(
        Box((0.110, 0.006, 0.050)),
        origin=Origin(xyz=(-0.300, -0.097, 0.120)),
        material=dark,
        name="exhaust_grille",
    )
    body.visual(
        Box((0.066, 0.040, 0.008)),
        origin=Origin(xyz=(-0.155, 0.0, 0.265)),
        material=accent,
        name="button_recess_bezel",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.047, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=accent,
        name="button_cap",
    )

    wand_0 = model.part("wand_0")
    wand_0.visual(
        Cylinder(radius=0.031, length=0.076),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis),
        material=accent,
        name="upper_pivot_boss",
    )
    wand_0.visual(
        Cylinder(radius=0.018, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, -0.2515)),
        material=tube_mat,
        name="upper_tube",
    )
    wand_0.visual(
        Cylinder(radius=0.026, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.491)),
        material=accent,
        name="lower_clamp_collar",
    )
    for side, y in enumerate((-0.048, 0.048)):
        wand_0.visual(
            Box((0.058, 0.016, 0.086)),
            origin=Origin(xyz=(0.0, y, -0.550)),
            material=accent,
            name=f"lower_fork_cheek_{side}",
        )
    for side, y in enumerate((-0.024, 0.024)):
        wand_0.visual(
            Box((0.036, 0.050, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.505)),
            material=accent,
            name=f"lower_fork_web_{side}",
        )
    wand_0.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, -0.550), rpy=y_axis),
        material=metal,
        name="lower_pivot_pin",
    )
    for side, y in enumerate((-0.063, 0.063)):
        wand_0.visual(
            Box((0.030, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, y * 0.92, -0.503)),
            material=dark,
            name=f"clamp_snap_tab_{side}",
        )

    wand_1 = model.part("wand_1")
    wand_1.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis),
        material=accent,
        name="upper_pivot_boss",
    )
    wand_1.visual(
        Cylinder(radius=0.016, length=0.410),
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
        material=tube_mat,
        name="lower_tube",
    )
    wand_1.visual(
        Cylinder(radius=0.024, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.415)),
        material=accent,
        name="nozzle_clamp_collar",
    )
    for side, y in enumerate((-0.047, 0.047)):
        wand_1.visual(
            Box((0.056, 0.016, 0.104)),
            origin=Origin(xyz=(0.0, y, -0.480)),
            material=accent,
            name=f"nozzle_fork_cheek_{side}",
        )
    for side, y in enumerate((-0.024, 0.024)):
        wand_1.visual(
            Box((0.034, 0.050, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.428)),
            material=accent,
            name=f"nozzle_fork_web_{side}",
        )
    wand_1.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.480), rpy=y_axis),
        material=metal,
        name="nozzle_pivot_pin",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.034, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_axis),
        material=accent,
        name="nozzle_pivot_boss",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.064)),
        material=accent,
        name="neck_tube",
    )
    floor_nozzle.visual(
        Box((0.260, 0.340, 0.055)),
        origin=Origin(xyz=(0.070, 0.0, -0.095)),
        material=shell,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.028, length=0.344),
        origin=Origin(xyz=(0.202, 0.0, -0.096), rpy=y_axis),
        material=dark,
        name="front_bumper_roll",
    )
    floor_nozzle.visual(
        Box((0.155, 0.245, 0.006)),
        origin=Origin(xyz=(0.083, 0.0, -0.125)),
        material=rubber,
        name="suction_mouth",
    )
    floor_nozzle.visual(
        Box((0.230, 0.285, 0.010)),
        origin=Origin(xyz=(0.054, 0.0, -0.066)),
        material=accent,
        name="snap_on_sole_plate",
    )
    for side, y in enumerate((-0.178, 0.178)):
        floor_nozzle.visual(
            Cylinder(radius=0.024, length=0.026),
            origin=Origin(xyz=(-0.040, y, -0.113), rpy=y_axis),
            material=rubber,
            name=f"side_wheel_{side}",
        )

    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(-0.155, 0.0, 0.269)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.06, lower=0.0, upper=0.004),
    )
    model.articulation(
        "body_to_wand_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand_0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.65, upper=0.85),
    )
    model.articulation(
        "wand_0_to_wand_1",
        ArticulationType.REVOLUTE,
        parent=wand_0,
        child=wand_1,
        origin=Origin(xyz=(0.0, 0.0, -0.550)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.2, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "wand_1_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=wand_1,
        child=floor_nozzle,
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.90, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand_0 = object_model.get_part("wand_0")
    wand_1 = object_model.get_part("wand_1")
    floor_nozzle = object_model.get_part("floor_nozzle")
    power_button = object_model.get_part("power_button")
    body_pitch = object_model.get_articulation("body_to_wand_0")
    elbow_pitch = object_model.get_articulation("wand_0_to_wand_1")
    nozzle_pitch = object_model.get_articulation("wand_1_to_floor_nozzle")
    button_slide = object_model.get_articulation("body_to_power_button")

    ctx.allow_overlap(
        body,
        wand_0,
        elem_a="body_pivot_pin",
        elem_b="upper_pivot_boss",
        reason="The zinc pin is intentionally captured through the molded wand pivot boss.",
    )
    ctx.allow_overlap(
        wand_0,
        wand_1,
        elem_a="lower_pivot_pin",
        elem_b="upper_pivot_boss",
        reason="The lower wand pivot uses a retained pin through the child boss.",
    )
    ctx.allow_overlap(
        wand_1,
        floor_nozzle,
        elem_a="nozzle_pivot_pin",
        elem_b="nozzle_pivot_boss",
        reason="The nozzle pitch joint is a pin-through-boss pivot with local nested geometry.",
    )

    ctx.expect_overlap(
        body,
        wand_0,
        axes="y",
        elem_a="body_pivot_pin",
        elem_b="upper_pivot_boss",
        min_overlap=0.070,
        name="body pivot pin spans wand boss",
    )
    ctx.expect_overlap(
        wand_0,
        wand_1,
        axes="y",
        elem_a="lower_pivot_pin",
        elem_b="upper_pivot_boss",
        min_overlap=0.065,
        name="wand elbow pin spans lower wand boss",
    )
    ctx.expect_overlap(
        wand_1,
        floor_nozzle,
        axes="y",
        elem_a="nozzle_pivot_pin",
        elem_b="nozzle_pivot_boss",
        min_overlap=0.075,
        name="nozzle pin spans nozzle boss",
    )
    ctx.expect_overlap(
        floor_nozzle,
        wand_1,
        axes="y",
        elem_a="nozzle_pivot_boss",
        elem_b="nozzle_pivot_pin",
        min_overlap=0.075,
        name="nozzle boss retained between fork cheeks",
    )
    ctx.expect_gap(
        power_button,
        body,
        axis="z",
        positive_elem="button_cap",
        negative_elem="button_recess_bezel",
        min_gap=0.0,
        max_gap=0.003,
        name="power button sits proud on molded recess",
    )

    rest_nozzle_origin = ctx.part_world_position(floor_nozzle)
    with ctx.pose({body_pitch: 0.55}):
        pitched_nozzle_origin = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "body wand joint sweeps the downstream assembly",
        rest_nozzle_origin is not None
        and pitched_nozzle_origin is not None
        and abs(pitched_nozzle_origin[0] - rest_nozzle_origin[0]) > 0.20,
        details=f"rest={rest_nozzle_origin}, pitched={pitched_nozzle_origin}",
    )

    with ctx.pose({elbow_pitch: 0.55}):
        elbow_nozzle_origin = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "middle wand segment articulates at the collar",
        rest_nozzle_origin is not None
        and elbow_nozzle_origin is not None
        and abs(elbow_nozzle_origin[0] - rest_nozzle_origin[0]) > 0.12,
        details=f"rest={rest_nozzle_origin}, elbow={elbow_nozzle_origin}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_shell")
    with ctx.pose({nozzle_pitch: 0.65}):
        tilted_shell_aabb = ctx.part_element_world_aabb(floor_nozzle, elem="nozzle_shell")
    if rest_shell_aabb is not None and tilted_shell_aabb is not None:
        rest_center_z = (rest_shell_aabb[0][2] + rest_shell_aabb[1][2]) / 2.0
        tilted_center_z = (tilted_shell_aabb[0][2] + tilted_shell_aabb[1][2]) / 2.0
        rest_center_x = (rest_shell_aabb[0][0] + rest_shell_aabb[1][0]) / 2.0
        tilted_center_x = (tilted_shell_aabb[0][0] + tilted_shell_aabb[1][0]) / 2.0
    else:
        rest_center_z = tilted_center_z = rest_center_x = tilted_center_x = None
    ctx.check(
        "floor nozzle pitches around its neck pivot",
        rest_center_z is not None
        and tilted_center_z is not None
        and rest_center_x is not None
        and tilted_center_x is not None
        and abs(tilted_center_z - rest_center_z) > 0.015
        and abs(tilted_center_x - rest_center_x) > 0.040,
        details=f"rest_aabb={rest_shell_aabb}, tilted_aabb={tilted_shell_aabb}",
    )

    rest_button = ctx.part_world_position(power_button)
    with ctx.pose({button_slide: 0.004}):
        pressed_button = ctx.part_world_position(power_button)
    ctx.check(
        "power button depresses into the handle recess",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[2] < rest_button[2] - 0.003,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
