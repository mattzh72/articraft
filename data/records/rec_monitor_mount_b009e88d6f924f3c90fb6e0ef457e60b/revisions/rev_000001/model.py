from __future__ import annotations

import math

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
    model = ArticulatedObject(name="wall_monitor_arm_no_screen")

    graphite = model.material("satin_graphite", rgba=(0.07, 0.075, 0.08, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    cover = model.material("soft_cable_cover", rgba=(0.025, 0.027, 0.030, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    screw_dark = model.material("black_fasteners", rgba=(0.005, 0.005, 0.006, 1.0))

    shoulder_x = 0.075
    primary_len = 0.320
    secondary_len = 0.520
    tilt_x = 0.120

    wall = model.part("wall_plate")
    wall.visual(
        Box((0.070, 0.280, 0.500)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=graphite,
        name="wall_slab",
    )
    wall.visual(
        Box((0.050, 0.180, 0.320)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=dark,
        name="raised_back",
    )
    wall.visual(
        Box((0.045, 0.160, 0.220)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=dark,
        name="standoff_block",
    )
    for y in (-0.082, 0.082):
        for z in (-0.170, 0.170):
            wall.visual(
                Cylinder(radius=0.013, length=0.008),
                origin=Origin(xyz=(-0.003, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"wall_screw_{'neg' if y < 0 else 'pos'}_{'low' if z < 0 else 'high'}",
            )
            wall.visual(
                Cylinder(radius=0.006, length=0.009),
                origin=Origin(xyz=(0.002, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=screw_dark,
                name=f"wall_screw_socket_{'neg' if y < 0 else 'pos'}_{'low' if z < 0 else 'high'}",
            )
    wall.visual(
        Box((0.155, 0.150, 0.028)),
        origin=Origin(xyz=(shoulder_x, 0.0, 0.117)),
        material=dark,
        name="shoulder_top_lug",
    )
    wall.visual(
        Box((0.155, 0.150, 0.028)),
        origin=Origin(xyz=(shoulder_x, 0.0, -0.117)),
        material=dark,
        name="shoulder_lower_lug",
    )
    wall.visual(
        Cylinder(radius=0.078, length=0.028),
        origin=Origin(xyz=(shoulder_x, 0.0, 0.117)),
        material=graphite,
        name="shoulder_top_boss",
    )
    wall.visual(
        Cylinder(radius=0.078, length=0.028),
        origin=Origin(xyz=(shoulder_x, 0.0, -0.117)),
        material=graphite,
        name="shoulder_lower_boss",
    )

    primary = model.part("primary_link")
    primary.visual(
        Cylinder(radius=0.052, length=0.206),
        material=graphite,
        name="shoulder_hub",
    )
    primary.visual(
        Box((0.220, 0.060, 0.050)),
        origin=Origin(xyz=(0.155, 0.0, -0.020)),
        material=graphite,
        name="primary_beam",
    )
    primary.visual(
        Box((0.185, 0.036, 0.018)),
        origin=Origin(xyz=(0.160, 0.0, 0.014)),
        material=dark,
        name="primary_trim_rib",
    )
    primary.visual(
        Box((0.026, 0.056, 0.200)),
        origin=Origin(xyz=(primary_len - 0.073, 0.0, 0.0)),
        material=graphite,
        name="elbow_rear_web",
    )
    primary.visual(
        Box((0.080, 0.130, 0.026)),
        origin=Origin(xyz=(primary_len - 0.035, 0.0, 0.101)),
        material=dark,
        name="elbow_top_lug",
    )
    primary.visual(
        Box((0.080, 0.130, 0.026)),
        origin=Origin(xyz=(primary_len - 0.035, 0.0, -0.101)),
        material=dark,
        name="elbow_lower_lug",
    )
    primary.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(primary_len, 0.0, 0.101)),
        material=graphite,
        name="elbow_top_boss",
    )
    primary.visual(
        Cylinder(radius=0.060, length=0.026),
        origin=Origin(xyz=(primary_len, 0.0, -0.101)),
        material=graphite,
        name="elbow_lower_boss",
    )

    secondary = model.part("secondary_link")
    secondary.visual(
        Cylinder(radius=0.048, length=0.176),
        material=graphite,
        name="elbow_hub",
    )
    secondary.visual(
        Box((0.430, 0.052, 0.045)),
        origin=Origin(xyz=(0.255, 0.0, 0.040)),
        material=graphite,
        name="secondary_beam",
    )
    secondary.visual(
        Box((0.350, 0.034, 0.014)),
        origin=Origin(xyz=(0.275, 0.0, 0.013)),
        material=dark,
        name="lower_return_flange",
    )
    secondary.visual(
        Box((0.026, 0.046, 0.170)),
        origin=Origin(xyz=(secondary_len - 0.051, 0.0, 0.0)),
        material=graphite,
        name="wrist_rear_web",
    )
    secondary.visual(
        Box((0.095, 0.110, 0.024)),
        origin=Origin(xyz=(secondary_len - 0.025, 0.0, 0.080)),
        material=dark,
        name="wrist_top_lug",
    )
    secondary.visual(
        Box((0.095, 0.110, 0.024)),
        origin=Origin(xyz=(secondary_len - 0.025, 0.0, -0.080)),
        material=dark,
        name="wrist_lower_lug",
    )
    secondary.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(secondary_len, 0.0, 0.080)),
        material=graphite,
        name="wrist_top_boss",
    )
    secondary.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(secondary_len, 0.0, -0.080)),
        material=graphite,
        name="wrist_lower_boss",
    )

    cable_spine = model.part("cable_spine")
    cable_spine.visual(
        Box((0.330, 0.032, 0.021)),
        origin=Origin(xyz=(0.265, 0.0, 0.073)),
        material=cover,
        name="cover_shell",
    )
    cable_spine.visual(
        Box((0.030, 0.034, 0.018)),
        origin=Origin(xyz=(0.095, 0.0, 0.071)),
        material=cover,
        name="cover_entry_lip",
    )
    cable_spine.visual(
        Box((0.030, 0.034, 0.018)),
        origin=Origin(xyz=(0.435, 0.0, 0.071)),
        material=cover,
        name="cover_exit_lip",
    )

    pan = model.part("pan_swivel")
    pan.visual(
        Cylinder(radius=0.038, length=0.136),
        material=graphite,
        name="pan_hub",
    )
    pan.visual(
        Box((0.075, 0.042, 0.040)),
        origin=Origin(xyz=(0.064, 0.0, 0.0)),
        material=graphite,
        name="pan_neck",
    )
    pan.visual(
        Box((0.030, 0.120, 0.036)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=dark,
        name="tilt_bridge",
    )
    pan.visual(
        Box((0.064, 0.018, 0.074)),
        origin=Origin(xyz=(tilt_x, 0.057, 0.0)),
        material=graphite,
        name="tilt_outer_cheek",
    )
    pan.visual(
        Box((0.064, 0.018, 0.074)),
        origin=Origin(xyz=(tilt_x, -0.057, 0.0)),
        material=graphite,
        name="tilt_inner_cheek",
    )

    cradle = model.part("tilt_cradle")
    cradle.visual(
        Cylinder(radius=0.023, length=0.096),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_barrel",
    )
    cradle.visual(
        Box((0.074, 0.046, 0.036)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=graphite,
        name="cradle_neck",
    )
    cradle.visual(
        Box((0.030, 0.090, 0.070)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=dark,
        name="cradle_saddle",
    )

    vesa = model.part("vesa_plate")
    vesa.visual(
        Box((0.018, 0.160, 0.160)),
        material=graphite,
        name="mount_plate",
    )
    vesa.visual(
        Box((0.010, 0.105, 0.105)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=dark,
        name="front_pressed_pad",
    )
    for y in (-0.050, 0.050):
        for z in (-0.050, 0.050):
            vesa.visual(
                Cylinder(radius=0.012, length=0.010),
                origin=Origin(xyz=(0.019, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"vesa_boss_{'neg' if y < 0 else 'pos'}_{'low' if z < 0 else 'high'}",
            )
            vesa.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(xyz=(0.024, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=screw_dark,
                name=f"vesa_hole_{'neg' if y < 0 else 'pos'}_{'low' if z < 0 else 'high'}",
            )

    model.articulation(
        "wall_to_primary",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=primary,
        origin=Origin(xyz=(shoulder_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.2, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(primary_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.3, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "secondary_to_cable",
        ArticulationType.FIXED,
        parent=secondary,
        child=cable_spine,
    )
    model.articulation(
        "secondary_to_pan",
        ArticulationType.REVOLUTE,
        parent=secondary,
        child=pan,
        origin=Origin(xyz=(secondary_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.6, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=cradle,
        origin=Origin(xyz=(tilt_x, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.1, lower=-0.42, upper=0.38),
    )
    model.articulation(
        "cradle_to_vesa",
        ArticulationType.FIXED,
        parent=cradle,
        child=vesa,
        origin=Origin(xyz=(0.106, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_plate")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    cable = object_model.get_part("cable_spine")
    pan = object_model.get_part("pan_swivel")
    cradle = object_model.get_part("tilt_cradle")
    vesa = object_model.get_part("vesa_plate")

    shoulder = object_model.get_articulation("wall_to_primary")
    elbow = object_model.get_articulation("primary_to_secondary")
    pan_joint = object_model.get_articulation("secondary_to_pan")
    tilt = object_model.get_articulation("pan_to_tilt")

    ctx.check(
        "primary user joints are articulated",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (shoulder, elbow, pan_joint, tilt)
        ),
        details="Shoulder, elbow, pan, and tilt must all be separate revolute joints.",
    )
    ctx.check(
        "vertical pivots use vertical axes",
        shoulder.axis == (0.0, 0.0, 1.0)
        and elbow.axis == (0.0, 0.0, 1.0)
        and pan_joint.axis == (0.0, 0.0, 1.0),
        details=f"axes={shoulder.axis}, {elbow.axis}, {pan_joint.axis}",
    )
    ctx.check(
        "tilt hinge is horizontal",
        tilt.axis == (0.0, -1.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    ctx.expect_gap(
        wall,
        primary,
        axis="z",
        positive_elem="shoulder_top_lug",
        negative_elem="shoulder_hub",
        max_gap=0.001,
        max_penetration=0.0001,
        name="shoulder upper bearing clearance",
    )
    ctx.expect_gap(
        primary,
        wall,
        axis="z",
        positive_elem="shoulder_hub",
        negative_elem="shoulder_lower_lug",
        max_gap=0.001,
        max_penetration=0.0001,
        name="shoulder lower bearing clearance",
    )
    ctx.expect_gap(
        primary,
        secondary,
        axis="z",
        positive_elem="elbow_top_lug",
        negative_elem="elbow_hub",
        max_gap=0.001,
        max_penetration=0.0001,
        name="elbow upper bearing clearance",
    )
    ctx.expect_gap(
        secondary,
        primary,
        axis="z",
        positive_elem="elbow_hub",
        negative_elem="elbow_lower_lug",
        max_gap=0.001,
        max_penetration=0.0001,
        name="elbow lower bearing clearance",
    )
    ctx.expect_gap(
        secondary,
        pan,
        axis="z",
        positive_elem="wrist_top_lug",
        negative_elem="pan_hub",
        max_gap=0.001,
        max_penetration=0.0001,
        name="pan upper bearing clearance",
    )
    ctx.expect_gap(
        pan,
        secondary,
        axis="z",
        positive_elem="pan_hub",
        negative_elem="wrist_lower_lug",
        max_gap=0.001,
        max_penetration=0.0001,
        name="pan lower bearing clearance",
    )
    ctx.expect_gap(
        pan,
        cradle,
        axis="y",
        positive_elem="tilt_outer_cheek",
        negative_elem="tilt_barrel",
        max_gap=0.001,
        max_penetration=0.0001,
        name="tilt outer cheek clearance",
    )
    ctx.expect_gap(
        cradle,
        pan,
        axis="y",
        positive_elem="tilt_barrel",
        negative_elem="tilt_inner_cheek",
        max_gap=0.001,
        max_penetration=0.0001,
        name="tilt inner cheek clearance",
    )
    ctx.expect_gap(
        cable,
        secondary,
        axis="z",
        positive_elem="cover_shell",
        negative_elem="secondary_beam",
        max_gap=0.001,
        max_penetration=0.0001,
        name="cable spine sits on secondary link",
    )
    ctx.expect_gap(
        vesa,
        cradle,
        axis="x",
        positive_elem="mount_plate",
        negative_elem="cradle_neck",
        max_gap=0.001,
        max_penetration=0.0001,
        name="VESA plate is mounted to tilt cradle",
    )

    rest_elbow = ctx.part_world_position(secondary)
    with ctx.pose({shoulder: 1.0}):
        swung_elbow = ctx.part_world_position(secondary)
        ctx.expect_gap(
            primary,
            wall,
            axis="x",
            positive_elem="primary_beam",
            negative_elem="standoff_block",
            min_gap=0.010,
            name="shoulder swing clears wall standoff",
        )
    ctx.check(
        "shoulder swing moves the arm sideways",
        rest_elbow is not None
        and swung_elbow is not None
        and swung_elbow[1] > rest_elbow[1] + 0.10,
        details=f"rest={rest_elbow}, swung={swung_elbow}",
    )

    rest_pan = ctx.part_world_position(pan)
    with ctx.pose({elbow: 1.0}):
        bent_pan = ctx.part_world_position(pan)
        ctx.expect_gap(
            secondary,
            primary,
            axis="z",
            positive_elem="secondary_beam",
            negative_elem="primary_beam",
            min_gap=0.010,
            name="folded links keep vertical clearance",
        )
    ctx.check(
        "elbow swing bends the secondary arm",
        rest_pan is not None and bent_pan is not None and bent_pan[1] > rest_pan[1] + 0.20,
        details=f"rest={rest_pan}, bent={bent_pan}",
    )

    rest_vesa = ctx.part_world_position(vesa)
    with ctx.pose({pan_joint: 0.7}):
        panned_vesa = ctx.part_world_position(vesa)
    ctx.check(
        "pan swivel turns the VESA head",
        rest_vesa is not None
        and panned_vesa is not None
        and panned_vesa[1] > rest_vesa[1] + 0.04,
        details=f"rest={rest_vesa}, panned={panned_vesa}",
    )

    with ctx.pose({tilt: 0.35}):
        tilted_vesa = ctx.part_world_position(vesa)
    ctx.check(
        "tilt hinge pitches the VESA head upward",
        rest_vesa is not None
        and tilted_vesa is not None
        and tilted_vesa[2] > rest_vesa[2] + 0.025,
        details=f"rest={rest_vesa}, tilted={tilted_vesa}",
    )

    return ctx.report()


object_model = build_object_model()
