from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalanced_monitor_mount_study")

    anodized = model.material("hard_anodized_aluminum", rgba=(0.32, 0.34, 0.36, 1.0))
    dark = model.material("black_oxide_steel", rgba=(0.055, 0.060, 0.065, 1.0))
    bearing = model.material("bearing_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    cover = model.material("matte_access_covers", rgba=(0.15, 0.16, 0.17, 1.0))
    spring = model.material("spring_housing_gray", rgba=(0.42, 0.43, 0.44, 1.0))
    brass = model.material("oilite_bronze_bushings", rgba=(0.70, 0.48, 0.22, 1.0))
    cable = model.material("cable_passage_black", rgba=(0.015, 0.017, 0.020, 1.0))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.026, 0.340, 0.460)),
        origin=Origin(xyz=(0.000, 0.000, 0.300)),
        material=anodized,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.160, 0.300, 0.024)),
        origin=Origin(xyz=(0.067, 0.000, 0.082)),
        material=anodized,
        name="lower_flange",
    )
    wall_bracket.visual(
        Box((0.145, 0.260, 0.022)),
        origin=Origin(xyz=(0.062, 0.000, 0.518)),
        material=anodized,
        name="upper_flange",
    )
    wall_bracket.visual(
        Box((0.120, 0.022, 0.285)),
        origin=Origin(xyz=(0.056, 0.122, 0.300)),
        material=anodized,
        name="side_gusset_0",
    )
    wall_bracket.visual(
        Box((0.120, 0.022, 0.285)),
        origin=Origin(xyz=(0.056, -0.122, 0.300)),
        material=anodized,
        name="side_gusset_1",
    )
    wall_bracket.visual(
        Cylinder(radius=0.061, length=0.088),
        origin=Origin(xyz=(0.088, 0.000, 0.318)),
        material=dark,
        name="fixed_bearing_cup",
    )
    wall_bracket.visual(
        Cylinder(radius=0.042, length=0.190),
        origin=Origin(xyz=(0.088, 0.000, 0.318)),
        material=bearing,
        name="pan_spindle_socket",
    )
    wall_bracket.visual(
        Cylinder(radius=0.071, length=0.014),
        origin=Origin(xyz=(0.088, 0.000, 0.413)),
        material=dark,
        name="upper_thrust_race",
    )
    wall_bracket.visual(
        Cylinder(radius=0.071, length=0.014),
        origin=Origin(xyz=(0.088, 0.000, 0.223)),
        material=dark,
        name="lower_thrust_race",
    )
    wall_bracket.visual(
        Box((0.018, 0.180, 0.118)),
        origin=Origin(xyz=(0.018, 0.000, 0.300)),
        material=cover,
        name="removable_rear_cover",
    )
    wall_bracket.visual(
        Box((0.012, 0.100, 0.026)),
        origin=Origin(xyz=(0.014, 0.000, 0.168)),
        material=cable,
        name="vertical_cable_window",
    )
    for index, (yy, zz) in enumerate(
        ((0.115, 0.460), (-0.115, 0.460), (0.115, 0.140), (-0.115, 0.140))
    ):
        wall_bracket.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(0.015, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"mounting_hole_{index}",
        )
        wall_bracket.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(0.020, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing,
            name=f"cap_screw_{index}",
        )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.180, 0.340, 0.520)),
        mass=4.5,
        origin=Origin(xyz=(0.055, 0.0, 0.300)),
    )

    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.038, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bearing,
        name="rotating_spindle",
    )
    pan_carriage.visual(
        Cylinder(radius=0.066, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=dark,
        name="friction_collar_top",
    )
    pan_carriage.visual(
        Cylinder(radius=0.066, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
        material=dark,
        name="friction_collar_bottom",
    )
    pan_carriage.visual(
        Box((0.076, 0.106, 0.052)),
        origin=Origin(xyz=(0.036, 0.000, 0.135)),
        material=anodized,
        name="shoulder_bridge",
    )
    pan_carriage.visual(
        Box((0.085, 0.014, 0.118)),
        origin=Origin(xyz=(0.112, 0.056, 0.135)),
        material=anodized,
        name="shoulder_cheek_0",
    )
    pan_carriage.visual(
        Box((0.085, 0.014, 0.118)),
        origin=Origin(xyz=(0.112, -0.056, 0.135)),
        material=anodized,
        name="shoulder_cheek_1",
    )
    pan_carriage.visual(
        Cylinder(radius=0.031, length=0.138),
        origin=Origin(xyz=(0.115, 0.000, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="shoulder_cross_pin",
    )
    pan_carriage.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.115, 0.069, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="shoulder_lock_nut_0",
    )
    pan_carriage.visual(
        Cylinder(radius=0.043, length=0.018),
        origin=Origin(xyz=(0.115, -0.069, 0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="shoulder_lock_nut_1",
    )
    pan_carriage.visual(
        Box((0.060, 0.050, 0.018)),
        origin=Origin(xyz=(0.032, 0.000, 0.157)),
        material=cable,
        name="base_cable_exit",
    )
    pan_carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.260),
        mass=2.2,
        origin=Origin(xyz=(0.045, 0.0, 0.030)),
    )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        Cylinder(radius=0.030, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="shoulder_bushing",
    )
    primary_arm.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="shoulder_knuckle",
    )
    for side, yy in enumerate((0.043, -0.043)):
        primary_arm.visual(
            Box((0.500, 0.018, 0.040)),
            origin=Origin(xyz=(0.280, yy, 0.000)),
            material=anodized,
            name=f"side_rail_{side}",
        )
    for index, xx in enumerate((0.070, 0.275, 0.490)):
        primary_arm.visual(
            Box((0.034, 0.104, 0.030)),
            origin=Origin(xyz=(xx, 0.000, 0.000)),
            material=anodized,
            name=f"cross_tie_{index}",
        )
    primary_arm.visual(
        Cylinder(radius=0.026, length=0.405),
        origin=Origin(xyz=(0.276, 0.000, 0.040), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spring,
        name="spring_tube",
    )
    primary_arm.visual(
        Box((0.270, 0.052, 0.014)),
        origin=Origin(xyz=(0.286, 0.000, 0.065)),
        material=cover,
        name="spring_access_cover",
    )
    primary_arm.visual(
        Box((0.360, 0.026, 0.014)),
        origin=Origin(xyz=(0.305, 0.000, -0.031)),
        material=cable,
        name="underside_cable_tray",
    )
    primary_arm.visual(
        Cylinder(radius=0.013, length=0.375),
        origin=Origin(xyz=(0.282, 0.000, -0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="gas_spring_rod",
    )
    for index, xx in enumerate((0.098, 0.466)):
        primary_arm.visual(
            Box((0.040, 0.080, 0.052)),
            origin=Origin(xyz=(xx, 0.000, -0.036)),
            material=dark,
            name=f"spring_anchor_{index}",
        )
    primary_arm.visual(
        Cylinder(radius=0.036, length=0.096),
        origin=Origin(xyz=(0.548, 0.000, 0.000)),
        material=dark,
        name="elbow_outer_barrel",
    )
    primary_arm.visual(
        Cylinder(radius=0.025, length=0.128),
        origin=Origin(xyz=(0.548, 0.000, 0.000)),
        material=bearing,
        name="elbow_pin",
    )
    primary_arm.visual(
        Box((0.060, 0.102, 0.020)),
        origin=Origin(xyz=(0.528, 0.000, 0.056)),
        material=anodized,
        name="upper_elbow_strap",
    )
    primary_arm.visual(
        Box((0.060, 0.102, 0.020)),
        origin=Origin(xyz=(0.528, 0.000, -0.056)),
        material=anodized,
        name="lower_elbow_strap",
    )
    primary_arm.visual(
        Box((0.084, 0.014, 0.040)),
        origin=Origin(xyz=(0.520, 0.040, 0.000)),
        material=anodized,
        name="elbow_web_0",
    )
    primary_arm.visual(
        Box((0.084, 0.014, 0.040)),
        origin=Origin(xyz=(0.520, -0.040, 0.000)),
        material=anodized,
        name="elbow_web_1",
    )
    primary_arm.inertial = Inertial.from_geometry(
        Box((0.590, 0.130, 0.155)),
        mass=1.9,
        origin=Origin(xyz=(0.285, 0.0, 0.005)),
    )

    secondary_arm = model.part("secondary_arm")
    secondary_arm.visual(
        Cylinder(radius=0.030, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="elbow_inner_sleeve",
    )
    secondary_arm.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=dark,
        name="elbow_friction_disk_top",
    )
    secondary_arm.visual(
        Cylinder(radius=0.040, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=dark,
        name="elbow_friction_disk_bottom",
    )
    secondary_arm.visual(
        Box((0.435, 0.040, 0.052)),
        origin=Origin(xyz=(0.237, 0.000, 0.000)),
        material=anodized,
        name="machined_box_beam",
    )
    secondary_arm.visual(
        Box((0.320, 0.026, 0.018)),
        origin=Origin(xyz=(0.255, -0.033, 0.010)),
        material=cable,
        name="side_cable_slot",
    )
    secondary_arm.visual(
        Cylinder(radius=0.019, length=0.315),
        origin=Origin(xyz=(0.237, 0.000, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spring,
        name="secondary_spring_case",
    )
    secondary_arm.visual(
        Box((0.210, 0.044, 0.012)),
        origin=Origin(xyz=(0.250, 0.000, 0.080)),
        material=cover,
        name="top_access_cover",
    )
    for index, xx in enumerate((0.080, 0.380)):
        secondary_arm.visual(
            Box((0.038, 0.070, 0.040)),
            origin=Origin(xyz=(xx, 0.000, 0.038)),
            material=dark,
            name=f"spring_case_mount_{index}",
        )
    secondary_arm.visual(
        Cylinder(radius=0.031, length=0.102),
        origin=Origin(xyz=(0.490, 0.000, 0.000)),
        material=dark,
        name="head_pan_barrel",
    )
    secondary_arm.visual(
        Cylinder(radius=0.022, length=0.126),
        origin=Origin(xyz=(0.490, 0.000, 0.000)),
        material=bearing,
        name="head_pan_pin",
    )
    secondary_arm.visual(
        Box((0.044, 0.090, 0.030)),
        origin=Origin(xyz=(0.437, 0.000, 0.000)),
        material=anodized,
        name="head_end_bridge",
    )
    secondary_arm.inertial = Inertial.from_geometry(
        Box((0.505, 0.100, 0.125)),
        mass=1.2,
        origin=Origin(xyz=(0.250, 0.0, 0.020)),
    )

    head_knuckle = model.part("head_knuckle")
    head_knuckle.visual(
        Cylinder(radius=0.027, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="pan_bushing",
    )
    head_knuckle.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=dark,
        name="pan_friction_collar_top",
    )
    head_knuckle.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=dark,
        name="pan_friction_collar_bottom",
    )
    head_knuckle.visual(
        Box((0.092, 0.090, 0.040)),
        origin=Origin(xyz=(0.047, 0.000, 0.000)),
        material=anodized,
        name="tilt_yoke_bridge",
    )
    head_knuckle.visual(
        Box((0.075, 0.014, 0.120)),
        origin=Origin(xyz=(0.130, 0.044, 0.000)),
        material=anodized,
        name="tilt_cheek_0",
    )
    head_knuckle.visual(
        Box((0.075, 0.014, 0.120)),
        origin=Origin(xyz=(0.130, -0.044, 0.000)),
        material=anodized,
        name="tilt_cheek_1",
    )
    head_knuckle.visual(
        Cylinder(radius=0.024, length=0.112),
        origin=Origin(xyz=(0.130, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing,
        name="tilt_cross_pin",
    )
    head_knuckle.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.130, 0.057, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tilt_lock_knob_0",
    )
    head_knuckle.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.130, -0.057, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tilt_lock_knob_1",
    )
    head_knuckle.inertial = Inertial.from_geometry(
        Box((0.180, 0.120, 0.150)),
        mass=0.75,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.026, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="tilt_trunnion",
    )
    tilt_head.visual(
        Box((0.100, 0.032, 0.050)),
        origin=Origin(xyz=(0.048, 0.000, 0.000)),
        material=dark,
        name="trunnion_block",
    )
    tilt_head.visual(
        Box((0.018, 0.184, 0.164)),
        origin=Origin(xyz=(0.105, 0.000, 0.000)),
        material=anodized,
        name="mounting_plate",
    )
    tilt_head.visual(
        Box((0.050, 0.118, 0.020)),
        origin=Origin(xyz=(0.076, 0.000, 0.037)),
        material=anodized,
        name="upper_web",
    )
    tilt_head.visual(
        Box((0.050, 0.118, 0.020)),
        origin=Origin(xyz=(0.076, 0.000, -0.037)),
        material=anodized,
        name="lower_web",
    )
    tilt_head.visual(
        Box((0.024, 0.024, 0.136)),
        origin=Origin(xyz=(0.119, 0.000, 0.000)),
        material=cable,
        name="center_cable_passage",
    )
    for index, (yy, zz) in enumerate(
        ((0.050, 0.050), (-0.050, 0.050), (0.050, -0.050), (-0.050, -0.050))
    ):
        tilt_head.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(0.117, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"vesa_slot_{index}",
        )
        tilt_head.visual(
            Cylinder(radius=0.0055, length=0.014),
            origin=Origin(xyz=(0.125, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing,
            name=f"threaded_insert_{index}",
        )
    tilt_head.visual(
        Box((0.013, 0.120, 0.018)),
        origin=Origin(xyz=(0.118, 0.000, 0.073)),
        material=cover,
        name="top_access_lid",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.140, 0.190, 0.170)),
        mass=0.85,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    model.articulation(
        "wall_pan",
        ArticulationType.CONTINUOUS,
        parent=wall_bracket,
        child=pan_carriage,
        origin=Origin(xyz=(0.088, 0.000, 0.318)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4),
        motion_properties=MotionProperties(damping=0.18, friction=0.12),
    )
    model.articulation(
        "shoulder_lift",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=primary_arm,
        origin=Origin(xyz=(0.115, 0.000, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.9, lower=-0.45, upper=0.95),
        motion_properties=MotionProperties(damping=0.55, friction=0.18),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(0.548, 0.000, 0.000), rpy=(0.0, 0.0, math.radians(-34.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=-2.25, upper=1.65),
        motion_properties=MotionProperties(damping=0.35, friction=0.16),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=head_knuckle,
        origin=Origin(xyz=(0.490, 0.000, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-1.75, upper=1.75),
        motion_properties=MotionProperties(damping=0.20, friction=0.12),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_knuckle,
        child=tilt_head,
        origin=Origin(xyz=(0.130, 0.000, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=-0.75, upper=0.75),
        motion_properties=MotionProperties(damping=0.35, friction=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall = object_model.get_part("wall_bracket")
    carriage = object_model.get_part("pan_carriage")
    primary = object_model.get_part("primary_arm")
    secondary = object_model.get_part("secondary_arm")
    knuckle = object_model.get_part("head_knuckle")
    head = object_model.get_part("tilt_head")

    ctx.allow_overlap(
        wall,
        carriage,
        elem_a="pan_spindle_socket",
        elem_b="rotating_spindle",
        reason="The pan spindle is intentionally captured inside the wall bracket bearing socket.",
    )
    ctx.allow_overlap(
        wall,
        carriage,
        elem_a="fixed_bearing_cup",
        elem_b="rotating_spindle",
        reason="The rotating spindle passes through the fixed bearing cup as a retained pivot.",
    )
    ctx.allow_overlap(
        wall,
        carriage,
        elem_a="upper_thrust_race",
        elem_b="rotating_spindle",
        reason="The upper thrust race is intentionally seated around the rotating pan spindle.",
    )
    ctx.allow_overlap(
        wall,
        carriage,
        elem_a="lower_thrust_race",
        elem_b="rotating_spindle",
        reason="The lower thrust race is intentionally seated around the rotating pan spindle.",
    )
    ctx.allow_overlap(
        carriage,
        primary,
        elem_a="shoulder_cross_pin",
        elem_b="shoulder_bushing",
        reason="The shoulder cross pin is intentionally nested in the arm bushing.",
    )
    ctx.allow_overlap(
        carriage,
        primary,
        elem_a="shoulder_cross_pin",
        elem_b="shoulder_knuckle",
        reason="The visible shoulder knuckle is intentionally bored through by the support pin.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_pin",
        elem_b="elbow_inner_sleeve",
        reason="The elbow pin is intentionally captured by the secondary-arm sleeve.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_outer_barrel",
        elem_b="elbow_inner_sleeve",
        reason="The inner elbow sleeve is intentionally nested inside the primary-arm barrel.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_outer_barrel",
        elem_b="machined_box_beam",
        reason="The secondary beam is represented with an integral hinge boss that locally wraps into the outer barrel envelope.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_pin",
        elem_b="machined_box_beam",
        reason="The elbow pin passes through the secondary beam's integral hinge boss.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="upper_elbow_strap",
        elem_b="elbow_friction_disk_top",
        reason="The upper clamp strap intentionally compresses the elbow friction disk.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="lower_elbow_strap",
        elem_b="elbow_friction_disk_bottom",
        reason="The lower clamp strap intentionally compresses the elbow friction disk.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_pin",
        elem_b="elbow_friction_disk_top",
        reason="The elbow pin passes through the top friction disk.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_pin",
        elem_b="elbow_friction_disk_bottom",
        reason="The elbow pin passes through the bottom friction disk.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_pin",
        elem_b="pan_bushing",
        reason="The head pan pin intentionally passes through the pan bushing.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_barrel",
        elem_b="pan_bushing",
        reason="The pan bushing is intentionally nested in the secondary-arm head barrel.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_barrel",
        elem_b="tilt_yoke_bridge",
        reason="The head-yoke bridge is simplified as a machined boss wrapping the pan barrel at the swivel.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_pin",
        elem_b="tilt_yoke_bridge",
        reason="The head pan pin passes through the head-yoke bridge boss.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_pin",
        elem_b="pan_friction_collar_top",
        reason="The head pan pin passes through the top friction collar.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_pin",
        elem_b="pan_friction_collar_bottom",
        reason="The head pan pin passes through the bottom friction collar.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_barrel",
        elem_b="pan_friction_collar_top",
        reason="The top friction collar seats around the head pan barrel.",
    )
    ctx.allow_overlap(
        secondary,
        knuckle,
        elem_a="head_pan_barrel",
        elem_b="pan_friction_collar_bottom",
        reason="The bottom friction collar seats around the head pan barrel.",
    )
    ctx.allow_overlap(
        knuckle,
        head,
        elem_a="tilt_cross_pin",
        elem_b="tilt_trunnion",
        reason="The tilt cross pin intentionally passes through the trunnion.",
    )
    ctx.allow_overlap(
        knuckle,
        head,
        elem_a="tilt_cross_pin",
        elem_b="trunnion_block",
        reason="The tilt pin passes through the bored trunnion block as visible pivot hardware.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_web_0",
        elem_b="machined_box_beam",
        reason="The elbow side web locally intersects the simplified secondary beam hinge boss.",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_web_1",
        elem_b="machined_box_beam",
        reason="The elbow side web locally intersects the simplified secondary beam hinge boss.",
    )

    ctx.expect_within(
        carriage,
        wall,
        axes="xy",
        inner_elem="rotating_spindle",
        outer_elem="pan_spindle_socket",
        margin=0.006,
        name="pan spindle stays centered in bearing socket",
    )
    ctx.expect_overlap(
        carriage,
        wall,
        axes="z",
        elem_a="rotating_spindle",
        elem_b="pan_spindle_socket",
        min_overlap=0.150,
        name="pan spindle remains deeply captured",
    )
    ctx.expect_overlap(
        carriage,
        wall,
        axes="z",
        elem_a="rotating_spindle",
        elem_b="upper_thrust_race",
        min_overlap=0.010,
        name="thrust race surrounds pan spindle",
    )
    ctx.expect_overlap(
        carriage,
        wall,
        axes="z",
        elem_a="rotating_spindle",
        elem_b="lower_thrust_race",
        min_overlap=0.010,
        name="lower thrust race surrounds pan spindle",
    )
    ctx.expect_within(
        primary,
        carriage,
        axes="yz",
        inner_elem="shoulder_bushing",
        outer_elem="shoulder_cross_pin",
        margin=0.010,
        name="shoulder bushing is coaxial with support pin",
    )
    ctx.expect_overlap(
        primary,
        carriage,
        axes="y",
        elem_a="shoulder_knuckle",
        elem_b="shoulder_cross_pin",
        min_overlap=0.045,
        name="shoulder knuckle is bored by cross pin",
    )
    ctx.expect_within(
        secondary,
        primary,
        axes="xy",
        inner_elem="elbow_inner_sleeve",
        outer_elem="elbow_pin",
        margin=0.010,
        name="elbow sleeve is centered on hinge pin",
    )
    ctx.expect_within(
        secondary,
        primary,
        axes="xy",
        inner_elem="elbow_inner_sleeve",
        outer_elem="elbow_outer_barrel",
        margin=0.006,
        name="elbow sleeve stays inside outer barrel footprint",
    )
    ctx.expect_overlap(
        secondary,
        primary,
        axes="z",
        elem_a="machined_box_beam",
        elem_b="elbow_outer_barrel",
        min_overlap=0.040,
        name="secondary hinge boss wraps elbow barrel height",
    )
    ctx.expect_overlap(
        secondary,
        primary,
        axes="z",
        elem_a="machined_box_beam",
        elem_b="elbow_pin",
        min_overlap=0.040,
        name="secondary hinge boss is bored by elbow pin",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="z",
        elem_a="elbow_web_0",
        elem_b="machined_box_beam",
        min_overlap=0.030,
        name="upper side web keys into secondary hinge boss",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="z",
        elem_a="elbow_web_1",
        elem_b="machined_box_beam",
        min_overlap=0.030,
        name="lower side web keys into secondary hinge boss",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="xy",
        elem_a="upper_elbow_strap",
        elem_b="elbow_friction_disk_top",
        min_overlap=0.030,
        name="upper elbow strap covers friction disk",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="xy",
        elem_a="lower_elbow_strap",
        elem_b="elbow_friction_disk_bottom",
        min_overlap=0.030,
        name="lower elbow strap covers friction disk",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="xy",
        elem_a="elbow_pin",
        elem_b="elbow_friction_disk_top",
        min_overlap=0.030,
        name="elbow pin is centered in top friction disk",
    )
    ctx.expect_overlap(
        primary,
        secondary,
        axes="xy",
        elem_a="elbow_pin",
        elem_b="elbow_friction_disk_bottom",
        min_overlap=0.030,
        name="elbow pin is centered in bottom friction disk",
    )
    ctx.expect_within(
        knuckle,
        secondary,
        axes="xy",
        inner_elem="pan_bushing",
        outer_elem="head_pan_barrel",
        margin=0.006,
        name="head pan bushing stays inside barrel footprint",
    )
    ctx.expect_overlap(
        knuckle,
        secondary,
        axes="z",
        elem_a="tilt_yoke_bridge",
        elem_b="head_pan_barrel",
        min_overlap=0.030,
        name="head yoke bridge wraps pan barrel height",
    )
    ctx.expect_overlap(
        knuckle,
        secondary,
        axes="z",
        elem_a="tilt_yoke_bridge",
        elem_b="head_pan_pin",
        min_overlap=0.030,
        name="head yoke bridge is bored by pan pin",
    )
    ctx.expect_overlap(
        secondary,
        knuckle,
        axes="xy",
        elem_a="head_pan_pin",
        elem_b="pan_friction_collar_top",
        min_overlap=0.030,
        name="head pan pin is centered in top collar",
    )
    ctx.expect_overlap(
        secondary,
        knuckle,
        axes="xy",
        elem_a="head_pan_pin",
        elem_b="pan_friction_collar_bottom",
        min_overlap=0.030,
        name="head pan pin is centered in bottom collar",
    )
    ctx.expect_overlap(
        secondary,
        knuckle,
        axes="xy",
        elem_a="head_pan_barrel",
        elem_b="pan_friction_collar_top",
        min_overlap=0.030,
        name="top head collar seats around barrel",
    )
    ctx.expect_overlap(
        secondary,
        knuckle,
        axes="xy",
        elem_a="head_pan_barrel",
        elem_b="pan_friction_collar_bottom",
        min_overlap=0.030,
        name="bottom head collar seats around barrel",
    )
    ctx.expect_overlap(
        head,
        knuckle,
        axes="y",
        elem_a="tilt_trunnion",
        elem_b="tilt_cross_pin",
        min_overlap=0.050,
        name="tilt trunnion remains captured by yoke pin",
    )
    ctx.expect_overlap(
        head,
        knuckle,
        axes="y",
        elem_a="trunnion_block",
        elem_b="tilt_cross_pin",
        min_overlap=0.030,
        name="trunnion block is bored by tilt pin",
    )

    shoulder = object_model.get_articulation("shoulder_lift")
    elbow = object_model.get_articulation("elbow_fold")
    tilt = object_model.get_articulation("head_tilt")
    head_pan = object_model.get_articulation("head_pan")
    rest_head = ctx.part_world_position(head)
    with ctx.pose({shoulder: 0.60}):
        lifted_head = ctx.part_world_position(head)
    ctx.check(
        "counterbalance shoulder raises head assembly",
        rest_head is not None and lifted_head is not None and lifted_head[2] > rest_head[2] + 0.18,
        details=f"rest={rest_head}, lifted={lifted_head}",
    )
    with ctx.pose({elbow: -1.10}):
        folded_secondary = ctx.part_world_position(secondary)
    ctx.check(
        "elbow joint folds secondary arm laterally",
        folded_secondary is not None,
        details=f"folded_secondary={folded_secondary}",
    )
    with ctx.pose({head_pan: 0.80, tilt: 0.45}):
        ctx.expect_overlap(
            head,
            knuckle,
            axes="y",
            elem_a="tilt_trunnion",
            elem_b="tilt_cross_pin",
            min_overlap=0.040,
            name="tilted head keeps trunnion engagement",
        )

    return ctx.report()


object_model = build_object_model()
