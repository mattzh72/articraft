from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _cyl_x(length: float, radius: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(length: float, radius: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _visual_cylinder_between(
    part,
    name: str,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material,
) -> None:
    """Add a round tube between two points in the XZ plane or along Y."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)

    if abs(dy) > 1e-9 and abs(dx) < 1e-9 and abs(dz) < 1e-9:
        rpy = (-math.pi / 2.0 if dy >= 0.0 else math.pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, math.atan2(dx, dz), 0.0)

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def _knob_mesh(name: str, diameter: float, height: float, *, indicator: str = "line"):
    return mesh_from_geometry(
        KnobGeometry(
            diameter,
            height,
            body_style="lobed",
            crown_radius=0.0015,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=10, depth=0.0012),
            indicator=KnobIndicator(style=indicator, mode="raised", angle_deg=0.0),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stationary_exercise_bike")

    satin_black = Material("satin_black_powdercoat", rgba=(0.015, 0.017, 0.018, 1.0))
    graphite = Material("dark_graphite_housing", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = Material("soft_black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    brushed = Material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    steel = Material("polished_steel", rgba=(0.86, 0.86, 0.82, 1.0))
    leather = Material("black_textured_saddle", rgba=(0.018, 0.018, 0.016, 1.0))
    accent = Material("anodized_blue_accent", rgba=(0.05, 0.22, 0.72, 1.0))
    red = Material("red_quick_release_tabs", rgba=(0.85, 0.05, 0.02, 1.0))
    screen = Material("dark_glass_display", rgba=(0.015, 0.03, 0.04, 1.0))

    frame = model.part("frame")

    # Heavy floor contact rails and welded main structure.
    cyl, rot = _cyl_y(0.86, 0.035)
    frame.visual(cyl, origin=Origin(xyz=(0.62, 0.0, 0.045), rpy=rot.rpy), material=satin_black, name="front_floor_tube")
    cyl, rot = _cyl_y(0.78, 0.035)
    frame.visual(cyl, origin=Origin(xyz=(-0.63, 0.0, 0.045), rpy=rot.rpy), material=satin_black, name="rear_floor_tube")
    cyl, rot = _cyl_x(1.30, 0.028)
    frame.visual(cyl, origin=Origin(xyz=(0.0, 0.0, 0.088), rpy=rot.rpy), material=satin_black, name="spine_tube")

    # Rubber stabilizer pads at the feet.
    for i, (x, y, nm) in enumerate(
        [
            (0.62, -0.43, "front_pad_0"),
            (0.62, 0.43, "front_pad_1"),
            (-0.63, -0.39, "rear_pad_0"),
            (-0.63, 0.39, "rear_pad_1"),
        ]
    ):
        frame.visual(
            Box((0.18, 0.055, 0.030)),
            origin=Origin(xyz=(x, y, 0.018)),
            material=rubber,
            name=nm,
        )

    # Welded triangular frame, crank support, and post sleeves.
    _visual_cylinder_between(frame, "rear_stay", (-0.63, 0.0, 0.09), (-0.44, 0.0, 0.66), 0.026, satin_black)
    frame.visual(
        Cylinder(radius=0.033, length=math.sqrt(0.12 * 0.12 + 0.52 * 0.52)),
        origin=Origin(xyz=(-0.06, 0.0, 0.36), rpy=(0.0, math.atan2(0.12, 0.52), 0.0)),
        material=satin_black,
        name="down_tube",
    )
    frame.visual(
        Cylinder(radius=0.026, length=math.sqrt(0.49 * 0.49 + 0.13 * 0.13)),
        origin=Origin(xyz=(-0.195, 0.0, 0.695), rpy=(0.0, math.atan2(0.49, -0.13), 0.0)),
        material=satin_black,
        name="top_tube",
    )
    _visual_cylinder_between(frame, "front_stay_lower", (0.62, 0.0, 0.09), (0.85, 0.0, 1.00), 0.030, satin_black)
    _visual_cylinder_between(frame, "front_stay_bridge", (0.85, 0.0, 1.00), (0.85, 0.12, 1.00), 0.020, satin_black)
    _visual_cylinder_between(frame, "front_stay_upper", (0.85, 0.12, 1.00), (0.30, 0.12, 0.82), 0.030, satin_black)
    _visual_cylinder_between(frame, "front_cluster_bridge", (0.30, 0.0, 0.82), (0.30, 0.12, 0.82), 0.015, satin_black)
    _visual_cylinder_between(frame, "fork_stay_bridge", (0.12, 0.0, 0.66), (0.12, 0.075, 0.66), 0.018, satin_black)
    frame.visual(
        Cylinder(radius=0.023, length=math.sqrt(0.30 * 0.30 + 0.11 * 0.11)),
        origin=Origin(xyz=(0.27, 0.075, 0.605), rpy=(0.0, math.atan2(0.30, -0.11), 0.0)),
        material=satin_black,
        name="fork_stay",
    )
    _visual_cylinder_between(frame, "lower_chain_stay", (-0.10, 0.0, 0.18), (0.62, 0.0, 0.28), 0.020, satin_black)
    frame.visual(
        Cylinder(radius=0.039, length=0.31),
        origin=Origin(xyz=(-0.43, 0.0, 0.785)),
        material=graphite,
        name="seat_sleeve",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.28),
        origin=Origin(xyz=(0.29, 0.0, 0.92)),
        material=graphite,
        name="handlebar_sleeve",
    )

    for x, z, nm in [
        (-0.63, 0.09, "rear_weld"),
        (-0.43, 0.66, "seat_cluster"),
        (0.00, 0.62, "crank_cluster"),
        (0.30, 0.82, "front_cluster"),
        (0.62, 0.09, "front_weld"),
    ]:
        frame.visual(
            Sphere(0.055 if nm == "crank_cluster" else (0.020 if nm == "front_cluster" else 0.043)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=satin_black,
            name=nm,
        )

    # Belt guard and premium flywheel shroud: two side plates with a visible spinning gap.
    frame.visual(
        Box((0.58, 0.035, 0.145)),
        origin=Origin(xyz=(0.23, 0.200, 0.575), rpy=(0.0, -0.18, 0.0)),
        material=graphite,
        name="belt_guard",
    )
    frame.visual(
        Box((0.18, 0.115, 0.060)),
        origin=Origin(xyz=(0.22, 0.160, 0.560), rpy=(0.0, -0.18, 0.0)),
        material=graphite,
        name="guard_mount_bridge",
    )
    frame.visual(
        Box((0.16, 0.045, 0.100)),
        origin=Origin(xyz=(0.28, 0.085, 0.560), rpy=(0.0, -0.18, 0.0)),
        material=graphite,
        name="guard_inner_web",
    )
    for y, nm in [(-0.055, "flywheel_cover_0"), (0.055, "flywheel_cover_1")]:
        frame.visual(
            Cylinder(radius=0.305, length=0.018),
            origin=Origin(xyz=(0.43, y, 0.53), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=nm,
        )
    cyl, rot = _cyl_y(0.15, 0.036)
    frame.visual(cyl, origin=Origin(xyz=(0.43, 0.0, 0.53), rpy=rot.rpy), material=brushed, name="flywheel_axle_boss")

    # Small console mast and display fixed to the handlebar structure support.
    _visual_cylinder_between(frame, "console_mast", (0.34, 0.052, 1.055), (0.20, 0.052, 1.130), 0.014, satin_black)
    frame.visual(
        Box((0.060, 0.030, 0.025)),
        origin=Origin(xyz=(0.315, 0.045, 1.055)),
        material=satin_black,
        name="console_side_bracket",
    )
    frame.visual(
        Box((0.16, 0.05, 0.035)),
        origin=Origin(xyz=(0.20, 0.052, 1.145), rpy=(0.0, -0.25, 0.0)),
        material=graphite,
        name="console_pod",
    )
    frame.visual(
        Box((0.125, 0.006, 0.055)),
        origin=Origin(xyz=(0.185, 0.025, 1.153), rpy=(0.0, -0.25, 0.0)),
        material=screen,
        name="display_glass",
    )

    # Crank axle and arms rotate as a single assembly about the bottom-bracket axis.
    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.047, length=0.255),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="bottom_bracket_spindle",
    )
    crank.visual(Box((0.040, 0.026, 0.210)), origin=Origin(xyz=(0.0, -0.138, -0.095)), material=brushed, name="right_crank_arm")
    crank.visual(Box((0.040, 0.026, 0.210)), origin=Origin(xyz=(0.0, 0.138, 0.095)), material=brushed, name="left_crank_arm")
    crank.visual(Sphere(0.055), origin=Origin(), material=steel, name="crank_hub")
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )

    # Independent rotating pedals mounted at the ends of the crank arms.
    for side, y, z in [("right", -0.205, -0.205), ("left", 0.205, 0.205)]:
        pedal = model.part(f"{side}_pedal")
        pedal.visual(Box((0.155, 0.055, 0.026)), origin=Origin(), material=rubber, name="pedal_body")
        pedal.visual(Box((0.120, 0.010, 0.048)), origin=Origin(xyz=(0.0, 0.0, 0.035)), material=brushed, name="toe_cage")
        cyl, rot = _cyl_y(0.108, 0.010)
        pedal.visual(cyl, origin=Origin(rpy=rot.rpy), material=steel, name="spindle_bar")
        model.articulation(
            f"crank_to_{side}_pedal",
            ArticulationType.CONTINUOUS,
            parent=crank,
            child=pedal,
            origin=Origin(xyz=(0.0, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=18.0),
        )

    # Flywheel spins behind the static side covers. Its angular velocity is coupled to the crank.
    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.198, length=0.032),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="inertia_disk",
    )
    flywheel.visual(
        mesh_from_geometry(TorusGeometry(0.218, 0.016, radial_segments=24, tubular_segments=72), "flywheel_outer_ring"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="outer_ring",
    )
    flywheel.visual(Box((0.430, 0.020, 0.026)), origin=Origin(), material=brushed, name="horizontal_spoke")
    flywheel.visual(Box((0.026, 0.020, 0.430)), origin=Origin(), material=brushed, name="vertical_spoke")
    flywheel.visual(
        Cylinder(radius=0.055, length=0.090),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="flywheel_hub",
    )
    model.articulation(
        "frame_to_flywheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.43, 0.0, 0.53)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=60.0),
        mimic=Mimic(joint="frame_to_crank", multiplier=3.2),
    )

    # Height-adjustable saddle post with a retained insertion length.
    seat_post = model.part("seat_post")
    seat_post.visual(Cylinder(radius=0.026, length=0.455), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=brushed, name="inner_post")
    seat_post.visual(Box((0.140, 0.075, 0.036)), origin=Origin(xyz=(0.0, 0.0, 0.238)), material=graphite, name="seat_clamp")
    cyl, rot = _cyl_y(0.170, 0.014)
    seat_post.visual(cyl, origin=Origin(xyz=(0.0, 0.0, 0.255), rpy=rot.rpy), material=steel, name="rail_crosspin")
    model.articulation(
        "frame_to_seat_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_post,
        origin=Origin(xyz=(-0.43, 0.0, 0.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.18, lower=0.0, upper=0.17),
    )

    saddle = model.part("saddle")
    saddle_shell = superellipse_side_loft(
        [
            (-0.155, 0.000, 0.035, 0.105),
            (-0.070, 0.000, 0.060, 0.230),
            (0.045, 0.000, 0.066, 0.255),
            (0.145, 0.000, 0.044, 0.145),
        ],
        exponents=3.0,
        segments=44,
    )
    saddle_shell.rotate_z(-math.pi / 2.0).translate(0.015, 0.0, 0.040)
    saddle.visual(mesh_from_geometry(saddle_shell, "sculpted_saddle"), material=leather, name="saddle_cushion")
    for y, nm in [(-0.035, "saddle_rail_0"), (0.035, "saddle_rail_1")]:
        cyl, rot = _cyl_x(0.235, 0.007)
        saddle.visual(cyl, origin=Origin(xyz=(0.005, y, 0.030), rpy=rot.rpy), material=steel, name=nm)
    saddle.visual(Box((0.075, 0.090, 0.028)), origin=Origin(xyz=(-0.015, 0.0, 0.011)), material=graphite, name="rail_clamp")
    saddle.visual(Box((0.220, 0.150, 0.014)), origin=Origin(xyz=(0.020, 0.0, 0.034)), material=graphite, name="saddle_base_plate")
    model.articulation(
        "seat_post_to_saddle",
        ArticulationType.PRISMATIC,
        parent=seat_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.272)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.08, lower=-0.055, upper=0.070),
    )

    # Height-adjustable handlebar post and multi-position grips.
    handlebar_post = model.part("handlebar_post")
    handlebar_post.visual(Cylinder(radius=0.026, length=0.435), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=brushed, name="inner_stem")
    handlebar_post.visual(Box((0.100, 0.135, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.258)), material=graphite, name="bar_clamp")
    handlebar_post.visual(Box((0.070, 0.075, 0.070)), origin=Origin(xyz=(0.050, 0.0, 0.296)), material=graphite, name="stem_to_bar_bridge")
    cyl, rot = _cyl_y(0.560, 0.018)
    handlebar_post.visual(cyl, origin=Origin(xyz=(0.090, 0.0, 0.330), rpy=rot.rpy), material=satin_black, name="wide_crossbar")
    for y, nm in [(-0.245, "grip_0"), (0.245, "grip_1")]:
        cyl, rot = _cyl_x(0.210, 0.019)
        handlebar_post.visual(cyl, origin=Origin(xyz=(0.190, y, 0.332), rpy=rot.rpy), material=rubber, name=nm)
        handlebar_post.visual(Box((0.060, 0.030, 0.030)), origin=Origin(xyz=(0.070, y, 0.330)), material=satin_black, name=f"bar_elbow_{0 if y < 0 else 1}")
    cyl, rot = _cyl_x(0.190, 0.012)
    handlebar_post.visual(cyl, origin=Origin(xyz=(0.245, -0.070, 0.390), rpy=rot.rpy), material=rubber, name="aero_grip_0")
    handlebar_post.visual(cyl, origin=Origin(xyz=(0.245, 0.070, 0.390), rpy=rot.rpy), material=rubber, name="aero_grip_1")
    _visual_cylinder_between(handlebar_post, "aero_support_0", (0.090, -0.070, 0.330), (0.205, -0.070, 0.386), 0.010, satin_black)
    _visual_cylinder_between(handlebar_post, "aero_support_1", (0.090, 0.070, 0.330), (0.205, 0.070, 0.386), 0.010, satin_black)
    model.articulation(
        "frame_to_handlebar_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handlebar_post,
        origin=Origin(xyz=(0.29, 0.0, 1.03)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.16, lower=0.0, upper=0.13),
    )

    # Articulated user adjustment points.
    for name, parent, xyz, diameter, height, mat in [
        ("seat_knob", frame, (-0.43, -0.065, 0.870), 0.056, 0.030, red),
        ("handlebar_knob", frame, (0.29, -0.065, 0.995), 0.052, 0.030, red),
    ]:
        knob = model.part(name)
        knob.visual(Cylinder(radius=0.008, length=0.080), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=steel, name="threaded_pin")
        knob.visual(_knob_mesh(f"{name}_mesh", diameter, height), origin=Origin(), material=mat, name="lobed_knob")
        model.articulation(
            f"frame_to_{name}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=knob,
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        )

    resistance_knob = model.part("resistance_knob")
    resistance_knob.visual(_knob_mesh("resistance_knob_mesh", 0.070, 0.036, indicator="wedge"), material=accent, name="dial_cap")
    resistance_knob.visual(Cylinder(radius=0.010, length=0.075), origin=Origin(xyz=(0.0, 0.0, -0.040)), material=steel, name="dial_stem")
    model.articulation(
        "frame_to_resistance_knob",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=resistance_knob,
        origin=Origin(xyz=(0.160, 0.080, 1.199)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0, lower=-2.4, upper=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    crank = object_model.get_part("crank")
    flywheel = object_model.get_part("flywheel")
    seat_post = object_model.get_part("seat_post")
    saddle = object_model.get_part("saddle")
    handlebar_post = object_model.get_part("handlebar_post")
    seat_knob = object_model.get_part("seat_knob")
    handlebar_knob = object_model.get_part("handlebar_knob")
    resistance_knob = object_model.get_part("resistance_knob")

    crank_joint = object_model.get_articulation("frame_to_crank")
    seat_height = object_model.get_articulation("frame_to_seat_post")
    saddle_slide = object_model.get_articulation("seat_post_to_saddle")
    bar_height = object_model.get_articulation("frame_to_handlebar_post")

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="crank_cluster",
        elem_b="bottom_bracket_spindle",
        reason="The bottom bracket spindle is intentionally captured through the frame bearing housing.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="crank_cluster",
        elem_b="crank_hub",
        reason="The crank hub is intentionally seated against the bottom bracket bearing housing.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="down_tube",
        elem_b="crank_hub",
        reason="The welded down tube locally blends into the bottom bracket shell behind the rotating crank hub.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="down_tube",
        elem_b="bottom_bracket_spindle",
        reason="The bottom bracket spindle passes through the welded down-tube/bearing-shell proxy.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="top_tube",
        elem_b="crank_hub",
        reason="The top tube is welded into the bottom bracket cluster behind the crank hub.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="top_tube",
        elem_b="bottom_bracket_spindle",
        reason="The bottom bracket spindle passes through the welded top-tube/bearing-shell proxy.",
    )
    ctx.expect_contact(
        frame,
        crank,
        elem_a="crank_cluster",
        elem_b="bottom_bracket_spindle",
        name="crank spindle is seated in bottom bracket housing",
    )
    ctx.expect_contact(
        frame,
        crank,
        elem_a="crank_cluster",
        elem_b="crank_hub",
        name="crank hub seats against bottom bracket shell",
    )
    ctx.expect_contact(
        frame,
        crank,
        elem_a="down_tube",
        elem_b="crank_hub",
        name="down tube meets the bottom bracket cluster",
    )
    ctx.expect_contact(
        frame,
        crank,
        elem_a="down_tube",
        elem_b="bottom_bracket_spindle",
        name="bottom bracket spindle passes through down-tube shell",
    )
    ctx.expect_contact(
        frame,
        crank,
        elem_a="top_tube",
        elem_b="crank_hub",
        name="top tube reaches the bottom bracket cluster",
    )
    ctx.expect_contact(
        frame,
        crank,
        elem_a="top_tube",
        elem_b="bottom_bracket_spindle",
        name="bottom bracket spindle passes through top-tube shell",
    )

    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="flywheel_hub",
        reason="The flywheel hub is intentionally captured inside the fixed axle boss.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="inertia_disk",
        reason="The fixed axle boss passes through the flywheel's central bearing zone.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="vertical_spoke",
        reason="The fixed axle boss passes through the central spoke-and-bearing stack of the flywheel.",
    )
    ctx.allow_overlap(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="horizontal_spoke",
        reason="The fixed axle boss passes through the central spoke-and-bearing stack of the flywheel.",
    )
    ctx.expect_contact(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="flywheel_hub",
        name="flywheel hub is seated in axle boss",
    )
    ctx.expect_contact(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="inertia_disk",
        name="flywheel disk surrounds the axle boss",
    )
    ctx.expect_contact(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="vertical_spoke",
        name="flywheel spoke stack surrounds the axle boss",
    )
    ctx.expect_contact(
        frame,
        flywheel,
        elem_a="flywheel_axle_boss",
        elem_b="horizontal_spoke",
        name="flywheel cross spoke surrounds the axle boss",
    )

    # The simplified solid sleeve visuals intentionally proxy retained telescoping tubes.
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="seat_sleeve",
        elem_b="inner_post",
        reason="The seat post is modeled as a sliding member retained inside the height sleeve proxy.",
    )
    ctx.expect_within(
        seat_post,
        frame,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="seat_sleeve",
        margin=0.003,
        name="seat post remains centered in its sleeve",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="seat_sleeve",
        min_overlap=0.090,
        name="seat post has retained sleeve insertion",
    )
    ctx.allow_overlap(
        frame,
        seat_post,
        elem_a="top_tube",
        elem_b="inner_post",
        reason="The hidden lower seat post passes through the welded seat-cluster/tube proxy before emerging from the sleeve.",
    )
    ctx.expect_overlap(
        seat_post,
        frame,
        axes="z",
        elem_a="inner_post",
        elem_b="top_tube",
        min_overlap=0.020,
        name="seat post extends through the welded seat cluster",
    )

    ctx.allow_overlap(
        frame,
        handlebar_post,
        elem_a="handlebar_sleeve",
        elem_b="inner_stem",
        reason="The handlebar stem is modeled as a sliding member retained inside the height sleeve proxy.",
    )
    ctx.expect_within(
        handlebar_post,
        frame,
        axes="xy",
        inner_elem="inner_stem",
        outer_elem="handlebar_sleeve",
        margin=0.003,
        name="handlebar stem remains centered in its sleeve",
    )
    ctx.expect_overlap(
        handlebar_post,
        frame,
        axes="z",
        elem_a="inner_stem",
        elem_b="handlebar_sleeve",
        min_overlap=0.080,
        name="handlebar stem has retained sleeve insertion",
    )

    for knob, sleeve, label in [
        (seat_knob, "seat_sleeve", "seat"),
        (handlebar_knob, "handlebar_sleeve", "handlebar"),
    ]:
        ctx.allow_overlap(
            frame,
            knob,
            elem_a=sleeve,
            elem_b="threaded_pin",
            reason=f"The {label} quick-release pin threads into the adjustment sleeve.",
        )
        ctx.expect_contact(
            frame,
            knob,
            elem_a=sleeve,
            elem_b="threaded_pin",
            name=f"{label} adjustment pin engages its sleeve",
        )

    ctx.allow_overlap(
        seat_knob,
        seat_post,
        elem_a="threaded_pin",
        elem_b="inner_post",
        reason="The seat quick-release pin bears into the sliding inner post to lock height.",
    )
    ctx.expect_contact(
        seat_knob,
        seat_post,
        elem_a="threaded_pin",
        elem_b="inner_post",
        name="seat adjustment pin reaches the inner post",
    )
    ctx.allow_overlap(
        handlebar_knob,
        handlebar_post,
        elem_a="threaded_pin",
        elem_b="inner_stem",
        reason="The handlebar quick-release pin bears into the sliding inner stem to lock height.",
    )
    ctx.expect_contact(
        handlebar_knob,
        handlebar_post,
        elem_a="threaded_pin",
        elem_b="inner_stem",
        name="handlebar adjustment pin reaches the inner stem",
    )

    ctx.allow_overlap(
        saddle,
        seat_post,
        elem_a="rail_clamp",
        elem_b="rail_crosspin",
        reason="The saddle rail clamp wraps around the crosspin in this simplified adjustment carriage.",
    )
    ctx.expect_contact(
        saddle,
        seat_post,
        elem_a="rail_clamp",
        elem_b="rail_crosspin",
        name="saddle clamp is seated on the rail crosspin",
    )

    ctx.allow_overlap(
        frame,
        resistance_knob,
        elem_a="console_pod",
        elem_b="dial_stem",
        reason="The resistance dial stem is intentionally captured inside the console pod.",
    )
    ctx.expect_contact(
        frame,
        resistance_knob,
        elem_a="console_pod",
        elem_b="dial_stem",
        name="resistance dial stem is seated in console",
    )

    ctx.expect_origin_distance(crank, flywheel, axes="xz", min_dist=0.35, max_dist=0.50, name="crank and flywheel are belt-spaced")
    ctx.expect_origin_gap(saddle, frame, axis="z", min_gap=0.15, name="saddle sits above the frame")

    rest_seat = ctx.part_world_position(seat_post)
    rest_bar = ctx.part_world_position(handlebar_post)
    rest_crank_aabb = ctx.part_world_aabb(crank)
    rest_saddle_aabb = ctx.part_world_aabb(saddle)
    rest_handlebar_aabb = ctx.part_world_aabb(handlebar_post)
    with ctx.pose({seat_height: 0.16, saddle_slide: 0.06, bar_height: 0.12, crank_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            seat_post,
            frame,
            axes="z",
            elem_a="inner_post",
            elem_b="seat_sleeve",
            min_overlap=0.030,
            name="extended seat post remains retained",
        )
        ctx.expect_overlap(
            handlebar_post,
            frame,
            axes="z",
            elem_a="inner_stem",
            elem_b="handlebar_sleeve",
            min_overlap=0.030,
            name="extended handlebar post remains retained",
        )
        raised_seat = ctx.part_world_position(seat_post)
        raised_bar = ctx.part_world_position(handlebar_post)
        turned_crank_aabb = ctx.part_world_aabb(crank)

    ctx.check(
        "seat height adjustment moves upward",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.12,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )
    ctx.check(
        "handlebar height adjustment moves upward",
        rest_bar is not None and raised_bar is not None and raised_bar[2] > rest_bar[2] + 0.09,
        details=f"rest={rest_bar}, raised={raised_bar}",
    )
    ctx.check(
        "crank rotation changes arm orientation",
        rest_crank_aabb is not None
        and turned_crank_aabb is not None
        and abs((rest_crank_aabb[1][0] - rest_crank_aabb[0][0]) - (turned_crank_aabb[1][0] - turned_crank_aabb[0][0])) > 0.05,
        details=f"rest_aabb={rest_crank_aabb}, turned_aabb={turned_crank_aabb}",
    )
    ctx.check(
        "handlebar grips sit above saddle",
        rest_saddle_aabb is not None
        and rest_handlebar_aabb is not None
        and rest_handlebar_aabb[1][2] > rest_saddle_aabb[1][2] + 0.05,
        details=f"saddle_aabb={rest_saddle_aabb}, handlebar_aabb={rest_handlebar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
