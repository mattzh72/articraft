from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _superellipse_loop(
    *,
    cx: float,
    cy: float,
    z: float,
    width: float,
    depth: float,
    exponent: float = 2.8,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    """One horizontal, rounded-rectangle-like section loop."""
    loop: list[tuple[float, float, float]] = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        c = math.cos(theta)
        s = math.sin(theta)
        x = cx + (width * 0.5) * math.copysign(abs(c) ** (2.0 / exponent), c)
        y = cy + (depth * 0.5) * math.copysign(abs(s) ** (2.0 / exponent), s)
        loop.append((x, y, z))
    return loop


def _lofted_pod(
    name: str,
    sections: list[tuple[float, float, float, float, float]],
    *,
    exponent: float = 2.8,
    segments: int = 64,
):
    """Create a premium rounded appliance pod from (z, width, depth, cx, cy)."""
    profiles = [
        _superellipse_loop(
            cx=cx,
            cy=cy,
            z=z,
            width=width,
            depth=depth,
            exponent=exponent,
            segments=segments,
        )
        for z, width, depth, cx, cy in sections
    ]
    return _mesh(LoftGeometry(profiles, cap=True, closed=True), name)


def _rounded_plate_mesh(name: str, width: float, depth: float, height: float, radius: float):
    profile = rounded_rect_profile(width, depth, radius, corner_segments=10)
    return _mesh(ExtrudeGeometry(profile, height, cap=True, center=True), name)


def _add_wand_segment(
    part,
    *,
    metal: Material,
    polymer: Material,
    tube_length: float,
    tube_radius: float,
    has_top_knuckle: bool,
    has_bottom_fork: bool,
    visual_prefix: str,
    knuckle_length: float = 0.080,
) -> None:
    if has_top_knuckle:
        part.visual(
            Cylinder(radius=0.030, length=knuckle_length),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polymer,
            name=f"{visual_prefix}_top_knuckle",
        )
        part.visual(
            Cylinder(radius=0.024, length=0.052),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=polymer,
            name=f"{visual_prefix}_top_collar",
        )

    part.visual(
        Cylinder(radius=tube_radius, length=tube_length),
        origin=Origin(xyz=(0.0, 0.0, -tube_length * 0.5 - 0.030)),
        material=metal,
        name=f"{visual_prefix}_metal_tube",
    )
    part.visual(
        Cylinder(radius=tube_radius + 0.006, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -tube_length - 0.027)),
        material=polymer,
        name="lower_lower_collar" if visual_prefix == "lower" else f"{visual_prefix}_lower_collar",
    )

    if has_bottom_fork:
        joint_z = -tube_length - 0.090
        # A top bridge keeps the fork as one supported yoke while staying clear
        # of the child knuckle at the joint center.
        part.visual(
            Box((0.116, 0.038, 0.036)),
            origin=Origin(xyz=(0.0, 0.0, joint_z + 0.062)),
            material=polymer,
            name=f"{visual_prefix}_fork_bridge",
        )
        for side, x in (("side_a", -0.050), ("side_b", 0.050)):
            part.visual(
                Box((0.014, 0.042, 0.104)),
                origin=Origin(xyz=(x, 0.0, joint_z)),
                material=polymer,
                name=f"{visual_prefix}_{side}_fork",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_articulated_vacuum")

    satin_graphite = model.material("satin_graphite", rgba=(0.13, 0.14, 0.15, 1.0))
    deep_polymer = model.material("deep_polymer", rgba=(0.045, 0.048, 0.052, 1.0))
    warm_metal = model.material("champagne_metal", rgba=(0.74, 0.67, 0.55, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.69, 0.71, 0.73, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.45, 0.56, 0.62, 0.42))
    elastomer = model.material("matte_elastomer", rgba=(0.018, 0.018, 0.019, 1.0))
    soft_button = model.material("soft_button", rgba=(0.23, 0.24, 0.25, 1.0))
    brush_blue = model.material("blue_brush_strip", rgba=(0.08, 0.29, 0.50, 1.0))

    body = model.part("body")
    body.visual(
        _lofted_pod(
            "body_motor_shell",
            [
                (0.810, 0.105, 0.120, 0.0, -0.020),
                (0.880, 0.160, 0.188, 0.0, -0.027),
                (1.020, 0.188, 0.226, 0.0, -0.025),
                (1.150, 0.176, 0.205, 0.0, -0.022),
                (1.225, 0.118, 0.138, 0.0, -0.018),
            ],
            exponent=3.1,
        ),
        material=satin_graphite,
        name="motor_shell",
    )
    body.visual(
        Cylinder(radius=0.071, length=0.245),
        origin=Origin(xyz=(0.0, 0.098, 0.990)),
        material=smoked_clear,
        name="dust_bin",
    )
    body.visual(
        Cylinder(radius=0.078, length=0.025),
        origin=Origin(xyz=(0.0, 0.098, 1.124)),
        material=warm_metal,
        name="cyclone_band",
    )
    body.visual(
        Cylinder(radius=0.064, length=0.022),
        origin=Origin(xyz=(0.0, 0.098, 0.852)),
        material=deep_polymer,
        name="bin_latch_ring",
    )
    body.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (0.0, -0.112, 0.885),
                    (0.0, -0.190, 0.970),
                    (0.0, -0.178, 1.112),
                    (0.0, -0.080, 1.184),
                ],
                radius=0.021,
                samples_per_segment=18,
                radial_segments=22,
            ),
            "body_handle_loop",
        ),
        material=deep_polymer,
        name="handle_loop",
    )
    body.visual(
        Box((0.088, 0.024, 0.082)),
        origin=Origin(xyz=(0.0, -0.128, 1.046)),
        material=deep_polymer,
        name="trigger_bridge",
    )
    for side, x in (("side_a", -0.035), ("side_b", 0.035)):
        body.visual(
            Box((0.010, 0.042, 0.032)),
            origin=Origin(xyz=(x, -0.145, 1.060)),
            material=deep_polymer,
            name=f"trigger_{side}_ear",
        )
    body.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (0.0, 0.056, 0.862),
                    (0.0, 0.082, 0.852),
                    (0.0, 0.100, 0.848),
                ],
                radius=0.026,
                samples_per_segment=10,
                radial_segments=20,
            ),
            "body_lower_socket_neck",
        ),
        material=deep_polymer,
        name="socket_neck",
    )
    body.visual(
        Box((0.124, 0.050, 0.026)),
        origin=Origin(xyz=(0.0, 0.102, 0.842)),
        material=deep_polymer,
        name="wand_fork_bridge",
    )
    for side, x in (("side_a", -0.055), ("side_b", 0.055)):
        body.visual(
            Box((0.016, 0.048, 0.094)),
            origin=Origin(xyz=(x, 0.104, 0.785)),
            material=deep_polymer,
            name=f"wand_{side}_fork",
        )
    for index, z in enumerate((0.940, 0.985, 1.030, 1.075)):
        body.visual(
            Box((0.006, 0.018, 0.026)),
            origin=Origin(xyz=(0.078, -0.100, z)),
            material=brushed_steel,
            name=f"vent_slit_{index}",
        )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_button,
        name="trigger_pivot",
    )
    trigger.visual(
        Box((0.030, 0.014, 0.072)),
        origin=Origin(xyz=(0.0, 0.000, -0.038)),
        material=soft_button,
        name="trigger_blade",
    )

    mode_button = model.part("mode_button")
    mode_button.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=soft_button,
        name="mode_button_cap",
    )

    upper_wand = model.part("upper_wand")
    _add_wand_segment(
        upper_wand,
        metal=warm_metal,
        polymer=deep_polymer,
        tube_length=0.315,
        tube_radius=0.017,
        has_top_knuckle=True,
        has_bottom_fork=True,
        visual_prefix="upper",
        knuckle_length=0.094,
    )

    lower_wand = model.part("lower_wand")
    _add_wand_segment(
        lower_wand,
        metal=warm_metal,
        polymer=deep_polymer,
        tube_length=0.250,
        tube_radius=0.0155,
        has_top_knuckle=True,
        has_bottom_fork=False,
        visual_prefix="lower",
        knuckle_length=0.086,
    )
    lower_wand.visual(
        Cylinder(radius=0.026, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.306)),
        material=deep_polymer,
        name="swivel_socket",
    )

    nozzle_neck = model.part("nozzle_neck")
    nozzle_neck.visual(
        Cylinder(radius=0.027, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=deep_polymer,
        name="swivel_collar",
    )
    nozzle_neck.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (0.0, 0.000, -0.030),
                    (0.0, 0.022, -0.010),
                    (0.0, 0.048, 0.050),
                    (0.0, 0.052, 0.085),
                ],
                radius=0.022,
                samples_per_segment=14,
                radial_segments=20,
            ),
            "nozzle_neck_elbow",
        ),
        material=deep_polymer,
        name="neck_elbow",
    )
    nozzle_neck.visual(
        Cylinder(radius=0.026, length=0.078),
        origin=Origin(xyz=(0.0, 0.052, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deep_polymer,
        name="nozzle_pitch_knuckle",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        _rounded_plate_mesh("nozzle_top_shell", 0.460, 0.140, 0.034, 0.034),
        origin=Origin(xyz=(0.0, 0.120, -0.090)),
        material=satin_graphite,
        name="top_shell",
    )
    floor_nozzle.visual(
        Box((0.070, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.100, -0.100)),
        material=deep_polymer,
        name="hinge_bridge",
    )
    for side, x in (("side_a", -0.047), ("side_b", 0.047)):
        floor_nozzle.visual(
            Box((0.016, 0.036, 0.064)),
            origin=Origin(xyz=(x, 0.000, -0.004)),
            material=deep_polymer,
            name=f"neck_{side}_fork",
        )
        floor_nozzle.visual(
            Box((0.016, 0.050, 0.088)),
            origin=Origin(xyz=(x, 0.035, -0.055)),
            material=deep_polymer,
            name=f"neck_{side}_strut",
        )
    floor_nozzle.visual(
        Box((0.430, 0.020, 0.027)),
        origin=Origin(xyz=(0.0, 0.185, -0.102)),
        material=elastomer,
        name="front_bumper",
    )
    floor_nozzle.visual(
        Box((0.410, 0.018, 0.017)),
        origin=Origin(xyz=(0.0, -0.115, -0.118)),
        material=elastomer,
        name="rear_squeegee",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.018, length=0.405),
        origin=Origin(xyz=(0.0, -0.115, -0.128), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=elastomer,
        name="rear_roller",
    )
    for x in (-0.206, 0.206):
        floor_nozzle.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, -0.115, -0.128), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=elastomer,
            name="side_wheel",
        )
    for side, x in (("side_a", -0.216), ("side_b", 0.216)):
        floor_nozzle.visual(
            Box((0.028, 0.315, 0.038)),
            origin=Origin(xyz=(x, 0.035, -0.108)),
            material=deep_polymer,
            name=f"side_{side}_rail",
        )
    for side, x in (("side_a", -0.183), ("side_b", 0.183)):
        floor_nozzle.visual(
            Box((0.026, 0.040, 0.060)),
            origin=Origin(xyz=(x, 0.084, -0.126)),
            material=deep_polymer,
            name=f"brush_{side}_bearing",
        )

    brush_roll = model.part("brush_roll")
    brush_roll.visual(
        Cylinder(radius=0.016, length=0.340),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="brush_core",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        brush_roll.visual(
            Box((0.334, 0.004, 0.010)),
            origin=Origin(
                xyz=(0.0, math.cos(angle) * 0.017, math.sin(angle) * 0.017),
                rpy=(angle, 0.0, 0.0),
            ),
            material=brush_blue if index % 2 == 0 else elastomer,
            name=f"brush_strip_{index}",
        )

    model.articulation(
        "body_to_trigger",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.0, -0.150, 1.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=0.32),
    )
    model.articulation(
        "body_to_mode_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button,
        origin=Origin(xyz=(0.0, -0.020, 1.225)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.06, lower=0.0, upper=0.006),
    )
    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(0.0, 0.104, 0.780)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "upper_wand_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, -0.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=-0.85, upper=0.45),
    )
    model.articulation(
        "lower_wand_to_nozzle_neck",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle_neck,
        origin=Origin(xyz=(0.0, 0.0, -0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.8, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "nozzle_neck_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=nozzle_neck,
        child=floor_nozzle,
        origin=Origin(xyz=(0.0, 0.052, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.8, lower=-0.50, upper=0.70),
    )
    model.articulation(
        "floor_nozzle_to_brush_roll",
        ArticulationType.CONTINUOUS,
        parent=floor_nozzle,
        child=brush_roll,
        origin=Origin(xyz=(0.0, 0.084, -0.126)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    nozzle_neck = object_model.get_part("nozzle_neck")
    floor_nozzle = object_model.get_part("floor_nozzle")
    brush_roll = object_model.get_part("brush_roll")

    upper_hinge = object_model.get_articulation("body_to_upper_wand")
    mid_hinge = object_model.get_articulation("upper_wand_to_lower_wand")
    swivel = object_model.get_articulation("lower_wand_to_nozzle_neck")
    nozzle_pitch = object_model.get_articulation("nozzle_neck_to_floor_nozzle")
    brush_spin = object_model.get_articulation("floor_nozzle_to_brush_roll")

    ctx.allow_overlap(
        lower_wand,
        nozzle_neck,
        elem_a="swivel_socket",
        elem_b="neck_elbow",
        reason="The nozzle swivel elbow is intentionally seated inside the lower wand socket as a captured rotating collar.",
    )
    ctx.allow_overlap(
        lower_wand,
        nozzle_neck,
        elem_a="lower_lower_collar",
        elem_b="neck_elbow",
        reason="The base collar intentionally wraps the start of the nozzle neck elbow to show a retained swivel socket.",
    )
    ctx.expect_overlap(
        lower_wand,
        nozzle_neck,
        axes="z",
        elem_a="swivel_socket",
        elem_b="neck_elbow",
        min_overlap=0.020,
        name="swivel elbow stays inserted in the lower wand socket",
    )
    ctx.expect_overlap(
        lower_wand,
        nozzle_neck,
        axes="z",
        elem_a="lower_lower_collar",
        elem_b="neck_elbow",
        min_overlap=0.020,
        name="lower collar wraps the nozzle swivel elbow",
    )

    ctx.check(
        "wand and nozzle have articulated joints",
        all(j is not None for j in (upper_hinge, mid_hinge, swivel, nozzle_pitch, brush_spin)),
        details="Expected body hinge, mid-wand hinge, nozzle swivel, pitch pivot, and brush roll spin.",
    )
    ctx.expect_overlap(
        upper_wand,
        lower_wand,
        axes="xy",
        min_overlap=0.020,
        name="wand segments align through their collared hinge",
    )
    ctx.expect_within(
        brush_roll,
        floor_nozzle,
        axes="xy",
        margin=0.030,
        name="brush roll remains inside the nozzle footprint",
    )
    body_aabb = ctx.part_world_aabb(body)
    nozzle_aabb = ctx.part_world_aabb(floor_nozzle)
    ctx.check(
        "body sits above the low floor nozzle like a stick vacuum",
        body_aabb is not None
        and nozzle_aabb is not None
        and body_aabb[0][2] > nozzle_aabb[1][2] + 0.50,
        details=f"body={body_aabb}, nozzle={nozzle_aabb}",
    )

    rest_upper_aabb = ctx.part_world_aabb(upper_wand)
    rest_neck_aabb = ctx.part_world_aabb(nozzle_neck)
    with ctx.pose({upper_hinge: 0.45, mid_hinge: -0.35, swivel: 0.45, nozzle_pitch: 0.30}):
        posed_upper_aabb = ctx.part_world_aabb(upper_wand)
        posed_neck_aabb = ctx.part_world_aabb(nozzle_neck)
        ctx.expect_within(
            brush_roll,
            floor_nozzle,
            axes="xy",
            margin=0.030,
            name="brush roll stays captured in nozzle while pitched",
        )
    ctx.check(
        "wand hinge visibly pitches the wand forward",
        rest_upper_aabb is not None
        and posed_upper_aabb is not None
        and posed_upper_aabb[1][1] > rest_upper_aabb[1][1] + 0.08,
        details=f"rest={rest_upper_aabb}, posed={posed_upper_aabb}",
    )
    ctx.check(
        "nozzle swivel moves the neck laterally",
        rest_neck_aabb is not None
        and posed_neck_aabb is not None
        and abs(posed_neck_aabb[0][0] - rest_neck_aabb[0][0]) > 0.010,
        details=f"rest={rest_neck_aabb}, posed={posed_neck_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
