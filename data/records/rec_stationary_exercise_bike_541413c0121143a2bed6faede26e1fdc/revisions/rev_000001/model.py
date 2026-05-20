from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annular_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    *,
    segments: int = 72,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments),
            [_circle_profile(inner_radius, segments)],
            length,
            center=True,
        ),
        name,
    )


def _rod_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
) -> None:
    """Add a round tube whose endpoints lie in a constant-Y plane."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1.0e-9:
        raise ValueError("_rod_between is intentionally limited to constant-Y tubes")
    theta_y = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) / 2.0, p0[1], (p0[2] + p1[2]) / 2.0),
            rpy=(0.0, theta_y, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_stationary_bike")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.019, 1.0))
    graphite = model.material("graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.82, 0.84, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.01, 0.01, 0.011, 1.0))
    datum_blue = model.material("datum_blue", rgba=(0.05, 0.23, 0.85, 1.0))
    index_white = model.material("index_white", rgba=(0.92, 0.92, 0.86, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(1.0, 0.74, 0.08, 1.0))

    crank_x = 0.12
    crank_z = 0.47
    saddle_x = -0.42
    saddle_joint_z = 0.81
    bar_x = 0.55
    bar_joint_z = 1.05

    frame = model.part("base_frame")

    # Floor reference rectangle with four leveling pads.  The wide, flat rails
    # make the model read as a calibration rig rather than a consumer trainer.
    for y in (-0.28, 0.28):
        frame.visual(
            Box((1.38, 0.055, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.037)),
            material=graphite,
            name=f"floor_rail_{'neg' if y < 0 else 'pos'}",
        )
    for x in (-0.56, 0.56):
        frame.visual(
            Box((0.075, 0.63, 0.045)),
            origin=Origin(xyz=(x, 0.0, 0.037)),
            material=graphite,
            name=f"floor_crossbar_{'rear' if x < 0 else 'front'}",
        )
        frame.visual(
            Box((0.075, 0.080, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=graphite,
            name=f"floor_bridge_{'rear' if x < 0 else 'front'}",
        )
    for x in (-0.61, 0.61):
        for y in (-0.28, 0.28):
            frame.visual(
                Cylinder(radius=0.042, length=0.018),
                origin=Origin(xyz=(x, y, 0.009)),
                material=rubber,
                name=f"leveling_foot_{'rear' if x < 0 else 'front'}_{'neg' if y < 0 else 'pos'}",
            )
            frame.visual(
                Box((0.052, 0.014, 0.004)),
                origin=Origin(xyz=(x, y + (0.030 if y < 0 else -0.030), 0.021)),
                material=index_white,
                name=f"foot_witness_{'rear' if x < 0 else 'front'}_{'neg' if y < 0 else 'pos'}",
            )

    frame.visual(
        Box((1.18, 0.075, 0.038)),
        origin=Origin(xyz=(0.02, 0.0, 0.082)),
        material=satin_steel,
        name="center_datum_beam",
    )

    # Welded frame tube stack.  The tubes intentionally overlap at the nodes,
    # matching welded steel construction and keeping the root link connected.
    _rod_between(frame, (-0.58, 0.0, 0.115), (0.64, 0.0, 0.115), 0.024, matte_black, "spine_tube")
    _rod_between(frame, (saddle_x, 0.0, 0.115), (saddle_x, 0.0, 0.395), 0.030, matte_black, "seat_lower_tube")
    _rod_between(frame, (bar_x, 0.0, 0.115), (bar_x, 0.0, 0.650), 0.030, matte_black, "bar_lower_tube")
    _rod_between(frame, (saddle_x, -0.055, 0.405), (crank_x - 0.180, -0.055, crank_z + 0.060), 0.026, matte_black, "rear_stay")
    _rod_between(frame, (bar_x, -0.070, 0.650), (crank_x + 0.180, -0.070, crank_z + 0.180), 0.029, matte_black, "front_down_tube")
    _rod_between(frame, (saddle_x, -0.058, 0.765), (bar_x, -0.058, 0.820), 0.023, matte_black, "top_datum_tube")
    _rod_between(frame, (crank_x, 0.0, 0.125), (crank_x, 0.0, crank_z - 0.100), 0.030, matte_black, "flywheel_pedestal")

    # Hollow flywheel side cover and a hollow bearing ring.  The center bore is
    # real clearance for the articulated crank spindle, not a hidden overlap.
    frame.visual(
        _annular_mesh(0.320, 0.075, 0.180, "flywheel_case_mesh"),
        origin=Origin(xyz=(crank_x, 0.0, crank_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="flywheel_case",
    )
    frame.visual(
        _annular_mesh(0.083, 0.022, 0.245, "bearing_ring_mesh"),
        origin=Origin(xyz=(crank_x, 0.0, crank_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="bearing_ring",
    )
    frame.visual(
        Box((0.230, 0.120, 0.012)),
        origin=Origin(xyz=(crank_x, 0.0, crank_z + 0.315)),
        material=datum_blue,
        name="flywheel_top_datum",
    )
    frame.visual(
        Box((0.105, 0.010, 0.050)),
        origin=Origin(xyz=(crank_x + 0.130, 0.093, crank_z + 0.005)),
        material=datum_blue,
        name="side_datum_pad",
    )
    for i, z in enumerate((crank_z - 0.20, crank_z - 0.10, crank_z, crank_z + 0.10, crank_z + 0.20)):
        frame.visual(
            Box((0.046, 0.006, 0.004)),
            origin=Origin(xyz=(crank_x - 0.230, 0.089, z)),
            material=index_white,
            name=f"flywheel_index_{i}",
        )

    # Hollow saddle and bar sleeves with external clamp bosses.
    frame.visual(
        _annular_mesh(0.041, 0.026, 0.400, "saddle_sleeve_mesh"),
        origin=Origin(xyz=(saddle_x, 0.0, 0.585)),
        material=satin_steel,
        name="saddle_sleeve",
    )
    frame.visual(
        _annular_mesh(0.056, 0.028, 0.080, "saddle_collar_mesh"),
        origin=Origin(xyz=(saddle_x, 0.0, 0.770)),
        material=brushed_aluminum,
        name="saddle_collar",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(saddle_x, 0.074, 0.770), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="saddle_clamp_boss",
    )
    frame.visual(
        Box((0.006, 0.050, 0.008)),
        origin=Origin(xyz=(saddle_x + 0.057, 0.0, saddle_joint_z)),
        material=warning_yellow,
        name="saddle_gap_gauge",
    )

    frame.visual(
        _annular_mesh(0.043, 0.027, 0.400, "bar_sleeve_mesh"),
        origin=Origin(xyz=(bar_x, 0.0, 0.825)),
        material=satin_steel,
        name="bar_sleeve",
    )
    frame.visual(
        _annular_mesh(0.058, 0.029, 0.080, "bar_collar_mesh"),
        origin=Origin(xyz=(bar_x, 0.0, 1.010)),
        material=brushed_aluminum,
        name="bar_collar",
    )
    frame.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(bar_x, 0.076, 1.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="bar_clamp_boss",
    )
    frame.visual(
        Box((0.006, 0.052, 0.008)),
        origin=Origin(xyz=(bar_x + 0.059, 0.0, bar_joint_z)),
        material=warning_yellow,
        name="bar_gap_gauge",
    )

    # Continuously rotating crank set on the explicit bearing ring.
    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.018, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="axle",
    )
    for side, y, z_sign, boss_name in (
        ("pos", 0.154, -1.0, "pedal_boss_pos"),
        ("neg", -0.154, 1.0, "pedal_boss_neg"),
    ):
        crank.visual(
            Cylinder(radius=0.055, length=0.026),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_aluminum,
            name=f"spider_{side}",
        )
        crank.visual(
            Box((0.040, 0.022, 0.330)),
            origin=Origin(xyz=(0.0, y, z_sign * 0.158)),
            material=brushed_aluminum,
            name=f"crank_arm_{side}",
        )
        crank.visual(
            Cylinder(radius=0.028, length=0.030),
            origin=Origin(xyz=(0.0, y, z_sign * 0.320), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_aluminum,
            name=boss_name,
        )
    crank.visual(
        _annular_mesh(0.142, 0.108, 0.012, "chainring_mesh"),
        origin=Origin(xyz=(0.0, 0.142, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="calibration_chainring",
    )
    for i in range(5):
        angle = 2.0 * math.pi * i / 5.0
        p0 = (0.052 * math.cos(angle), 0.142, 0.052 * math.sin(angle))
        p1 = (0.121 * math.cos(angle), 0.142, 0.121 * math.sin(angle))
        _rod_between(crank, p0, p1, 0.0065, satin_steel, f"chainring_spoke_{i}")
    crank.visual(
        Box((0.040, 0.004, 0.006)),
        origin=Origin(xyz=(0.145, 0.150, 0.0)),
        material=index_white,
        name="crank_zero_mark",
    )

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(crank_x, 0.0, crank_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.15, friction=0.02),
    )

    # Pedal blocks are separate revolute parts.  They are mounted on the crank
    # bosses at the outer faces, so the kinematic chain remains explicit.
    pedal_pos = model.part("pedal_pos")
    pedal_pos.visual(
        Cylinder(radius=0.010, length=0.054),
        origin=Origin(xyz=(0.0, 0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="spindle",
    )
    pedal_pos.visual(
        Box((0.160, 0.070, 0.032)),
        origin=Origin(xyz=(0.0, 0.088, 0.0)),
        material=rubber,
        name="pedal_block",
    )
    pedal_pos.visual(
        Box((0.130, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.126, 0.018)),
        material=index_white,
        name="pedal_datum_line",
    )
    model.articulation(
        "pedal_pos_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_pos,
        origin=Origin(xyz=(0.0, 0.169, -0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.01),
    )

    pedal_neg = model.part("pedal_neg")
    pedal_neg.visual(
        Cylinder(radius=0.010, length=0.054),
        origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="spindle",
    )
    pedal_neg.visual(
        Box((0.160, 0.070, 0.032)),
        origin=Origin(xyz=(0.0, -0.088, 0.0)),
        material=rubber,
        name="pedal_block",
    )
    pedal_neg.visual(
        Box((0.130, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.126, 0.018)),
        material=index_white,
        name="pedal_datum_line",
    )
    model.articulation(
        "pedal_neg_spin",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_neg,
        origin=Origin(xyz=(0.0, -0.169, 0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=20.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.01),
    )

    # Saddle height adjuster.  The post has retained hidden insertion through
    # the full travel and carries the saddle, rails, datum plate, and index marks.
    saddle_post = model.part("saddle_post")
    saddle_post.visual(
        Cylinder(radius=0.020, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin_steel,
        name="post_tube",
    )
    for i, z in enumerate((0.040, 0.075, 0.110, 0.145, 0.180)):
        saddle_post.visual(
            Box((0.006, 0.036, 0.004)),
            origin=Origin(xyz=(0.021, 0.0, z)),
            material=index_white,
            name=f"height_index_{i}",
        )
    saddle_post.visual(
        Box((0.092, 0.082, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=brushed_aluminum,
        name="saddle_clamp_head",
    )
    saddle_post.visual(
        Box((0.220, 0.105, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.315)),
        material=satin_steel,
        name="saddle_datum_plate",
    )
    for y in (-0.040, 0.040):
        saddle_post.visual(
            Cylinder(radius=0.007, length=0.255),
            origin=Origin(xyz=(-0.005, y, 0.302), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"saddle_rail_{'neg' if y < 0 else 'pos'}",
        )
    saddle_post.visual(
        Box((0.235, 0.180, 0.038)),
        origin=Origin(xyz=(-0.055, 0.0, 0.336)),
        material=rubber,
        name="saddle_wide_pad",
    )
    saddle_post.visual(
        Box((0.200, 0.085, 0.032)),
        origin=Origin(xyz=(0.085, 0.0, 0.336)),
        material=rubber,
        name="saddle_nose",
    )
    saddle_post.visual(
        Box((0.006, 0.055, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.005)),
        material=warning_yellow,
        name="post_zero_line",
    )
    model.articulation(
        "saddle_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle_post,
        origin=Origin(xyz=(saddle_x, 0.0, saddle_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.12, lower=0.0, upper=0.160),
        motion_properties=MotionProperties(damping=8.0, friction=2.0),
    )

    # Handlebar height adjuster with a broad transverse datum bar.
    bar_post = model.part("bar_post")
    bar_post.visual(
        Cylinder(radius=0.020, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=satin_steel,
        name="post_tube",
    )
    for i, z in enumerate((0.035, 0.070, 0.105, 0.140, 0.175)):
        bar_post.visual(
            Box((0.006, 0.038, 0.004)),
            origin=Origin(xyz=(0.021, 0.0, z)),
            material=index_white,
            name=f"height_index_{i}",
        )
    bar_post.visual(
        Box((0.082, 0.082, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=brushed_aluminum,
        name="bar_clamp_head",
    )
    bar_post.visual(
        Box((0.145, 0.360, 0.018)),
        origin=Origin(xyz=(0.040, 0.0, 0.295)),
        material=datum_blue,
        name="bar_datum_plate",
    )
    bar_post.visual(
        Box((0.070, 0.070, 0.060)),
        origin=Origin(xyz=(0.040, 0.0, 0.307)),
        material=brushed_aluminum,
        name="bar_stem_bridge",
    )
    bar_post.visual(
        Cylinder(radius=0.020, length=0.620),
        origin=Origin(xyz=(0.060, 0.0, 0.330), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="crossbar",
    )
    for y in (-0.325, 0.325):
        bar_post.visual(
            Cylinder(radius=0.027, length=0.145),
            origin=Origin(xyz=(0.060, y, 0.330), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{'neg' if y < 0 else 'pos'}",
        )
    bar_post.visual(
        Box((0.006, 0.055, 0.006)),
        origin=Origin(xyz=(0.018, 0.0, 0.005)),
        material=warning_yellow,
        name="post_zero_line",
    )
    model.articulation(
        "bar_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=bar_post,
        origin=Origin(xyz=(bar_x, 0.0, bar_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.12, lower=0.0, upper=0.140),
        motion_properties=MotionProperties(damping=8.0, friction=2.0),
    )

    # Clamp knobs rotate on explicit bosses instead of floating near the posts.
    saddle_knob = model.part("saddle_knob")
    saddle_knob.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="shaft",
    )
    saddle_knob.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.0, 0.054, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="handwheel",
    )
    saddle_knob.visual(
        Box((0.040, 0.004, 0.006)),
        origin=Origin(xyz=(0.000, 0.070, 0.018)),
        material=index_white,
        name="knob_index",
    )
    model.articulation(
        "saddle_knob_turn",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=saddle_knob,
        origin=Origin(xyz=(saddle_x, 0.109, 0.770)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0, lower=-2.0 * math.pi, upper=2.0 * math.pi),
    )

    bar_knob = model.part("bar_knob")
    bar_knob.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="shaft",
    )
    bar_knob.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.0, 0.054, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="handwheel",
    )
    bar_knob.visual(
        Box((0.040, 0.004, 0.006)),
        origin=Origin(xyz=(0.000, 0.070, 0.018)),
        material=index_white,
        name="knob_index",
    )
    model.articulation(
        "bar_knob_turn",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bar_knob,
        origin=Origin(xyz=(bar_x, 0.111, 1.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=6.0, lower=-2.0 * math.pi, upper=2.0 * math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("base_frame")
    crank = object_model.get_part("crank")
    saddle_post = object_model.get_part("saddle_post")
    bar_post = object_model.get_part("bar_post")
    saddle_knob = object_model.get_part("saddle_knob")
    bar_knob = object_model.get_part("bar_knob")
    pedal_pos = object_model.get_part("pedal_pos")
    pedal_neg = object_model.get_part("pedal_neg")

    crank_spin = object_model.get_articulation("crank_spin")
    saddle_height = object_model.get_articulation("saddle_height")
    bar_height = object_model.get_articulation("bar_height")

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="bearing_ring",
        elem_b="axle",
        reason="The crank axle is intentionally captured through the bearing-ring proxy bore.",
    )
    ctx.allow_overlap(
        frame,
        crank,
        elem_a="flywheel_case",
        elem_b="axle",
        reason="The axle intentionally passes through the side-cover bore of the flywheel housing proxy.",
    )
    ctx.allow_overlap(
        frame,
        saddle_post,
        elem_a="saddle_sleeve",
        elem_b="post_tube",
        reason="The saddle post is intentionally retained inside the hollow calibration sleeve.",
    )
    ctx.allow_overlap(
        frame,
        saddle_post,
        elem_a="saddle_collar",
        elem_b="post_tube",
        reason="The saddle post intentionally passes through the clamp collar bore.",
    )
    ctx.allow_overlap(
        frame,
        bar_post,
        elem_a="bar_sleeve",
        elem_b="post_tube",
        reason="The handlebar post is intentionally retained inside the hollow calibration sleeve.",
    )
    ctx.allow_overlap(
        frame,
        bar_post,
        elem_a="bar_collar",
        elem_b="post_tube",
        reason="The handlebar post intentionally passes through the clamp collar bore.",
    )

    # Bearing, pedals, and clamp knobs are physically mounted on named features.
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="bearing_ring",
        margin=0.0,
        name="crank axle centered in bearing ring",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_ring",
        min_overlap=0.220,
        name="crank axle spans bearing width",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="flywheel_case",
        margin=0.0,
        name="crank axle clearanced through flywheel case",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="flywheel_case",
        min_overlap=0.160,
        name="crank axle passes through flywheel housing",
    )
    ctx.expect_contact(
        pedal_pos,
        crank,
        elem_a="spindle",
        elem_b="pedal_boss_pos",
        contact_tol=0.003,
        name="positive pedal mounted to crank boss",
    )
    ctx.expect_contact(
        pedal_neg,
        crank,
        elem_a="spindle",
        elem_b="pedal_boss_neg",
        contact_tol=0.003,
        name="negative pedal mounted to crank boss",
    )
    ctx.expect_contact(
        saddle_knob,
        frame,
        elem_a="shaft",
        elem_b="saddle_clamp_boss",
        contact_tol=0.002,
        name="saddle knob shaft seats on clamp boss",
    )
    ctx.expect_contact(
        bar_knob,
        frame,
        elem_a="shaft",
        elem_b="bar_clamp_boss",
        contact_tol=0.002,
        name="bar knob shaft seats on clamp boss",
    )

    # Prismatic adjusters are retained in their sleeves at rest and at full
    # calibration travel.
    ctx.expect_within(
        saddle_post,
        frame,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="saddle_sleeve",
        margin=0.0,
        name="saddle post centered in sleeve",
    )
    ctx.expect_overlap(
        saddle_post,
        frame,
        axes="z",
        elem_a="post_tube",
        elem_b="saddle_sleeve",
        min_overlap=0.180,
        name="saddle post retained at low setting",
    )
    ctx.expect_within(
        saddle_post,
        frame,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="saddle_collar",
        margin=0.0,
        name="saddle post passes through clamp collar",
    )
    ctx.expect_overlap(
        saddle_post,
        frame,
        axes="z",
        elem_a="post_tube",
        elem_b="saddle_collar",
        min_overlap=0.070,
        name="saddle collar surrounds post",
    )
    ctx.expect_within(
        bar_post,
        frame,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="bar_sleeve",
        margin=0.0,
        name="bar post centered in sleeve",
    )
    ctx.expect_overlap(
        bar_post,
        frame,
        axes="z",
        elem_a="post_tube",
        elem_b="bar_sleeve",
        min_overlap=0.180,
        name="bar post retained at low setting",
    )
    ctx.expect_within(
        bar_post,
        frame,
        axes="xy",
        inner_elem="post_tube",
        outer_elem="bar_collar",
        margin=0.0,
        name="bar post passes through clamp collar",
    )
    ctx.expect_overlap(
        bar_post,
        frame,
        axes="z",
        elem_a="post_tube",
        elem_b="bar_collar",
        min_overlap=0.070,
        name="bar collar surrounds post",
    )

    saddle_low = ctx.part_world_position(saddle_post)
    with ctx.pose({saddle_height: 0.160}):
        ctx.expect_within(
            saddle_post,
            frame,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="saddle_sleeve",
            margin=0.0,
            name="saddle post stays centered at high setting",
        )
        ctx.expect_overlap(
            saddle_post,
            frame,
            axes="z",
            elem_a="post_tube",
            elem_b="saddle_sleeve",
            min_overlap=0.040,
            name="saddle post retained at high setting",
        )
        saddle_high = ctx.part_world_position(saddle_post)
    ctx.check(
        "saddle height adjuster moves upward",
        saddle_low is not None and saddle_high is not None and saddle_high[2] > saddle_low[2] + 0.150,
        details=f"low={saddle_low}, high={saddle_high}",
    )

    bar_low = ctx.part_world_position(bar_post)
    with ctx.pose({bar_height: 0.140}):
        ctx.expect_within(
            bar_post,
            frame,
            axes="xy",
            inner_elem="post_tube",
            outer_elem="bar_sleeve",
            margin=0.0,
            name="bar post stays centered at high setting",
        )
        ctx.expect_overlap(
            bar_post,
            frame,
            axes="z",
            elem_a="post_tube",
            elem_b="bar_sleeve",
            min_overlap=0.055,
            name="bar post retained at high setting",
        )
        bar_high = ctx.part_world_position(bar_post)
    ctx.check(
        "bar height adjuster moves upward",
        bar_low is not None and bar_high is not None and bar_high[2] > bar_low[2] + 0.130,
        details=f"low={bar_low}, high={bar_high}",
    )

    pedal_low = ctx.part_world_position(pedal_pos)
    with ctx.pose({crank_spin: math.pi / 2.0}):
        pedal_quarter = ctx.part_world_position(pedal_pos)
    ctx.check(
        "crank rotation carries pedal around bearing axis",
        pedal_low is not None
        and pedal_quarter is not None
        and pedal_quarter[2] > pedal_low[2] + 0.25
        and pedal_quarter[0] < pedal_low[0] - 0.25,
        details=f"low={pedal_low}, quarter_turn={pedal_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
