from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _profile_translate(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _profile_transform(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _hollow_tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length * 0.5))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _flywheel_cover_mesh():
    outer = superellipse_profile(0.54, 0.55, exponent=2.35, segments=72)
    cover = ExtrudeGeometry(outer, 0.18, center=True).rotate_x(pi / 2.0).translate(0.50, 0.0, 0.45)
    return _mesh(cover, "flywheel_cover")


def _flywheel_seam_mesh(width: float, height: float, y: float, name: str):
    profile = superellipse_profile(width, height, exponent=2.6, segments=72)
    seam = ExtrudeGeometry(profile, 0.006, center=True).rotate_x(pi / 2.0).translate(0.50, y, 0.45)
    return _mesh(seam, name)


def _chainring_mesh():
    outer = superellipse_profile(0.25, 0.25, exponent=2.0, segments=96)
    holes: list[list[tuple[float, float]]] = [
        superellipse_profile(0.115, 0.115, exponent=2.0, segments=64)
    ]
    slot = rounded_rect_profile(0.030, 0.075, 0.010, corner_segments=8)
    for i in range(5):
        a = 2.0 * pi * i / 5.0 + pi / 2.0
        holes.append(_profile_transform(slot, dx=cos(a) * 0.074, dy=sin(a) * 0.074, angle=a))
    ring = ExtrudeWithHolesGeometry(outer, holes, height=0.012, center=True)
    ring.rotate_x(pi / 2.0).translate(0.0, -0.108, 0.0)
    return _mesh(ring, "chainring")


def _pedal_body_mesh(sign: float, name: str):
    outline = rounded_rect_profile(0.120, 0.070, 0.015, corner_segments=8)
    body = ExtrudeGeometry(outline, 0.020, center=True).translate(0.0, 0.0, -0.008)
    # The extrusion is through local Z, so this creates a thin horizontal pedal pad.
    body.translate(0.0, sign * 0.092, 0.0)
    return _mesh(body, name)


def _saddle_mesh():
    outline = [
        (-0.185, -0.060),
        (-0.150, -0.105),
        (-0.040, -0.122),
        (0.112, -0.104),
        (0.178, -0.060),
        (0.178, 0.060),
        (0.112, 0.104),
        (-0.040, 0.122),
        (-0.150, 0.105),
        (-0.185, 0.060),
    ]
    saddle = ExtrudeGeometry(outline, 0.070, center=True).translate(-0.015, 0.0, 0.310)
    return _mesh(saddle, "saddle_cushion")


def _clamp_knob_mesh(name: str):
    knob = KnobGeometry(
        0.054,
        0.030,
        body_style="lobed",
        base_diameter=0.035,
        top_diameter=0.048,
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=10, depth=0.0010),
        bore=KnobBore(style="round", diameter=0.008),
    )
    return mesh_from_geometry(knob, name)


def _add_frame_visuals(
    frame,
    *,
    paint: Material,
    polymer: Material,
    dark: Material,
    rubber: Material,
    trim: Material,
) -> None:
    # Stabilizers and floor contact elastomer.
    frame.visual(
        Cylinder(radius=0.045, length=0.76),
        origin=Origin(xyz=(-0.62, 0.0, 0.055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="rear_stabilizer",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.70),
        origin=Origin(xyz=(0.72, 0.0, 0.055), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_stabilizer",
    )
    for x in (-0.62, 0.72):
        for y in (-0.33, 0.33):
            frame.visual(
                Box((0.15, 0.075, 0.030)),
                origin=Origin(xyz=(x, y, 0.027)),
                material=rubber,
                name=f"rubber_foot_{x}_{y}",
            )

    # Premium enclosed flywheel casing with understated seam strategy.
    frame.visual(_flywheel_cover_mesh(), material=polymer, name="flywheel_cover")
    frame.visual(_flywheel_seam_mesh(0.485, 0.500, -0.093, "near_side_seam"), material=trim, name="near_side_seam")
    frame.visual(_flywheel_seam_mesh(0.485, 0.500, 0.093, "far_side_seam"), material=trim, name="far_side_seam")
    frame.visual(
        Cylinder(radius=0.105, length=0.010),
        origin=Origin(xyz=(0.50, -0.100, 0.45), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="flywheel_recess_0",
    )
    frame.visual(
        Cylinder(radius=0.105, length=0.010),
        origin=Origin(xyz=(0.50, 0.100, 0.45), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="flywheel_recess_1",
    )
    frame.visual(
        Cylinder(radius=0.070, length=0.014),
        origin=Origin(xyz=(0.50, -0.108, 0.45), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="flywheel_hub_trim",
    )

    # Welded / hydroformed painted-metal frame paths. These intentionally
    # overlap at nodes so the frame reads as one supported structure.
    for name, points, radius in [
        (
            "base_spine",
            [(-0.64, 0.0, 0.075), (-0.28, 0.0, 0.082), (0.28, 0.0, 0.080), (0.72, 0.0, 0.075)],
            0.030,
        ),
        (
            "main_sweep",
            [(-0.56, 0.0, 0.095), (-0.25, 0.0, 0.230), (0.04, 0.0, 0.370), (0.33, 0.0, 0.555), (0.50, 0.0, 0.495)],
            0.034,
        ),
        (
            "seat_tube",
            [(-0.45, 0.0, 0.085), (-0.42, 0.0, 0.350), (-0.420, 0.0, 0.680)],
            0.032,
        ),
        (
            "bar_tube",
            [(0.52, 0.050, 0.565), (0.43, 0.065, 0.770), (0.35, 0.065, 0.855)],
            0.032,
        ),
        (
            "near_side_stay",
            [(-0.33, -0.075, 0.635), (-0.06, -0.085, 0.540), (0.32, -0.070, 0.550)],
            0.019,
        ),
        (
            "far_side_stay",
            [(-0.33, 0.075, 0.635), (-0.06, 0.085, 0.540), (0.32, 0.070, 0.550)],
            0.019,
        ),
    ]:
        frame.visual(
            _mesh(
                tube_from_spline_points(
                    points,
                    radius=radius,
                    samples_per_segment=14,
                    radial_segments=20,
                    cap_ends=True,
                ),
                name,
            ),
            material=paint,
            name=name,
        )

    # Bottom bracket bearing shell and external bearing caps for the crank.
    frame.visual(
        Cylinder(radius=0.068, length=0.170),
        origin=Origin(xyz=(0.04, 0.0, 0.465), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="bottom_bearing_shell",
    )
    frame.visual(
        Cylinder(radius=0.082, length=0.018),
        origin=Origin(xyz=(0.04, -0.091, 0.465), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="near_bearing_cap",
    )
    frame.visual(
        Cylinder(radius=0.082, length=0.018),
        origin=Origin(xyz=(0.04, 0.091, 0.465), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="far_bearing_cap",
    )

    # Hollow post sleeves plus clamp collars/bosses.
    frame.visual(
        _hollow_tube_mesh(0.036, 0.024, 0.300, "seat_outer_sleeve"),
        origin=Origin(xyz=(-0.36, 0.0, 0.680)),
        material=paint,
        name="seat_outer_sleeve",
    )
    frame.visual(
        _hollow_tube_mesh(0.047, 0.026, 0.052, "seat_clamp_collar"),
        origin=Origin(xyz=(-0.36, 0.0, 0.828)),
        material=dark,
        name="seat_clamp_collar",
    )
    frame.visual(
        Box((0.080, 0.078, 0.062)),
        origin=Origin(xyz=(-0.36, -0.064, 0.828)),
        material=dark,
        name="seat_clamp_boss",
    )
    frame.visual(
        Box((0.010, 0.006, 0.060)),
        origin=Origin(xyz=(-0.398, -0.105, 0.828)),
        material=trim,
        name="seat_clamp_split",
    )
    frame.visual(
        Box((0.030, 0.009, 0.052)),
        origin=Origin(xyz=(-0.36, -0.0230, 0.828)),
        material=trim,
        name="seat_pressure_pad",
    )

    frame.visual(
        _hollow_tube_mesh(0.034, 0.023, 0.320, "bar_outer_sleeve"),
        origin=Origin(xyz=(0.35, 0.0, 0.820)),
        material=paint,
        name="bar_outer_sleeve",
    )
    frame.visual(
        _hollow_tube_mesh(0.045, 0.025, 0.052, "bar_clamp_collar"),
        origin=Origin(xyz=(0.35, 0.0, 0.978)),
        material=dark,
        name="bar_clamp_collar",
    )
    frame.visual(
        Box((0.078, 0.076, 0.060)),
        origin=Origin(xyz=(0.35, -0.062, 0.978)),
        material=dark,
        name="bar_clamp_boss",
    )
    frame.visual(
        Box((0.010, 0.006, 0.058)),
        origin=Origin(xyz=(0.386, -0.103, 0.978)),
        material=trim,
        name="bar_clamp_split",
    )
    frame.visual(
        Box((0.030, 0.009, 0.052)),
        origin=Origin(xyz=(0.35, -0.0225, 0.978)),
        material=trim,
        name="bar_pressure_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stationary_bike")

    paint = model.material("satin_graphite_paint", rgba=(0.13, 0.145, 0.15, 1.0))
    polymer = model.material("warm_grey_polymer", rgba=(0.22, 0.225, 0.22, 1.0))
    dark = model.material("black_anodized_metal", rgba=(0.055, 0.060, 0.062, 1.0))
    trim = model.material("brushed_champagne_trim", rgba=(0.72, 0.66, 0.54, 1.0))
    rubber = model.material("matte_black_elastomer", rgba=(0.018, 0.018, 0.016, 1.0))
    saddle_vinyl = model.material("soft_black_vinyl", rgba=(0.025, 0.024, 0.023, 1.0))

    frame = model.part("frame")
    _add_frame_visuals(frame, paint=paint, polymer=polymer, dark=dark, rubber=rubber, trim=trim)

    saddle_post = model.part("saddle_post")
    saddle_post.visual(
        Cylinder(radius=0.0185, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=trim,
        name="seat_inner_post",
    )
    saddle_post.visual(
        Box((0.180, 0.090, 0.022)),
        origin=Origin(xyz=(-0.028, 0.0, 0.276)),
        material=dark,
        name="saddle_rail_plate",
    )
    saddle_post.visual(
        Cylinder(radius=0.008, length=0.250),
        origin=Origin(xyz=(-0.030, -0.035, 0.287), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim,
        name="saddle_rail_0",
    )
    saddle_post.visual(
        Cylinder(radius=0.008, length=0.250),
        origin=Origin(xyz=(-0.030, 0.035, 0.287), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim,
        name="saddle_rail_1",
    )
    saddle_post.visual(_saddle_mesh(), material=saddle_vinyl, name="saddle_cushion")

    bar_mast = model.part("bar_mast")
    bar_mast.visual(
        Cylinder(radius=0.018, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=trim,
        name="handle_inner_mast",
    )
    bar_mast.visual(
        _mesh(
            tube_from_spline_points(
                [(0.0, 0.0, 0.250), (0.035, 0.0, 0.335), (0.075, 0.0, 0.420)],
                radius=0.019,
                samples_per_segment=12,
                radial_segments=20,
            ),
            "handle_stem",
        ),
        material=dark,
        name="handle_stem",
    )
    bar_mast.visual(
        _mesh(
            tube_from_spline_points(
                [
                    (0.020, -0.310, 0.390),
                    (0.065, -0.185, 0.425),
                    (0.080, 0.000, 0.438),
                    (0.065, 0.185, 0.425),
                    (0.020, 0.310, 0.390),
                ],
                radius=0.016,
                samples_per_segment=16,
                radial_segments=20,
            ),
            "handlebar",
        ),
        material=dark,
        name="handlebar",
    )
    for y in (-0.345, 0.345):
        bar_mast.visual(
            Cylinder(radius=0.023, length=0.105),
            origin=Origin(xyz=(0.020, y, 0.390), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{y}",
        )
    bar_mast.visual(
        Box((0.155, 0.050, 0.070)),
        origin=Origin(xyz=(0.108, 0.0, 0.410)),
        material=polymer,
        name="display_pod",
    )
    bar_mast.visual(
        Box((0.105, 0.010, 0.043)),
        origin=Origin(xyz=(0.132, -0.026, 0.418)),
        material=dark,
        name="display_glass",
    )

    crankset = model.part("crankset")
    crankset.visual(
        Cylinder(radius=0.023, length=0.214),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="spindle",
    )
    crankset.visual(_chainring_mesh(), material=trim, name="chainring")
    crankset.visual(
        Box((0.032, 0.018, 0.330)),
        origin=Origin(xyz=(0.0, -0.112, -0.082)),
        material=dark,
        name="near_crank_arm",
    )
    crankset.visual(
        Box((0.032, 0.018, 0.330)),
        origin=Origin(xyz=(0.0, 0.112, 0.082)),
        material=dark,
        name="far_crank_arm",
    )
    crankset.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, -0.145, -0.170), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pedal_boss_0",
    )
    crankset.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, 0.145, 0.170), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="pedal_boss_1",
    )

    pedal_0 = model.part("pedal_0")
    pedal_0.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="pedal_axle",
    )
    pedal_0.visual(_pedal_body_mesh(-1.0, "pedal_pad_0"), material=rubber, name="pedal_pad")
    pedal_0.visual(
        Box((0.110, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.092, 0.006)),
        material=trim,
        name="pedal_tread_bar",
    )

    pedal_1 = model.part("pedal_1")
    pedal_1.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="pedal_axle",
    )
    pedal_1.visual(_pedal_body_mesh(1.0, "pedal_pad_1"), material=rubber, name="pedal_pad")
    pedal_1.visual(
        Box((0.110, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.092, 0.006)),
        material=trim,
        name="pedal_tread_bar",
    )

    seat_knob = model.part("seat_knob")
    seat_knob.visual(
        Cylinder(radius=0.0065, length=0.126),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="seat_screw",
    )
    seat_knob.visual(
        _clamp_knob_mesh("seat_knob_cap"),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="seat_knob_cap",
    )

    bar_knob = model.part("bar_knob")
    bar_knob.visual(
        Cylinder(radius=0.0065, length=0.124),
        origin=Origin(xyz=(0.0, -0.001, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="bar_screw",
    )
    bar_knob.visual(
        _clamp_knob_mesh("bar_knob_cap"),
        origin=Origin(xyz=(0.0, -0.068, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="bar_knob_cap",
    )

    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle_post,
        origin=Origin(xyz=(-0.36, 0.0, 0.830)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    model.articulation(
        "bar_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=bar_mast,
        origin=Origin(xyz=(0.35, 0.0, 0.980)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=130.0, velocity=0.25, lower=0.0, upper=0.120),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crankset,
        origin=Origin(xyz=(0.04, 0.0, 0.465)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=18.0),
    )
    model.articulation(
        "crank_to_pedal_0",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=pedal_0,
        origin=Origin(xyz=(0.0, -0.168, -0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        mimic=Mimic(joint="crank_spin", multiplier=-1.0),
    )
    model.articulation(
        "crank_to_pedal_1",
        ArticulationType.CONTINUOUS,
        parent=crankset,
        child=pedal_1,
        origin=Origin(xyz=(0.0, 0.168, 0.170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        mimic=Mimic(joint="crank_spin", multiplier=-1.0),
    )
    model.articulation(
        "seat_knob_turn",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=seat_knob,
        origin=Origin(xyz=(-0.36, -0.088, 0.828)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )
    model.articulation(
        "bar_knob_turn",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=bar_knob,
        origin=Origin(xyz=(0.35, -0.086, 0.978)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    saddle_post = object_model.get_part("saddle_post")
    bar_mast = object_model.get_part("bar_mast")
    crankset = object_model.get_part("crankset")
    pedal_0 = object_model.get_part("pedal_0")
    seat_knob = object_model.get_part("seat_knob")
    bar_knob = object_model.get_part("bar_knob")
    crank = object_model.get_articulation("crank_spin")
    seat_height = object_model.get_articulation("seat_height")
    bar_height = object_model.get_articulation("bar_height")

    ctx.allow_overlap(
        frame,
        crankset,
        elem_a="bottom_bearing_shell",
        elem_b="spindle",
        reason="The crank spindle is intentionally captured inside the explicit bottom-bracket bearing shell.",
    )
    ctx.expect_within(
        crankset,
        frame,
        axes="xz",
        inner_elem="spindle",
        outer_elem="bottom_bearing_shell",
        margin=0.001,
        name="spindle centered in bottom bracket shell",
    )
    ctx.expect_overlap(
        crankset,
        frame,
        axes="y",
        elem_a="spindle",
        elem_b="bottom_bearing_shell",
        min_overlap=0.12,
        name="spindle retained through bearing width",
    )
    for cap in ("near_bearing_cap", "far_bearing_cap"):
        ctx.allow_overlap(
            frame,
            crankset,
            elem_a=cap,
            elem_b="spindle",
            reason="The bottom-bracket bearing cap surrounds the captured crank spindle.",
        )
        ctx.expect_overlap(
            crankset,
            frame,
            axes="y",
            elem_a="spindle",
            elem_b=cap,
            min_overlap=0.010,
            name=f"spindle passes through {cap}",
        )

    ctx.allow_overlap(
        frame,
        seat_knob,
        elem_a="seat_clamp_boss",
        elem_b="seat_screw",
        reason="The seat adjuster screw is intentionally seated through the clamp boss.",
    )
    ctx.expect_within(
        seat_knob,
        frame,
        axes="xz",
        inner_elem="seat_screw",
        outer_elem="seat_clamp_boss",
        margin=0.002,
        name="seat screw passes through clamp boss",
    )
    ctx.expect_overlap(
        seat_knob,
        frame,
        axes="y",
        elem_a="seat_screw",
        elem_b="seat_clamp_boss",
        min_overlap=0.050,
        name="seat screw has threaded engagement",
    )
    ctx.allow_overlap(
        frame,
        seat_knob,
        elem_a="seat_clamp_collar",
        elem_b="seat_screw",
        reason="The seat adjuster screw passes through the clamp collar wall before bearing on the post.",
    )
    ctx.expect_overlap(
        seat_knob,
        frame,
        axes="y",
        elem_a="seat_screw",
        elem_b="seat_clamp_collar",
        min_overlap=0.015,
        name="seat screw crosses clamp collar wall",
    )
    ctx.allow_overlap(
        frame,
        seat_knob,
        elem_a="seat_outer_sleeve",
        elem_b="seat_screw",
        reason="The seat adjuster screw locally crosses the sleeve wall at the split clamp.",
    )
    ctx.expect_overlap(
        seat_knob,
        frame,
        axes="y",
        elem_a="seat_screw",
        elem_b="seat_outer_sleeve",
        min_overlap=0.006,
        name="seat screw reaches sleeve wall",
    )

    ctx.allow_overlap(
        frame,
        bar_knob,
        elem_a="bar_clamp_boss",
        elem_b="bar_screw",
        reason="The handlebar adjuster screw is intentionally seated through the clamp boss.",
    )
    ctx.expect_within(
        bar_knob,
        frame,
        axes="xz",
        inner_elem="bar_screw",
        outer_elem="bar_clamp_boss",
        margin=0.002,
        name="bar screw passes through clamp boss",
    )
    ctx.expect_overlap(
        bar_knob,
        frame,
        axes="y",
        elem_a="bar_screw",
        elem_b="bar_clamp_boss",
        min_overlap=0.048,
        name="bar screw has threaded engagement",
    )
    ctx.allow_overlap(
        frame,
        bar_knob,
        elem_a="bar_clamp_collar",
        elem_b="bar_screw",
        reason="The handlebar adjuster screw passes through the clamp collar wall before bearing on the mast.",
    )
    ctx.expect_overlap(
        bar_knob,
        frame,
        axes="y",
        elem_a="bar_screw",
        elem_b="bar_clamp_collar",
        min_overlap=0.015,
        name="bar screw crosses clamp collar wall",
    )
    ctx.allow_overlap(
        frame,
        bar_knob,
        elem_a="bar_outer_sleeve",
        elem_b="bar_screw",
        reason="The handlebar adjuster screw locally crosses the sleeve wall at the split clamp.",
    )
    ctx.expect_overlap(
        bar_knob,
        frame,
        axes="y",
        elem_a="bar_screw",
        elem_b="bar_outer_sleeve",
        min_overlap=0.006,
        name="bar screw reaches sleeve wall",
    )

    for joint, post, inner, outer, label in [
        (seat_height, saddle_post, "seat_inner_post", "seat_outer_sleeve", "saddle post"),
        (bar_height, bar_mast, "handle_inner_mast", "bar_outer_sleeve", "handlebar mast"),
    ]:
        ctx.expect_within(
            post,
            frame,
            axes="xy",
            inner_elem=inner,
            outer_elem=outer,
            margin=0.002,
            name=f"{label} centered in sleeve",
        )
        ctx.expect_overlap(
            post,
            frame,
            axes="z",
            elem_a=inner,
            elem_b=outer,
            min_overlap=0.14,
            name=f"{label} retained at low setting",
        )
        rest_pos = ctx.part_world_position(post)
        with ctx.pose({joint: 0.12}):
            ctx.expect_within(
                post,
                frame,
                axes="xy",
                inner_elem=inner,
                outer_elem=outer,
                margin=0.002,
                name=f"{label} centered at high setting",
            )
            ctx.expect_overlap(
                post,
                frame,
                axes="z",
                elem_a=inner,
                elem_b=outer,
                min_overlap=0.050,
                name=f"{label} retained at high setting",
            )
            high_pos = ctx.part_world_position(post)
        ctx.check(
            f"{label} moves upward on adjustment",
            rest_pos is not None and high_pos is not None and high_pos[2] > rest_pos[2] + 0.10,
            details=f"rest={rest_pos}, high={high_pos}",
        )

    ctx.expect_contact(
        saddle_post,
        frame,
        elem_a="seat_inner_post",
        elem_b="seat_pressure_pad",
        contact_tol=0.001,
        name="seat post is braced by clamp pressure pad",
    )
    ctx.expect_contact(
        bar_mast,
        frame,
        elem_a="handle_inner_mast",
        elem_b="bar_pressure_pad",
        contact_tol=0.001,
        name="handlebar mast is braced by clamp pressure pad",
    )

    rest_pedal = ctx.part_world_position(pedal_0)
    with ctx.pose({crank: pi / 2.0}):
        quarter_pedal = ctx.part_world_position(pedal_0)
    ctx.check(
        "crank rotation moves pedal around bottom bracket",
        rest_pedal is not None
        and quarter_pedal is not None
        and abs(quarter_pedal[0] - rest_pedal[0]) > 0.12
        and abs(quarter_pedal[2] - rest_pedal[2]) > 0.12,
        details=f"rest={rest_pedal}, quarter={quarter_pedal}",
    )

    return ctx.report()


object_model = build_object_model()
