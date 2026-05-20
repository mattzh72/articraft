from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _tube(part, name: str, points, radius: float, material, *, segments: int = 18) -> None:
    part.visual(
        _mesh(
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=14,
                radial_segments=segments,
                cap_ends=True,
            ),
            name,
        ),
        material=material,
        name=name,
    )


def _square_tube_mesh(name: str, outer: float, inner: float, height: float, *, radius: float = 0.008):
    return _mesh(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer, outer, radius, corner_segments=6),
            [rounded_rect_profile(inner, inner, radius * 0.55, corner_segments=6)],
            height,
            center=True,
        ),
        name,
    )


def _add_square_tube_boxes(part, prefix: str, center, outer: float, inner: float, height: float, material) -> None:
    x, y, z = center
    wall = (outer - inner) * 0.5
    side_span = inner + 2.0 * wall
    # Four wall boxes overlap at their corners, creating one real square sleeve
    # with an open, clearanced center for the moving post.
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(x, y + (inner + wall) * 0.5, z)),
        material=material,
        name=f"{prefix}_yp",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(x, y - (inner + wall) * 0.5, z)),
        material=material,
        name=f"{prefix}_yn",
    )
    part.visual(
        Box((wall, side_span, height)),
        origin=Origin(xyz=(x + (inner + wall) * 0.5, y, z)),
        material=material,
        name=f"{prefix}_xp",
    )
    part.visual(
        Box((wall, side_span, height)),
        origin=Origin(xyz=(x - (inner + wall) * 0.5, y, z)),
        material=material,
        name=f"{prefix}_xn",
    )


def _bearing_plate_mesh(name: str):
    return _mesh(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.17, 0.17, 0.018, corner_segments=8),
            [superellipse_profile(0.082, 0.082, exponent=2.0, segments=36)],
            0.020,
            center=True,
        ).rotate_x(pi / 2.0),
        name,
    )


def _side_torus(name: str, radius: float, tube: float, x: float, z: float):
    return _mesh(
        TorusGeometry(radius=radius, tube=tube, radial_segments=32, tubular_segments=72)
        .rotate_x(pi / 2.0)
        .translate(x, 0.0, z),
        name,
    )


def _lobed_knob_mesh(name: str, diameter: float = 0.070, height: float = 0.036):
    return _mesh(
        KnobGeometry(
            diameter,
            height,
            body_style="lobed",
            top_diameter=diameter * 0.86,
            base_diameter=diameter * 0.74,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=10, depth=0.0018, width=0.003),
            bore=KnobBore(style="round", diameter=0.010),
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_stationary_bike")

    frame_blue = model.material("powder_blue", rgba=(0.07, 0.17, 0.26, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    worn_steel = model.material("brushed_service_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    zinc = model.material("zinc_fasteners", rgba=(0.78, 0.77, 0.70, 1.0))
    rubber = model.material("service_rubber", rgba=(0.025, 0.025, 0.023, 1.0))
    vinyl = model.material("matte_saddle_vinyl", rgba=(0.055, 0.052, 0.050, 1.0))
    safety_yellow = model.material("service_yellow", rgba=(0.95, 0.68, 0.05, 1.0))
    translucent = model.material("smoked_polycarbonate", rgba=(0.25, 0.30, 0.33, 0.62))

    frame = model.part("frame")

    # Stable service base: two skid rails welded into chunky leveling crossbars.
    for x in (-0.64, 0.64):
        frame.visual(
            Cylinder(radius=0.038, length=0.86),
            origin=Origin(xyz=(x, 0.0, 0.075), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"crossbar_{'rear' if x < 0 else 'front'}",
        )
        for y in (-0.34, 0.34):
            frame.visual(
                Box((0.17, 0.13, 0.040)),
                origin=Origin(xyz=(x, y, 0.038)),
                material=rubber,
                name=f"rubber_leveler_{'rear' if x < 0 else 'front'}_{0 if y < 0 else 1}",
            )
    for y in (-0.34, 0.34):
        frame.visual(
            Cylinder(radius=0.030, length=1.30),
            origin=Origin(xyz=(0.0, y, 0.080), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"floor_skid_{0 if y < 0 else 1}",
        )
    frame.visual(
        Box((1.34, 0.90, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=dark_steel,
        name="base_tie_plate",
    )

    # Welded tube frame, deliberately thick with generous bracket overlaps.
    _tube(frame, "center_spine", [(-0.60, 0.0, 0.11), (-0.34, 0.0, 0.31), (-0.04, 0.0, 0.34), (0.16, 0.0, 0.24), (0.44, 0.0, 0.16), (0.62, 0.0, 0.12)], 0.030, frame_blue)
    _tube(frame, "seat_stay", [(-0.55, 0.0, 0.10), (-0.46, 0.0, 0.34), (-0.39, 0.0, 0.64)], 0.032, frame_blue)
    _tube(frame, "handle_stay", [(0.62, 0.0, 0.11), (0.66, 0.0, 0.36), (0.665, 0.0, 0.62)], 0.032, frame_blue)
    _tube(frame, "upper_truss", [(-0.42, 0.09, 0.73), (-0.10, 0.09, 0.78), (0.28, 0.09, 0.82), (0.66, 0.09, 0.78)], 0.026, frame_blue)
    frame.visual(Box((0.12, 0.060, 0.055)), origin=Origin(xyz=(-0.37, 0.090, 0.72)), material=frame_blue, name="seat_truss_tab")
    frame.visual(Box((0.12, 0.060, 0.055)), origin=Origin(xyz=(0.67, 0.090, 0.72)), material=frame_blue, name="bar_truss_tab")
    for y in (-0.44, 0.44):
        _tube(frame, f"side_lower_rail_{0 if y < 0 else 1}", [(-0.60, y, 0.10), (-0.18, y, 0.17), (0.30, y, 0.18), (0.62, y, 0.10)], 0.024, frame_blue)
        _tube(frame, f"flywheel_side_brace_{0 if y < 0 else 1}", [(0.36, y, 0.20), (0.36, y * 0.75, 0.42), (0.36, y, 0.64)], 0.020, frame_blue)

    # Flywheel guard/housing with access face, service lip, and bolted flange.
    frame.visual(_side_torus("flywheel_guard_ring", 0.245, 0.035, 0.42, 0.42), material=frame_blue, name="flywheel_guard_ring")
    for y, suffix in ((-0.042, "0"), (0.042, "1")):
        frame.visual(
            Cylinder(radius=0.116, length=0.018),
            origin=Origin(xyz=(0.42, y, 0.42), rpy=(pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"flywheel_bearing_face_{suffix}",
        )
        frame.visual(
            Box((0.060, 0.018, 0.110)),
            origin=Origin(xyz=(0.42, y, 0.590)),
            material=worn_steel,
            name=f"flywheel_bearing_tie_{suffix}",
        )
    frame.visual(
        Box((0.30, 0.016, 0.22)),
        origin=Origin(xyz=(0.36, -0.087, 0.44)),
        material=frame_blue,
        name="access_flange",
    )
    frame.visual(
        Box((0.23, 0.010, 0.15)),
        origin=Origin(xyz=(0.40, -0.096, 0.44)),
        material=translucent,
        name="inspection_window",
    )
    frame.visual(
        Box((0.34, 0.050, 0.26)),
        origin=Origin(xyz=(0.36, -0.060, 0.44)),
        material=frame_blue,
        name="service_web",
    )
    for z in (0.245, 0.595):
        frame.visual(
            Box((0.23, 0.070, 0.030)),
            origin=Origin(xyz=(0.36, 0.070, z)),
            material=dark_steel,
            name=f"guard_cross_tie_{0 if z < 0.4 else 1}",
        )
    frame.visual(Box((0.08, 0.64, 0.030)), origin=Origin(xyz=(0.36, 0.0, 0.185)), material=frame_blue, name="lower_guard_bridge")
    frame.visual(Box((0.08, 0.64, 0.030)), origin=Origin(xyz=(0.36, 0.0, 0.655)), material=frame_blue, name="upper_guard_bridge")
    # Fixed hinge leaves for the service door.
    frame.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.246, -0.100, 0.358)),
        material=zinc,
        name="frame_hinge_knuckle_0",
    )
    frame.visual(
        Box((0.026, 0.024, 0.070)),
        origin=Origin(xyz=(0.246, -0.087, 0.358)),
        material=zinc,
        name="frame_hinge_leaf_0",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.246, -0.100, 0.542)),
        material=zinc,
        name="frame_hinge_knuckle_1",
    )
    frame.visual(
        Box((0.026, 0.024, 0.070)),
        origin=Origin(xyz=(0.246, -0.087, 0.542)),
        material=zinc,
        name="frame_hinge_leaf_1",
    )

    # Crank bearing carrier: split side plates, open bearing rings, and gussets.
    frame.visual(
        _bearing_plate_mesh("crank_bearing_plate_0"),
        origin=Origin(xyz=(0.07, -0.075, 0.42)),
        material=dark_steel,
        name="crank_bearing_plate_0",
    )
    frame.visual(
        _bearing_plate_mesh("crank_bearing_plate_1"),
        origin=Origin(xyz=(0.07, 0.075, 0.42)),
        material=dark_steel,
        name="crank_bearing_plate_1",
    )
    frame.visual(_side_torus("crank_bearing_outer", 0.050, 0.011, 0.07, 0.42), material=worn_steel, name="crank_bearing_outer")
    frame.visual(Box((0.050, 0.160, 0.018)), origin=Origin(xyz=(0.07, 0.0, 0.475)), material=worn_steel, name="bearing_top_cap")
    for x in (-0.03, 0.17):
        frame.visual(
            Box((0.045, 0.18, 0.13)),
            origin=Origin(xyz=(x, 0.0, 0.36)),
            material=frame_blue,
            name=f"bearing_gusset_{0 if x < 0.1 else 1}",
        )

    # Square-tube telescoping sleeves, collars, and replaceable wear bushings.
    # Literal wall names are used by tests as exact-geometry contracts.
    frame.visual(Box((0.092, 0.016, 0.34)), origin=Origin(xyz=(-0.32, 0.038, 0.63)), material=dark_steel, name="seat_sleeve_yp")
    frame.visual(Box((0.092, 0.016, 0.34)), origin=Origin(xyz=(-0.32, -0.038, 0.63)), material=dark_steel, name="seat_sleeve_yn")
    frame.visual(Box((0.016, 0.092, 0.34)), origin=Origin(xyz=(-0.282, 0.0, 0.63)), material=dark_steel, name="seat_sleeve_xp")
    frame.visual(Box((0.016, 0.092, 0.34)), origin=Origin(xyz=(-0.358, 0.0, 0.63)), material=dark_steel, name="seat_sleeve_xn")
    frame.visual(Box((0.132, 0.0225, 0.090)), origin=Origin(xyz=(-0.32, 0.05475, 0.72)), material=frame_blue, name="seat_clamp_collar_yp")
    frame.visual(Box((0.132, 0.0225, 0.090)), origin=Origin(xyz=(-0.32, -0.05475, 0.72)), material=frame_blue, name="seat_clamp_collar_yn")
    frame.visual(Box((0.0225, 0.132, 0.090)), origin=Origin(xyz=(-0.26525, 0.0, 0.72)), material=frame_blue, name="seat_clamp_collar_xp")
    frame.visual(Box((0.0225, 0.132, 0.090)), origin=Origin(xyz=(-0.37475, 0.0, 0.72)), material=frame_blue, name="seat_clamp_collar_xn")
    frame.visual(Box((0.092, 0.016, 0.34)), origin=Origin(xyz=(0.72, 0.038, 0.62)), material=dark_steel, name="bar_sleeve_yp")
    frame.visual(Box((0.092, 0.016, 0.34)), origin=Origin(xyz=(0.72, -0.038, 0.62)), material=dark_steel, name="bar_sleeve_yn")
    frame.visual(Box((0.016, 0.092, 0.34)), origin=Origin(xyz=(0.758, 0.0, 0.62)), material=dark_steel, name="bar_sleeve_xp")
    frame.visual(Box((0.016, 0.092, 0.34)), origin=Origin(xyz=(0.682, 0.0, 0.62)), material=dark_steel, name="bar_sleeve_xn")
    frame.visual(Box((0.132, 0.0225, 0.090)), origin=Origin(xyz=(0.72, 0.05475, 0.72)), material=frame_blue, name="bar_clamp_collar_yp")
    frame.visual(Box((0.132, 0.0225, 0.090)), origin=Origin(xyz=(0.72, -0.05475, 0.72)), material=frame_blue, name="bar_clamp_collar_yn")
    frame.visual(Box((0.0225, 0.132, 0.090)), origin=Origin(xyz=(0.77475, 0.0, 0.72)), material=frame_blue, name="bar_clamp_collar_xp")
    frame.visual(Box((0.0225, 0.132, 0.090)), origin=Origin(xyz=(0.66525, 0.0, 0.72)), material=frame_blue, name="bar_clamp_collar_xn")
    for x, z, prefix in [(-0.32, 0.72, "seat"), (0.72, 0.72, "bar")]:
        frame.visual(
            _mesh(
                TorusGeometry(radius=0.020, tube=0.006, radial_segments=18, tubular_segments=36).rotate_x(pi / 2.0),
                f"{prefix}_thread_boss_mesh",
            ),
            origin=Origin(xyz=(x, -0.067, z)),
            material=zinc,
            name=f"{prefix}_thread_boss",
        )
        frame.visual(
            Box((0.075, 0.020, 0.014)),
            origin=Origin(xyz=(x, -0.064, z + 0.030)),
            material=zinc,
            name=f"{prefix}_split_ear_top",
        )
        frame.visual(
            Box((0.075, 0.020, 0.014)),
            origin=Origin(xyz=(x, -0.064, z - 0.030)),
            material=zinc,
            name=f"{prefix}_split_ear_bottom",
        )

    # Replaceable foot pads / toe stops bolted to the frame, fixed for inspection.
    for y in (-0.40, 0.40):
        frame.visual(
            Box((0.18, 0.060, 0.130)),
            origin=Origin(xyz=(-0.05, y, 0.145)),
            material=dark_steel,
            name=f"foot_pad_pedestal_{0 if y < 0 else 1}",
        )
        frame.visual(
            Box((0.27, 0.075, 0.036)),
            origin=Origin(xyz=(-0.05, y, 0.215)),
            material=rubber,
            name=f"service_foot_pad_{0 if y < 0 else 1}",
        )
        frame.visual(
            Box((0.060, 0.020, 0.080)),
            origin=Origin(xyz=(-0.17, y, 0.245)),
            material=zinc,
            name=f"toe_stop_{0 if y < 0 else 1}",
        )

    flywheel = model.part("flywheel")
    flywheel.visual(
        Cylinder(radius=0.178, length=0.050),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="weighted_disc",
    )
    flywheel.visual(
        Cylinder(radius=0.115, length=0.066),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )
    for angle, name in ((0.0, "spoke_0"), (pi / 2.0, "spoke_1")):
        flywheel.visual(
            Box((0.030, 0.035, 0.300)),
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=zinc,
            name=name,
        )

    crank = model.part("crankset")
    crank.visual(
        Cylinder(radius=0.027, length=0.245),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="spindle",
    )
    crank.visual(
        Cylinder(radius=0.122, length=0.020),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="chainring_guard",
    )
    crank.visual(
        _side_torus("service_chainring_mesh", 0.095, 0.007, 0.0, 0.0),
        origin=Origin(xyz=(0.0, -0.122, 0.0)),
        material=zinc,
        name="replaceable_chainring",
    )
    crank.visual(Box((0.028, 0.038, 0.205)), origin=Origin(xyz=(0.0, -0.134, -0.095)), material=worn_steel, name="crank_arm_0")
    crank.visual(Box((0.028, 0.038, 0.205)), origin=Origin(xyz=(0.0, 0.134, 0.095)), material=worn_steel, name="crank_arm_1")
    crank.visual(Cylinder(radius=0.024, length=0.042), origin=Origin(xyz=(0.0, -0.160, -0.195), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="pedal_boss_0")
    crank.visual(Cylinder(radius=0.024, length=0.042), origin=Origin(xyz=(0.0, 0.160, 0.195), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="pedal_boss_1")

    pedal_0 = model.part("pedal_0")
    pedal_0.visual(Cylinder(radius=0.009, length=0.115), origin=Origin(xyz=(0.0, -0.050, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="pedal_axle")
    pedal_0.visual(Box((0.145, 0.060, 0.026)), origin=Origin(xyz=(0.0, -0.112, 0.0)), material=dark_steel, name="pedal_body")
    pedal_0.visual(Box((0.155, 0.012, 0.050)), origin=Origin(xyz=(0.0, -0.145, 0.0)), material=rubber, name="wear_tread")
    pedal_0.visual(Box((0.025, 0.026, 0.070)), origin=Origin(xyz=(-0.070, -0.108, 0.0)), material=zinc, name="end_cage_0")
    pedal_0.visual(Box((0.025, 0.026, 0.070)), origin=Origin(xyz=(0.070, -0.108, 0.0)), material=zinc, name="end_cage_1")

    pedal_1 = model.part("pedal_1")
    pedal_1.visual(Cylinder(radius=0.009, length=0.115), origin=Origin(xyz=(0.0, 0.050, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=worn_steel, name="pedal_axle")
    pedal_1.visual(Box((0.145, 0.060, 0.026)), origin=Origin(xyz=(0.0, 0.112, 0.0)), material=dark_steel, name="pedal_body")
    pedal_1.visual(Box((0.155, 0.012, 0.050)), origin=Origin(xyz=(0.0, 0.145, 0.0)), material=rubber, name="wear_tread")
    pedal_1.visual(Box((0.025, 0.026, 0.070)), origin=Origin(xyz=(-0.070, 0.108, 0.0)), material=zinc, name="end_cage_0")
    pedal_1.visual(Box((0.025, 0.026, 0.070)), origin=Origin(xyz=(0.070, 0.108, 0.0)), material=zinc, name="end_cage_1")

    saddle_post = model.part("saddle_post")
    saddle_post.visual(Box((0.044, 0.044, 0.480)), origin=Origin(xyz=(0.0, 0.0, 0.240)), material=worn_steel, name="seat_post")
    saddle_post.visual(Box((0.032, 0.008, 0.080)), origin=Origin(xyz=(0.0, 0.026, 0.130)), material=rubber, name="seat_guide_pad")
    saddle_post.visual(Box((0.155, 0.090, 0.036)), origin=Origin(xyz=(0.0, 0.0, 0.478)), material=dark_steel, name="rail_clamp")
    saddle_post.visual(Box((0.105, 0.074, 0.050)), origin=Origin(xyz=(-0.010, 0.0, 0.510)), material=dark_steel, name="saddle_bridge")
    for y in (-0.037, 0.037):
        saddle_post.visual(Box((0.270, 0.014, 0.018)), origin=Origin(xyz=(-0.015, y, 0.525)), material=zinc, name=f"saddle_rail_{0 if y < 0 else 1}")
    saddle_mesh = superellipse_side_loft(
        [
            (-0.135, 0.545, 0.605, 0.230),
            (-0.060, 0.532, 0.620, 0.335),
            (0.060, 0.532, 0.620, 0.335),
            (0.135, 0.545, 0.605, 0.230),
        ],
        exponents=2.35,
        segments=42,
    ).translate(-0.035, 0.0, 0.0)
    saddle_post.visual(_mesh(saddle_mesh, "wide_service_saddle"), material=vinyl, name="saddle_pad")
    saddle_post.visual(Box((0.105, 0.070, 0.018)), origin=Origin(xyz=(-0.055, 0.0, 0.540)), material=safety_yellow, name="replaceable_saddle_label")

    handlebar_post = model.part("handlebar_post")
    handlebar_post.visual(Box((0.044, 0.044, 0.540)), origin=Origin(xyz=(0.0, 0.0, 0.270)), material=worn_steel, name="bar_post")
    handlebar_post.visual(Box((0.032, 0.008, 0.080)), origin=Origin(xyz=(0.0, 0.026, 0.140)), material=rubber, name="bar_guide_pad")
    handlebar_post.visual(Box((0.115, 0.080, 0.032)), origin=Origin(xyz=(0.0, 0.0, 0.545)), material=dark_steel, name="stem_clamp")
    _tube(handlebar_post, "handlebar_loop", [(0.00, 0.0, 0.540), (0.055, -0.14, 0.625), (0.085, -0.31, 0.640), (0.070, -0.39, 0.610)], 0.018, dark_steel)
    _tube(handlebar_post, "handlebar_loop_mirror", [(0.00, 0.0, 0.540), (0.055, 0.14, 0.625), (0.085, 0.31, 0.640), (0.070, 0.39, 0.610)], 0.018, dark_steel)
    for y in (-0.395, 0.395):
        handlebar_post.visual(Cylinder(radius=0.024, length=0.115), origin=Origin(xyz=(0.070, y, 0.610), rpy=(pi / 2.0, 0.0, 0.0)), material=rubber, name=f"service_grip_{0 if y < 0 else 1}")
    handlebar_post.visual(Box((0.030, 0.052, 0.095)), origin=Origin(xyz=(0.038, 0.0, 0.575)), material=dark_steel, name="timer_stalk")
    handlebar_post.visual(Box((0.135, 0.085, 0.036)), origin=Origin(xyz=(0.055, 0.0, 0.610)), material=safety_yellow, name="maintenance_timer")

    seat_lock = model.part("seat_lock_knob")
    seat_lock.visual(Cylinder(radius=0.008, length=0.082), origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=zinc, name="threaded_stub")
    seat_lock.visual(Cylinder(radius=0.034, length=0.032), origin=Origin(xyz=(0.0, -0.056, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=safety_yellow, name="hand_knob")
    seat_lock.visual(Box((0.074, 0.020, 0.020)), origin=Origin(xyz=(0.0, -0.056, 0.0)), material=safety_yellow, name="knob_lobe_x")
    seat_lock.visual(Box((0.020, 0.020, 0.074)), origin=Origin(xyz=(0.0, -0.056, 0.0)), material=safety_yellow, name="knob_lobe_z")

    bar_lock = model.part("bar_lock_knob")
    bar_lock.visual(Cylinder(radius=0.008, length=0.082), origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=zinc, name="threaded_stub")
    bar_lock.visual(Cylinder(radius=0.034, length=0.032), origin=Origin(xyz=(0.0, -0.056, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=safety_yellow, name="hand_knob")
    bar_lock.visual(Box((0.074, 0.020, 0.020)), origin=Origin(xyz=(0.0, -0.056, 0.0)), material=safety_yellow, name="knob_lobe_x")
    bar_lock.visual(Box((0.020, 0.020, 0.074)), origin=Origin(xyz=(0.0, -0.056, 0.0)), material=safety_yellow, name="knob_lobe_z")

    service_panel = model.part("service_panel")
    service_panel.visual(Box((0.215, 0.012, 0.165)), origin=Origin(xyz=(0.122, -0.002, 0.0)), material=frame_blue, name="hinged_cover")
    service_panel.visual(Box((0.185, 0.006, 0.125)), origin=Origin(xyz=(0.126, -0.010, 0.0)), material=translucent, name="flush_window")
    service_panel.visual(Cylinder(radius=0.015, length=0.155), origin=Origin(xyz=(0.0, -0.002, 0.0)), material=zinc, name="panel_hinge_barrel")
    service_panel.visual(Box((0.028, 0.014, 0.145)), origin=Origin(xyz=(0.018, -0.002, 0.0)), material=zinc, name="panel_hinge_leaf")
    for x, z in ((0.035, -0.060), (0.180, -0.060), (0.035, 0.060), (0.180, 0.060)):
        service_panel.visual(
            Cylinder(radius=0.006, length=0.007),
            origin=Origin(xyz=(x, -0.011, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"quarter_turn_fastener_{len(service_panel.visuals)}",
        )

    crank_joint = model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(0.07, 0.0, 0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=16.0),
        motion_properties=MotionProperties(damping=0.25, friction=0.05),
    )
    model.articulation(
        "flywheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=flywheel,
        origin=Origin(xyz=(0.42, 0.0, 0.42)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=32.0),
        mimic=Mimic(joint=crank_joint.name, multiplier=2.4),
        motion_properties=MotionProperties(damping=0.10, friction=0.02),
    )
    model.articulation(
        "pedal_spin_0",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_0,
        origin=Origin(xyz=(0.0, -0.170, -0.195)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "pedal_spin_1",
        ArticulationType.CONTINUOUS,
        parent=crank,
        child=pedal_1,
        origin=Origin(xyz=(0.0, 0.170, 0.195)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "seat_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle_post,
        origin=Origin(xyz=(-0.32, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.18, lower=0.0, upper=0.18),
        motion_properties=MotionProperties(damping=1.0, friction=2.0),
    )
    model.articulation(
        "bar_height",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handlebar_post,
        origin=Origin(xyz=(0.72, 0.0, 0.480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.18, lower=0.0, upper=0.16),
        motion_properties=MotionProperties(damping=1.0, friction=2.0),
    )
    model.articulation(
        "seat_lock_turn",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_lock,
        origin=Origin(xyz=(-0.32, -0.095, 0.720)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0, lower=-pi, upper=pi),
        motion_properties=MotionProperties(friction=0.4),
    )
    model.articulation(
        "bar_lock_turn",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bar_lock,
        origin=Origin(xyz=(0.72, -0.095, 0.720)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0, lower=-pi, upper=pi),
        motion_properties=MotionProperties(friction=0.4),
    )
    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=service_panel,
        origin=Origin(xyz=(0.246, -0.106, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.4, lower=0.0, upper=1.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    saddle_post = object_model.get_part("saddle_post")
    handlebar_post = object_model.get_part("handlebar_post")
    pedal_0 = object_model.get_part("pedal_0")
    service_panel = object_model.get_part("service_panel")

    crank_spin = object_model.get_articulation("crank_spin")
    seat_height = object_model.get_articulation("seat_height")
    bar_height = object_model.get_articulation("bar_height")
    service_panel_hinge = object_model.get_articulation("service_panel_hinge")

    ctx.allow_overlap(
        "crankset",
        "pedal_0",
        elem_a="pedal_boss_0",
        elem_b="pedal_axle",
        reason="The pedal axle is intentionally captured inside the crank-arm threaded boss.",
    )
    ctx.allow_overlap(
        "crankset",
        "pedal_1",
        elem_a="pedal_boss_1",
        elem_b="pedal_axle",
        reason="The pedal axle is intentionally captured inside the opposite crank-arm threaded boss.",
    )
    ctx.allow_overlap(
        "crankset",
        "frame",
        elem_a="spindle",
        elem_b="crank_bearing_plate_0",
        reason="The crank spindle is intentionally captured through the replaceable bearing carrier plate.",
    )
    ctx.allow_overlap(
        "crankset",
        "frame",
        elem_a="spindle",
        elem_b="crank_bearing_plate_1",
        reason="The crank spindle is intentionally captured through the opposite bearing carrier plate.",
    )
    ctx.allow_overlap(
        "frame",
        "service_panel",
        elem_a="frame_hinge_leaf_0",
        elem_b="panel_hinge_barrel",
        reason="The removable service door hinge barrel is locally captured by the frame hinge leaf.",
    )
    ctx.allow_overlap(
        "frame",
        "service_panel",
        elem_a="frame_hinge_leaf_1",
        elem_b="panel_hinge_barrel",
        reason="The second hinge leaf locally captures the same service door hinge barrel.",
    )
    ctx.allow_overlap(
        "frame",
        "service_panel",
        elem_a="frame_hinge_knuckle_0",
        elem_b="panel_hinge_barrel",
        reason="The rotating service door barrel shares the hinge pin volume with the fixed lower knuckle.",
    )
    ctx.allow_overlap(
        "frame",
        "service_panel",
        elem_a="frame_hinge_knuckle_1",
        elem_b="panel_hinge_barrel",
        reason="The rotating service door barrel shares the hinge pin volume with the fixed upper knuckle.",
    )
    ctx.expect_overlap(
        "crankset",
        "frame",
        axes="y",
        elem_a="spindle",
        elem_b="crank_bearing_plate_0",
        min_overlap=0.015,
        name="crank spindle passes through the service bearing plate",
    )
    ctx.expect_overlap(
        "service_panel",
        "frame",
        axes="z",
        elem_a="panel_hinge_barrel",
        elem_b="frame_hinge_leaf_0",
        min_overlap=0.015,
        name="service door hinge barrel is supported by the lower hinge leaf",
    )

    ctx.expect_gap(
        frame,
        saddle_post,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="seat_sleeve_yp",
        negative_elem="seat_post",
        name="seat post clears the positive sleeve wall",
    )
    ctx.expect_gap(
        saddle_post,
        frame,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="seat_post",
        negative_elem="seat_sleeve_yn",
        name="seat post clears the negative sleeve wall",
    )
    ctx.expect_overlap(
        saddle_post,
        frame,
        axes="z",
        elem_a="seat_post",
        elem_b="seat_sleeve_yp",
        min_overlap=0.11,
        name="seat post has collapsed insertion length",
    )
    ctx.expect_gap(
        frame,
        handlebar_post,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="bar_sleeve_yp",
        negative_elem="bar_post",
        name="handlebar post clears the positive sleeve wall",
    )
    ctx.expect_gap(
        handlebar_post,
        frame,
        axis="y",
        min_gap=0.004,
        max_gap=0.014,
        positive_elem="bar_post",
        negative_elem="bar_sleeve_yn",
        name="handlebar post clears the negative sleeve wall",
    )
    ctx.expect_overlap(
        handlebar_post,
        frame,
        axes="z",
        elem_a="bar_post",
        elem_b="bar_sleeve_yp",
        min_overlap=0.13,
        name="handlebar post has collapsed insertion length",
    )

    pedal_rest = ctx.part_world_position(pedal_0)
    with ctx.pose({crank_spin: pi / 2.0}):
        pedal_quarter = ctx.part_world_position(pedal_0)
    ctx.check(
        "crank carries pedal around the bearing axis",
        pedal_rest is not None
        and pedal_quarter is not None
        and abs(pedal_quarter[0] - pedal_rest[0]) > 0.12
        and abs(pedal_quarter[2] - pedal_rest[2]) > 0.12,
        details=f"rest={pedal_rest}, quarter_turn={pedal_quarter}",
    )

    rest_seat = ctx.part_world_position(saddle_post)
    with ctx.pose({seat_height: 0.18}):
        raised_seat = ctx.part_world_position(saddle_post)
        ctx.expect_overlap(
            saddle_post,
            frame,
            axes="z",
            elem_a="seat_post",
            elem_b="seat_sleeve_yp",
            min_overlap=0.10,
            name="raised seat post still has retained insertion",
        )
    ctx.check(
        "seat height adjuster travels upward",
        rest_seat is not None and raised_seat is not None and raised_seat[2] > rest_seat[2] + 0.15,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    rest_bar = ctx.part_world_position(handlebar_post)
    with ctx.pose({bar_height: 0.16}):
        raised_bar = ctx.part_world_position(handlebar_post)
        ctx.expect_overlap(
            handlebar_post,
            frame,
            axes="z",
            elem_a="bar_post",
            elem_b="bar_sleeve_yp",
            min_overlap=0.11,
            name="raised handlebar post still has retained insertion",
        )
    ctx.check(
        "handlebar height adjuster travels upward",
        rest_bar is not None and raised_bar is not None and raised_bar[2] > rest_bar[2] + 0.13,
        details=f"rest={rest_bar}, raised={raised_bar}",
    )

    ctx.expect_gap(
        frame,
        service_panel,
        axis="y",
        min_gap=None,
        max_gap=0.010,
        max_penetration=0.0,
        positive_elem="access_flange",
        negative_elem="hinged_cover",
        name="closed service panel sits just proud of the housing flange",
    )
    closed_aabb = ctx.part_element_world_aabb(service_panel, elem="hinged_cover")
    closed_panel_y = None if closed_aabb is None else (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
    with ctx.pose({service_panel_hinge: 1.0}):
        open_aabb = ctx.part_element_world_aabb(service_panel, elem="hinged_cover")
        open_panel_y = None if open_aabb is None else (open_aabb[0][1] + open_aabb[1][1]) * 0.5
    ctx.check(
        "service panel swings open on the hinge",
        closed_panel_y is not None and open_panel_y is not None and abs(open_panel_y - closed_panel_y) > 0.04,
        details=f"closed_y={closed_panel_y}, open_y={open_panel_y}",
    )

    return ctx.report()


object_model = build_object_model()
