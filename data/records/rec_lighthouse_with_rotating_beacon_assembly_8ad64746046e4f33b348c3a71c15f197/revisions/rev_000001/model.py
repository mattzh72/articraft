from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 72,
):
    half_height = 0.5 * height
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_height), (outer_radius, half_height)],
        [(inner_radius, -half_height), (inner_radius, half_height)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _arc_points(
    radius: float,
    z: float,
    angle_start: float,
    angle_end: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos(angle_start + (angle_end - angle_start) * t / (samples - 1)),
            radius * math.sin(angle_start + (angle_end - angle_start) * t / (samples - 1)),
            z,
        )
        for t in range(samples)
    ]


def _rail_segment_mesh(
    *,
    radius: float,
    z: float,
    angle_start: float,
    angle_end: float,
    tube_radius: float,
):
    return tube_from_spline_points(
        _arc_points(radius, z, angle_start, angle_end, 9),
        radius=tube_radius,
        samples_per_segment=6,
        radial_segments=16,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="harbor_lighthouse")

    masonry = model.material("masonry", rgba=(0.92, 0.93, 0.90, 1.0))
    red_roof = model.material("red_roof", rgba=(0.63, 0.15, 0.10, 1.0))
    ironwork = model.material("ironwork", rgba=(0.23, 0.26, 0.29, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.42, 0.46, 0.49, 1.0))
    glass = model.material("glass", rgba=(0.78, 0.90, 0.97, 0.28))
    brass = model.material("brass", rgba=(0.79, 0.70, 0.34, 0.68))
    lamp_black = model.material("lamp_black", rgba=(0.10, 0.11, 0.12, 1.0))

    tower = model.part("tower")
    tower_shell_mesh = _save_mesh(
        "tower_shell",
        LatheGeometry(
            [
                (0.0, 0.0),
                (2.75, 0.0),
                (2.75, 0.70),
                (2.30, 0.92),
                (2.12, 4.50),
                (1.78, 10.20),
                (1.46, 14.35),
                (1.38, 15.12),
                (1.38, 15.24),
                (1.60, 15.34),
                (1.60, 15.50),
                (0.0, 15.50),
            ],
            segments=88,
        ),
    )
    gallery_deck_mesh = _save_mesh(
        "gallery_deck",
        _ring_band(outer_radius=2.52, inner_radius=1.58, height=0.22, segments=72),
    )
    coping_ring_mesh = _save_mesh(
        "coping_ring",
        TorusGeometry(radius=2.28, tube=0.085, radial_segments=18, tubular_segments=72),
    )
    tower.visual(tower_shell_mesh, material=masonry, name="tower_shell")
    tower.visual(
        gallery_deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 15.39)),
        material=weathered_steel,
        name="gallery_deck",
    )
    tower.visual(
        coping_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 15.29)),
        material=weathered_steel,
        name="gallery_coping",
    )
    tower.visual(
        Box((0.28, 0.08, 2.25)),
        origin=Origin(xyz=(2.05, 0.0, 1.125)),
        material=lamp_black,
        name="entry_door",
    )
    tower.inertial = Inertial.from_geometry(
        Cylinder(radius=2.75, length=15.54),
        mass=88000.0,
        origin=Origin(xyz=(0.0, 0.0, 7.77)),
    )

    gallery = model.part("gallery_railing")
    rail_radius = 2.33
    gap_half_angle = 0.17
    top_z = 1.05
    mid_z = 0.56
    gallery.visual(
        _save_mesh(
            "gallery_top_rail_port",
            _rail_segment_mesh(
                radius=rail_radius,
                z=top_z,
                angle_start=gap_half_angle + 0.04,
                angle_end=math.pi,
                tube_radius=0.035,
            ),
        ),
        material=ironwork,
        name="top_rail_port",
    )
    gallery.visual(
        _save_mesh(
            "gallery_top_rail_starboard",
            _rail_segment_mesh(
                radius=rail_radius,
                z=top_z,
                angle_start=-math.pi,
                angle_end=-gap_half_angle - 0.04,
                tube_radius=0.035,
            ),
        ),
        material=ironwork,
        name="top_rail_starboard",
    )
    gallery.visual(
        _save_mesh(
            "gallery_mid_rail_port",
            _rail_segment_mesh(
                radius=rail_radius,
                z=mid_z,
                angle_start=gap_half_angle + 0.04,
                angle_end=math.pi,
                tube_radius=0.024,
            ),
        ),
        material=ironwork,
        name="mid_rail_port",
    )
    gallery.visual(
        _save_mesh(
            "gallery_mid_rail_starboard",
            _rail_segment_mesh(
                radius=rail_radius,
                z=mid_z,
                angle_start=-math.pi,
                angle_end=-gap_half_angle - 0.04,
                tube_radius=0.024,
            ),
        ),
        material=ironwork,
        name="mid_rail_starboard",
    )
    for index, angle in enumerate(
        [
            -2.75,
            -2.30,
            -1.85,
            -1.40,
            -0.95,
            -0.48,
            0.48,
            0.95,
            1.40,
            1.85,
            2.30,
            2.75,
        ]
    ):
        gallery.visual(
            Box((0.08, 0.08, 1.10)),
            origin=Origin(
                xyz=(rail_radius * math.cos(angle), rail_radius * math.sin(angle), 0.55),
                rpy=(0.0, 0.0, angle),
            ),
            material=ironwork,
            name=f"rail_post_{index:02d}",
        )
    strike_angle = -gap_half_angle - 0.04
    gallery.visual(
        Box((0.09, 0.09, 1.10)),
        origin=Origin(
            xyz=(rail_radius * math.cos(strike_angle), rail_radius * math.sin(strike_angle), 0.55),
            rpy=(0.0, 0.0, strike_angle),
        ),
        material=ironwork,
        name="gate_strike_post",
    )
    gallery.inertial = Inertial.from_geometry(
        Cylinder(radius=2.40, length=1.12),
        mass=520.0,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )

    lantern = model.part("lantern_room")
    lantern.visual(
        _save_mesh(
            "lantern_floor_ring",
            _ring_band(outer_radius=1.18, inner_radius=0.66, height=0.10, segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=weathered_steel,
        name="floor_ring",
    )
    lantern.visual(
        _save_mesh(
            "lantern_sill_ring",
            _ring_band(outer_radius=1.22, inner_radius=1.08, height=0.14, segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=ironwork,
        name="cage_sill",
    )
    lantern.visual(
        _save_mesh(
            "lantern_top_ring",
            _ring_band(outer_radius=1.20, inner_radius=1.04, height=0.12, segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, 2.58)),
        material=ironwork,
        name="cage_top_ring",
    )
    for index in range(8):
        angle = index * math.tau / 8.0
        post_radius = 1.13
        pane_angle = angle + math.tau / 16.0
        lantern.visual(
            Box((0.09, 0.12, 2.30)),
            origin=Origin(
                xyz=(post_radius * math.cos(angle), post_radius * math.sin(angle), 1.38),
                rpy=(0.0, 0.0, angle),
            ),
            material=ironwork,
            name=f"cage_post_{index:02d}",
        )
        lantern.visual(
            Box((0.025, 0.73, 2.28)),
            origin=Origin(
                xyz=(1.10 * math.cos(pane_angle), 1.10 * math.sin(pane_angle), 1.38),
                rpy=(0.0, 0.0, pane_angle),
            ),
            material=glass,
            name=f"glass_pane_{index:02d}",
        )
    lantern.inertial = Inertial.from_geometry(
        Cylinder(radius=1.24, length=2.70),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 1.35)),
    )

    roof = model.part("lantern_roof")
    roof.visual(
        _save_mesh(
            "roof_shell",
            LatheGeometry(
                [
                    (0.0, 0.0),
                    (1.22, 0.0),
                    (1.18, 0.08),
                    (1.00, 0.42),
                    (0.66, 0.88),
                    (0.30, 1.22),
                    (0.12, 1.38),
                    (0.0, 1.44),
                ],
                segments=72,
            ),
        ),
        material=red_roof,
        name="roof_shell",
    )
    roof.visual(
        _save_mesh(
            "roof_base_ring",
            TorusGeometry(radius=1.17, tube=0.05, radial_segments=16, tubular_segments=60),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=ironwork,
        name="roof_base_ring",
    )
    roof.visual(
        Cylinder(radius=0.15, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        material=ironwork,
        name="roof_vent",
    )
    roof.inertial = Inertial.from_geometry(
        Cylinder(radius=1.22, length=1.90),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
    )

    shaft = model.part("central_shaft")
    shaft.visual(
        Cylinder(radius=0.09, length=2.56),
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
        material=weathered_steel,
        name="shaft_column",
    )
    shaft.visual(
        _save_mesh(
            "shaft_base_collar",
            _ring_band(outer_radius=0.20, inner_radius=0.08, height=0.04, segments=48),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=weathered_steel,
        name="shaft_base_collar",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.14, length=2.56),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
    )

    carriage = model.part("beacon_carriage")
    carriage.visual(
        _save_mesh(
            "carriage_lower_bearing",
            _ring_band(outer_radius=0.56, inner_radius=0.11, height=0.20, segments=64),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=weathered_steel,
        name="lower_bearing",
    )
    for index in range(3):
        angle = index * math.tau / 3.0
        carriage.visual(
            Box((0.04, 0.08, 0.18)),
            origin=Origin(
                xyz=(0.11 * math.cos(angle), 0.11 * math.sin(angle), 0.16),
                rpy=(0.0, 0.0, angle),
            ),
            material=weathered_steel,
            name=f"guide_pad_{index:02d}",
        )
    carriage.visual(
        _save_mesh(
            "fresnel_lens_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.30, 0.00),
                    (0.35, 0.10),
                    (0.39, 0.62),
                    (0.35, 1.24),
                    (0.31, 1.56),
                ],
                [
                    (0.14, 0.02),
                    (0.17, 0.10),
                    (0.19, 0.62),
                    (0.17, 1.24),
                    (0.14, 1.54),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=brass,
        name="lens_shell",
    )
    carriage.visual(
        _save_mesh(
            "carriage_top_cap",
            _ring_band(outer_radius=0.36, inner_radius=0.13, height=0.10, segments=56),
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.85)),
        material=weathered_steel,
        name="lens_cap",
    )
    carriage.visual(
        Box((0.18, 0.12, 0.24)),
        origin=Origin(xyz=(0.58, 0.0, 0.62)),
        material=lamp_black,
        name="drive_motor",
    )
    carriage.visual(
        Box((0.34, 0.08, 0.10)),
        origin=Origin(xyz=(0.40, 0.0, 0.62)),
        material=weathered_steel,
        name="motor_support",
    )
    carriage.visual(
        Box((0.16, 0.18, 0.20)),
        origin=Origin(xyz=(-0.50, 0.0, 0.62)),
        material=lamp_black,
        name="counterweight_box",
    )
    carriage.visual(
        Box((0.34, 0.08, 0.08)),
        origin=Origin(xyz=(-0.34, 0.0, 0.62)),
        material=weathered_steel,
        name="counterweight_support",
    )
    carriage.visual(
        Box((0.42, 0.24, 0.06)),
        origin=Origin(xyz=(0.44, 0.0, 0.18)),
        material=weathered_steel,
        name="service_platform",
    )
    carriage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=2.00),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
    )

    gate = model.part("gallery_gate")
    gate_width = 0.82
    gate.visual(
        Box((0.07, 0.07, 1.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=ironwork,
        name="gate_hinge_stile",
    )
    gate.visual(
        Box((0.07, 0.07, 1.10)),
        origin=Origin(xyz=(0.0, -gate_width, 0.55)),
        material=ironwork,
        name="gate_free_stile",
    )
    gate.visual(
        Box((0.05, gate_width, 0.05)),
        origin=Origin(xyz=(0.0, -0.5 * gate_width, 1.05)),
        material=ironwork,
        name="gate_top_rail",
    )
    gate.visual(
        Box((0.04, gate_width, 0.04)),
        origin=Origin(xyz=(0.0, -0.5 * gate_width, 0.56)),
        material=ironwork,
        name="gate_mid_rail",
    )
    gate.inertial = Inertial.from_geometry(
        Box((0.08, gate_width + 0.08, 1.10)),
        mass=68.0,
        origin=Origin(xyz=(0.0, -0.5 * gate_width, 0.55)),
    )

    model.articulation(
        "tower_to_gallery_railing",
        ArticulationType.FIXED,
        parent=tower,
        child=gallery,
        origin=Origin(xyz=(0.0, 0.0, 15.50)),
    )
    model.articulation(
        "tower_to_lantern_room",
        ArticulationType.FIXED,
        parent=tower,
        child=lantern,
        origin=Origin(xyz=(0.0, 0.0, 15.50)),
    )
    model.articulation(
        "lantern_room_to_roof",
        ArticulationType.FIXED,
        parent=lantern,
        child=roof,
        origin=Origin(xyz=(0.0, 0.0, 2.64)),
    )
    model.articulation(
        "lantern_room_to_shaft",
        ArticulationType.FIXED,
        parent=lantern,
        child=shaft,
        origin=Origin(),
    )
    model.articulation(
        "shaft_to_beacon_carriage",
        ArticulationType.CONTINUOUS,
        parent=shaft,
        child=carriage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.9),
    )
    model.articulation(
        "gallery_railing_to_gate",
        ArticulationType.REVOLUTE,
        parent=gallery,
        child=gate,
        origin=Origin(
            xyz=(
                rail_radius * math.cos(gap_half_angle),
                rail_radius * math.sin(gap_half_angle),
                0.0,
            ),
            rpy=(0.0, 0.0, gap_half_angle),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    tower = object_model.get_part("tower")
    lantern = object_model.get_part("lantern_room")
    roof = object_model.get_part("lantern_roof")
    shaft = object_model.get_part("central_shaft")
    carriage = object_model.get_part("beacon_carriage")
    gate = object_model.get_part("gallery_gate")
    beacon_spin = object_model.get_articulation("shaft_to_beacon_carriage")
    gate_hinge = object_model.get_articulation("gallery_railing_to_gate")

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        lantern,
        tower,
        axis="z",
        positive_elem="floor_ring",
        negative_elem="gallery_deck",
        max_gap=0.01,
        max_penetration=0.0,
        name="lantern room stands on the gallery deck",
    )
    ctx.expect_gap(
        roof,
        lantern,
        axis="z",
        positive_elem="roof_shell",
        negative_elem="cage_top_ring",
        max_gap=0.01,
        max_penetration=0.0,
        name="roof seats on the lantern cage",
    )

    shaft_center = elem_center(shaft, "shaft_column")
    bearing_center = elem_center(carriage, "lower_bearing")
    shaft_aabb = ctx.part_element_world_aabb(shaft, elem="shaft_column")
    bearing_aabb = ctx.part_element_world_aabb(carriage, elem="lower_bearing")
    shaft_supports_carriage = (
        shaft_center is not None
        and bearing_center is not None
        and shaft_aabb is not None
        and bearing_aabb is not None
        and abs(shaft_center[0] - bearing_center[0]) < 0.01
        and abs(shaft_center[1] - bearing_center[1]) < 0.01
        and shaft_aabb[0][2] <= bearing_aabb[0][2] + 0.001
        and shaft_aabb[1][2] >= bearing_aabb[1][2] - 0.001
    )
    ctx.check(
        "beacon carriage is coaxially supported by the central shaft",
        shaft_supports_carriage,
        details=f"shaft_center={shaft_center}, bearing_center={bearing_center}, shaft_aabb={shaft_aabb}, bearing_aabb={bearing_aabb}",
    )

    motor_rest = elem_center(carriage, "drive_motor")
    with ctx.pose({beacon_spin: math.pi / 2.0}):
        motor_quarter = elem_center(carriage, "drive_motor")
    beacon_rotates = (
        motor_rest is not None
        and motor_quarter is not None
        and motor_quarter[1] > motor_rest[1] + 0.40
        and abs(motor_quarter[0]) < abs(motor_rest[0]) + 0.08
    )
    ctx.check(
        "beacon carriage rotates about the vertical shaft",
        beacon_rotates,
        details=f"rest={motor_rest}, quarter_turn={motor_quarter}, axis={beacon_spin.axis}",
    )

    free_edge_closed = elem_center(gate, "gate_free_stile")
    with ctx.pose({gate_hinge: 0.95}):
        free_edge_open = elem_center(gate, "gate_free_stile")
    closed_radius = math.hypot(free_edge_closed[0], free_edge_closed[1]) if free_edge_closed else None
    open_radius = math.hypot(free_edge_open[0], free_edge_open[1]) if free_edge_open else None
    gate_opens_outward = (
        closed_radius is not None
        and open_radius is not None
        and open_radius > closed_radius + 0.18
    )
    ctx.check(
        "gallery gate swings outward from the railing line",
        gate_opens_outward,
        details=f"closed_center={free_edge_closed}, open_center={free_edge_open}, closed_radius={closed_radius}, open_radius={open_radius}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
