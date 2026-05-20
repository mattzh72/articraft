from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)


WHEEL_CENTER_Z = 2.15
RIM_RADIUS = 1.22
HANGER_RADIUS = 1.20
SIDE_RING_Y = 0.22
GONDOLA_COUNT = 8


def _circle_points(radius: float, y: float, *, segments: int = 72) -> list[tuple[float, float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), y, radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _straight_tube(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    segments: int = 12,
):
    return wire_from_points(
        [p0, p1],
        radius=radius,
        radial_segments=segments,
        cap_ends=True,
        corner_mode="miter",
    )


def _wheel_truss_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    for y in (-SIDE_RING_Y, SIDE_RING_Y):
        geom.merge(
            tube_from_spline_points(
                _circle_points(RIM_RADIUS, y),
                radius=0.024,
                samples_per_segment=2,
                closed_spline=True,
                radial_segments=12,
                cap_ends=False,
                up_hint=(0.0, 1.0, 0.0),
            )
        )
        geom.merge(
            tube_from_spline_points(
                _circle_points(0.88, y),
                radius=0.019,
                samples_per_segment=2,
                closed_spline=True,
                radial_segments=12,
                cap_ends=False,
                up_hint=(0.0, 1.0, 0.0),
            )
        )
        # Radial spokes and a triangular annular truss.
        for i in range(16):
            a0 = 2.0 * pi * i / 16.0
            a1 = 2.0 * pi * (i + 0.5) / 16.0
            spoke_outer = (0.98 * cos(a0), y, 0.98 * sin(a0))
            spoke_inner = (0.12 * cos(a0), y, 0.12 * sin(a0))
            geom.merge(_straight_tube(spoke_inner, spoke_outer, radius=0.012, segments=10))
            outer0 = (RIM_RADIUS * cos(a0), y, RIM_RADIUS * sin(a0))
            outer1 = (RIM_RADIUS * cos(2.0 * pi * (i + 1) / 16.0), y, RIM_RADIUS * sin(2.0 * pi * (i + 1) / 16.0))
            inner_mid = (0.88 * cos(a1), y, 0.88 * sin(a1))
            geom.merge(_straight_tube(outer0, inner_mid, radius=0.010, segments=8))
            geom.merge(_straight_tube(inner_mid, outer1, radius=0.010, segments=8))

    return geom


def _add_frame_tube(part, name: str, p0, p1, *, radius: float, material) -> None:
    part.visual(
        mesh_from_geometry(_straight_tube(p0, p1, radius=radius), name),
        material=material,
        name=name,
    )


def _add_gondola_visuals(part, body_material, trim_material, seat_material) -> None:
    # The part frame is the small pivot axis; all cabin geometry hangs below it.
    part.visual(
        Cylinder(radius=0.035, length=0.30),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="pivot_sleeve",
    )
    for y in (-0.13, 0.13):
        part.visual(
            Cylinder(radius=0.013, length=0.31),
            origin=Origin(xyz=(0.0, y, -0.185)),
            material=trim_material,
            name=f"hanger_link_{'rear' if y < 0.0 else 'front'}",
        )
    part.visual(
        Box((0.46, 0.34, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.345)),
        material=trim_material,
        name="upper_yoke",
    )
    part.visual(
        Box((0.50, 0.36, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, -0.665)),
        material=body_material,
        name="cabin_floor",
    )
    part.visual(
        Box((0.50, 0.045, 0.37)),
        origin=Origin(xyz=(0.0, 0.178, -0.545)),
        material=body_material,
        name="front_panel",
    )
    part.visual(
        Box((0.50, 0.045, 0.37)),
        origin=Origin(xyz=(0.0, -0.178, -0.545)),
        material=body_material,
        name="rear_panel",
    )
    part.visual(
        Box((0.055, 0.36, 0.37)),
        origin=Origin(xyz=(0.252, 0.0, -0.555)),
        material=body_material,
        name="side_panel_0",
    )
    part.visual(
        Box((0.055, 0.36, 0.37)),
        origin=Origin(xyz=(-0.252, 0.0, -0.555)),
        material=body_material,
        name="side_panel_1",
    )
    for x in (-0.115, 0.115):
        part.visual(
            Box((0.16, 0.25, 0.045)),
            origin=Origin(xyz=(x, 0.0, -0.605)),
            material=seat_material,
            name=f"seat_{0 if x < 0.0 else 1}",
        )
    part.visual(
        Cylinder(radius=0.013, length=0.54),
        origin=Origin(xyz=(0.0, 0.155, -0.425), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_material,
        name="front_guard",
    )
    part.visual(
        Box((0.56, 0.40, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.300)),
        material=body_material,
        name="small_canopy",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="decorative_ferris_wheel")

    base_blue = model.material("base_blue", rgba=(0.07, 0.12, 0.26, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.90, 0.87, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    gold = model.material("warm_gold", rgba=(0.92, 0.63, 0.20, 1.0))
    brass = model.material("brass_pins", rgba=(0.86, 0.66, 0.28, 1.0))
    seat_tan = model.material("seat_tan", rgba=(0.78, 0.58, 0.38, 1.0))
    bulb_red = model.material("bulb_red", rgba=(1.00, 0.10, 0.08, 1.0))
    bulb_yellow = model.material("bulb_yellow", rgba=(1.00, 0.86, 0.18, 1.0))
    bulb_blue = model.material("bulb_blue", rgba=(0.10, 0.45, 1.00, 1.0))
    gondola_mats = [
        model.material("gondola_red", rgba=(0.78, 0.12, 0.10, 1.0)),
        model.material("gondola_teal", rgba=(0.05, 0.54, 0.58, 1.0)),
        model.material("gondola_yellow", rgba=(0.98, 0.72, 0.12, 1.0)),
        model.material("gondola_violet", rgba=(0.45, 0.24, 0.72, 1.0)),
    ]

    base = model.part("base")
    base.visual(Box((2.45, 1.55, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=base_blue, name="shared_plinth")
    base.visual(Box((2.18, 1.30, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.1325)), material=dark_steel, name="deck_plate")
    base.visual(Box((0.36, 0.20, 0.10)), origin=Origin(xyz=(-0.88, -0.62, 0.18)), material=dark_steel, name="foot_0")
    base.visual(Box((0.36, 0.20, 0.10)), origin=Origin(xyz=(0.88, -0.62, 0.18)), material=dark_steel, name="foot_1")
    base.visual(Box((0.36, 0.20, 0.10)), origin=Origin(xyz=(-0.88, 0.62, 0.18)), material=dark_steel, name="foot_2")
    base.visual(Box((0.36, 0.20, 0.10)), origin=Origin(xyz=(0.88, 0.62, 0.18)), material=dark_steel, name="foot_3")

    for y in (-0.62, 0.62):
        hub = (0.0, y, WHEEL_CENTER_Z)
        left_anchor = (-0.92, y, 0.15)
        right_anchor = (0.92, y, 0.15)
        _add_frame_tube(base, f"side_frame_{'rear' if y < 0.0 else 'front'}_leg_0", left_anchor, hub, radius=0.033, material=painted_steel)
        _add_frame_tube(base, f"side_frame_{'rear' if y < 0.0 else 'front'}_leg_1", right_anchor, hub, radius=0.033, material=painted_steel)
        _add_frame_tube(base, f"side_frame_{'rear' if y < 0.0 else 'front'}_base_tie", left_anchor, right_anchor, radius=0.025, material=painted_steel)
        _add_frame_tube(base, f"side_frame_{'rear' if y < 0.0 else 'front'}_brace_0", (-0.56, y, 0.86), (0.40, y, 1.26), radius=0.018, material=painted_steel)
        _add_frame_tube(base, f"side_frame_{'rear' if y < 0.0 else 'front'}_brace_1", (0.56, y, 0.86), (-0.40, y, 1.26), radius=0.018, material=painted_steel)
        base.visual(
            mesh_from_geometry(TorusGeometry(0.076, 0.018, radial_segments=14, tubular_segments=36).rotate_x(pi / 2.0), f"bearing_collar_{y}"),
            origin=Origin(xyz=hub),
            material=dark_steel,
            name=f"bearing_collar_{'rear' if y < 0.0 else 'front'}",
        )
        base.visual(
            Box((0.20, 0.080, 0.24)),
            origin=Origin(xyz=(0.0, y, WHEEL_CENTER_Z - 0.08)),
            material=dark_steel,
            name=f"bearing_block_{'rear' if y < 0.0 else 'front'}",
        )

    wheel = model.part("wheel")
    wheel.visual(mesh_from_geometry(_wheel_truss_geometry(), "ring_truss"), material=gold, name="ring_truss")
    wheel.visual(
        Cylinder(radius=0.042, length=1.16),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="center_axle",
    )
    wheel.visual(
        Cylinder(radius=0.13, length=0.54),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=painted_steel,
        name="hub_drum",
    )

    for i in range(24):
        angle = 2.0 * pi * i / 24.0
        mat = (bulb_red, bulb_yellow, bulb_blue)[i % 3]
        bulb_y = 0.245 if i % 2 == 0 else -0.245
        wheel.visual(
            Sphere(radius=0.030),
            origin=Origin(xyz=(1.245 * cos(angle), bulb_y, 1.245 * sin(angle))),
            material=mat,
            name=f"rim_bulb_{i}",
        )

    for i in range(GONDOLA_COUNT):
        angle = 2.0 * pi * i / GONDOLA_COUNT
        x = HANGER_RADIUS * cos(angle)
        z = HANGER_RADIUS * sin(angle)
        wheel.visual(
            Cylinder(radius=0.018, length=0.54),
            origin=Origin(xyz=(x, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brass,
            name=f"hanger_pin_{i}",
        )

    model.articulation(
        "base_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.55),
    )

    for i in range(GONDOLA_COUNT):
        gondola = model.part(f"gondola_{i}")
        _add_gondola_visuals(gondola, gondola_mats[i % len(gondola_mats)], dark_steel, seat_tan)
        angle = 2.0 * pi * i / GONDOLA_COUNT
        model.articulation(
            f"wheel_to_gondola_{i}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(HANGER_RADIUS * cos(angle), 0.0, HANGER_RADIUS * sin(angle))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.5, velocity=2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("base_to_wheel")

    ctx.check(
        "wheel uses continuous center-axis rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.expect_gap(
        wheel,
        base,
        axis="z",
        min_gap=0.10,
        positive_elem="ring_truss",
        negative_elem="shared_plinth",
        name="wheel clears the shared base",
    )

    rest_pos = ctx.part_world_position(object_model.get_part("gondola_0"))
    with ctx.pose({wheel_spin: pi / 4.0}):
        spun_pos = ctx.part_world_position(object_model.get_part("gondola_0"))
    ctx.check(
        "wheel motion carries rim gondolas around the hub",
        rest_pos is not None
        and spun_pos is not None
        and abs(spun_pos[0] - rest_pos[0]) > 0.20
        and abs(spun_pos[2] - rest_pos[2]) > 0.20,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    for i in range(GONDOLA_COUNT):
        gondola = object_model.get_part(f"gondola_{i}")
        pivot = object_model.get_articulation(f"wheel_to_gondola_{i}")
        ctx.check(
            f"gondola_{i} has its own pivot",
            pivot.articulation_type == ArticulationType.CONTINUOUS and tuple(pivot.axis) == (0.0, 1.0, 0.0),
            details=f"type={pivot.articulation_type}, axis={pivot.axis}",
        )
        ctx.allow_overlap(
            wheel,
            gondola,
            elem_a=f"hanger_pin_{i}",
            elem_b="pivot_sleeve",
            reason="The visible brass pin is intentionally captured inside the gondola pivot sleeve.",
        )
        ctx.expect_within(
            wheel,
            gondola,
            axes="xz",
            inner_elem=f"hanger_pin_{i}",
            outer_elem="pivot_sleeve",
            margin=0.002,
            name=f"gondola_{i} sleeve surrounds its pin radially",
        )
        ctx.expect_overlap(
            wheel,
            gondola,
            axes="y",
            elem_a=f"hanger_pin_{i}",
            elem_b="pivot_sleeve",
            min_overlap=0.25,
            name=f"gondola_{i} pivot has retained pin length",
        )

    return ctx.report()


object_model = build_object_model()
