from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


WHEEL_CENTER_Z = 1.85
RIM_RADIUS = 1.15
PIVOT_RADIUS = 0.65
SEAT_COUNT = 8


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _point_on_wheel(radius: float, angle: float, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.cos(angle), y, radius * math.sin(angle))


def _radial_midpoint(radius: float, angle: float, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.cos(angle), y, radius * math.sin(angle))


def _add_tube(part, name: str, points, *, radius: float, material, segments: int = 14) -> None:
    part.visual(
        _save_mesh(
            wire_from_points(
                points,
                radius=radius,
                radial_segments=segments,
                closed_path=False,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.018,
                corner_segments=6,
            ),
            name,
        ),
        material=material,
        name=name.replace(".obj", ""),
    )


def _add_open_seat_visuals(seat, *, body_material, frame_material) -> None:
    # The part frame sits on the hanger pivot.  All passenger-basket geometry
    # extends below local -Z so a mimic joint can keep it hanging upright while
    # the main wheel rotates.
    seat.visual(
        Cylinder(radius=0.028, length=0.240),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="pivot_sleeve",
    )
    for y in (-0.095, 0.095):
        seat.visual(
            Box((0.024, 0.018, 0.435)),
            origin=Origin(xyz=(0.0, y, -0.2425)),
            material=frame_material,
            name=f"hanger_arm_{0 if y < 0 else 1}",
        )

    seat.visual(
        Box((0.250, 0.320, 0.035)),
        origin=Origin(xyz=(0.030, 0.0, -0.440)),
        material=body_material,
        name="seat_pan",
    )
    seat.visual(
        Box((0.035, 0.320, 0.220)),
        origin=Origin(xyz=(-0.105, 0.0, -0.350)),
        material=body_material,
        name="seat_back",
    )
    seat.visual(
        Box((0.220, 0.024, 0.135)),
        origin=Origin(xyz=(0.025, -0.172, -0.365)),
        material=body_material,
        name="side_rail_0",
    )
    seat.visual(
        Box((0.220, 0.024, 0.135)),
        origin=Origin(xyz=(0.025, 0.172, -0.365)),
        material=body_material,
        name="side_rail_1",
    )
    seat.visual(
        Cylinder(radius=0.014, length=0.330),
        origin=Origin(xyz=(0.135, 0.0, -0.295), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_material,
        name="front_guard",
    )
    seat.visual(
        Box((0.030, 0.280, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
        material=frame_material,
        name="lower_crossbar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_carnival_observation_wheel")

    trailer_red = model.material("trailer_red", rgba=(0.75, 0.07, 0.04, 1.0))
    cream = model.material("cream_panels", rgba=(0.94, 0.82, 0.55, 1.0))
    blue = model.material("seat_blue", rgba=(0.05, 0.28, 0.78, 1.0))
    yellow = model.material("seat_yellow", rgba=(0.96, 0.70, 0.08, 1.0))
    steel = model.material("painted_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("tire_rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    safety_red = model.material("safety_red", rgba=(0.86, 0.06, 0.05, 1.0))

    trailer = model.part("trailer")
    trailer.visual(
        Box((1.60, 0.82, 0.16)),
        origin=Origin(xyz=(0.05, 0.0, 0.30)),
        material=trailer_red,
        name="deck",
    )
    trailer.visual(
        Box((1.22, 0.60, 0.055)),
        origin=Origin(xyz=(0.05, 0.0, 0.405)),
        material=cream,
        name="deck_panel",
    )
    trailer.visual(
        Box((0.16, 0.95, 0.055)),
        origin=Origin(xyz=(-0.43, 0.0, 0.225)),
        material=dark_steel,
        name="road_axle",
    )
    for y, name in [(-0.52, "road_wheel_0"), (0.52, "road_wheel_1")]:
        trailer.visual(
            Cylinder(radius=0.205, length=0.090),
            origin=Origin(xyz=(-0.43, y, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=name,
        )
        trailer.visual(
            Cylinder(radius=0.090, length=0.100),
            origin=Origin(xyz=(-0.43, y, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"{name}_hub",
        )

    # Trailer tongue and deployed stabilizers make the portable base read as a
    # roadable carnival unit rather than a permanent foundation.
    _add_tube(
        trailer,
        "tow_tongue.obj",
        [(-0.72, 0.0, 0.30), (-1.24, 0.0, 0.22)],
        radius=0.030,
        material=dark_steel,
    )
    trailer.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(-1.25, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hitch_eye",
    )
    for x in (-0.62, 0.62):
        for y in (-0.58, 0.58):
            trailer.visual(
                Box((0.055, 0.42, 0.050)),
                origin=Origin(xyz=(x, y * 0.77, 0.255)),
                material=dark_steel,
                name=f"outrigger_{x}_{y}",
            )
            trailer.visual(
                Box((0.180, 0.120, 0.035)),
                origin=Origin(xyz=(x, y * 1.04, 0.045)),
                material=steel,
                name=f"footpad_{x}_{y}",
            )
            trailer.visual(
                Cylinder(radius=0.022, length=0.230),
                origin=Origin(xyz=(x, y * 1.04, 0.160)),
                material=dark_steel,
                name=f"jack_{x}_{y}",
            )

    # Twin A-frame towers stand outside the rotating wheel and support the axle.
    axle_z = WHEEL_CENTER_Z
    for side_y in (-0.52, 0.52):
        trailer.visual(
            Box((1.28, 0.16, 0.10)),
            origin=Origin(xyz=(0.0, side_y * 0.90, 0.385)),
            material=dark_steel,
            name=f"tower_sill_{side_y}",
        )
        _add_tube(
            trailer,
            f"tower_leg_front_{side_y}.obj",
            [(-0.58, side_y, 0.39), (-0.23, side_y, 1.05), (0.0, side_y, axle_z)],
            radius=0.036,
            material=dark_steel,
        )
        _add_tube(
            trailer,
            f"tower_leg_rear_{side_y}.obj",
            [(0.58, side_y, 0.39), (0.23, side_y, 1.05), (0.0, side_y, axle_z)],
            radius=0.036,
            material=dark_steel,
        )
        _add_tube(
            trailer,
            f"tower_crossbrace_{side_y}.obj",
            [(-0.45, side_y, 0.58), (0.45, side_y, 1.28)],
            radius=0.020,
            material=steel,
        )
        _add_tube(
            trailer,
            f"tower_crossbrace_b_{side_y}.obj",
            [(0.45, side_y, 0.58), (-0.45, side_y, 1.28)],
            radius=0.020,
            material=steel,
        )
        trailer.visual(
            Box((0.24, 0.16, 0.16)),
            origin=Origin(xyz=(0.0, side_y, axle_z)),
            material=dark_steel,
            name=f"axle_pillow_{side_y}",
        )
    trailer.visual(
        Cylinder(radius=0.055, length=1.10),
        origin=Origin(xyz=(0.0, 0.0, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle_bar",
    )
    for y in (-0.52, 0.52):
        trailer.visual(
            Box((0.18, 0.070, 0.18)),
            origin=Origin(xyz=(0.0, y, axle_z)),
            material=dark_steel,
            name=f"bearing_block_{y}",
        )

    wheel = model.part("wheel")
    wheel.visual(
        _save_mesh(TorusGeometry(RIM_RADIUS, 0.026, radial_segments=20, tubular_segments=96).rotate_x(math.pi / 2.0), "outer_rim_a.obj"),
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
        material=safety_red,
        name="rim_0",
    )
    wheel.visual(
        _save_mesh(TorusGeometry(RIM_RADIUS, 0.026, radial_segments=20, tubular_segments=96).rotate_x(math.pi / 2.0), "outer_rim_b.obj"),
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        material=safety_red,
        name="rim_1",
    )
    wheel.visual(
        _save_mesh(TorusGeometry(RIM_RADIUS, 0.020, radial_segments=18, tubular_segments=96).rotate_x(math.pi / 2.0), "side_rim_a.obj"),
        origin=Origin(xyz=(0.0, -0.240, 0.0)),
        material=safety_red,
        name="side_rim_0",
    )
    wheel.visual(
        _save_mesh(TorusGeometry(RIM_RADIUS, 0.020, radial_segments=18, tubular_segments=96).rotate_x(math.pi / 2.0), "side_rim_b.obj"),
        origin=Origin(xyz=(0.0, 0.240, 0.0)),
        material=safety_red,
        name="side_rim_1",
    )
    wheel.visual(
        Cylinder(radius=0.165, length=0.560),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="central_hub",
    )
    wheel.visual(
        Cylinder(radius=0.060, length=0.600),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub_cap",
    )

    for index in range(SEAT_COUNT):
        angle = -math.pi / 2.0 + index * 2.0 * math.pi / SEAT_COUNT
        spoke_angle = angle + math.pi / SEAT_COUNT
        spoke_y = 0.240 if index % 2 == 0 else -0.240
        spoke_inner = 0.130
        spoke_outer = RIM_RADIUS + 0.050
        spoke_length = spoke_outer - spoke_inner
        spoke_mid = _point_on_wheel((spoke_outer + spoke_inner) * 0.5, spoke_angle, spoke_y)
        wheel.visual(
            Box((spoke_length, 0.028, 0.028)),
            origin=Origin(xyz=spoke_mid, rpy=(0.0, -spoke_angle, 0.0)),
            material=steel,
            name=f"spoke_{index}",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.520),
            origin=Origin(xyz=_point_on_wheel(RIM_RADIUS, angle, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"rim_tie_{index}",
        )

        pivot = _point_on_wheel(PIVOT_RADIUS, angle, 0.0)
        for y_side in (-0.320, 0.320):
            _add_tube(
                wheel,
                f"hanger_arm_{index}_{y_side}.obj",
                [
                    _radial_midpoint(RIM_RADIUS - 0.010, angle, y_side * 0.75),
                    _radial_midpoint((RIM_RADIUS + PIVOT_RADIUS) * 0.5, angle, y_side),
                    (pivot[0], y_side, pivot[2]),
                ],
                radius=0.012,
                material=dark_steel,
                segments=10,
            )
        wheel.visual(
            Cylinder(radius=0.016, length=0.800),
            origin=Origin(xyz=pivot, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"pivot_pin_{index}",
        )
        cheek_pitch = -angle
        for y_side in (-0.370, 0.370):
            wheel.visual(
                Box((0.070, 0.024, 0.082)),
                origin=Origin(xyz=(pivot[0], y_side, pivot[2]), rpy=(0.0, cheek_pitch, 0.0)),
                material=dark_steel,
                name=f"yoke_cheek_{index}_{0 if y_side < 0 else 1}",
            )

    wheel_spin = model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=trailer,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, WHEEL_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8),
    )

    for index in range(SEAT_COUNT):
        angle = -math.pi / 2.0 + index * 2.0 * math.pi / SEAT_COUNT
        pivot = _point_on_wheel(PIVOT_RADIUS, angle, 0.0)
        seat = model.part(f"seat_{index}")
        seat_material = blue if index % 2 == 0 else yellow
        _add_open_seat_visuals(seat, body_material=seat_material, frame_material=dark_steel)
        model.articulation(
            f"seat_pivot_{index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=seat,
            origin=Origin(xyz=pivot),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0),
            mimic=Mimic(joint="wheel_spin", multiplier=-1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wheel = object_model.get_part("wheel")
    trailer = object_model.get_part("trailer")
    wheel_spin = object_model.get_articulation("wheel_spin")

    ctx.allow_overlap(
        trailer,
        wheel,
        elem_a="axle_bar",
        elem_b="hub_cap",
        reason="The fixed trailer axle is intentionally represented as passing through the rotating wheel hub.",
    )
    ctx.allow_overlap(
        trailer,
        wheel,
        elem_a="axle_bar",
        elem_b="central_hub",
        reason="The rotating hub body is intentionally sleeved around the fixed trailer axle.",
    )
    ctx.expect_within(
        trailer,
        wheel,
        axes="xz",
        inner_elem="axle_bar",
        outer_elem="hub_cap",
        margin=0.006,
        name="axle shaft stays centered inside wheel hub",
    )
    ctx.expect_overlap(
        trailer,
        wheel,
        axes="y",
        elem_a="axle_bar",
        elem_b="hub_cap",
        min_overlap=0.50,
        name="axle shaft remains inserted through wheel hub",
    )
    ctx.expect_overlap(
        trailer,
        wheel,
        axes="y",
        elem_a="axle_bar",
        elem_b="central_hub",
        min_overlap=0.50,
        name="axle shaft remains inside main hub body",
    )

    for index in range(SEAT_COUNT):
        seat = object_model.get_part(f"seat_{index}")
        ctx.allow_overlap(
            wheel,
            seat,
            elem_a=f"pivot_pin_{index}",
            elem_b="pivot_sleeve",
            reason="The seat sleeve is intentionally captured around the wheel hanger pin so the gondola cannot shed from the rim.",
        )
        ctx.expect_within(
            "seat_%d" % index,
            "wheel",
            axes="xz",
            inner_elem="pivot_sleeve",
            outer_elem=f"pivot_pin_{index}",
            margin=0.014,
            name=f"seat_{index} sleeve centered on hanger pin",
        )
        ctx.expect_overlap(
            seat,
            wheel,
            axes="y",
            elem_a="pivot_sleeve",
            elem_b=f"pivot_pin_{index}",
            min_overlap=0.150,
            name=f"seat_{index} sleeve retained along pin",
        )

    with ctx.pose({wheel_spin: 1.25}):
        for index in (0, 2, 5):
            seat = object_model.get_part(f"seat_{index}")
            pivot_position = ctx.part_world_position(seat)
            pan_aabb = ctx.part_element_world_aabb(seat, elem="seat_pan")
            ok = pivot_position is not None and pan_aabb is not None and pan_aabb[1][2] < pivot_position[2] - 0.22
            ctx.check(
                f"seat_{index} hangs below pivot during rotation",
                ok,
                details=f"pivot={pivot_position}, pan_aabb={pan_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
