from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, radians, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * cos(2.0 * pi * i / segments),
            radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _normalize(vec: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vec
    mag = sqrt(x * x + y * y + z * z)
    return (x / mag, y / mag, z / mag)


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=(
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
            0.5 * (start[2] + end[2]),
        ),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _arc_points(
    center: tuple[float, float, float],
    radius: float,
    start_deg: float,
    end_deg: float,
    steps: int,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for i in range(steps + 1):
        t = i / steps
        angle = radians(start_deg + (end_deg - start_deg) * t)
        points.append(
            (
                center[0] + radius * sin(angle),
                center[1],
                center[2] + radius * cos(angle),
            )
        )
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="executive_floor_globe")

    walnut = model.material("walnut", rgba=(0.30, 0.20, 0.12, 1.0))
    dark_wood = model.material("dark_wood", rgba=(0.22, 0.14, 0.09, 1.0))
    antique_brass = model.material("antique_brass", rgba=(0.71, 0.56, 0.25, 1.0))
    bronze_shadow = model.material("bronze_shadow", rgba=(0.34, 0.27, 0.14, 1.0))
    ocean_blue = model.material("ocean_blue", rgba=(0.19, 0.37, 0.56, 1.0))
    parchment = model.material("parchment", rgba=(0.87, 0.82, 0.69, 1.0))

    sphere_radius = 0.21
    globe_center = (0.0, 0.0, 0.79)
    collar_center_z = 0.50
    collar_height = 0.028
    collar_outer_radius = 0.058
    collar_inner_radius = 0.029
    shelf_radius = 0.175
    shelf_height = 0.014
    shelf_center_z = 0.432
    tripod_hub_z = 0.33
    pin_half_length = sphere_radius + 0.016

    stand = model.part("stand")

    stand.visual(
        Cylinder(radius=0.045, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, tripod_hub_z), rpy=(0.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub",
    )

    pedestal_origin, pedestal_length = _segment_origin(
        (0.0, 0.0, tripod_hub_z + 0.030),
        (0.0, 0.0, shelf_center_z - 0.5 * shelf_height),
    )
    stand.visual(
        Cylinder(radius=0.026, length=pedestal_length),
        origin=pedestal_origin,
        material=dark_wood,
        name="pedestal",
    )

    stand.visual(
        Cylinder(radius=shelf_radius, length=shelf_height),
        origin=Origin(xyz=(0.0, 0.0, shelf_center_z)),
        material=walnut,
        name="shelf",
    )
    stand.visual(
        mesh_from_geometry(TorusGeometry(radius=0.162, tube=0.010), "shelf_rim"),
        origin=Origin(xyz=(0.0, 0.0, shelf_center_z + 0.008)),
        material=dark_wood,
        name="shelf_rim",
    )

    collar_geom = ExtrudeWithHolesGeometry(
        _circle_profile(collar_outer_radius, segments=56),
        [_circle_profile(collar_inner_radius, segments=56)],
        height=collar_height,
        center=True,
    )
    stand.visual(
        mesh_from_geometry(collar_geom, "collar"),
        origin=Origin(xyz=(0.0, 0.0, collar_center_z)),
        material=antique_brass,
        name="collar",
    )

    standoff_radius = 0.056
    standoff_top = collar_center_z - 0.5 * collar_height
    standoff_bottom = shelf_center_z + 0.5 * shelf_height
    for index, angle_deg in enumerate((30.0, 150.0, 270.0), start=1):
        angle = radians(angle_deg)
        x = standoff_radius * cos(angle)
        y = standoff_radius * sin(angle)
        stand.visual(
            Cylinder(radius=0.009, length=standoff_top - standoff_bottom),
            origin=Origin(xyz=(x, y, 0.5 * (standoff_top + standoff_bottom))),
            material=bronze_shadow,
            name=f"standoff_{index}",
        )

    foot_radius = 0.032
    foot_height = 0.014
    leg_top = (0.0, 0.0, tripod_hub_z + 0.008)
    for index, angle_deg in enumerate((0.0, 120.0, 240.0), start=1):
        angle = radians(angle_deg)
        foot_center = (0.31 * cos(angle), 0.31 * sin(angle), 0.5 * foot_height)
        stand.visual(
            Cylinder(radius=foot_radius, length=foot_height),
            origin=Origin(xyz=foot_center),
            material=dark_wood,
            name=f"foot_{index}",
        )
        foot_top = (foot_center[0], foot_center[1], foot_height)
        leg_origin, leg_length = _segment_origin(leg_top, foot_top)
        stand.visual(
            Cylinder(radius=0.017, length=leg_length),
            origin=leg_origin,
            material=walnut,
            name=f"leg_{index}",
        )

    upper_support = model.part("upper_support")
    upper_support.visual(
        Cylinder(radius=0.021, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bronze_shadow,
        name="spindle",
    )
    upper_support.visual(
        Cylinder(radius=0.042, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=antique_brass,
        name="shoulder",
    )
    upper_support.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=antique_brass,
        name="clip_flange",
    )

    meridian_radius = 0.245
    upper_globe_center = (0.0, 0.0, globe_center[2] - collar_center_z)
    frame_start = (0.0, 0.0, 0.024)
    meridian_arc = _arc_points(upper_globe_center, meridian_radius, -160.0, 160.0, 24)
    frame_path = [frame_start, meridian_arc[0], *meridian_arc[1:], frame_start]
    upper_support.visual(
        mesh_from_geometry(
            wire_from_points(
                frame_path,
                radius=0.007,
                radial_segments=18,
                closed_path=False,
                cap_ends=False,
                corner_mode="fillet",
                corner_radius=0.024,
                corner_segments=8,
            ),
            "support_frame",
        ),
        material=antique_brass,
        name="support_frame",
    )

    tilt = radians(23.5)
    polar_axis = _normalize((sin(tilt), 0.0, cos(tilt)))
    bearing_half_length = 0.009
    cup_offset = pin_half_length + bearing_half_length
    for name, sign in (("north_pivot", 1.0), ("south_pivot", -1.0)):
        center = (
            upper_globe_center[0] + sign * cup_offset * polar_axis[0],
            upper_globe_center[1] + sign * cup_offset * polar_axis[1],
            upper_globe_center[2] + sign * cup_offset * polar_axis[2],
        )
        seg_origin, seg_length = _segment_origin(
            (
                center[0] - bearing_half_length * polar_axis[0],
                center[1] - bearing_half_length * polar_axis[1],
                center[2] - bearing_half_length * polar_axis[2],
            ),
            (
                center[0] + bearing_half_length * polar_axis[0],
                center[1] + bearing_half_length * polar_axis[1],
                center[2] + bearing_half_length * polar_axis[2],
            ),
        )
        upper_support.visual(
            Cylinder(radius=0.012, length=seg_length),
            origin=seg_origin,
            material=bronze_shadow,
            name=name,
        )

    globe = model.part("globe")
    globe.visual(
        Sphere(radius=sphere_radius),
        material=ocean_blue,
        name="sphere",
    )
    pin_origin, pin_length = _segment_origin(
        (
            -pin_half_length * polar_axis[0],
            -pin_half_length * polar_axis[1],
            -pin_half_length * polar_axis[2],
        ),
        (
            pin_half_length * polar_axis[0],
            pin_half_length * polar_axis[1],
            pin_half_length * polar_axis[2],
        ),
    )
    globe.visual(
        Cylinder(radius=0.005, length=pin_length),
        origin=pin_origin,
        material=parchment,
        name="axis_pins",
    )

    model.articulation(
        "collar_turn",
        ArticulationType.CONTINUOUS,
        parent=stand,
        child=upper_support,
        origin=Origin(xyz=(0.0, 0.0, collar_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "globe_spin",
        ArticulationType.CONTINUOUS,
        parent=upper_support,
        child=globe,
        origin=Origin(xyz=upper_globe_center),
        axis=polar_axis,
        motion_limits=MotionLimits(effort=3.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    upper_support = object_model.get_part("upper_support")
    globe = object_model.get_part("globe")
    collar_turn = object_model.get_articulation("collar_turn")
    globe_spin = object_model.get_articulation("globe_spin")

    stand.get_visual("collar")
    stand.get_visual("shelf")
    upper_support.get_visual("shoulder")
    upper_support.get_visual("clip_flange")
    upper_support.get_visual("spindle")
    globe.get_visual("sphere")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "collar_turn_axis_vertical",
        tuple(round(v, 6) for v in collar_turn.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical collar axis, got {collar_turn.axis!r}",
    )
    ctx.check(
        "globe_spin_axis_tilted",
        abs(globe_spin.axis[1]) < 1e-6 and 0.35 < abs(globe_spin.axis[0]) < 0.45 and globe_spin.axis[2] > 0.9,
        details=f"Expected a realistic tilted polar axis, got {globe_spin.axis!r}",
    )
    ctx.check(
        "globe_centered_over_tripod",
        ctx.expect_origin_distance(globe, stand, axes="xy", max_dist=0.001, name="globe_xy_centering"),
        details="Globe origin drifted off the tripod centerline.",
    )

    with ctx.pose({collar_turn: 0.0}):
        ctx.expect_gap(
            upper_support,
            stand,
            axis="z",
            positive_elem="shoulder",
            negative_elem="collar",
            max_gap=0.0005,
            max_penetration=0.0,
            name="shoulder_seats_on_collar",
        )
        ctx.expect_gap(
            stand,
            upper_support,
            axis="z",
            positive_elem="collar",
            negative_elem="clip_flange",
            max_gap=0.0005,
            max_penetration=0.0,
            name="clip_captures_support_below_collar",
        )
        ctx.expect_overlap(
            upper_support,
            stand,
            axes="xy",
            elem_a="spindle",
            elem_b="collar",
            min_overlap=0.04,
            name="spindle_stays_inside_collar",
        )
        ctx.expect_gap(
            globe,
            stand,
            axis="z",
            positive_elem="sphere",
            negative_elem="shelf",
            min_gap=0.14,
            name="globe_clears_map_shelf",
        )
        ctx.expect_overlap(
            globe,
            stand,
            axes="xy",
            elem_a="sphere",
            elem_b="shelf",
            min_overlap=0.30,
            name="globe_remains_over_shelf",
        )

    with ctx.pose({collar_turn: 1.1}):
        ctx.expect_gap(
            upper_support,
            stand,
            axis="z",
            positive_elem="shoulder",
            negative_elem="collar",
            max_gap=0.0005,
            max_penetration=0.0,
            name="rotated_support_stays_seated",
        )
        ctx.expect_gap(
            stand,
            upper_support,
            axis="z",
            positive_elem="collar",
            negative_elem="clip_flange",
            max_gap=0.0005,
            max_penetration=0.0,
            name="rotated_support_stays_captured",
        )
        ctx.expect_overlap(
            upper_support,
            stand,
            axes="xy",
            elem_a="spindle",
            elem_b="collar",
            min_overlap=0.04,
            name="rotated_spindle_stays_inside_collar",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
