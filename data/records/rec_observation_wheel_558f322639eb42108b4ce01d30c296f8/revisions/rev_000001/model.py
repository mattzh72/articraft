from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def _add_tower(part, *, side_sign: float, frame_material, hardware_material) -> None:
    front_foot = (0.24, 0.0, 0.06)
    rear_foot = (-0.24, 0.0, 0.06)
    apex = (0.0, 0.0, 1.44)
    front_mid = (0.12, 0.0, 0.76)
    rear_mid = (-0.12, 0.0, 0.76)

    part.visual(
        Box((0.18, 0.08, 0.12)),
        origin=Origin(xyz=(front_foot[0], 0.0, 0.06)),
        material=frame_material,
        name="front_foot",
    )
    part.visual(
        Box((0.18, 0.08, 0.12)),
        origin=Origin(xyz=(rear_foot[0], 0.0, 0.06)),
        material=frame_material,
        name="rear_foot",
    )
    _add_member(part, front_foot, apex, 0.032, frame_material, name="front_leg")
    _add_member(part, rear_foot, apex, 0.032, frame_material, name="rear_leg")
    _add_member(part, front_mid, rear_mid, 0.024, frame_material, name="mid_brace")
    _add_member(part, front_foot, rear_mid, 0.018, hardware_material)
    _add_member(part, rear_foot, front_mid, 0.018, hardware_material)
    _add_member(part, (0.18, 0.0, 0.24), (-0.18, 0.0, 0.24), 0.020, hardware_material)
    part.visual(
        Box((0.26, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 1.38)),
        material=frame_material,
        name="headstock",
    )
    part.visual(
        Cylinder(radius=0.055, length=0.14),
        origin=Origin(
            xyz=(0.0, -0.07 * side_sign, 1.38),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_material,
        name="bearing_stub",
    )


def _add_gondola(part, *, cabin_material, roof_material, trim_material, accent_material) -> None:
    part.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="pivot_bar",
    )
    _add_member(part, (0.0, 0.0, -0.01), (0.0, 0.0, -0.14), 0.012, trim_material, name="hanger_post")
    _add_member(part, (0.0, 0.0, -0.11), (0.0, 0.045, -0.16), 0.010, trim_material)
    _add_member(part, (0.0, 0.0, -0.11), (0.0, -0.045, -0.16), 0.010, trim_material)
    part.visual(
        Box((0.24, 0.14, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.16)),
        material=roof_material,
        name="roof_panel",
    )
    part.visual(
        Box((0.22, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
        material=cabin_material,
        name="cabin_shell",
    )
    part.visual(
        Box((0.20, 0.12, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.315)),
        material=roof_material,
        name="floor_panel",
    )
    part.visual(
        Box((0.05, 0.10, 0.02)),
        origin=Origin(xyz=(0.115, 0.0, -0.32)),
        material=accent_material,
        name="entry_step",
    )


def _add_outrigger(part, *, side_sign: float, frame_material, pad_material) -> None:
    part.visual(
        Cylinder(radius=0.042, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.12, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.17 * side_sign, -0.03)),
        material=frame_material,
        name="outrigger_beam",
    )
    _add_member(
        part,
        (0.0, 0.05 * side_sign, -0.01),
        (0.0, 0.30 * side_sign, -0.11),
        0.018,
        frame_material,
    )
    part.visual(
        Box((0.08, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.31 * side_sign, -0.11)),
        material=frame_material,
        name="jack_leg",
    )
    part.visual(
        Box((0.16, 0.12, 0.025)),
        origin=Origin(xyz=(0.0, 0.31 * side_sign, -0.1825)),
        material=pad_material,
        name="support_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transportable_observation_wheel")

    frame_blue = model.material("frame_blue", rgba=(0.20, 0.34, 0.52, 1.0))
    light_panel = model.material("light_panel", rgba=(0.88, 0.89, 0.86, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    amber = model.material("amber", rgba=(0.76, 0.55, 0.21, 1.0))
    stabilizer_pad = model.material("stabilizer_pad", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("trailer_base")
    base.inertial = Inertial.from_geometry(
        Box((2.60, 1.20, 0.58)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )
    base.visual(
        Box((1.86, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, 0.26, 0.14)),
        material=frame_blue,
        name="left_longeron",
    )
    base.visual(
        Box((1.86, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, -0.26, 0.14)),
        material=frame_blue,
        name="right_longeron",
    )
    for cross_x in (-0.60, 0.00, 0.60):
        base.visual(
            Box((0.64, 0.10, 0.12)),
            origin=Origin(xyz=(cross_x, 0.0, 0.16)),
            material=dark_steel,
        )
    base.visual(
        Box((1.48, 0.70, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=light_panel,
        name="deck",
    )
    base.visual(
        Box((0.72, 0.52, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=frame_blue,
        name="tower_plinth",
    )
    _add_member(base, (0.56, 0.17, 0.22), (1.18, 0.0, 0.20), 0.040, frame_blue)
    _add_member(base, (0.56, -0.17, 0.22), (1.18, 0.0, 0.20), 0.040, frame_blue)
    base.visual(
        Box((0.14, 0.18, 0.14)),
        origin=Origin(xyz=(1.25, 0.0, 0.20)),
        material=dark_steel,
        name="tow_hitch",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.96),
        origin=Origin(xyz=(-0.18, 0.0, 0.32), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="road_axle",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.12),
        origin=Origin(xyz=(-0.18, 0.44, 0.32), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_road_tire",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.12),
        origin=Origin(xyz=(-0.18, -0.44, 0.32), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_road_tire",
    )
    base.visual(
        Box((0.12, 0.056, 0.14)),
        origin=Origin(xyz=(0.52, 0.34, 0.19)),
        material=frame_blue,
        name="left_outrigger_mount",
    )
    base.visual(
        Box((0.12, 0.056, 0.14)),
        origin=Origin(xyz=(0.52, -0.34, 0.19)),
        material=frame_blue,
        name="right_outrigger_mount",
    )

    left_tower = model.part("left_tower")
    left_tower.inertial = Inertial.from_geometry(
        Box((0.52, 0.16, 1.70)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )
    _add_tower(left_tower, side_sign=1.0, frame_material=frame_blue, hardware_material=dark_steel)

    right_tower = model.part("right_tower")
    right_tower.inertial = Inertial.from_geometry(
        Box((0.52, 0.16, 1.70)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )
    _add_tower(right_tower, side_sign=-1.0, frame_material=frame_blue, hardware_material=dark_steel)

    wheel = model.part("main_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.99, length=0.12),
        mass=165.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    rim_angles = [pi / 2.0 + 2.0 * pi * i / 24.0 for i in range(24)]
    front_rim_points = [(0.95 * cos(angle), 0.11, 0.95 * sin(angle)) for angle in rim_angles]
    rear_rim_points = [(0.95 * cos(angle), -0.11, 0.95 * sin(angle)) for angle in rim_angles]
    for idx in range(24):
        _add_member(
            wheel,
            front_rim_points[idx],
            front_rim_points[(idx + 1) % 24],
            0.032,
            frame_blue,
            name="front_rim" if idx == 0 else None,
        )
        _add_member(
            wheel,
            rear_rim_points[idx],
            rear_rim_points[(idx + 1) % 24],
            0.032,
            frame_blue,
            name="rear_rim" if idx == 0 else None,
        )
    wheel.visual(
        Cylinder(radius=0.12, length=0.24),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=0.18, length=0.028),
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    wheel.visual(
        Cylinder(radius=0.18, length=0.028),
        origin=Origin(xyz=(0.0, -0.095, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    spoke_angles = [pi / 2.0 + 2.0 * pi * i / 12.0 for i in range(12)]
    for angle in spoke_angles:
        hub_x = 0.18 * cos(angle)
        hub_z = 0.18 * sin(angle)
        rim_x = 0.95 * cos(angle)
        rim_z = 0.95 * sin(angle)
        _add_member(wheel, (hub_x, 0.095, hub_z), (rim_x, 0.11, rim_z), 0.014, steel)
        _add_member(wheel, (hub_x, -0.095, hub_z), (rim_x, -0.11, rim_z), 0.014, steel)
    gondola_angles = [pi / 2.0 - 2.0 * pi * i / 6.0 for i in range(6)]
    for angle in gondola_angles:
        rim_x = 0.95 * cos(angle)
        rim_z = 0.95 * sin(angle)
        _add_member(wheel, (rim_x, -0.11, rim_z), (rim_x, -0.06, rim_z), 0.018, dark_steel)
        _add_member(wheel, (rim_x, 0.06, rim_z), (rim_x, 0.11, rim_z), 0.018, dark_steel)

    left_outrigger = model.part("left_outrigger")
    left_outrigger.inertial = Inertial.from_geometry(
        Box((0.18, 0.42, 0.22)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.21, -0.08)),
    )
    _add_outrigger(left_outrigger, side_sign=1.0, frame_material=dark_steel, pad_material=stabilizer_pad)

    right_outrigger = model.part("right_outrigger")
    right_outrigger.inertial = Inertial.from_geometry(
        Box((0.18, 0.42, 0.22)),
        mass=22.0,
        origin=Origin(xyz=(0.0, -0.21, -0.08)),
    )
    _add_outrigger(right_outrigger, side_sign=-1.0, frame_material=dark_steel, pad_material=stabilizer_pad)

    model.articulation(
        "base_to_left_tower",
        ArticulationType.FIXED,
        parent=base,
        child=left_tower,
        origin=Origin(xyz=(0.0, 0.26, 0.51)),
    )
    model.articulation(
        "base_to_right_tower",
        ArticulationType.FIXED,
        parent=base,
        child=right_tower,
        origin=Origin(xyz=(0.0, -0.26, 0.51)),
    )
    main_wheel_spin = model.articulation(
        "main_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 1.95)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6),
    )
    left_outrigger_fold = model.articulation(
        "left_outrigger_fold",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_outrigger,
        origin=Origin(xyz=(0.52, 0.41, 0.22)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.15, upper=1.15),
    )
    right_outrigger_fold = model.articulation(
        "right_outrigger_fold",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_outrigger,
        origin=Origin(xyz=(0.52, -0.41, 0.22)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.15, upper=1.15),
    )

    for index, angle in enumerate(gondola_angles):
        gondola = model.part(f"gondola_{index}")
        gondola.inertial = Inertial.from_geometry(
            Box((0.26, 0.20, 0.38)),
            mass=16.0,
            origin=Origin(xyz=(0.0, 0.0, -0.19)),
        )
        _add_gondola(
            gondola,
            cabin_material=light_panel,
            roof_material=amber,
            trim_material=dark_steel,
            accent_material=steel,
        )
        model.articulation(
            f"wheel_to_gondola_{index}",
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=gondola,
            origin=Origin(xyz=(0.95 * cos(angle), 0.0, 0.95 * sin(angle))),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=3.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("trailer_base")
    left_tower = object_model.get_part("left_tower")
    right_tower = object_model.get_part("right_tower")
    wheel = object_model.get_part("main_wheel")
    left_outrigger = object_model.get_part("left_outrigger")
    right_outrigger = object_model.get_part("right_outrigger")
    top_gondola = object_model.get_part("gondola_0")
    bottom_gondola = object_model.get_part("gondola_3")

    wheel_spin = object_model.get_articulation("main_wheel_spin")
    left_fold = object_model.get_articulation("left_outrigger_fold")
    right_fold = object_model.get_articulation("right_outrigger_fold")
    gondola_spin = object_model.get_articulation("wheel_to_gondola_0")

    ctx.expect_contact(
        left_tower,
        base,
        contact_tol=0.002,
        name="left tower is mounted to the trailer base",
    )
    ctx.expect_contact(
        right_tower,
        base,
        contact_tol=0.002,
        name="right tower is mounted to the trailer base",
    )
    ctx.expect_origin_distance(
        left_tower,
        right_tower,
        axes="y",
        min_dist=0.48,
        max_dist=0.58,
        name="support towers straddle the wheel bay",
    )
    ctx.expect_gap(
        wheel,
        base,
        axis="z",
        min_gap=0.30,
        name="main wheel clears the trailer structure",
    )
    ctx.expect_gap(
        bottom_gondola,
        base,
        axis="z",
        positive_elem="cabin_shell",
        min_gap=0.10,
        name="bottom gondola clears the trailer deck",
    )

    rest_top_origin = ctx.part_world_position(top_gondola)
    with ctx.pose({wheel_spin: pi / 3.0}):
        rotated_top_origin = ctx.part_world_position(top_gondola)
    ctx.check(
        "wheel spin carries gondolas around the rim",
        rest_top_origin is not None
        and rotated_top_origin is not None
        and abs(rotated_top_origin[0] - rest_top_origin[0]) > 0.60
        and abs(rotated_top_origin[2] - rest_top_origin[2]) > 0.20,
        details=f"rest={rest_top_origin}, rotated={rotated_top_origin}",
    )

    step_rest = _aabb_center(ctx.part_element_world_aabb(top_gondola, elem="entry_step"))
    with ctx.pose({gondola_spin: 0.8}):
        step_rotated = _aabb_center(ctx.part_element_world_aabb(top_gondola, elem="entry_step"))
    ctx.check(
        "gondola cabin rotates on its own pivot",
        step_rest is not None
        and step_rotated is not None
        and abs(step_rotated[0] - step_rest[0]) > 0.03,
        details=f"rest={step_rest}, rotated={step_rotated}",
    )

    left_pad_deployed = _aabb_center(ctx.part_element_world_aabb(left_outrigger, elem="support_pad"))
    right_pad_deployed = _aabb_center(ctx.part_element_world_aabb(right_outrigger, elem="support_pad"))
    with ctx.pose({left_fold: 0.95, right_fold: 0.95}):
        left_pad_stowed = _aabb_center(ctx.part_element_world_aabb(left_outrigger, elem="support_pad"))
        right_pad_stowed = _aabb_center(ctx.part_element_world_aabb(right_outrigger, elem="support_pad"))
    ctx.check(
        "left outrigger folds upward at its hinge",
        left_pad_deployed is not None
        and left_pad_stowed is not None
        and left_pad_stowed[2] > left_pad_deployed[2] + 0.08,
        details=f"deployed={left_pad_deployed}, stowed={left_pad_stowed}",
    )
    ctx.check(
        "right outrigger folds upward at its hinge",
        right_pad_deployed is not None
        and right_pad_stowed is not None
        and right_pad_stowed[2] > right_pad_deployed[2] + 0.08,
        details=f"deployed={right_pad_deployed}, stowed={right_pad_stowed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
