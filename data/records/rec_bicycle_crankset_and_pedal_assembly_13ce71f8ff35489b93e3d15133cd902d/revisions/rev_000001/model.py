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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _chainring_outer_profile(
    *,
    teeth: int,
    root_radius: float,
    tip_radius: float,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    step = (2.0 * math.pi) / teeth
    for tooth_index in range(teeth):
        base = tooth_index * step
        points.append(
            (
                root_radius * math.cos(base),
                root_radius * math.sin(base),
            )
        )
        points.append(
            (
                tip_radius * math.cos(base + step * 0.32),
                tip_radius * math.sin(base + step * 0.32),
            )
        )
        points.append(
            (
                tip_radius * math.cos(base + step * 0.68),
                tip_radius * math.sin(base + step * 0.68),
            )
        )
        points.append(
            (
                root_radius * math.cos(base + step),
                root_radius * math.sin(base + step),
            )
        )
    return points


def _build_chainring_mesh():
    outer_profile = _chainring_outer_profile(
        teeth=25,
        root_radius=0.082,
        tip_radius=0.089,
    )
    center_hole = _circle_profile(0.030, segments=42)
    window = rounded_rect_profile(0.024, 0.040, 0.005, corner_segments=8)
    holes = [center_hole]
    for angle in (
        math.radians(30.0),
        math.radians(120.0),
        math.radians(210.0),
        math.radians(300.0),
    ):
        holes.append(
            _transform_profile(
                window,
                dx=0.044 * math.cos(angle),
                dy=0.044 * math.sin(angle),
                angle=angle,
            )
        )
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            holes,
            height=0.006,
            center=True,
        ),
        "bmx_chainring",
    )


def _build_ring_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 48,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=segments),
            [_circle_profile(inner_radius, segments=segments)],
            height=thickness,
            center=True,
        ),
        name,
    )


def _build_crank_mesh(name: str, *, pedal_x: float, pedal_z: float):
    arm_profile = rounded_rect_profile(0.026, 0.016, 0.0035, corner_segments=6)
    arm_geom = sweep_profile_along_spline(
        [
            (pedal_x * 0.22, 0.0, pedal_z * 0.18),
            (pedal_x * 0.34, 0.0, pedal_z * 0.28),
            (pedal_x * 0.68, 0.0, pedal_z * 0.72),
            (pedal_x, 0.0, pedal_z),
        ],
        profile=arm_profile,
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(arm_geom, name)


def _add_crank_visuals(
    part,
    *,
    arm_mesh,
    side_sign: float,
    pedal_x: float,
    pedal_z: float,
    arm_material,
    hardware_material,
) -> None:
    part.visual(arm_mesh, material=arm_material, name="crank_arm")
    part.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(
            xyz=(0.0, side_sign * 0.009, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=arm_material,
        name="spindle_boss",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(
            xyz=(pedal_x, side_sign * 0.013, pedal_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_material,
        name="pedal_eye",
    )
    part.visual(
        Box((0.034, 0.024, 0.036)),
        origin=Origin(
            xyz=(pedal_x * 0.12, side_sign * 0.0125, pedal_z * 0.13),
        ),
        material=arm_material,
        name="arm_root_bridge",
    )


def _add_pedal_visuals(part, *, side_sign: float, body_material, axle_material) -> None:
    body_offset_y = side_sign * 0.060
    barrel_offset_y = side_sign * 0.040
    part.visual(
        Box((0.106, 0.068, 0.014)),
        origin=Origin(xyz=(0.0, body_offset_y, 0.0)),
        material=body_material,
        name="platform_body",
    )
    part.visual(
        Box((0.016, 0.070, 0.018)),
        origin=Origin(xyz=(0.044, body_offset_y, 0.0)),
        material=body_material,
        name="front_cage",
    )
    part.visual(
        Box((0.016, 0.070, 0.018)),
        origin=Origin(xyz=(-0.044, body_offset_y, 0.0)),
        material=body_material,
        name="rear_cage",
    )
    part.visual(
        Box((0.038, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, barrel_offset_y, 0.0)),
        material=body_material,
        name="pedal_barrel_block",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.054),
        origin=Origin(
            xyz=(0.0, side_sign * 0.053, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=axle_material,
        name="pedal_barrel",
    )
    for x_pos in (-0.032, 0.0, 0.032):
        part.visual(
            Box((0.008, 0.070, 0.004)),
            origin=Origin(xyz=(x_pos, body_offset_y, 0.009)),
            material=axle_material,
            name=f"top_pin_strip_{int((x_pos + 0.032) * 1000):03d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bmx_freestyle_crankset")

    shell_black = model.material("shell_black", rgba=(0.12, 0.12, 0.13, 1.0))
    arm_black = model.material("arm_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.09, 0.09, 0.10, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    chainring_steel = model.material("chainring_steel", rgba=(0.42, 0.43, 0.45, 1.0))

    chainring_mesh = _build_chainring_mesh()
    bearing_seat_mesh = _build_ring_mesh(
        "bmx_bearing_seat",
        outer_radius=0.023,
        inner_radius=0.016,
        thickness=0.002,
    )
    right_crank_mesh = _build_crank_mesh(
        "bmx_right_crank",
        pedal_x=0.040,
        pedal_z=-0.148,
    )
    left_crank_mesh = _build_crank_mesh(
        "bmx_left_crank",
        pedal_x=-0.040,
        pedal_z=0.148,
    )

    bottom_bracket_shell = model.part("bottom_bracket_shell")
    shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.022, -0.034),
                (0.025, -0.031),
                (0.027, -0.024),
                (0.0275, 0.0),
                (0.027, 0.024),
                (0.025, 0.031),
                (0.022, 0.034),
            ],
            [
                (0.0185, -0.031),
                (0.0195, 0.0),
                (0.0185, 0.031),
            ],
            segments=72,
        ),
        "bmx_bottom_bracket_shell",
    )
    bottom_bracket_shell.visual(
        shell_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_black,
        name="shell_body",
    )
    for side_sign in (-1.0, 1.0):
        bottom_bracket_shell.visual(
            bearing_seat_mesh,
            origin=Origin(
                xyz=(0.0, side_sign * 0.033, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=spindle_steel,
            name=f"bearing_seat_{'right' if side_sign > 0 else 'left'}",
        )
    bottom_bracket_shell.inertial = Inertial.from_geometry(
        Box((0.076, 0.070, 0.058)),
        mass=0.70,
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.012, length=0.184),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spindle_steel,
        name="main_spindle",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(
            xyz=(0.0, -0.039, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="bearing_collar_left",
    )
    spindle.visual(
        Cylinder(radius=0.0218, length=0.002),
        origin=Origin(
            xyz=(0.0, -0.035, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="bearing_face_left",
    )
    spindle.visual(
        Cylinder(radius=0.016, length=0.046),
        origin=Origin(
            xyz=(0.0, -0.069, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="spline_end_left",
    )
    spindle.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.039, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="bearing_collar_right",
    )
    spindle.visual(
        Cylinder(radius=0.0218, length=0.002),
        origin=Origin(
            xyz=(0.0, 0.035, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="bearing_face_right",
    )
    spindle.visual(
        Cylinder(radius=0.016, length=0.046),
        origin=Origin(
            xyz=(0.0, 0.069, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=spindle_steel,
        name="spline_end_right",
    )
    spindle.inertial = Inertial.from_geometry(
        Box((0.040, 0.184, 0.040)),
        mass=1.20,
    )

    right_crank = model.part("right_crank")
    _add_crank_visuals(
        right_crank,
        arm_mesh=right_crank_mesh,
        side_sign=1.0,
        pedal_x=0.040,
        pedal_z=-0.148,
        arm_material=arm_black,
        hardware_material=spindle_steel,
    )
    right_crank.visual(
        Box((0.042, 0.006, 0.018)),
        origin=Origin(xyz=(0.028, -0.010, -0.026)),
        material=arm_black,
        name="chainring_mount_tab",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.160, 0.032, 0.165)),
        mass=0.62,
        origin=Origin(xyz=(0.020, 0.010, -0.074)),
    )

    left_crank = model.part("left_crank")
    _add_crank_visuals(
        left_crank,
        arm_mesh=left_crank_mesh,
        side_sign=-1.0,
        pedal_x=-0.040,
        pedal_z=0.148,
        arm_material=arm_black,
        hardware_material=spindle_steel,
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.160, 0.032, 0.165)),
        mass=0.62,
        origin=Origin(xyz=(-0.020, -0.010, 0.074)),
    )

    chainring = model.part("chainring")
    chainring.visual(
        chainring_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chainring_steel,
        name="chainring_plate",
    )
    chainring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.089, length=0.006),
        mass=0.38,
    )

    right_pedal = model.part("right_pedal")
    _add_pedal_visuals(
        right_pedal,
        side_sign=1.0,
        body_material=pedal_black,
        axle_material=spindle_steel,
    )
    right_pedal.inertial = Inertial.from_geometry(
        Box((0.108, 0.090, 0.022)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    _add_pedal_visuals(
        left_pedal,
        side_sign=-1.0,
        body_material=pedal_black,
        axle_material=spindle_steel,
    )
    left_pedal.inertial = Inertial.from_geometry(
        Box((0.108, 0.090, 0.022)),
        mass=0.34,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
    )

    model.articulation(
        "shell_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bottom_bracket_shell,
        child=spindle,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=18.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.0, 0.092, 0.0)),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(0.0, -0.092, 0.0)),
    )
    model.articulation(
        "right_crank_to_chainring",
        ArticulationType.FIXED,
        parent=right_crank,
        child=chainring,
        origin=Origin(xyz=(0.012, -0.016, 0.0)),
    )
    model.articulation(
        "right_crank_to_right_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal,
        origin=Origin(xyz=(0.040, 0.0, -0.148)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=35.0),
    )
    model.articulation(
        "left_crank_to_left_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal,
        origin=Origin(xyz=(-0.040, 0.0, 0.148)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottom_bracket_shell = object_model.get_part("bottom_bracket_shell")
    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    chainring = object_model.get_part("chainring")
    right_pedal = object_model.get_part("right_pedal")
    left_pedal = object_model.get_part("left_pedal")

    spindle_spin = object_model.get_articulation("shell_to_spindle")
    right_pedal_spin = object_model.get_articulation("right_crank_to_right_pedal")
    left_pedal_spin = object_model.get_articulation("left_crank_to_left_pedal")

    ctx.expect_gap(
        right_crank,
        spindle,
        axis="y",
        positive_elem="spindle_boss",
        negative_elem="spline_end_right",
        min_gap=0.0,
        max_gap=0.0,
        name="right crank seats flush on spindle end",
    )
    ctx.expect_gap(
        spindle,
        left_crank,
        axis="y",
        positive_elem="spline_end_left",
        negative_elem="spindle_boss",
        min_gap=0.0,
        max_gap=0.0,
        name="left crank seats flush on spindle end",
    )
    ctx.expect_gap(
        chainring,
        bottom_bracket_shell,
        axis="y",
        min_gap=0.030,
        name="chainring clears the bottom bracket shell",
    )
    ctx.expect_origin_gap(
        right_pedal,
        left_pedal,
        axis="y",
        min_gap=0.16,
        name="pedals remain on opposite sides of the spindle",
    )

    rest_right = ctx.part_world_position(right_pedal)
    rest_left = ctx.part_world_position(left_pedal)
    with ctx.pose({spindle_spin: math.pi / 2.0}):
        quarter_right = ctx.part_world_position(right_pedal)
        quarter_left = ctx.part_world_position(left_pedal)
    ctx.check(
        "spindle rotation lifts the right pedal and drops the left pedal",
        rest_right is not None
        and rest_left is not None
        and quarter_right is not None
        and quarter_left is not None
        and quarter_right[2] > rest_right[2] + 0.12
        and quarter_left[2] < rest_left[2] - 0.12,
        details=(
            f"rest_right={rest_right}, quarter_right={quarter_right}, "
            f"rest_left={rest_left}, quarter_left={quarter_left}"
        ),
    )

    def _extents(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return (
            max_corner[0] - min_corner[0],
            max_corner[1] - min_corner[1],
            max_corner[2] - min_corner[2],
        )

    with ctx.pose({spindle_spin: 0.0}):
        right_pedal_flat = _extents(ctx.part_element_world_aabb(right_pedal, elem="platform_body"))
        left_pedal_flat = _extents(ctx.part_element_world_aabb(left_pedal, elem="platform_body"))
        with ctx.pose({right_pedal_spin: math.pi / 2.0, left_pedal_spin: math.pi / 2.0}):
            right_pedal_turned = _extents(
                ctx.part_element_world_aabb(right_pedal, elem="platform_body")
            )
            left_pedal_turned = _extents(
                ctx.part_element_world_aabb(left_pedal, elem="platform_body")
            )

    ctx.check(
        "right pedal rotates about its axle",
        right_pedal_flat is not None
        and right_pedal_turned is not None
        and right_pedal_flat[0] > right_pedal_flat[2] + 0.05
        and right_pedal_turned[2] > right_pedal_turned[0] + 0.05,
        details=f"flat={right_pedal_flat}, turned={right_pedal_turned}",
    )
    ctx.check(
        "left pedal rotates about its axle",
        left_pedal_flat is not None
        and left_pedal_turned is not None
        and left_pedal_flat[0] > left_pedal_flat[2] + 0.05
        and left_pedal_turned[2] > left_pedal_turned[0] + 0.05,
        details=f"flat={left_pedal_flat}, turned={left_pedal_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
