from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _build_tire_mesh(radius: float, width: float) -> object:
    half_width = width * 0.5
    profile = [
        (radius * 0.48, -half_width * 0.98),
        (radius * 0.70, -half_width),
        (radius * 0.88, -half_width * 0.78),
        (radius * 0.98, -half_width * 0.40),
        (radius, 0.0),
        (radius * 0.98, half_width * 0.40),
        (radius * 0.88, half_width * 0.78),
        (radius * 0.70, half_width),
        (radius * 0.48, half_width * 0.98),
        (radius * 0.40, half_width * 0.34),
        (radius * 0.38, 0.0),
        (radius * 0.40, -half_width * 0.34),
        (radius * 0.48, -half_width * 0.98),
    ]
    return _mesh("wheelbarrow_tire", LatheGeometry(profile, segments=56).rotate_y(pi / 2.0))


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_tray_shell() -> MeshGeometry:
    geom = MeshGeometry()

    outer_floor = [
        (-0.18, -0.31, 0.05),
        (0.18, -0.31, 0.05),
        (0.14, 0.31, 0.00),
        (-0.14, 0.31, 0.00),
    ]
    outer_rim = [
        (-0.27, -0.35, 0.26),
        (0.27, -0.35, 0.26),
        (0.22, 0.38, 0.23),
        (-0.22, 0.38, 0.23),
    ]
    inner_rim = [
        (-0.248, -0.329, 0.237),
        (0.248, -0.329, 0.237),
        (0.198, 0.356, 0.201),
        (-0.198, 0.356, 0.201),
    ]
    inner_floor = [
        (-0.162, -0.292, 0.071),
        (0.162, -0.292, 0.071),
        (0.124, 0.292, 0.021),
        (-0.124, 0.292, 0.021),
    ]

    outer_floor_ids = [geom.add_vertex(*point) for point in outer_floor]
    outer_rim_ids = [geom.add_vertex(*point) for point in outer_rim]
    inner_rim_ids = [geom.add_vertex(*point) for point in inner_rim]
    inner_floor_ids = [geom.add_vertex(*point) for point in inner_floor]

    for index in range(4):
        next_index = (index + 1) % 4
        _add_quad(
            geom,
            outer_floor_ids[index],
            outer_floor_ids[next_index],
            outer_rim_ids[next_index],
            outer_rim_ids[index],
        )
        _add_quad(
            geom,
            outer_rim_ids[index],
            outer_rim_ids[next_index],
            inner_rim_ids[next_index],
            inner_rim_ids[index],
        )
        _add_quad(
            geom,
            inner_floor_ids[next_index],
            inner_floor_ids[index],
            inner_rim_ids[index],
            inner_rim_ids[next_index],
        )

    _add_quad(
        geom,
        outer_floor_ids[0],
        outer_floor_ids[1],
        outer_floor_ids[2],
        outer_floor_ids[3],
    )
    _add_quad(
        geom,
        inner_floor_ids[3],
        inner_floor_ids[2],
        inner_floor_ids[1],
        inner_floor_ids[0],
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.33, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.24, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    handle_grip = model.material("handle_grip", rgba=(0.60, 0.42, 0.22, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.74, 1.55, 0.64)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.04, 0.32)),
    )

    left_rail_points = [
        (-0.28, -0.62, 0.55),
        (-0.27, -0.30, 0.44),
        (-0.25, 0.05, 0.36),
        (-0.16, 0.42, 0.28),
        (-0.10, 0.64, 0.22),
    ]
    rail_radius = 0.019
    frame.visual(
        _mesh(
            "wheelbarrow_left_rail",
            tube_from_spline_points(
                left_rail_points,
                radius=rail_radius,
                samples_per_segment=16,
                radial_segments=16,
            ),
        ),
        material=painted_steel,
        name="left_rail",
    )
    frame.visual(
        _mesh(
            "wheelbarrow_right_rail",
            tube_from_spline_points(
                _mirror_x(left_rail_points),
                radius=rail_radius,
                samples_per_segment=16,
                radial_segments=16,
            ),
        ),
        material=painted_steel,
        name="right_rail",
    )

    frame.visual(
        Cylinder(radius=0.018, length=0.56),
        origin=Origin(xyz=(0.0, -0.28, 0.43), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="rear_crossbrace",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.48),
        origin=Origin(xyz=(0.0, 0.05, 0.35), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="mid_crossbrace",
    )
    frame.visual(
        Box((0.20, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.10, 0.355)),
        material=painted_steel,
        name="front_tray_rest",
    )
    frame.visual(
        Box((0.024, 0.040, 0.120)),
        origin=Origin(xyz=(-0.103, 0.625, 0.235)),
        material=painted_steel,
        name="left_fork_cheek",
    )
    frame.visual(
        Box((0.024, 0.040, 0.120)),
        origin=Origin(xyz=(0.103, 0.625, 0.235)),
        material=painted_steel,
        name="right_fork_cheek",
    )
    frame.visual(
        Box((0.022, 0.024, 0.024)),
        origin=Origin(xyz=(-0.091, 0.64, 0.20)),
        material=dark_steel,
        name="left_axle_pin",
    )
    frame.visual(
        Box((0.022, 0.024, 0.024)),
        origin=Origin(xyz=(0.091, 0.64, 0.20)),
        material=dark_steel,
        name="right_axle_pin",
    )

    frame.visual(
        _mesh(
            "wheelbarrow_left_leg",
            tube_from_spline_points(
                [(-0.245, -0.05, 0.39), (-0.24, -0.09, 0.20), (-0.23, -0.12, 0.02)],
                radius=0.017,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=painted_steel,
        name="left_leg",
    )
    frame.visual(
        _mesh(
            "wheelbarrow_right_leg",
            tube_from_spline_points(
                [(0.245, -0.05, 0.39), (0.24, -0.09, 0.20), (0.23, -0.12, 0.02)],
                radius=0.017,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=painted_steel,
        name="right_leg",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.44),
        origin=Origin(xyz=(0.0, -0.12, 0.03), rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="leg_spreader",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.11),
        origin=Origin(xyz=(-0.29, -0.63, 0.55), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_grip,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.11),
        origin=Origin(xyz=(0.29, -0.63, 0.55), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_grip,
        name="right_handle_grip",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.56, 0.80, 0.28)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.01, 0.14)),
    )
    tray.visual(_mesh("wheelbarrow_tray_shell", _build_tray_shell()), material=dark_steel, name="tray_shell")
    tray.visual(
        Box((0.18, 0.06, 0.05)),
        origin=Origin(xyz=(0.0, 0.03, 0.01)),
        material=painted_steel,
        name="front_support_saddle",
    )
    tray.visual(
        Box((0.16, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.26, 0.04)),
        material=painted_steel,
        name="rear_support_saddle",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.10),
        mass=4.2,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    front_wheel.visual(_build_tire_mesh(0.20, 0.10), material=rubber, name="tire")
    front_wheel.visual(
        Cylinder(radius=0.13, length=0.082),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.035, length=0.16),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=painted_steel,
        name="hub",
    )

    model.articulation(
        "tray_mount",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.09, 0.38)),
    )

    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.64, 0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=30.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    front_wheel = object_model.get_part("front_wheel")
    front_wheel_spin = object_model.get_articulation("front_wheel_spin")
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_contact(
        tray,
        frame,
        elem_a="front_support_saddle",
        elem_b="front_tray_rest",
        name="tray rests on the front tray support",
    )
    ctx.expect_gap(
        front_wheel,
        frame,
        axis="x",
        positive_elem="tire",
        negative_elem="left_fork_cheek",
        min_gap=0.02,
        max_gap=0.06,
        name="left fork cheek clears the tire sidewall",
    )
    ctx.expect_gap(
        frame,
        front_wheel,
        axis="x",
        positive_elem="right_fork_cheek",
        negative_elem="tire",
        min_gap=0.02,
        max_gap=0.06,
        name="right fork cheek clears the tire sidewall",
    )

    rest_aabb = ctx.part_world_aabb(front_wheel)
    with ctx.pose({front_wheel_spin: pi / 2.0}):
        spun_aabb = ctx.part_world_aabb(front_wheel)
    if rest_aabb is None or spun_aabb is None:
        ctx.fail("front wheel pose review available", "front wheel AABB could not be measured")
    else:
        rest_dims = tuple(rest_aabb[1][axis] - rest_aabb[0][axis] for axis in range(3))
        spun_dims = tuple(spun_aabb[1][axis] - spun_aabb[0][axis] for axis in range(3))
        ctx.check(
            "front wheel spin preserves wheel orientation envelope",
            abs(rest_dims[0] - spun_dims[0]) < 1e-4 and abs(rest_dims[2] - spun_dims[2]) < 1e-4,
            details=f"rest_dims={rest_dims}, spun_dims={spun_dims}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
