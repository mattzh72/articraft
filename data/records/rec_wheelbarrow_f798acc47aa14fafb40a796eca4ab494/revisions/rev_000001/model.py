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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_frame_mesh() -> MeshGeometry:
    left_rail_points = [
        (-0.27, -0.70, 0.72),
        (-0.25, -0.52, 0.67),
        (-0.22, -0.28, 0.58),
        (-0.18, 0.00, 0.48),
        (-0.12, 0.30, 0.39),
        (-0.08, 0.50, 0.31),
        (-0.07, 0.61, 0.21),
    ]
    right_rail_points = _mirror_x(left_rail_points)

    left_leg_points = [
        (-0.22, -0.28, 0.58),
        (-0.21, -0.42, 0.30),
        (-0.22, -0.58, 0.04),
    ]
    right_leg_points = _mirror_x(left_leg_points)

    tubes = [
        tube_from_spline_points(
            left_rail_points,
            radius=0.018,
            samples_per_segment=16,
            radial_segments=16,
        ),
        tube_from_spline_points(
            right_rail_points,
            radius=0.018,
            samples_per_segment=16,
            radial_segments=16,
        ),
        tube_from_spline_points(
            left_leg_points,
            radius=0.016,
            samples_per_segment=10,
            radial_segments=16,
        ),
        tube_from_spline_points(
            right_leg_points,
            radius=0.016,
            samples_per_segment=10,
            radial_segments=16,
        ),
        tube_from_spline_points(
            [(-0.18, 0.00, 0.48), (0.18, 0.00, 0.48)],
            radius=0.014,
            samples_per_segment=2,
            radial_segments=16,
        ),
        tube_from_spline_points(
            [(-0.12, 0.30, 0.39), (0.12, 0.30, 0.39)],
            radius=0.014,
            samples_per_segment=2,
            radial_segments=16,
        ),
        tube_from_spline_points(
            [(-0.10, 0.40, 0.35), (0.10, 0.40, 0.35)],
            radius=0.014,
            samples_per_segment=2,
            radial_segments=16,
        ),
        tube_from_spline_points(
            [(-0.215, -0.46, 0.235), (0.215, -0.46, 0.235)],
            radius=0.013,
            samples_per_segment=2,
            radial_segments=16,
        ),
    ]
    return _merge_geometries(tubes)


def _build_wheel_tire_mesh() -> MeshGeometry:
    half_width = 0.045
    profile = [
        (0.105, -half_width),
        (0.150, -half_width * 0.98),
        (0.177, -half_width * 0.74),
        (0.190, -half_width * 0.28),
        (0.190, half_width * 0.28),
        (0.177, half_width * 0.74),
        (0.150, half_width * 0.98),
        (0.105, half_width),
        (0.090, half_width * 0.55),
        (0.084, 0.0),
        (0.090, -half_width * 0.55),
        (0.105, -half_width),
    ]
    return LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    frame_paint = model.material("frame_paint", rgba=(0.22, 0.25, 0.20, 1.0))
    tray_paint = model.material("tray_paint", rgba=(0.82, 0.16, 0.11, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.11, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        _mesh("wheelbarrow_frame", _build_frame_mesh()),
        material=frame_paint,
        name="frame_tubes",
    )
    frame.visual(
        Cylinder(radius=0.023, length=0.14),
        origin=Origin(xyz=(-0.27, -0.74, 0.73), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.023, length=0.14),
        origin=Origin(xyz=(0.27, -0.74, 0.73), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
        name="right_handle_grip",
    )
    frame.visual(
        Box((0.06, 0.04, 0.034)),
        origin=Origin(xyz=(-0.22, -0.59, 0.017)),
        material=galvanized,
        name="left_foot_pad",
    )
    frame.visual(
        Box((0.06, 0.04, 0.034)),
        origin=Origin(xyz=(0.22, -0.59, 0.017)),
        material=galvanized,
        name="right_foot_pad",
    )
    frame.visual(
        Box((0.52, 0.76, 0.005)),
        origin=Origin(xyz=(0.0, -0.05, 0.515), rpy=(-0.19, 0.0, 0.0)),
        material=tray_paint,
        name="tray_bottom",
    )
    frame.visual(
        Box((0.025, 0.72, 0.16)),
        origin=Origin(xyz=(-0.248, -0.05, 0.595), rpy=(-0.19, 0.0, 0.0)),
        material=tray_paint,
        name="left_tray_side",
    )
    frame.visual(
        Box((0.025, 0.72, 0.16)),
        origin=Origin(xyz=(0.248, -0.05, 0.595), rpy=(-0.19, 0.0, 0.0)),
        material=tray_paint,
        name="right_tray_side",
    )
    frame.visual(
        Box((0.50, 0.025, 0.18)),
        origin=Origin(xyz=(0.0, 0.315, 0.535), rpy=(-0.19, 0.0, 0.0)),
        material=tray_paint,
        name="front_tray_wall",
    )
    frame.visual(
        Box((0.34, 0.020, 0.09)),
        origin=Origin(xyz=(0.0, -0.405, 0.595), rpy=(-0.19, 0.0, 0.0)),
        material=tray_paint,
        name="rear_tray_wall",
    )
    frame.visual(
        Box((0.44, 0.032, 0.028)),
        origin=Origin(xyz=(0.0, -0.52, 0.165)),
        material=galvanized,
        name="base_brace",
    )
    frame.visual(
        Box((0.028, 0.034, 0.060)),
        origin=Origin(xyz=(-0.0565, 0.60, 0.19)),
        material=galvanized,
        name="left_fork_dropout",
    )
    frame.visual(
        Box((0.028, 0.034, 0.060)),
        origin=Origin(xyz=(0.0565, 0.60, 0.19)),
        material=galvanized,
        name="right_fork_dropout",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.64, 1.44, 0.76)),
        mass=14.0,
        origin=Origin(xyz=(0.0, -0.06, 0.38)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        _mesh("wheelbarrow_front_tire", _build_wheel_tire_mesh()),
        material=tire_rubber,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(-0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="left_rim_face",
    )
    front_wheel.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="right_rim_face",
    )
    front_wheel.visual(
        Cylinder(radius=0.070, length=0.085),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_paint,
        name="hub_shell",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.10),
        mass=4.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.60, 0.19)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=24.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_wheel = object_model.get_part("front_wheel")

    ctx.expect_contact(
        frame,
        front_wheel,
        elem_a="left_fork_dropout",
        elem_b="hub_shell",
        name="left fork dropout supports the hub",
    )
    ctx.expect_contact(
        frame,
        front_wheel,
        elem_a="right_fork_dropout",
        elem_b="hub_shell",
        name="right fork dropout supports the hub",
    )
    ctx.expect_gap(
        frame,
        front_wheel,
        axis="z",
        positive_elem="tray_bottom",
        negative_elem="tire",
        min_gap=0.045,
        name="tray clears the front tire",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
