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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    frame_paint = model.material("frame_paint", rgba=(0.18, 0.21, 0.18, 1.0))
    tray_paint = model.material("tray_paint", rgba=(0.41, 0.51, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    grip = model.material("grip", rgba=(0.46, 0.33, 0.20, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.60, 0.64, 0.64)),
        mass=16.0,
        origin=Origin(xyz=(-0.08, 0.0, 0.32)),
    )

    left_handle_points = [
        (-0.86, 0.27, 0.60),
        (-0.70, 0.26, 0.54),
        (-0.46, 0.24, 0.45),
        (-0.12, 0.22, 0.37),
        (0.20, 0.18, 0.33),
        (0.46, 0.09, 0.28),
    ]
    for mesh_name, points in (
        ("left_handle_rail", left_handle_points),
        ("right_handle_rail", _mirror_y(left_handle_points)),
    ):
        frame.visual(
            _save_mesh(
                mesh_name,
                tube_from_spline_points(
                    points,
                    radius=0.016,
                    samples_per_segment=14,
                    radial_segments=18,
                ),
            ),
            material=frame_paint,
            name=mesh_name,
        )

    left_fork_points = [
        (0.46, 0.09, 0.28),
        (0.56, 0.08, 0.25),
        (0.64, 0.07, 0.21),
        (0.68, 0.07, 0.20),
    ]
    for mesh_name, points in (
        ("left_fork_blade", left_fork_points),
        ("right_fork_blade", _mirror_y(left_fork_points)),
    ):
        frame.visual(
            _save_mesh(
                mesh_name,
                tube_from_spline_points(
                    points,
                    radius=0.015,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=mesh_name,
        )

    left_leg_points = [
        (-0.46, 0.24, 0.45),
        (-0.49, 0.19, 0.27),
        (-0.49, 0.16, 0.18),
        (-0.53, 0.16, 0.014),
    ]
    for mesh_name, points in (
        ("left_rear_leg", left_leg_points),
        ("right_rear_leg", _mirror_y(left_leg_points)),
    ):
        frame.visual(
            _save_mesh(
                mesh_name,
                tube_from_spline_points(
                    points,
                    radius=0.014,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=mesh_name,
        )

    brace_specs = [
        (
            "mid_support_crossbar",
            [(-0.28, 0.23, 0.39), (-0.28, -0.23, 0.39)],
            0.013,
        ),
        (
            "rear_leg_crossbar",
            [(-0.49, 0.16, 0.18), (-0.49, -0.16, 0.18)],
            0.012,
        ),
        (
            "fork_crown",
            [(0.46, 0.09, 0.28), (0.46, -0.09, 0.28)],
            0.012,
        ),
        (
            "nose_brace",
            [(0.12, 0.0, 0.31), (0.46, 0.0, 0.28)],
            0.012,
        ),
    ]
    for mesh_name, points, radius in brace_specs:
        frame.visual(
            _save_mesh(
                mesh_name,
                tube_from_spline_points(
                    points,
                    radius=radius,
                    samples_per_segment=4,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=mesh_name,
        )

    left_tray_support_points = [
        (-0.46, 0.24, 0.45),
        (-0.38, 0.235, 0.38),
        (-0.30, 0.225, 0.34),
        (-0.22, 0.158, 0.332),
    ]
    for mesh_name, points in (
        ("left_tray_support", left_tray_support_points),
        ("right_tray_support", _mirror_y(left_tray_support_points)),
    ):
        frame.visual(
            _save_mesh(
                mesh_name,
                tube_from_spline_points(
                    points,
                    radius=0.010,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint,
            name=mesh_name,
        )

    frame.visual(
        Box((0.18, 0.030, 0.012)),
        origin=Origin(xyz=(-0.16, 0.151, 0.337)),
        material=frame_paint,
        name="left_tray_pad",
    )
    frame.visual(
        Box((0.18, 0.030, 0.012)),
        origin=Origin(xyz=(-0.16, -0.151, 0.337)),
        material=frame_paint,
        name="right_tray_pad",
    )

    frame.visual(
        Box((0.12, 0.028, 0.022)),
        origin=Origin(xyz=(-0.89, 0.27, 0.60)),
        material=grip,
        name="left_grip",
    )
    frame.visual(
        Box((0.12, 0.028, 0.022)),
        origin=Origin(xyz=(-0.89, -0.27, 0.60)),
        material=grip,
        name="right_grip",
    )
    wheel = model.part("front_wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.08),
        mass=3.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    wheel.visual(
        Cylinder(radius=0.20, length=0.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.125, length=0.055),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.030, length=0.11),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hub",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.62, 0.50, 0.18)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )
    tray.visual(
        Box((0.52, 0.276, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=tray_paint,
        name="bottom_panel",
    )
    tray.visual(
        Box((0.56, 0.010, 0.134)),
        origin=Origin(xyz=(0.01, 0.141, 0.070)),
        material=tray_paint,
        name="left_wall",
    )
    tray.visual(
        Box((0.56, 0.010, 0.134)),
        origin=Origin(xyz=(0.01, -0.141, 0.070)),
        material=tray_paint,
        name="right_wall",
    )
    tray.visual(
        Box((0.010, 0.34, 0.136)),
        origin=Origin(xyz=(0.262, 0.0, 0.071)),
        material=tray_paint,
        name="front_wall",
    )
    tray.visual(
        Box((0.010, 0.42, 0.100)),
        origin=Origin(xyz=(-0.262, 0.0, 0.053)),
        material=tray_paint,
        name="rear_wall",
    )

    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.68, 0.0, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=24.0),
    )
    model.articulation(
        "frame_to_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0177, 0.0, 0.343)),
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
