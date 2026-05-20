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
    model = ArticulatedObject(name="mass_manufactured_wheelbarrow")

    tray_paint = model.material("tray_paint", rgba=(0.34, 0.44, 0.22, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.13, 0.13, 0.14, 1.0))
    zinc = model.material("zinc", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip = model.material("grip", rgba=(0.16, 0.17, 0.17, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.55, 0.62, 0.62)),
        mass=12.0,
        origin=Origin(xyz=(0.05, 0.0, 0.31)),
    )

    left_handle_path = [
        (-0.56, 0.33, 0.60),
        (-0.38, 0.31, 0.53),
        (-0.16, 0.28, 0.43),
        (0.12, 0.22, 0.37),
        (0.44, 0.11, 0.29),
        (0.70, 0.085, 0.19),
    ]
    frame.visual(
        _save_mesh(
            "wheelbarrow_left_handle_rail",
            tube_from_spline_points(
                left_handle_path,
                radius=0.018,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="left_handle_rail",
    )
    frame.visual(
        _save_mesh(
            "wheelbarrow_right_handle_rail",
            tube_from_spline_points(
                _mirror_y(left_handle_path),
                radius=0.018,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="right_handle_rail",
    )

    left_leg_path = [
        (-0.23, 0.28, 0.398),
        (-0.31, 0.22, 0.22),
        (-0.39, 0.19, 0.02),
    ]
    frame.visual(
        _save_mesh(
            "wheelbarrow_left_rear_leg",
            tube_from_spline_points(
                left_leg_path,
                radius=0.017,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_paint,
        name="left_rear_leg",
    )
    frame.visual(
        _save_mesh(
            "wheelbarrow_right_rear_leg",
            tube_from_spline_points(
                _mirror_y(left_leg_path),
                radius=0.017,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_paint,
        name="right_rear_leg",
    )

    frame.visual(
        Box((0.10, 0.42, 0.02)),
        origin=Origin(xyz=(-0.39, 0.0, 0.01)),
        material=frame_paint,
        name="rear_foot_crossbar",
    )
    frame.visual(
        Box((0.24, 0.60, 0.03)),
        origin=Origin(xyz=(-0.19, 0.0, 0.398)),
        material=frame_paint,
        name="rear_tray_crossmember",
    )
    frame.visual(
        Box((0.18, 0.18, 0.020)),
        origin=Origin(xyz=(0.61, 0.0, 0.414)),
        material=frame_paint,
        name="fork_crown",
    )
    frame.visual(
        Box((0.09, 0.05, 0.05)),
        origin=Origin(xyz=(-0.08, 0.19, 0.4245)),
        material=zinc,
        name="left_rear_bracket",
    )
    frame.visual(
        Box((0.09, 0.05, 0.05)),
        origin=Origin(xyz=(-0.08, -0.19, 0.4245)),
        material=zinc,
        name="right_rear_bracket",
    )
    frame.visual(
        Box((0.09, 0.03, 0.089)),
        origin=Origin(xyz=(0.18, 0.17, 0.405)),
        material=zinc,
        name="left_front_bracket",
    )
    frame.visual(
        Box((0.09, 0.03, 0.089)),
        origin=Origin(xyz=(0.18, -0.17, 0.405)),
        material=zinc,
        name="right_front_bracket",
    )
    frame.visual(
        Box((0.05, 0.012, 0.22)),
        origin=Origin(xyz=(0.70, 0.067, 0.294)),
        material=frame_paint,
        name="left_fork_plate",
    )
    frame.visual(
        Box((0.05, 0.012, 0.22)),
        origin=Origin(xyz=(0.70, -0.067, 0.294)),
        material=frame_paint,
        name="right_fork_plate",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.70, 0.087, 0.21), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="left_axle_cap",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.70, -0.087, 0.21), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="right_axle_cap",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.13),
        origin=Origin(xyz=(-0.60, 0.33, 0.60), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.13),
        origin=Origin(xyz=(-0.60, -0.33, 0.60), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="right_grip",
    )

    tray = model.part("tray")
    tray.inertial = Inertial.from_geometry(
        Box((0.94, 0.66, 0.26)),
        mass=7.5,
        origin=Origin(xyz=(0.03, 0.0, 0.56)),
    )
    tray.visual(
        Box((0.82, 0.48, 0.008)),
        origin=Origin(xyz=(0.02, 0.0, 0.454)),
        material=tray_paint,
        name="floor_panel",
    )
    tray.visual(
        Box((0.78, 0.018, 0.18)),
        origin=Origin(xyz=(0.02, 0.249, 0.548)),
        material=tray_paint,
        name="left_side_wall",
    )
    tray.visual(
        Box((0.78, 0.018, 0.18)),
        origin=Origin(xyz=(0.02, -0.249, 0.548)),
        material=tray_paint,
        name="right_side_wall",
    )
    tray.visual(
        Box((0.018, 0.56, 0.18)),
        origin=Origin(xyz=(0.421, 0.0, 0.548)),
        material=tray_paint,
        name="front_wall",
    )
    tray.visual(
        Box((0.018, 0.46, 0.16)),
        origin=Origin(xyz=(-0.385, 0.0, 0.534)),
        material=tray_paint,
        name="rear_wall",
    )
    tray.visual(
        Box((0.82, 0.030, 0.018)),
        origin=Origin(xyz=(0.02, 0.262, 0.647)),
        material=tray_paint,
        name="left_rim_rail",
    )
    tray.visual(
        Box((0.82, 0.030, 0.018)),
        origin=Origin(xyz=(0.02, -0.262, 0.647)),
        material=tray_paint,
        name="right_rim_rail",
    )
    tray.visual(
        Box((0.024, 0.56, 0.018)),
        origin=Origin(xyz=(0.421, 0.0, 0.647)),
        material=tray_paint,
        name="front_rim_rail",
    )
    tray.visual(
        Box((0.09, 0.05, 0.012)),
        origin=Origin(xyz=(-0.08, 0.19, 0.4555)),
        material=zinc,
        name="left_rear_mount_pad",
    )
    tray.visual(
        Box((0.09, 0.05, 0.012)),
        origin=Origin(xyz=(-0.08, -0.19, 0.4555)),
        material=zinc,
        name="right_rear_mount_pad",
    )
    tray.visual(
        Box((0.10, 0.05, 0.012)),
        origin=Origin(xyz=(0.18, 0.17, 0.4555)),
        material=zinc,
        name="left_front_mount_pad",
    )
    tray.visual(
        Box((0.10, 0.05, 0.012)),
        origin=Origin(xyz=(0.18, -0.17, 0.4555)),
        material=zinc,
        name="right_front_mount_pad",
    )
    tray.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(-0.08, 0.19, 0.4635)),
        material=zinc,
        name="left_rear_bolt_head",
    )
    tray.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(-0.08, -0.19, 0.4635)),
        material=zinc,
        name="right_rear_bolt_head",
    )
    tray.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.18, 0.17, 0.4635)),
        material=zinc,
        name="left_front_bolt_head",
    )
    tray.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.18, -0.17, 0.4635)),
        material=zinc,
        name="right_front_bolt_head",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.10),
        mass=3.4,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )
    tire_profile = [
        (0.096, -0.038),
        (0.143, -0.047),
        (0.177, -0.042),
        (0.190, -0.020),
        (0.190, 0.020),
        (0.177, 0.042),
        (0.143, 0.047),
        (0.096, 0.038),
        (0.078, 0.015),
        (0.076, 0.0),
        (0.078, -0.015),
        (0.096, -0.038),
    ]
    wheel.visual(
        _save_mesh(
            "wheelbarrow_tire",
            LatheGeometry(tire_profile, segments=64).rotate_x(-pi / 2.0),
        ),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.132, length=0.078),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="rim_barrel",
    )
    wheel.visual(
        Cylinder(radius=0.148, length=0.008),
        origin=Origin(xyz=(0.0, 0.027, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="left_rim_face",
    )
    wheel.visual(
        Cylinder(radius=0.148, length=0.008),
        origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="right_rim_face",
    )
    wheel.visual(
        Cylinder(radius=0.043, length=0.092),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="hub_shell",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.03),
        origin=Origin(xyz=(0.0, 0.046, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="left_bearing_stub",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.03),
        origin=Origin(xyz=(0.0, -0.046, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="right_bearing_stub",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.FIXED,
        parent=frame,
        child=tray,
        origin=Origin(),
    )
    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.70, 0.0, 0.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tray = object_model.get_part("tray")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("wheel_spin")

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
        "required parts present",
        all(part is not None for part in (frame, tray, wheel)),
        details="frame, tray, and wheel should all exist",
    )

    ctx.expect_contact(
        tray,
        frame,
        elem_a="left_front_mount_pad",
        elem_b="left_front_bracket",
        name="left_front_mount_contacts_bracket",
    )
    ctx.expect_contact(
        tray,
        frame,
        elem_a="right_front_mount_pad",
        elem_b="right_front_bracket",
        name="right_front_mount_contacts_bracket",
    )
    ctx.expect_contact(
        tray,
        frame,
        elem_a="left_rear_mount_pad",
        elem_b="left_rear_bracket",
        name="left_rear_mount_contacts_bracket",
    )
    ctx.expect_contact(
        tray,
        frame,
        elem_a="right_rear_mount_pad",
        elem_b="right_rear_bracket",
        name="right_rear_mount_contacts_bracket",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="y",
        positive_elem="left_fork_plate",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.028,
        name="left_fork_plate_clears_tire",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="right_fork_plate",
        min_gap=0.006,
        max_gap=0.028,
        name="right_fork_plate_clears_tire",
    )
    ctx.expect_gap(
        tray,
        wheel,
        axis="z",
        min_gap=0.05,
        name="tray_clears_top_of_wheel",
    )
    ctx.expect_origin_gap(
        wheel,
        frame,
        axis="x",
        min_gap=0.60,
        max_gap=0.78,
        name="wheel_is_forward_of_frame",
    )

    with ctx.pose({wheel_spin: pi / 2.0}):
        ctx.expect_gap(
            frame,
            wheel,
            axis="y",
            positive_elem="left_fork_plate",
            negative_elem="tire",
            min_gap=0.006,
            max_gap=0.028,
            name="left_fork_clearance_persists_when_wheel_spins",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
