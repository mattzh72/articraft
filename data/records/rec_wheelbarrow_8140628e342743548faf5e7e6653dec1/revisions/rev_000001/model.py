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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_wheelbarrow")

    tray_paint = model.material("tray_paint", rgba=(0.70, 0.73, 0.72, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.21, 0.22, 0.24, 1.0))
    rim_polymer = model.material("rim_polymer", rgba=(0.60, 0.62, 0.66, 1.0))
    hub_dark = model.material("hub_dark", rgba=(0.30, 0.31, 0.33, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((1.60, 0.72, 0.66)),
        mass=16.0,
        origin=Origin(xyz=(-0.14, 0.0, 0.34)),
    )

    tray_floor_profile = sample_catmull_rom_spline_2d(
        [
            (-0.44, -0.23),
            (-0.18, -0.235),
            (0.10, -0.205),
            (0.24, -0.145),
            (0.30, 0.0),
            (0.24, 0.145),
            (0.10, 0.205),
            (-0.18, 0.235),
            (-0.44, 0.23),
        ],
        samples_per_segment=10,
        closed=True,
    )
    tray_floor = ExtrudeGeometry(tray_floor_profile, 0.012, center=True).translate(0.0, 0.0, 0.285)
    chassis.visual(_save_mesh("tray_floor", tray_floor), material=tray_paint, name="tray_floor")

    tray_side_profile = sample_catmull_rom_spline_2d(
        [
            (-0.44, 0.285),
            (-0.10, 0.282),
            (0.16, 0.280),
            (0.28, 0.285),
            (0.30, 0.345),
            (0.24, 0.374),
            (0.04, 0.408),
            (-0.26, 0.430),
            (-0.41, 0.392),
        ],
        samples_per_segment=8,
        closed=True,
    )
    left_wall = ExtrudeGeometry(tray_side_profile, 0.012, center=True).rotate_x(pi / 2.0).translate(0.0, 0.226, 0.0)
    right_wall = ExtrudeGeometry(tray_side_profile, 0.012, center=True).rotate_x(pi / 2.0).translate(0.0, -0.226, 0.0)
    chassis.visual(_save_mesh("tray_left_wall", left_wall), material=tray_paint, name="tray_left_wall")
    chassis.visual(_save_mesh("tray_right_wall", right_wall), material=tray_paint, name="tray_right_wall")

    chassis.visual(
        Box((0.018, 0.17, 0.10)),
        origin=Origin(xyz=(0.295, 0.0, 0.334), rpy=(0.0, 0.14, 0.0)),
        material=tray_paint,
        name="tray_front_wall",
    )
    chassis.visual(
        Box((0.018, 0.46, 0.09)),
        origin=Origin(xyz=(-0.406, 0.0, 0.329)),
        material=tray_paint,
        name="tray_rear_wall",
    )

    left_lip_points = [
        (-0.40, 0.233, 0.390),
        (-0.18, 0.228, 0.425),
        (0.08, 0.212, 0.408),
        (0.27, 0.150, 0.366),
    ]
    chassis.visual(
        _save_mesh(
            "left_rim_lip",
            tube_from_spline_points(left_lip_points, radius=0.009, samples_per_segment=18, radial_segments=14),
        ),
        material=tray_paint,
        name="left_rim_lip",
    )
    chassis.visual(
        _save_mesh(
            "right_rim_lip",
            tube_from_spline_points(_mirror_y(left_lip_points), radius=0.009, samples_per_segment=18, radial_segments=14),
        ),
        material=tray_paint,
        name="right_rim_lip",
    )
    chassis.visual(
        _save_mesh(
            "front_rim_lip",
            tube_from_spline_points(
                [(0.27, 0.150, 0.366), (0.31, 0.0, 0.357), (0.27, -0.150, 0.366)],
                radius=0.009,
                samples_per_segment=18,
                radial_segments=14,
            ),
        ),
        material=tray_paint,
        name="front_rim_lip",
    )
    chassis.visual(
        Cylinder(radius=0.009, length=0.44),
        origin=Origin(xyz=(-0.405, 0.0, 0.390), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tray_paint,
        name="rear_rim_lip",
    )

    left_rail_points = [
        (-0.98, 0.27, 0.60),
        (-0.78, 0.28, 0.56),
        (-0.58, 0.26, 0.43),
        (-0.54, 0.24, 0.37),
        (-0.30, 0.22, 0.33),
        (0.00, 0.19, 0.30),
        (0.24, 0.16, 0.28),
        (0.40, 0.13, 0.25),
        (0.53, 0.10, 0.19),
    ]
    chassis.visual(
        _save_mesh(
            "left_frame_rail",
            tube_from_spline_points(left_rail_points, radius=0.017, samples_per_segment=16, radial_segments=16),
        ),
        material=frame_paint,
        name="left_frame_rail",
    )
    chassis.visual(
        _save_mesh(
            "right_frame_rail",
            tube_from_spline_points(_mirror_y(left_rail_points), radius=0.017, samples_per_segment=16, radial_segments=16),
        ),
        material=frame_paint,
        name="right_frame_rail",
    )

    left_leg_points = [(-0.54, 0.24, 0.37), (-0.58, 0.26, 0.15), (-0.60, 0.28, 0.05)]
    chassis.visual(
        _save_mesh(
            "left_rear_leg",
            tube_from_spline_points(left_leg_points, radius=0.015, samples_per_segment=12, radial_segments=14),
        ),
        material=frame_paint,
        name="left_rear_leg",
    )
    chassis.visual(
        _save_mesh(
            "right_rear_leg",
            tube_from_spline_points(_mirror_y(left_leg_points), radius=0.015, samples_per_segment=12, radial_segments=14),
        ),
        material=frame_paint,
        name="right_rear_leg",
    )

    chassis.visual(
        Box((0.56, 0.05, 0.038)),
        origin=Origin(xyz=(-0.06, 0.18, 0.298)),
        material=frame_paint,
        name="left_support_runner",
    )
    chassis.visual(
        Box((0.56, 0.05, 0.038)),
        origin=Origin(xyz=(-0.06, -0.18, 0.298)),
        material=frame_paint,
        name="right_support_runner",
    )
    chassis.visual(
        Cylinder(radius=0.012, length=0.44),
        origin=Origin(xyz=(-0.24, 0.0, 0.314), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="rear_undertray_crossbrace",
    )
    chassis.visual(
        Cylinder(radius=0.012, length=0.35),
        origin=Origin(xyz=(0.12, 0.0, 0.292), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="front_undertray_crossbrace",
    )
    chassis.visual(
        Box((0.08, 0.28, 0.04)),
        origin=Origin(xyz=(0.285, 0.0, 0.277)),
        material=frame_paint,
        name="fork_bridge",
    )

    left_fork_arm_points = [
        (0.39, 0.13, 0.25),
        (0.45, 0.10, 0.23),
        (0.49, 0.075, 0.21),
        (0.52, 0.047, 0.185),
    ]
    chassis.visual(
        _save_mesh(
            "left_fork_arm",
            tube_from_spline_points(left_fork_arm_points, radius=0.015, samples_per_segment=14, radial_segments=14),
        ),
        material=frame_paint,
        name="left_fork_arm",
    )
    chassis.visual(
        _save_mesh(
            "right_fork_arm",
            tube_from_spline_points(_mirror_y(left_fork_arm_points), radius=0.015, samples_per_segment=14, radial_segments=14),
        ),
        material=frame_paint,
        name="right_fork_arm",
    )
    chassis.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(0.52, 0.047, 0.185), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="left_axle_collar",
    )
    chassis.visual(
        Cylinder(radius=0.017, length=0.022),
        origin=Origin(xyz=(0.52, -0.047, 0.185), rpy=(pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="right_axle_collar",
    )

    chassis.visual(
        Cylinder(radius=0.012, length=0.58),
        origin=Origin(xyz=(-0.60, 0.0, 0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="rear_foot_spreader",
    )
    chassis.visual(
        Box((0.08, 0.038, 0.048)),
        origin=Origin(xyz=(-0.60, 0.28, 0.024)),
        material=rubber,
        name="left_foot_pad",
    )
    chassis.visual(
        Box((0.08, 0.038, 0.048)),
        origin=Origin(xyz=(-0.60, -0.28, 0.024)),
        material=rubber,
        name="right_foot_pad",
    )

    chassis.visual(
        Cylinder(radius=0.022, length=0.16),
        origin=Origin(xyz=(-0.90, 0.27, 0.59), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    chassis.visual(
        Cylinder(radius=0.022, length=0.16),
        origin=Origin(xyz=(-0.90, -0.27, 0.59), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.185, length=0.082),
        mass=3.4,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    front_wheel.visual(
        _save_mesh("wheel_tire", TorusGeometry(0.147, 0.038, radial_segments=20, tubular_segments=40).rotate_x(pi / 2.0)),
        material=rubber,
        name="wheel_tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.122, length=0.046),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_polymer,
        name="wheel_rim_barrel",
    )
    front_wheel.visual(
        Cylinder(radius=0.138, length=0.008),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_polymer,
        name="left_rim_face",
    )
    front_wheel.visual(
        Cylinder(radius=0.138, length=0.008),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rim_polymer,
        name="right_rim_face",
    )
    front_wheel.visual(
        Cylinder(radius=0.051, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="wheel_hub",
    )
    front_wheel.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="left_hub_cap",
    )
    front_wheel.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_dark,
        name="right_hub_cap",
    )

    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_wheel,
        origin=Origin(xyz=(0.52, 0.0, 0.185)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=16.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    front_wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("front_wheel_spin")
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
        "wheel_spin_axis_is_lateral",
        abs(wheel_spin.axis[1]) > 0.99 and abs(wheel_spin.axis[0]) < 1e-6 and abs(wheel_spin.axis[2]) < 1e-6,
        details=f"wheel spin axis should run laterally through the axle, got {wheel_spin.axis}",
    )
    ctx.expect_contact(
        chassis,
        front_wheel,
        elem_a="left_axle_collar",
        elem_b="left_hub_cap",
        name="left_axle_support_contact",
    )
    ctx.expect_contact(
        chassis,
        front_wheel,
        elem_a="right_axle_collar",
        elem_b="right_hub_cap",
        name="right_axle_support_contact",
    )
    ctx.expect_gap(
        front_wheel,
        chassis,
        axis="x",
        positive_elem="wheel_tire",
        negative_elem="tray_floor",
        min_gap=0.025,
        max_gap=0.055,
        name="front_wheel_clears_tray_nose",
    )
    ctx.expect_origin_distance(
        front_wheel,
        chassis,
        axes="x",
        min_dist=0.45,
        max_dist=0.70,
        name="wheel_sits_forward_of_load_bay",
    )

    tire_aabb = ctx.part_element_world_aabb(front_wheel, elem="wheel_tire")
    left_foot_aabb = ctx.part_element_world_aabb(chassis, elem="left_foot_pad")
    right_foot_aabb = ctx.part_element_world_aabb(chassis, elem="right_foot_pad")
    if tire_aabb is None or left_foot_aabb is None or right_foot_aabb is None:
        ctx.fail("stance_tripod_grounding", "missing tire or rear foot pad geometry for stance check")
    else:
        wheel_bottom = tire_aabb[0][2]
        left_bottom = left_foot_aabb[0][2]
        right_bottom = right_foot_aabb[0][2]
        feet_mismatch = abs(left_bottom - right_bottom)
        support_plane_error = abs(((left_bottom + right_bottom) * 0.5) - wheel_bottom)
        ctx.check(
            "stance_tripod_grounding",
            feet_mismatch <= 0.005 and support_plane_error <= 0.012,
            details=(
                f"wheel bottom={wheel_bottom:.4f}, left foot={left_bottom:.4f}, "
                f"right foot={right_bottom:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
