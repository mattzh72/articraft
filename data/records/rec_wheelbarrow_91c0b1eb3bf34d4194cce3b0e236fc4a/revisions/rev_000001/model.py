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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    tray_green = model.material("tray_green", rgba=(0.24, 0.42, 0.22, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.23, 0.25, 0.26, 1.0))
    galvanized = model.material("galvanized", rgba=(0.73, 0.75, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    grip_black = model.material("grip_black", rgba=(0.11, 0.11, 0.11, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.68, 1.46, 0.62)),
        mass=17.5,
        origin=Origin(xyz=(0.0, -0.02, 0.31)),
    )

    # Open steel tray.
    frame.visual(
        Box((0.38, 0.60, 0.018)),
        origin=Origin(xyz=(0.0, 0.03, 0.44)),
        material=tray_green,
        name="tray_bottom",
    )
    frame.visual(
        Box((0.020, 0.60, 0.17)),
        origin=Origin(xyz=(0.204, 0.03, 0.522), rpy=(0.0, 0.22, 0.0)),
        material=tray_green,
        name="left_tray_wall",
    )
    frame.visual(
        Box((0.020, 0.60, 0.17)),
        origin=Origin(xyz=(-0.204, 0.03, 0.522), rpy=(0.0, -0.22, 0.0)),
        material=tray_green,
        name="right_tray_wall",
    )
    frame.visual(
        Box((0.34, 0.020, 0.18)),
        origin=Origin(xyz=(0.0, 0.337, 0.525), rpy=(-0.42, 0.0, 0.0)),
        material=tray_green,
        name="front_tray_wall",
    )
    frame.visual(
        Box((0.24, 0.020, 0.095)),
        origin=Origin(xyz=(0.0, -0.272, 0.478), rpy=(0.36, 0.0, 0.0)),
        material=tray_green,
        name="rear_tray_wall",
    )

    # Under-tray rails that mount the tray to the handle tubes.
    frame.visual(
        Box((0.070, 0.54, 0.032)),
        origin=Origin(xyz=(0.198, 0.03, 0.405)),
        material=frame_steel,
        name="left_support_rail",
    )
    frame.visual(
        Box((0.070, 0.54, 0.032)),
        origin=Origin(xyz=(-0.198, 0.03, 0.405)),
        material=frame_steel,
        name="right_support_rail",
    )

    left_handle_geom = tube_from_spline_points(
        [
            (0.245, -0.695, 0.575),
            (0.232, -0.390, 0.490),
            (0.210, -0.050, 0.435),
            (0.166, 0.235, 0.360),
            (0.110, 0.430, 0.275),
            (0.076, 0.475, 0.238),
        ],
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_handle_geom = tube_from_spline_points(
        [
            (-0.245, -0.695, 0.575),
            (-0.232, -0.390, 0.490),
            (-0.210, -0.050, 0.435),
            (-0.166, 0.235, 0.360),
            (-0.110, 0.430, 0.275),
            (-0.076, 0.475, 0.238),
        ],
        radius=0.018,
        samples_per_segment=16,
        radial_segments=18,
    )
    left_leg_geom = tube_from_spline_points(
        [
            (0.216, -0.305, 0.446),
            (0.205, -0.420, 0.300),
            (0.185, -0.515, 0.048),
        ],
        radius=0.015,
        samples_per_segment=10,
        radial_segments=16,
    )
    right_leg_geom = tube_from_spline_points(
        [
            (-0.216, -0.305, 0.446),
            (-0.205, -0.420, 0.300),
            (-0.185, -0.515, 0.048),
        ],
        radius=0.015,
        samples_per_segment=10,
        radial_segments=16,
    )

    frame.visual(
        mesh_from_geometry(left_handle_geom, "left_handle"),
        material=frame_steel,
        name="left_handle",
    )
    frame.visual(
        mesh_from_geometry(right_handle_geom, "right_handle"),
        material=frame_steel,
        name="right_handle",
    )
    frame.visual(
        mesh_from_geometry(left_leg_geom, "left_leg"),
        material=frame_steel,
        name="left_leg",
    )
    frame.visual(
        mesh_from_geometry(right_leg_geom, "right_leg"),
        material=frame_steel,
        name="right_leg",
    )

    frame.visual(
        Box((0.46, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.315, 0.452)),
        material=frame_steel,
        name="rear_cross_brace",
    )
    frame.visual(
        Box((0.32, 0.060, 0.035)),
        origin=Origin(xyz=(0.0, 0.215, 0.350)),
        material=frame_steel,
        name="front_cross_brace",
    )
    frame.visual(
        Box((0.060, 0.035, 0.014)),
        origin=Origin(xyz=(0.185, -0.520, 0.041)),
        material=frame_steel,
        name="left_foot",
    )
    frame.visual(
        Box((0.060, 0.035, 0.014)),
        origin=Origin(xyz=(-0.185, -0.520, 0.041)),
        material=frame_steel,
        name="right_foot",
    )
    frame.visual(
        Box((0.31, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, -0.520, 0.056)),
        material=frame_steel,
        name="rear_foot_tie",
    )
    frame.visual(
        Box((0.036, 0.065, 0.065)),
        origin=Origin(xyz=(0.097, 0.475, 0.225)),
        material=frame_steel,
        name="left_drop_plate",
    )
    frame.visual(
        Box((0.036, 0.065, 0.065)),
        origin=Origin(xyz=(-0.097, 0.475, 0.225)),
        material=frame_steel,
        name="right_drop_plate",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.067, 0.475, 0.225), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_steel,
        name="left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.067, 0.475, 0.225), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_steel,
        name="right_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(0.245, -0.698, 0.575), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(-0.245, -0.698, 0.575), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.09),
        mass=2.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    tire_profile = [
        (0.088, -0.043),
        (0.126, -0.045),
        (0.164, -0.040),
        (0.184, -0.025),
        (0.190, -0.008),
        (0.190, 0.008),
        (0.184, 0.025),
        (0.164, 0.040),
        (0.126, 0.045),
        (0.088, 0.043),
        (0.075, 0.018),
        (0.070, 0.0),
        (0.075, -0.018),
        (0.088, -0.043),
    ]
    front_wheel.visual(
        mesh_from_geometry(LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0), "front_tire"),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.122, length=0.050),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="rim_band",
    )
    front_wheel.visual(
        Cylinder(radius=0.145, length=0.010),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="left_rim_face",
    )
    front_wheel.visual(
        Cylinder(radius=0.145, length=0.010),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="right_rim_face",
    )
    front_wheel.visual(
        Cylinder(radius=0.040, length=0.110),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_steel,
        name="hub",
    )

    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.475, 0.225)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    wheel = object_model.get_part("front_wheel")
    wheel_spin = object_model.get_articulation("front_wheel_spin")

    ctx.expect_origin_gap(
        wheel,
        frame,
        axis="y",
        min_gap=0.42,
        name="front wheel sits forward of the tray and legs",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="z",
        positive_elem="tray_bottom",
        negative_elem="tire",
        min_gap=0.010,
        name="tray bottom clears the top of the wheel",
    )
    ctx.expect_overlap(
        wheel,
        frame,
        axes="x",
        elem_b="front_cross_brace",
        min_overlap=0.10,
        name="wheel stays centered under the fork assembly",
    )
    ctx.expect_contact(
        frame,
        wheel,
        elem_a="left_axle_stub",
        elem_b="hub",
        contact_tol=1e-5,
        name="left axle stub supports the wheel hub",
    )
    ctx.expect_contact(
        frame,
        wheel,
        elem_a="right_axle_stub",
        elem_b="hub",
        contact_tol=1e-5,
        name="right axle stub supports the wheel hub",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({wheel_spin: 1.4}):
        spun_pos = ctx.part_world_position(wheel)
    ctx.check(
        "wheel spins in place",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
