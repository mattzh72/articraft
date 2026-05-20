from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, segments: int = 48):
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelbarrow")

    steel = model.material("steel", rgba=(0.27, 0.30, 0.33, 1.0))
    tray_paint = model.material("tray_paint", rgba=(0.67, 0.09, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.77, 0.79, 0.81, 1.0))
    grip = model.material("grip", rgba=(0.23, 0.15, 0.08, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 1.55, 0.66)),
        mass=16.0,
        origin=Origin(xyz=(0.0, -0.05, 0.31)),
    )

    left_handle_rail = tube_from_spline_points(
        [
            (0.30, -0.74, 0.56),
            (0.29, -0.55, 0.49),
            (0.25, -0.30, 0.41),
            (0.19, 0.05, 0.37),
            (0.09, 0.50, 0.33),
        ],
        radius=0.019,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_handle_rail = tube_from_spline_points(
        [
            (-0.30, -0.74, 0.56),
            (-0.29, -0.55, 0.49),
            (-0.25, -0.30, 0.41),
            (-0.19, 0.05, 0.37),
            (-0.09, 0.50, 0.33),
        ],
        radius=0.019,
        samples_per_segment=16,
        radial_segments=18,
    )
    frame.visual(_mesh("frame_left_handle_rail", left_handle_rail), material=steel, name="left_handle_rail")
    frame.visual(_mesh("frame_right_handle_rail", right_handle_rail), material=steel, name="right_handle_rail")

    for name, x_sign in (("left_fork_blade", 1.0), ("right_fork_blade", -1.0)):
        fork_blade = tube_from_spline_points(
            [
                (0.09 * x_sign, 0.50, 0.33),
                (0.07 * x_sign, 0.57, 0.28),
                (0.065 * x_sign, 0.62, 0.20),
            ],
            radius=0.012,
            samples_per_segment=12,
            radial_segments=16,
        )
        frame.visual(_mesh(f"{name}_mesh", fork_blade), material=steel, name=name)

    for name, x_sign in (("left_leg", 1.0), ("right_leg", -1.0)):
        leg_geom = tube_from_spline_points(
            [
                (0.24 * x_sign, -0.24, 0.40),
                (0.24 * x_sign, -0.36, 0.23),
                (0.24 * x_sign, -0.46, 0.018),
            ],
            radius=0.016,
            samples_per_segment=10,
            radial_segments=16,
        )
        frame.visual(_mesh(f"{name}_mesh", leg_geom), material=steel, name=name)

    frame.visual(
        Cylinder(radius=0.016, length=0.30),
        origin=Origin(xyz=(0.0, 0.18, 0.354), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="front_crossbrace",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.34),
        origin=Origin(xyz=(0.0, 0.05, 0.37), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="mid_crossbrace",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.46),
        origin=Origin(xyz=(0.0, -0.28, 0.40), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rear_crossbrace",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.48),
        origin=Origin(xyz=(0.0, -0.42, 0.12), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="leg_spreader",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.058, 0.62, 0.20), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="left_axle_stub",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(-0.058, 0.62, 0.20), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="right_axle_stub",
    )

    frame.visual(
        Box((0.09, 0.035, 0.018)),
        origin=Origin(xyz=(0.24, -0.46, 0.009)),
        material=steel,
        name="left_foot",
    )
    frame.visual(
        Box((0.09, 0.035, 0.018)),
        origin=Origin(xyz=(-0.24, -0.46, 0.009)),
        material=steel,
        name="right_foot",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.16),
        origin=Origin(xyz=(0.30, -0.76, 0.56), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.16),
        origin=Origin(xyz=(-0.30, -0.76, 0.56), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip,
        name="right_grip",
    )
    frame.visual(
        Box((0.40, 0.78, 0.018)),
        origin=Origin(xyz=(0.0, -0.05, 0.411)),
        material=tray_paint,
        name="tray_bottom",
    )
    frame.visual(
        Box((0.022, 0.74, 0.14)),
        origin=Origin(xyz=(0.189, -0.05, 0.489)),
        material=tray_paint,
        name="tray_left_wall",
    )
    frame.visual(
        Box((0.022, 0.74, 0.14)),
        origin=Origin(xyz=(-0.189, -0.05, 0.489)),
        material=tray_paint,
        name="tray_right_wall",
    )
    frame.visual(
        Box((0.356, 0.022, 0.18)),
        origin=Origin(xyz=(0.0, 0.329, 0.501)),
        material=tray_paint,
        name="tray_front_wall",
    )
    frame.visual(
        Box((0.28, 0.022, 0.10)),
        origin=Origin(xyz=(0.0, -0.429, 0.461)),
        material=tray_paint,
        name="tray_rear_lip",
    )
    frame.visual(
        Box((0.030, 0.10, 0.036)),
        origin=Origin(xyz=(0.22, -0.21, 0.384)),
        material=steel,
        name="left_rear_tray_bracket",
    )
    frame.visual(
        Box((0.030, 0.10, 0.036)),
        origin=Origin(xyz=(-0.22, -0.21, 0.384)),
        material=steel,
        name="right_rear_tray_bracket",
    )
    frame.visual(
        Box((0.030, 0.10, 0.096)),
        origin=Origin(xyz=(0.15, 0.12, 0.354)),
        material=steel,
        name="left_front_tray_bracket",
    )
    frame.visual(
        Box((0.030, 0.10, 0.096)),
        origin=Origin(xyz=(-0.15, 0.12, 0.354)),
        material=steel,
        name="right_front_tray_bracket",
    )

    wheel = model.part("wheel")
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.082),
        mass=4.8,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    tire_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.19, 64),
        [_circle_profile(0.11, 64)],
        0.082,
        center=True,
    )
    tire_geom.rotate_y(pi / 2.0)
    wheel.visual(
        _mesh("front_tire_mesh", tire_geom),
        material=rubber,
        name="tire",
    )

    rim_geom = ExtrudeWithHolesGeometry(
        _circle_profile(0.112, 56),
        [_circle_profile(0.020, 40)],
        0.030,
        center=True,
    )
    rim_geom.rotate_y(pi / 2.0)
    wheel.visual(
        _mesh("front_rim_mesh", rim_geom),
        material=wheel_metal,
        name="rim",
    )
    wheel.visual(
        Cylinder(radius=0.040, length=0.098),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )

    model.articulation(
        "frame_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.62, 0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    wheel = object_model.get_part("wheel")
    wheel_joint = object_model.get_articulation("frame_to_wheel")

    ctx.expect_gap(
        frame,
        wheel,
        axis="x",
        positive_elem="left_fork_blade",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.040,
        name="left fork clears tire",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="x",
        positive_elem="tire",
        negative_elem="right_fork_blade",
        min_gap=0.006,
        max_gap=0.040,
        name="right fork clears tire",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="x",
        positive_elem="left_fork_blade",
        negative_elem="hub",
        min_gap=0.001,
        max_gap=0.020,
        name="left fork clears hub",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="x",
        positive_elem="hub",
        negative_elem="right_fork_blade",
        min_gap=0.001,
        max_gap=0.020,
        name="right fork clears hub",
    )
    ctx.expect_gap(
        frame,
        wheel,
        axis="x",
        positive_elem="left_axle_stub",
        negative_elem="tire",
        min_gap=0.006,
        max_gap=0.020,
        name="left axle stub stays outside the tire sidewall",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="x",
        positive_elem="tire",
        negative_elem="right_axle_stub",
        min_gap=0.006,
        max_gap=0.020,
        name="right axle stub stays outside the tire sidewall",
    )
    ctx.expect_gap(
        wheel,
        frame,
        axis="y",
        positive_elem="tire",
        negative_elem="left_foot",
        min_gap=0.70,
        name="wheel sits well ahead of the rear support plane",
    )
    left_fork = ctx.part_element_world_aabb(frame, elem="left_fork_blade")
    right_fork = ctx.part_element_world_aabb(frame, elem="right_fork_blade")
    left_foot = ctx.part_element_world_aabb(frame, elem="left_foot")
    right_foot = ctx.part_element_world_aabb(frame, elem="right_foot")

    def _center_x(aabb):
        return (aabb[0][0] + aabb[1][0]) * 0.5

    if left_fork and right_fork and left_foot and right_foot:
        front_support_width = _center_x(left_fork) - _center_x(right_fork)
        rear_support_width = _center_x(left_foot) - _center_x(right_foot)
        ctx.check(
            "rear stance is broader than the narrow front fork",
            rear_support_width > front_support_width + 0.30,
            details=f"front_support_width={front_support_width:.3f}, rear_support_width={rear_support_width:.3f}",
        )
    else:
        ctx.fail("rear stance is broader than the narrow front fork", "missing support element AABBs")
    ctx.check(
        "front wheel uses a continuous axle spin joint",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(wheel_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={wheel_joint.articulation_type}, axis={wheel_joint.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
