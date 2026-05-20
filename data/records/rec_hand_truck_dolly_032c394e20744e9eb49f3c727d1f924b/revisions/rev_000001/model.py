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
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="appliance_hand_truck")

    frame_red = model.material("frame_red", rgba=(0.78, 0.18, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    rail_radius = 0.018
    rail_x = 0.18
    frame_y = -0.03
    wheel_radius = 0.17
    wheel_width = 0.056
    wheel_center_x = 0.25
    wheel_center_y = -0.105
    wheel_center_z = wheel_radius
    nose_plate_width = 0.40
    nose_plate_depth = 0.24
    nose_plate_thickness = 0.01

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
        return [(-x, y, z) for x, y, z in points]

    def add_hook(frame_part, side: float, z_anchor: float, name: str) -> None:
        hook = wire_from_points(
            [
                (side * rail_x, frame_y, z_anchor),
                (side * rail_x, 0.018, z_anchor),
                (side * rail_x, 0.030, z_anchor - 0.050),
            ],
            radius=0.006,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.012,
            corner_segments=6,
        )
        frame_part.visual(save_mesh(name, hook), material=steel)

    def add_wheel(part, prefix: str) -> None:
        half_width = wheel_width * 0.5
        tire_profile = [
            (wheel_radius * 0.58, -half_width),
            (wheel_radius * 0.84, -half_width),
            (wheel_radius * 0.95, -half_width * 0.72),
            (wheel_radius, -half_width * 0.26),
            (wheel_radius, half_width * 0.26),
            (wheel_radius * 0.95, half_width * 0.72),
            (wheel_radius * 0.84, half_width),
            (wheel_radius * 0.58, half_width),
            (wheel_radius * 0.48, half_width * 0.36),
            (wheel_radius * 0.46, 0.0),
            (wheel_radius * 0.48, -half_width * 0.36),
            (wheel_radius * 0.58, -half_width),
        ]
        tire = LatheGeometry(tire_profile, segments=56).rotate_y(pi / 2.0)
        part.visual(save_mesh(f"{prefix}_tire", tire), material=rubber, name="tire")
        spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
        part.visual(
            Cylinder(radius=wheel_radius * 0.66, length=wheel_width * 0.64),
            origin=spin_origin,
            material=steel,
            name="rim",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.31, length=wheel_width),
            origin=spin_origin,
            material=dark_steel,
            name="hub",
        )
        part.visual(
            Cylinder(radius=wheel_radius * 0.12, length=wheel_width * 1.08),
            origin=spin_origin,
            material=steel,
            name="hub_cap",
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.28, 1.46)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.03, 0.73)),
    )

    left_rail_points = [
        (rail_x, -0.012, 0.05),
        (rail_x, -0.020, 0.24),
        (rail_x, frame_y, 0.88),
        (0.17, -0.050, 1.24),
        (0.14, -0.088, 1.39),
    ]
    right_rail_points = mirror_x(left_rail_points)
    handle_points = [
        (0.14, -0.088, 1.39),
        (0.08, -0.112, 1.44),
        (-0.08, -0.112, 1.44),
        (-0.14, -0.088, 1.39),
    ]

    frame.visual(
        save_mesh(
            "left_upright",
            tube_from_spline_points(left_rail_points, radius=rail_radius, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_red,
        name="left_upright",
    )
    frame.visual(
        save_mesh(
            "right_upright",
            tube_from_spline_points(right_rail_points, radius=rail_radius, samples_per_segment=14, radial_segments=18),
        ),
        material=frame_red,
        name="right_upright",
    )
    frame.visual(
        save_mesh(
            "handle_loop",
            wire_from_points(
                handle_points,
                radius=rail_radius,
                radial_segments=18,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.035,
                corner_segments=8,
            ),
        ),
        material=frame_red,
        name="handle_loop",
    )

    frame.visual(
        Cylinder(radius=0.016, length=0.29),
        origin=Origin(xyz=(0.0, -0.020, 0.34), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="lower_rung",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.34),
        origin=Origin(xyz=(0.0, -0.026, 0.72), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="middle_rung",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.34),
        origin=Origin(xyz=(0.0, -0.040, 1.06), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="upper_rung",
    )
    frame.visual(
        Box((0.032, 0.014, 0.78)),
        origin=Origin(xyz=(0.148, -0.022, 0.74)),
        material=steel,
        name="left_load_rail",
    )
    frame.visual(
        Box((0.032, 0.014, 0.78)),
        origin=Origin(xyz=(-0.148, -0.022, 0.74)),
        material=steel,
        name="right_load_rail",
    )
    frame.visual(
        Box((0.06, 0.07, 0.10)),
        origin=Origin(xyz=(0.155, -0.055, 0.05)),
        material=frame_red,
        name="left_base_shoe",
    )
    frame.visual(
        Box((0.06, 0.07, 0.10)),
        origin=Origin(xyz=(-0.155, -0.055, 0.05)),
        material=frame_red,
        name="right_base_shoe",
    )
    frame.visual(
        Box((0.07, 0.08, 0.10)),
        origin=Origin(xyz=(rail_x, -0.078, 0.17)),
        material=frame_red,
        name="left_axle_bracket",
    )
    frame.visual(
        Box((0.07, 0.08, 0.10)),
        origin=Origin(xyz=(-rail_x, -0.078, 0.17)),
        material=frame_red,
        name="right_axle_bracket",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.444),
        origin=Origin(xyz=(0.0, wheel_center_y, wheel_center_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    frame.visual(
        save_mesh(
            "left_diagonal_brace",
            tube_from_spline_points(
                [(rail_x, -0.030, 0.31), (rail_x, -0.055, 0.25), (rail_x, -0.090, 0.17)],
                radius=0.013,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_red,
        name="left_diagonal_brace",
    )
    frame.visual(
        save_mesh(
            "right_diagonal_brace",
            tube_from_spline_points(
                mirror_x([(rail_x, -0.030, 0.31), (rail_x, -0.055, 0.25), (rail_x, -0.090, 0.17)]),
                radius=0.013,
                samples_per_segment=8,
                radial_segments=16,
            ),
        ),
        material=frame_red,
        name="right_diagonal_brace",
    )
    frame.visual(
        Box((0.27, 0.034, 0.040)),
        origin=Origin(xyz=(0.0, -0.042, 0.038)),
        material=frame_red,
        name="hinge_crossmember",
    )
    frame.visual(
        Box((0.030, 0.028, 0.034)),
        origin=Origin(xyz=(0.040, -0.014, 0.018)),
        material=frame_red,
        name="left_inner_hinge_tab",
    )
    frame.visual(
        Box((0.030, 0.028, 0.034)),
        origin=Origin(xyz=(0.120, -0.014, 0.018)),
        material=frame_red,
        name="left_outer_hinge_tab",
    )
    frame.visual(
        Box((0.030, 0.028, 0.034)),
        origin=Origin(xyz=(-0.040, -0.014, 0.018)),
        material=frame_red,
        name="right_inner_hinge_tab",
    )
    frame.visual(
        Box((0.030, 0.028, 0.034)),
        origin=Origin(xyz=(-0.120, -0.014, 0.018)),
        material=frame_red,
        name="right_outer_hinge_tab",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(0.115, -0.088, 1.41), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(-0.115, -0.088, 1.41), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )

    add_hook(frame, 1.0, 0.82, "left_hook_lower")
    add_hook(frame, -1.0, 0.82, "right_hook_lower")
    add_hook(frame, 1.0, 1.04, "left_hook_upper")
    add_hook(frame, -1.0, 1.04, "right_hook_upper")

    nose_plate = model.part("nose_plate")
    nose_plate.inertial = Inertial.from_geometry(
        Box((nose_plate_width, nose_plate_depth, 0.08)),
        mass=4.5,
        origin=Origin(xyz=(0.0, nose_plate_depth * 0.5, 0.02)),
    )
    nose_plate.visual(
        Box((nose_plate_width, 0.205, nose_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.1375, -nose_plate_thickness * 0.5)),
        material=steel,
        name="deck",
    )
    nose_plate.visual(
        Box((0.016, 0.190, 0.060)),
        origin=Origin(
            xyz=(nose_plate_width * 0.5 - 0.008, 0.130, 0.030),
        ),
        material=steel,
        name="left_side_flange",
    )
    nose_plate.visual(
        Box((0.016, 0.190, 0.060)),
        origin=Origin(
            xyz=(-nose_plate_width * 0.5 + 0.008, 0.130, 0.030),
        ),
        material=steel,
        name="right_side_flange",
    )
    nose_plate.visual(
        Box((0.030, 0.175, 0.040)),
        origin=Origin(xyz=(0.11, 0.128, -0.030)),
        material=dark_steel,
        name="left_stiffener",
    )
    nose_plate.visual(
        Box((0.030, 0.175, 0.040)),
        origin=Origin(xyz=(-0.11, 0.128, -0.030)),
        material=dark_steel,
        name="right_stiffener",
    )
    nose_plate.visual(
        Box((0.026, 0.045, 0.024)),
        origin=Origin(xyz=(0.080, 0.0225, 0.012)),
        material=dark_steel,
        name="left_hinge_web",
    )
    nose_plate.visual(
        Box((0.026, 0.045, 0.024)),
        origin=Origin(xyz=(-0.080, 0.0225, 0.012)),
        material=dark_steel,
        name="right_hinge_web",
    )
    nose_plate.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.080, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_sleeve",
    )
    nose_plate.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_sleeve",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=3.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel(left_wheel, "left_wheel")

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_width),
        mass=3.6,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    add_wheel(right_wheel, "right_wheel")

    model.articulation(
        "frame_to_nose_plate",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=nose_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6, lower=0.0, upper=pi / 2.0),
    )
    model.articulation(
        "frame_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(wheel_center_x, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(-wheel_center_x, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    nose_plate = object_model.get_part("nose_plate")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    plate_hinge = object_model.get_articulation("frame_to_nose_plate")
    left_spin = object_model.get_articulation("frame_to_left_wheel")
    right_spin = object_model.get_articulation("frame_to_right_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(frame, left_wheel)
    ctx.expect_contact(frame, right_wheel)
    ctx.expect_contact(frame, nose_plate, elem_a="left_inner_hinge_tab", elem_b="left_hinge_sleeve")
    ctx.expect_contact(frame, nose_plate, elem_a="left_outer_hinge_tab", elem_b="left_hinge_sleeve")
    ctx.expect_contact(frame, nose_plate, elem_a="right_inner_hinge_tab", elem_b="right_hinge_sleeve")
    ctx.expect_contact(frame, nose_plate, elem_a="right_outer_hinge_tab", elem_b="right_hinge_sleeve")
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="x", min_dist=0.48, max_dist=0.52)
    ctx.expect_overlap(nose_plate, frame, axes="x", min_overlap=0.30)

    ctx.check(
        "nose_plate_hinge_axis_is_transverse",
        tuple(plate_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"Expected (1, 0, 0), got {plate_hinge.axis!r}",
    )
    ctx.check(
        "left_wheel_axis_is_axle_aligned",
        tuple(left_spin.axis) == (1.0, 0.0, 0.0),
        details=f"Expected (1, 0, 0), got {left_spin.axis!r}",
    )
    ctx.check(
        "right_wheel_axis_is_axle_aligned",
        tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=f"Expected (1, 0, 0), got {right_spin.axis!r}",
    )

    open_aabb = ctx.part_world_aabb(nose_plate)
    assert open_aabb is not None
    with ctx.pose({plate_hinge: pi / 2.0}):
        folded_aabb = ctx.part_world_aabb(nose_plate)
        assert folded_aabb is not None
        ctx.expect_contact(frame, nose_plate, elem_a="left_inner_hinge_tab", elem_b="left_hinge_sleeve")
        ctx.expect_contact(frame, nose_plate, elem_a="left_outer_hinge_tab", elem_b="left_hinge_sleeve")
        ctx.expect_contact(frame, nose_plate, elem_a="right_inner_hinge_tab", elem_b="right_hinge_sleeve")
        ctx.expect_contact(frame, nose_plate, elem_a="right_outer_hinge_tab", elem_b="right_hinge_sleeve")
        ctx.expect_overlap(nose_plate, frame, axes="xz", min_overlap=0.18)
        assert folded_aabb[1][2] > open_aabb[1][2] + 0.14
        assert folded_aabb[1][1] < open_aabb[1][1] - 0.14

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
