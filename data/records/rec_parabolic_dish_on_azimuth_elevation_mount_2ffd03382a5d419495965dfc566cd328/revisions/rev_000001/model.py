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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _tube(name: str, points, *, radius: float, samples_per_segment: int = 10, radial_segments: int = 16):
    return _mesh(
        name,
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_vsat_roof_tripod")

    galvanized = model.material("galvanized", rgba=(0.58, 0.61, 0.64, 1.0))
    head_gray = model.material("head_gray", rgba=(0.29, 0.31, 0.34, 1.0))
    reflector_white = model.material("reflector_white", rgba=(0.90, 0.91, 0.92, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.41, 0.43, 0.46, 1.0))
    receiver_black = model.material("receiver_black", rgba=(0.09, 0.10, 0.11, 1.0))

    leg_template = _tube(
        "tripod_leg",
        [
            (0.0, 0.0, 0.220),
            (0.170, 0.0, 0.135),
            (0.380, 0.0, 0.014),
        ],
        radius=0.018,
        samples_per_segment=8,
        radial_segments=14,
    )
    brace_template = _tube(
        "tripod_brace",
        [
            (0.0, 0.0, 0.135),
            (0.145, 0.0, 0.100),
            (0.305, 0.0, 0.048),
        ],
        radius=0.0105,
        samples_per_segment=6,
        radial_segments=12,
    )

    reflector_shell = _mesh(
        "reflector_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.028, -0.040),
                (0.115, -0.034),
                (0.215, -0.019),
                (0.300, 0.010),
                (0.360, 0.046),
            ],
            [
                (0.000, -0.034),
                (0.096, -0.030),
                (0.202, -0.017),
                (0.293, 0.008),
                (0.348, 0.041),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )
    rim_ring = _mesh(
        "reflector_rim_ring",
        TorusGeometry(
            radius=0.355,
            tube=0.008,
            radial_segments=18,
            tubular_segments=72,
        ).rotate_y(math.pi / 2.0),
    )
    rear_ring = _mesh(
        "rear_stiffener_ring",
        TorusGeometry(
            radius=0.190,
            tube=0.009,
            radial_segments=16,
            tubular_segments=56,
        ).rotate_y(math.pi / 2.0),
    )
    radial_rib = _tube(
        "rear_radial_rib",
        [
            (0.025, 0.0, 0.000),
            (0.070, 0.0, 0.045),
            (0.108, 0.0, 0.125),
            (0.122, 0.0, 0.190),
        ],
        radius=0.008,
        samples_per_segment=8,
        radial_segments=12,
    )
    cradle_left = _tube(
        "rear_cradle_left",
        [
            (-0.015, 0.050, -0.080),
            (0.010, 0.056, 0.000),
            (0.046, 0.060, 0.102),
            (0.092, 0.055, 0.192),
        ],
        radius=0.012,
        samples_per_segment=10,
        radial_segments=14,
    )
    cradle_right = _tube(
        "rear_cradle_right",
        [
            (-0.015, -0.050, -0.080),
            (0.010, -0.056, 0.000),
            (0.046, -0.060, 0.102),
            (0.092, -0.055, 0.192),
        ],
        radius=0.012,
        samples_per_segment=10,
        radial_segments=14,
    )
    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.032, length=0.480),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=galvanized,
        name="mast",
    )
    tripod_base.visual(
        Cylinder(radius=0.075, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=head_gray,
        name="crown_collar",
    )
    tripod_base.visual(
        Cylinder(radius=0.055, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
        material=head_gray,
        name="mast_top_plate",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        foot_x = 0.380 * math.cos(angle)
        foot_y = 0.380 * math.sin(angle)
        tripod_base.visual(
            Box((0.180, 0.080, 0.014)),
            origin=Origin(xyz=(foot_x, foot_y, 0.007), rpy=(0.0, 0.0, angle)),
            material=head_gray,
            name=f"foot_pad_{index}",
        )
        tripod_base.visual(
            leg_template,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=galvanized,
            name=f"leg_{index}",
        )
        tripod_base.visual(
            brace_template,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=galvanized,
            name=f"brace_{index}",
        )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=head_gray,
        name="turntable_drum",
    )
    azimuth_head.visual(
        Box((0.080, 0.180, 0.050)),
        origin=Origin(xyz=(-0.085, 0.0, 0.045)),
        material=head_gray,
        name="yoke_base",
    )
    azimuth_head.visual(
        Box((0.040, 0.184, 0.100)),
        origin=Origin(xyz=(-0.080, 0.0, 0.100)),
        material=head_gray,
        name="rear_bridge",
    )
    azimuth_head.visual(
        Box((0.050, 0.020, 0.100)),
        origin=Origin(xyz=(-0.050, 0.092, 0.095)),
        material=head_gray,
        name="left_connector",
    )
    azimuth_head.visual(
        Box((0.050, 0.020, 0.100)),
        origin=Origin(xyz=(-0.050, -0.092, 0.095)),
        material=head_gray,
        name="right_connector",
    )
    azimuth_head.visual(
        Box((0.085, 0.020, 0.230)),
        origin=Origin(xyz=(0.000, 0.102, 0.115)),
        material=head_gray,
        name="left_cheek",
    )
    azimuth_head.visual(
        Box((0.085, 0.020, 0.230)),
        origin=Origin(xyz=(0.000, -0.102, 0.115)),
        material=head_gray,
        name="right_cheek",
    )
    azimuth_head.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.000, 0.115, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="left_outer_pivot_cap",
    )
    azimuth_head.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.000, -0.115, 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="right_outer_pivot_cap",
    )

    dish_frame = model.part("dish_frame")
    dish_frame.visual(
        Box((0.050, 0.100, 0.100)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=head_gray,
        name="mount_block",
    )
    dish_frame.visual(
        Box((0.035, 0.045, 0.075)),
        origin=Origin(xyz=(0.005, 0.055, 0.0)),
        material=frame_gray,
        name="left_pivot_lug",
    )
    dish_frame.visual(
        Box((0.035, 0.045, 0.075)),
        origin=Origin(xyz=(0.005, -0.055, 0.0)),
        material=frame_gray,
        name="right_pivot_lug",
    )
    dish_frame.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, 0.082, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="pivot_bush_left",
    )
    dish_frame.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, -0.082, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="pivot_bush_right",
    )
    dish_frame.visual(
        Cylinder(radius=0.040, length=0.090),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
        name="center_boss",
    )
    dish_frame.visual(
        reflector_shell,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=reflector_white,
        name="reflector_shell",
    )
    dish_frame.visual(
        rim_ring,
        origin=Origin(xyz=(0.186, 0.0, 0.0)),
        material=frame_gray,
        name="reflector_rim",
    )
    dish_frame.visual(
        rear_ring,
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
        material=frame_gray,
        name="rear_stiffener_ring",
    )
    for index in range(4):
        angle = index * (math.pi / 2.0)
        dish_frame.visual(
            radial_rib,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=frame_gray,
            name=f"rear_rib_{index}",
        )
    dish_frame.visual(cradle_left, material=frame_gray, name="left_cradle_rib")
    dish_frame.visual(cradle_right, material=frame_gray, name="right_cradle_rib")
    dish_frame.visual(
        Cylinder(radius=0.012, length=0.122),
        origin=Origin(xyz=(0.040, 0.0, 0.142), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="rear_cradle_upper_bar",
    )
    dish_frame.visual(
        Cylinder(radius=0.009, length=0.112),
        origin=Origin(xyz=(0.000, 0.0, -0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="rear_cradle_lower_bar",
    )
    dish_frame.visual(
        Box((0.050, 0.036, 0.040)),
        origin=Origin(xyz=(0.096, 0.0, -0.020)),
        material=frame_gray,
        name="feed_arm_bracket",
    )
    dish_frame.visual(
        Cylinder(radius=0.011, length=0.220),
        origin=Origin(xyz=(0.195, 0.0, -0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
        name="feed_arm_tube",
    )
    dish_frame.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.295, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
        name="feed_hub",
    )
    dish_frame.visual(
        Cylinder(radius=0.023, length=0.055),
        origin=Origin(xyz=(0.325, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=receiver_black,
        name="feed_horn",
    )
    dish_frame.visual(
        Box((0.030, 0.046, 0.036)),
        origin=Origin(xyz=(0.350, 0.0, 0.000)),
        material=receiver_black,
        name="lnb_block",
    )

    model.articulation(
        "tripod_to_head",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4),
    )
    model.articulation(
        "head_to_dish",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=dish_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.9,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    azimuth_head = object_model.get_part("azimuth_head")
    dish_frame = object_model.get_part("dish_frame")

    tripod_to_head = object_model.get_articulation("tripod_to_head")
    head_to_dish = object_model.get_articulation("head_to_dish")

    mast_top_plate = tripod_base.get_visual("mast_top_plate")
    turntable_drum = azimuth_head.get_visual("turntable_drum")
    left_cheek = azimuth_head.get_visual("left_cheek")
    right_cheek = azimuth_head.get_visual("right_cheek")
    left_pivot = dish_frame.get_visual("pivot_bush_left")
    right_pivot = dish_frame.get_visual("pivot_bush_right")
    reflector = dish_frame.get_visual("reflector_shell")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    az_axis = tuple(tripod_to_head.axis)
    el_axis = tuple(head_to_dish.axis)
    ctx.check(
        "azimuth_joint_is_vertical",
        abs(az_axis[0]) < 1e-9 and abs(az_axis[1]) < 1e-9 and abs(abs(az_axis[2]) - 1.0) < 1e-9,
        details=f"expected vertical azimuth axis, got {tripod_to_head.axis}",
    )
    ctx.check(
        "elevation_joint_is_horizontal",
        abs(el_axis[0]) < 1e-9 and abs(abs(el_axis[1]) - 1.0) < 1e-9 and abs(el_axis[2]) < 1e-9,
        details=f"expected left-right elevation axis, got {head_to_dish.axis}",
    )

    ctx.expect_contact(
        azimuth_head,
        tripod_base,
        elem_a=turntable_drum,
        elem_b=mast_top_plate,
        name="turntable_seats_on_tripod_mast",
    )
    ctx.expect_contact(
        dish_frame,
        azimuth_head,
        elem_a=left_pivot,
        elem_b=left_cheek,
        name="left_side_pivot_clamps_dish_frame",
    )
    ctx.expect_contact(
        dish_frame,
        azimuth_head,
        elem_a=right_pivot,
        elem_b=right_cheek,
        name="right_side_pivot_clamps_dish_frame",
    )
    ctx.expect_gap(
        dish_frame,
        tripod_base,
        axis="x",
        min_gap=0.04,
        positive_elem=reflector,
        negative_elem=mast_top_plate,
        name="reflector_stays_forward_of_tripod_mast",
    )

    horn_rest = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
    if horn_rest is None:
        ctx.fail("feed_horn_rest_aabb_available", "feed_horn AABB unavailable at rest pose")
        return ctx.report()
    horn_rest_center_y = 0.5 * (horn_rest[0][1] + horn_rest[1][1])
    horn_rest_center_z = 0.5 * (horn_rest[0][2] + horn_rest[1][2])

    with ctx.pose({head_to_dish: 0.85}):
        horn_raised = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        if horn_raised is None:
            ctx.fail("feed_horn_raised_aabb_available", "feed_horn AABB unavailable in elevated pose")
        else:
            horn_raised_center_z = 0.5 * (horn_raised[0][2] + horn_raised[1][2])
            ctx.check(
                "feed_horn_rises_with_elevation",
                horn_raised_center_z > horn_rest_center_z + 0.18,
                details=(
                    f"expected feed horn to rise by > 0.18 m, got "
                    f"{horn_raised_center_z - horn_rest_center_z:.3f} m"
                ),
            )
            ctx.expect_contact(
                dish_frame,
                azimuth_head,
                elem_a=left_pivot,
                elem_b=left_cheek,
                name="left_pivot_remains_seated_when_raised",
            )
            ctx.expect_contact(
                dish_frame,
                azimuth_head,
                elem_a=right_pivot,
                elem_b=right_cheek,
                name="right_pivot_remains_seated_when_raised",
            )

    with ctx.pose({tripod_to_head: math.pi / 4.0}):
        horn_turned = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        if horn_turned is None:
            ctx.fail("feed_horn_turned_aabb_available", "feed_horn AABB unavailable in azimuth pose")
        else:
            horn_turned_center_y = 0.5 * (horn_turned[0][1] + horn_turned[1][1])
            ctx.check(
                "feed_horn_swings_sideways_in_azimuth",
                horn_turned_center_y > horn_rest_center_y + 0.15,
                details=(
                    f"expected feed horn y shift > 0.15 m, got "
                    f"{horn_turned_center_y - horn_rest_center_y:.3f} m"
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
