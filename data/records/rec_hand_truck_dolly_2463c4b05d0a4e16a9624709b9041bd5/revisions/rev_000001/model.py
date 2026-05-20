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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="convertible_hand_truck")

    frame_red = model.material("frame_red", rgba=(0.74, 0.10, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.11, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.66, 0.67, 0.69, 1.0))

    main_frame = model.part("main_frame")
    main_frame.visual(
        Cylinder(radius=0.018, length=1.08),
        origin=Origin(xyz=(-0.18, -0.02, 0.56)),
        material=frame_red,
        name="left_rail",
    )
    main_frame.visual(
        Cylinder(radius=0.018, length=1.08),
        origin=Origin(xyz=(0.18, -0.02, 0.56)),
        material=frame_red,
        name="right_rail",
    )
    handle_geom = wire_from_points(
        [
            (-0.18, -0.02, 0.94),
            (-0.18, -0.02, 1.16),
            (0.18, -0.02, 1.16),
            (0.18, -0.02, 0.94),
        ],
        radius=0.018,
        closed_path=False,
        cap_ends=False,
        corner_mode="fillet",
        corner_radius=0.07,
        corner_segments=12,
    )
    main_frame.visual(
        mesh_from_geometry(handle_geom, "hand_truck_handle"),
        material=frame_red,
        name="handle_loop",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.324),
        origin=Origin(xyz=(0.0, -0.02, 0.78), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="upper_crossbar",
    )
    main_frame.visual(
        Cylinder(radius=0.014, length=0.324),
        origin=Origin(xyz=(0.0, -0.02, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_red,
        name="mid_crossbar",
    )
    main_frame.visual(
        Box((0.34, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, -0.01, 0.03)),
        material=dark_steel,
        name="lower_mount_block",
    )
    main_frame.visual(
        Cylinder(radius=0.017, length=0.38),
        origin=Origin(xyz=(0.0, -0.055, 0.30), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="axle_beam",
    )
    main_frame.visual(
        Cylinder(radius=0.05, length=0.04),
        origin=Origin(xyz=(-0.21, -0.055, 0.30), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_axle_stub",
    )
    main_frame.visual(
        Cylinder(radius=0.05, length=0.04),
        origin=Origin(xyz=(0.21, -0.055, 0.30), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_axle_stub",
    )

    nose_plate = model.part("primary_nose_plate")
    nose_plate.visual(
        Box((0.34, 0.23, 0.012)),
        origin=Origin(xyz=(0.0, 0.115, 0.006)),
        material=steel,
        name="plate",
    )
    nose_plate.visual(
        Box((0.34, 0.025, 0.06)),
        origin=Origin(xyz=(0.0, 0.0125, 0.03)),
        material=steel,
        name="rear_flange",
    )
    nose_plate.visual(
        Box((0.03, 0.05, 0.045)),
        origin=Origin(xyz=(-0.155, 0.217, -0.0225)),
        material=dark_steel,
        name="left_hinge_ear",
    )
    nose_plate.visual(
        Box((0.03, 0.05, 0.045)),
        origin=Origin(xyz=(0.155, 0.217, -0.0225)),
        material=dark_steel,
        name="right_hinge_ear",
    )
    nose_plate.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(-0.155, 0.242, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_knuckle",
    )
    nose_plate.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.155, 0.242, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_knuckle",
    )

    secondary_toe = model.part("secondary_toe_section")
    secondary_toe.visual(
        Box((0.28, 0.12, 0.008)),
        origin=Origin(xyz=(0.0, -0.08, -0.008)),
        material=steel,
        name="toe_plate",
    )
    secondary_toe.visual(
        Box((0.28, 0.02, 0.012)),
        origin=Origin(xyz=(0.0, -0.01, -0.006)),
        material=dark_steel,
        name="hinge_leaf",
    )
    secondary_toe.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(-0.125, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_knuckle",
    )
    secondary_toe.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_knuckle",
    )
    secondary_toe.visual(
        Box((0.18, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, -0.065, -0.013)),
        material=dark_steel,
        name="center_rib",
    )

    left_wheel = model.part("left_rear_wheel")
    left_wheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.11, tube=0.025), "left_wheel_tire"),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.095, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_gray,
        name="rim_disc",
    )
    left_wheel.visual(
        Cylinder(radius=0.032, length=0.06),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_gray,
        name="hub",
    )

    right_wheel = model.part("right_rear_wheel")
    right_wheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.11, tube=0.025), "right_wheel_tire"),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.095, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_gray,
        name="rim_disc",
    )
    right_wheel.visual(
        Cylinder(radius=0.032, length=0.06),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_gray,
        name="hub",
    )

    model.articulation(
        "frame_to_nose_plate",
        ArticulationType.FIXED,
        parent=main_frame,
        child=nose_plate,
        origin=Origin(),
    )
    model.articulation(
        "nose_plate_to_secondary_toe",
        ArticulationType.REVOLUTE,
        parent=nose_plate,
        child=secondary_toe,
        origin=Origin(xyz=(0.0, 0.242, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=0.0,
            upper=pi,
        ),
    )
    model.articulation(
        "frame_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.26, -0.055, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=main_frame,
        child=right_wheel,
        origin=Origin(xyz=(0.26, -0.055, 0.30)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_frame = object_model.get_part("main_frame")
    nose_plate = object_model.get_part("primary_nose_plate")
    secondary_toe = object_model.get_part("secondary_toe_section")
    left_wheel = object_model.get_part("left_rear_wheel")
    right_wheel = object_model.get_part("right_rear_wheel")

    toe_hinge = object_model.get_articulation("nose_plate_to_secondary_toe")
    left_wheel_joint = object_model.get_articulation("frame_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("frame_to_right_wheel")

    lower_mount_block = main_frame.get_visual("lower_mount_block")
    left_axle_stub = main_frame.get_visual("left_axle_stub")
    right_axle_stub = main_frame.get_visual("right_axle_stub")
    plate_shell = nose_plate.get_visual("plate")
    rear_flange = nose_plate.get_visual("rear_flange")
    left_ear_knuckle = nose_plate.get_visual("left_hinge_knuckle")
    right_ear_knuckle = nose_plate.get_visual("right_hinge_knuckle")
    toe_plate = secondary_toe.get_visual("toe_plate")
    toe_leaf = secondary_toe.get_visual("hinge_leaf")
    toe_left_knuckle = secondary_toe.get_visual("left_hinge_knuckle")
    toe_right_knuckle = secondary_toe.get_visual("right_hinge_knuckle")
    left_hub = left_wheel.get_visual("hub")
    right_hub = right_wheel.get_visual("hub")

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

    ctx.expect_contact(
        main_frame,
        nose_plate,
        elem_a=lower_mount_block,
        elem_b=rear_flange,
        name="nose_plate_mounted_to_frame",
    )
    ctx.expect_contact(
        main_frame,
        left_wheel,
        elem_a=left_axle_stub,
        elem_b=left_hub,
        name="left_wheel_mounted_on_stub",
    )
    ctx.expect_contact(
        main_frame,
        right_wheel,
        elem_a=right_axle_stub,
        elem_b=right_hub,
        name="right_wheel_mounted_on_stub",
    )
    ctx.expect_origin_distance(
        left_wheel,
        right_wheel,
        axes="x",
        min_dist=0.50,
        max_dist=0.54,
        name="wheel_track_width_realistic",
    )
    ctx.expect_within(
        secondary_toe,
        nose_plate,
        axes="x",
        margin=0.03,
        inner_elem=toe_plate,
        outer_elem=plate_shell,
        name="secondary_toe_width_nested_under_primary_plate",
    )

    limits = toe_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({toe_hinge: limits.lower}):
            ctx.expect_gap(
                nose_plate,
                secondary_toe,
                axis="z",
                positive_elem=plate_shell,
                negative_elem=toe_plate,
                min_gap=0.003,
                max_gap=0.006,
                name="secondary_toe_stowed_below_primary_plate",
            )
            ctx.expect_overlap(
                secondary_toe,
                nose_plate,
                axes="xy",
                elem_a=toe_plate,
                elem_b=plate_shell,
                min_overlap=0.12,
                name="secondary_toe_stowed_under_plate_footprint",
            )
            ctx.expect_contact(
                nose_plate,
                secondary_toe,
                elem_a=left_ear_knuckle,
                elem_b=toe_left_knuckle,
                name="left_hinge_knuckle_contact_stowed",
            )
            ctx.expect_contact(
                nose_plate,
                secondary_toe,
                elem_a=right_ear_knuckle,
                elem_b=toe_right_knuckle,
                name="right_hinge_knuckle_contact_stowed",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="secondary_toe_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="secondary_toe_lower_no_floating")
        with ctx.pose({toe_hinge: limits.upper}):
            ctx.expect_gap(
                secondary_toe,
                nose_plate,
                axis="y",
                positive_elem=toe_leaf,
                negative_elem=plate_shell,
                min_gap=0.010,
                max_gap=0.014,
                name="secondary_toe_deployed_from_front_edge",
            )
            ctx.expect_overlap(
                secondary_toe,
                nose_plate,
                axes="x",
                elem_a=toe_plate,
                elem_b=plate_shell,
                min_overlap=0.26,
                name="secondary_toe_deployed_width_alignment",
            )
            ctx.expect_contact(
                nose_plate,
                secondary_toe,
                elem_a=left_ear_knuckle,
                elem_b=toe_left_knuckle,
                name="left_hinge_knuckle_contact_deployed",
            )
            ctx.expect_contact(
                nose_plate,
                secondary_toe,
                elem_a=right_ear_knuckle,
                elem_b=toe_right_knuckle,
                name="right_hinge_knuckle_contact_deployed",
            )
            ctx.fail_if_parts_overlap_in_current_pose(name="secondary_toe_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="secondary_toe_upper_no_floating")

    with ctx.pose({left_wheel_joint: 1.1, right_wheel_joint: -0.9}):
        ctx.expect_contact(
            main_frame,
            left_wheel,
            elem_a=left_axle_stub,
            elem_b=left_hub,
            name="left_wheel_contact_after_spin",
        )
        ctx.expect_contact(
            main_frame,
            right_wheel,
            elem_a=right_axle_stub,
            elem_b=right_hub,
            name="right_wheel_contact_after_spin",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="wheels_spun_no_overlap")
        ctx.fail_if_isolated_parts(name="wheels_spun_no_floating")

    frame_aabb = ctx.part_world_aabb(main_frame)
    if frame_aabb is not None:
        frame_height = frame_aabb[1][2] - frame_aabb[0][2]
        ctx.check(
            "frame_height_realistic",
            1.10 <= frame_height <= 1.22,
            f"frame height {frame_height:.3f} m is outside realistic hand-truck range",
        )

    plate_aabb = ctx.part_world_aabb(nose_plate)
    if plate_aabb is not None:
        plate_width = plate_aabb[1][0] - plate_aabb[0][0]
        plate_depth = plate_aabb[1][1] - plate_aabb[0][1]
        ctx.check(
            "primary_nose_plate_size_realistic",
            0.32 <= plate_width <= 0.36 and 0.22 <= plate_depth <= 0.26,
            (
                f"nose plate dimensions {plate_width:.3f} x {plate_depth:.3f} m "
                "are outside realistic range"
            ),
        )

    wheel_aabb = ctx.part_world_aabb(left_wheel)
    if wheel_aabb is not None:
        wheel_diameter = wheel_aabb[1][2] - wheel_aabb[0][2]
        ctx.check(
            "rear_wheel_diameter_realistic",
            0.25 <= wheel_diameter <= 0.28,
            f"rear wheel diameter {wheel_diameter:.3f} m is outside realistic range",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
