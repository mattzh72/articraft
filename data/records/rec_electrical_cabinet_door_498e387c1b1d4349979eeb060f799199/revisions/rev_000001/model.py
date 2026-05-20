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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_enclosure")

    body_width = 0.65
    body_depth = 0.28
    body_height = 0.90
    shell_thickness = 0.004

    door_width = 0.638
    door_height = body_height - 2.0 * shell_thickness
    door_skin_thickness = 0.004
    door_return_width = 0.014
    door_return_depth = 0.034

    hinge_axis_x = -0.319
    hinge_axis_y = 0.014
    hinge_barrel_radius = 0.008
    hinge_segment_length = 0.032
    hinge_lower_z = 0.18
    hinge_upper_z = 0.72

    rod_radius = 0.006
    rod_length = 0.70
    rod_origin_x = 0.586
    rod_origin_y = -0.028
    rod_origin_z = 0.10
    rod_travel = 0.045
    dog_length = 0.045
    dog_depth = 0.010
    dog_height = 0.014
    dog_offset_x = rod_radius + dog_length / 2.0
    dog_local_zs = (0.08, 0.35, 0.62)
    strike_zs = tuple(rod_origin_z + z for z in dog_local_zs)

    door_skin_center_y = -hinge_axis_y - door_skin_thickness / 2.0
    door_return_center_y = -hinge_axis_y - door_skin_thickness - door_return_depth / 2.0
    guide_back_center_y = -0.020
    guide_side_width = 0.0025

    enclosure_steel = model.material("enclosure_steel", rgba=(0.33, 0.36, 0.39, 1.0))
    galvanized = model.material("galvanized_hardware", rgba=(0.73, 0.75, 0.78, 1.0))
    zinc = model.material("zinc_lockwork", rgba=(0.66, 0.68, 0.71, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, body_height / 2.0)),
    )
    body.visual(
        Box((body_width, shell_thickness, body_height)),
        origin=Origin(xyz=(0.0, -body_depth + shell_thickness / 2.0, body_height / 2.0)),
        material=enclosure_steel,
        name="back_panel",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(xyz=(-body_width / 2.0 + shell_thickness / 2.0, -body_depth / 2.0, body_height / 2.0)),
        material=enclosure_steel,
        name="left_wall",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(xyz=(body_width / 2.0 - shell_thickness / 2.0, -body_depth / 2.0, body_height / 2.0)),
        material=enclosure_steel,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_thickness, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0, body_height - shell_thickness / 2.0)),
        material=enclosure_steel,
        name="top_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_thickness, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0, shell_thickness / 2.0)),
        material=enclosure_steel,
        name="bottom_wall",
    )

    for label, hinge_z in (("lower", hinge_lower_z), ("upper", hinge_upper_z)):
        body.visual(
            Box((0.024, 0.006, 0.12)),
            origin=Origin(xyz=(hinge_axis_x - 0.018, 0.003, hinge_z)),
            material=galvanized,
            name=f"{label}_hinge_body_leaf",
        )
        body.visual(
            Box((0.030, 0.016, hinge_segment_length)),
            origin=Origin(xyz=(hinge_axis_x - 0.011, 0.011, hinge_z - hinge_segment_length)),
            material=galvanized,
            name=f"{label}_hinge_body_lower_web",
        )
        body.visual(
            Box((0.030, 0.016, hinge_segment_length)),
            origin=Origin(xyz=(hinge_axis_x - 0.011, 0.011, hinge_z + hinge_segment_length)),
            material=galvanized,
            name=f"{label}_hinge_body_upper_web",
        )
        body.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_segment_length),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_z - hinge_segment_length)),
            material=galvanized,
            name=f"{label}_hinge_lower_knuckle",
        )
        body.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_segment_length),
            origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, hinge_z + hinge_segment_length)),
            material=galvanized,
            name=f"{label}_hinge_upper_knuckle",
        )

    strike_back_x = body_width / 2.0 - shell_thickness - 0.0015
    strike_ear_x = body_width / 2.0 - shell_thickness - 0.006
    for label, strike_z in zip(("top", "middle", "bottom"), strike_zs):
        body.visual(
            Box((0.003, 0.034, 0.055)),
            origin=Origin(xyz=(strike_back_x, -0.026, strike_z)),
            material=galvanized,
            name=f"{label}_strike_back",
        )
        body.visual(
            Box((0.012, 0.018, 0.012)),
            origin=Origin(xyz=(strike_ear_x, -0.032, strike_z + 0.018)),
            material=galvanized,
            name=f"{label}_strike_upper",
        )
        body.visual(
            Box((0.012, 0.018, 0.012)),
            origin=Origin(xyz=(strike_ear_x, -0.032, strike_z - 0.018)),
            material=galvanized,
            name=f"{label}_strike_lower",
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.060, door_height)),
        mass=8.5,
        origin=Origin(xyz=(door_width / 2.0, -0.010, door_height / 2.0)),
    )
    door.visual(
        Box((door_width, door_skin_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0, door_skin_center_y, door_height / 2.0)),
        material=enclosure_steel,
        name="door_skin",
    )
    door.visual(
        Box((door_return_width, door_return_depth, door_height)),
        origin=Origin(xyz=(door_return_width / 2.0, door_return_center_y, door_height / 2.0)),
        material=enclosure_steel,
        name="left_flange",
    )
    for name, center_z, height in (
        ("right_flange_lower", 0.057, 0.106),
        ("right_flange_lower_mid", 0.323, 0.150),
        ("right_flange_upper_mid", 0.586, 0.130),
        ("right_flange_upper", 0.842, 0.100),
    ):
        door.visual(
            Box((door_return_width, door_return_depth, height)),
            origin=Origin(xyz=(door_width - door_return_width / 2.0, door_return_center_y, center_z)),
            material=enclosure_steel,
            name=name,
        )
    door.visual(
        Box((door_width - 2.0 * door_return_width, door_return_depth, door_return_width)),
        origin=Origin(
            xyz=(door_width / 2.0, door_return_center_y, door_height - door_return_width / 2.0)
        ),
        material=enclosure_steel,
        name="top_flange",
    )
    door.visual(
        Box((door_width - 2.0 * door_return_width, door_return_depth, door_return_width)),
        origin=Origin(xyz=(door_width / 2.0, door_return_center_y, door_return_width / 2.0)),
        material=enclosure_steel,
        name="bottom_flange",
    )

    for label, hinge_z in (("lower", hinge_lower_z - shell_thickness), ("upper", hinge_upper_z - shell_thickness)):
        door.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_segment_length),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=galvanized,
            name=f"{label}_hinge_barrel",
        )
        door.visual(
            Box((0.036, 0.032, 0.12)),
            origin=Origin(xyz=(0.026, 0.002, hinge_z)),
            material=galvanized,
            name=f"{label}_hinge_mount",
        )
        door.visual(
            Box((0.024, 0.006, 0.12)),
            origin=Origin(xyz=(0.030, 0.021, hinge_z)),
            material=galvanized,
            name=f"{label}_hinge_door_leaf",
        )

    for index, guide_z in enumerate((0.14, 0.35, 0.56), start=1):
        door.visual(
            Box((0.020, 0.004, 0.050)),
            origin=Origin(xyz=(rod_origin_x, guide_back_center_y, guide_z)),
            material=zinc,
            name=f"guide_{index}_back",
        )
        door.visual(
            Box((guide_side_width, 0.016, 0.050)),
            origin=Origin(
                xyz=(rod_origin_x - rod_radius - guide_side_width / 2.0, rod_origin_y, guide_z)
            ),
            material=zinc,
            name=f"guide_{index}_left",
        )
        door.visual(
            Box((guide_side_width, 0.016, 0.050)),
            origin=Origin(
                xyz=(rod_origin_x + rod_radius + guide_side_width / 2.0, rod_origin_y, guide_z)
            ),
            material=zinc,
            name=f"guide_{index}_right",
        )

    lock_rod = model.part("lock_rod")
    lock_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=rod_radius, length=rod_length),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, rod_length / 2.0)),
    )
    lock_rod.visual(
        Cylinder(radius=rod_radius, length=rod_length),
        origin=Origin(xyz=(0.0, 0.0, rod_length / 2.0)),
        material=zinc,
        name="rod_shaft",
    )
    for label, dog_z in zip(("top", "middle", "bottom"), dog_local_zs):
        lock_rod.visual(
            Box((dog_length, dog_depth, dog_height)),
            origin=Origin(xyz=(dog_offset_x, 0.0, dog_z)),
            material=zinc,
            name=f"{label}_dog",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, shell_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "door_to_lock_rod",
        ArticulationType.PRISMATIC,
        parent=door,
        child=lock_rod,
        origin=Origin(xyz=(rod_origin_x, rod_origin_y, rod_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.15,
            lower=-rod_travel,
            upper=rod_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lock_rod = object_model.get_part("lock_rod")
    door_hinge = object_model.get_articulation("body_to_door")
    rod_slide = object_model.get_articulation("door_to_lock_rod")

    door_skin = door.get_visual("door_skin")
    lower_hinge_body = body.get_visual("lower_hinge_upper_knuckle")
    upper_hinge_body = body.get_visual("upper_hinge_lower_knuckle")
    lower_hinge_barrel = door.get_visual("lower_hinge_barrel")
    upper_hinge_barrel = door.get_visual("upper_hinge_barrel")
    guide_1_left = door.get_visual("guide_1_left")
    guide_3_right = door.get_visual("guide_3_right")
    rod_shaft = lock_rod.get_visual("rod_shaft")
    top_dog = lock_rod.get_visual("top_dog")
    middle_dog = lock_rod.get_visual("middle_dog")
    bottom_dog = lock_rod.get_visual("bottom_dog")
    top_strike_back = body.get_visual("top_strike_back")
    middle_strike_back = body.get_visual("middle_strike_back")
    bottom_strike_back = body.get_visual("bottom_strike_back")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "door_hinge_axis_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "lock_rod_axis_vertical",
        tuple(rod_slide.axis) == (0.0, 0.0, 1.0),
        details=f"lock rod axis was {rod_slide.axis}",
    )
    ctx.check(
        "door_hinge_range_realistic",
        door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper is not None
        and 1.7 <= door_hinge.motion_limits.upper <= 2.1,
        details="door hinge should swing from closed to roughly 110 degrees",
    )

    with ctx.pose({door_hinge: 0.0, rod_slide: 0.0}):
        ctx.expect_contact(
            body,
            door,
            elem_a=lower_hinge_body,
            elem_b=lower_hinge_barrel,
            name="lower_hinge_contact_closed",
        )
        ctx.expect_contact(
            body,
            door,
            elem_a=upper_hinge_body,
            elem_b=upper_hinge_barrel,
            name="upper_hinge_contact_closed",
        )
        door_skin_aabb = ctx.part_element_world_aabb(door, elem=door_skin)
        door_skin_flush = (
            door_skin_aabb is not None
            and abs(door_skin_aabb[1][1]) <= 1e-6
            and -0.0045 <= door_skin_aabb[0][1] <= -0.0035
        )
        ctx.check(
            "door_skin_flush_to_front_plane",
            door_skin_flush,
            details=f"door skin aabb was {door_skin_aabb}",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a=door_skin,
            min_overlap=0.60,
            name="door_covers_front_opening",
        )
        for dog, strike_back, label in (
            (top_dog, top_strike_back, "top"),
            (middle_dog, middle_strike_back, "middle"),
            (bottom_dog, bottom_strike_back, "bottom"),
        ):
            ctx.expect_overlap(
                lock_rod,
                body,
                axes="yz",
                elem_a=dog,
                elem_b=strike_back,
                min_overlap=0.006,
                name=f"{label}_dog_aligned_with_strike",
            )
            ctx.expect_gap(
                body,
                lock_rod,
                axis="x",
                positive_elem=strike_back,
                negative_elem=dog,
                max_gap=0.006,
                max_penetration=0.0,
                name=f"{label}_dog_engages_strike_depth",
            )

    rod_limits = rod_slide.motion_limits
    if rod_limits is not None and rod_limits.lower is not None and rod_limits.upper is not None:
        with ctx.pose({rod_slide: rod_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lock_rod_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="lock_rod_lower_no_floating")
            ctx.expect_contact(
                door,
                lock_rod,
                elem_a=guide_1_left,
                elem_b=rod_shaft,
                name="lock_rod_lower_guide_1_contact",
            )
            ctx.expect_contact(
                door,
                lock_rod,
                elem_a=guide_3_right,
                elem_b=rod_shaft,
                name="lock_rod_lower_guide_3_contact",
            )
        with ctx.pose({rod_slide: rod_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lock_rod_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="lock_rod_upper_no_floating")
            ctx.expect_contact(
                door,
                lock_rod,
                elem_a=guide_1_left,
                elem_b=rod_shaft,
                name="lock_rod_upper_guide_1_contact",
            )
            ctx.expect_contact(
                door,
                lock_rod,
                elem_a=guide_3_right,
                elem_b=rod_shaft,
                name="lock_rod_upper_guide_3_contact",
            )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower, rod_slide: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_overlap")
            ctx.fail_if_isolated_parts(name="door_closed_no_floating")
        with ctx.pose({door_hinge: door_limits.upper, rod_slide: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_no_floating")
            ctx.expect_contact(
                body,
                door,
                elem_a=lower_hinge_body,
                elem_b=lower_hinge_barrel,
                name="lower_hinge_contact_open",
            )
            ctx.expect_contact(
                body,
                door,
                elem_a=upper_hinge_body,
                elem_b=upper_hinge_barrel,
                name="upper_hinge_contact_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
