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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    def add_box(part, name, size, xyz, material):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    def add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    model = ArticulatedObject(name="mid_tower_atx_case")

    steel = model.material("steel", rgba=(0.23, 0.24, 0.26, 1.0))
    powder_black = model.material("powder_black", rgba=(0.09, 0.10, 0.11, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.13, 0.14, 0.15, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.58, 0.66, 0.74, 0.30))
    hinge_metal = model.material("hinge_metal", rgba=(0.50, 0.53, 0.57, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    case_w = 0.230
    case_d = 0.470
    case_h = 0.480
    half_w = case_w * 0.5
    half_d = case_d * 0.5
    base_z = 0.015
    mid_z = base_z + case_h * 0.5
    top_contact_z = 0.479

    chassis = model.part("chassis")

    foot_specs = (
        ("front_left_foot", (-0.072, -0.170)),
        ("front_right_foot", (0.072, -0.170)),
        ("rear_left_foot", (-0.072, 0.170)),
        ("rear_right_foot", (0.072, 0.170)),
    )
    for name, (x_pos, y_pos) in foot_specs:
        add_box(chassis, name, (0.042, 0.034, 0.016), (x_pos, y_pos, 0.008), rubber)

    add_box(chassis, "bottom_tray", (0.218, 0.452, 0.014), (0.0, 0.0, 0.022), steel)
    add_box(chassis, "right_side_wall", (0.012, 0.450, 0.456), (0.109, 0.0, 0.243), steel)
    add_box(chassis, "left_bottom_rail", (0.012, 0.432, 0.024), (-0.109, -0.004, 0.027), steel)
    add_box(chassis, "left_top_rail", (0.012, 0.432, 0.016), (-0.109, -0.004, 0.471), steel)
    add_box(chassis, "left_front_post", (0.012, 0.020, 0.432), (-0.109, -0.225, 0.255), steel)
    add_box(chassis, "left_rear_post", (0.012, 0.024, 0.432), (-0.109, 0.223, 0.255), steel)
    add_box(chassis, "front_right_post", (0.018, 0.018, 0.432), (0.106, -0.226, 0.255), steel)
    add_box(chassis, "front_top_beam", (0.218, 0.018, 0.024), (0.0, -0.226, 0.467), steel)
    add_box(chassis, "front_bottom_beam", (0.218, 0.018, 0.032), (0.0, -0.226, 0.031), steel)
    add_box(chassis, "rear_top_beam", (0.218, 0.024, 0.024), (0.0, 0.223, 0.467), steel)
    add_box(chassis, "rear_bottom_beam", (0.218, 0.024, 0.060), (0.0, 0.223, 0.045), steel)
    add_box(chassis, "right_top_rail", (0.018, 0.432, 0.016), (0.106, -0.004, 0.471), steel)

    add_cylinder(
        chassis,
        "glass_hinge_pin_upper",
        0.0035,
        0.022,
        (-0.1115, 0.2315, mid_z + 0.155),
        hinge_metal,
    )
    add_cylinder(
        chassis,
        "glass_hinge_pin_lower",
        0.0035,
        0.022,
        (-0.1115, 0.2315, mid_z - 0.155),
        hinge_metal,
    )
    add_cylinder(
        chassis,
        "top_hinge_pin_left",
        0.004,
        0.024,
        (-0.074, -0.217, 0.475),
        hinge_metal,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    add_cylinder(
        chassis,
        "top_hinge_pin_right",
        0.004,
        0.024,
        (0.074, -0.217, 0.475),
        hinge_metal,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    side_panel = model.part("tempered_glass_panel")
    side_panel_depth = 0.450
    side_panel_height = 0.442

    add_box(
        side_panel,
        "glass_lite",
        (0.004, side_panel_depth - 0.028, side_panel_height - 0.028),
        (-0.010, -0.225, 0.0),
        glass_tint,
    )
    add_box(
        side_panel,
        "rear_rail",
        (0.016, 0.014, side_panel_height),
        (-0.008, -0.007, 0.0),
        powder_black,
    )
    add_box(
        side_panel,
        "front_rail",
        (0.016, 0.014, side_panel_height),
        (-0.008, -(side_panel_depth - 0.007), 0.0),
        powder_black,
    )
    add_box(
        side_panel,
        "top_rail",
        (0.016, side_panel_depth, 0.014),
        (-0.008, -side_panel_depth * 0.5, side_panel_height * 0.5 - 0.007),
        powder_black,
    )
    add_box(
        side_panel,
        "bottom_rail",
        (0.016, side_panel_depth, 0.014),
        (-0.008, -side_panel_depth * 0.5, -side_panel_height * 0.5 + 0.007),
        powder_black,
    )
    add_cylinder(
        side_panel,
        "upper_hinge_knuckle",
        0.005,
        0.020,
        (-0.011, -0.005, 0.155),
        powder_black,
    )
    add_cylinder(
        side_panel,
        "lower_hinge_knuckle",
        0.005,
        0.020,
        (-0.011, -0.005, -0.155),
        powder_black,
    )

    front_door = model.part("front_mesh_door")
    door_w = 0.212
    door_h = 0.452
    door_t = 0.018

    add_box(front_door, "hinge_stile", (0.014, door_t, door_h), (-0.007, 0.0, 0.0), powder_black)
    add_box(
        front_door,
        "free_edge_stile",
        (0.014, door_t, door_h),
        (-(door_w - 0.007), 0.0, 0.0),
        powder_black,
    )
    add_box(
        front_door,
        "top_rail",
        (door_w, door_t, 0.018),
        (-door_w * 0.5, 0.0, door_h * 0.5 - 0.009),
        powder_black,
    )
    add_box(
        front_door,
        "bottom_rail",
        (door_w, door_t, 0.018),
        (-door_w * 0.5, 0.0, -door_h * 0.5 + 0.009),
        powder_black,
    )
    add_cylinder(front_door, "hinge_spine", 0.004, door_h * 0.88, (-0.004, 0.0, 0.0), hinge_metal)
    slat_height = door_h - 0.032
    slat_start = 0.030
    slat_step = 0.022
    for index in range(8):
        add_box(
            front_door,
            f"mesh_slat_{index}",
            (0.005, 0.004, slat_height),
            (-(slat_start + index * slat_step), 0.0, 0.0),
            mesh_black,
        )
    add_box(front_door, "door_handle", (0.010, 0.010, 0.150), (-0.200, -0.013, 0.020), hinge_metal)

    top_panel = model.part("top_vent_panel")
    top_w = 0.208
    top_d = 0.388
    top_t = 0.016

    add_box(
        top_panel,
        "front_rail",
        (top_w, 0.016, top_t),
        (0.0, 0.008, 0.0),
        powder_black,
    )
    add_box(
        top_panel,
        "rear_rail",
        (top_w, 0.018, top_t),
        (0.0, top_d - 0.009, 0.0),
        powder_black,
    )
    add_box(
        top_panel,
        "left_rail",
        (0.018, top_d, top_t),
        (-top_w * 0.5 + 0.009, top_d * 0.5, 0.0),
        powder_black,
    )
    add_box(
        top_panel,
        "right_rail",
        (0.018, top_d, top_t),
        (top_w * 0.5 - 0.009, top_d * 0.5, 0.0),
        powder_black,
    )
    for index in range(7):
        add_box(
            top_panel,
            f"vent_slat_{index}",
            (top_w - 0.034, 0.008, 0.008),
            (0.0, 0.046 + index * 0.046, 0.0),
            mesh_black,
        )
    add_cylinder(
        top_panel,
        "left_hinge_knuckle",
        0.005,
        0.026,
        (-0.074, 0.005, 0.0),
        hinge_metal,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    add_cylinder(
        top_panel,
        "right_hinge_knuckle",
        0.005,
        0.026,
        (0.074, 0.005, 0.0),
        hinge_metal,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    model.articulation(
        "chassis_to_tempered_glass_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(-half_w, half_d, mid_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "chassis_to_front_mesh_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(0.097, -half_d - door_t * 0.5, mid_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "chassis_to_top_vent_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(0.0, -0.217, top_contact_z + top_t * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.3,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("tempered_glass_panel")
    front_door = object_model.get_part("front_mesh_door")
    top_panel = object_model.get_part("top_vent_panel")
    side_hinge = object_model.get_articulation("chassis_to_tempered_glass_panel")
    door_hinge = object_model.get_articulation("chassis_to_front_mesh_door")
    top_hinge = object_model.get_articulation("chassis_to_top_vent_panel")

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

    ctx.expect_gap(
        chassis,
        side_panel,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-6,
        name="glass panel closes against the left-side frame",
    )
    ctx.expect_overlap(
        side_panel,
        chassis,
        axes="yz",
        min_overlap=0.35,
        name="glass panel covers the left-side opening",
    )
    ctx.expect_gap(
        chassis,
        front_door,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-6,
        name="front mesh door closes onto the front frame",
    )
    ctx.expect_overlap(
        front_door,
        chassis,
        axes="xz",
        min_overlap=0.18,
        name="front mesh door spans the front intake area",
    )
    ctx.expect_gap(
        top_panel,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        name="top vent panel seats on the roof opening",
    )
    ctx.expect_overlap(
        top_panel,
        chassis,
        axes="xy",
        min_overlap=0.18,
        name="top vent panel covers the roof vent opening",
    )

    with ctx.pose({side_hinge: 0.0}):
        side_closed = aabb_center(ctx.part_element_world_aabb(side_panel, elem="front_rail"))
    with ctx.pose({side_hinge: side_hinge.motion_limits.upper}):
        side_open = aabb_center(ctx.part_element_world_aabb(side_panel, elem="front_rail"))
    ctx.check(
        "tempered glass panel opens outward from the rear hinge pins",
        side_closed is not None
        and side_open is not None
        and side_open[0] < side_closed[0] - 0.08,
        details=f"closed={side_closed}, open={side_open}",
    )

    with ctx.pose({door_hinge: 0.0}):
        door_closed = aabb_center(ctx.part_element_world_aabb(front_door, elem="free_edge_stile"))
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        door_open = aabb_center(ctx.part_element_world_aabb(front_door, elem="free_edge_stile"))
    ctx.check(
        "front mesh door swings outward from the right edge",
        door_closed is not None
        and door_open is not None
        and door_open[1] < door_closed[1] - 0.08,
        details=f"closed={door_closed}, open={door_open}",
    )

    with ctx.pose({top_hinge: 0.0}):
        top_closed = aabb_center(ctx.part_element_world_aabb(top_panel, elem="rear_rail"))
    with ctx.pose({top_hinge: top_hinge.motion_limits.upper}):
        top_open = aabb_center(ctx.part_element_world_aabb(top_panel, elem="rear_rail"))
    ctx.check(
        "top ventilation panel lifts from the rear edge on front hinges",
        top_closed is not None
        and top_open is not None
        and top_open[2] > top_closed[2] + 0.06,
        details=f"closed={top_closed}, open={top_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
