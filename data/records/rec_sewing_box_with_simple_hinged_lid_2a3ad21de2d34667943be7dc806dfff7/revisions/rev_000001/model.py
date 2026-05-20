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
    model = ArticulatedObject(name="industrial_safety_sewing_box")

    structural_steel = model.material("structural_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    guard_yellow = model.material("guard_yellow", rgba=(0.88, 0.73, 0.16, 1.0))
    hardware = model.material("hardware", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    width = 0.40
    depth = 0.26
    body_height = 0.18
    plate = 0.004
    lid_plate = 0.003
    lid_drop = 0.040
    lid_clearance = 0.002
    hinge_radius = 0.008
    hinge_y = -(depth / 2.0) - 0.007
    hinge_z = body_height + 0.008
    open_angle = math.radians(74.0)

    lid_outer_width = width + 2.0 * (lid_clearance + lid_plate)
    lid_outer_depth = depth + 0.006
    lid_rear_edge = 0.005
    lid_top_center_y = lid_rear_edge + lid_outer_depth / 2.0
    lid_top_center_z = -0.0065
    flange_center_z = lid_top_center_z - (lid_plate / 2.0) - (lid_drop / 2.0)

    body = model.part("body")
    body.visual(
        Box((width, depth, plate)),
        origin=Origin(xyz=(0.0, 0.0, plate / 2.0)),
        material=structural_steel,
        name="bottom_plate",
    )
    body.visual(
        Box((width, plate, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - plate / 2.0, body_height / 2.0)),
        material=structural_steel,
        name="front_wall",
    )
    body.visual(
        Box((width, plate, body_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + plate / 2.0, body_height / 2.0)),
        material=structural_steel,
        name="back_wall",
    )
    body.visual(
        Box((plate, depth - 2.0 * plate, body_height)),
        origin=Origin(xyz=(-width / 2.0 + plate / 2.0, 0.0, body_height / 2.0)),
        material=structural_steel,
        name="left_wall",
    )
    body.visual(
        Box((plate, depth - 2.0 * plate, body_height)),
        origin=Origin(xyz=(width / 2.0 - plate / 2.0, 0.0, body_height / 2.0)),
        material=structural_steel,
        name="right_wall",
    )

    body.visual(
        Box((width - 2.0 * plate, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.022, body_height - 0.003)),
        material=structural_steel,
        name="front_seat_rail",
    )
    body.visual(
        Box((0.016, depth - 0.052, 0.006)),
        origin=Origin(xyz=(-width / 2.0 + 0.022, 0.004, body_height - 0.003)),
        material=structural_steel,
        name="left_seat_rail",
    )
    body.visual(
        Box((0.016, depth - 0.052, 0.006)),
        origin=Origin(xyz=(width / 2.0 - 0.022, 0.004, body_height - 0.003)),
        material=structural_steel,
        name="right_seat_rail",
    )

    body.visual(
        Box((0.34, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.004, body_height - 0.042)),
        material=guard_yellow,
        name="rear_reinforcement_strap",
    )
    body.visual(
        Box((0.34, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, hinge_y + 0.010, hinge_z - 0.010)),
        material=guard_yellow,
        name="body_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.054),
        origin=Origin(xyz=(-0.120, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="left_body_knuckle",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.054),
        origin=Origin(xyz=(0.0, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="center_body_knuckle",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.054),
        origin=Origin(xyz=(0.120, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="right_body_knuckle",
    )

    body.visual(
        Box((0.020, 0.050, 0.084)),
        origin=Origin(xyz=(-0.188, hinge_y - 0.011, hinge_z + 0.004)),
        material=guard_yellow,
        name="left_guard_tower",
    )
    body.visual(
        Box((0.020, 0.050, 0.084)),
        origin=Origin(xyz=(0.188, hinge_y - 0.011, hinge_z + 0.004)),
        material=guard_yellow,
        name="right_guard_tower",
    )
    body.visual(
        Box((0.376, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, hinge_y - 0.021, hinge_z + 0.004)),
        material=guard_yellow,
        name="rear_guard_shroud",
    )
    body.visual(
        Box((0.024, 0.018, 0.016)),
        origin=Origin(xyz=(-0.166, hinge_y - 0.017, hinge_z + 0.029)),
        material=guard_yellow,
        name="left_stop_canopy",
    )
    body.visual(
        Box((0.024, 0.018, 0.016)),
        origin=Origin(xyz=(0.166, hinge_y - 0.017, hinge_z + 0.029)),
        material=guard_yellow,
        name="right_stop_canopy",
    )
    body.visual(
        Box((0.028, 0.012, 0.074)),
        origin=Origin(xyz=(-0.188, -depth / 2.0 + 0.008, hinge_z - 0.025)),
        material=guard_yellow,
        name="left_hinge_doubler",
    )
    body.visual(
        Box((0.028, 0.012, 0.074)),
        origin=Origin(xyz=(0.188, -depth / 2.0 + 0.008, hinge_z - 0.025)),
        material=guard_yellow,
        name="right_hinge_doubler",
    )

    body.visual(
        Box((0.014, 0.012, 0.056)),
        origin=Origin(xyz=(-0.028, depth / 2.0 + 0.006, body_height - 0.024)),
        material=guard_yellow,
        name="left_lock_guard_cheek",
    )
    body.visual(
        Box((0.014, 0.012, 0.056)),
        origin=Origin(xyz=(0.028, depth / 2.0 + 0.006, body_height - 0.024)),
        material=guard_yellow,
        name="right_lock_guard_cheek",
    )
    body.visual(
        Box((0.070, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.006, body_height + 0.005)),
        material=guard_yellow,
        name="lock_guard_bridge",
    )

    strap_bolt_positions = (
        (-0.135, body_height - 0.032),
        (-0.045, body_height - 0.032),
        (0.045, body_height - 0.032),
        (0.135, body_height - 0.032),
        (-0.090, body_height - 0.048),
        (0.090, body_height - 0.048),
    )
    for index, (x_pos, z_pos) in enumerate(strap_bolt_positions, start=1):
        body.visual(
            Cylinder(radius=0.0046, length=0.004),
            origin=Origin(
                xyz=(x_pos, -depth / 2.0 - 0.008, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"strap_bolt_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((width + 0.05, depth + 0.06, body_height + 0.06)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, (body_height + 0.06) / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_outer_width, lid_outer_depth, lid_plate)),
        origin=Origin(xyz=(0.0, lid_top_center_y, lid_top_center_z)),
        material=structural_steel,
        name="lid_top_plate",
    )
    lid.visual(
        Box((lid_plate, lid_outer_depth - 0.020, lid_drop)),
        origin=Origin(
            xyz=(
                -lid_outer_width / 2.0 + lid_plate / 2.0,
                lid_top_center_y,
                flange_center_z,
            )
        ),
        material=structural_steel,
        name="left_side_flange",
    )
    lid.visual(
        Box((lid_plate, lid_outer_depth - 0.020, lid_drop)),
        origin=Origin(
            xyz=(
                lid_outer_width / 2.0 - lid_plate / 2.0,
                lid_top_center_y,
                flange_center_z,
            )
        ),
        material=structural_steel,
        name="right_side_flange",
    )
    lid.visual(
        Box((lid_outer_width, lid_plate, lid_drop)),
        origin=Origin(
            xyz=(0.0, lid_rear_edge + lid_outer_depth - lid_plate / 2.0, flange_center_z)
        ),
        material=structural_steel,
        name="front_flange",
    )
    lid.visual(
        Box((0.34, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.016, -0.010)),
        material=guard_yellow,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.063),
        origin=Origin(xyz=(-0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="left_lid_knuckle",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.063),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="right_lid_knuckle",
    )

    lid.visual(
        Box((0.008, 0.020, 0.042)),
        origin=Origin(
            xyz=(-0.162, 0.012, -0.002),
        ),
        material=guard_yellow,
        name="left_rear_guard",
    )
    lid.visual(
        Box((0.008, 0.020, 0.042)),
        origin=Origin(
            xyz=(0.162, 0.012, -0.002),
        ),
        material=guard_yellow,
        name="right_rear_guard",
    )
    lid.visual(
        Box((0.014, 0.010, 0.018)),
        origin=Origin(xyz=(-0.166, 0.010, 0.028)),
        material=guard_yellow,
        name="left_stop_tab",
    )
    lid.visual(
        Box((0.014, 0.010, 0.018)),
        origin=Origin(xyz=(0.166, 0.010, 0.028)),
        material=guard_yellow,
        name="right_stop_tab",
    )

    lid.visual(
        Box((0.090, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, lid_rear_edge + lid_outer_depth - 0.007, -0.014)),
        material=guard_yellow,
        name="front_reinforcement_rib",
    )
    lid.visual(
        Box((0.040, 0.003, 0.050)),
        origin=Origin(xyz=(0.0, lid_rear_edge + lid_outer_depth - 0.0015, -0.033)),
        material=guard_yellow,
        name="hasp_plate",
    )
    lid.visual(
        Box((0.006, 0.003, 0.018)),
        origin=Origin(xyz=(-0.011, lid_rear_edge + lid_outer_depth - 0.0015, -0.046)),
        material=guard_yellow,
        name="left_hasp_lug",
    )
    lid.visual(
        Box((0.006, 0.003, 0.018)),
        origin=Origin(xyz=(0.011, lid_rear_edge + lid_outer_depth - 0.0015, -0.046)),
        material=guard_yellow,
        name="right_hasp_lug",
    )
    lid.visual(
        Box((0.010, 0.005, 0.026)),
        origin=Origin(xyz=(0.0, lid_rear_edge + lid_outer_depth + 0.0010, -0.043)),
        material=guard_yellow,
        name="lock_tongue",
    )

    for index, x_pos in enumerate((-0.125, -0.040, 0.040, 0.125), start=1):
        lid.visual(
            Cylinder(radius=0.0042, length=0.004),
            origin=Origin(
                xyz=(x_pos, 0.020, -0.009),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"lid_leaf_bolt_{index}",
        )

    for index, x_pos in enumerate((-0.012, 0.012), start=1):
        lid.visual(
            Cylinder(radius=0.0038, length=0.004),
            origin=Origin(
                xyz=(x_pos, lid_rear_edge + lid_outer_depth - 0.0015, -0.032),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"hasp_bolt_{index}",
        )

    lid.visual(
        Box((0.120, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, lid_top_center_y, lid_top_center_z - 0.0065)),
        material=dark_rubber,
        name="anti_rattle_pad",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_outer_width, lid_outer_depth, lid_drop + 0.05)),
        mass=2.2,
        origin=Origin(xyz=(0.0, lid_top_center_y, -0.010)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=0.0,
            upper=open_angle,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")
    limits = hinge.motion_limits

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
        "lid hinge uses rear cross-axis with bounded travel",
        hinge.axis == (1.0, 0.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and math.radians(65.0) <= limits.upper <= math.radians(80.0),
        details=(
            f"axis={hinge.axis}, "
            f"limits={None if limits is None else (limits.lower, limits.upper)}"
        ),
    )

    closed_front = None
    open_front = None
    open_limit = math.radians(74.0) if limits is None or limits.upper is None else limits.upper

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="lid_top_plate",
            elem_b="front_seat_rail",
            contact_tol=0.0015,
            name="lid seats on reinforced front rail",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            name="closed lid covers body footprint",
        )
        closed_front = ctx.part_element_world_aabb(lid, elem="front_flange")
        hasp_plate_aabb = ctx.part_element_world_aabb(lid, elem="hasp_plate")
        left_cheek_aabb = ctx.part_element_world_aabb(body, elem="left_lock_guard_cheek")
        right_cheek_aabb = ctx.part_element_world_aabb(body, elem="right_lock_guard_cheek")
        bridge_aabb = ctx.part_element_world_aabb(body, elem="lock_guard_bridge")
        if (
            hasp_plate_aabb is None
            or left_cheek_aabb is None
            or right_cheek_aabb is None
            or bridge_aabb is None
        ):
            ctx.fail("guarded lockout geometry measurable", "missing front lockout AABB")
        else:
            plate_min, plate_max = hasp_plate_aabb
            left_min, left_max = left_cheek_aabb
            right_min, right_max = right_cheek_aabb
            bridge_min, bridge_max = bridge_aabb
            guarded = (
                left_max[0] <= plate_min[0] + 0.0015
                and right_min[0] >= plate_max[0] - 0.0015
                and bridge_min[2] >= plate_max[2]
                and bridge_min[2] <= plate_max[2] + 0.012
                and left_min[1] <= plate_min[1]
                and right_min[1] <= plate_min[1]
            )
            ctx.check(
                "hasp plate sits inside guarded lockout cage",
                guarded,
                details=(
                    f"plate={hasp_plate_aabb}, left={left_cheek_aabb}, "
                    f"right={right_cheek_aabb}, bridge={bridge_aabb}"
                ),
            )

    with ctx.pose({hinge: open_limit}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_stop_tab",
            elem_b="left_stop_canopy",
            contact_tol=0.003,
            name="left overtravel stop engages",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="right_stop_tab",
            elem_b="right_stop_canopy",
            contact_tol=0.003,
            name="right overtravel stop engages",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.12,
            positive_elem="front_flange",
            negative_elem="front_wall",
            name="front edge lifts clear when open",
        )
        open_front = ctx.part_element_world_aabb(lid, elem="front_flange")

    if closed_front is None or open_front is None:
        ctx.fail("front flange pose measurement available", "front flange AABB could not be measured")
    else:
        rise = open_front[1][2] - closed_front[1][2]
        ctx.check(
            "lid front edge rises substantially",
            rise > 0.20,
            details=f"front flange top rise={rise:.4f} m",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
