from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _deck_top_z(x_pos: float, *, run_length: float, low_top_z: float, high_top_z: float) -> float:
    if x_pos <= 0.0:
        return low_top_z
    if x_pos >= run_length:
        return high_top_z
    return low_top_z + ((high_top_z - low_top_z) * (x_pos / run_length))


def _deck_section(x_pos: float, width: float, bottom_z: float, top_z: float) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (x_pos, -half_width, bottom_z),
        (x_pos, -half_width, top_z),
        (x_pos, half_width, top_z),
        (x_pos, half_width, bottom_z),
    ]


def _segment_origin(point_a, point_b) -> tuple[Origin, float]:
    ax, ay, az = point_a
    bx, by, bz = point_b
    dx = bx - ax
    dy = by - ay
    dz = bz - az
    length = sqrt((dx * dx) + (dy * dy) + (dz * dz))
    mid = ((ax + bx) * 0.5, (ay + by) * 0.5, (az + bz) * 0.5)
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt((dx * dx) + (dy * dy)), dz)
    return Origin(xyz=mid, rpy=(0.0, pitch, yaw)), length


def _add_tube(part, point_a, point_b, *, radius: float, material, name: str | None = None) -> None:
    origin, length = _segment_origin(point_a, point_b)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_leg_pair_geometry(
    part,
    *,
    foot_x: float,
    foot_y: float,
    foot_top_z: float,
    frame_material,
    pin_material,
) -> None:
    part.visual(
        Box((0.16, 1.02, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=frame_material,
        name="mount_plate",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, -0.028), rpy=(pi * 0.5, 0.0, 0.0)),
        material=pin_material,
        name="hinge_barrel",
    )
    for sign, side_name in ((1.0, "left"), (-1.0, "right")):
        upper_node = (0.02, sign * 0.34, -0.028)
        lower_node = (foot_x, sign * foot_y, foot_top_z + 0.01)
        brace_node = (foot_x * 0.72, sign * (foot_y - 0.10), (foot_top_z * 0.52))
        _add_tube(part, upper_node, lower_node, radius=0.029, material=frame_material, name=f"{side_name}_leg")
        _add_tube(part, (-0.01, sign * 0.23, -0.028), brace_node, radius=0.021, material=frame_material)
        part.visual(
            Box((0.19, 0.12, 0.03)),
            origin=Origin(xyz=(foot_x + 0.03, sign * foot_y, foot_top_z - 0.015)),
            material=frame_material,
            name=f"{side_name}_foot_pad",
        )
    _add_tube(
        part,
        (foot_x * 0.68, foot_y - 0.03, foot_top_z + 0.18),
        (foot_x * 0.68, -foot_y + 0.03, foot_top_z + 0.18),
        radius=0.022,
        material=frame_material,
        name="lower_tie_bar",
    )


def _add_guard_rail_geometry(part, *, side_sign: float, rail_material) -> None:
    side_offset = side_sign * 0.04
    post_offset = side_sign * 0.04
    part.visual(
        Box((0.66, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, side_offset, 0.08)),
        material=rail_material,
        name="base_hinge_leaf",
    )
    _add_tube(part, (-0.27, post_offset, 0.05), (-0.27, post_offset, 0.96), radius=0.022, material=rail_material)
    _add_tube(part, (0.27, post_offset, 0.05), (0.27, post_offset, 0.96), radius=0.022, material=rail_material)
    _add_tube(part, (0.0, post_offset, 0.05), (0.0, post_offset, 0.54), radius=0.018, material=rail_material)
    _add_tube(
        part,
        (-0.29, post_offset, 0.96),
        (0.29, post_offset, 0.96),
        radius=0.022,
        material=rail_material,
        name="top_rail",
    )
    _add_tube(
        part,
        (-0.29, post_offset, 0.52),
        (0.29, post_offset, 0.52),
        radius=0.017,
        material=rail_material,
        name="mid_rail",
    )
    _add_tube(part, (-0.25, post_offset, 0.08), (-0.03, post_offset, 0.43), radius=0.016, material=rail_material)
    _add_tube(part, (0.25, post_offset, 0.08), (0.03, post_offset, 0.43), radius=0.016, material=rail_material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modular_yard_ramp")

    deck_steel = model.material("deck_steel", rgba=(0.65, 0.68, 0.70, 1.0))
    frame_blue = model.material("frame_blue", rgba=(0.18, 0.31, 0.53, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.88, 0.73, 0.15, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))

    deck_width = 1.80
    clear_width = 1.58
    curb_thickness = 0.11
    run_length = 3.05
    platform_length = 0.70
    total_length = run_length + platform_length
    low_top_z = 0.14
    high_top_z = 0.90
    deck_depth = 0.09
    deck_angle = atan2(high_top_z - low_top_z, run_length)
    rail_hinge_x = run_length + (platform_length * 0.52)
    rail_hinge_z = high_top_z + 0.005

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((total_length, deck_width, 1.02)),
        mass=720.0,
        origin=Origin(xyz=(total_length * 0.50, 0.0, 0.51)),
    )
    deck_shell = section_loft(
        [
            _deck_section(0.0, deck_width, low_top_z - deck_depth, low_top_z),
            _deck_section(run_length, deck_width, high_top_z - deck_depth, high_top_z),
            _deck_section(total_length, deck_width, high_top_z - deck_depth, high_top_z),
        ]
    )
    deck.visual(_save_mesh("yard_ramp_deck_shell", deck_shell), material=deck_steel, name="deck_shell")
    deck.visual(
        Box((0.32, clear_width, 0.012)),
        origin=Origin(xyz=(0.02, 0.0, 0.082), rpy=(0.0, -atan2(low_top_z - 0.022, 0.32), 0.0)),
        material=deck_steel,
        name="toe_lip",
    )
    for sign in (1.0, -1.0):
        side_y = sign * ((deck_width * 0.5) - (curb_thickness * 0.5))
        deck.visual(
            Box((run_length, curb_thickness, 0.18)),
            origin=Origin(
                xyz=(run_length * 0.5, side_y, ((low_top_z + high_top_z) * 0.5) + 0.03),
                rpy=(0.0, -deck_angle, 0.0),
            ),
            material=safety_yellow,
            name=f"{'left' if sign > 0.0 else 'right'}_incline_curb",
        )
        deck.visual(
            Box((platform_length + 0.02, curb_thickness, 0.18)),
            origin=Origin(
                xyz=(run_length + (platform_length * 0.5), side_y, high_top_z + 0.03),
            ),
            material=safety_yellow,
            name=f"{'left' if sign > 0.0 else 'right'}_platform_curb",
        )
        deck.visual(
            Box((0.70, 0.03, 0.06)),
            origin=Origin(
                xyz=(rail_hinge_x, sign * ((deck_width * 0.5) - 0.015), rail_hinge_z + 0.03),
            ),
            material=safety_yellow,
            name=f"{'left' if sign > 0.0 else 'right'}_rail_hinge_plate",
        )
    for sign in (1.0, -1.0):
        stringer_y = sign * 0.66
        deck.visual(
            Box((run_length - 0.05, 0.17, 0.14)),
            origin=Origin(
                xyz=(run_length * 0.5, stringer_y, ((low_top_z + high_top_z) * 0.5) - 0.16),
                rpy=(0.0, -deck_angle, 0.0),
            ),
            material=frame_blue,
            name=f"{'left' if sign > 0.0 else 'right'}_incline_stringer",
        )
        deck.visual(
            Box((platform_length, 0.17, 0.14)),
            origin=Origin(
                xyz=(run_length + (platform_length * 0.5), stringer_y, high_top_z - 0.16),
            ),
            material=frame_blue,
            name=f"{'left' if sign > 0.0 else 'right'}_platform_stringer",
        )
    deck.visual(
        Box((0.20, 1.18, 0.16)),
        origin=Origin(xyz=(total_length - 0.08, 0.0, high_top_z - 0.15)),
        material=frame_blue,
        name="rear_crossmember",
    )
    front_leg_hinge_x = 1.18
    rear_leg_hinge_x = 2.52
    front_leg_hinge_z = 0.30
    rear_leg_hinge_z = 0.64
    front_deck_bottom_z = _deck_top_z(
        front_leg_hinge_x,
        run_length=run_length,
        low_top_z=low_top_z,
        high_top_z=high_top_z,
    ) - deck_depth
    rear_deck_bottom_z = _deck_top_z(
        rear_leg_hinge_x,
        run_length=run_length,
        low_top_z=low_top_z,
        high_top_z=high_top_z,
    ) - deck_depth
    deck.visual(
        Box((0.22, 1.00, 0.02)),
        origin=Origin(xyz=(front_leg_hinge_x, 0.0, front_deck_bottom_z + 0.01)),
        material=dark_steel,
        name="front_leg_hinge_plate",
    )
    deck.visual(
        Box((0.22, 1.00, 0.02)),
        origin=Origin(xyz=(rear_leg_hinge_x, 0.0, rear_deck_bottom_z + 0.01)),
        material=dark_steel,
        name="rear_leg_hinge_plate",
    )
    incline_rib_positions = [0.20 + (0.17 * index) for index in range(16)]
    for rib_index, x_pos in enumerate(incline_rib_positions):
        deck.visual(
            Box((0.055, clear_width, 0.009)),
            origin=Origin(
                xyz=(x_pos, 0.0, _deck_top_z(x_pos, run_length=run_length, low_top_z=low_top_z, high_top_z=high_top_z) + 0.0045),
                rpy=(0.0, -deck_angle, 0.0),
            ),
            material=deck_steel,
            name=f"incline_rib_{rib_index:02d}",
        )
    for rib_index, x_pos in enumerate((3.14, 3.28, 3.42, 3.56)):
        deck.visual(
            Box((0.055, clear_width, 0.009)),
            origin=Origin(xyz=(x_pos, 0.0, high_top_z + 0.0045)),
            material=deck_steel,
            name=f"platform_rib_{rib_index:02d}",
        )

    front_leg_pair = model.part("front_leg_pair")
    _add_leg_pair_geometry(
        front_leg_pair,
        foot_x=0.36,
        foot_y=0.50,
        foot_top_z=-0.27,
        frame_material=frame_blue,
        pin_material=dark_steel,
    )
    front_leg_pair.inertial = Inertial.from_geometry(
        Box((0.62, 1.22, 0.34)),
        mass=42.0,
        origin=Origin(xyz=(0.28, 0.0, -0.14)),
    )
    front_leg_pair.visual(
        Box((0.06, 0.92, 0.05)),
        origin=Origin(
            xyz=(0.0, 0.0, (front_deck_bottom_z - front_leg_hinge_z) - 0.025),
        ),
        material=dark_steel,
        name="upper_mount_block",
    )

    rear_leg_pair = model.part("rear_leg_pair")
    _add_leg_pair_geometry(
        rear_leg_pair,
        foot_x=0.54,
        foot_y=0.54,
        foot_top_z=-0.62,
        frame_material=frame_blue,
        pin_material=dark_steel,
    )
    rear_leg_pair.inertial = Inertial.from_geometry(
        Box((0.84, 1.28, 0.70)),
        mass=58.0,
        origin=Origin(xyz=(0.40, 0.0, -0.30)),
    )
    rear_leg_pair.visual(
        Box((0.06, 0.92, 0.05)),
        origin=Origin(
            xyz=(0.0, 0.0, (rear_deck_bottom_z - rear_leg_hinge_z) - 0.025),
        ),
        material=dark_steel,
        name="upper_mount_block",
    )

    left_guard_rail = model.part("left_guard_rail")
    _add_guard_rail_geometry(left_guard_rail, side_sign=1.0, rail_material=safety_yellow)
    left_guard_rail.inertial = Inertial.from_geometry(
        Box((0.70, 0.14, 1.00)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.055, 0.50)),
    )

    right_guard_rail = model.part("right_guard_rail")
    _add_guard_rail_geometry(right_guard_rail, side_sign=-1.0, rail_material=safety_yellow)
    right_guard_rail.inertial = Inertial.from_geometry(
        Box((0.70, 0.14, 1.00)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.055, 0.50)),
    )

    model.articulation(
        "front_leg_pair_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_leg_pair,
        origin=Origin(xyz=(front_leg_hinge_x, 0.0, front_leg_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.8, lower=-1.30, upper=0.08),
    )
    model.articulation(
        "rear_leg_pair_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=rear_leg_pair,
        origin=Origin(xyz=(rear_leg_hinge_x, 0.0, rear_leg_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2600.0, velocity=0.8, lower=-1.30, upper=0.08),
    )
    model.articulation(
        "left_guard_rail_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=left_guard_rail,
        origin=Origin(xyz=(rail_hinge_x, deck_width * 0.5, rail_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=1.2, lower=-1.45, upper=0.05),
    )
    model.articulation(
        "right_guard_rail_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=right_guard_rail,
        origin=Origin(xyz=(rail_hinge_x, -(deck_width * 0.5), rail_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=1.2, lower=-0.05, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_leg_pair = object_model.get_part("front_leg_pair")
    rear_leg_pair = object_model.get_part("rear_leg_pair")
    left_guard_rail = object_model.get_part("left_guard_rail")
    right_guard_rail = object_model.get_part("right_guard_rail")

    front_leg_joint = object_model.get_articulation("front_leg_pair_fold")
    rear_leg_joint = object_model.get_articulation("rear_leg_pair_fold")
    left_rail_joint = object_model.get_articulation("left_guard_rail_fold")
    right_rail_joint = object_model.get_articulation("right_guard_rail_fold")

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

    expected_parts = ("deck", "front_leg_pair", "rear_leg_pair", "left_guard_rail", "right_guard_rail")
    present_parts = {part.name for part in object_model.parts}
    for part_name in expected_parts:
        ctx.check(f"has_{part_name}", part_name in present_parts, f"Missing required part: {part_name}")

    ctx.expect_contact(front_leg_pair, deck, name="front_leg_pair_attached_to_deck")
    ctx.expect_contact(rear_leg_pair, deck, name="rear_leg_pair_attached_to_deck")
    ctx.expect_contact(left_guard_rail, deck, name="left_guard_rail_attached_to_deck")
    ctx.expect_contact(right_guard_rail, deck, name="right_guard_rail_attached_to_deck")

    ctx.expect_origin_gap(front_leg_pair, deck, axis="x", min_gap=1.05, max_gap=1.30, name="front_leg_pair_x_placement")
    ctx.expect_origin_gap(rear_leg_pair, deck, axis="x", min_gap=2.35, max_gap=2.70, name="rear_leg_pair_x_placement")
    ctx.expect_origin_gap(rear_leg_pair, front_leg_pair, axis="x", min_gap=1.15, max_gap=1.55, name="leg_pair_spacing")
    ctx.expect_origin_gap(left_guard_rail, deck, axis="y", min_gap=0.86, max_gap=0.94, name="left_guard_rail_edge_mount")
    ctx.expect_origin_gap(deck, right_guard_rail, axis="y", min_gap=0.86, max_gap=0.94, name="right_guard_rail_edge_mount")
    ctx.expect_origin_gap(left_guard_rail, deck, axis="x", min_gap=3.30, max_gap=3.55, name="guard_rail_rear_position")

    ctx.check(
        "front_leg_joint_axis",
        tuple(round(value, 4) for value in front_leg_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected front leg joint axis (0,1,0), got {front_leg_joint.axis}",
    )
    ctx.check(
        "rear_leg_joint_axis",
        tuple(round(value, 4) for value in rear_leg_joint.axis) == (0.0, 1.0, 0.0),
        f"Expected rear leg joint axis (0,1,0), got {rear_leg_joint.axis}",
    )
    ctx.check(
        "left_guard_joint_axis",
        tuple(round(value, 4) for value in left_rail_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected left guard rail joint axis (1,0,0), got {left_rail_joint.axis}",
    )
    ctx.check(
        "right_guard_joint_axis",
        tuple(round(value, 4) for value in right_rail_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected right guard rail joint axis (1,0,0), got {right_rail_joint.axis}",
    )

    front_foot_rest = ctx.part_element_world_aabb(front_leg_pair, elem="left_foot_pad")
    rear_foot_rest = ctx.part_element_world_aabb(rear_leg_pair, elem="left_foot_pad")
    left_top_rest = ctx.part_element_world_aabb(left_guard_rail, elem="top_rail")
    right_top_rest = ctx.part_element_world_aabb(right_guard_rail, elem="top_rail")
    assert front_foot_rest is not None
    assert rear_foot_rest is not None
    assert left_top_rest is not None
    assert right_top_rest is not None

    with ctx.pose({front_leg_joint: -1.10}):
        front_foot_folded = ctx.part_element_world_aabb(front_leg_pair, elem="left_foot_pad")
        assert front_foot_folded is not None
        ctx.check(
            "front_leg_pair_folds_up",
            front_foot_folded[0][2] > front_foot_rest[0][2] + 0.30 and front_foot_folded[0][0] > front_foot_rest[0][0] + 0.04,
            f"Front leg pair did not fold upward/rearward enough: rest={front_foot_rest}, folded={front_foot_folded}",
        )

    with ctx.pose({rear_leg_joint: -1.10}):
        rear_foot_folded = ctx.part_element_world_aabb(rear_leg_pair, elem="left_foot_pad")
        assert rear_foot_folded is not None
        ctx.check(
            "rear_leg_pair_folds_up",
            rear_foot_folded[0][2] > rear_foot_rest[0][2] + 0.40 and rear_foot_folded[0][0] > rear_foot_rest[0][0] + 0.18,
            f"Rear leg pair did not fold upward/rearward enough: rest={rear_foot_rest}, folded={rear_foot_folded}",
        )

    with ctx.pose({left_rail_joint: -1.20, right_rail_joint: 1.20}):
        left_top_folded = ctx.part_element_world_aabb(left_guard_rail, elem="top_rail")
        right_top_folded = ctx.part_element_world_aabb(right_guard_rail, elem="top_rail")
        assert left_top_folded is not None
        assert right_top_folded is not None
        ctx.check(
            "left_guard_rail_folds_outward",
            left_top_folded[1][1] > left_top_rest[1][1] + 0.35 and left_top_folded[1][2] < left_top_rest[1][2] - 0.40,
            f"Left guard rail did not swing outward/down enough: rest={left_top_rest}, folded={left_top_folded}",
        )
        ctx.check(
            "right_guard_rail_folds_outward",
            right_top_folded[0][1] < right_top_rest[0][1] - 0.35 and right_top_folded[1][2] < right_top_rest[1][2] - 0.40,
            f"Right guard rail did not swing outward/down enough: rest={right_top_rest}, folded={right_top_folded}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
