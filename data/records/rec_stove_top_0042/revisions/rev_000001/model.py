from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    place_on_surface,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_fuel_range", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.77, 1.0))
    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.16, 0.16, 0.16, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.16, 0.20, 0.24, 0.55))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.80, 0.82, 0.84, 1.0))

    width = 0.76
    depth = 0.68
    body_height = 0.91
    wall = 0.02
    front_frame_depth = 0.045
    toe_kick_height = 0.10
    door_z0 = 0.12
    door_height = 0.56
    door_width = 0.64
    door_thickness = 0.045
    control_panel_height = 0.14
    control_panel_center_z = 0.79
    cavity_floor_thickness = 0.018
    cavity_floor_center_z = 0.149
    cavity_ceiling_thickness = 0.018
    cavity_ceiling_center_z = 0.689
    top_deck_thickness = 0.035
    top_deck_center_z = body_height - top_deck_thickness / 2.0
    inner_width = width - 2.0 * wall
    cavity_depth = depth - 0.09
    stile_width = (width - door_width) / 2.0
    front_center_y = -depth / 2.0 + front_frame_depth / 2.0
    hinge_origin_y = -depth / 2.0 + front_frame_depth

    range_body = model.part("range_body")
    range_body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(-(width / 2.0 - wall / 2.0), 0.0, body_height / 2.0)),
        material=stainless,
        name="left_side",
    )
    range_body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=((width / 2.0 - wall / 2.0), 0.0, body_height / 2.0)),
        material=stainless,
        name="right_side",
    )
    range_body.visual(
        Box((inner_width, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, body_height / 2.0)),
        material=matte_black,
        name="back_panel",
    )
    range_body.visual(
        Box((inner_width, cavity_depth, cavity_floor_thickness)),
        origin=Origin(xyz=(0.0, 0.045, cavity_floor_center_z)),
        material=matte_black,
        name="cavity_floor",
    )
    range_body.visual(
        Box((inner_width, cavity_depth, cavity_ceiling_thickness)),
        origin=Origin(xyz=(0.0, 0.045, cavity_ceiling_center_z)),
        material=matte_black,
        name="cavity_ceiling",
    )
    range_body.visual(
        Box((inner_width, depth - 2.0 * wall, top_deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_deck_center_z)),
        material=matte_black,
        name="top_support_deck",
    )
    range_body.visual(
        Box((width - 0.04, 0.08, toe_kick_height)),
        origin=Origin(xyz=(0.0, -0.24, toe_kick_height / 2.0)),
        material=matte_black,
        name="toe_kick",
    )
    range_body.visual(
        Box((stile_width, front_frame_depth, door_height + 0.04)),
        origin=Origin(
            xyz=(
                -(door_width / 2.0 + stile_width / 2.0),
                front_center_y,
                door_z0 + (door_height + 0.04) / 2.0,
            )
        ),
        material=stainless,
        name="left_stile",
    )
    range_body.visual(
        Box((stile_width, front_frame_depth, door_height + 0.04)),
        origin=Origin(
            xyz=(
                (door_width / 2.0 + stile_width / 2.0),
                front_center_y,
                door_z0 + (door_height + 0.04) / 2.0,
            )
        ),
        material=stainless,
        name="right_stile",
    )
    range_body.visual(
        Box((door_width, front_frame_depth, 0.04)),
        origin=Origin(xyz=(0.0, front_center_y, door_z0 + door_height + 0.02)),
        material=stainless,
        name="upper_oven_rail",
    )
    control_panel = range_body.visual(
        Box((width, front_frame_depth, control_panel_height)),
        origin=Origin(xyz=(0.0, front_center_y, control_panel_center_z)),
        material=stainless,
        name="control_panel",
    )
    range_body.visual(
        Cylinder(radius=0.011, length=0.05),
        origin=Origin(
            xyz=(-(door_width / 2.0 + 0.025), hinge_origin_y + 0.011, door_z0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="left_hinge_barrel",
    )
    range_body.visual(
        Cylinder(radius=0.011, length=0.05),
        origin=Origin(
            xyz=((door_width / 2.0 + 0.025), hinge_origin_y + 0.011, door_z0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="right_hinge_barrel",
    )
    range_body.inertial = Inertial.from_geometry(
        Box((width, depth, body_height)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((0.78, 0.69, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stainless,
        name="cooktop_slab",
    )
    cooktop.visual(
        Box((0.70, 0.59, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=matte_black,
        name="burner_deck",
    )

    burner_layout = [
        ("front_left", -0.22, -0.17, 0.040),
        ("rear_left", -0.22, 0.15, 0.038),
        ("center", 0.00, -0.01, 0.052),
        ("front_right", 0.22, -0.17, 0.040),
        ("rear_right", 0.22, 0.15, 0.038),
    ]
    for burner_name, bx, by, radius in burner_layout:
        cooktop.visual(
            Cylinder(radius=radius, length=0.004),
            origin=Origin(xyz=(bx, by, 0.014), rpy=(0.0, 0.0, 0.0)),
            material=cast_iron,
            name=f"{burner_name}_ring",
        )
        cooktop.visual(
            Cylinder(radius=radius * 0.58, length=0.010),
            origin=Origin(xyz=(bx, by, 0.021)),
            material=matte_black,
            name=f"{burner_name}_cap",
        )

    def add_grate(prefix: str, center_x: float, center_y: float, sx: float, sy: float) -> None:
        grate_z = 0.032
        grate_bar = 0.012
        grate_h = 0.012
        cooktop.visual(
            Box((sx, grate_bar, grate_h)),
            origin=Origin(xyz=(center_x, center_y - sy / 2.0 + grate_bar / 2.0, grate_z)),
            material=cast_iron,
            name=f"{prefix}_front_bar",
        )
        cooktop.visual(
            Box((sx, grate_bar, grate_h)),
            origin=Origin(xyz=(center_x, center_y + sy / 2.0 - grate_bar / 2.0, grate_z)),
            material=cast_iron,
            name=f"{prefix}_rear_bar",
        )
        cooktop.visual(
            Box((grate_bar, sy, grate_h)),
            origin=Origin(xyz=(center_x - sx / 2.0 + grate_bar / 2.0, center_y, grate_z)),
            material=cast_iron,
            name=f"{prefix}_left_bar",
        )
        cooktop.visual(
            Box((grate_bar, sy, grate_h)),
            origin=Origin(xyz=(center_x + sx / 2.0 - grate_bar / 2.0, center_y, grate_z)),
            material=cast_iron,
            name=f"{prefix}_right_bar",
        )
        cooktop.visual(
            Box((sx - grate_bar, grate_bar, grate_h)),
            origin=Origin(xyz=(center_x, center_y, grate_z)),
            material=cast_iron,
            name=f"{prefix}_cross_bar",
        )

    add_grate("left_grate", -0.22, -0.01, 0.25, 0.39)
    add_grate("center_grate", 0.0, -0.01, 0.17, 0.21)
    add_grate("right_grate", 0.22, -0.01, 0.25, 0.39)
    cooktop.inertial = Inertial.from_geometry(
        Box((0.78, 0.69, 0.05)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    oven_door = model.part("oven_door")
    oven_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(0.0, -door_thickness / 2.0, door_height / 2.0)),
        material=stainless,
        name="outer_panel",
    )
    oven_door.visual(
        Box((door_width * 0.82, door_thickness * 0.70, door_height * 0.78)),
        origin=Origin(xyz=(0.0, -door_thickness * 0.42, door_height * 0.46)),
        material=matte_black,
        name="inner_panel",
    )
    oven_door.visual(
        Box((door_width * 0.56, 0.010, door_height * 0.32)),
        origin=Origin(xyz=(0.0, -door_thickness + 0.005, door_height * 0.38)),
        material=dark_glass,
        name="window",
    )
    oven_door.visual(
        Cylinder(radius=0.012, length=door_width * 0.70),
        origin=Origin(
            xyz=(0.0, -door_thickness - 0.020, door_height * 0.88),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="handle_bar",
    )
    oven_door.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(
            xyz=(-(door_width * 0.26), -door_thickness - 0.010, door_height * 0.88),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="handle_left_post",
    )
    oven_door.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(
            xyz=((door_width * 0.26), -door_thickness - 0.010, door_height * 0.88),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_aluminum,
        name="handle_right_post",
    )
    oven_door.visual(
        Cylinder(radius=0.011, length=0.05),
        origin=Origin(
            xyz=(-(door_width / 2.0 - 0.025), 0.011, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="left_hinge_leaf",
    )
    oven_door.visual(
        Cylinder(radius=0.011, length=0.05),
        origin=Origin(
            xyz=((door_width / 2.0 - 0.025), 0.011, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_aluminum,
        name="right_hinge_leaf",
    )
    oven_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.08, door_height)),
        mass=16.0,
        origin=Origin(xyz=(0.0, -0.02, door_height / 2.0)),
    )

    def add_knob(name: str) -> object:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.031, length=0.036),
            origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="collar",
        )
        knob.visual(
            Box((0.004, 0.003, 0.014)),
            origin=Origin(xyz=(0.0, -0.035, 0.020)),
            material=satin_aluminum,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.07, 0.04, 0.07)),
            mass=0.10,
            origin=Origin(xyz=(0.0, -0.018, 0.0)),
        )
        return knob

    knob_specs = [
        ("left_knob_1", -0.24),
        ("left_knob_2", -0.15),
        ("left_knob_3", -0.06),
        ("right_knob_1", 0.10),
        ("right_knob_2", 0.19),
    ]
    knob_parts = [(add_knob(knob_name), knob_x) for knob_name, knob_x in knob_specs]

    model.articulation(
        "body_to_cooktop",
        ArticulationType.FIXED,
        parent=range_body,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, body_height)),
    )
    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=range_body,
        child=oven_door,
        origin=Origin(xyz=(0.0, hinge_origin_y, door_z0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=0.0,
            upper=1.45,
        ),
    )

    for index, (knob_part, knob_x) in enumerate(knob_parts, start=1):
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=range_body,
            child=knob_part,
            origin=place_on_surface(
                knob_part,
                control_panel,
                point_hint=(knob_x, -depth / 2.0, control_panel_center_z),
                child_axis="-y",
                clearance=0.0,
                spin=0.0,
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=5.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    range_body = object_model.get_part("range_body")
    cooktop = object_model.get_part("cooktop")
    oven_door = object_model.get_part("oven_door")
    knob_parts = [
        object_model.get_part("left_knob_1"),
        object_model.get_part("left_knob_2"),
        object_model.get_part("left_knob_3"),
        object_model.get_part("right_knob_1"),
        object_model.get_part("right_knob_2"),
    ]

    cooktop_joint = object_model.get_articulation("body_to_cooktop")
    door_joint = object_model.get_articulation("body_to_oven_door")
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(1, 6)]

    def aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    body_aabb = ctx.part_world_aabb(range_body)
    ctx.check(
        "range_body_realistic_width",
        body_aabb is not None and 0.72 <= body_aabb[1][0] - body_aabb[0][0] <= 0.82,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "range_body_realistic_depth",
        body_aabb is not None and 0.64 <= body_aabb[1][1] - body_aabb[0][1] <= 0.72,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "range_body_realistic_height",
        body_aabb is not None and 0.88 <= body_aabb[1][2] - body_aabb[0][2] <= 0.93,
        details=f"body_aabb={body_aabb}",
    )

    ctx.check(
        "cooktop_fixed_mount_type",
        cooktop_joint.articulation_type == ArticulationType.FIXED,
        details=f"joint_type={cooktop_joint.articulation_type}",
    )
    ctx.check(
        "oven_door_axis_is_left_right",
        tuple(round(value, 6) for value in door_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "oven_door_limits_downward_only",
        door_joint.motion_limits is not None
        and door_joint.motion_limits.lower == 0.0
        and door_joint.motion_limits.upper is not None
        and 1.3 <= door_joint.motion_limits.upper <= 1.55,
        details=f"limits={door_joint.motion_limits}",
    )
    for index, knob_joint in enumerate(knob_joints, start=1):
        limits = knob_joint.motion_limits
        ctx.check(
            f"knob_{index}_continuous_joint",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"joint_type={knob_joint.articulation_type}",
        )
        ctx.check(
            f"knob_{index}_axis_is_front_to_back",
            tuple(round(value, 6) for value in knob_joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={knob_joint.axis}",
        )
        ctx.check(
            f"knob_{index}_has_unbounded_continuous_limits",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits}",
        )

    ctx.expect_contact(cooktop, range_body, name="cooktop_contacts_body")
    with ctx.pose({door_joint: 0.0}):
        ctx.expect_contact(oven_door, range_body, name="door_closed_hinge_contact")
        closed_door_panel = ctx.part_element_world_aabb(oven_door, elem="outer_panel")
        ctx.expect_within(oven_door, range_body, axes="x", margin=0.07, name="door_within_body_width")

    knob_positions = [ctx.part_world_position(knob) for knob in knob_parts]
    ctx.check(
        "all_knob_positions_resolve",
        all(position is not None for position in knob_positions),
        details=f"knob_positions={knob_positions}",
    )
    knob_xs = [position[0] for position in knob_positions if position is not None]
    ctx.check(
        "knobs_sorted_left_to_right",
        knob_xs == sorted(knob_xs),
        details=f"knob_xs={knob_xs}",
    )
    ctx.check(
        "three_knobs_left_two_knobs_right",
        len(knob_xs) == 5 and max(knob_xs[:3]) < -0.02 and min(knob_xs[3:]) > 0.02,
        details=f"knob_xs={knob_xs}",
    )
    if len(knob_xs) == 5:
        gaps = [knob_xs[index + 1] - knob_xs[index] for index in range(4)]
        ctx.check(
            "center_gap_separates_knob_groups",
            gaps[2] > gaps[0] and gaps[2] > gaps[1] and gaps[2] > gaps[3],
            details=f"gaps={gaps}",
        )

    for index, (knob_part, knob_joint) in enumerate(zip(knob_parts, knob_joints), start=1):
        ctx.expect_contact(knob_part, range_body, name=f"knob_{index}_mounted_to_panel")
        rest_indicator = ctx.part_element_world_aabb(knob_part, elem="indicator")
        with ctx.pose({knob_joint: math.pi / 2.0}):
            ctx.expect_contact(knob_part, range_body, name=f"knob_{index}_rotated_stays_mounted")
            rotated_indicator = ctx.part_element_world_aabb(knob_part, elem="indicator")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"knob_{index}_rotated_no_overlap")
            ctx.fail_if_isolated_parts(name=f"knob_{index}_rotated_no_floating")
        if rest_indicator is not None and rotated_indicator is not None:
            rest_center = aabb_center(rest_indicator)
            rotated_center = aabb_center(rotated_indicator)
            ctx.check(
                f"knob_{index}_indicator_moves_with_rotation",
                abs(rotated_center[0] - rest_center[0]) > 0.012
                and abs(rotated_center[2] - rest_center[2]) > 0.012,
                details=f"rest_center={rest_center}, rotated_center={rotated_center}",
            )

    door_limits = door_joint.motion_limits
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_joint: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_lower_no_floating")
        with ctx.pose({door_joint: door_limits.upper}):
            ctx.expect_contact(oven_door, range_body, name="door_open_hinge_contact")
            open_door_panel = ctx.part_element_world_aabb(oven_door, elem="outer_panel")
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_upper_no_floating")
            if closed_door_panel is not None and open_door_panel is not None:
                closed_min_y = closed_door_panel[0][1]
                open_min_y = open_door_panel[0][1]
                closed_max_z = closed_door_panel[1][2]
                open_max_z = open_door_panel[1][2]
                ctx.check(
                    "door_swings_forward_when_open",
                    open_min_y < closed_min_y - 0.35,
                    details=f"closed_min_y={closed_min_y}, open_min_y={open_min_y}",
                )
                ctx.check(
                    "door_rotates_downward_when_open",
                    open_max_z < closed_max_z - 0.35,
                    details=f"closed_max_z={closed_max_z}, open_max_z={open_max_z}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
