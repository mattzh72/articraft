from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_kitchen_range", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.74, 0.75, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.15, 0.15, 0.16, 1.0))
    black = model.material("black", rgba=(0.08, 0.08, 0.09, 1.0))
    amber = model.material("amber", rgba=(0.82, 0.46, 0.12, 1.0))

    carcass = model.part("carcass")
    carcass.inertial = Inertial.from_geometry(
        Box((0.91, 0.72, 0.94)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
    )

    body_visuals = [
        ("left_side", Box((0.03, 0.72, 0.87)), (-0.44, 0.0, 0.435), stainless),
        ("right_side", Box((0.03, 0.72, 0.87)), (0.44, 0.0, 0.435), stainless),
        ("back_panel", Box((0.85, 0.03, 0.87)), (0.0, 0.345, 0.435), stainless),
        ("bottom_floor", Box((0.85, 0.63, 0.03)), (0.0, 0.015, 0.015), stainless),
        ("toe_kick", Box((0.85, 0.06, 0.06)), (0.0, -0.33, 0.03), dark_steel),
        ("oven_header", Box((0.85, 0.06, 0.09)), (0.0, -0.33, 0.675), stainless),
        ("control_rail_upper", Box((0.85, 0.09, 0.09)), (0.0, -0.36, 0.825), stainless),
        ("control_rail_lower_left", Box((0.305, 0.09, 0.06)), (-0.26, -0.36, 0.75), stainless),
        ("control_rail_lower_center", Box((0.05, 0.09, 0.06)), (0.0, -0.36, 0.75), stainless),
        ("control_rail_lower_right", Box((0.305, 0.09, 0.06)), (0.26, -0.36, 0.75), stainless),
        ("cooktop", Box((0.91, 0.72, 0.03)), (0.0, 0.0, 0.885), stainless),
    ]
    for name, geometry, xyz, material in body_visuals:
        carcass.visual(geometry, origin=Origin(xyz=xyz), material=material, name=name)
    carcass.visual(
        Cylinder(radius=0.024, length=0.84),
        origin=Origin(xyz=(0.0, -0.378, 0.790), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="front_bullnose_rail",
    )

    burner_xs = (-0.24, 0.0, 0.24)
    burner_ys = (-0.12, 0.12)
    burner_index = 1
    for y in burner_ys:
        for x in burner_xs:
            carcass.visual(
                Cylinder(radius=0.055, length=0.012),
                origin=Origin(xyz=(x, y, 0.906)),
                material=dark_steel,
                name=f"burner_{burner_index}_skirt",
            )
            carcass.visual(
                Cylinder(radius=0.035, length=0.010),
                origin=Origin(xyz=(x, y, 0.917)),
                material=black,
                name=f"burner_{burner_index}_cap",
            )
            burner_index += 1

    grate_centers = (-0.24, 0.0, 0.24)
    for grate_index, x_center in enumerate(grate_centers, start=1):
        for foot_index, (dx, dy) in enumerate(
            ((-0.075, -0.13), (-0.075, 0.13), (0.075, -0.13), (0.075, 0.13)),
            start=1,
        ):
            carcass.visual(
                Box((0.022, 0.022, 0.024)),
                origin=Origin(xyz=(x_center + dx, dy, 0.912)),
                material=cast_iron,
                name=f"grate_{grate_index}_foot_{foot_index}",
            )
        carcass.visual(
            Box((0.018, 0.280, 0.012)),
            origin=Origin(xyz=(x_center - 0.075, 0.0, 0.930)),
            material=cast_iron,
            name=f"grate_{grate_index}_left_rail",
        )
        carcass.visual(
            Box((0.018, 0.280, 0.012)),
            origin=Origin(xyz=(x_center + 0.075, 0.0, 0.930)),
            material=cast_iron,
            name=f"grate_{grate_index}_right_rail",
        )
        carcass.visual(
            Box((0.170, 0.018, 0.012)),
            origin=Origin(xyz=(x_center, -0.130, 0.930)),
            material=cast_iron,
            name=f"grate_{grate_index}_front_rail",
        )
        carcass.visual(
            Box((0.170, 0.018, 0.012)),
            origin=Origin(xyz=(x_center, 0.130, 0.930)),
            material=cast_iron,
            name=f"grate_{grate_index}_rear_rail",
        )
        carcass.visual(
            Box((0.018, 0.260, 0.012)),
            origin=Origin(xyz=(x_center, 0.0, 0.930)),
            material=cast_iron,
            name=f"grate_{grate_index}_center_bar",
        )
        carcass.visual(
            Box((0.140, 0.018, 0.012)),
            origin=Origin(xyz=(x_center, 0.0, 0.930)),
            material=cast_iron,
            name=f"grate_{grate_index}_cross_bar",
        )

    door = model.part("oven_door")
    door.inertial = Inertial.from_geometry(
        Box((0.83, 0.055, 0.54)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0275, 0.27)),
    )
    door.visual(
        Box((0.83, 0.055, 0.54)),
        origin=Origin(xyz=(0.0, 0.0275, 0.27)),
        material=stainless,
        name="door_panel",
    )
    door.visual(
        Box((0.03, 0.04, 0.08)),
        origin=Origin(xyz=(-0.27, -0.02, 0.42)),
        material=dark_steel,
        name="door_handle_left_post",
    )
    door.visual(
        Box((0.03, 0.04, 0.08)),
        origin=Origin(xyz=(0.27, -0.02, 0.42)),
        material=dark_steel,
        name="door_handle_right_post",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.68),
        origin=Origin(xyz=(0.0, -0.04, 0.42), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="door_handle_bar",
    )
    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=door,
        origin=Origin(xyz=(0.0, -0.36, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6, lower=0.0, upper=1.45),
    )

    def add_knob(part_name: str, joint_name: str, x_pos: float, z_pos: float, radius: float, length: float) -> None:
        knob = model.part(part_name)
        knob.inertial = Inertial.from_geometry(
            Box((radius * 2.0, length, radius * 2.0)),
            mass=0.25,
        )
        knob.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=black,
            name="knob_body",
        )
        knob.visual(
            Box((0.006, 0.004, 0.016)),
            origin=Origin(xyz=(0.0, -(length * 0.5) + 0.002, radius * 0.55)),
            material=amber,
            name="knob_pointer",
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=carcass,
            child=knob,
            origin=Origin(xyz=(x_pos, -0.405 - (length * 0.5), z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=8.0),
        )

    add_knob("knob_far_left", "knob_far_left_spin", -0.34, 0.826, 0.028, 0.032)
    add_knob("knob_left_inner", "knob_left_inner_spin", -0.18, 0.826, 0.032, 0.045)
    add_knob("knob_mid_left", "knob_mid_left_spin", -0.06, 0.826, 0.032, 0.045)
    add_knob("knob_mid_right", "knob_mid_right_spin", 0.06, 0.826, 0.032, 0.045)
    add_knob("knob_right_inner", "knob_right_inner_spin", 0.18, 0.826, 0.032, 0.045)
    add_knob("knob_far_right", "knob_far_right_spin", 0.34, 0.826, 0.028, 0.032)

    def add_button(part_name: str, joint_name: str, x_pos: float) -> None:
        for guide_side, guide_x in (("left", x_pos - 0.013), ("right", x_pos + 0.013)):
            carcass.visual(
                Box((0.002, 0.026, 0.060)),
                origin=Origin(xyz=(guide_x, -0.392, 0.750)),
                material=dark_steel,
                name=f"{part_name}_guide_{guide_side}",
            )
        button = model.part(part_name)
        button.inertial = Inertial.from_geometry(
            Box((0.024, 0.018, 0.024)),
            mass=0.08,
        )
        button.visual(
            Box((0.024, 0.018, 0.024)),
            material=black,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=black,
            name="button_cap",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=button,
            origin=Origin(xyz=(x_pos, -0.414, 0.748)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.012),
        )

    add_button("button_left", "button_left_press", -0.06)
    add_button("button_right", "button_right_press", 0.06)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("carcass")
    door = object_model.get_part("oven_door")
    knob_far_left = object_model.get_part("knob_far_left")
    knob_left_inner = object_model.get_part("knob_left_inner")
    knob_mid_left = object_model.get_part("knob_mid_left")
    knob_mid_right = object_model.get_part("knob_mid_right")
    knob_right_inner = object_model.get_part("knob_right_inner")
    knob_far_right = object_model.get_part("knob_far_right")
    button_left = object_model.get_part("button_left")
    button_right = object_model.get_part("button_right")

    door_hinge = object_model.get_articulation("oven_door_hinge")
    knob_joints = [
        object_model.get_articulation("knob_far_left_spin"),
        object_model.get_articulation("knob_left_inner_spin"),
        object_model.get_articulation("knob_mid_left_spin"),
        object_model.get_articulation("knob_mid_right_spin"),
        object_model.get_articulation("knob_right_inner_spin"),
        object_model.get_articulation("knob_far_right_spin"),
    ]
    button_left_joint = object_model.get_articulation("button_left_press")
    button_right_joint = object_model.get_articulation("button_right_press")

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

    expected_parts = {
        "carcass",
        "oven_door",
        "knob_far_left",
        "knob_left_inner",
        "knob_mid_left",
        "knob_mid_right",
        "knob_right_inner",
        "knob_far_right",
        "button_left",
        "button_right",
    }
    actual_parts = {part.name for part in object_model.parts}
    ctx.check(
        "all_expected_parts_present",
        expected_parts.issubset(actual_parts),
        details=f"missing parts: {sorted(expected_parts - actual_parts)}",
    )

    body_visual_names = {visual.name for visual in body.visuals if visual.name is not None}
    for burner_index in range(1, 7):
        ctx.check(
            f"burner_{burner_index}_cap_present",
            f"burner_{burner_index}_cap" in body_visual_names,
            details=f"missing burner_{burner_index}_cap visual",
        )

    for joint in knob_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_axis",
            joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"{joint.name} axis/limits incorrect: axis={joint.axis}, limits={limits}",
        )

    for joint in (button_left_joint, button_right_joint):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_prismatic_axis",
            joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.008 <= limits.upper <= 0.02,
            details=f"{joint.name} axis/limits incorrect: axis={joint.axis}, limits={limits}",
        )

    door_limits = door_hinge.motion_limits
    ctx.check(
        "door_hinge_axis_and_range",
        door_hinge.axis == (1.0, 0.0, 0.0)
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and 1.2 <= door_limits.upper <= 1.6,
        details=f"door hinge incorrect: axis={door_hinge.axis}, limits={door_limits}",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_aabb_exists", "carcass world AABB is unavailable")
    else:
        width = body_aabb[1][0] - body_aabb[0][0]
        depth = body_aabb[1][1] - body_aabb[0][1]
        height = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "range_body_realistic_size",
            0.88 <= width <= 0.96 and 0.70 <= depth <= 0.78 and 0.92 <= height <= 0.98,
            details=f"unexpected carcass size: width={width:.3f}, depth={depth:.3f}, height={height:.3f}",
        )

    knob_parts = [
        knob_far_left,
        knob_left_inner,
        knob_mid_left,
        knob_mid_right,
        knob_right_inner,
        knob_far_right,
    ]
    for knob in knob_parts:
        ctx.expect_contact(knob, body, contact_tol=1e-4, name=f"{knob.name}_mounted_to_rail")

    ctx.expect_contact(door, body, contact_tol=1e-4, name="door_closed_contacts_body")
    ctx.expect_overlap(door, body, axes="x", min_overlap=0.78, name="door_spans_body_width")
    ctx.expect_within(door, body, axes="x", margin=0.05, name="door_within_body_width")

    knob_positions = [ctx.part_world_position(knob) for knob in knob_parts]
    if any(position is None for position in knob_positions):
        ctx.fail("knob_positions_available", "one or more knob world positions are unavailable")
    else:
        knob_xs = [position[0] for position in knob_positions if position is not None]
        ctx.check(
            "six_knobs_ordered_across_control_rail",
            all(knob_xs[index] < knob_xs[index + 1] for index in range(len(knob_xs) - 1)),
            details=f"knob x positions are not strictly increasing: {knob_xs}",
        )

    button_left_pos = ctx.part_world_position(button_left)
    button_right_pos = ctx.part_world_position(button_right)
    knob_mid_left_pos = ctx.part_world_position(knob_mid_left)
    knob_mid_right_pos = ctx.part_world_position(knob_mid_right)
    if (
        button_left_pos is None
        or button_right_pos is None
        or knob_mid_left_pos is None
        or knob_mid_right_pos is None
    ):
        ctx.fail("button_and_middle_knob_positions_available", "missing button or middle knob position")
    else:
        ctx.check(
            "left_button_tucked_under_left_middle_knob",
            abs(button_left_pos[0] - knob_mid_left_pos[0]) <= 0.004
            and 0.055 <= knob_mid_left_pos[2] - button_left_pos[2] <= 0.10,
            details=f"left button / knob positions: button={button_left_pos}, knob={knob_mid_left_pos}",
        )
        ctx.check(
            "right_button_tucked_under_right_middle_knob",
            abs(button_right_pos[0] - knob_mid_right_pos[0]) <= 0.004
            and 0.055 <= knob_mid_right_pos[2] - button_right_pos[2] <= 0.10,
            details=f"right button / knob positions: button={button_right_pos}, knob={knob_mid_right_pos}",
        )

    left_end_aabb = ctx.part_world_aabb(knob_far_left)
    right_end_aabb = ctx.part_world_aabb(knob_far_right)
    mid_left_aabb = ctx.part_world_aabb(knob_mid_left)
    if left_end_aabb is None or right_end_aabb is None or mid_left_aabb is None:
        ctx.fail("knob_aabbs_available", "missing knob AABB for depth comparison")
    else:
        left_depth = left_end_aabb[1][1] - left_end_aabb[0][1]
        right_depth = right_end_aabb[1][1] - right_end_aabb[0][1]
        middle_depth = mid_left_aabb[1][1] - mid_left_aabb[0][1]
        ctx.check(
            "end_knobs_shorter_than_center_knobs",
            left_depth + 0.008 < middle_depth and right_depth + 0.008 < middle_depth,
            details=(
                f"left_end_depth={left_depth:.3f}, right_end_depth={right_depth:.3f}, "
                f"middle_depth={middle_depth:.3f}"
            ),
        )

    ctx.expect_origin_distance(
        button_left,
        button_right,
        axes="z",
        max_dist=0.002,
        name="buttons_share_same_height",
    )
    ctx.expect_origin_distance(
        button_left,
        button_right,
        axes="x",
        min_dist=0.10,
        max_dist=0.16,
        name="buttons_evenly_spaced_under_middle_pair",
    )
    ctx.expect_contact(button_left, body, contact_tol=1e-6, name="left_button_guided_by_body")
    ctx.expect_contact(button_right, body, contact_tol=1e-6, name="right_button_guided_by_body")

    knob_pose = {
        knob_joints[0]: 0.8,
        knob_joints[1]: -1.1,
        knob_joints[2]: 1.9,
        knob_joints[3]: -0.7,
        knob_joints[4]: 2.4,
        knob_joints[5]: -1.5,
    }
    with ctx.pose(knob_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="turned_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="turned_knobs_no_floating")

    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_lower_no_floating")
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_upper_no_floating")

    for button_joint in (button_left_joint, button_right_joint):
        limits = button_joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.lower}):
                moved_button = button_left if button_joint is button_left_joint else button_right
                ctx.expect_contact(
                    moved_button,
                    body,
                    contact_tol=1e-6,
                    name=f"{button_joint.name}_lower_guided_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button_joint.name}_lower_no_floating")
            with ctx.pose({button_joint: limits.upper}):
                moved_button = button_left if button_joint is button_left_joint else button_right
                ctx.expect_contact(
                    moved_button,
                    body,
                    contact_tol=1e-6,
                    name=f"{button_joint.name}_upper_guided_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{button_joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
