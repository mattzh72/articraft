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
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _rect_loop(
    width: float,
    depth: float,
    z: float,
    *,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    half_depth = depth * 0.5
    return [
        (-half_width, y_center - half_depth, z),
        (half_width, y_center - half_depth, z),
        (half_width, y_center + half_depth, z),
        (-half_width, y_center + half_depth, z),
    ]


def _clock_housing_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _rect_loop(0.220, 0.016, 0.000, y_center=0.000),
                _rect_loop(0.255, 0.082, 0.052, y_center=-0.034),
                _rect_loop(0.205, 0.068, 0.126, y_center=-0.028),
                _rect_loop(0.165, 0.050, 0.146, y_center=-0.020),
            ]
        ),
        ASSETS.mesh_path("compact_stove_clock_housing.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gas_stove", assets=ASSETS)

    enamel = model.material("enamel", rgba=(0.93, 0.93, 0.91, 1.0))
    cooktop_black = model.material("cooktop_black", rgba=(0.14, 0.14, 0.15, 1.0))
    grate_black = model.material("grate_black", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.37, 0.38, 0.40, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.18, 0.23, 0.27, 0.55))
    control_dark = model.material("control_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    marker_red = model.material("marker_red", rgba=(0.76, 0.19, 0.14, 1.0))

    body_width = 0.60
    body_depth = 0.62
    cooktop_height = 0.90
    total_height = 1.14
    shell_thickness = 0.018
    front_panel_thickness = 0.020
    back_panel_thickness = 0.020

    door_width = 0.496
    door_height = 0.578
    door_thickness = 0.035
    door_bottom = 0.10
    hinge_y = -(body_depth * 0.5) - 0.004

    control_bottom = door_bottom + 0.58
    control_height = 0.16

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, total_height)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, total_height * 0.5)),
    )

    body.visual(
        Box((shell_thickness, body_depth, cooktop_height)),
        origin=Origin(xyz=(-(body_width * 0.5) + (shell_thickness * 0.5), 0.0, cooktop_height * 0.5)),
        material=enamel,
        name="left_side",
    )
    body.visual(
        Box((shell_thickness, body_depth, cooktop_height)),
        origin=Origin(xyz=((body_width * 0.5) - (shell_thickness * 0.5), 0.0, cooktop_height * 0.5)),
        material=enamel,
        name="right_side",
    )
    body.visual(
        Box((body_width - (2.0 * shell_thickness), back_panel_thickness, cooktop_height)),
        origin=Origin(
            xyz=(
                0.0,
                (body_depth * 0.5) - (back_panel_thickness * 0.5),
                cooktop_height * 0.5,
            )
        ),
        material=enamel,
        name="rear_panel",
    )
    body.visual(
        Box((body_width, body_depth, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, cooktop_height - 0.015)),
        material=enamel,
        name="top_shell",
    )
    body.visual(
        Box((body_width - 0.030, body_depth - 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, cooktop_height + 0.002)),
        material=cooktop_black,
        name="cooktop_deck",
    )
    body.visual(
        Box((body_width - (2.0 * shell_thickness), front_panel_thickness, door_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth * 0.5) + (front_panel_thickness * 0.5),
                door_bottom * 0.5,
            )
        ),
        material=enamel,
        name="toe_kick",
    )
    body.visual(
        Box((body_width - 0.040, body_depth - 0.040, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=control_dark,
        name="bottom_plate",
    )
    body.visual(
        Box((body_width, front_panel_thickness, control_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth * 0.5) + (front_panel_thickness * 0.5),
                control_bottom + (control_height * 0.5),
            )
        ),
        material=enamel,
        name="control_panel",
    )
    body.visual(
        Box((body_width, 0.040, cooktop_height - (control_bottom + control_height))),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth * 0.5) + 0.020,
                control_bottom + control_height + ((cooktop_height - (control_bottom + control_height)) * 0.5),
            )
        ),
        material=enamel,
        name="front_skirt",
    )
    body.visual(
        Box((0.052, front_panel_thickness, door_height)),
        origin=Origin(
            xyz=(
                -0.274,
                -(body_depth * 0.5) + (front_panel_thickness * 0.5),
                door_bottom + (door_height * 0.5),
            )
        ),
        material=enamel,
        name="left_stile",
    )
    body.visual(
        Box((0.052, front_panel_thickness, door_height)),
        origin=Origin(
            xyz=(
                0.274,
                -(body_depth * 0.5) + (front_panel_thickness * 0.5),
                door_bottom + (door_height * 0.5),
            )
        ),
        material=enamel,
        name="right_stile",
    )

    oven_inner_depth = 0.537
    oven_inner_center_y = 0.0035
    oven_inner_width = 0.482
    oven_inner_height = 0.544
    body.visual(
        Box((shell_thickness, oven_inner_depth, door_height)),
        origin=Origin(xyz=(-0.241, oven_inner_center_y, door_bottom + (door_height * 0.5))),
        material=control_dark,
        name="oven_left_liner",
    )
    body.visual(
        Box((shell_thickness, oven_inner_depth, door_height)),
        origin=Origin(xyz=(0.241, oven_inner_center_y, door_bottom + (door_height * 0.5))),
        material=control_dark,
        name="oven_right_liner",
    )
    body.visual(
        Box((oven_inner_width, oven_inner_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, oven_inner_center_y, door_bottom + (shell_thickness * 0.5))),
        material=control_dark,
        name="oven_floor",
    )
    body.visual(
        Box((oven_inner_width, oven_inner_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, oven_inner_center_y, door_bottom + door_height - (shell_thickness * 0.5))),
        material=control_dark,
        name="oven_ceiling",
    )
    body.visual(
        Box((oven_inner_width, shell_thickness, oven_inner_height)),
        origin=Origin(xyz=(0.0, 0.281, door_bottom + (door_height * 0.5))),
        material=control_dark,
        name="oven_back_liner",
    )

    body.visual(
        Box((door_width * 0.90, 0.008, 0.010)),
        origin=Origin(
            xyz=(0.0, -0.3065, door_bottom - 0.005),
        ),
        material=grate_black,
        name="door_hinge_pin",
    )

    burner_positions = {
        "front_left": (-0.165, -0.165),
        "front_right": (0.165, -0.165),
        "rear_left": (-0.165, 0.165),
        "rear_right": (0.165, 0.165),
    }
    for burner_name, (x_pos, y_pos) in burner_positions.items():
        body.visual(
            Cylinder(radius=0.068, length=0.004),
            origin=Origin(xyz=(x_pos, y_pos, cooktop_height + 0.002)),
            material=burner_metal,
            name=f"burner_{burner_name}_tray",
        )
        body.visual(
            Cylinder(radius=0.045, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, cooktop_height + 0.007)),
            material=burner_metal,
            name=f"burner_{burner_name}_ring",
        )
        body.visual(
            Cylinder(radius=0.029, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, cooktop_height + 0.012)),
            material=control_dark,
            name=f"burner_{burner_name}_cap",
        )
        body.visual(
            Box((0.110, 0.012, 0.012)),
            origin=Origin(xyz=(x_pos, y_pos, cooktop_height + 0.015)),
            material=grate_black,
            name=f"burner_{burner_name}_grate_x",
        )
        body.visual(
            Box((0.012, 0.110, 0.012)),
            origin=Origin(xyz=(x_pos, y_pos, cooktop_height + 0.015)),
            material=grate_black,
            name=f"burner_{burner_name}_grate_y",
        )

    body.visual(
        Box((0.560, 0.030, total_height - cooktop_height)),
        origin=Origin(xyz=(0.0, 0.295, cooktop_height + ((total_height - cooktop_height) * 0.5))),
        material=enamel,
        name="backguard",
    )
    body.visual(
        _clock_housing_mesh(),
        origin=Origin(xyz=(0.0, 0.272, 0.975)),
        material=enamel,
        name="clock_housing",
    )
    body.visual(
        Box((0.170, 0.014, 0.070)),
        origin=Origin(xyz=(0.0, 0.202, 1.056)),
        material=enamel,
        name="clock_bezel",
    )
    body.visual(
        Box((0.150, 0.004, 0.050)),
        origin=Origin(xyz=(0.0, 0.205, 1.056)),
        material=glass_smoke,
        name="clock_display",
    )

    for foot_name, x_pos, y_pos in (
        ("front_left", -0.245, -0.245),
        ("front_right", 0.245, -0.245),
        ("rear_left", -0.245, 0.245),
        ("rear_right", 0.245, 0.245),
    ):
        body.visual(
            Cylinder(radius=0.018, length=0.035),
            origin=Origin(xyz=(x_pos, y_pos, 0.0175)),
            material=grate_black,
            name=f"foot_{foot_name}",
        )

    door = model.part("oven_door")
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(0.0, -0.014, door_height * 0.5)),
        material=enamel,
        name="door_panel",
    )
    door.visual(
        Box((0.300, 0.004, 0.230)),
        origin=Origin(xyz=(0.0, -0.030, door_height * 0.48)),
        material=glass_smoke,
        name="door_window",
    )
    door.visual(
        Cylinder(radius=0.0035, length=door_width * 0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=handle_steel,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(
            xyz=(0.0, -0.049, door_height * 0.82),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=handle_steel,
        name="door_handle",
    )
    door.visual(
        Box((0.018, 0.026, 0.055)),
        origin=Origin(xyz=(-0.150, -0.024, door_height * 0.82)),
        material=handle_steel,
        name="handle_left_post",
    )
    door.visual(
        Box((0.018, 0.026, 0.055)),
        origin=Origin(xyz=(0.150, -0.024, door_height * 0.82)),
        material=handle_steel,
        name="handle_right_post",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.075, door_height)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.020, door_height * 0.5)),
    )

    knob_specs = (
        ("left_upper_knob", -0.195, 0.790),
        ("left_lower_knob", -0.195, 0.715),
        ("right_upper_knob", 0.195, 0.790),
        ("right_lower_knob", 0.195, 0.715),
    )
    for part_name, x_pos, z_pos in knob_specs:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.028, length=0.022),
            origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=control_dark,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=cooktop_black,
            name="knob_skirt",
        )
        knob.visual(
            Box((0.005, 0.004, 0.011)),
            origin=Origin(xyz=(0.0, -0.024, 0.017)),
            material=marker_red,
            name="knob_indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.060, 0.028, 0.060)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.012, 0.0)),
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_pos, -(body_depth * 0.5), z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=10.0),
        )

    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, hinge_y, door_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(96.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    door = object_model.get_part("oven_door")
    door_joint = object_model.get_articulation("body_to_oven_door")
    knob_names = (
        "left_upper_knob",
        "left_lower_knob",
        "right_upper_knob",
        "right_lower_knob",
    )
    knobs = {name: object_model.get_part(name) for name in knob_names}
    knob_joints = {
        name: object_model.get_articulation(f"body_to_{name}")
        for name in knob_names
    }

    def part_aabb(part, *, check_name: str):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            ctx.fail(check_name, f"World AABB unavailable for part {part.name}.")
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        return aabb

    def elem_aabb(part, elem: str, *, check_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            ctx.fail(check_name, f"World AABB unavailable for {part.name}.{elem}.")
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        return aabb

    def aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(
        body,
        door,
        elem_a=body.get_visual("door_hinge_pin"),
        elem_b=door.get_visual("hinge_barrel"),
        reason="The oven door hinge barrel rotates concentrically around the fixed hinge pin.",
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    body_aabb = part_aabb(body, check_name="body_aabb_available")
    body_width = body_aabb[1][0] - body_aabb[0][0]
    body_depth = body_aabb[1][1] - body_aabb[0][1]
    body_height = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "stove_width_realistic",
        0.56 <= body_width <= 0.66,
        f"Expected compact stove width near 0.60 m, got {body_width:.3f} m.",
    )
    ctx.check(
        "stove_depth_realistic",
        0.58 <= body_depth <= 0.67,
        f"Expected depth near 0.62 m, got {body_depth:.3f} m.",
    )
    ctx.check(
        "stove_height_realistic",
        1.08 <= body_height <= 1.18,
        f"Expected top of backguard near 1.14 m, got {body_height:.3f} m.",
    )

    burner_centers = {
        burner_name: aabb_center(
            elem_aabb(body, f"burner_{burner_name}_cap", check_name=f"{burner_name}_burner_cap_aabb")
        )
        for burner_name in ("front_left", "front_right", "rear_left", "rear_right")
    }
    ctx.check(
        "burners_form_two_columns",
        burner_centers["front_left"][0] < -0.10
        and burner_centers["rear_left"][0] < -0.10
        and burner_centers["front_right"][0] > 0.10
        and burner_centers["rear_right"][0] > 0.10,
        f"Burner x centers were {burner_centers}.",
    )
    ctx.check(
        "burners_form_front_and_rear_rows",
        burner_centers["front_left"][1] < -0.10
        and burner_centers["front_right"][1] < -0.10
        and burner_centers["rear_left"][1] > 0.10
        and burner_centers["rear_right"][1] > 0.10,
        f"Burner y centers were {burner_centers}.",
    )

    clock_display_aabb = elem_aabb(body, "clock_display", check_name="clock_display_aabb")
    ctx.check(
        "clock_display_above_cooktop",
        clock_display_aabb[0][2] > 1.00,
        f"Clock display should sit on the tall backguard, got z-min {clock_display_aabb[0][2]:.3f}.",
    )

    ctx.check(
        "door_joint_type",
        door_joint.articulation_type == ArticulationType.REVOLUTE,
        f"Expected a revolute oven door joint, got {door_joint.articulation_type}.",
    )
    ctx.check(
        "door_axis_left_to_right",
        tuple(door_joint.axis) == (1.0, 0.0, 0.0),
        f"Expected door hinge axis along +X, got {door_joint.axis}.",
    )
    door_limits = door_joint.motion_limits
    ctx.check(
        "door_has_realistic_limits",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and math.radians(85.0) <= door_limits.upper <= math.radians(105.0),
        f"Expected oven door opening limit near 90-100 degrees, got {door_limits}.",
    )

    closed_panel_aabb = elem_aabb(door, "door_panel", check_name="closed_door_panel_aabb")
    left_stile_aabb = elem_aabb(body, "left_stile", check_name="left_stile_aabb")
    right_stile_aabb = elem_aabb(body, "right_stile", check_name="right_stile_aabb")
    ctx.check(
        "door_spans_front_opening_width",
        abs(closed_panel_aabb[0][0] - left_stile_aabb[1][0]) <= 0.010
        and abs(closed_panel_aabb[1][0] - right_stile_aabb[0][0]) <= 0.010,
        (
            "Door should fit between the two front stiles; "
            f"door x-span {closed_panel_aabb[0][0]:.3f}..{closed_panel_aabb[1][0]:.3f}, "
            f"stile inner faces {left_stile_aabb[1][0]:.3f} and {right_stile_aabb[0][0]:.3f}."
        ),
    )
    ctx.expect_gap(
        body,
        door,
        axis="z",
        positive_elem=body.get_visual("control_panel"),
        negative_elem=door.get_visual("door_panel"),
        min_gap=0.0,
        max_gap=0.004,
        name="door_top_gap_closed",
    )
    ctx.expect_gap(
        door,
        body,
        axis="z",
        positive_elem=door.get_visual("door_panel"),
        negative_elem=body.get_visual("toe_kick"),
        min_gap=0.0,
        max_gap=0.002,
        name="door_bottom_seats_on_kickplate_line",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=door.get_visual("hinge_barrel"),
        elem_b=body.get_visual("door_hinge_pin"),
        name="door_hinge_remains_supported_closed",
    )

    closed_door_aabb = part_aabb(door, check_name="closed_door_aabb")
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_joint: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_open_no_unintended_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_open_no_floating")
            ctx.expect_contact(
                door,
                body,
                elem_a=door.get_visual("hinge_barrel"),
                elem_b=body.get_visual("door_hinge_pin"),
                name="door_hinge_remains_supported_open",
            )
            open_door_aabb = part_aabb(door, check_name="open_door_aabb")
            ctx.check(
                "door_swings_outward",
                open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.20,
                (
                    "Expected the open door to project forward substantially; "
                    f"closed min-y {closed_door_aabb[0][1]:.3f}, open min-y {open_door_aabb[0][1]:.3f}."
                ),
            )
            ctx.check(
                "door_drops_down_when_open",
                open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.30,
                (
                    "Expected the open door top to drop well below the closed position; "
                    f"closed max-z {closed_door_aabb[1][2]:.3f}, open max-z {open_door_aabb[1][2]:.3f}."
                ),
            )

    expected_x = {
        "left_upper_knob": -0.195,
        "left_lower_knob": -0.195,
        "right_upper_knob": 0.195,
        "right_lower_knob": 0.195,
    }
    expected_z = {
        "left_upper_knob": 0.790,
        "left_lower_knob": 0.715,
        "right_upper_knob": 0.790,
        "right_lower_knob": 0.715,
    }
    for knob_name in knob_names:
        knob = knobs[knob_name]
        joint = knob_joints[knob_name]
        ctx.check(
            f"{knob_name}_joint_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"Expected continuous knob rotation, got {joint.articulation_type}.",
        )
        ctx.check(
            f"{knob_name}_axis_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"Expected front-to-back axis on {knob_name}, got {joint.axis}.",
        )
        knob_limits = joint.motion_limits
        ctx.check(
            f"{knob_name}_continuous_limits_unbounded",
            knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
            f"Continuous knobs should not have lower/upper bounds, got {knob_limits}.",
        )
        knob_pos = ctx.part_world_position(knob)
        if knob_pos is None:
            ctx.fail(f"{knob_name}_position_available", f"World position unavailable for {knob_name}.")
            knob_pos = (0.0, 0.0, 0.0)
        ctx.check(
            f"{knob_name}_x_location",
            abs(knob_pos[0] - expected_x[knob_name]) <= 0.010,
            f"Expected x near {expected_x[knob_name]:.3f}, got {knob_pos[0]:.3f}.",
        )
        ctx.check(
            f"{knob_name}_z_location",
            abs(knob_pos[2] - expected_z[knob_name]) <= 0.010,
            f"Expected z near {expected_z[knob_name]:.3f}, got {knob_pos[2]:.3f}.",
        )
        ctx.expect_contact(
            knob,
            body,
            elem_a=knob.get_visual("knob_body"),
            elem_b=body.get_visual("control_panel"),
            name=f"{knob_name}_mounted_to_control_panel",
        )
        with ctx.pose({joint: 1.70}):
            spun_pos = ctx.part_world_position(knob)
            if spun_pos is None:
                ctx.fail(
                    f"{knob_name}_spun_position_available",
                    f"World position unavailable for rotated {knob_name}.",
                )
            else:
                ctx.check(
                    f"{knob_name}_spin_keeps_axis_location",
                    abs(spun_pos[0] - knob_pos[0]) <= 1e-6
                    and abs(spun_pos[1] - knob_pos[1]) <= 1e-6
                    and abs(spun_pos[2] - knob_pos[2]) <= 1e-6,
                    f"Knob center moved under rotation from {knob_pos} to {spun_pos}.",
                )
            ctx.expect_contact(
                knob,
                body,
                elem_a=knob.get_visual("knob_body"),
                elem_b=body.get_visual("control_panel"),
                name=f"{knob_name}_remains_mounted_while_rotated",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
