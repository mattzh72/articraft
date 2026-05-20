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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_gas_stove", assets=ASSETS)

    enamel_cream = model.material("enamel_cream", rgba=(0.90, 0.84, 0.70, 1.0))
    enamel_ivory = model.material("enamel_ivory", rgba=(0.95, 0.91, 0.82, 1.0))
    cooktop_steel = model.material("cooktop_steel", rgba=(0.62, 0.63, 0.65, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.18, 0.19, 1.0))
    chrome = model.material("chrome", rgba=(0.79, 0.81, 0.84, 1.0))
    glass = model.material("oven_glass", rgba=(0.18, 0.22, 0.25, 0.42))
    brass = model.material("brass_marker", rgba=(0.78, 0.65, 0.34, 1.0))

    body_width = 0.74
    body_depth = 0.61
    body_radius = 0.060
    foot_height = 0.08
    chassis_height = 0.70
    cooktop_width = 0.76
    cooktop_depth = 0.66
    cooktop_radius = 0.082
    cooktop_thickness = 0.04
    door_width = 0.56
    door_height = 0.50
    door_thickness = 0.045
    door_bottom_world = 0.12
    opening_width = 0.572
    opening_height = 0.504
    opening_depth = 0.46
    cooktop_top_z = foot_height + chassis_height + cooktop_thickness
    front_face_y = -body_depth / 2.0

    def add_burner(body_part, label: str, x: float, y: float) -> None:
        body_part.visual(
            Cylinder(radius=0.074, length=0.004),
            origin=Origin(xyz=(x, y, cooktop_top_z + 0.002)),
            material=dark_trim,
            name=f"burner_{label}_pan",
        )
        body_part.visual(
            Cylinder(radius=0.058, length=0.012),
            origin=Origin(xyz=(x, y, cooktop_top_z + 0.010)),
            material=cast_iron,
            name=f"burner_{label}_ring",
        )
        body_part.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(xyz=(x, y, cooktop_top_z + 0.025)),
            material=cast_iron,
            name=f"burner_{label}_cap",
        )
        body_part.visual(
            Box((0.122, 0.016, 0.010)),
            origin=Origin(xyz=(x, y, cooktop_top_z + 0.035)),
            material=cast_iron,
            name=f"burner_{label}_grate_x",
        )
        body_part.visual(
            Box((0.016, 0.122, 0.010)),
            origin=Origin(xyz=(x, y, cooktop_top_z + 0.035)),
            material=cast_iron,
            name=f"burner_{label}_grate_y",
        )

    body = model.part("body")

    panel_thickness = 0.02
    straight_front_width = body_width - 2.0 * body_radius
    side_run_depth = body_depth - body_radius
    cooktop_center_z = foot_height + chassis_height + cooktop_thickness / 2.0

    body.visual(
        Box((body_width - 2.0 * panel_thickness, panel_thickness, chassis_height)),
        origin=Origin(
            xyz=(0.0, body_depth / 2.0 - panel_thickness / 2.0, foot_height + chassis_height / 2.0)
        ),
        material=enamel_cream,
        name="rear_panel",
    )
    body.visual(
        Box((panel_thickness, side_run_depth, chassis_height)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + panel_thickness / 2.0,
                body_radius / 2.0,
                foot_height + chassis_height / 2.0,
            )
        ),
        material=enamel_cream,
        name="left_side_panel",
    )
    body.visual(
        Box((panel_thickness, side_run_depth, chassis_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - panel_thickness / 2.0,
                body_radius / 2.0,
                foot_height + chassis_height / 2.0,
            )
        ),
        material=enamel_cream,
        name="right_side_panel",
    )
    body.visual(
        Cylinder(radius=body_radius, length=chassis_height),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + body_radius,
                front_face_y + body_radius,
                foot_height + chassis_height / 2.0,
            )
        ),
        material=enamel_cream,
        name="left_front_corner",
    )
    body.visual(
        Cylinder(radius=body_radius, length=chassis_height),
        origin=Origin(
            xyz=(
                body_width / 2.0 - body_radius,
                front_face_y + body_radius,
                foot_height + chassis_height / 2.0,
            )
        ),
        material=enamel_cream,
        name="right_front_corner",
    )
    body.visual(
        Box((straight_front_width, panel_thickness, door_bottom_world - foot_height)),
        origin=Origin(
            xyz=(0.0, front_face_y + panel_thickness / 2.0, foot_height + (door_bottom_world - foot_height) / 2.0)
        ),
        material=enamel_cream,
        name="toe_kick",
    )
    front_stile_width = (straight_front_width - door_width) / 2.0
    body.visual(
        Box((front_stile_width, panel_thickness, door_height)),
        origin=Origin(
            xyz=(
                -door_width / 2.0 - front_stile_width / 2.0,
                front_face_y + panel_thickness / 2.0,
                door_bottom_world + door_height / 2.0,
            )
        ),
        material=enamel_cream,
        name="left_front_stile",
    )
    body.visual(
        Box((front_stile_width, panel_thickness, door_height)),
        origin=Origin(
            xyz=(
                door_width / 2.0 + front_stile_width / 2.0,
                front_face_y + panel_thickness / 2.0,
                door_bottom_world + door_height / 2.0,
            )
        ),
        material=enamel_cream,
        name="right_front_stile",
    )
    body.visual(
        Box((straight_front_width, panel_thickness, 0.035)),
        origin=Origin(
            xyz=(0.0, front_face_y + panel_thickness / 2.0, door_bottom_world + door_height + 0.0175)
        ),
        material=enamel_cream,
        name="front_upper_fascia",
    )
    body.visual(
        Box((cooktop_width - 2.0 * cooktop_radius, cooktop_depth, cooktop_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cooktop_center_z)),
        material=enamel_ivory,
        name="cooktop_lip",
    )
    body.visual(
        Cylinder(radius=cooktop_radius, length=cooktop_thickness),
        origin=Origin(
            xyz=(
                -cooktop_width / 2.0 + cooktop_radius,
                -cooktop_depth / 2.0 + cooktop_radius,
                cooktop_center_z,
            )
        ),
        material=enamel_ivory,
        name="cooktop_left_corner",
    )
    body.visual(
        Cylinder(radius=cooktop_radius, length=cooktop_thickness),
        origin=Origin(
            xyz=(
                cooktop_width / 2.0 - cooktop_radius,
                -cooktop_depth / 2.0 + cooktop_radius,
                cooktop_center_z,
            )
        ),
        material=enamel_ivory,
        name="cooktop_right_corner",
    )
    body.visual(
        Box((body_width - 0.10, body_depth - 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, foot_height + 0.01)),
        material=dark_trim,
        name="base_frame",
    )
    body.visual(
        Box((0.66, 0.54, 0.008)),
        origin=Origin(xyz=(0.0, -0.01, cooktop_top_z + 0.004)),
        material=cooktop_steel,
        name="cooktop_deck",
    )
    body.visual(
        Box((0.66, 0.075, 0.105)),
        origin=Origin(xyz=(0.0, -0.3025, 0.7075)),
        material=enamel_ivory,
        name="control_shelf",
    )
    body.visual(
        Cylinder(radius=0.0375, length=0.105),
        origin=Origin(xyz=(-0.33, -0.3025, 0.7075)),
        material=enamel_ivory,
        name="control_shelf_left_cap",
    )
    body.visual(
        Cylinder(radius=0.0375, length=0.105),
        origin=Origin(xyz=(0.33, -0.3025, 0.7075)),
        material=enamel_ivory,
        name="control_shelf_right_cap",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.60),
        origin=Origin(xyz=(0.0, -0.332, 0.752), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="control_shelf_front_trim",
    )

    cavity_width = opening_width - 0.02
    cavity_height = opening_height - 0.02
    body.visual(
        Box((panel_thickness, opening_depth, cavity_height)),
        origin=Origin(
            xyz=(
                -opening_width / 2.0 + panel_thickness / 2.0,
                front_face_y + opening_depth / 2.0,
                door_bottom_world + opening_height / 2.0,
            )
        ),
        material=dark_trim,
        name="oven_left_wall",
    )
    body.visual(
        Box((panel_thickness, opening_depth, cavity_height)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 - panel_thickness / 2.0,
                front_face_y + opening_depth / 2.0,
                door_bottom_world + opening_height / 2.0,
            )
        ),
        material=dark_trim,
        name="oven_right_wall",
    )
    body.visual(
        Box((cavity_width, 0.44, 0.012)),
        origin=Origin(xyz=(0.0, -0.07, door_bottom_world + 0.006)),
        material=dark_trim,
        name="oven_floor",
    )
    body.visual(
        Box((cavity_width, 0.01, opening_height - 0.08)),
        origin=Origin(
            xyz=(
                0.0,
                front_face_y + opening_depth - 0.005,
                door_bottom_world + (opening_height - 0.08) / 2.0,
            )
        ),
        material=dark_trim,
        name="oven_back_panel",
    )
    body.visual(
        Box((cavity_width, 0.30, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, door_bottom_world + opening_height - 0.005)),
        material=dark_trim,
        name="oven_ceiling",
    )

    foot_positions = (
        (-0.29, -0.20),
        (0.29, -0.20),
        (-0.29, 0.20),
        (0.29, 0.20),
    )
    for index, (x_pos, y_pos) in enumerate(foot_positions, start=1):
        body.visual(
            Cylinder(radius=0.03, length=foot_height),
            origin=Origin(xyz=(x_pos, y_pos, foot_height / 2.0)),
            material=chrome,
            name=f"foot_{index}",
        )

    add_burner(body, "front_left", -0.18, -0.12)
    add_burner(body, "front_right", 0.18, -0.12)
    add_burner(body, "rear_left", -0.18, 0.12)
    add_burner(body, "rear_right", 0.18, 0.12)

    door = model.part("oven_door")
    stile_width = 0.10
    top_rail_height = 0.10
    bottom_rail_height = 0.18
    glass_width = door_width - 2.0 * stile_width
    glass_height = door_height - top_rail_height - bottom_rail_height

    door.visual(
        Box((stile_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(-door_width / 2.0 + stile_width / 2.0, -door_thickness / 2.0, door_height / 2.0)
        ),
        material=enamel_cream,
        name="door_left_stile",
    )
    door.visual(
        Box((stile_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(door_width / 2.0 - stile_width / 2.0, -door_thickness / 2.0, door_height / 2.0)
        ),
        material=enamel_cream,
        name="door_right_stile",
    )
    door.visual(
        Box((door_width, door_thickness, bottom_rail_height)),
        origin=Origin(xyz=(0.0, -door_thickness / 2.0, bottom_rail_height / 2.0)),
        material=enamel_cream,
        name="door_bottom_rail",
    )
    door.visual(
        Box((door_width, door_thickness, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                -door_thickness / 2.0,
                door_height - top_rail_height / 2.0,
            )
        ),
        material=enamel_cream,
        name="door_top_rail",
    )
    door.visual(
        Box((glass_width, 0.010, glass_height)),
        origin=Origin(
            xyz=(
                0.0,
                -door_thickness * 0.56,
                bottom_rail_height + glass_height / 2.0,
            )
        ),
        material=glass,
        name="window_glass",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(-0.21, -0.058, 0.41), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_post_left",
    )
    door.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(0.21, -0.058, 0.41), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_post_right",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.422),
        origin=Origin(xyz=(0.0, -0.070, 0.41), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_handle",
    )
    door.visual(
        Box((glass_width + 0.040, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.038, bottom_rail_height + glass_height + 0.010)),
        material=chrome,
        name="window_trim_top",
    )
    door.visual(
        Box((glass_width + 0.040, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, -0.038, bottom_rail_height - 0.010)),
        material=chrome,
        name="window_trim_bottom",
    )
    door.visual(
        Box((0.020, 0.006, glass_height)),
        origin=Origin(xyz=(-(glass_width + 0.020) / 2.0, -0.038, bottom_rail_height + glass_height / 2.0)),
        material=chrome,
        name="window_trim_left",
    )
    door.visual(
        Box((0.020, 0.006, glass_height)),
        origin=Origin(xyz=((glass_width + 0.020) / 2.0, -0.038, bottom_rail_height + glass_height / 2.0)),
        material=chrome,
        name="window_trim_right",
    )

    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, front_face_y, door_bottom_world)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(86.0),
        ),
    )

    knob_x_positions = (-0.24, -0.08, 0.08, 0.24)
    for index, x_pos in enumerate(knob_x_positions, start=1):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.035, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="knob_bezel",
        )
        knob.visual(
            Cylinder(radius=0.032, length=0.045),
            origin=Origin(xyz=(0.0, -0.0225, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name="knob_body",
        )
        knob.visual(
            Box((0.006, 0.003, 0.016)),
            origin=Origin(xyz=(0.0, -0.0435, 0.018)),
            material=brass,
            name="knob_marker",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_pos, -0.34, 0.7075)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    door = object_model.get_part("oven_door")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 5)]
    door_joint = object_model.get_articulation("body_to_oven_door")
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(1, 5)]

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

    body_aabb = ctx.part_world_aabb(body)
    assert body_aabb is not None
    body_size = tuple(body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3))
    ctx.check(
        "stove_realistic_width",
        0.72 <= body_size[0] <= 0.82,
        details=f"Expected stove width near 0.76 m, got {body_size[0]:.3f} m.",
    )
    ctx.check(
        "stove_realistic_depth",
        0.60 <= body_size[1] <= 0.70,
        details=f"Expected stove depth near 0.66 m, got {body_size[1]:.3f} m.",
    )
    ctx.check(
        "stove_realistic_height",
        0.82 <= body_size[2] <= 0.88,
        details=f"Expected stove height near 0.84 m, got {body_size[2]:.3f} m.",
    )

    shelf_aabb = ctx.part_element_world_aabb(body, elem="control_shelf")
    cooktop_aabb = ctx.part_element_world_aabb(body, elem="cooktop_lip")
    assert shelf_aabb is not None
    assert cooktop_aabb is not None
    ctx.check(
        "control_shelf_below_cooktop",
        shelf_aabb[1][2] < cooktop_aabb[0][2] + 1e-6,
        details="Control shelf should terminate below the cooktop lip.",
    )

    with ctx.pose({door_joint: 0.0}):
        ctx.expect_contact(door, body, name="door_closed_contact")
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="front_upper_fascia",
            max_gap=0.002,
            max_penetration=0.0,
            name="door_closed_front_gap",
        )
        ctx.expect_overlap(door, body, axes="x", min_overlap=0.54, name="door_spans_oven_opening")
        ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="door_closed_no_floating")

        closed_door_aabb = ctx.part_world_aabb(door)
        closed_window_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
        assert closed_door_aabb is not None
        assert closed_window_aabb is not None
        door_center_x = (closed_door_aabb[0][0] + closed_door_aabb[1][0]) / 2.0
        window_center_x = (closed_window_aabb[0][0] + closed_window_aabb[1][0]) / 2.0
        window_center_z = (closed_window_aabb[0][2] + closed_window_aabb[1][2]) / 2.0
        door_center_z = (closed_door_aabb[0][2] + closed_door_aabb[1][2]) / 2.0
        ctx.check(
            "oven_window_centered_horizontally",
            abs(window_center_x - door_center_x) <= 0.01,
            details="Oven window should be centered within the broad door.",
        )
        ctx.check(
            "oven_window_centered_vertically",
            abs(window_center_z - door_center_z) <= 0.06,
            details="Oven window should sit near the vertical center of the door.",
        )

    limits = door_joint.motion_limits
    assert limits is not None
    assert limits.upper is not None
    with ctx.pose({door_joint: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_contact(door, body, name="door_open_hinge_contact")
        open_door_aabb = ctx.part_world_aabb(door)
        assert open_door_aabb is not None
        ctx.check(
            "door_drops_down_when_open",
            open_door_aabb[1][2] < 0.22,
            details="Open oven door should rotate down to a near-horizontal serving position.",
        )

    for index, (knob, joint) in enumerate(zip(knobs, knob_joints, strict=True), start=1):
        ctx.expect_contact(knob, body, name=f"knob_{index}_mounted")
        knob_aabb = ctx.part_world_aabb(knob)
        assert knob_aabb is not None
        knob_center_z = (knob_aabb[0][2] + knob_aabb[1][2]) / 2.0
        ctx.check(
            f"knob_{index}_on_control_shelf",
            shelf_aabb[0][2] <= knob_center_z <= shelf_aabb[1][2],
            details=f"Knob {index} should lie on the raised front control shelf.",
        )
        ctx.check(
            f"knob_{index}_ahead_of_shelf_face",
            knob_aabb[0][1] < shelf_aabb[0][1],
            details=f"Knob {index} should protrude forward of the shelf face.",
        )
        ctx.check(
            f"knob_{index}_continuous_axis",
            joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"Knob {index} should rotate continuously about the front-to-back axis.",
        )
        knob_limits = joint.motion_limits
        ctx.check(
            f"knob_{index}_continuous_limits",
            knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
            details=f"Knob {index} should be continuous with no finite stops.",
        )

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    assert all(position is not None for position in knob_positions)
    knob_positions = [position for position in knob_positions if position is not None]
    ctx.check(
        "knob_row_ordered_left_to_right",
        knob_positions[0][0] < knob_positions[1][0] < knob_positions[2][0] < knob_positions[3][0],
        details="Knobs should form one evenly ordered front control row.",
    )

    burner_names = (
        "burner_front_left_cap",
        "burner_front_right_cap",
        "burner_rear_left_cap",
        "burner_rear_right_cap",
    )
    burner_aabbs = [ctx.part_element_world_aabb(body, elem=name) for name in burner_names]
    assert all(aabb is not None for aabb in burner_aabbs)
    burner_centers = [
        (
            (aabb[0][0] + aabb[1][0]) / 2.0,
            (aabb[0][1] + aabb[1][1]) / 2.0,
            (aabb[0][2] + aabb[1][2]) / 2.0,
        )
        for aabb in burner_aabbs
        if aabb is not None
    ]
    ctx.check(
        "burners_form_two_rows",
        burner_centers[0][1] < burner_centers[2][1] - 0.18
        and burner_centers[1][1] < burner_centers[3][1] - 0.18,
        details="The four burners should read as front and rear rows.",
    )
    ctx.check(
        "burners_are_left_right_symmetric",
        abs(burner_centers[0][0] + burner_centers[1][0]) <= 0.01
        and abs(burner_centers[2][0] + burner_centers[3][0]) <= 0.01,
        details="Burners should be symmetrically arranged about the stove centerline.",
    )

    ctx.check(
        "door_hinge_axis_is_horizontal",
        door_joint.articulation_type == ArticulationType.REVOLUTE and tuple(door_joint.axis) == (1.0, 0.0, 0.0),
        details="The oven door must open downward around a left-to-right horizontal hinge.",
    )
    ctx.check(
        "door_hinge_limits_realistic",
        limits.lower == 0.0 and 1.45 <= limits.upper <= 1.55,
        details="Retro oven door should open downward to about 85 degrees.",
    )

    with ctx.pose(
        {
            knob_joints[0]: math.pi / 2.0,
            knob_joints[1]: math.pi,
            knob_joints[2]: 3.0 * math.pi / 2.0,
            knob_joints[3]: math.pi / 4.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_rotation_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_rotation_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
