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

WIDTH = 0.91
DEPTH = 0.70
BODY_HEIGHT = 0.91
SIDE_THICKNESS = 0.03
BACK_THICKNESS = 0.02
TOP_THICKNESS = 0.012
FRONT_FRAME_THICKNESS = 0.028

CONTROL_HEIGHT = 0.118
OPENING_BOTTOM = 0.112
OPENING_HEIGHT = 0.598
OPENING_WIDTH = 0.790
VENT_HEIGHT = BODY_HEIGHT - TOP_THICKNESS - CONTROL_HEIGHT - OPENING_BOTTOM - OPENING_HEIGHT

DOOR_WIDTH = 0.778
DOOR_HEIGHT = 0.578
DOOR_THICKNESS = 0.022
DOOR_HINGE_Y = -DEPTH / 2.0 - 0.012
DOOR_HINGE_Z = OPENING_BOTTOM + 0.002
DOOR_OPEN_LIMIT = 1.52

KNOB_Z = BODY_HEIGHT - TOP_THICKNESS - CONTROL_HEIGHT * 0.50
KNOB_JOINT_Y = -DEPTH / 2.0 + 0.004
KNOB_XS = (-0.315, -0.189, -0.063, 0.063, 0.189, 0.315)

BURNER_XS = (-0.270, 0.000, 0.270)
BURNER_YS = (-0.150, 0.110)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_gas_range", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.77, 0.79, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.58, 0.60, 0.63, 1.0))
    enamel_black = model.material("enamel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.18, 0.18, 1.0))
    burner_black = model.material("burner_black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass_black = model.material("glass_black", rgba=(0.12, 0.14, 0.16, 0.75))
    indicator_red = model.material("indicator_red", rgba=(0.80, 0.18, 0.14, 1.0))

    chassis = model.part("chassis")

    chassis.visual(
        Box((SIDE_THICKNESS, DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + SIDE_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=stainless,
        name="left_side",
    )
    chassis.visual(
        Box((SIDE_THICKNESS, DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - SIDE_THICKNESS / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=stainless,
        name="right_side",
    )
    chassis.visual(
        Box((WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, DEPTH / 2.0 - BACK_THICKNESS / 2.0, BODY_HEIGHT / 2.0)
        ),
        material=stainless,
        name="back_panel",
    )
    chassis.visual(
        Box((WIDTH, DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - TOP_THICKNESS / 2.0)),
        material=enamel_black,
        name="cooktop_deck",
    )

    front_frame_y = -DEPTH / 2.0 + FRONT_FRAME_THICKNESS / 2.0
    front_frame_width = (WIDTH - OPENING_WIDTH) / 2.0
    vent_center_z = OPENING_BOTTOM + OPENING_HEIGHT + VENT_HEIGHT / 2.0

    chassis.visual(
        Box((WIDTH - 2.0 * SIDE_THICKNESS, FRONT_FRAME_THICKNESS, CONTROL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, front_frame_y, BODY_HEIGHT - TOP_THICKNESS - CONTROL_HEIGHT / 2.0)
        ),
        material=stainless,
        name="control_fascia",
    )
    chassis.visual(
        Box((front_frame_width, FRONT_FRAME_THICKNESS, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                -OPENING_WIDTH / 2.0 - front_frame_width / 2.0,
                front_frame_y,
                OPENING_BOTTOM + OPENING_HEIGHT / 2.0,
            )
        ),
        material=stainless,
        name="left_stile",
    )
    chassis.visual(
        Box((front_frame_width, FRONT_FRAME_THICKNESS, OPENING_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_WIDTH / 2.0 + front_frame_width / 2.0,
                front_frame_y,
                OPENING_BOTTOM + OPENING_HEIGHT / 2.0,
            )
        ),
        material=stainless,
        name="right_stile",
    )
    chassis.visual(
        Box((WIDTH - 2.0 * SIDE_THICKNESS, FRONT_FRAME_THICKNESS, OPENING_BOTTOM)),
        origin=Origin(xyz=(0.0, front_frame_y, OPENING_BOTTOM / 2.0)),
        material=dark_stainless,
        name="toe_kick",
    )
    chassis.visual(
        Box((WIDTH - 2.0 * SIDE_THICKNESS, FRONT_FRAME_THICKNESS, VENT_HEIGHT)),
        origin=Origin(xyz=(0.0, front_frame_y, vent_center_z)),
        material=stainless,
        name="oven_vent_strip",
    )

    for vent_index, vent_x in enumerate((-0.26, -0.13, 0.0, 0.13, 0.26), start=1):
        chassis.visual(
            Box((0.090, 0.006, 0.008)),
            origin=Origin(xyz=(vent_x, -DEPTH / 2.0 + 0.003, vent_center_z)),
            material=enamel_black,
            name=f"vent_slot_{vent_index}",
        )

    cavity_width = OPENING_WIDTH
    cavity_depth = DEPTH - FRONT_FRAME_THICKNESS - BACK_THICKNESS
    cavity_front_y = -DEPTH / 2.0 + FRONT_FRAME_THICKNESS
    cavity_wall_thickness = 0.018
    cavity_height = 0.548
    cavity_floor_z = 0.134
    cavity_center_y = cavity_front_y + cavity_depth / 2.0

    chassis.visual(
        Box((cavity_wall_thickness, cavity_depth, cavity_height)),
        origin=Origin(
            xyz=(
                -cavity_width / 2.0 + cavity_wall_thickness / 2.0,
                cavity_center_y,
                cavity_floor_z + cavity_height / 2.0,
            )
        ),
        material=enamel_black,
        name="oven_left_wall",
    )
    chassis.visual(
        Box((cavity_wall_thickness, cavity_depth, cavity_height)),
        origin=Origin(
            xyz=(
                cavity_width / 2.0 - cavity_wall_thickness / 2.0,
                cavity_center_y,
                cavity_floor_z + cavity_height / 2.0,
            )
        ),
        material=enamel_black,
        name="oven_right_wall",
    )
    chassis.visual(
        Box((cavity_width, cavity_wall_thickness, cavity_height)),
        origin=Origin(
            xyz=(
                0.0,
                cavity_front_y + cavity_depth - cavity_wall_thickness / 2.0,
                cavity_floor_z + cavity_height / 2.0,
            )
        ),
        material=enamel_black,
        name="oven_back_wall",
    )
    chassis.visual(
        Box((cavity_width, cavity_depth, cavity_wall_thickness)),
        origin=Origin(
            xyz=(0.0, cavity_center_y, cavity_floor_z + cavity_wall_thickness / 2.0)
        ),
        material=enamel_black,
        name="oven_floor",
    )
    chassis.visual(
        Box((cavity_width, cavity_depth, cavity_wall_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cavity_center_y,
                cavity_floor_z + cavity_height - cavity_wall_thickness / 2.0,
            )
        ),
        material=enamel_black,
        name="oven_ceiling",
    )

    for burner_index, (burner_y, burner_x) in enumerate(
        ((y, x) for y in BURNER_YS for x in BURNER_XS),
        start=1,
    ):
        chassis.visual(
            Cylinder(radius=0.072, length=0.004),
            origin=Origin(xyz=(burner_x, burner_y, BODY_HEIGHT + 0.002)),
            material=burner_black,
            name=f"burner_{burner_index}_pan",
        )
        chassis.visual(
            Cylinder(radius=0.052, length=0.010),
            origin=Origin(xyz=(burner_x, burner_y, BODY_HEIGHT + 0.007)),
            material=cast_iron,
            name=f"burner_{burner_index}_ring",
        )
        chassis.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(burner_x, burner_y, BODY_HEIGHT + 0.014)),
            material=burner_black,
            name=f"burner_{burner_index}_cap",
        )

    for knob_index, knob_x in enumerate(KNOB_XS, start=1):
        chassis.visual(
            Cylinder(radius=0.039, length=0.004),
            origin=Origin(
                xyz=(knob_x, -DEPTH / 2.0 + 0.002, KNOB_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_stainless,
            name=f"knob_bezel_{knob_index}",
        )
        chassis.visual(
            Box((0.010, 0.002, 0.020)),
            origin=Origin(xyz=(knob_x, -DEPTH / 2.0 + 0.001, KNOB_Z + 0.028)),
            material=indicator_red,
            name=f"knob_mark_{knob_index}",
        )

    for hinge_index, hinge_x in enumerate((-0.305, 0.305), start=1):
        chassis.visual(
            Box((0.050, 0.012, 0.021)),
            origin=Origin(xyz=(hinge_x, -0.356, 0.1015)),
            material=dark_stainless,
            name=f"hinge_support_{hinge_index}",
        )
        chassis.visual(
            Cylinder(radius=0.009, length=0.052),
            origin=Origin(
                xyz=(hinge_x, DOOR_HINGE_Y, DOOR_HINGE_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cast_iron,
            name=f"hinge_sleeve_{hinge_index}",
        )

    def add_grate(name: str, center_x: float) -> None:
        grate = model.part(name)
        grate_width = 0.255
        grate_depth = 0.408
        rail = 0.018
        rail_height = 0.014
        leg_height = 0.024
        top_z = leg_height + rail_height / 2.0

        grate.visual(
            Box((grate_width, rail, rail_height)),
            origin=Origin(xyz=(0.0, -grate_depth / 2.0 + rail / 2.0, top_z)),
            material=cast_iron,
            name="front_rail",
        )
        grate.visual(
            Box((grate_width, rail, rail_height)),
            origin=Origin(xyz=(0.0, grate_depth / 2.0 - rail / 2.0, top_z)),
            material=cast_iron,
            name="rear_rail",
        )
        grate.visual(
            Box((rail, grate_depth, rail_height)),
            origin=Origin(xyz=(-grate_width / 2.0 + rail / 2.0, 0.0, top_z)),
            material=cast_iron,
            name="left_rail",
        )
        grate.visual(
            Box((rail, grate_depth, rail_height)),
            origin=Origin(xyz=(grate_width / 2.0 - rail / 2.0, 0.0, top_z)),
            material=cast_iron,
            name="right_rail",
        )
        grate.visual(
            Box((grate_width * 0.92, rail, rail_height)),
            origin=Origin(xyz=(0.0, 0.0, top_z)),
            material=cast_iron,
            name="center_bar",
        )
        for rib_index, rib_x in enumerate((-0.072, 0.072), start=1):
            grate.visual(
                Box((rail, grate_depth * 0.88, rail_height)),
                origin=Origin(xyz=(rib_x, 0.0, top_z)),
                material=cast_iron,
                name=f"rib_{rib_index}",
            )
        for leg_name, leg_x, leg_y in (
            ("front_left_leg", -grate_width / 2.0 + 0.018, -grate_depth / 2.0 + 0.018),
            ("front_right_leg", grate_width / 2.0 - 0.018, -grate_depth / 2.0 + 0.018),
            ("rear_left_leg", -grate_width / 2.0 + 0.018, grate_depth / 2.0 - 0.018),
            ("rear_right_leg", grate_width / 2.0 - 0.018, grate_depth / 2.0 - 0.018),
        ):
            grate.visual(
                Box((0.018, 0.018, leg_height)),
                origin=Origin(xyz=(leg_x, leg_y, leg_height / 2.0)),
                material=cast_iron,
                name=leg_name,
            )

        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.FIXED,
            parent=chassis,
            child=grate,
            origin=Origin(xyz=(center_x, -0.020, BODY_HEIGHT)),
        )

    add_grate("grate_left", BURNER_XS[0])
    add_grate("grate_center", BURNER_XS[1])
    add_grate("grate_right", BURNER_XS[2])

    for knob_index, knob_x in enumerate(KNOB_XS, start=1):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.033, length=0.030),
            origin=Origin(
                xyz=(0.0, -0.015, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=enamel_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.023, length=0.020),
            origin=Origin(
                xyz=(0.0, -0.010, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_stainless,
            name="knob_face",
        )
        knob.visual(
            Box((0.006, 0.004, 0.020)),
            origin=Origin(xyz=(0.0, -0.031, 0.012)),
            material=indicator_red,
            name="indicator",
        )
        model.articulation(
            f"chassis_to_knob_{knob_index}",
            ArticulationType.CONTINUOUS,
            parent=chassis,
            child=knob,
            origin=Origin(xyz=(knob_x, KNOB_JOINT_Y, KNOB_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=8.0),
        )

    oven_door = model.part("oven_door")
    oven_door.visual(
        Cylinder(radius=0.007, length=DOOR_WIDTH * 0.82),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cast_iron,
        name="hinge_pin",
    )
    for hinge_index, hinge_x in enumerate((-0.305, 0.305), start=1):
        oven_door.visual(
            Box((0.050, 0.024, 0.020)),
            origin=Origin(xyz=(hinge_x, 0.006, 0.010)),
            material=dark_stainless,
            name=f"hinge_tab_{hinge_index}",
        )
    oven_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.023, DOOR_HEIGHT / 2.0 + 0.006)),
        material=stainless,
        name="door_panel",
    )
    oven_door.visual(
        Box((DOOR_WIDTH * 0.72, 0.012, DOOR_HEIGHT * 0.32)),
        origin=Origin(xyz=(0.0, 0.020, DOOR_HEIGHT * 0.61 + 0.006)),
        material=glass_black,
        name="door_glass",
    )
    oven_door.visual(
        Box((DOOR_WIDTH * 0.86, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, 0.018, DOOR_HEIGHT * 0.085 + 0.006)),
        material=enamel_black,
        name="door_lower_recess",
    )
    oven_door.visual(
        Cylinder(radius=0.011, length=DOOR_WIDTH * 0.74),
        origin=Origin(
            xyz=(0.0, -0.058, DOOR_HEIGHT * 0.24 + 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_stainless,
        name="handle_bar",
    )
    for handle_index, handle_x in enumerate((-0.250, 0.250), start=1):
        oven_door.visual(
            Box((0.024, 0.080, 0.028)),
            origin=Origin(xyz=(handle_x, -0.021, DOOR_HEIGHT * 0.24 + 0.006)),
            material=dark_stainless,
            name=f"handle_post_{handle_index}",
        )
    model.articulation(
        "chassis_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=oven_door,
        origin=Origin(xyz=(0.0, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=DOOR_OPEN_LIMIT,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    oven_door = object_model.get_part("oven_door")
    door_hinge = object_model.get_articulation("chassis_to_oven_door")

    grates = [
        object_model.get_part("grate_left"),
        object_model.get_part("grate_center"),
        object_model.get_part("grate_right"),
    ]
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 7)]
    knob_joints = [
        object_model.get_articulation(f"chassis_to_knob_{index}") for index in range(1, 7)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_sleeve_1",
        elem_b="hinge_pin",
        reason="The left oven hinge sleeve captures the rotating hinge pin.",
    )
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_sleeve_2",
        elem_b="hinge_pin",
        reason="The right oven hinge sleeve captures the rotating hinge pin.",
    )
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_support_1",
        elem_b="hinge_tab_1",
        reason="The left hinge knuckle nests inside the chassis support bracket at the pivot.",
    )
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_support_2",
        elem_b="hinge_tab_2",
        reason="The right hinge knuckle nests inside the chassis support bracket at the pivot.",
    )
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_sleeve_1",
        elem_b="hinge_tab_1",
        reason="The left door hinge tab rotates within the sleeve envelope.",
    )
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_sleeve_2",
        elem_b="hinge_tab_2",
        reason="The right door hinge tab rotates within the sleeve envelope.",
    )
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_support_1",
        elem_b="hinge_pin",
        reason="The simplified left hinge support stands in for a drilled bracket surrounding the hinge pin.",
    )
    ctx.allow_overlap(
        chassis,
        oven_door,
        elem_a="hinge_support_2",
        elem_b="hinge_pin",
        reason="The simplified right hinge support stands in for a drilled bracket surrounding the hinge pin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for burner_index in range(1, 7):
        chassis.get_visual(f"burner_{burner_index}_cap")
        chassis.get_visual(f"burner_{burner_index}_ring")
        chassis.get_visual(f"burner_{burner_index}_pan")

    chassis.get_visual("cooktop_deck")
    oven_door.get_visual("door_glass")
    oven_door.get_visual("handle_bar")

    for grate in grates:
        ctx.expect_contact(grate, chassis)

    burner_pairs = (
        ("grate_left", ("burner_1_cap", "burner_4_cap")),
        ("grate_center", ("burner_2_cap", "burner_5_cap")),
        ("grate_right", ("burner_3_cap", "burner_6_cap")),
    )
    for grate_name, burner_names in burner_pairs:
        for burner_name in burner_names:
            ctx.expect_overlap(
                grate_name,
                chassis,
                axes="xy",
                elem_b=burner_name,
                min_overlap=0.05,
                name=f"{grate_name}_{burner_name}_coverage",
            )

    for knob, joint in zip(knobs, knob_joints, strict=True):
        joint_name = joint.name
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"Expected CONTINUOUS articulation, got {joint.articulation_type!r}.",
        )
        ctx.check(
            f"{joint_name}_axis_is_front_to_back",
            joint.axis == (0.0, 1.0, 0.0),
            details=f"Expected axis (0, 1, 0), got {joint.axis!r}.",
        )
        ctx.check(
            f"{joint_name}_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            details="Continuous knob joints should not define lower or upper limits.",
        )
        knob_index = int(knob.name.split("_")[-1])
        ctx.expect_contact(
            knob,
            chassis,
            elem_a="knob_body",
            elem_b=f"knob_bezel_{knob_index}",
            name=f"{knob.name}_mounted_to_fascia",
        )

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    if all(position is not None for position in knob_positions):
        xs = [position[0] for position in knob_positions if position is not None]
        ys = [position[1] for position in knob_positions if position is not None]
        zs = [position[2] for position in knob_positions if position is not None]
        spacings = [xs[index + 1] - xs[index] for index in range(len(xs) - 1)]
        spacing_target = sum(spacings) / len(spacings)
        ctx.check(
            "knob_row_is_straight",
            max(abs(value - ys[0]) for value in ys) <= 1e-6
            and max(abs(value - zs[0]) for value in zs) <= 1e-6,
            details="Knob centers should form one straight horizontal row.",
        )
        ctx.check(
            "knob_spacing_is_even",
            max(abs(spacing - spacing_target) for spacing in spacings) <= 0.002,
            details=f"Knob x spacing varies too much: {spacings!r}",
        )

    indicator_rest = ctx.part_element_world_aabb(knobs[0], elem="indicator")
    with ctx.pose({knob_joints[0]: math.pi / 2.0}):
        indicator_turn = ctx.part_element_world_aabb(knobs[0], elem="indicator")
        ctx.expect_contact(
            knobs[0],
            chassis,
            elem_a="knob_body",
            elem_b="knob_bezel_1",
            name="knob_1_stays_mounted_when_rotated",
        )
    if indicator_rest is not None and indicator_turn is not None:
        rest_x = indicator_rest[1][0] - indicator_rest[0][0]
        rest_z = indicator_rest[1][2] - indicator_rest[0][2]
        turn_x = indicator_turn[1][0] - indicator_turn[0][0]
        turn_z = indicator_turn[1][2] - indicator_turn[0][2]
        ctx.check(
            "knob_1_indicator_rotates",
            rest_z > rest_x and turn_x > turn_z,
            details="The knob indicator should visibly rotate around the front-to-back axis.",
        )

    door_limits = door_hinge.motion_limits
    ctx.check(
        "oven_door_joint_axis_is_horizontal",
        door_hinge.axis == (1.0, 0.0, 0.0),
        details=f"Expected oven door hinge axis (1, 0, 0), got {door_hinge.axis!r}.",
    )
    ctx.check(
        "oven_door_joint_limits_are_present",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and door_limits.upper >= 1.45,
        details="The oven door should open downward from closed to roughly horizontal.",
    )

    handle_aabb = ctx.part_element_world_aabb(oven_door, elem="handle_bar")
    if handle_aabb is not None:
        handle_width = handle_aabb[1][0] - handle_aabb[0][0]
        handle_center_z = (handle_aabb[0][2] + handle_aabb[1][2]) / 2.0
        ctx.check(
            "oven_handle_is_full_width_lower_bar",
            handle_width >= DOOR_WIDTH * 0.65 and handle_center_z <= DOOR_HINGE_Z + DOOR_HEIGHT * 0.35,
            details="The oven door handle should span most of the door width and sit on the lower panel.",
        )

    door_panel_closed = ctx.part_element_world_aabb(oven_door, elem="door_panel")
    with ctx.pose({door_hinge: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="oven_door_closed_no_floating")

    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            door_panel_open = ctx.part_element_world_aabb(oven_door, elem="door_panel")
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_open_no_floating")
            if door_panel_closed is not None and door_panel_open is not None:
                closed_door_pos = (
                    (door_panel_closed[0][0] + door_panel_closed[1][0]) / 2.0,
                    (door_panel_closed[0][1] + door_panel_closed[1][1]) / 2.0,
                    (door_panel_closed[0][2] + door_panel_closed[1][2]) / 2.0,
                )
                open_door_pos = (
                    (door_panel_open[0][0] + door_panel_open[1][0]) / 2.0,
                    (door_panel_open[0][1] + door_panel_open[1][1]) / 2.0,
                    (door_panel_open[0][2] + door_panel_open[1][2]) / 2.0,
                )
                ctx.check(
                    "oven_door_swings_down",
                    open_door_pos[2] < closed_door_pos[2] - 0.22,
                    details=f"Door center did not drop enough: closed={closed_door_pos}, open={open_door_pos}.",
                )
                ctx.check(
                    "oven_door_swings_forward",
                    open_door_pos[1] < closed_door_pos[1] - 0.20,
                    details=f"Door center did not move outward enough: closed={closed_door_pos}, open={open_door_pos}.",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
