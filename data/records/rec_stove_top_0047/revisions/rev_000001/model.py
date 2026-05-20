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
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_kitchen_range", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.56, 0.58, 0.60, 1.0))
    enamel_black = model.material("enamel_black", rgba=(0.13, 0.13, 0.14, 1.0))
    grate_iron = model.material("grate_iron", rgba=(0.16, 0.16, 0.16, 1.0))
    burner_black = model.material("burner_black", rgba=(0.10, 0.10, 0.10, 1.0))
    glass_black = model.material("glass_black", rgba=(0.12, 0.14, 0.16, 0.55))
    knob_silver = model.material("knob_silver", rgba=(0.70, 0.72, 0.74, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.72, 0.16, 0.10, 1.0))
    cavity_dark = model.material("cavity_dark", rgba=(0.07, 0.07, 0.08, 1.0))

    width = 0.91
    depth = 0.69
    body_height = 0.91
    side_thickness = 0.025
    back_thickness = 0.020
    top_thickness = 0.030
    front_frame_depth = 0.040
    front_y = depth * 0.5
    back_y = -depth * 0.5

    door_width = 0.75
    door_height = 0.62
    door_thickness = 0.045
    door_bottom = 0.13
    door_axis_y = 0.345

    panel_depth = 0.08
    panel_height = 0.12
    panel_bottom = 0.76
    panel_top = panel_bottom + panel_height

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((width, depth, 1.00)),
        mass=108.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )
    chassis.visual(
        Box((width, depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, body_height - top_thickness * 0.5)),
        material=stainless,
        name="cooktop_deck",
    )
    chassis.visual(
        Box((side_thickness, depth, body_height - top_thickness)),
        origin=Origin(
            xyz=((width - side_thickness) * 0.5, 0.0, (body_height - top_thickness) * 0.5)
        ),
        material=stainless,
        name="right_side_panel",
    )
    chassis.visual(
        Box((side_thickness, depth, body_height - top_thickness)),
        origin=Origin(
            xyz=(-(width - side_thickness) * 0.5, 0.0, (body_height - top_thickness) * 0.5)
        ),
        material=stainless,
        name="left_side_panel",
    )
    chassis.visual(
        Box((width - 2.0 * side_thickness, back_thickness, body_height - top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                back_y + back_thickness * 0.5,
                (body_height - top_thickness) * 0.5,
            )
        ),
        material=dark_stainless,
        name="back_panel",
    )
    chassis.visual(
        Box((width - 2.0 * side_thickness, depth - 0.10, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, 0.010)),
        material=dark_stainless,
        name="bottom_floor",
    )
    chassis.visual(
        Box((width - 2.0 * side_thickness, front_frame_depth, 0.12)),
        origin=Origin(xyz=(0.0, front_y - front_frame_depth * 0.5, 0.06)),
        material=dark_stainless,
        name="toe_kick",
    )
    chassis.visual(
        Box((0.11, front_frame_depth, panel_bottom - 0.12)),
        origin=Origin(
            xyz=(
                (door_width + 0.11) * 0.5,
                front_y - front_frame_depth * 0.5,
                0.12 + (panel_bottom - 0.12) * 0.5,
            )
        ),
        material=stainless,
        name="right_front_stile",
    )
    chassis.visual(
        Box((0.11, front_frame_depth, panel_bottom - 0.12)),
        origin=Origin(
            xyz=(
                -(door_width + 0.11) * 0.5,
                front_y - front_frame_depth * 0.5,
                0.12 + (panel_bottom - 0.12) * 0.5,
            )
        ),
        material=stainless,
        name="left_front_stile",
    )
    chassis.visual(
        Box((width - 2.0 * side_thickness, 0.020, panel_height)),
        origin=Origin(
            xyz=(0.0, front_y - 0.080, panel_bottom + panel_height * 0.5)
        ),
        material=stainless,
        name="front_control_support",
    )
    chassis.visual(
        Box((width, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, back_y + 0.010, body_height + 0.025)),
        material=dark_stainless,
        name="rear_upstand",
    )

    cavity_width = 0.63
    cavity_front = 0.305
    cavity_back = -0.28
    cavity_floor_top = 0.18
    cavity_ceiling_bottom = 0.73
    liner_thickness = 0.015
    cavity_depth = cavity_front - cavity_back
    cavity_height = cavity_ceiling_bottom - cavity_floor_top
    cavity_center_y = (cavity_front + cavity_back) * 0.5
    cavity_center_z = (cavity_floor_top + cavity_ceiling_bottom) * 0.5
    chassis.visual(
        Box((cavity_width + 2.0 * liner_thickness, cavity_depth, liner_thickness)),
        origin=Origin(
            xyz=(0.0, cavity_center_y, cavity_floor_top + liner_thickness * 0.5)
        ),
        material=cavity_dark,
        name="oven_floor",
    )
    chassis.visual(
        Box((cavity_width + 2.0 * liner_thickness, cavity_depth, liner_thickness)),
        origin=Origin(
            xyz=(0.0, cavity_center_y, cavity_ceiling_bottom - liner_thickness * 0.5)
        ),
        material=cavity_dark,
        name="oven_ceiling",
    )
    chassis.visual(
        Box((liner_thickness, cavity_depth, cavity_height)),
        origin=Origin(
            xyz=(
                cavity_width * 0.5 + liner_thickness * 0.5,
                cavity_center_y,
                cavity_center_z,
            )
        ),
        material=cavity_dark,
        name="oven_right_wall",
    )
    chassis.visual(
        Box((liner_thickness, cavity_depth, cavity_height)),
        origin=Origin(
            xyz=(
                -(cavity_width * 0.5 + liner_thickness * 0.5),
                cavity_center_y,
                cavity_center_z,
            )
        ),
        material=cavity_dark,
        name="oven_left_wall",
    )
    chassis.visual(
        Box((cavity_width + 2.0 * liner_thickness, liner_thickness, cavity_height)),
        origin=Origin(
            xyz=(0.0, cavity_back + liner_thickness * 0.5, cavity_center_z)
        ),
        material=cavity_dark,
        name="oven_back_wall",
    )
    chassis.visual(
        Box((0.045, 0.020, cavity_height)),
        origin=Origin(
            xyz=(
                cavity_width * 0.5 + liner_thickness + 0.0225,
                cavity_front + 0.010,
                cavity_center_z,
            )
        ),
        material=cavity_dark,
        name="oven_right_front_flange",
    )
    chassis.visual(
        Box((0.045, 0.020, cavity_height)),
        origin=Origin(
            xyz=(
                -(cavity_width * 0.5 + liner_thickness + 0.0225),
                cavity_front + 0.010,
                cavity_center_z,
            )
        ),
        material=cavity_dark,
        name="oven_left_front_flange",
    )
    chassis.visual(
        Box((0.63, 0.44, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, 0.39)),
        material=dark_stainless,
        name="oven_rack",
    )

    burner_x_positions = (-0.26, 0.0, 0.26)
    burner_y_positions = (0.13, -0.10)
    burner_index = 1
    for y_pos in burner_y_positions:
        for x_pos in burner_x_positions:
            chassis.visual(
                Cylinder(radius=0.060, length=0.010),
                origin=Origin(xyz=(x_pos, y_pos, body_height + 0.005)),
                material=burner_black,
                name=f"burner_{burner_index}_base",
            )
            chassis.visual(
                Cylinder(radius=0.042, length=0.010),
                origin=Origin(xyz=(x_pos, y_pos, body_height + 0.015)),
                material=burner_black,
                name=f"burner_{burner_index}_cap",
            )
            burner_index += 1

    chassis.visual(
        Box((0.68, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.333, door_bottom,)),
        material=dark_stainless,
        name="door_hinge_cradle",
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((width, 0.014, panel_height)),
        origin=Origin(xyz=(0.0, 0.007, panel_height * 0.5)),
        material=stainless,
        name="panel_back",
    )
    control_panel.visual(
        Box((width, panel_depth, 0.012)),
        origin=Origin(xyz=(0.0, panel_depth * 0.5, panel_height - 0.006)),
        material=stainless,
        name="panel_top",
    )
    control_panel.visual(
        Box((width, panel_depth, 0.014)),
        origin=Origin(xyz=(0.0, panel_depth * 0.5, 0.007)),
        material=stainless,
        name="panel_bottom",
    )
    control_panel.visual(
        Box((0.018, panel_depth, panel_height)),
        origin=Origin(xyz=((width - 0.018) * 0.5, panel_depth * 0.5, panel_height * 0.5)),
        material=stainless,
        name="right_endcap",
    )
    control_panel.visual(
        Box((0.018, panel_depth, panel_height)),
        origin=Origin(xyz=(-(width - 0.018) * 0.5, panel_depth * 0.5, panel_height * 0.5)),
        material=stainless,
        name="left_endcap",
    )
    control_panel.visual(
        Box((width - 0.050, 0.004, 0.096)),
        origin=Origin(xyz=(0.0, panel_depth - 0.002, panel_height * 0.5)),
        material=enamel_black,
        name="panel_fascia",
    )
    button_x = 0.398
    control_panel.visual(
        Box((0.022, 0.022, 0.022)),
        origin=Origin(xyz=(button_x, 0.069, 0.074)),
        material=dark_stainless,
        name="button_guide",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((width, panel_depth, panel_height)),
        mass=4.5,
        origin=Origin(xyz=(0.0, panel_depth * 0.5, panel_height * 0.5)),
    )
    model.articulation(
        "chassis_to_control_panel",
        ArticulationType.FIXED,
        parent=chassis,
        child=control_panel,
        origin=Origin(xyz=(0.0, front_y, panel_bottom)),
    )

    def add_grate(name: str, center_x: float) -> None:
        grate = model.part(name)
        bar_z = 0.941
        foot_z = 0.921
        grate_width = 0.26
        grate_depth = 0.53
        bar_thickness = 0.018
        foot_size = 0.022

        grate.visual(
            Box((grate_width, bar_thickness, bar_thickness)),
            origin=Origin(xyz=(0.0, grate_depth * 0.5 - bar_thickness * 0.5, bar_z)),
            material=grate_iron,
            name="front_bar",
        )
        grate.visual(
            Box((grate_width, bar_thickness, bar_thickness)),
            origin=Origin(xyz=(0.0, -(grate_depth * 0.5 - bar_thickness * 0.5), bar_z)),
            material=grate_iron,
            name="rear_bar",
        )
        grate.visual(
            Box((bar_thickness, grate_depth, bar_thickness)),
            origin=Origin(xyz=(grate_width * 0.5 - bar_thickness * 0.5, 0.0, bar_z)),
            material=grate_iron,
            name="right_bar",
        )
        grate.visual(
            Box((bar_thickness, grate_depth, bar_thickness)),
            origin=Origin(xyz=(-(grate_width * 0.5 - bar_thickness * 0.5), 0.0, bar_z)),
            material=grate_iron,
            name="left_bar",
        )
        grate.visual(
            Box((bar_thickness, grate_depth - 2.0 * bar_thickness, bar_thickness)),
            origin=Origin(xyz=(0.0, 0.0, bar_z)),
            material=grate_iron,
            name="center_bar",
        )
        grate.visual(
            Box((grate_width - 2.0 * bar_thickness, bar_thickness, bar_thickness)),
            origin=Origin(xyz=(0.0, 0.130, bar_z)),
            material=grate_iron,
            name="upper_crossbar",
        )
        grate.visual(
            Box((grate_width - 2.0 * bar_thickness, bar_thickness, bar_thickness)),
            origin=Origin(xyz=(0.0, -0.130, bar_z)),
            material=grate_iron,
            name="lower_crossbar",
        )
        for foot_name, local_x, local_y in (
            (
                "front_right_foot",
                grate_width * 0.5 - foot_size * 0.5,
                grate_depth * 0.5 - foot_size * 0.5,
            ),
            (
                "front_left_foot",
                -(grate_width * 0.5 - foot_size * 0.5),
                grate_depth * 0.5 - foot_size * 0.5,
            ),
            (
                "rear_right_foot",
                grate_width * 0.5 - foot_size * 0.5,
                -(grate_depth * 0.5 - foot_size * 0.5),
            ),
            (
                "rear_left_foot",
                -(grate_width * 0.5 - foot_size * 0.5),
                -(grate_depth * 0.5 - foot_size * 0.5),
            ),
        ):
            grate.visual(
                Box((foot_size, foot_size, 0.022)),
                origin=Origin(xyz=(local_x, local_y, foot_z)),
                material=grate_iron,
                name=foot_name,
            )
        grate.inertial = Inertial.from_geometry(
            Box((grate_width, grate_depth, 0.06)),
            mass=6.0,
            origin=Origin(xyz=(0.0, 0.0, 0.93)),
        )
        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.FIXED,
            parent=chassis,
            child=grate,
            origin=Origin(xyz=(center_x, 0.015, 0.0)),
        )

    add_grate("left_grate", -0.26)
    add_grate("center_grate", 0.0)
    add_grate("right_grate", 0.26)

    knob_x_positions = (-0.30, -0.18, -0.06, 0.06, 0.18, 0.30)
    knob_z = 0.073

    def add_knob(index: int, center_x: float) -> None:
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.030, length=0.010),
            origin=Origin(
                xyz=(0.0, 0.005, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=dark_stainless,
            name="back_plate",
        )
        knob.visual(
            Cylinder(radius=0.028, length=0.032),
            origin=Origin(
                xyz=(0.0, 0.016, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_silver,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.004, 0.018)),
            origin=Origin(xyz=(0.0, 0.031, 0.013)),
            material=indicator_red,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.060, 0.040, 0.060)),
            mass=0.18,
            origin=Origin(xyz=(0.0, 0.016, 0.0)),
        )
        model.articulation(
            f"control_panel_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=control_panel,
            child=knob,
            origin=Origin(xyz=(center_x, panel_depth, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=10.0),
        )

    for knob_index, knob_x in enumerate(knob_x_positions, start=1):
        add_knob(knob_index, knob_x)

    oven_light_button = model.part("oven_light_button")
    oven_light_button.visual(
        Box((0.010, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=dark_stainless,
        name="button_stem",
    )
    oven_light_button.visual(
        Box((0.018, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        material=knob_silver,
        name="button_cap",
    )
    oven_light_button.inertial = Inertial.from_geometry(
        Box((0.018, 0.030, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
    )
    model.articulation(
        "control_panel_to_oven_light_button",
        ArticulationType.PRISMATIC,
        parent=control_panel,
        child=oven_light_button,
        origin=Origin(xyz=(button_x, panel_depth + 0.001, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    oven_door = model.part("oven_door")
    oven_door.visual(
        Cylinder(radius=0.008, length=0.62),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_stainless,
        name="hinge_rod",
    )
    oven_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(0.0, door_thickness * 0.5, door_height * 0.5)),
        material=stainless,
        name="door_panel",
    )
    oven_door.visual(
        Box((0.54, 0.006, 0.26)),
        origin=Origin(xyz=(0.0, door_thickness - 0.003, 0.37)),
        material=glass_black,
        name="door_window",
    )
    oven_door.visual(
        Box((0.032, 0.026, 0.060)),
        origin=Origin(xyz=(0.275, 0.057, 0.52)),
        material=dark_stainless,
        name="right_handle_post",
    )
    oven_door.visual(
        Box((0.032, 0.026, 0.060)),
        origin=Origin(xyz=(-0.275, 0.057, 0.52)),
        material=dark_stainless,
        name="left_handle_post",
    )
    oven_door.visual(
        Cylinder(radius=0.012, length=0.58),
        origin=Origin(
            xyz=(0.0, 0.080, 0.52),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=dark_stainless,
        name="handle_bar",
    )
    oven_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.090, door_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.045, door_height * 0.5)),
    )
    model.articulation(
        "chassis_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=oven_door,
        origin=Origin(xyz=(0.0, door_axis_y, door_bottom)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    control_panel = object_model.get_part("control_panel")
    left_grate = object_model.get_part("left_grate")
    center_grate = object_model.get_part("center_grate")
    right_grate = object_model.get_part("right_grate")
    oven_door = object_model.get_part("oven_door")
    oven_light_button = object_model.get_part("oven_light_button")
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 7)]

    door_joint = object_model.get_articulation("chassis_to_oven_door")
    button_joint = object_model.get_articulation("control_panel_to_oven_light_button")
    knob_joints = [
        object_model.get_articulation(f"control_panel_to_knob_{index}")
        for index in range(1, 7)
    ]

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
    ctx.allow_overlap(
        oven_light_button,
        control_panel,
        elem_a="button_stem",
        elem_b="button_guide",
        reason="The oven-light plunger rides inside a fixed guide sleeve in the control panel.",
    )
    ctx.allow_overlap(
        oven_door,
        chassis,
        elem_a="hinge_rod",
        elem_b="door_hinge_cradle",
        reason="The oven door hinge rod nests into a chassis cradle at the lower front hinge.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in (
        "chassis",
        "control_panel",
        "left_grate",
        "center_grate",
        "right_grate",
        "oven_door",
        "oven_light_button",
        "knob_1",
        "knob_2",
        "knob_3",
        "knob_4",
        "knob_5",
        "knob_6",
    ):
        ctx.check(
            f"{part_name}_present",
            object_model.get_part(part_name) is not None,
            details=f"Expected part {part_name} to exist.",
        )

    ctx.expect_contact(control_panel, chassis, contact_tol=0.0002)
    ctx.expect_contact(left_grate, chassis, contact_tol=0.0002)
    ctx.expect_contact(center_grate, chassis, contact_tol=0.0002)
    ctx.expect_contact(right_grate, chassis, contact_tol=0.0002)
    ctx.expect_contact(oven_door, chassis, contact_tol=0.0002)
    ctx.expect_contact(oven_light_button, control_panel, contact_tol=0.020)
    for knob_index, knob in enumerate(knobs, start=1):
        ctx.expect_contact(
            knob,
            control_panel,
            contact_tol=0.0002,
            name=f"knob_{knob_index}_contacts_panel",
        )

    for grate_name, burner_names in (
        ("left_grate", ("burner_1_cap", "burner_4_cap")),
        ("center_grate", ("burner_2_cap", "burner_5_cap")),
        ("right_grate", ("burner_3_cap", "burner_6_cap")),
    ):
        grate = object_model.get_part(grate_name)
        for burner_name in burner_names:
            ctx.expect_overlap(
                grate,
                chassis,
                axes="xy",
                elem_b=burner_name,
                min_overlap=0.070,
                name=f"{grate_name}_{burner_name}_coverage",
            )

    for index, joint in enumerate(knob_joints, start=1):
        limits = joint.motion_limits
        ctx.check(
            f"knob_{index}_continuous_axis",
            joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"Knob {index} should spin continuously about the front-to-back axis.",
        )

    button_limits = button_joint.motion_limits
    ctx.check(
        "button_prismatic_axis_and_travel",
        button_joint.axis == (0.0, -1.0, 0.0)
        and button_limits is not None
        and button_limits.lower == 0.0
        and button_limits.upper is not None
        and 0.004 <= button_limits.upper <= 0.010,
        details="The oven-light button should be a short inward-traveling plunger.",
    )

    door_limits = door_joint.motion_limits
    ctx.check(
        "door_hinge_axis_and_limits",
        door_joint.axis == (-1.0, 0.0, 0.0)
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and 1.2 <= door_limits.upper <= 1.6,
        details="The oven door should hinge downward about the lower x-axis with a realistic opening angle.",
    )

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    knob_x_positions = [position[0] for position in knob_positions if position is not None]
    spacing = [
        knob_x_positions[index + 1] - knob_x_positions[index]
        for index in range(len(knob_x_positions) - 1)
    ]
    if spacing:
        average_spacing = sum(spacing) / len(spacing)
        for index, gap in enumerate(spacing, start=1):
            ctx.check(
                f"knob_spacing_{index}",
                abs(gap - average_spacing) <= 0.002,
                details="Control knobs should be evenly spaced across the panel.",
            )
    button_position = ctx.part_world_position(oven_light_button)
    if button_position is not None and knob_positions[-1] is not None:
        ctx.check(
            "button_is_far_right",
            button_position[0] > knob_positions[-1][0] + 0.06,
            details="The oven-light button should sit to the far right of the knob row.",
        )

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    knob_indicator_rest = ctx.part_element_world_aabb(knobs[0], elem="indicator")
    with ctx.pose({knob_joints[0]: math.pi * 0.5}):
        knob_indicator_turn = ctx.part_element_world_aabb(knobs[0], elem="indicator")
        ctx.expect_contact(knobs[0], control_panel, contact_tol=0.0002)
    if knob_indicator_rest is not None and knob_indicator_turn is not None:
        rest_center = _aabb_center(knob_indicator_rest)
        turn_center = _aabb_center(knob_indicator_turn)
        ctx.check(
            "knob_indicator_moves_when_rotated",
            abs(turn_center[0] - rest_center[0]) >= 0.010
            or abs(turn_center[2] - rest_center[2]) >= 0.010,
            details="The knob indicator should move around the front-to-back axis when turned.",
        )

    button_rest = ctx.part_world_position(oven_light_button)
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: button_limits.upper}):
            button_pressed = ctx.part_world_position(oven_light_button)
            ctx.fail_if_isolated_parts(name="button_pressed_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(
                name="button_pressed_only_intended_overlap"
            )
            ctx.expect_contact(oven_light_button, control_panel, contact_tol=0.020)
        if button_rest is not None and button_pressed is not None:
            ctx.check(
                "button_moves_inward_when_pressed",
                button_pressed[1] <= button_rest[1] - 0.0035,
                details="The oven-light button should travel inward along the depth axis.",
            )

    door_rest = ctx.part_world_aabb(oven_door)
    if door_limits is not None and door_limits.upper is not None:
        with ctx.pose({door_joint: door_limits.upper}):
            door_open = ctx.part_world_aabb(oven_door)
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_no_floating")
            ctx.expect_contact(oven_door, chassis, contact_tol=0.020)
        if door_rest is not None and door_open is not None:
            ctx.check(
                "door_swings_forward_when_open",
                door_open[1][1] > door_rest[1][1] + 0.18,
                details="The oven door should project forward when opened downward.",
            )
            ctx.check(
                "door_drops_downward_when_open",
                door_open[1][2] < door_rest[1][2] - 0.35,
                details="The oven door should rotate down from the lower hinge.",
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
