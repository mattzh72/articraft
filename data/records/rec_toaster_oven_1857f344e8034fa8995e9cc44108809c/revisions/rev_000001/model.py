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
    model = ArticulatedObject(name="countertop_toaster_oven")

    body_metal = model.material("body_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    panel_trim = model.material("panel_trim", rgba=(0.11, 0.11, 0.12, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.20, 0.26, 0.30, 0.35))
    steel = model.material("steel", rgba=(0.76, 0.77, 0.79, 1.0))
    foot_black = model.material("foot_black", rgba=(0.08, 0.08, 0.08, 1.0))

    body_width = 0.50
    body_depth = 0.36
    body_height = 0.32
    shell_side = 0.018
    shell_top = 0.016
    shell_bottom = 0.014
    front_thickness = 0.018

    opening_width = 0.30
    opening_height = 0.205
    opening_bottom = 0.055
    opening_center_x = -0.055
    control_strip_width = 0.155

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, shell_bottom)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, shell_bottom * 0.5)),
        material=body_metal,
        name="bottom_panel",
    )
    body.visual(
        Box((body_width, body_depth, shell_top)),
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height - shell_top * 0.5)),
        material=body_metal,
        name="top_panel",
    )
    body.visual(
        Box((shell_side, body_depth, body_height - shell_bottom - shell_top)),
        origin=Origin(
            xyz=(
                -body_width * 0.5 + shell_side * 0.5,
                body_depth * 0.5,
                shell_bottom + (body_height - shell_bottom - shell_top) * 0.5,
            )
        ),
        material=body_metal,
        name="left_wall",
    )
    body.visual(
        Box((shell_side, body_depth, body_height - shell_bottom - shell_top)),
        origin=Origin(
            xyz=(
                body_width * 0.5 - shell_side * 0.5,
                body_depth * 0.5,
                shell_bottom + (body_height - shell_bottom - shell_top) * 0.5,
            )
        ),
        material=body_metal,
        name="right_wall",
    )
    body.visual(
        Box((body_width - 2.0 * shell_side, 0.014, body_height - shell_bottom - shell_top)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth - 0.007,
                shell_bottom + (body_height - shell_bottom - shell_top) * 0.5,
            )
        ),
        material=body_metal,
        name="back_wall",
    )

    left_rail_width = body_width * 0.5 + opening_center_x - opening_width * 0.5
    front_opening_right = opening_center_x + opening_width * 0.5
    front_span_left = -body_width * 0.5
    front_structure_width = front_opening_right - front_span_left

    body.visual(
        Box((left_rail_width, front_thickness, opening_height)),
        origin=Origin(
            xyz=(
                front_span_left + left_rail_width * 0.5,
                front_thickness * 0.5,
                opening_bottom + opening_height * 0.5,
            )
        ),
        material=body_metal,
        name="left_front_rail",
    )
    body.visual(
        Box((front_structure_width, front_thickness, opening_bottom - shell_bottom)),
        origin=Origin(
            xyz=(
                front_span_left + front_structure_width * 0.5,
                front_thickness * 0.5,
                shell_bottom + (opening_bottom - shell_bottom) * 0.5,
            )
        ),
        material=body_metal,
        name="bottom_sill",
    )
    body.visual(
        Box((front_structure_width, front_thickness, body_height - opening_bottom - opening_height)),
        origin=Origin(
            xyz=(
                front_span_left + front_structure_width * 0.5,
                front_thickness * 0.5,
                opening_bottom
                + opening_height
                + (body_height - opening_bottom - opening_height) * 0.5,
            )
        ),
        material=body_metal,
        name="top_lintel",
    )
    body.visual(
        Box((control_strip_width, front_thickness, body_height - shell_bottom)),
        origin=Origin(
            xyz=(
                front_opening_right + control_strip_width * 0.5,
                front_thickness * 0.5,
                shell_bottom + (body_height - shell_bottom) * 0.5,
            )
        ),
        material=panel_trim,
        name="control_panel_face",
    )
    body.visual(
        Box((0.014, body_depth - 0.030, body_height - 0.070)),
        origin=Origin(
            xyz=(
                front_opening_right + 0.007,
                0.015 + (body_depth - 0.030) * 0.5,
                0.025 + (body_height - 0.070) * 0.5,
            )
        ),
        material=panel_trim,
        name="control_partition",
    )
    body.visual(
        Box((control_strip_width - 0.026, 0.006, body_height - 0.086)),
        origin=Origin(
            xyz=(
                front_opening_right + control_strip_width * 0.5,
                front_thickness + 0.003,
                0.043 + (body_height - 0.086) * 0.5,
            )
        ),
        material=panel_trim,
        name="panel_recess",
    )
    for knob_index, knob_z in enumerate((0.244, 0.176, 0.108)):
        body.visual(
            Cylinder(radius=0.010, length=0.002),
            origin=Origin(
                xyz=(front_opening_right + control_strip_width * 0.5, 0.0, knob_z),
                rpy=(-math.pi * 0.5, 0.0, 0.0),
            ),
            material=steel,
            name=f"knob_bushing_{knob_index}",
        )

    foot_positions = (
        (-0.185, 0.050),
        (0.185, 0.050),
        (-0.185, 0.305),
        (0.185, 0.305),
    )
    for index, (x_pos, y_pos) in enumerate(foot_positions):
        body.visual(
            Box((0.040, 0.028, 0.008)),
            origin=Origin(xyz=(x_pos, y_pos, -0.004)),
            material=foot_black,
            name=f"foot_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=8.6,
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height * 0.5)),
    )

    door = model.part("door")
    door_width = opening_width + 0.014
    door_height = opening_height + 0.014
    frame_bar = 0.030
    door_thickness = 0.022

    door.visual(
        Box((door_width, door_thickness, frame_bar)),
        origin=Origin(xyz=(0.0, 0.0, frame_bar * 0.5)),
        material=body_metal,
        name="bottom_frame",
    )
    door.visual(
        Box((door_width, door_thickness, frame_bar)),
        origin=Origin(xyz=(0.0, 0.0, door_height - frame_bar * 0.5)),
        material=body_metal,
        name="top_frame",
    )
    door.visual(
        Box((frame_bar, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width * 0.5 + frame_bar * 0.5, 0.0, door_height * 0.5)),
        material=body_metal,
        name="left_frame",
    )
    door.visual(
        Box((frame_bar, door_thickness, door_height)),
        origin=Origin(xyz=(door_width * 0.5 - frame_bar * 0.5, 0.0, door_height * 0.5)),
        material=body_metal,
        name="right_frame",
    )
    door.visual(
        Box((door_width - 2.0 * frame_bar + 0.004, 0.008, door_height - 2.0 * frame_bar + 0.004)),
        origin=Origin(xyz=(0.0, -0.002, door_height * 0.5)),
        material=glass_tint,
        name="window_glass",
    )
    door.visual(
        Box((0.020, 0.028, 0.020)),
        origin=Origin(xyz=(-0.095, -0.020, door_height - 0.052)),
        material=steel,
        name="left_handle_post",
    )
    door.visual(
        Box((0.020, 0.028, 0.020)),
        origin=Origin(xyz=(0.095, -0.020, door_height - 0.052)),
        material=steel,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.228),
        origin=Origin(
            xyz=(0.0, -0.042, door_height - 0.052),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=steel,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, door_height * 0.5)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_center_x, -0.011, opening_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=1.6,
        ),
    )

    knob_x = front_opening_right + control_strip_width * 0.5
    knob_z_positions = (0.244, 0.176, 0.108)
    for knob_index, knob_z in enumerate(knob_z_positions):
        knob = model.part(f"knob_{knob_index}")
        knob.visual(
            Cylinder(radius=0.031, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=panel_trim,
            name="rear_skirt",
        )
        knob.visual(
            Cylinder(radius=0.026, length=0.030),
            origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=panel_trim,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(xyz=(0.0, -0.036, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=steel,
            name="face_cap",
        )
        knob.visual(
            Box((0.006, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, -0.031, 0.018)),
            material=steel,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.064, 0.044, 0.064)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.018, 0.0)),
        )
        model.articulation(
            f"body_to_knob_{knob_index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, -0.001, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=4.0,
                lower=-2.35,
                upper=2.35,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")
    upper_knob = object_model.get_part("knob_0")
    upper_knob_joint = object_model.get_articulation("body_to_knob_0")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        max_penetration=0.015,
        name="closed door sits nearly flush with the front face",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.18,
        name="closed door covers the oven opening footprint",
    )

    closed_top_aabb = ctx.part_element_world_aabb(door, elem="top_frame")
    with ctx.pose({door_hinge: 1.45}):
        opened_top_aabb = ctx.part_element_world_aabb(door, elem="top_frame")

    ctx.check(
        "door opens downward and outward",
        closed_top_aabb is not None
        and opened_top_aabb is not None
        and opened_top_aabb[1][1] < closed_top_aabb[1][1] - 0.10
        and opened_top_aabb[1][2] < closed_top_aabb[1][2] - 0.14,
        details=f"closed_top={closed_top_aabb}, opened_top={opened_top_aabb}",
    )
    ctx.check(
        "opened door clears below the main opening",
        opened_top_aabb is not None and opened_top_aabb[1][2] < 0.16,
        details=f"opened_top={opened_top_aabb}",
    )

    left_rail_aabb = ctx.part_element_world_aabb(body, elem="left_front_rail")
    panel_aabb = ctx.part_element_world_aabb(body, elem="control_panel_face")
    left_rail_width = None
    panel_width = None
    if left_rail_aabb is not None:
        left_rail_width = left_rail_aabb[1][0] - left_rail_aabb[0][0]
    if panel_aabb is not None:
        panel_width = panel_aabb[1][0] - panel_aabb[0][0]
    ctx.check(
        "control bay is wider than the left front rail",
        left_rail_width is not None
        and panel_width is not None
        and panel_width > left_rail_width + 0.08,
        details=f"left_rail_width={left_rail_width}, panel_width={panel_width}",
    )

    ctx.expect_gap(
        body,
        upper_knob,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        name="upper knob stands proud of the control panel",
    )
    knob_indicator_rest = ctx.part_element_world_aabb(upper_knob, elem="indicator")
    with ctx.pose({upper_knob_joint: 1.2}):
        knob_indicator_turned = ctx.part_element_world_aabb(upper_knob, elem="indicator")
    rest_center = None
    turned_center = None
    if knob_indicator_rest is not None:
        rest_center = tuple(
            (knob_indicator_rest[0][axis] + knob_indicator_rest[1][axis]) * 0.5 for axis in range(3)
        )
    if knob_indicator_turned is not None:
        turned_center = tuple(
            (knob_indicator_turned[0][axis] + knob_indicator_turned[1][axis]) * 0.5 for axis in range(3)
        )
    ctx.check(
        "upper knob indicator rotates around its shaft",
        rest_center is not None
        and turned_center is not None
        and (
            abs(turned_center[0] - rest_center[0]) > 0.008
            or abs(turned_center[2] - rest_center[2]) > 0.008
        ),
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
