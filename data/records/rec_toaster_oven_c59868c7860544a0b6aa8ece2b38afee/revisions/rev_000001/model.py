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

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    black_glass = model.material("black_glass", rgba=(0.12, 0.14, 0.16, 0.55))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    leg_black = model.material("leg_black", rgba=(0.09, 0.09, 0.09, 1.0))

    body = model.part("body")

    body_width = 0.50
    body_depth = 0.39
    body_height = 0.31
    foot_height = 0.015
    shell_thickness = 0.022
    floor_thickness = 0.020
    front_y = -body_depth / 2.0
    back_y = body_depth / 2.0
    bottom_z = foot_height
    top_z = foot_height + body_height

    control_panel_width = 0.112
    left_front_post = 0.024
    right_jamb = 0.018
    opening_width = body_width - control_panel_width - left_front_post - right_jamb
    opening_bottom = bottom_z + 0.033
    opening_height = 0.202
    opening_top = opening_bottom + opening_height
    opening_x_min = -body_width / 2.0 + left_front_post
    opening_x_max = opening_x_min + opening_width
    opening_center_x = (opening_x_min + opening_x_max) / 2.0
    front_frame_depth = 0.010
    control_panel_x_min = opening_x_max + right_jamb

    body.visual(
        Box((body_width, body_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_z + floor_thickness / 2.0)),
        material=dark_trim,
        name="floor_shell",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(-body_width / 2.0 + shell_thickness / 2.0, 0.0, bottom_z + body_height / 2.0)
        ),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((shell_thickness, body_depth, body_height)),
        origin=Origin(
            xyz=(body_width / 2.0 - shell_thickness / 2.0, 0.0, bottom_z + body_height / 2.0)
        ),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((body_width, body_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_z - shell_thickness / 2.0)),
        material=stainless,
        name="roof_shell",
    )
    body.visual(
        Box((body_width - 2.0 * shell_thickness, shell_thickness, body_height - floor_thickness - shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                back_y - shell_thickness / 2.0,
                bottom_z + floor_thickness + (body_height - floor_thickness - shell_thickness) / 2.0,
            )
        ),
        material=stainless,
        name="back_panel",
    )

    body.visual(
        Box((left_front_post, front_frame_depth, opening_height)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + left_front_post / 2.0,
                front_y + front_frame_depth / 2.0,
                opening_bottom + opening_height / 2.0,
            )
        ),
        material=dark_trim,
        name="left_front_post",
    )
    body.visual(
        Box((opening_width + left_front_post + right_jamb, front_frame_depth, opening_bottom - bottom_z)),
        origin=Origin(
            xyz=(
                (-body_width / 2.0 + control_panel_x_min) / 2.0,
                front_y + front_frame_depth / 2.0,
                bottom_z + (opening_bottom - bottom_z) / 2.0,
            )
        ),
        material=dark_trim,
        name="bottom_front_rail",
    )
    body.visual(
        Box((opening_width + left_front_post + right_jamb, front_frame_depth, top_z - opening_top)),
        origin=Origin(
            xyz=(
                (-body_width / 2.0 + control_panel_x_min) / 2.0,
                front_y + front_frame_depth / 2.0,
                opening_top + (top_z - opening_top) / 2.0,
            )
        ),
        material=dark_trim,
        name="top_front_rail",
    )
    body.visual(
        Box((right_jamb, front_frame_depth, opening_height)),
        origin=Origin(
            xyz=(
                opening_x_max + right_jamb / 2.0,
                front_y + front_frame_depth / 2.0,
                opening_bottom + opening_height / 2.0,
            )
        ),
        material=dark_trim,
        name="right_door_jamb",
    )
    body.visual(
        Box((control_panel_width, 0.012, body_height - 0.050)),
        origin=Origin(
            xyz=(
                control_panel_x_min + control_panel_width / 2.0,
                front_y - 0.006,
                bottom_z + 0.025 + (body_height - 0.050) / 2.0,
            )
        ),
        material=dark_trim,
        name="control_panel_face",
    )
    body.visual(
        Box((control_panel_width - 0.020, 0.006, body_height - 0.082)),
        origin=Origin(
            xyz=(
                control_panel_x_min + control_panel_width / 2.0,
                front_y - 0.013,
                bottom_z + 0.041 + (body_height - 0.082) / 2.0,
            )
        ),
        material=black_glass,
        name="control_panel_gloss",
    )

    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.185, -0.150),
            (0.185, -0.150),
            (-0.185, 0.150),
            (0.185, 0.150),
        )
    ):
        body.visual(
            Box((0.050, 0.032, foot_height)),
            origin=Origin(xyz=(x_pos, y_pos, foot_height / 2.0)),
            material=leg_black,
            name=f"foot_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + foot_height)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, (body_height + foot_height) / 2.0)),
    )

    door = model.part("door")
    door_width = 0.340
    door_height = 0.198
    door_thickness = 0.022
    stile_width = 0.032
    rail_height = 0.028

    door.visual(
        Box((stile_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width / 2.0 + stile_width / 2.0, door_thickness / 2.0, door_height / 2.0)),
        material=stainless,
        name="left_stile",
    )
    door.visual(
        Box((stile_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width / 2.0 - stile_width / 2.0, door_thickness / 2.0, door_height / 2.0)),
        material=stainless,
        name="right_stile",
    )
    door.visual(
        Box((door_width, door_thickness, rail_height)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0, rail_height / 2.0)),
        material=stainless,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width, door_thickness, rail_height)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0, door_height - rail_height / 2.0)),
        material=stainless,
        name="top_rail",
    )
    door.visual(
        Box((door_width - 2.0 * stile_width - 0.010, 0.006, door_height - 2.0 * rail_height - 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                0.015,
                rail_height + (door_height - 2.0 * rail_height - 0.012) / 2.0,
            )
        ),
        material=black_glass,
        name="door_glass",
    )
    for side, x_pos in (("left", -0.108), ("right", 0.108)):
        door.visual(
            Box((0.014, 0.036, 0.024)),
            origin=Origin(xyz=(x_pos, 0.004, 0.118)),
            material=dark_trim,
            name=f"{side}_handle_post",
        )
    door.visual(
        Box((0.220, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.020, 0.118)),
        material=dark_trim,
        name="handle_bar",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=1.6,
        origin=Origin(xyz=(0.0, door_thickness / 2.0, door_height / 2.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_center_x, front_y - 0.013, opening_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    def add_knob(name: str, *, z_pos: float, radius: float = 0.024) -> None:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=radius + 0.003, length=0.004),
            origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name="bezel",
        )
        knob.visual(
            Cylinder(radius=radius, length=0.028),
            origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.008, 0.012)),
            origin=Origin(xyz=(0.0, -0.040, radius * 0.68)),
            material=stainless,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Box((radius * 2.4, 0.050, radius * 2.4)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.022, 0.0)),
        )

        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(control_panel_x_min + control_panel_width / 2.0, front_y - 0.016, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.3,
                velocity=4.0,
                lower=-2.4,
                upper=2.4,
            ),
        )

    add_knob("knob_top", z_pos=0.248)
    add_knob("knob_middle", z_pos=0.182)
    add_knob("knob_bottom", z_pos=0.116, radius=0.026)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")
    top_knob = object_model.get_part("knob_top")
    middle_knob = object_model.get_part("knob_middle")
    bottom_knob = object_model.get_part("knob_bottom")
    top_knob_joint = object_model.get_articulation("body_to_knob_top")

    ctx.expect_overlap(
        door,
        body,
        axes="x",
        min_overlap=0.30,
        name="door spans the oven opening width",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
    with ctx.pose({door_hinge: 1.30}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")

    if closed_handle_aabb is None or open_handle_aabb is None:
        ctx.fail("door handle AABB is available", "Could not resolve handle_bar AABB in one or more poses.")
    else:
        closed_center_y = (closed_handle_aabb[0][1] + closed_handle_aabb[1][1]) / 2.0
        open_center_y = (open_handle_aabb[0][1] + open_handle_aabb[1][1]) / 2.0
        closed_center_z = (closed_handle_aabb[0][2] + closed_handle_aabb[1][2]) / 2.0
        open_center_z = (open_handle_aabb[0][2] + open_handle_aabb[1][2]) / 2.0

        ctx.check(
            "door handle swings forward when opened",
            open_center_y < closed_center_y - 0.08,
            details=f"closed_y={closed_center_y:.4f}, open_y={open_center_y:.4f}",
        )
        ctx.check(
            "door handle drops toward the counter when opened",
            open_center_z < closed_center_z - 0.05,
            details=f"closed_z={closed_center_z:.4f}, open_z={open_center_z:.4f}",
        )
        ctx.check(
            "opened door clears the countertop",
            open_handle_aabb[0][2] > 0.005,
            details=f"open_handle_min_z={open_handle_aabb[0][2]:.4f}",
        )

    for knob_name, knob_part in (
        ("top", top_knob),
        ("middle", middle_knob),
        ("bottom", bottom_knob),
    ):
        ctx.expect_gap(
            body,
            knob_part,
            axis="y",
            positive_elem="control_panel_gloss",
            negative_elem="knob_body",
            min_gap=0.004,
            max_gap=0.020,
            name=f"{knob_name} knob stands proud of the control panel",
        )

    closed_pointer_aabb = ctx.part_element_world_aabb(top_knob, elem="pointer")
    with ctx.pose({top_knob_joint: 1.6}):
        turned_pointer_aabb = ctx.part_element_world_aabb(top_knob, elem="pointer")

    if closed_pointer_aabb is None or turned_pointer_aabb is None:
        ctx.fail("top knob pointer AABB is available", "Could not resolve pointer AABB in one or more poses.")
    else:
        closed_center = (
            (closed_pointer_aabb[0][0] + closed_pointer_aabb[1][0]) / 2.0,
            (closed_pointer_aabb[0][2] + closed_pointer_aabb[1][2]) / 2.0,
        )
        turned_center = (
            (turned_pointer_aabb[0][0] + turned_pointer_aabb[1][0]) / 2.0,
            (turned_pointer_aabb[0][2] + turned_pointer_aabb[1][2]) / 2.0,
        )
        pointer_shift = math.hypot(turned_center[0] - closed_center[0], turned_center[1] - closed_center[1])
        ctx.check(
            "top knob visibly rotates about its shaft",
            pointer_shift > 0.015,
            details=f"closed={closed_center}, turned={turned_center}, shift={pointer_shift:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
