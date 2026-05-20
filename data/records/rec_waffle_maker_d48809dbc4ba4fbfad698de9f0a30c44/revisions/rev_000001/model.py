from __future__ import annotations

import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _filleted_box(size: tuple[float, float, float], radius: float):
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_waffle_maker")

    shell = model.material("shell", rgba=(0.17, 0.18, 0.19, 1.0))
    trim = model.material("trim", rgba=(0.28, 0.30, 0.32, 1.0))
    plate = model.material("plate", rgba=(0.10, 0.10, 0.11, 1.0))
    handle_mat = model.material("handle", rgba=(0.08, 0.08, 0.09, 1.0))
    foot = model.material("foot", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_filleted_box((0.292, 0.286, 0.052), 0.020), "base_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=shell,
        name="body_shell",
    )
    base.visual(
        Box((0.266, 0.260, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=trim,
        name="top_trim",
    )
    base.visual(
        Box((0.242, 0.242, 0.008)),
        origin=Origin(xyz=(0.004, 0.0, 0.066)),
        material=trim,
        name="lower_frame",
    )
    base.visual(
        Box((0.218, 0.218, 0.006)),
        origin=Origin(xyz=(0.004, 0.0, 0.071)),
        material=plate,
        name="lower_plate",
    )
    base.visual(
        Box((0.030, 0.130, 0.014)),
        origin=Origin(xyz=(-0.148, 0.0, 0.069)),
        material=trim,
        name="hinge_block",
    )
    for index, y_pos in enumerate((-0.096, 0.096)):
        base.visual(
            Box((0.026, 0.040, 0.040)),
            origin=Origin(xyz=(-0.146, y_pos, 0.078)),
            material=trim,
            name=f"hinge_cheek_{index}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.046),
            origin=Origin(xyz=(-0.142, y_pos, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim,
            name=f"hinge_barrel_{index}",
        )
    for index, y_pos in enumerate((-0.060, 0.060)):
        base.visual(
            Box((0.012, 0.032, 0.008)),
            origin=Origin(xyz=(-0.126, y_pos, 0.075)),
            material=trim,
            name=f"hinge_rest_{index}",
        )
        base.visual(
            Box((0.008, 0.020, 0.008)),
            origin=Origin(xyz=(-0.132, y_pos, 0.068)),
            material=trim,
            name=f"hinge_rib_{index}",
        )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.104, -0.102), (-0.104, 0.102), (0.104, -0.102), (0.104, 0.102))
    ):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.003)),
            material=foot,
            name=f"foot_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.010, length=0.102),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="hinge_barrel",
    )
    lid.visual(
        mesh_from_cadquery(_filleted_box((0.260, 0.282, 0.048), 0.018), "lid_shell"),
        origin=Origin(xyz=(0.147, 0.0, 0.024)),
        material=shell,
        name="shell",
    )
    lid.visual(
        Box((0.236, 0.254, 0.014)),
        origin=Origin(xyz=(0.151, 0.0, 0.046)),
        material=trim,
        name="top_cap",
    )
    lid.visual(
        Box((0.234, 0.234, 0.008)),
        origin=Origin(xyz=(0.146, 0.0, -0.001)),
        material=trim,
        name="upper_frame",
    )
    lid.visual(
        Box((0.212, 0.212, 0.006)),
        origin=Origin(xyz=(0.146, 0.0, -0.005)),
        material=plate,
        name="upper_plate",
    )
    for index, y_pos in enumerate((-0.060, 0.060)):
        lid.visual(
            Box((0.016, 0.030, 0.008)),
            origin=Origin(xyz=(0.010, y_pos, -0.001)),
            material=trim,
            name=f"rear_lug_{index}",
        )

    front_handle = model.part("front_handle")
    front_handle.visual(
        mesh_from_cadquery(_filleted_box((0.044, 0.118, 0.018), 0.005), "front_handle_grip"),
        origin=Origin(xyz=(0.024, 0.0, 0.010)),
        material=handle_mat,
        name="grip",
    )
    for index, y_pos in enumerate((-0.046, 0.046)):
        front_handle.visual(
            Box((0.018, 0.020, 0.020)),
            origin=Origin(xyz=(0.009, y_pos, 0.010)),
            material=handle_mat,
            name=f"post_{index}",
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.142, 0.0, 0.084)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "lid_to_front_handle",
        ArticulationType.FIXED,
        parent=lid,
        child=front_handle,
        origin=Origin(xyz=(0.277, 0.0, 0.012)),
    )

    for index, y_pos in enumerate((0.066, 0.094)):
        button = model.part(f"program_button_{index}")
        button.visual(
            Box((0.002, 0.010, 0.014)),
            origin=Origin(xyz=(0.001, 0.0, 0.0)),
            material=trim,
            name="stem",
        )
        button.visual(
            Box((0.006, 0.017, 0.017)),
            origin=Origin(xyz=(0.005, 0.0, 0.0)),
            material=trim,
            name="cap",
        )
        model.articulation(
            f"base_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(0.146, y_pos, 0.022)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.04,
                lower=-0.002,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    front_handle = object_model.get_part("front_handle")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    lid_hinge = object_model.get_articulation("base_to_lid")
    button_joint_0 = object_model.get_articulation("base_to_program_button_0")
    button_joint_1 = object_model.get_articulation("base_to_program_button_1")

    ctx.expect_contact(
        lid,
        base,
        elem_a="rear_lug_0",
        elem_b="hinge_rest_0",
        name="rear hinge lug 0 is supported by the hinge rest",
    )
    ctx.expect_contact(
        lid,
        base,
        elem_a="rear_lug_1",
        elem_b="hinge_rest_1",
        name="rear hinge lug 1 is supported by the hinge rest",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.001,
        max_gap=0.003,
        name="upper and lower cooking plates stay nearly closed without colliding",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.20,
        name="upper and lower cooking plates remain broadly aligned",
    )
    ctx.expect_gap(
        button_0,
        base,
        axis="x",
        positive_elem="cap",
        negative_elem="body_shell",
        min_gap=0.0015,
        max_gap=0.0025,
        name="program button 0 stands proud of the front corner housing",
    )
    ctx.expect_gap(
        button_1,
        base,
        axis="x",
        positive_elem="cap",
        negative_elem="body_shell",
        min_gap=0.0015,
        max_gap=0.0025,
        name="program button 1 stands proud of the front corner housing",
    )
    ctx.expect_contact(
        button_0,
        base,
        elem_a="stem",
        elem_b="body_shell",
        name="program button 0 is physically mounted to the housing",
    )
    ctx.expect_contact(
        button_1,
        base,
        elem_a="stem",
        elem_b="body_shell",
        name="program button 1 is physically mounted to the housing",
    )
    ctx.expect_origin_distance(
        button_0,
        button_1,
        axes="y",
        min_dist=0.020,
        max_dist=0.032,
        name="program buttons remain visibly discrete",
    )

    handle_rest = ctx.part_world_position(front_handle)
    lid_limits = lid_hinge.motion_limits
    if handle_rest is not None and lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            handle_open = ctx.part_world_position(front_handle)
        ctx.check(
            "lid opens upward on the rear hinge",
            handle_open is not None and handle_open[2] > handle_rest[2] + 0.08,
            details=f"rest={handle_rest}, open={handle_open}",
        )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    button_0_limits = button_joint_0.motion_limits
    if (
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_limits is not None
        and button_0_limits.lower is not None
    ):
        with ctx.pose({button_joint_0: button_0_limits.lower}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_while_0_pressed = ctx.part_world_position(button_1)
            ctx.expect_gap(
                button_0,
                base,
                axis="x",
                positive_elem="cap",
                negative_elem="body_shell",
                min_gap=0.0,
                max_gap=0.0005,
                name="program button 0 presses nearly flush with the front face",
            )
        ctx.check(
            "program button 0 presses independently",
            button_0_pressed is not None
            and button_1_while_0_pressed is not None
            and button_0_pressed[0] < button_0_rest[0] - 0.0015
            and abs(button_1_while_0_pressed[0] - button_1_rest[0]) < 1e-6,
            details=(
                f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
                f"button_1_rest={button_1_rest}, button_1_while_0_pressed={button_1_while_0_pressed}"
            ),
        )

    if button_1_rest is not None and button_joint_1.motion_limits is not None and button_joint_1.motion_limits.lower is not None:
        with ctx.pose({button_joint_1: button_joint_1.motion_limits.lower}):
            button_1_pressed = ctx.part_world_position(button_1)
        ctx.check(
            "program button 1 also has its own travel",
            button_1_pressed is not None and button_1_pressed[0] < button_1_rest[0] - 0.0015,
            details=f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
