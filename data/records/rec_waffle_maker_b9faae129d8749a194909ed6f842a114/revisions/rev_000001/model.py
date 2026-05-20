from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


BODY_WIDTH = 0.286
BODY_DEPTH = 0.252
BODY_HEIGHT = 0.086

HINGE_RADIUS = 0.0058
HINGE_Y = -(BODY_DEPTH * 0.5) - 0.0035
HINGE_Z = 0.078

LID_DEPTH = 0.238
LID_CENTER_Y = 0.121
LID_BUTTON_Y = 0.214
LID_BUTTON_Z = 0.036
BUTTON_TRAVEL = 0.0025
BUTTON_POSITIONS = (
    ("button_0", 0.067, "button_bezel_0"),
    ("button_1", 0.095, "button_bezel_1"),
)


def _rounded_section(width: float, depth: float, radius: float, *, z: float, y_shift: float = 0.0) -> list[tuple[float, float, float]]:
    return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _lower_shell_shape():
    return section_loft(
        [
            _rounded_section(BODY_WIDTH, BODY_DEPTH, 0.036, z=0.0),
            _rounded_section(BODY_WIDTH, BODY_DEPTH, 0.040, z=0.030),
            _rounded_section(BODY_WIDTH * 0.94, BODY_DEPTH * 0.90, 0.050, z=0.056),
            _rounded_section(BODY_WIDTH * 0.82, BODY_DEPTH * 0.72, 0.060, z=0.076),
        ]
    )


def _lid_shell_shape():
    return section_loft(
        [
            _rounded_section(BODY_WIDTH * 0.98, LID_DEPTH, 0.034, z=0.004, y_shift=LID_CENTER_Y),
            _rounded_section(BODY_WIDTH * 0.96, LID_DEPTH * 0.98, 0.040, z=0.022, y_shift=LID_CENTER_Y + 0.003),
            _rounded_section(BODY_WIDTH * 0.84, LID_DEPTH * 0.78, 0.054, z=0.050, y_shift=LID_CENTER_Y + 0.004),
            _rounded_section(BODY_WIDTH * 0.70, LID_DEPTH * 0.60, 0.058, z=0.074, y_shift=LID_CENTER_Y),
        ]
    )


def _handle_shape():
    return section_loft(
        [
            _rounded_section(0.112, 0.026, 0.010, z=0.012, y_shift=0.234),
            _rounded_section(0.108, 0.028, 0.012, z=0.018, y_shift=0.236),
            _rounded_section(0.094, 0.022, 0.010, z=0.026, y_shift=0.236),
            _rounded_section(0.078, 0.016, 0.007, z=0.032, y_shift=0.234),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bubble_waffle_maker")

    shell_red = model.material("shell_red", rgba=(0.79, 0.17, 0.17, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.11, 0.11, 0.12, 1.0))
    button_gray = model.material("button_gray", rgba=(0.72, 0.74, 0.77, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_geometry(_lower_shell_shape(), "lower_body_shell"),
        material=shell_red,
        name="body_shell",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.084),
        origin=Origin(
            xyz=(-0.078, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_dark,
        name="hinge_mount_0",
    )
    lower_body.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.084),
        origin=Origin(
            xyz=(0.078, HINGE_Y, HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_dark,
        name="hinge_mount_1",
    )
    for x_pos in (-0.078, 0.078):
        lower_body.visual(
            Box((0.090, 0.024, 0.018)),
            origin=Origin(xyz=(x_pos, HINGE_Y + 0.015, HINGE_Z - 0.009)),
            material=hinge_dark,
            name=f"hinge_bridge_{0 if x_pos < 0.0 else 1}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_shell_shape(), "lid_shell"),
        material=shell_red,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS - 0.0012, length=0.062),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_dark,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.074, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.005)),
        material=hinge_dark,
        name="hinge_lug",
    )
    lid.visual(
        mesh_from_geometry(_handle_shape(), "front_handle"),
        material=shell_dark,
        name="front_handle",
    )
    lid.visual(
        Box((0.052, 0.036, 0.016)),
        origin=Origin(xyz=(0.079, 0.214, 0.028)),
        material=shell_dark,
        name="button_pod",
    )

    model.articulation(
        "lower_body_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )

    for part_name, x_pos, _ in BUTTON_POSITIONS:
        button = model.part(part_name)
        button.visual(
            Cylinder(radius=0.0062, length=0.0042),
            origin=Origin(xyz=(0.0, 0.0, 0.0021)),
            material=button_gray,
            name="button_cap",
        )
        model.articulation(
            f"lid_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=lid,
            child=button,
            origin=Origin(xyz=(x_pos, LID_BUTTON_Y, LID_BUTTON_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lower_body = object_model.get_part("lower_body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lower_body_to_lid")

    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_joint_0 = object_model.get_articulation("lid_to_button_0")
    button_joint_1 = object_model.get_articulation("lid_to_button_1")

    ctx.expect_gap(
        lid,
        lower_body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        min_gap=0.002,
        max_gap=0.008,
        name="lid shell stays visibly separate around the seam",
    )
    ctx.expect_overlap(
        lid,
        lower_body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.200,
        name="lid shell covers the lower shell footprint",
    )
    ctx.expect_origin_distance(
        button_0,
        button_1,
        axes="x",
        min_dist=0.024,
        max_dist=0.040,
        name="program buttons remain visibly discrete from each other",
    )

    button_0_cap = ctx.part_element_world_aabb(button_0, elem="button_cap")
    button_1_cap = ctx.part_element_world_aabb(button_1, elem="button_cap")
    ctx.check(
        "button caps sit above the lid seam plane",
        button_0_cap is not None
        and
        button_1_cap is not None
        and button_0_cap[1][2] > 0.080
        and button_1_cap[1][2] > 0.080,
        details=f"button_0_cap={button_0_cap}, button_1_cap={button_1_cap}",
    )

    handle_rest = ctx.part_element_world_aabb(lid, elem="front_handle")
    hinge_limits = lid_hinge.motion_limits
    if hinge_limits is not None and hinge_limits.upper is not None:
        with ctx.pose({lid_hinge: hinge_limits.upper}):
            handle_open = ctx.part_element_world_aabb(lid, elem="front_handle")
        ctx.check(
            "lid opens upward from the rear hinge",
            handle_rest is not None
            and handle_open is not None
            and handle_open[1][2] > handle_rest[1][2] + 0.070,
            details=f"handle_rest={handle_rest}, handle_open={handle_open}",
        )

    button_0_limits = button_joint_0.motion_limits
    if button_0_limits is not None and button_0_limits.upper is not None:
        button_1_rest = ctx.part_element_world_aabb(button_1, elem="button_cap")
        with ctx.pose({button_joint_0: button_0_limits.upper}):
            button_0_pressed = ctx.part_element_world_aabb(button_0, elem="button_cap")
            button_1_while_0_pressed = ctx.part_element_world_aabb(button_1, elem="button_cap")
        ctx.check(
            "button 0 depresses without dragging button 1",
            button_0_cap is not None
            and button_0_pressed is not None
            and button_1_rest is not None
            and button_1_while_0_pressed is not None
            and button_0_pressed[1][2] < button_0_cap[1][2] - 0.0015
            and abs(button_1_while_0_pressed[1][2] - button_1_rest[1][2]) < 0.0005,
            details=(
                f"button_0_rest={button_0_cap}, button_0_pressed={button_0_pressed}, "
                f"button_1_rest={button_1_rest}, button_1_while_0_pressed={button_1_while_0_pressed}"
            ),
        )

    button_1_limits = button_joint_1.motion_limits
    if button_1_limits is not None and button_1_limits.upper is not None:
        button_0_rest = ctx.part_element_world_aabb(button_0, elem="button_cap")
        with ctx.pose({button_joint_1: button_1_limits.upper}):
            button_1_pressed = ctx.part_element_world_aabb(button_1, elem="button_cap")
            button_0_while_1_pressed = ctx.part_element_world_aabb(button_0, elem="button_cap")
        ctx.check(
            "button 1 depresses without dragging button 0",
            button_1_cap is not None
            and button_1_pressed is not None
            and button_0_rest is not None
            and button_0_while_1_pressed is not None
            and button_1_pressed[1][2] < button_1_cap[1][2] - 0.0015
            and abs(button_0_while_1_pressed[1][2] - button_0_rest[1][2]) < 0.0005,
            details=(
                f"button_1_rest={button_1_cap}, button_1_pressed={button_1_pressed}, "
                f"button_0_rest={button_0_rest}, button_0_while_1_pressed={button_0_while_1_pressed}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
