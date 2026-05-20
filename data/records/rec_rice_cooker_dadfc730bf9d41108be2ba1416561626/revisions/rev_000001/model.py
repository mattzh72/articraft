from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

BODY_DEPTH = 0.244
BODY_WIDTH = 0.244
BODY_SHELL_HEIGHT = 0.176
FOOT_HEIGHT = 0.008
BODY_TOP_Z = BODY_SHELL_HEIGHT + FOOT_HEIGHT
BODY_FRONT_X = BODY_DEPTH / 2.0
BODY_REAR_HINGE_X = -0.104
BODY_RADIUS = BODY_DEPTH / 2.0

LID_LENGTH = 0.226
LID_WIDTH = 0.268
LID_HEIGHT = 0.050

CONTROL_PANEL_DEPTH = 0.012
CONTROL_PANEL_WIDTH = 0.114
CONTROL_PANEL_HEIGHT = 0.062


def _body_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.092, 0.000),
                (0.108, 0.010),
                (0.120, 0.038),
                (BODY_RADIUS, 0.108),
                (0.118, BODY_SHELL_HEIGHT),
            ],
            [
                (0.000, 0.010),
                (0.086, 0.016),
                (0.102, 0.038),
                (0.104, 0.150),
                (0.104, 0.168),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ).translate(0.0, 0.0, FOOT_HEIGHT),
        "body_shell",
    )


def _liner_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.048, 0.000),
                (0.094, 0.006),
                (0.098, 0.020),
                (0.098, 0.116),
                (0.104, 0.120),
            ],
            [
                (0.000, 0.004),
                (0.086, 0.010),
                (0.092, 0.020),
                (0.092, 0.118),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "liner_shell",
    )


def _lid_shell():
    lid = LatheGeometry.from_shell_profiles(
        [
            (0.024, 0.000),
            (0.112, 0.006),
            (0.120, 0.018),
            (0.116, 0.032),
            (0.100, LID_HEIGHT),
        ],
        [
            (0.000, 0.006),
            (0.096, 0.012),
            (0.106, 0.020),
            (0.100, 0.040),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    ).translate(0.108, 0.0, 0.0)
    lid.merge(
        CylinderGeometry(radius=0.018, height=0.012, radial_segments=36).translate(0.030, 0.0, 0.024)
    )
    return mesh_from_geometry(lid, "lid_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.92, 1.0))
    shell_gray = model.material("shell_gray", rgba=(0.76, 0.78, 0.80, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    display_glass = model.material("display_glass", rgba=(0.09, 0.12, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.82, 0.84, 0.86, 1.0))
    button_red = model.material("button_red", rgba=(0.73, 0.17, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        _body_shell(),
        material=shell_white,
        name="shell",
    )
    body.visual(
        Box((CONTROL_PANEL_DEPTH, CONTROL_PANEL_WIDTH, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                BODY_FRONT_X + CONTROL_PANEL_DEPTH / 2.0 - 0.002,
                0.0,
                0.072,
            )
        ),
        material=shell_gray,
        name="control_panel",
    )
    body.visual(
        Box((0.004, 0.080, 0.024)),
        origin=Origin(xyz=(BODY_FRONT_X + 0.003, 0.0, 0.092)),
        material=display_glass,
        name="display",
    )
    body.visual(
        Box((0.016, 0.050, 0.012)),
        origin=Origin(xyz=(BODY_FRONT_X + 0.002, 0.0, 0.153)),
        material=trim_dark,
        name="latch",
    )
    body.visual(
        Box((0.016, 0.118, 0.006)),
        origin=Origin(xyz=(BODY_REAR_HINGE_X, 0.0, 0.175)),
        material=trim_dark,
        name="rear_hinge_bridge",
    )
    body.visual(
        Box((0.052, 0.016, 0.040)),
        origin=Origin(xyz=(0.011, -0.128, 0.203)),
        material=trim_dark,
        name="handle_post_0",
    )
    body.visual(
        Box((0.052, 0.016, 0.040)),
        origin=Origin(xyz=(0.011, 0.128, 0.203)),
        material=trim_dark,
        name="handle_post_1",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Box((0.022, 0.022, FOOT_HEIGHT)),
                origin=Origin(
                    xyz=(
                        x_sign * 0.072,
                        y_sign * 0.072,
                        FOOT_HEIGHT / 2.0 + 0.001,
                    )
                ),
                material=trim_dark,
                name=f"foot_{int((x_sign + 1) / 2)}_{int((y_sign + 1) / 2)}",
            )

    body.visual(
        _liner_shell(),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel,
        name="pot",
    )

    lid = model.part("lid")
    lid.visual(
        _lid_shell(),
        material=shell_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.014, 0.070, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, 0.004)),
        material=trim_dark,
        name="rear_hinge_tab",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, -0.146, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="pivot_0",
    )
    handle.visual(
        Cylinder(radius=0.009, length=0.020),
        origin=Origin(xyz=(0.0, 0.146, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="pivot_1",
    )
    handle_pitch = math.atan2(0.060, 0.085)
    handle.visual(
        Cylinder(radius=0.007, length=math.hypot(0.060, 0.085)),
        origin=Origin(xyz=(0.030, -0.146, 0.0425), rpy=(0.0, handle_pitch, 0.0)),
        material=trim_dark,
        name="strut_0",
    )
    handle.visual(
        Cylinder(radius=0.007, length=math.hypot(0.060, 0.085)),
        origin=Origin(xyz=(0.030, 0.146, 0.0425), rpy=(0.0, handle_pitch, 0.0)),
        material=trim_dark,
        name="strut_1",
    )
    handle.visual(
        Cylinder(radius=0.008, length=0.292),
        origin=Origin(xyz=(0.060, 0.0, 0.085), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="grip",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.007, 0.040, 0.013)),
        origin=Origin(xyz=(0.0035, 0.0, 0.0)),
        material=button_red,
        name="button_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(BODY_REAR_HINGE_X, 0.0, BODY_TOP_Z - 0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.032, 0.0, BODY_TOP_Z + 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-1.10,
            upper=0.35,
        ),
    )
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(BODY_FRONT_X + 0.008, 0.0, 0.050)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.03,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    button = object_model.get_part("power_button")

    lid_joint = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_handle")
    button_joint = object_model.get_articulation("body_to_power_button")

    lid_limits = lid_joint.motion_limits
    handle_limits = handle_joint.motion_limits
    button_limits = button_joint.motion_limits

    ctx.expect_contact(
        lid,
        body,
        elem_a="rear_hinge_tab",
        elem_b="rear_hinge_bridge",
        name="rear hinge tab rests on the body bridge",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.180,
        name="closed lid covers the cooker opening",
    )
    ctx.expect_gap(
        handle,
        lid,
        axis="z",
        positive_elem="grip",
        negative_elem="lid_shell",
        min_gap=0.030,
        name="carry handle stays visibly above the lid shell",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_joint: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "lid opens upward from the rear hinge",
            rest_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.12,
            details=f"rest={rest_lid_aabb!r}, open={open_lid_aabb!r}",
        )

    rest_handle_aabb = ctx.part_world_aabb(handle)
    if handle_limits is not None and handle_limits.lower is not None:
        with ctx.pose({handle_joint: handle_limits.lower}):
            folded_handle_aabb = ctx.part_world_aabb(handle)
        ctx.check(
            "handle folds rearward on the side pivots",
            rest_handle_aabb is not None
            and folded_handle_aabb is not None
            and folded_handle_aabb[0][0] < rest_handle_aabb[0][0] - 0.045,
            details=f"rest={rest_handle_aabb!r}, folded={folded_handle_aabb!r}",
        )

    rest_button_pos = ctx.part_world_position(button)
    if button_limits is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            "power button presses inward",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[0] < rest_button_pos[0] - 0.003,
            details=f"rest={rest_button_pos!r}, pressed={pressed_button_pos!r}",
        )

    return ctx.report()


object_model = build_object_model()
