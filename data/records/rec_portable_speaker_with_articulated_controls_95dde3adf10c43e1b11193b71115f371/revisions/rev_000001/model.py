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
)


def _add_side_handle(part, *, side_sign: int, material) -> None:
    hinge_y = 0.066

    for local_y, hinge_name, arm_name in (
        (hinge_y, "front_hinge", "front_arm"),
        (-hinge_y, "rear_hinge", "rear_arm"),
    ):
        part.visual(
            Cylinder(radius=0.013, length=0.024),
            origin=Origin(
                xyz=(side_sign * 0.004, local_y, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=hinge_name,
        )
        part.visual(
            Box((0.016, 0.020, 0.020)),
            origin=Origin(xyz=(side_sign * 0.012, local_y, -0.012)),
            material=material,
            name=f"{hinge_name}_web",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.112),
            origin=Origin(xyz=(side_sign * 0.022, local_y, -0.056)),
            material=material,
            name=arm_name,
        )

    part.visual(
        Cylinder(radius=0.012, length=0.132),
        origin=Origin(
            xyz=(side_sign * 0.026, 0.0, -0.116),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="grip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jobsite_speaker")

    shell = model.material("shell", rgba=(0.18, 0.19, 0.20, 1.0))
    frame = model.material("frame", rgba=(0.11, 0.12, 0.13, 1.0))
    bumper = model.material("bumper", rgba=(0.87, 0.62, 0.17, 1.0))
    grille = model.material("grille", rgba=(0.06, 0.06, 0.07, 1.0))
    handle = model.material("handle", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel = model.material("wheel", rgba=(0.14, 0.14, 0.15, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.32, 0.33, 0.35, 1.0))
    button = model.material("button", rgba=(0.55, 0.57, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.420, 0.200, 0.300)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=shell,
        name="shell",
    )
    body.visual(
        Box((0.320, 0.052, 0.100)),
        origin=Origin(xyz=(0.0, 0.104, 0.095)),
        material=frame,
        name="control_face",
    )
    body.visual(
        Box((0.336, 0.052, 0.032)),
        origin=Origin(xyz=(0.0, 0.104, 0.050)),
        material=frame,
        name="grille_top",
    )
    body.visual(
        Box((0.336, 0.052, 0.032)),
        origin=Origin(xyz=(0.0, 0.104, -0.132)),
        material=frame,
        name="grille_bottom",
    )
    body.visual(
        Box((0.036, 0.052, 0.182)),
        origin=Origin(xyz=(0.168, 0.104, -0.041)),
        material=frame,
        name="grille_side_0",
    )
    body.visual(
        Box((0.036, 0.052, 0.182)),
        origin=Origin(xyz=(-0.168, 0.104, -0.041)),
        material=frame,
        name="grille_side_1",
    )
    body.visual(
        Box((0.296, 0.014, 0.152)),
        origin=Origin(xyz=(0.0, 0.086, -0.041)),
        material=grille,
        name="grille",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.127, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="wheel_housing",
    )

    for bumper_index, (x_sign, z_sign) in enumerate(
        ((1, 1), (-1, 1), (1, -1), (-1, -1))
    ):
        body.visual(
            Box((0.046, 0.056, 0.076)),
            origin=Origin(xyz=(x_sign * 0.225, 0.102, z_sign * 0.104)),
            material=bumper,
            name=f"corner_bumper_{bumper_index}",
        )

    for side_index, side_sign in enumerate((1, -1)):
        for hinge_index, local_y in enumerate((0.066, -0.066)):
            body.visual(
                Box((0.020, 0.028, 0.028)),
                origin=Origin(xyz=(side_sign * 0.219, local_y, 0.085)),
                material=bumper,
                name=f"hinge_pad_{side_index}_{hinge_index}",
            )

    for handle_index, side_sign in enumerate((1, -1)):
        side_handle = model.part(f"side_handle_{handle_index}")
        _add_side_handle(side_handle, side_sign=side_sign, material=handle)
        model.articulation(
            f"body_to_side_handle_{handle_index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=side_handle,
            origin=Origin(xyz=(side_sign * 0.239, 0.0, 0.085)),
            axis=(0.0, -float(side_sign), 0.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=1.6,
                lower=0.0,
                upper=1.65,
            ),
        )

    selector_wheel = model.part("selector_wheel")
    selector_wheel.visual(
        Cylinder(radius=0.040, length=0.022),
        origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel,
        name="wheel",
    )
    selector_wheel.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(0.0, 0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel,
        name="rim",
    )
    selector_wheel.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_hub,
        name="hub",
    )
    model.articulation(
        "body_to_selector_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_wheel,
        origin=Origin(xyz=(0.0, 0.130, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=8.0,
        ),
    )

    for button_index, button_x in enumerate((-0.075, 0.0, 0.075)):
        button_part = model.part(f"button_{button_index}")
        button_part.visual(
            Box((0.038, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, 0.007, 0.0)),
            material=button,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button_part,
            origin=Origin(xyz=(button_x, 0.130, 0.052)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.10,
                lower=0.0,
                upper=0.0045,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    selector_wheel = object_model.get_part("selector_wheel")
    selector_joint = object_model.get_articulation("body_to_selector_wheel")

    ctx.check(
        "selector wheel uses continuous articulation",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={selector_joint.articulation_type}",
    )
    ctx.expect_gap(
        selector_wheel,
        body,
        axis="y",
        positive_elem="wheel",
        negative_elem="wheel_housing",
        max_gap=0.0015,
        max_penetration=0.0,
        name="selector wheel seats on front axle",
    )
    ctx.expect_overlap(
        selector_wheel,
        body,
        axes="xz",
        elem_a="wheel",
        elem_b="control_face",
        min_overlap=0.060,
        name="selector wheel stays centered on the control face",
    )

    for button_index in range(3):
        button_part = object_model.get_part(f"button_{button_index}")
        button_joint = object_model.get_articulation(f"body_to_button_{button_index}")

        ctx.check(
            f"button_{button_index} uses prismatic articulation",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"joint type={button_joint.articulation_type}",
        )
        ctx.expect_gap(
            button_part,
            body,
            axis="y",
            positive_elem="cap",
            negative_elem="control_face",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"button_{button_index} sits proud on the control face",
        )
        ctx.expect_overlap(
            button_part,
            body,
            axes="xz",
            elem_a="cap",
            elem_b="control_face",
            min_overlap=0.016,
            name=f"button_{button_index} stays within the control face",
        )

        limits = button_joint.motion_limits
        rest_position = ctx.part_world_position(button_part)
        if limits is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.upper}):
                pressed_position = ctx.part_world_position(button_part)
            ctx.check(
                f"button_{button_index} depresses inward",
                rest_position is not None
                and pressed_position is not None
                and pressed_position[1] < rest_position[1] - 0.003,
                details=f"rest={rest_position}, pressed={pressed_position}",
            )

    for handle_index, side_sign in enumerate((1, -1)):
        side_handle = object_model.get_part(f"side_handle_{handle_index}")
        handle_joint = object_model.get_articulation(f"body_to_side_handle_{handle_index}")
        limits = handle_joint.motion_limits

        ctx.check(
            f"side_handle_{handle_index} uses revolute articulation",
            handle_joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"joint type={handle_joint.articulation_type}",
        )

        closed_aabb = ctx.part_world_aabb(side_handle)
        if limits is not None and limits.upper is not None and closed_aabb is not None:
            with ctx.pose({handle_joint: limits.upper}):
                open_aabb = ctx.part_world_aabb(side_handle)

            if side_sign > 0:
                opens_correctly = (
                    open_aabb is not None
                    and open_aabb[1][0] > closed_aabb[1][0] + 0.025
                    and open_aabb[1][2] > closed_aabb[1][2] + 0.030
                )
            else:
                opens_correctly = (
                    open_aabb is not None
                    and open_aabb[0][0] < closed_aabb[0][0] - 0.025
                    and open_aabb[1][2] > closed_aabb[1][2] + 0.030
                )

            ctx.check(
                f"side_handle_{handle_index} folds out for carry position",
                opens_correctly,
                details=f"closed={closed_aabb}, open={open_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
