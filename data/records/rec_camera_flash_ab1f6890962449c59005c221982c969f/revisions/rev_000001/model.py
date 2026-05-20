from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_flash")

    body_mat = model.material("body_mat", color=(0.13, 0.13, 0.14))
    head_mat = model.material("head_mat", color=(0.17, 0.17, 0.18))
    trim_mat = model.material("trim_mat", color=(0.23, 0.24, 0.26))
    button_mat = model.material("button_mat", color=(0.33, 0.34, 0.36))
    lens_mat = model.material("lens_mat", color=(0.93, 0.94, 0.96, 0.96))
    metal_mat = model.material("metal_mat", color=(0.56, 0.58, 0.61))

    body = model.part("body")
    body.visual(
        Box((0.036, 0.028, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=body_mat,
        name="lower_shell",
    )
    body.visual(
        Box((0.028, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=body_mat,
        name="upper_shoulder",
    )
    body.visual(
        Box((0.002, 0.020, 0.038)),
        origin=Origin(xyz=(0.019, 0.0, 0.039)),
        material=trim_mat,
        name="control_strip",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=trim_mat,
        name="swivel_base",
    )
    body.visual(
        Box((0.012, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=trim_mat,
        name="shoe_stem",
    )
    body.visual(
        Box((0.022, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=metal_mat,
        name="shoe_plate",
    )
    body.visual(
        Box((0.022, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, -0.0075, -0.0155)),
        material=metal_mat,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.022, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.0075, -0.0155)),
        material=metal_mat,
        name="shoe_rail_1",
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=trim_mat,
        name="swivel_collar",
    )
    cradle.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=body_mat,
        name="neck_column",
    )
    cradle.visual(
        Box((0.058, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=body_mat,
        name="hinge_bridge",
    )
    cradle.visual(
        Box((0.004, 0.012, 0.016)),
        origin=Origin(xyz=(-0.029, 0.0, 0.024)),
        material=body_mat,
        name="hinge_ear_0",
    )
    cradle.visual(
        Box((0.004, 0.012, 0.016)),
        origin=Origin(xyz=(0.029, 0.0, 0.024)),
        material=body_mat,
        name="hinge_ear_1",
    )

    head = model.part("head")
    head.visual(
        Box((0.050, 0.034, 0.030)),
        origin=Origin(xyz=(0.0, 0.009, 0.018)),
        material=head_mat,
        name="head_shell",
    )
    head.visual(
        Box((0.044, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0275, 0.018)),
        material=lens_mat,
        name="flash_window",
    )
    head.visual(
        Box((0.038, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.0065, 0.019)),
        material=trim_mat,
        name="rear_panel",
    )
    head.visual(
        Cylinder(radius=0.0035, length=0.003),
        origin=Origin(xyz=(-0.0255, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_mat,
        name="hinge_pin_0",
    )
    head.visual(
        Cylinder(radius=0.0035, length=0.003),
        origin=Origin(xyz=(0.0255, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_mat,
        name="hinge_pin_1",
    )

    model.articulation(
        "body_to_cradle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=3.0,
            lower=-pi / 2.0,
            upper=pi / 2.0,
        ),
    )
    model.articulation(
        "cradle_to_head",
        ArticulationType.REVOLUTE,
        parent=cradle,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=3.0,
            lower=0.0,
            upper=1.5,
        ),
    )

    for idx, z in enumerate((0.050, 0.039, 0.028)):
        button = model.part(f"mode_button_{idx}")
        button.visual(
            Box((0.003, 0.010, 0.008)),
            origin=Origin(xyz=(0.0015, 0.0, 0.0)),
            material=button_mat,
            name="cap",
        )
        model.articulation(
            f"body_to_mode_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.020, 0.0, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0015,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head = object_model.get_part("head")
    cradle = object_model.get_part("cradle")
    swivel = object_model.get_articulation("body_to_cradle")
    tilt = object_model.get_articulation("cradle_to_head")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.020,
        name="head clears the body in rest pose",
    )
    ctx.expect_gap(
        cradle,
        body,
        axis="z",
        max_penetration=1e-6,
        max_gap=0.001,
        name="swivel cradle seats on the body turret",
    )

    rest_window = aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    tilted_window = None
    if tilt_upper is not None:
        with ctx.pose({tilt: tilt_upper}):
            ctx.expect_gap(
                head,
                body,
                axis="z",
                min_gap=0.015,
                name="tilted head still clears the body",
            )
            tilted_window = aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))

    ctx.check(
        "head tilts upward for bounce use",
        rest_window is not None
        and tilted_window is not None
        and tilted_window[2] > rest_window[2] + 0.010,
        details=f"rest_window={rest_window}, tilted_window={tilted_window}",
    )

    rest_head = aabb_center(ctx.part_world_aabb(head))
    swiveled_head = None
    with ctx.pose({swivel: 1.0}):
        swiveled_head = aabb_center(ctx.part_world_aabb(head))
        ctx.expect_gap(
            head,
            body,
            axis="z",
            min_gap=0.015,
            name="swiveled head stays above the body",
        )

    ctx.check(
        "head swivel moves the head sideways",
        rest_head is not None
        and swiveled_head is not None
        and abs(swiveled_head[0] - rest_head[0]) > 0.006,
        details=f"rest_head={rest_head}, swiveled_head={swiveled_head}",
    )

    button_names = [f"mode_button_{idx}" for idx in range(3)]
    button_parts = {name: object_model.get_part(name) for name in button_names}
    button_joints = {
        name: object_model.get_articulation(f"body_to_{name}")
        for name in button_names
    }
    button_rest = {name: ctx.part_world_position(part) for name, part in button_parts.items()}

    for idx, name in enumerate(button_names):
        joint = button_joints[name]
        pressed = None
        other_name = button_names[(idx + 1) % len(button_names)]
        other_pressed = None
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is not None:
            with ctx.pose({joint: upper}):
                pressed = ctx.part_world_position(button_parts[name])
                other_pressed = ctx.part_world_position(button_parts[other_name])

        ctx.check(
            f"{name} depresses inward",
            button_rest[name] is not None
            and pressed is not None
            and pressed[0] < button_rest[name][0] - 0.001,
            details=f"rest={button_rest[name]}, pressed={pressed}",
        )
        ctx.check(
            f"{name} moves independently",
            button_rest[other_name] is not None
            and other_pressed is not None
            and abs(other_pressed[0] - button_rest[other_name][0]) < 1e-7
            and abs(other_pressed[2] - button_rest[other_name][2]) < 1e-7,
            details=f"other_rest={button_rest[other_name]}, other_pressed={other_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
