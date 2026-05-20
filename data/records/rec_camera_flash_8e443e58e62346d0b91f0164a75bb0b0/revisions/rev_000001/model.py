from __future__ import annotations

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
    model = ArticulatedObject(name="hot_shoe_flash")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.20, 0.22, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    metal = model.material("metal", rgba=(0.55, 0.56, 0.58, 1.0))
    diffuser = model.material("diffuser", rgba=(0.90, 0.92, 0.94, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.018, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="shoe_plate",
    )
    for index, x_pos in enumerate((-0.006, 0.006)):
        body.visual(
            Box((0.003, 0.026, 0.002)),
            origin=Origin(xyz=(x_pos, 0.0, 0.001)),
            material=metal,
            name=f"shoe_rail_{index}",
        )
    body.visual(
        Cylinder(radius=0.001, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
        material=metal,
        name="contact_pin",
    )
    body.visual(
        Box((0.022, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_gray,
        name="pedestal",
    )
    body.visual(
        Box((0.038, 0.030, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=matte_black,
        name="body_shell",
    )
    body.visual(
        Box((0.028, 0.002, 0.028)),
        origin=Origin(xyz=(0.0, -0.016, 0.039)),
        material=charcoal,
        name="rear_panel",
    )
    body.visual(
        Box((0.032, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=charcoal,
        name="shoulder",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_gray,
        name="collar",
    )
    neck.visual(
        Box((0.018, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=charcoal,
        name="column",
    )
    neck.visual(
        Box((0.070, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.007, 0.024)),
        material=charcoal,
        name="bridge",
    )
    for index, x_pos in enumerate((-0.033, 0.033)):
        neck.visual(
            Box((0.004, 0.018, 0.016)),
            origin=Origin(xyz=(x_pos, 0.0, 0.026)),
            material=charcoal,
            name=f"cheek_{index}",
        )

    head = model.part("head")
    head.visual(
        Box((0.040, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material=charcoal,
        name="hinge_block",
    )
    head.visual(
        Box((0.058, 0.032, 0.034)),
        origin=Origin(xyz=(0.0, 0.016, 0.015)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.050, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, 0.0335, 0.017)),
        material=diffuser,
        name="flash_window",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-1.75,
            upper=1.75,
        ),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.15,
            upper=1.57,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")

    def aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((low + high) * 0.5 for low, high in zip(min_corner, max_corner))

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        positive_elem="collar",
        negative_elem="shoulder",
        max_gap=0.0005,
        max_penetration=0.0,
        name="swivel collar seats on the body shoulder",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        positive_elem="head_shell",
        negative_elem="shoulder",
        min_gap=0.018,
        name="flash head clears the body at rest",
    )

    rest_window = aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
    tilt_limits = tilt.motion_limits
    swivel_limits = swivel.motion_limits

    if rest_window is None or tilt_limits is None or tilt_limits.upper is None:
        ctx.fail("flash head tilt pose is measurable", details=f"rest_window={rest_window}, limits={tilt_limits}")
    else:
        with ctx.pose({tilt: tilt_limits.upper}):
            raised_window = aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
        ctx.check(
            "flash head tilts upward",
            raised_window is not None
            and raised_window[2] > rest_window[2] + 0.012
            and raised_window[1] < rest_window[1] - 0.012,
            details=f"rest={rest_window}, raised={raised_window}",
        )

    if rest_window is None or swivel_limits is None or swivel_limits.upper is None:
        ctx.fail("flash head swivel pose is measurable", details=f"rest_window={rest_window}, limits={swivel_limits}")
    else:
        with ctx.pose({swivel: swivel_limits.upper}):
            swung_window = aabb_center(ctx.part_element_world_aabb(head, elem="flash_window"))
        ctx.check(
            "neck swivels the head sideways",
            swung_window is not None
            and abs(swung_window[0]) > abs(rest_window[0]) + 0.020
            and abs(swung_window[1]) < rest_window[1],
            details=f"rest={rest_window}, swung={swung_window}",
        )

    return ctx.report()


object_model = build_object_model()
