from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.60
BODY_DEPTH = 0.30
BODY_HEIGHT = 0.16
SHELL_THICKNESS = 0.014
SIDE_DEPTH = 0.276
VISOR_TRAVEL = 0.125
BUTTON_TRAVEL = 0.006


def add_box(part, size, xyz, *, material, name):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slide_out_visor_range_hood")

    shell_finish = model.material("shell_finish", rgba=(0.72, 0.74, 0.76, 1.0))
    visor_finish = model.material("visor_finish", rgba=(0.63, 0.65, 0.68, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.20, 0.21, 0.22, 1.0))
    button_finish = model.material("button_finish", rgba=(0.88, 0.88, 0.86, 1.0))
    guide_finish = model.material("guide_finish", rgba=(0.35, 0.37, 0.40, 1.0))

    hood = model.part("hood")
    add_box(
        hood,
        (BODY_WIDTH, BODY_DEPTH, SHELL_THICKNESS),
        (0.0, BODY_DEPTH / 2.0, BODY_HEIGHT - SHELL_THICKNESS / 2.0),
        material=shell_finish,
        name="top_shell",
    )
    add_box(
        hood,
        (BODY_WIDTH, SHELL_THICKNESS, BODY_HEIGHT - SHELL_THICKNESS),
        (0.0, SHELL_THICKNESS / 2.0, (BODY_HEIGHT - SHELL_THICKNESS) / 2.0),
        material=shell_finish,
        name="rear_panel",
    )
    for side_name, x_sign in (("side_wall_0", -1.0), ("side_wall_1", 1.0)):
        add_box(
            hood,
            (SHELL_THICKNESS, SIDE_DEPTH, BODY_HEIGHT - SHELL_THICKNESS),
            (
                x_sign * (BODY_WIDTH / 2.0 - SHELL_THICKNESS / 2.0),
                SIDE_DEPTH / 2.0,
                (BODY_HEIGHT - SHELL_THICKNESS) / 2.0,
            ),
            material=shell_finish,
            name=side_name,
        )
    add_box(
        hood,
        (BODY_WIDTH, 0.022, 0.032),
        (0.0, BODY_DEPTH - 0.011, BODY_HEIGHT - 0.016),
        material=trim_finish,
        name="front_header",
    )
    add_box(
        hood,
        (BODY_WIDTH - 0.02, 0.018, 0.010),
        (0.0, 0.050, 0.010),
        material=trim_finish,
        name="rear_lamp_rail",
    )
    for guide_name, x_sign in (("guide_0", -1.0), ("guide_1", 1.0)):
        add_box(
            hood,
            (0.014, 0.205, 0.008),
            (
                x_sign * 0.279,
                0.165,
                0.055,
            ),
            material=guide_finish,
            name=guide_name,
        )

    visor = model.part("visor")
    add_box(
        visor,
        (0.552, 0.264, 0.006),
        (0.0, -0.152, 0.015),
        material=visor_finish,
        name="visor_panel",
    )
    add_box(
        visor,
        (0.590, 0.020, 0.046),
        (0.0, -0.010, 0.000),
        material=visor_finish,
        name="front_rail",
    )
    add_box(
        visor,
        (0.560, 0.014, 0.012),
        (0.0, -0.027, -0.014),
        material=trim_finish,
        name="lower_grip",
    )
    for runner_name, x_sign in (("runner_0", -1.0), ("runner_1", 1.0)):
        add_box(
            visor,
            (0.010, 0.192, 0.006),
            (x_sign * 0.268, -0.154, 0.020),
            material=guide_finish,
            name=runner_name,
        )

    button_offsets = (-0.060, 0.0, 0.060)
    button_names = ("button_0", "button_1", "button_2")
    for button_name in button_names:
        button = model.part(button_name)
        add_box(
            button,
            (0.016, 0.007, 0.010),
            (0.0, 0.0035, 0.0),
            material=button_finish,
            name="cap",
        )
        add_box(
            button,
            (0.010, 0.012, 0.012),
            (0.0, -0.006, 0.0),
            material=button_finish,
            name="stem",
        )

    model.articulation(
        "hood_to_visor",
        ArticulationType.PRISMATIC,
        parent=hood,
        child=visor,
        origin=Origin(xyz=(0.0, BODY_DEPTH, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=VISOR_TRAVEL,
        ),
    )

    for button_name, x_pos in zip(button_names, button_offsets):
        model.articulation(
            f"visor_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=visor,
            child=button_name,
            origin=Origin(xyz=(x_pos, 0.0, -0.006)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.10,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hood = object_model.get_part("hood")
    visor = object_model.get_part("visor")
    visor_slide = object_model.get_articulation("hood_to_visor")

    for button_name in ("button_0", "button_1", "button_2"):
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(f"visor_to_{button_name}")
        ctx.allow_overlap(
            button,
            visor,
            elem_a="stem",
            elem_b="front_rail",
            reason="The push-button stem occupies a simplified internal switch cavity inside the visor rail.",
        )
        ctx.expect_within(
            button,
            visor,
            axes="xz",
            inner_elem="cap",
            outer_elem="front_rail",
            margin=0.002,
            name=f"{button_name} sits within the visor rail face",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{button_name} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    with ctx.pose({visor_slide: 0.0}):
        ctx.expect_contact(
            hood,
            visor,
            elem_a="guide_0",
            elem_b="runner_0",
            name="left runner rides on the left internal guide",
        )
        ctx.expect_contact(
            hood,
            visor,
            elem_a="guide_1",
            elem_b="runner_1",
            name="right runner rides on the right internal guide",
        )
        ctx.expect_overlap(
            visor,
            hood,
            axes="x",
            min_overlap=0.52,
            name="closed visor remains centered under the hood shell",
        )

    rest_pos = ctx.part_world_position(visor)
    with ctx.pose({visor_slide: VISOR_TRAVEL}):
        extended_pos = ctx.part_world_position(visor)
        ctx.expect_overlap(
            visor,
            hood,
            axes="y",
            min_overlap=0.14,
            name="extended visor keeps enough retained insertion in the shell guides",
        )
        ctx.expect_overlap(
            visor,
            hood,
            axes="x",
            min_overlap=0.52,
            name="extended visor stays laterally captured by the hood body",
        )
    ctx.check(
        "visor extends forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
