from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LEG_X = (-0.45, 0.45)
OUTER_TOP_Z = 0.82
LIFT_TRAVEL = 0.35
HINGE_LOCAL = (0.0, -0.33, 0.245)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _add_square_sleeve(base, x: float, index: int, material: Material) -> None:
    """Four connected walls forming a clear square telescoping sleeve."""
    zc = (0.058 + OUTER_TOP_Z) * 0.5
    height = OUTER_TOP_Z - 0.058
    wall = 0.020
    outer = 0.142
    half = outer * 0.5 - wall * 0.5
    # +X / -X walls.
    for sign, suffix in ((1.0, "x_pos"), (-1.0, "x_neg")):
        base.visual(
            Box((wall, outer, height)),
            origin=Origin(xyz=(x + sign * half, 0.0, zc)),
            material=material,
            name=f"sleeve_{index}_{suffix}_wall",
        )
    # +Y / -Y walls. They overlap the side walls at the corners, making one
    # welded tube-shaped sleeve rather than four floating strips.
    for sign, suffix in ((1.0, "rear"), (-1.0, "front")):
        base.visual(
            Box((outer, wall, height)),
            origin=Origin(xyz=(x, sign * half, zc)),
            material=material,
            name=f"sleeve_{index}_{suffix}_wall",
        )
    # A thicker top collar makes the tube lip visible while preserving a clear opening.
    base.visual(
        Box((outer + 0.035, 0.026, 0.035)),
        origin=Origin(xyz=(x, half, OUTER_TOP_Z - 0.017)),
        material=material,
        name=f"sleeve_{index}_rear_collar",
    )
    base.visual(
        Box((outer + 0.035, 0.026, 0.035)),
        origin=Origin(xyz=(x, -half, OUTER_TOP_Z - 0.017)),
        material=material,
        name=f"sleeve_{index}_front_collar",
    )
    base.visual(
        Box((0.026, outer + 0.035, 0.035)),
        origin=Origin(xyz=(x + half, 0.0, OUTER_TOP_Z - 0.017)),
        material=material,
        name=f"sleeve_{index}_x_pos_collar",
    )
    base.visual(
        Box((0.026, outer + 0.035, 0.035)),
        origin=Origin(xyz=(x - half, 0.0, OUTER_TOP_Z - 0.017)),
        material=material,
        name=f"sleeve_{index}_x_neg_collar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_standing_desk")

    dark_steel = _mat(model, "dark powder coated steel", (0.035, 0.038, 0.040, 1.0))
    satin_steel = _mat(model, "brushed telescoping steel", (0.58, 0.60, 0.60, 1.0))
    black = _mat(model, "matte black plastic", (0.005, 0.005, 0.006, 1.0))
    rubber = _mat(model, "black rubber", (0.01, 0.01, 0.012, 1.0))
    wood = _mat(model, "warm laminated maple", (0.78, 0.58, 0.34, 1.0))
    edge = _mat(model, "dark edge banding", (0.10, 0.075, 0.045, 1.0))
    button_mat = _mat(model, "soft grey buttons", (0.16, 0.17, 0.18, 1.0))

    base = model.part("base_frame")
    # Two heavy floor feet, joined by cross braces, carry the square outer sleeves.
    for i, x in enumerate(LEG_X):
        base.visual(
            Box((0.19, 0.88, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.030)),
            material=dark_steel,
            name=f"foot_{i}",
        )
        base.visual(
            Box((0.22, 0.095, 0.030)),
            origin=Origin(xyz=(x, -0.385, 0.018)),
            material=rubber,
            name=f"front_glide_{i}",
        )
        base.visual(
            Box((0.22, 0.095, 0.030)),
            origin=Origin(xyz=(x, 0.385, 0.018)),
            material=rubber,
            name=f"rear_glide_{i}",
        )
        _add_square_sleeve(base, x, i, dark_steel)

    for y, name in ((-0.34, "front_floor_tie"), (0.34, "rear_floor_tie")):
        base.visual(
            Box((1.05, 0.075, 0.090)),
            origin=Origin(xyz=(0.0, y, 0.080)),
            material=dark_steel,
            name=name,
        )

    lift = model.part("lift_frame")
    # Both inner stages are one synchronized lifting carriage: two visible
    # telescoping members linked by a top frame so the worktop stays level.
    for i, x in enumerate(LEG_X):
        lift.visual(
            Box((0.064, 0.064, 0.640)),
            origin=Origin(xyz=(x, 0.0, -0.155)),
            material=satin_steel,
            name=f"inner_stage_{i}",
        )
        # Low-friction guide shoes touch the inside faces of the sleeve so the
        # moving stage is visibly supported without filling the hollow tube.
        for sign, suffix in ((1.0, "x_pos"), (-1.0, "x_neg")):
            lift.visual(
                Box((0.020, 0.050, 0.110)),
                origin=Origin(xyz=(x + sign * 0.041, 0.0, -0.270)),
                material=black,
                name=f"stage_{i}_{suffix}_shoe",
            )
        for sign, suffix in ((1.0, "rear"), (-1.0, "front")):
            lift.visual(
                Box((0.050, 0.020, 0.110)),
                origin=Origin(xyz=(x, sign * 0.041, -0.270)),
                material=black,
                name=f"stage_{i}_{suffix}_shoe",
            )
        lift.visual(
            Box((0.105, 0.105, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.173)),
            material=black,
            name=f"stage_cap_{i}",
        )

    lift.visual(
        Box((1.12, 0.105, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=dark_steel,
        name="top_crossbar",
    )
    for x, name in ((-0.45, "side_rail_0"), (0.45, "side_rail_1")):
        lift.visual(
            Box((0.085, 0.69, 0.055)),
            origin=Origin(xyz=(x, -0.165, 0.205)),
            material=dark_steel,
            name=name,
        )
    lift.visual(
        Box((1.16, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, -0.33, 0.220)),
        material=dark_steel,
        name="front_hinge_rail",
    )
    lift.visual(
        Cylinder(radius=0.023, length=1.18),
        origin=Origin(xyz=HINGE_LOCAL, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="tilt_hinge_pin",
    )
    for x, name in ((-0.52, "hinge_cheek_0"), (0.52, "hinge_cheek_1")):
        lift.visual(
            Box((0.050, 0.070, 0.135)),
            origin=Origin(xyz=(x, -0.33, 0.206)),
            material=dark_steel,
            name=name,
        )

    worktop = model.part("worktop")
    # The work surface frame is the front transverse hinge line. The board
    # extends rearward along local +Y, as on a drafting table whose rear edge rises.
    worktop.visual(
        Box((1.26, 0.78, 0.052)),
        origin=Origin(xyz=(0.0, 0.390, 0.058)),
        material=wood,
        name="board_panel",
    )
    worktop.visual(
        Box((1.29, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, 0.026, 0.085)),
        material=edge,
        name="front_paper_lip",
    )
    worktop.visual(
        Box((1.29, 0.032, 0.060)),
        origin=Origin(xyz=(0.0, 0.776, 0.058)),
        material=edge,
        name="rear_edge_band",
    )
    for x, name in ((-0.645, "side_edge_0"), (0.645, "side_edge_1")):
        worktop.visual(
            Box((0.030, 0.78, 0.058)),
            origin=Origin(xyz=(x, 0.390, 0.058)),
            material=edge,
            name=name,
        )
    for x, name in ((-0.33, "hinge_leaf_0"), (0.33, "hinge_leaf_1")):
        worktop.visual(
            Box((0.250, 0.040, 0.028)),
            origin=Origin(xyz=(x, 0.055, 0.019)),
            material=satin_steel,
            name=name,
        )

    control = model.part("control_strip")
    control.visual(
        Box((0.39, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="strip_housing",
    )
    control.visual(
        Box((0.31, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.028, 0.029)),
        material=dark_steel,
        name="button_bezel",
    )
    control.visual(
        Box((0.31, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.028, -0.023)),
        material=dark_steel,
        name="button_bezel_lower",
    )
    for x, name in ((-0.157, "bezel_side_0"), (-0.0525, "bezel_divider_0"), (0.0525, "bezel_divider_1"), (0.157, "bezel_side_1")):
        control.visual(
            Box((0.006, 0.006, 0.046)),
            origin=Origin(xyz=(x, -0.028, 0.003)),
            material=dark_steel,
            name=name,
        )

    buttons = []
    for i, x in enumerate((-0.105, 0.0, 0.105)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.057, 0.018, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.030, 0.030, 0.018)),
            origin=Origin(xyz=(0.0, 0.016, 0.0)),
            material=black,
            name="guide_stem",
        )
        buttons.append((button, x))

    lift_joint = model.articulation(
        "base_to_lift_frame",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.10, lower=0.0, upper=LIFT_TRAVEL),
    )
    # The hinge is carried by the moving lift frame and lets the board tilt upward.
    model.articulation(
        "lift_frame_to_worktop",
        ArticulationType.REVOLUTE,
        parent=lift,
        child=worktop,
        origin=Origin(xyz=HINGE_LOCAL),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.8, lower=0.0, upper=0.65),
    )
    model.articulation(
        "worktop_to_control_strip",
        ArticulationType.FIXED,
        parent=worktop,
        child=control,
        # Mounted on the front lower edge, just below the paper lip.
        origin=Origin(xyz=(0.0, -0.025, 0.058)),
    )
    for i, (button, x) in enumerate(buttons):
        model.articulation(
            f"control_strip_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=control,
            child=button,
            origin=Origin(xyz=(x, -0.034, 0.004)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    # Keep the two-column carriage synchronized in authoring metadata for review.
    lift_joint.meta["mechanism"] = "synchronized two-column standing-desk lift"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    lift = object_model.get_part("lift_frame")
    worktop = object_model.get_part("worktop")
    control = object_model.get_part("control_strip")
    lift_joint = object_model.get_articulation("base_to_lift_frame")
    tilt_joint = object_model.get_articulation("lift_frame_to_worktop")

    ctx.expect_overlap(
        lift,
        base,
        axes="z",
        elem_a="inner_stage_0",
        elem_b="sleeve_0_front_wall",
        min_overlap=0.35,
        name="stage 0 is retained in sleeve at low height",
    )
    ctx.expect_overlap(
        lift,
        base,
        axes="z",
        elem_a="inner_stage_1",
        elem_b="sleeve_1_front_wall",
        min_overlap=0.35,
        name="stage 1 is retained in sleeve at low height",
    )

    rest_lift = ctx.part_world_position(lift)
    rest_top_aabb = ctx.part_element_world_aabb(worktop, elem="board_panel")
    with ctx.pose({lift_joint: LIFT_TRAVEL}):
        raised_lift = ctx.part_world_position(lift)
        ctx.expect_overlap(
            lift,
            base,
            axes="z",
            elem_a="inner_stage_0",
            elem_b="sleeve_0_front_wall",
            min_overlap=0.10,
            name="stage 0 remains inserted at full lift",
        )
        ctx.expect_overlap(
            lift,
            base,
            axes="z",
            elem_a="inner_stage_1",
            elem_b="sleeve_1_front_wall",
            min_overlap=0.10,
            name="stage 1 remains inserted at full lift",
        )
    ctx.check(
        "lifting frame travels upward",
        rest_lift is not None and raised_lift is not None and raised_lift[2] > rest_lift[2] + 0.30,
        details=f"rest={rest_lift}, raised={raised_lift}",
    )

    with ctx.pose({tilt_joint: 0.60}):
        tilted_top_aabb = ctx.part_element_world_aabb(worktop, elem="board_panel")
    ctx.check(
        "worktop tilt raises rear board edge",
        rest_top_aabb is not None
        and tilted_top_aabb is not None
        and tilted_top_aabb[1][2] > rest_top_aabb[1][2] + 0.25,
        details=f"rest={rest_top_aabb}, tilted={tilted_top_aabb}",
    )

    ctx.expect_contact(
        worktop,
        control,
        elem_a="board_panel",
        elem_b="strip_housing",
        contact_tol=0.003,
        name="control strip is mounted to front edge",
    )

    for i in range(3):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"control_strip_to_button_{i}")
        ctx.allow_overlap(
            button,
            control,
            elem_a="guide_stem",
            elem_b="strip_housing",
            reason="Each button stem is intentionally captured inside the solid control-strip proxy as a short prismatic guide.",
        )
        ctx.expect_overlap(
            button,
            control,
            axes="y",
            elem_a="guide_stem",
            elem_b="strip_housing",
            min_overlap=0.015,
            name=f"button {i} guide stem remains inserted",
        )
        ctx.expect_contact(
            button,
            control,
            elem_a="button_cap",
            elem_b="strip_housing",
            contact_tol=0.0015,
            name=f"button {i} rests on strip face",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {i} moves inward on guide",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
