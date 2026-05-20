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
    model = ArticulatedObject(name="compact_linear_adjustment_module")

    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.02, 0.02, 0.02, 1.0))
    anodized_blue = model.material("anodized_blue", rgba=(0.05, 0.20, 0.55, 1.0))
    rubber = model.material("rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.360, 0.140, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.284, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=brushed_steel,
        name="guide_rail",
    )
    for idx, x in enumerate((-0.140, 0.140)):
        base.visual(
            Box((0.024, 0.086, 0.052)),
            origin=Origin(xyz=(x, 0.0, 0.044)),
            material=dark_steel,
            name=("stop_0" if idx == 0 else "stop_1"),
        )
        base.visual(
            Box((0.010, 0.034, 0.008)),
            origin=Origin(xyz=(x * 0.98, 0.0, 0.044)),
            material=black_oxide,
            name=f"rail_cap_{idx}",
        )
        for y in (-0.026, 0.026):
            base.visual(
                Cylinder(radius=0.0045, length=0.004),
                origin=Origin(xyz=(x, y, 0.072)),
                material=black_oxide,
                name=f"stop_screw_{idx}_{0 if y < 0 else 1}",
            )
    for x in (-0.150, 0.150):
        for y in (-0.052, 0.052):
            base.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(x, y, 0.020)),
                material=black_oxide,
                name=f"mount_screw_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    clamp = model.part("clamp_block")
    clamp.visual(
        Box((0.070, 0.074, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=anodized_blue,
        name="carriage_shell",
    )
    for side_idx, y in enumerate((-0.027, 0.027)):
        clamp.visual(
            Box((0.070, 0.020, 0.021)),
            origin=Origin(xyz=(0.0, y, -0.0195)),
            material=anodized_blue,
            name=f"carriage_wall_{side_idx}",
        )
    clamp.visual(
        Box((0.045, 0.046, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material=rubber,
        name="top_pad",
    )
    for idx, x in enumerate((-0.015, 0.015)):
        clamp.visual(
            Cylinder(radius=0.0035, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.023)),
            material=black_oxide,
            name=f"clamp_stem_{idx}",
        )
        clamp.visual(
            Cylinder(radius=0.0085, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=black_oxide,
            name=f"clamp_screw_{idx}",
        )

    model.articulation(
        "rail_to_clamp",
        ArticulationType.PRISMATIC,
        parent=base,
        child=clamp,
        origin=Origin(xyz=(-0.087, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.174),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    clamp = object_model.get_part("clamp_block")
    slide = object_model.get_articulation("rail_to_clamp")

    ctx.expect_contact(
        clamp,
        base,
        elem_a="carriage_shell",
        elem_b="guide_rail",
        contact_tol=0.001,
        name="carriage bears on the guide rail",
    )
    ctx.expect_within(
        clamp,
        base,
        axes="y",
        inner_elem="carriage_shell",
        outer_elem="guide_rail",
        margin=0.030,
        name="carriage wraps the rail laterally",
    )
    ctx.expect_gap(
        clamp,
        base,
        axis="x",
        positive_elem="carriage_shell",
        negative_elem="stop_0",
        min_gap=0.005,
        name="lower travel stop remains clear",
    )

    rest_pos = ctx.part_world_position(clamp)
    with ctx.pose({slide: 0.174}):
        ctx.expect_contact(
            clamp,
            base,
            elem_a="carriage_shell",
            elem_b="guide_rail",
            contact_tol=0.001,
            name="extended carriage still rides on rail",
        )
        ctx.expect_gap(
            base,
            clamp,
            axis="x",
            positive_elem="stop_1",
            negative_elem="carriage_shell",
            min_gap=0.005,
            name="upper travel stop remains clear",
        )
        extended_pos = ctx.part_world_position(clamp)

    ctx.check(
        "prismatic joint advances along rail",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.16,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
