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


def _add_section(
    part,
    *,
    prefix: str,
    rail_center_y: float,
    rail_width: float,
    rail_depth: float,
    length: float,
    rung_radius: float,
    rung_count: int,
    rung_bottom: float,
    rung_top: float,
    rail_material,
    rung_material,
) -> None:
    for side, side_name in ((-1.0, "0"), (1.0, "1")):
        part.visual(
            Box((rail_depth, rail_width, length)),
            origin=Origin(xyz=(rail_depth * 0.5, side * rail_center_y, length * 0.5)),
            material=rail_material,
            name=f"{prefix}_rail_{side_name}",
        )

    rung_span = (2.0 * rail_center_y) - rail_width + 0.004
    for index in range(rung_count):
        t = index / max(rung_count - 1, 1)
        z_pos = rung_bottom + (rung_top - rung_bottom) * t
        part.visual(
            Cylinder(radius=rung_radius, length=rung_span),
            origin=Origin(
                xyz=(rail_depth * 0.52, 0.0, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rung_material,
            name=f"{prefix}_rung_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    aluminum_dark = model.material("aluminum_dark", rgba=(0.64, 0.67, 0.71, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    plastic = model.material("plastic", rgba=(0.10, 0.10, 0.11, 1.0))

    base_section = model.part("base_section")
    base_length = 4.35
    base_rail_center_y = 0.205
    base_rail_width = 0.032
    base_rail_depth = 0.068
    _add_section(
        base_section,
        prefix="base",
        rail_center_y=base_rail_center_y,
        rail_width=base_rail_width,
        rail_depth=base_rail_depth,
        length=base_length,
        rung_radius=0.016,
        rung_count=14,
        rung_bottom=0.42,
        rung_top=4.02,
        rail_material=aluminum,
        rung_material=aluminum_dark,
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        base_section.visual(
            Box((0.076, 0.042, 0.050)),
            origin=Origin(xyz=(0.038, side * base_rail_center_y, 0.025)),
            material=rubber,
            name=f"foot_{suffix}",
        )
        base_section.visual(
            Box((0.040, 0.026, 0.090)),
            origin=Origin(xyz=(0.020, side * base_rail_center_y, base_length - 0.110)),
            material=plastic,
            name=f"top_guide_{suffix}",
        )

    fly_section = model.part("fly_section")
    fly_length = 3.55
    fly_rail_center_y = 0.182
    fly_rail_width = 0.028
    fly_rail_depth = 0.056
    _add_section(
        fly_section,
        prefix="fly",
        rail_center_y=fly_rail_center_y,
        rail_width=fly_rail_width,
        rail_depth=fly_rail_depth,
        length=fly_length,
        rung_radius=0.015,
        rung_count=12,
        rung_bottom=0.34,
        rung_top=3.24,
        rail_material=aluminum,
        rung_material=aluminum_dark,
    )
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        fly_section.visual(
            Box((0.016, 0.040, 0.100)),
            origin=Origin(
                xyz=(0.008, side * (fly_rail_center_y + 0.020), 0.62),
            ),
            material=plastic,
            name=f"guide_shoe_{suffix}",
        )
        fly_section.visual(
            Box((0.014, 0.038, 0.085)),
            origin=Origin(
                xyz=(0.049, side * (fly_rail_center_y + 0.022), fly_length - 0.085),
            ),
            material=aluminum_dark,
            name=f"hinge_bracket_{suffix}",
        )

    standoff_bar = model.part("standoff_bar")
    arm_y = 0.235
    arm_thickness_x = 0.022
    arm_thickness_y = 0.020
    arm_length = 0.345
    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        standoff_bar.visual(
            Cylinder(radius=0.011, length=0.068),
            origin=Origin(
                xyz=(0.0, side * arm_y, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=aluminum_dark,
            name=f"hinge_barrel_{suffix}",
        )
        standoff_bar.visual(
            Box((arm_thickness_x, arm_thickness_y, arm_length)),
            origin=Origin(
                xyz=(0.0, side * arm_y, arm_length * 0.5 - 0.010),
            ),
            material=aluminum,
            name=f"arm_{suffix}",
        )
    standoff_bar.visual(
        Box((0.026, (2.0 * arm_y) + 0.030, 0.022)),
        origin=Origin(xyz=(0.004, 0.0, arm_length - 0.014)),
        material=aluminum,
        name="crossbar",
    )
    standoff_bar.visual(
        Box((0.018, (2.0 * arm_y) + 0.010, 0.012)),
        origin=Origin(xyz=(0.016, 0.0, arm_length - 0.050)),
        material=aluminum_dark,
        name="crossbar_brace",
    )

    fly_rest_bottom_z = 1.08
    fly_extension = 1.32
    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=fly_section,
        origin=Origin(xyz=(base_rail_depth, 0.0, fly_rest_bottom_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=fly_extension,
            effort=650.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "standoff_hinge",
        ArticulationType.REVOLUTE,
        parent=fly_section,
        child=standoff_bar,
        origin=Origin(xyz=(fly_rail_depth + 0.011, 0.0, fly_length - 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.15,
            effort=35.0,
            velocity=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fly_section = object_model.get_part("fly_section")
    standoff_bar = object_model.get_part("standoff_bar")
    fly_slide = object_model.get_articulation("fly_slide")
    standoff_hinge = object_model.get_articulation("standoff_hinge")

    ctx.expect_gap(
        fly_section,
        base_section,
        axis="x",
        positive_elem="fly_rail_0",
        negative_elem="base_rail_0",
        max_penetration=0.0,
        max_gap=0.0005,
        name="fly rail back face stays seated on the base rail guide plane",
    )
    ctx.expect_within(
        fly_section,
        base_section,
        axes="y",
        margin=0.0,
        name="fly section stays centered between the base rails",
    )
    ctx.expect_overlap(
        fly_section,
        base_section,
        axes="z",
        min_overlap=2.40,
        name="collapsed fly section remains deeply nested in the base section",
    )
    ctx.expect_gap(
        standoff_bar,
        fly_section,
        axis="x",
        positive_elem="crossbar",
        max_penetration=0.0,
        max_gap=0.020,
        name="folded stand-off bar nests close to the fly section",
    )

    rest_position = ctx.part_world_position(fly_section)
    with ctx.pose({fly_slide: 1.32}):
        ctx.expect_gap(
            fly_section,
            base_section,
            axis="x",
            positive_elem="fly_rail_0",
            negative_elem="base_rail_0",
            max_penetration=0.0,
            max_gap=0.0005,
            name="extended fly rail still tracks on the base guide plane",
        )
        ctx.expect_within(
            fly_section,
            base_section,
            axes="y",
            margin=0.0,
            name="extended fly section still tracks between the base rails",
        )
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=1.15,
            name="extended fly section retains enough overlap for engagement",
        )
        extended_position = ctx.part_world_position(fly_section)

    ctx.check(
        "fly section extends upward",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 1.0,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    with ctx.pose({standoff_hinge: 1.05}):
        ctx.expect_gap(
            standoff_bar,
            fly_section,
            axis="x",
            positive_elem="crossbar",
            min_gap=0.22,
            name="deployed stand-off bar holds its crossbar away from the ladder face",
        )
        ctx.expect_overlap(
            standoff_bar,
            fly_section,
            axes="y",
            elem_a="crossbar",
            min_overlap=0.34,
            name="stand-off crossbar still spans the ladder width when deployed",
        )

    return ctx.report()


object_model = build_object_model()
