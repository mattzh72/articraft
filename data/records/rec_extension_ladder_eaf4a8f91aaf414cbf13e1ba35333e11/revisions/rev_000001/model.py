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


def _add_rung(
    part,
    *,
    z: float,
    y: float,
    length: float,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    aluminum_dark = model.material("aluminum_dark", rgba=(0.56, 0.59, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    warning = model.material("warning", rgba=(0.85, 0.67, 0.10, 1.0))

    rail_x_base = 0.175
    rail_x_fly = 0.160
    base_depth_y = 0.026
    fly_depth_y = 0.022
    fly_y = 0.040
    hinge_y = 0.059
    hinge_z = 5.52

    base_section = model.part("base_section")
    for idx, rail_x in enumerate((-rail_x_base, rail_x_base)):
        base_section.visual(
            Box((0.074, base_depth_y, 5.60)),
            origin=Origin(xyz=(rail_x, 0.0, 2.80)),
            material=aluminum,
            name=f"base_rail_{idx}",
        )
        base_section.visual(
            Box((0.086, 0.048, 0.050)),
            origin=Origin(xyz=(rail_x, 0.0, 0.025)),
            material=rubber,
            name=f"foot_{idx}",
        )
        base_section.visual(
            Box((0.074, 0.032, 0.070)),
            origin=Origin(xyz=(rail_x, 0.0, 5.565)),
            material=aluminum_dark,
            name=f"top_cap_{idx}",
        )

    base_rung_positions = [0.43 + 0.305 * i for i in range(16)]
    for idx, z in enumerate(base_rung_positions):
        _add_rung(
            base_section,
            z=z,
            y=0.0,
            length=0.320,
            radius=0.017,
            material=aluminum_dark,
            name=f"base_rung_{idx}",
        )

    base_section.visual(
        Box((0.090, 0.028, 0.160)),
        origin=Origin(xyz=(-rail_x_base, 0.002, 5.08)),
        material=aluminum_dark,
        name="guide_hook_0",
    )
    base_section.visual(
        Box((0.090, 0.028, 0.160)),
        origin=Origin(xyz=(rail_x_base, 0.002, 5.08)),
        material=aluminum_dark,
        name="guide_hook_1",
    )
    base_section.visual(
        Box((0.280, 0.022, 0.050)),
        origin=Origin(xyz=(0.0, 0.010, 5.23)),
        material=warning,
        name="top_stop",
    )

    fly_section = model.part("fly_section")
    for idx, rail_x in enumerate((-rail_x_fly, rail_x_fly)):
        fly_section.visual(
            Box((0.060, fly_depth_y, 4.75)),
            origin=Origin(xyz=(rail_x, fly_y, 3.275)),
            material=aluminum,
            name=f"fly_rail_{idx}",
        )
        fly_section.visual(
            Box((0.060, 0.030, 0.070)),
            origin=Origin(xyz=(rail_x, fly_y, 5.615)),
            material=aluminum_dark,
            name=f"fly_cap_{idx}",
        )
        for shoe_idx, shoe_z in enumerate((1.30, 2.35, 3.40, 4.45)):
            fly_section.visual(
                Box((0.034, 0.016, 0.100)),
                origin=Origin(xyz=(rail_x, 0.021, shoe_z)),
                material=aluminum_dark,
                name=f"guide_shoe_{idx}_{shoe_idx}",
            )

    fly_rung_positions = [1.20 + 0.305 * i for i in range(13)]
    for idx, z in enumerate(fly_rung_positions):
        _add_rung(
            fly_section,
            z=z,
            y=fly_y,
            length=0.286,
            radius=0.016,
            material=aluminum_dark,
            name=f"fly_rung_{idx}",
        )

    fly_section.visual(
        Box((0.390, 0.026, 0.060)),
        origin=Origin(xyz=(0.0, fly_y + 0.005, 5.40)),
        material=aluminum_dark,
        name="top_bridge",
    )
    fly_section.visual(
        Box((0.210, 0.018, 0.042)),
        origin=Origin(xyz=(0.0, fly_y + 0.024, 5.30)),
        material=warning,
        name="bumper_block",
    )
    fly_section.visual(
        Box((0.070, 0.016, 0.090)),
        origin=Origin(xyz=(0.0, fly_y + 0.016, 5.349)),
        material=aluminum_dark,
        name="bumper_web",
    )

    for idx, rail_x in enumerate((-rail_x_fly, rail_x_fly)):
        for barrel_idx, barrel_x in enumerate((rail_x - 0.013, rail_x + 0.013)):
            fly_section.visual(
                Cylinder(radius=0.0085, length=0.010),
                origin=Origin(
                    xyz=(barrel_x, hinge_y, hinge_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=aluminum_dark,
                name=f"hinge_knuckle_{idx}_{barrel_idx}",
            )

    for idx in range(2):
        standoff = model.part(f"standoff_{idx}")
        standoff.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum_dark,
            name="barrel",
        )
        standoff.visual(
            Box((0.028, 0.018, 0.440)),
            origin=Origin(xyz=(0.0, 0.018, -0.220)),
            material=aluminum,
            name="arm",
        )
        standoff.visual(
            Box((0.090, 0.034, 0.018)),
            origin=Origin(xyz=(0.0, 0.021, -0.438)),
            material=rubber,
            name="pad",
        )
        standoff.visual(
            Box((0.018, 0.016, 0.052)),
            origin=Origin(xyz=(0.0, 0.008, -0.026)),
            material=aluminum_dark,
            name="boss",
        )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base_section,
        child=fly_section,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=2.10,
        ),
    )
    model.articulation(
        "standoff_0_hinge",
        ArticulationType.REVOLUTE,
        parent=fly_section,
        child="standoff_0",
        origin=Origin(xyz=(-rail_x_fly, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "standoff_1_hinge",
        ArticulationType.REVOLUTE,
        parent=fly_section,
        child="standoff_1",
        origin=Origin(xyz=(rail_x_fly, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fly_section = object_model.get_part("fly_section")
    fly_slide = object_model.get_articulation("fly_slide")
    standoff_0 = object_model.get_part("standoff_0")
    standoff_1 = object_model.get_part("standoff_1")
    standoff_0_hinge = object_model.get_articulation("standoff_0_hinge")
    standoff_1_hinge = object_model.get_articulation("standoff_1_hinge")

    ctx.expect_overlap(
        fly_section,
        base_section,
        axes="x",
        min_overlap=0.22,
        name="fly section stays centered on the ladder width",
    )
    ctx.expect_overlap(
        fly_section,
        base_section,
        axes="z",
        min_overlap=4.00,
        name="collapsed fly remains deeply nested in the base section",
    )
    ctx.expect_gap(
        standoff_0,
        fly_section,
        axis="y",
        positive_elem="arm",
        negative_elem="fly_rail_0",
        max_gap=0.020,
        max_penetration=0.0,
        name="first standoff folds flat to the fly rail",
    )
    ctx.expect_gap(
        standoff_1,
        fly_section,
        axis="y",
        positive_elem="arm",
        negative_elem="fly_rail_1",
        max_gap=0.020,
        max_penetration=0.0,
        name="second standoff folds flat to the fly rail",
    )

    rest_pos = ctx.part_world_position(fly_section)
    slide_upper = fly_slide.motion_limits.upper if fly_slide.motion_limits is not None else None
    with ctx.pose({fly_slide: slide_upper}):
        ctx.expect_overlap(
            fly_section,
            base_section,
            axes="z",
            min_overlap=2.50,
            name="extended fly retains insertion in the base section",
        )
        extended_pos = ctx.part_world_position(fly_section)

    ctx.check(
        "fly section extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.9,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    deployed_angle = 1.25
    with ctx.pose(
        {
            standoff_0_hinge: deployed_angle,
            standoff_1_hinge: deployed_angle,
        }
    ):
        ctx.expect_gap(
            standoff_0,
            fly_section,
            axis="y",
            positive_elem="pad",
            negative_elem="fly_rail_0",
            min_gap=0.23,
            name="first standoff swings out to hold off the wall",
        )
        ctx.expect_gap(
            standoff_1,
            fly_section,
            axis="y",
            positive_elem="pad",
            negative_elem="fly_rail_1",
            min_gap=0.23,
            name="second standoff swings out to hold off the wall",
        )

    return ctx.report()


object_model = build_object_model()
