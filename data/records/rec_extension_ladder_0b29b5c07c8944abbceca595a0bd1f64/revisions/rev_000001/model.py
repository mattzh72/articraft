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


def _add_ladder_section(
    part,
    *,
    material: str,
    rail_width: float,
    rail_depth: float,
    rail_length: float,
    ladder_width: float,
    rung_length: float,
    rung_depth: float,
    rung_height: float,
    rung_positions: list[float],
    rung_y: float,
    add_feet: bool = False,
) -> None:
    rail_x = ladder_width / 2.0 - rail_width / 2.0

    for side_name, x_pos in (("left", -rail_x), ("right", rail_x)):
        part.visual(
            Box((rail_width, rail_depth, rail_length)),
            origin=Origin(xyz=(x_pos, 0.0, rail_length / 2.0)),
            material=material,
            name=f"{side_name}_rail",
        )

    if add_feet:
        for side_name, x_pos in (("left", -rail_x), ("right", rail_x)):
            part.visual(
                Box((rail_width + 0.014, rail_depth + 0.018, 0.036)),
                origin=Origin(xyz=(x_pos, 0.0, 0.018)),
                material="rubber",
                name=f"{side_name}_foot",
            )

    for index, z_pos in enumerate(rung_positions):
        part.visual(
            Box((rung_length, rung_depth, rung_height)),
            origin=Origin(xyz=(0.0, rung_y, z_pos)),
            material=material,
            name=f"rung_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder_with_rung_tray")

    model.material("aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("safety_orange", rgba=(0.88, 0.49, 0.14, 1.0))

    base = model.part("base_section")
    _add_ladder_section(
        base,
        material="aluminum",
        rail_width=0.038,
        rail_depth=0.072,
        rail_length=4.12,
        ladder_width=0.460,
        rung_length=0.402,
        rung_depth=0.030,
        rung_height=0.028,
        rung_positions=[0.34 + 0.305 * index for index in range(12)],
        rung_y=0.0,
        add_feet=True,
    )

    rail_x_base = 0.460 / 2.0 - 0.038 / 2.0
    for side_name, x_pos in (("left", -rail_x_base), ("right", rail_x_base)):
        base.visual(
            Box((0.028, 0.018, 0.080)),
            origin=Origin(xyz=(x_pos * 0.97, 0.022, 3.78)),
            material="graphite",
            name=f"{side_name}_guide_cap",
        )

    fly = model.part("fly_section")
    _add_ladder_section(
        fly,
        material="aluminum",
        rail_width=0.032,
        rail_depth=0.060,
        rail_length=3.05,
        ladder_width=0.390,
        rung_length=0.338,
        rung_depth=0.024,
        rung_height=0.024,
        rung_positions=[0.24 + 0.305 * index for index in range(10)],
        rung_y=-0.004,
    )

    rail_x_fly = 0.390 / 2.0 - 0.032 / 2.0
    for side_name, x_pos in (("left", -rail_x_fly), ("right", rail_x_fly)):
        fly.visual(
            Box((0.022, 0.018, 0.180)),
            origin=Origin(xyz=(x_pos, -0.021, 0.28)),
            material="graphite",
            name=f"{side_name}_lower_guide",
        )
        fly.visual(
            Box((0.022, 0.018, 0.180)),
            origin=Origin(xyz=(x_pos, -0.021, 1.10)),
            material="graphite",
            name=f"{side_name}_mid_guide",
        )
        fly.visual(
            Box((0.008, 0.018, 0.050)),
            origin=Origin(xyz=(-0.159 if side_name == "left" else 0.159, 0.031, 2.14)),
            material="graphite",
            name=f"{side_name}_tray_bracket",
        )

    tray = model.part("tray")
    tray.visual(
        Box((0.304, 0.018, 0.220)),
        origin=Origin(xyz=(0.0, 0.009, -0.110)),
        material="safety_orange",
        name="panel",
    )
    tray.visual(
        Box((0.304, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, 0.016, -0.211)),
        material="safety_orange",
        name="front_lip",
    )
    tray.visual(
        Box((0.014, 0.030, 0.048)),
        origin=Origin(xyz=(-0.145, 0.015, -0.024)),
        material="safety_orange",
        name="left_ear",
    )
    tray.visual(
        Box((0.014, 0.030, 0.048)),
        origin=Origin(xyz=(0.145, 0.015, -0.024)),
        material="safety_orange",
        name="right_ear",
    )
    tray.visual(
        Box((0.070, 0.034, 0.150)),
        origin=Origin(xyz=(0.0, 0.017, -0.103)),
        material="safety_orange",
        name="center_rib",
    )
    tray.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="graphite",
        name="left_hinge_pin",
    )
    tray.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="graphite",
        name="right_hinge_pin",
    )

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.066, 1.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.45,
            lower=0.0,
            upper=1.10,
        ),
    )
    model.articulation(
        "tray_hinge",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=tray,
        origin=Origin(xyz=(0.0, 0.034, 2.14)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    tray = object_model.get_part("tray")
    fly_slide = object_model.get_articulation("fly_slide")
    tray_hinge = object_model.get_articulation("tray_hinge")

    fly_limits = fly_slide.motion_limits
    tray_limits = tray_hinge.motion_limits
    fly_upper = 1.10 if fly_limits is None or fly_limits.upper is None else fly_limits.upper
    tray_open = 1.35 if tray_limits is None or tray_limits.upper is None else min(1.35, tray_limits.upper)

    ctx.expect_overlap(
        fly,
        base,
        axes="x",
        min_overlap=0.30,
        name="fly remains centered between the base rails",
    )

    rest_fly_pos = ctx.part_world_position(fly)
    with ctx.pose({fly_slide: fly_upper}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            min_overlap=2.0,
            name="extended fly keeps substantial rail engagement",
        )
        extended_fly_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly section extends upward",
        rest_fly_pos is not None
        and extended_fly_pos is not None
        and extended_fly_pos[2] > rest_fly_pos[2] + 1.0,
        details=f"rest={rest_fly_pos}, extended={extended_fly_pos}",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(tray, elem="panel")
    with ctx.pose({tray_hinge: tray_open}):
        ctx.expect_within(
            tray,
            fly,
            axes="x",
            inner_elem="panel",
            margin=0.02,
            name="opened tray stays between the fly rails",
        )
        deployed_panel_aabb = ctx.part_element_world_aabb(tray, elem="panel")

    ctx.check(
        "tray folds forward for painter access",
        rest_panel_aabb is not None
        and deployed_panel_aabb is not None
        and deployed_panel_aabb[1][1] > rest_panel_aabb[1][1] + 0.18,
        details=f"rest={rest_panel_aabb}, deployed={deployed_panel_aabb}",
    )
    ctx.check(
        "tray lifts into a near-level working step",
        rest_panel_aabb is not None
        and deployed_panel_aabb is not None
        and deployed_panel_aabb[0][2] > rest_panel_aabb[0][2] + 0.14,
        details=f"rest={rest_panel_aabb}, deployed={deployed_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
