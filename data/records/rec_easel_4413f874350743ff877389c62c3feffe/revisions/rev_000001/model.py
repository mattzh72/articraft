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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_sided_display_easel")

    aluminum = Material("satin_aluminum", color=(0.72, 0.74, 0.72, 1.0))
    dark_metal = Material("black_powder_coat", color=(0.015, 0.017, 0.018, 1.0))
    rubber = Material("matte_rubber", color=(0.02, 0.02, 0.018, 1.0))
    hardware = Material("brushed_hardware", color=(0.55, 0.56, 0.55, 1.0))
    nylon = Material("white_nylon", color=(0.86, 0.84, 0.78, 1.0))

    # World frame: X is the display width, Y is the double-sided direction,
    # and Z is vertical.  The foot base is intentionally broad and low.
    foot_base = model.part("foot_base")
    foot_base.visual(
        Box((0.82, 0.46, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_metal,
        name="base_plate",
    )
    foot_base.visual(
        Box((0.88, 0.065, 0.030)),
        origin=Origin(xyz=(0.0, 0.185, 0.015)),
        material=rubber,
        name="foot_pad_0",
    )
    foot_base.visual(
        Box((0.88, 0.065, 0.030)),
        origin=Origin(xyz=(0.0, -0.185, 0.015)),
        material=rubber,
        name="foot_pad_1",
    )

    # Hollow square outer sleeve made from four rails and a top collar; this
    # leaves real clearance around the sliding inner column instead of using a
    # solid proxy.
    sleeve_height = 0.68
    sleeve_center_z = 0.055 + sleeve_height / 2.0
    sleeve_wall = 0.016
    sleeve_outer = 0.120
    rail_xy = sleeve_outer / 2.0 - sleeve_wall / 2.0
    foot_base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(rail_xy, 0.0, sleeve_center_z)),
        material=aluminum,
        name="sleeve_wall_0",
    )
    foot_base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(-rail_xy, 0.0, sleeve_center_z)),
        material=aluminum,
        name="sleeve_wall_1",
    )
    foot_base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, rail_xy, sleeve_center_z)),
        material=aluminum,
        name="sleeve_wall_2",
    )
    foot_base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, -rail_xy, sleeve_center_z)),
        material=aluminum,
        name="sleeve_wall_3",
    )
    collar_z = 0.055 + sleeve_height + 0.010
    foot_base.visual(
        Box((sleeve_wall, sleeve_outer, 0.040)),
        origin=Origin(xyz=(rail_xy, 0.0, collar_z)),
        material=hardware,
        name="top_collar_0",
    )
    foot_base.visual(
        Box((sleeve_wall, sleeve_outer, 0.040)),
        origin=Origin(xyz=(-rail_xy, 0.0, collar_z)),
        material=hardware,
        name="top_collar_1",
    )
    foot_base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, 0.040)),
        origin=Origin(xyz=(0.0, rail_xy, collar_z)),
        material=hardware,
        name="top_collar_2",
    )
    foot_base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, 0.040)),
        origin=Origin(xyz=(0.0, -rail_xy, collar_z)),
        material=hardware,
        name="top_collar_3",
    )

    upright_column = model.part("upright_column")
    upright_column.visual(
        Box((0.052, 0.052, 1.320)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="inner_column",
    )
    upright_column.visual(
        Box((0.105, 0.070, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        material=hardware,
        name="top_head",
    )
    for pad_name, x, y in (
        ("glide_pad_0", 0.034, 0.0),
        ("glide_pad_1", -0.034, 0.0),
        ("glide_pad_2", 0.0, 0.034),
        ("glide_pad_3", 0.0, -0.034),
    ):
        upright_column.visual(
            Box((0.020 if x else 0.038, 0.038 if x else 0.020, 0.180)),
            origin=Origin(xyz=(x, y, -0.500)),
            material=nylon,
            name=pad_name,
        )

    # Opposite yokes at the top support the two display tray hinges.  The yoke
    # cheeks touch the tray hinge barrels at their end faces without occupying
    # the same space.
    for side, y, bridge_name, yoke_0_name, yoke_1_name in (
        ("front", 0.080, "front_yoke_bridge", "front_yoke_0", "front_yoke_1"),
        ("rear", -0.080, "rear_yoke_bridge", "rear_yoke_0", "rear_yoke_1"),
    ):
        upright_column.visual(
            Box((0.200, 0.054, 0.032)),
            origin=Origin(xyz=(0.0, y * 0.625, 0.610)),
            material=hardware,
            name=bridge_name,
        )
        for yoke_name, x in ((yoke_0_name, -0.086), (yoke_1_name, 0.086)):
            upright_column.visual(
                Box((0.028, 0.042, 0.090)),
                origin=Origin(xyz=(x, y, 0.660)),
                material=hardware,
                name=yoke_name,
            )

    model.articulation(
        "column_slide",
        ArticulationType.PRISMATIC,
        parent=foot_base,
        child=upright_column,
        origin=Origin(xyz=(0.0, 0.0, 0.745)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.20, lower=0.0, upper=0.32),
    )

    def add_tray(index: int, sign: float) -> None:
        """Add one side tray and its hinged top clip rail.

        sign=+1 faces +Y, sign=-1 faces -Y.
        """

        tray = model.part(f"tray_{index}")
        clip = model.part(f"clip_{index}")

        tilt = -sign * math.radians(20.0)
        length = 0.500
        start = 0.120
        center_y = sign * (start + 0.5 * length * math.cos(math.radians(20.0)))
        center_z = -0.5 * length * math.sin(math.radians(20.0))
        ledge_y = sign * (start + length * math.cos(math.radians(20.0)) + 0.018)
        ledge_z = -length * math.sin(math.radians(20.0)) - 0.030
        clip_y = sign * 0.300
        clip_z = 0.170

        tray.visual(
            Cylinder(radius=0.022, length=0.144),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="hinge_barrel",
        )
        tray.visual(
            Box((0.070, 0.135, 0.026)),
            origin=Origin(xyz=(0.0, sign * 0.062, -0.013)),
            material=hardware,
            name="hinge_neck",
        )
        tray.visual(
            Box((0.640, length, 0.032)),
            origin=Origin(xyz=(0.0, center_y, center_z), rpy=(tilt, 0.0, 0.0)),
            material=dark_metal,
            name="main_panel",
        )
        tray.visual(
            Box((0.640, 0.075, 0.055)),
            origin=Origin(xyz=(0.0, ledge_y, ledge_z)),
            material=dark_metal,
            name="lower_ledge",
        )
        tray.visual(
            Box((0.640, 0.020, 0.070)),
            origin=Origin(xyz=(0.0, ledge_y + sign * 0.038, ledge_z + 0.034)),
            material=dark_metal,
            name="front_lip",
        )
        for side_idx, x in enumerate((-0.315, 0.315)):
            tray.visual(
                Box((0.025, 0.040, 0.255)),
                origin=Origin(xyz=(x, clip_y, 0.050)),
                material=hardware,
                name=f"clip_post_{side_idx}",
            )
        for side_idx, x in enumerate((-0.315, 0.315)):
            tray.visual(
                Box((0.026, 0.410, 0.025)),
                origin=Origin(xyz=(x, sign * 0.365, -0.078), rpy=(tilt, 0.0, 0.0)),
                material=dark_metal,
                name=f"side_flange_{side_idx}",
            )

        model.articulation(
            f"tray_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=upright_column,
            child=tray,
            origin=Origin(xyz=(0.0, sign * 0.080, 0.660)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.30, upper=0.75),
        )

        clip.visual(
            Cylinder(radius=0.014, length=0.605),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="clip_hinge_pin",
        )
        clip.visual(
            Box((0.550, 0.030, 0.060)),
            origin=Origin(xyz=(0.0, sign * 0.026, -0.028)),
            material=hardware,
            name="clip_web",
        )
        clip.visual(
            Box((0.550, 0.036, 0.052)),
            origin=Origin(xyz=(0.0, sign * 0.046, -0.048)),
            material=dark_metal,
            name="clamp_bar",
        )
        clip.visual(
            Box((0.180, 0.040, 0.016)),
            origin=Origin(xyz=(-0.155, sign * 0.052, -0.082)),
            material=rubber,
            name="rubber_pad_0",
        )
        clip.visual(
            Box((0.180, 0.040, 0.016)),
            origin=Origin(xyz=(0.155, sign * 0.052, -0.082)),
            material=rubber,
            name="rubber_pad_1",
        )

        model.articulation(
            f"clip_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=tray,
            child=clip,
            origin=Origin(xyz=(0.0, clip_y, clip_z)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.20, upper=0.95),
        )

    add_tray(0, 1.0)
    add_tray(1, -1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    foot_base = object_model.get_part("foot_base")
    upright_column = object_model.get_part("upright_column")
    tray_0 = object_model.get_part("tray_0")
    tray_1 = object_model.get_part("tray_1")
    clip_0 = object_model.get_part("clip_0")
    clip_1 = object_model.get_part("clip_1")

    column_slide = object_model.get_articulation("column_slide")
    tray_hinge_0 = object_model.get_articulation("tray_hinge_0")
    tray_hinge_1 = object_model.get_articulation("tray_hinge_1")
    clip_hinge_0 = object_model.get_articulation("clip_hinge_0")
    clip_hinge_1 = object_model.get_articulation("clip_hinge_1")

    # The sliding column is retained inside a true hollow sleeve.  Nylon glides
    # touch each sleeve wall and the inner column remains inserted at full lift.
    ctx.expect_gap(
        foot_base,
        upright_column,
        axis="x",
        positive_elem="sleeve_wall_0",
        negative_elem="glide_pad_0",
        max_gap=0.001,
        max_penetration=0.00001,
        name="positive x glide touches sleeve",
    )
    ctx.expect_gap(
        upright_column,
        foot_base,
        axis="x",
        positive_elem="glide_pad_1",
        negative_elem="sleeve_wall_1",
        max_gap=0.001,
        max_penetration=0.00001,
        name="negative x glide touches sleeve",
    )
    ctx.expect_gap(
        foot_base,
        upright_column,
        axis="y",
        positive_elem="sleeve_wall_2",
        negative_elem="glide_pad_2",
        max_gap=0.001,
        max_penetration=0.00001,
        name="positive y glide touches sleeve",
    )
    ctx.expect_gap(
        upright_column,
        foot_base,
        axis="y",
        positive_elem="glide_pad_3",
        negative_elem="sleeve_wall_3",
        max_gap=0.001,
        max_penetration=0.00001,
        name="negative y glide touches sleeve",
    )
    ctx.expect_overlap(
        upright_column,
        foot_base,
        axes="z",
        elem_a="inner_column",
        elem_b="sleeve_wall_0",
        min_overlap=0.50,
        name="collapsed column remains deeply inserted",
    )

    rest_column_position = ctx.part_world_position(upright_column)
    with ctx.pose({column_slide: 0.32}):
        ctx.expect_overlap(
            upright_column,
            foot_base,
            axes="z",
            elem_a="inner_column",
            elem_b="sleeve_wall_0",
            min_overlap=0.25,
            name="extended column retains sleeve insertion",
        )
        extended_column_position = ctx.part_world_position(upright_column)
    ctx.check(
        "prismatic column raises the tray head",
        rest_column_position is not None
        and extended_column_position is not None
        and extended_column_position[2] > rest_column_position[2] + 0.30,
        details=f"rest={rest_column_position}, extended={extended_column_position}",
    )

    # The two tray arms are on opposite sides of the column and their hinge
    # barrels sit against the top yoke cheeks.
    ctx.expect_origin_gap(
        tray_0,
        upright_column,
        axis="y",
        min_gap=0.075,
        max_gap=0.085,
        name="front tray hinge is on positive side",
    )
    ctx.expect_origin_gap(
        upright_column,
        tray_1,
        axis="y",
        min_gap=0.075,
        max_gap=0.085,
        name="rear tray hinge is on negative side",
    )
    ctx.expect_gap(
        upright_column,
        tray_0,
        axis="x",
        positive_elem="front_yoke_1",
        negative_elem="hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="front tray barrel seats in yoke",
    )
    ctx.expect_gap(
        upright_column,
        tray_1,
        axis="x",
        positive_elem="rear_yoke_1",
        negative_elem="hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear tray barrel seats in yoke",
    )

    ctx.expect_gap(
        clip_0,
        tray_0,
        axis="z",
        positive_elem="clamp_bar",
        negative_elem="main_panel",
        min_gap=0.05,
        name="front clip rail sits above tray panel",
    )
    ctx.expect_gap(
        clip_1,
        tray_1,
        axis="z",
        positive_elem="clamp_bar",
        negative_elem="main_panel",
        min_gap=0.05,
        name="rear clip rail sits above tray panel",
    )

    def element_mid_z(part, elem: str) -> float | None:
        box = ctx.part_element_world_aabb(part, elem=elem)
        if box is None:
            return None
        return 0.5 * (box[0][2] + box[1][2])

    tray_0_rest_z = element_mid_z(tray_0, "lower_ledge")
    tray_1_rest_z = element_mid_z(tray_1, "lower_ledge")
    with ctx.pose({tray_hinge_0: 0.75, tray_hinge_1: 0.75}):
        tray_0_open_z = element_mid_z(tray_0, "lower_ledge")
        tray_1_open_z = element_mid_z(tray_1, "lower_ledge")
    ctx.check(
        "both display trays swing upward from top hinges",
        tray_0_rest_z is not None
        and tray_1_rest_z is not None
        and tray_0_open_z is not None
        and tray_1_open_z is not None
        and tray_0_open_z > tray_0_rest_z + 0.25
        and tray_1_open_z > tray_1_rest_z + 0.25,
        details=(
            f"front rest/open={tray_0_rest_z}/{tray_0_open_z}, "
            f"rear rest/open={tray_1_rest_z}/{tray_1_open_z}"
        ),
    )

    clip_0_rest_z = element_mid_z(clip_0, "clamp_bar")
    clip_1_rest_z = element_mid_z(clip_1, "clamp_bar")
    with ctx.pose({clip_hinge_0: 0.95, clip_hinge_1: 0.95}):
        clip_0_open_z = element_mid_z(clip_0, "clamp_bar")
        clip_1_open_z = element_mid_z(clip_1, "clamp_bar")
    ctx.check(
        "both top clip rails lift on their hinges",
        clip_0_rest_z is not None
        and clip_1_rest_z is not None
        and clip_0_open_z is not None
        and clip_1_open_z is not None
        and clip_0_open_z > clip_0_rest_z + 0.04
        and clip_1_open_z > clip_1_rest_z + 0.04,
        details=(
            f"front rest/open={clip_0_rest_z}/{clip_0_open_z}, "
            f"rear rest/open={clip_1_rest_z}/{clip_1_open_z}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
