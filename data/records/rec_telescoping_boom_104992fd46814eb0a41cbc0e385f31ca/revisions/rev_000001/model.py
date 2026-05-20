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


def _add_rect_tube(
    part,
    *,
    x_min: float,
    x_max: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    material,
    prefix: str,
) -> None:
    """Add a four-wall, open-ended rectangular tube using primitive wall solids."""
    length = x_max - x_min
    center_x = (x_min + x_max) * 0.5
    part.visual(
        Box((length, outer_y, wall)),
        origin=Origin(xyz=(center_x, 0.0, outer_z * 0.5 - wall * 0.5)),
        material=material,
        name=f"{prefix}_top",
    )
    part.visual(
        Box((length, outer_y, wall)),
        origin=Origin(xyz=(center_x, 0.0, -outer_z * 0.5 + wall * 0.5)),
        material=material,
        name=f"{prefix}_bottom",
    )
    part.visual(
        Box((length, wall, outer_z)),
        origin=Origin(xyz=(center_x, -outer_y * 0.5 + wall * 0.5, 0.0)),
        material=material,
        name=f"{prefix}_side_neg",
    )
    part.visual(
        Box((length, wall, outer_z)),
        origin=Origin(xyz=(center_x, outer_y * 0.5 - wall * 0.5, 0.0)),
        material=material,
        name=f"{prefix}_side_pos",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_telescoping_reach")

    wall_mat = model.material("painted_wall", rgba=(0.78, 0.78, 0.74, 1.0))
    mount_mat = model.material("powder_coated_mount", rgba=(0.08, 0.09, 0.10, 1.0))
    screw_mat = model.material("dark_screw_heads", rgba=(0.02, 0.02, 0.02, 1.0))
    outer_mat = model.material("blue_outer_sleeve", rgba=(0.05, 0.17, 0.33, 1.0))
    stage0_mat = model.material("brushed_stage_0", rgba=(0.68, 0.72, 0.73, 1.0))
    stage1_mat = model.material("brushed_stage_1", rgba=(0.78, 0.80, 0.78, 1.0))
    stage2_mat = model.material("bright_inner_bar", rgba=(0.86, 0.87, 0.83, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    mount = model.part("wall_mount")
    mount.visual(
        Box((0.025, 0.550, 0.420)),
        origin=Origin(xyz=(-0.0725, 0.0, 0.0)),
        material=wall_mat,
        name="wall_panel",
    )
    mount.visual(
        Box((0.035, 0.340, 0.280)),
        origin=Origin(xyz=(-0.0425, 0.0, 0.0)),
        material=mount_mat,
        name="rear_plate",
    )
    mount.visual(
        Box((0.025, 0.180, 0.140)),
        origin=Origin(xyz=(-0.0125, 0.0, 0.0)),
        material=mount_mat,
        name="saddle_block",
    )
    for y in (-0.125, 0.125):
        for z in (-0.100, 0.100):
            mount.visual(
                Cylinder(radius=0.017, length=0.007),
                origin=Origin(xyz=(-0.0215, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=screw_mat,
                name=f"screw_{'neg' if y < 0 else 'pos'}_{'low' if z < 0 else 'high'}",
            )

    outer = model.part("outer_sleeve")
    _add_rect_tube(
        outer,
        x_min=0.000,
        x_max=0.620,
        outer_y=0.140,
        outer_z=0.105,
        wall=0.012,
        material=outer_mat,
        prefix="sleeve",
    )
    _add_rect_tube(
        outer,
        x_min=0.570,
        x_max=0.620,
        outer_y=0.158,
        outer_z=0.123,
        wall=0.016,
        material=outer_mat,
        prefix="front_collar",
    )
    _add_rect_tube(
        outer,
        x_min=0.000,
        x_max=0.035,
        outer_y=0.176,
        outer_z=0.140,
        wall=0.018,
        material=outer_mat,
        prefix="rear_flange",
    )

    section_0 = model.part("section_0")
    _add_rect_tube(
        section_0,
        x_min=-0.340,
        x_max=0.280,
        outer_y=0.104,
        outer_z=0.071,
        wall=0.009,
        material=stage0_mat,
        prefix="section",
    )
    _add_rect_tube(
        section_0,
        x_min=0.240,
        x_max=0.280,
        outer_y=0.116,
        outer_z=0.083,
        wall=0.013,
        material=stage0_mat,
        prefix="front_collar",
    )
    for side, y in (("neg", -0.0545), ("pos", 0.0545)):
        section_0.visual(
            Box((0.100, 0.007, 0.024)),
            origin=Origin(xyz=(-0.280, y, 0.0)),
            material=rubber_mat,
            name=f"side_pad_{side}",
        )

    section_1 = model.part("section_1")
    _add_rect_tube(
        section_1,
        x_min=-0.250,
        x_max=0.240,
        outer_y=0.078,
        outer_z=0.047,
        wall=0.007,
        material=stage1_mat,
        prefix="section",
    )
    _add_rect_tube(
        section_1,
        x_min=0.204,
        x_max=0.240,
        outer_y=0.088,
        outer_z=0.057,
        wall=0.011,
        material=stage1_mat,
        prefix="front_collar",
    )
    for side, y in (("neg", -0.0405), ("pos", 0.0405)):
        section_1.visual(
            Box((0.050, 0.005, 0.016)),
            origin=Origin(xyz=(-0.225, y, 0.0)),
            material=rubber_mat,
            name=f"side_pad_{side}",
        )

    section_2 = model.part("section_2")
    section_2.visual(
        Box((0.390, 0.055, 0.029)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=stage2_mat,
        name="inner_bar",
    )
    section_2.visual(
        Box((0.050, 0.075, 0.052)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=rubber_mat,
        name="end_tip",
    )
    for side, y in (("neg", -0.0295), ("pos", 0.0295)):
        section_2.visual(
            Box((0.045, 0.005, 0.012)),
            origin=Origin(xyz=(-0.1675, y, 0.0)),
            material=rubber_mat,
            name=f"side_pad_{side}",
        )

    model.articulation(
        "mount_to_outer",
        ArticulationType.FIXED,
        parent=mount,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_section_0",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=section_0,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.220),
    )
    model.articulation(
        "section_0_to_1",
        ArticulationType.PRISMATIC,
        parent=section_0,
        child=section_1,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.32, lower=0.0, upper=0.170),
    )
    model.articulation(
        "section_1_to_2",
        ArticulationType.PRISMATIC,
        parent=section_1,
        child=section_2,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.30, lower=0.0, upper=0.130),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mount = object_model.get_part("wall_mount")
    outer = object_model.get_part("outer_sleeve")
    section_0 = object_model.get_part("section_0")
    section_1 = object_model.get_part("section_1")
    section_2 = object_model.get_part("section_2")

    j0 = object_model.get_articulation("outer_to_section_0")
    j1 = object_model.get_articulation("section_0_to_1")
    j2 = object_model.get_articulation("section_1_to_2")

    ctx.expect_gap(
        outer,
        mount,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem="saddle_block",
        name="outer sleeve seats on rear mount",
    )
    ctx.expect_overlap(
        outer,
        mount,
        axes="yz",
        min_overlap=0.080,
        elem_b="saddle_block",
        name="mount footprint carries sleeve",
    )

    with ctx.pose({j0: 0.0, j1: 0.0, j2: 0.0}):
        ctx.expect_overlap(
            section_0,
            outer,
            axes="x",
            min_overlap=0.320,
            name="section 0 retained in outer sleeve at rest",
        )
        ctx.expect_overlap(
            section_1,
            section_0,
            axes="x",
            min_overlap=0.230,
            name="section 1 retained in section 0 at rest",
        )
        ctx.expect_overlap(
            section_2,
            section_1,
            axes="x",
            min_overlap=0.170,
            elem_a="inner_bar",
            name="section 2 retained in section 1 at rest",
        )
        rest_tip = ctx.part_world_position(section_2)

    with ctx.pose({j0: 0.220, j1: 0.170, j2: 0.130}):
        ctx.expect_overlap(
            section_0,
            outer,
            axes="x",
            min_overlap=0.105,
            name="section 0 overlap remains extended",
        )
        ctx.expect_overlap(
            section_1,
            section_0,
            axes="x",
            min_overlap=0.075,
            name="section 1 overlap remains extended",
        )
        ctx.expect_overlap(
            section_2,
            section_1,
            axes="x",
            min_overlap=0.055,
            elem_a="inner_bar",
            name="section 2 overlap remains extended",
        )
        extended_tip = ctx.part_world_position(section_2)

    ctx.check(
        "serial sections extend along shared axis",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[0] > rest_tip[0] + 0.500,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
