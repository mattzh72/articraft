from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _cheek_plate_mesh(depth: float, height: float, hole_radius: float, name: str):
    outer = [
        (-depth / 2.0, -height / 2.0),
        (depth / 2.0, -height / 2.0),
        (depth / 2.0, height / 2.0),
        (-depth / 2.0, height / 2.0),
    ]
    plate = ExtrudeWithHolesGeometry(
        outer,
        [list(reversed(_circle_profile(hole_radius, 64)))],
        0.055,
        center=True,
    )
    return mesh_from_geometry(plate, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_trunnion_table")

    cast_iron = model.material("oiled_cast_iron", rgba=(0.18, 0.19, 0.19, 1.0))
    dark_iron = model.material("dark_blued_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    bright_steel = model.material("bolt_bright_steel", rgba=(0.78, 0.78, 0.72, 1.0))
    slot_black = model.material("shadowed_t_slot", rgba=(0.015, 0.016, 0.018, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.265, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_iron,
        name="fixed_round_base",
    )
    pedestal.visual(
        Cylinder(radius=0.225, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=ground_steel,
        name="lower_bearing_race",
    )
    for index in range(12):
        angle = 2.0 * math.pi * index / 12.0
        radius = 0.250
        pedestal.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.056)
            ),
            material=bright_steel,
            name=f"flange_bolt_{index}",
        )

    rotary_base = model.part("rotary_base")
    rotary_base.visual(
        Cylinder(radius=0.235, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_iron,
        name="rotary_platter",
    )
    rotary_base.visual(
        Box((0.560, 0.390, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=cast_iron,
        name="saddle_block",
    )
    rotary_base.visual(
        Cylinder(radius=0.195, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=ground_steel,
        name="machined_rotary_face",
    )
    for groove, radius in enumerate((0.145, 0.180)):
        rotary_base.visual(
            mesh_from_geometry(TorusGeometry(radius=radius, tube=0.0025), f"rotary_groove_{groove}"),
            origin=Origin(xyz=(0.0, 0.0, 0.118)),
            material=slot_black,
            name=f"rotary_groove_{groove}",
        )

    table_axis_z = 0.285
    cheek_x = 0.275
    for side, x in enumerate((-cheek_x, cheek_x)):
        upper_bearing_name = ("upper_bearing_0", "upper_bearing_1")[side]
        lower_bearing_name = ("lower_bearing_0", "lower_bearing_1")[side]
        rotary_base.visual(
            Box((0.055, 0.360, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.130)),
            material=cast_iron,
            name=f"cheek_foot_{side}",
        )
        rotary_base.visual(
            Box((0.055, 0.360, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.420)),
            material=cast_iron,
            name=f"cheek_bridge_{side}",
        )
        for post, y in enumerate((-0.155, 0.155)):
            rotary_base.visual(
                Box((0.055, 0.050, 0.300)),
                origin=Origin(xyz=(x, y, 0.275)),
                material=cast_iron,
                name=f"cheek_post_{side}_{post}",
            )
        rotary_base.visual(
            Box((0.060, 0.125, 0.024)),
            origin=Origin(xyz=(x, 0.0, table_axis_z + 0.054)),
            material=ground_steel,
            name=upper_bearing_name,
        )
        rotary_base.visual(
            Box((0.060, 0.125, 0.024)),
            origin=Origin(xyz=(x, 0.0, table_axis_z - 0.054)),
            material=ground_steel,
            name=lower_bearing_name,
        )
        for pad, y in enumerate((-0.080, 0.080)):
            rotary_base.visual(
                Box((0.060, 0.024, 0.130)),
                origin=Origin(xyz=(x, y, table_axis_z)),
                material=ground_steel,
                name=f"side_bearing_{side}_{pad}",
            )
            rotary_base.visual(
                Box((0.055, 0.048, 0.026)),
                origin=Origin(xyz=(x, -0.111 if y < 0.0 else 0.111, table_axis_z)),
                material=dark_iron,
                name=f"side_web_{side}_{pad}",
            )
        rotary_base.visual(
            Box((0.055, 0.050, 0.016)),
            origin=Origin(xyz=(x, 0.0, table_axis_z + 0.098)),
            material=dark_iron,
            name=f"upper_web_{side}",
        )
        rotary_base.visual(
            Box((0.055, 0.050, 0.035)),
            origin=Origin(xyz=(x, 0.0, table_axis_z - 0.108)),
            material=dark_iron,
            name=f"lower_web_{side}",
        )

        outer_x = x + (-0.036 if x < 0.0 else 0.036)
        rotary_base.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.078, tube=0.011, radial_segments=18, tubular_segments=48),
                f"trunnion_flange_{side}",
            ),
            origin=Origin(xyz=(outer_x, 0.0, table_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=ground_steel,
            name=f"trunnion_flange_{side}",
        )
        for bolt in range(6):
            angle = 2.0 * math.pi * bolt / 6.0
            bolt_radius = 0.084
            rotary_base.visual(
                Cylinder(radius=0.0065, length=0.012),
                origin=Origin(
                    xyz=(
                        outer_x + (-0.008 if x < 0.0 else 0.008),
                        bolt_radius * math.cos(angle),
                        table_axis_z + bolt_radius * math.sin(angle),
                    ),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=bright_steel,
                name=f"trunnion_bolt_{side}_{bolt}",
            )

        for y in (-0.158, 0.158):
            rotary_base.visual(
                Box((0.080, 0.026, 0.255)),
                origin=Origin(xyz=(x, y, 0.228)),
                material=dark_iron,
                name=f"cheek_rib_{side}_{0 if y < 0 else 1}",
            )

    table = model.part("table")
    table.visual(
        Box((0.420, 0.280, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=ground_steel,
        name="table_slab",
    )
    for slot, y in enumerate((-0.085, 0.0, 0.085)):
        table.visual(
            Box((0.350, 0.020, 0.006)),
            origin=Origin(xyz=(0.0, y, 0.038)),
            material=slot_black,
            name=f"t_slot_{slot}",
        )
    for side, x in enumerate((-0.253, 0.253)):
        journal_name = ("trunnion_journal_0", "trunnion_journal_1")[side]
        table.visual(
            Cylinder(radius=0.042, length=0.138),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=ground_steel,
            name=journal_name,
        )
        table.visual(
            Cylinder(radius=0.064, length=0.018),
            origin=Origin(
                xyz=(-0.224 if x < 0.0 else 0.224, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=ground_steel,
            name=f"trunnion_shoulder_{side}",
        )
        table.visual(
            Box((0.030, 0.118, 0.098)),
            origin=Origin(xyz=(-0.208 if x < 0.0 else 0.208, 0.0, 0.0)),
            material=ground_steel,
            name=f"table_lug_{side}",
        )
        table.visual(
            Cylinder(radius=0.036, length=0.014),
            origin=Origin(
                xyz=(-0.323 if x < 0.0 else 0.323, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_iron,
            name=f"journal_end_{side}",
        )

    model.articulation(
        "pedestal_to_base",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=rotary_base,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0),
    )
    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=rotary_base,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, table_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=-0.90, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    rotary_base = object_model.get_part("rotary_base")
    table = object_model.get_part("table")
    base_spin = object_model.get_articulation("pedestal_to_base")
    table_tilt = object_model.get_articulation("base_to_table")

    ctx.expect_contact(
        rotary_base,
        pedestal,
        elem_a="rotary_platter",
        elem_b="fixed_round_base",
        name="rotary platter sits on the fixed base",
    )
    ctx.expect_gap(
        table,
        rotary_base,
        axis="z",
        min_gap=0.030,
        positive_elem="table_slab",
        negative_elem="machined_rotary_face",
        name="table clears the rotary face at level",
    )
    ctx.expect_overlap(
        table,
        rotary_base,
        axes="x",
        elem_a="trunnion_journal_0",
        elem_b="upper_bearing_0",
        min_overlap=0.030,
        name="first journal spans cheek thickness",
    )
    ctx.expect_contact(
        rotary_base,
        table,
        elem_a="upper_bearing_0",
        elem_b="trunnion_journal_0",
        contact_tol=0.001,
        name="first upper bearing touches journal",
    )
    ctx.expect_contact(
        table,
        rotary_base,
        elem_a="trunnion_journal_0",
        elem_b="lower_bearing_0",
        contact_tol=0.001,
        name="first lower bearing touches journal",
    )
    ctx.expect_overlap(
        table,
        rotary_base,
        axes="x",
        elem_a="trunnion_journal_1",
        elem_b="upper_bearing_1",
        min_overlap=0.030,
        name="second journal spans cheek thickness",
    )
    ctx.expect_contact(
        rotary_base,
        table,
        elem_a="upper_bearing_1",
        elem_b="trunnion_journal_1",
        contact_tol=0.001,
        name="second upper bearing touches journal",
    )
    ctx.expect_contact(
        table,
        rotary_base,
        elem_a="trunnion_journal_1",
        elem_b="lower_bearing_1",
        contact_tol=0.001,
        name="second lower bearing touches journal",
    )

    with ctx.pose({table_tilt: 0.90}):
        ctx.expect_gap(
            table,
            rotary_base,
            axis="z",
            min_gap=0.025,
            positive_elem="table_slab",
            negative_elem="machined_rotary_face",
            name="table clears base at forward tilt",
        )
    with ctx.pose({table_tilt: -0.90}):
        ctx.expect_gap(
            table,
            rotary_base,
            axis="z",
            min_gap=0.025,
            positive_elem="table_slab",
            negative_elem="machined_rotary_face",
            name="table clears base at rear tilt",
        )
    with ctx.pose({base_spin: math.pi / 2.0, table_tilt: 0.45}):
        ctx.expect_gap(
            table,
            rotary_base,
            axis="z",
            min_gap=0.020,
            positive_elem="table_slab",
            negative_elem="machined_rotary_face",
            name="combined spin and tilt stays above rotary face",
        )

    return ctx.report()


object_model = build_object_model()
