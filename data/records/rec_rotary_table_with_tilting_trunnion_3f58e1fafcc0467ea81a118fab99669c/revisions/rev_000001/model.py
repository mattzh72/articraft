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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_trunnion_rotary_table")

    cast_blue = model.material("painted_cast_iron_blue", color=(0.08, 0.18, 0.31, 1.0))
    dark_blue = model.material("dark_enamel_blue", color=(0.03, 0.08, 0.14, 1.0))
    machined = model.material("machined_steel", color=(0.60, 0.62, 0.60, 1.0))
    dark_slot = model.material("blackened_t_slot_shadow", color=(0.015, 0.015, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.36, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_blue,
        name="floor_foot",
    )
    base.visual(
        Cylinder(radius=0.18, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=cast_blue,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=0.23, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.2525)),
        material=machined,
        name="fixed_bearing_race",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.32, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=machined,
        name="turntable_platter",
    )
    turntable.visual(
        Cylinder(radius=0.26, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=dark_slot,
        name="top_register_circle",
    )

    # Two cheek frames rise from the rotating platter.  The split box layout
    # leaves a real window around the trunnion shaft instead of burying the
    # shaft in a solid block.
    for i, x in enumerate((-0.31, 0.31)):
        turntable.visual(
            Box((0.06, 0.30, 0.22)),
            origin=Origin(xyz=(x, 0.0, 0.177)),
            material=cast_blue,
            name=f"cheek_lower_{i}",
        )
        turntable.visual(
            Box((0.06, 0.30, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.4375)),
            material=cast_blue,
            name=f"cheek_cap_{i}",
        )
        for j, y in enumerate((-0.125, 0.125)):
            turntable.visual(
                Box((0.06, 0.055, 0.18)),
                origin=Origin(xyz=(x, y, 0.348)),
                material=cast_blue,
                name=f"cheek_post_{i}_{j}",
            )
    turntable.visual(
        mesh_from_geometry(TorusGeometry(radius=0.048, tube=0.022, radial_segments=32, tubular_segments=48), "bearing_ring_0"),
        origin=Origin(xyz=(-0.31, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="bearing_ring_0",
    )
    turntable.visual(
        mesh_from_geometry(TorusGeometry(radius=0.048, tube=0.022, radial_segments=32, tubular_segments=48), "bearing_ring_1"),
        origin=Origin(xyz=(0.31, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="bearing_ring_1",
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.225, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined,
        name="face_plate",
    )
    table.visual(
        Cylinder(radius=0.060, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="center_hub",
    )
    table.visual(
        Cylinder(radius=0.028, length=0.75),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="tilt_shaft",
    )
    for i, x in enumerate((-0.36, 0.36)):
        table.visual(
            Cylinder(radius=0.042, length=0.025),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=machined,
            name=f"shaft_collar_{i}",
        )
    for i, y in enumerate((-0.075, 0.075)):
        table.visual(
            Box((0.34, 0.022, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.029)),
            material=dark_slot,
            name=f"t_slot_{i}",
        )

    model.articulation(
        "base_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )

    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=table,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.5, lower=-1.35, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    turntable = object_model.get_part("turntable")
    table = object_model.get_part("table")
    base_spin = object_model.get_articulation("base_spin")
    table_tilt = object_model.get_articulation("table_tilt")

    ctx.check(
        "two visible revolute axes",
        base_spin.axis == (0.0, 0.0, 1.0) and table_tilt.axis == (1.0, 0.0, 0.0),
        details=f"base axis={base_spin.axis}, tilt axis={table_tilt.axis}",
    )
    ctx.allow_overlap(
        turntable,
        table,
        elem_a="bearing_ring_0",
        elem_b="tilt_shaft",
        reason="The modeled trunnion shaft is intentionally captured by the left bearing bushing with a tiny interference fit.",
    )
    ctx.allow_overlap(
        turntable,
        table,
        elem_a="bearing_ring_1",
        elem_b="tilt_shaft",
        reason="The modeled trunnion shaft is intentionally captured by the right bearing bushing with a tiny interference fit.",
    )
    ctx.expect_within(
        table,
        turntable,
        axes="xy",
        inner_elem="face_plate",
        outer_elem="turntable_platter",
        margin=0.0,
        name="work face sits within rotary platter footprint",
    )
    ctx.expect_overlap(
        table,
        turntable,
        axes="x",
        elem_a="tilt_shaft",
        elem_b="bearing_ring_0",
        min_overlap=0.015,
        name="shaft passes through one trunnion bearing",
    )
    ctx.expect_within(
        table,
        turntable,
        axes="yz",
        inner_elem="tilt_shaft",
        outer_elem="bearing_ring_0",
        margin=0.0,
        name="shaft is centered in one bearing bore",
    )
    ctx.expect_overlap(
        table,
        turntable,
        axes="x",
        elem_a="tilt_shaft",
        elem_b="bearing_ring_1",
        min_overlap=0.015,
        name="shaft passes through opposite trunnion bearing",
    )
    ctx.expect_within(
        table,
        turntable,
        axes="yz",
        inner_elem="tilt_shaft",
        outer_elem="bearing_ring_1",
        margin=0.0,
        name="shaft is centered in opposite bearing bore",
    )

    rest_aabb = ctx.part_world_aabb(table)
    with ctx.pose({table_tilt: 1.0}):
        tilted_aabb = ctx.part_world_aabb(table)
    if rest_aabb is None or tilted_aabb is None:
        ctx.fail("tilt pose can be measured", "missing table AABB")
    else:
        rest_span = rest_aabb[1][2] - rest_aabb[0][2]
        tilted_span = tilted_aabb[1][2] - tilted_aabb[0][2]
        ctx.check(
            "table visibly tilts about trunnion shaft",
            tilted_span > rest_span + 0.12,
            details=f"rest_z_span={rest_span:.3f}, tilted_z_span={tilted_span:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
