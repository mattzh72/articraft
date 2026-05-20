from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


MAIN_BEARING_Z = 0.17
OFFSET_AXIS_X = 0.62
UPPER_BEARING_Z = 0.43


def _add_radial_slots(part, *, radius: float, z: float, length: float, width: float, thickness: float, count: int, name: str) -> None:
    """Raised dark inserts that read as machined T-slots on a rotary face."""
    inner_r = radius - length * 0.5
    slot_center_r = inner_r + length * 0.5
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        part.visual(
            Box((length, width, thickness)),
            origin=Origin(
                xyz=(slot_center_r * math.cos(angle), slot_center_r * math.sin(angle), z),
                rpy=(0.0, 0.0, angle),
            ),
            material="slot_shadow",
            name=f"{name}_{i}",
        )


def _add_bolt_circle(
    part,
    *,
    radius: float,
    z: float,
    bolt_radius: float,
    height: float,
    count: int,
    name: str,
    material: str = "blackened_steel",
    center: tuple[float, float] = (0.0, 0.0),
) -> None:
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        part.visual(
            Cylinder(radius=bolt_radius, length=height),
            origin=Origin(xyz=(center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle), z)),
            material=material,
            name=f"{name}_{i}",
        )


def _rib_mesh(length: float, height: float, thickness: float, name: str):
    """Triangular side gusset, extruded in local Y after a +90 deg roll."""
    profile = [(-length * 0.5, 0.0), (length * 0.5, 0.0), (length * 0.5, height)]
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_indexing_head")

    model.material("cast_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("brushed_face", rgba=(0.76, 0.77, 0.74, 1.0))
    model.material("dark_oil", rgba=(0.045, 0.050, 0.055, 1.0))
    model.material("slot_shadow", rgba=(0.02, 0.025, 0.030, 1.0))
    model.material("blackened_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    model.material("index_blue", rgba=(0.12, 0.18, 0.26, 1.0))

    bed = model.part("bed")
    bed.visual(
        Box((1.02, 0.62, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material="cast_iron",
        name="bed_casting",
    )
    bed.visual(
        Box((0.92, 0.11, 0.028)),
        origin=Origin(xyz=(0.0, 0.275, 0.094)),
        material="cast_iron",
        name="front_way",
    )
    bed.visual(
        Box((0.92, 0.11, 0.028)),
        origin=Origin(xyz=(0.0, -0.275, 0.094)),
        material="cast_iron",
        name="rear_way",
    )
    bed.visual(
        Cylinder(radius=0.30, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material="cast_iron",
        name="pedestal_bearing",
    )
    bed.visual(
        Cylinder(radius=0.215, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material="dark_oil",
        name="lower_oil_seal",
    )
    bed.visual(
        Cylinder(radius=0.120, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.163)),
        material="machined_steel",
        name="center_spigot",
    )
    for x in (-0.43, 0.43):
        for y in (-0.22, 0.22):
            bed.visual(
                Cylinder(radius=0.045, length=0.018),
                origin=Origin(xyz=(x, y, 0.089)),
                material="blackened_steel",
                name=f"bed_bolt_{x:+.0f}_{y:+.0f}".replace("+", "p").replace("-", "m"),
            )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        Cylinder(radius=0.34, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material="machined_steel",
        name="lower_turntable",
    )
    lower_stage.visual(
        Cylinder(radius=0.305, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material="brushed_face",
        name="main_machined_face",
    )
    lower_stage.visual(
        Cylinder(radius=0.250, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material="dark_oil",
        name="lower_bearing_skirt",
    )
    lower_stage.visual(
        Cylinder(radius=0.130, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material="machined_steel",
        name="main_bearing_cap",
    )
    _add_radial_slots(lower_stage, radius=0.275, z=0.105, length=0.195, width=0.026, thickness=0.006, count=4, name="main_slot")
    _add_bolt_circle(lower_stage, radius=0.235, z=0.107, bolt_radius=0.014, height=0.012, count=8, name="main_cap_screw")

    # The offset support is part of the primary rotary stage so it swings with the large table.
    lower_stage.visual(
        Box((0.64, 0.235, 0.080)),
        origin=Origin(xyz=(0.345, 0.0, 0.145)),
        material="cast_iron",
        name="offset_saddle",
    )
    lower_stage.visual(
        Cylinder(radius=0.155, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
        material="cast_iron",
        name="main_column",
    )
    lower_stage.visual(
        Box((0.545, 0.235, 0.080)),
        origin=Origin(xyz=(0.365, 0.0, 0.300)),
        material="cast_iron",
        name="upper_bridge",
    )
    lower_stage.visual(
        Box((0.245, 0.270, 0.250)),
        origin=Origin(xyz=(OFFSET_AXIS_X, 0.0, 0.305)),
        material="cast_iron",
        name="offset_upright",
    )
    lower_stage.visual(
        Cylinder(radius=0.205, length=0.090),
        origin=Origin(xyz=(OFFSET_AXIS_X, 0.0, 0.385)),
        material="machined_steel",
        name="offset_bearing_cap",
    )
    lower_stage.visual(
        Cylinder(radius=0.105, length=0.018),
        origin=Origin(xyz=(OFFSET_AXIS_X, 0.0, 0.421)),
        material="dark_oil",
        name="upper_oil_seal",
    )
    rib = _rib_mesh(0.50, 0.145, 0.020, "offset_bridge_rib")
    for y, side_name in ((0.125, "front"), (-0.125, "rear")):
        lower_stage.visual(
            rib,
            origin=Origin(xyz=(0.360, y, 0.181), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="cast_iron",
            name=f"{side_name}_bridge_rib",
        )
    for y, side_name in ((0.143, "front"), (-0.143, "rear")):
        lower_stage.visual(
            Box((0.040, 0.018, 0.245)),
            origin=Origin(xyz=(OFFSET_AXIS_X - 0.145, y, 0.285)),
            material="cast_iron",
            name=f"{side_name}_upright_web",
        )
    _add_bolt_circle(
        lower_stage,
        radius=0.185,
        z=0.434,
        bolt_radius=0.007,
        height=0.010,
        count=6,
        name="upper_cap_screw",
        center=(OFFSET_AXIS_X, 0.0),
    )

    spin_head = model.part("spin_head")
    spin_head.visual(
        Cylinder(radius=0.158, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material="machined_steel",
        name="head_base",
    )
    spin_head.visual(
        Cylinder(radius=0.190, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material="brushed_face",
        name="head_faceplate",
    )
    spin_head.visual(
        Cylinder(radius=0.112, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material="machined_steel",
        name="head_bearing_cap",
    )
    spin_head.visual(
        Cylinder(radius=0.056, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.1975)),
        material="index_blue",
        name="indexing_nose",
    )
    _add_radial_slots(spin_head, radius=0.156, z=0.124, length=0.110, width=0.018, thickness=0.005, count=3, name="head_slot")
    _add_bolt_circle(spin_head, radius=0.126, z=0.126, bolt_radius=0.010, height=0.010, count=6, name="head_cap_screw")
    spin_head.visual(
        Box((0.115, 0.012, 0.008)),
        origin=Origin(xyz=(0.085, 0.0, 0.222)),
        material="slot_shadow",
        name="index_mark",
    )

    model.articulation(
        "main_axis",
        ArticulationType.CONTINUOUS,
        parent=bed,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, MAIN_BEARING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.8),
    )
    model.articulation(
        "offset_axis",
        ArticulationType.CONTINUOUS,
        parent=lower_stage,
        child=spin_head,
        origin=Origin(xyz=(OFFSET_AXIS_X, 0.0, UPPER_BEARING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.4),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    lower_stage = object_model.get_part("lower_stage")
    spin_head = object_model.get_part("spin_head")
    main_axis = object_model.get_articulation("main_axis")
    offset_axis = object_model.get_articulation("offset_axis")

    ctx.check(
        "parallel vertical indexing axes",
        tuple(main_axis.axis) == (0.0, 0.0, 1.0) and tuple(offset_axis.axis) == (0.0, 0.0, 1.0),
        details=f"main={main_axis.axis}, offset={offset_axis.axis}",
    )
    ctx.expect_origin_distance(
        lower_stage,
        spin_head,
        axes="xy",
        min_dist=0.60,
        max_dist=0.64,
        name="upper head axis is visibly offset from main axis",
    )
    ctx.expect_gap(
        lower_stage,
        bed,
        axis="z",
        positive_elem="lower_turntable",
        negative_elem="pedestal_bearing",
        max_gap=0.002,
        max_penetration=0.0,
        name="lower rotary table bears on fixed pedestal",
    )
    ctx.expect_gap(
        spin_head,
        lower_stage,
        axis="z",
        positive_elem="head_base",
        negative_elem="offset_bearing_cap",
        max_gap=0.002,
        max_penetration=0.00001,
        name="upper spin head sits on offset bearing cap",
    )

    for q_main, q_head in ((math.pi / 2.0, 0.0), (math.pi, 2.0 * math.pi / 3.0)):
        with ctx.pose({main_axis: q_main, offset_axis: q_head}):
            ctx.expect_gap(
                lower_stage,
                bed,
                axis="z",
                max_gap=0.002,
                max_penetration=0.00001,
                name=f"main stage clears bed at q={q_main:.2f}",
            )
            ctx.expect_gap(
                spin_head,
                lower_stage,
                axis="z",
                positive_elem="head_base",
                negative_elem="offset_bearing_cap",
                max_gap=0.002,
                max_penetration=0.00001,
                name=f"offset head clears support at q={q_head:.2f}",
            )
            ctx.expect_origin_distance(
                lower_stage,
                spin_head,
                axes="xy",
                min_dist=0.60,
                max_dist=0.64,
                name=f"offset axis remains carried by bridge at q={q_main:.2f}",
            )

    return ctx.report()


object_model = build_object_model()
