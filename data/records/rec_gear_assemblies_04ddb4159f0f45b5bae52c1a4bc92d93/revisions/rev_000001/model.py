from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


SHAFT_Z = 0.170
SHAFT_RADIUS = 0.008
SHAFT_LENGTH = 0.380
GEAR_ROOT_RADIUS = 0.060
GEAR_OUTER_RADIUS = 0.072
GEAR_WIDTH = 0.036
GEAR_TEETH = 24
CENTER_DISTANCE = 2.0 * GEAR_OUTER_RADIUS + 0.002
SHAFT_X = {
    "input": -CENTER_DISTANCE,
    "idler": 0.0,
    "output": CENTER_DISTANCE,
}


def _toothed_gear_mesh(name: str, *, phase: float = 0.0):
    """Simple exposed spur-gear visual: straight teeth around a solid web."""
    pitch = 2.0 * math.pi / GEAR_TEETH
    profile = []
    for tooth in range(GEAR_TEETH):
        center = phase + tooth * pitch
        for angle, radius in (
            (center - 0.50 * pitch, GEAR_ROOT_RADIUS),
            (center - 0.22 * pitch, GEAR_OUTER_RADIUS),
            (center + 0.22 * pitch, GEAR_OUTER_RADIUS),
        ):
            profile.append((radius * math.cos(angle), radius * math.sin(angle)))
    return mesh_from_geometry(
        ExtrudeGeometry(profile, GEAR_WIDTH, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="exposed_three_shaft_spur_train")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_iron = model.material("dark_bearing", rgba=(0.035, 0.035, 0.032, 1.0))
    brass = model.material("oiled_bronze_gears", rgba=(0.86, 0.61, 0.24, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.74, 1.0))
    collar_mat = model.material("blue_set_collars", rgba=(0.05, 0.16, 0.36, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((0.520, 0.360, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=cast_iron,
        name="floor_plate",
    )
    for y, suffix in ((-0.160, "0"), (0.160, "1")):
        base.visual(
            Box((0.540, 0.026, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.044)),
            material=cast_iron,
            name=f"side_rail_{suffix}",
        )
    for label, x in SHAFT_X.items():
        base.visual(
            Box((0.052, 0.310, 0.020)),
            origin=Origin(xyz=(x, 0.0, 0.036)),
            material=cast_iron,
            name=f"{label}_cross_rib",
        )

    outer_bearing = mesh_from_geometry(
        TorusGeometry(0.022, 0.006, radial_segments=40, tubular_segments=12),
        "bearing_outer_ring",
    )
    inner_bearing = mesh_from_geometry(
        TorusGeometry(0.014, 0.004, radial_segments=36, tubular_segments=10),
        "bearing_inner_bush",
    )
    for label, x in SHAFT_X.items():
        for idx, y in enumerate((-0.136, 0.136)):
            base.visual(
                Box((0.050, 0.030, 0.122)),
                origin=Origin(xyz=(x, y, 0.084)),
                material=cast_iron,
                name=f"{label}_pedestal_{idx}",
            )
            base.visual(
                Box((0.022, 0.030, 0.017)),
                origin=Origin(xyz=(x, y, SHAFT_Z - SHAFT_RADIUS - 0.0085)),
                material=dark_iron,
                name=f"{label}_saddle_{idx}",
            )
            base.visual(
                outer_bearing,
                origin=Origin(xyz=(x, y, SHAFT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=steel,
                name=f"{label}_bearing_{idx}",
            )
            base.visual(
                inner_bearing,
                origin=Origin(xyz=(x, y, SHAFT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_iron,
                name=f"{label}_bush_{idx}",
            )

    gear_meshes = {
        "input": _toothed_gear_mesh("input_spur_gear", phase=0.0),
        "idler": _toothed_gear_mesh("idler_spur_gear", phase=math.pi / GEAR_TEETH),
        "output": _toothed_gear_mesh("output_spur_gear", phase=0.0),
    }

    for label, x in SHAFT_X.items():
        shaft = model.part(f"{label}_shaft")
        shaft.visual(
            Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="shaft_core",
        )
        shaft.visual(
            gear_meshes[label],
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="spur_gear",
        )
        shaft.visual(
            Cylinder(radius=0.027, length=0.060),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="gear_hub",
        )
        for idx, y in enumerate((-0.095, 0.095)):
            shaft.visual(
                Cylinder(radius=0.014, length=0.014),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=collar_mat,
                name=f"set_collar_{idx}",
            )
        model.articulation(
            f"{label}_bearing_axis",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=shaft,
            origin=Origin(xyz=(x, 0.0, SHAFT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=20.0),
            mimic=(
                Mimic("input_bearing_axis", multiplier=-1.0, offset=0.0)
                if label == "idler"
                else Mimic("input_bearing_axis", multiplier=1.0, offset=0.0)
                if label == "output"
                else None
            ),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")
    input_axis = object_model.get_articulation("input_bearing_axis")
    idler_axis = object_model.get_articulation("idler_bearing_axis")
    output_axis = object_model.get_articulation("output_bearing_axis")

    shaft_parts = {
        "input": input_shaft,
        "idler": idler_shaft,
        "output": output_shaft,
    }
    shaft_joints = (input_axis, idler_axis, output_axis)

    ctx.check(
        "three supported shaft links",
        all(part is not None for part in shaft_parts.values()),
        details=f"shaft_parts={shaft_parts}",
    )
    ctx.check(
        "each shaft joint spins about the supported y axis",
        all(
            joint is not None
            and joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 1.0, 0.0)
            for joint in shaft_joints
        ),
        details=f"joints={shaft_joints}",
    )
    ctx.check(
        "idler and output are geared from the input",
        idler_axis.mimic is not None
        and idler_axis.mimic.joint == "input_bearing_axis"
        and abs(idler_axis.mimic.multiplier + 1.0) < 1e-9
        and output_axis.mimic is not None
        and output_axis.mimic.joint == "input_bearing_axis"
        and abs(output_axis.mimic.multiplier - 1.0) < 1e-9,
        details=f"idler_mimic={idler_axis.mimic}, output_mimic={output_axis.mimic}",
    )

    for label, shaft in shaft_parts.items():
        for idx in (0, 1):
            ctx.expect_contact(
                shaft,
                base,
                elem_a="shaft_core",
                elem_b=f"{label}_saddle_{idx}",
                contact_tol=0.001,
                name=f"{label} shaft rests in bearing saddle {idx}",
            )
            ctx.expect_overlap(
                shaft,
                base,
                axes="y",
                elem_a="shaft_core",
                elem_b=f"{label}_bearing_{idx}",
                min_overlap=0.010,
                name=f"{label} shaft passes through bearing ring {idx}",
            )
            ctx.expect_within(
                shaft,
                base,
                axes="xz",
                inner_elem="shaft_core",
                outer_elem=f"{label}_bearing_{idx}",
                margin=0.001,
                name=f"{label} shaft is centered in bearing ring {idx}",
            )

    ctx.expect_origin_gap(
        idler_shaft,
        input_shaft,
        axis="x",
        min_gap=CENTER_DISTANCE - 1e-6,
        max_gap=CENTER_DISTANCE + 1e-6,
        name="input to idler center distance",
    )
    ctx.expect_origin_gap(
        output_shaft,
        idler_shaft,
        axis="x",
        min_gap=CENTER_DISTANCE - 1e-6,
        max_gap=CENTER_DISTANCE + 1e-6,
        name="idler to output center distance",
    )
    ctx.expect_gap(
        idler_shaft,
        input_shaft,
        axis="x",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem="spur_gear",
        negative_elem="spur_gear",
        name="input and idler tooth tip clearance",
    )
    ctx.expect_gap(
        output_shaft,
        idler_shaft,
        axis="x",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem="spur_gear",
        negative_elem="spur_gear",
        name="idler and output tooth tip clearance",
    )
    ctx.expect_overlap(
        input_shaft,
        idler_shaft,
        axes="z",
        min_overlap=0.10,
        elem_a="spur_gear",
        elem_b="spur_gear",
        name="input and idler gears share face height",
    )
    ctx.expect_overlap(
        idler_shaft,
        output_shaft,
        axes="z",
        min_overlap=0.10,
        elem_a="spur_gear",
        elem_b="spur_gear",
        name="idler and output gears share face height",
    )

    rest_positions = {
        label: ctx.part_world_position(shaft)
        for label, shaft in shaft_parts.items()
    }
    with ctx.pose({input_axis: 0.45}):
        posed_positions = {
            label: ctx.part_world_position(shaft)
            for label, shaft in shaft_parts.items()
        }
    ctx.check(
        "shaft centers stay fixed while the train rotates",
        all(
            rest_positions[label] is not None
            and posed_positions[label] is not None
            and all(
                abs(rest_positions[label][i] - posed_positions[label][i]) < 1e-8
                for i in range(3)
            )
            for label in shaft_parts
        ),
        details=f"rest={rest_positions}, posed={posed_positions}",
    )

    return ctx.report()


object_model = build_object_model()
