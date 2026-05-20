from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GEAR_WIDTH = 0.050
SHAFT_RADIUS = 0.016
SHAFT_LENGTH = 0.360
SHAFT_BACK_Y = -0.020
GEAR_Y = 0.045
FRAME_Y = -0.130
BEARING_THICKNESS = 0.060
BEARING_BLOCK_HOLE_RADIUS = 0.027
BEARING_RACE_HOLE_RADIUS = 0.0215


def _toothed_gear(
    *,
    teeth: int,
    root_radius: float,
    outer_radius: float,
    width: float,
    bore_radius: float,
    hub_radius: float,
    phase: float = 0.0,
    spoke_count: int = 6,
) -> cq.Workplane:
    """Build a simple extruded spur gear with visible straight teeth."""

    points: list[tuple[float, float]] = []
    pitch = 2.0 * math.pi / teeth
    for i in range(teeth):
        center = phase + i * pitch
        for angle, radius in (
            (center - 0.46 * pitch, root_radius),
            (center - 0.24 * pitch, outer_radius),
            (center + 0.24 * pitch, outer_radius),
            (center + 0.46 * pitch, root_radius),
        ):
            points.append((radius * math.cos(angle), radius * math.sin(angle)))

    body = cq.Workplane("XY").polyline(points).close().extrude(width, both=True)
    hub = cq.Workplane("XY").circle(hub_radius).extrude(width * 1.35, both=True)
    gear = body.union(hub)

    bore = cq.Workplane("XY").circle(bore_radius).extrude(width * 1.8, both=True)
    gear = gear.cut(bore)

    if spoke_count > 0:
        hole_radius = max(0.008, min((root_radius - hub_radius) * 0.22, 0.026))
        hole_circle = (root_radius + hub_radius) * 0.53
        for i in range(spoke_count):
            angle = phase + (i + 0.5) * 2.0 * math.pi / spoke_count
            x = hole_circle * math.cos(angle)
            y = hole_circle * math.sin(angle)
            hole = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(hole_radius)
                .extrude(width * 1.9, both=True)
            )
            gear = gear.cut(hole)

    return gear


def _bearing_block(*, width: float = 0.112, height: float = 0.112) -> cq.Workplane:
    """Flanged bearing plate with a true shaft clearance hole."""

    block = (
        cq.Workplane("XY")
        .rect(width, height)
        .circle(BEARING_BLOCK_HOLE_RADIUS)
        .extrude(BEARING_THICKNESS, both=True)
    )
    for x in (-width * 0.34, width * 0.34):
        for y in (-height * 0.34, height * 0.34):
            hole = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(0.006)
                .extrude(BEARING_THICKNESS * 1.4, both=True)
            )
            block = block.cut(hole)
    return block


def _bearing_race() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.039)
        .circle(BEARING_RACE_HOLE_RADIUS)
        .extrude(0.018, both=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_spur_gear_train_bench")

    frame_mat = model.material("painted_cast_frame", rgba=(0.12, 0.15, 0.17, 1.0))
    edge_mat = model.material("machined_edge", rgba=(0.42, 0.45, 0.46, 1.0))
    gear_mat = model.material("oiled_steel_gears", rgba=(0.68, 0.62, 0.48, 1.0))
    shaft_mat = model.material("polished_shafts", rgba=(0.78, 0.80, 0.80, 1.0))
    bearing_mat = model.material("bearing_races", rgba=(0.18, 0.19, 0.20, 1.0))
    fastener_mat = model.material("dark_fasteners", rgba=(0.03, 0.03, 0.035, 1.0))

    centers = {
        "input": (-0.270, 0.282),
        "idler": (-0.003, 0.282),
        "output": (0.190, 0.282),
    }

    frame = model.part("side_frame")
    frame.visual(
        Box((0.86, 0.36, 0.038)),
        origin=Origin(xyz=(-0.035, -0.030, 0.019)),
        material=frame_mat,
        name="bench_base",
    )
    frame.visual(
        Box((0.82, 0.052, 0.052)),
        origin=Origin(xyz=(-0.035, FRAME_Y, 0.096)),
        material=frame_mat,
        name="lower_rail",
    )
    for name, x in (("foot_post_0", -0.380), ("foot_post_1", 0.310)):
        frame.visual(
            Box((0.052, 0.052, 0.070)),
            origin=Origin(xyz=(x, FRAME_Y, 0.054)),
            material=frame_mat,
            name=name,
        )
    frame.visual(
        Box((0.82, 0.046, 0.046)),
        origin=Origin(xyz=(-0.035, FRAME_Y, 0.478)),
        material=frame_mat,
        name="upper_rail",
    )
    for name, x in (("post_0", -0.445), ("post_1", 0.375)):
        frame.visual(
            Box((0.055, 0.052, 0.405)),
            origin=Origin(xyz=(x, FRAME_Y, 0.287)),
            material=frame_mat,
            name=name,
        )
    for name, (x, z) in centers.items():
        frame.visual(
            Box((0.040, 0.048, 0.146)),
            origin=Origin(xyz=(x, FRAME_Y, 0.162)),
            material=frame_mat,
            name=f"{name}_pedestal",
        )
        frame.visual(
            mesh_from_cadquery(_bearing_block(), f"{name}_bearing_block"),
            origin=Origin(xyz=(x, FRAME_Y, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=edge_mat,
            name=f"{name}_bearing_block",
        )
        frame.visual(
            mesh_from_cadquery(_bearing_race(), f"{name}_bearing_race"),
            origin=Origin(xyz=(x, FRAME_Y + 0.028, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=bearing_mat,
            name=f"{name}_bearing_race",
        )
        for ix, sx in enumerate((-0.038, 0.038)):
            for iz, sz in enumerate((-0.038, 0.038)):
                frame.visual(
                    Cylinder(radius=0.0068, length=0.010),
                    origin=Origin(
                        xyz=(x + sx, FRAME_Y + 0.030, z + sz),
                        rpy=(-math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=fastener_mat,
                    name=f"{name}_bolt_{ix}_{iz}",
                )
    frame.visual(
        Box((0.390, 0.034, 0.030)),
        origin=Origin(xyz=(-0.260, FRAME_Y - 0.004, 0.378), rpy=(0.0, -0.62, 0.0)),
        material=frame_mat,
        name="brace_0",
    )
    frame.visual(
        Box((0.330, 0.034, 0.030)),
        origin=Origin(xyz=(0.205, FRAME_Y - 0.004, 0.386), rpy=(0.0, 0.64, 0.0)),
        material=frame_mat,
        name="brace_1",
    )

    gear_specs = {
        "input_shaft": {
            "center": centers["input"],
            "teeth": 40,
            "root": 0.132,
            "outer": 0.154,
            "hub": 0.044,
            "phase": 0.0,
            "spokes": 6,
        },
        "idler_shaft": {
            "center": centers["idler"],
            "teeth": 28,
            "root": 0.093,
            "outer": 0.109,
            "hub": 0.038,
            "phase": math.pi / 28.0,
            "spokes": 5,
        },
        "output_shaft": {
            "center": centers["output"],
            "teeth": 20,
            "root": 0.064,
            "outer": 0.080,
            "hub": 0.034,
            "phase": math.pi,
            "spokes": 0,
        },
    }

    for part_name, spec in gear_specs.items():
        x, z = spec["center"]
        shaft = model.part(part_name)
        shaft.visual(
            Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
            origin=Origin(xyz=(0.0, SHAFT_BACK_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shaft_mat,
            name="shaft",
        )
        shaft.visual(
            mesh_from_cadquery(
                _toothed_gear(
                    teeth=spec["teeth"],
                    root_radius=spec["root"],
                    outer_radius=spec["outer"],
                    width=GEAR_WIDTH,
                    bore_radius=SHAFT_RADIUS * 0.92,
                    hub_radius=spec["hub"],
                    phase=spec["phase"],
                    spoke_count=spec["spokes"],
                ),
                f"{part_name}_gear",
                tolerance=0.0008,
                angular_tolerance=0.08,
            ),
            origin=Origin(xyz=(0.0, GEAR_Y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=gear_mat,
            name="spur_gear",
        )
        shaft.visual(
            Cylinder(radius=0.023, length=0.020),
            origin=Origin(
                xyz=(0.0, GEAR_Y + GEAR_WIDTH * 0.66, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=shaft_mat,
            name="front_collar",
        )
        shaft.visual(
            Cylinder(radius=0.022, length=0.030),
            origin=Origin(xyz=(0.0, FRAME_Y + 0.028, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shaft_mat,
            name="bearing_journal",
        )

        mimic = None
        if part_name == "idler_shaft":
            mimic = Mimic(joint="side_frame_to_input_shaft", multiplier=-40.0 / 28.0)
        elif part_name == "output_shaft":
            mimic = Mimic(joint="side_frame_to_input_shaft", multiplier=40.0 / 20.0)

        model.articulation(
            f"side_frame_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=shaft,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
            mimic=mimic,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("side_frame")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")
    joints = [
        object_model.get_articulation("side_frame_to_input_shaft"),
        object_model.get_articulation("side_frame_to_idler_shaft"),
        object_model.get_articulation("side_frame_to_output_shaft"),
    ]

    ctx.check(
        "three supported rotating shafts",
        len(joints) == 3
        and all(j.parent == "side_frame" for j in joints)
        and {j.child for j in joints} == {"input_shaft", "idler_shaft", "output_shaft"},
        details=f"joints={[(j.name, j.parent, j.child) for j in joints]}",
    )
    ctx.check(
        "shaft axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    for shaft in (input_shaft, idler_shaft, output_shaft):
        prefix = shaft.name.split("_")[0]
        ctx.allow_overlap(
            frame,
            shaft,
            elem_a=f"{prefix}_bearing_race",
            elem_b="bearing_journal",
            reason="The rotating journal is intentionally captured with a tiny seated fit inside the visible bearing race.",
        )
        ctx.expect_within(
            shaft,
            frame,
            axes="xz",
            inner_elem="bearing_journal",
            margin=0.006,
            name=f"{shaft.name} journal sits inside its bearing block footprint",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="y",
            elem_a="bearing_journal",
            elem_b=f"{prefix}_bearing_race",
            min_overlap=0.010,
            name=f"{shaft.name} journal passes through visible bearing race",
        )

    ctx.expect_gap(
        idler_shaft,
        input_shaft,
        axis="x",
        positive_elem="spur_gear",
        negative_elem="spur_gear",
        min_gap=0.001,
        max_gap=0.012,
        name="input and idler teeth are close but not intersecting",
    )
    ctx.expect_gap(
        output_shaft,
        idler_shaft,
        axis="x",
        positive_elem="spur_gear",
        negative_elem="spur_gear",
        min_gap=0.001,
        max_gap=0.012,
        name="idler and output teeth are close but not intersecting",
    )

    with ctx.pose({joints[0]: math.pi / 5.0}):
        ctx.expect_overlap(
            input_shaft,
            idler_shaft,
            axes="y",
            elem_a="spur_gear",
            elem_b="spur_gear",
            min_overlap=GEAR_WIDTH * 0.80,
            name="gears remain in the same running plane when rotated",
        )

    return ctx.report()


object_model = build_object_model()
