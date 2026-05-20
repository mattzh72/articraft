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


AXIS_Z = 0.24
BEARING_X = 0.30
SHAFT_RADIUS = 0.032
BEARING_INNER_RADIUS = SHAFT_RADIUS


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Orient an SDK cylinder so its local length runs along world/local X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _add_bolt_head(part, *, x: float, y: float, z: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=_x_cylinder_origin(x, y, z),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_roll_axis_spindle")

    anodized = model.material("anodized_frame", rgba=(0.18, 0.22, 0.25, 1.0))
    dark_cap = model.material("dark_caps", rgba=(0.05, 0.055, 0.06, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    shaft_steel = model.material("polished_shaft", rgba=(0.74, 0.76, 0.78, 1.0))
    flange_blue = model.material("blue_flange", rgba=(0.06, 0.22, 0.52, 1.0))
    fastener = model.material("black_fasteners", rgba=(0.015, 0.015, 0.017, 1.0))
    datum_red = model.material("red_datum", rgba=(0.82, 0.06, 0.035, 1.0))

    bearing_race_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=(BEARING_INNER_RADIUS + 0.078) * 0.5,
            tube=(0.078 - BEARING_INNER_RADIUS) * 0.5,
            radial_segments=32,
            tubular_segments=24,
        ).rotate_y(math.pi / 2.0),
        "bearing_race",
    )

    frame = model.part("frame")
    frame.visual(
        Box((0.86, 0.34, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=anodized,
        name="base_plate",
    )
    for y in (-0.135, 0.135):
        frame.visual(
            Box((0.78, 0.030, 0.034)),
            origin=Origin(xyz=(0.0, y, 0.060)),
            material=anodized,
            name=f"side_rail_{0 if y < 0 else 1}",
        )
    frame.visual(
        Box((0.10, 0.31, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=anodized,
        name="center_crossbar",
    )

    for i, x in enumerate((-BEARING_X, BEARING_X)):
        frame.visual(
            Box((0.12, 0.25, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=anodized,
            name=f"bearing_foot_{i}",
        )
        for y in (-0.082, 0.082):
            frame.visual(
                Box((0.086, 0.044, 0.102)),
                origin=Origin(xyz=(x, y, 0.135)),
                material=anodized,
                name=f"bearing_leg_{i}_{0 if y < 0 else 1}",
            )
        for y in (-0.087, 0.087):
            frame.visual(
                Box((0.088, 0.032, 0.206)),
                origin=Origin(xyz=(x, y, 0.205)),
                material=anodized,
                name=f"bearing_side_web_{i}_{0 if y < 0 else 1}",
            )
        frame.visual(
            bearing_race_mesh,
            origin=Origin(xyz=(x, 0.0, AXIS_Z)),
            material=bearing_metal,
            name=f"bearing_race_{i}",
        )
        frame.visual(
            Box((0.092, 0.190, 0.030)),
            origin=Origin(xyz=(x, 0.0, AXIS_Z + 0.076)),
            material=dark_cap,
            name=f"bearing_cap_{i}",
        )
        for bx in (-0.036, 0.036):
            for by in (-0.092, 0.092):
                frame.visual(
                    Cylinder(radius=0.007, length=0.015),
                    origin=Origin(xyz=(x + bx, by, 0.092)),
                    material=fastener,
                    name=f"mount_bolt_{i}_{0 if bx < 0 else 1}_{0 if by < 0 else 1}",
                )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.76),
        origin=_x_cylinder_origin(0.0, 0.0, 0.0),
        material=shaft_steel,
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=0.096, length=0.046),
        origin=_x_cylinder_origin(0.0, 0.0, 0.0),
        material=flange_blue,
        name="carried_flange",
    )
    for x in (-0.067, 0.067):
        spindle.visual(
            Cylinder(radius=0.054, length=0.080),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=shaft_steel,
            name=f"flange_hub_{0 if x < 0 else 1}",
        )
    for x in (-0.165, 0.165):
        spindle.visual(
            Cylinder(radius=0.043, length=0.036),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=dark_cap,
            name=f"spacer_collar_{0 if x < 0 else 1}",
        )
    for x in (-0.382, 0.382):
        spindle.visual(
            Cylinder(radius=0.047, length=0.030),
            origin=_x_cylinder_origin(x, 0.0, 0.0),
            material=dark_cap,
            name=f"end_cap_{0 if x < 0 else 1}",
        )

    bolt_radius = 0.067
    for n in range(6):
        angle = n * math.tau / 6.0
        y = math.cos(angle) * bolt_radius
        z = math.sin(angle) * bolt_radius
        _add_bolt_head(
            spindle,
            x=0.023,
            y=y,
            z=z,
            material=fastener,
            name=f"flange_bolt_{n}",
        )
    spindle.visual(
        Box((0.010, 0.026, 0.070)),
        origin=Origin(xyz=(0.035, 0.0, 0.080)),
        material=datum_red,
        name="datum_mark",
    )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    spindle = object_model.get_part("spindle")
    roll_axis = object_model.get_articulation("roll_axis")

    for race_name in ("bearing_race_0", "bearing_race_1"):
        ctx.allow_overlap(
            frame,
            spindle,
            elem_a=race_name,
            elem_b="shaft",
            reason="The shaft is intentionally captured by the bearing race proxy so the revolute support is physical.",
        )

    ctx.check(
        "single roll-axis revolute joint",
        len(object_model.articulations) == 1
        and roll_axis.articulation_type == ArticulationType.REVOLUTE
        and tuple(roll_axis.axis) == (1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}, axis={roll_axis.axis}",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_race_0",
        margin=0.003,
        name="shaft centered through first bearing race",
    )
    ctx.expect_within(
        spindle,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_race_1",
        margin=0.003,
        name="shaft centered through second bearing race",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_race_0",
        min_overlap=0.020,
        name="shaft spans first bearing support",
    )
    ctx.expect_overlap(
        spindle,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_race_1",
        min_overlap=0.020,
        name="shaft spans second bearing support",
    )
    with ctx.pose({roll_axis: math.pi / 2.0}):
        ctx.expect_within(
            spindle,
            frame,
            axes="yz",
            inner_elem="shaft",
            outer_elem="bearing_race_0",
            margin=0.003,
            name="rotated shaft remains coaxial in bearing",
        )
        datum_aabb = ctx.part_element_world_aabb(spindle, elem="datum_mark")
        ctx.check(
            "datum mark visibly rolls around shaft",
            datum_aabb is not None and abs((datum_aabb[0][1] + datum_aabb[1][1]) * 0.5) > 0.055,
            details=f"datum_aabb={datum_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
