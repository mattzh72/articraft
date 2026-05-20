from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


ROLL_LIMIT = math.radians(45.0)
PITCH_LIMIT = math.radians(60.0)
ROLL_AXIS_Z = 0.220


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]):
    """CadQuery cylinder whose axis is the module front-back (Y) axis."""
    x, y, z = center
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, y + length / 2.0, 0.0))
    )


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]):
    """CadQuery cylinder whose axis is the module left-right (X) axis."""
    x, y, z = center
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length)
        .translate((x - length / 2.0, 0.0, 0.0))
    )


def _build_base_bracket_mesh():
    """Base plate with two bored bearing towers for the roll axis."""
    plate = _box((0.360, 0.420, 0.025), (0.0, 0.0, 0.0125))
    tower_front = _box((0.135, 0.050, 0.175), (0.0, 0.170, 0.1125))
    tower_rear = _box((0.135, 0.050, 0.175), (0.0, -0.170, 0.1125))
    boss_front = _cylinder_y(0.047, 0.060, (0.0, 0.170, ROLL_AXIS_Z))
    boss_rear = _cylinder_y(0.047, 0.060, (0.0, -0.170, ROLL_AXIS_Z))
    bore = _cylinder_y(0.021, 0.500, (0.0, 0.0, ROLL_AXIS_Z))

    base = (
        plate.union(tower_front)
        .union(tower_rear)
        .union(boss_front)
        .union(boss_rear)
        .cut(bore)
        .edges("|Z")
        .fillet(0.006)
    )
    return base


def _build_roll_frame_mesh():
    """Outer roll cage with trunnion stubs and clear pitch-axis bores."""
    cage = _box((0.280, 0.180, 0.020), (0.0, 0.0, 0.105))
    cage = cage.union(_box((0.280, 0.180, 0.020), (0.0, 0.0, -0.105)))
    cage = cage.union(_box((0.020, 0.180, 0.230), (0.130, 0.0, 0.0)))
    cage = cage.union(_box((0.020, 0.180, 0.230), (-0.130, 0.0, 0.0)))

    # Front and rear trunnion hubs are tied to the rectangular cage by four
    # spokes at each end, leaving the center open for the pitch cradle.
    for y in (-0.084, 0.084):
        cage = cage.union(_cylinder_y(0.030, 0.026, (0.0, y, 0.0)))
        cage = cage.union(_box((0.260, 0.014, 0.012), (0.0, y, 0.0)))
        cage = cage.union(_box((0.012, 0.014, 0.210), (0.0, y, 0.0)))

    cage = cage.union(_cylinder_y(0.013, 0.125, (0.0, 0.130, 0.0)))
    cage = cage.union(_cylinder_y(0.013, 0.125, (0.0, -0.130, 0.0)))

    # Clearance for the inner pitch axle through the side members.
    pitch_bore = _cylinder_x(0.016, 0.360, (0.0, 0.0, 0.0))
    cage = cage.cut(pitch_bore).edges("|Y").fillet(0.003)
    return cage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pitch_roll_module")

    dark_metal = Material("mat_dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    black_rubber = Material("mat_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    roll_green = Material("mat_roll_green", rgba=(0.06, 0.40, 0.30, 1.0))
    blue_plate = Material("mat_blue_plate", rgba=(0.05, 0.22, 0.75, 1.0))
    brushed_steel = Material("mat_brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    screw_black = Material("mat_screw_black", rgba=(0.005, 0.005, 0.006, 1.0))

    base = model.part("base_bracket")
    base.visual(
        mesh_from_cadquery(_build_base_bracket_mesh(), "base_bracket"),
        material=dark_metal,
        name="base_shell",
    )
    for i, x in enumerate((-0.135, 0.135)):
        for j, y in enumerate((-0.165, 0.165)):
            base.visual(
                Cylinder(0.018, 0.008),
                origin=Origin(xyz=(x, y, -0.004)),
                material=black_rubber,
                name=f"rubber_foot_{i}_{j}",
            )

    roll_frame = model.part("roll_frame")
    roll_frame.visual(
        mesh_from_cadquery(_build_roll_frame_mesh(), "roll_frame"),
        material=roll_green,
        name="roll_cage",
    )
    # Bright bearing collars make the front-back roll axis legible.
    for y, label in ((0.162, "front"), (-0.162, "rear")):
        roll_frame.visual(
            Cylinder(0.023, 0.010),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"{label}_roll_collar",
        )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(0.0175, 0.272),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="pitch_axle",
    )
    for x, label in ((-0.055, "side_0"), (0.055, "side_1")):
        pitch_cradle.visual(
            Box((0.012, 0.072, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.034)),
            material=blue_plate,
            name=f"{label}_cheek",
        )
    pitch_cradle.visual(
        Box((0.135, 0.078, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=blue_plate,
        name="top_plate",
    )
    pitch_cradle.visual(
        Box((0.105, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=blue_plate,
        name="cradle_bridge",
    )
    for i, x in enumerate((-0.043, 0.043)):
        for j, y in enumerate((-0.024, 0.024)):
            pitch_cradle.visual(
                Cylinder(0.0045, 0.003),
                origin=Origin(xyz=(x, y, 0.0755)),
                material=screw_black,
                name=f"screw_{i}_{j}",
            )

    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=roll_frame,
        origin=Origin(xyz=(0.0, 0.0, ROLL_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-ROLL_LIMIT,
            upper=ROLL_LIMIT,
        ),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=roll_frame,
        child=pitch_cradle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-PITCH_LIMIT,
            upper=PITCH_LIMIT,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roll = object_model.get_articulation("roll_joint")
    pitch = object_model.get_articulation("pitch_joint")
    base = object_model.get_part("base_bracket")
    roll_frame = object_model.get_part("roll_frame")
    pitch_cradle = object_model.get_part("pitch_cradle")

    ctx.allow_overlap(
        base,
        roll_frame,
        elem_a="base_shell",
        elem_b="front_roll_collar",
        reason="The front roll collar is intentionally seated as a captured bearing fit in the bored base bracket.",
    )
    ctx.allow_overlap(
        base,
        roll_frame,
        elem_a="base_shell",
        elem_b="rear_roll_collar",
        reason="The rear roll collar is intentionally seated as a captured bearing fit in the bored base bracket.",
    )
    ctx.allow_overlap(
        roll_frame,
        pitch_cradle,
        elem_a="roll_cage",
        elem_b="pitch_axle",
        reason="The pitch axle is intentionally captured in the roll-frame side bores.",
    )

    ctx.check(
        "roll axis is front-back with 45 degree stops",
        roll.axis == (0.0, 1.0, 0.0)
        and roll.motion_limits is not None
        and abs(roll.motion_limits.lower + ROLL_LIMIT) < 1e-6
        and abs(roll.motion_limits.upper - ROLL_LIMIT) < 1e-6,
        details=f"axis={roll.axis}, limits={roll.motion_limits}",
    )
    ctx.check(
        "pitch axis is left-right with 60 degree stops",
        pitch.axis == (1.0, 0.0, 0.0)
        and pitch.motion_limits is not None
        and abs(pitch.motion_limits.lower + PITCH_LIMIT) < 1e-6
        and abs(pitch.motion_limits.upper - PITCH_LIMIT) < 1e-6,
        details=f"axis={pitch.axis}, limits={pitch.motion_limits}",
    )

    ctx.expect_overlap(
        roll_frame,
        base,
        axes="y",
        elem_a="roll_cage",
        elem_b="base_shell",
        min_overlap=0.030,
        name="roll trunnions remain carried by the base bearings",
    )
    ctx.expect_overlap(
        roll_frame,
        base,
        axes="y",
        elem_a="front_roll_collar",
        elem_b="base_shell",
        min_overlap=0.006,
        name="front roll collar is retained in its bearing block",
    )
    ctx.expect_overlap(
        roll_frame,
        base,
        axes="y",
        elem_a="rear_roll_collar",
        elem_b="base_shell",
        min_overlap=0.006,
        name="rear roll collar is retained in its bearing block",
    )
    ctx.expect_overlap(
        pitch_cradle,
        roll_frame,
        axes="x",
        elem_a="pitch_axle",
        elem_b="roll_cage",
        min_overlap=0.040,
        name="pitch axle spans the roll-frame side bores",
    )
    ctx.expect_within(
        pitch_cradle,
        roll_frame,
        axes="xz",
        inner_elem="top_plate",
        outer_elem="roll_cage",
        margin=0.002,
        name="top plate sits inside the roll cage at neutral pitch",
    )

    with ctx.pose({roll: ROLL_LIMIT, pitch: PITCH_LIMIT}):
        top_aabb = ctx.part_element_world_aabb(pitch_cradle, elem="top_plate")
        ctx.check(
            "fully tilted top plate remains above the base deck",
            top_aabb is not None and top_aabb[0][2] > 0.035,
            details=f"top_plate_aabb={top_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
