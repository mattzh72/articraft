from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PITCH_AXIS_X = 0.042
PITCH_AXIS_Z = 0.065
ROLL_AXIS_X = 0.108


def _axis_cylinder(radius: float, length: float, center: tuple[float, float, float], axis: tuple[float, float, float]):
    start = (
        center[0] - axis[0] * length / 2.0,
        center[1] - axis[1] * length / 2.0,
        center[2] - axis[2] * length / 2.0,
    )
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*axis))


def _wp_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _wp_cylinder(radius: float, length: float, center: tuple[float, float, float], axis: tuple[float, float, float]):
    return cq.Workplane("XY").add(_axis_cylinder(radius, length, center, axis))


def _base_link_shape():
    base = _wp_box((0.160, 0.095, 0.012), (0.000, 0.000, 0.006))
    base = base.union(_wp_box((0.052, 0.011, 0.086), (0.038, 0.038, 0.055)))
    base = base.union(_wp_box((0.052, 0.011, 0.086), (0.038, -0.038, 0.055)))

    # Compact split bearing bosses on the two outside faces of the pitch yoke.
    base = base.union(_wp_cylinder(0.021, 0.022, (PITCH_AXIS_X, 0.049, PITCH_AXIS_Z), (0.0, 1.0, 0.0)))
    base = base.union(_wp_cylinder(0.021, 0.022, (PITCH_AXIS_X, -0.049, PITCH_AXIS_Z), (0.0, 1.0, 0.0)))

    # Low triangular-looking side ribs are represented by stout rectangular webs.
    base = base.union(_wp_box((0.074, 0.008, 0.018), (0.004, 0.026, 0.021)))
    base = base.union(_wp_box((0.074, 0.008, 0.018), (0.004, -0.026, 0.021)))

    bore = _wp_cylinder(0.0125, 0.140, (PITCH_AXIS_X, 0.0, PITCH_AXIS_Z), (0.0, 1.0, 0.0))
    base = base.cut(bore)
    return base


def _pitch_carrier_shape():
    carrier = _wp_box((0.032, 0.046, 0.028), (0.010, 0.000, 0.000))

    # Two rectangular link plates bridge the pitch trunnion to the roll bearing.
    carrier = carrier.union(_wp_box((0.096, 0.006, 0.024), (0.053, 0.024, 0.000)))
    carrier = carrier.union(_wp_box((0.096, 0.006, 0.024), (0.053, -0.024, 0.000)))

    # Static outer roll-bearing housing, coaxial with the wrist output axis.
    carrier = carrier.union(_wp_cylinder(0.031, 0.034, (ROLL_AXIS_X, 0.0, 0.0), (1.0, 0.0, 0.0)))
    roll_bore = _wp_cylinder(0.018, 0.060, (ROLL_AXIS_X, 0.0, 0.0), (1.0, 0.0, 0.0))
    carrier = carrier.cut(roll_bore)
    return carrier


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_wrist_module")

    model.material("dark_hardcoat", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("brushed_aluminum", rgba=(0.66, 0.69, 0.72, 1.0))
    model.material("bearing_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    model.material("black_oxide", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("index_orange", rgba=(0.92, 0.40, 0.12, 1.0))

    base = model.part("base_link")
    base.visual(
        mesh_from_cadquery(_base_link_shape(), "base_link_yoke"),
        material="dark_hardcoat",
        name="base_yoke",
    )
    for x_pos in (-0.052, 0.052):
        for y_pos in (-0.032, 0.032):
            base.visual(
                Cylinder(radius=0.0048, length=0.003),
                origin=Origin(xyz=(x_pos, y_pos, 0.013), rpy=(0.0, 0.0, 0.0)),
                material="black_oxide",
                name=f"mount_screw_{x_pos:+.3f}_{y_pos:+.3f}",
            )

    pitch_carrier = model.part("pitch_carrier")
    pitch_carrier.visual(
        mesh_from_cadquery(_pitch_carrier_shape(), "pitch_carrier_body"),
        material="brushed_aluminum",
        name="carrier_body",
    )
    pitch_carrier.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bearing_steel",
        name="pitch_pin",
    )
    for y_pos in (-0.043, 0.043):
        pitch_carrier.visual(
            Cylinder(radius=0.013, length=0.003),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="black_oxide",
            name=f"pitch_retainer_{y_pos:+.3f}",
        )

    roll_stage = model.part("roll_stage")
    roll_stage.visual(
        Cylinder(radius=0.014, length=0.082),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="bearing_steel",
        name="roll_shaft",
    )
    for x_pos in (-0.019, 0.019):
        roll_stage.visual(
            Cylinder(radius=0.026, length=0.004),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="bearing_steel",
            name=f"thrust_collar_{x_pos:+.3f}",
        )
    roll_stage.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="brushed_aluminum",
        name="output_flange",
    )
    roll_stage.visual(
        Box((0.014, 0.058, 0.046)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material="brushed_aluminum",
        name="output_plate",
    )
    for y_pos in (-0.019, 0.019):
        for z_pos in (-0.016, 0.016):
            roll_stage.visual(
                Cylinder(radius=0.0032, length=0.004),
                origin=Origin(xyz=(0.067, y_pos, z_pos), rpy=(0.0, pi / 2.0, 0.0)),
                material="black_oxide",
                name=f"plate_screw_{y_pos:+.3f}_{z_pos:+.3f}",
            )
    roll_stage.visual(
        Box((0.003, 0.010, 0.010)),
        origin=Origin(xyz=(0.065, 0.0, 0.021)),
        material="index_orange",
        name="roll_index_mark",
    )

    model.articulation(
        "pitch_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pitch_carrier,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.65, upper=0.90),
    )
    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pitch_carrier,
        child=roll_stage,
        origin=Origin(xyz=(ROLL_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.5, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pitch = object_model.get_articulation("pitch_hinge")
    roll = object_model.get_articulation("roll_axis")
    base = object_model.get_part("base_link")
    carrier = object_model.get_part("pitch_carrier")
    output = object_model.get_part("roll_stage")

    ctx.check(
        "pitch hinge uses lateral axis",
        tuple(round(v, 3) for v in pitch.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pitch.axis}",
    )
    ctx.check(
        "roll stage is coaxial with output shaft",
        tuple(round(v, 3) for v in roll.axis) == (1.0, 0.0, 0.0)
        and all(abs(a - b) < 1e-6 for a, b in zip(roll.origin.xyz, (ROLL_AXIS_X, 0.0, 0.0))),
        details=f"axis={roll.axis}, origin={roll.origin.xyz}",
    )
    ctx.expect_within(
        carrier,
        base,
        axes="y",
        inner_elem="pitch_pin",
        outer_elem="base_yoke",
        margin=0.001,
        name="pitch pin is retained between bearing housings",
    )
    ctx.expect_within(
        output,
        carrier,
        axes="yz",
        inner_elem="roll_shaft",
        outer_elem="carrier_body",
        margin=0.003,
        name="roll shaft stays centered in bearing bore",
    )

    rest_pos = ctx.part_world_position(output)
    with ctx.pose({pitch: 0.75}):
        raised_pos = ctx.part_world_position(output)
    ctx.check(
        "positive pitch raises the coaxial roll stage",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.050,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_plate = ctx.part_element_world_aabb(output, elem="output_plate")
    with ctx.pose({roll: pi / 2.0}):
        rolled_plate = ctx.part_element_world_aabb(output, elem="output_plate")
    if rest_plate is not None and rolled_plate is not None:
        rest_size = tuple(rest_plate[1][i] - rest_plate[0][i] for i in range(3))
        rolled_size = tuple(rolled_plate[1][i] - rolled_plate[0][i] for i in range(3))
        ok = rest_size[1] > rest_size[2] * 1.15 and rolled_size[2] > rolled_size[1] * 1.15
    else:
        rest_size = rolled_size = None
        ok = False
    ctx.check(
        "roll joint rotates the rectangular output plate",
        ok,
        details=f"rest_size={rest_size}, rolled_size={rolled_size}",
    )

    return ctx.report()


object_model = build_object_model()
