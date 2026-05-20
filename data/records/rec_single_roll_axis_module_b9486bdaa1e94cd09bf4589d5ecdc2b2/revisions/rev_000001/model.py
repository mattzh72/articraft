from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _x_axis_shell(name: str, outer_radius: float, inner_radius: float, length: float):
    """Build a thin hollow cylinder whose axis is local X."""
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, name)


def _x_axis_pulley(name: str, inner_radius: float, body_radius: float, flange_radius: float, length: float):
    """Build a narrow flanged pulley/ring around the roll axis."""
    edge = min(0.004, length * 0.18)
    outer_profile = [
        (flange_radius, -length / 2.0),
        (flange_radius, -length / 2.0 + edge),
        (body_radius, -length / 2.0 + edge * 1.5),
        (body_radius, length / 2.0 - edge * 1.5),
        (flange_radius, length / 2.0 - edge),
        (flange_radius, length / 2.0),
    ]
    inner_profile = [
        (inner_radius, -length / 2.0),
        (inner_radius, length / 2.0),
    ]
    pulley = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    pulley.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(pulley, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_sensor_roll_stage")

    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    steel = model.material("bearing_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.012, 0.014, 1.0))
    tube_blue = model.material("blue_anodized_tube", rgba=(0.06, 0.16, 0.34, 1.0))
    glass = model.material("smoked_glass", rgba=(0.02, 0.04, 0.06, 0.72))
    white = model.material("white_index_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    stage = model.part("stage")
    stage.visual(
        Box((0.90, 0.32, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_aluminum,
        name="base_plate",
    )

    # Minimal paired bearing supports.  Their bored rings are centered on the
    # same X roll axis as the tube.
    for x, prefix in ((0.32, "front"), (-0.32, "rear")):
        stage.visual(
            Box((0.070, 0.120, 0.110)),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=brushed_aluminum,
            name=f"{prefix}_pedestal",
        )
        stage.visual(
            _x_axis_shell(f"{prefix}_bearing_ring_mesh", 0.076, 0.056, 0.055),
            origin=Origin(xyz=(x, 0.0, 0.200)),
            material=steel,
            name=f"{prefix}_bearing_ring",
        )
        # Low, simple cap screws make the blocks read as machined hardware.
        for y in (-0.044, 0.044):
            stage.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=Origin(xyz=(x, y, 0.028), rpy=(0.0, 0.0, 0.0)),
                material=steel,
                name=f"{prefix}_bolt_{'inner' if y < 0.0 else 'outer'}",
            )

    # Side-mounted motor and compact drive pulley bracket, all fixed to the
    # stage so the only authored moving mechanism is the tube roll joint.
    stage.visual(
        Box((0.160, 0.036, 0.145)),
        origin=Origin(xyz=(0.160, -0.157, 0.0975)),
        material=brushed_aluminum,
        name="motor_bracket",
    )
    stage.visual(
        Box((0.110, 0.085, 0.090)),
        origin=Origin(xyz=(0.160, -0.115, 0.200)),
        material=black,
        name="motor_body",
    )
    stage.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.102, -0.115, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="motor_face",
    )
    stage.visual(
        _x_axis_pulley("motor_pulley_mesh", 0.006, 0.018, 0.022, 0.030),
        origin=Origin(xyz=(0.085, -0.096, 0.200)),
        material=steel,
        name="motor_pulley",
    )

    sensor_tube = model.part("sensor_tube")
    sensor_tube.visual(
        _x_axis_shell("sensor_tube_shell_mesh", 0.049, 0.041, 0.700),
        origin=Origin(),
        material=tube_blue,
        name="tube_shell",
    )
    sensor_tube.visual(
        _x_axis_pulley("drive_ring_mesh", 0.048, 0.066, 0.073, 0.032),
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
        material=steel,
        name="drive_ring",
    )
    for x, prefix in ((0.32, "front"), (-0.32, "rear")):
        sensor_tube.visual(
            _x_axis_shell(f"{prefix}_journal_mesh", 0.0565, 0.046, 0.046),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=steel,
            name=f"{prefix}_journal",
        )
    sensor_tube.visual(
        Cylinder(radius=0.043, length=0.008),
        origin=Origin(xyz=(0.352, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    sensor_tube.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(-0.356, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="rear_cap",
    )
    sensor_tube.visual(
        Box((0.360, 0.010, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, 0.051)),
        material=white,
        name="index_stripe",
    )

    model.articulation(
        "stage_to_tube",
        ArticulationType.CONTINUOUS,
        parent=stage,
        child=sensor_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stage = object_model.get_part("stage")
    sensor_tube = object_model.get_part("sensor_tube")
    roll = object_model.get_articulation("stage_to_tube")

    ctx.check(
        "single roll articulation",
        len(object_model.articulations) == 1,
        details=f"articulations={len(object_model.articulations)}",
    )
    ctx.check(
        "roll axis is tube axis",
        tuple(round(v, 6) for v in roll.axis) == (1.0, 0.0, 0.0),
        details=f"axis={roll.axis}",
    )
    ctx.check(
        "roll joint is continuous",
        roll.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={roll.articulation_type}",
    )

    for prefix in ("front", "rear"):
        ctx.allow_overlap(
            sensor_tube,
            stage,
            elem_a=f"{prefix}_journal",
            elem_b=f"{prefix}_bearing_ring",
            reason="The journal is intentionally seated in the bearing bore with a tiny local interference fit so the rotating tube is physically supported.",
        )
        ctx.expect_within(
            sensor_tube,
            stage,
            axes="yz",
            inner_elem=f"{prefix}_journal",
            outer_elem=f"{prefix}_bearing_ring",
            margin=0.0,
            name=f"{prefix} journal coaxial in bearing",
        )
        ctx.expect_overlap(
            sensor_tube,
            stage,
            axes="x",
            elem_a=f"{prefix}_journal",
            elem_b=f"{prefix}_bearing_ring",
            min_overlap=0.040,
            name=f"{prefix} journal retained by support",
        )

    return ctx.report()


object_model = build_object_model()
