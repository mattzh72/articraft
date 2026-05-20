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
    mesh_from_geometry,
)
import cadquery as cq


AXLE_Z = 1.28
GATE_X = -0.55
GATE_Z = 2.42


def _circle_profile(radius: float, segments: int = 72, *, clockwise: bool = False):
    points = []
    order = range(segments - 1, -1, -1) if clockwise else range(segments)
    for i in order:
        a = 2.0 * math.pi * i / segments
        points.append((radius * math.cos(a), radius * math.sin(a)))
    return points


def _rect_profile(width: float, height: float):
    hw = width * 0.5
    hh = height * 0.5
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def _annular_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, 96),
        [_circle_profile(inner_radius, 96, clockwise=True)],
        thickness,
    )
    # The extrusion is born in the XY plane with thickness along local Z; rotate
    # it so the waterwheel ring lies in XZ and its thickness spans the axle.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _bearing_plate_mesh(name: str):
    geom = ExtrudeWithHolesGeometry(
        _rect_profile(0.46, 0.46),
        [_circle_profile(0.13, 48, clockwise=True)],
        0.045,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_waterwheel_millrace")

    stone = model.material("mottled_stone", color=(0.48, 0.46, 0.42, 1.0))
    dark_stone = model.material("dark_wet_stone", color=(0.30, 0.31, 0.30, 1.0))
    wood = model.material("weathered_oak", color=(0.48, 0.28, 0.12, 1.0))
    dark_wood = model.material("wet_dark_oak", color=(0.30, 0.16, 0.07, 1.0))
    iron = model.material("blackened_iron", color=(0.05, 0.05, 0.045, 1.0))
    water = model.material("clear_water", color=(0.20, 0.46, 0.82, 0.55))

    millrace = model.part("millrace")
    millrace.visual(
        Box((3.45, 2.18, 0.18)),
        origin=Origin(xyz=(-0.30, 0.0, 0.09)),
        material=stone,
        name="stone_foundation",
    )
    millrace.visual(
        Box((2.70, 0.74, 0.08)),
        origin=Origin(xyz=(0.18, 0.0, 0.22)),
        material=dark_stone,
        name="tailrace_floor",
    )
    for side, y in enumerate((-0.43, 0.43)):
        millrace.visual(
            Box((2.70, 0.10, 0.30)),
            origin=Origin(xyz=(0.18, -0.42 if y < 0.0 else 0.42, 0.37)),
            material=stone,
            name=f"tailrace_wall_{side}",
        )

    for side, y in enumerate((-0.78, 0.78)):
        millrace.visual(
            Box((0.42, 0.24, 1.40)),
            origin=Origin(xyz=(0.0, y, 0.88)),
            material=stone,
            name=f"side_support_{side}",
        )
        millrace.visual(
            _bearing_plate_mesh(f"bearing_plate_{side}"),
            origin=Origin(xyz=(0.0, -0.6825 if y < 0.0 else 0.6825, AXLE_Z)),
            material=iron,
            name=f"bearing_plate_{side}",
        )

    # Stone piers that carry the elevated feed chute back to the foundation.
    for side, y in enumerate((-0.40, 0.40)):
        millrace.visual(
            Box((0.22, 0.18, 2.30)),
            origin=Origin(xyz=(-1.34, y, 1.33)),
            material=stone,
            name=f"chute_pier_{side}",
        )

    # Elevated feed chute and split water strips, leaving a dry slot where the
    # sliding sluice gate passes through.
    millrace.visual(
        Box((1.55, 0.56, 0.08)),
        origin=Origin(xyz=(-0.80, 0.0, 2.38)),
        material=dark_wood,
        name="feed_chute_floor",
    )
    for side, y in enumerate((-0.32, 0.32)):
        millrace.visual(
            Box((1.55, 0.08, 0.26)),
            origin=Origin(xyz=(-0.80, y, 2.54)),
            material=wood,
            name=f"feed_chute_wall_{side}",
        )
    millrace.visual(
        Box((0.80, 0.40, 0.025)),
        origin=Origin(xyz=(-1.05, 0.0, 2.4325)),
        material=water,
        name="upstream_water",
    )
    millrace.visual(
        Box((0.36, 0.40, 0.025)),
        origin=Origin(xyz=(-0.27, 0.0, 2.4325)),
        material=water,
        name="falling_water_lip",
    )

    # Vertical guides for the sliding sluice gate.
    for side, y in enumerate((-0.305, 0.305)):
        millrace.visual(
            Box((0.11, 0.055, 0.78)),
            origin=Origin(xyz=(GATE_X, y, 2.76)),
            material=iron,
            name=f"gate_guide_{side}",
        )
    millrace.visual(
        Box((0.12, 0.72, 0.08)),
        origin=Origin(xyz=(GATE_X, 0.0, 3.16)),
        material=iron,
        name="gate_crossbar",
    )

    wheel = model.part("wheel")
    rim_mesh = _annular_mesh(0.98, 0.86, 0.065, "wooden_rim")
    inner_rim_mesh = _annular_mesh(0.62, 0.53, 0.055, "inner_rim")
    for side, y in enumerate((-0.30, 0.30)):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=wood,
            name=f"outer_rim_{side}",
        )
        wheel.visual(
            inner_rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=dark_wood,
            name=f"inner_rim_{side}",
        )

    wheel.visual(
        Cylinder(radius=0.080, length=1.20),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.17, length=0.74),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub",
    )

    spoke_length = 0.72
    spoke_mid = 0.50
    for side_y in (-0.30, 0.30):
        for i in range(12):
            phi = 2.0 * math.pi * i / 12.0
            wheel.visual(
                Box((spoke_length, 0.075, 0.060)),
                origin=Origin(
                    xyz=(spoke_mid * math.cos(phi), side_y, spoke_mid * math.sin(phi)),
                    rpy=(0.0, -phi, 0.0),
                ),
                material=wood,
                name=f"spoke_{0 if side_y < 0 else 1}_{i}",
            )

    for i in range(18):
        phi = 2.0 * math.pi * i / 18.0
        wheel.visual(
            Box((0.13, 0.68, 0.18)),
            origin=Origin(
                xyz=(0.935 * math.cos(phi), 0.0, 0.935 * math.sin(phi)),
                rpy=(0.0, -phi, 0.0),
            ),
            material=dark_wood,
            name=f"bucket_{i}",
        )
        wheel.visual(
            Box((0.045, 0.66, 0.15)),
            origin=Origin(
                xyz=(1.005 * math.cos(phi), 0.0, 1.005 * math.sin(phi)),
                rpy=(0.0, -phi, 0.0),
            ),
            material=wood,
            name=f"bucket_lip_{i}",
        )

    sluice_gate = model.part("sluice_gate")
    sluice_gate.visual(
        Box((0.08, 0.46, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=dark_wood,
        name="gate_panel",
    )
    sluice_gate.visual(
        Box((0.045, 0.050, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=iron,
        name="lift_stem",
    )
    sluice_gate.visual(
        Box((0.055, 0.30, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=iron,
        name="lift_handle",
    )

    model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=millrace,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.0),
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=millrace,
        child=sluice_gate,
        origin=Origin(xyz=(GATE_X, 0.0, GATE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    millrace = object_model.get_part("millrace")
    wheel = object_model.get_part("wheel")
    gate = object_model.get_part("sluice_gate")
    spin = object_model.get_articulation("wheel_spin")
    slide = object_model.get_articulation("gate_slide")

    ctx.check(
        "wheel has a continuous axle joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "sluice gate has a vertical prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )

    ctx.expect_gap(
        millrace,
        wheel,
        axis="y",
        min_gap=0.005,
        max_gap=0.10,
        positive_elem="bearing_plate_1",
        negative_elem="axle",
        name="axle sits close to positive bearing",
    )
    ctx.expect_gap(
        wheel,
        millrace,
        axis="y",
        min_gap=0.005,
        max_gap=0.10,
        positive_elem="axle",
        negative_elem="bearing_plate_0",
        name="axle sits close to negative bearing",
    )
    ctx.expect_within(
        gate,
        millrace,
        axes="y",
        margin=0.0,
        inner_elem="gate_panel",
        outer_elem="gate_crossbar",
        name="gate panel fits within guide span",
    )
    ctx.expect_gap(
        gate,
        millrace,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem="gate_panel",
        negative_elem="feed_chute_floor",
        name="lowered gate seats on chute floor",
    )

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({slide: 0.25, spin: math.pi / 3.0}):
        raised_pos = ctx.part_world_position(gate)
        ctx.expect_within(
            gate,
            millrace,
            axes="y",
            margin=0.0,
            inner_elem="gate_panel",
            outer_elem="gate_crossbar",
            name="raised gate stays between guides",
        )
        ctx.expect_gap(
            millrace,
            gate,
            axis="z",
            min_gap=0.02,
            positive_elem="gate_crossbar",
            negative_elem="gate_panel",
            name="raised gate remains below top crossbar",
        )
    ctx.check(
        "gate raises upward along guides",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
