from __future__ import annotations

from math import pi, sqrt

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_WIDTH = 0.66
PLATE_HEIGHT = 0.42
PLATE_THICKNESS = 0.036
BOSS_LENGTH = 0.046
BOSS_RADIUS = 0.038
BORE_RADIUS = 0.015
BEARING_OUTER_RADIUS = 0.023
SHAFT_RADIUS = 0.016
GEAR_MODULE = 0.004
GEAR_WIDTH = 0.024
TOOTH_CLEARANCE = 0.003

SHAFT_CENTERS = {
    "input": (-0.285, -0.055),
    "compound_0": (-0.155, 0.029),
    "compound_1": (-0.043, -0.039),
    "compound_2": (0.058, 0.022),
    "output": (0.161, -0.008),
}

GEAR_PLANES = {
    "stage_0": 0.125,
    "stage_1": 0.205,
    "stage_2": 0.285,
    "stage_3": 0.365,
}

GEAR_STACKS = {
    "input_shaft": [("input_gear", 24, "stage_0", "gear_gold")],
    "compound_shaft_0": [
        ("compound_0_large", 48, "stage_0", "gear_gold"),
        ("compound_0_pinion", 18, "stage_1", "pinion_steel"),
    ],
    "compound_shaft_1": [
        ("compound_1_large", 42, "stage_1", "gear_gold"),
        ("compound_1_pinion", 18, "stage_2", "pinion_steel"),
    ],
    "compound_shaft_2": [
        ("compound_2_large", 36, "stage_2", "gear_gold"),
        ("compound_2_pinion", 16, "stage_3", "pinion_steel"),
    ],
    "output_shaft": [("output_gear", 32, "stage_3", "gear_gold")],
}


def _gear_outer_radius(teeth: int) -> float:
    return GEAR_MODULE * (teeth + 2) * 0.5


def _center_cq(shape):
    bb = shape.BoundingBox()
    cx = (bb.xmin + bb.xmax) * 0.5
    cy = (bb.ymin + bb.ymax) * 0.5
    cz = (bb.zmin + bb.zmax) * 0.5
    return shape.translate((-cx, -cy, -cz))


def _make_side_frame_shape():
    shaft_points = list(SHAFT_CENTERS.values())
    plate = cq.Workplane("XZ").rect(PLATE_WIDTH, PLATE_HEIGHT).extrude(PLATE_THICKNESS, both=True)

    bores = (
        cq.Workplane("XZ")
        .pushPoints(shaft_points)
        .circle(BORE_RADIUS)
        .extrude(PLATE_THICKNESS * 4.0, both=True)
    )
    plate = plate.cut(bores)

    bosses = (
        cq.Workplane("XZ")
        .workplane(offset=PLATE_THICKNESS * 0.5 - 0.002)
        .pushPoints(shaft_points)
        .circle(BOSS_RADIUS)
        .circle(BORE_RADIUS)
        .extrude(BOSS_LENGTH + 0.002)
    )
    plate = plate.union(bosses)

    bolt_points = [
        (-PLATE_WIDTH * 0.41, -PLATE_HEIGHT * 0.38),
        (-PLATE_WIDTH * 0.41, PLATE_HEIGHT * 0.38),
        (PLATE_WIDTH * 0.41, -PLATE_HEIGHT * 0.38),
        (PLATE_WIDTH * 0.41, PLATE_HEIGHT * 0.38),
    ]
    bolt_holes = (
        cq.Workplane("XZ")
        .pushPoints(bolt_points)
        .circle(0.011)
        .extrude(PLATE_THICKNESS * 4.0, both=True)
    )
    plate = plate.cut(bolt_holes)

    window_points = [(-0.220, 0.125), (-0.005, 0.122), (0.095, -0.117)]
    windows = (
        cq.Workplane("XZ")
        .pushPoints(window_points)
        .rect(0.090, 0.034)
        .extrude(PLATE_THICKNESS * 4.0, both=True)
    )
    plate = plate.cut(windows)

    return plate


def _make_bearing_liner_shape():
    return cq.Workplane("XZ").circle(BEARING_OUTER_RADIUS).circle(BORE_RADIUS).extrude(0.014, both=True)


def _make_gear_shape(teeth: int):
    # The vendored gear generator is numerically happier in millimetre-sized
    # CadQuery units; the mesh export scales the result back to metres.
    module_mm = GEAR_MODULE * 1000.0
    width_mm = GEAR_WIDTH * 1000.0
    pitch_diameter = module_mm * teeth
    hub_diameter = max(36.0, pitch_diameter * 0.42)
    hub_length = width_mm + 10.0
    shape = SpurGear(
        module_mm,
        teeth,
        width_mm,
        clearance=0.4,
        backlash=0.2,
    ).build(
        hub_d=hub_diameter,
        hub_length=hub_length,
        chamfer=0.7,
    )
    return _center_cq(shape)


def _expected_pair_spacing(teeth_a: int, teeth_b: int) -> float:
    return _gear_outer_radius(teeth_a) + _gear_outer_radius(teeth_b) + TOOTH_CLEARANCE


def _actual_spacing(a: tuple[float, float], b: tuple[float, float]) -> float:
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def _add_shaft_visuals(part, gears, gear_meshes) -> None:
    max_plane = max(GEAR_PLANES[stage] for _, _, stage, _ in gears)
    shaft_start = -0.014
    shaft_end = max_plane + GEAR_WIDTH * 0.5 + 0.035
    shaft_length = shaft_end - shaft_start
    shaft_center_y = (shaft_start + shaft_end) * 0.5
    axis_origin = Origin(xyz=(0.0, shaft_center_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0))

    part.visual(
        Cylinder(radius=SHAFT_RADIUS, length=shaft_length),
        origin=axis_origin,
        material="shaft_steel",
        name="shaft_core",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, BOSS_LENGTH + PLATE_THICKNESS * 0.5 + 0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="shaft_steel",
        name="inner_collar",
    )
    part.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, shaft_end + 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="shaft_steel",
        name="end_cap",
    )

    for visual_name, teeth, stage, material in gears:
        plane_y = GEAR_PLANES[stage]
        part.visual(
            gear_meshes[teeth],
            origin=Origin(xyz=(0.0, plane_y, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=material,
            name=visual_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_spur_train")

    model.material("frame_paint", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("bearing_bronze", rgba=(0.78, 0.56, 0.26, 1.0))
    model.material("gear_gold", rgba=(0.86, 0.67, 0.28, 1.0))
    model.material("pinion_steel", rgba=(0.54, 0.57, 0.60, 1.0))
    model.material("shaft_steel", rgba=(0.19, 0.20, 0.21, 1.0))
    model.material("red_mark", rgba=(0.78, 0.08, 0.05, 1.0))

    frame = model.part("side_frame")
    frame.visual(
        mesh_from_cadquery(_make_side_frame_shape(), "side_frame"),
        material="frame_paint",
        name="side_plate",
    )
    frame.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, PLATE_THICKNESS + BOSS_LENGTH, PLATE_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
    )

    unique_teeth = sorted({teeth for gears in GEAR_STACKS.values() for _, teeth, _, _ in gears})
    gear_meshes = {
        teeth: mesh_from_cadquery(_make_gear_shape(teeth), f"spur_{teeth}_tooth", unit_scale=0.001)
        for teeth in unique_teeth
    }

    shaft_order = [
        ("input_shaft", "input", 0.38),
        ("compound_shaft_0", "compound_0", 0.52),
        ("compound_shaft_1", "compound_1", 0.50),
        ("compound_shaft_2", "compound_2", 0.48),
        ("output_shaft", "output", 0.42),
    ]
    for part_name, center_key, mass in shaft_order:
        shaft = model.part(part_name)
        _add_shaft_visuals(shaft, GEAR_STACKS[part_name], gear_meshes)
        max_plane = max(GEAR_PLANES[stage] for _, _, stage, _ in GEAR_STACKS[part_name])
        shaft.inertial = Inertial.from_geometry(
            Cylinder(radius=max(_gear_outer_radius(teeth) for _, teeth, _, _ in GEAR_STACKS[part_name]), length=GEAR_WIDTH),
            mass=mass,
            origin=Origin(xyz=(0.0, max_plane * 0.5, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        )

        x, z = SHAFT_CENTERS[center_key]
        model.articulation(
            f"{center_key}_spin",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=shaft,
            origin=Origin(xyz=(x, 0.0, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0, lower=-2.0 * pi, upper=2.0 * pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("side_frame")

    expected_joints = {
        "input_spin": "input_shaft",
        "compound_0_spin": "compound_shaft_0",
        "compound_1_spin": "compound_shaft_1",
        "compound_2_spin": "compound_shaft_2",
        "output_spin": "output_shaft",
    }
    for joint_name, child_name in expected_joints.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_is_revolute",
            joint is not None and joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint_name} should be a separate supported revolute shaft joint.",
        )
        if joint is not None:
            ctx.check(
                f"{joint_name}_axis_centerline",
                tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
                details=f"axis={joint.axis!r}",
            )
        shaft = object_model.get_part(child_name)
        ctx.allow_overlap(
            shaft,
            frame,
            elem_a="shaft_core",
            elem_b="side_plate",
            reason="The rotating shaft journal is intentionally captured in the side-frame bearing bore with a tiny hidden visual embed.",
        )
        ctx.expect_within(
            shaft,
            frame,
            axes="xz",
            inner_elem="shaft_core",
            outer_elem="side_plate",
            margin=0.0,
            name=f"{child_name}_centerline_lies_in_plate",
        )
        ctx.expect_overlap(
            shaft,
            frame,
            axes="y",
            elem_a="shaft_core",
            elem_b="side_plate",
            min_overlap=0.020,
            name=f"{child_name}_passes_through_support",
        )

    for part_name, gears in GEAR_STACKS.items():
        shaft = object_model.get_part(part_name)
        for visual_name, _, _, _ in gears:
            ctx.expect_gap(
                shaft,
                frame,
                axis="y",
                positive_elem=visual_name,
                negative_elem="side_plate",
                min_gap=0.009,
                name=f"{visual_name}_clears_side_support",
            )

    meshing_pairs = [
        ("input_to_compound_0", "input", "compound_0", 24, 48),
        ("compound_0_to_1", "compound_0", "compound_1", 18, 42),
        ("compound_1_to_2", "compound_1", "compound_2", 18, 36),
        ("compound_2_to_output", "compound_2", "output", 16, 32),
    ]
    for name, a, b, teeth_a, teeth_b in meshing_pairs:
        actual = _actual_spacing(SHAFT_CENTERS[a], SHAFT_CENTERS[b])
        expected = _expected_pair_spacing(teeth_a, teeth_b)
        ctx.check(
            f"{name}_tooth_tip_spacing",
            abs(actual - expected) <= 0.0025,
            details=f"actual={actual:.4f}, expected={expected:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
