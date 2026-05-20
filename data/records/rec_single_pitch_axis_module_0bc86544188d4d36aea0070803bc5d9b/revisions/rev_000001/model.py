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


def _bearing_block_mesh(name: str):
    """Pillow-block-like support with a real through bore along local Y."""
    width_x = 0.18
    depth_y = 0.13
    height_z = 0.15
    bore_diameter = 0.062
    bore_center_z = 0.100

    body = cq.Workplane("XY").box(
        width_x,
        depth_y,
        height_z,
        centered=(True, True, False),
    )

    bore = (
        cq.Workplane("XZ")
        .center(0.0, bore_center_z)
        .circle(bore_diameter / 2.0)
        .extrude(depth_y * 1.6, both=True)
    )
    body = body.cut(bore)

    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.08)


def _bushing_ring_mesh(name: str):
    """Thin annular bearing liner, axis along local Y."""
    outer_diameter = 0.082
    inner_diameter = 0.054
    thickness_y = 0.014

    ring = (
        cq.Workplane("XZ")
        .circle(outer_diameter / 2.0)
        .circle(inner_diameter / 2.0)
        .extrude(thickness_y, both=True)
    )
    return mesh_from_cadquery(ring, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_cradle")

    painted_steel = Material("painted_steel", rgba=(0.10, 0.18, 0.24, 1.0))
    dark_steel = Material("dark_steel", rgba=(0.03, 0.035, 0.04, 1.0))
    machined_steel = Material("machined_steel", rgba=(0.66, 0.68, 0.66, 1.0))
    bearing_casting = Material("bearing_casting", rgba=(0.36, 0.39, 0.39, 1.0))
    bronze = Material("bronze_bushing", rgba=(0.70, 0.45, 0.18, 1.0))
    plate_finish = Material("equipment_plate_black", rgba=(0.015, 0.017, 0.020, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.95, 0.95, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="base_plate",
    )

    for x in (-0.40, 0.40):
        frame.visual(
            Box((0.08, 0.86, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0675)),
            material=painted_steel,
            name=f"cross_tie_{0 if x < 0 else 1}",
        )

    for side, suffix, block_name in (
        (-1.0, "0", "bearing_block_0"),
        (1.0, "1", "bearing_block_1"),
    ):
        frame.visual(
            Box((0.24, 0.060, 0.36)),
            origin=Origin(xyz=(0.0, side * 0.350, 0.220)),
            material=painted_steel,
            name=f"side_support_{suffix}",
        )
        frame.visual(
            _bearing_block_mesh(block_name),
            origin=Origin(xyz=(0.0, side * 0.420, 0.385)),
            material=bearing_casting,
            name=block_name,
        )

        for face, tag in ((side * 0.350, "inner"), (side * 0.490, "outer")):
            frame.visual(
                _bushing_ring_mesh(f"bushing_{suffix}_{tag}"),
                origin=Origin(xyz=(0.0, face, 0.485)),
                material=bronze,
                name=f"bushing_{suffix}_{tag}",
            )

    equipment_plate = model.part("equipment_plate")
    equipment_plate.visual(
        Box((0.70, 0.58, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=plate_finish,
        name="plate_panel",
    )
    equipment_plate.visual(
        Cylinder(radius=0.022, length=1.02),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="axle",
    )

    for side, suffix in ((-1.0, "0"), (1.0, "1")):
        equipment_plate.visual(
            Box((0.16, 0.040, 0.155)),
            origin=Origin(xyz=(0.0, side * 0.300, -0.045)),
            material=plate_finish,
            name=f"hanger_cheek_{suffix}",
        )
        equipment_plate.visual(
            Cylinder(radius=0.038, length=0.028),
            origin=Origin(xyz=(0.0, side * 0.315, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=machined_steel,
            name=f"shaft_collar_{suffix}",
        )

    for i, (x, y) in enumerate(
        ((-0.25, -0.18), (-0.25, 0.18), (0.25, -0.18), (0.25, 0.18))
    ):
        equipment_plate.visual(
            Cylinder(radius=0.018, length=0.008),
            origin=Origin(xyz=(x, y, -0.0735)),
            material=machined_steel,
            name=f"mount_bolt_{i}",
        )

    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=equipment_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.70, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    equipment_plate = object_model.get_part("equipment_plate")
    pitch_joint = object_model.get_articulation("pitch_joint")

    ctx.check(
        "single pitch joint",
        len(object_model.articulations) == 1
        and pitch_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in pitch_joint.axis) == (0.0, 1.0, 0.0),
        details=f"joint_count={len(object_model.articulations)}, axis={pitch_joint.axis}",
    )

    with ctx.pose({pitch_joint: 0.0}):
        ctx.expect_gap(
            equipment_plate,
            frame,
            axis="z",
            positive_elem="plate_panel",
            negative_elem="base_plate",
            min_gap=0.25,
            name="plate is suspended above the base",
        )
        ctx.expect_overlap(
            equipment_plate,
            frame,
            axes="y",
            elem_a="axle",
            elem_b="bearing_block_0",
            min_overlap=0.08,
            name="axle retained in bearing block 0",
        )
        ctx.expect_overlap(
            equipment_plate,
            frame,
            axes="y",
            elem_a="axle",
            elem_b="bearing_block_1",
            min_overlap=0.08,
            name="axle retained in bearing block 1",
        )
        rest_panel_aabb = ctx.part_element_world_aabb(equipment_plate, elem="plate_panel")

    with ctx.pose({pitch_joint: 0.55}):
        ctx.expect_gap(
            equipment_plate,
            frame,
            axis="z",
            positive_elem="plate_panel",
            negative_elem="base_plate",
            min_gap=0.12,
            name="tilted plate clears the base",
        )
        tilted_panel_aabb = ctx.part_element_world_aabb(equipment_plate, elem="plate_panel")

    ctx.check(
        "plate visibly pitches",
        rest_panel_aabb is not None
        and tilted_panel_aabb is not None
        and tilted_panel_aabb[1][2] > rest_panel_aabb[1][2] + 0.10,
        details=f"rest={rest_panel_aabb}, tilted={tilted_panel_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
