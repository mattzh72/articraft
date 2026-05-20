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


def rectangular_tube(length: float, width: float, height: float, wall: float, radius: float = 0.0):
    """Open-ended rectangular structural tube, authored along local X."""
    outer = cq.Workplane("XY").box(length, width, height)
    inner = cq.Workplane("XY").box(length + 0.04, width - 2.0 * wall, height - 2.0 * wall)
    tube = outer.cut(inner)
    if radius > 0.0:
        tube = tube.edges("|X").fillet(radius)
    return tube


def add_box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def add_bolt(part, name, xyz, radius, length, material, axis="z"):
    rpy = (0.0, 0.0, 0.0)
    if axis == "y":
        rpy = (pi / 2.0, 0.0, 0.0)
    elif axis == "x":
        rpy = (0.0, pi / 2.0, 0.0)
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def add_top_cover(part, prefix, x, tube_height, material_cover, material_bolt, width_y):
    cover_z = tube_height / 2.0 + 0.006
    add_box(part, f"{prefix}_cover", (0.24, width_y, 0.012), (x, 0.0, cover_z), material_cover)
    for i, (dx, dy) in enumerate(((-0.085, -0.040), (-0.085, 0.040), (0.085, -0.040), (0.085, 0.040))):
        add_bolt(part, f"{prefix}_bolt_{i}", (x + dx, dy, cover_z + 0.009), 0.008, 0.006, material_bolt)


def add_side_bolts(part, prefix, x, y_surface, z_values, material):
    idx = 0
    for side in (-1.0, 1.0):
        y = side * (y_surface + 0.004)
        for z in z_values:
            add_bolt(part, f"{prefix}_side_bolt_{idx}", (x, y, z), 0.007, 0.008, material, axis="y")
            idx += 1


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_boom_mechanical_study")

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.23, 0.24, 1.0))
    dark_steel = model.material("dark_oxide_steel", rgba=(0.045, 0.047, 0.050, 1.0))
    raw_edge = model.material("machined_edges", rgba=(0.58, 0.60, 0.57, 1.0))
    black_pad = model.material("black_wear_pad", rgba=(0.015, 0.016, 0.014, 1.0))
    access_blue = model.material("blued_access_cover", rgba=(0.07, 0.10, 0.13, 1.0))
    bronze = model.material("bronze_bushing", rgba=(0.58, 0.39, 0.18, 1.0))

    # Fixed fabricated root support and largest sleeve.
    root = model.part("root_mount")
    add_box(root, "floor_plate", (0.90, 0.54, 0.050), (0.28, 0.0, 0.025), dark_steel)
    add_box(root, "rear_anchor", (0.075, 0.54, 0.320), (-0.13, 0.0, 0.210), dark_steel)
    add_box(root, "saddle_plate", (0.96, 0.37, 0.055), (0.52, 0.0, 0.6475), dark_steel)
    for i, y in enumerate((-0.165, 0.165)):
        add_box(root, f"longitudinal_web_{i}", (0.84, 0.040, 0.625), (0.42, y, 0.3625), dark_steel)
    for i, x in enumerate((-0.030, 0.265, 0.555)):
        add_box(root, f"cross_tie_{i}", (0.055, 0.43, 0.115), (x, 0.0, 0.1075), dark_steel)
    for i, (x, y) in enumerate(((-0.05, -0.20), (-0.05, 0.20), (0.55, -0.20), (0.55, 0.20))):
        add_bolt(root, f"base_anchor_bolt_{i}", (x, y, 0.055), 0.018, 0.012, raw_edge)

    root.visual(
        mesh_from_cadquery(rectangular_tube(1.25, 0.340, 0.250, 0.025, radius=0.006), "outer_sleeve_mesh"),
        origin=Origin(xyz=(0.625, 0.0, 0.800)),
        material=painted_steel,
        name="outer_sleeve",
    )
    add_box(root, "rear_sleeve_cap", (0.080, 0.375, 0.285), (0.000, 0.0, 0.800), painted_steel)
    add_top_cover(root, "outer_service", 0.340, 0.250, access_blue, raw_edge, 0.210)
    # Front guide pack: bolted wear pads around the exit mouth of the outer sleeve.
    add_box(root, "outer_top_wear_pad", (0.180, 0.170, 0.016), (1.165, 0.0, 0.933), black_pad)
    add_box(root, "outer_side_wear_pad_0", (0.180, 0.014, 0.125), (1.165, -0.177, 0.800), black_pad)
    add_box(root, "outer_side_wear_pad_1", (0.180, 0.014, 0.125), (1.165, 0.177, 0.800), black_pad)
    add_box(root, "outer_inner_top_pad", (0.170, 0.120, 0.0175), (1.165, 0.0, 0.89125), black_pad)
    add_box(root, "outer_inner_side_pad_0", (0.170, 0.0225, 0.095), (1.165, -0.13375, 0.800), black_pad)
    add_box(root, "outer_inner_side_pad_1", (0.170, 0.0225, 0.095), (1.165, 0.13375, 0.800), black_pad)
    add_box(root, "outer_stop_bar", (0.040, 0.360, 0.035), (1.245, 0.0, 0.9425), raw_edge)
    add_side_bolts(root, "outer_guide", 1.165, 0.184, (0.762, 0.838), raw_edge)

    # First moving rectangular section.  Its local frame is the mouth of the outer sleeve.
    stage_1 = model.part("stage_1")
    stage_1.visual(
        mesh_from_cadquery(rectangular_tube(1.65, 0.245, 0.165, 0.018, radius=0.004), "stage_1_tube_mesh"),
        origin=Origin(xyz=(-0.125, 0.0, 0.0)),
        material=painted_steel,
        name="stage_1_tube",
    )
    add_top_cover(stage_1, "stage_1_service", 0.380, 0.165, access_blue, raw_edge, 0.145)
    add_box(stage_1, "stage_1_top_wear_pad", (0.145, 0.115, 0.012), (0.585, 0.0, 0.0885), black_pad)
    add_box(stage_1, "stage_1_side_wear_pad_0", (0.145, 0.012, 0.080), (0.585, -0.1285, 0.0), black_pad)
    add_box(stage_1, "stage_1_side_wear_pad_1", (0.145, 0.012, 0.080), (0.585, 0.1285, 0.0), black_pad)
    add_box(stage_1, "stage_1_inner_top_pad", (0.135, 0.080, 0.0095), (0.585, 0.0, 0.05975), black_pad)
    add_box(stage_1, "stage_1_inner_side_pad_0", (0.135, 0.017, 0.055), (0.585, -0.096, 0.0), black_pad)
    add_box(stage_1, "stage_1_inner_side_pad_1", (0.135, 0.017, 0.055), (0.585, 0.096, 0.0), black_pad)
    add_box(stage_1, "stage_1_stop_block", (0.050, 0.270, 0.026), (0.675, 0.0, 0.0955), raw_edge)
    add_side_bolts(stage_1, "stage_1_guide", 0.585, 0.1345, (-0.030, 0.030), raw_edge)

    # Second moving section, nested in stage_1.
    stage_2 = model.part("stage_2")
    stage_2.visual(
        mesh_from_cadquery(rectangular_tube(1.36, 0.175, 0.110, 0.014, radius=0.003), "stage_2_tube_mesh"),
        origin=Origin(xyz=(-0.100, 0.0, 0.0)),
        material=painted_steel,
        name="stage_2_tube",
    )
    add_top_cover(stage_2, "stage_2_service", 0.290, 0.110, access_blue, raw_edge, 0.104)
    add_box(stage_2, "stage_2_top_wear_pad", (0.125, 0.085, 0.010), (0.485, 0.0, 0.060), black_pad)
    add_box(stage_2, "stage_2_side_wear_pad_0", (0.125, 0.010, 0.058), (0.485, -0.0925, 0.0), black_pad)
    add_box(stage_2, "stage_2_side_wear_pad_1", (0.125, 0.010, 0.058), (0.485, 0.0925, 0.0), black_pad)
    add_box(stage_2, "stage_2_inner_top_pad", (0.115, 0.062, 0.0085), (0.485, 0.0, 0.03675), black_pad)
    add_box(stage_2, "stage_2_inner_side_pad_0", (0.115, 0.0135, 0.038), (0.485, -0.06675, 0.0), black_pad)
    add_box(stage_2, "stage_2_inner_side_pad_1", (0.115, 0.0135, 0.038), (0.485, 0.06675, 0.0), black_pad)
    add_box(stage_2, "stage_2_stop_block", (0.045, 0.205, 0.022), (0.560, 0.0, 0.076), raw_edge)
    add_side_bolts(stage_2, "stage_2_guide", 0.485, 0.0975, (-0.020, 0.020), raw_edge)

    # Final narrow stage with a functional pin-lug tip fixture.
    stage_3 = model.part("stage_3")
    stage_3.visual(
        mesh_from_cadquery(rectangular_tube(1.10, 0.120, 0.065, 0.010, radius=0.002), "stage_3_tube_mesh"),
        origin=Origin(xyz=(-0.050, 0.0, 0.0)),
        material=painted_steel,
        name="stage_3_tube",
    )
    add_top_cover(stage_3, "stage_3_service", 0.210, 0.065, access_blue, raw_edge, 0.074)
    add_box(stage_3, "stage_3_stop_block", (0.040, 0.145, 0.018), (0.475, 0.0, 0.0415), raw_edge)
    add_box(stage_3, "tip_base_block", (0.085, 0.120, 0.070), (0.5425, 0.0, 0.0), raw_edge)
    add_box(stage_3, "tip_lug_0", (0.180, 0.018, 0.145), (0.655, -0.069, 0.025), raw_edge)
    add_box(stage_3, "tip_lug_1", (0.180, 0.018, 0.145), (0.655, 0.069, 0.025), raw_edge)
    add_bolt(stage_3, "tip_pin", (0.660, 0.0, 0.030), 0.017, 0.170, dark_steel, axis="y")
    add_bolt(stage_3, "tip_bushing_0", (0.660, -0.079, 0.030), 0.027, 0.014, bronze, axis="y")
    add_bolt(stage_3, "tip_bushing_1", (0.660, 0.079, 0.030), 0.027, 0.014, bronze, axis="y")

    model.articulation(
        "root_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=root,
        child=stage_1,
        origin=Origin(xyz=(1.250, 0.0, 0.800)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.18, lower=0.0, upper=0.65),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.700, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6200.0, velocity=0.16, lower=0.0, upper=0.55),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.580, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3900.0, velocity=0.14, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_mount")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    j1 = object_model.get_articulation("root_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")
    j3 = object_model.get_articulation("stage_2_to_stage_3")

    ctx.expect_within(stage_1, root, axes="yz", inner_elem="stage_1_tube", outer_elem="outer_sleeve", margin=0.0, name="stage 1 is centered in outer sleeve")
    ctx.expect_within(stage_2, stage_1, axes="yz", inner_elem="stage_2_tube", outer_elem="stage_1_tube", margin=0.0, name="stage 2 is centered in stage 1")
    ctx.expect_within(stage_3, stage_2, axes="yz", inner_elem="stage_3_tube", outer_elem="stage_2_tube", margin=0.0, name="stage 3 is centered in stage 2")
    ctx.expect_overlap(stage_1, root, axes="x", elem_a="stage_1_tube", elem_b="outer_sleeve", min_overlap=0.90, name="collapsed stage 1 retains deep sleeve insertion")
    ctx.expect_overlap(stage_2, stage_1, axes="x", elem_a="stage_2_tube", elem_b="stage_1_tube", min_overlap=0.72, name="collapsed stage 2 retains deep sleeve insertion")
    ctx.expect_overlap(stage_3, stage_2, axes="x", elem_a="stage_3_tube", elem_b="stage_2_tube", min_overlap=0.55, name="collapsed stage 3 retains deep sleeve insertion")

    rest_tip = ctx.part_world_position(stage_3)
    with ctx.pose({j1: 0.65, j2: 0.55, j3: 0.45}):
        ctx.expect_within(stage_1, root, axes="yz", inner_elem="stage_1_tube", outer_elem="outer_sleeve", margin=0.0, name="extended stage 1 remains centered")
        ctx.expect_within(stage_2, stage_1, axes="yz", inner_elem="stage_2_tube", outer_elem="stage_1_tube", margin=0.0, name="extended stage 2 remains centered")
        ctx.expect_within(stage_3, stage_2, axes="yz", inner_elem="stage_3_tube", outer_elem="stage_2_tube", margin=0.0, name="extended stage 3 remains centered")
        ctx.expect_overlap(stage_1, root, axes="x", elem_a="stage_1_tube", elem_b="outer_sleeve", min_overlap=0.28, name="extended stage 1 keeps retained insertion")
        ctx.expect_overlap(stage_2, stage_1, axes="x", elem_a="stage_2_tube", elem_b="stage_1_tube", min_overlap=0.20, name="extended stage 2 keeps retained insertion")
        ctx.expect_overlap(stage_3, stage_2, axes="x", elem_a="stage_3_tube", elem_b="stage_2_tube", min_overlap=0.13, name="extended stage 3 keeps retained insertion")
        extended_tip = ctx.part_world_position(stage_3)

    ctx.check(
        "serial stages extend outward along boom axis",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 1.55,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
