from __future__ import annotations

import math

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


def _cover_mesh(name: str, length: float, width: float, thickness: float, hole_diameter: float):
    """Small removable bolted access cover with real through-holes."""

    holes = [
        (-0.38 * length, -0.28 * width),
        (-0.38 * length, 0.28 * width),
        (0.38 * length, -0.28 * width),
        (0.38 * length, 0.28 * width),
    ]
    plate = (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .faces(">Z")
        .workplane()
        .pushPoints(holes)
        .hole(hole_diameter)
    )
    return mesh_from_cadquery(plate, name, tolerance=0.0006, angular_tolerance=0.08)


def _box(part, name: str, size, xyz, material: str):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl_z(part, name: str, radius: float, length: float, xyz, material: str):
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl_y(part, name: str, radius: float, length: float, xyz, material: str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_x(part, name: str, radius: float, length: float, xyz, material: str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_prismatic_chain")

    model.material("blasted_steel", rgba=(0.46, 0.48, 0.48, 1.0))
    model.material("dark_oxide", rgba=(0.035, 0.038, 0.040, 1.0))
    model.material("ground_rail", rgba=(0.82, 0.84, 0.80, 1.0))
    model.material("anodized_stage", rgba=(0.25, 0.30, 0.33, 1.0))
    model.material("wear_pad", rgba=(0.06, 0.055, 0.050, 1.0))
    model.material("stop_red", rgba=(0.55, 0.06, 0.035, 1.0))
    model.material("brass_shim", rgba=(0.78, 0.55, 0.21, 1.0))

    cover_base = _cover_mesh("base_access_cover", 0.34, 0.045, 0.006, 0.010)
    cover_stage_1 = _cover_mesh("stage_1_access_cover", 0.26, 0.038, 0.006, 0.008)
    cover_stage_2 = _cover_mesh("stage_2_access_cover", 0.20, 0.032, 0.005, 0.006)

    base = model.part("base")
    _box(base, "bed_plate", (1.02, 0.36, 0.025), (0.0, 0.0, 0.0125), "blasted_steel")
    _box(base, "rear_foot", (0.14, 0.50, 0.030), (-0.39, 0.0, 0.015), "dark_oxide")
    _box(base, "front_foot", (0.14, 0.50, 0.030), (0.39, 0.0, 0.015), "dark_oxide")
    _box(base, "rear_riser", (0.16, 0.28, 0.055), (-0.26, 0.0, 0.050), "blasted_steel")
    _box(base, "front_riser", (0.16, 0.28, 0.055), (0.26, 0.0, 0.050), "blasted_steel")
    base.visual(Box((0.75, 0.27, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.065)), material="ground_rail", name="guide_floor")
    base.visual(Box((0.75, 0.025, 0.120)), origin=Origin(xyz=(0.0, 0.1225, 0.135)), material="blasted_steel", name="guide_wall_0")
    _box(base, "guide_wall_1", (0.75, 0.025, 0.120), (0.0, -0.1225, 0.135), "blasted_steel")
    _box(base, "top_lip_0", (0.75, 0.064, 0.018), (0.0, 0.083, 0.204), "blasted_steel")
    _box(base, "top_lip_1", (0.75, 0.064, 0.018), (0.0, -0.083, 0.204), "blasted_steel")
    _box(base, "rear_stop", (0.025, 0.050, 0.070), (-0.3875, 0.160, 0.125), "stop_red")
    _box(base, "front_stop", (0.025, 0.050, 0.070), (0.3875, 0.160, 0.125), "stop_red")
    base.visual(cover_base, origin=Origin(xyz=(0.0, 0.083, 0.216)), material="dark_oxide", name="side_cover_0")
    base.visual(cover_base, origin=Origin(xyz=(0.0, -0.083, 0.216)), material="dark_oxide", name="side_cover_1")
    for i, x in enumerate((-0.44, -0.34, 0.34, 0.44)):
        _cyl_z(base, f"foot_bolt_{i}", 0.013, 0.006, (x, 0.190, 0.033), "dark_oxide")
        _cyl_z(base, f"foot_bolt_{i + 4}", 0.013, 0.006, (x, -0.190, 0.033), "dark_oxide")
    for i, x in enumerate((-0.13, 0.13)):
        _cyl_z(base, f"cover_screw_{i}", 0.006, 0.005, (x, 0.083, 0.2215), "ground_rail")
        _cyl_z(base, f"cover_screw_{i + 2}", 0.006, 0.005, (x, -0.083, 0.2215), "ground_rail")

    stage_1 = model.part("stage_1")
    stage_1.visual(Box((0.84, 0.184, 0.055)), origin=Origin(xyz=(-0.18, 0.0, 0.0)), material="anodized_stage", name="main_slide")
    stage_1.visual(Box((0.58, 0.018, 0.038)), origin=Origin(xyz=(-0.18, 0.101, 0.0)), material="wear_pad", name="pad_0")
    _box(stage_1, "pad_1", (0.58, 0.018, 0.038), (-0.18, -0.101, 0.0), "wear_pad")
    _box(stage_1, "rear_stop_block", (0.035, 0.205, 0.070), (-0.54, 0.0, 0.005), "stop_red")
    _box(stage_1, "front_stop_block", (0.035, 0.205, 0.070), (0.245, 0.0, 0.005), "stop_red")
    _box(stage_1, "web", (0.46, 0.050, 0.120), (0.04, 0.0, 0.0875), "anodized_stage")
    stage_1.visual(Box((0.62, 0.190, 0.014)), origin=Origin(xyz=(0.11, 0.0, 0.1485)), material="ground_rail", name="guide_floor")
    stage_1.visual(Box((0.62, 0.020, 0.098)), origin=Origin(xyz=(0.11, 0.092, 0.2045)), material="anodized_stage", name="guide_wall_0")
    _box(stage_1, "guide_wall_1", (0.62, 0.020, 0.098), (0.11, -0.092, 0.2045), "anodized_stage")
    _box(stage_1, "top_lip_0", (0.62, 0.052, 0.016), (0.11, 0.066, 0.2615), "anodized_stage")
    _box(stage_1, "top_lip_1", (0.62, 0.052, 0.016), (0.11, -0.066, 0.2615), "anodized_stage")
    _box(stage_1, "rail_stop", (0.025, 0.035, 0.060), (0.4325, 0.1195, 0.189), "stop_red")
    _box(stage_1, "rear_brace", (0.045, 0.180, 0.090), (-0.215, 0.0, 0.185), "anodized_stage")
    stage_1.visual(cover_stage_1, origin=Origin(xyz=(0.11, 0.066, 0.272)), material="dark_oxide", name="side_cover_0")
    stage_1.visual(cover_stage_1, origin=Origin(xyz=(0.11, -0.066, 0.272)), material="dark_oxide", name="side_cover_1")
    for i, x in enumerate((0.01, 0.21)):
        _cyl_z(stage_1, f"cover_screw_{i}", 0.005, 0.004, (x, 0.066, 0.277), "ground_rail")
        _cyl_z(stage_1, f"cover_screw_{i + 2}", 0.005, 0.004, (x, -0.066, 0.277), "ground_rail")

    stage_2 = model.part("stage_2")
    stage_2.visual(Box((0.68, 0.138, 0.045)), origin=Origin(xyz=(-0.14, 0.0, 0.0)), material="blasted_steel", name="main_slide")
    stage_2.visual(Box((0.45, 0.013, 0.030)), origin=Origin(xyz=(-0.14, 0.0755, 0.0)), material="wear_pad", name="pad_0")
    _box(stage_2, "pad_1", (0.45, 0.013, 0.030), (-0.14, -0.0755, 0.0), "wear_pad")
    _box(stage_2, "rear_stop_block", (0.030, 0.155, 0.058), (-0.455, 0.0, 0.0), "stop_red")
    _box(stage_2, "front_stop_block", (0.030, 0.155, 0.058), (0.205, 0.0, 0.0), "stop_red")
    _box(stage_2, "web", (0.34, 0.040, 0.075), (0.03, 0.0, 0.060), "blasted_steel")
    stage_2.visual(Box((0.50, 0.145, 0.012)), origin=Origin(xyz=(0.10, 0.0, 0.1015)), material="ground_rail", name="guide_floor")
    stage_2.visual(Box((0.50, 0.017, 0.078)), origin=Origin(xyz=(0.10, 0.0695, 0.1465)), material="blasted_steel", name="guide_wall_0")
    _box(stage_2, "guide_wall_1", (0.50, 0.017, 0.078), (0.10, -0.0695, 0.1465), "blasted_steel")
    _box(stage_2, "top_lip_0", (0.50, 0.040, 0.012), (0.10, 0.049, 0.1915), "blasted_steel")
    _box(stage_2, "top_lip_1", (0.50, 0.040, 0.012), (0.10, -0.049, 0.1915), "blasted_steel")
    _box(stage_2, "rail_stop", (0.022, 0.031, 0.048), (0.361, 0.0935, 0.135), "stop_red")
    stage_2.visual(cover_stage_2, origin=Origin(xyz=(0.10, 0.049, 0.2000)), material="dark_oxide", name="side_cover_0")
    stage_2.visual(cover_stage_2, origin=Origin(xyz=(0.10, -0.049, 0.2000)), material="dark_oxide", name="side_cover_1")
    for i, x in enumerate((0.03, 0.17)):
        _cyl_z(stage_2, f"cover_screw_{i}", 0.004, 0.0035, (x, 0.049, 0.20425), "ground_rail")
        _cyl_z(stage_2, f"cover_screw_{i + 2}", 0.004, 0.0035, (x, -0.049, 0.20425), "ground_rail")

    stage_3 = model.part("stage_3")
    stage_3.visual(Box((0.56, 0.102, 0.035)), origin=Origin(xyz=(-0.11, 0.0, 0.0)), material="anodized_stage", name="main_slide")
    stage_3.visual(Box((0.38, 0.010, 0.024)), origin=Origin(xyz=(-0.11, 0.056, 0.0)), material="wear_pad", name="pad_0")
    _box(stage_3, "pad_1", (0.38, 0.010, 0.024), (-0.11, -0.056, 0.0), "wear_pad")
    _box(stage_3, "rear_stop_block", (0.026, 0.120, 0.045), (-0.375, 0.0, 0.0), "stop_red")
    _box(stage_3, "front_stop_block", (0.026, 0.120, 0.045), (0.175, 0.0, 0.0), "stop_red")
    _box(stage_3, "nose_plate", (0.050, 0.128, 0.070), (0.195, 0.0, 0.006), "blasted_steel")
    _box(stage_3, "clevis_cheek_0", (0.095, 0.018, 0.085), (0.252, 0.050, 0.018), "blasted_steel")
    _box(stage_3, "clevis_cheek_1", (0.095, 0.018, 0.085), (0.252, -0.050, 0.018), "blasted_steel")
    _box(stage_3, "clevis_bridge", (0.026, 0.118, 0.070), (0.215, 0.0, 0.018), "blasted_steel")
    _cyl_y(stage_3, "cross_pin", 0.008, 0.122, (0.260, 0.0, 0.026), "ground_rail")
    _cyl_x(stage_3, "adjuster_screw", 0.006, 0.070, (0.125, 0.0, 0.0295), "dark_oxide")
    _box(stage_3, "shim_plate", (0.090, 0.106, 0.006), (0.065, 0.0, 0.0205), "brass_shim")

    model.articulation(
        "slide_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(0.350, 0.0, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=750.0, velocity=0.35, lower=0.0, upper=0.28),
    )
    model.articulation(
        "slide_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.430, 0.0, 0.190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=520.0, velocity=0.32, lower=0.0, upper=0.23),
    )
    model.articulation(
        "slide_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(0.350, 0.0, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.28, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    slide_1 = object_model.get_articulation("slide_1")
    slide_2 = object_model.get_articulation("slide_2")
    slide_3 = object_model.get_articulation("slide_3")

    ctx.check(
        "three serial prismatic joints",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in (slide_1, slide_2, slide_3))
        and slide_1.parent == "base"
        and slide_1.child == "stage_1"
        and slide_2.parent == "stage_1"
        and slide_2.child == "stage_2"
        and slide_3.parent == "stage_2"
        and slide_3.child == "stage_3",
        details="Expected base -> stage_1 -> stage_2 -> stage_3 prismatic chain.",
    )

    ctx.expect_contact(stage_1, base, elem_a="pad_0", elem_b="guide_wall_0", contact_tol=1e-5)
    ctx.expect_contact(stage_2, stage_1, elem_a="pad_0", elem_b="guide_wall_0", contact_tol=1e-5)
    ctx.expect_contact(stage_3, stage_2, elem_a="pad_0", elem_b="guide_wall_0", contact_tol=1e-5)
    ctx.expect_within(stage_1, base, axes="yz", inner_elem="main_slide", margin=0.004, name="stage 1 fits base guide")
    ctx.expect_within(stage_2, stage_1, axes="yz", inner_elem="main_slide", margin=0.004, name="stage 2 fits stage 1 guide")
    ctx.expect_within(stage_3, stage_2, axes="yz", inner_elem="main_slide", margin=0.004, name="stage 3 fits stage 2 guide")
    ctx.expect_overlap(stage_1, base, axes="x", elem_a="main_slide", elem_b="guide_floor", min_overlap=0.30)
    ctx.expect_overlap(stage_2, stage_1, axes="x", elem_a="main_slide", elem_b="guide_floor", min_overlap=0.22)
    ctx.expect_overlap(stage_3, stage_2, axes="x", elem_a="main_slide", elem_b="guide_floor", min_overlap=0.18)

    rest_tip = ctx.part_world_position(stage_3)
    with ctx.pose({slide_1: 0.28, slide_2: 0.23, slide_3: 0.18}):
        ctx.expect_overlap(stage_1, base, axes="x", elem_a="main_slide", elem_b="guide_floor", min_overlap=0.18)
        ctx.expect_overlap(stage_2, stage_1, axes="x", elem_a="main_slide", elem_b="guide_floor", min_overlap=0.16)
        ctx.expect_overlap(stage_3, stage_2, axes="x", elem_a="main_slide", elem_b="guide_floor", min_overlap=0.12)
        extended_tip = ctx.part_world_position(stage_3)

    ctx.check(
        "chain extends along positive x",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.65,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
