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


def _square_tube(outer: float, inner: float, length: float) -> cq.Workplane:
    """Open ended square mast tube, centered on X/Y and rising from local Z=0."""
    return (
        cq.Workplane("XY")
        .box(outer, outer, length, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .rect(inner, inner)
        .cutThruAll()
    )


def _rounded_plate(size_x: float, size_y: float, thickness: float) -> cq.Workplane:
    """Compact camera/sensor mounting plate with two through slots."""
    plate = (
        cq.Workplane("XY")
        .box(size_x, size_y, thickness)
        .edges("|Z")
        .fillet(0.010)
    )
    for x in (-0.045, 0.045):
        plate = (
            plate.faces(">Z")
            .workplane()
            .center(x, 0.0)
            .slot2D(0.040, 0.012, 0.0)
            .cutThruAll()
        )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_inspection_mast")

    dark_steel = model.material("dark_steel", rgba=(0.06, 0.07, 0.075, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.65, 0.67, 1.0))
    pale_aluminum = model.material("pale_aluminum", rgba=(0.78, 0.80, 0.80, 1.0))
    orange = model.material("safety_orange", rgba=(1.0, 0.42, 0.06, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))

    # Fixed welded base frame and the stationary outer mast sleeve.
    base = model.part("base_frame")
    base.visual(
        Box((0.45, 0.34, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_plate",
    )
    for y in (-0.13, 0.13):
        base.visual(
            Box((0.70, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0275)),
            material=dark_steel,
            name=f"skid_rail_{0 if y < 0 else 1}",
        )
    for x in (-0.31, 0.31):
        base.visual(
            Box((0.055, 0.42, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=dark_steel,
            name=f"cross_bar_{0 if x < 0 else 1}",
        )
    for x in (-0.31, 0.31):
        for y in (-0.13, 0.13):
            base.visual(
                Box((0.085, 0.060, 0.018)),
                origin=Origin(xyz=(x, y, -0.009)),
                material=rubber,
                name=f"rubber_foot_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )
    base.visual(
        mesh_from_cadquery(_square_tube(0.130, 0.112, 0.860), "base_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=black,
        name="base_sleeve",
    )
    base.visual(
        mesh_from_cadquery(_square_tube(0.160, 0.126, 0.060), "base_top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=dark_steel,
        name="base_top_collar",
    )
    for y in (-0.066, 0.066):
        base.visual(
            Box((0.245, 0.014, 0.125)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=dark_steel,
            name=f"gusset_plate_{0 if y < 0 else 1}",
        )
    for x in (-0.066, 0.066):
        base.visual(
            Box((0.014, 0.245, 0.125)),
            origin=Origin(xyz=(x, 0.0, 0.095)),
            material=dark_steel,
            name=f"side_gusset_{0 if x < 0 else 1}",
        )

    # First moving mast tube: long enough below its joint to remain inserted
    # after full extension.
    lower = model.part("lower_stage")
    lower.visual(
        mesh_from_cadquery(_square_tube(0.092, 0.070, 1.500), "lower_tube"),
        origin=Origin(xyz=(0.0, 0.0, -0.700)),
        material=aluminum,
        name="lower_tube",
    )
    lower.visual(
        mesh_from_cadquery(_square_tube(0.140, 0.090, 0.035), "lower_stop_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=orange,
        name="lower_stop_collar",
    )
    lower.visual(
        mesh_from_cadquery(_square_tube(0.112, 0.090, 0.060), "lower_top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        material=orange,
        name="lower_top_collar",
    )
    lower.visual(
        Cylinder(radius=0.008, length=0.055),
        origin=Origin(xyz=(0.078, 0.0, 0.780), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="lower_clamp_knob",
    )

    # Second moving mast tube nested in the lower stage.
    upper = model.part("upper_stage")
    upper.visual(
        mesh_from_cadquery(_square_tube(0.058, 0.039, 1.150), "upper_tube"),
        origin=Origin(xyz=(0.0, 0.0, -0.600)),
        material=pale_aluminum,
        name="upper_tube",
    )
    upper.visual(
        mesh_from_cadquery(_square_tube(0.092, 0.057, 0.032), "upper_stop_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=orange,
        name="upper_stop_collar",
    )
    upper.visual(
        mesh_from_cadquery(_square_tube(0.076, 0.057, 0.050), "upper_top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=orange,
        name="upper_top_collar",
    )
    upper.visual(
        Cylinder(radius=0.0065, length=0.044),
        origin=Origin(xyz=(0.054, 0.0, 0.535), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="upper_clamp_knob",
    )

    # Compact rotating inspection pan head.  The rectangular plate makes the
    # vertical-axis rotation visually obvious.
    head = model.part("pan_head")
    head.visual(
        Cylinder(radius=0.018, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=black,
        name="head_stem",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="turntable_bearing",
    )
    head.visual(
        Box((0.135, 0.100, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=black,
        name="pan_motor_body",
    )
    head.visual(
        mesh_from_cadquery(_rounded_plate(0.185, 0.105, 0.014), "mounting_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=aluminum,
        name="mounting_plate",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=dark_steel,
        name="center_mount_screw",
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.905)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.22, lower=0.0, upper=0.50),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.PRISMATIC,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.800)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.22, lower=0.0, upper=0.40),
    )
    model.articulation(
        "upper_to_head",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_frame")
    lower = object_model.get_part("lower_stage")
    upper = object_model.get_part("upper_stage")
    head = object_model.get_part("pan_head")
    base_slide = object_model.get_articulation("base_to_lower")
    upper_slide = object_model.get_articulation("lower_to_upper")
    pan = object_model.get_articulation("upper_to_head")

    ctx.expect_within(
        lower,
        base,
        axes="xy",
        inner_elem="lower_tube",
        outer_elem="base_sleeve",
        margin=0.0,
        name="lower mast nests inside base sleeve",
    )
    ctx.expect_overlap(
        lower,
        base,
        axes="z",
        elem_a="lower_tube",
        elem_b="base_sleeve",
        min_overlap=0.20,
        name="lower mast retained in base sleeve",
    )
    ctx.expect_within(
        upper,
        lower,
        axes="xy",
        inner_elem="upper_tube",
        outer_elem="lower_tube",
        margin=0.0,
        name="upper mast nests inside lower tube",
    )
    ctx.expect_overlap(
        upper,
        lower,
        axes="z",
        elem_a="upper_tube",
        elem_b="lower_tube",
        min_overlap=0.20,
        name="upper mast retained in lower tube",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({base_slide: 0.50, upper_slide: 0.40}):
        ctx.expect_overlap(
            lower,
            base,
            axes="z",
            elem_a="lower_tube",
            elem_b="base_sleeve",
            min_overlap=0.18,
            name="extended lower stage remains captured",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="z",
            elem_a="upper_tube",
            elem_b="lower_tube",
            min_overlap=0.18,
            name="extended upper stage remains captured",
        )
        extended_head_pos = ctx.part_world_position(head)

    ctx.check(
        "mast extends upward",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[2] > rest_head_pos[2] + 0.85,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    plate_aabb_0 = ctx.part_element_world_aabb(head, elem="mounting_plate")
    with ctx.pose({pan: math.pi / 2.0}):
        plate_aabb_90 = ctx.part_element_world_aabb(head, elem="mounting_plate")

    def _xy_size(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (hi[0] - lo[0], hi[1] - lo[1])

    size_0 = _xy_size(plate_aabb_0)
    size_90 = _xy_size(plate_aabb_90)
    ctx.check(
        "pan head rotates about vertical mast axis",
        size_0 is not None
        and size_90 is not None
        and size_90[0] < size_0[0] - 0.040
        and size_90[1] > size_0[1] + 0.040,
        details=f"size_0={size_0}, size_90={size_90}",
    )

    return ctx.report()


object_model = build_object_model()
