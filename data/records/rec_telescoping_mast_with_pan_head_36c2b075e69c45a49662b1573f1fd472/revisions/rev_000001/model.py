from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _square_tube_visuals(
    part,
    *,
    prefix: str,
    outer: float,
    inner: float,
    length: float,
    center_z: float,
    material: str,
) -> None:
    """Build a hollow square mast section from overlapping wall members."""

    wall = (outer - inner) / 2.0
    half = outer / 2.0
    side_x = half - wall / 2.0
    side_y = half - wall / 2.0

    part.visual(
        Box((wall, outer, length)),
        origin=Origin(xyz=(side_x, 0.0, center_z)),
        material=material,
        name=f"{prefix}_side_0",
    )
    part.visual(
        Box((wall, outer, length)),
        origin=Origin(xyz=(-side_x, 0.0, center_z)),
        material=material,
        name=f"{prefix}_side_1",
    )
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, side_y, center_z)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((outer, wall, length)),
        origin=Origin(xyz=(0.0, -side_y, center_z)),
        material=material,
        name=f"{prefix}_rear",
    )


def _add_flat_hole_markers(
    part,
    *,
    x: float,
    y: float,
    z: float,
    radius: float,
    material: str,
    prefix: str,
) -> None:
    for ix, sx in enumerate((-1.0, 1.0)):
        for iy, sy in enumerate((-1.0, 1.0)):
            part.visual(
                Cylinder(radius=radius, length=0.003),
                origin=Origin(xyz=(sx * x, sy * y, z)),
                material=material,
                name=f"{prefix}_{ix}_{iy}",
            )


def _add_square_guide_pads(
    part,
    *,
    prefix: str,
    inner_half: float,
    outer_contact_half: float,
    pad_length: float,
    center_z: float,
    material: str,
) -> None:
    """Wear pads that make the telescoping tube visibly and physically guided."""

    thickness = outer_contact_half - (inner_half - 0.0005)
    center = (outer_contact_half + inner_half - 0.0005) / 2.0
    part.visual(
        Box((0.050, thickness, pad_length)),
        origin=Origin(xyz=(0.0, center, center_z)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((0.050, thickness, pad_length)),
        origin=Origin(xyz=(0.0, -center, center_z)),
        material=material,
        name=f"{prefix}_rear",
    )
    part.visual(
        Box((thickness, 0.050, pad_length)),
        origin=Origin(xyz=(center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_side_0",
    )
    part.visual(
        Box((thickness, 0.050, pad_length)),
        origin=Origin(xyz=(-center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_side_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_survey_mast")

    model.material("powder_black", rgba=(0.015, 0.017, 0.018, 1.0))
    model.material("cast_iron", rgba=(0.07, 0.075, 0.078, 1.0))
    model.material("rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    model.material("survey_yellow", rgba=(0.95, 0.63, 0.10, 1.0))
    model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.68, 1.0))
    model.material("dark_hardware", rgba=(0.025, 0.028, 0.030, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.68, 0.48, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="cast_iron",
        name="foot_plate",
    )
    base.visual(
        Box((0.40, 0.20, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material="cast_iron",
        name="center_weight",
    )
    base.visual(
        Box((0.18, 0.18, 0.085)),
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
        material="dark_hardware",
        name="mast_socket",
    )
    for ix, x in enumerate((-0.285, 0.285)):
        for iy, y in enumerate((-0.185, 0.185)):
            base.visual(
                Cylinder(radius=0.045, length=0.018),
                origin=Origin(xyz=(x, y, 0.009)),
                material="rubber",
                name=f"rubber_foot_{ix}_{iy}",
            )

    _square_tube_visuals(
        base,
        prefix="lower_tube",
        outer=0.120,
        inner=0.096,
        length=0.920,
        center_z=0.560,
        material="powder_black",
    )
    _square_tube_visuals(
        base,
        prefix="lower_collar",
        outer=0.170,
        inner=0.094,
        length=0.090,
        center_z=0.995,
        material="dark_hardware",
    )
    base.visual(
        Box((0.060, 0.046, 0.060)),
        origin=Origin(xyz=(0.115, 0.0, 1.000)),
        material="dark_hardware",
        name="lower_clamp_lug",
    )

    middle = model.part("middle_section")
    _square_tube_visuals(
        middle,
        prefix="middle_tube",
        outer=0.085,
        inner=0.067,
        length=1.100,
        center_z=-0.270,
        material="survey_yellow",
    )
    _square_tube_visuals(
        middle,
        prefix="middle_collar",
        outer=0.125,
        inner=0.066,
        length=0.080,
        center_z=0.280,
        material="dark_hardware",
    )
    middle.visual(
        Box((0.056, 0.040, 0.054)),
        origin=Origin(xyz=(0.090, 0.0, 0.280)),
        material="dark_hardware",
        name="middle_clamp_lug",
    )
    _add_square_guide_pads(
        middle,
        prefix="middle_guide",
        inner_half=0.0425,
        outer_contact_half=0.0480,
        pad_length=0.110,
        center_z=-0.700,
        material="dark_hardware",
    )

    upper = model.part("upper_section")
    _square_tube_visuals(
        upper,
        prefix="upper_tube",
        outer=0.058,
        inner=0.044,
        length=0.950,
        center_z=-0.245,
        material="brushed_aluminum",
    )
    upper.visual(
        Box((0.070, 0.070, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material="brushed_aluminum",
        name="top_cap_ring",
    )
    _add_square_guide_pads(
        upper,
        prefix="upper_guide",
        inner_half=0.0290,
        outer_contact_half=0.0335,
        pad_length=0.100,
        center_z=-0.600,
        material="dark_hardware",
    )

    head = model.part("head_plate")
    head.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="dark_hardware",
        name="bearing_disk",
    )
    head.visual(
        Box((0.280, 0.200, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material="brushed_aluminum",
        name="pan_plate",
    )
    head.visual(
        Box((0.160, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material="dark_hardware",
        name="instrument_slot",
    )
    _add_flat_hole_markers(
        head,
        x=0.105,
        y=0.065,
        z=0.0450,
        radius=0.012,
        material="powder_black",
        prefix="mount_hole",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(0.055, 0.028, body_style="lobed", crown_radius=0.0015),
        "lobed_clamp_knob",
    )
    small_knob_mesh = mesh_from_geometry(
        KnobGeometry(0.048, 0.026, body_style="lobed", crown_radius=0.0012),
        "small_lobed_clamp_knob",
    )

    lower_knob = model.part("lower_clamp_knob")
    lower_knob.visual(
        Cylinder(radius=0.006, length=0.035),
        origin=Origin(xyz=(0.0175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_hardware",
        name="clamp_screw",
    )
    lower_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="powder_black",
        name="knob_cap",
    )

    middle_knob = model.part("middle_clamp_knob")
    middle_knob.visual(
        Cylinder(radius=0.005, length=0.031),
        origin=Origin(xyz=(0.0155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_hardware",
        name="clamp_screw",
    )
    middle_knob.visual(
        small_knob_mesh,
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="powder_black",
        name="knob_cap",
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent=base,
        child=middle,
        origin=Origin(xyz=(0.0, 0.0, 1.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.22, lower=0.0, upper=0.58),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.20, lower=0.0, upper=0.55),
    )
    model.articulation(
        "upper_to_head",
        ArticulationType.CONTINUOUS,
        parent=upper,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5),
    )
    model.articulation(
        "base_to_lower_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=lower_knob,
        origin=Origin(xyz=(0.145, 0.0, 1.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )
    model.articulation(
        "middle_to_middle_knob",
        ArticulationType.CONTINUOUS,
        parent=middle,
        child=middle_knob,
        origin=Origin(xyz=(0.118, 0.0, 0.280)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    middle = object_model.get_part("middle_section")
    upper = object_model.get_part("upper_section")
    head = object_model.get_part("head_plate")
    base_to_middle = object_model.get_articulation("base_to_middle")
    middle_to_upper = object_model.get_articulation("middle_to_upper")
    pan = object_model.get_articulation("upper_to_head")

    ctx.expect_origin_distance(
        base,
        middle,
        axes="xy",
        max_dist=0.002,
        name="middle section is centered in lower sleeve",
    )
    ctx.expect_overlap(
        middle,
        base,
        axes="z",
        elem_a="middle_tube_front",
        elem_b="lower_tube_front",
        min_overlap=0.70,
        name="collapsed middle section stays deeply inserted",
    )
    ctx.expect_overlap(
        upper,
        middle,
        axes="z",
        elem_a="upper_tube_front",
        elem_b="middle_tube_front",
        min_overlap=0.60,
        name="collapsed upper section stays deeply inserted",
    )
    ctx.expect_gap(
        head,
        upper,
        axis="z",
        positive_elem="bearing_disk",
        negative_elem="upper_tube_front",
        max_gap=0.002,
        max_penetration=0.000001,
        name="pan bearing seats on mast top",
    )

    rest_middle = ctx.part_world_position(middle)
    rest_upper = ctx.part_world_position(upper)
    rest_head = ctx.part_world_position(head)
    with ctx.pose({base_to_middle: 0.58, middle_to_upper: 0.55, pan: math.pi / 2.0}):
        ctx.expect_overlap(
            middle,
            base,
            axes="z",
            elem_a="middle_tube_front",
            elem_b="lower_tube_front",
            min_overlap=0.20,
            name="extended middle section remains captured",
        )
        ctx.expect_overlap(
            upper,
            middle,
            axes="z",
            elem_a="upper_tube_front",
            elem_b="middle_tube_front",
            min_overlap=0.12,
            name="extended upper section remains captured",
        )
        extended_middle = ctx.part_world_position(middle)
        extended_upper = ctx.part_world_position(upper)
        extended_head = ctx.part_world_position(head)

    ctx.check(
        "middle section translates upward",
        rest_middle is not None
        and extended_middle is not None
        and extended_middle[2] > rest_middle[2] + 0.55,
        details=f"rest={rest_middle}, extended={extended_middle}",
    )
    ctx.check(
        "upper section translates upward",
        rest_upper is not None
        and extended_upper is not None
        and extended_upper[2] > rest_upper[2] + 1.08,
        details=f"rest={rest_upper}, extended={extended_upper}",
    )
    ctx.check(
        "pan head rides at deployed mast top",
        rest_head is not None
        and extended_head is not None
        and extended_head[2] > rest_head[2] + 1.08,
        details=f"rest={rest_head}, extended={extended_head}",
    )

    return ctx.report()


object_model = build_object_model()
