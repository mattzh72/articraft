from __future__ import annotations

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


def _rect_tube_x(
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    start_x: float,
) -> cq.Workplane:
    """Closed-wall rectangular tube, open along local X."""
    inner_y = outer_y - 2.0 * wall
    inner_z = outer_z - 2.0 * wall
    outer = cq.Workplane("XY").box(length, outer_y, outer_z)
    cutter = cq.Workplane("XY").box(length + 0.010, inner_y, inner_z)
    return outer.cut(cutter).translate((start_x + 0.5 * length, 0.0, 0.0))


def _add_guide_pads(
    part,
    *,
    prefix: str,
    x: float,
    length: float,
    child_half: float,
    parent_inner_half: float,
    pad_material: Material,
) -> None:
    """Four low-friction pads that keep a smaller square tube bearing on its sleeve."""
    thickness = (parent_inner_half - child_half) + 0.0004
    offset = parent_inner_half - 0.5 * thickness
    part.visual(
        Box((length, 0.018, thickness)),
        origin=Origin(xyz=(x, 0.0, offset)),
        material=pad_material,
        name=f"{prefix}_top_pad",
    )
    part.visual(
        Box((length, 0.018, thickness)),
        origin=Origin(xyz=(x, 0.0, -offset)),
        material=pad_material,
        name=f"{prefix}_bottom_pad",
    )
    part.visual(
        Box((length, thickness, 0.018)),
        origin=Origin(xyz=(x, offset, 0.0)),
        material=pad_material,
        name=f"{prefix}_side_pad_0",
    )
    part.visual(
        Box((length, thickness, 0.018)),
        origin=Origin(xyz=(x, -offset, 0.0)),
        material=pad_material,
        name=f"{prefix}_side_pad_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_telescoping_mast_arm")

    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black = Material("black_hardware", rgba=(0.015, 0.016, 0.018, 1.0))
    outer_metal = Material("outer_anodized_aluminum", rgba=(0.33, 0.36, 0.38, 1.0))
    middle_metal = Material("brushed_aluminum", rgba=(0.56, 0.60, 0.62, 1.0))
    inner_metal = Material("light_aluminum", rgba=(0.72, 0.76, 0.76, 1.0))
    tip_metal = Material("pale_aluminum", rgba=(0.83, 0.86, 0.84, 1.0))
    faceplate_orange = Material("safety_orange_faceplate", rgba=(0.95, 0.42, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.220, 0.025)),
        origin=Origin(xyz=(0.030, 0.0, 0.0125)),
        material=dark_steel,
        name="foot_plate",
    )
    base.visual(
        Box((0.145, 0.090, 0.055)),
        origin=Origin(xyz=(0.030, 0.0, 0.0525)),
        material=dark_steel,
        name="saddle",
    )
    base.visual(
        Box((0.145, 0.018, 0.115)),
        origin=Origin(xyz=(0.030, 0.049, 0.0825)),
        material=dark_steel,
        name="side_cheek_0",
    )
    base.visual(
        Box((0.145, 0.018, 0.115)),
        origin=Origin(xyz=(0.030, -0.049, 0.0825)),
        material=dark_steel,
        name="side_cheek_1",
    )
    base.visual(
        Box((0.030, 0.145, 0.018)),
        origin=Origin(xyz=(-0.050, 0.0, 0.034)),
        material=black,
        name="rear_tie_bar",
    )

    outer_section = model.part("outer_section")
    outer_section.visual(
        mesh_from_cadquery(
            _rect_tube_x(length=0.380, outer_y=0.080, outer_z=0.080, wall=0.005, start_x=-0.020),
            "outer_section_tube",
            tolerance=0.0005,
        ),
        material=outer_metal,
        name="outer_tube",
    )
    outer_section.visual(
        mesh_from_cadquery(
            _rect_tube_x(length=0.028, outer_y=0.092, outer_z=0.092, wall=0.011, start_x=0.332),
            "outer_section_lip",
            tolerance=0.0005,
        ),
        material=black,
        name="outer_lip",
    )

    middle_section = model.part("middle_section")
    middle_section.visual(
        mesh_from_cadquery(
            _rect_tube_x(length=0.360, outer_y=0.060, outer_z=0.060, wall=0.004, start_x=-0.280),
            "middle_section_tube",
            tolerance=0.0005,
        ),
        material=middle_metal,
        name="middle_tube",
    )
    middle_section.visual(
        mesh_from_cadquery(
            _rect_tube_x(length=0.022, outer_y=0.068, outer_z=0.068, wall=0.008, start_x=0.058),
            "middle_section_lip",
            tolerance=0.0005,
        ),
        material=black,
        name="middle_lip",
    )
    _add_guide_pads(
        middle_section,
        prefix="middle",
        x=-0.225,
        length=0.060,
        child_half=0.030,
        parent_inner_half=0.035,
        pad_material=black,
    )

    inner_section = model.part("inner_section")
    inner_section.visual(
        mesh_from_cadquery(
            _rect_tube_x(length=0.285, outer_y=0.044, outer_z=0.044, wall=0.0035, start_x=-0.220),
            "inner_section_tube",
            tolerance=0.0005,
        ),
        material=inner_metal,
        name="inner_tube",
    )
    inner_section.visual(
        mesh_from_cadquery(
            _rect_tube_x(length=0.018, outer_y=0.052, outer_z=0.052, wall=0.0075, start_x=0.047),
            "inner_section_lip",
            tolerance=0.0005,
        ),
        material=black,
        name="inner_lip",
    )
    _add_guide_pads(
        inner_section,
        prefix="inner",
        x=-0.175,
        length=0.050,
        child_half=0.022,
        parent_inner_half=0.026,
        pad_material=black,
    )

    tip_section = model.part("tip_section")
    tip_section.visual(
        mesh_from_cadquery(
            _rect_tube_x(length=0.225, outer_y=0.032, outer_z=0.032, wall=0.003, start_x=-0.170),
            "tip_section_tube",
            tolerance=0.0005,
        ),
        material=tip_metal,
        name="tip_tube",
    )
    _add_guide_pads(
        tip_section,
        prefix="tip",
        x=-0.130,
        length=0.040,
        child_half=0.016,
        parent_inner_half=0.0185,
        pad_material=black,
    )
    tip_section.visual(
        Box((0.018, 0.090, 0.080)),
        origin=Origin(xyz=(0.064, 0.0, 0.0)),
        material=faceplate_orange,
        name="faceplate",
    )
    for iy, y in enumerate((-0.027, 0.027)):
        for iz, z in enumerate((-0.022, 0.022)):
            tip_section.visual(
                Cylinder(radius=0.004, length=0.007),
                origin=Origin(xyz=(0.0765, y, z), rpy=(0.0, 1.57079632679, 0.0)),
                material=black,
                name=f"faceplate_bolt_{iy}_{iz}",
            )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base,
        child=outer_section,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_section,
        child=middle_section,
        origin=Origin(xyz=(0.360, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.220, effort=90.0, velocity=0.25),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_section,
        child=inner_section,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.180, effort=70.0, velocity=0.25),
    )
    model.articulation(
        "inner_to_tip",
        ArticulationType.PRISMATIC,
        parent=inner_section,
        child=tip_section,
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.140, effort=45.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    tip = object_model.get_part("tip_section")
    outer_slide = object_model.get_articulation("outer_to_middle")
    middle_slide = object_model.get_articulation("middle_to_inner")
    inner_slide = object_model.get_articulation("inner_to_tip")

    def allow_guide_pad_pair(child, parent, prefix: str, parent_tube: str) -> None:
        for pad in (
            f"{prefix}_top_pad",
            f"{prefix}_bottom_pad",
            f"{prefix}_side_pad_0",
            f"{prefix}_side_pad_1",
        ):
            ctx.allow_overlap(
                child,
                parent,
                elem_a=pad,
                elem_b=parent_tube,
                reason=(
                    "The hidden low-friction guide pad is modeled with slight compression "
                    "against the sleeve wall so the prismatic box section is physically supported."
                ),
            )
            ctx.expect_contact(
                child,
                parent,
                elem_a=pad,
                elem_b=parent_tube,
                contact_tol=0.0005,
                name=f"{pad} bears on {parent_tube}",
            )

    allow_guide_pad_pair(middle, outer, "middle", "outer_tube")
    allow_guide_pad_pair(inner, middle, "inner", "middle_tube")
    allow_guide_pad_pair(tip, inner, "tip", "inner_tube")

    ctx.expect_gap(
        outer,
        base,
        axis="z",
        positive_elem="outer_tube",
        negative_elem="saddle",
        max_gap=0.001,
        max_penetration=0.000001,
        name="outer section rests on base saddle",
    )

    for joint in (outer_slide, middle_slide, inner_slide):
        ctx.check(
            f"{joint.name} slides along mast axis",
            tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.expect_within(middle, outer, axes="yz", inner_elem="middle_tube", outer_elem="outer_tube", margin=0.0)
    ctx.expect_within(inner, middle, axes="yz", inner_elem="inner_tube", outer_elem="middle_tube", margin=0.0)
    ctx.expect_within(tip, inner, axes="yz", inner_elem="tip_tube", outer_elem="inner_tube", margin=0.0)
    ctx.expect_overlap(middle, outer, axes="x", elem_a="middle_tube", elem_b="outer_tube", min_overlap=0.20)
    ctx.expect_overlap(inner, middle, axes="x", elem_a="inner_tube", elem_b="middle_tube", min_overlap=0.16)
    ctx.expect_overlap(tip, inner, axes="x", elem_a="tip_tube", elem_b="inner_tube", min_overlap=0.12)

    rest_tip = ctx.part_world_position(tip)
    with ctx.pose({outer_slide: 0.220, middle_slide: 0.180, inner_slide: 0.140}):
        ctx.expect_within(middle, outer, axes="yz", inner_elem="middle_tube", outer_elem="outer_tube", margin=0.0)
        ctx.expect_within(inner, middle, axes="yz", inner_elem="inner_tube", outer_elem="middle_tube", margin=0.0)
        ctx.expect_within(tip, inner, axes="yz", inner_elem="tip_tube", outer_elem="inner_tube", margin=0.0)
        ctx.expect_overlap(middle, outer, axes="x", elem_a="middle_tube", elem_b="outer_tube", min_overlap=0.055)
        ctx.expect_overlap(inner, middle, axes="x", elem_a="inner_tube", elem_b="middle_tube", min_overlap=0.035)
        ctx.expect_overlap(tip, inner, axes="x", elem_a="tip_tube", elem_b="inner_tube", min_overlap=0.025)
        extended_tip = ctx.part_world_position(tip)

    ctx.check(
        "tip extends forward",
        rest_tip is not None and extended_tip is not None and extended_tip[0] > rest_tip[0] + 0.50,
        details=f"rest_tip={rest_tip}, extended_tip={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
