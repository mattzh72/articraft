from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_rect_tube(
    part,
    prefix: str,
    outer_xy: tuple[float, float],
    inner_xy: tuple[float, float],
    height: float,
    *,
    center_z: float,
    material,
):
    """Four clean rigid wall plates that read as an open rectangular mast tube."""
    outer_x, outer_y = outer_xy
    inner_x, inner_y = inner_xy
    wall_x = (outer_x - inner_x) * 0.5
    wall_y = (outer_y - inner_y) * 0.5
    x_center = inner_x * 0.5 + wall_x * 0.5
    y_center = inner_y * 0.5 + wall_y * 0.5
    part.visual(
        Box((wall_x, outer_y, height)),
        origin=Origin(xyz=(x_center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_wall_pos_x",
    )
    part.visual(
        Box((wall_x, outer_y, height)),
        origin=Origin(xyz=(-x_center, 0.0, center_z)),
        material=material,
        name=f"{prefix}_wall_neg_x",
    )
    part.visual(
        Box((outer_x, wall_y, height)),
        origin=Origin(xyz=(0.0, y_center, center_z)),
        material=material,
        name=f"{prefix}_wall_pos_y",
    )
    part.visual(
        Box((outer_x, wall_y, height)),
        origin=Origin(xyz=(0.0, -y_center, center_z)),
        material=material,
        name=f"{prefix}_wall_neg_y",
    )


def _add_guide_pads(
    part,
    prefix: str,
    outer_xy: tuple[float, float],
    guide_inner_xy: tuple[float, float],
    *,
    center_z: float,
    height: float,
    material,
):
    """Small sacrificial slide shoes that touch the surrounding guide collar."""
    outer_x, outer_y = outer_xy
    guide_x, guide_y = guide_inner_xy
    pad_x = (guide_x - outer_x) * 0.5
    pad_y = (guide_y - outer_y) * 0.5
    if pad_x > 0:
        x_center = outer_x * 0.5 + pad_x * 0.5
        part.visual(
            Box((pad_x, min(outer_y * 0.45, 0.034), height)),
            origin=Origin(xyz=(x_center, 0.0, center_z)),
            material=material,
            name=f"{prefix}_guide_pad_pos_x",
        )
        part.visual(
            Box((pad_x, min(outer_y * 0.45, 0.034), height)),
            origin=Origin(xyz=(-x_center, 0.0, center_z)),
            material=material,
            name=f"{prefix}_guide_pad_neg_x",
        )
    if pad_y > 0:
        y_center = outer_y * 0.5 + pad_y * 0.5
        part.visual(
            Box((min(outer_x * 0.45, 0.042), pad_y, height)),
            origin=Origin(xyz=(0.0, y_center, center_z)),
            material=material,
            name=f"{prefix}_guide_pad_pos_y",
        )
        part.visual(
            Box((min(outer_x * 0.45, 0.042), pad_y, height)),
            origin=Origin(xyz=(0.0, -y_center, center_z)),
            material=material,
            name=f"{prefix}_guide_pad_neg_y",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_antenna_mast")

    dark_paint = model.material("charcoal_powdercoat", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    bolt_steel = model.material("dark_bolt_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    sheet_metal = model.material("light_sheet_metal", rgba=(0.78, 0.80, 0.78, 1.0))
    aluminum = model.material("satin_painted_aluminum", rgba=(0.62, 0.67, 0.70, 1.0))
    worn_track = model.material("dark_wear_tracks", rgba=(0.12, 0.13, 0.14, 1.0))
    radome = model.material("matte_black_radome", rgba=(0.02, 0.025, 0.03, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.42, 0.34, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_paint,
        name="base_plate",
    )
    for i, (x, y) in enumerate(((-0.16, -0.12), (-0.16, 0.12), (0.16, -0.12), (0.16, 0.12))):
        base.visual(
            Box((0.055, 0.045, 0.018)),
            origin=Origin(xyz=(x, y, -0.009)),
            material=black_rubber,
            name=f"rubber_foot_{i}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(x, y, 0.051)),
            material=bolt_steel,
            name=f"hold_down_bolt_{i}",
        )
    _add_rect_tube(
        base,
        "base_socket",
        outer_xy=(0.180, 0.140),
        inner_xy=(0.130, 0.090),
        height=0.480,
        center_z=0.285,
        material=dark_paint,
    )
    _add_rect_tube(
        base,
        "base_collar",
        outer_xy=(0.220, 0.180),
        inner_xy=(0.128, 0.088),
        height=0.060,
        center_z=0.555,
        material=sheet_metal,
    )

    stage_0 = model.part("stage_0")
    _add_rect_tube(
        stage_0,
        "stage_0_tube",
        outer_xy=(0.112, 0.074),
        inner_xy=(0.086, 0.054),
        height=0.920,
        center_z=-0.060,
        material=aluminum,
    )
    _add_guide_pads(
        stage_0,
        "stage_0",
        outer_xy=(0.112, 0.074),
        guide_inner_xy=(0.128, 0.088),
        center_z=-0.030,
        height=0.050,
        material=black_rubber,
    )
    for y in (-0.038, 0.038):
        stage_0.visual(
            Box((0.066, 0.003, 0.500)),
            origin=Origin(xyz=(0.0, y, 0.120)),
            material=worn_track,
            name=f"stage_0_wear_strip_{'front' if y > 0 else 'rear'}",
        )
    _add_rect_tube(
        stage_0,
        "stage_0_collar",
        outer_xy=(0.130, 0.095),
        inner_xy=(0.083, 0.053),
        height=0.055,
        center_z=0.3925,
        material=sheet_metal,
    )
    for x in (-0.042, 0.042):
        stage_0.visual(
            Box((0.020, 0.012, 0.018)),
            origin=Origin(xyz=(x, 0.053, 0.3925)),
            material=bolt_steel,
            name=f"stage_0_clamp_{0 if x < 0 else 1}",
        )

    stage_1 = model.part("stage_1")
    _add_rect_tube(
        stage_1,
        "stage_1_tube",
        outer_xy=(0.074, 0.044),
        inner_xy=(0.054, 0.030),
        height=0.760,
        center_z=-0.040,
        material=aluminum,
    )
    _add_guide_pads(
        stage_1,
        "stage_1",
        outer_xy=(0.074, 0.044),
        guide_inner_xy=(0.083, 0.053),
        center_z=-0.0125,
        height=0.044,
        material=black_rubber,
    )
    for y in (-0.02275, 0.02275):
        stage_1.visual(
            Box((0.043, 0.0025, 0.410)),
            origin=Origin(xyz=(0.0, y, 0.080)),
            material=worn_track,
            name=f"stage_1_wear_strip_{'front' if y > 0 else 'rear'}",
        )
    _add_rect_tube(
        stage_1,
        "stage_1_collar",
        outer_xy=(0.090, 0.065),
        inner_xy=(0.052, 0.032),
        height=0.050,
        center_z=0.327,
        material=sheet_metal,
    )
    for x in (-0.027, 0.027):
        stage_1.visual(
            Box((0.015, 0.010, 0.015)),
            origin=Origin(xyz=(x, 0.037, 0.327)),
            material=bolt_steel,
            name=f"stage_1_clamp_{0 if x < 0 else 1}",
        )

    stage_2 = model.part("stage_2")
    _add_rect_tube(
        stage_2,
        "stage_2_tube",
        outer_xy=(0.044, 0.024),
        inner_xy=(0.026, 0.010),
        height=0.580,
        center_z=-0.020,
        material=aluminum,
    )
    _add_guide_pads(
        stage_2,
        "stage_2",
        outer_xy=(0.044, 0.024),
        guide_inner_xy=(0.052, 0.032),
        center_z=0.002,
        height=0.038,
        material=black_rubber,
    )
    for y in (-0.01275, 0.01275):
        stage_2.visual(
            Box((0.026, 0.002, 0.320)),
            origin=Origin(xyz=(0.0, y, 0.050)),
            material=worn_track,
            name=f"stage_2_wear_strip_{'front' if y > 0 else 'rear'}",
        )
    stage_2.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=sheet_metal,
        name="antenna_stem",
    )
    stage_2.visual(
        Box((0.105, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.3625)),
        material=radome,
        name="antenna_head",
    )

    model.articulation(
        "base_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_0,
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.42),
    )
    model.articulation(
        "stage_0_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=stage_0,
        child=stage_1,
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.16, lower=0.0, upper=0.34),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.14, lower=0.0, upper=0.26),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    stage_0 = object_model.get_part("stage_0")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    j0 = object_model.get_articulation("base_to_stage_0")
    j1 = object_model.get_articulation("stage_0_to_stage_1")
    j2 = object_model.get_articulation("stage_1_to_stage_2")

    def _expect_centered_in_rect_guide(inner, outer, inner_prefix: str, guide_prefix: str, label: str) -> None:
        ctx.expect_gap(
            outer,
            inner,
            axis="x",
            positive_elem=f"{guide_prefix}_wall_pos_x",
            negative_elem=f"{inner_prefix}_wall_pos_x",
            min_gap=0.001,
            max_gap=0.020,
            name=f"{label} has positive-x guide clearance",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="x",
            positive_elem=f"{inner_prefix}_wall_neg_x",
            negative_elem=f"{guide_prefix}_wall_neg_x",
            min_gap=0.001,
            max_gap=0.020,
            name=f"{label} has negative-x guide clearance",
        )
        ctx.expect_gap(
            outer,
            inner,
            axis="y",
            positive_elem=f"{guide_prefix}_wall_pos_y",
            negative_elem=f"{inner_prefix}_wall_pos_y",
            min_gap=0.001,
            max_gap=0.020,
            name=f"{label} has positive-y guide clearance",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="y",
            positive_elem=f"{inner_prefix}_wall_neg_y",
            negative_elem=f"{guide_prefix}_wall_neg_y",
            min_gap=0.001,
            max_gap=0.020,
            name=f"{label} has negative-y guide clearance",
        )

    _expect_centered_in_rect_guide(
        stage_0,
        base,
        "stage_0_tube",
        "base_collar",
        "largest sliding section",
    )
    ctx.expect_overlap(
        stage_0,
        base,
        axes="z",
        min_overlap=0.35,
        name="largest sliding section has deep retained insertion when collapsed",
    )
    _expect_centered_in_rect_guide(
        stage_1,
        stage_0,
        "stage_1_tube",
        "stage_0_collar",
        "middle sliding section",
    )
    ctx.expect_overlap(
        stage_1,
        stage_0,
        axes="z",
        min_overlap=0.30,
        name="middle sliding section remains nested when collapsed",
    )
    _expect_centered_in_rect_guide(
        stage_2,
        stage_1,
        "stage_2_tube",
        "stage_1_collar",
        "small sliding section",
    )
    ctx.expect_overlap(
        stage_2,
        stage_1,
        axes="z",
        min_overlap=0.25,
        name="small sliding section remains nested when collapsed",
    )

    rest_head_aabb = ctx.part_element_world_aabb(stage_2, elem="antenna_head")
    rest_stage_pos = ctx.part_world_position(stage_2)
    with ctx.pose({j0: 0.42, j1: 0.34, j2: 0.26}):
        ctx.expect_overlap(
            stage_0,
            base,
            axes="z",
            min_overlap=0.030,
            name="largest section keeps insertion at full extension",
        )
        ctx.expect_overlap(
            stage_1,
            stage_0,
            axes="z",
            min_overlap=0.055,
            name="middle section keeps insertion at full extension",
        )
        ctx.expect_overlap(
            stage_2,
            stage_1,
            axes="z",
            min_overlap=0.045,
            name="small section keeps insertion at full extension",
        )
        extended_head_aabb = ctx.part_element_world_aabb(stage_2, elem="antenna_head")
        extended_stage_pos = ctx.part_world_position(stage_2)

    ctx.check(
        "antenna head rises along the mast axis",
        rest_head_aabb is not None
        and extended_head_aabb is not None
        and extended_head_aabb[1][2] > rest_head_aabb[1][2] + 0.95,
        details=f"rest={rest_head_aabb}, extended={extended_head_aabb}",
    )
    ctx.check(
        "nested prismatic joints move the top stage upward",
        rest_stage_pos is not None
        and extended_stage_pos is not None
        and extended_stage_pos[2] > rest_stage_pos[2] + 0.95,
        details=f"rest={rest_stage_pos}, extended={extended_stage_pos}",
    )

    return ctx.report()


object_model = build_object_model()
