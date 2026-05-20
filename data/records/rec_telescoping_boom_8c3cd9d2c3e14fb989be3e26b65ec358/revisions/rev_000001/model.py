from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _rectangular_tube(length: float, outer_y: float, outer_z: float, wall: float, *, band_len: float, band_extra: float) -> cq.Workplane:
    """A real open-ended rectangular steel tube with a thicker front guide collar."""
    main = cq.Workplane("XY").box(length, outer_y, outer_z)
    collar = (
        cq.Workplane("XY")
        .box(band_len, outer_y + 2.0 * band_extra, outer_z + 2.0 * band_extra)
        .translate((length / 2.0 - band_len / 2.0, 0.0, 0.0))
    )
    inner_cut = cq.Workplane("XY").box(length + 0.04, outer_y - 2.0 * wall, outer_z - 2.0 * wall)
    return main.union(collar).cut(inner_cut)


def _add_guide_pads(part, *, prefix: str, x_front: float, outer_y: float, outer_z: float, pad_len: float, material: Material) -> None:
    """Black replaceable wear pads bolted around the front mouth of a sliding tube."""
    x = x_front - pad_len / 2.0
    pad_t = 0.006
    pad_w = 0.030
    part.visual(
        Box((pad_len, outer_y + 0.026, pad_t)),
        origin=Origin(xyz=(x, 0.0, outer_z / 2.0 + pad_t / 2.0)),
        material=material,
        name=f"{prefix}_top_pad",
    )
    part.visual(
        Box((pad_len, outer_y + 0.026, pad_t)),
        origin=Origin(xyz=(x, 0.0, -outer_z / 2.0 - pad_t / 2.0)),
        material=material,
        name=f"{prefix}_bottom_pad",
    )
    for idx, y in enumerate((-outer_y / 2.0 - pad_t / 2.0, outer_y / 2.0 + pad_t / 2.0)):
        part.visual(
            Box((pad_len, pad_t, pad_w)),
            origin=Origin(xyz=(x, y, 0.0)),
            material=material,
            name=f"{prefix}_side_pad_{idx}",
        )


def _add_bearing_shoes(
    part,
    *,
    prefix: str,
    x_center: float,
    section_y: float,
    section_z: float,
    target_inner_y: float,
    target_inner_z: float,
    pad_len: float,
    material: Material,
) -> None:
    """Low-friction shoes that touch the inside faces of the enclosing tube."""
    z_pad_t = (target_inner_z - section_z) / 2.0
    y_pad_t = (target_inner_y - section_y) / 2.0
    pad_y = section_y * 0.42
    pad_z = section_z * 0.45

    part.visual(
        Box((pad_len, pad_y, z_pad_t)),
        origin=Origin(xyz=(x_center, 0.0, section_z / 2.0 + z_pad_t / 2.0)),
        material=material,
        name=f"{prefix}_top_shoe",
    )
    part.visual(
        Box((pad_len, pad_y, z_pad_t)),
        origin=Origin(xyz=(x_center, 0.0, -section_z / 2.0 - z_pad_t / 2.0)),
        material=material,
        name=f"{prefix}_bottom_shoe",
    )
    for idx, y in enumerate((-section_y / 2.0 - y_pad_t / 2.0, section_y / 2.0 + y_pad_t / 2.0)):
        part.visual(
            Box((pad_len, y_pad_t, pad_z)),
            origin=Origin(xyz=(x_center, y, 0.0)),
            material=material,
            name=f"{prefix}_side_shoe_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_rectangular_boom")

    mount_mat = model.material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    yellow_mat = model.material("safety_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    mid_mat = model.material("worn_yellow_steel", rgba=(0.88, 0.60, 0.10, 1.0))
    inner_mat = model.material("bright_inner_section", rgba=(1.0, 0.78, 0.12, 1.0))
    wear_mat = model.material("black_polymer_wear_pads", rgba=(0.01, 0.012, 0.014, 1.0))

    boom_z = 0.85

    root = model.part("root_mount")
    root.visual(
        Box((0.82, 0.48, 0.060)),
        origin=Origin(xyz=(0.16, 0.0, 0.030)),
        material=mount_mat,
        name="base_plate",
    )
    root.visual(
        Box((0.18, 0.20, 0.72)),
        origin=Origin(xyz=(0.14, 0.0, 0.390)),
        material=mount_mat,
        name="pedestal",
    )
    root.visual(
        Box((0.50, 0.28, 0.055)),
        origin=Origin(xyz=(0.24, 0.0, 0.7225)),
        material=mount_mat,
        name="saddle_plate",
    )
    for idx, y in enumerate((-0.145, 0.145)):
        root.visual(
            Box((0.48, 0.040, 0.060)),
            origin=Origin(xyz=(0.31, y, 0.625), rpy=(0.0, -0.48, 0.0)),
            material=mount_mat,
            name=f"diagonal_brace_{idx}",
        )

    socket_len = 0.42
    socket_x = 0.23
    socket_inner_y = 0.205
    socket_inner_z = 0.165
    socket_wall = 0.020
    socket_outer_y = socket_inner_y + 2.0 * socket_wall
    root.visual(
        Box((socket_len, socket_outer_y, socket_wall)),
        origin=Origin(xyz=(socket_x, 0.0, boom_z + socket_inner_z / 2.0 + socket_wall / 2.0)),
        material=mount_mat,
        name="socket_top",
    )
    root.visual(
        Box((socket_len, socket_outer_y, socket_wall)),
        origin=Origin(xyz=(socket_x, 0.0, boom_z - socket_inner_z / 2.0 - socket_wall / 2.0)),
        material=mount_mat,
        name="socket_bottom",
    )
    for idx, y in enumerate((-socket_inner_y / 2.0 - socket_wall / 2.0, socket_inner_y / 2.0 + socket_wall / 2.0)):
        root.visual(
            Box((socket_len, socket_wall, socket_inner_z)),
            origin=Origin(xyz=(socket_x, y, boom_z)),
            material=mount_mat,
            name=f"socket_side_{idx}",
        )

    outer = model.part("outer_section")
    outer_min_x, outer_max_x = -0.45, 1.00
    outer_len = outer_max_x - outer_min_x
    outer_center_x = (outer_min_x + outer_max_x) / 2.0
    outer_y, outer_z, outer_wall = 0.180, 0.140, 0.008
    outer.visual(
        mesh_from_cadquery(
            _rectangular_tube(outer_len, outer_y, outer_z, outer_wall, band_len=0.12, band_extra=0.010),
            "outer_section_tube",
            tolerance=0.001,
        ),
        origin=Origin(xyz=(outer_center_x, 0.0, 0.0)),
        material=yellow_mat,
        name="outer_tube",
    )
    _add_bearing_shoes(
        outer,
        prefix="outer_rear",
        x_center=-0.20,
        section_y=outer_y,
        section_z=outer_z,
        target_inner_y=socket_inner_y,
        target_inner_z=socket_inner_z,
        pad_len=0.140,
        material=wear_mat,
    )
    _add_guide_pads(outer, prefix="outer", x_front=outer_max_x + 0.010, outer_y=outer_y + 0.020, outer_z=outer_z + 0.020, pad_len=0.085, material=wear_mat)

    middle = model.part("middle_section")
    middle_min_x, middle_max_x = -0.90, 0.45
    middle_len = middle_max_x - middle_min_x
    middle_center_x = (middle_min_x + middle_max_x) / 2.0
    middle_y, middle_z, middle_wall = 0.150, 0.110, 0.007
    middle.visual(
        mesh_from_cadquery(
            _rectangular_tube(middle_len, middle_y, middle_z, middle_wall, band_len=0.11, band_extra=0.008),
            "middle_section_tube",
            tolerance=0.001,
        ),
        origin=Origin(xyz=(middle_center_x, 0.0, 0.0)),
        material=mid_mat,
        name="middle_tube",
    )
    mid_shoe_len = 0.140
    mid_shoe_x = -0.72
    mid_shoe_z_t = (outer_z - 2.0 * outer_wall - middle_z) / 2.0
    mid_shoe_y_t = (outer_y - 2.0 * outer_wall - middle_y) / 2.0
    middle.visual(
        Box((mid_shoe_len, middle_y * 0.42, mid_shoe_z_t)),
        origin=Origin(xyz=(mid_shoe_x, 0.0, middle_z / 2.0 + mid_shoe_z_t / 2.0)),
        material=wear_mat,
        name="middle_rear_top_shoe",
    )
    middle.visual(
        Box((mid_shoe_len, middle_y * 0.42, mid_shoe_z_t)),
        origin=Origin(xyz=(mid_shoe_x, 0.0, -middle_z / 2.0 - mid_shoe_z_t / 2.0)),
        material=wear_mat,
        name="middle_rear_bottom_shoe",
    )
    middle.visual(
        Box((mid_shoe_len, mid_shoe_y_t, middle_z * 0.45)),
        origin=Origin(xyz=(mid_shoe_x, -middle_y / 2.0 - mid_shoe_y_t / 2.0, 0.0)),
        material=wear_mat,
        name="middle_rear_side_shoe_0",
    )
    middle.visual(
        Box((mid_shoe_len, mid_shoe_y_t, middle_z * 0.45)),
        origin=Origin(xyz=(mid_shoe_x, middle_y / 2.0 + mid_shoe_y_t / 2.0, 0.0)),
        material=wear_mat,
        name="middle_rear_side_shoe_1",
    )
    _add_guide_pads(middle, prefix="middle", x_front=middle_max_x + 0.008, outer_y=middle_y + 0.016, outer_z=middle_z + 0.016, pad_len=0.075, material=wear_mat)

    inner = model.part("inner_section")
    inner_min_x, inner_max_x = -0.75, 0.45
    inner_len = inner_max_x - inner_min_x
    inner_center_x = (inner_min_x + inner_max_x) / 2.0
    inner_y, inner_z, inner_wall = 0.120, 0.080, 0.006
    inner.visual(
        mesh_from_cadquery(
            _rectangular_tube(inner_len, inner_y, inner_z, inner_wall, band_len=0.10, band_extra=0.006),
            "inner_section_tube",
            tolerance=0.001,
        ),
        origin=Origin(xyz=(inner_center_x, 0.0, 0.0)),
        material=inner_mat,
        name="inner_tube",
    )
    inner.visual(
        Box((0.050, inner_y + 0.030, inner_z + 0.030)),
        origin=Origin(xyz=(inner_max_x + 0.025, 0.0, 0.0)),
        material=wear_mat,
        name="end_cap",
    )
    inner_shoe_len = 0.120
    inner_shoe_x = -0.62
    inner_shoe_z_t = (middle_z - 2.0 * middle_wall - inner_z) / 2.0
    inner_shoe_y_t = (middle_y - 2.0 * middle_wall - inner_y) / 2.0
    inner.visual(
        Box((inner_shoe_len, inner_y * 0.42, inner_shoe_z_t)),
        origin=Origin(xyz=(inner_shoe_x, 0.0, inner_z / 2.0 + inner_shoe_z_t / 2.0)),
        material=wear_mat,
        name="inner_rear_top_shoe",
    )
    inner.visual(
        Box((inner_shoe_len, inner_y * 0.42, inner_shoe_z_t)),
        origin=Origin(xyz=(inner_shoe_x, 0.0, -inner_z / 2.0 - inner_shoe_z_t / 2.0)),
        material=wear_mat,
        name="inner_rear_bottom_shoe",
    )
    inner.visual(
        Box((inner_shoe_len, inner_shoe_y_t, inner_z * 0.45)),
        origin=Origin(xyz=(inner_shoe_x, -inner_y / 2.0 - inner_shoe_y_t / 2.0, 0.0)),
        material=wear_mat,
        name="inner_rear_side_shoe_0",
    )
    inner.visual(
        Box((inner_shoe_len, inner_shoe_y_t, inner_z * 0.45)),
        origin=Origin(xyz=(inner_shoe_x, inner_y / 2.0 + inner_shoe_y_t / 2.0, 0.0)),
        material=wear_mat,
        name="inner_rear_side_shoe_1",
    )

    model.articulation(
        "root_to_outer",
        ArticulationType.PRISMATIC,
        parent=root,
        child=outer,
        origin=Origin(xyz=(0.36, 0.0, boom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.18, lower=0.0, upper=0.35),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.88, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2600.0, velocity=0.20, lower=0.0, upper=0.45),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.22, lower=0.0, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_mount")
    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_section")
    inner = object_model.get_part("inner_section")
    root_slide = object_model.get_articulation("root_to_outer")
    middle_slide = object_model.get_articulation("outer_to_middle")
    inner_slide = object_model.get_articulation("middle_to_inner")

    ctx.allow_overlap(
        middle,
        outer,
        elem_a="middle_rear_top_shoe",
        elem_b="outer_tube",
        reason="The rear polymer top shoe is intentionally preloaded against the outer tube's inside guide face to support the middle section.",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="xy",
        elem_a="middle_rear_top_shoe",
        elem_b="outer_tube",
        min_overlap=0.05,
        name="middle rear top shoe is seated in the outer guide",
    )
    ctx.allow_overlap(
        middle,
        outer,
        elem_a="middle_rear_bottom_shoe",
        elem_b="outer_tube",
        reason="The rear polymer bottom shoe is intentionally preloaded against the outer tube's inside guide face to support the middle section.",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="xy",
        elem_a="middle_rear_bottom_shoe",
        elem_b="outer_tube",
        min_overlap=0.05,
        name="middle rear bottom shoe is seated in the outer guide",
    )
    ctx.allow_overlap(
        middle,
        outer,
        elem_a="middle_rear_side_shoe_0",
        elem_b="outer_tube",
        reason="The rear polymer side shoe is intentionally preloaded against the outer tube's inside guide face to remove sliding play.",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="xz",
        elem_a="middle_rear_side_shoe_0",
        elem_b="outer_tube",
        min_overlap=0.04,
        name="middle rear side shoe is seated in the outer guide",
    )
    ctx.allow_overlap(
        middle,
        outer,
        elem_a="middle_rear_side_shoe_1",
        elem_b="outer_tube",
        reason="The opposite rear polymer side shoe is intentionally preloaded against the outer tube's inside guide face to keep the middle section centered.",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="xz",
        elem_a="middle_rear_side_shoe_1",
        elem_b="outer_tube",
        min_overlap=0.04,
        name="opposite middle rear side shoe is seated in the outer guide",
    )
    ctx.allow_overlap(
        inner,
        middle,
        elem_a="inner_rear_side_shoe_0",
        elem_b="middle_tube",
        reason="The rear polymer side shoe is intentionally preloaded against the middle tube's inside guide face to remove sliding play.",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="xz",
        elem_a="inner_rear_side_shoe_0",
        elem_b="middle_tube",
        min_overlap=0.03,
        name="inner rear side shoe is seated in the middle guide",
    )
    ctx.allow_overlap(
        inner,
        middle,
        elem_a="inner_rear_side_shoe_1",
        elem_b="middle_tube",
        reason="The opposite rear polymer side shoe is intentionally preloaded against the middle tube's inside guide face to keep the inner section centered.",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="xz",
        elem_a="inner_rear_side_shoe_1",
        elem_b="middle_tube",
        min_overlap=0.03,
        name="opposite inner rear side shoe is seated in the middle guide",
    )
    ctx.allow_overlap(
        inner,
        middle,
        elem_a="inner_rear_top_shoe",
        elem_b="middle_tube",
        reason="The rear polymer top shoe is intentionally preloaded against the middle tube's inside guide face to support the inner slider.",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="xy",
        elem_a="inner_rear_top_shoe",
        elem_b="middle_tube",
        min_overlap=0.05,
        name="inner rear top shoe is seated in the middle guide",
    )
    ctx.allow_overlap(
        inner,
        middle,
        elem_a="inner_rear_bottom_shoe",
        elem_b="middle_tube",
        reason="The opposite rear polymer top-bottom shoe is intentionally preloaded against the middle tube's inside guide face to keep the inner section centered.",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="xy",
        elem_a="inner_rear_bottom_shoe",
        elem_b="middle_tube",
        min_overlap=0.05,
        name="inner rear bottom shoe is seated in the middle guide",
    )

    for joint in (root_slide, middle_slide, inner_slide):
        ctx.check(
            f"{joint.name} is a prismatic boom stage",
            joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_overlap(
        outer,
        root,
        axes="x",
        elem_a="outer_tube",
        elem_b="socket_top",
        min_overlap=0.25,
        name="outer section is captured by root socket when retracted",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_tube",
        elem_b="outer_tube",
        min_overlap=0.45,
        name="middle section is captured by outer tube when retracted",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_tube",
        elem_b="middle_tube",
        min_overlap=0.45,
        name="inner section is captured by middle tube when retracted",
    )

    rest_outer = ctx.part_world_position(outer)
    with ctx.pose({root_slide: 0.35, middle_slide: 0.45, inner_slide: 0.40}):
        extended_outer = ctx.part_world_position(outer)
        ctx.expect_overlap(
            outer,
            root,
            axes="x",
            elem_a="outer_tube",
            elem_b="socket_top",
            min_overlap=0.16,
            name="extended outer section retains root insertion",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_tube",
            elem_b="outer_tube",
            min_overlap=0.30,
            name="extended middle section remains inside outer tube",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_tube",
            elem_b="middle_tube",
            min_overlap=0.30,
            name="extended inner section remains inside middle tube",
        )

    ctx.check(
        "root slide extends along the boom axis",
        rest_outer is not None and extended_outer is not None and extended_outer[0] > rest_outer[0] + 0.30,
        details=f"rest={rest_outer}, extended={extended_outer}",
    )

    return ctx.report()


object_model = build_object_model()
