from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LADDER_WIDTH = 0.42
RAIL_CENTER_X = 0.185
BASE_LENGTH = 1.22
FOLD_LENGTH = 1.16
FLY_LENGTH = 1.12
HINGE_Z = 1.20
SECTION_OFFSET_Y = 0.054
FLY_OFFSET_Y = 0.024
BASE_RAIL_OUTER = (0.056, 0.024)
FOLD_RAIL_OUTER = (0.052, 0.022)
FLY_RAIL_OUTER = (0.044, 0.018)
SLIDE_TRAVEL = 0.56
FOLD_TRAVEL = pi


def _rect_tube_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    wall: float,
    length: float,
    corner_radius: float,
    cap: bool = True,
):
    inner_width = width - 2.0 * wall
    inner_depth = depth - 2.0 * wall
    inner_radius = max(min(corner_radius - wall, inner_width * 0.35, inner_depth * 0.35), 0.001)
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, depth, corner_radius, corner_segments=6),
        [rounded_rect_profile(inner_width, inner_depth, inner_radius, corner_segments=6)],
        height=length,
        cap=cap,
        center=False,
    )
    return mesh_from_geometry(geom, name)


def _add_rungs(
    part,
    *,
    z_positions: tuple[float, ...],
    span: float,
    depth: float,
    height: float,
    material,
    name_prefix: str,
) -> None:
    for index, z_pos in enumerate(z_positions, start=1):
        part.visual(
            Box((span, depth, height)),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=material,
            name=f"{name_prefix}_rung_{index}",
        )


def _overall_z_span(ctx: TestContext, parts: tuple) -> float | None:
    z_min = None
    z_max = None
    for part in parts:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        lo, hi = aabb
        z_min = lo[2] if z_min is None else min(z_min, lo[2])
        z_max = hi[2] if z_max is None else max(z_max, hi[2])
    if z_min is None or z_max is None:
        return None
    return z_max - z_min


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_flat_extension_ladder")

    aluminium = model.material("aluminium", rgba=(0.78, 0.80, 0.82, 1.0))
    dull_aluminium = model.material("dull_aluminium", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.27, 0.29, 1.0))
    nylon = model.material("nylon", rgba=(0.18, 0.18, 0.19, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base_section = model.part("base_section")
    base_section.inertial = Inertial.from_geometry(
        Box((LADDER_WIDTH, 0.08, BASE_LENGTH)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, BASE_LENGTH * 0.5)),
    )
    base_left_rail = _rect_tube_mesh(
        "base_left_rail",
        width=BASE_RAIL_OUTER[0],
        depth=BASE_RAIL_OUTER[1],
        wall=0.0025,
        length=BASE_LENGTH,
        corner_radius=0.005,
    )
    base_right_rail = _rect_tube_mesh(
        "base_right_rail",
        width=BASE_RAIL_OUTER[0],
        depth=BASE_RAIL_OUTER[1],
        wall=0.0025,
        length=BASE_LENGTH,
        corner_radius=0.005,
    )
    base_section.visual(
        base_left_rail,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminium,
        name="left_stile",
    )
    base_section.visual(
        base_right_rail,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminium,
        name="right_stile",
    )
    _add_rungs(
        base_section,
        z_positions=(0.20, 0.44, 0.68, 0.92),
        span=2.0 * RAIL_CENTER_X - BASE_RAIL_OUTER[0],
        depth=0.030,
        height=0.022,
        material=dull_aluminium,
        name_prefix="base",
    )
    base_section.visual(
        Box((2.0 * RAIL_CENTER_X - BASE_RAIL_OUTER[0], 0.022, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        material=dull_aluminium,
        name="upper_cross_brace",
    )
    base_section.visual(
        Box((0.040, 0.018, 0.060)),
        origin=Origin(xyz=(-0.199, 0.010, HINGE_Z - 0.030)),
        material=dark_steel,
        name="left_hinge_lug",
    )
    base_section.visual(
        Box((0.040, 0.018, 0.060)),
        origin=Origin(xyz=(0.199, 0.010, HINGE_Z - 0.030)),
        material=dark_steel,
        name="right_hinge_lug",
    )
    base_section.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(-0.219, 0.018, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    base_section.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.219, 0.018, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_hinge_barrel",
    )
    base_section.visual(
        Box((0.060, 0.030, 0.020)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, 0.010)),
        material=rubber,
        name="left_foot",
    )
    base_section.visual(
        Box((0.060, 0.030, 0.020)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, 0.010)),
        material=rubber,
        name="right_foot",
    )

    fold_frame = model.part("fold_frame")
    fold_frame.inertial = Inertial.from_geometry(
        Box((LADDER_WIDTH, 0.08, FOLD_LENGTH)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, FOLD_LENGTH * 0.5)),
    )
    fold_left_rail = _rect_tube_mesh(
        "fold_left_rail",
        width=FOLD_RAIL_OUTER[0],
        depth=FOLD_RAIL_OUTER[1],
        wall=0.0025,
        length=FOLD_LENGTH,
        corner_radius=0.0045,
        cap=False,
    )
    fold_right_rail = _rect_tube_mesh(
        "fold_right_rail",
        width=FOLD_RAIL_OUTER[0],
        depth=FOLD_RAIL_OUTER[1],
        wall=0.0025,
        length=FOLD_LENGTH,
        corner_radius=0.0045,
        cap=False,
    )
    fold_frame.visual(
        fold_left_rail,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, 0.0)),
        material=dull_aluminium,
        name="left_stile",
    )
    fold_frame.visual(
        fold_right_rail,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, 0.0)),
        material=dull_aluminium,
        name="right_stile",
    )
    _add_rungs(
        fold_frame,
        z_positions=(0.18, 0.42, 0.66, 0.90),
        span=2.0 * RAIL_CENTER_X - FOLD_RAIL_OUTER[0],
        depth=0.026,
        height=0.020,
        material=dull_aluminium,
        name_prefix="fold",
    )
    fold_frame.visual(
        Box((2.0 * RAIL_CENTER_X - FOLD_RAIL_OUTER[0], 0.020, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_steel,
        name="bottom_bridge",
    )
    fold_frame.visual(
        Box((2.0 * RAIL_CENTER_X - FOLD_RAIL_OUTER[0], 0.020, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        material=dark_steel,
        name="top_bridge",
    )
    fold_frame.visual(
        Box((0.040, 0.022, 0.070)),
        origin=Origin(xyz=(-0.199, -0.018, 0.015)),
        material=dark_steel,
        name="left_hinge_cheek",
    )
    fold_frame.visual(
        Box((0.040, 0.022, 0.070)),
        origin=Origin(xyz=(0.199, -0.018, 0.015)),
        material=dark_steel,
        name="right_hinge_cheek",
    )
    fold_frame.visual(
        Box((0.040, 0.004, 0.92)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.009, 0.50)),
        material=nylon,
        name="left_guide_strip",
    )
    fold_frame.visual(
        Box((0.040, 0.004, 0.92)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.009, 0.50)),
        material=nylon,
        name="right_guide_strip",
    )

    fly_section = model.part("fly_section")
    fly_section.inertial = Inertial.from_geometry(
        Box((LADDER_WIDTH, 0.06, FLY_LENGTH)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, FLY_LENGTH * 0.5)),
    )
    fly_left_rail = _rect_tube_mesh(
        "fly_left_rail",
        width=FLY_RAIL_OUTER[0],
        depth=FLY_RAIL_OUTER[1],
        wall=0.0025,
        length=FLY_LENGTH,
        corner_radius=0.004,
    )
    fly_right_rail = _rect_tube_mesh(
        "fly_right_rail",
        width=FLY_RAIL_OUTER[0],
        depth=FLY_RAIL_OUTER[1],
        wall=0.0025,
        length=FLY_LENGTH,
        corner_radius=0.004,
    )
    fly_section.visual(
        fly_left_rail,
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminium,
        name="left_stile",
    )
    fly_section.visual(
        fly_right_rail,
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, 0.0)),
        material=aluminium,
        name="right_stile",
    )
    _add_rungs(
        fly_section,
        z_positions=(0.16, 0.40, 0.64, 0.88),
        span=2.0 * RAIL_CENTER_X - FLY_RAIL_OUTER[0],
        depth=0.022,
        height=0.020,
        material=dull_aluminium,
        name_prefix="fly",
    )
    fly_section.visual(
        Box((0.040, 0.016, 0.200)),
        origin=Origin(xyz=(-RAIL_CENTER_X, -0.005, 0.120)),
        material=nylon,
        name="left_slider_pad",
    )
    fly_section.visual(
        Box((0.040, 0.016, 0.200)),
        origin=Origin(xyz=(RAIL_CENTER_X, -0.005, 0.120)),
        material=nylon,
        name="right_slider_pad",
    )
    fly_section.visual(
        Box((0.048, 0.020, 0.024)),
        origin=Origin(xyz=(-RAIL_CENTER_X, 0.0, FLY_LENGTH - 0.012)),
        material=nylon,
        name="left_top_cap",
    )
    fly_section.visual(
        Box((0.048, 0.020, 0.024)),
        origin=Origin(xyz=(RAIL_CENTER_X, 0.0, FLY_LENGTH - 0.012)),
        material=nylon,
        name="right_top_cap",
    )

    model.articulation(
        "base_to_fold_frame",
        ArticulationType.REVOLUTE,
        parent=base_section,
        child=fold_frame,
        origin=Origin(xyz=(0.0, SECTION_OFFSET_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.4, lower=0.0, upper=FOLD_TRAVEL),
    )
    model.articulation(
        "fold_frame_to_fly",
        ArticulationType.PRISMATIC,
        parent=fold_frame,
        child=fly_section,
        origin=Origin(xyz=(0.0, FLY_OFFSET_Y, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_section = object_model.get_part("base_section")
    fold_frame = object_model.get_part("fold_frame")
    fly_section = object_model.get_part("fly_section")
    fold_joint = object_model.get_articulation("base_to_fold_frame")
    slide_joint = object_model.get_articulation("fold_frame_to_fly")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fold hinge runs across ladder width",
        fold_joint.axis == (-1.0, 0.0, 0.0),
        details=f"axis={fold_joint.axis}",
    )
    ctx.check(
        "fly section slides along ladder length",
        slide_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={slide_joint.axis}",
    )

    with ctx.pose({fold_joint: 0.0, slide_joint: 0.0}):
        ctx.expect_gap(
            fold_frame,
            base_section,
            axis="y",
            positive_elem="left_stile",
            negative_elem="left_stile",
            min_gap=0.028,
            max_gap=0.036,
            name="upper section stands proud of the lower section for folding clearance",
        )
        ctx.expect_overlap(
            fold_frame,
            base_section,
            axes="x",
            min_overlap=0.34,
            name="folding and base sections keep common ladder width",
        )
        ctx.expect_gap(
            fly_section,
            fold_frame,
            axis="y",
            positive_elem="left_stile",
            negative_elem="left_stile",
            min_gap=0.003,
            max_gap=0.008,
            name="fly section runs just ahead of the upper rails",
        )
        ctx.expect_contact(
            fly_section,
            fold_frame,
            elem_a="left_slider_pad",
            elem_b="left_guide_strip",
            contact_tol=1e-4,
            name="left guide pad bears on the upper left stile",
        )
        ctx.expect_contact(
            fly_section,
            fold_frame,
            elem_a="right_slider_pad",
            elem_b="right_guide_strip",
            contact_tol=1e-4,
            name="right guide pad bears on the upper right stile",
        )

    rest_pos = ctx.part_world_position(fly_section)
    with ctx.pose({fold_joint: 0.0, slide_joint: SLIDE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap in fully extended pose")
        ctx.expect_gap(
            fly_section,
            fold_frame,
            axis="y",
            positive_elem="left_stile",
            negative_elem="left_stile",
            min_gap=0.003,
            max_gap=0.008,
            name="extended fly still tracks ahead of the upper rails",
        )
        ctx.expect_contact(
            fly_section,
            fold_frame,
            elem_a="left_slider_pad",
            elem_b="left_guide_strip",
            contact_tol=1e-4,
            name="left guide pad stays in contact at full extension",
        )
        ctx.expect_contact(
            fly_section,
            fold_frame,
            elem_a="right_slider_pad",
            elem_b="right_guide_strip",
            contact_tol=1e-4,
            name="right guide pad stays in contact at full extension",
        )
        ctx.expect_overlap(
            fly_section,
            fold_frame,
            axes="z",
            elem_a="left_slider_pad",
            elem_b="left_guide_strip",
            min_overlap=0.18,
            name="left guide retains overlap at full extension",
        )
        ctx.expect_overlap(
            fly_section,
            fold_frame,
            axes="z",
            elem_a="right_slider_pad",
            elem_b="right_guide_strip",
            min_overlap=0.18,
            name="right guide retains overlap at full extension",
        )
        extended_pos = ctx.part_world_position(fly_section)

    ctx.check(
        "fly extends upward when the prismatic joint opens",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.40,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({fold_joint: FOLD_TRAVEL, slide_joint: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlap in folded storage pose")
        ctx.expect_within(
            fly_section,
            base_section,
            axes="xz",
            margin=0.05,
            name="folded fly nests inside the base storage envelope",
        )
        storage_span = _overall_z_span(ctx, (base_section, fold_frame, fly_section))

    with ctx.pose({fold_joint: 0.0, slide_joint: SLIDE_TRAVEL}):
        deployed_span = _overall_z_span(ctx, (base_section, fold_frame, fly_section))

    ctx.check(
        "fold-flat pose shortens the overall storage length",
        storage_span is not None
        and deployed_span is not None
        and storage_span < 1.30
        and deployed_span > storage_span + 1.20,
        details=f"storage_span={storage_span}, deployed_span={deployed_span}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
