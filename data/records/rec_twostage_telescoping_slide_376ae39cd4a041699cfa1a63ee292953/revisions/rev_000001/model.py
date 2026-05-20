from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.55
OUTER_WIDTH = 0.03
OUTER_HEIGHT = 0.055
OUTER_WALL = 0.003

INNER_LENGTH = 0.42
INNER_WIDTH = 0.021
INNER_HEIGHT = 0.041
INNER_WALL = 0.0025

PAD_START = 0.02
PAD_LENGTH = 0.36
PAD_WIDTH = 0.012
PAD_THICKNESS = 0.004

SLIDE_TRAVEL = 0.28

TRAY_LENGTH = 0.09
TRAY_WIDTH = 0.07
TRAY_BASE_THICKNESS = 0.003
TRAY_SIDE_HEIGHT = 0.012
TRAY_WALL = 0.0025

SUPPORT_START = INNER_LENGTH - 0.06
SUPPORT_LENGTH = 0.04
OUTRIGGER_WIDTH = 0.012
OUTRIGGER_THICKNESS = 0.004
OUTRIGGER_CENTER_Y = 0.014
POST_WIDTH = 0.008
POST_CENTER_Y = 0.02
POST_HEIGHT = 0.028

TRAY_MOUNT_Z = INNER_HEIGHT / 2.0 + PAD_THICKNESS + POST_HEIGHT


def _add_box_visual(part, *, name: str, size: tuple[float, float, float], xyz: tuple[float, float, float], material) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_c_channel(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    material,
) -> None:
    half_width = width / 2.0
    half_height = height / 2.0
    flange_span = width - wall

    _add_box_visual(
        part,
        name=f"{prefix}_web",
        size=(length, wall, height),
        xyz=(length / 2.0, -half_width + wall / 2.0, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_top_flange",
        size=(length, flange_span, wall),
        xyz=(length / 2.0, wall / 2.0, half_height - wall / 2.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_bottom_flange",
        size=(length, flange_span, wall),
        xyz=(length / 2.0, wall / 2.0, -half_height + wall / 2.0),
        material=material,
    )


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return (
        max_corner[0] - min_corner[0],
        max_corner[1] - min_corner[1],
        max_corner[2] - min_corner[2],
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    zinc_steel = model.material("zinc_steel", color=(0.72, 0.74, 0.77, 1.0))
    darker_steel = model.material("dark_steel", color=(0.47, 0.49, 0.52, 1.0))
    polymer_pad = model.material("polymer_pad", color=(0.14, 0.14, 0.16, 1.0))

    outer = model.part("outer_channel")
    _add_c_channel(
        outer,
        prefix="outer",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        material=zinc_steel,
    )
    _add_box_visual(
        outer,
        name="outer_rear_mount_plate",
        size=(0.035, OUTER_WIDTH, 0.004),
        xyz=(0.0175, 0.0, 0.0),
        material=darker_steel,
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner = model.part("inner_channel")
    _add_c_channel(
        inner,
        prefix="inner",
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        material=darker_steel,
    )
    _add_box_visual(
        inner,
        name="top_pad",
        size=(PAD_LENGTH, PAD_WIDTH, PAD_THICKNESS),
        xyz=(PAD_START + PAD_LENGTH / 2.0, INNER_WALL / 2.0, INNER_HEIGHT / 2.0 + PAD_THICKNESS / 2.0),
        material=polymer_pad,
    )
    _add_box_visual(
        inner,
        name="bottom_pad",
        size=(PAD_LENGTH, PAD_WIDTH, PAD_THICKNESS),
        xyz=(PAD_START + PAD_LENGTH / 2.0, INNER_WALL / 2.0, -INNER_HEIGHT / 2.0 - PAD_THICKNESS / 2.0),
        material=polymer_pad,
    )
    _add_box_visual(
        inner,
        name="mount_outrigger",
        size=(SUPPORT_LENGTH, OUTRIGGER_WIDTH, OUTRIGGER_THICKNESS),
        xyz=(
            SUPPORT_START + SUPPORT_LENGTH / 2.0,
            OUTRIGGER_CENTER_Y,
            INNER_HEIGHT / 2.0 + OUTRIGGER_THICKNESS / 2.0,
        ),
        material=darker_steel,
    )
    _add_box_visual(
        inner,
        name="mount_post",
        size=(SUPPORT_LENGTH, POST_WIDTH, POST_HEIGHT),
        xyz=(
            SUPPORT_START + SUPPORT_LENGTH / 2.0,
            POST_CENTER_Y,
            INNER_HEIGHT / 2.0 + PAD_THICKNESS + POST_HEIGHT / 2.0,
        ),
        material=darker_steel,
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, 0.05, 0.075)),
        mass=1.1,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    tray = model.part("mounting_tray")
    _add_box_visual(
        tray,
        name="tray_base",
        size=(TRAY_LENGTH, TRAY_WIDTH, TRAY_BASE_THICKNESS),
        xyz=(TRAY_LENGTH / 2.0, 0.0, TRAY_BASE_THICKNESS / 2.0),
        material=zinc_steel,
    )
    _add_box_visual(
        tray,
        name="tray_left_wall",
        size=(TRAY_LENGTH, TRAY_WALL, TRAY_SIDE_HEIGHT),
        xyz=(
            TRAY_LENGTH / 2.0,
            TRAY_WIDTH / 2.0 - TRAY_WALL / 2.0,
            TRAY_BASE_THICKNESS + TRAY_SIDE_HEIGHT / 2.0,
        ),
        material=zinc_steel,
    )
    _add_box_visual(
        tray,
        name="tray_right_wall",
        size=(TRAY_LENGTH, TRAY_WALL, TRAY_SIDE_HEIGHT),
        xyz=(
            TRAY_LENGTH / 2.0,
            -TRAY_WIDTH / 2.0 + TRAY_WALL / 2.0,
            TRAY_BASE_THICKNESS + TRAY_SIDE_HEIGHT / 2.0,
        ),
        material=zinc_steel,
    )
    _add_box_visual(
        tray,
        name="tray_front_wall",
        size=(TRAY_WALL, TRAY_WIDTH, TRAY_SIDE_HEIGHT),
        xyz=(
            TRAY_LENGTH - TRAY_WALL / 2.0,
            0.0,
            TRAY_BASE_THICKNESS + TRAY_SIDE_HEIGHT / 2.0,
        ),
        material=zinc_steel,
    )
    _add_box_visual(
        tray,
        name="tray_rear_wall",
        size=(TRAY_WALL, TRAY_WIDTH, TRAY_SIDE_HEIGHT * 0.75),
        xyz=(
            TRAY_WALL / 2.0,
            0.0,
            TRAY_BASE_THICKNESS + (TRAY_SIDE_HEIGHT * 0.75) / 2.0,
        ),
        material=zinc_steel,
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_LENGTH, TRAY_WIDTH, TRAY_BASE_THICKNESS + TRAY_SIDE_HEIGHT)),
        mass=0.35,
        origin=Origin(
            xyz=(
                TRAY_LENGTH / 2.0,
                0.0,
                (TRAY_BASE_THICKNESS + TRAY_SIDE_HEIGHT) / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.4,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_mounting_tray",
        ArticulationType.FIXED,
        parent=inner,
        child=tray,
        origin=Origin(xyz=(SUPPORT_START, 0.0, TRAY_MOUNT_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_channel")
    inner = object_model.get_part("inner_channel")
    tray = object_model.get_part("mounting_tray")
    slide = object_model.get_articulation("outer_to_inner_slide")
    limits = slide.motion_limits

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
        "slide_joint_is_prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"Expected PRISMATIC, got {slide.articulation_type!r}",
    )
    ctx.check(
        "slide_axis_runs_along_length",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"Expected axis (1, 0, 0), got {slide.axis!r}",
    )
    ctx.check(
        "slide_has_realistic_travel",
        limits is not None and limits.lower == 0.0 and limits.upper == SLIDE_TRAVEL,
        details=f"Unexpected motion limits: {limits!r}",
    )

    outer_size = _aabb_size(ctx.part_world_aabb(outer))
    if outer_size is not None:
        ctx.check(
            "outer_channel_dimensions",
            0.54 <= outer_size[0] <= 0.56
            and 0.029 <= outer_size[1] <= 0.031
            and 0.054 <= outer_size[2] <= 0.056,
            details=f"Outer channel size was {outer_size!r}",
        )

    tray_size = _aabb_size(ctx.part_world_aabb(tray))
    if tray_size is not None:
        ctx.check(
            "mounting_tray_dimensions",
            0.089 <= tray_size[0] <= 0.091
            and 0.069 <= tray_size[1] <= 0.071
            and 0.011 <= tray_size[2] <= 0.016,
            details=f"Tray size was {tray_size!r}",
        )

    with ctx.pose({slide: 0.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="slide_retracted_no_overlap")
        ctx.fail_if_isolated_parts(name="slide_retracted_no_floating")
        ctx.expect_origin_gap(
            inner,
            outer,
            axis="x",
            min_gap=0.0,
            max_gap=0.0,
            name="inner_origin_retracted",
        )
        ctx.expect_contact(
            inner,
            outer,
            elem_a="top_pad",
            elem_b="outer_top_flange",
            name="top_pad_contacts_outer_channel_when_retracted",
        )
        ctx.expect_contact(
            inner,
            outer,
            elem_a="bottom_pad",
            elem_b="outer_bottom_flange",
            name="bottom_pad_contacts_outer_channel_when_retracted",
        )
        ctx.expect_contact(
            tray,
            inner,
            elem_a="tray_base",
            elem_b="mount_post",
            name="tray_base_contacts_mount_post",
        )
        ctx.expect_gap(
            tray,
            outer,
            axis="z",
            min_gap=0.024,
            max_gap=0.029,
            positive_elem="tray_base",
            name="tray_clears_outer_channel_above",
        )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        upper = limits.upper
        with ctx.pose({slide: upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="slide_extended_no_overlap")
            ctx.fail_if_isolated_parts(name="slide_extended_no_floating")
            ctx.expect_origin_gap(
                inner,
                outer,
                axis="x",
                min_gap=upper - 0.001,
                max_gap=upper + 0.001,
                name="inner_origin_extended",
            )
            ctx.expect_contact(
                inner,
                outer,
                elem_a="top_pad",
                elem_b="outer_top_flange",
                name="top_pad_contacts_outer_channel_when_extended",
            )
            ctx.expect_contact(
                inner,
                outer,
                elem_a="bottom_pad",
                elem_b="outer_bottom_flange",
                name="bottom_pad_contacts_outer_channel_when_extended",
            )
            ctx.expect_contact(
                tray,
                inner,
                elem_a="tray_base",
                elem_b="mount_post",
                name="extended_tray_base_contacts_mount_post",
            )
            ctx.expect_gap(
                tray,
                outer,
                axis="z",
                min_gap=0.024,
                max_gap=0.029,
                positive_elem="tray_base",
                name="extended_tray_clears_outer_channel_above",
            )

            tray_base_aabb = ctx.part_element_world_aabb(tray, elem="tray_base")
            outer_aabb = ctx.part_world_aabb(outer)
            if tray_base_aabb is not None and outer_aabb is not None:
                ctx.check(
                    "tray_projects_past_outer_front_when_extended",
                    tray_base_aabb[1][0] > outer_aabb[1][0] + 0.10,
                    details=(
                        f"Tray front x={tray_base_aabb[1][0]:.4f}, "
                        f"outer front x={outer_aabb[1][0]:.4f}"
                    ),
                )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
