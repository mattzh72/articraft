from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.48
OUTER_WIDTH = 0.055
OUTER_HEIGHT = 0.022
OUTER_WALL = 0.0025
OUTER_RUNNER_HEIGHT = 0.0
OUTER_RUNNER_WIDTH = 0.0

MIDDLE_LENGTH = 0.36
MIDDLE_WIDTH = 0.044
MIDDLE_HEIGHT = 0.014
MIDDLE_WALL = 0.0018
MIDDLE_RUNNER_HEIGHT = 0.0
MIDDLE_RUNNER_WIDTH = 0.0

INNER_LENGTH = 0.28
INNER_WIDTH = 0.032
INNER_HEIGHT = 0.009
INNER_WALL = 0.0016

MIDDLE_TRAVEL = 0.26
INNER_TRAVEL = 0.20

TRAY_LENGTH = 0.16
TRAY_WIDTH = 0.14
TRAY_THICKNESS = 0.003
TRAY_FRONT_LIP_HEIGHT = 0.018
TRAY_FRONT_LIP_THICKNESS = 0.010
TRAY_REAR_OVERLAP = 0.050
TRAY_SUPPORT_HEIGHT = 0.012
TRAY_SUPPORT_WIDTH = 0.008


def add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def add_channel_visuals(
    part,
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    material,
    prefix: str,
    runner_height: float = 0.0,
    runner_width: float = 0.0,
) -> None:
    side_height = height - wall

    add_box_visual(
        part,
        name=f"{prefix}_bottom",
        size=(length, width, wall),
        xyz=(length * 0.5, 0.0, wall * 0.5),
        material=material,
    )
    add_box_visual(
        part,
        name=f"{prefix}_left_wall",
        size=(length, wall, side_height),
        xyz=(length * 0.5, width * 0.5 - wall * 0.5, wall + side_height * 0.5),
        material=material,
    )
    add_box_visual(
        part,
        name=f"{prefix}_right_wall",
        size=(length, wall, side_height),
        xyz=(length * 0.5, -width * 0.5 + wall * 0.5, wall + side_height * 0.5),
        material=material,
    )

    if runner_height > 0.0 and runner_width > 0.0:
        add_box_visual(
            part,
            name=f"{prefix}_left_runner",
            size=(length, runner_width, runner_height),
            xyz=(
                length * 0.5,
                width * 0.5 - wall - runner_width * 0.5,
                wall + runner_height * 0.5,
            ),
            material=material,
        )
        add_box_visual(
            part,
            name=f"{prefix}_right_runner",
            size=(length, runner_width, runner_height),
            xyz=(
                length * 0.5,
                -width * 0.5 + wall + runner_width * 0.5,
                wall + runner_height * 0.5,
            ),
            material=material,
        )


def add_tray_visuals(part, *, material) -> None:
    add_box_visual(
        part,
        name="tray_left_support",
        size=(TRAY_REAR_OVERLAP, TRAY_SUPPORT_WIDTH, TRAY_SUPPORT_HEIGHT),
        xyz=(TRAY_REAR_OVERLAP * 0.5, INNER_WIDTH * 0.5 - TRAY_SUPPORT_WIDTH * 0.5, TRAY_SUPPORT_HEIGHT * 0.5),
        material=material,
    )
    add_box_visual(
        part,
        name="tray_right_support",
        size=(TRAY_REAR_OVERLAP, TRAY_SUPPORT_WIDTH, TRAY_SUPPORT_HEIGHT),
        xyz=(TRAY_REAR_OVERLAP * 0.5, -INNER_WIDTH * 0.5 + TRAY_SUPPORT_WIDTH * 0.5, TRAY_SUPPORT_HEIGHT * 0.5),
        material=material,
    )
    add_box_visual(
        part,
        name="tray_plate_top",
        size=(TRAY_LENGTH, TRAY_WIDTH, TRAY_THICKNESS),
        xyz=(TRAY_LENGTH * 0.5, 0.0, TRAY_SUPPORT_HEIGHT + TRAY_THICKNESS * 0.5),
        material=material,
    )
    add_box_visual(
        part,
        name="tray_front_lip",
        size=(TRAY_FRONT_LIP_THICKNESS, TRAY_WIDTH, TRAY_FRONT_LIP_HEIGHT),
        xyz=(
            TRAY_LENGTH - TRAY_FRONT_LIP_THICKNESS * 0.5,
            0.0,
            TRAY_SUPPORT_HEIGHT + TRAY_THICKNESS + TRAY_FRONT_LIP_HEIGHT * 0.5,
        ),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    outer_steel = model.material("outer_steel", rgba=(0.28, 0.30, 0.34, 1.0))
    middle_steel = model.material("middle_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    inner_steel = model.material("inner_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    tray_finish = model.material("tray_finish", rgba=(0.18, 0.30, 0.44, 1.0))

    outer_frame = model.part("outer_frame")
    add_channel_visuals(
        outer_frame,
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        material=outer_steel,
        prefix="outer",
        runner_height=OUTER_RUNNER_HEIGHT,
        runner_width=OUTER_RUNNER_WIDTH,
    )

    middle_slide = model.part("middle_slide")
    add_channel_visuals(
        middle_slide,
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        material=middle_steel,
        prefix="middle",
        runner_height=MIDDLE_RUNNER_HEIGHT,
        runner_width=MIDDLE_RUNNER_WIDTH,
    )

    inner_slide = model.part("inner_slide")
    add_channel_visuals(
        inner_slide,
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        material=inner_steel,
        prefix="inner",
    )

    tray_plate = model.part("tray_plate")
    add_tray_visuals(tray_plate, material=tray_finish)

    model.articulation(
        "outer_to_middle_slide",
        ArticulationType.PRISMATIC,
        parent=outer_frame,
        child=middle_slide,
        origin=Origin(xyz=(0.0, 0.0, OUTER_WALL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.60,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=middle_slide,
        child=inner_slide,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_WALL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.60,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_tray",
        ArticulationType.FIXED,
        parent=inner_slide,
        child=tray_plate,
        origin=Origin(xyz=(INNER_LENGTH - TRAY_REAR_OVERLAP, 0.0, INNER_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_frame = object_model.get_part("outer_frame")
    middle_slide = object_model.get_part("middle_slide")
    inner_slide = object_model.get_part("inner_slide")
    tray_plate = object_model.get_part("tray_plate")

    outer_to_middle = object_model.get_articulation("outer_to_middle_slide")
    middle_to_inner = object_model.get_articulation("middle_to_inner_slide")
    inner_to_tray = object_model.get_articulation("inner_to_tray")

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
        "outer_frame_is_only_root",
        [part.name for part in object_model.root_parts()] == ["outer_frame"],
        f"roots={[part.name for part in object_model.root_parts()]}",
    )
    ctx.check(
        "outer_to_middle_prismatic_axis",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0),
        f"type={outer_to_middle.articulation_type}, axis={outer_to_middle.axis}",
    )
    ctx.check(
        "middle_to_inner_prismatic_axis",
        middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        f"type={middle_to_inner.articulation_type}, axis={middle_to_inner.axis}",
    )
    ctx.check(
        "inner_to_tray_fixed",
        inner_to_tray.articulation_type == ArticulationType.FIXED,
        f"type={inner_to_tray.articulation_type}",
    )

    ctx.expect_contact(
        outer_frame,
        middle_slide,
        contact_tol=0.0005,
        name="outer_and_middle_are_in_support_contact",
    )
    ctx.expect_contact(
        middle_slide,
        inner_slide,
        contact_tol=0.0005,
        name="middle_and_inner_are_in_support_contact",
    )
    ctx.expect_contact(
        inner_slide,
        tray_plate,
        contact_tol=0.0005,
        name="tray_is_mounted_to_inner_slide",
    )
    ctx.expect_within(
        middle_slide,
        outer_frame,
        axes="yz",
        margin=0.0,
        name="middle_fits_inside_outer_cross_section",
    )
    ctx.expect_within(
        inner_slide,
        middle_slide,
        axes="yz",
        margin=0.0,
        name="inner_fits_inside_middle_cross_section",
    )
    ctx.expect_overlap(
        middle_slide,
        outer_frame,
        axes="x",
        min_overlap=0.30,
        name="middle_has_large_retracted_engagement",
    )
    ctx.expect_overlap(
        inner_slide,
        middle_slide,
        axes="x",
        min_overlap=0.24,
        name="inner_has_large_retracted_engagement",
    )
    ctx.expect_overlap(
        tray_plate,
        inner_slide,
        axes="xy",
        min_overlap=0.03,
        name="tray_overlaps_inner_mount_zone",
    )

    for joint, prefix in (
        (outer_to_middle, "outer_to_middle"),
        (middle_to_inner, "middle_to_inner"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_upper_no_floating")

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="slide_articulation_clearance_sweep")

    outer_upper = outer_to_middle.motion_limits.upper
    inner_upper = middle_to_inner.motion_limits.upper
    if outer_upper is not None and inner_upper is not None:
        with ctx.pose({outer_to_middle: outer_upper, middle_to_inner: inner_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="full_extension_no_overlap")
            ctx.fail_if_isolated_parts(name="full_extension_no_floating")
            ctx.expect_overlap(
                middle_slide,
                outer_frame,
                axes="x",
                min_overlap=MIDDLE_LENGTH - MIDDLE_TRAVEL - 0.005,
                name="middle_keeps_engagement_at_full_extension",
            )
            ctx.expect_overlap(
                inner_slide,
                middle_slide,
                axes="x",
                min_overlap=INNER_LENGTH - INNER_TRAVEL - 0.005,
                name="inner_keeps_engagement_at_full_extension",
            )
            ctx.expect_contact(
                outer_frame,
                middle_slide,
                contact_tol=0.0005,
                name="middle_stays_supported_at_full_extension",
            )
            ctx.expect_contact(
                middle_slide,
                inner_slide,
                contact_tol=0.0005,
                name="inner_stays_supported_at_full_extension",
            )
            ctx.expect_contact(
                inner_slide,
                tray_plate,
                contact_tol=0.0005,
                name="tray_stays_attached_at_full_extension",
            )
            ctx.expect_gap(
                tray_plate,
                outer_frame,
                axis="x",
                min_gap=0.12,
                name="tray_plate_projects_beyond_outer_frame",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
