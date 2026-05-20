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


OUTER_LENGTH = 0.45
OUTER_WIDTH = 0.10
OUTER_FLOOR_THICKNESS = 0.006
OUTER_WALL_THICKNESS = 0.008
OUTER_WALL_HEIGHT = 0.032
OUTER_REAR_STOP = 0.020

MIDDLE_LENGTH = 0.34
MIDDLE_WIDTH = 0.076
MIDDLE_FLOOR_THICKNESS = 0.004
MIDDLE_WALL_THICKNESS = 0.008
MIDDLE_WALL_HEIGHT = 0.026
MIDDLE_REAR_STOP = 0.018

INNER_LENGTH = 0.24
INNER_WIDTH = 0.060
INNER_FLOOR_THICKNESS = 0.004
INNER_WALL_THICKNESS = 0.008
INNER_WALL_HEIGHT = 0.022
INNER_REAR_STOP = 0.012

MIDDLE_TRAVEL = 0.18
INNER_TRAVEL = 0.16


def _u_channel(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    floor_thickness: float,
    wall_thickness: float,
    wall_height: float,
    material: str,
) -> None:
    wall_y = (width - wall_thickness) / 2.0

    part.visual(
        Box((length, width, floor_thickness)),
        origin=Origin(xyz=(length / 2.0, 0.0, floor_thickness / 2.0)),
        material=material,
        name=f"{prefix}_floor",
    )
    part.visual(
        Box((length, wall_thickness, wall_height)),
        origin=Origin(xyz=(length / 2.0, wall_y, floor_thickness + wall_height / 2.0)),
        material=material,
        name=f"{prefix}_wall_left",
    )
    part.visual(
        Box((length, wall_thickness, wall_height)),
        origin=Origin(xyz=(length / 2.0, -wall_y, floor_thickness + wall_height / 2.0)),
        material=material,
        name=f"{prefix}_wall_right",
    )


def _stop_block(
    part,
    *,
    name: str,
    length: float,
    width: float,
    height: float,
    x0: float,
    z0: float,
    material: str,
) -> None:
    part.visual(
        Box((length, width, height)),
        origin=Origin(xyz=(x0 + length / 2.0, 0.0, z0 + height / 2.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    model.material("outer_slide_finish", rgba=(0.42, 0.44, 0.48, 1.0))
    model.material("middle_slide_finish", rgba=(0.63, 0.65, 0.69, 1.0))
    model.material("inner_slide_finish", rgba=(0.18, 0.19, 0.21, 1.0))

    outer_slide = model.part("outer_slide")
    _u_channel(
        outer_slide,
        prefix="outer_slide",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        floor_thickness=OUTER_FLOOR_THICKNESS,
        wall_thickness=OUTER_WALL_THICKNESS,
        wall_height=OUTER_WALL_HEIGHT,
        material="outer_slide_finish",
    )
    _stop_block(
        outer_slide,
        name="outer_rear_stop",
        length=OUTER_REAR_STOP,
        width=OUTER_WIDTH - 0.020,
        height=0.014,
        x0=0.0,
        z0=OUTER_FLOOR_THICKNESS,
        material="outer_slide_finish",
    )

    middle_slide = model.part("middle_slide")
    _u_channel(
        middle_slide,
        prefix="middle_slide",
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        floor_thickness=MIDDLE_FLOOR_THICKNESS,
        wall_thickness=MIDDLE_WALL_THICKNESS,
        wall_height=MIDDLE_WALL_HEIGHT,
        material="middle_slide_finish",
    )
    _stop_block(
        middle_slide,
        name="middle_rear_stop",
        length=MIDDLE_REAR_STOP,
        width=MIDDLE_WIDTH - 0.016,
        height=0.012,
        x0=0.0,
        z0=MIDDLE_FLOOR_THICKNESS,
        material="middle_slide_finish",
    )

    inner_slide = model.part("inner_slide")
    _u_channel(
        inner_slide,
        prefix="inner_slide",
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        floor_thickness=INNER_FLOOR_THICKNESS,
        wall_thickness=INNER_WALL_THICKNESS,
        wall_height=INNER_WALL_HEIGHT,
        material="inner_slide_finish",
    )
    _stop_block(
        inner_slide,
        name="inner_rear_stop",
        length=INNER_REAR_STOP,
        width=INNER_WIDTH - 0.016,
        height=0.010,
        x0=0.0,
        z0=INNER_FLOOR_THICKNESS,
        material="inner_slide_finish",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_slide,
        child=middle_slide,
        origin=Origin(xyz=(OUTER_REAR_STOP, 0.0, OUTER_FLOOR_THICKNESS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )

    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_slide,
        child=inner_slide,
        origin=Origin(xyz=(MIDDLE_REAR_STOP, 0.0, MIDDLE_FLOOR_THICKNESS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.45,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_slide = object_model.get_part("outer_slide")
    middle_slide = object_model.get_part("middle_slide")
    inner_slide = object_model.get_part("inner_slide")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "slide articulations are serial prismatic x-joints",
        (
            outer_to_middle.articulation_type == ArticulationType.PRISMATIC
            and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
            and tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0)
            and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0)
        ),
        details=(
            f"outer_to_middle={outer_to_middle.articulation_type}/{outer_to_middle.axis}, "
            f"middle_to_inner={middle_to_inner.articulation_type}/{middle_to_inner.axis}"
        ),
    )
    ctx.check(
        "slide travel limits stay positive and realistic",
        (
            outer_to_middle.motion_limits is not None
            and middle_to_inner.motion_limits is not None
            and outer_to_middle.motion_limits.lower == 0.0
            and middle_to_inner.motion_limits.lower == 0.0
            and outer_to_middle.motion_limits.upper is not None
            and middle_to_inner.motion_limits.upper is not None
            and 0.12 <= outer_to_middle.motion_limits.upper <= 0.22
            and 0.12 <= middle_to_inner.motion_limits.upper <= 0.20
        ),
        details=(
            f"outer travel={outer_to_middle.motion_limits}, "
            f"inner travel={middle_to_inner.motion_limits}"
        ),
    )

    ctx.expect_contact(
        middle_slide,
        outer_slide,
        name="middle section is seated on the grounded outer body",
    )
    ctx.expect_contact(
        inner_slide,
        middle_slide,
        name="inner section is seated on the middle section",
    )
    ctx.expect_within(
        middle_slide,
        outer_slide,
        axes="yz",
        margin=1e-5,
        name="middle section stays nested within outer guide width and height",
    )
    ctx.expect_within(
        inner_slide,
        middle_slide,
        axes="yz",
        margin=1e-5,
        name="inner section stays nested within middle guide width and height",
    )

    with ctx.pose({outer_to_middle: MIDDLE_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_origin_gap(
            middle_slide,
            outer_slide,
            axis="x",
            min_gap=OUTER_REAR_STOP + MIDDLE_TRAVEL - 0.001,
            max_gap=OUTER_REAR_STOP + MIDDLE_TRAVEL + 0.001,
            name="middle stage extends forward along the slide axis",
        )
        ctx.expect_origin_gap(
            inner_slide,
            middle_slide,
            axis="x",
            min_gap=MIDDLE_REAR_STOP + INNER_TRAVEL - 0.001,
            max_gap=MIDDLE_REAR_STOP + INNER_TRAVEL + 0.001,
            name="inner stage extends forward relative to the middle stage",
        )
        ctx.expect_overlap(
            middle_slide,
            outer_slide,
            axes="x",
            min_overlap=0.24,
            name="middle stage retains engaged overlap at full extension",
        )
        ctx.expect_overlap(
            inner_slide,
            middle_slide,
            axes="x",
            min_overlap=0.15,
            name="inner stage retains engaged overlap at full extension",
        )
        ctx.expect_overlap(
            middle_slide,
            outer_slide,
            axes="y",
            min_overlap=0.07,
            name="middle stage remains laterally captured in the outer guide",
        )
        ctx.expect_overlap(
            inner_slide,
            middle_slide,
            axes="y",
            min_overlap=0.055,
            name="inner stage remains laterally captured in the middle guide",
        )
        ctx.expect_overlap(
            middle_slide,
            outer_slide,
            axes="z",
            min_overlap=0.028,
            name="middle stage retains vertical wall engagement in the outer guide",
        )
        ctx.expect_overlap(
            inner_slide,
            middle_slide,
            axes="z",
            min_overlap=0.024,
            name="inner stage retains vertical wall engagement in the middle guide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
