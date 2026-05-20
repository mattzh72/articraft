from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_WIDTH = 0.140
SUPPORT_HEIGHT = 0.220
PLATE_THICKNESS = 0.008
SUPPORT_PAD_THICKNESS = 0.012
SUPPORT_PAD_WIDTH = 0.056
SUPPORT_PAD_HEIGHT = 0.048
SUPPORT_TOTAL_DEPTH = PLATE_THICKNESS + SUPPORT_PAD_THICKNESS

GUIDE_LENGTH = 0.320
GUIDE_WIDTH = 0.050
GUIDE_HEIGHT = 0.040
GUIDE_FLOOR_THICKNESS = 0.004
GUIDE_SIDE_THICKNESS = 0.004
GUIDE_SIDE_HEIGHT = 0.028
GUIDE_SIDE_CENTER_Y = 0.023
GUIDE_SIDE_CENTER_Z = -0.004
GUIDE_FLOOR_CENTER_Z = -0.018
GUIDE_LIP_WIDTH = 0.012
GUIDE_LIP_THICKNESS = 0.004
GUIDE_LIP_CENTER_Y = 0.015
GUIDE_LIP_CENTER_Z = 0.012

SLIDER_LENGTH = 0.340
SLIDER_BODY_WIDTH = 0.024
SLIDER_BODY_HEIGHT = 0.016
SLIDER_BODY_CENTER_Z = -0.006
RUNNER_WIDTH = 0.008
RUNNER_HEIGHT = 0.008
RUNNER_Y = 0.017
RUNNER_CENTER_Z = 0.006
FRONT_PAD_THICKNESS = 0.008
FRONT_PAD_WIDTH = 0.070
FRONT_PAD_HEIGHT = 0.060

SLIDE_HOME_OFFSET = 0.030
SLIDE_TRAVEL = 0.170


def _support_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .rect(SUPPORT_WIDTH, SUPPORT_HEIGHT)
        .extrude(PLATE_THICKNESS)
    )
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.045, -0.075),
                (0.045, -0.075),
                (-0.045, 0.075),
                (0.045, 0.075),
            ]
        )
        .hole(0.010)
    )
    support = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(SUPPORT_PAD_WIDTH, SUPPORT_PAD_HEIGHT)
        .extrude(SUPPORT_PAD_THICKNESS)
    )
    return support


def _outer_guide_shape() -> cq.Workplane:
    outer_shell = cq.Workplane("XY").box(
        GUIDE_LENGTH,
        GUIDE_WIDTH,
        GUIDE_HEIGHT,
        centered=(False, True, True),
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH + 0.010,
            GUIDE_WIDTH - 2.0 * GUIDE_WALL,
            GUIDE_HEIGHT - 2.0 * GUIDE_WALL,
            centered=(False, True, True),
        )
        .translate((-0.005, 0.0, 0.0))
    )
    top_slot = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH + 0.010,
            GUIDE_SLOT_WIDTH,
            GUIDE_SLOT_DEPTH,
            centered=(False, True, True),
        )
        .translate((-0.005, 0.0, GUIDE_HEIGHT * 0.5 - GUIDE_SLOT_DEPTH * 0.5))
    )
    return outer_shell.cut(cavity).cut(top_slot)


def _moving_section_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            SLIDER_LENGTH,
            SLIDER_BODY_WIDTH,
            SLIDER_BODY_HEIGHT,
            centered=(False, True, True),
        )
        .translate((0.0, 0.0, SLIDER_BODY_CENTER_Z))
    )
    left_runner = (
        cq.Workplane("XY")
        .box(
            SLIDER_LENGTH,
            RUNNER_WIDTH,
            RUNNER_HEIGHT,
            centered=(False, True, True),
        )
        .translate((0.0, RUNNER_Y, RUNNER_CENTER_Z))
    )
    right_runner = (
        cq.Workplane("XY")
        .box(
            SLIDER_LENGTH,
            RUNNER_WIDTH,
            RUNNER_HEIGHT,
            centered=(False, True, True),
        )
        .translate((0.0, -RUNNER_Y, RUNNER_CENTER_Z))
    )
    front_pad = (
        cq.Workplane("XY")
        .box(
            FRONT_PAD_THICKNESS,
            FRONT_PAD_WIDTH,
            FRONT_PAD_HEIGHT,
            centered=(False, True, True),
        )
        .translate((SLIDER_LENGTH - FRONT_PAD_THICKNESS, 0.0, 0.0))
    )
    front_pad = (
        front_pad.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.020, -0.016),
                (0.020, -0.016),
                (-0.020, 0.016),
                (0.020, 0.016),
            ]
        )
        .hole(0.006)
    )
    return body.union(left_runner).union(right_runner).union(front_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_telescoping_slide")

    model.material("support_paint", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("outer_steel", rgba=(0.66, 0.68, 0.72, 1.0))
    model.material("inner_steel", rgba=(0.82, 0.84, 0.87, 1.0))

    wall_support = model.part("wall_support")
    wall_support.visual(
        mesh_from_cadquery(_support_shape(), "wall_support"),
        material="support_paint",
        name="support_body",
    )

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_FLOOR_THICKNESS)),
        material="outer_steel",
        origin=Origin(xyz=(GUIDE_LENGTH / 2.0, 0.0, GUIDE_FLOOR_CENTER_Z)),
        name="guide_floor",
    )
    outer_guide.visual(
        Box((GUIDE_LENGTH, GUIDE_SIDE_THICKNESS, GUIDE_SIDE_HEIGHT)),
        material="outer_steel",
        origin=Origin(
            xyz=(GUIDE_LENGTH / 2.0, GUIDE_SIDE_CENTER_Y, GUIDE_SIDE_CENTER_Z)
        ),
        name="guide_left_wall",
    )
    outer_guide.visual(
        Box((GUIDE_LENGTH, GUIDE_SIDE_THICKNESS, GUIDE_SIDE_HEIGHT)),
        material="outer_steel",
        origin=Origin(
            xyz=(GUIDE_LENGTH / 2.0, -GUIDE_SIDE_CENTER_Y, GUIDE_SIDE_CENTER_Z)
        ),
        name="guide_right_wall",
    )
    outer_guide.visual(
        Box((GUIDE_LENGTH, GUIDE_LIP_WIDTH, GUIDE_LIP_THICKNESS)),
        material="outer_steel",
        origin=Origin(
            xyz=(GUIDE_LENGTH / 2.0, GUIDE_LIP_CENTER_Y, GUIDE_LIP_CENTER_Z)
        ),
        name="guide_left_lip",
    )
    outer_guide.visual(
        Box((GUIDE_LENGTH, GUIDE_LIP_WIDTH, GUIDE_LIP_THICKNESS)),
        material="outer_steel",
        origin=Origin(
            xyz=(GUIDE_LENGTH / 2.0, -GUIDE_LIP_CENTER_Y, GUIDE_LIP_CENTER_Z)
        ),
        name="guide_right_lip",
    )

    moving_section = model.part("moving_section")
    moving_section.visual(
        Box((SLIDER_LENGTH, SLIDER_BODY_WIDTH, SLIDER_BODY_HEIGHT)),
        material="inner_steel",
        origin=Origin(
            xyz=(SLIDER_LENGTH / 2.0, 0.0, SLIDER_BODY_CENTER_Z)
        ),
        name="slider_body",
    )
    moving_section.visual(
        Box((SLIDER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        material="inner_steel",
        origin=Origin(xyz=(SLIDER_LENGTH / 2.0, RUNNER_Y, RUNNER_CENTER_Z)),
        name="left_runner",
    )
    moving_section.visual(
        Box((SLIDER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        material="inner_steel",
        origin=Origin(xyz=(SLIDER_LENGTH / 2.0, -RUNNER_Y, RUNNER_CENTER_Z)),
        name="right_runner",
    )
    moving_section.visual(
        Box((FRONT_PAD_THICKNESS, FRONT_PAD_WIDTH, FRONT_PAD_HEIGHT)),
        material="inner_steel",
        origin=Origin(
            xyz=(SLIDER_LENGTH - FRONT_PAD_THICKNESS / 2.0, 0.0, 0.0)
        ),
        name="front_pad",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=wall_support,
        child=outer_guide,
        origin=Origin(xyz=(SUPPORT_TOTAL_DEPTH, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_moving",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=moving_section,
        origin=Origin(xyz=(SLIDE_HOME_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_support = object_model.get_part("wall_support")
    outer_guide = object_model.get_part("outer_guide")
    moving_section = object_model.get_part("moving_section")
    slide = object_model.get_articulation("outer_to_moving")

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

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            outer_guide,
            wall_support,
            name="outer_guide_is_carried_by_wall_support",
        )
        ctx.expect_contact(
            moving_section,
            outer_guide,
            name="moving_section_is_supported_by_outer_guide",
        )
        ctx.expect_overlap(
            moving_section,
            outer_guide,
            axes="yz",
            min_overlap=0.030,
            name="moving_section_is_nested_inside_guide_cross_section",
        )
        ctx.expect_gap(
            moving_section,
            wall_support,
            axis="x",
            min_gap=0.020,
            name="moving_section_projects_forward_of_wall_support",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            moving_section,
            outer_guide,
            name="moving_section_remains_supported_when_extended",
        )
        ctx.expect_overlap(
            moving_section,
            outer_guide,
            axes="yz",
            min_overlap=0.030,
            name="moving_section_stays_aligned_when_extended",
        )

    with ctx.pose({slide: 0.0}):
        home_position = ctx.part_world_position(moving_section)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        extended_position = ctx.part_world_position(moving_section)
    motion_ok = (
        home_position is not None
        and extended_position is not None
        and extended_position[0] > home_position[0] + 0.120
    )
    ctx.check(
        "prismatic_extension_moves_forward",
        motion_ok,
        details=(
            f"expected +x extension greater than 0.120 m, got "
            f"{None if home_position is None or extended_position is None else extended_position[0] - home_position[0]:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
