from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import dist, pi

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


BASE_LENGTH = 0.22
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.016
BRACKET_CHEEK_GAP = 0.074
BRACKET_CHEEK_THICKNESS = 0.016
SHOULDER_HEIGHT = 0.118

UPPER_LINK_LENGTH = 0.30
UPPER_BEAM_LENGTH = 0.245
UPPER_BEAM_WIDTH = 0.026
UPPER_BEAM_HEIGHT = 0.034
UPPER_HUB_RADIUS = 0.022
UPPER_HUB_LENGTH = BRACKET_CHEEK_GAP
ELBOW_FORK_LENGTH = 0.052
ELBOW_FORK_GAP = 0.030
ELBOW_FORK_PLATE_THICKNESS = 0.012
ELBOW_FORK_HEIGHT = 0.052

FOREARM_LENGTH = 0.295
FOREARM_BEAM_LENGTH = 0.205
FOREARM_BEAM_WIDTH = 0.024
FOREARM_BEAM_HEIGHT = 0.028
FOREARM_HUB_RADIUS = 0.018
FOREARM_HUB_LENGTH = ELBOW_FORK_GAP
SLIDER_GUIDE_ORIGIN_X = 0.215
SLIDER_HOUSING_LENGTH = 0.105
SLIDER_HOUSING_WIDTH = 0.048
SLIDER_HOUSING_HEIGHT = 0.046
SLIDER_HOUSING_INNER_WIDTH = 0.028
SLIDER_HOUSING_INNER_HEIGHT = 0.024

SLIDER_TRAVEL = 0.080
SLIDER_BODY_LENGTH = 0.090
SLIDER_BODY_WIDTH = SLIDER_HOUSING_INNER_WIDTH
SLIDER_BODY_HEIGHT = 0.018
SLIDER_HEAD_LENGTH = 0.018
SLIDER_HEAD_WIDTH = SLIDER_HOUSING_INNER_WIDTH
SLIDER_HEAD_HEIGHT = 0.024

SHOULDER_LIMITS = MotionLimits(effort=40.0, velocity=1.5, lower=-0.9, upper=1.2)
ELBOW_LIMITS = MotionLimits(effort=28.0, velocity=1.8, lower=0.0, upper=1.45)
SLIDER_LIMITS = MotionLimits(effort=20.0, velocity=0.20, lower=0.0, upper=SLIDER_TRAVEL)


def _base_bracket_shape() -> cq.Workplane:
    cheek_height = 0.066
    cheek_y = BRACKET_CHEEK_GAP / 2.0 + BRACKET_CHEEK_THICKNESS / 2.0

    plate = cq.Workplane("XY").box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS).translate(
        (0.0, 0.0, BASE_THICKNESS / 2.0)
    )
    pedestal = cq.Workplane("XY").box(0.082, 0.060, 0.036).translate((-0.030, 0.0, 0.034))
    bridge = cq.Workplane("XY").box(
        0.040,
        BRACKET_CHEEK_GAP + 2.0 * BRACKET_CHEEK_THICKNESS,
        0.016,
    ).translate((-0.028, 0.0, 0.070))
    cheek = cq.Workplane("XY").box(0.032, BRACKET_CHEEK_THICKNESS, cheek_height).translate(
        (0.0, cheek_y, SHOULDER_HEIGHT - cheek_height / 2.0)
    )
    opposite_cheek = cheek.mirror("XZ")
    rear_strut = cq.Workplane("XY").box(0.050, BRACKET_CHEEK_THICKNESS, 0.054).translate(
        (-0.034, cheek_y, 0.047)
    )
    opposite_rear_strut = rear_strut.mirror("XZ")
    bracket = (
        plate.union(pedestal)
        .union(bridge)
        .union(cheek)
        .union(opposite_cheek)
        .union(rear_strut)
        .union(opposite_rear_strut)
    )

    for x_pos in (-0.074, 0.074):
        for y_pos in (-0.043, 0.043):
            hole = (
                cq.Workplane("XY")
                .circle(0.006)
                .extrude(BASE_THICKNESS + 0.004)
                .translate((x_pos, y_pos, -0.002))
            )
            bracket = bracket.cut(hole)

    return bracket


def _upper_link_shape() -> cq.Workplane:
    shoulder_knuckle = cq.Workplane("XY").box(0.030, UPPER_HUB_LENGTH, 0.036)
    root_neck = cq.Workplane("XY").box(0.024, 0.032, 0.030).translate((0.026, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.220, UPPER_BEAM_WIDTH, 0.032).translate((0.135, 0.0, 0.0))
    fork_y = ELBOW_FORK_GAP / 2.0 + ELBOW_FORK_PLATE_THICKNESS / 2.0
    yoke_root = cq.Workplane("XY").box(0.024, 0.018, 0.030).translate((0.252, 0.0, 0.0))
    fork_plate = cq.Workplane("XY").box(
        0.036,
        ELBOW_FORK_PLATE_THICKNESS,
        0.046,
    ).translate((0.272, fork_y, 0.0))
    opposite_fork_plate = fork_plate.mirror("XZ")

    return (
        shoulder_knuckle.union(root_neck)
        .union(beam)
        .union(yoke_root)
        .union(fork_plate)
        .union(opposite_fork_plate)
    )


def _forearm_link_shape() -> cq.Workplane:
    elbow_knuckle = cq.Workplane("XY").box(0.032, FOREARM_HUB_LENGTH, 0.034)
    root_neck = cq.Workplane("XY").box(0.024, 0.020, 0.024).translate((0.026, 0.0, 0.0))
    beam = cq.Workplane("XY").box(0.188, FOREARM_BEAM_WIDTH, FOREARM_BEAM_HEIGHT).translate(
        (0.132, 0.0, 0.0)
    )
    guide_block = cq.Workplane("XY").box(0.024, 0.040, 0.026).translate((0.214, 0.0, 0.0))
    rail_y = SLIDER_BODY_WIDTH / 2.0 + 0.004
    guide_rail = cq.Workplane("XY").box(0.090, 0.008, 0.024).translate((0.260, rail_y, 0.0))
    opposite_guide_rail = guide_rail.mirror("XZ")

    return (
        elbow_knuckle.union(root_neck)
        .union(beam)
        .union(guide_block)
        .union(guide_rail)
        .union(opposite_guide_rail)
    )


def _slider_stage_shape() -> cq.Workplane:
    rail = cq.Workplane("XY").box(
        SLIDER_BODY_LENGTH, SLIDER_BODY_WIDTH, SLIDER_BODY_HEIGHT
    ).translate((SLIDER_BODY_LENGTH / 2.0, 0.0, 0.0))
    head = cq.Workplane("XY").box(
        SLIDER_HEAD_LENGTH, SLIDER_HEAD_WIDTH, SLIDER_HEAD_HEIGHT
    ).translate((SLIDER_BODY_LENGTH + SLIDER_HEAD_LENGTH / 2.0, 0.0, 0.0))
    return rail.union(head)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shoulder_elbow_extension_chain")

    model.material("powder_steel", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("anodized_gray", rgba=(0.52, 0.56, 0.60, 1.0))
    model.material("graphite", rgba=(0.18, 0.19, 0.22, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        material="powder_steel",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        name="base_bracket_body",
    )
    base_bracket.visual(
        Box((0.082, 0.060, 0.036)),
        material="powder_steel",
        origin=Origin(xyz=(-0.030, 0.0, 0.034)),
        name="base_pedestal",
    )
    base_bracket.visual(
        Box((0.024, 0.032, 0.022)),
        material="powder_steel",
        origin=Origin(xyz=(-0.028, 0.0, 0.059)),
        name="base_column",
    )
    base_bracket.visual(
        Box((0.040, BRACKET_CHEEK_GAP + 2.0 * BRACKET_CHEEK_THICKNESS, 0.016)),
        material="powder_steel",
        origin=Origin(xyz=(-0.028, 0.0, 0.070)),
        name="base_bridge",
    )
    for side, y_pos in (("left", 0.045), ("right", -0.045)):
        base_bracket.visual(
            Box((0.032, BRACKET_CHEEK_THICKNESS, 0.066)),
            material="powder_steel",
            origin=Origin(xyz=(0.0, y_pos, 0.085)),
            name=f"shoulder_cheek_{side}",
        )
        base_bracket.visual(
            Box((0.050, BRACKET_CHEEK_THICKNESS, 0.054)),
            material="powder_steel",
            origin=Origin(xyz=(-0.034, y_pos, 0.047)),
            name=f"shoulder_strut_{side}",
        )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.018, length=UPPER_HUB_LENGTH),
        material="machined_aluminum",
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="shoulder_barrel",
    )
    upper_link.visual(
        Box((0.028, 0.032, 0.030)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        name="upper_root_neck",
    )
    upper_link.visual(
        Box((0.220, UPPER_BEAM_WIDTH, 0.032)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.024, 0.030, 0.030)),
        material="machined_aluminum",
        origin=Origin(xyz=(0.252, 0.0, 0.0)),
        name="elbow_yoke_root",
    )
    for side, y_pos in (("left", 0.021), ("right", -0.021)):
        upper_link.visual(
            Box((0.028, ELBOW_FORK_PLATE_THICKNESS, 0.046)),
            material="machined_aluminum",
            origin=Origin(xyz=(0.274, y_pos, 0.0)),
            name=f"elbow_fork_{side}",
        )

    forearm_link = model.part("forearm_link")
    forearm_link.visual(
        Cylinder(radius=0.014, length=FOREARM_HUB_LENGTH),
        material="anodized_gray",
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        name="elbow_barrel",
    )
    forearm_link.visual(
        Box((0.022, 0.018, 0.022)),
        material="anodized_gray",
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        name="forearm_root_neck",
    )
    forearm_link.visual(
        Box((0.176, FOREARM_BEAM_WIDTH, FOREARM_BEAM_HEIGHT)),
        material="anodized_gray",
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        name="forearm_beam",
    )
    forearm_link.visual(
        Box((0.024, 0.040, 0.026)),
        material="anodized_gray",
        origin=Origin(xyz=(0.203, 0.0, 0.0)),
        name="tip_guide_block",
    )
    forearm_link.visual(
        Box((0.090, SLIDER_BODY_WIDTH, 0.004)),
        material="anodized_gray",
        origin=Origin(xyz=(0.260, 0.0, -0.011)),
        name="slider_guide_floor",
    )

    slider_stage = model.part("slider_stage")
    slider_stage.visual(
        Box((SLIDER_BODY_LENGTH, SLIDER_BODY_WIDTH, SLIDER_BODY_HEIGHT)),
        material="graphite",
        origin=Origin(xyz=(SLIDER_BODY_LENGTH / 2.0, 0.0, 0.0)),
        name="slider_rail",
    )
    slider_stage.visual(
        Box((SLIDER_HEAD_LENGTH, SLIDER_HEAD_WIDTH, SLIDER_HEAD_HEIGHT)),
        material="graphite",
        origin=Origin(
            xyz=(
                SLIDER_BODY_LENGTH + SLIDER_HEAD_LENGTH / 2.0,
                0.0,
                (SLIDER_HEAD_HEIGHT - SLIDER_BODY_HEIGHT) / 2.0,
            )
        ),
        name="slider_head",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=SHOULDER_LIMITS,
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm_link,
        origin=Origin(xyz=(UPPER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=ELBOW_LIMITS,
    )
    model.articulation(
        "tip_extension",
        ArticulationType.PRISMATIC,
        parent=forearm_link,
        child=slider_stage,
        origin=Origin(xyz=(SLIDER_GUIDE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=SLIDER_LIMITS,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    upper_link = object_model.get_part("upper_link")
    forearm_link = object_model.get_part("forearm_link")
    slider_stage = object_model.get_part("slider_stage")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    tip_extension = object_model.get_articulation("tip_extension")

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

    ctx.expect_contact(
        upper_link,
        base_bracket,
        contact_tol=0.0006,
        name="shoulder hub captured by base bracket",
    )
    ctx.expect_contact(
        forearm_link,
        upper_link,
        contact_tol=0.0006,
        name="elbow hub seated in upper fork",
    )

    with ctx.pose({tip_extension: 0.0}):
        ctx.expect_within(
            slider_stage,
            forearm_link,
            axes="yz",
            margin=0.004,
            name="retracted slider stays within guide width and height",
        )
        ctx.expect_overlap(
            slider_stage,
            forearm_link,
            axes="yz",
            min_overlap=0.020,
            name="retracted slider shares forearm guide footprint",
        )

    with ctx.pose({tip_extension: SLIDER_TRAVEL}):
        ctx.expect_within(
            slider_stage,
            forearm_link,
            axes="yz",
            margin=0.004,
            name="extended slider remains aligned to guide channel",
        )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0}):
        upper_closed_aabb = ctx.part_world_aabb(upper_link)
    with ctx.pose({shoulder_joint: SHOULDER_LIMITS.upper, elbow_joint: 0.0}):
        upper_raised_aabb = ctx.part_world_aabb(upper_link)

    shoulder_lifts = (
        upper_closed_aabb is not None
        and upper_raised_aabb is not None
        and upper_raised_aabb[1][2] > upper_closed_aabb[1][2] + 0.12
    )
    ctx.check(
        "shoulder positive rotation raises the upper link",
        shoulder_lifts,
        details=(
            f"closed={upper_closed_aabb}, raised={upper_raised_aabb}, "
            f"expected max-z increase > 0.12 m"
        ),
    )

    with ctx.pose({shoulder_joint: 0.30, elbow_joint: 0.0}):
        forearm_straight_aabb = ctx.part_world_aabb(forearm_link)
    with ctx.pose({shoulder_joint: 0.30, elbow_joint: ELBOW_LIMITS.upper}):
        forearm_flexed_aabb = ctx.part_world_aabb(forearm_link)

    elbow_flexes_up = (
        forearm_straight_aabb is not None
        and forearm_flexed_aabb is not None
        and forearm_flexed_aabb[1][2] > forearm_straight_aabb[1][2] + 0.08
    )
    ctx.check(
        "elbow positive rotation folds the forearm upward",
        elbow_flexes_up,
        details=(
            f"straight={forearm_straight_aabb}, flexed={forearm_flexed_aabb}, "
            f"expected max-z increase > 0.08 m"
        ),
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, tip_extension: 0.0}):
        slider_retracted_pos = ctx.part_world_position(slider_stage)
    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, tip_extension: SLIDER_TRAVEL}):
        slider_extended_pos = ctx.part_world_position(slider_stage)

    slider_extends = (
        slider_retracted_pos is not None
        and slider_extended_pos is not None
        and slider_extended_pos[0] > slider_retracted_pos[0] + 0.06
        and dist(slider_retracted_pos, slider_extended_pos) > 0.07
    )
    ctx.check(
        "prismatic tip stage extends along the distal link axis",
        slider_extends,
        details=(
            f"retracted={slider_retracted_pos}, extended={slider_extended_pos}, "
            f"expected > 0.06 m outward x-travel"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
