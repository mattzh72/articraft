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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LINK1_LENGTH = 0.252
SHOULDER_AXIS = (0.0, -1.0, 0.0)
ELBOW_AXIS = (0.0, -1.0, 0.0)

LINK1_THICK = 0.022
LINK2_THICK = 0.018
SUPPORT_CHEEK_THICK = 0.006
LINK1_FORK_CHEEK_THICK = 0.007

HOUSING_OUTER_Y = 0.022
SLIDER_THICK = 0.010
SLIDE_STROKE = 0.055
SLIDER_JOINT_Z = -0.180


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_dual_link_slider_arm")

    model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("aluminum", rgba=(0.70, 0.72, 0.76, 1.0))
    model.material("anodized_stage", rgba=(0.26, 0.28, 0.31, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.180, 0.100, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material="charcoal",
        name="top_plate",
    )
    support.visual(
        Box((0.062, 0.040, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material="charcoal",
        name="hanger_bridge",
    )
    support.visual(
        Box((0.034, SUPPORT_CHEEK_THICK, 0.066)),
        origin=Origin(xyz=(0.0, 0.014, 0.015)),
        material="charcoal",
        name="left_hanger_cheek",
    )
    support.visual(
        Box((0.034, SUPPORT_CHEEK_THICK, 0.066)),
        origin=Origin(xyz=(0.0, -0.014, 0.015)),
        material="charcoal",
        name="right_hanger_cheek",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.180, 0.100, 0.100)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    link1 = model.part("link1")
    link1.visual(
        Box((0.030, LINK1_THICK, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material="aluminum",
        name="shoulder_lug",
    )
    link1.visual(
        Cylinder(radius=0.018, length=LINK1_THICK),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="aluminum",
        name="shoulder_barrel",
    )
    link1.visual(
        Box((0.026, 0.020, 0.198)),
        origin=Origin(xyz=(0.0, 0.0, -0.133)),
        material="aluminum",
        name="upper_bar",
    )
    link1.visual(
        Box((0.022, LINK1_FORK_CHEEK_THICK, 0.080)),
        origin=Origin(xyz=(0.0, 0.0125, -0.272)),
        material="aluminum",
        name="left_fork_cheek",
    )
    link1.visual(
        Box((0.022, LINK1_FORK_CHEEK_THICK, 0.080)),
        origin=Origin(xyz=(0.0, -0.0125, -0.272)),
        material="aluminum",
        name="right_fork_cheek",
    )
    link1.inertial = Inertial.from_geometry(
        Box((0.050, LINK1_THICK, 0.270)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
    )

    link2 = model.part("link2")
    link2.visual(
        Box((0.022, LINK2_THICK, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material="aluminum",
        name="elbow_lug",
    )
    link2.visual(
        Cylinder(radius=0.015, length=LINK2_THICK),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="aluminum",
        name="elbow_barrel",
    )
    link2.visual(
        Box((0.024, 0.016, 0.126)),
        origin=Origin(xyz=(0.0, 0.0, -0.117)),
        material="aluminum",
        name="lower_bar",
    )
    link2.visual(
        Box((0.040, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.169)),
        material="aluminum",
        name="channel_top_bridge",
    )
    link2.visual(
        Box((0.006, 0.018, 0.080)),
        origin=Origin(xyz=(0.017, 0.0, -0.220)),
        material="aluminum",
        name="channel_left_rail",
    )
    link2.visual(
        Box((0.006, 0.018, 0.080)),
        origin=Origin(xyz=(-0.017, 0.0, -0.220)),
        material="aluminum",
        name="channel_right_rail",
    )
    link2.inertial = Inertial.from_geometry(
        Box((0.062, HOUSING_OUTER_Y, 0.270)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.028, SLIDER_THICK, 0.092)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material="anodized_stage",
        name="slider_rail",
    )
    slider.visual(
        Box((0.032, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.097)),
        material="anodized_stage",
        name="slider_tip",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.032, 0.014, 0.102)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
    )

    model.articulation(
        "support_to_link1",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link1,
        origin=Origin(),
        axis=SHOULDER_AXIS,
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.05,
        ),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(0.0, 0.0, -LINK1_LENGTH)),
        axis=ELBOW_AXIS,
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.4,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "link2_to_slider",
        ArticulationType.PRISMATIC,
        parent=link2,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, SLIDER_JOINT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.15,
            lower=0.0,
            upper=SLIDE_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    slider = object_model.get_part("slider")

    shoulder = object_model.get_articulation("support_to_link1")
    elbow = object_model.get_articulation("link1_to_link2")
    stage = object_model.get_articulation("link2_to_slider")

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

    with ctx.pose({shoulder: 0.0, elbow: 0.0, stage: 0.0}):
        ctx.expect_contact(link1, support, name="support_carries_upper_link")
        ctx.expect_contact(link2, link1, name="upper_link_carries_lower_link")
        ctx.expect_contact(slider, link2, name="slider_is_guided_when_retracted")
        ctx.expect_within(slider, link2, axes="xy", margin=0.0, name="slider_stays_within_housing_section")
        ctx.expect_overlap(slider, link2, axes="xy", min_overlap=0.010, name="slider_tracks_inside_housing_plan")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, stage: SLIDE_STROKE}):
        ctx.expect_contact(slider, link2, name="slider_remains_guided_at_full_extension")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, stage: 0.0}):
        link2_rest = ctx.part_world_position(link2)
        slider_rest = ctx.part_world_position(slider)

    with ctx.pose({shoulder: 0.60, elbow: 0.0, stage: 0.0}):
        link2_swung = ctx.part_world_position(link2)

    with ctx.pose({shoulder: 0.0, elbow: 0.75, stage: 0.0}):
        slider_bent = ctx.part_world_position(slider)

    with ctx.pose({shoulder: 0.0, elbow: 0.0, stage: SLIDE_STROKE}):
        slider_extended = ctx.part_world_position(slider)

    ctx.check(
        "shoulder_positive_motion_swings_arm_forward",
        (
            link2_rest is not None
            and link2_swung is not None
            and (link2_swung[0] > (link2_rest[0] + 0.10))
            and (link2_swung[2] > (link2_rest[2] + 0.03))
        ),
        details=f"rest={link2_rest}, swung={link2_swung}",
    )
    ctx.check(
        "elbow_positive_motion_advances_terminal_stage_forward",
        (
            slider_rest is not None
            and slider_bent is not None
            and (slider_bent[0] > (slider_rest[0] + 0.10))
        ),
        details=f"rest={slider_rest}, bent={slider_bent}",
    )
    ctx.check(
        "prismatic_stage_extends_downward_from_forearm",
        (
            slider_rest is not None
            and slider_extended is not None
            and abs(slider_extended[0] - slider_rest[0]) < 1e-6
            and abs(slider_extended[1] - slider_rest[1]) < 1e-6
            and (slider_extended[2] < (slider_rest[2] - 0.045))
        ),
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
