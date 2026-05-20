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


BASE_L = 0.220
BASE_W = 0.160
BASE_T = 0.018
BASE_BODY_L = 0.110
BASE_BODY_W = 0.094
BASE_BODY_H = 0.058

SHOULDER_AXIS_Z = BASE_T + BASE_BODY_H + 0.030
SHOULDER_GAP = 0.056
SHOULDER_FORK_T = 0.012
SHOULDER_FORK_L = 0.042
SHOULDER_FORK_H = 0.060
SHOULDER_BARREL_R = 0.020

UPPER_LEN = 0.220
UPPER_HUB_L = 0.032
UPPER_HUB_W = 0.036
UPPER_HUB_H = 0.040
UPPER_BEAM_L = 0.156
UPPER_BEAM_W = 0.040
UPPER_BEAM_H = 0.034

ELBOW_GAP = 0.046
ELBOW_FORK_T = 0.010
ELBOW_FORK_L = 0.040
ELBOW_FORK_H = 0.048
ELBOW_ROOT_L = 0.014
ELBOW_BARREL_R = 0.018

DISTAL_LEN = 0.150
DISTAL_HUB_L = 0.026
DISTAL_HUB_W = 0.034
DISTAL_HUB_H = 0.038
DISTAL_BEAM_L = 0.074
DISTAL_BEAM_W = 0.032
DISTAL_BEAM_H = 0.028
DISTAL_GUIDE_L = DISTAL_LEN - DISTAL_HUB_L - DISTAL_BEAM_L
DISTAL_GUIDE_W = 0.048
DISTAL_GUIDE_H = 0.042

SLIDER_TRAVEL = 0.075
SLIDER_RAIL_LEN = 0.062
SLIDER_RAIL_W = 0.024
SLIDER_RAIL_H = 0.020
SLIDER_FACE_T = 0.008
SLIDER_FACE_W = 0.036
SLIDER_FACE_H = 0.030


def _aabb_size(aabb):
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_elbow_arm")

    model.material("dark_base", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("graphite", rgba=(0.27, 0.30, 0.34, 1.0))
    model.material("machined_alloy", rgba=(0.68, 0.71, 0.75, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_L, BASE_W, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)),
        material="dark_base",
        name="base_plate",
    )
    base.visual(
        Box((BASE_BODY_L, BASE_BODY_W, BASE_BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_T + (BASE_BODY_H / 2.0))),
        material="dark_base",
        name="base_pedestal",
    )
    base.visual(
        Box((SHOULDER_FORK_L, SHOULDER_FORK_T, SHOULDER_FORK_H)),
        origin=Origin(
            xyz=(
                0.0,
                (SHOULDER_GAP / 2.0) + (SHOULDER_FORK_T / 2.0),
                SHOULDER_AXIS_Z,
            )
        ),
        material="dark_base",
        name="shoulder_fork_left",
    )
    base.visual(
        Box((SHOULDER_FORK_L, SHOULDER_FORK_T, SHOULDER_FORK_H)),
        origin=Origin(
            xyz=(
                0.0,
                -(SHOULDER_GAP / 2.0) - (SHOULDER_FORK_T / 2.0),
                SHOULDER_AXIS_Z,
            )
        ),
        material="dark_base",
        name="shoulder_fork_right",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, SHOULDER_AXIS_Z + 0.030)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (SHOULDER_AXIS_Z + 0.030) / 2.0)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=SHOULDER_BARREL_R, length=SHOULDER_GAP),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="graphite",
        name="shoulder_barrel",
    )
    upper_link.visual(
        Box((UPPER_HUB_L, UPPER_HUB_W, UPPER_HUB_H)),
        origin=Origin(xyz=(UPPER_HUB_L / 2.0, 0.0, 0.0)),
        material="graphite",
        name="upper_hub",
    )
    upper_link.visual(
        Box((UPPER_BEAM_L, UPPER_BEAM_W, UPPER_BEAM_H)),
        origin=Origin(
            xyz=(
                UPPER_HUB_L + (UPPER_BEAM_L / 2.0),
                0.0,
                0.0,
            )
        ),
        material="graphite",
        name="upper_beam",
    )
    upper_link.visual(
        Box((ELBOW_ROOT_L, 0.040, 0.040)),
        origin=Origin(
            xyz=(
                UPPER_LEN - ELBOW_FORK_L - (ELBOW_ROOT_L / 2.0) + 0.004,
                0.0,
                0.0,
            )
        ),
        material="graphite",
        name="elbow_root",
    )
    upper_link.visual(
        Box((0.030, ELBOW_GAP + (2.0 * ELBOW_FORK_T), 0.022)),
        origin=Origin(
            xyz=(
                UPPER_LEN - ELBOW_FORK_L - 0.006,
                0.0,
                0.0,
            )
        ),
        material="graphite",
        name="elbow_bridge",
    )
    upper_link.visual(
        Box((ELBOW_FORK_L, ELBOW_FORK_T, ELBOW_FORK_H)),
        origin=Origin(
            xyz=(
                UPPER_LEN - (ELBOW_FORK_L / 2.0),
                (ELBOW_GAP / 2.0) + (ELBOW_FORK_T / 2.0),
                0.0,
            )
        ),
        material="graphite",
        name="elbow_fork_left",
    )
    upper_link.visual(
        Box((ELBOW_FORK_L, ELBOW_FORK_T, ELBOW_FORK_H)),
        origin=Origin(
            xyz=(
                UPPER_LEN - (ELBOW_FORK_L / 2.0),
                -(ELBOW_GAP / 2.0) - (ELBOW_FORK_T / 2.0),
                0.0,
            )
        ),
        material="graphite",
        name="elbow_fork_right",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((UPPER_LEN, 0.066, 0.060)),
        mass=2.0,
        origin=Origin(xyz=(UPPER_LEN / 2.0, 0.0, 0.0)),
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        Cylinder(radius=ELBOW_BARREL_R, length=ELBOW_GAP),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="machined_alloy",
        name="elbow_barrel",
    )
    distal_link.visual(
        Box((DISTAL_HUB_L, DISTAL_HUB_W, DISTAL_HUB_H)),
        origin=Origin(xyz=(DISTAL_HUB_L / 2.0, 0.0, 0.0)),
        material="machined_alloy",
        name="distal_hub",
    )
    distal_link.visual(
        Box((DISTAL_BEAM_L, DISTAL_BEAM_W, DISTAL_BEAM_H)),
        origin=Origin(
            xyz=(
                DISTAL_HUB_L + (DISTAL_BEAM_L / 2.0),
                0.0,
                0.0,
            )
        ),
        material="machined_alloy",
        name="distal_beam",
    )
    distal_link.visual(
        Box((DISTAL_GUIDE_L, DISTAL_GUIDE_W, DISTAL_GUIDE_H)),
        origin=Origin(
            xyz=(
                DISTAL_HUB_L + DISTAL_BEAM_L + (DISTAL_GUIDE_L / 2.0),
                0.0,
                0.0,
            )
        ),
        material="machined_alloy",
        name="distal_guide",
    )
    distal_link.inertial = Inertial.from_geometry(
        Box((DISTAL_LEN, DISTAL_GUIDE_W, 0.048)),
        mass=1.2,
        origin=Origin(xyz=(DISTAL_LEN / 2.0, 0.0, 0.0)),
    )

    tip_slider = model.part("tip_slider")
    tip_slider.visual(
        Box((SLIDER_RAIL_LEN, SLIDER_RAIL_W, SLIDER_RAIL_H)),
        origin=Origin(xyz=(SLIDER_RAIL_LEN / 2.0, 0.0, 0.0)),
        material="machined_alloy",
        name="tip_rail",
    )
    tip_slider.visual(
        Box((SLIDER_FACE_T, SLIDER_FACE_W, SLIDER_FACE_H)),
        origin=Origin(
            xyz=(SLIDER_RAIL_LEN + (SLIDER_FACE_T / 2.0), 0.0, 0.0)
        ),
        material="machined_alloy",
        name="tip_face",
    )
    tip_slider.inertial = Inertial.from_geometry(
        Box((SLIDER_RAIL_LEN + SLIDER_FACE_T, SLIDER_FACE_W, SLIDER_FACE_H)),
        mass=0.4,
        origin=Origin(xyz=((SLIDER_RAIL_LEN + SLIDER_FACE_T) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_upper",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=1.05,
            effort=35.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "upper_to_distal",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=distal_link,
        origin=Origin(xyz=(UPPER_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.10,
            upper=0.85,
            effort=24.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "distal_to_tip_slider",
        ArticulationType.PRISMATIC,
        parent=distal_link,
        child=tip_slider,
        origin=Origin(xyz=(DISTAL_LEN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDER_TRAVEL,
            effort=12.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_link = object_model.get_part("upper_link")
    distal_link = object_model.get_part("distal_link")
    tip_slider = object_model.get_part("tip_slider")

    shoulder = object_model.get_articulation("base_to_upper")
    elbow = object_model.get_articulation("upper_to_distal")
    slider = object_model.get_articulation("distal_to_tip_slider")

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
        "joint_stack_matches_prompt",
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and slider.articulation_type == ArticulationType.PRISMATIC
        and shoulder.axis == (0.0, -1.0, 0.0)
        and elbow.axis == (0.0, -1.0, 0.0)
        and slider.axis == (1.0, 0.0, 0.0),
        details="Expected two pitch revolutes followed by a forward prismatic tip slider.",
    )

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base_is_grounded",
        base_aabb is not None and abs(base_aabb[0][2]) <= 1e-6,
        details=f"Base should sit on z=0, got {base_aabb}.",
    )

    ctx.expect_contact(base, upper_link, name="shoulder_joint_is_physically_supported")
    ctx.expect_contact(upper_link, distal_link, name="elbow_joint_is_physically_supported")
    ctx.expect_contact(distal_link, tip_slider, name="tip_slider_is_seated_on_distal_link")
    ctx.expect_gap(
        tip_slider,
        distal_link,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        name="tip_slider_starts_flush_with_guide_face",
    )

    upper_size = _aabb_size(ctx.part_world_aabb(upper_link))
    distal_size = _aabb_size(ctx.part_world_aabb(distal_link))
    tip_size = _aabb_size(ctx.part_world_aabb(tip_slider))
    ctx.check(
        "distal_link_is_shorter_than_upper_link",
        distal_size[0] < upper_size[0],
        details=f"Upper link size {upper_size}, distal link size {distal_size}.",
    )
    ctx.check(
        "tip_slider_is_clearly_smaller_than_links",
        tip_size[0] < distal_size[0]
        and tip_size[0] < upper_size[0]
        and tip_size[1] < distal_size[1]
        and tip_size[1] < upper_size[1]
        and tip_size[2] < distal_size[2]
        and tip_size[2] < upper_size[2],
        details=(
            f"Upper {upper_size}, distal {distal_size}, tip slider {tip_size}; "
            "the slider should stay visibly smaller than both links."
        ),
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, slider: 0.0}):
        tip_rest = ctx.part_world_position(tip_slider)
        distal_rest = ctx.part_world_position(distal_link)
    with ctx.pose({shoulder: shoulder.motion_limits.upper, elbow: 0.0, slider: 0.0}):
        distal_raised = ctx.part_world_position(distal_link)
    ctx.check(
        "positive_shoulder_motion_raises_the_chain",
        tip_rest is not None
        and distal_rest is not None
        and distal_raised is not None
        and distal_raised[2] > distal_rest[2] + 0.05,
        details=(
            f"Rest distal origin={distal_rest}, raised distal origin={distal_raised}; "
            "positive shoulder motion should lift the elbow."
        ),
    )

    with ctx.pose({shoulder: 0.25, elbow: 0.0, slider: 0.0}):
        tip_at_neutral_elbow = ctx.part_world_position(tip_slider)
    with ctx.pose({shoulder: 0.25, elbow: elbow.motion_limits.upper, slider: 0.0}):
        tip_at_raised_elbow = ctx.part_world_position(tip_slider)
    ctx.check(
        "positive_elbow_motion_raises_the_tip",
        tip_at_neutral_elbow is not None
        and tip_at_raised_elbow is not None
        and tip_at_raised_elbow[2] > tip_at_neutral_elbow[2] + 0.03,
        details=(
            f"Neutral tip origin={tip_at_neutral_elbow}, raised tip origin={tip_at_raised_elbow}; "
            "positive elbow motion should lift the distal stage."
        ),
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, slider: slider.motion_limits.upper}):
        tip_extended = ctx.part_world_position(tip_slider)
        ctx.expect_gap(
            tip_slider,
            distal_link,
            axis="x",
            min_gap=0.06,
            max_gap=0.09,
            name="tip_slider_extends_forward_from_distal_link",
        )
    ctx.check(
        "positive_prismatic_motion_moves_tip_forward",
        tip_rest is not None and tip_extended is not None and tip_extended[0] > tip_rest[0] + 0.06,
        details=f"Rest tip origin={tip_rest}, extended tip origin={tip_extended}.",
    )

    with ctx.pose(
        {
            shoulder: 0.85,
            elbow: 0.40,
            slider: slider.motion_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_compact_service_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
