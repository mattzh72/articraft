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
)


BASE_LENGTH = 0.18
BASE_WIDTH = 0.09
BASE_THICKNESS = 0.012
PIVOT_X = -0.050
PIVOT_Z = 0.046
PEDESTAL_LENGTH = 0.052
PEDESTAL_WIDTH = 0.040
PEDESTAL_HEIGHT = 0.022
CHEEK_THICKNESS = 0.008
CHEEK_GAP = 0.014
CHEEK_LENGTH = 0.018
CHEEK_HEIGHT = 0.026

HINGE_BARREL_RADIUS = 0.006
HINGE_BARREL_LENGTH = CHEEK_GAP
GUIDE_WEB_LENGTH = 0.018
GUIDE_WEB_WIDTH = 0.012
GUIDE_WEB_HEIGHT = 0.012
GUIDE_WEB_CENTER_Z = -0.004
GUIDE_PLATE_START = 0.018
GUIDE_PLATE_LENGTH = 0.172
GUIDE_PLATE_WIDTH = 0.026
GUIDE_PLATE_THICKNESS = 0.006
GUIDE_PLATE_CENTER_Z = -0.011
GUIDE_RAIL_LENGTH = 0.132
GUIDE_RAIL_START = 0.034
GUIDE_RAIL_HEIGHT = 0.008
GUIDE_RAIL_INNER_WIDTH = 0.016
GUIDE_RAIL_THICKNESS = (GUIDE_PLATE_WIDTH - GUIDE_RAIL_INNER_WIDTH) / 2.0
GUIDE_RAIL_CENTER_Z = GUIDE_PLATE_CENTER_Z + GUIDE_PLATE_THICKNESS / 2.0 + GUIDE_RAIL_HEIGHT / 2.0

SLIDER_BODY_LENGTH = 0.132
SLIDER_BODY_WIDTH = 0.014
SLIDER_BODY_HEIGHT = 0.004
SLIDER_BODY_CENTER_Z = GUIDE_PLATE_CENTER_Z + GUIDE_PLATE_THICKNESS / 2.0 + SLIDER_BODY_HEIGHT / 2.0
SLIDER_STEM_LENGTH = 0.022
SLIDER_STEM_WIDTH = 0.008
SLIDER_STEM_HEIGHT = 0.012
SLIDER_STEM_CENTER_X = 0.138
SLIDER_STEM_CENTER_Z = 0.000
SLIDER_MOUNT_LENGTH = 0.014
SLIDER_MOUNT_WIDTH = 0.010
SLIDER_MOUNT_HEIGHT = 0.018
SLIDER_MOUNT_CENTER_Z = 0.005
PRISMATIC_ORIGIN_X = 0.040
SLIDE_MAX = 0.085

TIP_PIVOT_X = 0.146
TIP_TAB_LENGTH = 0.040
TIP_TAB_THICKNESS = 0.006
TIP_TAB_HEIGHT = 0.014
TIP_TAB_NOSE_RADIUS = 0.007


def _box(length: float, width: float, height: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center_xyz)


def _cylinder_y(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center_xyz)


def _base_support_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.006)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.060, -0.025),
                (-0.060, 0.025),
                (0.060, -0.025),
                (0.060, 0.025),
            ]
        )
        .hole(0.010)
    )

    backbone = _box(0.088, 0.024, 0.018, (-0.016, 0.0, BASE_THICKNESS + 0.009))
    pedestal = _box(
        PEDESTAL_LENGTH,
        PEDESTAL_WIDTH,
        PEDESTAL_HEIGHT,
        (PIVOT_X - 0.012, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0),
    )
    left_cheek = _box(
        CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CHEEK_HEIGHT,
        (
            PIVOT_X,
            CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0,
            PIVOT_Z,
        ),
    )
    right_cheek = _box(
        CHEEK_LENGTH,
        CHEEK_THICKNESS,
        CHEEK_HEIGHT,
        (
            PIVOT_X,
            -(CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0),
            PIVOT_Z,
        ),
    )

    return plate.union(backbone).union(pedestal).union(left_cheek).union(right_cheek)


def _outer_guide_shape() -> cq.Workplane:
    barrel = _cylinder_y(HINGE_BARREL_RADIUS, HINGE_BARREL_LENGTH, (0.0, 0.0, 0.0))
    web = _box(
        GUIDE_WEB_LENGTH,
        GUIDE_WEB_WIDTH,
        GUIDE_WEB_HEIGHT,
        (GUIDE_WEB_LENGTH / 2.0, 0.0, GUIDE_WEB_CENTER_Z),
    )
    guide_plate = _box(
        GUIDE_PLATE_LENGTH,
        GUIDE_PLATE_WIDTH,
        GUIDE_PLATE_THICKNESS,
        (GUIDE_PLATE_START + GUIDE_PLATE_LENGTH / 2.0, 0.0, GUIDE_PLATE_CENTER_Z),
    )
    left_rail = _box(
        GUIDE_RAIL_LENGTH,
        GUIDE_RAIL_THICKNESS,
        GUIDE_RAIL_HEIGHT,
        (
            GUIDE_RAIL_START + GUIDE_RAIL_LENGTH / 2.0,
            GUIDE_RAIL_INNER_WIDTH / 2.0 + GUIDE_RAIL_THICKNESS / 2.0,
            GUIDE_RAIL_CENTER_Z,
        ),
    )
    right_rail = _box(
        GUIDE_RAIL_LENGTH,
        GUIDE_RAIL_THICKNESS,
        GUIDE_RAIL_HEIGHT,
        (
            GUIDE_RAIL_START + GUIDE_RAIL_LENGTH / 2.0,
            -(GUIDE_RAIL_INNER_WIDTH / 2.0 + GUIDE_RAIL_THICKNESS / 2.0),
            GUIDE_RAIL_CENTER_Z,
        ),
    )
    return barrel.union(web).union(guide_plate).union(left_rail).union(right_rail)


def _slider_shape() -> cq.Workplane:
    body = _box(
        SLIDER_BODY_LENGTH,
        SLIDER_BODY_WIDTH,
        SLIDER_BODY_HEIGHT,
        (SLIDER_BODY_LENGTH / 2.0, 0.0, SLIDER_BODY_CENTER_Z),
    )
    stem = _box(
        SLIDER_STEM_LENGTH,
        SLIDER_STEM_WIDTH,
        SLIDER_STEM_HEIGHT,
        (SLIDER_STEM_CENTER_X, 0.0, SLIDER_STEM_CENTER_Z),
    )
    mount = _box(
        SLIDER_MOUNT_LENGTH,
        SLIDER_MOUNT_WIDTH,
        SLIDER_MOUNT_HEIGHT,
        (TIP_PIVOT_X - SLIDER_MOUNT_LENGTH / 2.0, 0.0, SLIDER_MOUNT_CENTER_Z),
    )
    return body.union(stem).union(mount)


def _tip_fork_shape() -> cq.Workplane:
    tab = _box(TIP_TAB_LENGTH, TIP_TAB_THICKNESS, TIP_TAB_HEIGHT, (TIP_TAB_LENGTH / 2.0, 0.0, 0.0))
    nose = _cylinder_y(TIP_TAB_NOSE_RADIUS, TIP_TAB_THICKNESS, (TIP_TAB_LENGTH, 0.0, 0.0))
    lightening_slot = (
        cq.Workplane("XZ")
        .center(TIP_TAB_LENGTH * 0.72, 0.0)
        .slot2D(0.014, 0.004)
        .extrude(TIP_TAB_THICKNESS / 2.0, both=True)
    )
    return tab.union(nose).cut(lightening_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinge_slide_hinge_chain")

    model.material("powder_coat", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("guide_gray", rgba=(0.31, 0.35, 0.40, 1.0))
    model.material("slider_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("safety_orange", rgba=(0.86, 0.42, 0.14, 1.0))

    base_support = model.part("base_support")
    base_support.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        material="powder_coat",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        name="base_plate",
    )
    base_support.visual(
        Box((0.032, 0.036, 0.022)),
        material="powder_coat",
        origin=Origin(xyz=(-0.076, 0.0, 0.023)),
        name="rear_pedestal",
    )
    base_support.visual(
        Box((0.018, CHEEK_THICKNESS, 0.026)),
        material="powder_coat",
        origin=Origin(xyz=(PIVOT_X, CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0, PIVOT_Z)),
        name="left_cheek",
    )
    base_support.visual(
        Box((0.018, CHEEK_THICKNESS, 0.026)),
        material="powder_coat",
        origin=Origin(xyz=(PIVOT_X, -(CHEEK_GAP / 2.0 + CHEEK_THICKNESS / 2.0), PIVOT_Z)),
        name="right_cheek",
    )
    base_support.visual(
        Box((0.010, CHEEK_GAP + 2.0 * CHEEK_THICKNESS, 0.012)),
        material="powder_coat",
        origin=Origin(xyz=(PIVOT_X - 0.009, 0.0, 0.036)),
        name="cheek_bridge",
    )

    outer_guide = model.part("outer_guide")
    outer_guide.visual(
        Box((0.014, CHEEK_GAP, 0.012)),
        material="guide_gray",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="hinge_tongue",
    )
    outer_guide.visual(
        Box((0.020, 0.012, 0.022)),
        material="guide_gray",
        origin=Origin(xyz=(0.017, 0.0, -0.011)),
        name="drop_web",
    )
    outer_guide.visual(
        Box((0.172, 0.026, 0.006)),
        material="guide_gray",
        origin=Origin(xyz=(0.104, 0.0, -0.019)),
        name="guide_plate",
    )
    outer_guide.visual(
        Box((0.132, 0.005, 0.008)),
        material="guide_gray",
        origin=Origin(xyz=(0.100, 0.0105, -0.012)),
        name="left_rail",
    )
    outer_guide.visual(
        Box((0.132, 0.005, 0.008)),
        material="guide_gray",
        origin=Origin(xyz=(0.100, -0.0105, -0.012)),
        name="right_rail",
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.132, 0.012, 0.004)),
        material="slider_silver",
        origin=Origin(xyz=(0.066, 0.0, -0.014)),
        name="carrier",
    )
    slider.visual(
        Box((0.018, 0.008, 0.012)),
        material="slider_silver",
        origin=Origin(xyz=(0.133, 0.0, -0.006)),
        name="stem",
    )
    slider.visual(
        Box((0.014, 0.010, 0.018)),
        material="slider_silver",
        origin=Origin(xyz=(TIP_PIVOT_X - 0.007, 0.0, 0.008)),
        name="mount_block",
    )

    tip_fork = model.part("tip_fork")
    tip_fork.visual(
        Box((TIP_TAB_LENGTH, TIP_TAB_THICKNESS, TIP_TAB_HEIGHT)),
        material="safety_orange",
        origin=Origin(xyz=(TIP_TAB_LENGTH / 2.0, 0.0, 0.0)),
        name="tab",
    )

    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=outer_guide,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=outer_guide,
        child=slider,
        origin=Origin(xyz=(PRISMATIC_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.20, lower=0.0, upper=SLIDE_MAX),
    )
    model.articulation(
        "tip_hinge",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=tip_fork,
        origin=Origin(xyz=(TIP_PIVOT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.45, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    outer_guide = object_model.get_part("outer_guide")
    slider = object_model.get_part("slider")
    tip_fork = object_model.get_part("tip_fork")
    base_hinge = object_model.get_articulation("base_hinge")
    guide_slide = object_model.get_articulation("guide_slide")
    tip_hinge = object_model.get_articulation("tip_hinge")

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

    with ctx.pose({base_hinge: 0.0, guide_slide: 0.0, tip_hinge: 0.0}):
        ctx.expect_contact(base_support, outer_guide, name="root_hinge_is_supported")
        ctx.expect_contact(outer_guide, slider, name="slider_rests_on_guide")
        ctx.expect_contact(slider, tip_fork, name="tip_hinge_is_supported")
        ctx.expect_within(
            slider,
            outer_guide,
            axes="yz",
            margin=0.001,
            inner_elem="carrier",
            name="slider_fits_guide_profile",
        )

    with ctx.pose({base_hinge: 0.0, guide_slide: SLIDE_MAX, tip_hinge: 0.0}):
        ctx.expect_contact(outer_guide, slider, name="slider_stays_supported_at_full_extension")
        ctx.expect_overlap(slider, outer_guide, axes="x", min_overlap=0.050, name="slider_retains_engagement")
        ctx.expect_within(
            slider,
            outer_guide,
            axes="yz",
            margin=0.001,
            inner_elem="carrier",
            name="slider_profile_stays_captured_at_full_extension",
        )

    with ctx.pose({base_hinge: 0.0, guide_slide: 0.040, tip_hinge: 0.0}):
        tip_flat_box = ctx.part_world_aabb(tip_fork)
    with ctx.pose({base_hinge: 0.72, guide_slide: 0.040, tip_hinge: 0.0}):
        tip_lifted_box = ctx.part_world_aabb(tip_fork)
    ctx.check(
        "base_hinge_lifts_chain",
        tip_flat_box is not None
        and tip_lifted_box is not None
        and tip_lifted_box[1][2] > tip_flat_box[1][2] + 0.060,
        f"expected root hinge to raise the chain; closed={tip_flat_box}, lifted={tip_lifted_box}",
    )

    with ctx.pose({base_hinge: 0.0, guide_slide: 0.0, tip_hinge: 0.0}):
        retracted_tip_pos = ctx.part_world_position(tip_fork)
    with ctx.pose({base_hinge: 0.0, guide_slide: SLIDE_MAX, tip_hinge: 0.0}):
        extended_tip_pos = ctx.part_world_position(tip_fork)
    ctx.check(
        "prismatic_stage_extends_outward",
        retracted_tip_pos is not None
        and extended_tip_pos is not None
        and extended_tip_pos[0] > retracted_tip_pos[0] + 0.080,
        f"expected positive slide motion to extend outward; retracted={retracted_tip_pos}, extended={extended_tip_pos}",
    )

    with ctx.pose({base_hinge: 0.0, guide_slide: 0.050, tip_hinge: 0.0}):
        tip_closed_box = ctx.part_world_aabb(tip_fork)
    with ctx.pose({base_hinge: 0.0, guide_slide: 0.050, tip_hinge: 0.90}):
        tip_open_box = ctx.part_world_aabb(tip_fork)
        ctx.expect_contact(slider, tip_fork, name="tip_fork_remains_pinned_when_open")
    ctx.check(
        "tip_hinge_folds_upward",
        tip_closed_box is not None
        and tip_open_box is not None
        and tip_open_box[1][2] > tip_closed_box[1][2] + 0.025,
        f"expected tip hinge to lift the fork free end; closed={tip_closed_box}, open={tip_open_box}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
