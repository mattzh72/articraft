from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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


BASE_PLATE_X = 0.160
BASE_PLATE_Y = 0.120
BASE_PLATE_Z = 0.012
BASE_CHEEK_T = 0.012
BASE_CHEEK_CENTER_Y = 0.030
BASE_INNER_GAP = (2.0 * BASE_CHEEK_CENTER_Y) - BASE_CHEEK_T
SHOULDER_AXIS_Z = 0.105
SHOULDER_PIN_R = 0.009
SHOULDER_BARREL_R = 0.016
SHOULDER_BARREL_Y = BASE_INNER_GAP
SHOULDER_LENGTH = 0.200

ELBOW_PIN_R = 0.0075
ELBOW_BARREL_R = 0.0125
ELBOW_BARREL_Y = 0.026
ELBOW_SLIDE_ORIGIN_X = 0.132
SLIDE_HOUSING_X = 0.118
SLIDE_HOUSING_Y = 0.026
SLIDE_HOUSING_Z = 0.022
SLIDE_GUIDE_Y = 0.014
SLIDE_GUIDE_Z = 0.010
TIP_TRAVEL = 0.055


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _solid(shape: cq.Workplane):
    return shape.findSolid()


def _base_bracket_shape() -> cq.Workplane:
    foot = _solid(cq.Workplane("XY").box(BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z).translate((0.0, 0.0, BASE_PLATE_Z / 2.0)))
    left_cheek = _solid(cq.Workplane("XY").box(0.024, BASE_CHEEK_T, 0.110).translate((0.0, BASE_CHEEK_CENTER_Y, 0.055)))
    right_cheek = _solid(cq.Workplane("XY").box(0.024, BASE_CHEEK_T, 0.110).translate((0.0, -BASE_CHEEK_CENTER_Y, 0.055)))
    rear_bridge = _solid(cq.Workplane("XY").box(0.028, BASE_INNER_GAP, 0.030).translate((-0.018, 0.0, 0.048)))
    toe_rib = _solid(cq.Workplane("XY").box(0.030, BASE_INNER_GAP, 0.020).translate((0.010, 0.0, 0.022)))
    bracket = foot.fuse(left_cheek).fuse(right_cheek).fuse(rear_bridge).fuse(toe_rib)
    return cq.Workplane(obj=bracket)


def _shoulder_link_shape() -> cq.Workplane:
    rear_hub = _solid(_cylinder_y(SHOULDER_BARREL_R, SHOULDER_BARREL_Y, (0.0, 0.0, 0.0)))
    root_web = _solid(cq.Workplane("XY").box(0.030, 0.024, 0.024).translate((0.038, 0.0, 0.0)))
    main_beam = _solid(cq.Workplane("XY").box(0.116, 0.026, 0.022).translate((0.108, 0.0, 0.0)))
    clevis_block = _solid(cq.Workplane("XY").box(0.026, 0.038, 0.032).translate((SHOULDER_LENGTH, 0.0, 0.0)))

    body = rear_hub.fuse(root_web).fuse(main_beam).fuse(clevis_block)
    rear_bore = _solid(_cylinder_y(SHOULDER_PIN_R, 0.070, (0.0, 0.0, 0.0)))
    front_slot = _solid(cq.Workplane("XY").box(0.030, 0.026, 0.024).translate((SHOULDER_LENGTH, 0.0, 0.0)))
    elbow_bore = _solid(_cylinder_y(ELBOW_PIN_R, 0.050, (SHOULDER_LENGTH, 0.0, 0.0)))
    lightening_slot = _solid(cq.Workplane("XY").box(0.054, 0.010, 0.010).translate((0.108, 0.0, 0.0)))
    shoulder = body.cut(rear_bore).cut(front_slot).cut(elbow_bore).cut(lightening_slot)
    return cq.Workplane(obj=shoulder)


def _elbow_link_shape() -> cq.Workplane:
    rear_hub = _solid(_cylinder_y(ELBOW_BARREL_R, ELBOW_BARREL_Y, (0.0, 0.0, 0.0)))
    arm_beam = _solid(cq.Workplane("XY").box(0.104, 0.022, 0.018).translate((0.070, 0.0, 0.0)))
    arm_cap = _solid(cq.Workplane("XY").box(0.020, 0.024, 0.020).translate((0.122, 0.0, 0.0)))
    lower_way = _solid(cq.Workplane("XY").box(SLIDE_HOUSING_X, SLIDE_HOUSING_Y, 0.004).translate((0.191, 0.0, -0.007)))
    left_guide = _solid(cq.Workplane("XY").box(SLIDE_HOUSING_X, 0.004, 0.018).translate((0.191, 0.011, 0.002)))
    right_guide = _solid(cq.Workplane("XY").box(SLIDE_HOUSING_X, 0.004, 0.018).translate((0.191, -0.011, 0.002)))
    nose_cap = _solid(cq.Workplane("XY").box(0.018, 0.024, 0.018).translate((0.250, 0.0, 0.002)))

    body = rear_hub.fuse(arm_beam).fuse(arm_cap).fuse(lower_way).fuse(left_guide).fuse(right_guide).fuse(nose_cap)
    rear_bore = _solid(_cylinder_y(ELBOW_PIN_R, 0.040, (0.0, 0.0, 0.0)))
    elbow = body.cut(rear_bore)
    return cq.Workplane(obj=elbow)


def _tip_stage_shape() -> cq.Workplane:
    guide_rail = _solid(cq.Workplane("XY").box(0.086, SLIDE_GUIDE_Y, SLIDE_GUIDE_Z).translate((0.043, 0.0, 0.0)))
    tool_block = _solid(cq.Workplane("XY").box(0.032, 0.034, 0.020).translate((0.089, 0.0, 0.005)))
    nose = _solid(
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(0.020)
        .translate((0.0, 0.0, -0.010))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((0.111, 0.0, 0.005))
    )
    tip = guide_rail.fuse(tool_block).fuse(nose)
    return cq.Workplane(obj=tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="revolute_revolute_prismatic_chain")

    model.material("powder_coat", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("anodized_dark", rgba=(0.19, 0.21, 0.23, 1.0))
    model.material("accent_orange", rgba=(0.87, 0.43, 0.14, 1.0))

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((BASE_PLATE_X, BASE_PLATE_Y, BASE_PLATE_Z)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_Z / 2.0)),
        material="powder_coat",
        name="base_foot",
    )
    base_bracket.visual(
        Box((0.024, BASE_CHEEK_T, 0.110)),
        origin=Origin(xyz=(0.0, BASE_CHEEK_CENTER_Y, 0.055)),
        material="powder_coat",
        name="left_cheek",
    )
    base_bracket.visual(
        Box((0.024, BASE_CHEEK_T, 0.110)),
        origin=Origin(xyz=(0.0, -BASE_CHEEK_CENTER_Y, 0.055)),
        material="powder_coat",
        name="right_cheek",
    )
    base_bracket.visual(
        Box((0.022, BASE_INNER_GAP, 0.030)),
        origin=Origin(xyz=(-0.020, 0.0, 0.045)),
        material="powder_coat",
        name="rear_bridge",
    )
    base_bracket.visual(
        Box((0.018, BASE_INNER_GAP, 0.016)),
        origin=Origin(xyz=(-0.004, 0.0, 0.020)),
        material="powder_coat",
        name="toe_rib",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((BASE_PLATE_X, BASE_PLATE_Y, SHOULDER_AXIS_Z + 0.016)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, (SHOULDER_AXIS_Z + 0.016) / 2.0)),
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        Cylinder(radius=SHOULDER_BARREL_R, length=BASE_INNER_GAP),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_aluminum",
        name="shoulder_barrel",
    )
    shoulder_link.visual(
        Box((0.032, 0.024, 0.024)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material="machined_aluminum",
        name="shoulder_root",
    )
    shoulder_link.visual(
        Box((0.122, 0.024, 0.022)),
        origin=Origin(xyz=(0.097, 0.0, 0.0)),
        material="machined_aluminum",
        name="shoulder_beam",
    )
    shoulder_link.visual(
        Box((0.032, 0.022, 0.022)),
        origin=Origin(xyz=(0.166, 0.0, 0.0)),
        material="machined_aluminum",
        name="shoulder_front_spine",
    )
    shoulder_link.visual(
        Box((0.028, 0.006, 0.030)),
        origin=Origin(xyz=(0.186, 0.016, 0.0)),
        material="machined_aluminum",
        name="shoulder_upper_ear",
    )
    shoulder_link.visual(
        Box((0.028, 0.006, 0.030)),
        origin=Origin(xyz=(0.186, -0.016, 0.0)),
        material="machined_aluminum",
        name="shoulder_lower_ear",
    )
    shoulder_link.visual(
        Box((0.020, 0.034, 0.008)),
        origin=Origin(xyz=(0.176, 0.0, -0.014)),
        material="machined_aluminum",
        name="shoulder_lower_gusset",
    )
    shoulder_link.inertial = Inertial.from_geometry(
        Box((0.210, 0.056, 0.040)),
        mass=1.0,
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
    )

    elbow_link = model.part("elbow_link")
    elbow_link.visual(
        Cylinder(radius=ELBOW_BARREL_R, length=0.026),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_aluminum",
        name="elbow_barrel",
    )
    elbow_link.visual(
        Box((0.036, 0.020, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material="machined_aluminum",
        name="elbow_root",
    )
    elbow_link.visual(
        Box((0.090, 0.020, 0.018)),
        origin=Origin(xyz=(0.076, 0.0, 0.0)),
        material="machined_aluminum",
        name="elbow_beam",
    )
    elbow_link.visual(
        Box((0.110, 0.022, 0.004)),
        origin=Origin(xyz=(0.167, 0.0, -0.007)),
        material="machined_aluminum",
        name="slide_floor",
    )
    elbow_link.visual(
        Box((0.106, 0.004, 0.018)),
        origin=Origin(xyz=(0.169, 0.009, 0.0)),
        material="machined_aluminum",
        name="slide_left_rail",
    )
    elbow_link.visual(
        Box((0.106, 0.004, 0.018)),
        origin=Origin(xyz=(0.169, -0.009, 0.0)),
        material="machined_aluminum",
        name="slide_right_rail",
    )
    elbow_link.inertial = Inertial.from_geometry(
        Box((0.225, SLIDE_HOUSING_Y, SLIDE_HOUSING_Z)),
        mass=0.8,
        origin=Origin(xyz=(0.1125, 0.0, 0.0)),
    )

    tip_stage = model.part("tip_stage")
    tip_stage.visual(
        Box((0.070, 0.014, 0.010)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material="anodized_dark",
        name="tip_rail",
    )
    tip_stage.visual(
        Box((0.032, 0.030, 0.018)),
        origin=Origin(xyz=(0.082, 0.0, 0.004)),
        material="anodized_dark",
        name="tip_head",
    )
    tip_stage.visual(
        Box((0.016, 0.024, 0.014)),
        origin=Origin(xyz=(0.102, 0.0, 0.004)),
        material="anodized_dark",
        name="tip_nose",
    )
    tip_stage.visual(
        Box((0.010, 0.022, 0.004)),
        origin=Origin(xyz=(0.082, 0.0, 0.015)),
        material="accent_orange",
        name="tip_stage_pad",
    )
    tip_stage.inertial = Inertial.from_geometry(
        Box((0.140, 0.040, 0.026)),
        mass=0.25,
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.3, lower=0.0, upper=1.20),
    )
    model.articulation(
        "shoulder_to_elbow",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=elbow_link,
        origin=Origin(xyz=(SHOULDER_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.6, lower=0.0, upper=1.35),
    )
    model.articulation(
        "elbow_to_tip_stage",
        ArticulationType.PRISMATIC,
        parent=elbow_link,
        child=tip_stage,
        origin=Origin(xyz=(ELBOW_SLIDE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=TIP_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_bracket = object_model.get_part("base_bracket")
    shoulder_link = object_model.get_part("shoulder_link")
    elbow_link = object_model.get_part("elbow_link")
    tip_stage = object_model.get_part("tip_stage")

    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_elbow")
    slide_joint = object_model.get_articulation("elbow_to_tip_stage")

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

    ctx.expect_contact(base_bracket, shoulder_link, name="shoulder_is_carried_on_base_pin")
    ctx.expect_contact(shoulder_link, elbow_link, name="elbow_is_carried_on_shoulder_pin")
    ctx.expect_contact(elbow_link, tip_stage, name="tip_stage_is_supported_in_slide")

    with ctx.pose({slide_joint: TIP_TRAVEL}):
        ctx.expect_contact(elbow_link, tip_stage, name="tip_stage_remains_supported_when_extended")
        ctx.expect_overlap(tip_stage, elbow_link, axes="x", min_overlap=0.028, name="tip_stage_retains_rail_engagement")
        ctx.expect_overlap(tip_stage, elbow_link, axes="yz", min_overlap=0.016, name="tip_stage_stays_aligned_in_housing")

    with ctx.pose({shoulder_joint: 1.05, elbow_joint: 0.90, slide_joint: TIP_TRAVEL}):
        ctx.expect_contact(elbow_link, tip_stage, name="tip_stage_remains_supported_in_working_pose")
        ctx.expect_overlap(tip_stage, elbow_link, axes="yz", min_overlap=0.012, name="tip_stage_stays_captured_in_working_pose")

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, slide_joint: 0.0}):
        elbow_rest = ctx.part_world_position(elbow_link)
        tip_rest = ctx.part_world_position(tip_stage)
    with ctx.pose({shoulder_joint: 1.05, elbow_joint: 0.0, slide_joint: 0.0}):
        elbow_raised = ctx.part_world_position(elbow_link)
    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 0.0, slide_joint: 0.0}):
        tip_elbow_straight = ctx.part_world_position(tip_stage)
    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 1.05, slide_joint: 0.0}):
        tip_elbow_bent = ctx.part_world_position(tip_stage)
    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0, slide_joint: TIP_TRAVEL}):
        tip_extended = ctx.part_world_position(tip_stage)

    ctx.check(
        "shoulder_positive_motion_raises_elbow",
        elbow_rest is not None
        and elbow_raised is not None
        and elbow_raised[2] > elbow_rest[2] + 0.14,
        details=f"rest={elbow_rest}, raised={elbow_raised}",
    )
    ctx.check(
        "elbow_positive_motion_raises_tip",
        tip_elbow_straight is not None
        and tip_elbow_bent is not None
        and tip_elbow_bent[2] > tip_elbow_straight[2] + 0.05,
        details=f"straight={tip_elbow_straight}, bent={tip_elbow_bent}",
    )
    ctx.check(
        "prismatic_stage_extends_forward",
        tip_rest is not None and tip_extended is not None and tip_extended[0] > tip_rest[0] + 0.045,
        details=f"rest={tip_rest}, extended={tip_extended}",
    )
    ctx.check(
        "joint_axes_match_pitch_pitch_slide_chain",
        tuple(shoulder_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(elbow_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(slide_joint.axis) == (1.0, 0.0, 0.0)
        and isclose(shoulder_joint.motion_limits.upper or 0.0, 1.20, abs_tol=1e-9)
        and isclose(elbow_joint.motion_limits.upper or 0.0, 1.35, abs_tol=1e-9)
        and isclose(slide_joint.motion_limits.upper or 0.0, TIP_TRAVEL, abs_tol=1e-9),
        details=(
            f"shoulder_axis={shoulder_joint.axis}, elbow_axis={elbow_joint.axis}, "
            f"slide_axis={slide_joint.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
