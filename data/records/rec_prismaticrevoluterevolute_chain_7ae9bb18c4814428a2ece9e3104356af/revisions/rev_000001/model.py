from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_LENGTH = 1.18
RAIL_WIDTH = 0.24
RAIL_BASE_HEIGHT = 0.045
GUIDE_LENGTH = 0.92
GUIDE_WIDTH = 0.055
GUIDE_HEIGHT = 0.020
GUIDE_OFFSET_Y = 0.070
CENTER_RIB_WIDTH = 0.060
CENTER_RIB_HEIGHT = 0.010

CARRIAGE_LENGTH = 0.22
CARRIAGE_SHOE_HEIGHT = 0.038
CARRIAGE_BRIDGE_WIDTH = 0.172
CARRIAGE_BRIDGE_HEIGHT = 0.032
TURRET_DIAMETER = 0.120
TURRET_HEIGHT = 0.028
TURRET_CAP_HEIGHT = 0.006

UPPER_ARM_LENGTH = 0.34
UPPER_ARM_THICKNESS = 0.024
UPPER_ARM_BODY_WIDTH = 0.056
UPPER_ARM_ROOT_DIAMETER = 0.112
UPPER_ARM_ELBOW_DIAMETER = 0.090

FOREARM_LENGTH = 0.27
FOREARM_THICKNESS = 0.022
FOREARM_BODY_WIDTH = 0.046
FOREARM_ROOT_DIAMETER = 0.084
FOREARM_END_DIAMETER = 0.060

SLIDE_HOME_X = -0.26
SLIDE_TRAVEL = 0.52


def _box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _disc(diameter: float, thickness: float, center: tuple[float, float, float]) -> cq.Workplane:
    cx, cy, cz = center
    return cq.Workplane("XY").circle(diameter / 2.0).extrude(thickness).translate((cx, cy, cz - thickness / 2.0))


def _rail_shape() -> cq.Workplane:
    base = _box(RAIL_LENGTH, RAIL_WIDTH, RAIL_BASE_HEIGHT, (0.0, 0.0, RAIL_BASE_HEIGHT / 2.0))
    guide_left = _box(
        GUIDE_LENGTH,
        GUIDE_WIDTH,
        GUIDE_HEIGHT,
        (0.0, GUIDE_OFFSET_Y, RAIL_BASE_HEIGHT + GUIDE_HEIGHT / 2.0),
    )
    guide_right = _box(
        GUIDE_LENGTH,
        GUIDE_WIDTH,
        GUIDE_HEIGHT,
        (0.0, -GUIDE_OFFSET_Y, RAIL_BASE_HEIGHT + GUIDE_HEIGHT / 2.0),
    )
    center_rib = _box(
        GUIDE_LENGTH * 0.86,
        CENTER_RIB_WIDTH,
        CENTER_RIB_HEIGHT,
        (0.0, 0.0, RAIL_BASE_HEIGHT + CENTER_RIB_HEIGHT / 2.0),
    )
    end_pad_left = _box(0.12, RAIL_WIDTH * 0.82, 0.014, (-(RAIL_LENGTH / 2.0 - 0.06), 0.0, 0.007))
    end_pad_right = _box(0.12, RAIL_WIDTH * 0.82, 0.014, ((RAIL_LENGTH / 2.0 - 0.06), 0.0, 0.007))

    relief = _box(0.66, 0.085, 0.012, (0.0, 0.0, RAIL_BASE_HEIGHT + 0.006))

    rail = (
        base.union(guide_left)
        .union(guide_right)
        .union(center_rib)
        .union(end_pad_left)
        .union(end_pad_right)
        .cut(relief)
    )
    return rail


def _carriage_shape() -> cq.Workplane:
    shoe_left = _box(
        CARRIAGE_LENGTH,
        GUIDE_WIDTH,
        CARRIAGE_SHOE_HEIGHT,
        (0.0, GUIDE_OFFSET_Y, CARRIAGE_SHOE_HEIGHT / 2.0),
    )
    shoe_right = _box(
        CARRIAGE_LENGTH,
        GUIDE_WIDTH,
        CARRIAGE_SHOE_HEIGHT,
        (0.0, -GUIDE_OFFSET_Y, CARRIAGE_SHOE_HEIGHT / 2.0),
    )
    bridge = _box(
        CARRIAGE_LENGTH * 0.96,
        CARRIAGE_BRIDGE_WIDTH,
        CARRIAGE_BRIDGE_HEIGHT,
        (0.0, 0.0, CARRIAGE_SHOE_HEIGHT + CARRIAGE_BRIDGE_HEIGHT / 2.0),
    )
    turret = cq.Workplane("XY").circle(TURRET_DIAMETER / 2.0).extrude(TURRET_HEIGHT).translate(
        (0.0, 0.0, CARRIAGE_SHOE_HEIGHT + CARRIAGE_BRIDGE_HEIGHT)
    )
    turret_cap = cq.Workplane("XY").circle(TURRET_DIAMETER * 0.34).extrude(TURRET_CAP_HEIGHT).translate(
        (0.0, 0.0, CARRIAGE_SHOE_HEIGHT + CARRIAGE_BRIDGE_HEIGHT + TURRET_HEIGHT)
    )
    cable_boss = _box(
        0.050,
        0.095,
        0.018,
        (-0.060, 0.0, CARRIAGE_SHOE_HEIGHT + CARRIAGE_BRIDGE_HEIGHT + 0.009),
    )

    window = _box(0.11, 0.080, CARRIAGE_BRIDGE_HEIGHT * 1.4, (0.015, 0.0, CARRIAGE_SHOE_HEIGHT + CARRIAGE_BRIDGE_HEIGHT / 2.0))

    carriage = (
        shoe_left.union(shoe_right)
        .union(bridge)
        .union(turret)
        .union(turret_cap)
        .union(cable_boss)
        .cut(window)
    )
    return carriage


def _upper_arm_shape() -> cq.Workplane:
    root_disc = _disc(UPPER_ARM_ROOT_DIAMETER, UPPER_ARM_THICKNESS, (0.0, 0.0, 0.0))
    elbow_disc = _disc(UPPER_ARM_ELBOW_DIAMETER, UPPER_ARM_THICKNESS, (UPPER_ARM_LENGTH, 0.0, 0.0))
    beam = _box(
        UPPER_ARM_LENGTH,
        UPPER_ARM_BODY_WIDTH,
        UPPER_ARM_THICKNESS,
        (UPPER_ARM_LENGTH / 2.0, 0.0, 0.0),
    )
    shoulder_pad = _box(0.10, 0.082, UPPER_ARM_THICKNESS, (0.055, 0.0, 0.0))
    arm = root_disc.union(elbow_disc).union(beam).union(shoulder_pad)

    window = _box(0.15, 0.024, UPPER_ARM_THICKNESS * 1.8, (0.18, 0.0, 0.0))
    elbow_window = _box(0.060, 0.020, UPPER_ARM_THICKNESS * 1.8, (UPPER_ARM_LENGTH - 0.055, 0.0, 0.0))
    return arm.cut(window).cut(elbow_window)


def _forearm_shape() -> cq.Workplane:
    root_disc = _disc(FOREARM_ROOT_DIAMETER, FOREARM_THICKNESS, (0.0, 0.0, 0.0))
    end_disc = _disc(FOREARM_END_DIAMETER, FOREARM_THICKNESS, (FOREARM_LENGTH, 0.0, 0.0))
    beam = _box(
        FOREARM_LENGTH,
        FOREARM_BODY_WIDTH,
        FOREARM_THICKNESS,
        (FOREARM_LENGTH / 2.0, 0.0, 0.0),
    )
    forearm = root_disc.union(end_disc).union(beam)

    window = _box(0.13, 0.018, FOREARM_THICKNESS * 1.8, (0.15, 0.0, 0.0))
    tip_flat = _box(0.030, 0.050, 0.006, (FOREARM_LENGTH - 0.010, 0.0, 0.0))
    return forearm.cut(window).union(tip_flat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_carriage_elbow_arm")

    model.material("rail_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("carriage_charcoal", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("arm_silver", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("accent_black", rgba=(0.08, 0.09, 0.10, 1.0))

    rail = model.part("rail")
    rail.visual(mesh_from_cadquery(_rail_shape(), "rail_body"), material="rail_gray", name="rail_body")

    carriage = model.part("carriage")
    carriage.visual(mesh_from_cadquery(_carriage_shape(), "carriage_body"), material="carriage_charcoal", name="carriage_body")

    upper_arm = model.part("upper_arm")
    upper_arm.visual(mesh_from_cadquery(_upper_arm_shape(), "upper_arm_body"), material="arm_silver", name="upper_arm_body")

    forearm = model.part("forearm")
    forearm.visual(mesh_from_cadquery(_forearm_shape(), "forearm_body"), material="arm_silver", name="forearm_body")

    model.articulation(
        "rail_slide",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, RAIL_BASE_HEIGHT + GUIDE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=400.0, velocity=0.35),
    )
    model.articulation(
        "carriage_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=upper_arm,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CARRIAGE_SHOE_HEIGHT
                + CARRIAGE_BRIDGE_HEIGHT
                + TURRET_HEIGHT
                + TURRET_CAP_HEIGHT
                + UPPER_ARM_THICKNESS / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=60.0, velocity=1.6),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(
            xyz=(
                UPPER_ARM_LENGTH,
                0.0,
                UPPER_ARM_THICKNESS / 2.0 + FOREARM_THICKNESS / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.35, upper=2.35, effort=35.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")

    slide = object_model.get_articulation("rail_slide")
    shoulder = object_model.get_articulation("carriage_to_upper_arm")
    elbow = object_model.get_articulation("upper_arm_to_forearm")

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
        "joint_axes_match_prompt",
        slide.axis == (1.0, 0.0, 0.0) and shoulder.axis == (0.0, 0.0, 1.0) and elbow.axis == (0.0, 0.0, 1.0),
        details=(
            f"expected slide axis (1,0,0) and shoulder/elbow axes (0,0,1); "
            f"got {slide.axis}, {shoulder.axis}, {elbow.axis}"
        ),
    )

    ctx.expect_contact(carriage, rail, name="carriage_is_supported_by_rail")
    ctx.expect_contact(upper_arm, carriage, name="upper_arm_is_mounted_to_carriage")
    ctx.expect_contact(forearm, upper_arm, name="forearm_is_mounted_to_upper_arm")
    ctx.expect_gap(upper_arm, rail, axis="z", min_gap=0.035, name="upper_arm_clears_rail")
    ctx.expect_gap(forearm, rail, axis="z", min_gap=0.045, name="forearm_clears_rail")

    rail_aabb = ctx.part_world_aabb(rail)
    upper_aabb = ctx.part_world_aabb(upper_arm)
    fore_aabb = ctx.part_world_aabb(forearm)
    if rail_aabb is not None and upper_aabb is not None and fore_aabb is not None:
        rail_length = rail_aabb[1][0] - rail_aabb[0][0]
        upper_length = upper_aabb[1][0] - upper_aabb[0][0]
        fore_length = fore_aabb[1][0] - fore_aabb[0][0]
        ctx.check(
            "rail_stage_reads_as_primary_positioner",
            rail_length > upper_length * 2.5 and rail_length > fore_length * 3.2,
            details=(
                f"rail length={rail_length:.3f}, upper span={upper_length:.3f}, "
                f"forearm span={fore_length:.3f}"
            ),
        )

    home_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide.motion_limits.upper}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(carriage, rail, name="extended_carriage_remains_supported")
        ctx.fail_if_parts_overlap_in_current_pose(name="extended_pose_has_no_overlaps")

    if home_carriage_pos is not None and extended_carriage_pos is not None:
        ctx.check(
            "prismatic_stage_moves_in_positive_x",
            extended_carriage_pos[0] > home_carriage_pos[0] + 0.40,
            details=f"carriage x moved from {home_carriage_pos[0]:.3f} to {extended_carriage_pos[0]:.3f}",
        )

    home_upper_aabb = ctx.part_world_aabb(upper_arm)
    with ctx.pose({shoulder: 0.85}):
        swung_upper_aabb = ctx.part_world_aabb(upper_arm)
        ctx.fail_if_parts_overlap_in_current_pose(name="shoulder_swing_pose_has_no_overlaps")

    if home_upper_aabb is not None and swung_upper_aabb is not None:
        ctx.check(
            "shoulder_positive_rotation_swings_toward_positive_y",
            swung_upper_aabb[1][1] > home_upper_aabb[1][1] + 0.12,
            details=(
                f"upper-arm y_max changed from {home_upper_aabb[1][1]:.3f} "
                f"to {swung_upper_aabb[1][1]:.3f}"
            ),
        )

    home_forearm_aabb = ctx.part_world_aabb(forearm)
    with ctx.pose({elbow: 1.10}):
        bent_forearm_aabb = ctx.part_world_aabb(forearm)
        ctx.fail_if_parts_overlap_in_current_pose(name="elbow_bend_pose_has_no_overlaps")

    if home_forearm_aabb is not None and bent_forearm_aabb is not None:
        ctx.check(
            "elbow_positive_rotation_bends_toward_positive_y",
            bent_forearm_aabb[1][1] > home_forearm_aabb[1][1] + 0.10,
            details=(
                f"forearm y_max changed from {home_forearm_aabb[1][1]:.3f} "
                f"to {bent_forearm_aabb[1][1]:.3f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
