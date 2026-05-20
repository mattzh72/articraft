from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


SUPPORT_PLATE_GAP = 0.090
SUPPORT_PLATE_T = 0.018
SUPPORT_PLATE_Y = (SUPPORT_PLATE_GAP / 2.0) + (SUPPORT_PLATE_T / 2.0)
SHOULDER_BARREL_R = 0.016
SHOULDER_BARREL_LEN = SUPPORT_PLATE_GAP

UPPER_LINK_LEN = 0.240
UPPER_BODY_W = 0.036
UPPER_FORK_OUTER_W = 0.052
FORE_HUB_W = 0.018
UPPER_FORK_SLOT_LEN = 0.072
UPPER_FORK_SLOT_H = 0.034

FORE_LINK_LEN = 0.230
FORE_BODY_W = 0.044
FORE_BODY_H = 0.052

RAM_SECTION_W = 0.034
RAM_SECTION_H = 0.022
RAM_STROKE = 0.090
RAM_JOINT_X = FORE_LINK_LEN

SHOULDER_LIMITS = (-0.35, 1.10)
ELBOW_LIMITS = (-1.05, 1.25)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .cylinder(length, radius, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _support_shape() -> cq.Workplane:
    base = _box((0.240, 0.180, 0.020), (-0.080, 0.0, -0.140))
    side_left = _box((0.106, SUPPORT_PLATE_T, 0.156), (-0.071, SUPPORT_PLATE_Y, -0.042))
    side_right = _box((0.106, SUPPORT_PLATE_T, 0.156), (-0.071, -SUPPORT_PLATE_Y, -0.042))
    rear_bridge = _box((0.026, 0.126, 0.098), (-0.114, 0.0, -0.086))
    top_bridge = _box((0.036, 0.126, 0.022), (-0.096, 0.0, 0.026))
    lower_spine = _box((0.120, 0.072, 0.050), (-0.078, 0.0, -0.106))

    gusset_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.120, -0.140),
                (-0.120, -0.030),
                (-0.030, 0.015),
                (-0.010, 0.015),
                (-0.010, -0.140),
            ]
        )
        .close()
        .extrude(0.014, both=True)
        .translate((0.0, SUPPORT_PLATE_Y - 0.028, 0.0))
    )
    gusset_left = gusset_profile
    gusset_right = gusset_profile.mirror("XZ")

    return (
        base.union(side_left)
        .union(side_right)
        .union(rear_bridge)
        .union(top_bridge)
        .union(lower_spine)
        .union(gusset_left)
        .union(gusset_right)
    )


def _upper_link_shape() -> cq.Workplane:
    shoulder_barrel = _y_cylinder(SHOULDER_BARREL_R, SHOULDER_BARREL_LEN, (0.0, 0.0, 0.0))
    shoulder_hub = _box((0.040, UPPER_BODY_W, 0.040), (0.020, 0.0, 0.0))
    boom = _box((0.150, UPPER_BODY_W, 0.038), (0.110, 0.0, 0.0))
    boom_cap = _box((0.060, 0.044, 0.030), (0.190, 0.0, 0.0))
    fork_block = _box((0.060, UPPER_FORK_OUTER_W, 0.044), (UPPER_LINK_LEN - 0.030, 0.0, 0.0))
    fork_cut = _box(
        (UPPER_FORK_SLOT_LEN, 0.028, UPPER_FORK_SLOT_H),
        (UPPER_LINK_LEN - 0.030, 0.0, 0.0),
    )

    return shoulder_barrel.union(shoulder_hub).union(boom).union(boom_cap).union(fork_block).cut(fork_cut)


def _fore_link_shape() -> cq.Workplane:
    hub = _box((0.024, FORE_HUB_W, 0.030), (0.012, 0.0, 0.0))
    body = _box((0.168, FORE_BODY_W, FORE_BODY_H), (0.108, 0.0, 0.0))
    nose = _box((0.050, 0.050, 0.040), (FORE_LINK_LEN - 0.025, 0.0, 0.0))
    top_relief = _box((0.090, 0.024, 0.018), (0.120, 0.0, 0.017))
    bottom_relief = _box((0.090, 0.024, 0.018), (0.120, 0.0, -0.017))

    return hub.union(body).union(nose).cut(top_relief).cut(bottom_relief)


def _output_ram_shape() -> cq.Workplane:
    shank = _box((0.100, 0.028, 0.020), (0.050, 0.0, 0.0))
    collar = _box((0.022, 0.034, 0.026), (0.011, 0.0, 0.0))
    nose = _box((0.036, 0.040, 0.026), (0.118, 0.0, 0.0))
    tip = _box((0.018, 0.026, 0.018), (0.145, 0.0, 0.0))
    return shank.union(collar).union(nose).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_two_link_arm")

    model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("boom_orange", rgba=(0.82, 0.43, 0.11, 1.0))
    model.material("forelink_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("ram_silver", rgba=(0.77, 0.79, 0.82, 1.0))

    support = model.part("rear_support")
    support.visual(
        Box((0.180, 0.160, 0.020)),
        origin=Origin(xyz=(-0.120, 0.0, -0.140)),
        material="graphite",
        name="base_skid",
    )
    support.visual(
        Box((0.100, SUPPORT_PLATE_T, 0.140)),
        origin=Origin(xyz=(-0.050, SUPPORT_PLATE_Y, -0.040)),
        material="graphite",
        name="left_cheek",
    )
    support.visual(
        Box((0.100, SUPPORT_PLATE_T, 0.140)),
        origin=Origin(xyz=(-0.050, -SUPPORT_PLATE_Y, -0.040)),
        material="graphite",
        name="right_cheek",
    )
    support.visual(
        Box((0.020, 0.122, 0.080)),
        origin=Origin(xyz=(-0.100, 0.0, -0.080)),
        material="graphite",
        name="rear_bridge",
    )
    support.visual(
        Box((0.100, 0.050, 0.040)),
        origin=Origin(xyz=(-0.085, 0.0, -0.110)),
        material="graphite",
        name="lower_brace",
    )
    support.visual(
        Box((0.025, 0.122, 0.018)),
        origin=Origin(xyz=(-0.0875, 0.0, 0.018)),
        material="graphite",
        name="top_strap",
    )

    upper = model.part("upper_link")
    upper.visual(
        Cylinder(radius=SHOULDER_BARREL_R, length=SHOULDER_BARREL_LEN),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="boom_orange",
        name="shoulder_barrel",
    )
    upper.visual(
        Box((0.034, 0.034, 0.034)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material="boom_orange",
        name="shoulder_knuckle",
    )
    upper.visual(
        Box((0.142, 0.028, 0.030)),
        origin=Origin(xyz=(0.102, 0.0, 0.0)),
        material="boom_orange",
        name="upper_spine",
    )
    upper.visual(
        Box((0.058, 0.040, 0.028)),
        origin=Origin(xyz=(0.188, 0.0, 0.0)),
        material="boom_orange",
        name="fore_clevis_block",
    )
    upper.visual(
        Box((0.044, 0.020, 0.042)),
        origin=Origin(xyz=(0.218, 0.020, 0.0)),
        material="boom_orange",
        name="fork_left",
    )
    upper.visual(
        Box((0.044, 0.020, 0.042)),
        origin=Origin(xyz=(0.218, -0.020, 0.0)),
        material="boom_orange",
        name="fork_right",
    )

    fore = model.part("fore_link")
    fore.visual(
        Box((0.024, 0.020, 0.030)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material="forelink_gray",
        name="elbow_lug",
    )
    fore.visual(
        Box((0.146, 0.036, 0.044)),
        origin=Origin(xyz=(0.097, 0.0, 0.0)),
        material="forelink_gray",
        name="fore_body",
    )
    fore.visual(
        Box((0.018, 0.040, 0.038)),
        origin=Origin(xyz=(0.173, 0.0, 0.0)),
        material="forelink_gray",
        name="nose_transition",
    )
    fore.visual(
        Box((0.054, 0.044, 0.034)),
        origin=Origin(xyz=(0.203, 0.0, 0.0)),
        material="forelink_gray",
        name="nose_housing",
    )

    ram = model.part("output_ram")
    ram.visual(
        Box((0.020, 0.034, 0.026)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material="ram_silver",
        name="rear_collar",
    )
    ram.visual(
        Box((0.090, 0.022, 0.018)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material="ram_silver",
        name="ram_shank",
    )
    ram.visual(
        Box((0.032, 0.030, 0.022)),
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        material="ram_silver",
        name="nose_block",
    )
    ram.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.151, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="ram_silver",
        name="nose_tip",
    )

    model.articulation(
        "support_to_upper",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
            effort=45.0,
            velocity=1.1,
        ),
    )
    model.articulation(
        "upper_to_fore",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=fore,
        origin=Origin(xyz=(UPPER_LINK_LEN, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
            effort=28.0,
            velocity=1.3,
        ),
    )
    model.articulation(
        "fore_to_ram",
        ArticulationType.PRISMATIC,
        parent=fore,
        child=ram,
        origin=Origin(xyz=(RAM_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=RAM_STROKE,
            effort=18.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("rear_support")
    upper = object_model.get_part("upper_link")
    fore = object_model.get_part("fore_link")
    ram = object_model.get_part("output_ram")
    shoulder = object_model.get_articulation("support_to_upper")
    elbow = object_model.get_articulation("upper_to_fore")
    slider = object_model.get_articulation("fore_to_ram")

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

    ctx.expect_contact(upper, support, name="upper_link_is_seated_in_support")
    ctx.expect_contact(fore, upper, name="fore_link_is_seated_in_upper_fork")
    ctx.expect_contact(ram, fore, name="output_ram_is_guided_by_fore_link")
    ctx.expect_within(ram, fore, axes="yz", margin=0.0, name="ram_stays_within_fore_link_cross_section")

    with ctx.pose({slider: RAM_STROKE}):
        ctx.expect_within(
            ram,
            fore,
            axes="yz",
            margin=0.0,
            name="extended_ram_stays_within_fore_link_cross_section",
        )

    ctx.check(
        "shoulder_axis_matches_link_pitch",
        shoulder.axis == (0.0, -1.0, 0.0),
        details=f"unexpected shoulder axis {shoulder.axis}",
    )
    ctx.check(
        "elbow_axis_matches_link_pitch",
        elbow.axis == (0.0, -1.0, 0.0),
        details=f"unexpected elbow axis {elbow.axis}",
    )
    ctx.check(
        "ram_axis_points_forward",
        slider.axis == (1.0, 0.0, 0.0),
        details=f"unexpected ram axis {slider.axis}",
    )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, slider: 0.0}):
        upper_closed = ctx.part_world_aabb(upper)
        fore_closed = ctx.part_world_aabb(fore)
        ram_closed = ctx.part_world_aabb(ram)
    with ctx.pose({shoulder: SHOULDER_LIMITS[1], elbow: 0.0, slider: 0.0}):
        upper_raised = ctx.part_world_aabb(upper)
    with ctx.pose({shoulder: 0.55, elbow: 0.0, slider: 0.0}):
        fore_flat = ctx.part_world_aabb(fore)
    with ctx.pose({shoulder: 0.55, elbow: ELBOW_LIMITS[1], slider: 0.0}):
        fore_raised = ctx.part_world_aabb(fore)
    with ctx.pose({shoulder: 0.0, elbow: 0.0, slider: RAM_STROKE}):
        ram_extended = ctx.part_world_aabb(ram)

    if upper_closed is None or upper_raised is None:
        ctx.fail("shoulder_motion_probe_available", "upper-link AABB unavailable during shoulder pose checks")
    else:
        ctx.check(
            "positive_shoulder_raises_upper_link",
            upper_raised[1][2] > upper_closed[1][2] + 0.10,
            details=(
                f"upper link did not lift enough: closed max z={upper_closed[1][2]:.4f}, "
                f"raised max z={upper_raised[1][2]:.4f}"
            ),
        )

    if fore_flat is None or fore_raised is None or fore_closed is None:
        ctx.fail("elbow_motion_probe_available", "fore-link AABB unavailable during elbow pose checks")
    else:
        ctx.check(
            "positive_elbow_raises_fore_link",
            fore_raised[1][2] > fore_flat[1][2] + 0.06,
            details=(
                f"fore link did not lift enough: neutral max z={fore_flat[1][2]:.4f}, "
                f"raised max z={fore_raised[1][2]:.4f}"
            ),
        )

    if ram_closed is None or ram_extended is None:
        ctx.fail("ram_motion_probe_available", "output-ram AABB unavailable during extension checks")
    else:
        ctx.check(
            "ram_extension_moves_nose_forward",
            ram_extended[1][0] >= ram_closed[1][0] + 0.085,
            details=(
                f"ram did not extend enough: closed max x={ram_closed[1][0]:.4f}, "
                f"extended max x={ram_extended[1][0]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
