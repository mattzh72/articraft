from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_leg")

    painted_metal = model.material("painted_metal", rgba=(0.83, 0.85, 0.87, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    elastomer_black = model.material("elastomer_black", rgba=(0.10, 0.10, 0.11, 1.0))

    def xy_loop(
        width: float,
        depth: float,
        z: float,
        *,
        radius: float,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x + x_shift, y + y_shift, z)
            for x, y in rounded_rect_profile(width, depth, radius)
        ]

    def yz_loop(
        width: float,
        height: float,
        x: float,
        *,
        radius: float,
        y_shift: float = 0.0,
        z_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y + y_shift, z + z_shift)
            for y, z in rounded_rect_profile(width, height, radius)
        ]

    def refined_loft(sections: list[list[tuple[float, float, float]]]):
        return repair_loft(section_loft(sections), repair="mesh")

    hip_housing = model.part("hip_housing")
    hip_shell = refined_loft(
        [
            xy_loop(0.094, 0.076, 0.040, radius=0.018, x_shift=0.004),
            xy_loop(0.126, 0.094, 0.088, radius=0.024, x_shift=0.012),
            xy_loop(0.114, 0.090, 0.136, radius=0.022, x_shift=0.016),
        ]
    )
    hip_housing.visual(
        mesh_from_geometry(hip_shell, "hip_housing_shell"),
        material=painted_metal,
        name="hip_shell",
    )
    hip_pad = ExtrudeGeometry(rounded_rect_profile(0.090, 0.064, 0.014), 0.012)
    hip_housing.visual(
        mesh_from_geometry(hip_pad, "hip_mount_pad"),
        origin=Origin(xyz=(0.016, 0.0, 0.142)),
        material=polymer_dark,
        name="mount_pad",
    )
    hip_housing.visual(
        Box((0.016, 0.046, 0.066)),
        origin=Origin(xyz=(0.060, 0.0, 0.070)),
        material=polymer_dark,
        name="front_bay_cover",
    )
    hip_housing.visual(
        Box((0.090, 0.104, 0.104)),
        origin=Origin(xyz=(0.012, 0.0, 0.092)),
        material=polymer_dark,
        name="hip_core_block",
    )
    for side, side_name in ((1.0, "left"), (-1.0, "right")):
        hip_housing.visual(
            Box((0.082, 0.008, 0.096)),
            origin=Origin(xyz=(0.010, side * 0.049, 0.008)),
            material=painted_metal,
            name=f"hip_side_cheek_{side_name}",
        )
        hip_housing.visual(
            Box((0.020, 0.010, 0.100)),
            origin=Origin(xyz=(0.032, side * 0.048, 0.006)),
            material=painted_metal,
            name=f"hip_front_strut_{side_name}",
        )
        hip_housing.visual(
            Box((0.018, 0.010, 0.092)),
            origin=Origin(xyz=(-0.018, side * 0.048, 0.010)),
            material=painted_metal,
            name=f"hip_rear_strut_{side_name}",
        )
        hip_housing.visual(
            Cylinder(radius=0.028, length=0.012),
            origin=Origin(xyz=(0.0, side * 0.049, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polymer_dark,
            name=f"hip_bearing_{side_name}",
        )

    thigh = model.part("thigh")
    thigh_shell = refined_loft(
        [
            xy_loop(0.084, 0.072, -0.022, radius=0.017, x_shift=0.006),
            xy_loop(0.096, 0.078, -0.092, radius=0.020, x_shift=0.010),
            xy_loop(0.084, 0.068, -0.214, radius=0.017, x_shift=0.008),
            xy_loop(0.070, 0.060, -0.286, radius=0.014, x_shift=0.004),
        ]
    )
    thigh.visual(
        mesh_from_geometry(thigh_shell, "thigh_shell"),
        material=painted_metal,
        name="thigh_shell",
    )
    thigh.visual(
        Cylinder(radius=0.034, length=0.086),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer_dark,
        name="hip_hub",
    )
    thigh.visual(
        Box((0.014, 0.042, 0.168)),
        origin=Origin(xyz=(-0.024, 0.0, -0.170)),
        material=polymer_dark,
        name="thigh_actuator_bay",
    )
    thigh.visual(
        Box((0.032, 0.040, 0.114)),
        origin=Origin(xyz=(-0.012, 0.0, -0.084)),
        material=polymer_dark,
        name="thigh_spine",
    )
    thigh.visual(
        Box((0.072, 0.094, 0.026)),
        origin=Origin(xyz=(0.002, 0.0, -0.260)),
        material=polymer_dark,
        name="thigh_knee_bridge",
    )
    for side, side_name in ((1.0, "left"), (-1.0, "right")):
        thigh.visual(
            Box((0.016, 0.010, 0.074)),
            origin=Origin(xyz=(0.026, side * 0.042, -0.304)),
            material=painted_metal,
            name=f"knee_front_strut_{side_name}",
        )
        thigh.visual(
            Box((0.016, 0.010, 0.072)),
            origin=Origin(xyz=(-0.014, side * 0.042, -0.302)),
            material=painted_metal,
            name=f"knee_rear_strut_{side_name}",
        )
        thigh.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(
                xyz=(0.0, side * 0.042, -0.340),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=polymer_dark,
            name=f"knee_bearing_{side_name}",
        )

    shank = model.part("shank")
    shank_shell = refined_loft(
        [
            xy_loop(0.070, 0.060, -0.020, radius=0.015, x_shift=0.004),
            xy_loop(0.078, 0.064, -0.122, radius=0.016, x_shift=0.006),
            xy_loop(0.066, 0.056, -0.240, radius=0.014, x_shift=0.004),
            xy_loop(0.058, 0.050, -0.286, radius=0.012, x_shift=0.002),
        ]
    )
    shank.visual(
        mesh_from_geometry(shank_shell, "shank_shell"),
        material=painted_metal,
        name="shank_shell",
    )
    shank.visual(
        Cylinder(radius=0.031, length=0.074),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer_dark,
        name="knee_hub",
    )
    shank.visual(
        Box((0.010, 0.036, 0.154)),
        origin=Origin(xyz=(0.022, 0.0, -0.166)),
        material=polymer_dark,
        name="shank_actuator_bay",
    )
    shank.visual(
        Box((0.028, 0.036, 0.108)),
        origin=Origin(xyz=(0.018, 0.0, -0.082)),
        material=polymer_dark,
        name="shank_spine",
    )
    shank.visual(
        Box((0.060, 0.084, 0.032)),
        origin=Origin(xyz=(0.004, 0.0, -0.260)),
        material=polymer_dark,
        name="shank_ankle_bridge",
    )
    for side, side_name in ((1.0, "left"), (-1.0, "right")):
        shank.visual(
            Box((0.014, 0.010, 0.065)),
            origin=Origin(xyz=(0.020, side * 0.036, -0.307)),
            material=painted_metal,
            name=f"ankle_front_strut_{side_name}",
        )
        shank.visual(
            Box((0.014, 0.010, 0.063)),
            origin=Origin(xyz=(-0.012, side * 0.036, -0.306)),
            material=painted_metal,
            name=f"ankle_rear_strut_{side_name}",
        )
        shank.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(
                xyz=(0.0, side * 0.037, -0.340),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=polymer_dark,
            name=f"ankle_bearing_{side_name}",
        )

    foot = model.part("foot")
    foot_shell = refined_loft(
        [
            yz_loop(0.042, 0.030, 0.032, radius=0.008, z_shift=-0.024),
            yz_loop(0.070, 0.046, 0.092, radius=0.014, z_shift=-0.030),
            yz_loop(0.074, 0.042, 0.162, radius=0.014, z_shift=-0.036),
            yz_loop(0.060, 0.032, 0.214, radius=0.010, z_shift=-0.040),
            yz_loop(0.040, 0.018, 0.246, radius=0.006, z_shift=-0.042),
        ]
    )
    foot.visual(
        mesh_from_geometry(foot_shell, "foot_shell"),
        material=painted_metal,
        name="foot_shell",
    )
    foot.visual(
        Cylinder(radius=0.026, length=0.062),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polymer_dark,
        name="ankle_hub",
    )
    foot.visual(
        Box((0.060, 0.038, 0.034)),
        origin=Origin(xyz=(0.018, 0.0, -0.018)),
        material=polymer_dark,
        name="ankle_neck",
    )
    sole = ExtrudeGeometry(rounded_rect_profile(0.240, 0.084, 0.018), 0.012)
    foot.visual(
        mesh_from_geometry(sole, "foot_sole"),
        origin=Origin(xyz=(0.095, 0.0, -0.052)),
        material=elastomer_black,
        name="sole",
    )
    foot.visual(
        Box((0.032, 0.060, 0.014)),
        origin=Origin(xyz=(0.212, 0.0, -0.051)),
        material=elastomer_black,
        name="toe_bumper",
    )

    hip_joint = model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_housing,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=4.0,
            lower=math.radians(-25.0),
            upper=math.radians(70.0),
        ),
    )
    knee_joint = model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=5.0,
            lower=0.0,
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=5.0,
            lower=math.radians(-35.0),
            upper=math.radians(25.0),
        ),
    )

    hip_joint.meta["design_intent"] = "positive flexion swings the thigh forward"
    knee_joint.meta["design_intent"] = "positive flexion folds the shank rearward"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_housing = object_model.get_part("hip_housing")
    thigh = object_model.get_part("thigh")
    shank = object_model.get_part("shank")
    foot = object_model.get_part("foot")
    hip_joint = object_model.get_articulation("hip_pitch")
    knee_joint = object_model.get_articulation("knee_pitch")
    ankle_joint = object_model.get_articulation("ankle_pitch")

    def aabb_center(part):
        bounds = ctx.part_world_aabb(part)
        if bounds is None:
            return None
        minimum, maximum = bounds
        return tuple((lo + hi) * 0.5 for lo, hi in zip(minimum, maximum))

    def aabb_extrema(part):
        bounds = ctx.part_world_aabb(part)
        if bounds is None:
            return None
        minimum, maximum = bounds
        return minimum, maximum

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

    ctx.expect_overlap(hip_housing, thigh, axes="xy", min_overlap=0.045, name="hip_joint_regions_align")
    ctx.expect_overlap(thigh, shank, axes="xy", min_overlap=0.040, name="knee_joint_regions_align")
    ctx.expect_overlap(shank, foot, axes="xy", min_overlap=0.035, name="ankle_joint_regions_align")

    hip_limits = hip_joint.motion_limits
    knee_limits = knee_joint.motion_limits
    ankle_limits = ankle_joint.motion_limits
    ctx.check(
        "articulation_limits_are_plausible",
        hip_limits is not None
        and knee_limits is not None
        and ankle_limits is not None
        and hip_limits.lower is not None
        and hip_limits.upper is not None
        and knee_limits.lower == 0.0
        and knee_limits.upper is not None
        and ankle_limits.lower is not None
        and ankle_limits.upper is not None
        and hip_limits.lower < 0.0 < hip_limits.upper
        and knee_limits.upper > math.radians(100.0)
        and ankle_limits.lower < 0.0 < ankle_limits.upper,
        details="Hip, knee, and ankle should read as plausible load-bearing pitch joints.",
    )

    rest_thigh_center = aabb_center(thigh)
    with ctx.pose({hip_joint: math.radians(55.0)}):
        flexed_thigh_center = aabb_center(thigh)
    ctx.check(
        "hip_positive_motion_moves_thigh_forward",
        rest_thigh_center is not None
        and flexed_thigh_center is not None
        and flexed_thigh_center[0] > rest_thigh_center[0] + 0.10,
        details="Positive hip flexion should carry the thigh mass forward in +X.",
    )

    rest_shank_center = aabb_center(shank)
    with ctx.pose({knee_joint: math.radians(95.0)}):
        flexed_shank_center = aabb_center(shank)
    ctx.check(
        "knee_positive_motion_folds_shank_rearward",
        rest_shank_center is not None
        and flexed_shank_center is not None
        and flexed_shank_center[0] < rest_shank_center[0] - 0.08,
        details="Positive knee flexion should draw the shank rearward in -X.",
    )

    rest_foot_bounds = aabb_extrema(foot)
    with ctx.pose({ankle_joint: math.radians(20.0)}):
        dorsiflexed_foot_bounds = aabb_extrema(foot)
    ctx.check(
        "ankle_positive_motion_lifts_toe",
        rest_foot_bounds is not None
        and dorsiflexed_foot_bounds is not None
        and dorsiflexed_foot_bounds[1][2] > rest_foot_bounds[1][2] + 0.015,
        details="Positive ankle motion should increase the foot's upper Z extent like toe-up dorsiflexion.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
