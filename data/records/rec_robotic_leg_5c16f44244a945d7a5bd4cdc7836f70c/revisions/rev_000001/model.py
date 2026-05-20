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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_leg_module")

    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.11, 0.12, 0.13, 1.0))
    titanium = model.material("titanium", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    amber = model.material("amber", rgba=(0.83, 0.54, 0.14, 1.0))

    upper_housing = model.part("upper_housing")
    upper_housing.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.22)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )
    upper_housing.visual(
        Box((0.18, 0.15, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_graphite,
        name="top_cap",
    )
    upper_housing.visual(
        Box((0.15, 0.026, 0.17)),
        origin=Origin(xyz=(0.0, 0.057, -0.135)),
        material=graphite,
        name="left_yoke",
    )
    upper_housing.visual(
        Box((0.15, 0.026, 0.17)),
        origin=Origin(xyz=(0.0, -0.057, -0.135)),
        material=graphite,
        name="right_yoke",
    )
    upper_housing.visual(
        Box((0.045, 0.032, 0.17)),
        origin=Origin(xyz=(-0.055, 0.041, -0.135)),
        material=dark_graphite,
        name="left_rear_rib",
    )
    upper_housing.visual(
        Box((0.045, 0.032, 0.17)),
        origin=Origin(xyz=(-0.055, -0.041, -0.135)),
        material=dark_graphite,
        name="right_rear_rib",
    )
    upper_housing.visual(
        Box((0.04, 0.15, 0.08)),
        origin=Origin(xyz=(0.07, 0.0, -0.08)),
        material=graphite,
        name="front_bridge",
    )
    upper_housing.visual(
        Box((0.035, 0.09, 0.10)),
        origin=Origin(xyz=(-0.0725, 0.0, -0.11)),
        material=dark_graphite,
        name="motor_block",
    )
    upper_housing.visual(
        _mesh(
            "hip_guard_hoop",
            tube_from_spline_points(
                [
                    (-0.055, 0.052, -0.145),
                    (-0.015, 0.085, -0.13),
                    (0.07, 0.093, -0.115),
                    (0.155, 0.0, -0.11),
                    (0.07, -0.093, -0.115),
                    (-0.015, -0.085, -0.13),
                    (-0.055, -0.052, -0.145),
                ],
                radius=0.008,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=titanium,
        name="hip_guard_hoop",
    )

    thigh_link = model.part("thigh_link")
    thigh_link.inertial = Inertial.from_geometry(
        Box((0.13, 0.08, 0.42)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.21)),
    )
    thigh_link.visual(
        Box((0.07, 0.05, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=titanium,
        name="hip_clevis",
    )
    thigh_link.visual(
        Box((0.10, 0.05, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=graphite,
        name="thigh_beam",
    )
    thigh_link.visual(
        Box((0.10, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.285)),
        material=dark_graphite,
        name="knee_bridge",
    )
    thigh_link.visual(
        Box((0.09, 0.016, 0.075)),
        origin=Origin(xyz=(0.0, 0.028, -0.3425)),
        material=titanium,
        name="knee_left_yoke",
    )
    thigh_link.visual(
        Box((0.09, 0.016, 0.075)),
        origin=Origin(xyz=(0.0, -0.028, -0.3425)),
        material=titanium,
        name="knee_right_yoke",
    )
    thigh_link.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.036, -0.36), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=titanium,
        name="knee_left_boss",
    )
    thigh_link.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, -0.036, -0.36), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=titanium,
        name="knee_right_boss",
    )
    thigh_link.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="thigh_sensor_pod",
    )

    shank_link = model.part("shank_link")
    shank_link.inertial = Inertial.from_geometry(
        Box((0.10, 0.08, 0.38)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, -0.19)),
    )
    shank_link.visual(
        Box((0.065, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.0225)),
        material=titanium,
        name="knee_clevis",
    )
    shank_link.visual(
        Box((0.08, 0.045, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=graphite,
        name="shank_beam",
    )
    shank_link.visual(
        Box((0.08, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=dark_graphite,
        name="ankle_bridge",
    )
    shank_link.visual(
        Box((0.07, 0.014, 0.07)),
        origin=Origin(xyz=(0.0, 0.024, -0.305)),
        material=titanium,
        name="ankle_left_yoke",
    )
    shank_link.visual(
        Box((0.07, 0.014, 0.07)),
        origin=Origin(xyz=(0.0, -0.024, -0.305)),
        material=titanium,
        name="ankle_right_yoke",
    )
    shank_link.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.031, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=titanium,
        name="ankle_left_boss",
    )
    shank_link.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, -0.031, -0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=titanium,
        name="ankle_right_boss",
    )

    foot_section = model.part("foot_section")
    foot_section.inertial = Inertial.from_geometry(
        Box((0.20, 0.12, 0.16)),
        mass=2.2,
        origin=Origin(xyz=(0.05, 0.0, -0.08)),
    )
    foot_section.visual(
        Box((0.055, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=titanium,
        name="ankle_clevis",
    )
    foot_section.visual(
        Box((0.11, 0.065, 0.08)),
        origin=Origin(xyz=(0.015, 0.0, -0.08)),
        material=graphite,
        name="ankle_block",
    )
    foot_section.visual(
        Box((0.22, 0.11, 0.04)),
        origin=Origin(xyz=(0.07, 0.0, -0.14)),
        material=rubber,
        name="foot_pad",
    )
    foot_section.visual(
        Box((0.075, 0.055, 0.025)),
        origin=Origin(xyz=(0.15, 0.0, -0.1075)),
        material=amber,
        name="toe_cap",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_housing,
        child=thigh_link,
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.2,
            lower=math.radians(-55.0),
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh_link,
        child=shank_link,
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(130.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank_link,
        child=foot_section,
        origin=Origin(xyz=(0.0, 0.0, -0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=3.0,
            lower=math.radians(-35.0),
            upper=math.radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    upper_housing = object_model.get_part("upper_housing")
    thigh_link = object_model.get_part("thigh_link")
    shank_link = object_model.get_part("shank_link")
    foot_section = object_model.get_part("foot_section")

    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")

    ctx.expect_overlap(
        upper_housing,
        thigh_link,
        axes="xy",
        elem_a="hip_guard_hoop",
        elem_b="hip_clevis",
        min_overlap=0.05,
        name="guard hoop wraps around the hip-stage footprint",
    )
    ctx.expect_origin_gap(
        thigh_link,
        shank_link,
        axis="z",
        min_gap=0.30,
        name="shank hangs below the thigh at rest",
    )
    ctx.expect_origin_gap(
        shank_link,
        foot_section,
        axis="z",
        min_gap=0.30,
        name="foot hangs below the shank at rest",
    )

    rest_foot_pos = ctx.part_world_position(foot_section)
    with ctx.pose({hip: hip.motion_limits.lower}):
        hip_forward_foot_pos = ctx.part_world_position(foot_section)
    with ctx.pose({hip: hip.motion_limits.upper}):
        hip_backward_foot_pos = ctx.part_world_position(foot_section)
    ctx.check(
        "hip swings the distal chain fore and aft",
        (
            rest_foot_pos is not None
            and hip_forward_foot_pos is not None
            and hip_backward_foot_pos is not None
            and hip_forward_foot_pos[0] > rest_foot_pos[0] + 0.45
            and hip_backward_foot_pos[0] < rest_foot_pos[0] - 0.45
        ),
        details=(
            f"rest={rest_foot_pos}, hip_lower={hip_forward_foot_pos}, "
            f"hip_upper={hip_backward_foot_pos}"
        ),
    )

    with ctx.pose({knee: knee.motion_limits.upper}):
        knee_fold_foot_pos = ctx.part_world_position(foot_section)
    ctx.check(
        "knee fold lifts the foot upward",
        (
            rest_foot_pos is not None
            and knee_fold_foot_pos is not None
            and knee_fold_foot_pos[2] > rest_foot_pos[2] + 0.45
        ),
        details=f"rest={rest_foot_pos}, knee_fold={knee_fold_foot_pos}",
    )

    def _aabb_center_x(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_foot_center_x = _aabb_center_x(ctx.part_world_aabb(foot_section))
    with ctx.pose({ankle: ankle.motion_limits.lower}):
        ankle_down_center_x = _aabb_center_x(ctx.part_world_aabb(foot_section))
    with ctx.pose({ankle: ankle.motion_limits.upper}):
        ankle_up_center_x = _aabb_center_x(ctx.part_world_aabb(foot_section))
    ctx.check(
        "ankle pitch changes the foot attitude",
        (
            rest_foot_center_x is not None
            and ankle_down_center_x is not None
            and ankle_up_center_x is not None
            and ankle_down_center_x > rest_foot_center_x + 0.02
            and ankle_up_center_x < rest_foot_center_x - 0.02
        ),
        details=(
            f"rest_center_x={rest_foot_center_x}, ankle_lower={ankle_down_center_x}, "
            f"ankle_upper={ankle_up_center_x}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
