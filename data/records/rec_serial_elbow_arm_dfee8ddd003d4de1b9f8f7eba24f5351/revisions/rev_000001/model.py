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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

SHOULDER_AXIS_X = 0.045
SHOULDER_AXIS_Z = 0.145
ELBOW_AXIS_X = 0.245


CYLINDER_Y_RPY = (pi / 2.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_serial_elbow_arm")

    housing_color = model.material("housing_color", color=(0.18, 0.19, 0.21, 1.0))
    upper_color = model.material("upper_color", color=(0.63, 0.67, 0.71, 1.0))
    forearm_color = model.material("forearm_color", color=(0.46, 0.50, 0.56, 1.0))
    pad_color = model.material("pad_color", color=(0.10, 0.10, 0.11, 1.0))

    housing = model.part("shoulder_housing")
    housing.visual(
        Box((0.18, 0.16, 0.018)),
        origin=Origin(xyz=(-0.025, 0.0, 0.009)),
        material=housing_color,
        name="base_plate",
    )
    housing.visual(
        Box((0.094, 0.094, 0.108)),
        origin=Origin(xyz=(-0.047, 0.0, 0.072)),
        material=housing_color,
        name="column",
    )
    housing.visual(
        Box((0.040, 0.048, 0.030)),
        origin=Origin(xyz=(0.002, 0.0, 0.111)),
        material=housing_color,
        name="shoulder_nose",
    )
    housing.visual(
        Box((0.050, 0.018, 0.070)),
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.029, SHOULDER_AXIS_Z)),
        material=housing_color,
        name="left_yoke_ear",
    )
    housing.visual(
        Box((0.050, 0.018, 0.070)),
        origin=Origin(xyz=(SHOULDER_AXIS_X, -0.029, SHOULDER_AXIS_Z)),
        material=housing_color,
        name="right_yoke_ear",
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=CYLINDER_Y_RPY),
        material=upper_color,
        name="shoulder_knuckle",
    )
    upper_link.visual(
        Box((0.040, 0.026, 0.034)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=upper_color,
        name="shoulder_neck",
    )
    upper_link.visual(
        Box((0.144, 0.034, 0.048)),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=upper_color,
        name="upper_beam",
    )
    upper_link.visual(
        Box((0.040, 0.060, 0.040)),
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        material=upper_color,
        name="clevis_root",
    )
    upper_link.visual(
        Box((0.050, 0.020, 0.080)),
        origin=Origin(xyz=(0.235, 0.030, 0.0)),
        material=upper_color,
        name="left_clevis_ear",
    )
    upper_link.visual(
        Box((0.050, 0.020, 0.080)),
        origin=Origin(xyz=(0.235, -0.030, 0.0)),
        material=upper_color,
        name="right_clevis_ear",
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.034, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=CYLINDER_Y_RPY),
        material=forearm_color,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.070, 0.022, 0.032)),
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
        material=forearm_color,
        name="forearm_beam",
    )
    forearm.visual(
        Box((0.024, 0.028, 0.024)),
        origin=Origin(xyz=(0.116, 0.0, 0.0)),
        material=forearm_color,
        name="pad_neck",
    )
    forearm.visual(
        Box((0.054, 0.068, 0.016)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=pad_color,
        name="end_pad",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=upper_link,
        origin=Origin(xyz=(SHOULDER_AXIS_X, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-1.20,
            upper=1.20,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=forearm,
        origin=Origin(xyz=(ELBOW_AXIS_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=2.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_size(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    ctx = TestContext(object_model)
    housing = object_model.get_part("shoulder_housing")
    upper_link = object_model.get_part("upper_link")
    forearm = object_model.get_part("forearm")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

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
    ctx.expect_origin_gap(
        upper_link,
        housing,
        axis="z",
        min_gap=0.13,
        max_gap=0.16,
        name="upper link shoulder axis sits above housing base",
    )
    ctx.expect_origin_gap(
        forearm,
        upper_link,
        axis="x",
        min_gap=0.23,
        max_gap=0.27,
        name="elbow joint sits downstream of shoulder joint",
    )
    ctx.expect_overlap(
        housing,
        upper_link,
        axes="yz",
        min_overlap=0.04,
        name="shoulder hardware shares the same joint envelope",
    )
    ctx.expect_overlap(
        upper_link,
        forearm,
        axes="yz",
        min_overlap=0.04,
        name="clevis and elbow hub align in the same joint envelope",
    )
    ctx.expect_contact(
        housing,
        upper_link,
        name="shoulder joint hardware is physically seated",
    )
    ctx.expect_contact(
        upper_link,
        forearm,
        name="elbow joint hardware is physically seated",
    )

    shoulder_axis_ok = tuple(shoulder_joint.axis) == (0.0, 1.0, 0.0)
    elbow_axis_ok = tuple(elbow_joint.axis) == (0.0, 1.0, 0.0)
    limits_ok = (
        shoulder_joint.motion_limits is not None
        and elbow_joint.motion_limits is not None
        and shoulder_joint.motion_limits.lower is not None
        and shoulder_joint.motion_limits.upper is not None
        and elbow_joint.motion_limits.lower is not None
        and elbow_joint.motion_limits.upper is not None
    )
    ctx.check(
        "parallel horizontal joint axes",
        shoulder_axis_ok and elbow_axis_ok and limits_ok,
        details=(
            f"shoulder axis={shoulder_joint.axis}, elbow axis={elbow_joint.axis}, "
            f"shoulder limits={shoulder_joint.motion_limits}, elbow limits={elbow_joint.motion_limits}"
        ),
    )

    upper_aabb = ctx.part_world_aabb(upper_link)
    forearm_aabb = ctx.part_world_aabb(forearm)
    if upper_aabb is None or forearm_aabb is None:
        ctx.fail("arm bounds resolved", "upper_link or forearm AABB was unavailable")
    else:
        upper_size = aabb_size(upper_aabb)
        forearm_size = aabb_size(forearm_aabb)
        ctx.check(
            "upper link longer than forearm",
            upper_size[0] > forearm_size[0] + 0.035,
            details=f"upper={upper_size}, forearm={forearm_size}",
        )
        ctx.check(
            "links have visibly different cross sections",
            upper_size[1] > forearm_size[1] + 0.008
            and upper_size[2] > forearm_size[2] + 0.008,
            details=f"upper={upper_size}, forearm={forearm_size}",
        )

    end_pad_rest = ctx.part_element_world_aabb(forearm, elem="end_pad")
    if end_pad_rest is None:
        ctx.fail("end pad present", "end_pad AABB was unavailable")
    else:
        rest_center = aabb_center(end_pad_rest)
        with ctx.pose({shoulder_joint: 0.70, elbow_joint: 0.0}):
            pad_after_shoulder = ctx.part_element_world_aabb(forearm, elem="end_pad")
        with ctx.pose({shoulder_joint: 0.0, elbow_joint: 1.30}):
            pad_after_elbow = ctx.part_element_world_aabb(forearm, elem="end_pad")

        if pad_after_shoulder is None or pad_after_elbow is None:
            ctx.fail("pad pose samples available", "end_pad AABB missing in one or more articulated poses")
        else:
            shoulder_center = aabb_center(pad_after_shoulder)
            elbow_center = aabb_center(pad_after_elbow)
            shoulder_delta = (
                shoulder_center[0] - rest_center[0],
                shoulder_center[1] - rest_center[1],
                shoulder_center[2] - rest_center[2],
            )
            elbow_delta = (
                elbow_center[0] - rest_center[0],
                elbow_center[1] - rest_center[1],
                elbow_center[2] - rest_center[2],
            )
            ctx.check(
                "shoulder sweeps arm in xz plane",
                abs(shoulder_delta[2]) > 0.080 and abs(shoulder_delta[1]) < 0.010,
                details=f"rest={rest_center}, shoulder_pose={shoulder_center}, delta={shoulder_delta}",
            )
            ctx.check(
                "elbow folds forearm in xz plane",
                abs(elbow_delta[2]) > 0.070 and abs(elbow_delta[1]) < 0.010,
                details=f"rest={rest_center}, elbow_pose={elbow_center}, delta={elbow_delta}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
