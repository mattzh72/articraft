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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_articulated_vacuum")

    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    body_mid = model.material("body_mid", rgba=(0.28, 0.30, 0.33, 1.0))
    metal = model.material("metal", rgba=(0.70, 0.72, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    accent = model.material("accent", rgba=(0.16, 0.61, 0.59, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.24, 0.16, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=body_dark,
        name="lower_chassis",
    )
    body.visual(
        Box((0.14, 0.126, 0.052)),
        origin=Origin(xyz=(0.020, 0.0, 0.068)),
        material=body_mid,
        name="motor_housing",
    )
    body.visual(
        Box((0.070, 0.144, 0.024)),
        origin=Origin(xyz=(0.094, 0.0, 0.050)),
        material=accent,
        name="front_nose",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(-0.045, 0.078, 0.030), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_wheel",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.014),
        origin=Origin(xyz=(-0.045, -0.078, 0.030), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_wheel",
    )
    body.visual(
        Box((0.034, 0.034, 0.050)),
        origin=Origin(xyz=(-0.109, 0.0, 0.067)),
        material=body_dark,
        name="pivot_bridge",
    )
    body.visual(
        Box((0.018, 0.014, 0.030)),
        origin=Origin(xyz=(-0.101, 0.018, 0.092)),
        material=body_dark,
        name="left_shoulder_cheek",
    )
    body.visual(
        Box((0.018, 0.014, 0.030)),
        origin=Origin(xyz=(-0.101, -0.018, 0.092)),
        material=body_dark,
        name="right_shoulder_cheek",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.24, 0.16, 0.11)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="base_collar",
    )
    upper_wand.visual(
        Box((0.032, 0.028, 0.022)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=body_mid,
        name="base_socket",
    )
    upper_wand.visual(
        Cylinder(radius=0.011, length=0.136),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="main_tube",
    )
    upper_wand.visual(
        Box((0.060, 0.026, 0.022)),
        origin=Origin(xyz=(0.095, 0.0, 0.021)),
        material=body_dark,
        name="handle_grip",
    )
    upper_wand.visual(
        Box((0.020, 0.030, 0.022)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material=body_mid,
        name="elbow_bridge",
    )
    upper_wand.visual(
        Box((0.020, 0.014, 0.028)),
        origin=Origin(xyz=(0.170, 0.018, 0.0)),
        material=body_mid,
        name="left_elbow_cheek",
    )
    upper_wand.visual(
        Box((0.020, 0.014, 0.028)),
        origin=Origin(xyz=(0.170, -0.018, 0.0)),
        material=body_mid,
        name="right_elbow_cheek",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.19, 0.04, 0.05)),
        mass=0.55,
        origin=Origin(xyz=(0.095, 0.0, 0.010)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="base_collar",
    )
    lower_wand.visual(
        Box((0.030, 0.028, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=body_mid,
        name="base_socket",
    )
    lower_wand.visual(
        Cylinder(radius=0.010, length=0.148),
        origin=Origin(xyz=(0.104, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="main_tube",
    )
    lower_wand.visual(
        Box((0.024, 0.030, 0.020)),
        origin=Origin(xyz=(0.168, 0.0, 0.0)),
        material=body_mid,
        name="tip_bridge",
    )
    lower_wand.visual(
        Box((0.020, 0.014, 0.026)),
        origin=Origin(xyz=(0.168, 0.018, 0.0)),
        material=body_mid,
        name="left_nozzle_cheek",
    )
    lower_wand.visual(
        Box((0.020, 0.014, 0.026)),
        origin=Origin(xyz=(0.168, -0.018, 0.0)),
        material=body_mid,
        name="right_nozzle_cheek",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.18, 0.04, 0.04)),
        mass=0.40,
        origin=Origin(xyz=(0.090, 0.0, 0.0)),
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=body_mid,
        name="pivot_collar",
    )
    nozzle.visual(
        Box((0.050, 0.030, 0.080)),
        origin=Origin(xyz=(0.025, 0.0, -0.040)),
        material=body_mid,
        name="neck_block",
    )
    nozzle.visual(
        Box((0.220, 0.060, 0.024)),
        origin=Origin(xyz=(0.110, 0.0, -0.092)),
        material=body_dark,
        name="head_shell",
    )
    nozzle.visual(
        Box((0.030, 0.062, 0.010)),
        origin=Origin(xyz=(0.205, 0.0, -0.084)),
        material=accent,
        name="front_bumper",
    )
    nozzle.visual(
        Box((0.170, 0.040, 0.004)),
        origin=Origin(xyz=(0.102, 0.0, -0.102)),
        material=rubber,
        name="squeegee_strip",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.22, 0.06, 0.11)),
        mass=0.50,
        origin=Origin(xyz=(0.11, 0.0, -0.050)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_wand,
        origin=Origin(xyz=(-0.092, 0.0, 0.104)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.45,
            upper=1.55,
        ),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-1.55,
            upper=0.35,
        ),
    )
    model.articulation(
        "lower_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=nozzle,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.5,
            lower=-1.20,
            upper=0.90,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    nozzle = object_model.get_part("nozzle")
    shoulder = object_model.get_articulation("body_to_upper_wand")
    elbow = object_model.get_articulation("upper_to_lower_wand")
    nozzle_pitch = object_model.get_articulation("lower_to_nozzle")

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

    ctx.expect_contact(body, upper_wand, contact_tol=0.0015, name="body_mount_contacts_upper_wand")
    ctx.expect_contact(upper_wand, lower_wand, contact_tol=0.0015, name="upper_and_lower_wand_contact_at_elbow")
    ctx.expect_contact(lower_wand, nozzle, contact_tol=0.0015, name="lower_wand_contacts_nozzle_pivot")

    body_aabb = ctx.part_world_aabb(body)
    head_aabb = ctx.part_element_world_aabb(nozzle, elem="head_shell")
    if body_aabb is not None and head_aabb is not None:
        ctx.check(
            "deployed_head_reaches_floor_ahead_of_body",
            abs(head_aabb[0][2]) <= 0.003
            and head_aabb[1][0] > body_aabb[1][0] + 0.06
            and head_aabb[0][1] < body_aabb[1][1]
            and head_aabb[1][1] > body_aabb[0][1],
            (
                "Expected the floor head to sit on the ground and extend forward "
                "of the compact body in the default cleaning pose."
            ),
        )

    with ctx.pose(
        {
            shoulder: 1.55,
            elbow: -1.30,
            nozzle_pitch: -0.80,
        }
    ):
        folded_upper = ctx.part_world_aabb(upper_wand)
        folded_lower = ctx.part_world_aabb(lower_wand)
        folded_head = ctx.part_element_world_aabb(nozzle, elem="head_shell")
        folded_body = ctx.part_world_aabb(body)

        if (
            folded_upper is not None
            and folded_lower is not None
            and folded_head is not None
            and folded_body is not None
        ):
            folded_span_x = max(
                folded_upper[1][0],
                folded_lower[1][0],
                folded_head[1][0],
            ) - min(
                folded_body[0][0],
                folded_upper[0][0],
                folded_lower[0][0],
                folded_head[0][0],
            )
            ctx.check(
                "stowed_pose_stays_compact",
                folded_span_x <= 0.42
                and folded_head[0][2] > folded_body[1][2] - 0.005
                and folded_head[1][2] < folded_body[1][2] + 0.16
                and folded_head[0][1] >= folded_body[0][1] - 0.015
                and folded_head[1][1] <= folded_body[1][1] + 0.015,
                (
                    "Expected the folded wand and nozzle to park above the body "
                    "within a desktop-friendly stowed envelope."
                ),
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
