from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_camera_mount")

    mount_gray = model.material("mount_gray", rgba=(0.66, 0.68, 0.71, 1.0))
    bracket_dark = model.material("bracket_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    head_gray = model.material("head_gray", rgba=(0.80, 0.82, 0.84, 1.0))
    smoked_dome = model.material("smoked_dome", rgba=(0.11, 0.13, 0.16, 0.38))
    lens_black = model.material("lens_black", rgba=(0.03, 0.04, 0.05, 1.0))

    base = model.part("base_mount")
    pan_bracket = model.part("pan_bracket")
    camera_head = model.part("camera_head")

    plate_x = 0.240
    plate_y = 0.160
    plate_z = 0.010

    arm_angle = 24.0 * pi / 180.0
    arm_len = 0.110
    arm_width = 0.050
    arm_height = 0.026
    arm_axis_x = cos(arm_angle)
    arm_axis_z = sin(arm_angle)
    arm_root = (-0.010, 0.0, 0.034)
    pan_seat_lift = 0.5 * arm_height * cos(arm_angle)
    pan_origin = (
        arm_root[0] + arm_len * arm_axis_x,
        0.0,
        arm_root[2] + arm_len * arm_axis_z + pan_seat_lift,
    )

    base.visual(
        Box((plate_x, plate_y, plate_z)),
        origin=Origin(xyz=(0.0, 0.0, plate_z * 0.5)),
        material=mount_gray,
        name="base_plate",
    )
    base.visual(
        Box((0.064, 0.072, 0.036)),
        origin=Origin(xyz=(-0.008, 0.0, 0.028)),
        material=mount_gray,
        name="riser_pedestal",
    )
    base.visual(
        Box((arm_len, arm_width, arm_height)),
        origin=Origin(
            xyz=(
                arm_root[0] + 0.5 * arm_len * arm_axis_x,
                0.0,
                arm_root[2] + 0.5 * arm_len * arm_axis_z,
            ),
            rpy=(0.0, -arm_angle, 0.0),
        ),
        material=mount_gray,
        name="riser_arm",
    )
    base.visual(
        Box((0.072, 0.046, 0.012)),
        origin=Origin(
            xyz=(
                arm_root[0] + 0.028 * arm_axis_x,
                0.0,
                arm_root[2] + 0.028 * arm_axis_z - 0.017,
            ),
            rpy=(0.0, -arm_angle, 0.0),
        ),
        material=mount_gray,
        name="riser_gusset",
    )
    base.visual(
        Box((0.050, 0.050, 0.008)),
        origin=Origin(
            xyz=(pan_origin[0], 0.0, pan_origin[2] - 0.004),
        ),
        material=mount_gray,
        name="tip_pad",
    )

    for index, (bx, by) in enumerate(
        (
            (-0.084, -0.054),
            (-0.084, 0.054),
            (0.084, -0.054),
            (0.084, 0.054),
        ),
        start=1,
    ):
        base.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(bx, by, plate_z + 0.002)),
            material=bracket_dark,
            name=f"bolt_pad_{index}",
        )

    pan_bracket.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=bracket_dark,
        name="pan_base_disc",
    )
    pan_bracket.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=bracket_dark,
        name="pan_neck",
    )
    pan_bracket.visual(
        Box((0.036, 0.088, 0.008)),
        origin=Origin(xyz=(0.018, 0.0, 0.024)),
        material=bracket_dark,
        name="lower_yoke_bridge",
    )
    pan_bracket.visual(
        Box((0.040, 0.008, 0.032)),
        origin=Origin(xyz=(0.036, -0.044, 0.044)),
        material=bracket_dark,
        name="left_yoke_arm",
    )
    pan_bracket.visual(
        Box((0.040, 0.008, 0.032)),
        origin=Origin(xyz=(0.036, 0.044, 0.044)),
        material=bracket_dark,
        name="right_yoke_arm",
    )
    pan_bracket.visual(
        Box((0.026, 0.088, 0.008)),
        origin=Origin(xyz=(0.055, 0.0, 0.056)),
        material=bracket_dark,
        name="upper_yoke_bridge",
    )

    camera_head.visual(
        Cylinder(radius=0.034, length=0.016),
        origin=Origin(xyz=(0.016, 0.0, -0.010)),
        material=head_gray,
        name="upper_cap",
    )
    camera_head.visual(
        Sphere(radius=0.037),
        origin=Origin(xyz=(0.018, 0.0, -0.053)),
        material=smoked_dome,
        name="dome_bubble",
    )
    camera_head.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=head_gray,
        name="left_trunnion",
    )
    camera_head.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.035, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=head_gray,
        name="right_trunnion",
    )
    camera_head.visual(
        Box((0.030, 0.028, 0.008)),
        origin=Origin(xyz=(0.039, 0.0, -0.027)),
        material=head_gray,
        name="visor",
    )
    camera_head.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.049, 0.0, -0.039), rpy=(0.0, pi * 0.5, 0.0)),
        material=lens_black,
        name="lens_shroud",
    )

    model.articulation(
        "base_to_pan",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pan_bracket,
        origin=Origin(xyz=pan_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_bracket,
        child=camera_head,
        origin=Origin(xyz=(0.055, 0.0, 0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.95,
        ),
    )

    return model


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_mount")
    pan_bracket = object_model.get_part("pan_bracket")
    camera_head = object_model.get_part("camera_head")
    pan_joint = object_model.get_articulation("base_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_tilt")

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
        "pan joint is vertical revolute",
        pan_joint.articulation_type == ArticulationType.REVOLUTE
        and pan_joint.axis == (0.0, 0.0, 1.0)
        and pan_joint.motion_limits is not None
        and pan_joint.motion_limits.lower is not None
        and pan_joint.motion_limits.upper is not None
        and pan_joint.motion_limits.lower <= -2.5
        and pan_joint.motion_limits.upper >= 2.5,
        details=(
            f"type={pan_joint.articulation_type}, axis={pan_joint.axis}, "
            f"limits={pan_joint.motion_limits}"
        ),
    )
    ctx.check(
        "tilt joint is horizontal revolute",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.axis == (0.0, 1.0, 0.0)
        and tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower is not None
        and tilt_joint.motion_limits.upper is not None
        and tilt_joint.motion_limits.lower < 0.0 < tilt_joint.motion_limits.upper,
        details=(
            f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}, "
            f"limits={tilt_joint.motion_limits}"
        ),
    )

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_gap(
            pan_bracket,
            base,
            axis="z",
            positive_elem="pan_base_disc",
            negative_elem="tip_pad",
            max_gap=0.001,
            max_penetration=0.0,
            name="pan base seats on riser tip pad",
        )
        ctx.expect_overlap(
            pan_bracket,
            base,
            axes="xy",
            elem_a="pan_base_disc",
            elem_b="tip_pad",
            min_overlap=0.038,
            name="pan base disc stays fully supported by tip pad",
        )
        ctx.expect_contact(
            camera_head,
            pan_bracket,
            elem_a="left_trunnion",
            elem_b="left_yoke_arm",
            name="left trunnion contacts left yoke arm",
        )
        ctx.expect_contact(
            camera_head,
            pan_bracket,
            elem_a="right_trunnion",
            elem_b="right_yoke_arm",
            name="right trunnion contacts right yoke arm",
        )

    lens_rest_center = _aabb_center(
        ctx.part_element_world_aabb(camera_head, elem="lens_shroud")
    )
    with ctx.pose({pan_joint: 0.9, tilt_joint: 0.0}):
        lens_panned_center = _aabb_center(
            ctx.part_element_world_aabb(camera_head, elem="lens_shroud")
        )
    ctx.check(
        "pan rotation swings the camera head around the vertical axis",
        lens_rest_center is not None
        and lens_panned_center is not None
        and lens_panned_center[1] > lens_rest_center[1] + 0.02,
        details=f"rest={lens_rest_center}, panned={lens_panned_center}",
    )

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.65}):
        lens_tilted_center = _aabb_center(
            ctx.part_element_world_aabb(camera_head, elem="lens_shroud")
        )
    ctx.check(
        "positive tilt drives the lens downward on the horizontal knuckle axis",
        lens_rest_center is not None
        and lens_tilted_center is not None
        and lens_tilted_center[2] < lens_rest_center[2] - 0.015,
        details=f"rest={lens_rest_center}, tilted={lens_tilted_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
