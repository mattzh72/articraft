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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parapet_cctv_bracket")

    bracket_gray = model.material("bracket_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    zinc = model.material("zinc", rgba=(0.72, 0.74, 0.76, 1.0))
    camera_white = model.material("camera_white", rgba=(0.90, 0.92, 0.93, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.18, 0.24, 0.28, 1.0))

    mount_bracket = model.part("mount_bracket")
    mount_bracket.visual(
        Box((0.012, 0.145, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bracket_gray,
        name="front_clamp_plate",
    )
    mount_bracket.visual(
        Box((0.012, 0.125, 0.165)),
        origin=Origin(xyz=(0.158, 0.0, 0.0275)),
        material=bracket_gray,
        name="rear_clamp_plate",
    )
    mount_bracket.visual(
        Box((0.170, 0.125, 0.014)),
        origin=Origin(xyz=(0.079, 0.0, 0.117)),
        material=bracket_gray,
        name="top_clamp_bridge",
    )
    mount_bracket.visual(
        Box((0.030, 0.082, 0.052)),
        origin=Origin(xyz=(0.018, 0.0, 0.054)),
        material=bracket_gray,
        name="arm_base_block",
    )
    mount_bracket.visual(
        Box((0.205, 0.042, 0.034)),
        origin=Origin(xyz=(0.1085, 0.0, 0.055)),
        material=bracket_gray,
        name="horizontal_arm",
    )
    mount_bracket.visual(
        Box((0.038, 0.064, 0.014)),
        origin=Origin(xyz=(0.194, 0.0, 0.079)),
        material=bracket_gray,
        name="pan_seat",
    )
    mount_bracket.visual(
        Box((0.096, 0.010, 0.040)),
        origin=Origin(xyz=(0.045, 0.018, 0.022), rpy=(0.0, -0.78, 0.0)),
        material=bracket_gray,
        name="left_arm_brace",
    )
    mount_bracket.visual(
        Box((0.096, 0.010, 0.040)),
        origin=Origin(xyz=(0.045, -0.018, 0.022), rpy=(0.0, -0.78, 0.0)),
        material=bracket_gray,
        name="right_arm_brace",
    )

    for side_name, side_y in (("upper", 0.031), ("lower", -0.031)):
        mount_bracket.visual(
            Cylinder(radius=0.005, length=0.082),
            origin=Origin(
                xyz=(0.123, side_y, -0.018),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"{side_name}_clamp_screw_shaft",
        )
        mount_bracket.visual(
            Cylinder(radius=0.010, length=0.008),
            origin=Origin(
                xyz=(0.078, side_y, -0.018),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"{side_name}_clamp_pad",
        )
        mount_bracket.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(
                xyz=(0.169, side_y, -0.018),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=zinc,
            name=f"{side_name}_clamp_knob",
        )

    mount_bracket.inertial = Inertial.from_geometry(
        Box((0.230, 0.170, 0.245)),
        mass=4.2,
        origin=Origin(xyz=(0.092, 0.0, 0.036)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=matte_black,
        name="azimuth_bearing",
    )
    pan_head.visual(
        Box((0.018, 0.030, 0.092)),
        origin=Origin(xyz=(-0.016, 0.0, 0.072)),
        material=matte_black,
        name="pan_post",
    )
    pan_head.visual(
        Box((0.018, 0.012, 0.082)),
        origin=Origin(xyz=(-0.006, 0.046, 0.072)),
        material=matte_black,
        name="left_yoke_ear",
    )
    pan_head.visual(
        Box((0.018, 0.012, 0.082)),
        origin=Origin(xyz=(-0.006, -0.046, 0.072)),
        material=matte_black,
        name="right_yoke_ear",
    )
    pan_head.visual(
        Box((0.018, 0.092, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.117)),
        material=matte_black,
        name="yoke_top_bridge",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.060, 0.100, 0.130)),
        mass=0.55,
        origin=Origin(xyz=(-0.006, 0.0, 0.066)),
    )

    camera_housing = model.part("camera_housing")
    camera_housing.visual(
        Cylinder(radius=0.039, length=0.180),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="camera_body",
    )
    camera_housing.visual(
        Box((0.155, 0.094, 0.008)),
        origin=Origin(xyz=(0.102, 0.0, 0.043)),
        material=camera_white,
        name="sunshield",
    )
    camera_housing.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.173, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_bezel",
    )
    camera_housing.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.183, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_window",
    )
    camera_housing.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="left_trunnion",
    )
    camera_housing.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="right_trunnion",
    )
    camera_housing.visual(
        Box((0.042, 0.048, 0.026)),
        origin=Origin(xyz=(0.034, 0.0, -0.028)),
        material=matte_black,
        name="service_pod",
    )
    camera_housing.inertial = Inertial.from_geometry(
        Box((0.190, 0.100, 0.095)),
        mass=1.1,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )

    model.articulation(
        "mount_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=mount_bracket,
        child=pan_head,
        origin=Origin(xyz=(0.194, 0.0, 0.086)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.5),
    )
    model.articulation(
        "pan_head_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=math.radians(-20.0),
            upper=math.radians(85.0),
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

    mount_bracket = object_model.get_part("mount_bracket")
    pan_head = object_model.get_part("pan_head")
    camera_housing = object_model.get_part("camera_housing")
    pan_joint = object_model.get_articulation("mount_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_camera")

    ctx.check(
        "pan joint is continuous azimuth",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(pan_joint.axis) == (0.0, 0.0, 1.0)
        and pan_joint.motion_limits is not None
        and pan_joint.motion_limits.lower is None
        and pan_joint.motion_limits.upper is None,
        details=(
            f"type={pan_joint.articulation_type}, axis={pan_joint.axis}, "
            f"limits={pan_joint.motion_limits}"
        ),
    )
    ctx.check(
        "tilt joint is revolute about horizontal y axis",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(tilt_joint.axis) == (0.0, 1.0, 0.0)
        and tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower is not None
        and tilt_joint.motion_limits.upper is not None
        and tilt_joint.motion_limits.upper > 1.2
        and tilt_joint.motion_limits.lower < 0.0,
        details=(
            f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}, "
            f"limits={tilt_joint.motion_limits}"
        ),
    )

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_contact(
            pan_head,
            mount_bracket,
            elem_a="azimuth_bearing",
            elem_b="pan_seat",
            contact_tol=1e-6,
            name="azimuth bearing seats on arm platform",
        )
        ctx.expect_contact(
            pan_head,
            camera_housing,
            elem_a="left_yoke_ear",
            elem_b="left_trunnion",
            contact_tol=1e-6,
            name="left trunnion bears on left yoke ear",
        )
        ctx.expect_overlap(
            camera_housing,
            pan_head,
            axes="z",
            elem_a="left_trunnion",
            elem_b="left_yoke_ear",
            min_overlap=0.020,
            name="left trunnion aligns vertically with yoke ear",
        )

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        rest_front = elem_center(camera_housing, "front_bezel")
        rest_axis = ctx.part_world_position(pan_head)

    with ctx.pose({pan_joint: math.pi / 2.0, tilt_joint: 0.0}):
        quarter_turn_front = elem_center(camera_housing, "front_bezel")
        quarter_turn_axis = ctx.part_world_position(pan_head)

    ctx.check(
        "azimuth rotation swings the camera head sideways",
        rest_front is not None
        and rest_axis is not None
        and quarter_turn_front is not None
        and quarter_turn_axis is not None
        and rest_front[0] > rest_axis[0] + 0.12
        and abs(quarter_turn_front[0] - quarter_turn_axis[0]) < 0.03
        and quarter_turn_front[1] > quarter_turn_axis[1] + 0.12,
        details=(
            f"rest_front={rest_front}, rest_axis={rest_axis}, "
            f"quarter_turn_front={quarter_turn_front}, "
            f"quarter_turn_axis={quarter_turn_axis}"
        ),
    )

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        level_front = elem_center(camera_housing, "front_bezel")

    with ctx.pose({pan_joint: 0.0, tilt_joint: math.radians(70.0)}):
        tilted_front = elem_center(camera_housing, "front_bezel")

    ctx.check(
        "positive tilt drives the camera nose downward",
        level_front is not None
        and tilted_front is not None
        and tilted_front[2] < level_front[2] - 0.10
        and tilted_front[0] < level_front[0] - 0.03,
        details=f"level_front={level_front}, tilted_front={tilted_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
