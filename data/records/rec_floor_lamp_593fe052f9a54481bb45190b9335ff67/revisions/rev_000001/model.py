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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_arm_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.13, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.90, 0.84, 1.0))
    brass = model.material("brass", rgba=(0.71, 0.60, 0.34, 1.0))

    base_radius = 0.17
    base_thickness = 0.035
    post_radius = 0.015
    post_height = 1.23
    shoulder_height = base_thickness + post_height
    shoulder_x = 0.040

    arm1_len = 0.42
    arm2_len = 0.385

    ear_radius_root = 0.018
    ear_radius_arm1 = 0.016
    ear_radius_arm2 = 0.014
    ear_length = 0.018
    ear_y = 0.035
    barrel_length = 0.052

    shade_profile_outer = [
        (0.014, 0.000),
        (0.019, 0.012),
        (0.041, 0.060),
        (0.069, 0.145),
        (0.082, 0.190),
    ]
    shade_profile_inner = [
        (0.0115, 0.004),
        (0.0160, 0.014),
        (0.0375, 0.060),
        (0.0650, 0.145),
        (0.0785, 0.188),
    ]
    shade_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            shade_profile_outer,
            shade_profile_inner,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "lamp_shade_shell",
    )

    base_post = model.part("base_post")
    base_post.visual(
        Cylinder(radius=base_radius, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=matte_black,
        name="base_disc",
    )
    base_post.visual(
        Cylinder(radius=post_radius, length=post_height),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + post_height / 2.0)),
        material=satin_black,
        name="vertical_post",
    )
    base_post.visual(
        Box((0.086, 0.100, 0.030)),
        origin=Origin(xyz=(-0.020, 0.0, shoulder_height)),
        material=satin_black,
        name="shoulder_bridge",
    )
    base_post.visual(
        Cylinder(radius=ear_radius_root, length=ear_length),
        origin=Origin(
            xyz=(shoulder_x, ear_y, shoulder_height),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="shoulder_left_ear",
    )
    base_post.visual(
        Cylinder(radius=ear_radius_root, length=ear_length),
        origin=Origin(
            xyz=(shoulder_x, -ear_y, shoulder_height),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="shoulder_right_ear",
    )

    arm_1 = model.part("arm_1")
    arm_1.visual(
        Cylinder(radius=0.015, length=barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="shoulder_barrel",
    )
    arm_1.visual(
        Box((0.064, 0.032, 0.028)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=satin_black,
        name="shoulder_neck",
    )
    arm_1.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.225, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=matte_black,
        name="arm_1_tube",
    )
    arm_1.visual(
        Box((0.036, 0.100, 0.028)),
        origin=Origin(xyz=(arm1_len - 0.033, 0.0, 0.0)),
        material=satin_black,
        name="elbow_bridge",
    )
    arm_1.visual(
        Cylinder(radius=ear_radius_arm1, length=ear_length),
        origin=Origin(
            xyz=(arm1_len, ear_y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="elbow_left_ear",
    )
    arm_1.visual(
        Cylinder(radius=ear_radius_arm1, length=ear_length),
        origin=Origin(
            xyz=(arm1_len, -ear_y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="elbow_right_ear",
    )

    arm_2 = model.part("arm_2")
    arm_2.visual(
        Cylinder(radius=0.0145, length=barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="root_barrel",
    )
    arm_2.visual(
        Box((0.060, 0.030, 0.026)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material=satin_black,
        name="elbow_neck",
    )
    arm_2.visual(
        Cylinder(radius=0.011, length=0.270),
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=matte_black,
        name="arm_2_tube",
    )
    arm_2.visual(
        Box((0.040, 0.090, 0.026)),
        origin=Origin(xyz=(arm2_len - 0.033, 0.0, 0.0)),
        material=satin_black,
        name="shade_bridge",
    )
    arm_2.visual(
        Cylinder(radius=ear_radius_arm2, length=0.016),
        origin=Origin(
            xyz=(arm2_len, 0.032, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="shade_left_ear",
    )
    arm_2.visual(
        Cylinder(radius=ear_radius_arm2, length=0.016),
        origin=Origin(
            xyz=(arm2_len, -0.032, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="shade_right_ear",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="shade_barrel",
    )
    shade.visual(
        Box((0.034, 0.024, 0.020)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=satin_black,
        name="shade_knuckle",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.038),
        origin=Origin(xyz=(0.048, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="socket_housing",
    )
    shade.visual(
        shade_shell,
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=warm_white,
        name="shade_shell",
    )

    model.articulation(
        "post_to_arm_1",
        ArticulationType.REVOLUTE,
        parent=base_post,
        child=arm_1,
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=-0.55,
            upper=1.05,
        ),
    )
    model.articulation(
        "arm_1_to_arm_2",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=arm_2,
        origin=Origin(xyz=(arm1_len, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "arm_2_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm_2,
        child=shade,
        origin=Origin(xyz=(arm2_len, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.95,
            upper=0.85,
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

    base_post = object_model.get_part("base_post")
    arm_1 = object_model.get_part("arm_1")
    arm_2 = object_model.get_part("arm_2")
    shade = object_model.get_part("shade")

    shoulder = object_model.get_articulation("post_to_arm_1")
    elbow = object_model.get_articulation("arm_1_to_arm_2")
    shade_tilt = object_model.get_articulation("arm_2_to_shade")

    with ctx.pose({shoulder: 0.0, elbow: 0.0, shade_tilt: 0.0}):
        ctx.expect_contact(
            base_post,
            arm_1,
            elem_a="shoulder_left_ear",
            elem_b="shoulder_barrel",
            name="shoulder barrel contacts left ear",
        )
        ctx.expect_contact(
            base_post,
            arm_1,
            elem_a="shoulder_right_ear",
            elem_b="shoulder_barrel",
            name="shoulder barrel contacts right ear",
        )
        ctx.expect_contact(
            arm_1,
            arm_2,
            elem_a="elbow_left_ear",
            elem_b="root_barrel",
            name="elbow barrel contacts left ear",
        )
        ctx.expect_contact(
            arm_1,
            arm_2,
            elem_a="elbow_right_ear",
            elem_b="root_barrel",
            name="elbow barrel contacts right ear",
        )
        ctx.expect_contact(
            arm_2,
            shade,
            elem_a="shade_left_ear",
            elem_b="shade_barrel",
            name="shade barrel contacts left ear",
        )
        ctx.expect_contact(
            arm_2,
            shade,
            elem_a="shade_right_ear",
            elem_b="shade_barrel",
            name="shade barrel contacts right ear",
        )

    def center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_arm_1_tip = center_z(ctx.part_element_world_aabb(arm_1, elem="elbow_bridge"))
    with ctx.pose({shoulder: 0.75}):
        raised_arm_1_tip = center_z(ctx.part_element_world_aabb(arm_1, elem="elbow_bridge"))
    ctx.check(
        "shoulder joint raises first arm",
        rest_arm_1_tip is not None
        and raised_arm_1_tip is not None
        and raised_arm_1_tip > rest_arm_1_tip + 0.18,
        details=f"rest_z={rest_arm_1_tip}, raised_z={raised_arm_1_tip}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.0}):
        rest_arm_2_tip = center_z(ctx.part_element_world_aabb(arm_2, elem="shade_bridge"))
    with ctx.pose({shoulder: 0.35, elbow: 0.95}):
        raised_arm_2_tip = center_z(ctx.part_element_world_aabb(arm_2, elem="shade_bridge"))
    ctx.check(
        "elbow joint swings second arm upward",
        rest_arm_2_tip is not None
        and raised_arm_2_tip is not None
        and raised_arm_2_tip > rest_arm_2_tip + 0.10,
        details=f"rest_z={rest_arm_2_tip}, raised_z={raised_arm_2_tip}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.45, shade_tilt: 0.0}):
        neutral_shade_z = center_z(ctx.part_world_aabb(shade))
    with ctx.pose({shoulder: 0.35, elbow: 0.45, shade_tilt: 0.55}):
        tilted_shade_z = center_z(ctx.part_world_aabb(shade))
    ctx.check(
        "shade tilt pitches the head downward",
        neutral_shade_z is not None
        and tilted_shade_z is not None
        and tilted_shade_z < neutral_shade_z - 0.02,
        details=f"neutral_z={neutral_shade_z}, tilted_z={tilted_shade_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
