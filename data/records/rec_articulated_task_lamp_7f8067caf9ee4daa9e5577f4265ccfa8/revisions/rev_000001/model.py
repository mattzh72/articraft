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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aquarium_clip_on_led_bar_lamp")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.22, 0.22, 0.24, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.60, 0.63, 0.67, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.94, 0.95, 0.96, 0.95))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.016, 0.055, 0.116)),
        origin=Origin(xyz=(-0.008, 0.0, -0.072)),
        material=matte_black,
        name="rear_column",
    )
    clamp_base.visual(
        Box((0.058, 0.055, 0.012)),
        origin=Origin(xyz=(0.028, 0.0, -0.014)),
        material=matte_black,
        name="top_jaw",
    )
    clamp_base.visual(
        Box((0.010, 0.055, 0.028)),
        origin=Origin(xyz=(0.052, 0.0, -0.028)),
        material=matte_black,
        name="front_hook",
    )
    clamp_base.visual(
        Box((0.044, 0.040, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, -0.108)),
        material=matte_black,
        name="bottom_jaw",
    )
    clamp_base.visual(
        Cylinder(radius=0.005, length=0.094),
        origin=Origin(xyz=(0.028, 0.0, -0.067)),
        material=satin_black,
        name="clamp_screw",
    )
    clamp_base.visual(
        Box((0.022, 0.022, 0.006)),
        origin=Origin(xyz=(0.028, 0.0, -0.105)),
        material=satin_black,
        name="pressure_pad",
    )
    clamp_base.visual(
        Box((0.010, 0.036, 0.008)),
        origin=Origin(xyz=(0.028, 0.0, -0.118)),
        material=satin_black,
        name="knob_crossbar_y",
    )
    clamp_base.visual(
        Box((0.028, 0.008, 0.008)),
        origin=Origin(xyz=(0.028, 0.0, -0.118)),
        material=satin_black,
        name="knob_crossbar_x",
    )
    clamp_base.visual(
        Box((0.018, 0.006, 0.028)),
        origin=Origin(xyz=(0.002, 0.015, -0.002)),
        material=matte_black,
        name="left_shoulder_cheek",
    )
    clamp_base.visual(
        Box((0.018, 0.006, 0.028)),
        origin=Origin(xyz=(0.002, -0.015, -0.002)),
        material=matte_black,
        name="right_shoulder_cheek",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_aluminum,
        name="base_pivot_hub",
    )
    lower_arm.visual(
        Box((0.132, 0.016, 0.010)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=anodized_aluminum,
        name="lower_beam",
    )
    lower_arm.visual(
        Box((0.016, 0.030, 0.010)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material=anodized_aluminum,
        name="elbow_bridge",
    )
    lower_arm.visual(
        Box((0.024, 0.006, 0.020)),
        origin=Origin(xyz=(0.149, 0.015, 0.0)),
        material=anodized_aluminum,
        name="left_elbow_tine",
    )
    lower_arm.visual(
        Box((0.024, 0.006, 0.020)),
        origin=Origin(xyz=(0.149, -0.015, 0.0)),
        material=anodized_aluminum,
        name="right_elbow_tine",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_aluminum,
        name="elbow_pivot_hub",
    )
    upper_arm.visual(
        Box((0.145, 0.014, 0.010)),
        origin=Origin(xyz=(0.0725, 0.0, 0.0)),
        material=anodized_aluminum,
        name="upper_beam",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.014)),
        origin=Origin(xyz=(0.154, 0.0, 0.0)),
        material=anodized_aluminum,
        name="tip_bridge",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.004)),
        origin=Origin(xyz=(0.170, 0.0, 0.008)),
        material=anodized_aluminum,
        name="upper_tip_ear",
    )
    upper_arm.visual(
        Box((0.020, 0.022, 0.004)),
        origin=Origin(xyz=(0.170, 0.0, -0.008)),
        material=anodized_aluminum,
        name="lower_tip_ear",
    )

    led_bar = model.part("led_bar")
    led_bar.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="roll_hub",
    )
    led_bar.visual(
        Box((0.024, 0.058, 0.020)),
        origin=Origin(xyz=(0.010, 0.0, -0.010)),
        material=satin_black,
        name="center_mount",
    )
    led_bar.visual(
        Box((0.024, 0.320, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, -0.016)),
        material=satin_black,
        name="bar_shell",
    )
    led_bar.visual(
        Box((0.021, 0.286, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, -0.021)),
        material=diffuser_white,
        name="led_diffuser",
    )

    model.articulation(
        "clamp_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.35,
            upper=1.30,
        ),
    )
    model.articulation(
        "lower_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.2,
            lower=-0.25,
            upper=1.70,
        ),
    )
    model.articulation(
        "upper_arm_to_led_bar",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=led_bar,
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-1.70,
            upper=1.70,
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

    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    led_bar = object_model.get_part("led_bar")

    shoulder = object_model.get_articulation("clamp_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_upper_arm")
    wrist = object_model.get_articulation("upper_arm_to_led_bar")

    ctx.expect_contact(
        clamp_base,
        lower_arm,
        name="shoulder joint keeps lower arm mounted to clamp",
    )
    ctx.expect_contact(
        lower_arm,
        upper_arm,
        name="elbow joint keeps upper arm mounted to lower arm",
    )
    ctx.expect_contact(
        upper_arm,
        led_bar,
        name="tip joint keeps LED bar mounted to upper arm",
    )

    rest_tip = ctx.part_world_position(led_bar)
    with ctx.pose({shoulder: 0.75}):
        raised_tip = ctx.part_world_position(led_bar)
    ctx.check(
        "shoulder joint raises the lamp head",
        rest_tip is not None
        and raised_tip is not None
        and raised_tip[2] > rest_tip[2] + 0.08,
        details=f"rest_tip={rest_tip}, raised_tip={raised_tip}",
    )

    with ctx.pose({shoulder: 0.35, elbow: 0.0}):
        straight_tip = ctx.part_world_position(led_bar)
    with ctx.pose({shoulder: 0.35, elbow: 1.0}):
        bent_tip = ctx.part_world_position(led_bar)
    ctx.check(
        "elbow joint changes the upper link aim",
        straight_tip is not None
        and bent_tip is not None
        and bent_tip[2] > straight_tip[2] + 0.05,
        details=f"straight_tip={straight_tip}, bent_tip={bent_tip}",
    )

    with ctx.pose({shoulder: 0.45, elbow: 0.60, wrist: 0.0}):
        flat_bar_aabb = ctx.part_element_world_aabb(led_bar, elem="bar_shell")
    with ctx.pose({shoulder: 0.45, elbow: 0.60, wrist: 1.55}):
        rolled_bar_aabb = ctx.part_element_world_aabb(led_bar, elem="bar_shell")

    flat_y_span = None
    flat_z_span = None
    rolled_y_span = None
    rolled_z_span = None
    if flat_bar_aabb is not None:
        flat_y_span = flat_bar_aabb[1][1] - flat_bar_aabb[0][1]
        flat_z_span = flat_bar_aabb[1][2] - flat_bar_aabb[0][2]
    if rolled_bar_aabb is not None:
        rolled_y_span = rolled_bar_aabb[1][1] - rolled_bar_aabb[0][1]
        rolled_z_span = rolled_bar_aabb[1][2] - rolled_bar_aabb[0][2]

    ctx.check(
        "LED bar rolls about the arm terminal axis",
        flat_y_span is not None
        and flat_z_span is not None
        and rolled_y_span is not None
        and rolled_z_span is not None
        and flat_y_span > flat_z_span * 6.0
        and rolled_z_span > rolled_y_span * 2.0,
        details=(
            f"flat_y_span={flat_y_span}, flat_z_span={flat_z_span}, "
            f"rolled_y_span={rolled_y_span}, rolled_z_span={rolled_z_span}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
