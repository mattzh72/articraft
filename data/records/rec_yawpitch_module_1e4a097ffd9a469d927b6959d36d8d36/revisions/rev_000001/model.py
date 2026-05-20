from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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

def _yaw_cheek_shape() -> cq.Workplane:
    profile = [
        (-0.024, 0.000),
        (-0.027, 0.028),
        (-0.025, 0.070),
        (-0.018, 0.098),
        (0.018, 0.098),
        (0.025, 0.070),
        (0.027, 0.028),
        (0.024, 0.000),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.010)


def _yaw_plate_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.048).extrude(0.010)


def _yaw_hub_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.023).extrude(0.030)


def _cradle_side_shape() -> cq.Workplane:
    profile = [
        (-0.018, 0.010),
        (0.004, 0.012),
        (0.028, 0.004),
        (0.040, -0.012),
        (0.044, -0.030),
        (0.040, -0.040),
        (-0.004, -0.040),
        (-0.016, -0.022),
    ]
    return cq.Workplane("XZ").polyline(profile).close().extrude(0.008)


def _trunnion_shape(offset: float) -> cq.Workplane:
    return cq.Workplane("XZ").workplane(offset=offset).circle(0.010).extrude(0.009)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_pan_tilt_head")

    anodized_black = model.material("anodized_black", rgba=(0.13, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.19, 0.20, 1.0))
    hardware_gray = model.material("hardware_gray", rgba=(0.33, 0.34, 0.36, 1.0))

    lower_support = model.part("lower_support")
    lower_support.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").circle(0.060).extrude(0.012),
            "support_base",
        ),
        material=anodized_black,
        name="support_base",
    )
    lower_support.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .workplane(offset=0.012)
            .circle(0.031)
            .extrude(0.040, taper=-6),
            "support_column",
        ),
        material=satin_black,
        name="support_column",
    )
    lower_support.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").workplane(offset=0.052).circle(0.038).extrude(0.023),
            "support_collar",
        ),
        material=hardware_gray,
        name="support_collar",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_plate_shape(), "yaw_plate"),
        material=hardware_gray,
        name="yaw_plate",
    )
    yaw_stage.visual(
        mesh_from_cadquery(_yaw_hub_shape(), "yaw_hub"),
        material=satin_black,
        name="yaw_hub",
    )
    cheek = _yaw_cheek_shape()
    yaw_stage.visual(
        mesh_from_cadquery(cheek.translate((0.0, 0.050, 0.0)), "yaw_cheek_pos"),
        material=anodized_black,
        name="yaw_cheek_pos",
    )
    yaw_stage.visual(
        mesh_from_cadquery(cheek.translate((0.0, -0.040, 0.0)), "yaw_cheek_neg"),
        material=anodized_black,
        name="yaw_cheek_neg",
    )

    pitch_cradle = model.part("pitch_cradle")
    cradle_side = _cradle_side_shape()
    pitch_cradle.visual(
        mesh_from_cadquery(
            cradle_side.translate((0.0, 0.032, 0.0)),
            "cradle_side_pos",
        ),
        material=anodized_black,
        name="cradle_side_pos",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(
            cradle_side.translate((0.0, -0.024, 0.0)),
            "cradle_side_neg",
        ),
        material=anodized_black,
        name="cradle_side_neg",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.056, 0.056, 0.010).translate((0.020, 0.0, -0.035)),
            "mount_platform",
        ),
        material=satin_black,
        name="mount_platform",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.006, 0.056, 0.032).translate((-0.008, 0.0, -0.020)),
            "rear_brace",
        ),
        material=satin_black,
        name="rear_brace",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.006, 0.056, 0.018).translate((0.037, 0.0, -0.025)),
            "front_lip",
        ),
        material=hardware_gray,
        name="front_lip",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_trunnion_shape(-0.040), "trunnion_pos"),
        material=hardware_gray,
        name="trunnion_pos",
    )
    pitch_cradle.visual(
        mesh_from_cadquery(_trunnion_shape(0.031), "trunnion_neg"),
        material=hardware_gray,
        name="trunnion_neg",
    )

    model.articulation(
        "support_to_yaw",
        ArticulationType.REVOLUTE,
        parent=lower_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.95,
            upper=1.15,
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

    lower_support = object_model.get_part("lower_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("support_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    support_collar = lower_support.get_visual("support_collar")
    yaw_plate = yaw_stage.get_visual("yaw_plate")
    yaw_cheek_pos = yaw_stage.get_visual("yaw_cheek_pos")
    yaw_cheek_neg = yaw_stage.get_visual("yaw_cheek_neg")
    mount_platform = pitch_cradle.get_visual("mount_platform")
    trunnion_pos = pitch_cradle.get_visual("trunnion_pos")
    trunnion_neg = pitch_cradle.get_visual("trunnion_neg")

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        ctx.expect_contact(
            yaw_stage,
            lower_support,
            elem_a=yaw_plate,
            elem_b=support_collar,
            name="yaw plate seats on the lower support collar",
        )
        ctx.expect_overlap(
            yaw_stage,
            lower_support,
            axes="xy",
            elem_a=yaw_plate,
            elem_b=support_collar,
            min_overlap=0.070,
            name="yaw bearing surfaces overlap in plan",
        )
        ctx.expect_contact(
            pitch_cradle,
            yaw_stage,
            elem_a=trunnion_pos,
            elem_b=yaw_cheek_pos,
            name="positive trunnion seats on the positive yaw cheek",
        )
        ctx.expect_contact(
            pitch_cradle,
            yaw_stage,
            elem_a=trunnion_neg,
            elem_b=yaw_cheek_neg,
            name="negative trunnion seats on the negative yaw cheek",
        )
        ctx.expect_gap(
            pitch_cradle,
            lower_support,
            axis="z",
            min_gap=0.040,
            name="neutral pitch cradle clears the grounded support",
        )
        neutral_platform_center = _aabb_center(
            ctx.part_element_world_aabb(pitch_cradle, elem=mount_platform)
        )

    with ctx.pose({yaw_joint: 1.0, pitch_joint: 0.0}):
        ctx.expect_contact(
            yaw_stage,
            lower_support,
            elem_a=yaw_plate,
            elem_b=support_collar,
            name="yaw plate stays seated while panned",
        )
        yawed_platform_center = _aabb_center(
            ctx.part_element_world_aabb(pitch_cradle, elem=mount_platform)
        )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.85}):
        ctx.expect_contact(
            pitch_cradle,
            yaw_stage,
            elem_a=trunnion_pos,
            elem_b=yaw_cheek_pos,
            name="positive trunnion stays seated through positive pitch",
        )
        ctx.expect_contact(
            pitch_cradle,
            yaw_stage,
            elem_a=trunnion_neg,
            elem_b=yaw_cheek_neg,
            name="negative trunnion stays seated through positive pitch",
        )
        pitched_platform_center = _aabb_center(
            ctx.part_element_world_aabb(pitch_cradle, elem=mount_platform)
        )

    ctx.check(
        "positive yaw swings the platform toward +Y",
        neutral_platform_center is not None
        and yawed_platform_center is not None
        and yawed_platform_center[1] > neutral_platform_center[1] + 0.012,
        details=(
            f"neutral_platform_center={neutral_platform_center}, "
            f"yawed_platform_center={yawed_platform_center}"
        ),
    )
    ctx.check(
        "positive pitch lifts the platform",
        neutral_platform_center is not None
        and pitched_platform_center is not None
        and pitched_platform_center[2] > neutral_platform_center[2] + 0.018,
        details=(
            f"neutral_platform_center={neutral_platform_center}, "
            f"pitched_platform_center={pitched_platform_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
