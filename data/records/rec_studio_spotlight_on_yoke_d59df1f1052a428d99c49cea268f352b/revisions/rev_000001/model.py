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
    model = ArticulatedObject(name="studio_spotlight_on_yoke_stand")

    powder_black = model.material("powder_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.15, 0.15, 0.16, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.62, 0.73, 0.80, 0.55))

    support_base = model.part("support_base")
    support_base.visual(
        Box((0.56, 0.44, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=powder_black,
        name="base_plate",
    )
    support_base.visual(
        Box((0.25, 0.21, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=dark_gray,
        name="ballast_block",
    )
    support_base.visual(
        Cylinder(radius=0.055, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=satin_black,
        name="stand_column",
    )
    support_base.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.3325)),
        material=dark_gray,
        name="pan_bearing_housing",
    )
    support_base.inertial = Inertial.from_geometry(
        Box((0.56, 0.44, 0.35)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.075, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_gray,
        name="pan_turret",
    )
    yoke.visual(
        Box((0.09, 0.12, 0.12)),
        origin=Origin(xyz=(-0.08, 0.0, 0.0825)),
        material=dark_gray,
        name="rear_post",
    )
    yoke.visual(
        Box((0.11, 0.29, 0.05)),
        origin=Origin(xyz=(-0.03, 0.0, 0.110)),
        material=powder_black,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.05, 0.03, 0.30)),
        origin=Origin(xyz=(-0.005, 0.16, 0.245)),
        material=powder_black,
        name="left_arm",
    )
    yoke.visual(
        Box((0.05, 0.03, 0.30)),
        origin=Origin(xyz=(-0.005, -0.16, 0.245)),
        material=powder_black,
        name="right_arm",
    )
    yoke.visual(
        Box((0.05, 0.29, 0.04)),
        origin=Origin(xyz=(-0.005, 0.0, 0.395)),
        material=powder_black,
        name="top_bridge",
    )
    yoke.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.16, 0.25),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="left_pivot_collar",
    )
    yoke.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(
            xyz=(0.0, -0.16, 0.25),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_gray,
        name="right_pivot_collar",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.22, 0.36, 0.44)),
        mass=2.6,
        origin=Origin(xyz=(-0.02, 0.0, 0.22)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.095, length=0.20),
        origin=Origin(
            xyz=(0.015, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="lamp_body",
    )
    lamp_head.visual(
        Cylinder(radius=0.080, length=0.050),
        origin=Origin(
            xyz=(-0.110, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_gray,
        name="rear_cap",
    )
    lamp_head.visual(
        Cylinder(radius=0.105, length=0.035),
        origin=Origin(
            xyz=(0.1325, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.088, length=0.004),
        origin=Origin(
            xyz=(0.152, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.13, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion",
    )
    lamp_head.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(
            xyz=(0.0, -0.13, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion",
    )
    lamp_head.visual(
        Box((0.045, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
        material=dark_gray,
        name="left_trunnion_bracket",
    )
    lamp_head.visual(
        Box((0.045, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, -0.10, 0.0)),
        material=dark_gray,
        name="right_trunnion_bracket",
    )
    lamp_head.visual(
        Box((0.065, 0.06, 0.040)),
        origin=Origin(xyz=(-0.118, 0.0, 0.0)),
        material=dark_gray,
        name="rear_driver_box",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.31, 0.30, 0.22)),
        mass=3.8,
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
    )

    model.articulation(
        "pan_joint",
        ArticulationType.REVOLUTE,
        parent=support_base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-1.05,
            upper=1.35,
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
    support_base = object_model.get_part("support_base")
    yoke = object_model.get_part("yoke")
    lamp_head = object_model.get_part("lamp_head")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    def aabb_dims(aabb):
        if aabb is None:
            return None
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    ctx.expect_gap(
        lamp_head,
        support_base,
        axis="z",
        min_gap=0.10,
        name="lamp head clears the fixed support at rest",
    )
    ctx.expect_gap(
        yoke,
        lamp_head,
        axis="y",
        positive_elem="left_pivot_collar",
        negative_elem="left_trunnion",
        max_gap=0.001,
        max_penetration=0.0,
        name="left trunnion seats against the left yoke collar",
    )
    ctx.expect_gap(
        lamp_head,
        yoke,
        axis="y",
        positive_elem="right_trunnion",
        negative_elem="right_pivot_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="right trunnion seats against the right yoke collar",
    )

    support_dims = aabb_dims(ctx.part_world_aabb(support_base))
    lamp_dims = aabb_dims(ctx.part_world_aabb(lamp_head))
    ctx.check(
        "fixed support footprint is broader than the rotating lamp body",
        support_dims is not None
        and lamp_dims is not None
        and support_dims[0] > lamp_dims[0] + 0.18
        and support_dims[1] > lamp_dims[1] + 0.14,
        details=f"support_dims={support_dims}, lamp_dims={lamp_dims}",
    )

    rest_front = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_bezel"))
    with ctx.pose({tilt_joint: 0.9}):
        tilted_front = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_bezel"))
    ctx.check(
        "positive tilt raises the front of the lamp",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.07,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    rest_front = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_bezel"))
    with ctx.pose({pan_joint: 0.8}):
        panned_front = aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_bezel"))
    ctx.check(
        "pan joint swings the lamp around the vertical axis",
        rest_front is not None
        and panned_front is not None
        and abs(panned_front[1] - rest_front[1]) > 0.09,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
