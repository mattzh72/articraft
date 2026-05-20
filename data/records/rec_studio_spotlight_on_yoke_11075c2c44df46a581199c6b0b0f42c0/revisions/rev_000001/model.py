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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_on_yoke_stand")

    stand_paint = model.material("stand_paint", rgba=(0.14, 0.14, 0.15, 1.0))
    yoke_paint = model.material("yoke_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    lamp_paint = model.material("lamp_paint", rgba=(0.25, 0.26, 0.28, 1.0))
    bezel_paint = model.material("bezel_paint", rgba=(0.09, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.78, 0.84, 0.88, 0.55))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.23, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=stand_paint,
        name="floor_base",
    )
    stand.visual(
        Cylinder(radius=0.04, length=0.86),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=stand_paint,
        name="center_post",
    )
    stand.visual(
        Cylinder(radius=0.075, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=stand_paint,
        name="head_collar",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 1.00)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.09, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=yoke_paint,
        name="pan_plate",
    )
    yoke.visual(
        Box((0.10, 0.40, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=yoke_paint,
        name="lower_bridge",
    )
    yoke.visual(
        Box((0.055, 0.05, 0.34)),
        origin=Origin(xyz=(0.0, 0.175, 0.27)),
        material=yoke_paint,
        name="left_arm",
    )
    yoke.visual(
        Box((0.055, 0.05, 0.34)),
        origin=Origin(xyz=(0.0, -0.175, 0.27)),
        material=yoke_paint,
        name="right_arm",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.10, 0.40, 0.44)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    lamp = model.part("lamp")
    lamp_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.086, -0.11),
                (0.095, -0.10),
                (0.108, -0.06),
                (0.118, 0.06),
                (0.121, 0.15),
                (0.116, 0.19),
                (0.102, 0.21),
            ],
            inner_profile=[
                (0.040, -0.10),
                (0.058, -0.084),
                (0.095, -0.055),
                (0.104, 0.06),
                (0.106, 0.146),
                (0.100, 0.182),
                (0.086, 0.202),
            ],
            segments=56,
        ),
        "lamp_shell",
    )
    lamp.visual(
        Cylinder(radius=0.014, length=0.30),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bezel_paint,
        name="trunnion_axle",
    )
    lamp.visual(
        lamp_shell,
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_paint,
        name="body_shell",
    )
    lamp.visual(
        Cylinder(radius=0.126, length=0.022),
        origin=Origin(xyz=(0.215, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bezel_paint,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.086, length=0.006),
        origin=Origin(xyz=(0.212, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp.visual(
        Box((0.06, 0.12, 0.11)),
        origin=Origin(xyz=(-0.075, 0.0, 0.0)),
        material=lamp_paint,
        name="rear_housing",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.36, 0.30, 0.25)),
        mass=6.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "stand_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=math.radians(-80.0),
            upper=math.radians(55.0),
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

    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("stand_pan")
    tilt = object_model.get_articulation("yoke_tilt")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.expect_gap(
        yoke,
        stand,
        axis="z",
        positive_elem="pan_plate",
        negative_elem="head_collar",
        max_gap=0.001,
        max_penetration=1e-6,
        name="yoke pan plate sits on the stand collar",
    )
    ctx.expect_overlap(
        yoke,
        stand,
        axes="xy",
        elem_a="pan_plate",
        elem_b="head_collar",
        min_overlap=0.12,
        name="pan plate stays centered over the stand collar",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="trunnion_axle",
        elem_b="left_arm",
        contact_tol=1e-6,
        name="lamp axle bears on the left yoke arm",
    )
    ctx.expect_contact(
        lamp,
        yoke,
        elem_a="trunnion_axle",
        elem_b="right_arm",
        contact_tol=1e-6,
        name="lamp axle bears on the right yoke arm",
    )

    rest_front = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_bezel"))
    with ctx.pose({tilt: math.radians(35.0)}):
        tilted_front = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_bezel"))
    ctx.check(
        "positive tilt raises the lamp nose",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[2] > rest_front[2] + 0.08,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    with ctx.pose({pan: math.radians(45.0)}):
        panned_front = aabb_center(ctx.part_element_world_aabb(lamp, elem="front_bezel"))
    ctx.check(
        "positive pan swings the lamp toward positive y",
        rest_front is not None
        and panned_front is not None
        and panned_front[1] > rest_front[1] + 0.10,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
