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
    model = ArticulatedObject(name="searchlight_tower")

    support_dark = model.material("support_dark", rgba=(0.22, 0.24, 0.26, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    lamp_gray = model.material("lamp_gray", rgba=(0.68, 0.70, 0.73, 1.0))
    bezel_dark = model.material("bezel_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    lens_blue = model.material("lens_blue", rgba=(0.67, 0.79, 0.89, 0.82))

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.34, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=support_dark,
        name="base_foot",
    )
    support.visual(
        Cylinder(radius=0.08, length=1.48),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=machinery_gray,
        name="mast",
    )
    support.visual(
        Cylinder(radius=0.12, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.51)),
        material=support_dark,
        name="top_collar",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.68, 0.68, 1.56)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.22, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=support_dark,
        name="turntable",
    )
    pan_head.visual(
        Cylinder(radius=0.11, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=machinery_gray,
        name="pedestal_core",
    )
    pan_head.visual(
        Box((0.18, 0.58, 0.08)),
        origin=Origin(xyz=(-0.03, 0.0, 0.34)),
        material=machinery_gray,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.10, 0.05, 0.52)),
        origin=Origin(xyz=(-0.03, 0.315, 0.64)),
        material=machinery_gray,
        name="left_yoke_arm",
    )
    pan_head.visual(
        Box((0.10, 0.05, 0.52)),
        origin=Origin(xyz=(-0.03, -0.315, 0.64)),
        material=machinery_gray,
        name="right_yoke_arm",
    )
    pan_head.visual(
        Cylinder(radius=0.05, length=0.03),
        origin=Origin(xyz=(0.0, 0.275, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_dark,
        name="left_tilt_boss",
    )
    pan_head.visual(
        Cylinder(radius=0.05, length=0.03),
        origin=Origin(xyz=(0.0, -0.275, 0.64), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_dark,
        name="right_tilt_boss",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.44, 0.68, 0.92)),
        mass=140.0,
        origin=Origin(xyz=(-0.01, 0.0, 0.46)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.22, length=0.40),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lamp_gray,
        name="main_body",
    )
    lamp.visual(
        Cylinder(radius=0.27, length=0.06),
        origin=Origin(xyz=(0.35, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bezel_dark,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.21, length=0.012),
        origin=Origin(xyz=(0.374, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_blue,
        name="lens",
    )
    lamp.visual(
        Cylinder(radius=0.15, length=0.18),
        origin=Origin(xyz=(-0.17, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="rear_housing",
    )
    lamp.visual(
        Box((0.12, 0.20, 0.18)),
        origin=Origin(xyz=(-0.29, 0.0, 0.0)),
        material=support_dark,
        name="rear_ballast_box",
    )
    lamp.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.0, 0.22, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_dark,
        name="left_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(0.0, -0.22, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=support_dark,
        name="right_trunnion",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.66, 0.58, 0.58)),
        mass=110.0,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_pan",
        ArticulationType.REVOLUTE,
        parent=support,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=650.0,
            velocity=0.9,
            lower=-1.8,
            upper=1.8,
        ),
    )
    model.articulation(
        "pan_to_lamp",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.64)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=420.0,
            velocity=1.0,
            lower=-0.45,
            upper=1.10,
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
    support = object_model.get_part("support")
    pan_head = object_model.get_part("pan_head")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("support_to_pan")
    tilt = object_model.get_articulation("pan_to_lamp")

    def _center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    def _span(aabb, axis_index):
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.expect_contact(
        pan_head,
        support,
        elem_a="turntable",
        elem_b="top_collar",
        contact_tol=1e-5,
        name="turntable sits on mast collar",
    )
    ctx.expect_contact(
        lamp,
        pan_head,
        elem_a="left_trunnion",
        elem_b="left_tilt_boss",
        contact_tol=1e-5,
        name="left trunnion is carried by left yoke boss",
    )
    ctx.expect_contact(
        lamp,
        pan_head,
        elem_a="right_trunnion",
        elem_b="right_tilt_boss",
        contact_tol=1e-5,
        name="right trunnion is carried by right yoke boss",
    )

    mast_aabb = ctx.part_element_world_aabb(support, elem="mast")
    body_aabb = ctx.part_element_world_aabb(lamp, elem="main_body")
    turntable_aabb = ctx.part_element_world_aabb(pan_head, elem="turntable")
    rotating_member_large = (
        mast_aabb is not None
        and body_aabb is not None
        and turntable_aabb is not None
        and _span(body_aabb, 1) > 2.5 * _span(mast_aabb, 1)
        and _span(turntable_aabb, 0) > 2.5 * _span(mast_aabb, 0)
    )
    ctx.check(
        "rotating member is large relative to the fixed mast",
        rotating_member_large,
        details=(
            f"mast={mast_aabb}, main_body={body_aabb}, "
            f"turntable={turntable_aabb}"
        ),
    )

    front_rest = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        front_tilted = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    tilt_lifts_front = (
        front_rest is not None
        and front_tilted is not None
        and _center(front_tilted)[2] > _center(front_rest)[2] + 0.16
    )
    ctx.check(
        "positive tilt raises the lamp front",
        tilt_lifts_front,
        details=f"rest={front_rest}, tilted={front_tilted}",
    )

    with ctx.pose({pan: 0.0}):
        front_pan_rest = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    with ctx.pose({pan: 1.0}):
        front_pan_swept = ctx.part_element_world_aabb(lamp, elem="front_bezel")
    pan_swings_head = (
        front_pan_rest is not None
        and front_pan_swept is not None
        and abs(_center(front_pan_swept)[1] - _center(front_pan_rest)[1]) > 0.22
    )
    ctx.check(
        "pan joint swings the lamp head around the mast",
        pan_swings_head,
        details=f"rest={front_pan_rest}, swept={front_pan_swept}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
