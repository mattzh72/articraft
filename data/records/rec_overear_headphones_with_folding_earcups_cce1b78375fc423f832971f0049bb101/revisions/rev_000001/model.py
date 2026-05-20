from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_folding_headphones")

    model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("soft_black", rgba=(0.03, 0.028, 0.026, 1.0))
    model.material("dark_graphite", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("gunmetal", rgba=(0.34, 0.35, 0.36, 1.0))
    model.material("brushed_steel", rgba=(0.68, 0.68, 0.64, 1.0))
    model.material("speaker_fabric", rgba=(0.006, 0.006, 0.007, 1.0))

    headband = model.part("headband")
    headband.visual(
        mesh_from_geometry(_headband_band_geometry(), "arched_headband"),
        material="matte_black",
        name="outer_band",
    )
    headband.visual(
        mesh_from_geometry(_headband_pad_geometry(), "headband_pad"),
        material="soft_black",
        name="cushion_pad",
    )
    for side in (-1.0, 1.0):
        y = side * 0.145
        headband.visual(
            Cylinder(radius=0.019, length=0.018),
            origin=Origin(xyz=(-0.030, y, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
            material="gunmetal",
            name=f"hinge_barrel_{int(side)}_a",
        )
        headband.visual(
            Cylinder(radius=0.019, length=0.018),
            origin=Origin(xyz=(0.030, y, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
            material="gunmetal",
            name=f"hinge_barrel_{int(side)}_b",
        )
        headband.visual(
            Box((0.018, 0.018, 0.052)),
            origin=Origin(xyz=(-0.030, y, 0.252)),
            material="matte_black",
            name=f"hinge_strap_{int(side)}_a",
        )
        headband.visual(
            Box((0.018, 0.018, 0.052)),
            origin=Origin(xyz=(0.030, y, 0.252)),
            material="matte_black",
            name=f"hinge_strap_{int(side)}_b",
        )
        headband.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(-0.042, y, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_steel",
            name=f"hinge_pin_{int(side)}_a",
        )
        headband.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.042, y, 0.235), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_steel",
            name=f"hinge_pin_{int(side)}_b",
        )

    left_yoke = _make_yoke(model, "left_yoke")
    right_yoke = _make_yoke(model, "right_yoke")
    left_earcup = _make_earcup(model, "left_earcup", side=1.0)
    right_earcup = _make_earcup(model, "right_earcup", side=-1.0)

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(0.0, 0.145, 0.235)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.0, -0.145, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "left_cup_pivot",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.60, upper=0.60),
    )
    model.articulation(
        "right_cup_pivot",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=(0.0, 0.0, -0.165)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.60, upper=0.60),
    )

    return model


def _headband_band_geometry():
    path = [
        (0.0, 0.145, 0.260),
        (0.0, 0.120, 0.325),
        (0.0, 0.055, 0.382),
        (0.0, 0.0, 0.398),
        (0.0, -0.055, 0.382),
        (0.0, -0.120, 0.325),
        (0.0, -0.145, 0.260),
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.060, 0.018, radius=0.006, corner_segments=8),
        samples_per_segment=10,
        spline="catmull_rom",
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _headband_pad_geometry():
    path = [
        (0.0, 0.080, 0.342),
        (0.0, 0.040, 0.372),
        (0.0, 0.0, 0.383),
        (0.0, -0.040, 0.372),
        (0.0, -0.080, 0.342),
    ]
    return sweep_profile_along_spline(
        path,
        profile=rounded_rect_profile(0.048, 0.012, radius=0.005, corner_segments=8),
        samples_per_segment=10,
        spline="catmull_rom",
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _make_yoke(model: ArticulatedObject, name: str):
    yoke = model.part(name)

    yoke.visual(
        Cylinder(radius=0.015, length=0.024),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="gunmetal",
        name="fold_knuckle",
    )
    yoke.visual(
        Box((0.044, 0.026, 0.066)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material="matte_black",
        name="hinge_neck",
    )
    yoke.visual(
        Box((0.172, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material="matte_black",
        name="top_bridge",
    )
    for x, arm_name, boss_name, screw_name in (
        (-0.076, "fork_arm_neg", "pivot_boss_neg", "pivot_screw_neg"),
        (0.076, "fork_arm_pos", "pivot_boss_pos", "pivot_screw_pos"),
    ):
        yoke.visual(
            Box((0.024, 0.026, 0.150)),
            origin=Origin(xyz=(x, 0.0, -0.133)),
            material="matte_black",
            name=arm_name,
        )
        yoke.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=Origin(xyz=(x, 0.0, -0.165), rpy=(0.0, pi / 2.0, 0.0)),
            material="gunmetal",
            name=boss_name,
        )
        yoke.visual(
            Cylinder(radius=0.009, length=0.017),
            origin=Origin(xyz=(x, 0.0, -0.165), rpy=(0.0, pi / 2.0, 0.0)),
            material="brushed_steel",
            name=screw_name,
        )

    return yoke


def _make_earcup(model: ArticulatedObject, name: str, *, side: float):
    earcup = model.part(name)
    inward = -side
    outward = side

    earcup.visual(
        mesh_from_geometry(_oval_solid(0.112, 0.138, 0.052), f"{name}_cup_shell"),
        origin=Origin(xyz=(0.0, outward * 0.006, 0.0)),
        material="dark_graphite",
        name="cup_shell",
    )
    earcup.visual(
        mesh_from_geometry(_oval_ring(0.116, 0.142, 0.070, 0.092, 0.028), f"{name}_ear_pad"),
        origin=Origin(xyz=(0.0, inward * 0.029, 0.0)),
        material="soft_black",
        name="ear_pad",
    )
    earcup.visual(
        mesh_from_geometry(_oval_solid(0.064, 0.086, 0.006), f"{name}_speaker_cloth"),
        origin=Origin(xyz=(0.0, inward * 0.045, 0.0)),
        material="speaker_fabric",
        name="speaker_cloth",
    )
    earcup.visual(
        mesh_from_geometry(_oval_ring(0.104, 0.126, 0.080, 0.100, 0.006), f"{name}_outer_trim"),
        origin=Origin(xyz=(0.0, outward * 0.034, 0.0)),
        material="gunmetal",
        name="outer_trim",
    )
    earcup.visual(
        mesh_from_geometry(_oval_solid(0.070, 0.086, 0.008), f"{name}_outer_badge"),
        origin=Origin(xyz=(0.0, outward * 0.038, 0.0)),
        material="matte_black",
        name="outer_badge",
    )
    for x, trunnion_name in ((-0.057, "trunnion_neg"), (0.057, "trunnion_pos")):
        earcup.visual(
            Cylinder(radius=0.017, length=0.024),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="gunmetal",
            name=trunnion_name,
        )

    if side > 0.0:
        earcup.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(0.0, 0.020, -0.073), rpy=(pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name="cable_strain_relief",
        )

    return earcup


def _oval_solid(width: float, height: float, depth: float):
    geom = ExtrudeGeometry(
        superellipse_profile(width, height, exponent=2.7, segments=64),
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    return geom.rotate_x(-pi / 2.0)


def _oval_ring(width: float, height: float, hole_width: float, hole_height: float, depth: float):
    geom = ExtrudeWithHolesGeometry(
        superellipse_profile(width, height, exponent=2.7, segments=72),
        [superellipse_profile(hole_width, hole_height, exponent=2.4, segments=72)],
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    return geom.rotate_x(-pi / 2.0)


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_earcup = object_model.get_part("left_earcup")
    right_earcup = object_model.get_part("right_earcup")
    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_cup_pivot = object_model.get_articulation("left_cup_pivot")

    for side_name, yoke, cup in (
        ("left", left_yoke, left_earcup),
        ("right", right_yoke, right_earcup),
    ):
        ctx.allow_overlap(
            cup,
            yoke,
            elem_a="trunnion_neg",
            elem_b="fork_arm_neg",
            reason="The earcup trunnion is intentionally captured a few millimeters inside the fork arm pivot socket.",
        )
        ctx.allow_overlap(
            cup,
            yoke,
            elem_a="trunnion_pos",
            elem_b="fork_arm_pos",
            reason="The opposite earcup trunnion is intentionally captured a few millimeters inside the fork arm pivot socket.",
        )
        ctx.expect_gap(
            cup,
            yoke,
            axis="x",
            positive_elem="trunnion_neg",
            negative_elem="fork_arm_neg",
            max_penetration=0.006,
            name=f"{side_name} negative trunnion has shallow captured fit",
        )
        ctx.expect_gap(
            yoke,
            cup,
            axis="x",
            positive_elem="fork_arm_pos",
            negative_elem="trunnion_pos",
            max_penetration=0.006,
            name=f"{side_name} positive trunnion has shallow captured fit",
        )
        ctx.expect_origin_gap(
            yoke,
            cup,
            axis="z",
            min_gap=0.140,
            max_gap=0.180,
            name=f"{side_name} cup hangs below the fold hinge",
        )

    left_rest = ctx.part_world_position(left_earcup)
    right_rest = ctx.part_world_position(right_earcup)
    with ctx.pose({left_fold: 1.20, right_fold: 1.20}):
        left_folded = ctx.part_world_position(left_earcup)
        right_folded = ctx.part_world_position(right_earcup)
    ctx.check(
        "fold hinges swing cups inward and upward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[1] < left_rest[1] - 0.05
        and right_folded[1] > right_rest[1] + 0.05
        and left_folded[2] > left_rest[2] + 0.05
        and right_folded[2] > right_rest[2] + 0.05,
        details=f"left_rest={left_rest}, left_folded={left_folded}, right_rest={right_rest}, right_folded={right_folded}",
    )

    with ctx.pose({left_cup_pivot: 0.55}):
        tilted_aabb = ctx.part_world_aabb(left_earcup)
    rest_aabb = ctx.part_world_aabb(left_earcup)
    ctx.check(
        "earcup pivots on the yoke axis",
        rest_aabb is not None
        and tilted_aabb is not None
        and abs((tilted_aabb[1][1] - tilted_aabb[0][1]) - (rest_aabb[1][1] - rest_aabb[0][1])) > 0.010,
        details=f"rest_aabb={rest_aabb}, tilted_aabb={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
