from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _axis_x() -> Origin:
    return Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _axis_y() -> Origin:
    return Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_over_ear_headphones")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    soft_foam = model.material("soft_foam", rgba=(0.005, 0.005, 0.006, 1.0))
    graphite = model.material("graphite", rgba=(0.10, 0.105, 0.11, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.035, 0.037, 0.040, 1.0))
    cloth = model.material("black_cloth", rgba=(0.018, 0.017, 0.015, 1.0))

    headband = model.part("headband")
    band_path = [
        (-0.125, 0.0, 0.050),
        (-0.142, 0.0, 0.125),
        (-0.110, 0.0, 0.220),
        (0.000, 0.0, 0.285),
        (0.110, 0.0, 0.220),
        (0.142, 0.0, 0.125),
        (0.125, 0.0, 0.050),
    ]
    headband.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                band_path,
                radius=0.008,
                samples_per_segment=16,
                radial_segments=20,
                cap_ends=True,
            ),
            "outer_band",
        ),
        material=matte_black,
        name="outer_band",
    )
    pad_path = [
        (-0.086, 0.0, 0.215),
        (-0.040, 0.0, 0.248),
        (0.000, 0.0, 0.263),
        (0.040, 0.0, 0.248),
        (0.086, 0.0, 0.215),
    ]
    headband.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                pad_path,
                radius=0.016,
                samples_per_segment=14,
                radial_segments=22,
                cap_ends=True,
            ),
            "band_pad",
        ),
        material=soft_foam,
        name="band_pad",
    )
    for side, sx in (("left", -1.0), ("right", 1.0)):
        headband.visual(
            Cylinder(radius=0.012, length=0.044),
            origin=Origin(xyz=(sx * 0.125, 0.0, 0.050), rpy=_axis_y().rpy),
            material=dark_metal,
            name=f"{side}_fold_barrel",
        )

    def add_yoke(side: str, sx: float):
        yoke = model.part(f"{side}_yoke")
        cup_x = sx * 0.030
        cup_z = -0.070
        arm_y = 0.072

        # Alternating hinge knuckles surround the fixed barrel on the headband.
        for i, y in enumerate((-0.031, 0.031)):
            yoke.visual(
                Cylinder(radius=0.012, length=0.018),
                origin=Origin(xyz=(0.0, y, 0.0), rpy=_axis_y().rpy),
                material=dark_metal,
                name=f"hinge_knuckle_{i}",
            )
            yoke.visual(
                Box((0.014, 0.018, 0.030)),
                origin=Origin(xyz=(0.0, y, 0.011)),
                material=dark_metal,
                name=f"hinge_tab_{i}",
            )

        for i, y in enumerate((-0.052, 0.052)):
            yoke.visual(
                Box((0.014, 0.052, 0.010)),
                origin=Origin(xyz=(0.0, y, 0.020)),
                material=dark_metal,
                name=f"hinge_bridge_{i}",
            )

        yoke.visual(
            Box((0.012, 0.154, 0.010)),
            origin=Origin(xyz=(cup_x, 0.0, 0.020)),
            material=dark_metal,
            name="upper_yoke_bridge",
        )

        for i, y in enumerate((-arm_y, arm_y)):
            yoke.visual(
                Box((abs(cup_x) + 0.018, 0.010, 0.010)),
                origin=Origin(xyz=(cup_x / 2.0, y, 0.020)),
                material=dark_metal,
                name=f"top_arm_{i}",
            )
            yoke.visual(
                Box((0.010, 0.010, 0.078)),
                origin=Origin(xyz=(cup_x, y, -0.019)),
                material=dark_metal,
                name=f"side_arm_{i}",
            )
            yoke.visual(
                Cylinder(radius=0.014, length=0.014),
                origin=Origin(xyz=(cup_x, y, cup_z), rpy=_axis_y().rpy),
                material=dark_metal,
                name=f"pivot_bushing_{i}",
            )

        return yoke, (cup_x, 0.0, cup_z)

    def add_earcup(side: str, sx: float):
        cup = model.part(f"{side}_earcup")
        inward_x = -sx * 0.022
        outward_x = sx * 0.022
        cup.visual(
            Cylinder(radius=0.058, length=0.038),
            origin=_axis_x(),
            material=graphite,
            name="cup_shell",
        )
        cup.visual(
            mesh_from_geometry(
                TorusGeometry(radius=0.042, tube=0.011, radial_segments=30, tubular_segments=48),
                f"{side}_ear_pad",
            ),
            origin=Origin(xyz=(inward_x, 0.0, 0.0), rpy=_axis_x().rpy),
            material=soft_foam,
            name="ear_pad",
        )
        cup.visual(
            Cylinder(radius=0.033, length=0.004),
            origin=Origin(xyz=(inward_x - sx * 0.003, 0.0, 0.0), rpy=_axis_x().rpy),
            material=cloth,
            name="speaker_cloth",
        )
        cup.visual(
            Cylinder(radius=0.045, length=0.006),
            origin=Origin(xyz=(outward_x, 0.0, 0.0), rpy=_axis_x().rpy),
            material=matte_black,
            name="outer_cap",
        )
        cup.visual(
            Cylinder(radius=0.006, length=0.154),
            origin=_axis_y(),
            material=dark_metal,
            name="pivot_pin",
        )
        return cup

    left_yoke, left_cup_origin = add_yoke("left", -1.0)
    right_yoke, right_cup_origin = add_yoke("right", 1.0)
    left_earcup = add_earcup("left", -1.0)
    right_earcup = add_earcup("right", 1.0)

    model.articulation(
        "left_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=left_yoke,
        origin=Origin(xyz=(-0.125, 0.0, 0.050)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "right_fold",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=right_yoke,
        origin=Origin(xyz=(0.125, 0.0, 0.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "left_swivel",
        ArticulationType.REVOLUTE,
        parent=left_yoke,
        child=left_earcup,
        origin=Origin(xyz=left_cup_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=1.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "right_swivel",
        ArticulationType.REVOLUTE,
        parent=right_yoke,
        child=right_earcup,
        origin=Origin(xyz=right_cup_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=1.5, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    for side in ("left", "right"):
        yoke = object_model.get_part(f"{side}_yoke")
        cup = object_model.get_part(f"{side}_earcup")
        for bushing in ("pivot_bushing_0", "pivot_bushing_1"):
            ctx.allow_overlap(
                yoke,
                cup,
                elem_a=bushing,
                elem_b="pivot_pin",
                reason="The cup trunnion pin is intentionally seated inside the yoke bushing.",
            )
            ctx.expect_overlap(
                yoke,
                cup,
                axes="xyz",
                elem_a=bushing,
                elem_b="pivot_pin",
                min_overlap=0.004,
                name=f"{side} {bushing} captures the cup pin",
            )

    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_swivel = object_model.get_articulation("left_swivel")
    right_swivel = object_model.get_articulation("right_swivel")

    ctx.check(
        "fold hinges travel about ninety degrees",
        abs(left_fold.motion_limits.upper - math.pi / 2.0) < 0.01
        and abs(right_fold.motion_limits.upper - math.pi / 2.0) < 0.01,
        details=f"left={left_fold.motion_limits}, right={right_fold.motion_limits}",
    )
    ctx.check(
        "cup swivels are slight",
        left_swivel.motion_limits.lower <= -0.34
        and left_swivel.motion_limits.upper >= 0.34
        and right_swivel.motion_limits.lower <= -0.34
        and right_swivel.motion_limits.upper >= 0.34,
        details=f"left={left_swivel.motion_limits}, right={right_swivel.motion_limits}",
    )

    left_cup = object_model.get_part("left_earcup")
    right_cup = object_model.get_part("right_earcup")
    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: math.pi / 2.0, right_fold: math.pi / 2.0}):
        left_folded = ctx.part_world_position(left_cup)
        right_folded = ctx.part_world_position(right_cup)
    ctx.check(
        "left cup folds inward and upward",
        left_rest is not None
        and left_folded is not None
        and left_folded[0] > left_rest[0] + 0.06
        and left_folded[2] > left_rest[2] + 0.035,
        details=f"rest={left_rest}, folded={left_folded}",
    )
    ctx.check(
        "right cup folds inward and upward",
        right_rest is not None
        and right_folded is not None
        and right_folded[0] < right_rest[0] - 0.06
        and right_folded[2] > right_rest[2] + 0.035,
        details=f"rest={right_rest}, folded={right_folded}",
    )

    return ctx.report()


object_model = build_object_model()
