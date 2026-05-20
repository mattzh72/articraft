from __future__ import annotations

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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


BLACK = "satin_black"
RUBBER = "soft_black_rubber"
GUNMETAL = "dark_gunmetal"
ACCENT = "brushed_steel"


def _origin_x_axis(xyz=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _origin_y_axis(xyz=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_headphones")
    model.material(BLACK, rgba=(0.015, 0.015, 0.018, 1.0))
    model.material(RUBBER, rgba=(0.002, 0.002, 0.002, 1.0))
    model.material(GUNMETAL, rgba=(0.12, 0.13, 0.14, 1.0))
    model.material(ACCENT, rgba=(0.62, 0.62, 0.58, 1.0))

    headband = model.part("headband")
    hinge_half_width = 0.230
    arch_points = [
        (-hinge_half_width, 0.0, 0.165),
        (-0.185, 0.0, 0.235),
        (-0.095, 0.0, 0.305),
        (0.000, 0.0, 0.335),
        (0.095, 0.0, 0.305),
        (0.185, 0.0, 0.235),
        (hinge_half_width, 0.0, 0.165),
    ]
    band = sweep_profile_along_spline(
        arch_points,
        profile=rounded_rect_profile(0.050, 0.018, radius=0.006, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    headband.visual(mesh_from_geometry(band, "arched_headband"), material=BLACK, name="arched_band")

    # Two-piece hinge sockets at each end: the central child barrel nests between
    # the front and rear lugs while the upper bridge ties the socket into the band.
    fold_hinge_z = 0.085
    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        x = sx * hinge_half_width
        headband.visual(
            Box((0.046, 0.072, 0.058)),
            origin=Origin(xyz=(x, 0.0, 0.155)),
            material=BLACK,
            name=f"{side_name}_band_socket",
        )
        headband.visual(
            Box((0.036, 0.118, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.124)),
            material=GUNMETAL,
            name=f"{side_name}_hinge_bridge",
        )
        for y, lug_name in ((-0.043, "rear_lug"), (0.043, "front_lug")):
            headband.visual(
                Box((0.038, 0.024, 0.060)),
                origin=Origin(xyz=(x, y, fold_hinge_z)),
                material=GUNMETAL,
                name=f"{side_name}_{lug_name}",
            )
        headband.visual(
            Cylinder(radius=0.006, length=0.118),
            origin=_origin_y_axis((x, 0.0, fold_hinge_z)),
            material=ACCENT,
            name=f"{side_name}_fold_pin",
        )

    def build_side(side_name: str, sx: float) -> None:
        hinge_x = sx * hinge_half_width
        hinge_z = fold_hinge_z
        pivot_z = -0.150
        yoke = model.part(f"{side_name}_yoke")

        yoke.visual(
            Cylinder(radius=0.017, length=0.058),
            origin=_origin_y_axis(),
            material=GUNMETAL,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.030, 0.040, 0.060)),
            origin=Origin(xyz=(0.0, 0.0, -0.036)),
            material=GUNMETAL,
            name="drop_neck",
        )
        yoke.visual(
            Box((0.032, 0.190, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.070)),
            material=GUNMETAL,
            name="fork_bridge",
        )
        for y, boss_name in ((-0.087, "rear_boss"), (0.087, "front_boss")):
            yoke.visual(
                Box((0.026, 0.020, 0.060)),
                origin=Origin(xyz=(0.0, y, -0.102)),
                material=GUNMETAL,
                name=f"{boss_name}_upper_arm",
            )
            yoke.visual(
                Box((0.026, 0.020, 0.037)),
                origin=Origin(xyz=(0.0, y, -0.187)),
                material=GUNMETAL,
                name=f"{boss_name}_lower_arm",
            )
            yoke.visual(
                Cylinder(radius=0.024, length=0.014),
                origin=_origin_y_axis((0.0, y, pivot_z)),
                material=GUNMETAL,
                name=boss_name,
            )

        cup = model.part(f"{side_name}_earcup")
        inner_sign = -sx
        outer_sign = sx
        cup.visual(
            Cylinder(radius=0.071, length=0.058),
            origin=_origin_x_axis((outer_sign * 0.006, 0.0, 0.0)),
            material=BLACK,
            name="cup_shell",
        )
        cup.visual(
            Cylinder(radius=0.064, length=0.018),
            origin=_origin_x_axis((outer_sign * 0.038, 0.0, 0.0)),
            material=GUNMETAL,
            name="outer_cap",
        )
        cushion = TorusGeometry(radius=0.050, tube=0.014, radial_segments=28, tubular_segments=48)
        cup.visual(
            mesh_from_geometry(cushion, f"{side_name}_cushion_ring"),
            origin=_origin_x_axis((inner_sign * 0.037, 0.0, 0.0)),
            material=RUBBER,
            name="cushion_ring",
        )
        cup.visual(
            Cylinder(radius=0.037, length=0.006),
            origin=_origin_x_axis((inner_sign * 0.043, 0.0, 0.0)),
            material=GUNMETAL,
            name="speaker_grille",
        )
        cup.visual(
            Cylinder(radius=0.007, length=0.184),
            origin=_origin_y_axis(),
            material=ACCENT,
            name="pivot_pin",
        )

        fold_axis = (0.0, sx, 0.0)
        model.articulation(
            f"{side_name}_fold",
            ArticulationType.REVOLUTE,
            parent=headband,
            child=yoke,
            origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
            axis=fold_axis,
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=2.0,
                lower=0.0,
                upper=math.radians(100.0),
            ),
        )
        model.articulation(
            f"{side_name}_cup_pivot",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=cup,
            origin=Origin(xyz=(0.0, 0.0, pivot_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=3.0,
                lower=-math.pi / 2.0,
                upper=math.pi / 2.0,
            ),
        )

    build_side("left", -1.0)
    build_side("right", 1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_cup = object_model.get_part("left_earcup")
    right_cup = object_model.get_part("right_earcup")
    left_fold = object_model.get_articulation("left_fold")
    right_fold = object_model.get_articulation("right_fold")
    left_pivot = object_model.get_articulation("left_cup_pivot")
    right_pivot = object_model.get_articulation("right_cup_pivot")

    for yoke, cup, side_name in (
        (left_yoke, left_cup, "left"),
        (right_yoke, right_cup, "right"),
    ):
        ctx.allow_overlap(
            headband,
            yoke,
            elem_a=f"{side_name}_fold_pin",
            elem_b="fold_barrel",
            reason="The visible hinge pin intentionally passes through the fold barrel.",
        )
        ctx.expect_overlap(
            headband,
            yoke,
            axes="xz",
            elem_a=f"{side_name}_fold_pin",
            elem_b="fold_barrel",
            min_overlap=0.010,
            name=f"{side_name} fold pin crosses barrel",
        )
        ctx.expect_overlap(
            headband,
            yoke,
            axes="y",
            elem_a=f"{side_name}_fold_pin",
            elem_b="fold_barrel",
            min_overlap=0.050,
            name=f"{side_name} fold pin spans barrel",
        )
        for boss in ("rear_boss", "front_boss"):
            ctx.allow_overlap(
                yoke,
                cup,
                elem_a=boss,
                elem_b="pivot_pin",
                reason="The earcup trunnion pin is intentionally captured inside the yoke boss bore.",
            )
            ctx.expect_overlap(
                cup,
                yoke,
                axes="xz",
                elem_a="pivot_pin",
                elem_b=boss,
                min_overlap=0.010,
                name=f"{side_name} pin passes through {boss}",
            )
            ctx.expect_overlap(
                cup,
                yoke,
                axes="y",
                elem_a="pivot_pin",
                elem_b=boss,
                min_overlap=0.010,
                name=f"{side_name} pin retained by {boss}",
            )

    ctx.check(
        "earcups have plus minus ninety degree pivots",
        all(
            abs(j.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(j.motion_limits.upper - math.pi / 2.0) < 1e-6
            for j in (left_pivot, right_pivot)
        ),
    )
    ctx.check(
        "fold hinges travel about one hundred degrees",
        all(abs(j.motion_limits.upper - math.radians(100.0)) < 1e-6 for j in (left_fold, right_fold)),
    )

    left_rest = ctx.part_world_position(left_cup)
    right_rest = ctx.part_world_position(right_cup)
    with ctx.pose({left_fold: math.radians(100.0), right_fold: math.radians(100.0)}):
        ctx.expect_gap(
            right_cup,
            left_cup,
            axis="x",
            min_gap=0.004,
            name="folded earcups clear each other",
        )
        ctx.expect_gap(
            right_yoke,
            left_yoke,
            axis="x",
            min_gap=0.010,
            name="folded yokes clear each other",
        )
        left_folded = ctx.part_world_position(left_cup)
        right_folded = ctx.part_world_position(right_cup)
    ctx.check(
        "side assemblies fold inward",
        left_rest is not None
        and right_rest is not None
        and left_folded is not None
        and right_folded is not None
        and left_folded[0] > left_rest[0] + 0.10
        and right_folded[0] < right_rest[0] - 0.10,
        details=f"left_rest={left_rest}, left_folded={left_folded}, right_rest={right_rest}, right_folded={right_folded}",
    )

    return ctx.report()


object_model = build_object_model()
