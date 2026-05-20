from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
)


def _cushion_ring_mesh(name: str):
    """Soft oval annular pad, authored in the Y/Z face and extruded along X."""
    ring = ExtrudeWithHolesGeometry(
        superellipse_profile(0.112, 0.088, exponent=2.45, segments=72),
        [superellipse_profile(0.072, 0.050, exponent=2.25, segments=72)],
        0.020,
        cap=True,
        center=True,
    )
    # Geometry extrudes along local Z; rotate it so the cushion thickness is X.
    ring.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(ring, name)


def _oval_shell_mesh(name: str):
    shell = ExtrudeGeometry(
        superellipse_profile(0.106, 0.082, exponent=2.65, segments=72),
        0.034,
        cap=True,
        center=True,
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, name)


def _speaker_grille_mesh(name: str):
    grille = ExtrudeGeometry(
        superellipse_profile(0.070, 0.048, exponent=2.4, segments=56),
        0.003,
        cap=True,
        center=True,
    )
    grille.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(grille, name)


def _headband_frame_mesh():
    band_path = [
        (-0.108, 0.0, 0.064),
        (-0.096, 0.0, 0.118),
        (-0.058, 0.0, 0.170),
        (0.0, 0.0, 0.190),
        (0.058, 0.0, 0.170),
        (0.096, 0.0, 0.118),
        (0.108, 0.0, 0.064),
    ]
    frame = sweep_profile_along_spline(
        band_path,
        profile=rounded_rect_profile(0.048, 0.010, 0.003, corner_segments=8),
        samples_per_segment=14,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(frame, "headband_frame")


def _headband_pad_mesh():
    pad_path = [
        (-0.075, 0.0, 0.104),
        (-0.052, 0.0, 0.146),
        (0.0, 0.0, 0.165),
        (0.052, 0.0, 0.146),
        (0.075, 0.0, 0.104),
    ]
    pad = sweep_profile_along_spline(
        pad_path,
        profile=rounded_rect_profile(0.052, 0.018, 0.006, corner_segments=10),
        samples_per_segment=16,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(pad, "headband_pad")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_over_ear_headphones")

    leather = Material("soft_black_leather", rgba=(0.015, 0.014, 0.013, 1.0))
    fabric = Material("charcoal_acoustic_fabric", rgba=(0.03, 0.032, 0.034, 1.0))
    graphite = Material("satin_graphite", rgba=(0.12, 0.125, 0.13, 1.0))
    black_metal = Material("anodized_black_metal", rgba=(0.035, 0.037, 0.04, 1.0))
    aluminum = Material("brushed_aluminum", rgba=(0.62, 0.60, 0.56, 1.0))
    stitch = Material("subtle_stitching", rgba=(0.18, 0.18, 0.17, 1.0))

    headband = model.part("headband")
    headband.visual(_headband_frame_mesh(), material=graphite, name="arched_spring_band")
    headband.visual(_headband_pad_mesh(), material=leather, name="underside_pad")

    # Side hinge saddles: two cheeks around each folding barrel, mounted to a
    # top block so the hardware reads as a supported manufactured assembly.
    for index, x in enumerate((-0.108, 0.108)):
        headband.visual(
            Box((0.026, 0.066, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.061)),
            material=black_metal,
            name=f"hinge_cap_{index}",
        )
        if index == 0:
            headband.visual(
                Box((0.020, 0.006, 0.028)),
                origin=Origin(xyz=(x, -0.030, 0.041)),
                material=black_metal,
                name="hinge_cheek_0_0",
            )
            headband.visual(
                Box((0.020, 0.006, 0.028)),
                origin=Origin(xyz=(x, 0.030, 0.041)),
                material=black_metal,
                name="hinge_cheek_0_1",
            )
        else:
            headband.visual(
                Box((0.020, 0.006, 0.028)),
                origin=Origin(xyz=(x, -0.030, 0.041)),
                material=black_metal,
                name="hinge_cheek_1_0",
            )
            headband.visual(
                Box((0.020, 0.006, 0.028)),
                origin=Origin(xyz=(x, 0.030, 0.041)),
                material=black_metal,
                name="hinge_cheek_1_1",
            )

    # Subtle padded-band transverse seams embedded into the cushion.
    for seam_i, x in enumerate((-0.050, -0.025, 0.0, 0.025, 0.050)):
        z = 0.160 - 0.55 * abs(x)
        headband.visual(
            Box((0.004, 0.058, 0.012)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=stitch,
            name=f"pad_seam_{seam_i}",
        )

    yokes = []
    for index in (0, 1):
        yoke = model.part(f"yoke_{index}")
        yokes.append(yoke)
        yoke.visual(
            Cylinder(radius=0.009, length=0.054),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="fold_barrel",
        )
        yoke.visual(
            Box((0.018, 0.118, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.021)),
            material=black_metal,
            name="upper_bridge",
        )
        yoke.visual(
            Box((0.012, 0.016, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=black_metal,
            name="barrel_web",
        )
        yoke.visual(
            Box((0.010, 0.008, 0.092)),
            origin=Origin(xyz=(0.0, -0.055, -0.058)),
            material=aluminum,
            name="fork_arm_rear",
        )
        yoke.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.0, -0.052, -0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="cup_pivot_disc_rear",
        )
        yoke.visual(
            Box((0.010, 0.008, 0.092)),
            origin=Origin(xyz=(0.0, 0.055, -0.058)),
            material=aluminum,
            name="fork_arm_front",
        )
        yoke.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(0.0, 0.052, -0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="cup_pivot_disc_front",
        )

    # Each cup is mirrored so the cushion faces toward the listener's head and
    # the harder satin shell faces outward.
    for index, inward in enumerate((1.0, -1.0)):
        cup = model.part(f"earcup_{index}")
        cup.visual(
            _oval_shell_mesh(f"earcup_shell_{index}"),
            origin=Origin(xyz=(-inward * 0.007, 0.0, 0.0)),
            material=graphite,
            name="outer_shell",
        )
        cup.visual(
            _cushion_ring_mesh(f"ear_cushion_{index}"),
            origin=Origin(xyz=(inward * 0.019, 0.0, 0.0)),
            material=leather,
            name="ear_cushion",
        )
        cup.visual(
            _speaker_grille_mesh(f"speaker_grille_{index}"),
            origin=Origin(xyz=(inward * 0.029, 0.0, 0.0)),
            material=fabric,
            name="speaker_grille",
        )
        for stripe_i, z in enumerate((-0.018, -0.009, 0.0, 0.009, 0.018)):
            cup.visual(
                Box((0.0025, 0.052, 0.0018)),
                origin=Origin(xyz=(inward * 0.031, 0.0, z)),
                material=stitch,
                name=f"grille_slot_{stripe_i}",
            )
        cup.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.0, -0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_metal,
            name="pivot_boss_rear",
        )
        cup.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.0, 0.046, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_metal,
            name="pivot_boss_front",
        )
        # Small polished brand badge on each outer cap.
        cup.visual(
            Cylinder(radius=0.014, length=0.0025),
            origin=Origin(xyz=(-inward * 0.0235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="outer_badge",
        )

    model.articulation(
        "headband_to_yoke_0",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=yokes[0],
        origin=Origin(xyz=(-0.108, 0.0, 0.041)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "headband_to_yoke_1",
        ArticulationType.REVOLUTE,
        parent=headband,
        child=yokes[1],
        origin=Origin(xyz=(0.108, 0.0, 0.041)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    for index, yoke in enumerate(yokes):
        model.articulation(
            f"yoke_to_earcup_{index}",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=f"earcup_{index}",
            origin=Origin(xyz=(0.0, 0.0, -0.105)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.35, upper=0.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    headband = object_model.get_part("headband")
    yoke_0 = object_model.get_part("yoke_0")
    yoke_1 = object_model.get_part("yoke_1")
    earcup_0 = object_model.get_part("earcup_0")
    earcup_1 = object_model.get_part("earcup_1")
    fold_0 = object_model.get_articulation("headband_to_yoke_0")
    fold_1 = object_model.get_articulation("headband_to_yoke_1")

    ctx.expect_contact(
        headband,
        yoke_0,
        elem_a="hinge_cheek_0_0",
        elem_b="fold_barrel",
        contact_tol=0.0005,
        name="left folding barrel is captured by rear cheek",
    )
    ctx.expect_contact(
        headband,
        yoke_1,
        elem_a="hinge_cheek_1_1",
        elem_b="fold_barrel",
        contact_tol=0.0005,
        name="right folding barrel is captured by front cheek",
    )
    ctx.expect_overlap(
        yoke_0,
        earcup_0,
        axes="z",
        elem_a="cup_pivot_disc_front",
        elem_b="pivot_boss_front",
        min_overlap=0.005,
        name="left cup pivot disc aligns with boss height",
    )
    ctx.expect_overlap(
        yoke_1,
        earcup_1,
        axes="z",
        elem_a="cup_pivot_disc_front",
        elem_b="pivot_boss_front",
        min_overlap=0.005,
        name="right cup pivot disc aligns with boss height",
    )

    rest_0 = ctx.part_world_position(earcup_0)
    rest_1 = ctx.part_world_position(earcup_1)
    with ctx.pose({fold_0: 1.0, fold_1: 1.0}):
        folded_0 = ctx.part_world_position(earcup_0)
        folded_1 = ctx.part_world_position(earcup_1)

    ctx.check(
        "folding hinges swing earcups inward",
        rest_0 is not None
        and rest_1 is not None
        and folded_0 is not None
        and folded_1 is not None
        and folded_0[0] > rest_0[0] + 0.050
        and folded_1[0] < rest_1[0] - 0.050,
        details=f"rest=({rest_0}, {rest_1}), folded=({folded_0}, {folded_1})",
    )

    return ctx.report()


object_model = build_object_model()
