from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


def _lens_mesh(name: str, *, width: float, height: float, thickness: float, exponent: float = 2.45):
    """Thin superellipse lens in the local X-Z plane, with thickness along Y."""
    profile = superellipse_profile(width, height, exponent=exponent, segments=72)
    geom = ExtrudeGeometry(profile, thickness, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _tube_mesh(name: str, points, *, radius: float, samples: int = 10, radial_segments: int = 16):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="half_rim_reading_glasses_flip_clip")

    metal = model.material("dark_burnished_metal", rgba=(0.05, 0.047, 0.043, 1.0))
    clear = model.material("clear_prescription_lens", rgba=(0.72, 0.90, 1.0, 0.30))
    tint = model.material("smoky_sun_lens", rgba=(0.18, 0.13, 0.055, 0.58))
    silicone = model.material("soft_clear_nose_pad", rgba=(0.85, 0.88, 0.82, 0.55))

    brow = model.part("brow_frame")

    # Half-rim upper brow wire: one continuous arched tube over both clear lenses.
    brow.visual(
        _tube_mesh(
            "upper_brow_bar",
            [
                (-0.070, 0.000, 0.021),
                (-0.058, 0.000, 0.028),
                (-0.034, 0.000, 0.032),
                (-0.010, 0.000, 0.028),
                (0.000, -0.001, 0.024),
                (0.010, 0.000, 0.028),
                (0.034, 0.000, 0.032),
                (0.058, 0.000, 0.028),
                (0.070, 0.000, 0.021),
            ],
            radius=0.00155,
            samples=12,
            radial_segments=18,
        ),
        material=metal,
        name="upper_brow_bar",
    )

    # Fixed clear reading lenses hang from the upper brow bar; the lower half is rimless.
    for idx, cx in enumerate((-0.033, 0.033)):
        brow.visual(
            _lens_mesh(f"clear_lens_{idx}", width=0.050, height=0.036, thickness=0.0016),
            origin=Origin(xyz=(cx, 0.000, 0.011)),
            material=clear,
            name=f"clear_lens_{idx}",
        )
        brow.visual(
            Box((0.043, 0.0022, 0.0022)),
            origin=Origin(xyz=(cx, 0.000, 0.0285)),
            material=metal,
            name=f"lens_channel_{idx}",
        )

    # Nose bridge and pad arms, all tied back into the brow wire.
    brow.visual(
        _tube_mesh(
            "saddle_bridge",
            [
                (-0.016, 0.000, 0.028),
                (-0.007, -0.004, 0.017),
                (0.000, -0.006, 0.014),
                (0.007, -0.004, 0.017),
                (0.016, 0.000, 0.028),
            ],
            radius=0.00125,
            samples=10,
            radial_segments=16,
        ),
        material=metal,
        name="saddle_bridge",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        brow.visual(
            _tube_mesh(
                f"nose_arm_{idx}",
                [
                    (0.0070 * sx, -0.004, 0.017),
                    (0.0090 * sx, -0.010, 0.007),
                    (0.0120 * sx, -0.013, -0.001),
                ],
                radius=0.00075,
                samples=8,
                radial_segments=12,
            ),
            material=metal,
            name=f"nose_arm_{idx}",
        )
        brow.visual(
            _lens_mesh(f"nose_pad_{idx}", width=0.0075, height=0.014, thickness=0.0022, exponent=2.0),
            origin=Origin(xyz=(0.012 * sx, -0.013, -0.001), rpy=(0.0, 0.0, -0.22 * sx)),
            material=silicone,
            name=f"nose_pad_{idx}",
        )

    # Outer temple hinge blocks, fixed to the brow ends.
    for idx, sx in enumerate((-1.0, 1.0)):
        brow.visual(
            Box((0.005, 0.006, 0.016)),
            origin=Origin(xyz=(0.069 * sx, -0.0005, 0.020)),
            material=metal,
            name=f"temple_endpiece_{idx}",
        )
        brow.visual(
            Cylinder(radius=0.00155, length=0.013),
            origin=Origin(xyz=(0.069 * sx, 0.0025, 0.020)),
            material=metal,
            name=f"temple_hinge_barrel_{idx}",
        )

    # Two small hinge saddles that carry the flip-up clip pivot axis above the lenses.
    for idx, cx in enumerate((-0.036, 0.036)):
        brow.visual(
            _tube_mesh(
                f"clip_hinge_stand_{idx}",
                [
                    (cx, 0.001, 0.029),
                    (cx, 0.003, 0.033),
                    (cx, 0.003, 0.036),
                ],
                radius=0.00085,
                samples=6,
                radial_segments=12,
            ),
            material=metal,
            name=f"clip_hinge_stand_{idx}",
        )
        brow.visual(
            Cylinder(radius=0.00125, length=0.005),
            origin=Origin(xyz=(cx, 0.0025, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"clip_hinge_socket_{idx}",
        )

    # Wire temples are separate folding parts.  The local frame is the hinge axis.
    for idx, sx in enumerate((-1.0, 1.0)):
        temple = model.part(f"temple_{idx}")
        temple.visual(
            Cylinder(radius=0.00135, length=0.011),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=metal,
            name="temple_hinge_sleeve",
        )
        temple.visual(
            Box((0.0038, 0.0065, 0.006)),
            origin=Origin(xyz=(0.0, -0.0035, -0.001)),
            material=metal,
            name="temple_hinge_leaf",
        )
        temple.visual(
            _tube_mesh(
                f"wire_temple_{idx}",
                [
                    (0.0, -0.006, 0.000),
                    (-0.003 * sx, -0.030, -0.001),
                    (-0.006 * sx, -0.075, -0.003),
                    (-0.004 * sx, -0.108, -0.010),
                    (-0.001 * sx, -0.128, -0.023),
                    (0.002 * sx, -0.137, -0.033),
                ],
                radius=0.00115,
                samples=10,
                radial_segments=14,
            ),
            material=metal,
            name="wire_temple",
        )

        model.articulation(
            f"brow_to_temple_{idx}",
            ArticulationType.REVOLUTE,
            parent=brow,
            child=temple,
            origin=Origin(xyz=(0.069 * sx, 0.0025, 0.020)),
            axis=(0.0, 0.0, 1.0 if sx < 0.0 else -1.0),
            motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=0.0, upper=1.65),
        )

    # Flip-up sun clip: a single tinted front frame with two visible pivot barrels.
    clip = model.part("sun_clip")
    clip.visual(
        _tube_mesh(
            "clip_top_wire",
            [
                (-0.061, 0.010, -0.014),
                (-0.039, 0.010, -0.010),
                (-0.010, 0.010, -0.013),
                (0.000, 0.010, -0.016),
                (0.010, 0.010, -0.013),
                (0.039, 0.010, -0.010),
                (0.061, 0.010, -0.014),
            ],
            radius=0.00125,
            samples=10,
            radial_segments=16,
        ),
        material=metal,
        name="clip_top_wire",
    )
    for idx, cx in enumerate((-0.033, 0.033)):
        clip.visual(
            _lens_mesh(f"tinted_lens_{idx}", width=0.052, height=0.034, thickness=0.0015),
            origin=Origin(xyz=(cx, 0.011, -0.027)),
            material=tint,
            name=f"tinted_lens_{idx}",
        )
        clip.visual(
            Cylinder(radius=0.00110, length=0.006),
            origin=Origin(xyz=(cx, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"clip_hinge_barrel_{idx}",
        )
        clip.visual(
            _tube_mesh(
                f"clip_hinge_arm_{idx}",
                [
                    (cx, 0.000, 0.000),
                    (cx, 0.006, -0.006),
                    (cx, 0.010, -0.012),
                ],
                radius=0.00075,
                samples=6,
                radial_segments=12,
            ),
            material=metal,
            name=f"clip_hinge_arm_{idx}",
        )
        clip.visual(
            Box((0.044, 0.0020, 0.0018)),
            origin=Origin(xyz=(cx, 0.011, -0.011)),
            material=metal,
            name=f"tint_channel_{idx}",
        )
    clip.visual(
        _tube_mesh(
            "clip_bridge",
            [
                (-0.010, 0.010, -0.016),
                (-0.004, 0.011, -0.023),
                (0.004, 0.011, -0.023),
                (0.010, 0.010, -0.016),
            ],
            radius=0.00085,
            samples=8,
            radial_segments=12,
        ),
        material=metal,
        name="clip_bridge",
    )

    model.articulation(
        "brow_to_sun_clip",
        ArticulationType.REVOLUTE,
        parent=brow,
        child=clip,
        origin=Origin(xyz=(0.0, 0.0025, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    brow = object_model.get_part("brow_frame")
    clip = object_model.get_part("sun_clip")
    temple_0 = object_model.get_part("temple_0")
    temple_1 = object_model.get_part("temple_1")
    clip_joint = object_model.get_articulation("brow_to_sun_clip")
    temple_joint_0 = object_model.get_articulation("brow_to_temple_0")
    temple_joint_1 = object_model.get_articulation("brow_to_temple_1")

    for idx in (0, 1):
        ctx.allow_overlap(
            brow,
            object_model.get_part(f"temple_{idx}"),
            elem_a=f"temple_hinge_barrel_{idx}",
            elem_b="temple_hinge_sleeve",
            reason="The temple hinge sleeve is intentionally modeled as captured on the small vertical hinge pin.",
        )
        ctx.expect_overlap(
            brow,
            object_model.get_part(f"temple_{idx}"),
            axes="z",
            elem_a=f"temple_hinge_barrel_{idx}",
            elem_b="temple_hinge_sleeve",
            min_overlap=0.008,
            name=f"temple {idx} hinge sleeve is retained on its pin",
        )
        ctx.allow_overlap(
            brow,
            clip,
            elem_a=f"clip_hinge_socket_{idx}",
            elem_b=f"clip_hinge_barrel_{idx}",
            reason="The flip-clip hinge barrel is intentionally seated in the brow-mounted pivot socket.",
        )
        ctx.expect_overlap(
            brow,
            clip,
            axes="x",
            elem_a=f"clip_hinge_socket_{idx}",
            elem_b=f"clip_hinge_barrel_{idx}",
            min_overlap=0.002,
            name=f"sun clip hinge {idx} is seated in its pivot socket",
        )

    ctx.check(
        "three user-facing revolute mechanisms",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations),
    )

    ctx.expect_gap(
        clip,
        brow,
        axis="y",
        min_gap=0.004,
        positive_elem="tinted_lens_0",
        negative_elem="clear_lens_0",
        name="sun clip lens sits in front of clear lens",
    )
    ctx.expect_overlap(
        clip,
        brow,
        axes="xz",
        elem_a="tinted_lens_1",
        elem_b="clear_lens_1",
        min_overlap=0.025,
        name="tinted front covers the main reading lens",
    )

    rest_clip = ctx.part_element_world_aabb(clip, elem="tinted_lens_0")
    with ctx.pose({clip_joint: 1.55}):
        raised_clip = ctx.part_element_world_aabb(clip, elem="tinted_lens_0")
    rest_clip_z = None if rest_clip is None else 0.5 * (rest_clip[0][2] + rest_clip[1][2])
    raised_clip_z = None if raised_clip is None else 0.5 * (raised_clip[0][2] + raised_clip[1][2])
    ctx.check(
        "sun clip flips upward",
        rest_clip_z is not None and raised_clip_z is not None and raised_clip_z > rest_clip_z + 0.020,
        details=f"rest_z={rest_clip_z}, raised_z={raised_clip_z}",
    )

    rest_0 = ctx.part_world_aabb(temple_0)
    rest_1 = ctx.part_world_aabb(temple_1)
    with ctx.pose({temple_joint_0: 1.25, temple_joint_1: 1.25}):
        folded_0 = ctx.part_world_aabb(temple_0)
        folded_1 = ctx.part_world_aabb(temple_1)
    ctx.check(
        "temples fold inward",
        rest_0 is not None
        and rest_1 is not None
        and folded_0 is not None
        and folded_1 is not None
        and folded_0[1][0] > rest_0[1][0] + 0.020
        and folded_1[0][0] < rest_1[0][0] - 0.020,
        details=f"rest_0={rest_0}, folded_0={folded_0}, rest_1={rest_1}, folded_1={folded_1}",
    )

    return ctx.report()


object_model = build_object_model()
