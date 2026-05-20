from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
        ),
        name,
    )


def _grip_outer_profile(
    *,
    base_radius: float,
    ridge_radius: float,
    length: float,
    ridge_count: int,
) -> list[tuple[float, float]]:
    z_min = -0.5 * length
    pitch = length / ridge_count
    profile: list[tuple[float, float]] = [(base_radius, z_min)]
    for ridge_index in range(ridge_count):
        start = z_min + ridge_index * pitch
        profile.extend(
            [
                (base_radius, start + 0.18 * pitch),
                (ridge_radius, start + 0.48 * pitch),
                (base_radius, start + 0.82 * pitch),
            ]
        )
    profile.append((base_radius, 0.5 * length))
    return profile


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_zoom_lens")

    lens_white = model.material("lens_white", rgba=(0.90, 0.91, 0.88, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.68, 0.69, 0.72, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.10, 0.18, 0.24, 0.82))

    barrel = model.part("barrel")

    barrel.visual(
        _shell_mesh(
            "barrel_shell",
            outer_profile=[
                (0.0430, 0.0000),
                (0.0435, 0.0120),
                (0.0440, 0.0520),
                (0.0505, 0.0600),
                (0.0505, 0.1180),
                (0.0470, 0.1260),
                (0.0470, 0.1820),
                (0.0515, 0.1900),
                (0.0515, 0.2550),
                (0.0515, 0.2740),
                (0.0515, 0.2950),
            ],
            inner_profile=[
                (0.0340, 0.0000),
                (0.0355, 0.0600),
                (0.0375, 0.1400),
                (0.0400, 0.2250),
                (0.0415, 0.2950),
            ],
        ),
        material=lens_white,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.0415, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=mount_metal,
        name="mount_flange",
    )
    barrel.visual(
        Cylinder(radius=0.0440, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_gray,
        name="rear_sleeve",
    )
    barrel.visual(
        Cylinder(radius=0.0475, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark_gray,
        name="collar_band",
    )
    barrel.visual(
        _shell_mesh(
            "hood_shell",
            outer_profile=[
                (0.0550, 0.0000),
                (0.0580, 0.0100),
                (0.0610, 0.0340),
                (0.0630, 0.0500),
            ],
            inner_profile=[
                (0.0505, 0.0000),
                (0.0545, 0.0280),
                (0.0570, 0.0500),
            ],
            segments=80,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
        material=matte_black,
        name="hood",
    )
    barrel.visual(
        Cylinder(radius=0.0505, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.2915)),
        material=glass_dark,
        name="front_element",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _shell_mesh(
            "zoom_ring_shell",
            outer_profile=_grip_outer_profile(
                base_radius=0.0555,
                ridge_radius=0.0569,
                length=0.064,
                ridge_count=12,
            ),
            inner_profile=[
                (0.0515, -0.032),
                (0.0515, 0.032),
            ],
        ),
        material=matte_black,
        name="zoom_shell",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _shell_mesh(
            "focus_ring_shell",
            outer_profile=_grip_outer_profile(
                base_radius=0.0558,
                ridge_radius=0.0575,
                length=0.036,
                ridge_count=8,
            ),
            inner_profile=[
                (0.0515, -0.018),
                (0.0515, 0.018),
            ],
        ),
        material=matte_black,
        name="focus_shell",
    )

    tripod_collar = model.part("tripod_collar")
    tripod_collar.visual(
        _shell_mesh(
            "tripod_collar_ring",
            outer_profile=[
                (0.0572, -0.021),
                (0.0586, -0.016),
                (0.0586, 0.016),
                (0.0572, 0.021),
            ],
            inner_profile=[
                (0.0475, -0.021),
                (0.0475, 0.021),
            ],
        ),
        material=dark_gray,
        name="collar_ring",
    )
    tripod_collar.visual(
        Box((0.016, 0.013, 0.030)),
        origin=Origin(xyz=(0.0, 0.0625, 0.0)),
        material=dark_gray,
        name="clamp_block",
    )
    tripod_collar.visual(
        Cylinder(radius=0.0038, length=0.022),
        origin=Origin(xyz=(0.0, 0.0715, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lock_knob",
    )
    tripod_collar.visual(
        Box((0.018, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, -0.0765, 0.0)),
        material=dark_gray,
        name="foot_strut",
    )
    tripod_collar.visual(
        Box((0.026, 0.010, 0.080)),
        origin=Origin(xyz=(0.0, -0.0990, 0.0)),
        material=dark_gray,
        name="foot",
    )

    model.articulation(
        "zoom_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.211)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.266)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )
    model.articulation(
        "collar_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=tripod_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    tripod_collar = object_model.get_part("tripod_collar")

    zoom_rotation = object_model.get_articulation("zoom_rotation")
    focus_rotation = object_model.get_articulation("focus_rotation")
    collar_rotation = object_model.get_articulation("collar_rotation")

    barrel_aabb = ctx.part_world_aabb(barrel)
    barrel_length = None
    if barrel_aabb is not None:
        barrel_length = barrel_aabb[1][2] - barrel_aabb[0][2]
    ctx.check(
        "barrel reads as telephoto length",
        barrel_length is not None and barrel_length >= 0.33,
        details=f"barrel_length={barrel_length}",
    )

    for name, joint in (
        ("zoom ring uses continuous rotation", zoom_rotation),
        ("focus ring uses continuous rotation", focus_rotation),
        ("tripod collar uses continuous rotation", collar_rotation),
    ):
        limits = joint.motion_limits
        ctx.check(
            name,
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    ctx.allow_overlap(
        barrel,
        zoom_ring,
        elem_a="barrel_shell",
        elem_b="zoom_shell",
        reason="The zoom sleeve is modeled as a coaxial shell proxy nested over the barrel shell.",
    )
    ctx.allow_overlap(
        barrel,
        focus_ring,
        elem_a="barrel_shell",
        elem_b="focus_shell",
        reason="The focus sleeve is modeled as a coaxial shell proxy nested over the front barrel shell.",
    )
    ctx.allow_overlap(
        barrel,
        tripod_collar,
        elem_a="collar_band",
        elem_b="collar_ring",
        reason="The tripod collar rides around the support band as a coaxial rotating sleeve proxy.",
    )

    ctx.expect_overlap(
        zoom_ring,
        barrel,
        axes="xy",
        min_overlap=0.105,
        name="zoom ring wraps the main barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="xy",
        min_overlap=0.105,
        name="focus ring wraps the front barrel",
    )
    ctx.expect_overlap(
        tripod_collar,
        barrel,
        axes="xy",
        min_overlap=0.110,
        name="tripod collar wraps the support band",
    )
    ctx.expect_within(
        barrel,
        zoom_ring,
        axes="xy",
        inner_elem="barrel_shell",
        outer_elem="zoom_shell",
        margin=0.006,
        name="zoom ring stays centered around the barrel shell",
    )
    ctx.expect_within(
        barrel,
        focus_ring,
        axes="xy",
        inner_elem="barrel_shell",
        outer_elem="focus_shell",
        margin=0.006,
        name="focus ring stays centered around the front barrel shell",
    )
    ctx.expect_within(
        barrel,
        tripod_collar,
        axes="xy",
        inner_elem="collar_band",
        outer_elem="collar_ring",
        margin=0.003,
        name="tripod collar stays centered on the support band",
    )
    ctx.expect_origin_gap(
        focus_ring,
        zoom_ring,
        axis="z",
        min_gap=0.055,
        max_gap=0.070,
        name="focus ring sits ahead of the zoom ring",
    )
    ctx.expect_origin_gap(
        zoom_ring,
        tripod_collar,
        axis="z",
        min_gap=0.040,
        max_gap=0.060,
        name="zoom ring sits ahead of the tripod collar",
    )
    ctx.expect_gap(
        focus_ring,
        zoom_ring,
        axis="z",
        min_gap=0.004,
        max_gap=0.012,
        name="focus and zoom rings remain visually separate",
    )

    with ctx.pose({zoom_rotation: 1.1, focus_rotation: -0.9}):
        ctx.expect_overlap(
            zoom_ring,
            barrel,
            axes="xy",
            min_overlap=0.105,
            name="zoom ring remains coaxial when rotated",
        )
        ctx.expect_overlap(
            focus_ring,
            barrel,
            axes="xy",
            min_overlap=0.105,
            name="focus ring remains coaxial when rotated",
        )

    with ctx.pose({collar_rotation: 0.0}):
        rest_center = _aabb_center(ctx.part_element_world_aabb(tripod_collar, elem="foot"))
    with ctx.pose({collar_rotation: math.pi / 2.0}):
        quarter_turn_center = _aabb_center(ctx.part_element_world_aabb(tripod_collar, elem="foot"))

    ctx.check(
        "tripod collar foot rotates around the lens axis",
        rest_center is not None
        and quarter_turn_center is not None
        and rest_center[1] < -0.085
        and abs(rest_center[0]) < 0.020
        and quarter_turn_center[0] > 0.085
        and abs(quarter_turn_center[1]) < 0.020,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


object_model = build_object_model()
