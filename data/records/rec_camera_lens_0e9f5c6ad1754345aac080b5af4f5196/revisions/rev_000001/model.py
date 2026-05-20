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
    mesh_from_cadquery,
)


LENS_LENGTH = 0.058
BARREL_RADIUS = 0.0260
MOUNT_RADIUS = 0.0310
FRONT_BARREL_RADIUS = 0.0285
FILTER_RING_RADIUS = 0.0304

APERTURE_START = 0.0052
APERTURE_LENGTH = 0.0098
APERTURE_INNER_RADIUS = 0.0266
APERTURE_OUTER_RADIUS = 0.0292
APERTURE_RIB_RADIUS = 0.0298

FOCUS_START = 0.0180
FOCUS_LENGTH = 0.0255
FOCUS_INNER_RADIUS = 0.0274
FOCUS_OUTER_RADIUS = 0.0312
FOCUS_RIB_RADIUS = 0.0319


def _annular_ring(
    *,
    start_z: float,
    length: float,
    inner_radius: float,
    outer_radius: float,
    rib_radius: float,
    rib_count: int,
) -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .workplane(offset=start_z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    pitch = length / rib_count
    rib_length = min(0.0024, pitch * 0.62)
    for index in range(rib_count):
        rib_z = start_z + (index + 0.5) * pitch - (0.5 * rib_length)
        ring = ring.union(
            cq.Workplane("XY")
            .workplane(offset=rib_z)
            .circle(rib_radius)
            .circle(inner_radius)
            .extrude(rib_length)
        )
    return ring


def _barrel_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").circle(BARREL_RADIUS).extrude(0.046)
    shell = shell.union(cq.Workplane("XY").circle(MOUNT_RADIUS).extrude(0.0028))
    shell = shell.union(
        cq.Workplane("XY").workplane(offset=0.046).circle(FRONT_BARREL_RADIUS).extrude(0.009)
    )
    shell = shell.union(
        cq.Workplane("XY").workplane(offset=0.055).circle(FILTER_RING_RADIUS).extrude(0.003)
    )
    shell = shell.cut(cq.Workplane("XY").workplane(offset=-0.001).circle(0.0204).extrude(0.060))
    shell = shell.cut(cq.Workplane("XY").workplane(offset=0.047).circle(0.0220).extrude(0.009))
    return shell


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mirrorless_prime_lens")

    barrel_black = model.material("barrel_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.13, 0.13, 0.14, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    index_white = model.material("index_white", rgba=(0.93, 0.94, 0.96, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.12, 0.19, 0.26, 0.72))
    coating_blue = model.material("coating_blue", rgba=(0.18, 0.28, 0.38, 0.55))
    seam_gray = model.material("seam_gray", rgba=(0.30, 0.30, 0.31, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_barrel_shell(), "barrel_shell"),
        material=barrel_black,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=APERTURE_INNER_RADIUS, length=APERTURE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, APERTURE_START + (0.5 * APERTURE_LENGTH))),
        material=barrel_black,
        name="aperture_track",
    )
    barrel.visual(
        Cylinder(radius=FOCUS_INNER_RADIUS, length=FOCUS_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, FOCUS_START + (0.5 * FOCUS_LENGTH))),
        material=barrel_black,
        name="focus_track",
    )
    barrel.visual(
        Cylinder(radius=0.0309, length=0.0021),
        origin=Origin(xyz=(0.0, 0.0, 0.00105)),
        material=mount_metal,
        name="mount_flange",
    )
    barrel.visual(
        Box((0.0010, 0.0034, 0.0016)),
        origin=Origin(xyz=(APERTURE_INNER_RADIUS - 0.0002, 0.0, APERTURE_START + APERTURE_LENGTH + 0.0005)),
        material=index_white,
        name="index_line",
    )

    front_glass = model.part("front_glass")
    front_glass.visual(
        Cylinder(radius=0.0220, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0526)),
        material=glass_blue,
        name="front_glass",
    )
    front_glass.visual(
        Cylinder(radius=0.0172, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0540)),
        material=coating_blue,
        name="front_coating",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(
            _annular_ring(
                start_z=FOCUS_START,
                length=FOCUS_LENGTH,
                inner_radius=FOCUS_INNER_RADIUS,
                outer_radius=FOCUS_OUTER_RADIUS,
                rib_radius=FOCUS_RIB_RADIUS,
                rib_count=7,
            ),
            "focus_ring",
        ),
        material=rubber_black,
        name="focus_band",
    )
    focus_ring.visual(
        Box((0.0012, 0.0024, 0.0052)),
        origin=Origin(xyz=(FOCUS_RIB_RADIUS - 0.0004, 0.0, FOCUS_START + (0.5 * FOCUS_LENGTH))),
        material=seam_gray,
        name="focus_mark",
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_cadquery(
            _annular_ring(
                start_z=APERTURE_START,
                length=APERTURE_LENGTH,
                inner_radius=APERTURE_INNER_RADIUS,
                outer_radius=APERTURE_OUTER_RADIUS,
                rib_radius=APERTURE_RIB_RADIUS,
                rib_count=5,
            ),
            "aperture_ring",
        ),
        material=rubber_black,
        name="aperture_band",
    )
    aperture_ring.visual(
        Box((0.0012, 0.0022, 0.0034)),
        origin=Origin(
            xyz=(APERTURE_RIB_RADIUS - 0.0004, 0.0, APERTURE_START + (0.5 * APERTURE_LENGTH))
        ),
        material=index_white,
        name="aperture_mark",
    )

    model.articulation(
        "front_mount",
        ArticulationType.FIXED,
        parent=barrel,
        child=front_glass,
        origin=Origin(),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=focus_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    model.articulation(
        "aperture_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=aperture_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.18, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    focus_ring = object_model.get_part("focus_ring")
    aperture_ring = object_model.get_part("aperture_ring")
    front_glass = object_model.get_part("front_glass")
    focus_rotation = object_model.get_articulation("focus_rotation")
    aperture_rotation = object_model.get_articulation("aperture_rotation")

    ctx.allow_overlap(
        barrel,
        focus_ring,
        elem_a="focus_track",
        elem_b="focus_band",
        reason="The focus ring is modeled as a rotating sleeve riding on a simplified coaxial journal track inside the barrel.",
    )
    ctx.allow_overlap(
        barrel,
        aperture_ring,
        elem_a="aperture_track",
        elem_b="aperture_band",
        reason="The aperture ring is modeled as a rotating sleeve riding on a simplified rear control journal inside the barrel.",
    )

    ctx.expect_origin_distance(
        focus_ring,
        barrel,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="focus ring stays centered on the barrel axis",
    )
    ctx.expect_origin_distance(
        aperture_ring,
        barrel,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="aperture ring stays centered on the barrel axis",
    )
    ctx.expect_origin_distance(
        front_glass,
        barrel,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="front glass stays centered in the barrel opening",
    )

    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="z",
        elem_a="focus_band",
        elem_b="focus_track",
        min_overlap=0.023,
        name="focus ring spans the forward grip track",
    )
    ctx.expect_overlap(
        aperture_ring,
        barrel,
        axes="z",
        elem_a="aperture_band",
        elem_b="aperture_track",
        min_overlap=0.008,
        name="aperture ring spans the rear control band",
    )
    glass_center = _aabb_center(ctx.part_element_world_aabb(front_glass, elem="front_glass"))
    ctx.check(
        "front element sits near the front opening",
        glass_center is not None and glass_center[2] > 0.050 and glass_center[2] < 0.0555,
        details=f"center={glass_center}",
    )

    focus_mark_rest = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_mark"))
    with ctx.pose({focus_rotation: 0.95}):
        focus_mark_turned = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_mark"))
    focus_moves = (
        focus_mark_rest is not None
        and focus_mark_turned is not None
        and math.hypot(
            focus_mark_turned[0] - focus_mark_rest[0],
            focus_mark_turned[1] - focus_mark_rest[1],
        )
        > 0.018
        and abs(focus_mark_turned[2] - focus_mark_rest[2]) < 0.001
    )
    ctx.check(
        "focus ring rotates about the lens axis",
        focus_moves,
        details=f"rest={focus_mark_rest}, turned={focus_mark_turned}",
    )

    aperture_mark_rest = _aabb_center(
        ctx.part_element_world_aabb(aperture_ring, elem="aperture_mark")
    )
    with ctx.pose({aperture_rotation: -0.85}):
        aperture_mark_turned = _aabb_center(
            ctx.part_element_world_aabb(aperture_ring, elem="aperture_mark")
        )
    aperture_moves = (
        aperture_mark_rest is not None
        and aperture_mark_turned is not None
        and math.hypot(
            aperture_mark_turned[0] - aperture_mark_rest[0],
            aperture_mark_turned[1] - aperture_mark_rest[1],
        )
        > 0.013
        and abs(aperture_mark_turned[2] - aperture_mark_rest[2]) < 0.001
    )
    ctx.check(
        "aperture ring rotates about its rear coaxial band",
        aperture_moves,
        details=f"rest={aperture_mark_rest}, turned={aperture_mark_turned}",
    )

    return ctx.report()


object_model = build_object_model()
