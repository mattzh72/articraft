from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(2.0 * math.pi * i / segments), radius * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _regular_ngon_profile(radius: float, sides: int, rotation: float = math.pi / 8.0) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(rotation + 2.0 * math.pi * i / sides), radius * math.sin(rotation + 2.0 * math.pi * i / sides))
        for i in range(sides)
    ]


def _hole_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return list(reversed(_circle_profile(radius, segments)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_guarded_thumbstick")

    cast_dark = model.material("dark_cast_alloy", rgba=(0.10, 0.11, 0.12, 1.0))
    cast_panel = model.material("matte_panel_casting", rgba=(0.055, 0.060, 0.065, 1.0))
    machined = model.material("brushed_pin_steel", rgba=(0.58, 0.58, 0.54, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.011, 0.012, 1.0))
    shadow = model.material("dark_recess_shadow", rgba=(0.0, 0.0, 0.0, 1.0))

    # Root housing: a squat console plate with a real octagonal raised bezel,
    # bearing towers for the outer gimbal, trim retainers, and a dark entry seal.
    housing = model.part("housing")
    housing.visual(
        Box((0.230, 0.180, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=cast_panel,
        name="base_plate",
    )
    housing.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _regular_ngon_profile(0.088, 8),
                [_hole_profile(0.058, 72)],
                0.018,
                center=True,
            ),
            "octagonal_bezel",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=cast_dark,
        name="octagonal_bezel",
    )
    housing.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.051, 72),
                [_hole_profile(0.023, 48)],
                0.006,
                center=True,
            ),
            "boot_retain_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=black,
        name="boot_retain_ring",
    )
    housing.visual(
        Box((0.028, 0.068, 0.062)),
        origin=Origin(xyz=(0.088, 0.0, 0.053)),
        material=cast_dark,
        name="bearing_tower_0",
    )
    housing.visual(
        Box((0.028, 0.068, 0.062)),
        origin=Origin(xyz=(-0.088, 0.0, 0.053)),
        material=cast_dark,
        name="bearing_tower_1",
    )
    housing.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(0.070, 0.0, 0.072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="bearing_bush_0",
    )
    housing.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(-0.070, 0.0, 0.072), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="bearing_bush_1",
    )
    for i, (x, y) in enumerate(((0.0, 0.082), (0.0, -0.082), (0.082, 0.0), (-0.082, 0.0))):
        housing.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(x, y, 0.042)),
            material=machined,
            name=f"trim_retainer_{i}",
        )
    housing.visual(
        Box((0.050, 0.008, 0.007)),
        origin=Origin(xyz=(0.0, 0.067, 0.0435)),
        material=cast_dark,
        name="travel_stop_0",
    )
    housing.visual(
        Box((0.050, 0.008, 0.007)),
        origin=Origin(xyz=(0.0, -0.067, 0.0435)),
        material=cast_dark,
        name="travel_stop_1",
    )
    housing.visual(
        Box((0.008, 0.050, 0.007)),
        origin=Origin(xyz=(0.067, 0.0, 0.0435)),
        material=cast_dark,
        name="travel_stop_2",
    )
    housing.visual(
        Box((0.008, 0.050, 0.007)),
        origin=Origin(xyz=(-0.067, 0.0, 0.0435)),
        material=cast_dark,
        name="travel_stop_3",
    )

    # Outer fork/yoke: a compact cast ring with a central clearance window,
    # short trunnion pins on the housing axis, and machined bushings for the
    # orthogonal inner-ring axis.
    outer_fork = model.part("outer_fork")
    outer_fork.visual(
        Box((0.090, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.043, 0.0)),
        material=cast_dark,
        name="frame_arm_0",
    )
    outer_fork.visual(
        Box((0.090, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.043, 0.0)),
        material=cast_dark,
        name="frame_arm_1",
    )
    outer_fork.visual(
        Box((0.014, 0.078, 0.014)),
        origin=Origin(xyz=(0.047, 0.0, 0.0)),
        material=cast_dark,
        name="frame_side_0",
    )
    outer_fork.visual(
        Box((0.014, 0.078, 0.014)),
        origin=Origin(xyz=(-0.047, 0.0, 0.0)),
        material=cast_dark,
        name="frame_side_1",
    )
    outer_fork.visual(
        Box((0.022, 0.010, 0.014)),
        origin=Origin(xyz=(0.040, 0.037, 0.0), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=cast_dark,
        name="corner_web_0",
    )
    outer_fork.visual(
        Box((0.022, 0.010, 0.014)),
        origin=Origin(xyz=(-0.040, 0.037, 0.0), rpy=(0.0, 0.0, -math.pi / 4.0)),
        material=cast_dark,
        name="corner_web_1",
    )
    outer_fork.visual(
        Box((0.022, 0.010, 0.014)),
        origin=Origin(xyz=(0.040, -0.037, 0.0), rpy=(0.0, 0.0, -math.pi / 4.0)),
        material=cast_dark,
        name="corner_web_2",
    )
    outer_fork.visual(
        Box((0.022, 0.010, 0.014)),
        origin=Origin(xyz=(-0.040, -0.037, 0.0), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=cast_dark,
        name="corner_web_3",
    )
    outer_fork.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_dark,
        name="trunnion_boss_0",
    )
    outer_fork.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_dark,
        name="trunnion_boss_1",
    )
    outer_fork.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="trunnion_pin_0",
    )
    outer_fork.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(-0.063, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="trunnion_pin_1",
    )
    outer_fork.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0435, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="ring_bushing_0",
    )
    outer_fork.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, -0.0435, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="ring_bushing_1",
    )
    model.articulation(
        "housing_to_outer_fork",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outer_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.30, upper=0.30),
    )

    # Inner ring and stick: a tight machined gimbal ring with short pivot pins,
    # a webbed hub, and a stubby rubber-capped control post.
    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.027, 72),
                [_hole_profile(0.014, 48)],
                0.012,
                center=True,
            ),
            "inner_ring_band",
        ),
        material=machined,
        name="ring_band",
    )
    inner_ring.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(),
        material=machined,
        name="center_hub",
    )
    inner_ring.visual(
        Box((0.040, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined,
        name="hub_spoke_x",
    )
    inner_ring.visual(
        Box((0.006, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined,
        name="hub_spoke_y",
    )
    inner_ring.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="ring_pin_0",
    )
    inner_ring.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, -0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="ring_pin_1",
    )
    inner_ring.visual(
        Cylinder(radius=0.011, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=black,
        name="control_post",
    )
    inner_ring.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=black,
        name="thumb_cap",
    )
    inner_ring.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shadow,
        name="post_socket_shadow",
    )

    model.articulation(
        "outer_fork_to_inner_ring",
        ArticulationType.REVOLUTE,
        parent=outer_fork,
        child=inner_ring,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.30, upper=0.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    outer_fork = object_model.get_part("outer_fork")
    inner_ring = object_model.get_part("inner_ring")
    outer_joint = object_model.get_articulation("housing_to_outer_fork")
    inner_joint = object_model.get_articulation("outer_fork_to_inner_ring")

    ctx.allow_overlap(
        housing,
        outer_fork,
        elem_a="bearing_bush_0",
        elem_b="trunnion_pin_0",
        reason="The outer fork pivot pin is intentionally captured inside the machined housing bushing.",
    )
    ctx.allow_overlap(
        housing,
        outer_fork,
        elem_a="bearing_bush_1",
        elem_b="trunnion_pin_1",
        reason="The opposite outer fork pivot pin is intentionally captured inside the machined housing bushing.",
    )

    ctx.expect_gap(
        outer_fork,
        housing,
        axis="z",
        positive_elem="frame_arm_0",
        negative_elem="octagonal_bezel",
        min_gap=0.010,
        name="fork arm clears octagonal bezel at center",
    )
    ctx.expect_gap(
        housing,
        outer_fork,
        axis="x",
        positive_elem="bearing_bush_0",
        negative_elem="trunnion_pin_0",
        max_penetration=0.008,
        name="positive trunnion is seated by housing bushing",
    )
    ctx.expect_gap(
        outer_fork,
        housing,
        axis="x",
        positive_elem="trunnion_pin_1",
        negative_elem="bearing_bush_1",
        max_penetration=0.008,
        name="negative trunnion is seated by housing bushing",
    )
    ctx.expect_gap(
        outer_fork,
        inner_ring,
        axis="y",
        positive_elem="ring_bushing_0",
        negative_elem="ring_pin_0",
        min_gap=0.0005,
        max_gap=0.010,
        name="inner ring positive pin runs in fork bushing clearance",
    )
    ctx.expect_gap(
        inner_ring,
        outer_fork,
        axis="y",
        positive_elem="ring_pin_1",
        negative_elem="ring_bushing_1",
        min_gap=0.0005,
        max_gap=0.010,
        name="inner ring negative pin runs in fork bushing clearance",
    )

    for outer_q in (-0.30, 0.30):
        with ctx.pose({outer_joint: outer_q}):
            lowered_arm = "frame_arm_0" if outer_q < 0.0 else "frame_arm_1"
            ctx.expect_gap(
                outer_fork,
                housing,
                axis="z",
                positive_elem=lowered_arm,
                negative_elem="travel_stop_0",
                min_gap=0.0025,
                name=f"outer fork clears raised stops at {outer_q:+.2f} rad",
            )
            aabb = ctx.part_world_aabb(inner_ring)
            ctx.check(
                f"inner assembly remains above housing at outer {outer_q:+.2f} rad",
                aabb is not None and aabb[0][2] > 0.039,
                details=f"aabb={aabb}",
            )

    for inner_q in (-0.30, 0.30):
        with ctx.pose({inner_joint: inner_q}):
            aabb = ctx.part_world_aabb(inner_ring)
            ctx.check(
                f"inner ring tilt stays inside fork guard at {inner_q:+.2f} rad",
                aabb is not None and abs(aabb[0][0]) < 0.080 and abs(aabb[1][0]) < 0.080 and aabb[0][2] > -0.020,
                details=f"aabb={aabb}",
            )

    return ctx.report()


object_model = build_object_model()
