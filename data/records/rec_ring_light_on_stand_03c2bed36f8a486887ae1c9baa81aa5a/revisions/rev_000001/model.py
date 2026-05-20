from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 96) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(math.tau * index / segments), radius * math.sin(math.tau * index / segments))
        for index in range(segments)
    ]


def _annulus_mesh(name: str, outer_radius: float, inner_radius: float, depth: float, segments: int = 128):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments),
            [_circle_profile(inner_radius, segments)],
            depth,
            center=True,
        ),
        name,
    )


def _tube_shell_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, height)],
            [(inner_radius, 0.0), (inner_radius, height)],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_ring_light")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.058, 0.064, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.91, 0.70, 0.76))
    warm_led = model.material("warm_led", rgba=(1.0, 0.72, 0.32, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.105, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=matte_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.087, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=satin_black,
        name="raised_top_disc",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_metal,
        name="mast_socket",
    )
    base.visual(
        _tube_shell_mesh("outer_sleeve", outer_radius=0.0175, inner_radius=0.0118, height=0.278),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=dark_metal,
        name="outer_sleeve",
    )
    base.visual(
        _tube_shell_mesh("slide_collar", outer_radius=0.025, inner_radius=0.0122, height=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=satin_black,
        name="slide_collar",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0094, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=brushed_steel,
        name="inner_mast",
    )
    mast.visual(
        Cylinder(radius=0.0122, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=satin_black,
        name="lower_bushing",
    )
    mast.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=satin_black,
        name="top_collar",
    )
    mast.visual(
        Box((0.050, 0.030, 0.160)),
        origin=Origin(xyz=(0.0, 0.125, 0.175)),
        material=satin_black,
        name="rear_offset_block",
    )
    mast.visual(
        Box((0.050, 0.150, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.105)),
        material=satin_black,
        name="rear_neck",
    )
    mast.visual(
        Box((0.340, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, 0.075, 0.244)),
        material=satin_black,
        name="yoke_bridge",
    )
    mast.visual(
        Box((0.020, 0.038, 0.104)),
        origin=Origin(xyz=(-0.158, 0.075, 0.292)),
        material=satin_black,
        name="yoke_cheek_0",
    )
    mast.visual(
        Box((0.020, 0.038, 0.104)),
        origin=Origin(xyz=(0.158, 0.075, 0.292)),
        material=satin_black,
        name="yoke_cheek_1",
    )

    head = model.part("head")
    head.visual(
        _annulus_mesh("ring_body", outer_radius=0.132, inner_radius=0.092, depth=0.024),
        origin=Origin(xyz=(0.0, -0.115, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="ring_body",
    )
    head.visual(
        _annulus_mesh("front_diffuser", outer_radius=0.125, inner_radius=0.098, depth=0.004),
        origin=Origin(xyz=(0.0, -0.129, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_diffuser,
        name="front_diffuser",
    )
    for index in range(16):
        angle = math.tau * index / 16.0
        head.visual(
            Sphere(radius=0.0045),
            origin=Origin(
                xyz=(
                    0.112 * math.cos(angle),
                    -0.133,
                    0.112 * math.sin(angle),
                )
            ),
            material=warm_led,
            name=f"led_lens_{index:02d}",
        )
    head.visual(
        Box((0.040, 0.120, 0.050)),
        origin=Origin(xyz=(-0.116, -0.055, 0.0)),
        material=matte_black,
        name="side_lug_0",
    )
    head.visual(
        Box((0.040, 0.120, 0.050)),
        origin=Origin(xyz=(0.116, -0.055, 0.0)),
        material=matte_black,
        name="side_lug_1",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="center_hub",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.340),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="trunnion_shaft",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(-0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lock_knob_0",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="lock_knob_1",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.20, lower=0.0, upper=0.150),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.075, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.8, lower=-math.pi / 4.0, upper=math.pi / 4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head")
    slide = object_model.get_articulation("mast_slide")
    tilt = object_model.get_articulation("head_tilt")

    ctx.allow_overlap(
        base,
        mast,
        elem_a="outer_sleeve",
        elem_b="lower_bushing",
        reason="The sliding bushing is intentionally seated inside the sleeve with slight proxy interference.",
    )
    ctx.allow_overlap(
        head,
        mast,
        elem_a="trunnion_shaft",
        elem_b="yoke_cheek_0",
        reason="The tilt axle is intentionally captured through the yoke cheek bore proxy.",
    )
    ctx.allow_overlap(
        head,
        mast,
        elem_a="trunnion_shaft",
        elem_b="yoke_cheek_1",
        reason="The tilt axle is intentionally captured through the yoke cheek bore proxy.",
    )

    ctx.check(
        "mast travel is 150 mm",
        slide.motion_limits is not None
        and abs((slide.motion_limits.upper or 0.0) - (slide.motion_limits.lower or 0.0) - 0.150) < 1e-6,
        details=f"limits={slide.motion_limits}",
    )
    ctx.check(
        "head tilt covers plus minus forty five degrees",
        tilt.motion_limits is not None
        and abs((tilt.motion_limits.lower or 0.0) + math.pi / 4.0) < 1e-6
        and abs((tilt.motion_limits.upper or 0.0) - math.pi / 4.0) < 1e-6,
        details=f"limits={tilt.motion_limits}",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.001,
        name="inner mast remains centered in outer sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.11,
        name="collapsed mast has retained insertion",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="lower_bushing",
        elem_b="outer_sleeve",
        min_overlap=0.025,
        name="lower bushing is seated in sleeve",
    )

    rest_pos = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.150}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.065,
            name="extended mast remains inserted in sleeve",
        )
        extended_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.145,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_overlap(
        head,
        mast,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="yoke_cheek_0",
        min_overlap=0.005,
        name="trunnion reaches cheek 0",
    )
    ctx.expect_overlap(
        head,
        mast,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="yoke_cheek_1",
        min_overlap=0.005,
        name="trunnion reaches cheek 1",
    )

    rest_aabb = ctx.part_element_world_aabb(head, elem="ring_body")
    with ctx.pose({tilt: math.pi / 4.0}):
        ctx.expect_gap(
            mast,
            head,
            axis="y",
            positive_elem="rear_offset_block",
            negative_elem="ring_body",
            min_gap=0.006,
            name="tilted ring clears rear yoke post",
        )
        ctx.expect_gap(
            head,
            mast,
            axis="z",
            positive_elem="ring_body",
            negative_elem="rear_neck",
            min_gap=0.006,
            name="tilted ring clears lower yoke neck",
        )
        tilted_aabb = ctx.part_element_world_aabb(head, elem="ring_body")
    ctx.check(
        "head tilt visibly changes ring pitch",
        rest_aabb is not None
        and tilted_aabb is not None
        and (tilted_aabb[1][1] - tilted_aabb[0][1]) > (rest_aabb[1][1] - rest_aabb[0][1]) + 0.08,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
