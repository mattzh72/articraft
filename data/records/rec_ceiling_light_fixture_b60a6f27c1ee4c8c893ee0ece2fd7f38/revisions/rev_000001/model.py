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


SLIDER_STARTS = (-0.55, 0.0, 0.55)
SLIDE_TRAVEL = 0.18
RAIL_HALF_LENGTH = 0.90
RAIL_BOTTOM_Z = -0.0175


def _spotlight_can_mesh():
    """Thin-walled, lathed spotlight can with a small rolled front lip."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.043, -0.022),
            (0.052, -0.040),
            (0.056, -0.118),
            (0.060, -0.198),
            (0.057, -0.212),
        ],
        [
            (0.034, -0.025),
            (0.041, -0.044),
            (0.047, -0.118),
            (0.051, -0.192),
            (0.051, -0.208),
        ],
        segments=56,
        start_cap="round",
        end_cap="round",
        lip_samples=7,
    )


def _add_track_visuals(rail, materials) -> None:
    rail.visual(
        Box((1.90, 0.13, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=materials["ceiling"],
        name="ceiling_strip",
    )
    rail.visual(
        Box((1.76, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=materials["ceiling"],
        name="ceiling_neck",
    )
    rail.visual(
        Box((1.80, 0.070, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=materials["paint"],
        name="rail_body",
    )
    rail.visual(
        Box((1.72, 0.026, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, -0.0165)),
        material=materials["shadow"],
        name="recessed_slot",
    )
    for index, y in enumerate((-0.037, 0.037)):
        rail.visual(
            Box((1.76, 0.004, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.002)),
            material=materials["shadow"],
            name=f"side_groove_{index}",
        )
    for index, x in enumerate((-0.78, 0.78)):
        rail.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.055)),
            material=materials["screw"],
            name=f"screw_cap_{index}",
        )


def _add_collar_visuals(collar, materials) -> None:
    collar.visual(
        Box((0.120, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=materials["paint"],
        name="shoe",
    )
    for index, y in enumerate((-0.049, 0.049)):
        collar.visual(
            Box((0.100, 0.012, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.010)),
            material=materials["paint"],
            name=f"side_cheek_{index}",
        )
    collar.visual(
        Box((0.030, 0.030, 0.077)),
        origin=Origin(xyz=(0.0, 0.0, -0.0585)),
        material=materials["paint"],
        name="stem",
    )
    collar.visual(
        Box((0.050, 0.160, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=materials["paint"],
        name="yoke_bridge",
    )
    for index, y in enumerate((-0.076, 0.076)):
        collar.visual(
            Box((0.040, 0.012, 0.072)),
            origin=Origin(xyz=(0.0, y, -0.135)),
            material=materials["paint"],
            name=f"yoke_arm_{index}",
        )
        collar.visual(
            Cylinder(radius=0.019, length=0.012),
            origin=Origin(xyz=(0.0, y, -0.135), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=materials["metal"],
            name=f"bushing_{index}",
        )


def _add_head_visuals(head, can_mesh, materials) -> None:
    head.visual(
        Cylinder(radius=0.011, length=0.142),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=materials["metal"],
        name="pivot_axle",
    )
    head.visual(
        Cylinder(radius=0.039, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=materials["paint"],
        name="pivot_hub",
    )
    head.visual(
        can_mesh,
        material=materials["paint"],
        name="can_shell",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.207)),
        material=materials["lens"],
        name="lens",
    )
    head.visual(
        Cylinder(radius=0.061, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=materials["metal"],
        name="front_trim",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_head_track_lighting")

    materials = {
        "ceiling": model.material("ceiling_white", rgba=(0.94, 0.92, 0.86, 1.0)),
        "paint": model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0)),
        "shadow": model.material("track_shadow", rgba=(0.0, 0.0, 0.0, 1.0)),
        "metal": model.material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0)),
        "screw": model.material("screw_heads", rgba=(0.42, 0.42, 0.41, 1.0)),
        "lens": model.material("warm_lens", rgba=(1.00, 0.86, 0.48, 0.68)),
    }

    rail = model.part("rail")
    _add_track_visuals(rail, materials)

    can_mesh = mesh_from_geometry(_spotlight_can_mesh(), "spotlight_can_shell")

    for index, x in enumerate(SLIDER_STARTS):
        collar = model.part(f"collar_{index}")
        _add_collar_visuals(collar, materials)

        head = model.part(f"head_{index}")
        _add_head_visuals(head, can_mesh, materials)

        model.articulation(
            f"collar_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=rail,
            child=collar,
            origin=Origin(xyz=(x, 0.0, RAIL_BOTTOM_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=22.0,
                velocity=0.25,
                lower=-SLIDE_TRAVEL,
                upper=SLIDE_TRAVEL,
            ),
        )
        model.articulation(
            f"head_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=collar,
            child=head,
            origin=Origin(xyz=(0.0, 0.0, -0.135)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=1.2,
                lower=-0.9,
                upper=0.9,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")

    slides = [object_model.get_articulation(f"collar_slide_{i}") for i in range(3)]
    pivots = [object_model.get_articulation(f"head_pivot_{i}") for i in range(3)]

    ctx.check(
        "three prismatic rail sliders",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in slides),
        details=str([j.articulation_type for j in slides]),
    )
    ctx.check(
        "three horizontal head pivots",
        all(
            j.articulation_type == ArticulationType.REVOLUTE
            and abs(j.axis[1] - 1.0) < 1e-9
            and abs(j.axis[0]) < 1e-9
            and abs(j.axis[2]) < 1e-9
            for j in pivots
        ),
        details=str([(j.articulation_type, j.axis) for j in pivots]),
    )

    for index, slide in enumerate(slides):
        collar = object_model.get_part(f"collar_{index}")
        head = object_model.get_part(f"head_{index}")
        ctx.allow_overlap(
            head,
            collar,
            elem_a="pivot_axle",
            elem_b="bushing_0",
            reason="The spotlight trunnion axle is intentionally captured in the collar bushing.",
        )
        ctx.allow_overlap(
            head,
            collar,
            elem_a="pivot_axle",
            elem_b="bushing_1",
            reason="The spotlight trunnion axle is intentionally captured in the collar bushing.",
        )
        ctx.expect_gap(
            rail,
            collar,
            axis="z",
            positive_elem="rail_body",
            negative_elem="shoe",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"collar_{index} shoe rides on rail underside",
        )
        ctx.expect_within(
            collar,
            rail,
            axes="x",
            inner_elem="shoe",
            outer_elem="rail_body",
            margin=0.0,
            name=f"collar_{index} starts inside rail length",
        )
        rest_position = ctx.part_world_position(collar)
        with ctx.pose({slide: SLIDE_TRAVEL}):
            extended_position = ctx.part_world_position(collar)
            ctx.expect_within(
                collar,
                rail,
                axes="x",
                inner_elem="shoe",
                outer_elem="rail_body",
                margin=0.0,
                name=f"collar_{index} remains captured at slide limit",
            )
        ctx.check(
            f"collar_{index} slides along rail x",
            rest_position is not None
            and extended_position is not None
            and extended_position[0] > rest_position[0] + 0.15,
            details=f"rest={rest_position}, extended={extended_position}",
        )
        ctx.expect_gap(
            head,
            collar,
            axis="y",
            positive_elem="pivot_axle",
            negative_elem="bushing_0",
            max_gap=0.002,
            max_penetration=0.002,
            name=f"head_{index} axle captured by negative bushing",
        )
        ctx.expect_gap(
            collar,
            head,
            axis="y",
            positive_elem="bushing_1",
            negative_elem="pivot_axle",
            max_gap=0.002,
            max_penetration=0.002,
            name=f"head_{index} axle captured by positive bushing",
        )

    head = object_model.get_part("head_1")
    pivot = object_model.get_articulation("head_pivot_1")

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    rest_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")
    with ctx.pose({pivot: 0.8}):
        tipped_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")
    with ctx.pose({pivot: -0.8}):
        reverse_lens_aabb = ctx.part_element_world_aabb(head, elem="lens")

    rest_lens = _aabb_center(rest_lens_aabb) if rest_lens_aabb is not None else None
    tipped_lens = _aabb_center(tipped_lens_aabb) if tipped_lens_aabb is not None else None
    reverse_lens = _aabb_center(reverse_lens_aabb) if reverse_lens_aabb is not None else None
    ctx.check(
        "spot head pivots about horizontal collar axis",
        rest_lens is not None
        and tipped_lens is not None
        and reverse_lens is not None
        and tipped_lens[0] < rest_lens[0] - 0.08
        and reverse_lens[0] > rest_lens[0] + 0.08
        and tipped_lens[2] > rest_lens[2] + 0.035,
        details=f"rest={rest_lens}, tipped={tipped_lens}, reverse={reverse_lens}",
    )

    return ctx.report()


object_model = build_object_model()
