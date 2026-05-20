from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _polar(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def _annular_segment(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 40,
) -> MeshGeometry:
    """Closed vertical annular sector used for curved glass guide walls/tracks."""
    geom = MeshGeometry()
    rows: list[tuple[int, int, int, int]] = []

    for i in range(segments + 1):
        t = i / segments
        angle = start_angle + (end_angle - start_angle) * t
        outer_bottom = geom.add_vertex(*_polar(outer_radius, angle, z_min))
        outer_top = geom.add_vertex(*_polar(outer_radius, angle, z_max))
        inner_bottom = geom.add_vertex(*_polar(inner_radius, angle, z_min))
        inner_top = geom.add_vertex(*_polar(inner_radius, angle, z_max))
        rows.append((outer_bottom, outer_top, inner_bottom, inner_top))

    for i in range(segments):
        ob0, ot0, ib0, it0 = rows[i]
        ob1, ot1, ib1, it1 = rows[i + 1]

        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        geom.add_face(ib0, it1, ib1)
        geom.add_face(ib0, it0, it1)
        geom.add_face(ot0, ot1, it1)
        geom.add_face(ot0, it1, it0)
        geom.add_face(ob0, ib1, ob1)
        geom.add_face(ob0, ib0, ib1)

    first = rows[0]
    last = rows[-1]
    geom.add_face(first[0], first[1], first[3])
    geom.add_face(first[0], first[3], first[2])
    geom.add_face(last[0], last[3], last[1])
    geom.add_face(last[0], last[2], last[3])
    return geom


def _rotated_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    return (
        x * math.cos(yaw) - y * math.sin(yaw),
        x * math.sin(yaw) + y * math.cos(yaw),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="campus_center_revolving_door")

    stone = model.material("warm_stone", rgba=(0.62, 0.57, 0.49, 1.0))
    concrete = model.material("concrete_sill", rgba=(0.42, 0.42, 0.39, 1.0))
    dark_metal = model.material("dark_bronze_metal", rgba=(0.08, 0.07, 0.06, 1.0))
    brushed_metal = model.material("brushed_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.50, 0.78, 0.95, 0.36))
    sign = model.material("campus_sign_panel", rgba=(0.06, 0.17, 0.28, 1.0))

    facade = model.part("facade")

    # A thick campus-center facade wall with an open, flush revolving-door bay.
    wall_thickness = 0.22
    facade.visual(
        Box((1.78, wall_thickness, 3.45)),
        origin=Origin(xyz=(-2.06, 0.0, 1.725)),
        material=stone,
        name="wall_side_0",
    )
    facade.visual(
        Box((1.78, wall_thickness, 3.45)),
        origin=Origin(xyz=(2.06, 0.0, 1.725)),
        material=stone,
        name="wall_side_1",
    )
    facade.visual(
        Box((6.0, wall_thickness, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, 3.09)),
        material=stone,
        name="wall_lintel",
    )
    facade.visual(
        Box((6.2, 3.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=concrete,
        name="floor_slab",
    )
    facade.visual(
        Box((6.0, 0.055, 0.34)),
        origin=Origin(xyz=(0.0, -0.137, 3.05)),
        material=sign,
        name="sign_band",
    )

    # Dark metal jambs and threshold frame the circular drum flush with the facade.
    facade.visual(
        Box((0.07, 0.085, 2.70)),
        origin=Origin(xyz=(-1.205, -0.142, 1.35)),
        material=dark_metal,
        name="jamb_0",
    )
    facade.visual(
        Box((0.07, 0.085, 2.70)),
        origin=Origin(xyz=(1.205, -0.142, 1.35)),
        material=dark_metal,
        name="jamb_1",
    )
    facade.visual(
        Box((2.48, 0.085, 0.08)),
        origin=Origin(xyz=(0.0, -0.142, 2.66)),
        material=dark_metal,
        name="head_frame",
    )
    facade.visual(
        Box((2.48, 0.10, 0.055)),
        origin=Origin(xyz=(0.0, -0.145, 0.0275)),
        material=dark_metal,
        name="threshold",
    )

    guide_inner = 1.12
    guide_outer = 1.18
    right_start = math.radians(-64.0)
    right_end = math.radians(64.0)
    left_start = math.radians(116.0)
    left_end = math.radians(244.0)
    for suffix, start, end in (
        ("0", right_start, right_end),
        ("1", left_start, left_end),
    ):
        facade.visual(
            mesh_from_geometry(
                _annular_segment(guide_inner, guide_outer, 0.07, 2.55, start, end),
                f"guide_glass_{suffix}",
            ),
            material=glass,
            name=f"guide_glass_{suffix}",
        )
        facade.visual(
            mesh_from_geometry(
                _annular_segment(guide_inner - 0.015, guide_outer + 0.015, 0.0, 0.07, start, end),
                f"floor_track_{suffix}",
            ),
            material=dark_metal,
            name=f"floor_track_{suffix}",
        )
        facade.visual(
            mesh_from_geometry(
                _annular_segment(guide_inner - 0.015, guide_outer + 0.015, 2.55, 2.62, start, end),
                f"ceiling_track_{suffix}",
            ),
            material=dark_metal,
            name=f"ceiling_track_{suffix}",
        )
        for end_index, angle in enumerate((start, end)):
            x, y = _polar((guide_inner + guide_outer) * 0.5, angle, 0.0)[:2]
            facade.visual(
                Cylinder(radius=0.032, length=2.55),
                origin=Origin(xyz=(x, y, 1.325)),
                material=dark_metal,
                name=f"guide_mullion_{suffix}_{end_index}",
            )

    # Bearing caps visually carry the central post without blocking the opening.
    facade.visual(
        Cylinder(radius=0.13, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_metal,
        name="bottom_bearing",
    )
    facade.visual(
        Cylinder(radius=0.13, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.61)),
        material=brushed_metal,
        name="top_bearing",
    )

    wings = model.part("wings")
    wings.visual(
        Cylinder(radius=0.065, length=2.53),
        origin=Origin(xyz=(0.0, 0.0, 1.315)),
        material=brushed_metal,
        name="central_post",
    )

    # The panel inner edges tuck slightly into the central post's clamps so the
    # four wings read as one carried rotating assembly rather than loose leaves.
    post_radius = 0.060
    panel_length = 0.93
    panel_thickness = 0.035
    panel_height = 2.18
    panel_bottom = 0.25
    panel_center_z = panel_bottom + panel_height / 2.0
    rail_height = 0.055

    for index, yaw in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        cx, cy = _rotated_xy(post_radius + panel_length / 2.0, 0.0, yaw)
        outer_x, outer_y = _rotated_xy(post_radius + panel_length, 0.0, yaw)
        wings.visual(
            Box((panel_length, panel_thickness, panel_height)),
            origin=Origin(xyz=(cx, cy, panel_center_z), rpy=(0.0, 0.0, yaw)),
            material=glass,
            name=f"wing_glass_{index}",
        )
        wings.visual(
            Box((panel_length, 0.052, rail_height)),
            origin=Origin(xyz=(cx, cy, panel_bottom + rail_height / 2.0), rpy=(0.0, 0.0, yaw)),
            material=dark_metal,
            name=f"wing_bottom_rail_{index}",
        )
        wings.visual(
            Box((panel_length, 0.052, rail_height)),
            origin=Origin(xyz=(cx, cy, panel_bottom + panel_height - rail_height / 2.0), rpy=(0.0, 0.0, yaw)),
            material=dark_metal,
            name=f"wing_top_rail_{index}",
        )
        wings.visual(
            Cylinder(radius=0.025, length=panel_height),
            origin=Origin(xyz=(outer_x, outer_y, panel_center_z)),
            material=dark_metal,
            name=f"wing_outer_stile_{index}",
        )

    model.articulation(
        "facade_to_wings",
        ArticulationType.CONTINUOUS,
        parent=facade,
        child=wings,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    facade = object_model.get_part("facade")
    wings = object_model.get_part("wings")
    spin = object_model.get_articulation("facade_to_wings")

    ctx.check(
        "wing assembly has continuous vertical rotation",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "four rectangular wings are present",
        all(wings.get_visual(f"wing_glass_{i}") is not None for i in range(4)),
        details="expected four named glass wing panels",
    )
    ctx.expect_gap(
        wings,
        facade,
        axis="z",
        positive_elem="central_post",
        negative_elem="bottom_bearing",
        min_gap=0.0,
        max_gap=0.002,
        name="central post seats on bottom bearing",
    )
    ctx.expect_gap(
        facade,
        wings,
        axis="z",
        positive_elem="top_bearing",
        negative_elem="central_post",
        min_gap=0.0,
        max_gap=0.002,
        name="top bearing captures central post",
    )

    rest_aabb = ctx.part_element_world_aabb(wings, elem="wing_glass_0")
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_aabb = ctx.part_element_world_aabb(wings, elem="wing_glass_0")

    if rest_aabb is not None and quarter_aabb is not None:
        rest_center = (
            (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0,
            (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0,
        )
        quarter_center = (
            (quarter_aabb[0][0] + quarter_aabb[1][0]) / 2.0,
            (quarter_aabb[0][1] + quarter_aabb[1][1]) / 2.0,
        )
        moved_correctly = rest_center[0] > 0.45 and abs(rest_center[1]) < 0.04 and quarter_center[1] > 0.45
    else:
        rest_center = quarter_center = None
        moved_correctly = False

    ctx.check(
        "one wing advances around the central post",
        moved_correctly,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_center}",
    )

    return ctx.report()


object_model = build_object_model()
