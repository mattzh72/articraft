from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _superellipse_loop(
    depth: float,
    width: float,
    z: float,
    *,
    center_x: float = 0.0,
    exponent: float = 3.0,
    segments: int = 72,
) -> list[tuple[float, float, float]]:
    """Rounded-rectangle/superellipse footprint loop in X/Y at height z."""
    pts: list[tuple[float, float, float]] = []
    a = depth * 0.5
    b = width * 0.5
    power = 2.0 / exponent
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = center_x + a * math.copysign(abs(c) ** power, c)
        y = b * math.copysign(abs(s) ** power, s)
        pts.append((x, y, z))
    return pts


def _add_loop(geom: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in loop]


def _connect_loops(geom: MeshGeometry, lower: list[int], upper: list[int], *, flip: bool = False) -> None:
    n = len(lower)
    for i in range(n):
        j = (i + 1) % n
        if flip:
            geom.add_face(lower[i], upper[i], upper[j])
            geom.add_face(lower[i], upper[j], lower[j])
        else:
            geom.add_face(lower[i], lower[j], upper[j])
            geom.add_face(lower[i], upper[j], upper[i])


def _fan_cap(geom: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool = False) -> None:
    c = geom.add_vertex(*center)
    n = len(loop)
    for i in range(n):
        j = (i + 1) % n
        if flip:
            geom.add_face(c, loop[j], loop[i])
        else:
            geom.add_face(c, loop[i], loop[j])


def _bin_shell_mesh() -> MeshGeometry:
    """Thin, top-open tapered shell for a compact bathroom step bin."""
    segments = 80
    height = 0.320
    floor_z = 0.018
    wall = 0.006

    outer_bottom = _superellipse_loop(0.205, 0.145, 0.0, exponent=3.4, segments=segments)
    outer_top = _superellipse_loop(0.225, 0.165, height, exponent=3.2, segments=segments)
    inner_top = _superellipse_loop(
        0.225 - 2.0 * wall,
        0.165 - 2.0 * wall,
        height - 0.002,
        exponent=3.2,
        segments=segments,
    )
    inner_floor = _superellipse_loop(
        0.205 - 2.0 * wall,
        0.145 - 2.0 * wall,
        floor_z,
        exponent=3.4,
        segments=segments,
    )

    geom = MeshGeometry()
    ob = _add_loop(geom, outer_bottom)
    ot = _add_loop(geom, outer_top)
    it = _add_loop(geom, inner_top)
    ib = _add_loop(geom, inner_floor)

    _connect_loops(geom, ob, ot)
    _connect_loops(geom, ib, it, flip=True)
    _connect_loops(geom, it, ot, flip=True)  # rounded rolled rim
    _fan_cap(geom, ib, (0.0, 0.0, floor_z), flip=True)  # visible inner floor
    _fan_cap(geom, ob, (0.0, 0.0, 0.0))  # underside
    return geom


def _domed_lid_mesh() -> MeshGeometry:
    """Low superellipse dome; its local frame is the rear hinge line."""
    segments = 80
    depth = 0.235
    width = 0.176
    center_x = -0.111

    geom = MeshGeometry()
    skirt_bottom = _add_loop(
        geom,
        _superellipse_loop(depth, width, 0.000, center_x=center_x, exponent=3.6, segments=segments),
    )
    skirt_top = _add_loop(
        geom,
        _superellipse_loop(depth * 0.985, width * 0.985, 0.010, center_x=center_x, exponent=3.5, segments=segments),
    )
    crown_0 = _add_loop(
        geom,
        _superellipse_loop(depth * 0.88, width * 0.88, 0.022, center_x=center_x, exponent=3.0, segments=segments),
    )
    crown_1 = _add_loop(
        geom,
        _superellipse_loop(depth * 0.55, width * 0.55, 0.034, center_x=center_x, exponent=2.4, segments=segments),
    )

    _connect_loops(geom, skirt_bottom, skirt_top)
    _connect_loops(geom, skirt_top, crown_0)
    _connect_loops(geom, crown_0, crown_1)
    _fan_cap(geom, crown_1, (center_x, 0.0, 0.040))
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_bathroom_step_bin")

    white = model.material("satin_white", rgba=(0.86, 0.88, 0.86, 1.0))
    warm_gray = model.material("warm_gray_plastic", rgba=(0.47, 0.49, 0.48, 1.0))
    dark = model.material("dark_rubber", rgba=(0.035, 0.036, 0.035, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    shadow = model.material("shadow_inside", rgba=(0.08, 0.09, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_bin_shell_mesh(), "bin_shell"),
        material=white,
        name="bin_shell",
    )
    # Dark inner floor makes the top opening read as a hollow bin rather than a solid block.
    # Rear hinge band and brackets are fixed to the body; the lid joint is coaxial.
    body.visual(
        Cylinder(radius=0.006, length=0.154),
        origin=Origin(xyz=(0.113, 0.0, 0.326), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="rear_hinge_band",
    )
    body.visual(
        Box((0.030, 0.018, 0.010)),
        origin=Origin(xyz=(0.104, -0.055, 0.319)),
        material=brushed,
        name="hinge_bracket_0",
    )
    body.visual(
        Box((0.030, 0.018, 0.010)),
        origin=Origin(xyz=(0.104, 0.055, 0.319)),
        material=brushed,
        name="hinge_bracket_1",
    )
    # Captured front pedal shaft and small cheek supports.
    body.visual(
        Cylinder(radius=0.004, length=0.135),
        origin=Origin(xyz=(-0.116, 0.0, 0.048), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="pedal_axle",
    )
    body.visual(
        Box((0.050, 0.012, 0.025)),
        origin=Origin(xyz=(-0.095, -0.066, 0.040)),
        material=white,
        name="pedal_cheek_0",
    )
    body.visual(
        Box((0.050, 0.012, 0.025)),
        origin=Origin(xyz=(-0.095, 0.066, 0.040)),
        material=white,
        name="pedal_cheek_1",
    )
    # Rear soft-close mechanism under a separate rotating cap.
    body.visual(
        Cylinder(radius=0.005, length=0.045),
        origin=Origin(xyz=(0.136, 0.0, 0.329)),
        material=dark,
        name="soft_close_damper",
    )
    body.visual(
        Box((0.058, 0.084, 0.008)),
        origin=Origin(xyz=(0.128, 0.0, 0.320)),
        material=warm_gray,
        name="rear_service_deck",
    )
    body.visual(
        Box((0.030, 0.072, 0.014)),
        origin=Origin(xyz=(0.142, 0.0, 0.326)),
        material=warm_gray,
        name="damper_base",
    )
    body.visual(
        Box((0.030, 0.012, 0.018)),
        origin=Origin(xyz=(0.147, -0.030, 0.339)),
        material=brushed,
        name="cover_hinge_leaf_0",
    )
    body.visual(
        Box((0.030, 0.012, 0.018)),
        origin=Origin(xyz=(0.147, 0.030, 0.339)),
        material=brushed,
        name="cover_hinge_leaf_1",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.158, -0.030, 0.344), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="cover_hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.158, 0.030, 0.344), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="cover_hinge_knuckle_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_domed_lid_mesh(), "domed_lid"),
        material=white,
        name="domed_lid",
    )
    lid.visual(
        Box((0.030, 0.145, 0.006)),
        origin=Origin(xyz=(-0.012, 0.0, 0.002)),
        material=brushed,
        name="lid_hinge_leaf",
    )

    pedal = model.part("pedal")
    pedal.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.078, 0.104, 0.018, corner_segments=8), 0.014, cap=True, center=True),
            "pedal_pad",
        ),
        origin=Origin(xyz=(-0.045, 0.0, -0.023)),
        material=dark,
        name="pedal_pad",
    )
    pedal.visual(
        Cylinder(radius=0.007, length=0.100),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="pivot_sleeve",
    )
    pedal.visual(
        Box((0.056, 0.008, 0.018)),
        origin=Origin(xyz=(-0.026, -0.032, -0.011)),
        material=brushed,
        name="pedal_link_0",
    )
    pedal.visual(
        Box((0.056, 0.008, 0.018)),
        origin=Origin(xyz=(-0.026, 0.032, -0.011)),
        material=brushed,
        name="pedal_link_1",
    )

    damper_cover = model.part("damper_cover")
    damper_cover.visual(
        Cylinder(radius=0.0038, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="cover_center_knuckle",
    )
    damper_cover.visual(
        Box((0.020, 0.030, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, 0.006)),
        material=warm_gray,
        name="cover_hinge_leaf",
    )
    damper_cover.visual(
        Box((0.034, 0.074, 0.006)),
        origin=Origin(xyz=(-0.017, 0.0, 0.018)),
        material=warm_gray,
        name="cap_top",
    )
    damper_cover.visual(
        Box((0.034, 0.004, 0.020)),
        origin=Origin(xyz=(-0.017, -0.039, 0.009)),
        material=warm_gray,
        name="cap_side_0",
    )
    damper_cover.visual(
        Box((0.034, 0.004, 0.020)),
        origin=Origin(xyz=(-0.017, 0.039, 0.009)),
        material=warm_gray,
        name="cap_side_1",
    )
    damper_cover.visual(
        Box((0.005, 0.074, 0.018)),
        origin=Origin(xyz=(-0.034, 0.0, 0.008)),
        material=warm_gray,
        name="cap_front_lip",
    )
    damper_cover.visual(
        Box((0.006, 0.030, 0.018)),
        origin=Origin(xyz=(-0.003, 0.0, 0.012)),
        material=warm_gray,
        name="cap_rear_bridge",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.113, 0.0, 0.332)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(-0.116, 0.0, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.35, upper=0.0),
    )
    model.articulation(
        "body_to_damper_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper_cover,
        origin=Origin(xyz=(0.158, 0.0, 0.344)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.4, lower=0.0, upper=1.10),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    damper_cover = object_model.get_part("damper_cover")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")
    cover_hinge = object_model.get_articulation("body_to_damper_cover")

    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_axle",
        elem_b="pivot_sleeve",
        reason="The pedal sleeve intentionally surrounds the fixed front axle as a captured hinge.",
    )

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="domed_lid",
            negative_elem="bin_shell",
            min_gap=0.004,
            max_gap=0.020,
            name="closed lid sits just above the rolled body rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="domed_lid",
            elem_b="bin_shell",
            min_overlap=0.140,
            name="lid footprint covers compact bin opening",
        )
        ctx.expect_overlap(
            body,
            pedal,
            axes="y",
            elem_a="pedal_axle",
            elem_b="pivot_sleeve",
            min_overlap=0.100,
            name="pedal sleeve spans the front axle",
        )
        ctx.expect_within(
            body,
            pedal,
            axes="xz",
            inner_elem="pedal_axle",
            outer_elem="pivot_sleeve",
            margin=0.003,
            name="pedal axle is centered in sleeve cross-section",
        )
        ctx.expect_overlap(
            damper_cover,
            body,
            axes="xy",
            elem_a="cap_top",
            elem_b="soft_close_damper",
            min_overlap=0.006,
            name="damper cover caps the rear soft-close cylinder",
        )
        ctx.expect_gap(
            damper_cover,
            body,
            axis="z",
            positive_elem="cap_top",
            negative_elem="soft_close_damper",
            min_gap=0.004,
            max_gap=0.020,
            name="damper cap clears the cylinder below",
        )

        closed_lid_aabb = ctx.part_world_aabb(lid)
        closed_pedal_aabb = ctx.part_world_aabb(pedal)
        closed_cover_aabb = ctx.part_world_aabb(damper_cover)

    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({pedal_hinge: -0.30}):
        pressed_pedal_aabb = ctx.part_world_aabb(pedal)
    with ctx.pose({cover_hinge: 1.10}):
        open_cover_aabb = ctx.part_world_aabb(damper_cover)

    ctx.check(
        "lid rotates upward on rear hinge band",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.070,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "front pedal rotates downward from low pivot",
        closed_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < closed_pedal_aabb[0][2] - 0.010,
        details=f"closed={closed_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )
    ctx.check(
        "damper cover flips upward on short rear hinge",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.015,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
