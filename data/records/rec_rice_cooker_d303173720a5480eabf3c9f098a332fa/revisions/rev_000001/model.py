from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _superellipse_loop(
    width: float,
    depth: float,
    z: float,
    *,
    center_y: float = 0.0,
    segments: int = 72,
    exponent: float = 2.7,
) -> list[tuple[float, float, float]]:
    """Counter-clockwise oval/squircle loop in the XY plane."""

    a = width * 0.5
    b = depth * 0.5
    pts: list[tuple[float, float, float]] = []
    power = 2.0 / exponent
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = a * math.copysign(abs(c) ** power, c)
        y = center_y + b * math.copysign(abs(s) ** power, s)
        pts.append((x, y, z))
    return pts


def _lofted_oval_mesh(
    sections: list[tuple[float, float, float]],
    *,
    center_y: float = 0.0,
    segments: int = 72,
    cap_bottom: bool = False,
    cap_top: bool = False,
    exponent: float = 2.7,
) -> MeshGeometry:
    """Loft superellipse section loops given as (width, depth, z)."""

    geom = MeshGeometry()
    loop_indices: list[list[int]] = []
    for width, depth, z in sections:
        loop = _superellipse_loop(
            width, depth, z, center_y=center_y, segments=segments, exponent=exponent
        )
        loop_indices.append([geom.add_vertex(x, y, z) for x, y, z in loop])

    for a_loop, b_loop in zip(loop_indices, loop_indices[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            a0, a1 = a_loop[i], a_loop[j]
            b0, b1 = b_loop[i], b_loop[j]
            geom.add_face(a0, a1, b1)
            geom.add_face(a0, b1, b0)

    if cap_bottom:
        cx = geom.add_vertex(0.0, center_y, sections[0][2])
        first = loop_indices[0]
        for i in range(segments):
            geom.add_face(cx, first[(i + 1) % segments], first[i])

    if cap_top:
        cx = geom.add_vertex(0.0, center_y, sections[-1][2])
        last = loop_indices[-1]
        for i in range(segments):
            geom.add_face(cx, last[i], last[(i + 1) % segments])

    return geom


def _oval_ring_mesh(
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    z_min: float,
    z_max: float,
    *,
    center_y: float = 0.0,
    segments: int = 72,
    exponent: float = 2.7,
) -> MeshGeometry:
    """Thin oval rim with an open center."""

    geom = MeshGeometry()
    loops: list[list[int]] = []
    for width, depth, z in (
        (outer_width, outer_depth, z_min),
        (outer_width, outer_depth, z_max),
        (inner_width, inner_depth, z_min),
        (inner_width, inner_depth, z_max),
    ):
        loops.append(
            [
                geom.add_vertex(x, y, z)
                for x, y, z in _superellipse_loop(
                    width,
                    depth,
                    z,
                    center_y=center_y,
                    segments=segments,
                    exponent=exponent,
                )
            ]
        )

    outer_bot, outer_top, inner_bot, inner_top = loops
    for i in range(segments):
        j = (i + 1) % segments
        # Outer vertical wall.
        geom.add_face(outer_bot[i], outer_bot[j], outer_top[j])
        geom.add_face(outer_bot[i], outer_top[j], outer_top[i])
        # Inner vertical throat.
        geom.add_face(inner_bot[j], inner_bot[i], inner_top[i])
        geom.add_face(inner_bot[j], inner_top[i], inner_top[j])
        # Top annulus.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        # Lower annulus.
        geom.add_face(outer_bot[j], outer_bot[i], inner_bot[i])
        geom.add_face(outer_bot[j], inner_bot[i], inner_bot[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_rice_cooker")

    warm_white = model.material("warm_white_plastic", rgba=(0.93, 0.92, 0.86, 1.0))
    pearl_white = model.material("pearl_lid_plastic", rgba=(0.96, 0.95, 0.90, 1.0))
    dark_glass = model.material("smoked_control_panel", rgba=(0.045, 0.052, 0.058, 1.0))
    charcoal = model.material("charcoal_black", rgba=(0.015, 0.016, 0.018, 1.0))
    soft_gray = model.material("soft_gray_trim", rgba=(0.62, 0.62, 0.58, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.73, 0.72, 0.68, 1.0))
    amber = model.material("amber_indicator", rgba=(1.0, 0.55, 0.08, 1.0))
    green = model.material("green_indicator", rgba=(0.12, 0.72, 0.25, 1.0))
    print_white = model.material("printed_white", rgba=(0.92, 0.92, 0.85, 1.0))

    base = model.part("base")

    body_shell = _lofted_oval_mesh(
        [
            (0.310, 0.360, 0.018),
            (0.355, 0.420, 0.105),
            (0.372, 0.432, 0.185),
            (0.350, 0.398, 0.258),
        ],
        cap_bottom=True,
        cap_top=False,
        exponent=2.85,
    )
    base.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        material=warm_white,
        name="body_shell",
    )

    top_rim = _oval_ring_mesh(
        0.350,
        0.398,
        0.250,
        0.292,
        0.250,
        0.268,
        exponent=2.85,
    )
    base.visual(
        mesh_from_geometry(top_rim, "top_rim"),
        material=warm_white,
        name="top_rim",
    )

    inner_pot = _lofted_oval_mesh(
        [
            (0.214, 0.248, 0.080),
            (0.236, 0.272, 0.170),
            (0.254, 0.296, 0.253),
        ],
        cap_bottom=True,
        cap_top=False,
        exponent=2.35,
    )
    base.visual(
        mesh_from_geometry(inner_pot, "inner_pot"),
        material=stainless,
        name="inner_pot",
    )

    base.visual(
        Box((0.236, 0.012, 0.132)),
        origin=Origin(xyz=(0.0, -0.219, 0.145)),
        material=dark_glass,
        name="front_panel",
    )
    base.visual(
        Box((0.092, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.226, 0.218)),
        material=soft_gray,
        name="front_latch",
    )
    base.visual(
        Cylinder(radius=0.043, length=0.005),
        origin=Origin(xyz=(0.0, -0.224, 0.116), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="dial_bezel",
    )
    # Simple printed cook/warm cues and tick marks on the broad panel.
    base.visual(
        Cylinder(radius=0.0065, length=0.002),
        origin=Origin(xyz=(-0.072, -0.2255, 0.149), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="cook_light",
    )
    base.visual(
        Cylinder(radius=0.0065, length=0.002),
        origin=Origin(xyz=(0.072, -0.2255, 0.149), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=green,
        name="warm_light",
    )
    for idx, (x, z, sx, sz) in enumerate(
        [
            (-0.050, 0.088, 0.026, 0.004),
            (0.050, 0.088, 0.026, 0.004),
            (0.000, 0.172, 0.004, 0.020),
        ]
    ):
        base.visual(
            Box((sx, 0.0035, sz)),
            origin=Origin(xyz=(x, -0.2260, z)),
            material=print_white,
            name=f"panel_mark_{idx}",
        )

    # Rear hinge barrels and molded tabs are part of the stationary housing.
    for idx, x in enumerate((-0.095, 0.095)):
        base.visual(
            Cylinder(radius=0.014, length=0.082),
            origin=Origin(xyz=(x, 0.211, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_white,
            name=f"rear_hinge_barrel_{idx}",
        )
        base.visual(
            Box((0.072, 0.030, 0.040)),
            origin=Origin(xyz=(x, 0.228, 0.276)),
            material=warm_white,
            name=f"rear_hinge_tab_{idx}",
        )
    base.visual(
        Box((0.250, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, 0.213, 0.257)),
        material=warm_white,
        name="rear_hinge_shelf",
    )

    for idx, (x, y) in enumerate(((-0.115, -0.125), (0.115, -0.125), (-0.105, 0.118), (0.105, 0.118))):
        base.visual(
            Cylinder(radius=0.022, length=0.018),
            origin=Origin(xyz=(x, y, 0.011)),
            material=charcoal,
            name=f"foot_{idx}",
        )

    lid = model.part("lid")
    lid_crown = _lofted_oval_mesh(
        [
            (0.338, 0.386, -0.026),
            (0.352, 0.402, 0.012),
            (0.328, 0.374, 0.046),
            (0.260, 0.300, 0.070),
        ],
        center_y=-0.222,
        cap_bottom=False,
        cap_top=True,
        exponent=2.85,
    )
    lid.visual(
        mesh_from_geometry(lid_crown, "lid_crown"),
        material=pearl_white,
        name="lid_crown",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.082, -0.170, 0.079)),
        material=charcoal,
        name="steam_vent",
    )
    for idx, x in enumerate((0.074, 0.082, 0.090)):
        lid.visual(
            Box((0.003, 0.018, 0.002)),
            origin=Origin(xyz=(x, -0.170, 0.089)),
            material=soft_gray,
            name=f"vent_slot_{idx}",
        )
    handle_recess = _oval_ring_mesh(
        0.145,
        0.058,
        0.108,
        0.030,
        0.0,
        0.004,
        segments=48,
        exponent=2.8,
    )
    handle_recess.translate(0.0, -0.300, 0.069)
    lid.visual(
        mesh_from_geometry(handle_recess, "handle_recess"),
        material=soft_gray,
        name="handle_recess",
    )
    lid.visual(
        Cylinder(radius=0.013, length=0.108),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pearl_white,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.118, 0.026, 0.020)),
        origin=Origin(xyz=(0.0, -0.014, -0.010)),
        material=pearl_white,
        name="lid_hinge_tab",
    )

    dial = model.part("selector_dial")
    selector_cap = KnobGeometry(
        0.066,
        0.026,
        body_style="skirted",
        top_diameter=0.052,
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0011, width=0.0022),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=90.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(selector_cap, "selector_cap"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="selector_cap",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="selector_axle",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.211, 0.300)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.32),
    )
    model.articulation(
        "dial_axle",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.2265, 0.116)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("selector_dial")
    lid_hinge = object_model.get_articulation("lid_hinge")
    dial_axle = object_model.get_articulation("dial_axle")

    ctx.allow_overlap(
        base,
        dial,
        elem_a="front_panel",
        elem_b="selector_axle",
        reason="The hidden selector axle intentionally passes through the front control panel.",
    )
    ctx.allow_overlap(
        base,
        dial,
        elem_a="dial_bezel",
        elem_b="selector_axle",
        reason="The axle is captured by the dial bezel so the control reads as mounted, not floating.",
    )

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_crown",
        elem_b="top_rim",
        min_overlap=0.240,
        name="closed lid covers oval cooker opening",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_crown",
        negative_elem="top_rim",
        min_gap=0.002,
        max_gap=0.018,
        name="lid crown sits just above top rim seam",
    )
    ctx.expect_gap(
        base,
        dial,
        axis="y",
        positive_elem="dial_bezel",
        negative_elem="selector_cap",
        max_penetration=0.0005,
        max_gap=0.003,
        name="selector dial seats on front bezel",
    )
    ctx.expect_within(
        dial,
        base,
        axes="xz",
        inner_elem="selector_axle",
        outer_elem="front_panel",
        margin=0.001,
        name="selector axle is centered in the panel",
    )
    ctx.expect_overlap(
        dial,
        base,
        axes="y",
        elem_a="selector_axle",
        elem_b="front_panel",
        min_overlap=0.003,
        name="selector axle has retained insertion",
    )

    rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_crown")
    with ctx.pose({lid_hinge: 1.05}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_crown")
    ctx.check(
        "lid opens upward on rear hinge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.075,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )
    ctx.check(
        "selector dial is continuous",
        dial_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint={dial_axle}",
    )

    return ctx.report()


object_model = build_object_model()
