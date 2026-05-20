from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_rect_tube_x(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    x_offset: float = 0.0,
) -> MeshGeometry:
    """Closed rectangular tube shell with a through opening along local +X."""
    inner_w = width - 2.0 * wall
    inner_h = height - 2.0 * wall
    if inner_w <= 0.0 or inner_h <= 0.0:
        raise ValueError("wall thickness is too large for the rectangular tube")

    geom = MeshGeometry()
    x0 = x_offset - length / 2.0
    x1 = x_offset + length / 2.0
    y0, y1 = -width / 2.0, width / 2.0
    z0, z1 = -height / 2.0, height / 2.0
    iy0, iy1 = -inner_w / 2.0, inner_w / 2.0
    iz0, iz1 = -inner_h / 2.0, inner_h / 2.0

    sections: list[dict[str, int]] = []
    for x in (x0, x1):
        sections.append(
            {
                "o0": geom.add_vertex(x, y0, z0),
                "o1": geom.add_vertex(x, y1, z0),
                "o2": geom.add_vertex(x, y1, z1),
                "o3": geom.add_vertex(x, y0, z1),
                "i0": geom.add_vertex(x, iy0, iz0),
                "i1": geom.add_vertex(x, iy1, iz0),
                "i2": geom.add_vertex(x, iy1, iz1),
                "i3": geom.add_vertex(x, iy0, iz1),
            }
        )

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    back, front = sections
    for a, b in (("o0", "o1"), ("o1", "o2"), ("o2", "o3"), ("o3", "o0")):
        quad(back[a], front[a], front[b], back[b])
    for a, b in (("i1", "i0"), ("i2", "i1"), ("i3", "i2"), ("i0", "i3")):
        quad(back[a], back[b], front[b], front[a])
    for section in (back, front):
        quad(section["o0"], section["o1"], section["i1"], section["i0"])
        quad(section["o1"], section["o2"], section["i2"], section["i1"])
        quad(section["o2"], section["o3"], section["i3"], section["i2"])
        quad(section["o3"], section["o0"], section["i0"], section["i3"])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_telescoping_boom")

    powder_black = model.material("powder_black", rgba=(0.04, 0.045, 0.05, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.45, 0.48, 0.50, 1.0))
    anodized_gray = model.material("anodized_gray", rgba=(0.25, 0.27, 0.29, 1.0))
    polymer_pad = model.material("polymer_pad", rgba=(0.035, 0.035, 0.032, 1.0))

    root = model.part("root_housing")
    root.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=0.72,
                width=0.32,
                height=0.16,
                wall=0.026,
                x_offset=-0.34,
            ),
            "root_rectangular_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=powder_black,
        name="root_sleeve",
    )
    root.visual(
        Box((0.82, 0.44, 0.028)),
        origin=Origin(xyz=(-0.34, 0.0, 0.014)),
        material=dark_graphite,
        name="base_plate",
    )
    root.visual(
        Box((0.74, 0.29, 0.020)),
        origin=Origin(xyz=(-0.34, 0.0, 0.193)),
        material=dark_graphite,
        name="top_cover",
    )
    root.visual(
        Box((0.74, 0.040, 0.040)),
        origin=Origin(xyz=(-0.34, -0.210, 0.038)),
        material=dark_graphite,
        name="mount_flange_0",
    )
    root.visual(
        Box((0.74, 0.040, 0.040)),
        origin=Origin(xyz=(-0.34, 0.210, 0.038)),
        material=dark_graphite,
        name="mount_flange_1",
    )
    root.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=0.075,
                width=0.34,
                height=0.18,
                wall=0.036,
                x_offset=0.000,
            ),
            "root_front_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=powder_black,
        name="front_collar",
    )

    outer = model.part("outer_tube")
    outer.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=1.12,
                width=0.180,
                height=0.080,
                wall=0.008,
                x_offset=0.060,
            ),
            "outer_rectangular_tube",
        ),
        material=anodized_gray,
        name="outer_shell",
    )
    outer.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=0.070,
                width=0.198,
                height=0.095,
                wall=0.018,
                x_offset=0.610,
            ),
            "outer_front_band",
        ),
        material=satin_steel,
        name="front_band",
    )
    outer.visual(
        Box((0.52, 0.018, 0.014)),
        origin=Origin(xyz=(0.020, -0.050, 0.047)),
        material=polymer_pad,
        name="wear_strip_0",
    )
    outer.visual(
        Box((0.52, 0.018, 0.014)),
        origin=Origin(xyz=(0.020, 0.050, 0.047)),
        material=polymer_pad,
        name="wear_strip_1",
    )

    middle = model.part("middle_tube")
    middle.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=0.90,
                width=0.132,
                height=0.050,
                wall=0.006,
                x_offset=-0.330,
            ),
            "middle_rectangular_tube",
        ),
        material=satin_steel,
        name="middle_shell",
    )
    middle.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=0.060,
                width=0.145,
                height=0.062,
                wall=0.013,
                x_offset=0.100,
            ),
            "middle_front_band",
        ),
        material=anodized_gray,
        name="front_band",
    )
    middle.visual(
        Box((0.38, 0.014, 0.007)),
        origin=Origin(xyz=(-0.150, -0.037, 0.0285)),
        material=polymer_pad,
        name="wear_strip_0",
    )
    middle.visual(
        Box((0.38, 0.014, 0.007)),
        origin=Origin(xyz=(-0.150, 0.037, 0.0285)),
        material=polymer_pad,
        name="wear_strip_1",
    )

    inner = model.part("inner_tube")
    inner.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=0.75,
                width=0.092,
                height=0.030,
                wall=0.0045,
                x_offset=-0.285,
            ),
            "inner_rectangular_tube",
        ),
        material=anodized_gray,
        name="inner_shell",
    )
    inner.visual(
        mesh_from_geometry(
            _hollow_rect_tube_x(
                length=0.050,
                width=0.104,
                height=0.040,
                wall=0.010,
                x_offset=0.075,
            ),
            "inner_front_band",
        ),
        material=satin_steel,
        name="front_band",
    )
    inner.visual(
        Box((0.30, 0.010, 0.004)),
        origin=Origin(xyz=(-0.200, -0.026, 0.017)),
        material=polymer_pad,
        name="wear_strip_0",
    )
    inner.visual(
        Box((0.30, 0.010, 0.004)),
        origin=Origin(xyz=(-0.200, 0.026, 0.017)),
        material=polymer_pad,
        name="wear_strip_1",
    )

    model.articulation(
        "root_to_outer",
        ArticulationType.PRISMATIC,
        parent=root,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.18, lower=0.0, upper=0.35),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.58, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.18, lower=0.0, upper=0.37),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.18, lower=0.0, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_housing")
    outer = object_model.get_part("outer_tube")
    middle = object_model.get_part("middle_tube")
    inner = object_model.get_part("inner_tube")
    root_to_outer = object_model.get_articulation("root_to_outer")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

    for strip in ("wear_strip_0", "wear_strip_1"):
        ctx.allow_overlap(
            outer,
            root,
            elem_a=strip,
            elem_b="root_sleeve",
            reason=(
                "Low-profile telescoping booms use slightly compressed polymer guide "
                "strips against the root sleeve to remove rattle and carry the slider."
            ),
        )
        ctx.allow_overlap(
            outer,
            root,
            elem_a=strip,
            elem_b="front_collar",
            reason=(
                "The same polymer guide strip bears lightly against the root collar "
                "at the sleeve mouth to keep the low boom aligned."
            ),
        )
        ctx.allow_overlap(
            middle,
            outer,
            elem_a=strip,
            elem_b="outer_shell",
            reason=(
                "The middle stage guide strip is intentionally seated against the "
                "inside of the outer rectangular tube as a sliding bearing."
            ),
        )
        ctx.allow_overlap(
            middle,
            outer,
            elem_a=strip,
            elem_b="front_band",
            reason=(
                "The guide strip also runs through the outer stage front wear band "
                "with a small represented preload."
            ),
        )
        ctx.allow_overlap(
            inner,
            middle,
            elem_a=strip,
            elem_b="middle_shell",
            reason=(
                "The inner stage guide strip is intentionally seated against the "
                "inside of the middle rectangular tube as a sliding bearing."
            ),
        )
        ctx.expect_overlap(
            outer,
            root,
            axes="x",
            elem_a=strip,
            elem_b="root_sleeve",
            min_overlap=0.20,
            name=f"{strip} remains guided inside root sleeve",
        )
        ctx.expect_overlap(
            outer,
            root,
            axes="x",
            elem_a=strip,
            elem_b="front_collar",
            min_overlap=0.05,
            name=f"{strip} bears through root collar",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a=strip,
            elem_b="outer_shell",
            min_overlap=0.30,
            name=f"middle {strip} remains guided inside outer tube",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a=strip,
            elem_b="front_band",
            min_overlap=0.035,
            name=f"middle {strip} bears through outer front band",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a=strip,
            elem_b="middle_shell",
            min_overlap=0.25,
            name=f"inner {strip} remains guided inside middle tube",
        )

    ctx.expect_overlap(
        outer,
        root,
        axes="x",
        elem_a="outer_shell",
        elem_b="root_sleeve",
        min_overlap=0.45,
        name="outer tube starts deeply inserted in root housing",
    )
    ctx.expect_overlap(
        middle,
        outer,
        axes="x",
        elem_a="middle_shell",
        elem_b="outer_shell",
        min_overlap=0.55,
        name="middle tube starts nested in outer tube",
    )
    ctx.expect_overlap(
        inner,
        middle,
        axes="x",
        elem_a="inner_shell",
        elem_b="middle_shell",
        min_overlap=0.55,
        name="inner tube starts nested in middle tube",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        inner_elem="middle_shell",
        outer_elem="outer_shell",
        margin=0.0,
        name="middle tube is laterally centered within outer stage",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        inner_elem="inner_shell",
        outer_elem="middle_shell",
        margin=0.0,
        name="inner tube is laterally centered within middle stage",
    )

    rest_outer = ctx.part_world_position(outer)
    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    with ctx.pose(
        {
            root_to_outer: 0.35,
            outer_to_middle: 0.37,
            middle_to_inner: 0.30,
        }
    ):
        ctx.expect_overlap(
            outer,
            root,
            axes="x",
            elem_a="outer_shell",
            elem_b="root_sleeve",
            min_overlap=0.12,
            name="outer tube remains retained at full extension",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_shell",
            elem_b="outer_shell",
            min_overlap=0.35,
            name="middle tube remains retained at full extension",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_shell",
            elem_b="middle_shell",
            min_overlap=0.25,
            name="inner tube remains retained at full extension",
        )
        extended_outer = ctx.part_world_position(outer)
        extended_middle = ctx.part_world_position(middle)
        extended_inner = ctx.part_world_position(inner)

    ctx.check(
        "each rectangular tube extends along boom direction",
        rest_outer is not None
        and rest_middle is not None
        and rest_inner is not None
        and extended_outer is not None
        and extended_middle is not None
        and extended_inner is not None
        and extended_outer[0] > rest_outer[0] + 0.30
        and extended_middle[0] > rest_middle[0] + 0.65
        and extended_inner[0] > rest_inner[0] + 0.90,
        details=(
            f"rest={(rest_outer, rest_middle, rest_inner)}, "
            f"extended={(extended_outer, extended_middle, extended_inner)}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
