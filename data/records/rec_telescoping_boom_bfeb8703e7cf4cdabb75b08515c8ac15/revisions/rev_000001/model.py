from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def rectangular_tube_mesh(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    x_min: float = 0.0,
) -> MeshGeometry:
    """A single watertight open-ended rectangular box-beam mesh along +X."""
    x_max = x_min + length
    outer = [
        (-width / 2.0, -height / 2.0),
        (width / 2.0, -height / 2.0),
        (width / 2.0, height / 2.0),
        (-width / 2.0, height / 2.0),
    ]
    inner_width = width - 2.0 * wall
    inner_height = height - 2.0 * wall
    inner = [
        (-inner_width / 2.0, -inner_height / 2.0),
        (inner_width / 2.0, -inner_height / 2.0),
        (inner_width / 2.0, inner_height / 2.0),
        (-inner_width / 2.0, inner_height / 2.0),
    ]

    geom = MeshGeometry()
    indices: dict[tuple[int, str, int], int] = {}
    for end, x in enumerate((x_min, x_max)):
        for kind, profile in (("outer", outer), ("inner", inner)):
            for i, (y, z) in enumerate(profile):
                indices[(end, kind, i)] = geom.add_vertex(x, y, z)

    def idx(end: int, kind: str, i: int) -> int:
        return indices[(end, kind, i % 4)]

    def quad(a: int, b: int, c: int, d: int) -> None:
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)

    for i in range(4):
        j = i + 1
        # Outside faces.
        quad(idx(0, "outer", i), idx(1, "outer", i), idx(1, "outer", j), idx(0, "outer", j))
        # Inside bore faces.
        quad(idx(0, "inner", j), idx(1, "inner", j), idx(1, "inner", i), idx(0, "inner", i))
        # Ring faces closing the wall thickness at both open ends.
        quad(idx(0, "outer", j), idx(0, "outer", i), idx(0, "inner", i), idx(0, "inner", j))
        quad(idx(1, "outer", i), idx(1, "outer", j), idx(1, "inner", j), idx(1, "inner", i))

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_service_boom")

    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.08, 1.0))
    warm_yellow = model.material("inner_yellow", rgba=(0.92, 0.77, 0.25, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.55, 0.58, 0.57, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        Box((0.09, 0.52, 0.42)),
        origin=Origin(xyz=(-0.10, 0.0, 0.0)),
        material=dark_steel,
        name="mounting_plate",
    )
    root_bracket.visual(
        Box((0.24, 0.34, 0.040)),
        origin=Origin(xyz=(0.060, 0.0, 0.110)),
        material=dark_steel,
        name="socket_top",
    )
    root_bracket.visual(
        Box((0.24, 0.34, 0.040)),
        origin=Origin(xyz=(0.060, 0.0, -0.110)),
        material=dark_steel,
        name="socket_bottom",
    )
    root_bracket.visual(
        Box((0.24, 0.040, 0.190)),
        origin=Origin(xyz=(0.060, 0.140, 0.0)),
        material=dark_steel,
        name="socket_cheek_0",
    )
    root_bracket.visual(
        Box((0.24, 0.040, 0.190)),
        origin=Origin(xyz=(0.060, -0.140, 0.0)),
        material=dark_steel,
        name="socket_cheek_1",
    )
    root_bracket.visual(
        Box((0.18, 0.040, 0.26)),
        origin=Origin(xyz=(-0.005, 0.215, 0.0), rpy=(0.0, 0.45, 0.0)),
        material=dark_steel,
        name="side_gusset_0",
    )
    root_bracket.visual(
        Box((0.18, 0.040, 0.26)),
        origin=Origin(xyz=(-0.005, -0.215, 0.0), rpy=(0.0, 0.45, 0.0)),
        material=dark_steel,
        name="side_gusset_1",
    )
    for i, (y, z) in enumerate(((-0.17, -0.14), (0.17, -0.14), (-0.17, 0.14), (0.17, 0.14))):
        root_bracket.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(xyz=(-0.151, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"bolt_head_{i}",
        )

    outer_beam = model.part("outer_beam")
    outer_beam.visual(
        mesh_from_geometry(
            rectangular_tube_mesh(length=1.35, width=0.240, height=0.180, wall=0.018),
            "outer_box_beam",
        ),
        material=safety_yellow,
        name="outer_tube",
    )
    outer_beam.visual(
        Box((0.070, 0.270, 0.014)),
        origin=Origin(xyz=(1.318, 0.0, 0.097)),
        material=dark_steel,
        name="outer_lip_top",
    )
    outer_beam.visual(
        Box((0.070, 0.270, 0.014)),
        origin=Origin(xyz=(1.318, 0.0, -0.097)),
        material=dark_steel,
        name="outer_lip_bottom",
    )
    outer_beam.visual(
        Box((0.070, 0.014, 0.205)),
        origin=Origin(xyz=(1.318, 0.127, 0.0)),
        material=dark_steel,
        name="outer_lip_side_0",
    )
    outer_beam.visual(
        Box((0.070, 0.014, 0.205)),
        origin=Origin(xyz=(1.318, -0.127, 0.0)),
        material=dark_steel,
        name="outer_lip_side_1",
    )

    first_beam = model.part("first_beam")
    first_beam.visual(
        mesh_from_geometry(
            rectangular_tube_mesh(length=1.15, width=0.170, height=0.144, wall=0.014, x_min=-0.850),
            "first_rectangular_beam",
        ),
        material=warm_yellow,
        name="first_tube",
    )
    first_beam.visual(
        Box((0.060, 0.196, 0.010)),
        origin=Origin(xyz=(0.272, 0.0, 0.076)),
        material=dark_steel,
        name="first_lip_top",
    )
    first_beam.visual(
        Box((0.060, 0.196, 0.010)),
        origin=Origin(xyz=(0.272, 0.0, -0.076)),
        material=dark_steel,
        name="first_lip_bottom",
    )

    second_beam = model.part("second_beam")
    second_beam.visual(
        mesh_from_geometry(
            rectangular_tube_mesh(length=0.95, width=0.120, height=0.116, wall=0.011, x_min=-0.700),
            "second_rectangular_beam",
        ),
        material=worn_steel,
        name="second_tube",
    )
    second_beam.visual(
        Box((0.085, 0.130, 0.118)),
        origin=Origin(xyz=(0.285, 0.0, 0.0)),
        material=dark_steel,
        name="fork_bridge",
    )
    second_beam.visual(
        Box((0.330, 0.038, 0.118)),
        origin=Origin(xyz=(0.455, 0.055, 0.0)),
        material=dark_steel,
        name="fork_tine_0",
    )
    second_beam.visual(
        Box((0.330, 0.038, 0.118)),
        origin=Origin(xyz=(0.455, -0.055, 0.0)),
        material=dark_steel,
        name="fork_tine_1",
    )
    second_beam.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.530, 0.076, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="fork_pin_boss_0",
    )
    second_beam.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.530, -0.076, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="fork_pin_boss_1",
    )

    model.articulation(
        "bracket_to_outer",
        ArticulationType.FIXED,
        parent=root_bracket,
        child=outer_beam,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_first",
        ArticulationType.PRISMATIC,
        parent=outer_beam,
        child=first_beam,
        origin=Origin(xyz=(1.350, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.35, lower=0.0, upper=0.60),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first_beam,
        child=second_beam,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    outer = object_model.get_part("outer_beam")
    first = object_model.get_part("first_beam")
    second = object_model.get_part("second_beam")
    outer_slide = object_model.get_articulation("outer_to_first")
    inner_slide = object_model.get_articulation("first_to_second")

    ctx.allow_overlap(
        outer,
        first,
        elem_a="outer_tube",
        elem_b="first_tube",
        reason=(
            "The first rectangular boom stage is intentionally retained inside the outer sleeve; "
            "the mesh collision proxy reports that nested telescoping insertion as overlap."
        ),
    )
    ctx.allow_overlap(
        first,
        second,
        elem_a="first_tube",
        elem_b="second_tube",
        reason=(
            "The second rectangular boom stage is intentionally retained inside the first sleeve; "
            "the mesh collision proxy reports that nested telescoping insertion as overlap."
        ),
    )

    ctx.expect_contact(
        root,
        outer,
        elem_a="socket_top",
        elem_b="outer_tube",
        contact_tol=0.002,
        name="outer box beam is seated against the fixed root bracket",
    )
    ctx.expect_within(
        first,
        outer,
        axes="yz",
        inner_elem="first_tube",
        outer_elem="outer_tube",
        margin=0.0,
        name="first rectangular stage fits inside the outer box beam section",
    )
    ctx.expect_overlap(
        first,
        outer,
        axes="x",
        elem_a="first_tube",
        elem_b="outer_tube",
        min_overlap=0.55,
        name="first stage has long retained insertion when collapsed",
    )
    ctx.expect_within(
        second,
        first,
        axes="yz",
        inner_elem="second_tube",
        outer_elem="first_tube",
        margin=0.0,
        name="second rectangular stage fits inside the first stage section",
    )
    ctx.expect_overlap(
        second,
        first,
        axes="x",
        elem_a="second_tube",
        elem_b="first_tube",
        min_overlap=0.60,
        name="second stage has long retained insertion when collapsed",
    )

    first_rest = ctx.part_world_position(first)
    second_rest = ctx.part_world_position(second)
    with ctx.pose({outer_slide: 0.60, inner_slide: 0.50}):
        ctx.expect_overlap(
            first,
            outer,
            axes="x",
            elem_a="first_tube",
            elem_b="outer_tube",
            min_overlap=0.20,
            name="first stage remains captured at full extension",
        )
        ctx.expect_overlap(
            second,
            first,
            axes="x",
            elem_a="second_tube",
            elem_b="first_tube",
            min_overlap=0.18,
            name="second stage remains captured at full extension",
        )
        ctx.expect_gap(
            second,
            outer,
            axis="x",
            positive_elem="fork_tine_0",
            negative_elem="outer_tube",
            min_gap=0.65,
            name="forked tip moves well beyond the stout outer beam",
        )
        first_extended = ctx.part_world_position(first)
        second_extended = ctx.part_world_position(second)

    ctx.check(
        "sliding stages translate along the shared boom axis",
        first_rest is not None
        and first_extended is not None
        and second_rest is not None
        and second_extended is not None
        and first_extended[0] > first_rest[0] + 0.55
        and second_extended[0] > second_rest[0] + 1.05
        and abs(first_extended[1] - first_rest[1]) < 1e-6
        and abs(second_extended[2] - second_rest[2]) < 1e-6,
        details=(
            f"first rest/extended={first_rest}/{first_extended}, "
            f"second rest/extended={second_rest}/{second_extended}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
