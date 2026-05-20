from __future__ import annotations

from math import pi

import cadquery as cq
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
    mesh_from_cadquery,
    mesh_from_geometry,
)


PIVOT_X = 0.55
PIVOT_Z = 2.35
BOOM_LENGTH = 5.45
BOOM_ROOT_X = 0.08


def _tapered_boom_mesh() -> MeshGeometry:
    """Closed tapered rectangular boom, authored in the pivot frame."""
    x0 = BOOM_ROOT_X
    x1 = BOOM_ROOT_X + BOOM_LENGTH
    root_y = 0.070
    root_z = 0.075
    tip_y = 0.040
    tip_z = 0.045

    verts = [
        (x0, -root_y, -root_z),
        (x0, root_y, -root_z),
        (x0, root_y, root_z),
        (x0, -root_y, root_z),
        (x1, -tip_y, -tip_z),
        (x1, tip_y, -tip_z),
        (x1, tip_y, tip_z),
        (x1, -tip_y, tip_z),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 7, 6),
        (4, 6, 5),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]

    geom = MeshGeometry()
    for v in verts:
        geom.add_vertex(*v)
    for f in faces:
        geom.add_face(*f)
    return geom


def _make_hinge_bracket_shape() -> cq.Workplane:
    """Cast-iron yoke bracket in a local frame whose origin is the pivot pin."""
    cheek_y = 0.240
    cheek_thickness = 0.055
    cheek = cq.Workplane("XY").box(0.74, cheek_thickness, 0.48)
    bracket = cheek.translate((-0.15, cheek_y, 0.0)).union(
        cheek.translate((-0.15, -cheek_y, 0.0))
    )

    # Rear mounting casting: overlaps the post and leaves the center clear for
    # the short counterweight tail.
    bracket = bracket.union(cq.Workplane("XY").box(0.12, 0.60, 0.54).translate((-0.54, 0.0, 0.0)))
    bracket = bracket.union(cq.Workplane("XY").box(0.24, 0.52, 0.070).translate((-0.41, 0.0, 0.225)))
    bracket = bracket.union(cq.Workplane("XY").box(0.24, 0.52, 0.070).translate((-0.41, 0.0, -0.225)))

    # Raised bosses around the pin holes on both outer cheeks.
    boss = cq.Workplane("XY").cylinder(0.040, 0.105).rotate((0, 0, 0), (1, 0, 0), 90)
    bracket = bracket.union(boss.translate((0.0, cheek_y + 0.045, 0.0)))
    bracket = bracket.union(boss.translate((0.0, -cheek_y - 0.045, 0.0)))

    # Through bore for the steel pivot pin.
    pin_bore = cq.Workplane("XY").cylinder(0.72, 0.058).rotate((0, 0, 0), (1, 0, 0), 90)
    return bracket.cut(pin_bore)


def _make_footing_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(1.05, 0.95, 0.32).edges("|Z").chamfer(0.035)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="railway_crossing_gate")

    model.material("aged_concrete", rgba=(0.52, 0.51, 0.48, 1.0))
    model.material("painted_steel", rgba=(0.08, 0.09, 0.08, 1.0))
    model.material("cast_iron", rgba=(0.035, 0.037, 0.04, 1.0))
    model.material("galvanized_pin", rgba=(0.62, 0.64, 0.63, 1.0))
    model.material("boom_white", rgba=(0.94, 0.92, 0.84, 1.0))
    model.material("signal_red", rgba=(0.82, 0.04, 0.03, 1.0))

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_make_footing_shape(), "concrete_footing"),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material="aged_concrete",
        name="concrete_footing",
    )
    post.visual(
        Cylinder(radius=0.090, length=2.12),
        origin=Origin(xyz=(0.0, 0.0, 1.34)),
        material="painted_steel",
        name="tall_post",
    )
    post.visual(
        Cylinder(radius=0.155, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material="painted_steel",
        name="base_flange",
    )
    post.visual(
        mesh_from_cadquery(_make_hinge_bracket_shape(), "hinge_bracket"),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        material="cast_iron",
        name="hinge_bracket",
    )

    boom = model.part("boom")
    boom.visual(
        mesh_from_geometry(_tapered_boom_mesh(), "tapered_boom"),
        material="boom_white",
        name="tapered_boom",
    )
    boom.visual(
        Cylinder(radius=0.112, length=0.300),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="cast_iron",
        name="pivot_hub",
    )
    boom.visual(
        Cylinder(radius=0.061, length=0.650),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="galvanized_pin",
        name="pivot_pin",
    )
    boom.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.335, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="galvanized_pin",
        name="pin_head_0",
    )
    boom.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, -0.335, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="galvanized_pin",
        name="pin_head_1",
    )
    boom.visual(
        Box((0.42, 0.095, 0.095)),
        origin=Origin(xyz=(-0.210, 0.0, 0.0)),
        material="cast_iron",
        name="counterweight_tail",
    )
    boom.visual(
        Box((0.24, 0.315, 0.290)),
        origin=Origin(xyz=(-0.285, 0.0, 0.0)),
        material="cast_iron",
        name="counterweight",
    )

    for i, x in enumerate((0.55, 1.22, 1.89, 2.56, 3.23, 3.90, 4.57, 5.20)):
        t = max(0.0, min(1.0, (x - BOOM_ROOT_X) / BOOM_LENGTH))
        stripe_y = 0.150 * (1.0 - t) + 0.095 * t
        stripe_z = 0.160 * (1.0 - t) + 0.100 * t
        boom.visual(
            Box((0.285, stripe_y, stripe_z)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material="signal_red",
            name=f"stripe_{i}",
        )
    boom.visual(
        Box((0.040, 0.095, 0.105)),
        origin=Origin(xyz=(BOOM_ROOT_X + BOOM_LENGTH + 0.020, 0.0, 0.0)),
        material="signal_red",
        name="end_cap",
    )

    model.articulation(
        "post_to_boom",
        ArticulationType.REVOLUTE,
        parent=post,
        child=boom,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=900.0, velocity=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    boom = object_model.get_part("boom")
    hinge = object_model.get_articulation("post_to_boom")

    ctx.expect_within(
        boom,
        post,
        axes="y",
        inner_elem="pivot_hub",
        outer_elem="hinge_bracket",
        margin=0.001,
        name="hub sits between hinge cheeks",
    )
    ctx.allow_overlap(
        post,
        boom,
        elem_a="hinge_bracket",
        elem_b="pivot_pin",
        reason="The steel pivot pin is intentionally captured in the bracket bore with a tiny modeled interference fit.",
    )
    ctx.expect_overlap(
        boom,
        post,
        axes="y",
        elem_a="pivot_pin",
        elem_b="hinge_bracket",
        min_overlap=0.45,
        name="pin spans through both bracket cheeks",
    )
    ctx.expect_within(
        boom,
        post,
        axes="y",
        inner_elem="counterweight",
        outer_elem="hinge_bracket",
        margin=0.001,
        name="counterweight clears bracket cheek span",
    )
    ctx.expect_gap(
        boom,
        post,
        axis="x",
        positive_elem="tapered_boom",
        negative_elem="tall_post",
        min_gap=0.50,
        name="long arm projects outward from post",
    )

    closed_aabb = ctx.part_world_aabb(boom)
    with ctx.pose({hinge: 1.20}):
        raised_aabb = ctx.part_world_aabb(boom)
    ctx.check(
        "boom raises upward about the pivot",
        closed_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > closed_aabb[1][2] + 3.5,
        details=f"closed={closed_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
