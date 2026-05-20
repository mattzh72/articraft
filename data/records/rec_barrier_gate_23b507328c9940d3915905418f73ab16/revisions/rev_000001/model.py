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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(cx: float, cy: float, radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_bollard_barrier")

    galvanized = Material("galvanized_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_steel = Material("dark_hinge_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    base_paint = Material("black_base_plate", rgba=(0.025, 0.027, 0.028, 1.0))
    bolt_finish = Material("zinc_anchor_bolts", rgba=(0.72, 0.72, 0.68, 1.0))
    reflective_white = Material("reflective_white", rgba=(0.92, 0.94, 0.90, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.38, 0.28, 0.025)),
        origin=Origin(xyz=(0.045, 0.0, 0.0125)),
        material=base_paint,
        name="base_plate",
    )
    for index, (x, y) in enumerate(((-0.095, -0.095), (-0.095, 0.095), (0.175, -0.095), (0.175, 0.095))):
        base.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x, y, 0.031)),
            material=bolt_finish,
            name=f"anchor_bolt_{index}",
        )

    # The fixed clevis is welded to the plate; its cross pin captures the moving
    # barrel on the post base.
    base.visual(
        Box((0.115, 0.205, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=dark_steel,
        name="hinge_pedestal",
    )
    for side, y in enumerate((-0.108, 0.108)):
        base.visual(
            Box((0.090, 0.030, 0.150)),
            origin=Origin(xyz=(0.0, y, 0.100)),
            material=dark_steel,
            name=f"hinge_cheek_{side}",
        )
        base.visual(
            Cylinder(radius=0.047, length=0.034),
            origin=Origin(xyz=(0.0, y, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"hinge_boss_{side}",
        )
    base.visual(
        Cylinder(radius=0.016, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.100), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_finish,
        name="hinge_pin",
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.043, length=0.170),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="hinge_barrel",
    )
    post.visual(
        Cylinder(radius=0.064, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=galvanized,
        name="post_base_collar",
    )
    post.visual(
        Cylinder(radius=0.055, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=galvanized,
        name="post_tube",
    )
    post.visual(
        Sphere(radius=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=galvanized,
        name="rounded_cap",
    )
    for index, z in enumerate((0.415, 0.665)):
        post.visual(
            Cylinder(radius=0.058, length=0.048),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=reflective_white,
            name=f"reflective_band_{index}",
        )

    receiver_outer = [
        (-0.168, 0.042),
        (-0.055, 0.042),
        (-0.055, 0.158),
        (-0.168, 0.158),
    ]
    receiver_hole = _circle_profile(-0.122, 0.100, 0.024)
    receiver_mesh = ExtrudeWithHolesGeometry(receiver_outer, [receiver_hole], 0.018).rotate_x(math.pi / 2.0)
    post.visual(
        mesh_from_geometry(receiver_mesh, "padlock_receiver"),
        material=dark_steel,
        name="padlock_receiver",
    )

    model.articulation(
        "base_to_post",
        ArticulationType.REVOLUTE,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    hinge = object_model.get_articulation("base_to_post")

    ctx.allow_overlap(
        base,
        post,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The visible hinge pin is intentionally captured inside the moving hinge barrel.",
    )

    ctx.expect_within(
        base,
        post,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="hinge pin is centered in barrel",
    )
    ctx.expect_overlap(
        base,
        post,
        axes="y",
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        min_overlap=0.160,
        name="hinge pin spans the barrel",
    )

    upright_tube_aabb = ctx.part_element_world_aabb(post, elem="post_tube")
    ctx.check(
        "post starts upright",
        upright_tube_aabb is not None
        and (upright_tube_aabb[1][2] - upright_tube_aabb[0][2]) > 0.75
        and (upright_tube_aabb[1][0] - upright_tube_aabb[0][0]) < 0.13,
        details=f"post_tube_aabb={upright_tube_aabb}",
    )

    with ctx.pose({hinge: math.pi / 2.0}):
        folded_tube_aabb = ctx.part_element_world_aabb(post, elem="post_tube")
        ctx.check(
            "post folds to horizontal",
            folded_tube_aabb is not None
            and (folded_tube_aabb[1][0] - folded_tube_aabb[0][0]) > 0.75
            and (folded_tube_aabb[1][2] - folded_tube_aabb[0][2]) < 0.13,
            details=f"post_tube_aabb={folded_tube_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
