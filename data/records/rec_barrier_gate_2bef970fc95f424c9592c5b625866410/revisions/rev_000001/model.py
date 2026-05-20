from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


OUTER_TOP_Z = 0.045
OUTER_BOTTOM_Z = -0.900
OUTER_SIDE = 0.240
OUTER_BORE = 0.170
INNER_SIDE = 0.138
POST_LENGTH = 0.900
POST_TRAVEL = 0.650
HINGE_X = -0.125
HINGE_Z = 0.018


def _square_ring_geometry(
    outer_side: float,
    inner_side: float,
    height: float,
    *,
    outer_radius: float = 0.006,
    inner_radius: float = 0.004,
):
    """A through-open square tube/ring extruded along local Z."""
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_side, outer_side, outer_radius, corner_segments=3),
        [rounded_rect_profile(inner_side, inner_side, inner_radius, corner_segments=3)],
        height,
        cap=True,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_street_bollard")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.98, 0.72, 0.08, 1.0))
    black = model.material("black_band", rgba=(0.02, 0.02, 0.018, 1.0))
    concrete = model.material("brushed_concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    slab_outer = 0.660
    slab_hole = OUTER_SIDE
    slab_thickness = 0.070
    slab_band = (slab_outer - slab_hole) * 0.5
    outer_sleeve.visual(
        Box((slab_outer, slab_band, slab_thickness)),
        origin=Origin(xyz=(0.0, (slab_outer + slab_hole) * 0.25, -slab_thickness * 0.5)),
        material=concrete,
        name="concrete_front",
    )
    outer_sleeve.visual(
        Box((slab_outer, slab_band, slab_thickness)),
        origin=Origin(xyz=(0.0, -(slab_outer + slab_hole) * 0.25, -slab_thickness * 0.5)),
        material=concrete,
        name="concrete_rear",
    )
    outer_sleeve.visual(
        Box((slab_band, slab_hole, slab_thickness)),
        origin=Origin(xyz=((slab_outer + slab_hole) * 0.25, 0.0, -slab_thickness * 0.5)),
        material=concrete,
        name="concrete_side_0",
    )
    outer_sleeve.visual(
        Box((slab_band, slab_hole, slab_thickness)),
        origin=Origin(xyz=(-(slab_outer + slab_hole) * 0.25, 0.0, -slab_thickness * 0.5)),
        material=concrete,
        name="concrete_side_1",
    )

    sleeve_height = OUTER_TOP_Z - OUTER_BOTTOM_Z
    outer_sleeve.visual(
        Box((OUTER_SIDE, (OUTER_SIDE - OUTER_BORE) * 0.5, sleeve_height)),
        origin=Origin(xyz=(0.0, (OUTER_SIDE + OUTER_BORE) * 0.25, (OUTER_TOP_Z + OUTER_BOTTOM_Z) * 0.5)),
        material=galvanized,
        name="sleeve_front_wall",
    )
    outer_sleeve.visual(
        Box((OUTER_SIDE, (OUTER_SIDE - OUTER_BORE) * 0.5, sleeve_height)),
        origin=Origin(xyz=(0.0, -(OUTER_SIDE + OUTER_BORE) * 0.25, (OUTER_TOP_Z + OUTER_BOTTOM_Z) * 0.5)),
        material=galvanized,
        name="sleeve_rear_wall",
    )
    outer_sleeve.visual(
        Box(((OUTER_SIDE - OUTER_BORE) * 0.5, OUTER_BORE, sleeve_height)),
        origin=Origin(xyz=((OUTER_SIDE + OUTER_BORE) * 0.25, 0.0, (OUTER_TOP_Z + OUTER_BOTTOM_Z) * 0.5)),
        material=galvanized,
        name="sleeve_side_wall_0",
    )
    outer_sleeve.visual(
        Box(((OUTER_SIDE - OUTER_BORE) * 0.5, OUTER_BORE, sleeve_height)),
        origin=Origin(xyz=(-(OUTER_SIDE + OUTER_BORE) * 0.25, 0.0, (OUTER_TOP_Z + OUTER_BOTTOM_Z) * 0.5)),
        material=galvanized,
        name="sleeve_side_wall_1",
    )
    flange_outer = 0.285
    flange_inner = 0.178
    flange_thickness = (flange_outer - flange_inner) * 0.5
    flange_z = OUTER_TOP_Z - 0.006
    outer_sleeve.visual(
        Box((flange_outer, flange_thickness, 0.012)),
        origin=Origin(xyz=(0.0, (flange_outer + flange_inner) * 0.25, flange_z)),
        material=galvanized,
        name="flange_front",
    )
    outer_sleeve.visual(
        Box((flange_outer, flange_thickness, 0.012)),
        origin=Origin(xyz=(0.0, -(flange_outer + flange_inner) * 0.25, flange_z)),
        material=galvanized,
        name="flange_rear",
    )
    outer_sleeve.visual(
        Box((flange_thickness, flange_inner, 0.012)),
        origin=Origin(xyz=((flange_outer + flange_inner) * 0.25, 0.0, flange_z)),
        material=galvanized,
        name="flange_side_0",
    )
    outer_sleeve.visual(
        Box((flange_thickness, flange_inner, 0.012)),
        origin=Origin(xyz=(-(flange_outer + flange_inner) * 0.25, 0.0, flange_z)),
        material=galvanized,
        name="flange_side_1",
    )
    outer_sleeve.visual(
        Box((OUTER_SIDE, OUTER_SIDE, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_BOTTOM_Z + 0.010)),
        material=dark_steel,
        name="dark_socket_bottom",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Box((INNER_SIDE, INNER_SIDE, POST_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, -POST_LENGTH * 0.5)),
        material=safety_yellow,
        name="post_body",
    )
    # Four narrow black bands are modeled as shallow strapped plates on the square post.
    for idx, zc in enumerate((-0.165, -0.330)):
        inner_post.visual(
            Box((INNER_SIDE + 0.004, 0.006, 0.040)),
            origin=Origin(xyz=(0.0, INNER_SIDE * 0.5 + 0.003, zc)),
            material=black,
            name=f"front_band_{idx}",
        )
        inner_post.visual(
            Box((INNER_SIDE + 0.004, 0.006, 0.040)),
            origin=Origin(xyz=(0.0, -INNER_SIDE * 0.5 - 0.003, zc)),
            material=black,
            name=f"rear_band_{idx}",
        )
        inner_post.visual(
            Box((0.006, INNER_SIDE + 0.004, 0.040)),
            origin=Origin(xyz=(INNER_SIDE * 0.5 + 0.003, 0.0, zc)),
            material=black,
            name=f"side_band_{idx}_0",
        )
        inner_post.visual(
            Box((0.006, INNER_SIDE + 0.004, 0.040)),
            origin=Origin(xyz=(-INNER_SIDE * 0.5 - 0.003, 0.0, zc)),
            material=black,
            name=f"side_band_{idx}_1",
        )
    inner_post.visual(
        Box((0.016, 0.095, 0.025)),
        origin=Origin(xyz=(-0.077, 0.0, -0.0125)),
        material=dark_steel,
        name="hinge_leaf",
    )
    inner_post.visual(
        Box((INNER_SIDE + 0.010, INNER_SIDE + 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=dark_steel,
        name="top_plate",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=POST_TRAVEL),
    )

    cap = model.part("cap")
    cap.visual(
        Box((0.300, 0.300, 0.016)),
        # The child frame is the hinge pin axis; in the closed pose the plate
        # extends along +X from that pin and rests on the sleeve rim.
        origin=Origin(xyz=(0.150, 0.0, -0.010)),
        material=galvanized,
        name="cap_plate",
    )
    dome_mesh = DomeGeometry(0.105, radial_segments=40, height_segments=10, closed=True)
    dome_mesh.scale(1.0, 1.0, 0.24)
    dome_mesh.translate(0.150, 0.0, -0.002)
    cap.visual(
        mesh_from_geometry(dome_mesh, "raised_dome_cap"),
        material=galvanized,
        name="dome",
    )
    cap.visual(
        Cylinder(radius=0.012, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    cap.visual(
        Cylinder(radius=0.006, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="hinge_pin",
    )

    model.articulation(
        "post_to_cap",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=cap,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_post = object_model.get_part("inner_post")
    cap = object_model.get_part("cap")
    post_slide = object_model.get_articulation("sleeve_to_post")
    cap_hinge = object_model.get_articulation("post_to_cap")

    ctx.expect_within(
        inner_post,
        outer_sleeve,
        axes="xy",
        inner_elem="post_body",
        name="square post is centered in the fixed sleeve footprint",
    )
    ctx.expect_overlap(
        inner_post,
        outer_sleeve,
        axes="z",
        elem_a="post_body",
        elem_b="sleeve_front_wall",
        min_overlap=0.80,
        name="lowered post remains deeply captured in the sleeve",
    )
    ctx.expect_gap(
        cap,
        outer_sleeve,
        axis="z",
        positive_elem="cap_plate",
        negative_elem="flange_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="lowered cap plate seats on the square sleeve rim",
    )
    ctx.expect_overlap(
        cap,
        outer_sleeve,
        axes="xy",
        elem_a="cap_plate",
        min_overlap=0.22,
        name="closed cap covers the square recess",
    )

    rest_post_pos = ctx.part_world_position(inner_post)
    with ctx.pose({post_slide: POST_TRAVEL}):
        ctx.expect_overlap(
            inner_post,
            outer_sleeve,
            axes="z",
            elem_a="post_body",
            elem_b="sleeve_front_wall",
            min_overlap=0.20,
            name="extended post still retains insertion in the sleeve",
        )
        extended_post_pos = ctx.part_world_position(inner_post)

    ctx.check(
        "prismatic joint raises the inner square post",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.60,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    closed_aabb = ctx.part_world_aabb(cap)
    with ctx.pose({cap_hinge: 1.20}):
        opened_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "cap hinge lifts the free edge upward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.08,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
