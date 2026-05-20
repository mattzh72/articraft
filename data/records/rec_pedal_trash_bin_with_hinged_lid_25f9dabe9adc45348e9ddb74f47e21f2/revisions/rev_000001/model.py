from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


BODY_DEPTH = 0.240
BODY_WIDTH = 0.180
BODY_HEIGHT = 0.340
WALL = 0.005

HINGE_X = -BODY_DEPTH / 2.0 - 0.004
HINGE_Z = BODY_HEIGHT - 0.002


def _rounded_bin_shell(depth: float, width: float, height: float, wall: float) -> cq.Workplane:
    """Thin open-topped rounded shell with a front sight notch for the liner handle."""
    outer = (
        cq.Workplane("XY")
        .rect(depth, width)
        .extrude(height)
        .edges("|Z")
        .fillet(0.020)
    )
    shell = outer.faces(">Z").shell(-wall)
    notch = (
        cq.Workplane("XY")
        .box(0.050, 0.092, 0.070)
        .translate((depth / 2.0 + 0.006, 0.0, height - 0.043))
    )
    return shell.cut(notch)


def _inner_bucket_shell() -> cq.Workplane:
    """Removable plastic bucket: a smaller hollow liner sitting on the outer base."""
    depth = 0.190
    width = 0.140
    height = 0.305
    bucket = (
        cq.Workplane("XY")
        .rect(depth, width)
        .extrude(height)
        .edges("|Z")
        .fillet(0.012)
        .faces(">Z")
        .shell(-0.003)
    )
    # A small rolled top lip makes the removable liner visible below the lid line.
    lip_outer = (
        cq.Workplane("XY")
        .rect(depth + 0.014, width + 0.014)
        .extrude(0.008)
        .edges("|Z")
        .fillet(0.014)
    )
    lip_inner = (
        cq.Workplane("XY")
        .rect(depth - 0.010, width - 0.010)
        .extrude(0.012)
        .edges("|Z")
        .fillet(0.009)
        .translate((0.0, 0.0, -0.002))
    )
    lip = lip_outer.cut(lip_inner).translate((0.0, 0.0, height - 0.002))
    return bucket.union(lip).translate((0.0, 0.0, WALL))


def _superellipse_loop(depth: float, width: float, z: float, *, cx: float, n: float = 3.0, segments: int = 64):
    pts = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = cx + (depth / 2.0) * math.copysign(abs(c) ** (2.0 / n), c)
        y = (width / 2.0) * math.copysign(abs(s) ** (2.0 / n), s)
        pts.append((x, y, z))
    return pts


def _domed_lid_geometry():
    """A shallow rounded-rectangle dome whose local frame is on the rear hinge axis."""
    cx = 0.130
    sections = [
        _superellipse_loop(0.260, 0.194, 0.009, cx=cx, n=3.4),
        _superellipse_loop(0.264, 0.198, 0.016, cx=cx, n=3.2),
        _superellipse_loop(0.246, 0.184, 0.030, cx=cx, n=2.8),
        _superellipse_loop(0.178, 0.132, 0.041, cx=cx, n=2.2),
    ]
    return section_loft(sections, cap=True, solid=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_bathroom_step_bin")

    white = model.material("warm_white_enamel", rgba=(0.91, 0.90, 0.86, 1.0))
    dark = model.material("shadow_gray", rgba=(0.10, 0.105, 0.11, 1.0))
    stainless = model.material("soft_brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    liner_mat = model.material("pale_inner_plastic", rgba=(0.78, 0.80, 0.78, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    wire_mat = model.material("dull_wire_gray", rgba=(0.45, 0.46, 0.45, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=white,
        name="body_floor",
    )
    body.visual(
        Box((WALL, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 + WALL / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=white,
        name="rear_wall",
    )
    for idx, y in enumerate((-BODY_WIDTH / 2.0 + WALL / 2.0, BODY_WIDTH / 2.0 - WALL / 2.0)):
        body.visual(
            Box((BODY_DEPTH, WALL, BODY_HEIGHT)),
            origin=Origin(xyz=(0.0, y, BODY_HEIGHT / 2.0)),
            material=white,
            name=f"side_wall_{idx}",
        )
    body.visual(
        Box((WALL, BODY_WIDTH, 0.030)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 - WALL / 2.0, 0.0, 0.020)),
        material=white,
        name="front_kick",
    )
    for idx, y in enumerate((-0.066, 0.066)):
        body.visual(
            Box((WALL, 0.044, 0.220)),
            origin=Origin(xyz=(BODY_DEPTH / 2.0 - WALL / 2.0, y, 0.142)),
            material=white,
            name=f"front_post_{idx}",
        )
    body.visual(
        Box((0.012, BODY_WIDTH, 0.008)),
        origin=Origin(xyz=(-BODY_DEPTH / 2.0 + 0.006, 0.0, BODY_HEIGHT + 0.003)),
        material=white,
        name="top_rim_rear",
    )
    for idx, y in enumerate((-BODY_WIDTH / 2.0 + 0.006, BODY_WIDTH / 2.0 - 0.006)):
        body.visual(
            Box((BODY_DEPTH, 0.012, 0.008)),
            origin=Origin(xyz=(0.0, y, BODY_HEIGHT + 0.003)),
            material=white,
            name=f"top_rim_side_{idx}",
        )
    for idx, y in enumerate((-0.068, 0.068)):
        body.visual(
            Box((0.012, 0.040, 0.008)),
            origin=Origin(xyz=(BODY_DEPTH / 2.0 - 0.006, y, BODY_HEIGHT + 0.003)),
            material=white,
            name=f"top_rim_front_{idx}",
        )
    body.visual(
        Box((0.004, 0.100, 0.120)),
        origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.001, 0.0, 0.118)),
        material=dark,
        name="front_recess",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.048),
        origin=Origin(xyz=(HINGE_X, -0.063, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.048),
        origin=Origin(xyz=(HINGE_X, 0.063, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_knuckle_1",
    )
    for idx, y in enumerate((-0.063, 0.063)):
        body.visual(
            Box((0.008, 0.052, 0.014)),
            origin=Origin(xyz=(HINGE_X + 0.003, y, HINGE_Z - 0.007)),
            material=stainless,
            name=f"rear_hinge_leaf_{idx}",
        )
    for idx, y in enumerate((-0.072, 0.072)):
        body.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(BODY_DEPTH / 2.0 - 0.006, y, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"pedal_bushing_{idx}",
        )

    inner_bucket = model.part("inner_bucket")
    inner_bucket.visual(
        mesh_from_cadquery(_inner_bucket_shell(), "inner_bucket_shell"),
        material=liner_mat,
        name="bucket_shell",
    )
    for idx, y in enumerate((-0.080, 0.080)):
        inner_bucket.visual(
            Cylinder(radius=0.0055, length=0.007),
            origin=Origin(xyz=(-0.012, y, 0.320), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wire_mat,
            name=f"handle_pivot_{idx}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_domed_lid_geometry(), "gently_domed_lid"),
        material=stainless,
        name="lid_dome",
    )
    lid.visual(
        Cylinder(radius=0.0045, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.020, 0.076, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, 0.006)),
        material=stainless,
        name="lid_hinge_leaf",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Box((0.084, 0.108, 0.012)),
        origin=Origin(xyz=(0.046, 0.0, -0.018)),
        material=stainless,
        name="foot_plate",
    )
    pedal.visual(
        Box((0.074, 0.098, 0.004)),
        origin=Origin(xyz=(0.050, 0.0, -0.011)),
        material=black,
        name="rubber_pad",
    )
    pedal.visual(
        Cylinder(radius=0.0042, length=0.148),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="pedal_axle",
    )
    for idx, y in enumerate((-0.036, 0.036)):
        pedal.visual(
            Box((0.054, 0.007, 0.020)),
            origin=Origin(xyz=(0.024, y, -0.006)),
            material=stainless,
            name=f"pedal_arm_{idx}",
        )
    pedal.visual(
        Box((0.014, 0.018, 0.012)),
        origin=Origin(xyz=(-0.002, 0.0, 0.003)),
        material=stainless,
        name="link_boss",
    )
    pedal.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, 0.0, 0.0), (-0.010, 0.0, 0.050), (-0.006, 0.0, 0.142)],
                radius=0.0027,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
            "pedal_lifter_link",
        ),
        material=stainless,
        name="lifter_link",
    )

    bucket_handle = model.part("bucket_handle")
    bucket_handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, -0.080, 0.0),
                    (-0.010, -0.054, 0.012),
                    (-0.014, 0.0, 0.020),
                    (-0.010, 0.054, 0.012),
                    (0.0, 0.080, 0.0),
                ],
                radius=0.0024,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "inner_bucket_wire_handle",
        ),
        material=wire_mat,
        name="wire_bail",
    )

    model.articulation(
        "body_to_bucket",
        ArticulationType.FIXED,
        parent=body,
        child=inner_bucket,
        origin=Origin(),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(BODY_DEPTH / 2.0 - 0.006, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=0.42),
    )
    model.articulation(
        "bucket_to_handle",
        ArticulationType.REVOLUTE,
        parent=inner_bucket,
        child=bucket_handle,
        origin=Origin(xyz=(-0.012, 0.0, 0.320)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.1, upper=1.1),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    inner_bucket = object_model.get_part("inner_bucket")
    bucket_handle = object_model.get_part("bucket_handle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")
    handle_hinge = object_model.get_articulation("bucket_to_handle")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="lid_dome",
            negative_elem="top_rim_rear",
            name="closed lid sits just above rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_dome",
            min_overlap=0.14,
            name="lid covers narrow body footprint",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_dome")
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_dome")
    ctx.check(
        "lid hinge lifts the lid",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.030,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="foot_plate")
    with ctx.pose({pedal_hinge: 0.35}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="foot_plate")
        ctx.expect_gap(
            pedal,
            body,
            axis="z",
            min_gap=-0.004,
            positive_elem="foot_plate",
            negative_elem="body_floor",
            name="pressed pedal remains above floor shell",
        )
    ctx.check(
        "pedal rotates downward on lower pivot",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.006,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    ctx.expect_within(
        inner_bucket,
        body,
        axes="xy",
        inner_elem="bucket_shell",
        margin=0.004,
        name="inner bucket fits inside slim body",
    )
    ctx.expect_overlap(
        bucket_handle,
        inner_bucket,
        axes="y",
        elem_a="wire_bail",
        elem_b="bucket_shell",
        min_overlap=0.12,
        name="bucket handle spans between side pivots",
    )

    upright_handle_aabb = ctx.part_element_world_aabb(bucket_handle, elem="wire_bail")
    with ctx.pose({handle_hinge: 0.75}):
        swung_handle_aabb = ctx.part_element_world_aabb(bucket_handle, elem="wire_bail")
    ctx.check(
        "inner bucket handle rotates on side pivots",
        upright_handle_aabb is not None
        and swung_handle_aabb is not None
        and abs(swung_handle_aabb[1][0] - upright_handle_aabb[1][0]) > 0.012,
        details=f"upright={upright_handle_aabb}, swung={swung_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
