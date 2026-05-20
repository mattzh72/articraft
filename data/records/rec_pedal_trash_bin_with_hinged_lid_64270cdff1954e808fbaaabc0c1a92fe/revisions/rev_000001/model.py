from __future__ import annotations

from math import pi

import cadquery as cq

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
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_RADIUS = 0.147
LID_RADIUS = 0.151
BODY_HEIGHT = 0.342
HINGE_Y = LID_RADIUS
HINGE_Z = 0.345
PEDAL_PIVOT = (0.0, -0.158, 0.055)
HANDLE_PIVOT_Z = 0.318


def _body_shell_cq():
    """Thin cylindrical metal office-bin shell with a real open cavity and bottom."""
    return (
        cq.Workplane("XY")
        .circle(BODY_RADIUS)
        .extrude(BODY_HEIGHT)
        .faces(">Z")
        .workplane()
        .circle(0.131)
        .cutBlind(-(BODY_HEIGHT - 0.030))
    )


def _inner_bucket_cq():
    """Slightly recessed removable plastic liner visible below the lid line."""
    return (
        cq.Workplane("XY")
        .circle(0.128)
        .extrude(0.300)
        .faces(">Z")
        .workplane()
        .circle(0.110)
        .cutBlind(-0.280)
        .translate((0.0, 0.0, 0.030))
    )


def _rounded_lid_mesh():
    """Low domed circular lid with a downturned skirt, modeled around its center."""
    return LatheGeometry(
        [
            (0.000, -0.002),
            (0.129, -0.002),
            (0.147, 0.001),
            (LID_RADIUS, 0.007),
            (0.148, 0.015),
            (0.137, 0.024),
            (0.105, 0.031),
            (0.050, 0.035),
            (0.000, 0.036),
        ],
        segments=112,
        closed=True,
    )


def _bucket_handle_mesh():
    return tube_from_spline_points(
        [
            (-0.116, 0.000, 0.000),
            (-0.100, -0.056, -0.004),
            (-0.044, -0.104, -0.007),
            (0.044, -0.104, -0.007),
            (0.100, -0.056, -0.004),
            (0.116, 0.000, 0.000),
        ],
        radius=0.003,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cylindrical_office_pedal_bin")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_metal = model.material("dark_hinge_metal", rgba=(0.08, 0.085, 0.085, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    bucket_plastic = model.material("dark_inner_bucket", rgba=(0.07, 0.073, 0.070, 1.0))
    rubber = model.material("pedal_rubber", rgba=(0.005, 0.005, 0.005, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_cq(), "body_shell", tolerance=0.0008, angular_tolerance=0.08),
        material=stainless,
        name="body_shell",
    )
    # Rear yoke: outside, visible, and high enough to carry the lid hinge barrel.
    body.visual(
        Box((0.145, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, 0.150, 0.321)),
        material=dark_metal,
        name="rear_mount_pad",
    )
    body.visual(
        Box((0.018, 0.026, 0.046)),
        origin=Origin(xyz=(-0.064, 0.156, HINGE_Z)),
        material=dark_metal,
        name="rear_yoke_0",
    )
    body.visual(
        Box((0.018, 0.026, 0.046)),
        origin=Origin(xyz=(0.064, 0.156, HINGE_Z)),
        material=dark_metal,
        name="rear_yoke_1",
    )
    # Lower front clevis for the pedal pivot.
    body.visual(
        Box((0.132, 0.010, 0.027)),
        origin=Origin(xyz=(0.0, -0.150, 0.044)),
        material=dark_metal,
        name="front_pivot_pad",
    )
    body.visual(
        Box((0.017, 0.026, 0.036)),
        origin=Origin(xyz=(-0.060, -0.151, PEDAL_PIVOT[2])),
        material=dark_metal,
        name="pedal_yoke_0",
    )
    body.visual(
        Box((0.017, 0.026, 0.036)),
        origin=Origin(xyz=(0.060, -0.151, PEDAL_PIVOT[2])),
        material=dark_metal,
        name="pedal_yoke_1",
    )

    bucket = model.part("inner_bucket")
    bucket.visual(
        mesh_from_cadquery(_inner_bucket_cq(), "inner_bucket_shell", tolerance=0.0008, angular_tolerance=0.08),
        material=bucket_plastic,
        name="bucket_shell",
    )
    bucket.visual(
        Cylinder(radius=0.0075, length=0.016),
        origin=Origin(xyz=(-0.116, 0.0, HANDLE_PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_plastic,
        name="handle_socket_0",
    )
    bucket.visual(
        Cylinder(radius=0.0075, length=0.016),
        origin=Origin(xyz=(0.116, 0.0, HANDLE_PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_plastic,
        name="handle_socket_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_rounded_lid_mesh(), "rounded_lid"),
        origin=Origin(xyz=(0.0, -LID_RADIUS, 0.0)),
        material=stainless,
        name="lid_shell",
    )
    lid.visual(
        Box((0.096, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.003, 0.004)),
        material=dark_metal,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.150),
        origin=Origin(xyz=(0.0, 0.010, 0.003), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )

    pedal = model.part("front_pedal")
    pedal.visual(
        Cylinder(radius=0.006, length=0.136),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    pedal.visual(
        Box((0.125, 0.072, 0.012)),
        origin=Origin(xyz=(0.0, -0.047, -0.018), rpy=(-0.10, 0.0, 0.0)),
        material=dark_metal,
        name="pedal_tab",
    )
    pedal.visual(
        Box((0.074, 0.042, 0.010)),
        origin=Origin(xyz=(0.0, -0.024, -0.006), rpy=(-0.10, 0.0, 0.0)),
        material=dark_metal,
        name="pedal_neck",
    )
    pedal.visual(
        Box((0.102, 0.047, 0.004)),
        origin=Origin(xyz=(0.0, -0.052, -0.010), rpy=(-0.10, 0.0, 0.0)),
        material=rubber,
        name="rubber_pad",
    )

    handle = model.part("bucket_handle")
    handle.visual(
        mesh_from_geometry(_bucket_handle_mesh(), "bucket_handle_wire"),
        material=black_plastic,
        name="handle_wire",
    )

    model.articulation(
        "body_to_bucket",
        ArticulationType.FIXED,
        parent=body,
        child=bucket,
        origin=Origin(),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=PEDAL_PIVOT),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=0.45),
    )
    model.articulation(
        "handle_pivots",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bucket = object_model.get_part("inner_bucket")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("front_pedal")
    handle = object_model.get_part("bucket_handle")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_pivot = object_model.get_articulation("pedal_pivot")
    handle_pivots = object_model.get_articulation("handle_pivots")

    ctx.allow_overlap(
        body,
        bucket,
        elem_a="body_shell",
        elem_b="bucket_shell",
        reason=(
            "The removable liner is intentionally nested inside the visually hollow outer bin; "
            "the exact mesh proxy treats that cavity as occupied."
        ),
    )
    ctx.allow_overlap(
        handle,
        bucket,
        elem_a="handle_wire",
        elem_b="bucket_shell",
        reason=(
            "The bucket handle wire swings within the liner opening while seating at side pivots; "
            "the exact liner proxy occupies the cavity."
        ),
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_yoke_0",
        elem_b="hinge_barrel",
        reason="The lid hinge barrel is intentionally captured inside the rear yoke bracket.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="rear_yoke_1",
        elem_b="hinge_barrel",
        reason="The lid hinge barrel is intentionally captured inside the rear yoke bracket.",
    )
    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_yoke_0",
        elem_b="pivot_barrel",
        reason="The pedal pivot barrel is intentionally captured inside the front clevis.",
    )
    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_yoke_1",
        elem_b="pivot_barrel",
        reason="The pedal pivot barrel is intentionally captured inside the front clevis.",
    )
    ctx.allow_overlap(
        handle,
        bucket,
        elem_a="handle_wire",
        elem_b="handle_socket_0",
        reason="The inner-bucket handle wire is intentionally seated in the molded side socket.",
    )
    ctx.allow_overlap(
        handle,
        bucket,
        elem_a="handle_wire",
        elem_b="handle_socket_1",
        reason="The inner-bucket handle wire is intentionally seated in the molded side socket.",
    )

    ctx.expect_within(
        bucket,
        body,
        axes="xy",
        inner_elem="bucket_shell",
        outer_elem="body_shell",
        margin=0.003,
        name="inner bucket is nested inside the cylindrical shell",
    )
    ctx.expect_overlap(
        bucket,
        body,
        axes="z",
        elem_a="bucket_shell",
        elem_b="body_shell",
        min_overlap=0.25,
        name="inner bucket is deeply seated in the bin shell",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        max_penetration=0.0005,
        max_gap=0.012,
        name="closed rounded lid sits just above the rolled rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xyz",
        elem_a="hinge_barrel",
        elem_b="rear_yoke_0",
        min_overlap=0.006,
        name="rear yoke aligns with the lid hinge barrel",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xyz",
        elem_a="hinge_barrel",
        elem_b="rear_yoke_1",
        min_overlap=0.006,
        name="opposite rear yoke aligns with the lid hinge barrel",
    )
    ctx.expect_overlap(
        pedal,
        body,
        axes="xyz",
        elem_a="pivot_barrel",
        elem_b="pedal_yoke_0",
        min_overlap=0.006,
        name="pedal barrel aligns with the lower front clevis",
    )
    ctx.expect_overlap(
        pedal,
        body,
        axes="xyz",
        elem_a="pivot_barrel",
        elem_b="pedal_yoke_1",
        min_overlap=0.006,
        name="opposite pedal clevis aligns with the pivot barrel",
    )
    ctx.expect_overlap(
        handle,
        bucket,
        axes="xz",
        elem_a="handle_wire",
        elem_b="handle_socket_0",
        min_overlap=0.004,
        name="bucket handle ends align with side sockets",
    )
    ctx.expect_within(
        handle,
        bucket,
        axes="xy",
        inner_elem="handle_wire",
        outer_elem="bucket_shell",
        margin=0.012,
        name="bucket handle wire stays within the liner mouth footprint",
    )
    ctx.expect_overlap(
        handle,
        bucket,
        axes="xz",
        elem_a="handle_wire",
        elem_b="handle_socket_1",
        min_overlap=0.004,
        name="opposite bucket handle end aligns with side socket",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    closed_pedal_aabb = ctx.part_world_aabb(pedal)
    closed_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({pedal_pivot: 0.40}):
        pressed_pedal_aabb = ctx.part_world_aabb(pedal)
    with ctx.pose({handle_pivots: 1.0}):
        raised_handle_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.050,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )
    ctx.check(
        "front pedal tab presses downward about its lower pivot",
        closed_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < closed_pedal_aabb[0][2] - 0.010,
        details=f"closed={closed_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )
    ctx.check(
        "inner-bucket handle swings upward on side pivots",
        closed_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.045,
        details=f"closed={closed_handle_aabb}, raised={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
