from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


BIN_RADIUS = 0.160
BIN_HEIGHT = 0.600
HINGE_Y = BIN_RADIUS + 0.012
HINGE_Z = BIN_HEIGHT + 0.046
PEDAL_Y = -BIN_RADIUS - 0.014
PEDAL_Z = 0.082


def _merge(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _body_shell() -> MeshGeometry:
    """Thin open cylindrical stainless shell with a curled top lip."""

    return LatheGeometry.from_shell_profiles(
        [
            (0.145, 0.010),
            (0.159, 0.018),
            (0.160, 0.120),
            (0.160, 0.500),
            (0.158, 0.585),
            (0.164, 0.604),
            (0.162, 0.611),
        ],
        [
            (0.025, 0.018),
            (0.151, 0.030),
            (0.152, 0.500),
            (0.151, 0.575),
            (0.150, 0.596),
            (0.154, 0.604),
        ],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def _inner_bucket_shell() -> MeshGeometry:
    """Removable plastic liner sitting clear inside the stainless can."""

    return LatheGeometry.from_shell_profiles(
        [
            (0.045, 0.040),
            (0.128, 0.048),
            (0.143, 0.110),
            (0.145, 0.500),
            (0.142, 0.562),
            (0.147, 0.574),
        ],
        [
            (0.024, 0.052),
            (0.121, 0.062),
            (0.136, 0.118),
            (0.137, 0.498),
            (0.135, 0.553),
            (0.139, 0.563),
        ],
        segments=80,
        start_cap="round",
        end_cap="round",
        lip_samples=6,
    )


def _lid_mesh() -> MeshGeometry:
    """Shallow domed lid, authored in the lid frame whose origin is on the rear hinge."""

    dome = LatheGeometry(
        [
            (0.000, 0.039),
            (0.045, 0.038),
            (0.095, 0.031),
            (0.135, 0.016),
            (0.156, -0.002),
            (0.157, -0.016),
            (0.147, -0.022),
            (0.055, -0.018),
            (0.000, -0.014),
        ],
        segments=96,
    ).translate(0.0, -0.165, -0.006)

    rear_leaf = CylinderGeometry(radius=0.010, height=0.080, radial_segments=28)
    rear_leaf.rotate_y(math.pi / 2.0).translate(0.0, -0.001, 0.000)
    lid_tab = CylinderGeometry(radius=0.006, height=0.100, radial_segments=24)
    lid_tab.rotate_y(math.pi / 2.0).translate(0.0, -0.038, -0.004)
    bridge = BoxGeometry((0.118, 0.038, 0.007)).translate(0.0, -0.024, -0.004)
    return _merge([dome, bridge, rear_leaf, lid_tab])


def _bucket_handle_mesh() -> MeshGeometry:
    """Small bail handle with molded pivot pins at the two bucket side pivots."""

    wire = tube_from_spline_points(
        [
            (-0.123, 0.000, 0.000),
            (-0.108, -0.035, -0.006),
            (-0.058, -0.078, -0.015),
            (0.000, -0.092, -0.018),
            (0.058, -0.078, -0.015),
            (0.108, -0.035, -0.006),
            (0.123, 0.000, 0.000),
        ],
        radius=0.0032,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    left_pin = CylinderGeometry(radius=0.0055, height=0.022, radial_segments=20)
    left_pin.rotate_y(math.pi / 2.0).translate(-0.123, 0.0, 0.0)
    right_pin = CylinderGeometry(radius=0.0055, height=0.022, radial_segments=20)
    right_pin.rotate_y(math.pi / 2.0).translate(0.123, 0.0, 0.0)
    return _merge([wire, left_pin, right_pin])


def _pedal_mesh() -> MeshGeometry:
    """Narrow front pedal bar and two arms, local origin on the lower front pivot."""

    pedal = MeshGeometry()
    pedal.merge(CylinderGeometry(radius=0.010, height=0.220, radial_segments=24).rotate_y(math.pi / 2.0))
    pedal.merge(CylinderGeometry(radius=0.009, height=0.235, radial_segments=28).rotate_y(math.pi / 2.0).translate(0.0, -0.054, -0.004))
    pedal.merge(CylinderGeometry(radius=0.006, height=0.073, radial_segments=18).rotate_x(math.pi / 2.0).translate(-0.070, -0.028, -0.002))
    pedal.merge(CylinderGeometry(radius=0.006, height=0.073, radial_segments=18).rotate_x(math.pi / 2.0).translate(0.070, -0.028, -0.002))
    return pedal


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_step_bin")

    stainless = model.material("brushed_stainless", rgba=(0.78, 0.80, 0.79, 1.0))
    bright_stainless = model.material("polished_lid", rgba=(0.88, 0.89, 0.86, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    bucket_plastic = model.material("inner_bucket_plastic", rgba=(0.32, 0.34, 0.35, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.020, 0.022, 0.024, 1.0))

    body = model.part("body")
    body.visual(mesh_from_geometry(_body_shell(), "body_shell"), material=stainless, name="outer_shell")
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.158, tube=0.005, radial_segments=16, tubular_segments=96).translate(0.0, 0.0, 0.604), "rolled_rim"),
        material=bright_stainless,
        name="rolled_rim",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.148, tube=0.010, radial_segments=16, tubular_segments=96).translate(0.0, 0.0, 0.018), "rubber_foot_ring"),
        material=black_rubber,
        name="foot_ring",
    )
    body.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.148, tube=0.006, radial_segments=12, tubular_segments=80).translate(0.0, 0.0, 0.522),
            "bucket_support_ring",
        ),
        material=dark_plastic,
        name="bucket_support_ring",
    )
    # Rear hinge band visibly bridges the rim to the horizontal lid pivot.
    body.visual(
        Box((0.180, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, HINGE_Y - 0.004, BIN_HEIGHT + 0.008)),
        material=stainless,
        name="rear_hinge_band",
    )
    body.visual(
        Box((0.036, 0.018, 0.048)),
        origin=Origin(xyz=(-0.070, HINGE_Y - 0.003, BIN_HEIGHT + 0.023)),
        material=stainless,
        name="hinge_lug_0",
    )
    body.visual(
        Box((0.036, 0.018, 0.048)),
        origin=Origin(xyz=(0.070, HINGE_Y - 0.003, BIN_HEIGHT + 0.023)),
        material=stainless,
        name="hinge_lug_1",
    )
    # Lower front pivot brackets are fixed to the shell and flank the moving pedal bar.
    body.visual(
        Box((0.240, 0.014, 0.058)),
        origin=Origin(xyz=(0.0, PEDAL_Y + 0.020, PEDAL_Z)),
        material=stainless,
        name="front_pedal_mount",
    )
    body.visual(
        Box((0.036, 0.034, 0.040)),
        origin=Origin(xyz=(-0.095, PEDAL_Y + 0.003, PEDAL_Z)),
        material=stainless,
        name="pedal_bracket_0",
    )
    body.visual(
        Box((0.036, 0.034, 0.040)),
        origin=Origin(xyz=(0.095, PEDAL_Y + 0.003, PEDAL_Z)),
        material=stainless,
        name="pedal_bracket_1",
    )
    body.visual(
        Box((0.010, 0.006, 0.455)),
        origin=Origin(xyz=(-0.159, -0.006, 0.292)),
        material=Material("soft_vertical_reflection", rgba=(0.96, 0.97, 0.95, 0.62)),
        name="vertical_highlight",
    )

    inner_bucket = model.part("inner_bucket")
    inner_bucket.visual(mesh_from_geometry(_inner_bucket_shell(), "inner_bucket_shell"), material=bucket_plastic, name="bucket_shell")
    inner_bucket.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(-0.133, 0.0, 0.545), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="left_pivot_boss",
    )
    inner_bucket.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.133, 0.0, 0.545), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="right_pivot_boss",
    )
    inner_bucket.visual(
        mesh_from_geometry(TorusGeometry(radius=0.139, tube=0.003, radial_segments=12, tubular_segments=72), "dark_inner_lip"),
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        material=shadow,
        name="dark_inner_lip",
    )

    lid = model.part("lid")
    lid.visual(mesh_from_geometry(_lid_mesh(), "domed_lid"), material=bright_stainless, name="domed_lid")
    lid.visual(
        Cylinder(radius=0.0115, length=0.038),
        origin=Origin(xyz=(-0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="left_hinge_gap",
    )
    lid.visual(
        Cylinder(radius=0.0115, length=0.038),
        origin=Origin(xyz=(0.054, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="right_hinge_gap",
    )

    pedal = model.part("pedal")
    pedal.visual(mesh_from_geometry(_pedal_mesh(), "pedal_bar"), material=black_rubber, name="pedal_bar")

    bucket_handle = model.part("bucket_handle")
    bucket_handle.visual(mesh_from_geometry(_bucket_handle_mesh(), "bucket_handle"), material=dark_plastic, name="handle_wire")

    model.articulation(
        "body_to_inner_bucket",
        ArticulationType.FIXED,
        parent=body,
        child=inner_bucket,
        origin=Origin(),
    )
    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # The closed lid extends forward along local -Y; -X makes positive q open upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=0.0, upper=math.radians(82.0)),
    )
    model.articulation(
        "front_pedal_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, PEDAL_Y, PEDAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=math.radians(-18.0), upper=math.radians(18.0)),
    )
    model.articulation(
        "bucket_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=inner_bucket,
        child=bucket_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=math.radians(-70.0), upper=math.radians(70.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    inner_bucket = object_model.get_part("inner_bucket")
    bucket_handle = object_model.get_part("bucket_handle")

    lid_joint = object_model.get_articulation("rear_lid_hinge")
    pedal_joint = object_model.get_articulation("front_pedal_pivot")
    handle_joint = object_model.get_articulation("bucket_handle_pivot")

    ctx.allow_overlap(
        inner_bucket,
        bucket_handle,
        elem_a="left_pivot_boss",
        elem_b="handle_wire",
        reason="The bucket handle's molded side pin is intentionally captured in the removable liner's pivot boss.",
    )
    ctx.allow_overlap(
        inner_bucket,
        bucket_handle,
        elem_a="right_pivot_boss",
        elem_b="handle_wire",
        reason="The bucket handle's molded side pin is intentionally captured in the removable liner's pivot boss.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_lug_0",
        elem_b="left_hinge_gap",
        reason="The lid hinge barrel is locally captured between the rear hinge lugs.",
    )
    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_lug_1",
        elem_b="right_hinge_gap",
        reason="The lid hinge barrel is locally captured between the rear hinge lugs.",
    )
    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_bracket_0",
        elem_b="pedal_bar",
        reason="The lower pedal pivot shaft is intentionally retained inside the front bracket.",
    )
    ctx.allow_overlap(
        body,
        pedal,
        elem_a="pedal_bracket_1",
        elem_b="pedal_bar",
        reason="The lower pedal pivot shaft is intentionally retained inside the front bracket.",
    )
    ctx.allow_overlap(
        body,
        inner_bucket,
        elem_a="bucket_support_ring",
        elem_b="bucket_shell",
        reason="The removable inner bucket shoulder is seated on the hidden support ring inside the can.",
    )

    ctx.expect_overlap(lid, body, axes="xy", elem_a="domed_lid", elem_b="rolled_rim", min_overlap=0.10, name="lid covers cylindrical rim")
    ctx.expect_gap(lid, body, axis="z", positive_elem="domed_lid", negative_elem="rolled_rim", min_gap=0.001, max_gap=0.040, name="closed lid sits just above rim")
    ctx.expect_overlap(inner_bucket, body, axes="xy", elem_a="bucket_shell", elem_b="bucket_support_ring", min_overlap=0.08, name="inner bucket rests on support ring")
    ctx.expect_overlap(pedal, body, axes="x", elem_a="pedal_bar", elem_b="pedal_bracket_0", min_overlap=0.010, name="pedal spans front pivot brackets")
    ctx.expect_overlap(pedal, body, axes="x", elem_a="pedal_bar", elem_b="pedal_bracket_1", min_overlap=0.010, name="pedal spans both front brackets")
    ctx.expect_overlap(lid, body, axes="x", elem_a="left_hinge_gap", elem_b="hinge_lug_0", min_overlap=0.012, name="left hinge barrel is retained")
    ctx.expect_overlap(lid, body, axes="x", elem_a="right_hinge_gap", elem_b="hinge_lug_1", min_overlap=0.012, name="right hinge barrel is retained")
    ctx.expect_overlap(bucket_handle, inner_bucket, axes="x", elem_a="handle_wire", elem_b="left_pivot_boss", min_overlap=0.004, name="handle left pin engages bucket boss")
    ctx.expect_overlap(bucket_handle, inner_bucket, axes="x", elem_a="handle_wire", elem_b="right_pivot_boss", min_overlap=0.004, name="handle right pin engages bucket boss")

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: math.radians(70.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.expect_gap(lid, inner_bucket, axis="z", min_gap=0.015, positive_elem="domed_lid", negative_elem="bucket_shell", name="opened lid clears inner bucket")

    rest_pedal_aabb = ctx.part_world_aabb(pedal)
    with ctx.pose({pedal_joint: math.radians(14.0)}):
        pressed_pedal_aabb = ctx.part_world_aabb(pedal)

    rest_handle_aabb = ctx.part_world_aabb(bucket_handle)
    with ctx.pose({handle_joint: math.radians(-55.0)}):
        raised_handle_aabb = ctx.part_world_aabb(bucket_handle)

    ctx.check(
        "lid hinge lifts lid upward",
        closed_lid_aabb is not None and open_lid_aabb is not None and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.060,
        details=f"closed_aabb={closed_lid_aabb}, open_aabb={open_lid_aabb}",
    )
    ctx.check(
        "pedal bar presses downward",
        rest_pedal_aabb is not None and pressed_pedal_aabb is not None and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.006,
        details=f"rest_aabb={rest_pedal_aabb}, pressed_aabb={pressed_pedal_aabb}",
    )
    ctx.check(
        "bucket handle swings upward below lid line",
        rest_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > rest_handle_aabb[1][2] + 0.030
        and raised_handle_aabb[1][2] < HINGE_Z,
        details=f"rest_aabb={rest_handle_aabb}, raised_aabb={raised_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
