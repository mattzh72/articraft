from __future__ import annotations

import math

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
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_trunnion_pitch_module")

    dark_cast = Material("dark_cast_iron", rgba=(0.08, 0.085, 0.09, 1.0))
    graphite = Material("graphite_black", rgba=(0.025, 0.027, 0.03, 1.0))
    parkerized = Material("parkerized_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    worn_edge = Material("worn_bearing_edge", rgba=(0.42, 0.43, 0.40, 1.0))
    face_paint = Material("warm_grey_carried_face", rgba=(0.55, 0.56, 0.53, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.48, 0.34, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_cast,
        name="heavy_foot",
    )
    support.visual(
        Box((0.22, 0.18, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=parkerized,
        name="pedestal_plinth",
    )
    support.visual(
        Box((0.135, 0.135, 0.290)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_cast,
        name="short_pedestal",
    )
    support.visual(
        Box((0.23, 0.17, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.3925)),
        material=dark_cast,
        name="axis_saddle",
    )

    # One-piece cheeked yoke with real trunnion bores and an open center span.
    yoke = TrunnionYokeGeometry(
        (0.300, 0.160, 0.220),
        span_width=0.160,
        trunnion_diameter=0.082,
        trunnion_center_z=0.140,
        base_thickness=0.045,
        corner_radius=0.008,
        center=False,
    )
    support.visual(
        mesh_from_geometry(yoke, "trunnion_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        material=dark_cast,
        name="side_cheeks",
    )

    bearing_ring = TorusGeometry(
        radius=0.0505,
        tube=0.010,
        radial_segments=18,
        tubular_segments=36,
    ).rotate_y(math.pi / 2.0)
    for index, x in enumerate((-0.154, 0.154)):
        support.visual(
            mesh_from_geometry(bearing_ring, f"bearing_ring_{index}"),
            origin=Origin(xyz=(x, 0.0, 0.540)),
            material=worn_edge,
            name=f"bearing_ring_{index}",
        )

    for index, (x, y) in enumerate(
        ((-0.190, -0.120), (-0.190, 0.120), (0.190, -0.120), (0.190, 0.120))
    ):
        support.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x, y, 0.085)),
            material=graphite,
            name=f"hold_down_bolt_{index}",
        )

    tilt_face = model.part("tilt_face")
    tilt_face.visual(
        Cylinder(radius=0.0405, length=0.320),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edge,
        name="trunnion_pin",
    )
    tilt_face.visual(
        Cylinder(radius=0.052, length=0.100),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=parkerized,
        name="central_barrel",
    )
    for index, x in enumerate((-0.073, 0.073)):
        tilt_face.visual(
            Cylinder(radius=0.046, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=parkerized,
            name=f"inner_shoulder_{index}",
        )
    tilt_face.visual(
        Box((0.078, 0.155, 0.060)),
        origin=Origin(xyz=(0.0, 0.086, 0.0)),
        material=parkerized,
        name="face_neck",
    )
    tilt_face.visual(
        Box((0.150, 0.026, 0.165)),
        origin=Origin(xyz=(0.0, 0.170, 0.0)),
        material=face_paint,
        name="carried_face",
    )
    tilt_face.visual(
        Box((0.124, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.184, 0.055)),
        material=graphite,
        name="upper_grip_slot",
    )
    tilt_face.visual(
        Box((0.124, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.184, -0.055)),
        material=graphite,
        name="lower_grip_slot",
    )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=tilt_face,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    tilt_face = object_model.get_part("tilt_face")
    pitch_axis = object_model.get_articulation("pitch_axis")

    for ring_name in ("bearing_ring_0", "bearing_ring_1"):
        ctx.allow_overlap(
            support,
            tilt_face,
            elem_a=ring_name,
            elem_b="trunnion_pin",
            reason="The trunnion pin is intentionally captured in the fixed bearing race.",
        )
        ctx.expect_overlap(
            tilt_face,
            support,
            axes="x",
            elem_a="trunnion_pin",
            elem_b=ring_name,
            min_overlap=0.010,
            name=f"{ring_name} captures trunnion pin end",
        )

    ctx.expect_within(
        tilt_face,
        support,
        axes="x",
        inner_elem="central_barrel",
        outer_elem="side_cheeks",
        margin=0.002,
        name="moving barrel is captured between cheeks",
    )
    ctx.expect_gap(
        tilt_face,
        support,
        axis="y",
        positive_elem="carried_face",
        negative_elem="side_cheeks",
        min_gap=0.020,
        name="carried face sits proud of cheek package",
    )

    rest_aabb = ctx.part_element_world_aabb(tilt_face, elem="carried_face")
    with ctx.pose({pitch_axis: 0.45}):
        pitched_aabb = ctx.part_element_world_aabb(tilt_face, elem="carried_face")
        ctx.expect_gap(
            tilt_face,
            support,
            axis="y",
            positive_elem="carried_face",
            negative_elem="side_cheeks",
            min_gap=0.010,
            name="pitched face clears front of cheeks",
        )

    ctx.check(
        "positive pitch raises carried face",
        rest_aabb is not None
        and pitched_aabb is not None
        and pitched_aabb[0][2] > rest_aabb[0][2] + 0.015,
        details=f"rest={rest_aabb}, pitched={pitched_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
