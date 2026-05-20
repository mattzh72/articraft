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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_pitch_trunnion")

    zinc = model.material("zinc_plated_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark = model.material("black_oxide_fasteners", rgba=(0.035, 0.035, 0.035, 1.0))
    blue = model.material("blue_anodized_head", rgba=(0.05, 0.18, 0.42, 1.0))
    shaft_mat = model.material("ground_steel_shaft", rgba=(0.78, 0.79, 0.76, 1.0))

    axis_z = 0.310

    frame = model.part("frame")
    frame.visual(
        Box((0.74, 0.58, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=zinc,
        name="base_plate",
    )
    for idx, y in enumerate((-0.205, 0.205)):
        frame.visual(
            Box((0.66, 0.045, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.050)),
            material=zinc,
            name=f"base_rail_{idx}",
        )
    for idx, x in enumerate((-0.300, 0.300)):
        frame.visual(
            Box((0.045, 0.50, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=zinc,
            name=f"cross_tie_{idx}",
        )

    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.0325, tube=0.0110, radial_segments=32, tubular_segments=16),
        "bearing_ring",
    )
    cap_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.046, tube=0.007, radial_segments=32, tubular_segments=12),
        "retainer_cap_ring",
    )

    support_specs = (
        (0, -0.235, "bearing_ring_0"),
        (1, 0.235, "bearing_ring_1"),
    )
    for side, y, bearing_name in support_specs:
        frame.visual(
            Box((0.190, 0.060, 0.045)),
            origin=Origin(xyz=(0.0, y, 0.057)),
            material=zinc,
            name=f"support_foot_{side}",
        )
        for post, x in enumerate((-0.064, 0.064)):
            frame.visual(
                Box((0.042, 0.060, 0.320)),
                origin=Origin(xyz=(x, y, 0.196)),
                material=zinc,
                name=f"support_post_{side}_{post}",
            )
        frame.visual(
            Box((0.170, 0.060, 0.035)),
            origin=Origin(xyz=(0.0, y, axis_z + 0.053)),
            material=zinc,
            name=f"support_cap_{side}",
        )
        frame.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, y, axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=shaft_mat,
            name=bearing_name,
        )

        cap_y = y + (0.033 if y > 0.0 else -0.033)
        frame.visual(
            cap_mesh,
            origin=Origin(xyz=(0.0, cap_y, axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"retainer_cap_{side}",
        )
        bolt_y = y + (0.043 if y > 0.0 else -0.043)
        for bolt, (bx, bz) in enumerate(((-0.035, -0.035), (-0.035, 0.035), (0.035, -0.035), (0.035, 0.035))):
            frame.visual(
                Cylinder(radius=0.0055, length=0.006),
                origin=Origin(
                    xyz=(bx, bolt_y, axis_z + bz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=dark,
                name=f"cap_screw_{side}_{bolt}",
            )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.022, length=0.630),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shaft_mat,
        name="trunnion_shaft",
    )
    head.visual(
        Cylinder(radius=0.052, length=0.300),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="center_hub",
    )
    head.visual(
        Box((0.260, 0.300, 0.125)),
        origin=Origin(xyz=(0.020, 0.0, -0.085)),
        material=blue,
        name="head_body",
    )
    head.visual(
        Box((0.038, 0.320, 0.220)),
        origin=Origin(xyz=(0.165, 0.0, -0.095)),
        material=blue,
        name="tool_face",
    )
    head.visual(
        Box((0.055, 0.240, 0.155)),
        origin=Origin(xyz=(-0.132, 0.0, -0.085)),
        material=blue,
        name="rear_counterweight",
    )
    for side, y in enumerate((-0.312, 0.312)):
        head.visual(
            Cylinder(radius=0.034, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=shaft_mat,
            name=f"shaft_end_cap_{side}",
        )
    for bolt, (by, bz) in enumerate(((-0.105, -0.060), (-0.105, 0.050), (0.105, -0.060), (0.105, 0.050))):
        head.visual(
            Cylinder(radius=0.0075, length=0.007),
            origin=Origin(
                xyz=(0.187, by, bz - 0.095),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark,
            name=f"face_screw_{bolt}",
        )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.60, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    head = object_model.get_part("head")
    pitch = object_model.get_articulation("pitch_axis")

    ctx.allow_overlap(
        frame,
        head,
        elem_a="bearing_ring_0",
        elem_b="trunnion_shaft",
        reason="The shaft is intentionally captured in the simplified bearing bushing with a tiny seated interference.",
    )
    ctx.allow_overlap(
        frame,
        head,
        elem_a="bearing_ring_1",
        elem_b="trunnion_shaft",
        reason="The shaft is intentionally captured in the simplified bearing bushing with a tiny seated interference.",
    )

    ctx.check(
        "single pitch revolute joint",
        len(object_model.articulations) == 1
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and abs(pitch.axis[1]) > 0.99,
        details=f"joints={object_model.articulations}",
    )
    ctx.expect_overlap(
        head,
        frame,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_0",
        min_overlap=0.015,
        name="shaft passes through bearing ring 0",
    )
    ctx.expect_overlap(
        head,
        frame,
        axes="y",
        elem_a="trunnion_shaft",
        elem_b="bearing_ring_1",
        min_overlap=0.015,
        name="shaft passes through bearing ring 1",
    )
    ctx.expect_within(
        head,
        frame,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_ring_0",
        margin=0.0,
        name="shaft centered in bearing ring 0",
    )
    ctx.expect_within(
        head,
        frame,
        axes="xz",
        inner_elem="trunnion_shaft",
        outer_elem="bearing_ring_1",
        margin=0.0,
        name="shaft centered in bearing ring 1",
    )

    rest_face = ctx.part_element_world_aabb(head, elem="tool_face")
    with ctx.pose({pitch: 0.55}):
        tilted_face = ctx.part_element_world_aabb(head, elem="tool_face")
    ctx.check(
        "tool face tilts about carried shaft",
        rest_face is not None
        and tilted_face is not None
        and tilted_face[1][2] > rest_face[1][2] + 0.055,
        details=f"rest={rest_face}, tilted={tilted_face}",
    )

    return ctx.report()


object_model = build_object_model()
