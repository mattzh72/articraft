from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


FRAME_HALF_WIDTH = 0.22
PAD_WIDTH = 0.28
PAD_THICKNESS = 0.065


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_weight_bench")

    frame_paint = model.material("frame_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    vinyl = model.material("vinyl", rgba=(0.09, 0.09, 0.10, 1.0))
    vinyl_side = model.material("vinyl_side", rgba=(0.15, 0.15, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.07, 0.07, 0.07, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.38, 0.40, 0.43, 1.0))

    frame = model.part("frame")
    left_rail = wire_from_points(
        [
            (0.54, FRAME_HALF_WIDTH, 0.06),
            (0.42, FRAME_HALF_WIDTH, 0.06),
            (0.27, FRAME_HALF_WIDTH, 0.18),
            (0.10, FRAME_HALF_WIDTH, 0.28),
            (-0.10, FRAME_HALF_WIDTH, 0.28),
            (-0.36, FRAME_HALF_WIDTH, 0.22),
            (-0.56, FRAME_HALF_WIDTH, 0.06),
        ],
        radius=0.022,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.065,
        corner_segments=10,
        radial_segments=18,
    )
    right_rail = wire_from_points(
        _mirror_y(
            [
                (0.54, FRAME_HALF_WIDTH, 0.06),
                (0.42, FRAME_HALF_WIDTH, 0.06),
                (0.27, FRAME_HALF_WIDTH, 0.18),
                (0.10, FRAME_HALF_WIDTH, 0.28),
                (-0.10, FRAME_HALF_WIDTH, 0.28),
                (-0.36, FRAME_HALF_WIDTH, 0.22),
                (-0.56, FRAME_HALF_WIDTH, 0.06),
            ]
        ),
        radius=0.022,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.065,
        corner_segments=10,
        radial_segments=18,
    )
    frame.visual(_mesh("frame_left_rail", left_rail), material=frame_paint, name="left_rail")
    frame.visual(_mesh("frame_right_rail", right_rail), material=frame_paint, name="right_rail")
    frame.visual(
        Cylinder(radius=0.020, length=0.44),
        origin=Origin(xyz=(0.48, 0.0, 0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="front_axle_bar",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.48),
        origin=Origin(xyz=(0.10, 0.0, 0.28), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="center_cross",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.48),
        origin=Origin(xyz=(-0.28, 0.0, 0.22), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="rear_cross",
    )
    frame.visual(
        Cylinder(radius=0.022, length=0.56),
        origin=Origin(xyz=(-0.58, 0.0, 0.06), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="rear_foot",
    )
    frame.visual(
        Box((0.34, 0.10, 0.10)),
        origin=Origin(xyz=(0.08, 0.0, 0.28)),
        material=frame_paint,
        name="seat_bridge",
    )
    frame.visual(
        Box((0.32, 0.06, 0.09)),
        origin=Origin(xyz=(-0.22, 0.0, 0.21)),
        material=frame_paint,
        name="rear_bridge",
    )
    frame.visual(
        Box((0.14, 0.18, 0.05)),
        origin=Origin(xyz=(0.16, 0.13, 0.27)),
        material=frame_paint,
        name="seat_connector_0",
    )
    frame.visual(
        Box((0.14, 0.18, 0.05)),
        origin=Origin(xyz=(0.16, -0.13, 0.27)),
        material=frame_paint,
        name="seat_connector_1",
    )
    for x_pos, stem in ((0.00, "back"), (0.08, "seat")):
        for index, y_pos in enumerate((-0.11, 0.11)):
            frame.visual(
                Box((0.030, 0.040, 0.070)),
                origin=Origin(xyz=(x_pos, y_pos, 0.300)),
                material=steel,
                name=f"{stem}_tab_{index}",
            )
    frame.visual(
        Box((0.050, 0.240, 0.030)),
        origin=Origin(xyz=(0.00, 0.0, 0.315)),
        material=steel,
        name="back_saddle",
    )
    frame.visual(
        Box((0.050, 0.240, 0.030)),
        origin=Origin(xyz=(0.08, 0.0, 0.315)),
        material=steel,
        name="seat_saddle",
    )
    for index, y_pos in enumerate((-0.095, 0.095)):
        frame.visual(
            Box((0.030, 0.030, 0.100)),
            origin=Origin(xyz=(-0.38, y_pos, 0.170)),
            material=steel,
            name=f"support_tab_{index}",
        )
        frame.visual(
            Box((0.030, 0.030, 0.120)),
            origin=Origin(xyz=(0.10, y_pos, 0.180)),
            material=steel,
            name=f"brace_tab_{index}",
        )
    frame.visual(
        Box((0.050, 0.220, 0.030)),
        origin=Origin(xyz=(-0.38, 0.0, 0.225)),
        material=steel,
        name="support_saddle",
    )
    frame.visual(
        Box((0.050, 0.220, 0.030)),
        origin=Origin(xyz=(0.10, 0.0, 0.235)),
        material=steel,
        name="brace_saddle",
    )
    frame.visual(
        Box((0.060, 0.070, 0.080)),
        origin=Origin(xyz=(0.46, 0.16, 0.110)),
        material=frame_paint,
        name="front_upright_0",
    )
    frame.visual(
        Box((0.060, 0.070, 0.080)),
        origin=Origin(xyz=(0.46, -0.16, 0.110)),
        material=frame_paint,
        name="front_upright_1",
    )
    frame.visual(
        Box((0.060, 0.140, 0.060)),
        origin=Origin(xyz=(0.46, 0.214, 0.090)),
        material=steel,
        name="wheel_arm_0",
    )
    frame.visual(
        Box((0.060, 0.140, 0.060)),
        origin=Origin(xyz=(0.46, -0.214, 0.090)),
        material=steel,
        name="wheel_arm_1",
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.78, PAD_WIDTH, PAD_THICKNESS)),
        origin=Origin(xyz=(-0.35, 0.0, 0.062)),
        material=vinyl,
        name="back_pad",
    )
    backrest.visual(
        Box((0.76, PAD_WIDTH - 0.02, 0.020)),
        origin=Origin(xyz=(-0.35, 0.0, 0.020)),
        material=vinyl_side,
        name="back_base",
    )
    backrest.visual(
        Box((0.50, 0.038, 0.024)),
        origin=Origin(xyz=(-0.28, 0.080, 0.020)),
        material=steel,
        name="back_bracket_0",
    )
    backrest.visual(
        Box((0.50, 0.038, 0.024)),
        origin=Origin(xyz=(-0.28, -0.080, 0.020)),
        material=steel,
        name="back_bracket_1",
    )
    backrest.visual(
        Box((0.12, 0.18, 0.022)),
        origin=Origin(xyz=(-0.02, 0.0, 0.018)),
        material=steel,
        name="back_hinge_plate",
    )
    backrest.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="back_barrel",
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.32, PAD_WIDTH, PAD_THICKNESS)),
        origin=Origin(xyz=(0.14, 0.0, 0.062)),
        material=vinyl,
        name="seat_pad",
    )
    seat.visual(
        Box((0.30, PAD_WIDTH - 0.02, 0.020)),
        origin=Origin(xyz=(0.14, 0.0, 0.020)),
        material=vinyl_side,
        name="seat_base",
    )
    seat.visual(
        Box((0.20, 0.038, 0.024)),
        origin=Origin(xyz=(0.12, 0.080, 0.020)),
        material=steel,
        name="seat_bracket_0",
    )
    seat.visual(
        Box((0.20, 0.038, 0.024)),
        origin=Origin(xyz=(0.12, -0.080, 0.020)),
        material=steel,
        name="seat_bracket_1",
    )
    seat.visual(
        Box((0.10, 0.18, 0.022)),
        origin=Origin(xyz=(0.02, 0.0, 0.018)),
        material=steel,
        name="seat_hinge_plate",
    )
    seat.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="seat_barrel",
    )

    rear_link = model.part("rear_link")
    rear_link.visual(
        Cylinder(radius=0.012, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_tube",
    )
    rear_link.visual(
        _mesh(
            "rear_link_side_0",
            wire_from_points(
                [(0.0, 0.055, 0.0), (0.10, 0.055, 0.09), (0.22, 0.055, 0.22)],
                radius=0.011,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=8,
                radial_segments=16,
            ),
        ),
        material=steel,
        name="link_side_0",
    )
    rear_link.visual(
        _mesh(
            "rear_link_side_1",
            wire_from_points(
                [(0.0, -0.055, 0.0), (0.10, -0.055, 0.09), (0.22, -0.055, 0.22)],
                radius=0.011,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.040,
                corner_segments=8,
                radial_segments=16,
            ),
        ),
        material=steel,
        name="link_side_1",
    )
    rear_link.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.22, 0.0, 0.20), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="link_head",
    )
    rear_link.visual(
        Box((0.060, 0.110, 0.018)),
        origin=Origin(xyz=(0.22, 0.0, 0.165)),
        material=satin_black,
        name="link_pad",
    )

    brace = model.part("brace")
    brace.visual(
        Cylinder(radius=0.011, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="brace_pivot",
    )
    brace.visual(
        _mesh(
            "brace_side_0",
            wire_from_points(
                [(0.0, 0.065, 0.0), (-0.11, 0.065, -0.04), (-0.23, 0.065, -0.08)],
                radius=0.010,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.035,
                corner_segments=8,
                radial_segments=16,
            ),
        ),
        material=steel,
        name="brace_side_0",
    )
    brace.visual(
        _mesh(
            "brace_side_1",
            wire_from_points(
                [(0.0, -0.065, 0.0), (-0.11, -0.065, -0.04), (-0.23, -0.065, -0.08)],
                radius=0.010,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.035,
                corner_segments=8,
                radial_segments=16,
            ),
        ),
        material=steel,
        name="brace_side_1",
    )
    brace.visual(
        Cylinder(radius=0.011, length=0.16),
        origin=Origin(xyz=(-0.23, 0.0, -0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="brace_tail",
    )
    for index, y_pos in enumerate((-0.30, 0.30)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.055, length=0.028),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.032, length=0.032),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_gray,
            name="hub",
        )
        wheel.visual(
            Box((0.016, 0.010, 0.020)),
            origin=Origin(xyz=(0.040, 0.0, 0.0)),
            material=wheel_gray,
            name="marker",
        )
        model.articulation(
            f"wheel_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.48, y_pos, 0.08)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
        )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-0.15, upper=1.25),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.08, 0.0, 0.335)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6, lower=-0.10, upper=0.42),
    )
    model.articulation(
        "support_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_link,
        origin=Origin(xyz=(-0.38, 0.0, 0.12)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.70, upper=0.50),
    )
    model.articulation(
        "brace_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brace,
        origin=Origin(xyz=(0.10, 0.0, 0.14)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.20),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    rear_link = object_model.get_part("rear_link")
    brace = object_model.get_part("brace")
    wheel_0 = object_model.get_part("wheel_0")

    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    support_pivot = object_model.get_articulation("support_pivot")
    brace_pivot = object_model.get_articulation("brace_pivot")
    wheel_spin = object_model.get_articulation("wheel_0_spin")

    ctx.expect_gap(
        seat,
        backrest,
        axis="x",
        positive_elem="seat_pad",
        negative_elem="back_pad",
        min_gap=0.010,
        max_gap=0.035,
        name="split pads keep a narrow center gap",
    )
    ctx.expect_overlap(
        seat,
        backrest,
        axes="y",
        elem_a="seat_pad",
        elem_b="back_pad",
        min_overlap=0.24,
        name="seat and backrest stay aligned across the bench width",
    )
    ctx.expect_overlap(
        backrest,
        frame,
        axes="y",
        elem_a="back_pad",
        min_overlap=0.20,
        name="backrest remains centered over the frame width",
    )
    ctx.expect_overlap(
        seat,
        frame,
        axes="y",
        elem_a="seat_pad",
        min_overlap=0.20,
        name="seat remains centered over the frame width",
    )

    back_rest = _aabb_center(ctx.part_element_world_aabb(backrest, elem="back_pad"))
    if back_hinge.motion_limits is not None and back_hinge.motion_limits.upper is not None:
        with ctx.pose({back_hinge: back_hinge.motion_limits.upper}):
            back_raised = _aabb_center(ctx.part_element_world_aabb(backrest, elem="back_pad"))
        ctx.check(
            "backrest raises around its hinge",
            back_rest is not None
            and back_raised is not None
            and back_raised[2] > back_rest[2] + 0.18,
            details=f"rest={back_rest}, raised={back_raised}",
        )

    seat_rest = _aabb_center(ctx.part_element_world_aabb(seat, elem="seat_pad"))
    if seat_hinge.motion_limits is not None and seat_hinge.motion_limits.upper is not None:
        with ctx.pose({seat_hinge: seat_hinge.motion_limits.upper}):
            seat_raised = _aabb_center(ctx.part_element_world_aabb(seat, elem="seat_pad"))
        ctx.check(
            "seat tilts upward at the front hinge",
            seat_rest is not None
            and seat_raised is not None
            and seat_raised[2] > seat_rest[2] + 0.03,
            details=f"rest={seat_rest}, raised={seat_raised}",
        )

    link_rest = _aabb_center(ctx.part_element_world_aabb(rear_link, elem="link_head"))
    if support_pivot.motion_limits is not None and support_pivot.motion_limits.upper is not None:
        with ctx.pose({support_pivot: support_pivot.motion_limits.upper}):
            link_raised = _aabb_center(ctx.part_element_world_aabb(rear_link, elem="link_head"))
        ctx.check(
            "rear support link rotates upward from its lower pivot",
            link_rest is not None
            and link_raised is not None
            and link_raised[2] > link_rest[2] + 0.02,
            details=f"rest={link_rest}, raised={link_raised}",
        )

    brace_rest = _aabb_center(ctx.part_element_world_aabb(brace, elem="brace_tail"))
    if brace_pivot.motion_limits is not None and brace_pivot.motion_limits.upper is not None:
        with ctx.pose({brace_pivot: brace_pivot.motion_limits.upper}):
            brace_tucked = _aabb_center(ctx.part_element_world_aabb(brace, elem="brace_tail"))
        ctx.check(
            "mid-frame brace tucks upward under the bench",
            brace_rest is not None
            and brace_tucked is not None
            and brace_tucked[2] > brace_rest[2] + 0.12,
            details=f"rest={brace_rest}, tucked={brace_tucked}",
        )

    marker_rest = _aabb_center(ctx.part_element_world_aabb(wheel_0, elem="marker"))
    with ctx.pose({wheel_spin: 1.0}):
        marker_spun = _aabb_center(ctx.part_element_world_aabb(wheel_0, elem="marker"))
    ctx.check(
        "front transport wheel spins on its axle",
        marker_rest is not None
        and marker_spun is not None
        and (
            abs(marker_spun[0] - marker_rest[0]) > 0.010
            or abs(marker_spun[2] - marker_rest[2]) > 0.010
        ),
        details=f"rest={marker_rest}, spun={marker_spun}",
    )

    return ctx.report()


object_model = build_object_model()
