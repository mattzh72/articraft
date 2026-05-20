from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_pad_mesh(name: str, *, length: float, width: float, thickness: float, radius: float):
    profile = rounded_rect_profile(length, width, radius=radius, corner_segments=8)
    return mesh_from_geometry(ExtrudeGeometry.centered(profile, thickness), name)


def _add_xz_member(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    width: float,
    height: float,
    material,
    name: str,
):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    pitch = atan2(-dz, dx)
    part.visual(
        Box((length, width, height)),
        origin=Origin(
            xyz=(
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            ),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_wheel_part(part, *, tire_material, hub_material):
    part.visual(
        Cylinder(radius=0.07, length=0.038),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.041, length=0.042),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.019, length=0.050),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_material,
        name="cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_bench")

    frame_paint = model.material("frame_paint", rgba=(0.14, 0.15, 0.16, 1.0))
    frame_gloss = model.material("frame_gloss", rgba=(0.19, 0.20, 0.22, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.08, 0.08, 0.09, 1.0))
    pad_board = model.material("pad_board", rgba=(0.24, 0.24, 0.25, 1.0))
    roller_poly = model.material("roller_poly", rgba=(0.28, 0.29, 0.30, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.50, 0.52, 0.55, 1.0))

    frame = model.part("frame")

    frame.visual(
        Box((0.10, 0.56, 0.06)),
        origin=Origin(xyz=(-0.42, 0.0, 0.03)),
        material=frame_gloss,
        name="rear_foot",
    )
    frame.visual(
        Box((0.10, 0.56, 0.06)),
        origin=Origin(xyz=(0.44, 0.0, 0.03)),
        material=frame_gloss,
        name="front_foot",
    )
    frame.visual(
        Box((0.06, 0.44, 0.05)),
        origin=Origin(xyz=(-0.28, 0.0, 0.12)),
        material=frame_paint,
        name="support_cross",
    )
    frame.visual(
        Box((0.06, 0.44, 0.05)),
        origin=Origin(xyz=(-0.08, 0.0, 0.34)),
        material=frame_paint,
        name="rear_upper_cross",
    )
    frame.visual(
        Box((0.06, 0.44, 0.05)),
        origin=Origin(xyz=(0.24, 0.0, 0.33)),
        material=frame_paint,
        name="front_upper_cross",
    )
    frame.visual(
        Box((0.08, 0.44, 0.05)),
        origin=Origin(xyz=(0.06, 0.0, 0.09)),
        material=frame_paint,
        name="brace_cross",
    )

    for side_name, side_y in (("pos", 0.21), ("neg", -0.21)):
        _add_xz_member(
            frame,
            start=(-0.40, side_y, 0.06),
            end=(-0.28, side_y, 0.15),
            width=0.05,
            height=0.04,
            material=frame_paint,
            name=f"rear_link_{side_name}",
        )
        _add_xz_member(
            frame,
            start=(-0.28, side_y, 0.15),
            end=(-0.08, side_y, 0.34),
            width=0.05,
            height=0.04,
            material=frame_paint,
            name=f"rear_upright_{side_name}",
        )
        _add_xz_member(
            frame,
            start=(-0.08, side_y, 0.34),
            end=(0.24, side_y, 0.33),
            width=0.05,
            height=0.04,
            material=frame_paint,
            name=f"top_rail_{side_name}",
        )
        _add_xz_member(
            frame,
            start=(0.24, side_y, 0.33),
            end=(0.42, side_y, 0.06),
            width=0.05,
            height=0.04,
            material=frame_paint,
            name=f"front_upright_{side_name}",
        )
        _add_xz_member(
            frame,
            start=(-0.28, side_y, 0.15),
            end=(0.42, side_y, 0.06),
            width=0.05,
            height=0.04,
            material=frame_paint,
            name=f"lower_rail_{side_name}",
        )

        cheek_y = 0.11 if side_y > 0.0 else -0.11

        frame.visual(
            Box((0.05, 0.02, 0.10)),
            origin=Origin(xyz=(-0.08, cheek_y, 0.385)),
            material=frame_gloss,
            name=f"backrest_cheek_{side_name}",
        )
        frame.visual(
            Box((0.05, 0.02, 0.09)),
            origin=Origin(xyz=(0.24, cheek_y, 0.37)),
            material=frame_gloss,
            name=f"seat_cheek_{side_name}",
        )
        frame.visual(
            Box((0.04, 0.02, 0.08)),
            origin=Origin(xyz=(-0.28, cheek_y, 0.16)),
            material=frame_gloss,
            name=f"support_cheek_{side_name}",
        )

    frame.visual(
        Box((0.03, 0.02, 0.05)),
        origin=Origin(xyz=(0.06, 0.055, 0.115)),
        material=frame_gloss,
        name="brace_cheek_0",
    )
    frame.visual(
        Box((0.03, 0.02, 0.05)),
        origin=Origin(xyz=(0.06, -0.055, 0.115)),
        material=frame_gloss,
        name="brace_cheek_1",
    )

    for index, side in enumerate((1.0, -1.0)):
        frame.visual(
            Box((0.04, 0.04, 0.06)),
            origin=Origin(xyz=(-0.49, side * 0.18, 0.07)),
            material=frame_gloss,
            name=f"wheel_arm_{index}",
        )
        frame.visual(
            Box((0.02, 0.008, 0.12)),
            origin=Origin(xyz=(-0.52, side * 0.157, 0.10)),
            material=frame_gloss,
            name=f"wheel_fork_inner_{index}",
        )
        frame.visual(
            Box((0.02, 0.008, 0.12)),
            origin=Origin(xyz=(-0.52, side * 0.203, 0.10)),
            material=frame_gloss,
            name=f"wheel_fork_outer_{index}",
        )
        frame.visual(
            Box((0.02, 0.062, 0.03)),
            origin=Origin(xyz=(-0.52, side * 0.18, 0.155)),
            material=frame_gloss,
            name=f"wheel_bridge_{index}",
        )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.015, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gloss,
        name="barrel",
    )
    backrest.visual(
        Box((0.74, 0.24, 0.02)),
        origin=Origin(xyz=(-0.37, 0.0, 0.02)),
        material=pad_board,
        name="plate",
    )
    backrest.visual(
        _rounded_pad_mesh(
            "backrest_pad",
            length=0.88,
            width=0.31,
            thickness=0.065,
            radius=0.040,
        ),
        origin=Origin(xyz=(-0.44, 0.0, 0.0625)),
        material=pad_vinyl,
        name="pad",
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.015, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gloss,
        name="barrel",
    )
    seat.visual(
        Box((0.30, 0.24, 0.02)),
        origin=Origin(xyz=(-0.13, 0.0, 0.02)),
        material=pad_board,
        name="plate",
    )
    seat.visual(
        _rounded_pad_mesh(
            "seat_pad",
            length=0.34,
            width=0.31,
            thickness=0.070,
            radius=0.035,
        ),
        origin=Origin(xyz=(-0.13, 0.0, 0.065)),
        material=pad_vinyl,
        name="pad",
    )

    support_arm = model.part("support_arm")
    support_arm.visual(
        Cylinder(radius=0.017, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.025), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gloss,
        name="pivot_barrel",
    )
    _add_xz_member(
        support_arm,
        start=(0.0, 0.08, 0.03),
        end=(-0.04, 0.08, 0.24),
        width=0.04,
        height=0.028,
        material=frame_paint,
        name="beam_0",
    )
    _add_xz_member(
        support_arm,
        start=(0.0, -0.08, 0.03),
        end=(-0.04, -0.08, 0.24),
        width=0.04,
        height=0.028,
        material=frame_paint,
        name="beam_1",
    )
    support_arm.visual(
        Box((0.05, 0.12, 0.09)),
        origin=Origin(xyz=(-0.025, 0.0, 0.17)),
        material=frame_gloss,
        name="web",
    )
    support_arm.visual(
        Box((0.06, 0.20, 0.032)),
        origin=Origin(xyz=(-0.05, 0.0, 0.248)),
        material=frame_paint,
        name="top_bar",
    )
    support_arm.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(-0.05, 0.0, 0.260), rpy=(pi / 2.0, 0.0, 0.0)),
        material=roller_poly,
        name="roller",
    )

    brace = model.part("brace")
    brace.visual(
        Cylinder(radius=0.012, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_gloss,
        name="pivot_barrel",
    )
    _add_xz_member(
        brace,
        start=(0.0, 0.0, 0.025),
        end=(-0.19, 0.0, 0.160),
        width=0.08,
        height=0.022,
        material=frame_paint,
        name="link",
    )
    brace.visual(
        Box((0.05, 0.10, 0.03)),
        origin=Origin(xyz=(-0.19, 0.0, 0.160)),
        material=frame_gloss,
        name="head",
    )

    wheel_0 = model.part("wheel_0")
    _add_wheel_part(wheel_0, tire_material=rubber, hub_material=wheel_hub)

    wheel_1 = model.part("wheel_1")
    _add_wheel_part(wheel_1, tire_material=rubber, hub_material=wheel_hub)

    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.08, 0.0, 0.43)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.6, lower=0.0, upper=1.25),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.24, 0.0, 0.41)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.4, lower=0.0, upper=0.34),
    )
    model.articulation(
        "support_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.28, 0.0, 0.15)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.5, lower=0.0, upper=0.75),
    )
    model.articulation(
        "brace_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brace,
        origin=Origin(xyz=(0.06, 0.0, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=0.0, upper=1.05),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_0,
        origin=Origin(xyz=(-0.59, 0.18, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_1,
        origin=Origin(xyz=(-0.59, -0.18, 0.10)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_arm = object_model.get_part("support_arm")
    brace = object_model.get_part("brace")

    backrest_hinge = object_model.get_articulation("backrest_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    support_arm_hinge = object_model.get_articulation("support_arm_hinge")
    brace_hinge = object_model.get_articulation("brace_hinge")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    with ctx.pose(
        {
            backrest_hinge: 0.0,
            seat_hinge: 0.0,
            support_arm_hinge: 0.0,
            brace_hinge: 0.0,
        }
    ):
        ctx.expect_gap(
            seat,
            backrest,
            axis="x",
            positive_elem="pad",
            negative_elem="pad",
            max_gap=0.025,
            max_penetration=0.0,
            name="split pads stay close without overlap",
        )
        ctx.expect_overlap(
            seat,
            backrest,
            axes="y",
            elem_a="pad",
            elem_b="pad",
            min_overlap=0.28,
            name="split pads align across the bench width",
        )
        ctx.expect_gap(
            backrest,
            support_arm,
            axis="z",
            positive_elem="plate",
            negative_elem="roller",
            min_gap=0.006,
            max_gap=0.040,
            name="support arm nests below the backrest plate",
        )

    def max_z(aabb):
        return None if aabb is None else aabb[1][2]

    def center_x(aabb):
        return None if aabb is None else (aabb[0][0] + aabb[1][0]) * 0.5

    backrest_upper = backrest_hinge.motion_limits.upper if backrest_hinge.motion_limits is not None else None
    if backrest_upper is not None:
        rest_backrest = ctx.part_element_world_aabb(backrest, elem="pad")
        with ctx.pose({backrest_hinge: backrest_upper}):
            raised_backrest = ctx.part_element_world_aabb(backrest, elem="pad")
        ctx.check(
            "backrest pitches upward",
            rest_backrest is not None
            and raised_backrest is not None
            and max_z(raised_backrest) > max_z(rest_backrest) + 0.22,
            details=f"rest={rest_backrest}, raised={raised_backrest}",
        )

    seat_upper = seat_hinge.motion_limits.upper if seat_hinge.motion_limits is not None else None
    if seat_upper is not None:
        rest_seat = ctx.part_element_world_aabb(seat, elem="pad")
        with ctx.pose({seat_hinge: seat_upper}):
            raised_seat = ctx.part_element_world_aabb(seat, elem="pad")
        ctx.check(
            "seat front hinge lifts the rear of the seat",
            rest_seat is not None
            and raised_seat is not None
            and max_z(raised_seat) > max_z(rest_seat) + 0.05,
            details=f"rest={rest_seat}, raised={raised_seat}",
        )

    arm_upper = support_arm_hinge.motion_limits.upper if support_arm_hinge.motion_limits is not None else None
    if arm_upper is not None:
        rest_roller = ctx.part_element_world_aabb(support_arm, elem="roller")
        with ctx.pose({support_arm_hinge: arm_upper}):
            raised_roller = ctx.part_element_world_aabb(support_arm, elem="roller")
        ctx.check(
            "support arm swings forward from the lower frame pivot",
            rest_roller is not None
            and raised_roller is not None
            and center_x(raised_roller) > center_x(rest_roller) + 0.12,
            details=f"rest={rest_roller}, raised={raised_roller}",
        )

    brace_upper = brace_hinge.motion_limits.upper if brace_hinge.motion_limits is not None else None
    if brace_upper is not None:
        rest_head = ctx.part_element_world_aabb(brace, elem="head")
        with ctx.pose({brace_hinge: brace_upper}):
            tucked_head = ctx.part_element_world_aabb(brace, elem="head")
            ctx.expect_gap(
                seat,
                brace,
                axis="z",
                positive_elem="plate",
                negative_elem="head",
                min_gap=0.03,
                name="tucked brace stays below the seat structure",
            )
        ctx.check(
            "brace tucks forward under the bench",
            rest_head is not None
            and tucked_head is not None
            and center_x(tucked_head) > center_x(rest_head) + 0.15,
            details=f"rest={rest_head}, tucked={tucked_head}",
        )

    for name, joint in (("wheel_0_spin", wheel_0_spin), ("wheel_1_spin", wheel_1_spin)):
        limits = joint.motion_limits
        ctx.check(
            f"{name} stays continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    ctx.expect_overlap(
        "wheel_0",
        frame,
        axes="z",
        elem_a="tire",
        elem_b="rear_foot",
        min_overlap=0.02,
        name="transport wheel height lines up with the rear foot",
    )

    return ctx.report()


object_model = build_object_model()
