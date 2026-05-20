from __future__ import annotations

from math import pi

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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _pad_mesh(length: float, width: float, thickness: float, radius: float, name: str):
    return _mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(length, width, radius, corner_segments=8),
            thickness,
            cap=True,
            center=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apartment_gym_bench")

    frame_paint = model.material("frame_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    pad_vinyl = model.material("pad_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    board_dark = model.material("board_dark", rgba=(0.16, 0.13, 0.11, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    wheel_core = model.material("wheel_core", rgba=(0.56, 0.58, 0.62, 1.0))
    hardware = model.material("hardware", rgba=(0.62, 0.64, 0.67, 1.0))

    frame = model.part("frame")
    frame.visual(Box((0.88, 0.05, 0.08)), origin=Origin(xyz=(-0.06, 0.0, 0.29)), material=frame_paint, name="spine")
    frame.visual(Box((0.06, 0.05, 0.31)), origin=Origin(xyz=(-0.46, 0.0, 0.155)), material=frame_paint, name="rear_upright")
    frame.visual(Box((0.07, 0.05, 0.31)), origin=Origin(xyz=(0.18, 0.0, 0.155)), material=frame_paint, name="front_upright")
    frame.visual(Box((0.10, 0.34, 0.04)), origin=Origin(xyz=(-0.46, 0.0, 0.02)), material=frame_paint, name="rear_foot")
    frame.visual(Box((0.10, 0.30, 0.05)), origin=Origin(xyz=(0.41, 0.0, 0.08)), material=frame_paint, name="front_crossmember")
    frame.visual(Box((0.28, 0.04, 0.04)), origin=Origin(xyz=(0.31, 0.0, 0.10)), material=frame_paint, name="front_lower_rail")
    frame.visual(Box((0.18, 0.04, 0.04)), origin=Origin(xyz=(-0.38, 0.0, 0.10)), material=frame_paint, name="rear_lower_rail")
    frame.visual(Box((0.32, 0.10, 0.05)), origin=Origin(xyz=(0.07, 0.0, 0.325)), material=frame_paint, name="seat_bridge")
    frame.visual(Box((0.28, 0.09, 0.05)), origin=Origin(xyz=(-0.18, 0.0, 0.325)), material=frame_paint, name="back_bridge")

    for y in (-0.155, 0.155):
        frame.visual(Box((0.014, 0.02, 0.08)), origin=Origin(xyz=(0.239, y, 0.345)), material=frame_paint)
        frame.visual(Box((0.014, 0.02, 0.08)), origin=Origin(xyz=(-0.098, y, 0.345)), material=frame_paint)
    for y in (-0.095, 0.095):
        frame.visual(Box((0.025, 0.025, 0.10)), origin=Origin(xyz=(-0.43, y * 0.85, 0.105)), material=frame_paint)
        frame.visual(Box((0.04, 0.028, 0.06)), origin=Origin(xyz=(0.43, y * 1.38, 0.08)), material=frame_paint)

    frame.visual(
        Cylinder(radius=0.013, length=0.34),
        origin=Origin(xyz=(0.23, 0.0, 0.345), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="seat_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.34),
        origin=Origin(xyz=(-0.08, 0.0, 0.345), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="back_hinge_barrel",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.22),
        origin=Origin(xyz=(-0.43, 0.0, 0.105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="ladder_pivot_barrel",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.43, 0.145, 0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="wheel_stub_0",
    )
    frame.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.43, -0.145, 0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="wheel_stub_1",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.12),
        origin=Origin(xyz=(0.46, 0.045, 0.08), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.12),
        origin=Origin(xyz=(0.46, -0.045, 0.08), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_paint,
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.52, 0.0, 0.08), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_paint,
        name="front_handle",
    )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        _pad_mesh(0.31, 0.28, 0.05, 0.035, "seat_pad_cushion"),
        origin=Origin(xyz=(-0.163, 0.0, 0.038)),
        material=pad_vinyl,
        name="cushion",
    )
    seat_pad.visual(Box((0.26, 0.22, 0.01)), origin=Origin(xyz=(-0.155, 0.0, 0.010)), material=board_dark, name="board")
    seat_pad.visual(Box((0.05, 0.04, 0.016)), origin=Origin(xyz=(-0.040, 0.075, 0.011)), material=board_dark, name="hinge_tab_0")
    seat_pad.visual(Box((0.05, 0.04, 0.016)), origin=Origin(xyz=(-0.040, -0.075, 0.011)), material=board_dark, name="hinge_tab_1")
    seat_pad.visual(Box((0.06, 0.12, 0.016)), origin=Origin(xyz=(-0.23, 0.0, 0.012)), material=board_dark, name="rear_mount")

    back_pad = model.part("back_pad")
    back_pad.visual(
        _pad_mesh(0.71, 0.28, 0.055, 0.035, "back_pad_cushion"),
        origin=Origin(xyz=(-0.365, 0.0, 0.041)),
        material=pad_vinyl,
        name="cushion",
    )
    back_pad.visual(Box((0.64, 0.22, 0.01)), origin=Origin(xyz=(-0.355, 0.0, 0.006)), material=board_dark, name="board")
    back_pad.visual(Box((0.05, 0.04, 0.018)), origin=Origin(xyz=(-0.055, 0.075, 0.009)), material=board_dark, name="hinge_tab_0")
    back_pad.visual(Box((0.05, 0.04, 0.018)), origin=Origin(xyz=(-0.055, -0.075, 0.009)), material=board_dark, name="hinge_tab_1")
    back_pad.visual(
        Box((0.09, 0.18, 0.02)),
        origin=Origin(xyz=(-0.26, 0.0, 0.014)),
        material=hardware,
        name="support_catch",
    )
    back_pad.visual(Box((0.20, 0.16, 0.014)), origin=Origin(xyz=(-0.30, 0.0, 0.016)), material=board_dark, name="underbrace")

    ladder = model.part("ladder")
    ladder.visual(Box((0.384, 0.018, 0.034)), origin=Origin(xyz=(0.208, 0.075, 0.0)), material=frame_paint, name="rail_0")
    ladder.visual(Box((0.384, 0.018, 0.034)), origin=Origin(xyz=(0.208, -0.075, 0.0)), material=frame_paint, name="rail_1")
    ladder.visual(
        Box((0.03, 0.18, 0.07)),
        origin=Origin(xyz=(0.385, 0.0, 0.020)),
        material=frame_paint,
        name="top_stop",
    )
    for index, x in enumerate((0.16, 0.27, 0.36)):
        ladder.visual(
            Cylinder(radius=0.012, length=0.15),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"rung_{index}",
        )

    for wheel_name in ("wheel_0", "wheel_1"):
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.040, length=0.026),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.028, length=0.020),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_core,
            name="hub",
        )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_pad,
        origin=Origin(xyz=(0.23, 0.0, 0.345)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4, lower=0.0, upper=0.42),
    )
    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(-0.08, 0.0, 0.345)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.3, lower=0.0, upper=1.32),
    )
    model.articulation(
        "ladder_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=ladder,
        origin=Origin(xyz=(-0.43, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=1.16),
    )
    model.articulation(
        "wheel_spin_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=model.get_part("wheel_0"),
        origin=Origin(xyz=(0.43, 0.165, 0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "wheel_spin_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=model.get_part("wheel_1"),
        origin=Origin(xyz=(0.43, -0.165, 0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat_pad = object_model.get_part("seat_pad")
    back_pad = object_model.get_part("back_pad")
    ladder = object_model.get_part("ladder")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    seat_hinge = object_model.get_articulation("seat_hinge")
    back_hinge = object_model.get_articulation("back_hinge")
    ladder_pivot = object_model.get_articulation("ladder_pivot")
    wheel_spin_0 = object_model.get_articulation("wheel_spin_0")
    wheel_spin_1 = object_model.get_articulation("wheel_spin_1")

    def _max_z(aabb):
        return None if aabb is None else aabb[1][2]

    handle_aabb = ctx.part_element_world_aabb(frame, elem="front_handle")
    cross_aabb = ctx.part_element_world_aabb(frame, elem="front_crossmember")
    wheel_0_pos = ctx.part_world_position(wheel_0)
    wheel_1_pos = ctx.part_world_position(wheel_1)
    handle_center_y = None if handle_aabb is None else 0.5 * (handle_aabb[0][1] + handle_aabb[1][1])
    handle_min_x = None if handle_aabb is None else handle_aabb[0][0]
    cross_max_x = None if cross_aabb is None else cross_aabb[1][0]
    ctx.check(
        "front handle projects ahead and stays between wheels",
        handle_center_y is not None
        and handle_min_x is not None
        and cross_max_x is not None
        and wheel_0_pos is not None
        and wheel_1_pos is not None
        and handle_min_x > cross_max_x + 0.035
        and wheel_0_pos[1] > 0.0
        and wheel_1_pos[1] < 0.0
        and wheel_1_pos[1] < handle_center_y < wheel_0_pos[1],
        details=(
            f"handle_aabb={handle_aabb}, cross_aabb={cross_aabb}, "
            f"wheel_0_pos={wheel_0_pos}, wheel_1_pos={wheel_1_pos}"
        ),
    )

    seat_rest_aabb = ctx.part_world_aabb(seat_pad)
    back_rest_aabb = ctx.part_world_aabb(back_pad)
    ladder_rest_aabb = ctx.part_world_aabb(ladder)

    seat_upper = None if seat_hinge.motion_limits is None else seat_hinge.motion_limits.upper
    if seat_upper is not None:
        with ctx.pose({seat_hinge: seat_upper}):
            seat_open_aabb = ctx.part_world_aabb(seat_pad)
        ctx.check(
            "seat pad tilts upward",
            _max_z(seat_rest_aabb) is not None
            and _max_z(seat_open_aabb) is not None
            and _max_z(seat_open_aabb) > _max_z(seat_rest_aabb) + 0.10,
            details=f"rest={seat_rest_aabb}, open={seat_open_aabb}",
        )

    back_upper = None if back_hinge.motion_limits is None else back_hinge.motion_limits.upper
    if back_upper is not None:
        with ctx.pose({back_hinge: back_upper}):
            back_open_aabb = ctx.part_world_aabb(back_pad)
        ctx.check(
            "back pad lifts to an incline range",
            _max_z(back_rest_aabb) is not None
            and _max_z(back_open_aabb) is not None
            and _max_z(back_open_aabb) > _max_z(back_rest_aabb) + 0.45,
            details=f"rest={back_rest_aabb}, open={back_open_aabb}",
        )

    ladder_upper = None if ladder_pivot.motion_limits is None else ladder_pivot.motion_limits.upper
    if ladder_upper is not None:
        with ctx.pose({ladder_pivot: ladder_upper}):
            ladder_open_aabb = ctx.part_world_aabb(ladder)
        ctx.check(
            "rear ladder bracket swings upward",
            _max_z(ladder_rest_aabb) is not None
            and _max_z(ladder_open_aabb) is not None
            and _max_z(ladder_open_aabb) > _max_z(ladder_rest_aabb) + 0.20,
            details=f"rest={ladder_rest_aabb}, raised={ladder_open_aabb}",
        )

    with ctx.pose({back_hinge: 0.60, ladder_pivot: 0.99}):
        ctx.expect_overlap(
            ladder,
            back_pad,
            axes="xy",
            elem_a="top_stop",
            elem_b="support_catch",
            min_overlap=0.02,
            name="ladder stop stays under the back support catch",
        )
        ctx.expect_gap(
            back_pad,
            ladder,
            axis="z",
            positive_elem="support_catch",
            negative_elem="top_stop",
            max_gap=0.002,
            max_penetration=0.001,
            name="ladder stop seats tightly under the support catch",
        )

    wheel_limits_ok = all(
        joint.motion_limits is not None
        and joint.motion_limits.lower is None
        and joint.motion_limits.upper is None
        for joint in (wheel_spin_0, wheel_spin_1)
    )
    ctx.check(
        "transport wheels use continuous axle motion",
        wheel_limits_ok,
        details=(
            f"wheel_spin_0_limits={wheel_spin_0.motion_limits}, "
            f"wheel_spin_1_limits={wheel_spin_1.motion_limits}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
