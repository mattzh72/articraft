from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CABINET_WIDTH = 0.76
CABINET_DEPTH = 0.66
CABINET_HEIGHT = 1.86
WALL_THICKNESS = 0.028
FRONT_Y = CABINET_DEPTH / 2.0

WINDOW_OUTER_WIDTH = 0.50
WINDOW_OUTER_HEIGHT = 0.44
WINDOW_INNER_WIDTH = 0.39
WINDOW_INNER_HEIGHT = 0.31
WINDOW_DEPTH = 0.040
WINDOW_CENTER_Z = 1.06

DOOR_OPENING_WIDTH = 0.56
DOOR_OPENING_HEIGHT = 0.74
DOOR_WIDTH = 0.53
DOOR_HEIGHT = 0.70
DOOR_THICKNESS = 0.020
DOOR_CENTER_Z = 0.47

HANDLE_PIVOT_X = CABINET_WIDTH / 2.0 + 0.040
HANDLE_PIVOT_Y = 0.10
HANDLE_PIVOT_Z = 1.14


def _cabinet_shell_mesh():
    back_y = -CABINET_DEPTH / 2.0
    front_y = CABINET_DEPTH / 2.0

    outer_profile = (
        cq.Workplane("YZ")
        .moveTo(back_y, 0.0)
        .lineTo(front_y - 0.065, 0.0)
        .lineTo(front_y, 0.12)
        .lineTo(front_y, 1.43)
        .threePointArc((0.22, 1.79), (0.01, CABINET_HEIGHT))
        .lineTo(back_y + 0.02, CABINET_HEIGHT)
        .lineTo(back_y, 1.68)
        .close()
    )

    shell = outer_profile.extrude(CABINET_WIDTH / 2.0, both=True).faces("<Z").shell(-WALL_THICKNESS)

    window_cut = cq.Workplane("XY").box(
        WINDOW_INNER_WIDTH + 0.05,
        0.14,
        WINDOW_INNER_HEIGHT + 0.05,
    ).translate((0.0, FRONT_Y - 0.055, WINDOW_CENTER_Z))

    door_cut = cq.Workplane("XY").box(
        DOOR_OPENING_WIDTH,
        0.14,
        DOOR_OPENING_HEIGHT,
    ).translate((0.0, FRONT_Y - 0.055, DOOR_CENTER_Z))

    side_plate = cq.Workplane("XY").box(0.010, 0.18, 0.22).translate(
        (CABINET_WIDTH / 2.0 + 0.005, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)
    )

    front_trim = cq.Workplane("XY").box(0.62, 0.018, 0.12).translate((0.0, FRONT_Y + 0.009, 1.58))

    return shell.cut(window_cut).cut(door_cut).union(side_plate).union(front_trim)


def _window_frame_mesh():
    frame = cq.Workplane("XY").box(
        WINDOW_OUTER_WIDTH,
        WINDOW_DEPTH,
        WINDOW_OUTER_HEIGHT,
    ).translate((0.0, WINDOW_DEPTH / 2.0, 0.0))

    opening = cq.Workplane("XY").box(
        WINDOW_INNER_WIDTH,
        WINDOW_DEPTH + 0.01,
        WINDOW_INNER_HEIGHT,
    ).translate((0.0, WINDOW_DEPTH / 2.0, 0.0))

    mullion_offset = WINDOW_INNER_WIDTH / 3.0
    mullion = lambda x: cq.Workplane("XY").box(0.016, WINDOW_DEPTH, WINDOW_INNER_HEIGHT + 0.02).translate(
        (x, WINDOW_DEPTH / 2.0, 0.0)
    )

    return frame.cut(opening).union(mullion(-mullion_offset / 2.0)).union(mullion(mullion_offset / 2.0))


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reel_slot_machine")

    cabinet_finish = model.material("cabinet_finish", rgba=(0.54, 0.05, 0.08, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.80, 0.67, 0.21, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.08, 0.08, 0.10, 0.82))
    reel_finish = model.material("reel_finish", rgba=(0.94, 0.92, 0.85, 1.0))
    metal_finish = model.material("metal_finish", rgba=(0.73, 0.75, 0.79, 1.0))
    black_finish = model.material("black_finish", rgba=(0.10, 0.10, 0.12, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell_mesh(), "cabinet_shell"),
        material=cabinet_finish,
        name="shell",
    )
    cabinet.visual(
        Box((0.56, 0.018, 0.10)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.009, 0.74)),
        material=trim_finish,
        name="panel_band",
    )
    for index, x_pos in enumerate((-0.24, 0.24)):
        cabinet.visual(
            Box((0.020, 0.030, 0.46)),
            origin=Origin(xyz=(x_pos, FRONT_Y + 0.015, WINDOW_CENTER_Z)),
            material=trim_finish,
            name=f"window_mount_{index}",
        )

    reel_window = model.part("reel_window")
    reel_window.visual(
        mesh_from_cadquery(_window_frame_mesh(), "reel_window_frame"),
        material=trim_finish,
        name="frame",
    )
    reel_window.visual(
        Box((WINDOW_INNER_WIDTH * 0.98, 0.008, WINDOW_INNER_HEIGHT * 0.92)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=dark_glass,
        name="glass",
    )
    for index, x_pos in enumerate((-0.12, 0.0, 0.12)):
        reel_window.visual(
            Box((0.090, 0.004, 0.23)),
            origin=Origin(xyz=(x_pos, -0.002, 0.0)),
            material=reel_finish,
            name=f"reel_{index}",
        )

    service_door = model.part("service_door")
    service_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, -DOOR_THICKNESS / 2.0, 0.0)),
        material=cabinet_finish,
        name="panel",
    )
    service_door.visual(
        Box((DOOR_WIDTH - 0.08, 0.004, DOOR_HEIGHT - 0.10)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, 0.002, 0.0)),
        material=trim_finish,
        name="bead",
    )
    service_door.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(DOOR_WIDTH - 0.055, 0.004, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_finish,
        name="lock",
    )

    side_handle = model.part("side_handle")
    side_handle.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_finish,
        name="hub",
    )
    side_handle.visual(
        Cylinder(radius=0.018, length=0.34),
        origin=Origin(xyz=(0.024, 0.0, -0.17)),
        material=black_finish,
        name="arm",
    )
    side_handle.visual(
        Sphere(radius=0.050),
        origin=Origin(xyz=(0.024, 0.0, -0.39)),
        material=black_finish,
        name="knob",
    )

    model.articulation(
        "cabinet_to_reel_window",
        ArticulationType.FIXED,
        parent=cabinet,
        child=reel_window,
        origin=Origin(xyz=(0.0, FRONT_Y + 0.030, WINDOW_CENTER_Z)),
    )
    model.articulation(
        "cabinet_to_service_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=service_door,
        origin=Origin(xyz=(-DOOR_WIDTH / 2.0, FRONT_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=30.0, velocity=1.2),
    )
    model.articulation(
        "cabinet_to_side_handle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=side_handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.0, effort=18.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    reel_window = object_model.get_part("reel_window")
    service_door = object_model.get_part("service_door")
    side_handle = object_model.get_part("side_handle")

    door_joint = object_model.get_articulation("cabinet_to_service_door")
    handle_joint = object_model.get_articulation("cabinet_to_side_handle")

    ctx.expect_overlap(
        reel_window,
        cabinet,
        axes="xz",
        min_overlap=0.40,
        name="reel window stays broad on the cabinet front",
    )
    ctx.expect_overlap(
        service_door,
        cabinet,
        axes="xz",
        min_overlap=0.50,
        name="service door covers the lower cabinet frontage",
    )
    ctx.expect_origin_gap(
        reel_window,
        service_door,
        axis="z",
        min_gap=0.45,
        max_gap=0.80,
        name="reel window sits above the service door",
    )
    ctx.expect_origin_distance(
        side_handle,
        cabinet,
        axes="x",
        min_dist=0.39,
        max_dist=0.45,
        name="pull handle stays mounted outboard on the cabinet side",
    )

    closed_door_center = _aabb_center(ctx.part_world_aabb(service_door))
    if door_joint.motion_limits is not None and door_joint.motion_limits.upper is not None:
        with ctx.pose({door_joint: door_joint.motion_limits.upper}):
            open_door_center = _aabb_center(ctx.part_world_aabb(service_door))
        ctx.check(
            "service door swings outward on its side hinge",
            closed_door_center is not None
            and open_door_center is not None
            and open_door_center[1] > closed_door_center[1] + 0.12
            and open_door_center[0] < closed_door_center[0] - 0.12,
            details=f"closed_center={closed_door_center}, open_center={open_door_center}",
        )

    rest_knob_center = _aabb_center(ctx.part_element_world_aabb(side_handle, elem="knob"))
    if handle_joint.motion_limits is not None and handle_joint.motion_limits.upper is not None:
        with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
            pulled_knob_center = _aabb_center(ctx.part_element_world_aabb(side_handle, elem="knob"))
        ctx.check(
            "pull handle rotates forward through a large throw",
            rest_knob_center is not None
            and pulled_knob_center is not None
            and pulled_knob_center[1] > rest_knob_center[1] + 0.20
            and pulled_knob_center[2] > rest_knob_center[2] + 0.05,
            details=f"rest_knob={rest_knob_center}, pulled_knob={pulled_knob_center}",
        )

    return ctx.report()


object_model = build_object_model()
