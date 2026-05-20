from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


FRAME_OUTER_W = 0.50
FRAME_OUTER_H = 0.64
FRAME_BORDER = 0.04
FRAME_T = 0.012

BODY_OUTER_W = 0.44
BODY_OUTER_H = 0.58
BODY_DEPTH = 0.32
BODY_WALL = 0.02
BACK_PANEL_T = 0.024

DOOR_W = 0.412
DOOR_H = 0.552
DOOR_T = 0.080
DOOR_BACK_Z = 0.005
HINGE_OFFSET = 0.012
HINGE_X = -(DOOR_W * 0.5 + HINGE_OFFSET)

DIAL_LOCAL_X = HINGE_OFFSET + DOOR_W * 0.5
DIAL_LOCAL_Y = 0.085
HANDLE_LOCAL_X = DIAL_LOCAL_X
HANDLE_LOCAL_Y = -0.115


def _add_safe_body(model: ArticulatedObject):
    frame_paint = model.material("frame_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    shell_paint = model.material("shell_paint", rgba=(0.13, 0.14, 0.15, 1.0))
    body = model.part("safe_body")
    body.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_W, FRAME_OUTER_H, BODY_DEPTH)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    body.visual(
        Box((BODY_OUTER_W, BODY_OUTER_H, BACK_PANEL_T)),
        origin=Origin(xyz=(0.0, 0.0, -(BODY_DEPTH - BACK_PANEL_T * 0.5))),
        material=shell_paint,
        name="back_panel",
    )
    body.visual(
        Box((BODY_WALL, BODY_OUTER_H, BODY_DEPTH - BACK_PANEL_T)),
        origin=Origin(xyz=(-(BODY_OUTER_W * 0.5 - BODY_WALL * 0.5), 0.0, -0.148)),
        material=shell_paint,
        name="left_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_OUTER_H, BODY_DEPTH - BACK_PANEL_T)),
        origin=Origin(xyz=((BODY_OUTER_W * 0.5 - BODY_WALL * 0.5), 0.0, -0.148)),
        material=shell_paint,
        name="right_wall",
    )
    body.visual(
        Box((BODY_OUTER_W - 2.0 * BODY_WALL, BODY_WALL, BODY_DEPTH - BACK_PANEL_T)),
        origin=Origin(xyz=(0.0, BODY_OUTER_H * 0.5 - BODY_WALL * 0.5, -0.148)),
        material=shell_paint,
        name="top_wall",
    )
    body.visual(
        Box((BODY_OUTER_W - 2.0 * BODY_WALL, BODY_WALL, BODY_DEPTH - BACK_PANEL_T)),
        origin=Origin(xyz=(0.0, -(BODY_OUTER_H * 0.5 - BODY_WALL * 0.5), -0.148)),
        material=shell_paint,
        name="bottom_wall",
    )

    body.visual(
        Box((FRAME_OUTER_W, FRAME_BORDER, FRAME_T)),
        origin=Origin(xyz=(0.0, FRAME_OUTER_H * 0.5 - FRAME_BORDER * 0.5, -0.002)),
        material=frame_paint,
        name="frame_top_rail",
    )
    body.visual(
        Box((FRAME_OUTER_W, FRAME_BORDER, FRAME_T)),
        origin=Origin(xyz=(0.0, -(FRAME_OUTER_H * 0.5 - FRAME_BORDER * 0.5), -0.002)),
        material=frame_paint,
        name="frame_bottom_rail",
    )
    body.visual(
        Box((FRAME_BORDER, FRAME_OUTER_H - 2.0 * FRAME_BORDER, FRAME_T)),
        origin=Origin(xyz=(-(FRAME_OUTER_W * 0.5 - FRAME_BORDER * 0.5), 0.0, -0.002)),
        material=frame_paint,
        name="frame_left_rail",
    )
    body.visual(
        Box((FRAME_BORDER, FRAME_OUTER_H - 2.0 * FRAME_BORDER, FRAME_T)),
        origin=Origin(xyz=((FRAME_OUTER_W * 0.5 - FRAME_BORDER * 0.5), 0.0, -0.002)),
        material=frame_paint,
        name="frame_right_rail",
    )

    body.visual(
        Box((0.020, DOOR_H + 0.028, 0.024)),
        origin=Origin(xyz=(HINGE_X - 0.010, 0.0, 0.010)),
        material=shell_paint,
        name="hinge_mount",
    )

    return body


def _add_door(model: ArticulatedObject):
    door_paint = model.material("door_paint", rgba=(0.28, 0.29, 0.31, 1.0))
    plate_paint = model.material("plate_paint", rgba=(0.18, 0.19, 0.20, 1.0))

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_H, DOOR_T)),
        mass=12.5,
        origin=Origin(xyz=(HINGE_OFFSET + DOOR_W * 0.5, 0.0, DOOR_T * 0.5)),
    )

    door.visual(
        Box((DOOR_W, DOOR_H, DOOR_T)),
        origin=Origin(xyz=(HINGE_OFFSET + DOOR_W * 0.5, 0.0, DOOR_T * 0.5)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_W - 0.072, DOOR_H - 0.100, 0.014)),
        origin=Origin(xyz=(HINGE_OFFSET + DOOR_W * 0.5, 0.0, DOOR_T - 0.007)),
        material=plate_paint,
        name="door_face_plate",
    )
    door.visual(
        Box((DOOR_W - 0.092, DOOR_H - 0.120, 0.022)),
        origin=Origin(xyz=(HINGE_OFFSET + DOOR_W * 0.5, 0.0, 0.011)),
        material=plate_paint,
        name="door_back_plate",
    )
    door.visual(
        Box((0.016, DOOR_H - 0.030, 0.018)),
        origin=Origin(xyz=(0.008, 0.0, 0.014)),
        material=plate_paint,
        name="hinge_leaf",
    )
    door.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(DIAL_LOCAL_X, DIAL_LOCAL_Y + 0.050, DOOR_T - 0.003)),
        material=plate_paint,
        name="dial_index_marker",
    )

    for bolt_y in (-0.16, 0.0, 0.16):
        door.visual(
            Cylinder(radius=0.012, length=0.028),
            origin=Origin(
                xyz=(HINGE_OFFSET + DOOR_W - 0.006, bolt_y, 0.020),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=plate_paint,
            name=f"locking_bolt_{int((bolt_y + 0.16) * 1000)}",
        )

    return door


def _add_dial(model: ArticulatedObject):
    bright_steel = model.material("dial_bright_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_steel = model.material("dial_dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    dial = model.part("dial")
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.048, length=0.040),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    dial.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="dial_collar",
    )
    dial.visual(
        Cylinder(radius=0.048, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=bright_steel,
        name="dial_body",
    )
    dial.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=dark_steel,
        name="dial_cap",
    )
    dial.visual(
        Box((0.010, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_steel,
        name="dial_grip_bar",
    )

    return dial


def _add_handle(model: ArticulatedObject):
    bright_steel = model.material("handle_bright_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_steel = model.material("handle_dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))

    handle = model.part("wheel_handle")
    handle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.040),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    handle.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="handle_collar",
    )
    handle.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_steel,
        name="handle_hub",
    )
    handle.visual(
        mesh_from_geometry(TorusGeometry(radius=0.060, tube=0.006), "wall_safe_handle_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=bright_steel,
        name="handle_ring",
    )
    for index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0), start=1):
        handle.visual(
            Box((0.068, 0.012, 0.012)),
            origin=Origin(xyz=(0.034, 0.0, 0.030), rpy=(0.0, 0.0, angle)),
            material=bright_steel,
            name=f"handle_spoke_{index}",
        )

    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    body = _add_safe_body(model)
    door = _add_door(model)
    dial = _add_dial(model)
    handle = _add_handle(model)

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, 0.0, DOOR_BACK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(DIAL_LOCAL_X, DIAL_LOCAL_Y, DOOR_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "door_to_wheel_handle",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=handle,
        origin=Origin(xyz=(HANDLE_LOCAL_X, HANDLE_LOCAL_Y, DOOR_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("safe_body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    handle = object_model.get_part("wheel_handle")

    door_hinge = object_model.get_articulation("body_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    handle_spin = object_model.get_articulation("door_to_wheel_handle")

    door_limits = door_hinge.motion_limits
    dial_limits = dial_spin.motion_limits
    handle_limits = handle_spin.motion_limits

    ctx.check(
        "door uses a left-side vertical hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (0.0, -1.0, 0.0)
        and door_hinge.origin.xyz[0] < 0.0
        and door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper is not None
        and door_limits.upper >= 1.5,
        details=f"axis={door_hinge.axis}, origin={door_hinge.origin.xyz}, limits={door_limits}",
    )
    ctx.check(
        "dial spins continuously on the door-normal shaft",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(dial_spin.axis) == (0.0, 0.0, 1.0)
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"axis={dial_spin.axis}, limits={dial_limits}",
    )
    ctx.check(
        "wheel handle spins continuously on a coaxial door-normal shaft",
        handle_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(handle_spin.axis) == (0.0, 0.0, 1.0)
        and handle_limits is not None
        and handle_limits.lower is None
        and handle_limits.upper is None
        and abs(dial_spin.origin.xyz[0] - handle_spin.origin.xyz[0]) <= 1e-6,
        details=(
            f"dial_axis={dial_spin.axis}, handle_axis={handle_spin.axis}, "
            f"dial_origin={dial_spin.origin.xyz}, handle_origin={handle_spin.origin.xyz}"
        ),
    )

    ctx.expect_contact(
        dial,
        door,
        elem_a="dial_collar",
        elem_b="door_panel",
        name="dial collar seats against the door slab",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="handle_collar",
        elem_b="door_panel",
        name="wheel handle collar seats against the door slab",
    )
    ctx.expect_origin_distance(
        dial,
        handle,
        axes="x",
        max_dist=0.001,
        name="dial and wheel handle share the central vertical centerline",
    )
    ctx.expect_origin_gap(
        dial,
        handle,
        axis="y",
        min_gap=0.16,
        max_gap=0.24,
        name="dial sits above the wheel handle",
    )

    with ctx.pose({door_hinge: 0.0}):
        closed_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        closed_dial_aabb = ctx.part_world_aabb(dial)
    with ctx.pose({door_hinge: 1.2}):
        open_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        open_dial_aabb = ctx.part_world_aabb(dial)

    ctx.check(
        "door swings outward from the frame",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.20,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )
    ctx.check(
        "door-mounted hardware follows the opening door",
        closed_dial_aabb is not None
        and open_dial_aabb is not None
        and open_dial_aabb[1][2] > closed_dial_aabb[1][2] + 0.12,
        details=f"closed={closed_dial_aabb}, open={open_dial_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
