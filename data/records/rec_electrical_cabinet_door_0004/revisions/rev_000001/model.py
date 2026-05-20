from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)

BODY_WIDTH = 1.00
BODY_DEPTH = 0.44
BODY_HEIGHT = 2.10

PLINTH_WIDTH = 0.94
PLINTH_DEPTH = 0.38
PLINTH_HEIGHT = 0.08

WALL_THICKNESS = 0.025
BACK_THICKNESS = 0.018
FRAME_THICKNESS = 0.012
TOP_PANEL_THICKNESS = 0.025
BOTTOM_PANEL_THICKNESS = 0.020

JAMB_WIDTH = 0.035
CENTER_STILE_WIDTH = 0.036

DOOR_THICKNESS = 0.030
DOOR_REST_GAP = 0.001
HINGE_FRONT_OFFSET = 0.008
OUTER_REVEAL = 0.012
CENTER_SEAM = 0.004
DOOR_BOTTOM = 0.10
DOOR_HEIGHT = 1.95
DOOR_WIDTH = (BODY_WIDTH / 2.0) - OUTER_REVEAL - (CENTER_SEAM / 2.0)
DOOR_TOP = DOOR_BOTTOM + DOOR_HEIGHT

HINGE_AXIS_Y = (BODY_DEPTH / 2.0) + DOOR_REST_GAP + DOOR_THICKNESS + HINGE_FRONT_OFFSET
HINGE_LEAF_DEPTH = HINGE_AXIS_Y - (BODY_DEPTH / 2.0)
HINGE_RADIUS = 0.008
HINGE_MOUNT_WIDTH = 0.020
HINGE_KNUCKLE_WIDTH = 0.018
HINGE_KNUCKLE_DEPTH = 0.012
HINGE_KNUCKLE_OFFSET = 0.010
HINGE_KNUCKLE_ATTACH_Y = -0.002
HINGE_BLOCK_HEIGHT = 0.22
LOWER_HINGE_Z = 0.30
UPPER_HINGE_Z = BODY_HEIGHT - 0.30

HANDLE_WIDTH = 0.024
HANDLE_GRIP_DEPTH = 0.014
HANDLE_GRIP_HEIGHT = 0.16
HANDLE_SPINDLE_RADIUS = 0.0035
HANDLE_SPINDLE_LENGTH = 0.004
HANDLE_PIVOT_RADIUS = 0.007
HANDLE_PIVOT_LENGTH = 0.006
HANDLE_PIVOT_STANDOFF = HANDLE_SPINDLE_LENGTH
HANDLE_GRIP_Z_OFFSET = -0.055
HANDLE_TOE_Z_OFFSET = -0.115
HANDLE_CENTER_Z = 1.12
HANDLE_INSET = 0.055
HANDLE_SWING = math.radians(30.0)

LATCH_BOLT_WIDTH = 0.014
LATCH_BOLT_DEPTH = 0.004
LATCH_BOLT_HEIGHT = 0.20
STRIKE_WIDTH = 0.012
STRIKE_DEPTH = 0.010
STRIKE_HEIGHT = 0.22
STRIKE_X_OFFSET = 0.010


def _left_hinge_x() -> float:
    return -(BODY_WIDTH / 2.0) + OUTER_REVEAL


def _right_hinge_x() -> float:
    return (BODY_WIDTH / 2.0) - OUTER_REVEAL


def _door_panel_center_y() -> float:
    return -(HINGE_FRONT_OFFSET + (DOOR_THICKNESS / 2.0))


def _door_front_trim_y() -> float:
    return -(HINGE_FRONT_OFFSET + 0.002)


def _door_front_surface_y() -> float:
    return -HINGE_FRONT_OFFSET


def _door_latch_bolt_y() -> float:
    return -(HINGE_FRONT_OFFSET + DOOR_THICKNESS - (LATCH_BOLT_DEPTH / 2.0))


def _handle_local_z() -> float:
    return HANDLE_CENTER_Z - DOOR_BOTTOM


def _aabb_center_x(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][0] + aabb[1][0]) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mcc_panel")

    steel = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.61, 0.63, 0.66, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.16, 0.17, 0.19, 1.0))
    plinth_gray = model.material("plinth_gray", rgba=(0.28, 0.29, 0.31, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((PLINTH_WIDTH, PLINTH_DEPTH, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT / 2.0)),
        material=plinth_gray,
        name="plinth",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT - PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH / 2.0) + (WALL_THICKNESS / 2.0),
                0.0,
                PLINTH_HEIGHT + ((BODY_HEIGHT - PLINTH_HEIGHT) / 2.0),
            )
        ),
        material=steel,
        name="left_wall",
    )
    cabinet.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, BODY_HEIGHT - PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0),
                0.0,
                PLINTH_HEIGHT + ((BODY_HEIGHT - PLINTH_HEIGHT) / 2.0),
            )
        ),
        material=steel,
        name="right_wall",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * WALL_THICKNESS), BACK_THICKNESS, BODY_HEIGHT - PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH / 2.0) + (BACK_THICKNESS / 2.0),
                PLINTH_HEIGHT + ((BODY_HEIGHT - PLINTH_HEIGHT) / 2.0),
            )
        ),
        material=steel,
        name="back_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * WALL_THICKNESS), BODY_DEPTH - BACK_THICKNESS, TOP_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS / 2.0,
                BODY_HEIGHT - (TOP_PANEL_THICKNESS / 2.0),
            )
        ),
        material=steel,
        name="roof_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * WALL_THICKNESS), BODY_DEPTH - BACK_THICKNESS, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + (BOTTOM_PANEL_THICKNESS / 2.0),
            )
        ),
        material=steel,
        name="floor_panel",
    )
    cabinet.visual(
        Box((JAMB_WIDTH, FRAME_THICKNESS, DOOR_TOP - PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH / 2.0) + (JAMB_WIDTH / 2.0),
                (BODY_DEPTH / 2.0) - (FRAME_THICKNESS / 2.0),
                PLINTH_HEIGHT + ((DOOR_TOP - PLINTH_HEIGHT) / 2.0),
            )
        ),
        material=frame_gray,
        name="left_jamb",
    )
    cabinet.visual(
        Box((JAMB_WIDTH, FRAME_THICKNESS, DOOR_TOP - PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH / 2.0) - (JAMB_WIDTH / 2.0),
                (BODY_DEPTH / 2.0) - (FRAME_THICKNESS / 2.0),
                PLINTH_HEIGHT + ((DOOR_TOP - PLINTH_HEIGHT) / 2.0),
            )
        ),
        material=frame_gray,
        name="right_jamb",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * JAMB_WIDTH), FRAME_THICKNESS, BODY_HEIGHT - DOOR_TOP)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH / 2.0) - (FRAME_THICKNESS / 2.0),
                DOOR_TOP + ((BODY_HEIGHT - DOOR_TOP) / 2.0),
            )
        ),
        material=frame_gray,
        name="top_rail",
    )
    cabinet.visual(
        Box((BODY_WIDTH - (2.0 * JAMB_WIDTH), FRAME_THICKNESS, DOOR_BOTTOM - PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH / 2.0) - (FRAME_THICKNESS / 2.0),
                PLINTH_HEIGHT + ((DOOR_BOTTOM - PLINTH_HEIGHT) / 2.0),
            )
        ),
        material=frame_gray,
        name="bottom_rail",
    )
    cabinet.visual(
        Box((CENTER_STILE_WIDTH, FRAME_THICKNESS, DOOR_TOP - PLINTH_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH / 2.0) - (FRAME_THICKNESS / 2.0),
                PLINTH_HEIGHT + ((DOOR_TOP - PLINTH_HEIGHT) / 2.0),
            )
        ),
        material=frame_gray,
        name="center_stile",
    )

    for side_name, hinge_x in (("left", _left_hinge_x()), ("right", _right_hinge_x())):
        mount_x = hinge_x - (HINGE_MOUNT_WIDTH / 2.0) if side_name == "left" else hinge_x + (HINGE_MOUNT_WIDTH / 2.0)
        for mount_name, hinge_z in (("lower", LOWER_HINGE_Z), ("upper", UPPER_HINGE_Z)):
            cabinet.visual(
                Box((HINGE_MOUNT_WIDTH, HINGE_LEAF_DEPTH, HINGE_BLOCK_HEIGHT)),
                origin=Origin(
                    xyz=(
                        mount_x,
                        (BODY_DEPTH / 2.0) + (HINGE_LEAF_DEPTH / 2.0),
                        hinge_z,
                    )
                ),
                material=dark_hardware,
                name=f"{side_name}_{mount_name}_hinge_mount",
            )

    cabinet.visual(
        Box((STRIKE_WIDTH, STRIKE_DEPTH, STRIKE_HEIGHT)),
        origin=Origin(
            xyz=(
                -STRIKE_X_OFFSET,
                (BODY_DEPTH / 2.0) - (STRIKE_DEPTH / 2.0),
                HANDLE_CENTER_Z,
            )
        ),
        material=dark_hardware,
        name="left_strike",
    )
    cabinet.visual(
        Box((STRIKE_WIDTH, STRIKE_DEPTH, STRIKE_HEIGHT)),
        origin=Origin(
            xyz=(
                STRIKE_X_OFFSET,
                (BODY_DEPTH / 2.0) - (STRIKE_DEPTH / 2.0),
                HANDLE_CENTER_Z,
            )
        ),
        material=dark_hardware,
        name="right_strike",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH + 0.06, BODY_HEIGHT)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, _door_panel_center_y(), DOOR_HEIGHT / 2.0)),
        material=steel,
        name="panel",
    )
    left_door.visual(
        Box((0.05, 0.004, DOOR_HEIGHT - 0.16)),
        origin=Origin(xyz=(0.035, _door_front_trim_y(), DOOR_HEIGHT / 2.0)),
        material=frame_gray,
        name="outer_edge_stile",
    )
    left_door.visual(
        Box((DOOR_WIDTH - 0.10, 0.004, 0.06)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, _door_front_trim_y(), DOOR_HEIGHT - 0.06)),
        material=frame_gray,
        name="top_face_rail",
    )
    left_door.visual(
        Box((DOOR_WIDTH - 0.10, 0.004, 0.06)),
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, _door_front_trim_y(), 0.06)),
        material=frame_gray,
        name="bottom_face_rail",
    )
    left_door.visual(
        Box((LATCH_BOLT_WIDTH, LATCH_BOLT_DEPTH, LATCH_BOLT_HEIGHT)),
        origin=Origin(
            xyz=(
                DOOR_WIDTH - 0.009,
                _door_latch_bolt_y(),
                _handle_local_z(),
            )
        ),
        material=dark_hardware,
        name="latch_bolt",
    )
    left_door.visual(
        Box((0.07, 0.003, 0.26)),
        origin=Origin(
            xyz=(
                DOOR_WIDTH - 0.042,
                _door_front_trim_y(),
                _handle_local_z(),
            )
        ),
        material=frame_gray,
        name="latch_escutcheon",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS + 0.02, DOOR_HEIGHT)),
        mass=38.0,
        origin=Origin(xyz=(DOOR_WIDTH / 2.0, _door_panel_center_y(), DOOR_HEIGHT / 2.0)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(-(DOOR_WIDTH / 2.0), _door_panel_center_y(), DOOR_HEIGHT / 2.0)),
        material=steel,
        name="panel",
    )
    right_door.visual(
        Box((0.05, 0.004, DOOR_HEIGHT - 0.16)),
        origin=Origin(xyz=(-0.035, _door_front_trim_y(), DOOR_HEIGHT / 2.0)),
        material=frame_gray,
        name="outer_edge_stile",
    )
    right_door.visual(
        Box((DOOR_WIDTH - 0.10, 0.004, 0.06)),
        origin=Origin(xyz=(-(DOOR_WIDTH / 2.0), _door_front_trim_y(), DOOR_HEIGHT - 0.06)),
        material=frame_gray,
        name="top_face_rail",
    )
    right_door.visual(
        Box((DOOR_WIDTH - 0.10, 0.004, 0.06)),
        origin=Origin(xyz=(-(DOOR_WIDTH / 2.0), _door_front_trim_y(), 0.06)),
        material=frame_gray,
        name="bottom_face_rail",
    )
    right_door.visual(
        Box((LATCH_BOLT_WIDTH, LATCH_BOLT_DEPTH, LATCH_BOLT_HEIGHT)),
        origin=Origin(
            xyz=(
                -(DOOR_WIDTH - 0.009),
                _door_latch_bolt_y(),
                _handle_local_z(),
            )
        ),
        material=dark_hardware,
        name="latch_bolt",
    )
    right_door.visual(
        Box((0.07, 0.003, 0.26)),
        origin=Origin(
            xyz=(
                -(DOOR_WIDTH - 0.042),
                _door_front_trim_y(),
                _handle_local_z(),
            )
        ),
        material=frame_gray,
        name="latch_escutcheon",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS + 0.02, DOOR_HEIGHT)),
        mass=38.0,
        origin=Origin(xyz=(-(DOOR_WIDTH / 2.0), _door_panel_center_y(), DOOR_HEIGHT / 2.0)),
    )

    left_handle = model.part("left_handle")
    left_handle.visual(
        Cylinder(radius=HANDLE_SPINDLE_RADIUS, length=HANDLE_SPINDLE_LENGTH),
        origin=Origin(
            xyz=(0.0, HANDLE_SPINDLE_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_hardware,
        name="spindle",
    )
    left_handle.visual(
        Cylinder(radius=HANDLE_PIVOT_RADIUS, length=HANDLE_PIVOT_LENGTH),
        origin=Origin(
            xyz=(0.0, HANDLE_PIVOT_STANDOFF + (HANDLE_PIVOT_LENGTH / 2.0), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_hardware,
        name="pivot_hub",
    )
    left_handle.visual(
        Box((HANDLE_WIDTH, HANDLE_GRIP_DEPTH, HANDLE_GRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HANDLE_PIVOT_STANDOFF + HANDLE_PIVOT_LENGTH + (HANDLE_GRIP_DEPTH / 2.0),
                HANDLE_GRIP_Z_OFFSET,
            )
        ),
        material=dark_hardware,
        name="grip",
    )
    left_handle.visual(
        Box((HANDLE_WIDTH * 0.58, HANDLE_GRIP_DEPTH * 1.2, 0.035)),
        origin=Origin(
            xyz=(
                0.0,
                HANDLE_PIVOT_STANDOFF + HANDLE_PIVOT_LENGTH + (HANDLE_GRIP_DEPTH * 0.6),
                HANDLE_TOE_Z_OFFSET,
            )
        ),
        material=dark_hardware,
        name="toe_piece",
    )
    left_handle.inertial = Inertial.from_geometry(
        Box((0.03, 0.03, 0.18)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.018, HANDLE_GRIP_Z_OFFSET)),
    )

    right_handle = model.part("right_handle")
    right_handle.visual(
        Cylinder(radius=HANDLE_SPINDLE_RADIUS, length=HANDLE_SPINDLE_LENGTH),
        origin=Origin(
            xyz=(0.0, HANDLE_SPINDLE_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_hardware,
        name="spindle",
    )
    right_handle.visual(
        Cylinder(radius=HANDLE_PIVOT_RADIUS, length=HANDLE_PIVOT_LENGTH),
        origin=Origin(
            xyz=(0.0, HANDLE_PIVOT_STANDOFF + (HANDLE_PIVOT_LENGTH / 2.0), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_hardware,
        name="pivot_hub",
    )
    right_handle.visual(
        Box((HANDLE_WIDTH, HANDLE_GRIP_DEPTH, HANDLE_GRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HANDLE_PIVOT_STANDOFF + HANDLE_PIVOT_LENGTH + (HANDLE_GRIP_DEPTH / 2.0),
                HANDLE_GRIP_Z_OFFSET,
            )
        ),
        material=dark_hardware,
        name="grip",
    )
    right_handle.visual(
        Box((HANDLE_WIDTH * 0.58, HANDLE_GRIP_DEPTH * 1.2, 0.035)),
        origin=Origin(
            xyz=(
                0.0,
                HANDLE_PIVOT_STANDOFF + HANDLE_PIVOT_LENGTH + (HANDLE_GRIP_DEPTH * 0.6),
                HANDLE_TOE_Z_OFFSET,
            )
        ),
        material=dark_hardware,
        name="toe_piece",
    )
    right_handle.inertial = Inertial.from_geometry(
        Box((0.03, 0.03, 0.18)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.018, HANDLE_GRIP_Z_OFFSET)),
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(xyz=(_left_hinge_x(), HINGE_AXIS_Y, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(xyz=(_right_hinge_x(), HINGE_AXIS_Y, DOOR_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-math.radians(125.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "left_handle_turn",
        ArticulationType.REVOLUTE,
        parent=left_door,
        child=left_handle,
        origin=Origin(
            xyz=(
                DOOR_WIDTH - HANDLE_INSET,
                _door_front_surface_y(),
                _handle_local_z(),
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-HANDLE_SWING,
            upper=HANDLE_SWING,
        ),
    )
    model.articulation(
        "right_handle_turn",
        ArticulationType.REVOLUTE,
        parent=right_door,
        child=right_handle,
        origin=Origin(
            xyz=(
                -(DOOR_WIDTH - HANDLE_INSET),
                _door_front_surface_y(),
                _handle_local_z(),
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-HANDLE_SWING,
            upper=HANDLE_SWING,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")

    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    left_handle_turn = object_model.get_articulation("left_handle_turn")
    right_handle_turn = object_model.get_articulation("right_handle_turn")

    left_panel = left_door.get_visual("panel")
    right_panel = right_door.get_visual("panel")
    left_outer_stile = left_door.get_visual("outer_edge_stile")
    right_outer_stile = right_door.get_visual("outer_edge_stile")
    left_latch_bolt = left_door.get_visual("latch_bolt")
    right_latch_bolt = right_door.get_visual("latch_bolt")

    left_jamb = cabinet.get_visual("left_jamb")
    right_jamb = cabinet.get_visual("right_jamb")
    center_stile = cabinet.get_visual("center_stile")
    left_strike = cabinet.get_visual("left_strike")
    right_strike = cabinet.get_visual("right_strike")
    left_lower_mount = cabinet.get_visual("left_lower_hinge_mount")
    left_upper_mount = cabinet.get_visual("left_upper_hinge_mount")
    right_lower_mount = cabinet.get_visual("right_lower_hinge_mount")
    right_upper_mount = cabinet.get_visual("right_upper_hinge_mount")

    left_hub = left_handle.get_visual("pivot_hub")
    right_hub = right_handle.get_visual("pivot_hub")
    left_grip = left_handle.get_visual("grip")
    right_grip = right_handle.get_visual("grip")
    left_spindle = left_handle.get_visual("spindle")
    right_spindle = right_handle.get_visual("spindle")
    left_escutcheon = left_door.get_visual("latch_escutcheon")
    right_escutcheon = right_door.get_visual("latch_escutcheon")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        left_handle,
        left_door,
        elem_a=left_spindle,
        elem_b=left_panel,
        reason="The left handle spindle passes through the door skin into the internal latch mechanism.",
    )
    ctx.allow_overlap(
        left_handle,
        left_door,
        elem_a=left_spindle,
        reason="The left handle spindle intentionally penetrates the left door assembly to drive the center-stile latch hardware.",
    )
    ctx.allow_overlap(
        right_handle,
        right_door,
        elem_a=right_spindle,
        elem_b=right_panel,
        reason="The right handle spindle passes through the door skin into the internal latch mechanism.",
    )
    ctx.allow_overlap(
        right_handle,
        right_door,
        elem_a=right_spindle,
        reason="The right handle spindle intentionally penetrates the right door assembly to drive the center-stile latch hardware.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        left_door,
        cabinet,
        axis="y",
        positive_elem=left_panel,
        negative_elem=left_jamb,
        max_gap=0.002,
        max_penetration=0.0,
        name="left door seats just proud of the left jamb",
    )
    ctx.expect_gap(
        right_door,
        cabinet,
        axis="y",
        positive_elem=right_panel,
        negative_elem=right_jamb,
        max_gap=0.002,
        max_penetration=0.0,
        name="right door seats just proud of the right jamb",
    )
    ctx.expect_gap(
        left_door,
        cabinet,
        axis="y",
        positive_elem=left_panel,
        negative_elem=center_stile,
        max_gap=0.002,
        max_penetration=0.0,
        name="left door closes against the center stile plane",
    )
    ctx.expect_gap(
        right_door,
        cabinet,
        axis="y",
        positive_elem=right_panel,
        negative_elem=center_stile,
        max_gap=0.002,
        max_penetration=0.0,
        name="right door closes against the center stile plane",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        positive_elem=right_panel,
        negative_elem=left_panel,
        min_gap=0.0035,
        max_gap=0.0055,
        name="closed doors meet with a narrow center seam",
    )

    ctx.expect_overlap(
        left_door,
        cabinet,
        axes="xz",
        elem_a=left_latch_bolt,
        elem_b=left_strike,
        min_overlap=0.010,
        name="left latch aligns to the center stile strike",
    )
    ctx.expect_overlap(
        right_door,
        cabinet,
        axes="xz",
        elem_a=right_latch_bolt,
        elem_b=right_strike,
        min_overlap=0.010,
        name="right latch aligns to the center stile strike",
    )
    ctx.expect_gap(
        left_door,
        cabinet,
        axis="y",
        positive_elem=left_latch_bolt,
        negative_elem=left_strike,
        min_gap=0.001,
        max_gap=0.006,
        name="left latch sits just ahead of the strike hardware",
    )
    ctx.expect_gap(
        right_door,
        cabinet,
        axis="y",
        positive_elem=right_latch_bolt,
        negative_elem=right_strike,
        min_gap=0.001,
        max_gap=0.006,
        name="right latch sits just ahead of the strike hardware",
    )

    ctx.expect_gap(
        left_door,
        cabinet,
        axis="x",
        positive_elem=left_outer_stile,
        negative_elem=left_lower_mount,
        min_gap=0.009,
        max_gap=0.011,
        name="left lower hinge leaf sits just outside the left door edge",
    )
    ctx.expect_gap(
        left_door,
        cabinet,
        axis="x",
        positive_elem=left_outer_stile,
        negative_elem=left_upper_mount,
        min_gap=0.009,
        max_gap=0.011,
        name="left upper hinge leaf sits just outside the left door edge",
    )
    ctx.expect_gap(
        cabinet,
        right_door,
        axis="x",
        positive_elem=right_lower_mount,
        negative_elem=right_outer_stile,
        min_gap=0.009,
        max_gap=0.011,
        name="right lower hinge leaf sits just outside the right door edge",
    )
    ctx.expect_gap(
        cabinet,
        right_door,
        axis="x",
        positive_elem=right_upper_mount,
        negative_elem=right_outer_stile,
        min_gap=0.009,
        max_gap=0.011,
        name="right upper hinge leaf sits just outside the right door edge",
    )
    ctx.expect_overlap(
        left_door,
        cabinet,
        axes="yz",
        elem_a=left_outer_stile,
        elem_b=left_lower_mount,
        min_overlap=0.003,
        name="left lower hinge zone aligns along the door edge",
    )
    ctx.expect_overlap(
        left_door,
        cabinet,
        axes="yz",
        elem_a=left_outer_stile,
        elem_b=left_upper_mount,
        min_overlap=0.003,
        name="left upper hinge zone aligns along the door edge",
    )
    ctx.expect_overlap(
        cabinet,
        right_door,
        axes="yz",
        elem_a=right_lower_mount,
        elem_b=right_outer_stile,
        min_overlap=0.003,
        name="right lower hinge zone aligns along the door edge",
    )
    ctx.expect_overlap(
        cabinet,
        right_door,
        axes="yz",
        elem_a=right_upper_mount,
        elem_b=right_outer_stile,
        min_overlap=0.003,
        name="right upper hinge zone aligns along the door edge",
    )

    ctx.expect_contact(
        left_handle,
        left_door,
        name="left handle is mounted to the door face",
    )
    ctx.expect_contact(
        right_handle,
        right_door,
        name="right handle is mounted to the door face",
    )
    ctx.expect_gap(
        right_handle,
        left_handle,
        axis="x",
        min_gap=0.08,
        name="closed handles remain separated across the center seam",
    )

    left_closed_panel_aabb = ctx.part_element_world_aabb(left_door, elem=left_panel)
    right_closed_panel_aabb = ctx.part_element_world_aabb(right_door, elem=right_panel)
    left_rest_grip_aabb = ctx.part_element_world_aabb(left_handle, elem=left_grip)
    right_rest_grip_aabb = ctx.part_element_world_aabb(right_handle, elem=right_grip)

    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="left_door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="left_door_open_no_floating")
        left_open_panel_aabb = ctx.part_element_world_aabb(left_door, elem=left_panel)
        ctx.check(
            "left door swings away from the center opening",
            left_closed_panel_aabb is not None
            and left_open_panel_aabb is not None
            and left_open_panel_aabb[0][1] > left_closed_panel_aabb[1][1] + 0.01,
            details="Left door did not move outward around the left-side hinge as expected.",
        )

    with ctx.pose({right_hinge: right_hinge.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="right_door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="right_door_open_no_floating")
        right_open_panel_aabb = ctx.part_element_world_aabb(right_door, elem=right_panel)
        ctx.check(
            "right door swings away from the center opening",
            right_closed_panel_aabb is not None
            and right_open_panel_aabb is not None
            and right_open_panel_aabb[0][1] > right_closed_panel_aabb[1][1] + 0.01,
            details="Right door did not move outward around the right-side hinge as expected.",
        )

    with ctx.pose({left_hinge: left_hinge.motion_limits.upper, right_hinge: right_hinge.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="both_doors_open_no_overlap")
        ctx.fail_if_isolated_parts(name="both_doors_open_no_floating")

    with ctx.pose({left_handle_turn: left_handle_turn.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="left_handle_turned_no_overlap")
        left_turned_grip_aabb = ctx.part_element_world_aabb(left_handle, elem=left_grip)
        ctx.expect_gap(
            left_handle,
            left_door,
            axis="y",
            positive_elem=left_hub,
            negative_elem=left_escutcheon,
            min_gap=0.0,
            max_gap=0.005,
            name="left handle stays closely seated to the escutcheon while turning",
        )
        ctx.check(
            "left handle rotates about its latch hub",
            left_rest_grip_aabb is not None
            and left_turned_grip_aabb is not None
            and _aabb_center_x(left_rest_grip_aabb) is not None
            and _aabb_center_x(left_turned_grip_aabb) is not None
            and abs(_aabb_center_x(left_turned_grip_aabb) - _aabb_center_x(left_rest_grip_aabb)) > 0.02,
            details="Left handle grip center did not shift enough to show real rotation.",
        )

    with ctx.pose({right_handle_turn: right_handle_turn.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="right_handle_turned_no_overlap")
        right_turned_grip_aabb = ctx.part_element_world_aabb(right_handle, elem=right_grip)
        ctx.expect_gap(
            right_handle,
            right_door,
            axis="y",
            positive_elem=right_hub,
            negative_elem=right_escutcheon,
            min_gap=0.0,
            max_gap=0.005,
            name="right handle stays closely seated to the escutcheon while turning",
        )
        ctx.check(
            "right handle rotates about its latch hub",
            right_rest_grip_aabb is not None
            and right_turned_grip_aabb is not None
            and _aabb_center_x(right_rest_grip_aabb) is not None
            and _aabb_center_x(right_turned_grip_aabb) is not None
            and abs(_aabb_center_x(right_turned_grip_aabb) - _aabb_center_x(right_rest_grip_aabb)) > 0.02,
            details="Right handle grip center did not shift enough to show real rotation.",
        )

    for joint in (left_hinge, right_hinge):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
