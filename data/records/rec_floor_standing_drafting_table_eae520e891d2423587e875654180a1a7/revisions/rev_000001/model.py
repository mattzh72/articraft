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


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_drafting_table")

    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    aluminum = model.material("aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    black_handle = model.material("black_handle", rgba=(0.10, 0.10, 0.11, 1.0))
    maple = model.material("maple", rgba=(0.77, 0.67, 0.50, 1.0))
    drafting_surface = model.material("drafting_surface", rgba=(0.92, 0.91, 0.86, 1.0))
    edge_strip = model.material("edge_strip", rgba=(0.46, 0.35, 0.24, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((1.05, 0.74, 1.30)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )
    base.visual(
        Box((1.00, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, -0.30, 0.02)),
        material=painted_steel,
        name="left_sled_runner",
    )
    base.visual(
        Box((1.00, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.30, 0.02)),
        material=painted_steel,
        name="right_sled_runner",
    )
    base.visual(
        Box((0.12, 0.68, 0.08)),
        origin=Origin(xyz=(0.26, 0.0, 0.06)),
        material=painted_steel,
        name="front_tie_beam",
    )
    base.visual(
        Box((0.12, 0.68, 0.08)),
        origin=Origin(xyz=(-0.26, 0.0, 0.06)),
        material=painted_steel,
        name="rear_tie_beam",
    )
    base.visual(
        Box((0.26, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=painted_steel,
        name="pedestal_plinth",
    )
    base.visual(
        Box((0.18, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=painted_steel,
        name="standard_collar",
    )
    base.visual(
        Cylinder(radius=0.052, length=1.14),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=aluminum,
        name="vertical_standard",
    )
    base.visual(
        Cylinder(radius=0.082, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=painted_steel,
        name="lower_standard_boss",
    )
    base.visual(
        Box((0.56, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, -0.14, 0.06)),
        material=painted_steel,
        name="left_longitudinal_stretcher",
    )
    base.visual(
        Box((0.56, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.14, 0.06)),
        material=painted_steel,
        name="right_longitudinal_stretcher",
    )
    _add_member(
        base,
        (0.20, -0.10, 0.12),
        (0.0, -0.03, 0.44),
        0.018,
        painted_steel,
        name="left_lower_brace",
    )
    _add_member(
        base,
        (0.20, 0.10, 0.12),
        (0.0, 0.03, 0.44),
        0.018,
        painted_steel,
        name="right_lower_brace",
    )
    _add_member(
        base,
        (-0.18, -0.10, 0.12),
        (0.0, -0.03, 0.36),
        0.016,
        painted_steel,
        name="left_rear_brace",
    )
    _add_member(
        base,
        (-0.18, 0.10, 0.12),
        (0.0, 0.03, 0.36),
        0.016,
        painted_steel,
        name="right_rear_brace",
    )

    upper_frame = model.part("upper_frame")
    upper_frame.inertial = Inertial.from_geometry(
        Box((0.40, 1.02, 0.68)),
        mass=18.0,
        origin=Origin(xyz=(0.20, 0.0, 0.04)),
    )
    upper_frame.visual(
        Box((0.056, 0.20, 0.32)),
        origin=Origin(xyz=(0.08, 0.0, 0.02)),
        material=painted_steel,
        name="front_guide",
    )
    upper_frame.visual(
        Box((0.056, 0.20, 0.32)),
        origin=Origin(xyz=(-0.08, 0.0, 0.02)),
        material=painted_steel,
        name="rear_guide",
    )
    upper_frame.visual(
        Box((0.14, 0.056, 0.32)),
        origin=Origin(xyz=(0.0, -0.08, 0.02)),
        material=painted_steel,
        name="left_guide",
    )
    upper_frame.visual(
        Box((0.14, 0.056, 0.32)),
        origin=Origin(xyz=(0.0, 0.08, 0.02)),
        material=painted_steel,
        name="right_guide",
    )
    upper_frame.visual(
        Box((0.08, 0.18, 0.18)),
        origin=Origin(xyz=(0.13, 0.0, 0.01)),
        material=painted_steel,
        name="carriage_nose",
    )
    upper_frame.visual(
        Box((0.22, 0.14, 0.10)),
        origin=Origin(xyz=(0.23, 0.0, -0.08)),
        material=painted_steel,
        name="lower_spine",
    )
    upper_frame.visual(
        Box((0.18, 0.12, 0.08)),
        origin=Origin(xyz=(0.19, 0.0, 0.18)),
        material=painted_steel,
        name="upper_spine",
    )
    upper_frame.visual(
        Box((0.18, 0.96, 0.06)),
        origin=Origin(xyz=(0.24, 0.0, -0.16)),
        material=painted_steel,
        name="lower_crossmember",
    )
    upper_frame.visual(
        Box((0.10, 0.96, 0.05)),
        origin=Origin(xyz=(0.22, 0.0, 0.34)),
        material=painted_steel,
        name="upper_crossmember",
    )
    upper_frame.visual(
        Box((0.12, 0.04, 0.58)),
        origin=Origin(xyz=(0.22, -0.48, 0.08)),
        material=painted_steel,
        name="left_cheek",
    )
    upper_frame.visual(
        Box((0.12, 0.04, 0.58)),
        origin=Origin(xyz=(0.22, 0.48, 0.08)),
        material=painted_steel,
        name="right_cheek",
    )
    upper_frame.visual(
        Cylinder(radius=0.038, length=0.04),
        origin=Origin(xyz=(0.18, -0.50, 0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_pivot_boss",
    )
    upper_frame.visual(
        Cylinder(radius=0.038, length=0.04),
        origin=Origin(xyz=(0.18, 0.50, 0.04), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_pivot_boss",
    )
    _add_member(
        upper_frame,
        (0.16, -0.05, 0.18),
        (0.22, -0.46, 0.28),
        0.016,
        painted_steel,
        name="left_upper_brace",
    )
    _add_member(
        upper_frame,
        (0.16, 0.05, 0.18),
        (0.22, 0.46, 0.28),
        0.016,
        painted_steel,
        name="right_upper_brace",
    )
    _add_member(
        upper_frame,
        (0.18, -0.06, -0.08),
        (0.22, -0.46, -0.12),
        0.016,
        painted_steel,
        name="left_lower_frame_brace",
    )
    _add_member(
        upper_frame,
        (0.18, 0.06, -0.08),
        (0.22, 0.46, -0.12),
        0.016,
        painted_steel,
        name="right_lower_frame_brace",
    )

    board = model.part("board")
    board.inertial = Inertial.from_geometry(
        Box((0.78, 0.90, 0.08)),
        mass=12.5,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
    )
    board.visual(
        Box((0.74, 0.88, 0.030)),
        origin=Origin(xyz=(0.37, 0.0, 0.0)),
        material=maple,
        name="board_core",
    )
    board.visual(
        Box((0.70, 0.84, 0.004)),
        origin=Origin(xyz=(0.37, 0.0, 0.017)),
        material=drafting_surface,
        name="drawing_surface",
    )
    board.visual(
        Box((0.10, 0.70, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, -0.03)),
        material=edge_strip,
        name="rear_stiffener",
    )
    board.visual(
        Box((0.08, 0.70, 0.04)),
        origin=Origin(xyz=(0.40, 0.0, -0.025)),
        material=edge_strip,
        name="mid_stiffener",
    )
    board.visual(
        Box((0.03, 0.80, 0.02)),
        origin=Origin(xyz=(0.72, 0.0, 0.015)),
        material=aluminum,
        name="paper_lip",
    )
    board.visual(
        Cylinder(radius=0.026, length=0.04),
        origin=Origin(xyz=(0.04, -0.44, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_trunnion",
    )
    board.visual(
        Cylinder(radius=0.026, length=0.04),
        origin=Origin(xyz=(0.04, 0.44, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_trunnion",
    )

    crank_handle = model.part("crank_handle")
    crank_handle.inertial = Inertial.from_geometry(
        Box((0.18, 0.10, 0.20)),
        mass=0.7,
        origin=Origin(xyz=(0.08, 0.06, 0.05)),
    )
    crank_handle.visual(
        Cylinder(radius=0.012, length=0.08),
        origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="crank_shaft",
    )
    crank_handle.visual(
        Box((0.024, 0.02, 0.06)),
        origin=Origin(xyz=(0.0, 0.08, 0.03)),
        material=painted_steel,
        name="crank_web",
    )
    crank_handle.visual(
        Box((0.20, 0.02, 0.024)),
        origin=Origin(xyz=(0.10, 0.08, 0.03)),
        material=painted_steel,
        name="crank_arm",
    )
    crank_handle.visual(
        Cylinder(radius=0.013, length=0.12),
        origin=Origin(xyz=(0.20, 0.08, 0.05)),
        material=black_handle,
        name="hand_grip",
    )

    model.articulation(
        "base_to_upper_frame",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=0.28,
        ),
    )
    model.articulation(
        "upper_frame_to_board",
        ArticulationType.REVOLUTE,
        parent=upper_frame,
        child=board,
        origin=Origin(xyz=(0.18, 0.0, 0.04)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=-0.10,
            upper=1.35,
        ),
    )
    model.articulation(
        "upper_frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=upper_frame,
        child=crank_handle,
        origin=Origin(xyz=(0.22, 0.50, -0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_frame = object_model.get_part("upper_frame")
    board = object_model.get_part("board")
    crank_handle = object_model.get_part("crank_handle")

    lift = object_model.get_articulation("base_to_upper_frame")
    tilt = object_model.get_articulation("upper_frame_to_board")
    crank = object_model.get_articulation("upper_frame_to_crank")

    ctx.check(
        "lift joint is vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lift.axis}",
    )
    ctx.check(
        "board joint pitches about horizontal axis",
        tuple(tilt.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.expect_within(
        board,
        upper_frame,
        axes="y",
        margin=0.03,
        name="board stays between the side cheeks at rest",
    )

    rest_frame_pos = ctx.part_world_position(upper_frame)
    with ctx.pose({lift: lift.motion_limits.upper}):
        raised_frame_pos = ctx.part_world_position(upper_frame)
    ctx.check(
        "upper frame slides upward on the standard",
        rest_frame_pos is not None
        and raised_frame_pos is not None
        and raised_frame_pos[2] > rest_frame_pos[2] + 0.20,
        details=f"rest={rest_frame_pos}, raised={raised_frame_pos}",
    )

    rest_board_aabb = ctx.part_world_aabb(board)
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_board_aabb = ctx.part_world_aabb(board)
        ctx.expect_within(
            board,
            upper_frame,
            axes="y",
            margin=0.03,
            name="board remains between the cheeks when tilted",
        )
    ctx.check(
        "board front edge rises when the board tilts",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and tilted_board_aabb[1][2] > rest_board_aabb[1][2] + 0.30,
        details=f"rest={rest_board_aabb}, tilted={tilted_board_aabb}",
    )

    def _aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

    rest_grip = ctx.part_element_world_aabb(crank_handle, elem="hand_grip")
    with ctx.pose({crank: math.pi / 2.0}):
        turned_grip = ctx.part_element_world_aabb(crank_handle, elem="hand_grip")
    ctx.check(
        "crank grip sweeps a circular path",
        rest_grip is not None
        and turned_grip is not None
        and abs(_aabb_center(turned_grip)[0] - _aabb_center(rest_grip)[0]) > 0.05
        and abs(_aabb_center(turned_grip)[2] - _aabb_center(rest_grip)[2]) > 0.12,
        details=f"rest={rest_grip}, turned={turned_grip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
