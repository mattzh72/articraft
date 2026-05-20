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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _tube_between(part, name: str, start, end, radius: float, material: Material) -> None:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length tube {name}")
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _joint_ball(part, name: str, center, radius: float, material: Material) -> None:
    part.visual(
        Sphere(radius=radius),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _board_top_mesh() -> object:
    upper = [
        (-0.640, 0.165),
        (-0.540, 0.190),
        (-0.300, 0.205),
        (0.000, 0.198),
        (0.270, 0.165),
        (0.470, 0.105),
        (0.610, 0.035),
        (0.665, 0.000),
    ]
    lower = [(x, -y) for x, y in reversed(upper[:-1])]
    outline = upper + lower
    return cq.Workplane("XY").polyline(outline).close().extrude(0.045)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    fabric = model.material("blue_heat_resistant_fabric", rgba=(0.18, 0.43, 0.82, 1.0))
    edge = model.material("dark_blue_bound_edge", rgba=(0.05, 0.12, 0.22, 1.0))
    metal = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_metal = model.material("dark_painted_bracket", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.015, 0.014, 0.012, 1.0))
    rest_mat = model.material("perforated_iron_rest", rgba=(0.63, 0.65, 0.64, 1.0))

    board = model.part("board")
    board.visual(
        mesh_from_cadquery(_board_top_mesh(), "ironing_board_top", tolerance=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=fabric,
        name="fabric_board",
    )

    # A bound edge and shallow base bracket make the top read as a padded board
    # carried by a metal mounting frame rather than a plain slab.
    board.visual(
        Box((1.18, 0.035, 0.020)),
        origin=Origin(xyz=(-0.015, 0.198, 0.066)),
        material=edge,
        name="side_binding_0",
    )
    board.visual(
        Box((1.18, 0.035, 0.020)),
        origin=Origin(xyz=(-0.015, -0.198, 0.066)),
        material=edge,
        name="side_binding_1",
    )
    board.visual(
        Box((0.400, 0.235, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, 0.045)),
        material=dark_metal,
        name="base_bracket_plate",
    )
    board.visual(
        Box((0.280, 0.060, 0.052)),
        origin=Origin(xyz=(0.000, 0.235, 0.032)),
        material=dark_metal,
        name="hinge_cheek_0",
    )
    board.visual(
        Box((0.280, 0.060, 0.052)),
        origin=Origin(xyz=(0.000, -0.235, 0.032)),
        material=dark_metal,
        name="hinge_cheek_1",
    )
    _tube_between(board, "hinge_socket_0", (-0.070, 0.235, 0.018), (0.070, 0.235, 0.018), 0.025, dark_metal)
    _tube_between(board, "hinge_socket_1", (-0.070, -0.235, 0.018), (0.070, -0.235, 0.018), 0.025, dark_metal)

    # A small fixed end rest supports the blunt end and deliberately sits far
    # from the folding leg footprint for a lower, wider stance.
    _tube_between(board, "fixed_rest_top", (-0.540, -0.145, 0.058), (-0.540, 0.145, 0.058), 0.014, metal)
    _tube_between(board, "fixed_rest_leg_0", (-0.540, 0.145, 0.058), (-0.800, 0.300, -0.680), 0.014, metal)
    _tube_between(board, "fixed_rest_leg_1", (-0.540, -0.145, 0.058), (-0.800, -0.300, -0.680), 0.014, metal)
    _tube_between(board, "fixed_rest_foot", (-0.840, -0.340, -0.680), (-0.760, 0.340, -0.680), 0.017, metal)
    _joint_ball(board, "fixed_rest_joint_0", (-0.800, 0.300, -0.680), 0.024, metal)
    _joint_ball(board, "fixed_rest_joint_1", (-0.800, -0.300, -0.680), 0.024, metal)
    board.visual(
        Box((0.100, 0.060, 0.030)),
        origin=Origin(xyz=(-0.800, 0.365, -0.680)),
        material=rubber,
        name="fixed_foot_cap_0",
    )
    board.visual(
        Box((0.100, 0.060, 0.030)),
        origin=Origin(xyz=(-0.800, -0.365, -0.680)),
        material=rubber,
        name="fixed_foot_cap_1",
    )

    # Fixed iron rest plate on the blunt end of the board.
    board.visual(
        Box((0.220, 0.230, 0.010)),
        origin=Origin(xyz=(-0.520, 0.0, 0.104)),
        material=rest_mat,
        name="iron_rest_plate",
    )
    for index, y in enumerate((-0.075, 0.0, 0.075)):
        _tube_between(
            board,
            f"iron_rest_rail_{index}",
            (-0.610, y, 0.112),
            (-0.430, y, 0.112),
            0.006,
            metal,
        )

    leg = model.part("leg_frame")
    _tube_between(leg, "hinge_tube", (0.0, -0.180, 0.0), (0.0, 0.180, 0.0), 0.018, metal)
    _tube_between(leg, "side_leg_0", (0.0, 0.160, 0.0), (0.560, 0.325, -0.720), 0.018, metal)
    _tube_between(leg, "side_leg_1", (0.0, -0.160, 0.0), (0.560, -0.325, -0.720), 0.018, metal)
    _tube_between(leg, "floor_bar", (0.560, -0.355, -0.720), (0.560, 0.355, -0.720), 0.018, metal)
    _joint_ball(leg, "leg_floor_joint_0", (0.560, 0.325, -0.720), 0.028, metal)
    _joint_ball(leg, "leg_floor_joint_1", (0.560, -0.325, -0.720), 0.028, metal)
    leg.visual(
        Box((0.100, 0.065, 0.032)),
        origin=Origin(xyz=(0.560, 0.370, -0.720)),
        material=rubber,
        name="leg_foot_cap_0",
    )
    leg.visual(
        Box((0.100, 0.065, 0.032)),
        origin=Origin(xyz=(0.560, -0.370, -0.720)),
        material=rubber,
        name="leg_foot_cap_1",
    )

    model.articulation(
        "board_to_leg",
        ArticulationType.REVOLUTE,
        parent=board,
        child=leg,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=0.92),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board")
    leg = object_model.get_part("leg_frame")
    hinge = object_model.get_articulation("board_to_leg")

    ctx.check(
        "single folding leg hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"hinge type={hinge.articulation_type}",
    )
    ctx.expect_overlap(
        leg,
        board,
        axes="y",
        elem_a="hinge_tube",
        elem_b="base_bracket_plate",
        min_overlap=0.18,
        name="leg hinge centered under bracket",
    )
    ctx.expect_gap(
        board,
        leg,
        axis="z",
        positive_elem="base_bracket_plate",
        negative_elem="floor_bar",
        min_gap=0.55,
        name="open leg drops well below board",
    )
    ctx.expect_gap(
        leg,
        board,
        axis="x",
        positive_elem="floor_bar",
        negative_elem="fixed_rest_foot",
        min_gap=1.20,
        name="supports are spread far apart",
    )

    rest_z = ctx.part_element_world_aabb(leg, elem="floor_bar")[0][2]
    with ctx.pose({hinge: 0.92}):
        folded_z = ctx.part_element_world_aabb(leg, elem="floor_bar")[0][2]

    ctx.check(
        "positive hinge motion folds leg upward",
        rest_z is not None and folded_z is not None and folded_z > rest_z + 0.55,
        details=f"rest_z={rest_z}, folded_z={folded_z}",
    )

    return ctx.report()


object_model = build_object_model()
