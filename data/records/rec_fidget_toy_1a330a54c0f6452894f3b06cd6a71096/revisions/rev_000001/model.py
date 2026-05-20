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


CUBE_SIZE = 0.038
SEAM_GAP = 0.0012
HINGE_AXIS_OFFSET = 0.0026
HINGE_LEAF_THICKNESS = 0.0024
HINGE_LEAF_WIDTH = 0.022
HINGE_LEAF_DEPTH = 0.009
HINGE_PIN_RADIUS = 0.0018
HINGE_PIN_LENGTH = 0.018
PITCH = CUBE_SIZE + SEAM_GAP
ROOT_CENTER = (-1.5 * PITCH, 0.5 * PITCH, 0.0)


def _child_center_from_joint(
    dir_x: int,
    dir_y: int,
    z_side: int,
) -> tuple[float, float, float]:
    half_span = CUBE_SIZE * 0.5 + SEAM_GAP * 0.5
    vertical = CUBE_SIZE * 0.5 + HINGE_AXIS_OFFSET
    return (
        dir_x * half_span,
        dir_y * half_span,
        -z_side * vertical,
    )


def _joint_origin_from_cube_center(
    cube_center: tuple[float, float, float],
    dir_x: int,
    dir_y: int,
    z_side: int,
) -> Origin:
    half_span = CUBE_SIZE * 0.5 + SEAM_GAP * 0.5
    vertical = CUBE_SIZE * 0.5 + HINGE_AXIS_OFFSET
    return Origin(
        xyz=(
            cube_center[0] + dir_x * half_span,
            cube_center[1] + dir_y * half_span,
            cube_center[2] + z_side * vertical,
        )
    )


def _hinge_axis(dir_x: int, dir_y: int, z_side: int) -> tuple[float, float, float]:
    if dir_x != 0:
        return (0.0, float(-dir_x * z_side), 0.0)
    return (float(dir_y * z_side), 0.0, 0.0)


def _add_cube_visuals(part, cube_center: tuple[float, float, float], cube_material) -> None:
    part.visual(
        Box((CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)),
        origin=Origin(xyz=cube_center),
        material=cube_material,
        name="cube_body",
    )


def _add_hinge_leaf(
    part,
    cube_center: tuple[float, float, float],
    *,
    edge_dir: tuple[int, int],
    z_side: int,
    material,
    name: str,
) -> None:
    dir_x, dir_y = edge_dir
    leaf_z = cube_center[2] + z_side * (
        CUBE_SIZE * 0.5 + HINGE_LEAF_THICKNESS * 0.5 - 0.0004
    )
    if dir_x != 0:
        part.visual(
            Box((HINGE_LEAF_DEPTH, HINGE_LEAF_WIDTH, HINGE_LEAF_THICKNESS)),
            origin=Origin(
                xyz=(
                    cube_center[0] + dir_x * (CUBE_SIZE * 0.5 - HINGE_LEAF_DEPTH * 0.5),
                    cube_center[1],
                    leaf_z,
                )
            ),
            material=material,
            name=name,
        )
    else:
        part.visual(
            Box((HINGE_LEAF_WIDTH, HINGE_LEAF_DEPTH, HINGE_LEAF_THICKNESS)),
            origin=Origin(
                xyz=(
                    cube_center[0],
                    cube_center[1] + dir_y * (CUBE_SIZE * 0.5 - HINGE_LEAF_DEPTH * 0.5),
                    leaf_z,
                )
            ),
            material=material,
            name=name,
        )


def _add_hinge_pin(
    part,
    cube_center: tuple[float, float, float],
    *,
    dir_x: int,
    dir_y: int,
    z_side: int,
    material,
    name: str,
) -> None:
    axis_origin = _joint_origin_from_cube_center(cube_center, dir_x, dir_y, z_side).xyz
    if dir_x != 0:
        pin_rpy = (math.pi * 0.5, 0.0, 0.0)
    else:
        pin_rpy = (0.0, math.pi * 0.5, 0.0)
    part.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=HINGE_PIN_LENGTH),
        origin=Origin(xyz=axis_origin, rpy=pin_rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="infinity_cube")

    hinge_black = model.material("hinge_black", rgba=(0.10, 0.10, 0.12, 1.0))
    cube_materials = [
        model.material("cube_red", rgba=(0.83, 0.22, 0.18, 1.0)),
        model.material("cube_orange", rgba=(0.91, 0.49, 0.16, 1.0)),
        model.material("cube_yellow", rgba=(0.93, 0.76, 0.20, 1.0)),
        model.material("cube_green", rgba=(0.35, 0.69, 0.30, 1.0)),
        model.material("cube_teal", rgba=(0.18, 0.66, 0.64, 1.0)),
        model.material("cube_blue", rgba=(0.22, 0.46, 0.82, 1.0)),
        model.material("cube_purple", rgba=(0.46, 0.32, 0.72, 1.0)),
        model.material("cube_magenta", rgba=(0.78, 0.28, 0.58, 1.0)),
    ]

    joint_specs = [
        ("cube_0", "cube_1", 1, 0, 1),
        ("cube_1", "cube_2", 1, 0, -1),
        ("cube_2", "cube_3", 1, 0, 1),
        ("cube_3", "cube_4", 0, -1, -1),
        ("cube_4", "cube_5", -1, 0, 1),
        ("cube_5", "cube_6", -1, 0, -1),
        ("cube_6", "cube_7", -1, 0, 1),
    ]

    incoming = {child: (dir_x, dir_y, z_side) for _, child, dir_x, dir_y, z_side in joint_specs}
    centers: dict[str, tuple[float, float, float]] = {"cube_0": ROOT_CENTER}
    parts = {}

    for index in range(8):
        name = f"cube_{index}"
        part = model.part(name)
        if index == 0:
            cube_center = ROOT_CENTER
        else:
            dir_x, dir_y, z_side = incoming[name]
            cube_center = _child_center_from_joint(dir_x, dir_y, z_side)
        centers[name] = cube_center
        _add_cube_visuals(part, cube_center, cube_materials[index])
        part.inertial = Inertial.from_geometry(
            Box((CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)),
            mass=0.055,
            origin=Origin(xyz=cube_center),
        )
        parts[name] = part

    for parent_name, child_name, dir_x, dir_y, z_side in joint_specs:
        parent = parts[parent_name]
        child = parts[child_name]

        _add_hinge_leaf(
            parent,
            centers[parent_name],
            edge_dir=(dir_x, dir_y),
            z_side=z_side,
            material=hinge_black,
            name=f"{parent_name}_to_{child_name}_parent_leaf",
        )
        _add_hinge_leaf(
            child,
            centers[child_name],
            edge_dir=(-dir_x, -dir_y),
            z_side=z_side,
            material=hinge_black,
            name=f"{parent_name}_to_{child_name}_child_leaf",
        )
        _add_hinge_pin(
            parent,
            centers[parent_name],
            dir_x=dir_x,
            dir_y=dir_y,
            z_side=z_side,
            material=hinge_black,
            name=f"{parent_name}_to_{child_name}_pin",
        )

        model.articulation(
            f"{parent_name}_to_{child_name}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=_joint_origin_from_cube_center(centers[parent_name], dir_x, dir_y, z_side),
            axis=_hinge_axis(dir_x, dir_y, z_side),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=6.0,
                lower=0.0,
                upper=math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joint_specs = [
        ("cube_0", "cube_1", 1, 0),
        ("cube_1", "cube_2", 1, 0),
        ("cube_2", "cube_3", 1, 0),
        ("cube_3", "cube_4", 0, -1),
        ("cube_4", "cube_5", -1, 0),
        ("cube_5", "cube_6", -1, 0),
        ("cube_6", "cube_7", -1, 0),
    ]

    cubes = [object_model.get_part(f"cube_{index}") for index in range(8)]
    for index, cube in enumerate(cubes):
        ctx.check(f"cube_{index} exists", cube is not None, details=f"missing cube_{index}")

    for parent_name, child_name, dir_x, dir_y in joint_specs:
        parent = object_model.get_part(parent_name)
        child = object_model.get_part(child_name)
        if dir_x != 0:
            positive = child if dir_x > 0 else parent
            negative = parent if dir_x > 0 else child
            ctx.expect_gap(
                positive,
                negative,
                axis="x",
                min_gap=SEAM_GAP - 0.0003,
                max_gap=SEAM_GAP + 0.0003,
                positive_elem="cube_body",
                negative_elem="cube_body",
                name=f"{parent_name} and {child_name} keep the hinge seam along x",
            )
            ctx.expect_overlap(
                parent,
                child,
                axes="yz",
                min_overlap=CUBE_SIZE - 0.003,
                elem_a="cube_body",
                elem_b="cube_body",
                name=f"{parent_name} and {child_name} stay aligned on yz",
            )
        else:
            positive = child if dir_y > 0 else parent
            negative = parent if dir_y > 0 else child
            ctx.expect_gap(
                positive,
                negative,
                axis="y",
                min_gap=SEAM_GAP - 0.0003,
                max_gap=SEAM_GAP + 0.0003,
                positive_elem="cube_body",
                negative_elem="cube_body",
                name=f"{parent_name} and {child_name} keep the hinge seam along y",
            )
            ctx.expect_overlap(
                parent,
                child,
                axes="xz",
                min_overlap=CUBE_SIZE - 0.003,
                elem_a="cube_body",
                elem_b="cube_body",
                name=f"{parent_name} and {child_name} stay aligned on xz",
            )

    first_joint = object_model.get_articulation("cube_0_to_cube_1")
    second_joint = object_model.get_articulation("cube_1_to_cube_2")

    def _aabb_center_z(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    cube_1 = object_model.get_part("cube_1")
    cube_2 = object_model.get_part("cube_2")

    rest_cube_1_z = _aabb_center_z(cube_1)
    with ctx.pose({first_joint: math.pi * 0.5}):
        raised_cube_1_z = _aabb_center_z(cube_1)
    ctx.check(
        "first hinge folds cube_1 upward",
        rest_cube_1_z is not None
        and raised_cube_1_z is not None
        and raised_cube_1_z > rest_cube_1_z + 0.010,
        details=f"rest_z={rest_cube_1_z}, posed_z={raised_cube_1_z}",
    )

    rest_cube_2_z = _aabb_center_z(cube_2)
    with ctx.pose({second_joint: math.pi * 0.5}):
        lowered_cube_2_z = _aabb_center_z(cube_2)
    ctx.check(
        "second hinge folds cube_2 to the opposite side",
        rest_cube_2_z is not None
        and lowered_cube_2_z is not None
        and lowered_cube_2_z < rest_cube_2_z - 0.010,
        details=f"rest_z={rest_cube_2_z}, posed_z={lowered_cube_2_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
