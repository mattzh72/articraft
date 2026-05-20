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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broom_cupboard")

    case_width = 0.56
    case_depth = 0.46
    case_height = 1.95
    side_thickness = 0.018
    top_bottom_thickness = 0.018
    back_thickness = 0.006

    door_thickness = 0.020
    door_bottom_gap = 0.020
    door_top_gap = 0.020
    door_height = case_height - door_bottom_gap - door_top_gap

    hinge_axis_x = -case_width / 2.0 + 0.022
    hinge_axis_y = case_depth / 2.0 + door_thickness / 2.0
    door_total_width = 0.536
    door_panel_offset_x = 0.012
    door_panel_width = door_total_width - door_panel_offset_x

    upper_hinge_z = case_height - 0.24
    lower_hinge_z = 0.24
    hinge_leaf_length = 0.160
    hinge_leaf_depth = 0.010
    hinge_leaf_width = 0.014
    hinge_barrel_radius = 0.005
    hinge_barrel_length = 0.120

    latch_pivot_x = door_total_width - 0.044
    latch_pivot_y = door_thickness / 2.0 + 0.004
    latch_pivot_z = door_height * 0.53
    latch_arm_length = 0.060

    cabinet_paint = model.material("cabinet_paint", rgba=(0.82, 0.83, 0.80, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.58, 0.59, 0.61, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.20, 0.21, 0.22, 1.0))
    back_panel_color = model.material("back_panel", rgba=(0.74, 0.75, 0.72, 1.0))

    case = model.part("cabinet_case")
    case.visual(
        Box((side_thickness, case_depth, case_height)),
        origin=Origin(xyz=(-case_width / 2.0 + side_thickness / 2.0, 0.0, case_height / 2.0)),
        material=cabinet_paint,
        name="left_side_panel",
    )
    case.visual(
        Box((side_thickness, case_depth, case_height)),
        origin=Origin(xyz=(case_width / 2.0 - side_thickness / 2.0, 0.0, case_height / 2.0)),
        material=cabinet_paint,
        name="right_side_panel",
    )
    case.visual(
        Box((case_width - 2.0 * side_thickness, case_depth, top_bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                top_bottom_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="bottom_panel",
    )
    case.visual(
        Box((case_width - 2.0 * side_thickness, case_depth, top_bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                case_height - top_bottom_thickness / 2.0,
            )
        ),
        material=cabinet_paint,
        name="top_panel",
    )
    case.visual(
        Box((case_width - 2.0 * side_thickness, back_thickness, case_height - 2.0 * top_bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -case_depth / 2.0 + back_thickness / 2.0,
                case_height / 2.0,
            )
        ),
        material=back_panel_color,
        name="back_panel",
    )
    case.visual(
        Box((case_width - 2.0 * side_thickness, 0.140, 0.080)),
        origin=Origin(
            xyz=(
                0.0,
                -case_depth / 2.0 + 0.070,
                0.040,
            )
        ),
        material=cabinet_paint,
        name="rear_plinth",
    )

    for prefix, hinge_z in (("upper", upper_hinge_z), ("lower", lower_hinge_z)):
        case.visual(
            Box((hinge_leaf_width, hinge_leaf_depth, hinge_leaf_length)),
            origin=Origin(
                xyz=(
                    -case_width / 2.0 + hinge_leaf_width / 2.0,
                    case_depth / 2.0 + hinge_leaf_depth / 2.0,
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"case_{prefix}_hinge_leaf",
        )
        case.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(
                    hinge_axis_x - hinge_barrel_radius * 2.0,
                    hinge_axis_y,
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"case_{prefix}_hinge_barrel",
        )

    door = model.part("door")
    door.visual(
        Box((door_panel_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                door_panel_offset_x + door_panel_width / 2.0,
                0.0,
                door_height / 2.0,
            )
        ),
        material=cabinet_paint,
        name="door_slab",
    )
    door.visual(
        Box((door_panel_width - 0.080, 0.008, door_height - 0.180)),
        origin=Origin(
            xyz=(
                door_panel_offset_x + door_panel_width / 2.0,
                -0.006,
                door_height / 2.0,
            )
        ),
        material=back_panel_color,
        name="door_recess_panel",
    )

    for prefix, hinge_z in (("upper", upper_hinge_z - door_bottom_gap), ("lower", lower_hinge_z - door_bottom_gap)):
        door.visual(
            Box((0.016, hinge_leaf_depth, hinge_leaf_length)),
            origin=Origin(
                xyz=(
                    0.008,
                    -hinge_leaf_depth / 2.0,
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"door_{prefix}_hinge_leaf",
        )
        door.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    hinge_z,
                )
            ),
            material=hinge_metal,
            name=f"door_{prefix}_hinge_barrel",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="latch_boss",
    )
    latch.visual(
        Box((0.012, 0.006, latch_arm_length)),
        origin=Origin(xyz=(0.0, 0.0, latch_arm_length / 2.0)),
        material=latch_metal,
        name="latch_arm",
    )

    model.articulation(
        "case_to_door",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, door_bottom_gap)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.8,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(latch_pivot_x, latch_pivot_y, latch_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
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

    case = object_model.get_part("cabinet_case")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_joint = object_model.get_articulation("case_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    ctx.check(
        "cupboard parts exist",
        all(part is not None for part in (case, door, latch)),
        details=f"parts={[part.name for part in (case, door, latch)]}",
    )
    ctx.check(
        "door hinge is vertical",
        tuple(round(value, 6) for value in door_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.check(
        "latch pivots on a short door-normal axis",
        tuple(round(value, 6) for value in latch_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={latch_joint.axis}",
    )

    closed_door_aabb = None
    opened_door_aabb = None
    parked_latch_aabb = None
    turned_latch_aabb = None

    with ctx.pose({door_joint: 0.0, latch_joint: 0.0}):
        ctx.expect_overlap(
            door,
            case,
            axes="xz",
            elem_a="door_slab",
            min_overlap=0.45,
            name="door covers the cupboard opening in the closed pose",
        )
        ctx.expect_gap(
            door,
            case,
            axis="y",
            positive_elem="door_slab",
            negative_elem="left_side_panel",
            min_gap=0.0,
            max_gap=0.001,
            name="door sits flush with the cabinet front plane",
        )
        ctx.expect_contact(
            latch,
            door,
            elem_a="latch_boss",
            elem_b="door_slab",
            contact_tol=1e-6,
            name="latch pivot boss is mounted to the door face",
        )
        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")
        parked_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_arm")

    with ctx.pose({door_joint: 1.25, latch_joint: 0.0}):
        opened_door_aabb = ctx.part_element_world_aabb(door, elem="door_slab")

    ctx.check(
        "door swings outward on the side hinges",
        (
            closed_door_aabb is not None
            and opened_door_aabb is not None
            and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.20
        ),
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )

    with ctx.pose({door_joint: 0.0, latch_joint: 1.15}):
        turned_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_arm")

    ctx.check(
        "latch rotates away from its parked vertical position",
        (
            parked_latch_aabb is not None
            and turned_latch_aabb is not None
            and turned_latch_aabb[1][0] > parked_latch_aabb[1][0] + 0.025
            and turned_latch_aabb[1][2] < parked_latch_aabb[1][2] - 0.010
        ),
        details=f"parked={parked_latch_aabb}, turned={turned_latch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
