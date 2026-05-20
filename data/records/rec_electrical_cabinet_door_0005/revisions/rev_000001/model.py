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

CABINET_W = 0.42
CABINET_H = 0.62
CABINET_D = 0.12
STEEL_T = 0.0016
FRAME_FACE = 0.018
MOUNTING_PLATE_W = 0.31
MOUNTING_PLATE_H = 0.51
MOUNTING_PLATE_T = 0.006
BREAKER_PLATE_W = 0.30
BREAKER_PLATE_H = 0.46
BREAKER_PLATE_T = 0.014
MODULE_W = 0.027
MODULE_H = 0.056
MODULE_T = 0.011
DOOR_W = 0.429
DOOR_H = 0.624
DOOR_SKIN_T = 0.0014
DOOR_RETURN_D = 0.014
DOOR_EDGE_FACE = 0.016
PANEL_OFFSET_X = 0.0045
PANEL_CENTER_Y = -0.0026
HINGE_AXIS_X = -CABINET_W / 2.0 - 0.0045
HINGE_AXIS_Y = CABINET_D + 0.0045
PIN_R = 0.0022
PIN_LEN = 0.045
KNUCKLE_R = 0.0040
KNUCKLE_LEN = 0.11
LOWER_PIN_Z = 0.13
UPPER_PIN_Z = CABINET_H - 0.13
HINGE_TAB_X = 0.0048
HINGE_TAB_Y = 0.005
HINGE_TAB_Z = 0.007


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="distribution_board")

    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.91, 0.92, 0.93, 1.0))
    mounting_plate = model.material("mounting_plate", rgba=(0.94, 0.94, 0.92, 1.0))
    breaker_white = model.material("breaker_white", rgba=(0.96, 0.96, 0.94, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_W, STEEL_T, CABINET_H)),
        origin=Origin(xyz=(0.0, STEEL_T / 2.0, CABINET_H / 2.0)),
        material=steel,
        name="back_panel",
    )
    cabinet.visual(
        Box((STEEL_T, CABINET_D - STEEL_T, CABINET_H)),
        origin=Origin(
            xyz=(
                -CABINET_W / 2.0 + STEEL_T / 2.0,
                STEEL_T + (CABINET_D - STEEL_T) / 2.0,
                CABINET_H / 2.0,
            )
        ),
        material=steel,
        name="left_side",
    )
    cabinet.visual(
        Box((STEEL_T, CABINET_D - STEEL_T, CABINET_H)),
        origin=Origin(
            xyz=(
                CABINET_W / 2.0 - STEEL_T / 2.0,
                STEEL_T + (CABINET_D - STEEL_T) / 2.0,
                CABINET_H / 2.0,
            )
        ),
        material=steel,
        name="right_side",
    )
    cabinet.visual(
        Box((CABINET_W - 2.0 * STEEL_T, CABINET_D - STEEL_T, STEEL_T)),
        origin=Origin(
            xyz=(0.0, STEEL_T + (CABINET_D - STEEL_T) / 2.0, CABINET_H - STEEL_T / 2.0)
        ),
        material=steel,
        name="top_side",
    )
    cabinet.visual(
        Box((CABINET_W - 2.0 * STEEL_T, CABINET_D - STEEL_T, STEEL_T)),
        origin=Origin(
            xyz=(0.0, STEEL_T + (CABINET_D - STEEL_T) / 2.0, STEEL_T / 2.0)
        ),
        material=steel,
        name="bottom_side",
    )
    cabinet.visual(
        Box((CABINET_W - 2.0 * FRAME_FACE, STEEL_T, FRAME_FACE)),
        origin=Origin(
            xyz=(0.0, CABINET_D - STEEL_T / 2.0, CABINET_H - FRAME_FACE / 2.0)
        ),
        material=steel,
        name="front_flange_top",
    )
    cabinet.visual(
        Box((CABINET_W - 2.0 * FRAME_FACE, STEEL_T, FRAME_FACE)),
        origin=Origin(xyz=(0.0, CABINET_D - STEEL_T / 2.0, FRAME_FACE / 2.0)),
        material=steel,
        name="front_flange_bottom",
    )
    cabinet.visual(
        Box((FRAME_FACE, STEEL_T, CABINET_H - 2.0 * FRAME_FACE)),
        origin=Origin(
            xyz=(-CABINET_W / 2.0 + FRAME_FACE / 2.0, CABINET_D - STEEL_T / 2.0, CABINET_H / 2.0)
        ),
        material=steel,
        name="front_flange_hinge",
    )
    cabinet.visual(
        Box((FRAME_FACE, STEEL_T, CABINET_H - 2.0 * FRAME_FACE)),
        origin=Origin(
            xyz=(CABINET_W / 2.0 - FRAME_FACE / 2.0, CABINET_D - STEEL_T / 2.0, CABINET_H / 2.0)
        ),
        material=steel,
        name="front_flange_latch",
    )
    cabinet.visual(
        Box((MOUNTING_PLATE_W, MOUNTING_PLATE_T, MOUNTING_PLATE_H)),
        origin=Origin(
            xyz=(0.0, STEEL_T + MOUNTING_PLATE_T / 2.0, CABINET_H / 2.0)
        ),
        material=mounting_plate,
        name="mounting_plate",
    )
    cabinet.visual(
        Box((BREAKER_PLATE_W, BREAKER_PLATE_T, BREAKER_PLATE_H)),
        origin=Origin(
            xyz=(
                0.0,
                STEEL_T + MOUNTING_PLATE_T + BREAKER_PLATE_T / 2.0,
                CABINET_H / 2.0,
            )
        ),
        material=breaker_white,
        name="breaker_plate",
    )

    row_zs = [0.115, 0.185, 0.255, 0.325, 0.395, 0.465]
    col_xs = [-0.034, 0.034]
    for index, (row_z, col_x) in enumerate(
        [(z, x) for z in row_zs for x in col_xs],
        start=1,
    ):
        cabinet.visual(
            Box((MODULE_W, MODULE_T, MODULE_H)),
            origin=Origin(
                xyz=(
                    col_x,
                    STEEL_T
                    + MOUNTING_PLATE_T
                    + BREAKER_PLATE_T
                    + MODULE_T / 2.0,
                    row_z,
                )
            ),
            material=dark_trim if index % 4 == 0 else breaker_white,
            name=f"breaker_module_{index}",
        )

    cabinet.visual(
        Box((HINGE_TAB_X, HINGE_TAB_Y, HINGE_TAB_Z)),
        origin=Origin(
            xyz=(
                -CABINET_W / 2.0 - HINGE_TAB_X / 2.0,
                CABINET_D + HINGE_TAB_Y / 2.0,
                LOWER_PIN_Z - PIN_LEN / 2.0 + HINGE_TAB_Z / 2.0,
            )
        ),
        material=steel,
        name="hinge_tab_lower",
    )
    cabinet.visual(
        Box((HINGE_TAB_X, HINGE_TAB_Y, HINGE_TAB_Z)),
        origin=Origin(
            xyz=(
                -CABINET_W / 2.0 - HINGE_TAB_X / 2.0,
                CABINET_D + HINGE_TAB_Y / 2.0,
                UPPER_PIN_Z + PIN_LEN / 2.0 - HINGE_TAB_Z / 2.0,
            )
        ),
        material=steel,
        name="hinge_tab_upper",
    )
    cabinet.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, LOWER_PIN_Z)),
        material=steel,
        name="hinge_pin_lower",
    )
    cabinet.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, UPPER_PIN_Z)),
        material=steel,
        name="hinge_pin_upper",
    )

    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_W, CABINET_D, CABINET_H)),
        mass=8.5,
        origin=Origin(xyz=(0.0, CABINET_D / 2.0, CABINET_H / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_SKIN_T, DOOR_H)),
        origin=Origin(xyz=(PANEL_OFFSET_X + DOOR_W / 2.0, PANEL_CENTER_Y, 0.0)),
        material=painted_steel,
        name="door_skin",
    )
    door.visual(
        Box((DOOR_W - 2.0 * DOOR_EDGE_FACE, DOOR_RETURN_D, STEEL_T)),
        origin=Origin(
            xyz=(
                PANEL_OFFSET_X + DOOR_W / 2.0,
                PANEL_CENTER_Y - DOOR_SKIN_T / 2.0 - DOOR_RETURN_D / 2.0,
                DOOR_H / 2.0 - STEEL_T / 2.0,
            )
        ),
        material=painted_steel,
        name="return_top",
    )
    door.visual(
        Box((DOOR_W - 2.0 * DOOR_EDGE_FACE, DOOR_RETURN_D, STEEL_T)),
        origin=Origin(
            xyz=(
                PANEL_OFFSET_X + DOOR_W / 2.0,
                PANEL_CENTER_Y - DOOR_SKIN_T / 2.0 - DOOR_RETURN_D / 2.0,
                -DOOR_H / 2.0 + STEEL_T / 2.0,
            )
        ),
        material=painted_steel,
        name="return_bottom",
    )
    door.visual(
        Box((STEEL_T, DOOR_RETURN_D, DOOR_H - 2.0 * DOOR_EDGE_FACE)),
        origin=Origin(
            xyz=(
                PANEL_OFFSET_X + DOOR_W - STEEL_T / 2.0,
                PANEL_CENTER_Y - DOOR_SKIN_T / 2.0 - DOOR_RETURN_D / 2.0,
                0.0,
            )
        ),
        material=painted_steel,
        name="return_latch",
    )
    door.visual(
        Box((0.018, 0.008, 0.09)),
        origin=Origin(xyz=(KNUCKLE_R + 0.009, -0.0002, -0.14)),
        material=painted_steel,
        name="hinge_leaf_lower",
    )
    door.visual(
        Box((0.018, 0.008, 0.09)),
        origin=Origin(xyz=(KNUCKLE_R + 0.009, -0.0002, 0.14)),
        material=painted_steel,
        name="hinge_leaf_upper",
    )
    door.visual(
        Cylinder(radius=KNUCKLE_R, length=KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, -0.14)),
        material=painted_steel,
        name="hinge_knuckle_lower",
    )
    door.visual(
        Cylinder(radius=KNUCKLE_R, length=KNUCKLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=painted_steel,
        name="hinge_knuckle_upper",
    )
    door.visual(
        Box((0.024, 0.008, 0.12)),
        origin=Origin(
            xyz=(
                PANEL_OFFSET_X + DOOR_W - 0.03,
                PANEL_CENTER_Y + DOOR_SKIN_T / 2.0 + 0.004,
                0.0,
            )
        ),
        material=dark_trim,
        name="handle",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, 0.02, DOOR_H)),
        mass=2.9,
        origin=Origin(xyz=(PANEL_OFFSET_X + DOOR_W / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, CABINET_H / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")
    back_panel = cabinet.get_visual("back_panel")
    left_side = cabinet.get_visual("left_side")
    front_flange_top = cabinet.get_visual("front_flange_top")
    breaker_plate = cabinet.get_visual("breaker_plate")
    hinge_pin_lower = cabinet.get_visual("hinge_pin_lower")
    hinge_pin_upper = cabinet.get_visual("hinge_pin_upper")
    door_skin = door.get_visual("door_skin")
    hinge_leaf_upper = door.get_visual("hinge_leaf_upper")
    hinge_knuckle_lower = door.get_visual("hinge_knuckle_lower")
    hinge_knuckle_upper = door.get_visual("hinge_knuckle_upper")
    handle = door.get_visual("handle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=hinge_knuckle_lower,
        elem_b=hinge_pin_lower,
        reason="lower hinge pin runs inside the rolled knuckle sleeve",
    )
    ctx.allow_overlap(
        door,
        cabinet,
        elem_a=hinge_knuckle_upper,
        elem_b=hinge_pin_upper,
        reason="upper hinge pin runs inside the rolled knuckle sleeve",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_origin_far_from_geometry(
        tol=0.01,
        name="hinge_axis_close_to_physical_hardware",
    )
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=96,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
    )

    limits = door_hinge.motion_limits
    ctx.check(
        "door_hinge_axis_vertical",
        tuple(round(value, 6) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical hinge axis, got {door_hinge.axis}",
    )
    ctx.check(
        "door_hinge_limit_is_180_degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-6
        and abs(limits.upper - math.pi) < 1e-6,
        details=f"expected limits [0, pi], got {limits}",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        min_gap=0.001,
        max_gap=0.003,
        positive_elem=door_skin,
        negative_elem=front_flange_top,
        name="door_skin_sits_just_proud_of_frame",
    )
    ctx.expect_within(
        cabinet,
        door,
        axes="xz",
        margin=0.02,
        inner_elem=breaker_plate,
        outer_elem=door_skin,
        name="door_covers_breaker_field",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        min_overlap=0.28,
        elem_a=door_skin,
        elem_b=back_panel,
        name="door_aligned_with_cabinet_opening",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="y",
        min_gap=0.11,
        positive_elem=front_flange_top,
        negative_elem=back_panel,
        name="steel_box_is_recessed",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=hinge_knuckle_upper,
        elem_b=hinge_pin_upper,
        name="upper_hinge_knuckle_captures_pin",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="x",
        min_gap=0.001,
        max_gap=0.005,
        positive_elem=left_side,
        negative_elem=hinge_pin_upper,
        name="hinge_pins_sit_outboard_of_side_jamb",
    )
    ctx.expect_contact(
        door,
        door,
        elem_a=handle,
        elem_b=door_skin,
        name="handle_is_surface_mounted_to_door_skin",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({door_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_lower_no_floating")
        with ctx.pose({door_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="door_hinge_upper_no_floating")

    with ctx.pose({door_hinge: math.pi / 2.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="x",
            min_gap=0.001,
            max_gap=0.01,
            positive_elem=left_side,
            negative_elem=door_skin,
            name="half_open_door_clears_side_jamb",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="z",
            min_overlap=0.60,
            elem_a=door_skin,
            elem_b=back_panel,
            name="half_open_door_keeps_vertical_alignment",
        )

    with ctx.pose({door_hinge: math.pi}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="x",
            min_gap=0.006,
            max_gap=0.02,
            positive_elem=left_side,
            negative_elem=door_skin,
            name="door_folds_fully_back_at_180_degrees",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="z",
            min_overlap=0.60,
            elem_a=door_skin,
            elem_b=back_panel,
            name="open_door_stays_aligned_with_cabinet_height",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
