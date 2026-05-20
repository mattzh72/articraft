from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 1.20
CABINET_DEPTH = 0.60
CABINET_HEIGHT = 0.94
TOP_THICKNESS = 0.03
TOP_FRONT_OVERHANG = 0.01
TOP_SIDE_OVERHANG = 0.01
SIDE_THICKNESS = 0.018
BACK_THICKNESS = 0.009
FACE_FRAME_THICKNESS = 0.018
CENTER_DIVIDER_THICKNESS = 0.018
PLINTH_HEIGHT = 0.08
PLINTH_SETBACK = 0.04
BOTTOM_PANEL_THICKNESS = 0.018

DRAWER_FACE_WIDTH = 0.551
DRAWER_FACE_HEIGHT = 0.194
DRAWER_BODY_WIDTH = 0.53
DRAWER_BODY_DEPTH = 0.47
DRAWER_BODY_HEIGHT = 0.14
DRAWER_BOTTOM_THICKNESS = 0.012
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_PULL_LENGTH = 0.16
DRAWER_TRAVEL = 0.32

WORK_SHELF_FACE_HEIGHT = 0.051
WORK_SHELF_FACE_WIDTH = 1.14
WORK_SHELF_DEPTH = 0.43
WORK_SHELF_WIDTH = 1.10
WORK_SHELF_THICKNESS = 0.022
WORK_SHELF_TRAVEL = 0.28
WORK_SHELF_SLOT_HEIGHT = 0.055

DRAWER_ROW_BOTTOMS = (0.116, 0.324, 0.532)
WORK_SHELF_BOTTOM = 0.826


def add_box(part, name, size, xyz, material) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def add_drawer_geometry(part, face_material, box_material, handle_material) -> None:
    add_box(
        part,
        "front_face",
        (0.02, DRAWER_FACE_WIDTH, DRAWER_FACE_HEIGHT),
        (0.01, 0.0, DRAWER_FACE_HEIGHT / 2.0),
        face_material,
    )
    add_box(
        part,
        "bottom",
        (DRAWER_BODY_DEPTH, DRAWER_BODY_WIDTH, DRAWER_BOTTOM_THICKNESS),
        (
            -DRAWER_BODY_DEPTH / 2.0,
            0.0,
            0.004 + DRAWER_BOTTOM_THICKNESS / 2.0,
        ),
        box_material,
    )
    side_center_z = 0.004 + DRAWER_BOTTOM_THICKNESS + DRAWER_BODY_HEIGHT / 2.0
    add_box(
        part,
        "side_0",
        (DRAWER_BODY_DEPTH, DRAWER_SIDE_THICKNESS, DRAWER_BODY_HEIGHT),
        (
            -DRAWER_BODY_DEPTH / 2.0,
            -(DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0),
            side_center_z,
        ),
        box_material,
    )
    add_box(
        part,
        "side_1",
        (DRAWER_BODY_DEPTH, DRAWER_SIDE_THICKNESS, DRAWER_BODY_HEIGHT),
        (
            -DRAWER_BODY_DEPTH / 2.0,
            DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_THICKNESS / 2.0,
            side_center_z,
        ),
        box_material,
    )
    add_box(
        part,
        "back",
        (DRAWER_SIDE_THICKNESS, DRAWER_BODY_WIDTH, DRAWER_BODY_HEIGHT),
        (
            -DRAWER_BODY_DEPTH + DRAWER_SIDE_THICKNESS / 2.0,
            0.0,
            side_center_z,
        ),
        box_material,
    )

    pull_center_z = DRAWER_FACE_HEIGHT * 0.58
    add_box(
        part,
        "pull_post_0",
        (0.012, 0.016, 0.022),
        (0.026, -0.05, pull_center_z),
        handle_material,
    )
    add_box(
        part,
        "pull_post_1",
        (0.012, 0.016, 0.022),
        (0.026, 0.05, pull_center_z),
        handle_material,
    )
    add_box(
        part,
        "pull_bar",
        (0.012, DRAWER_PULL_LENGTH, 0.02),
        (0.038, 0.0, pull_center_z),
        handle_material,
    )


def add_work_shelf_geometry(part, face_material, top_material, handle_material) -> None:
    add_box(
        part,
        "front_face",
        (0.02, WORK_SHELF_FACE_WIDTH, WORK_SHELF_FACE_HEIGHT),
        (0.01, 0.0, WORK_SHELF_FACE_HEIGHT / 2.0),
        face_material,
    )
    add_box(
        part,
        "worktop",
        (WORK_SHELF_DEPTH, WORK_SHELF_WIDTH, WORK_SHELF_THICKNESS),
        (
            -WORK_SHELF_DEPTH / 2.0,
            0.0,
            0.01 + WORK_SHELF_THICKNESS / 2.0,
        ),
        top_material,
    )
    add_box(
        part,
        "stiffener_0",
        (WORK_SHELF_DEPTH - 0.04, 0.02, 0.03),
        (-WORK_SHELF_DEPTH / 2.0 + 0.02, -(WORK_SHELF_WIDTH / 2.0 - 0.01), 0.015),
        face_material,
    )
    add_box(
        part,
        "stiffener_1",
        (WORK_SHELF_DEPTH - 0.04, 0.02, 0.03),
        (-WORK_SHELF_DEPTH / 2.0 + 0.02, WORK_SHELF_WIDTH / 2.0 - 0.01, 0.015),
        face_material,
    )
    pull_center_z = WORK_SHELF_FACE_HEIGHT * 0.55
    add_box(
        part,
        "pull_post_0",
        (0.012, 0.018, 0.022),
        (0.026, -0.16, pull_center_z),
        handle_material,
    )
    add_box(
        part,
        "pull_post_1",
        (0.012, 0.018, 0.022),
        (0.026, 0.16, pull_center_z),
        handle_material,
    )
    add_box(
        part,
        "pull_bar",
        (0.012, 0.38, 0.02),
        (0.038, 0.0, pull_center_z),
        handle_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="craft_drawer_cabinet")

    carcass_mat = model.material("carcass", rgba=(0.86, 0.82, 0.74, 1.0))
    face_mat = model.material("drawer_face", rgba=(0.90, 0.86, 0.79, 1.0))
    box_mat = model.material("drawer_box", rgba=(0.77, 0.73, 0.66, 1.0))
    top_mat = model.material("work_top", rgba=(0.68, 0.51, 0.33, 1.0))
    runner_mat = model.material("runner", rgba=(0.63, 0.66, 0.70, 1.0))
    handle_mat = model.material("handle", rgba=(0.26, 0.29, 0.32, 1.0))

    carcass = model.part("carcass")

    top_width = CABINET_WIDTH + 2.0 * TOP_SIDE_OVERHANG
    top_depth = CABINET_DEPTH + TOP_FRONT_OVERHANG
    add_box(
        carcass,
        "top",
        (top_depth, top_width, TOP_THICKNESS),
        (
            (-CABINET_DEPTH + TOP_FRONT_OVERHANG) / 2.0,
            0.0,
            CABINET_HEIGHT - TOP_THICKNESS / 2.0,
        ),
        top_mat,
    )

    side_height = CABINET_HEIGHT - TOP_THICKNESS - PLINTH_HEIGHT
    side_center_z = PLINTH_HEIGHT + side_height / 2.0
    add_box(
        carcass,
        "side_0",
        (CABINET_DEPTH, SIDE_THICKNESS, side_height),
        (-CABINET_DEPTH / 2.0, -(CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0), side_center_z),
        carcass_mat,
    )
    add_box(
        carcass,
        "side_1",
        (CABINET_DEPTH, SIDE_THICKNESS, side_height),
        (-CABINET_DEPTH / 2.0, CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, side_center_z),
        carcass_mat,
    )
    add_box(
        carcass,
        "back",
        (BACK_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, side_height),
        (-CABINET_DEPTH + BACK_THICKNESS / 2.0, 0.0, side_center_z),
        carcass_mat,
    )

    bottom_depth = CABINET_DEPTH - BACK_THICKNESS
    add_box(
        carcass,
        "bottom",
        (bottom_depth, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BOTTOM_PANEL_THICKNESS),
        (-bottom_depth / 2.0, 0.0, PLINTH_HEIGHT + BOTTOM_PANEL_THICKNESS / 2.0),
        carcass_mat,
    )

    plinth_side_depth = CABINET_DEPTH - PLINTH_SETBACK
    plinth_side_center_x = -(PLINTH_SETBACK + plinth_side_depth / 2.0)
    add_box(
        carcass,
        "plinth_side_0",
        (plinth_side_depth, SIDE_THICKNESS, PLINTH_HEIGHT),
        (plinth_side_center_x, -(CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0), PLINTH_HEIGHT / 2.0),
        carcass_mat,
    )
    add_box(
        carcass,
        "plinth_side_1",
        (plinth_side_depth, SIDE_THICKNESS, PLINTH_HEIGHT),
        (plinth_side_center_x, CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, PLINTH_HEIGHT / 2.0),
        carcass_mat,
    )
    add_box(
        carcass,
        "plinth_front",
        (FACE_FRAME_THICKNESS, CABINET_WIDTH - 2.0 * PLINTH_SETBACK, PLINTH_HEIGHT),
        (-PLINTH_SETBACK + FACE_FRAME_THICKNESS / 2.0, 0.0, PLINTH_HEIGHT / 2.0),
        carcass_mat,
    )
    add_box(
        carcass,
        "plinth_back",
        (FACE_FRAME_THICKNESS, CABINET_WIDTH - 2.0 * SIDE_THICKNESS, PLINTH_HEIGHT),
        (-CABINET_DEPTH + FACE_FRAME_THICKNESS / 2.0, 0.0, PLINTH_HEIGHT / 2.0),
        carcass_mat,
    )

    inner_depth = CABINET_DEPTH - BACK_THICKNESS - FACE_FRAME_THICKNESS
    inner_center_x = -FACE_FRAME_THICKNESS - inner_depth / 2.0
    opening_width = (CABINET_WIDTH - 2.0 * SIDE_THICKNESS - CENTER_DIVIDER_THICKNESS) / 2.0
    bay_centers_y = (
        -(CENTER_DIVIDER_THICKNESS / 2.0 + opening_width / 2.0),
        CENTER_DIVIDER_THICKNESS / 2.0 + opening_width / 2.0,
    )

    divider_height = WORK_SHELF_BOTTOM - 0.046 - (PLINTH_HEIGHT + BOTTOM_PANEL_THICKNESS)
    add_box(
        carcass,
        "center_divider",
        (bottom_depth, CENTER_DIVIDER_THICKNESS, divider_height),
        (
            -bottom_depth / 2.0,
            0.0,
            PLINTH_HEIGHT + BOTTOM_PANEL_THICKNESS + divider_height / 2.0,
        ),
        carcass_mat,
    )

    row_gap_height = DRAWER_ROW_BOTTOMS[1] - (DRAWER_ROW_BOTTOMS[0] + DRAWER_FACE_HEIGHT)
    for gap_index, gap_center_z in enumerate(
        (
            DRAWER_ROW_BOTTOMS[0] + DRAWER_FACE_HEIGHT + row_gap_height / 2.0,
            DRAWER_ROW_BOTTOMS[1] + DRAWER_FACE_HEIGHT + row_gap_height / 2.0,
        )
    ):
        for bay_index, bay_center_y in enumerate(bay_centers_y):
            add_box(
                carcass,
                f"drawer_rail_{gap_index}_{bay_index}",
                (FACE_FRAME_THICKNESS, opening_width, row_gap_height),
                (-FACE_FRAME_THICKNESS / 2.0, bay_center_y, gap_center_z),
                carcass_mat,
            )
            add_box(
                carcass,
                f"drawer_shelf_{gap_index}_{bay_index}",
                (inner_depth, opening_width, row_gap_height),
                (inner_center_x, bay_center_y, gap_center_z),
                carcass_mat,
            )

    shelf_band_height = WORK_SHELF_BOTTOM - (DRAWER_ROW_BOTTOMS[2] + DRAWER_FACE_HEIGHT)
    for bay_index, bay_center_y in enumerate(bay_centers_y):
        add_box(
            carcass,
            f"shelf_rail_{bay_index}",
            (FACE_FRAME_THICKNESS, opening_width, shelf_band_height),
            (
                -FACE_FRAME_THICKNESS / 2.0,
                bay_center_y,
                DRAWER_ROW_BOTTOMS[2] + DRAWER_FACE_HEIGHT + shelf_band_height / 2.0,
            ),
            carcass_mat,
        )

    add_box(
        carcass,
        "top_apron",
        (
            FACE_FRAME_THICKNESS,
            CABINET_WIDTH - 2.0 * SIDE_THICKNESS,
            CABINET_HEIGHT - TOP_THICKNESS - (WORK_SHELF_BOTTOM + WORK_SHELF_SLOT_HEIGHT),
        ),
        (
            -FACE_FRAME_THICKNESS / 2.0,
            0.0,
            WORK_SHELF_BOTTOM
            + WORK_SHELF_SLOT_HEIGHT
            + (CABINET_HEIGHT - TOP_THICKNESS - (WORK_SHELF_BOTTOM + WORK_SHELF_SLOT_HEIGHT)) / 2.0,
        ),
        carcass_mat,
    )

    runner_length = 0.42
    runner_height = 0.016
    runner_thickness = 0.01
    runner_center_x = -0.08 - runner_length / 2.0
    for row_index, drawer_bottom in enumerate(DRAWER_ROW_BOTTOMS):
        for bay_index, bay_center_y in enumerate(bay_centers_y):
            runner_z = drawer_bottom + 0.078
            add_box(
                carcass,
                f"guide_{row_index}_{bay_index}_0",
                (runner_length, runner_thickness, runner_height),
                (
                    runner_center_x,
                    bay_center_y - (opening_width / 2.0 - runner_thickness / 2.0),
                    runner_z,
                ),
                runner_mat,
            )
            add_box(
                carcass,
                f"guide_{row_index}_{bay_index}_1",
                (runner_length, runner_thickness, runner_height),
                (
                    runner_center_x,
                    bay_center_y + (opening_width / 2.0 - runner_thickness / 2.0),
                    runner_z,
                ),
                runner_mat,
            )

    shelf_runner_length = 0.45
    shelf_runner_height = 0.01
    shelf_runner_thickness = 0.014
    shelf_runner_center_x = -0.07 - shelf_runner_length / 2.0
    shelf_runner_center_z = WORK_SHELF_BOTTOM + 0.005
    add_box(
        carcass,
        "shelf_runner_0",
        (shelf_runner_length, shelf_runner_thickness, shelf_runner_height),
        (
            shelf_runner_center_x,
            -(CABINET_WIDTH / 2.0 - SIDE_THICKNESS - shelf_runner_thickness / 2.0),
            shelf_runner_center_z,
        ),
        runner_mat,
    )
    add_box(
        carcass,
        "shelf_runner_1",
        (shelf_runner_length, shelf_runner_thickness, shelf_runner_height),
        (
            shelf_runner_center_x,
            CABINET_WIDTH / 2.0 - SIDE_THICKNESS - shelf_runner_thickness / 2.0,
            shelf_runner_center_z,
        ),
        runner_mat,
    )

    for row_index, drawer_bottom in enumerate(DRAWER_ROW_BOTTOMS):
        for bay_index, bay_center_y in enumerate(bay_centers_y):
            name = f"drawer_{row_index}_{bay_index}"
            drawer = model.part(name)
            add_drawer_geometry(drawer, face_mat, box_mat, handle_mat)
            model.articulation(
                f"{name}_slide",
                ArticulationType.PRISMATIC,
                parent=carcass,
                child=drawer,
                origin=Origin(xyz=(0.0, bay_center_y, drawer_bottom)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(
                    effort=90.0,
                    velocity=0.35,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
                ),
            )

    work_shelf = model.part("work_shelf")
    add_work_shelf_geometry(work_shelf, face_mat, top_mat, handle_mat)
    model.articulation(
        "work_shelf_slide",
        ArticulationType.PRISMATIC,
        parent=carcass,
        child=work_shelf,
        origin=Origin(xyz=(0.0, 0.0, WORK_SHELF_BOTTOM)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=WORK_SHELF_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    work_shelf = object_model.get_part("work_shelf")
    top_drawer = object_model.get_part("drawer_2_0")

    ctx.expect_origin_gap(
        work_shelf,
        top_drawer,
        axis="z",
        min_gap=0.24,
        name="work shelf sits above the upper drawer row",
    )

    slide_checks = [
        ("drawer_0_0", "drawer_0_0_slide", 0.20, 0.14),
        ("drawer_0_1", "drawer_0_1_slide", 0.20, 0.14),
        ("drawer_1_0", "drawer_1_0_slide", 0.20, 0.14),
        ("drawer_1_1", "drawer_1_1_slide", 0.20, 0.14),
        ("drawer_2_0", "drawer_2_0_slide", 0.20, 0.14),
        ("drawer_2_1", "drawer_2_1_slide", 0.20, 0.14),
        ("work_shelf", "work_shelf_slide", 0.18, 0.13),
    ]

    for part_name, joint_name, min_motion, min_insertion in slide_checks:
        moving_part = object_model.get_part(part_name)
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        upper = 0.0 if limits is None or limits.upper is None else limits.upper

        rest_pos = ctx.part_world_position(moving_part)
        with ctx.pose({joint: upper}):
            extended_pos = ctx.part_world_position(moving_part)
            ctx.expect_overlap(
                moving_part,
                carcass,
                axes="x",
                min_overlap=min_insertion,
                name=f"{part_name} keeps retained insertion at full extension",
            )
            ctx.expect_within(
                moving_part,
                carcass,
                axes="yz",
                margin=0.0,
                name=f"{part_name} stays within the cabinet span while sliding",
            )

        ctx.check(
            f"{part_name} slides outward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + min_motion,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


object_model = build_object_model()
