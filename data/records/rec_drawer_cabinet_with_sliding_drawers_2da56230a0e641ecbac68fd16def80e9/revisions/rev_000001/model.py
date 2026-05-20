from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 1.18
CABINET_DEPTH = 0.74
BODY_HEIGHT = 1.06
PLINTH_HEIGHT = 0.08
WALL_THICKNESS = 0.02
FRONT_POST_DEPTH = 0.03
FRONT_POST_WIDTH = 0.03
TOP_HEADER_HEIGHT = 0.07
BOTTOM_DECK_THICKNESS = 0.016

TRAY_WIDTH = 1.04
TRAY_DEPTH = 0.64
TRAY_BOTTOM_THICKNESS = 0.012
TRAY_SIDE_THICKNESS = 0.012
TRAY_SIDE_HEIGHT = 0.038
TRAY_FRONT_LIP_THICKNESS = 0.016
TRAY_FRONT_LIP_HEIGHT = 0.060

RUNNER_LENGTH = 0.52
RUNNER_WIDTH = 0.022
RUNNER_HEIGHT = 0.018
FIXED_RAIL_LENGTH = 0.56
FIXED_RAIL_WIDTH = 0.028
FIXED_RAIL_HEIGHT = 0.020

SHELF_REAR_X = -0.304
SHELF_BOTTOM_ZS = (0.16, 0.35, 0.54, 0.73)
SHELF_TRAVEL = 0.48

LID_THICKNESS = 0.024
LID_DEPTH = 0.75
LID_WIDTH = 1.184
LID_OPEN_LIMIT = 1.20


def _runner_center_y() -> float:
    return TRAY_WIDTH * 0.5 + 0.013


def _fixed_rail_center_y() -> float:
    return CABINET_WIDTH * 0.5 - WALL_THICKNESS + 0.002 - FIXED_RAIL_WIDTH * 0.5


def _build_shelf(model: ArticulatedObject, index: int, shelf_bottom_z: float, shell_mat, rail_mat) -> None:
    shelf = model.part(f"shelf_{index}")

    shelf.visual(
        Box((TRAY_DEPTH, TRAY_WIDTH, TRAY_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(TRAY_DEPTH * 0.5, 0.0, TRAY_BOTTOM_THICKNESS * 0.5)),
        material=shell_mat,
        name="tray_base",
    )
    shelf.visual(
        Box((TRAY_DEPTH, TRAY_SIDE_THICKNESS, TRAY_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                TRAY_DEPTH * 0.5,
                TRAY_WIDTH * 0.5 - TRAY_SIDE_THICKNESS * 0.5,
                TRAY_SIDE_HEIGHT * 0.5,
            )
        ),
        material=shell_mat,
        name="tray_left_wall",
    )
    shelf.visual(
        Box((TRAY_DEPTH, TRAY_SIDE_THICKNESS, TRAY_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                TRAY_DEPTH * 0.5,
                -TRAY_WIDTH * 0.5 + TRAY_SIDE_THICKNESS * 0.5,
                TRAY_SIDE_HEIGHT * 0.5,
            )
        ),
        material=shell_mat,
        name="tray_right_wall",
    )
    shelf.visual(
        Box((TRAY_SIDE_THICKNESS, TRAY_WIDTH, TRAY_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(TRAY_SIDE_THICKNESS * 0.5, 0.0, TRAY_SIDE_HEIGHT * 0.5)
        ),
        material=shell_mat,
        name="tray_back_wall",
    )
    shelf.visual(
        Box((TRAY_FRONT_LIP_THICKNESS, TRAY_WIDTH, TRAY_FRONT_LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                TRAY_DEPTH - TRAY_FRONT_LIP_THICKNESS * 0.5,
                0.0,
                TRAY_FRONT_LIP_HEIGHT * 0.5,
            )
        ),
        material=shell_mat,
        name="front_lip",
    )
    shelf.visual(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(RUNNER_LENGTH * 0.5, _runner_center_y(), 0.016)
        ),
        material=rail_mat,
        name="runner_left",
    )
    shelf.visual(
        Box((RUNNER_LENGTH, RUNNER_WIDTH, RUNNER_HEIGHT)),
        origin=Origin(
            xyz=(RUNNER_LENGTH * 0.5, -_runner_center_y(), 0.016)
        ),
        material=rail_mat,
        name="runner_right",
    )
    shelf.visual(
        Box((0.36, 0.028, 0.014)),
        origin=Origin(xyz=(0.24, TRAY_WIDTH * 0.5 + 0.006, 0.020)),
        material=rail_mat,
        name="left_slide_bracket",
    )
    shelf.visual(
        Box((0.36, 0.028, 0.014)),
        origin=Origin(xyz=(0.24, -TRAY_WIDTH * 0.5 - 0.006, 0.020)),
        material=rail_mat,
        name="right_slide_bracket",
    )
    shelf.visual(
        Box((0.028, 0.30, 0.012)),
        origin=Origin(
            xyz=(TRAY_DEPTH - 0.014, 0.0, TRAY_FRONT_LIP_HEIGHT - 0.003)
        ),
        material=rail_mat,
        name="pull_grip",
    )
    shelf.inertial = Inertial.from_geometry(
        Box((TRAY_DEPTH, TRAY_WIDTH + 0.07, 0.085)),
        mass=5.8,
        origin=Origin(xyz=(TRAY_DEPTH * 0.5, 0.0, 0.0425)),
    )

    model.articulation(
        f"body_to_shelf_{index}",
        ArticulationType.PRISMATIC,
        parent="body",
        child=shelf,
        origin=Origin(xyz=(SHELF_REAR_X, 0.0, shelf_bottom_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=0.0,
            upper=SHELF_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mortuary_supply_cabinet")

    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.79, 0.81, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.84, 0.86, 0.88, 1.0))
    slide_steel = model.material("slide_steel", rgba=(0.64, 0.67, 0.70, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.46, 0.49, 0.52, 1.0))

    body = model.part("body")
    case_height = BODY_HEIGHT - PLINTH_HEIGHT
    case_center_z = PLINTH_HEIGHT + case_height * 0.5

    body.visual(
        Box((CABINET_DEPTH, WALL_THICKNESS, case_height)),
        origin=Origin(
            xyz=(0.0, CABINET_WIDTH * 0.5 - WALL_THICKNESS * 0.5, case_center_z)
        ),
        material=brushed_steel,
        name="left_wall",
    )
    body.visual(
        Box((CABINET_DEPTH, WALL_THICKNESS, case_height)),
        origin=Origin(
            xyz=(0.0, -CABINET_WIDTH * 0.5 + WALL_THICKNESS * 0.5, case_center_z)
        ),
        material=brushed_steel,
        name="right_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, CABINET_WIDTH - 2.0 * WALL_THICKNESS + 0.004, case_height)),
        origin=Origin(
            xyz=(-CABINET_DEPTH * 0.5 + WALL_THICKNESS * 0.5, 0.0, case_center_z)
        ),
        material=brushed_steel,
        name="back_wall",
    )
    body.visual(
        Box((CABINET_DEPTH - 0.08, CABINET_WIDTH - 0.10, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
        material=shadow_steel,
        name="plinth",
    )
    body.visual(
        Box(
            (
                CABINET_DEPTH - WALL_THICKNESS,
                CABINET_WIDTH - 2.0 * WALL_THICKNESS + 0.004,
                BOTTOM_DECK_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(0.0, 0.0, PLINTH_HEIGHT + BOTTOM_DECK_THICKNESS * 0.5)
        ),
        material=brushed_steel,
        name="bottom_deck",
    )
    body.visual(
        Box((FRONT_POST_DEPTH, FRONT_POST_WIDTH, case_height)),
        origin=Origin(
            xyz=(
                CABINET_DEPTH * 0.5 - FRONT_POST_DEPTH * 0.5,
                CABINET_WIDTH * 0.5 - FRONT_POST_WIDTH * 0.5,
                case_center_z,
            )
        ),
        material=brushed_steel,
        name="left_front_post",
    )
    body.visual(
        Box((FRONT_POST_DEPTH, FRONT_POST_WIDTH, case_height)),
        origin=Origin(
            xyz=(
                CABINET_DEPTH * 0.5 - FRONT_POST_DEPTH * 0.5,
                -CABINET_WIDTH * 0.5 + FRONT_POST_WIDTH * 0.5,
                case_center_z,
            )
        ),
        material=brushed_steel,
        name="right_front_post",
    )
    body.visual(
        Box((0.052, CABINET_WIDTH - 0.02, TOP_HEADER_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_DEPTH * 0.5 - 0.026,
                0.0,
                BODY_HEIGHT - TOP_HEADER_HEIGHT * 0.5,
            )
        ),
        material=polished_steel,
        name="top_header",
    )
    body.visual(
        Box((0.05, 0.10, 0.022)),
        origin=Origin(
            xyz=(-CABINET_DEPTH * 0.5 + 0.016, CABINET_WIDTH * 0.35, BODY_HEIGHT - 0.011)
        ),
        material=slide_steel,
        name="left_hinge_mount",
    )
    body.visual(
        Box((0.05, 0.10, 0.022)),
        origin=Origin(
            xyz=(-CABINET_DEPTH * 0.5 + 0.016, -CABINET_WIDTH * 0.35, BODY_HEIGHT - 0.011)
        ),
        material=slide_steel,
        name="right_hinge_mount",
    )

    fixed_rail_y = _fixed_rail_center_y()
    for shelf_index, shelf_bottom_z in enumerate(SHELF_BOTTOM_ZS, start=1):
        rail_z = shelf_bottom_z + 0.018
        body.visual(
            Box((FIXED_RAIL_LENGTH, FIXED_RAIL_WIDTH, FIXED_RAIL_HEIGHT)),
            origin=Origin(
                xyz=(-0.020, fixed_rail_y, rail_z)
            ),
            material=slide_steel,
            name=f"fixed_left_rail_{shelf_index}",
        )
        body.visual(
            Box((FIXED_RAIL_LENGTH, FIXED_RAIL_WIDTH, FIXED_RAIL_HEIGHT)),
            origin=Origin(
                xyz=(-0.020, -fixed_rail_y, rail_z)
            ),
            material=slide_steel,
            name=f"fixed_right_rail_{shelf_index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((CABINET_DEPTH, CABINET_WIDTH, BODY_HEIGHT)),
        mass=96.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    for shelf_index, shelf_bottom_z in enumerate(SHELF_BOTTOM_ZS, start=1):
        _build_shelf(
            model,
            index=shelf_index,
            shelf_bottom_z=shelf_bottom_z,
            shell_mat=polished_steel,
            rail_mat=slide_steel,
        )

    lid = model.part("lid")
    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, LID_THICKNESS)),
        origin=Origin(xyz=(LID_DEPTH * 0.5, 0.0, LID_THICKNESS * 0.5)),
        material=polished_steel,
        name="lid_panel",
    )
    lid.visual(
        Box((0.09, 0.08, 0.010)),
        origin=Origin(xyz=(0.045, CABINET_WIDTH * 0.35, 0.005)),
        material=slide_steel,
        name="left_hinge_leaf",
    )
    lid.visual(
        Box((0.09, 0.08, 0.010)),
        origin=Origin(xyz=(0.045, -CABINET_WIDTH * 0.35, 0.005)),
        material=slide_steel,
        name="right_hinge_leaf",
    )
    lid.visual(
        Box((0.035, 0.24, 0.018)),
        origin=Origin(xyz=(LID_DEPTH - 0.060, 0.0, LID_THICKNESS + 0.009)),
        material=slide_steel,
        name="lid_pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_DEPTH, LID_WIDTH, 0.05)),
        mass=9.5,
        origin=Origin(xyz=(LID_DEPTH * 0.5, 0.0, 0.025)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-CABINET_DEPTH * 0.5, 0.0, BODY_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=LID_OPEN_LIMIT,
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

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            contact_tol=0.0015,
            name="lid seats on the cabinet body when closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.60,
            elem_a="lid_panel",
            name="lid covers the cabinet top opening when closed",
        )
        closed_pull = ctx.part_element_world_aabb(lid, elem="lid_pull")

    with ctx.pose({lid_hinge: LID_OPEN_LIMIT}):
        open_pull = ctx.part_element_world_aabb(lid, elem="lid_pull")

    lid_lifts = (
        closed_pull is not None
        and open_pull is not None
        and open_pull[0][2] > closed_pull[0][2] + 0.30
        and open_pull[1][0] < closed_pull[1][0] - 0.20
    )
    ctx.check(
        "lid rotates upward from the rear hinges",
        lid_lifts,
        details=f"closed_pull={closed_pull}, open_pull={open_pull}",
    )

    for shelf_index in range(1, 5):
        shelf = object_model.get_part(f"shelf_{shelf_index}")
        slide = object_model.get_articulation(f"body_to_shelf_{shelf_index}")

        with ctx.pose({slide: 0.0}):
            ctx.expect_within(
                shelf,
                body,
                axes="yz",
                margin=0.0,
                name=f"shelf {shelf_index} stays inside cabinet width and height when closed",
            )
            ctx.expect_overlap(
                shelf,
                body,
                axes="x",
                elem_a="runner_left",
                elem_b=f"fixed_left_rail_{shelf_index}",
                min_overlap=0.40,
                name=f"shelf {shelf_index} left runner is fully nested on the fixed slide",
            )
            ctx.expect_overlap(
                shelf,
                body,
                axes="x",
                elem_a="runner_right",
                elem_b=f"fixed_right_rail_{shelf_index}",
                min_overlap=0.40,
                name=f"shelf {shelf_index} right runner is fully nested on the fixed slide",
            )
            closed_aabb = ctx.part_world_aabb(shelf)

        with ctx.pose({slide: SHELF_TRAVEL}):
            ctx.expect_within(
                shelf,
                body,
                axes="yz",
                margin=0.0,
                name=f"shelf {shelf_index} stays aligned between the side walls when extended",
            )
            ctx.expect_overlap(
                shelf,
                body,
                axes="x",
                elem_a="runner_left",
                elem_b=f"fixed_left_rail_{shelf_index}",
                min_overlap=0.07,
                name=f"shelf {shelf_index} left slide retains insertion at full extension",
            )
            ctx.expect_overlap(
                shelf,
                body,
                axes="x",
                elem_a="runner_right",
                elem_b=f"fixed_right_rail_{shelf_index}",
                min_overlap=0.07,
                name=f"shelf {shelf_index} right slide retains insertion at full extension",
            )
            extended_aabb = ctx.part_world_aabb(shelf)

        extends_forward = (
            closed_aabb is not None
            and extended_aabb is not None
            and extended_aabb[1][0] > closed_aabb[1][0] + 0.40
        )
        ctx.check(
            f"shelf {shelf_index} extends outward on its slides",
            extends_forward,
            details=f"closed_aabb={closed_aabb}, extended_aabb={extended_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
