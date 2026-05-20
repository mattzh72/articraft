from __future__ import annotations

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


FREEZER_LENGTH = 2.04
FREEZER_WIDTH = 0.94
BASE_HEIGHT = 0.08
OUTER_WALL_HEIGHT = 0.72
TOP_DECK_HEIGHT = 0.044
RAIL_HEIGHT = 0.012
OUTER_WALL_TOP_Z = BASE_HEIGHT + OUTER_WALL_HEIGHT
TOP_DECK_TOP_Z = OUTER_WALL_TOP_Z + TOP_DECK_HEIGHT
RAIL_TOP_Z = TOP_DECK_TOP_Z + RAIL_HEIGHT

INNER_WIDTH = 0.72
INNER_LENGTH = 1.82
SIDE_WALL_THICKNESS = 0.055
END_WALL_THICKNESS = 0.055
LINER_THICKNESS = 0.018
LINER_FLOOR_THICKNESS = 0.018
LINER_FLOOR_TOP_Z = 0.128
LINER_WALL_HEIGHT = OUTER_WALL_TOP_Z - LINER_FLOOR_TOP_Z

BAY_CENTERS = (-0.64, -0.04, 0.56)
RAIL_LENGTH = 0.56
LID_TRAVEL = 0.10


def _add_lid(model: ArticulatedObject, body, *, index: int, center_x: float) -> None:
    lid = model.part(f"lid_{index}")

    runner_height = 0.010
    frame_height = 0.024
    lid_length = 0.48
    lid_width = 0.76
    runner_length = 0.40
    runner_width = 0.018
    frame_rail_width = 0.040
    runner_y = (INNER_WIDTH / 2.0) + 0.002
    frame_y = (lid_width / 2.0) - (frame_rail_width / 2.0)
    frame_x = (lid_length / 2.0) - (frame_rail_width / 2.0)
    glass_z = runner_height + 0.012

    lid.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(0.0, -runner_y, runner_height / 2.0)),
        material="rail_aluminum",
        name="front_runner",
    )
    lid.visual(
        Box((runner_length, runner_width, runner_height)),
        origin=Origin(xyz=(0.0, runner_y, runner_height / 2.0)),
        material="rail_aluminum",
        name="rear_runner",
    )
    lid.visual(
        Box((lid_length, frame_rail_width, frame_height)),
        origin=Origin(xyz=(0.0, -frame_y, runner_height + (frame_height / 2.0))),
        material="lid_frame",
        name="front_frame",
    )
    lid.visual(
        Box((lid_length, frame_rail_width, frame_height)),
        origin=Origin(xyz=(0.0, frame_y, runner_height + (frame_height / 2.0))),
        material="lid_frame",
        name="rear_frame",
    )
    lid.visual(
        Box((frame_rail_width, lid_width, frame_height)),
        origin=Origin(xyz=(-frame_x, 0.0, runner_height + (frame_height / 2.0))),
        material="lid_frame",
        name="left_frame",
    )
    lid.visual(
        Box((frame_rail_width, lid_width, frame_height)),
        origin=Origin(xyz=(frame_x, 0.0, runner_height + (frame_height / 2.0))),
        material="lid_frame",
        name="right_frame",
    )
    lid.visual(
        Box((0.40, 0.68, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, glass_z)),
        material="lid_glass",
        name="glass",
    )
    lid.visual(
        Box((0.12, 0.022, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                -((lid_width / 2.0) - 0.028),
                runner_height + frame_height + 0.006,
            )
        ),
        material="handle_dark",
        name="pull",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, runner_height + frame_height + 0.020)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    model.articulation(
        f"lid_slide_{index}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(center_x, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.22,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="island_freezer")

    model.material("body_white", rgba=(0.90, 0.92, 0.94, 1.0))
    model.material("base_graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("liner_white", rgba=(0.95, 0.96, 0.97, 1.0))
    model.material("rail_aluminum", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("lid_frame", rgba=(0.44, 0.46, 0.49, 1.0))
    model.material("lid_glass", rgba=(0.70, 0.84, 0.92, 0.34))
    model.material("control_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("accent_red", rgba=(0.73, 0.11, 0.10, 1.0))
    model.material("metal_dark", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("handle_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("lock_black", rgba=(0.10, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((FREEZER_LENGTH, FREEZER_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material="base_graphite",
        name="base_skirt",
    )
    body.visual(
        Box((FREEZER_LENGTH - 0.08, SIDE_WALL_THICKNESS, OUTER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(FREEZER_WIDTH / 2.0) + (SIDE_WALL_THICKNESS / 2.0),
                BASE_HEIGHT + (OUTER_WALL_HEIGHT / 2.0),
            )
        ),
        material="body_white",
        name="front_shell",
    )
    body.visual(
        Box((FREEZER_LENGTH - 0.08, SIDE_WALL_THICKNESS, OUTER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (FREEZER_WIDTH / 2.0) - (SIDE_WALL_THICKNESS / 2.0),
                BASE_HEIGHT + (OUTER_WALL_HEIGHT / 2.0),
            )
        ),
        material="body_white",
        name="rear_shell",
    )
    body.visual(
        Box((END_WALL_THICKNESS, FREEZER_WIDTH - 0.10, OUTER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(FREEZER_LENGTH / 2.0) + (END_WALL_THICKNESS / 2.0),
                0.0,
                BASE_HEIGHT + (OUTER_WALL_HEIGHT / 2.0),
            )
        ),
        material="body_white",
        name="left_shell",
    )
    body.visual(
        Box((END_WALL_THICKNESS, FREEZER_WIDTH - 0.10, OUTER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                (FREEZER_LENGTH / 2.0) - (END_WALL_THICKNESS / 2.0),
                0.0,
                BASE_HEIGHT + (OUTER_WALL_HEIGHT / 2.0),
            )
        ),
        material="body_white",
        name="right_shell",
    )

    body.visual(
        Box((INNER_LENGTH, INNER_WIDTH, LINER_FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                LINER_FLOOR_TOP_Z - (LINER_FLOOR_THICKNESS / 2.0),
            )
        ),
        material="liner_white",
        name="liner_floor",
    )
    body.visual(
        Box((INNER_LENGTH, LINER_THICKNESS, LINER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(INNER_WIDTH / 2.0) + (LINER_THICKNESS / 2.0),
                LINER_FLOOR_TOP_Z + (LINER_WALL_HEIGHT / 2.0),
            )
        ),
        material="liner_white",
        name="front_liner",
    )
    body.visual(
        Box((INNER_LENGTH, LINER_THICKNESS, LINER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (INNER_WIDTH / 2.0) - (LINER_THICKNESS / 2.0),
                LINER_FLOOR_TOP_Z + (LINER_WALL_HEIGHT / 2.0),
            )
        ),
        material="liner_white",
        name="rear_liner",
    )
    body.visual(
        Box((LINER_THICKNESS, INNER_WIDTH, LINER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(INNER_LENGTH / 2.0) + (LINER_THICKNESS / 2.0),
                0.0,
                LINER_FLOOR_TOP_Z + (LINER_WALL_HEIGHT / 2.0),
            )
        ),
        material="liner_white",
        name="left_liner",
    )
    body.visual(
        Box((LINER_THICKNESS, INNER_WIDTH, LINER_WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                (INNER_LENGTH / 2.0) - (LINER_THICKNESS / 2.0),
                0.0,
                LINER_FLOOR_TOP_Z + (LINER_WALL_HEIGHT / 2.0),
            )
        ),
        material="liner_white",
        name="right_liner",
    )

    for divider_index, divider_x in enumerate((-0.34, 0.26)):
        body.visual(
            Box((0.018, INNER_WIDTH, LINER_WALL_HEIGHT)),
            origin=Origin(
                xyz=(
                    divider_x,
                    0.0,
                    LINER_FLOOR_TOP_Z + (LINER_WALL_HEIGHT / 2.0),
                )
            ),
            material="liner_white",
            name=f"divider_{divider_index}",
        )

    body.visual(
        Box((FREEZER_LENGTH, 0.10, TOP_DECK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(FREEZER_WIDTH / 2.0) + 0.05,
                OUTER_WALL_TOP_Z + (TOP_DECK_HEIGHT / 2.0),
            )
        ),
        material="body_white",
        name="front_rim",
    )
    body.visual(
        Box((FREEZER_LENGTH, 0.10, TOP_DECK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (FREEZER_WIDTH / 2.0) - 0.05,
                OUTER_WALL_TOP_Z + (TOP_DECK_HEIGHT / 2.0),
            )
        ),
        material="body_white",
        name="rear_rim",
    )
    body.visual(
        Box((0.12, INNER_WIDTH, TOP_DECK_HEIGHT)),
        origin=Origin(xyz=(-0.96, 0.0, OUTER_WALL_TOP_Z + (TOP_DECK_HEIGHT / 2.0))),
        material="body_white",
        name="left_deck",
    )
    body.visual(
        Box((0.04, INNER_WIDTH, TOP_DECK_HEIGHT)),
        origin=Origin(xyz=(-0.34, 0.0, OUTER_WALL_TOP_Z + (TOP_DECK_HEIGHT / 2.0))),
        material="body_white",
        name="cross_deck_0",
    )
    body.visual(
        Box((0.04, INNER_WIDTH, TOP_DECK_HEIGHT)),
        origin=Origin(xyz=(0.26, 0.0, OUTER_WALL_TOP_Z + (TOP_DECK_HEIGHT / 2.0))),
        material="body_white",
        name="cross_deck_1",
    )
    body.visual(
        Box((0.18, 0.56, TOP_DECK_HEIGHT)),
        origin=Origin(xyz=(0.93, 0.08, OUTER_WALL_TOP_Z + (TOP_DECK_HEIGHT / 2.0))),
        material="body_white",
        name="right_deck",
    )
    body.visual(
        Box((0.06, 0.16, TOP_DECK_HEIGHT)),
        origin=Origin(xyz=(0.87, -0.29, OUTER_WALL_TOP_Z + (TOP_DECK_HEIGHT / 2.0))),
        material="body_white",
        name="control_wall",
    )
    body.visual(
        Box((0.10, 0.16, 0.044)),
        origin=Origin(xyz=(0.95, -0.29, 0.772)),
        material="body_white",
        name="control_support",
    )
    body.visual(
        Box((0.10, 0.16, 0.012)),
        origin=Origin(xyz=(0.95, -0.29, 0.794)),
        material="control_dark",
        name="control_floor",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.95, -0.315, 0.809)),
        material="metal_dark",
        name="control_knob",
    )
    body.visual(
        Box((0.018, 0.030, 0.008)),
        origin=Origin(xyz=(0.915, -0.252, 0.804)),
        material="accent_red",
        name="power_button",
    )
    body.visual(
        Box((0.018, 0.030, 0.008)),
        origin=Origin(xyz=(0.950, -0.252, 0.804)),
        material="metal_dark",
        name="mode_button",
    )

    for index, center_x in enumerate(BAY_CENTERS):
        for rail_side, y_pos in (("front", -0.362), ("rear", 0.362)):
            body.visual(
                Box((RAIL_LENGTH, 0.026, RAIL_HEIGHT)),
                origin=Origin(
                    xyz=(
                        center_x,
                        y_pos,
                        TOP_DECK_TOP_Z + (RAIL_HEIGHT / 2.0),
                    )
                ),
                material="rail_aluminum",
                name=f"{rail_side}_rail_{index}",
            )

    body.visual(
        Box((0.004, 0.060, 0.045)),
        origin=Origin(xyz=((FREEZER_LENGTH / 2.0) + 0.002, -0.29, 0.56)),
        material="metal_dark",
        name="lock_escutcheon",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(
            xyz=((FREEZER_LENGTH / 2.0) + 0.002, -0.29, 0.56),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material="lock_black",
        name="lock_cylinder",
    )
    body.visual(
        Box((0.006, 0.110, 0.018)),
        origin=Origin(xyz=((FREEZER_LENGTH / 2.0) + 0.003, -0.29, 0.61)),
        material="metal_dark",
        name="flap_mount",
    )

    body.inertial = Inertial.from_geometry(
        Box((FREEZER_LENGTH, FREEZER_WIDTH, 0.90)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    for index, center_x in enumerate(BAY_CENTERS):
        _add_lid(model, body, index=index, center_x=center_x)

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Box((0.008, 0.104, 0.086)),
        origin=Origin(xyz=(0.004, 0.0, -0.043)),
        material="body_white",
        name="flap_plate",
    )
    lock_flap.visual(
        Box((0.016, 0.104, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, -0.083)),
        material="body_white",
        name="flap_lip",
    )
    lock_flap.visual(
        Cylinder(radius=0.005, length=0.104),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material="metal_dark",
        name="flap_hinge",
    )
    lock_flap.inertial = Inertial.from_geometry(
        Box((0.016, 0.104, 0.090)),
        mass=0.35,
        origin=Origin(xyz=(0.008, 0.0, -0.040)),
    )
    model.articulation(
        "lock_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_flap,
        origin=Origin(xyz=((FREEZER_LENGTH / 2.0) + 0.006, -0.29, 0.61)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flap = object_model.get_part("lock_flap")
    flap_hinge = object_model.get_articulation("lock_flap_hinge")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected freezer body bounds.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("retail_freezer_length", 1.95 <= size[0] <= 2.10, f"size={size!r}")
        ctx.check("retail_freezer_width", 0.88 <= size[1] <= 0.98, f"size={size!r}")
        ctx.check("retail_freezer_height", 0.84 <= size[2] <= 0.90, f"size={size!r}")

    for index in range(3):
        lid = object_model.get_part(f"lid_{index}")
        slide = object_model.get_articulation(f"lid_slide_{index}")
        limits = slide.motion_limits
        rail_front = f"front_rail_{index}"
        rail_rear = f"rear_rail_{index}"

        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_runner",
            negative_elem=rail_front,
            max_gap=0.001,
            max_penetration=0.0,
            name=f"lid_{index} front runner seats on rail",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="rear_runner",
            negative_elem=rail_rear,
            max_gap=0.001,
            max_penetration=0.0,
            name=f"lid_{index} rear runner seats on rail",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            elem_a="glass",
            elem_b="liner_floor",
            min_overlap=0.60,
            name=f"lid_{index} spans the freezer well",
        )

        rest_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({slide: limits.upper if limits is not None and limits.upper is not None else LID_TRAVEL}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="front_runner",
                negative_elem=rail_front,
                max_gap=0.001,
                max_penetration=0.0,
                name=f"lid_{index} front runner stays on rail when open",
            )
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="rear_runner",
                negative_elem=rail_rear,
                max_gap=0.001,
                max_penetration=0.0,
                name=f"lid_{index} rear runner stays on rail when open",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="x",
                elem_a="front_runner",
                elem_b=rail_front,
                min_overlap=0.10,
                name=f"lid_{index} front runner retains rail engagement",
            )
            open_aabb = ctx.part_world_aabb(lid)

        moved = False
        if rest_aabb is not None and open_aabb is not None:
            moved = float(open_aabb[0][0]) > float(rest_aabb[0][0]) + 0.08
        ctx.check(
            f"lid_{index} moves toward the park deck",
            moved,
            details=f"rest={rest_aabb!r}, open={open_aabb!r}",
        )

    ctx.expect_gap(
        flap,
        body,
        axis="x",
        positive_elem="flap_plate",
        negative_elem="right_shell",
        min_gap=0.004,
        max_gap=0.012,
        name="lock flap sits just proud of the side wall",
    )
    ctx.expect_overlap(
        flap,
        body,
        axes="yz",
        elem_a="flap_plate",
        elem_b="lock_escutcheon",
        min_overlap=0.035,
        name="lock flap covers the key cylinder area",
    )

    closed_plate = ctx.part_element_world_aabb(flap, elem="flap_plate")
    with ctx.pose({flap_hinge: 1.0}):
        opened_plate = ctx.part_element_world_aabb(flap, elem="flap_plate")
    flap_opens = False
    if closed_plate is not None and opened_plate is not None:
        flap_opens = float(opened_plate[1][0]) > float(closed_plate[1][0]) + 0.05
    ctx.check(
        "lock flap opens outward",
        flap_opens,
        details=f"closed={closed_plate!r}, opened={opened_plate!r}",
    )

    return ctx.report()


object_model = build_object_model()
