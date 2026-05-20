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
    mesh_from_geometry,
    section_loft,
)


ROAD_LEVEL_Z = 0.0
PIT_DEPTH = 1.22
FLOOR_THICKNESS = 0.10
FRAME_CAP_THICKNESS = 0.03
WALL_HEIGHT = PIT_DEPTH - FLOOR_THICKNESS - FRAME_CAP_THICKNESS

OUTER_LENGTH = 1.22
OUTER_WIDTH = 3.62
OPENING_LENGTH = 0.98
OPENING_WIDTH = 3.30
END_WALL_THICKNESS = (OUTER_LENGTH - OPENING_LENGTH) / 2.0
SIDE_WALL_THICKNESS = (OUTER_WIDTH - OPENING_WIDTH) / 2.0

WEDGE_TRAVEL = 0.85
DECK_LENGTH = 0.92
DECK_WIDTH = 3.22
DECK_THICKNESS = 0.03
BODY_WIDTH = 3.16
BODY_TOP_Z = -DECK_THICKNESS
BODY_BOTTOM_Z = -1.10
BODY_TOP_FRONT_X = 0.18
BODY_BOTTOM_FRONT_X = -0.46
BODY_REAR_X = 0.46


def _wedge_body_mesh():
    half_width = BODY_WIDTH / 2.0
    section_a = [
        (BODY_TOP_FRONT_X, -half_width, BODY_TOP_Z),
        (BODY_REAR_X, -half_width, BODY_TOP_Z),
        (BODY_REAR_X, -half_width, BODY_BOTTOM_Z),
        (BODY_BOTTOM_FRONT_X, -half_width, BODY_BOTTOM_Z),
    ]
    section_b = [
        (BODY_TOP_FRONT_X, half_width, BODY_TOP_Z),
        (BODY_REAR_X, half_width, BODY_TOP_Z),
        (BODY_REAR_X, half_width, BODY_BOTTOM_Z),
        (BODY_BOTTOM_FRONT_X, half_width, BODY_BOTTOM_Z),
    ]
    return mesh_from_geometry(section_loft([section_a, section_b]), "wedge_body")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anti_ram_rising_wedge_barrier")

    concrete = model.material("concrete", rgba=(0.52, 0.53, 0.54, 1.0))
    steel_frame = model.material("steel_frame", rgba=(0.24, 0.26, 0.29, 1.0))
    wedge_steel = model.material("wedge_steel", rgba=(0.36, 0.38, 0.41, 1.0))
    wear_plate = model.material("wear_plate", rgba=(0.45, 0.47, 0.50, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.92, 0.73, 0.14, 1.0))

    channel_frame = model.part("channel_frame")

    channel_frame.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -PIT_DEPTH + FLOOR_THICKNESS / 2.0)),
        material=concrete,
        name="pit_floor",
    )

    wall_center_z = -PIT_DEPTH + FLOOR_THICKNESS + WALL_HEIGHT / 2.0
    channel_frame.visual(
        Box((END_WALL_THICKNESS, OUTER_WIDTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -(OPENING_LENGTH / 2.0 + END_WALL_THICKNESS / 2.0),
                0.0,
                wall_center_z,
            )
        ),
        material=concrete,
        name="front_wall",
    )
    channel_frame.visual(
        Box((END_WALL_THICKNESS, OUTER_WIDTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                OPENING_LENGTH / 2.0 + END_WALL_THICKNESS / 2.0,
                0.0,
                wall_center_z,
            )
        ),
        material=concrete,
        name="rear_wall",
    )
    channel_frame.visual(
        Box((OPENING_LENGTH, SIDE_WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(OPENING_WIDTH / 2.0 + SIDE_WALL_THICKNESS / 2.0),
                wall_center_z,
            )
        ),
        material=concrete,
        name="left_wall",
    )
    channel_frame.visual(
        Box((OPENING_LENGTH, SIDE_WALL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                OPENING_WIDTH / 2.0 + SIDE_WALL_THICKNESS / 2.0,
                wall_center_z,
            )
        ),
        material=concrete,
        name="right_wall",
    )

    cap_center_z = -FRAME_CAP_THICKNESS / 2.0
    channel_frame.visual(
        Box((END_WALL_THICKNESS, OUTER_WIDTH, FRAME_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                -(OPENING_LENGTH / 2.0 + END_WALL_THICKNESS / 2.0),
                0.0,
                cap_center_z,
            )
        ),
        material=steel_frame,
        name="front_top_rail",
    )
    channel_frame.visual(
        Box((END_WALL_THICKNESS, OUTER_WIDTH, FRAME_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                OPENING_LENGTH / 2.0 + END_WALL_THICKNESS / 2.0,
                0.0,
                cap_center_z,
            )
        ),
        material=steel_frame,
        name="rear_top_rail",
    )
    channel_frame.visual(
        Box((OPENING_LENGTH, SIDE_WALL_THICKNESS, FRAME_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(OPENING_WIDTH / 2.0 + SIDE_WALL_THICKNESS / 2.0),
                cap_center_z,
            )
        ),
        material=steel_frame,
        name="left_top_rail",
    )
    channel_frame.visual(
        Box((OPENING_LENGTH, SIDE_WALL_THICKNESS, FRAME_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                OPENING_WIDTH / 2.0 + SIDE_WALL_THICKNESS / 2.0,
                cap_center_z,
            )
        ),
        material=steel_frame,
        name="right_top_rail",
    )
    channel_frame.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, PIT_DEPTH)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.0, -PIT_DEPTH / 2.0)),
    )

    wedge_plate = model.part("wedge_plate")
    wedge_plate.visual(
        _wedge_body_mesh(),
        material=wedge_steel,
        name="wedge_body",
    )
    wedge_plate.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -DECK_THICKNESS / 2.0)),
        material=wear_plate,
        name="deck_plate",
    )
    wedge_plate.visual(
        Box((0.008, 3.00, 0.12)),
        origin=Origin(xyz=(BODY_REAR_X + 0.004, 0.0, -0.18)),
        material=warning_yellow,
        name="rear_warning_band",
    )
    wedge_plate.visual(
        Box((0.78, 0.08, 0.28)),
        origin=Origin(xyz=(0.0, -(OPENING_WIDTH / 2.0 - 0.04), -0.58)),
        material=steel_frame,
        name="left_guide_shoe",
    )
    wedge_plate.visual(
        Box((0.78, 0.08, 0.28)),
        origin=Origin(xyz=(0.0, OPENING_WIDTH / 2.0 - 0.04, -0.58)),
        material=steel_frame,
        name="right_guide_shoe",
    )
    wedge_plate.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 1.10)),
        mass=1500.0,
        origin=Origin(xyz=(0.0, 0.0, -0.55)),
    )

    model.articulation(
        "frame_to_wedge_plate",
        ArticulationType.PRISMATIC,
        parent=channel_frame,
        child=wedge_plate,
        origin=Origin(xyz=(0.0, 0.0, ROAD_LEVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200000.0,
            velocity=0.35,
            lower=0.0,
            upper=WEDGE_TRAVEL,
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
    channel_frame = object_model.get_part("channel_frame")
    wedge_plate = object_model.get_part("wedge_plate")
    lift_joint = object_model.get_articulation("frame_to_wedge_plate")

    ctx.check(
        "wedge lift joint uses vertical prismatic motion",
        lift_joint.articulation_type == ArticulationType.PRISMATIC and lift_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={lift_joint.articulation_type}, axis={lift_joint.axis}",
    )

    with ctx.pose({lift_joint: 0.0}):
        ctx.expect_gap(
            wedge_plate,
            channel_frame,
            axis="z",
            positive_elem="wedge_body",
            negative_elem="pit_floor",
            min_gap=0.015,
            name="wedge body clears the pit floor at rest",
        )

        deck_aabb = ctx.part_element_world_aabb(wedge_plate, elem="deck_plate")
        left_rail_aabb = ctx.part_element_world_aabb(channel_frame, elem="left_top_rail")
        front_wall_aabb = ctx.part_element_world_aabb(channel_frame, elem="front_wall")
        rear_wall_aabb = ctx.part_element_world_aabb(channel_frame, elem="rear_wall")
        left_wall_aabb = ctx.part_element_world_aabb(channel_frame, elem="left_wall")
        right_wall_aabb = ctx.part_element_world_aabb(channel_frame, elem="right_wall")

        deck_is_flush = (
            deck_aabb is not None
            and left_rail_aabb is not None
            and abs(deck_aabb[1][2] - left_rail_aabb[1][2]) <= 1e-4
        )
        ctx.check(
            "deck plate sits flush with the road-level frame",
            deck_is_flush,
            details=f"deck_aabb={deck_aabb}, left_rail_aabb={left_rail_aabb}",
        )

        opening_contains_deck = (
            deck_aabb is not None
            and front_wall_aabb is not None
            and rear_wall_aabb is not None
            and left_wall_aabb is not None
            and right_wall_aabb is not None
            and deck_aabb[0][0] >= front_wall_aabb[1][0] + 0.01
            and deck_aabb[1][0] <= rear_wall_aabb[0][0] - 0.01
            and deck_aabb[0][1] >= left_wall_aabb[1][1] + 0.01
            and deck_aabb[1][1] <= right_wall_aabb[0][1] - 0.01
        )
        ctx.check(
            "deck plate stays inside the channel opening footprint",
            opening_contains_deck,
            details=(
                f"deck_aabb={deck_aabb}, front_wall_aabb={front_wall_aabb}, "
                f"rear_wall_aabb={rear_wall_aabb}, left_wall_aabb={left_wall_aabb}, "
                f"right_wall_aabb={right_wall_aabb}"
            ),
        )

        rest_position = ctx.part_world_position(wedge_plate)

    upper_limit = lift_joint.motion_limits.upper if lift_joint.motion_limits is not None else None
    with ctx.pose({lift_joint: upper_limit}):
        extended_position = ctx.part_world_position(wedge_plate)
        wedge_body_aabb = ctx.part_element_world_aabb(wedge_plate, elem="wedge_body")

        ctx.check(
            "wedge rises above road level when extended",
            rest_position is not None
            and extended_position is not None
            and extended_position[2] > rest_position[2] + 0.80,
            details=f"rest_position={rest_position}, extended_position={extended_position}",
        )

        ctx.expect_overlap(
            wedge_plate,
            channel_frame,
            axes="z",
            elem_a="wedge_body",
            elem_b="left_wall",
            min_overlap=0.20,
            name="raised wedge retains insertion inside the channel",
        )

        ctx.check(
            "extended wedge still retains buried structure below grade",
            wedge_body_aabb is not None and wedge_body_aabb[0][2] < -0.20,
            details=f"wedge_body_aabb={wedge_body_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
