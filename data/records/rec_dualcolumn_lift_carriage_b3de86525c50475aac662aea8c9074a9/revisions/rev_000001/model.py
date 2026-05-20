from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


COLUMN_X = 0.22
COLUMN_RADIUS = 0.028
COLUMN_LENGTH = 0.82

BASE_PAD_SIZE = (0.18, 0.26, 0.02)
BASE_PLINTH_SIZE = (0.58, 0.16, 0.06)
TOP_CROSSHEAD_SIZE = (0.58, 0.14, 0.06)

GUIDE_OUTER_THICK = 0.05
GUIDE_INNER_THICK = 0.045
GUIDE_DEPTH = 0.12
GUIDE_BLOCK_HEIGHT = 0.14
GUIDE_CAP_THICK = 0.02
GUIDE_RIB_DEPTH = 0.022
GUIDE_TOTAL_HEIGHT = GUIDE_BLOCK_HEIGHT + (2.0 * GUIDE_CAP_THICK)

CARRIAGE_FRONT_PLATE_WIDTH = 2.0 * (COLUMN_X - COLUMN_RADIUS - GUIDE_INNER_THICK)
CARRIAGE_FRONT_PLATE_DEPTH = 0.028
CARRIAGE_FRONT_PLATE_HEIGHT = GUIDE_TOTAL_HEIGHT
CARRIAGE_REAR_TIE_HEIGHT = 0.10

BASE_TOP_Z = BASE_PAD_SIZE[2] + BASE_PLINTH_SIZE[2]
COLUMN_CENTER_Z = BASE_TOP_Z + (COLUMN_LENGTH / 2.0)
TOP_CROSSHEAD_CENTER_Z = BASE_TOP_Z + COLUMN_LENGTH + (TOP_CROSSHEAD_SIZE[2] / 2.0)

CARRIAGE_HOME_Z = 0.27
CARRIAGE_TRAVEL = 0.50


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage")

    model.material("painted_frame", rgba=(0.27, 0.29, 0.32, 1.0))
    model.material("guide_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_orange", rgba=(0.86, 0.42, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box(BASE_PAD_SIZE),
        origin=Origin(xyz=(-COLUMN_X, 0.0, BASE_PAD_SIZE[2] / 2.0)),
        material="painted_frame",
        name="left_base_pad",
    )
    frame.visual(
        Box(BASE_PAD_SIZE),
        origin=Origin(xyz=(COLUMN_X, 0.0, BASE_PAD_SIZE[2] / 2.0)),
        material="painted_frame",
        name="right_base_pad",
    )
    frame.visual(
        Box(BASE_PLINTH_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_PAD_SIZE[2] + (BASE_PLINTH_SIZE[2] / 2.0))),
        material="painted_frame",
        name="base_plinth",
    )
    frame.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_LENGTH),
        origin=Origin(xyz=(-COLUMN_X, 0.0, COLUMN_CENTER_Z)),
        material="guide_steel",
        name="left_column",
    )
    frame.visual(
        Cylinder(radius=COLUMN_RADIUS, length=COLUMN_LENGTH),
        origin=Origin(xyz=(COLUMN_X, 0.0, COLUMN_CENTER_Z)),
        material="guide_steel",
        name="right_column",
    )
    for side, x_pos in (("left", -COLUMN_X), ("right", COLUMN_X)):
        frame.visual(
            Cylinder(radius=0.042, length=0.018),
            origin=Origin(xyz=(x_pos, 0.0, BASE_TOP_Z + 0.009)),
            material="painted_frame",
            name=f"{side}_lower_collar",
        )
        frame.visual(
            Cylinder(radius=0.042, length=0.018),
            origin=Origin(xyz=(x_pos, 0.0, BASE_TOP_Z + COLUMN_LENGTH - 0.009)),
            material="painted_frame",
            name=f"{side}_upper_collar",
        )
    frame.visual(
        Box(TOP_CROSSHEAD_SIZE),
        origin=Origin(xyz=(0.0, 0.0, TOP_CROSSHEAD_CENTER_Z)),
        material="painted_frame",
        name="top_crosshead",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.58, 0.26, TOP_CROSSHEAD_CENTER_Z + (TOP_CROSSHEAD_SIZE[2] / 2.0))),
        mass=42.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (TOP_CROSSHEAD_CENTER_Z + (TOP_CROSSHEAD_SIZE[2] / 2.0)) / 2.0,
            )
        ),
    )

    carriage = model.part("carriage")

    left_outer_center_x = -COLUMN_X - COLUMN_RADIUS - (GUIDE_OUTER_THICK / 2.0)
    left_inner_center_x = -COLUMN_X + COLUMN_RADIUS + (GUIDE_INNER_THICK / 2.0)
    right_inner_center_x = COLUMN_X - COLUMN_RADIUS - (GUIDE_INNER_THICK / 2.0)
    right_outer_center_x = COLUMN_X + COLUMN_RADIUS + (GUIDE_OUTER_THICK / 2.0)
    cap_width = (left_inner_center_x + (GUIDE_INNER_THICK / 2.0)) - (
        left_outer_center_x - (GUIDE_OUTER_THICK / 2.0)
    )

    carriage.visual(
        Box((GUIDE_OUTER_THICK, GUIDE_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(left_outer_center_x, 0.0, 0.0)),
        material="carriage_orange",
        name="left_outer_block",
    )
    carriage.visual(
        Box((GUIDE_INNER_THICK, GUIDE_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(left_inner_center_x, 0.0, 0.0)),
        material="carriage_orange",
        name="left_inner_block",
    )
    carriage.visual(
        Box((GUIDE_OUTER_THICK, GUIDE_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(right_outer_center_x, 0.0, 0.0)),
        material="carriage_orange",
        name="right_outer_block",
    )
    carriage.visual(
        Box((GUIDE_INNER_THICK, GUIDE_DEPTH, GUIDE_BLOCK_HEIGHT)),
        origin=Origin(xyz=(right_inner_center_x, 0.0, 0.0)),
        material="carriage_orange",
        name="right_inner_block",
    )
    rib_y = (GUIDE_DEPTH - GUIDE_RIB_DEPTH) / 2.0
    for side, center_x in (
        ("left", (left_outer_center_x + left_inner_center_x) / 2.0),
        ("right", (right_outer_center_x + right_inner_center_x) / 2.0),
    ):
        for y_name, y_pos in (("front", rib_y), ("rear", -rib_y)):
            carriage.visual(
                Box((cap_width, GUIDE_RIB_DEPTH, GUIDE_CAP_THICK)),
                origin=Origin(
                    xyz=(
                        center_x,
                        y_pos,
                        (GUIDE_BLOCK_HEIGHT / 2.0) + (GUIDE_CAP_THICK / 2.0),
                    )
                ),
                material="carriage_orange",
                name=f"{side}_top_{y_name}_rib",
            )
            carriage.visual(
                Box((cap_width, GUIDE_RIB_DEPTH, GUIDE_CAP_THICK)),
                origin=Origin(
                    xyz=(
                        center_x,
                        y_pos,
                        -((GUIDE_BLOCK_HEIGHT / 2.0) + (GUIDE_CAP_THICK / 2.0)),
                    )
                ),
                material="carriage_orange",
                name=f"{side}_bottom_{y_name}_rib",
            )
    carriage.visual(
        Box(
            (
                CARRIAGE_FRONT_PLATE_WIDTH,
                CARRIAGE_FRONT_PLATE_DEPTH,
                CARRIAGE_FRONT_PLATE_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                (GUIDE_DEPTH / 2.0) + (CARRIAGE_FRONT_PLATE_DEPTH / 2.0),
                0.0,
            )
        ),
        material="carriage_orange",
        name="front_plate",
    )
    carriage.visual(
        Box(
            (
                CARRIAGE_FRONT_PLATE_WIDTH,
                CARRIAGE_FRONT_PLATE_DEPTH,
                CARRIAGE_REAR_TIE_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -((GUIDE_DEPTH / 2.0) + (CARRIAGE_FRONT_PLATE_DEPTH / 2.0)),
                0.0,
            )
        ),
        material="carriage_orange",
        name="rear_tie",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.62, 0.16, GUIDE_TOTAL_HEIGHT)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=1500.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")
    limits = slide.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parts_present",
        frame is not None and carriage is not None,
        "Expected frame and carriage parts.",
    )
    ctx.check(
        "vertical_prismatic_joint",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical prismatic joint, got type={slide.articulation_type} axis={slide.axis}.",
    )
    ctx.check(
        "travel_limits_reasonable",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and 0.45 <= limits.upper <= 0.55,
        f"Expected about 0.50 m of upward travel, got {limits}.",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="left_inner_block",
            elem_b="left_column",
            contact_tol=5e-4,
            name="left_guide_contacts_left_column_at_home",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_inner_block",
            elem_b="right_column",
            contact_tol=5e-4,
            name="right_guide_contacts_right_column_at_home",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="front_plate",
            negative_elem="base_plinth",
            min_gap=0.09,
            max_gap=0.11,
            name="carriage_clears_base_in_home_pose",
        )
        ctx.expect_origin_gap(
            carriage,
            frame,
            axis="z",
            min_gap=CARRIAGE_HOME_Z,
            max_gap=CARRIAGE_HOME_Z,
            name="carriage_home_origin_height",
        )

    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="left_inner_block",
            elem_b="left_column",
            contact_tol=5e-4,
            name="left_guide_contacts_left_column_when_raised",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_inner_block",
            elem_b="right_column",
            contact_tol=5e-4,
            name="right_guide_contacts_right_column_when_raised",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="top_crosshead",
            negative_elem="front_plate",
            min_gap=0.03,
            max_gap=0.05,
            name="carriage_clears_crosshead_when_raised",
        )
        ctx.expect_origin_gap(
            carriage,
            frame,
            axis="z",
            min_gap=CARRIAGE_HOME_Z + CARRIAGE_TRAVEL,
            max_gap=CARRIAGE_HOME_Z + CARRIAGE_TRAVEL,
            name="carriage_raised_origin_height",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
