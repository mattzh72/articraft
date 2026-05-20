from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


UPRIGHT_CENTER_X = 0.23
UPRIGHT_WIDTH = 0.08
UPRIGHT_DEPTH = 0.07
UPRIGHT_HEIGHT = 1.48
UPRIGHT_WALL = 0.008

TOP_TIE_WIDTH = (2.0 * UPRIGHT_CENTER_X) + UPRIGHT_WIDTH
TOP_TIE_DEPTH = 0.07
TOP_TIE_HEIGHT = 0.08
TOP_TIE_WALL = 0.008

FOOT_WIDTH = 0.12
FOOT_DEPTH = 0.24
FOOT_HEIGHT = 0.06
FOOT_CENTER_Y = -0.03

RAIL_WIDTH = 0.036
RAIL_DEPTH = 0.014
RAIL_HEIGHT = 1.22
RAIL_CENTER_Y = (UPRIGHT_DEPTH / 2.0) + (RAIL_DEPTH / 2.0)
RAIL_CENTER_Z = 0.79

PLATE_WIDTH = 0.34
PLATE_HEIGHT = 0.52
PLATE_THICKNESS = 0.016
PLATE_CENTER_Y = 0.083
PLATE_HOLE_D = 0.022

CHEEK_WIDTH = 0.05
CHEEK_DEPTH = 0.022
CHEEK_HEIGHT = 0.44
CHEEK_CENTER_X = 0.195
CHEEK_CENTER_Y = 0.078

BRACE_WIDTH = 0.30
BRACE_DEPTH = 0.022
BRACE_HEIGHT = 0.045
BRACE_CENTER_Y = 0.072
BRACE_CENTER_Z = 0.205

GUIDE_WIDTH = 0.044
GUIDE_DEPTH = 0.018
GUIDE_HEIGHT = 0.065
GUIDE_CENTER_Y = (RAIL_CENTER_Y + (RAIL_DEPTH / 2.0)) + (GUIDE_DEPTH / 2.0)
GUIDE_OFFSET_Z = 0.18

CARRIAGE_HOME_Z = 0.47
CARRIAGE_TRAVEL = 0.58


def _hollow_tube(size_x: float, size_y: float, size_z: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(size_x, size_y, size_z)
    inner = cq.Workplane("XY").box(
        size_x - (2.0 * wall),
        size_y - (2.0 * wall),
        size_z + 0.004,
    )
    return outer.cut(inner)


def _make_upright_shape() -> cq.Workplane:
    return _hollow_tube(UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT, UPRIGHT_WALL)


def _make_top_tie_shape() -> cq.Workplane:
    return _hollow_tube(TOP_TIE_WIDTH, TOP_TIE_DEPTH, TOP_TIE_HEIGHT, TOP_TIE_WALL)


def _make_foot_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)


def _make_carriage_body_shape() -> cq.Workplane:
    tooling_plate = (
        cq.Workplane("XY")
        .box(PLATE_WIDTH, PLATE_THICKNESS, PLATE_HEIGHT)
        .translate((0.0, PLATE_CENTER_Y, 0.0))
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.10, -0.16),
                (0.00, -0.16),
                (0.10, -0.16),
                (-0.10, 0.00),
                (0.00, 0.00),
                (0.10, 0.00),
                (-0.10, 0.16),
                (0.00, 0.16),
                (0.10, 0.16),
            ]
        )
        .hole(PLATE_HOLE_D)
    )

    left_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_WIDTH, CHEEK_DEPTH, CHEEK_HEIGHT)
        .translate((-CHEEK_CENTER_X, CHEEK_CENTER_Y, 0.0))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(CHEEK_WIDTH, CHEEK_DEPTH, CHEEK_HEIGHT)
        .translate((CHEEK_CENTER_X, CHEEK_CENTER_Y, 0.0))
    )
    upper_brace = (
        cq.Workplane("XY")
        .box(BRACE_WIDTH, BRACE_DEPTH, BRACE_HEIGHT)
        .translate((0.0, BRACE_CENTER_Y, BRACE_CENTER_Z))
    )
    lower_brace = (
        cq.Workplane("XY")
        .box(BRACE_WIDTH, BRACE_DEPTH, BRACE_HEIGHT)
        .translate((0.0, BRACE_CENTER_Y, -BRACE_CENTER_Z))
    )

    return tooling_plate.union(left_cheek).union(right_cheek).union(upper_brace).union(lower_brace)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_lift_carriage")

    model.material("frame_steel", rgba=(0.27, 0.29, 0.31, 1.0))
    model.material("rail_steel", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("tooling_orange", rgba=(0.88, 0.47, 0.14, 1.0))
    model.material("guide_dark", rgba=(0.10, 0.11, 0.13, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_upright_shape(), "left_upright"),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, 0.0, FOOT_HEIGHT + (UPRIGHT_HEIGHT / 2.0))),
        material="frame_steel",
        name="left_upright",
    )
    frame.visual(
        mesh_from_cadquery(_make_upright_shape(), "right_upright"),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, 0.0, FOOT_HEIGHT + (UPRIGHT_HEIGHT / 2.0))),
        material="frame_steel",
        name="right_upright",
    )
    frame.visual(
        mesh_from_cadquery(_make_top_tie_shape(), "top_tie"),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                FOOT_HEIGHT + UPRIGHT_HEIGHT + (TOP_TIE_HEIGHT / 2.0),
            )
        ),
        material="frame_steel",
        name="top_tie",
    )
    frame.visual(
        mesh_from_cadquery(_make_foot_shape(), "left_foot"),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, FOOT_CENTER_Y, FOOT_HEIGHT / 2.0)),
        material="frame_steel",
        name="left_foot",
    )
    frame.visual(
        mesh_from_cadquery(_make_foot_shape(), "right_foot"),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, FOOT_CENTER_Y, FOOT_HEIGHT / 2.0)),
        material="frame_steel",
        name="right_foot",
    )
    frame.visual(
        Box((RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        material="rail_steel",
        name="left_rail",
    )
    frame.visual(
        Box((RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, RAIL_CENTER_Y, RAIL_CENTER_Z)),
        material="rail_steel",
        name="right_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((TOP_TIE_WIDTH, FOOT_DEPTH, FOOT_HEIGHT + UPRIGHT_HEIGHT + TOP_TIE_HEIGHT)),
        mass=92.0,
        origin=Origin(
            xyz=(0.0, FOOT_CENTER_Y / 2.0, (FOOT_HEIGHT + UPRIGHT_HEIGHT + TOP_TIE_HEIGHT) / 2.0)
        ),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_body_shape(), "carriage_body"),
        material="tooling_orange",
        name="tooling_plate",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, GUIDE_CENTER_Y, GUIDE_OFFSET_Z)),
        material="guide_dark",
        name="left_upper_guide",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, GUIDE_CENTER_Y, -GUIDE_OFFSET_Z)),
        material="guide_dark",
        name="left_lower_guide",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, GUIDE_CENTER_Y, GUIDE_OFFSET_Z)),
        material="guide_dark",
        name="right_upper_guide",
    )
    carriage.visual(
        Box((GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, GUIDE_CENTER_Y, -GUIDE_OFFSET_Z)),
        material="guide_dark",
        name="right_lower_guide",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.50, 0.12, 0.60)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.30,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")

    left_rail = frame.get_visual("left_rail")
    right_rail = frame.get_visual("right_rail")
    tooling_plate = carriage.get_visual("tooling_plate")
    left_upper_guide = carriage.get_visual("left_upper_guide")
    left_lower_guide = carriage.get_visual("left_lower_guide")
    right_upper_guide = carriage.get_visual("right_upper_guide")
    right_lower_guide = carriage.get_visual("right_lower_guide")

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
        "vertical_prismatic_joint",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical prismatic joint, got type={slide.articulation_type} axis={slide.axis}",
    )
    ctx.check(
        "slide_limits_match_carriage_travel",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == CARRIAGE_TRAVEL,
        details=(
            "expected limits "
            f"[0.0, {CARRIAGE_TRAVEL}] but got {slide.motion_limits}"
        ),
    )

    closed_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        raised_pos = ctx.part_world_position(carriage)
    if closed_pos is not None and raised_pos is not None:
        ctx.check(
            "carriage_moves_full_vertical_travel",
            abs((raised_pos[2] - closed_pos[2]) - CARRIAGE_TRAVEL) <= 1e-6,
            details=f"expected z travel {CARRIAGE_TRAVEL}, got {raised_pos[2] - closed_pos[2]}",
        )
    else:
        ctx.fail("carriage_moves_full_vertical_travel", "could not resolve carriage positions")

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=left_upper_guide,
            elem_b=left_rail,
            name="left_upper_guide_contacts_left_rail_at_home",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=left_lower_guide,
            elem_b=left_rail,
            name="left_lower_guide_contacts_left_rail_at_home",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=right_upper_guide,
            elem_b=right_rail,
            name="right_upper_guide_contacts_right_rail_at_home",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=right_lower_guide,
            elem_b=right_rail,
            name="right_lower_guide_contacts_right_rail_at_home",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="y",
            min_gap=0.020,
            max_gap=0.080,
            positive_elem=tooling_plate,
            negative_elem="top_tie",
            name="tooling_plate_stands_proud_of_frame",
        )

    with ctx.pose({slide: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=left_upper_guide,
            elem_b=left_rail,
            name="left_upper_guide_contacts_left_rail_raised",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=left_lower_guide,
            elem_b=left_rail,
            name="left_lower_guide_contacts_left_rail_raised",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=right_upper_guide,
            elem_b=right_rail,
            name="right_upper_guide_contacts_right_rail_raised",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=right_lower_guide,
            elem_b=right_rail,
            name="right_lower_guide_contacts_right_rail_raised",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
