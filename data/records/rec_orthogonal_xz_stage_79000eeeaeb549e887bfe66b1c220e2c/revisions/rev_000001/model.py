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


BODY_LENGTH = 0.300
BODY_WIDTH = 0.170
BASE_THICKNESS = 0.016
SIDE_LIP_HEIGHT = 0.020
SIDE_LIP_THICKNESS = 0.016
END_CAP_THICKNESS = 0.024

X_RAIL_LENGTH = 0.240
X_RAIL_WIDTH = 0.024
X_RAIL_HEIGHT = 0.018
X_RAIL_Y = 0.050
X_TRAVEL = 0.050

X_SHOE_LENGTH = 0.132
X_SHOE_WIDTH = 0.024
X_SHOE_HEIGHT = 0.018

CARRIAGE_LENGTH = 0.168
CARRIAGE_WIDTH = 0.150
CARRIAGE_PLATE_THICKNESS = 0.024
CARRIAGE_TOWER_WIDTH = 0.064
CARRIAGE_TOWER_DEPTH = 0.028
CARRIAGE_TOWER_Y = -0.006
CARRIAGE_TOWER_HEIGHT = 0.130
CARRIAGE_TOP_CAP_WIDTH = 0.086
CARRIAGE_TOP_CAP_DEPTH = 0.032
CARRIAGE_TOP_CAP_THICKNESS = 0.018

Z_RAIL_X = 0.024
Z_RAIL_Y = 0.015
Z_RAIL_WIDTH = 0.014
Z_RAIL_DEPTH = 0.016
Z_RAIL_HEIGHT = 0.148
Z_TRAVEL = 0.085

Z_JOINT_Y = Z_RAIL_Y
Z_JOINT_Z = X_SHOE_HEIGHT + CARRIAGE_PLATE_THICKNESS

Z_SLIDER_WIDTH = 0.034
Z_SLIDER_DEPTH = 0.024
Z_SLIDER_HEIGHT = 0.082

UPPER_MAST_WIDTH = 0.032
UPPER_MAST_DEPTH = 0.022
UPPER_MAST_Y = 0.024
UPPER_MAST_HEIGHT = 0.170
UPPER_NOSE_WIDTH = 0.040
UPPER_NOSE_DEPTH = 0.040
UPPER_NOSE_Y = 0.028
UPPER_NOSE_HEIGHT = 0.044
UPPER_HEAD_WIDTH = 0.090
UPPER_HEAD_DEPTH = 0.060
UPPER_HEAD_Y = 0.024
UPPER_HEAD_THICKNESS = 0.016


def _add_box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _guide_body_shell():
    base = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    left_lip = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, SIDE_LIP_THICKNESS, SIDE_LIP_HEIGHT)
        .translate((0.0, BODY_WIDTH / 2.0 - SIDE_LIP_THICKNESS / 2.0, SIDE_LIP_HEIGHT / 2.0))
    )
    right_lip = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, SIDE_LIP_THICKNESS, SIDE_LIP_HEIGHT)
        .translate((0.0, -(BODY_WIDTH / 2.0 - SIDE_LIP_THICKNESS / 2.0), SIDE_LIP_HEIGHT / 2.0))
    )
    left_end = (
        cq.Workplane("XY")
        .box(
            END_CAP_THICKNESS,
            BODY_WIDTH - 2.0 * SIDE_LIP_THICKNESS,
            SIDE_LIP_HEIGHT,
        )
        .translate(
            (
                -(BODY_LENGTH / 2.0 - END_CAP_THICKNESS / 2.0),
                0.0,
                SIDE_LIP_HEIGHT / 2.0,
            )
        )
    )
    right_end = (
        cq.Workplane("XY")
        .box(
            END_CAP_THICKNESS,
            BODY_WIDTH - 2.0 * SIDE_LIP_THICKNESS,
            SIDE_LIP_HEIGHT,
        )
        .translate(
            (
                BODY_LENGTH / 2.0 - END_CAP_THICKNESS / 2.0,
                0.0,
                SIDE_LIP_HEIGHT / 2.0,
            )
        )
    )
    center_spine = (
        cq.Workplane("XY")
        .box(0.190, 0.026, 0.006)
        .translate((0.0, 0.0, BASE_THICKNESS + 0.003))
    )
    return base.union(left_lip).union(right_lip).union(left_end).union(right_end).union(center_spine)


def _lower_carriage_shell():
    plate = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_PLATE_THICKNESS)
        .translate((0.0, 0.0, X_SHOE_HEIGHT + CARRIAGE_PLATE_THICKNESS / 2.0))
    )
    relief = (
        cq.Workplane("XY")
        .box(0.098, 0.072, 0.012)
        .translate((0.0, 0.0, X_SHOE_HEIGHT + 0.006))
    )
    plate = plate.cut(relief)

    tower = (
        cq.Workplane("XY")
        .box(CARRIAGE_TOWER_WIDTH, CARRIAGE_TOWER_DEPTH, CARRIAGE_TOWER_HEIGHT)
        .translate(
            (
                0.0,
                CARRIAGE_TOWER_Y,
                X_SHOE_HEIGHT + CARRIAGE_PLATE_THICKNESS + CARRIAGE_TOWER_HEIGHT / 2.0,
            )
        )
    )
    tower_slot = (
        cq.Workplane("XY")
        .box(0.040, 0.030, 0.100)
        .translate((0.0, 0.010, X_SHOE_HEIGHT + CARRIAGE_PLATE_THICKNESS + 0.050))
    )
    tower = tower.cut(tower_slot)
    top_cap = (
        cq.Workplane("XY")
        .box(CARRIAGE_TOP_CAP_WIDTH, CARRIAGE_TOP_CAP_DEPTH, CARRIAGE_TOP_CAP_THICKNESS)
        .translate(
            (
                0.0,
                CARRIAGE_TOWER_Y,
                X_SHOE_HEIGHT
                + CARRIAGE_PLATE_THICKNESS
                + CARRIAGE_TOWER_HEIGHT
                + CARRIAGE_TOP_CAP_THICKNESS / 2.0,
            )
        )
    )
    left_rib = (
        cq.Workplane("XY")
        .box(0.020, 0.028, 0.082)
        .translate((0.042, -0.004, X_SHOE_HEIGHT + 0.041))
    )
    right_rib = (
        cq.Workplane("XY")
        .box(0.020, 0.028, 0.082)
        .translate((-0.042, -0.004, X_SHOE_HEIGHT + 0.041))
    )
    return plate.union(tower).union(top_cap).union(left_rib).union(right_rib)


def _upper_guide_shell():
    mast = (
        cq.Workplane("XY")
        .box(UPPER_MAST_WIDTH, UPPER_MAST_DEPTH, UPPER_MAST_HEIGHT)
        .translate(
            (
                0.0,
                UPPER_MAST_Y,
                Z_SLIDER_HEIGHT + UPPER_MAST_HEIGHT / 2.0,
            )
        )
    )
    nose = (
        cq.Workplane("XY")
        .box(UPPER_NOSE_WIDTH, UPPER_NOSE_DEPTH, UPPER_NOSE_HEIGHT)
        .translate(
            (
                0.0,
                UPPER_NOSE_Y,
                Z_SLIDER_HEIGHT + UPPER_MAST_HEIGHT - UPPER_NOSE_HEIGHT / 2.0,
            )
        )
    )
    head = (
        cq.Workplane("XY")
        .box(UPPER_HEAD_WIDTH, UPPER_HEAD_DEPTH, UPPER_HEAD_THICKNESS)
        .translate(
            (
                0.0,
                UPPER_HEAD_Y,
                Z_SLIDER_HEIGHT + UPPER_MAST_HEIGHT + UPPER_HEAD_THICKNESS / 2.0,
            )
        )
    )
    rear_stiffener = (
        cq.Workplane("XY")
        .box(0.018, 0.018, 0.060)
        .translate((0.0, 0.006, 0.112))
    )
    return mast.union(nose).union(head).union(rear_stiffener)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_xz_stage")

    model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("machined_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("guide_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    model.material("stage_blue", rgba=(0.29, 0.38, 0.53, 1.0))
    model.material("dark_trim", rgba=(0.11, 0.12, 0.14, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        mesh_from_cadquery(_guide_body_shell(), "guide_body_shell"),
        material="body_dark",
        name="body_shell",
    )
    _add_box(
        guide_body,
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (0.0, X_RAIL_Y, BASE_THICKNESS + X_RAIL_HEIGHT / 2.0),
        "guide_steel",
        "x_rail_left",
    )
    _add_box(
        guide_body,
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (0.0, -X_RAIL_Y, BASE_THICKNESS + X_RAIL_HEIGHT / 2.0),
        "guide_steel",
        "x_rail_right",
    )
    guide_body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BASE_THICKNESS + X_RAIL_HEIGHT)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + X_RAIL_HEIGHT) / 2.0)),
    )

    lower_carriage = model.part("lower_carriage")
    _add_box(
        lower_carriage,
        (CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_PLATE_THICKNESS),
        (0.0, 0.0, X_SHOE_HEIGHT + CARRIAGE_PLATE_THICKNESS / 2.0),
        "machined_gray",
        "carriage_shell",
    )
    _add_box(
        lower_carriage,
        (0.018, 0.028, 0.148),
        (0.034, -0.006, Z_JOINT_Z + 0.074),
        "machined_gray",
        "z_frame_left",
    )
    _add_box(
        lower_carriage,
        (0.018, 0.028, 0.148),
        (-0.034, -0.006, Z_JOINT_Z + 0.074),
        "machined_gray",
        "z_frame_right",
    )
    _add_box(
        lower_carriage,
        (0.070, 0.014, 0.118),
        (0.0, -0.022, Z_JOINT_Z + 0.059),
        "machined_gray",
        "z_frame_back",
    )
    _add_box(
        lower_carriage,
        (CARRIAGE_TOP_CAP_WIDTH, CARRIAGE_TOP_CAP_DEPTH, CARRIAGE_TOP_CAP_THICKNESS),
        (0.0, -0.006, 0.181),
        "machined_gray",
        "z_frame_cross",
    )
    _add_box(
        lower_carriage,
        (X_SHOE_LENGTH, X_SHOE_WIDTH, X_SHOE_HEIGHT),
        (0.0, X_RAIL_Y, X_SHOE_HEIGHT / 2.0),
        "dark_trim",
        "x_shoe_left",
    )
    _add_box(
        lower_carriage,
        (X_SHOE_LENGTH, X_SHOE_WIDTH, X_SHOE_HEIGHT),
        (0.0, -X_RAIL_Y, X_SHOE_HEIGHT / 2.0),
        "dark_trim",
        "x_shoe_right",
    )
    _add_box(
        lower_carriage,
        (Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT),
        (Z_RAIL_X, Z_RAIL_Y, Z_JOINT_Z + Z_RAIL_HEIGHT / 2.0),
        "guide_steel",
        "z_rail_left",
    )
    _add_box(
        lower_carriage,
        (Z_RAIL_WIDTH, Z_RAIL_DEPTH, Z_RAIL_HEIGHT),
        (-Z_RAIL_X, Z_RAIL_Y, Z_JOINT_Z + Z_RAIL_HEIGHT / 2.0),
        "guide_steel",
        "z_rail_right",
    )
    lower_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.190)),
        mass=2.3,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    upper_guide = model.part("upper_guide")
    upper_guide.visual(
        mesh_from_cadquery(_upper_guide_shell(), "upper_guide_shell"),
        material="stage_blue",
        name="upper_shell",
    )
    _add_box(
        upper_guide,
        (Z_SLIDER_WIDTH, Z_SLIDER_DEPTH, Z_SLIDER_HEIGHT),
        (0.0, 0.0, Z_SLIDER_HEIGHT / 2.0),
        "machined_gray",
        "z_slider",
    )
    upper_guide.inertial = Inertial.from_geometry(
        Box((UPPER_HEAD_WIDTH, UPPER_HEAD_DEPTH, Z_SLIDER_HEIGHT + UPPER_MAST_HEIGHT + UPPER_HEAD_THICKNESS)),
        mass=1.2,
        origin=Origin(
            xyz=(
                0.0,
                UPPER_HEAD_Y,
                (Z_SLIDER_HEIGHT + UPPER_MAST_HEIGHT + UPPER_HEAD_THICKNESS) / 2.0,
            )
        ),
    )

    model.articulation(
        "guide_body_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + X_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=220.0,
            velocity=0.28,
        ),
    )
    model.articulation(
        "lower_carriage_to_upper_guide",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_guide,
        origin=Origin(xyz=(0.0, Z_JOINT_Y, Z_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=140.0,
            velocity=0.18,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    lower_carriage = object_model.get_part("lower_carriage")
    upper_guide = object_model.get_part("upper_guide")
    x_slide = object_model.get_articulation("guide_body_to_lower_carriage")
    z_slide = object_model.get_articulation("lower_carriage_to_upper_guide")

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

    ctx.expect_contact(
        lower_carriage,
        guide_body,
        elem_a="x_shoe_left",
        elem_b="x_rail_left",
        name="left carriage shoe is supported by the left x rail",
    )
    ctx.expect_contact(
        lower_carriage,
        guide_body,
        elem_a="x_shoe_right",
        elem_b="x_rail_right",
        name="right carriage shoe is supported by the right x rail",
    )
    ctx.expect_overlap(
        lower_carriage,
        guide_body,
        axes="x",
        elem_a="x_shoe_left",
        elem_b="x_rail_left",
        min_overlap=0.120,
        name="left x shoe has substantial guide engagement at center",
    )
    ctx.expect_gap(
        lower_carriage,
        guide_body,
        axis="z",
        positive_elem="carriage_shell",
        negative_elem="body_shell",
        min_gap=0.030,
        max_gap=0.080,
        name="broad lower carriage clears the fixed guide body",
    )
    ctx.expect_contact(
        upper_guide,
        lower_carriage,
        elem_a="z_slider",
        elem_b="z_rail_left",
        name="upper guide slider bears on the left vertical rail",
    )
    ctx.expect_contact(
        upper_guide,
        lower_carriage,
        elem_a="z_slider",
        elem_b="z_rail_right",
        name="upper guide slider bears on the right vertical rail",
    )

    rest_carriage = ctx.part_world_position(lower_carriage)
    with ctx.pose({x_slide: X_TRAVEL}):
        moved_carriage = ctx.part_world_position(lower_carriage)
        ctx.expect_overlap(
            lower_carriage,
            guide_body,
            axes="x",
            elem_a="x_shoe_left",
            elem_b="x_rail_left",
            min_overlap=0.070,
            name="x carriage retains rail engagement at positive travel",
        )
    ctx.check(
        "positive x travel moves the lower carriage to +x",
        rest_carriage is not None
        and moved_carriage is not None
        and moved_carriage[0] > rest_carriage[0] + 0.040,
        details=f"rest={rest_carriage}, moved={moved_carriage}",
    )

    rest_upper = ctx.part_world_position(upper_guide)
    with ctx.pose({z_slide: Z_TRAVEL}):
        raised_upper = ctx.part_world_position(upper_guide)
        ctx.expect_overlap(
            upper_guide,
            lower_carriage,
            axes="z",
            elem_a="z_slider",
            elem_b="z_rail_left",
            min_overlap=0.060,
            name="z slider retains insertion in the upright guide at max rise",
        )
    ctx.check(
        "positive z travel raises the upper guide",
        rest_upper is not None
        and raised_upper is not None
        and raised_upper[2] > rest_upper[2] + 0.070,
        details=f"rest={rest_upper}, raised={raised_upper}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
