from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


CASE_WIDTH = 0.92
CASE_DEPTH = 0.42
BASE_HEIGHT = 0.14
LID_HEIGHT = 0.075
WALL = 0.008

BED_HEIGHT = 0.012
BED_Z = 0.052
BED_SUPPORT_HEIGHT = BED_Z - BED_HEIGHT / 2.0 - WALL

HINGE_RADIUS = 0.006
HINGE_Y = CASE_DEPTH / 2.0 + 0.006
HINGE_Z = BASE_HEIGHT

CATCH_PIVOT_Z = 0.108
CATCH_PIVOT_Y = -(CASE_DEPTH / 2.0) - 0.006
CATCH_X_OFFSET = 0.275


def _add_box(part, name, size, xyz, material):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, pi / 2.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_synth_case")

    shell_abs = model.material("shell_abs", rgba=(0.19, 0.20, 0.22, 1.0))
    liner = model.material("liner", rgba=(0.12, 0.12, 0.14, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.71, 0.74, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.62, 1.0))
    latch_rubber = model.material("latch_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, BASE_HEIGHT)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    wall_height = BASE_HEIGHT - WALL
    inner_width = CASE_WIDTH - 2.0 * WALL
    inner_depth = CASE_DEPTH - 2.0 * WALL

    _add_box(
        lower_shell,
        "base_floor",
        (CASE_WIDTH, CASE_DEPTH, WALL),
        (0.0, 0.0, WALL / 2.0),
        shell_abs,
    )
    _add_box(
        lower_shell,
        "base_left_wall",
        (WALL, inner_depth, wall_height),
        (-(CASE_WIDTH / 2.0 - WALL / 2.0), 0.0, WALL + wall_height / 2.0),
        shell_abs,
    )
    _add_box(
        lower_shell,
        "base_right_wall",
        (WALL, inner_depth, wall_height),
        ((CASE_WIDTH / 2.0 - WALL / 2.0), 0.0, WALL + wall_height / 2.0),
        shell_abs,
    )
    _add_box(
        lower_shell,
        "base_front_wall",
        (inner_width, WALL, wall_height),
        (0.0, -(CASE_DEPTH / 2.0 - WALL / 2.0), WALL + wall_height / 2.0),
        shell_abs,
    )
    _add_box(
        lower_shell,
        "base_rear_wall",
        (inner_width, WALL, wall_height),
        (0.0, CASE_DEPTH / 2.0 - WALL / 2.0, WALL + wall_height / 2.0),
        shell_abs,
    )

    bed_width = CASE_WIDTH - 2.0 * (WALL + 0.036)
    bed_depth = CASE_DEPTH - 2.0 * (WALL + 0.055)
    _add_box(
        lower_shell,
        "instrument_bed",
        (bed_width, bed_depth, BED_HEIGHT),
        (0.0, 0.0, BED_Z),
        liner,
    )
    _add_box(
        lower_shell,
        "bed_support_left",
        (0.022, bed_depth - 0.040, BED_SUPPORT_HEIGHT),
        (-(bed_width / 2.0 - 0.011), 0.0, WALL + BED_SUPPORT_HEIGHT / 2.0),
        aluminum,
    )
    _add_box(
        lower_shell,
        "bed_support_right",
        (0.022, bed_depth - 0.040, BED_SUPPORT_HEIGHT),
        ((bed_width / 2.0 - 0.011), 0.0, WALL + BED_SUPPORT_HEIGHT / 2.0),
        aluminum,
    )
    _add_box(
        lower_shell,
        "bed_support_front",
        (bed_width - 0.10, 0.020, BED_SUPPORT_HEIGHT),
        (0.0, -(bed_depth / 2.0 - 0.010), WALL + BED_SUPPORT_HEIGHT / 2.0),
        aluminum,
    )
    _add_box(
        lower_shell,
        "bed_support_rear",
        (bed_width - 0.10, 0.020, BED_SUPPORT_HEIGHT),
        (0.0, bed_depth / 2.0 - 0.010, WALL + BED_SUPPORT_HEIGHT / 2.0),
        aluminum,
    )

    _add_box(
        lower_shell,
        "opening_rim_front",
        (inner_width - 0.012, 0.014, 0.010),
        (0.0, -(CASE_DEPTH / 2.0 - WALL - 0.007), BASE_HEIGHT - 0.005),
        aluminum,
    )
    _add_box(
        lower_shell,
        "opening_rim_rear",
        (inner_width - 0.012, 0.014, 0.010),
        (0.0, CASE_DEPTH / 2.0 - WALL - 0.007, BASE_HEIGHT - 0.005),
        aluminum,
    )
    _add_box(
        lower_shell,
        "opening_rim_left",
        (0.014, inner_depth - 0.040, 0.010),
        (-(CASE_WIDTH / 2.0 - WALL - 0.007), 0.0, BASE_HEIGHT - 0.005),
        aluminum,
    )
    _add_box(
        lower_shell,
        "opening_rim_right",
        (0.014, inner_depth - 0.040, 0.010),
        ((CASE_WIDTH / 2.0 - WALL - 0.007), 0.0, BASE_HEIGHT - 0.005),
        aluminum,
    )

    for suffix, x_center in (("left", -CATCH_X_OFFSET), ("right", CATCH_X_OFFSET)):
        _add_box(
            lower_shell,
            f"{suffix}_pivot_ear_left",
            (0.006, 0.016, 0.022),
            (x_center - 0.016, CATCH_PIVOT_Y + 0.004, CATCH_PIVOT_Z),
            steel,
        )
        _add_box(
            lower_shell,
            f"{suffix}_pivot_ear_right",
            (0.006, 0.016, 0.022),
            (x_center + 0.016, CATCH_PIVOT_Y + 0.004, CATCH_PIVOT_Z),
            steel,
        )
        _add_box(
            lower_shell,
            f"{suffix}_pivot_base",
            (0.038, 0.010, 0.014),
            (x_center, CATCH_PIVOT_Y + 0.007, CATCH_PIVOT_Z - 0.012),
            steel,
        )

    _add_cylinder(
        lower_shell,
        "hinge_knuckle_left",
        HINGE_RADIUS,
        0.220,
        (-0.235, HINGE_Y, HINGE_Z),
        steel,
    )
    _add_cylinder(
        lower_shell,
        "hinge_knuckle_right",
        HINGE_RADIUS,
        0.220,
        (0.235, HINGE_Y, HINGE_Z),
        steel,
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, LID_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -CASE_DEPTH / 2.0, LID_HEIGHT / 2.0)),
    )

    lid_wall_height = LID_HEIGHT - WALL
    lid_inner_width = CASE_WIDTH - 2.0 * WALL
    lid_inner_depth = CASE_DEPTH - 2.0 * WALL

    _add_box(
        lid,
        "lid_top_panel",
        (CASE_WIDTH, CASE_DEPTH, WALL),
        (0.0, -CASE_DEPTH / 2.0, LID_HEIGHT - WALL / 2.0),
        shell_abs,
    )
    _add_box(
        lid,
        "lid_left_wall",
        (WALL, lid_inner_depth, lid_wall_height),
        (-(CASE_WIDTH / 2.0 - WALL / 2.0), -CASE_DEPTH / 2.0, lid_wall_height / 2.0),
        shell_abs,
    )
    _add_box(
        lid,
        "lid_right_wall",
        (WALL, lid_inner_depth, lid_wall_height),
        ((CASE_WIDTH / 2.0 - WALL / 2.0), -CASE_DEPTH / 2.0, lid_wall_height / 2.0),
        shell_abs,
    )
    _add_box(
        lid,
        "lid_front_wall",
        (lid_inner_width, WALL, lid_wall_height),
        (0.0, -(CASE_DEPTH - WALL / 2.0), lid_wall_height / 2.0),
        shell_abs,
    )
    _add_box(
        lid,
        "lid_rear_wall",
        (lid_inner_width, WALL, lid_wall_height),
        (0.0, -(HINGE_RADIUS + WALL / 2.0 + 0.002), lid_wall_height / 2.0),
        shell_abs,
    )
    _add_box(
        lid,
        "lid_inner_liner",
        (CASE_WIDTH - 0.080, CASE_DEPTH - 0.090, 0.010),
        (0.0, -CASE_DEPTH / 2.0, LID_HEIGHT - WALL - 0.005),
        liner,
    )
    _add_box(
        lid,
        "left_striker",
        (0.046, 0.010, 0.006),
        (-CATCH_X_OFFSET, -(CASE_DEPTH + 0.008), 0.017),
        steel,
    )
    _add_box(
        lid,
        "left_striker_mount",
        (0.022, 0.014, 0.014),
        (-CATCH_X_OFFSET, -(CASE_DEPTH + 0.0015), 0.017),
        steel,
    )
    _add_box(
        lid,
        "right_striker",
        (0.046, 0.010, 0.006),
        (CATCH_X_OFFSET, -(CASE_DEPTH + 0.008), 0.017),
        steel,
    )
    _add_box(
        lid,
        "right_striker_mount",
        (0.022, 0.014, 0.014),
        (CATCH_X_OFFSET, -(CASE_DEPTH + 0.0015), 0.017),
        steel,
    )
    _add_cylinder(
        lid,
        "lid_hinge_knuckle_center",
        HINGE_RADIUS,
        0.200,
        (0.0, 0.0, 0.0),
        steel,
    )
    _add_box(
        lid,
        "lid_hinge_leaf_left",
        (0.088, 0.010, 0.012),
        (-0.148, -0.0105, 0.004),
        steel,
    )
    _add_box(
        lid,
        "lid_hinge_leaf_right",
        (0.088, 0.010, 0.012),
        (0.148, -0.0105, 0.004),
        steel,
    )
    _add_box(
        lid,
        "lid_hinge_leaf_center",
        (0.200, 0.010, 0.012),
        (0.0, -0.0105, 0.004),
        steel,
    )

    left_catch = model.part("left_catch")
    left_catch.inertial = Inertial.from_geometry(
        Box((0.040, 0.024, 0.050)),
        mass=0.09,
        origin=Origin(xyz=(0.0, -0.006, 0.022)),
    )
    _add_cylinder(left_catch, "left_barrel", 0.006, 0.026, (0.0, 0.0, 0.0), steel)
    _add_box(
        left_catch,
        "left_latch_body",
        (0.032, 0.006, 0.040),
        (0.0, -0.008, 0.020),
        steel,
    )
    _add_box(
        left_catch,
        "left_hook",
        (0.030, 0.014, 0.006),
        (0.0, 0.003, 0.043),
        latch_rubber,
    )
    _add_box(
        left_catch,
        "left_hook_web",
        (0.016, 0.006, 0.010),
        (0.0, -0.003, 0.037),
        steel,
    )

    right_catch = model.part("right_catch")
    right_catch.inertial = Inertial.from_geometry(
        Box((0.040, 0.024, 0.050)),
        mass=0.09,
        origin=Origin(xyz=(0.0, -0.006, 0.022)),
    )
    _add_cylinder(right_catch, "right_barrel", 0.006, 0.026, (0.0, 0.0, 0.0), steel)
    _add_box(
        right_catch,
        "right_latch_body",
        (0.032, 0.006, 0.040),
        (0.0, -0.008, 0.020),
        steel,
    )
    _add_box(
        right_catch,
        "right_hook",
        (0.030, 0.014, 0.006),
        (0.0, 0.003, 0.043),
        latch_rubber,
    )
    _add_box(
        right_catch,
        "right_hook_web",
        (0.016, 0.006, 0.010),
        (0.0, -0.003, 0.037),
        steel,
    )

    model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "lower_shell_to_left_catch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_catch,
        origin=Origin(xyz=(-CATCH_X_OFFSET, CATCH_PIVOT_Y, CATCH_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "lower_shell_to_right_catch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_catch,
        origin=Origin(xyz=(CATCH_X_OFFSET, CATCH_PIVOT_Y, CATCH_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
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

    expected_parts = ("lower_shell", "lid", "left_catch", "right_catch")
    missing_parts = []
    for part_name in expected_parts:
        try:
            object_model.get_part(part_name)
        except Exception:
            missing_parts.append(part_name)
    ctx.check(
        "expected parts are present",
        not missing_parts,
        details=f"missing_parts={missing_parts}",
    )
    if missing_parts:
        return ctx.report()

    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    left_catch = object_model.get_part("left_catch")
    right_catch = object_model.get_part("right_catch")
    lid_hinge = object_model.get_articulation("lower_shell_to_lid")
    left_hinge = object_model.get_articulation("lower_shell_to_left_catch")
    right_hinge = object_model.get_articulation("lower_shell_to_right_catch")

    with ctx.pose({lid_hinge: 0.0, left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            positive_elem="lid_front_wall",
            negative_elem="base_front_wall",
            max_gap=0.002,
            max_penetration=0.0,
            name="lid front edge seats on the lower shell",
        )
        ctx.expect_contact(
            left_catch,
            lid,
            elem_a="left_hook",
            elem_b="left_striker",
            contact_tol=0.0015,
            name="left catch closes onto its striker",
        )
        ctx.expect_contact(
            right_catch,
            lid,
            elem_a="right_hook",
            elem_b="right_striker",
            contact_tol=0.0015,
            name="right catch closes onto its striker",
        )

    closed_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_wall")
    with ctx.pose({lid_hinge: 1.15}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_wall")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[0][2] > closed_lid_front[0][2] + 0.16,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )

    closed_left_hook = ctx.part_element_world_aabb(left_catch, elem="left_hook")
    with ctx.pose({left_hinge: 0.95}):
        open_left_hook = ctx.part_element_world_aabb(left_catch, elem="left_hook")
    ctx.check(
        "left catch rotates outward to unlatch",
        closed_left_hook is not None
        and open_left_hook is not None
        and open_left_hook[0][1] < closed_left_hook[0][1] - 0.012,
        details=f"closed={closed_left_hook}, open={open_left_hook}",
    )

    closed_right_hook = ctx.part_element_world_aabb(right_catch, elem="right_hook")
    with ctx.pose({right_hinge: 0.95}):
        open_right_hook = ctx.part_element_world_aabb(right_catch, elem="right_hook")
    ctx.check(
        "right catch rotates outward to unlatch",
        closed_right_hook is not None
        and open_right_hook is not None
        and open_right_hook[0][1] < closed_right_hook[0][1] - 0.012,
        details=f"closed={closed_right_hook}, open={open_right_hook}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
