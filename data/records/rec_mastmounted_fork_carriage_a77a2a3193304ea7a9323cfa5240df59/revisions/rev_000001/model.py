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


MAST_WIDTH = 0.58
MAST_DEPTH = 0.20
MAST_HEIGHT = 1.85
RAIL_CENTER_X = 0.19
RAIL_WIDTH = 0.09
RAIL_DEPTH = 0.06
RAIL_HEIGHT = 1.68

CARRIAGE_WIDTH = 0.62
CARRIAGE_HEIGHT = 0.58
CARRIAGE_FRONT_DEPTH = 0.03
CARRIAGE_FRONT_CENTER_Y = 0.085
CARRIAGE_LOW_Z = 0.76
CARRIAGE_TRAVEL = 0.72

FORK_CENTER_X = 0.17
FORK_WIDTH = 0.09
FORK_SHANK_THICKNESS = 0.035
FORK_SHANK_HEIGHT = 0.40
FORK_TINE_LENGTH = 0.82
FORK_TINE_THICKNESS = 0.035
FORK_MOUNT_Y = 0.11
FORK_MOUNT_Z = -0.27
def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_fork_carriage")

    model.material("mast_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    model.material("carriage_red", rgba=(0.73, 0.16, 0.11, 1.0))
    model.material("fork_steel", rgba=(0.12, 0.12, 0.13, 1.0))

    mast = model.part("mast")
    _add_box(mast, (MAST_WIDTH, 0.10, 0.10), (0.0, -0.08, 0.05), "mast_steel", "base_crossmember")
    _add_box(mast, (0.26, 0.03, 1.60), (0.0, -0.045, 0.90), "mast_steel", "rear_spine")
    _add_box(
        mast,
        (RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT),
        (-RAIL_CENTER_X, 0.0, 0.10 + (RAIL_HEIGHT / 2.0)),
        "mast_steel",
        "left_rail",
    )
    _add_box(
        mast,
        (RAIL_WIDTH, RAIL_DEPTH, RAIL_HEIGHT),
        (RAIL_CENTER_X, 0.0, 0.10 + (RAIL_HEIGHT / 2.0)),
        "mast_steel",
        "right_rail",
    )
    _add_box(mast, (0.50, 0.08, 0.08), (0.0, -0.01, 1.74), "mast_steel", "top_bridge")
    mast.inertial = Inertial.from_geometry(
        Box((MAST_WIDTH, MAST_DEPTH, MAST_HEIGHT)),
        mass=140.0,
        origin=Origin(xyz=(0.0, -0.05, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    _add_box(
        carriage,
        (CARRIAGE_WIDTH, CARRIAGE_FRONT_DEPTH, CARRIAGE_HEIGHT),
        (0.0, CARRIAGE_FRONT_CENTER_Y, 0.0),
        "carriage_red",
        "front_plate",
    )
    _add_box(carriage, (0.50, 0.05, 0.10), (0.0, 0.09, -0.22), "carriage_red", "lower_bar")
    _add_box(carriage, (0.42, 0.04, 0.08), (0.0, 0.088, 0.22), "carriage_red", "upper_bar")
    _add_box(carriage, (0.06, 0.05, 0.22), (0.0, 0.086, -0.02), "carriage_red", "center_lift_block")
    for side_name, rail_x in (("left", -RAIL_CENTER_X), ("right", RAIL_CENTER_X)):
        _add_box(
            carriage,
            (RAIL_WIDTH + 0.03, 0.04, 0.54),
            (rail_x, 0.05, 0.0),
            "carriage_red",
            f"{side_name}_front_guide",
        )
        _add_box(
            carriage,
            (0.016, 0.06, 0.54),
            (rail_x - (RAIL_WIDTH / 2.0) - 0.008, 0.0, 0.0),
            "carriage_red",
            f"{side_name}_outer_side_guide",
        )
        _add_box(
            carriage,
            (0.016, 0.06, 0.54),
            (rail_x + (RAIL_WIDTH / 2.0) + 0.008, 0.0, 0.0),
            "carriage_red",
            f"{side_name}_inner_side_guide",
        )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, 0.18, 0.78)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    left_fork = model.part("left_fork")
    _add_box(
        left_fork,
        (FORK_WIDTH, FORK_SHANK_THICKNESS, FORK_SHANK_HEIGHT),
        (0.0, FORK_SHANK_THICKNESS / 2.0, -(FORK_SHANK_HEIGHT / 2.0)),
        "fork_steel",
        "upright_shank",
    )
    _add_box(
        left_fork,
        (FORK_WIDTH, FORK_TINE_LENGTH, FORK_TINE_THICKNESS),
        (0.0, FORK_TINE_LENGTH / 2.0, -FORK_SHANK_HEIGHT + (FORK_TINE_THICKNESS / 2.0)),
        "fork_steel",
        "tine",
    )
    _add_box(left_fork, (FORK_WIDTH, 0.12, 0.10), (0.0, 0.06, -0.35), "fork_steel", "heel")
    left_fork.inertial = Inertial.from_geometry(
        Box((FORK_WIDTH, FORK_TINE_LENGTH, FORK_SHANK_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, FORK_TINE_LENGTH / 2.6, -(FORK_SHANK_HEIGHT / 2.0))),
    )

    right_fork = model.part("right_fork")
    _add_box(
        right_fork,
        (FORK_WIDTH, FORK_SHANK_THICKNESS, FORK_SHANK_HEIGHT),
        (0.0, FORK_SHANK_THICKNESS / 2.0, -(FORK_SHANK_HEIGHT / 2.0)),
        "fork_steel",
        "upright_shank",
    )
    _add_box(
        right_fork,
        (FORK_WIDTH, FORK_TINE_LENGTH, FORK_TINE_THICKNESS),
        (0.0, FORK_TINE_LENGTH / 2.0, -FORK_SHANK_HEIGHT + (FORK_TINE_THICKNESS / 2.0)),
        "fork_steel",
        "tine",
    )
    _add_box(right_fork, (FORK_WIDTH, 0.12, 0.10), (0.0, 0.06, -0.35), "fork_steel", "heel")
    right_fork.inertial = Inertial.from_geometry(
        Box((FORK_WIDTH, FORK_TINE_LENGTH, FORK_SHANK_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, FORK_TINE_LENGTH / 2.6, -(FORK_SHANK_HEIGHT / 2.0))),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_LOW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=16000.0,
            velocity=0.55,
        ),
    )
    model.articulation(
        "carriage_to_left_fork",
        ArticulationType.FIXED,
        parent=carriage,
        child=left_fork,
        origin=Origin(xyz=(-FORK_CENTER_X, FORK_MOUNT_Y, FORK_MOUNT_Z)),
    )
    model.articulation(
        "carriage_to_right_fork",
        ArticulationType.FIXED,
        parent=carriage,
        child=right_fork,
        origin=Origin(xyz=(FORK_CENTER_X, FORK_MOUNT_Y, FORK_MOUNT_Z)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    left_fork = object_model.get_part("left_fork")
    right_fork = object_model.get_part("right_fork")
    lift = object_model.get_articulation("mast_to_carriage")

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

    ctx.expect_contact(carriage, mast, contact_tol=5e-4, name="carriage_guides_touch_mast")
    ctx.expect_contact(left_fork, carriage, contact_tol=1e-6, name="left_fork_seated_on_carriage")
    ctx.expect_contact(right_fork, carriage, contact_tol=1e-6, name="right_fork_seated_on_carriage")
    ctx.expect_origin_distance(
        left_fork,
        right_fork,
        axes="x",
        min_dist=0.32,
        max_dist=0.36,
        name="fork_spacing_is_narrow_warehouse_width",
    )

    with ctx.pose({lift: lift.motion_limits.lower}):
        low_carriage = ctx.part_world_position(carriage)
    with ctx.pose({lift: lift.motion_limits.upper}):
        high_carriage = ctx.part_world_position(carriage)
        ctx.expect_contact(carriage, mast, contact_tol=5e-4, name="carriage_guides_touch_mast_at_full_lift")

    if low_carriage is not None and high_carriage is not None:
        ctx.check(
            "carriage_moves_straight_up",
            abs(high_carriage[0] - low_carriage[0]) < 1e-6
            and abs(high_carriage[1] - low_carriage[1]) < 1e-6
            and (high_carriage[2] - low_carriage[2]) > 0.70,
            details=(
                f"low={low_carriage}, high={high_carriage}; "
                "expected pure vertical lift over the mast rails"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
