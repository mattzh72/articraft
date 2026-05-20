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


OUTER_LENGTH = 0.450
OUTER_HEIGHT = 0.046
OUTER_DEPTH = 0.0155
OUTER_WALL = 0.0016
OUTER_LIP_DEPTH = 0.0025
OUTER_LIP_HEIGHT = 0.0045

MIDDLE_LENGTH = 0.355
MIDDLE_HEIGHT = OUTER_HEIGHT - 2.0 * OUTER_WALL
MIDDLE_DEPTH = 0.0098
MIDDLE_WALL = 0.0014
MIDDLE_LIP_DEPTH = 0.0020
MIDDLE_LIP_HEIGHT = 0.0038

INNER_LENGTH = 0.265
INNER_HEIGHT = MIDDLE_HEIGHT - 2.0 * MIDDLE_WALL
INNER_DEPTH = 0.0060
INNER_WALL = 0.0012
INNER_LIP_DEPTH = 0.0015
INNER_LIP_HEIGHT = 0.0030

OUTER_TO_MIDDLE_HOME = 0.087
MIDDLE_TO_INNER_HOME = 0.085
OUTER_TO_MIDDLE_Y = -0.00045
MIDDLE_TO_INNER_Y = 0.00030
OUTER_TO_MIDDLE_TRAVEL = 0.260
MIDDLE_TO_INNER_TRAVEL = 0.180


def _rail_visual_specs(
    *,
    length: float,
    height: float,
    depth: float,
    wall: float,
    lip_depth: float,
    lip_height: float,
    open_side: str,
    add_end_tab: bool,
) -> list[tuple[str, tuple[float, float, float], tuple[float, float, float]]]:
    if open_side not in {"+y", "-y"}:
        raise ValueError(f"Unsupported open side: {open_side}")

    mouth_sign = 1.0 if open_side == "+y" else -1.0

    def y_coord(value: float) -> float:
        return mouth_sign * value

    flange_depth = depth - wall - lip_depth
    back_web_center_y = -depth / 2.0 + wall / 2.0
    flange_center_y = (wall - lip_depth) / 2.0
    lip_center_y = depth / 2.0 - lip_depth / 2.0
    x_center = length / 2.0

    specs = [
        ("web", (length, wall, height), (x_center, y_coord(back_web_center_y), 0.0)),
        (
            "top_flange",
            (length, flange_depth, wall),
            (x_center, y_coord(flange_center_y), height / 2.0 - wall / 2.0),
        ),
        (
            "bottom_flange",
            (length, flange_depth, wall),
            (x_center, y_coord(flange_center_y), -height / 2.0 + wall / 2.0),
        ),
        (
            "top_lip",
            (length, lip_depth, lip_height),
            (x_center, y_coord(lip_center_y), height / 2.0 - lip_height / 2.0),
        ),
        (
            "bottom_lip",
            (length, lip_depth, lip_height),
            (x_center, y_coord(lip_center_y), -height / 2.0 + lip_height / 2.0),
        ),
    ]

    if add_end_tab:
        specs.append(
            (
                "end_bridge",
                (0.006, lip_depth, height - 2.0 * lip_height),
                (
                    length - 0.003,
                    y_coord(lip_center_y),
                    0.0,
                ),
            )
        )

    return specs


def _add_rail_part(
    model: ArticulatedObject,
    *,
    name: str,
    material: str,
    length: float,
    depth: float,
    height: float,
    wall: float,
    lip_depth: float,
    lip_height: float,
    open_side: str,
    add_end_tab: bool,
    mass: float,
):
    part = model.part(name)
    for visual_name, size, center in _rail_visual_specs(
        length=length,
        height=height,
        depth=depth,
        wall=wall,
        lip_depth=lip_depth,
        lip_height=lip_height,
        open_side=open_side,
        add_end_tab=add_end_tab,
    ):
        part.visual(
            Box(size),
            origin=Origin(xyz=center),
            material=material,
            name=visual_name,
        )
    part.inertial = Inertial.from_geometry(
        Box((length, depth, height)),
        mass=mass,
        origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide")

    model.material("outer_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("middle_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("inner_steel", rgba=(0.79, 0.80, 0.82, 1.0))

    outer = _add_rail_part(
        model,
        name="outer_rail",
        material="outer_steel",
        length=OUTER_LENGTH,
        depth=OUTER_DEPTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        lip_depth=OUTER_LIP_DEPTH,
        lip_height=OUTER_LIP_HEIGHT,
        open_side="+y",
        add_end_tab=False,
        mass=0.70,
    )
    middle = _add_rail_part(
        model,
        name="middle_runner",
        material="middle_steel",
        length=MIDDLE_LENGTH,
        depth=MIDDLE_DEPTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        lip_depth=MIDDLE_LIP_DEPTH,
        lip_height=MIDDLE_LIP_HEIGHT,
        open_side="-y",
        add_end_tab=True,
        mass=0.42,
    )
    inner = _add_rail_part(
        model,
        name="inner_runner",
        material="inner_steel",
        length=INNER_LENGTH,
        depth=INNER_DEPTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        lip_depth=INNER_LIP_DEPTH,
        lip_height=INNER_LIP_HEIGHT,
        open_side="+y",
        add_end_tab=True,
        mass=0.28,
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME, OUTER_TO_MIDDLE_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.50,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, MIDDLE_TO_INNER_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.55,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_rail")
    middle = object_model.get_part("middle_runner")
    inner = object_model.get_part("inner_runner")
    outer_stage = object_model.get_articulation("outer_to_middle")
    inner_stage = object_model.get_articulation("middle_to_inner")

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
        "outer_to_middle_axis_is_longitudinal",
        tuple(outer_stage.axis) == (1.0, 0.0, 0.0),
        f"Expected (1, 0, 0), got {outer_stage.axis}",
    )
    ctx.check(
        "middle_to_inner_axis_is_longitudinal",
        tuple(inner_stage.axis) == (1.0, 0.0, 0.0),
        f"Expected (1, 0, 0), got {inner_stage.axis}",
    )
    ctx.check(
        "outer_to_middle_travel_limit",
        outer_stage.motion_limits is not None
        and outer_stage.motion_limits.lower == 0.0
        and outer_stage.motion_limits.upper == OUTER_TO_MIDDLE_TRAVEL,
        "Outer-to-middle prismatic limits do not match the intended travel.",
    )
    ctx.check(
        "middle_to_inner_travel_limit",
        inner_stage.motion_limits is not None
        and inner_stage.motion_limits.lower == 0.0
        and inner_stage.motion_limits.upper == MIDDLE_TO_INNER_TRAVEL,
        "Middle-to-inner prismatic limits do not match the intended travel.",
    )

    with ctx.pose({outer_stage: 0.0, inner_stage: 0.0}):
        ctx.expect_contact(
            middle,
            outer,
            contact_tol=0.0002,
            name="middle_runner_is_supported_by_outer_rail",
        )
        ctx.expect_contact(
            inner,
            middle,
            contact_tol=0.0002,
            name="inner_runner_is_supported_by_middle_runner",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="middle_runner_nests_within_outer_cross_section",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="inner_runner_nests_within_middle_cross_section",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="yz",
            min_overlap=0.009,
            name="middle_runner_shares_outer_bearing_envelope",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="yz",
            min_overlap=0.005,
            name="inner_runner_shares_middle_bearing_envelope",
        )

    with ctx.pose(
        {
            outer_stage: OUTER_TO_MIDDLE_TRAVEL,
            inner_stage: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_contact(
            middle,
            outer,
            contact_tol=0.0002,
            name="middle_runner_remains_carried_at_full_extension",
        )
        ctx.expect_contact(
            inner,
            middle,
            contact_tol=0.0002,
            name="inner_runner_remains_carried_at_full_extension",
        )
        ctx.expect_origin_gap(
            middle,
            outer,
            axis="x",
            min_gap=OUTER_TO_MIDDLE_HOME + OUTER_TO_MIDDLE_TRAVEL - 0.002,
            max_gap=OUTER_TO_MIDDLE_HOME + OUTER_TO_MIDDLE_TRAVEL + 0.002,
            name="middle_runner_advances_along_outer_rail",
        )
        ctx.expect_origin_gap(
            inner,
            middle,
            axis="x",
            min_gap=MIDDLE_TO_INNER_HOME + MIDDLE_TO_INNER_TRAVEL - 0.002,
            max_gap=MIDDLE_TO_INNER_HOME + MIDDLE_TO_INNER_TRAVEL + 0.002,
            name="inner_runner_advances_along_middle_runner",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
