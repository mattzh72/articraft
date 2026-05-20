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


WALL_T = 0.0025
BASE_T = 0.0030
LIP_W = 0.010
RAIL_T = 0.0030
STOP_X = 0.008
STOP_H = 0.012
LUG_H = 0.008
STOP_RUNNER_CLEAR = 0.008

OUTER_LEN = 0.460
OUTER_W = 0.340
OUTER_H = 0.090
OUTER_FRONT_H = 0.012
OUTER_SUPPORT_TOP = 0.018
OUTER_RAIL_W = 0.014
OUTER_RAIL_LEN = 0.340
OUTER_RAIL_X = -0.010

MIDDLE_LEN = 0.370
MIDDLE_W = 0.303
MIDDLE_H = 0.070
MIDDLE_FRONT_H = 0.010
MIDDLE_SUPPORT_TOP = 0.014
MIDDLE_RUNNER_W = 0.010
MIDDLE_RUNNER_H = 0.006
MIDDLE_RUNNER_LEN = 0.260
MIDDLE_RUNNER_X = -0.005
MIDDLE_LUG_X = 0.009
MIDDLE_RAIL_W = 0.014
MIDDLE_RAIL_LEN = 0.260
MIDDLE_RAIL_X = -0.008

INNER_LEN = 0.290
INNER_W = 0.278
INNER_H = 0.052
INNER_FRONT_H = 0.008
INNER_SUPPORT_TOP = 0.010
INNER_RUNNER_W = 0.010
INNER_RUNNER_H = 0.006
INNER_RUNNER_LEN = 0.190
INNER_RUNNER_X = -0.004
INNER_LUG_X = -0.002
INNER_RAIL_W = 0.032
INNER_RAIL_LEN = 0.170
INNER_RAIL_X = -0.002

PLATFORM_LEN = 0.170
PLATFORM_W = 0.200
PLATFORM_DECK_T = 0.004
PLATFORM_DECK_Z = 0.009
PLATFORM_RUNNER_W = 0.010
PLATFORM_RUNNER_H = 0.005
PLATFORM_RUNNER_LEN = 0.100
PLATFORM_RUNNER_X = -0.005
PLATFORM_LUG_X = 0.002
PLATFORM_FRONT_H = 0.006
PLATFORM_REAR_H = 0.010
PLATFORM_SIDE_T = 0.004

OUTER_TO_MIDDLE_X = -0.025
MIDDLE_TO_INNER_X = -0.022
INNER_TO_PLATFORM_X = -0.045

OUTER_TRAVEL = 0.160
MIDDLE_TRAVEL = 0.132
INNER_TRAVEL = 0.110


def add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def add_tray(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    front_height: float,
    material,
    internal_rail_top: float | None = None,
    internal_rail_width: float | None = None,
    internal_rail_length: float | None = None,
    internal_rail_x: float = 0.0,
    external_runner_width: float | None = None,
    external_runner_height: float | None = None,
    external_runner_length: float | None = None,
    external_runner_x: float = 0.0,
    stop_lug_x: float | None = None,
):
    add_box(
        part,
        name=f"{prefix}_base",
        size=(length, width, BASE_T),
        xyz=(0.0, 0.0, BASE_T / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_left_wall",
        size=(length, WALL_T, height),
        xyz=(0.0, width / 2.0 - WALL_T / 2.0, height / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_right_wall",
        size=(length, WALL_T, height),
        xyz=(0.0, -width / 2.0 + WALL_T / 2.0, height / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_rear_wall",
        size=(WALL_T, width - 2.0 * WALL_T, height),
        xyz=(-length / 2.0 + WALL_T / 2.0, 0.0, height / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_front_lip",
        size=(WALL_T, width - 2.0 * WALL_T, front_height),
        xyz=(length / 2.0 - WALL_T / 2.0, 0.0, front_height / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_left_return",
        size=(length - WALL_T, LIP_W, WALL_T),
        xyz=(WALL_T / 2.0, width / 2.0 - WALL_T - LIP_W / 2.0, height - WALL_T / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_right_return",
        size=(length - WALL_T, LIP_W, WALL_T),
        xyz=(WALL_T / 2.0, -width / 2.0 + WALL_T + LIP_W / 2.0, height - WALL_T / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_rear_return",
        size=(LIP_W, width - 2.0 * WALL_T - 2.0 * LIP_W, WALL_T),
        xyz=(-length / 2.0 + WALL_T + LIP_W / 2.0, 0.0, height - WALL_T / 2.0),
        material=material,
    )

    if (
        internal_rail_top is not None
        and internal_rail_width is not None
        and internal_rail_length is not None
    ):
        rail_y = width / 2.0 - WALL_T - internal_rail_width / 2.0
        rail_z = internal_rail_top - RAIL_T / 2.0
        add_box(
            part,
            name=f"{prefix}_left_rail",
            size=(internal_rail_length, internal_rail_width, RAIL_T),
            xyz=(internal_rail_x, rail_y, rail_z),
            material=material,
        )
        add_box(
            part,
            name=f"{prefix}_right_rail",
            size=(internal_rail_length, internal_rail_width, RAIL_T),
            xyz=(internal_rail_x, -rail_y, rail_z),
            material=material,
        )
        stop_x = internal_rail_x + internal_rail_length / 2.0 - STOP_X / 2.0
        stop_z = internal_rail_top + STOP_RUNNER_CLEAR + STOP_H / 2.0
        add_box(
            part,
            name=f"{prefix}_left_stop",
            size=(STOP_X, internal_rail_width, STOP_H),
            xyz=(stop_x, rail_y, stop_z),
            material=material,
        )
        add_box(
            part,
            name=f"{prefix}_right_stop",
            size=(STOP_X, internal_rail_width, STOP_H),
            xyz=(stop_x, -rail_y, stop_z),
            material=material,
        )

    if (
        external_runner_width is not None
        and external_runner_height is not None
        and external_runner_length is not None
    ):
        runner_y = width / 2.0 + external_runner_width / 2.0
        runner_z = external_runner_height / 2.0
        add_box(
            part,
            name=f"{prefix}_left_runner",
            size=(external_runner_length, external_runner_width, external_runner_height),
            xyz=(external_runner_x, runner_y, runner_z),
            material=material,
        )
        add_box(
            part,
            name=f"{prefix}_right_runner",
            size=(external_runner_length, external_runner_width, external_runner_height),
            xyz=(external_runner_x, -runner_y, runner_z),
            material=material,
        )
        if stop_lug_x is not None:
            lug_z = external_runner_height + LUG_H / 2.0
            add_box(
                part,
                name=f"{prefix}_left_stop_lug",
                size=(STOP_X, external_runner_width, LUG_H),
                xyz=(stop_lug_x, runner_y, lug_z),
                material=material,
            )
            add_box(
                part,
                name=f"{prefix}_right_stop_lug",
                size=(STOP_X, external_runner_width, LUG_H),
                xyz=(stop_lug_x, -runner_y, lug_z),
                material=material,
            )


def add_platform(part, *, prefix: str, material):
    add_box(
        part,
        name=f"{prefix}_deck",
        size=(PLATFORM_LEN, PLATFORM_W, PLATFORM_DECK_T),
        xyz=(0.0, 0.0, PLATFORM_DECK_Z + PLATFORM_DECK_T / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_left_skirt",
        size=(PLATFORM_LEN, PLATFORM_SIDE_T, PLATFORM_DECK_Z + PLATFORM_DECK_T),
        xyz=(
            0.0,
            PLATFORM_W / 2.0 - PLATFORM_SIDE_T / 2.0,
            (PLATFORM_DECK_Z + PLATFORM_DECK_T) / 2.0,
        ),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_right_skirt",
        size=(PLATFORM_LEN, PLATFORM_SIDE_T, PLATFORM_DECK_Z + PLATFORM_DECK_T),
        xyz=(
            0.0,
            -PLATFORM_W / 2.0 + PLATFORM_SIDE_T / 2.0,
            (PLATFORM_DECK_Z + PLATFORM_DECK_T) / 2.0,
        ),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_rear_upstand",
        size=(WALL_T, PLATFORM_W - 2.0 * PLATFORM_SIDE_T, PLATFORM_REAR_H),
        xyz=(-PLATFORM_LEN / 2.0 + WALL_T / 2.0, 0.0, PLATFORM_REAR_H / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_front_toe",
        size=(WALL_T, PLATFORM_W - 2.0 * PLATFORM_SIDE_T, PLATFORM_FRONT_H),
        xyz=(PLATFORM_LEN / 2.0 - WALL_T / 2.0, 0.0, PLATFORM_FRONT_H / 2.0),
        material=material,
    )
    runner_y = PLATFORM_W / 2.0 + PLATFORM_RUNNER_W / 2.0
    add_box(
        part,
        name=f"{prefix}_left_runner",
        size=(PLATFORM_RUNNER_LEN, PLATFORM_RUNNER_W, PLATFORM_RUNNER_H),
        xyz=(PLATFORM_RUNNER_X, runner_y, PLATFORM_RUNNER_H / 2.0),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_right_runner",
        size=(PLATFORM_RUNNER_LEN, PLATFORM_RUNNER_W, PLATFORM_RUNNER_H),
        xyz=(PLATFORM_RUNNER_X, -runner_y, PLATFORM_RUNNER_H / 2.0),
        material=material,
    )
    lug_z = PLATFORM_RUNNER_H + LUG_H / 2.0
    add_box(
        part,
        name=f"{prefix}_left_stop_lug",
        size=(STOP_X, PLATFORM_RUNNER_W, LUG_H),
        xyz=(PLATFORM_LUG_X, runner_y, lug_z),
        material=material,
    )
    add_box(
        part,
        name=f"{prefix}_right_stop_lug",
        size=(STOP_X, PLATFORM_RUNNER_W, LUG_H),
        xyz=(PLATFORM_LUG_X, -runner_y, lug_z),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_drawer_shuttle")

    outer_mat = model.material("outer_powder_coat", rgba=(0.19, 0.22, 0.25, 1.0))
    middle_mat = model.material("middle_powder_coat", rgba=(0.34, 0.37, 0.41, 1.0))
    inner_mat = model.material("inner_powder_coat", rgba=(0.52, 0.56, 0.60, 1.0))
    platform_mat = model.material("terminal_platform", rgba=(0.72, 0.76, 0.80, 1.0))

    outer = model.part("outer_tray")
    add_tray(
        outer,
        prefix="outer",
        length=OUTER_LEN,
        width=OUTER_W,
        height=OUTER_H,
        front_height=OUTER_FRONT_H,
        material=outer_mat,
        internal_rail_top=OUTER_SUPPORT_TOP,
        internal_rail_width=OUTER_RAIL_W,
        internal_rail_length=OUTER_RAIL_LEN,
        internal_rail_x=OUTER_RAIL_X,
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LEN, OUTER_W, OUTER_H)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, OUTER_H / 2.0)),
    )

    middle = model.part("middle_tray")
    add_tray(
        middle,
        prefix="middle",
        length=MIDDLE_LEN,
        width=MIDDLE_W,
        height=MIDDLE_H,
        front_height=MIDDLE_FRONT_H,
        material=middle_mat,
        internal_rail_top=MIDDLE_SUPPORT_TOP,
        internal_rail_width=MIDDLE_RAIL_W,
        internal_rail_length=MIDDLE_RAIL_LEN,
        internal_rail_x=MIDDLE_RAIL_X,
        external_runner_width=MIDDLE_RUNNER_W,
        external_runner_height=MIDDLE_RUNNER_H,
        external_runner_length=MIDDLE_RUNNER_LEN,
        external_runner_x=MIDDLE_RUNNER_X,
        stop_lug_x=MIDDLE_LUG_X,
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LEN, MIDDLE_W + 2.0 * MIDDLE_RUNNER_W, MIDDLE_H)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_H / 2.0)),
    )

    inner = model.part("inner_tray")
    add_tray(
        inner,
        prefix="inner",
        length=INNER_LEN,
        width=INNER_W,
        height=INNER_H,
        front_height=INNER_FRONT_H,
        material=inner_mat,
        internal_rail_top=INNER_SUPPORT_TOP,
        internal_rail_width=INNER_RAIL_W,
        internal_rail_length=INNER_RAIL_LEN,
        internal_rail_x=INNER_RAIL_X,
        external_runner_width=INNER_RUNNER_W,
        external_runner_height=INNER_RUNNER_H,
        external_runner_length=INNER_RUNNER_LEN,
        external_runner_x=INNER_RUNNER_X,
        stop_lug_x=INNER_LUG_X,
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LEN, INNER_W + 2.0 * INNER_RUNNER_W, INNER_H)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, INNER_H / 2.0)),
    )

    platform = model.part("terminal_platform")
    add_platform(platform, prefix="platform", material=platform_mat)
    platform.inertial = Inertial.from_geometry(
        Box((PLATFORM_LEN, PLATFORM_W + 2.0 * PLATFORM_RUNNER_W, PLATFORM_DECK_Z + PLATFORM_DECK_T)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, (PLATFORM_DECK_Z + PLATFORM_DECK_T) / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_X, 0.0, OUTER_SUPPORT_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.40,
            lower=0.0,
            upper=OUTER_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_X, 0.0, MIDDLE_SUPPORT_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.45,
            lower=0.0,
            upper=MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_platform",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=platform,
        origin=Origin(xyz=(INNER_TO_PLATFORM_X, 0.0, INNER_SUPPORT_TOP)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.50,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tray")
    middle = object_model.get_part("middle_tray")
    inner = object_model.get_part("inner_tray")
    platform = object_model.get_part("terminal_platform")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_platform = object_model.get_articulation("inner_to_platform")

    outer_left_rail = outer.get_visual("outer_left_rail")
    middle_left_runner = middle.get_visual("middle_left_runner")
    middle_left_rail = middle.get_visual("middle_left_rail")
    middle_left_lug = middle.get_visual("middle_left_stop_lug")
    inner_left_runner = inner.get_visual("inner_left_runner")
    inner_left_rail = inner.get_visual("inner_left_rail")
    inner_left_lug = inner.get_visual("inner_left_stop_lug")
    platform_left_runner = platform.get_visual("platform_left_runner")
    platform_left_lug = platform.get_visual("platform_left_stop_lug")
    outer_left_stop = outer.get_visual("outer_left_stop")
    middle_left_stop = middle.get_visual("middle_left_stop")
    inner_left_stop = inner.get_visual("inner_left_stop")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    with ctx.pose(
        {
            outer_to_middle: 0.0,
            middle_to_inner: 0.0,
            inner_to_platform: 0.0,
        }
    ):
        ctx.expect_within(middle, outer, axes="yz", margin=0.0, name="middle_nested_in_outer_yz")
        ctx.expect_within(inner, middle, axes="yz", margin=0.0, name="inner_nested_in_middle_yz")
        ctx.expect_within(platform, inner, axes="yz", margin=0.0, name="platform_nested_in_inner_yz")
        ctx.expect_contact(
            middle,
            outer,
            elem_a=middle_left_runner,
            elem_b=outer_left_rail,
            contact_tol=0.0005,
            name="middle_left_runner_supported_at_rest",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a=inner_left_runner,
            elem_b=middle_left_rail,
            contact_tol=0.0005,
            name="inner_left_runner_supported_at_rest",
        )
        ctx.expect_contact(
            platform,
            inner,
            elem_a=platform_left_runner,
            elem_b=inner_left_rail,
            contact_tol=0.0005,
            name="platform_left_runner_supported_at_rest",
        )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TRAVEL,
            middle_to_inner: MIDDLE_TRAVEL,
            inner_to_platform: INNER_TRAVEL,
        }
    ):
        ctx.expect_within(middle, outer, axes="yz", margin=0.0, name="middle_stays_captured_open_yz")
        ctx.expect_within(inner, middle, axes="yz", margin=0.0, name="inner_stays_captured_open_yz")
        ctx.expect_within(platform, inner, axes="yz", margin=0.0, name="platform_stays_captured_open_yz")
        ctx.expect_contact(
            middle,
            outer,
            elem_a=middle_left_runner,
            elem_b=outer_left_rail,
            contact_tol=0.0005,
            name="middle_left_runner_supported_open",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a=inner_left_runner,
            elem_b=middle_left_rail,
            contact_tol=0.0005,
            name="inner_left_runner_supported_open",
        )
        ctx.expect_contact(
            platform,
            inner,
            elem_a=platform_left_runner,
            elem_b=inner_left_rail,
            contact_tol=0.0005,
            name="platform_left_runner_supported_open",
        )
        ctx.expect_gap(
            outer,
            middle,
            axis="x",
            positive_elem=outer_left_stop,
            negative_elem=middle_left_lug,
            min_gap=0.002,
            max_gap=0.006,
            name="outer_to_middle_stop_gap_open",
        )
        ctx.expect_gap(
            middle,
            inner,
            axis="x",
            positive_elem=middle_left_stop,
            negative_elem=inner_left_lug,
            min_gap=0.0015,
            max_gap=0.005,
            name="middle_to_inner_stop_gap_open",
        )
        ctx.expect_gap(
            inner,
            platform,
            axis="x",
            positive_elem=inner_left_stop,
            negative_elem=platform_left_lug,
            min_gap=0.002,
            max_gap=0.006,
            name="inner_to_platform_stop_gap_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
