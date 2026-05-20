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


SHEET = 0.003
LIP_THICK = 0.0024

BODY_W = 0.360
BODY_D = 0.520
BODY_SIDE_H = 0.100
BODY_BACK_H = 0.090
BODY_SILL_H = 0.016
BODY_LIP_W = 0.012
BODY_RAIL_W = 0.018
BODY_RAIL_LEN = 0.440
BODY_RAIL_Y = -0.020
BODY_RAIL_Z = 0.010
BODY_CAP_W = BODY_RAIL_W
BODY_STOP_W = 0.015
BODY_STOP_D = 0.012
BODY_STOP_H = 0.007
BODY_STOP_Y = 0.207
BODY_FRONT_CHEEK_D = 0.034
BODY_FRONT_CHEEK_H = 0.040

MIDDLE_W = 0.312
MIDDLE_D = 0.420
MIDDLE_SIDE_H = 0.072
MIDDLE_FRONT_H = 0.0
MIDDLE_LIP_W = 0.010
MIDDLE_RUNNER_W = 0.018
MIDDLE_RUNNER_H = 0.010
MIDDLE_RUNNER_LEN = 0.360
MIDDLE_RUNNER_Y = -0.015
MIDDLE_RUNNER_Z = 0.0
BODY_CAP_Z = 0.0
MIDDLE_STOP_W = 0.012
MIDDLE_STOP_D = 0.012
MIDDLE_STOP_H = 0.010
MIDDLE_STOP_Y = 0.008
MIDDLE_GUIDE_W = 0.018
MIDDLE_GUIDE_LEN = 0.220
MIDDLE_GUIDE_Y = 0.090
MIDDLE_GUIDE_Z = 0.015
INNER_RUNNER_W = 0.018
INNER_RUNNER_H = 0.008
INNER_RUNNER_Z = 0.0
MIDDLE_CAP_W = MIDDLE_GUIDE_W
MIDDLE_CAP_Z = 0.0
MIDDLE_FIXED_STOP_W = 0.012
MIDDLE_FIXED_STOP_D = 0.008
MIDDLE_FIXED_STOP_H = SHEET
MIDDLE_FIXED_STOP_Y = MIDDLE_GUIDE_Y + MIDDLE_GUIDE_LEN / 2.0 + MIDDLE_FIXED_STOP_D / 2.0
MIDDLE_FIXED_STOP_Z = 0.0
MIDDLE_FRONT_CHEEK_D = 0.028
MIDDLE_FRONT_CHEEK_H = 0.032

INNER_W = 0.252
INNER_D = 0.170
INNER_SIDE_H = 0.050
INNER_REAR_H = 0.045
INNER_FRONT_H = 0.052
INNER_LIP_W = 0.008
INNER_RUNNER_LEN = 0.128
INNER_RUNNER_Y = -0.004
INNER_STOP_W = 0.012
INNER_STOP_D = 0.012
INNER_STOP_H = 0.008
INNER_STOP_Y = -0.011

MID_CLOSED_Y = 0.020
MID_CLOSED_Z = 0.006
MID_TRAVEL = 0.180

INNER_CLOSED_Y = 0.110
INNER_CLOSED_Z = 0.003
INNER_TRAVEL = 0.085

MIDDLE_RUNNER_Z = BODY_RAIL_Z + SHEET - MID_CLOSED_Z
BODY_CAP_Z = MID_CLOSED_Z + MIDDLE_RUNNER_Z + MIDDLE_RUNNER_H
INNER_RUNNER_Z = MIDDLE_GUIDE_Z + SHEET - INNER_CLOSED_Z
MIDDLE_CAP_Z = INNER_CLOSED_Z + INNER_RUNNER_Z + INNER_RUNNER_H
MIDDLE_FIXED_STOP_Z = MIDDLE_CAP_Z


def add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    sx, sy, sz = size
    x, y, z = xyz
    part.visual(
        Box(size),
        origin=Origin(xyz=(x, y, z + sz / 2.0)),
        material=material,
        name=name,
    )


def add_tray_shell_visuals(
    part,
    *,
    prefix: str,
    width: float,
    depth: float,
    side_height: float,
    front_height: float,
    rear_height: float,
    lip_width: float,
    material,
) -> None:
    inner_width = width - 2.0 * SHEET
    add_box_visual(part, name=f"{prefix}_floor", size=(width, depth, SHEET), xyz=(0.0, 0.0, 0.0), material=material)
    add_box_visual(
        part,
        name=f"{prefix}_left_wall",
        size=(SHEET, depth, side_height),
        xyz=(-width / 2.0 + SHEET / 2.0, 0.0, 0.0),
        material=material,
    )
    add_box_visual(
        part,
        name=f"{prefix}_right_wall",
        size=(SHEET, depth, side_height),
        xyz=(width / 2.0 - SHEET / 2.0, 0.0, 0.0),
        material=material,
    )
    if rear_height > 0.0:
        add_box_visual(
            part,
            name=f"{prefix}_rear_wall",
            size=(inner_width, SHEET, rear_height),
            xyz=(0.0, -depth / 2.0 + SHEET / 2.0, 0.0),
            material=material,
        )
    if front_height > 0.0:
        add_box_visual(
            part,
            name=f"{prefix}_front_wall",
            size=(inner_width, SHEET, front_height),
            xyz=(0.0, depth / 2.0 - SHEET / 2.0, 0.0),
            material=material,
        )
    add_box_visual(
        part,
        name=f"{prefix}_left_lip",
        size=(lip_width, depth, LIP_THICK),
        xyz=(-width / 2.0 - lip_width / 2.0, 0.0, side_height - LIP_THICK),
        material=material,
    )
    add_box_visual(
        part,
        name=f"{prefix}_right_lip",
        size=(lip_width, depth, LIP_THICK),
        xyz=(width / 2.0 + lip_width / 2.0, 0.0, side_height - LIP_THICK),
        material=material,
    )
    if rear_height > 0.0:
        add_box_visual(
            part,
            name=f"{prefix}_rear_lip",
            size=(width + 2.0 * lip_width, lip_width, LIP_THICK),
            xyz=(0.0, -depth / 2.0 - lip_width / 2.0, rear_height - LIP_THICK),
            material=material,
        )
    if front_height > 0.0:
        add_box_visual(
            part,
            name=f"{prefix}_front_lip",
            size=(width + 2.0 * lip_width, lip_width, LIP_THICK),
            xyz=(0.0, depth / 2.0 + lip_width / 2.0, front_height - LIP_THICK),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_sample_tray_carrier")

    body_color = model.material("body_graphite", rgba=(0.19, 0.21, 0.24, 1.0))
    middle_color = model.material("tray_silver", rgba=(0.76, 0.78, 0.80, 1.0))
    inner_color = model.material("tray_bluegray", rgba=(0.54, 0.63, 0.69, 1.0))

    outer_body = model.part("outer_body")
    add_box_visual(outer_body, name="body_floor", size=(BODY_W, BODY_D, SHEET), xyz=(0.0, 0.0, 0.0), material=body_color)
    add_box_visual(
        outer_body,
        name="body_left_wall",
        size=(SHEET, BODY_D, BODY_SIDE_H),
        xyz=(-BODY_W / 2.0 + SHEET / 2.0, 0.0, 0.0),
        material=body_color,
    )
    add_box_visual(
        outer_body,
        name="body_right_wall",
        size=(SHEET, BODY_D, BODY_SIDE_H),
        xyz=(BODY_W / 2.0 - SHEET / 2.0, 0.0, 0.0),
        material=body_color,
    )
    add_box_visual(
        outer_body,
        name="body_back_wall",
        size=(BODY_W - 2.0 * SHEET, SHEET, BODY_BACK_H),
        xyz=(0.0, -BODY_D / 2.0 + SHEET / 2.0, 0.0),
        material=body_color,
    )
    add_box_visual(
        outer_body,
        name="body_left_lip",
        size=(BODY_LIP_W, BODY_D, LIP_THICK),
        xyz=(-BODY_W / 2.0 - BODY_LIP_W / 2.0, 0.0, BODY_SIDE_H - LIP_THICK),
        material=body_color,
    )
    add_box_visual(
        outer_body,
        name="body_right_lip",
        size=(BODY_LIP_W, BODY_D, LIP_THICK),
        xyz=(BODY_W / 2.0 + BODY_LIP_W / 2.0, 0.0, BODY_SIDE_H - LIP_THICK),
        material=body_color,
    )
    add_box_visual(
        outer_body,
        name="body_rear_lip",
        size=(BODY_W + 2.0 * BODY_LIP_W, BODY_LIP_W, LIP_THICK),
        xyz=(0.0, -BODY_D / 2.0 - BODY_LIP_W / 2.0, BODY_BACK_H - LIP_THICK),
        material=body_color,
    )
    for side_name, side in (("left", -1.0), ("right", 1.0)):
        add_box_visual(
            outer_body,
            name=f"body_{side_name}_front_cheek",
            size=(SHEET, BODY_FRONT_CHEEK_D, BODY_FRONT_CHEEK_H),
            xyz=(
                side * (BODY_W / 2.0 - SHEET / 2.0),
                BODY_D / 2.0 - BODY_FRONT_CHEEK_D / 2.0,
                0.0,
            ),
            material=body_color,
        )
        add_box_visual(
            outer_body,
            name=f"body_{side_name}_rail",
            size=(BODY_RAIL_W, BODY_RAIL_LEN, SHEET),
            xyz=(
                side * (BODY_W / 2.0 - SHEET - BODY_RAIL_W / 2.0),
                BODY_RAIL_Y,
                BODY_RAIL_Z,
            ),
            material=body_color,
        )
        add_box_visual(
            outer_body,
            name=f"body_{side_name}_cap",
            size=(BODY_CAP_W, BODY_RAIL_LEN, SHEET),
            xyz=(
                side * (BODY_W / 2.0 - SHEET - BODY_CAP_W / 2.0),
                BODY_RAIL_Y,
                BODY_CAP_Z,
            ),
            material=body_color,
        )
        add_box_visual(
            outer_body,
            name=f"body_{side_name}_stop",
            size=(BODY_STOP_W, BODY_STOP_D, BODY_STOP_H),
            xyz=(
                side * (BODY_W / 2.0 - SHEET - BODY_STOP_W / 2.0),
                BODY_D / 2.0 - BODY_FRONT_CHEEK_D - BODY_STOP_D / 2.0,
                BODY_FRONT_CHEEK_H - BODY_STOP_H,
            ),
            material=body_color,
        )
    outer_body.inertial = Inertial.from_geometry(
        Box((BODY_W + 2.0 * BODY_LIP_W, BODY_D, BODY_SIDE_H)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_SIDE_H / 2.0)),
    )

    middle_tray = model.part("middle_tray")
    add_tray_shell_visuals(
        middle_tray,
        prefix="middle",
        width=MIDDLE_W,
        depth=MIDDLE_D,
        side_height=MIDDLE_SIDE_H,
        front_height=0.0,
        rear_height=MIDDLE_SIDE_H,
        lip_width=MIDDLE_LIP_W,
        material=middle_color,
    )
    for side_name, side in (("left", -1.0), ("right", 1.0)):
        add_box_visual(
            middle_tray,
            name=f"middle_{side_name}_front_cheek",
            size=(SHEET, MIDDLE_FRONT_CHEEK_D, MIDDLE_FRONT_CHEEK_H),
            xyz=(
                side * (MIDDLE_W / 2.0 - SHEET / 2.0),
                MIDDLE_D / 2.0 - MIDDLE_FRONT_CHEEK_D / 2.0,
                0.0,
            ),
            material=middle_color,
        )
        add_box_visual(
            middle_tray,
            name=f"middle_{side_name}_runner",
            size=(MIDDLE_RUNNER_W, MIDDLE_RUNNER_LEN, MIDDLE_RUNNER_H),
            xyz=(
                side * (MIDDLE_W / 2.0 + MIDDLE_RUNNER_W / 2.0),
                MIDDLE_RUNNER_Y,
                MIDDLE_RUNNER_Z,
            ),
            material=middle_color,
        )
        add_box_visual(
            middle_tray,
            name=f"middle_{side_name}_moving_stop",
            size=(MIDDLE_STOP_W, MIDDLE_STOP_D, MIDDLE_STOP_H),
            xyz=(
                side * (MIDDLE_W / 2.0 + MIDDLE_STOP_W / 2.0),
                MIDDLE_STOP_Y,
                MIDDLE_RUNNER_Z + MIDDLE_RUNNER_H,
            ),
            material=middle_color,
        )
        add_box_visual(
            middle_tray,
            name=f"middle_{side_name}_guide",
            size=(MIDDLE_GUIDE_W, MIDDLE_GUIDE_LEN, SHEET),
            xyz=(
                side * (MIDDLE_W / 2.0 - SHEET - MIDDLE_GUIDE_W / 2.0),
                MIDDLE_GUIDE_Y,
                MIDDLE_GUIDE_Z,
            ),
            material=middle_color,
        )
        add_box_visual(
            middle_tray,
            name=f"middle_{side_name}_guide_cap",
            size=(MIDDLE_CAP_W, MIDDLE_GUIDE_LEN, SHEET),
            xyz=(
                side * (MIDDLE_W / 2.0 - SHEET - MIDDLE_CAP_W / 2.0),
                MIDDLE_GUIDE_Y,
                MIDDLE_CAP_Z,
            ),
            material=middle_color,
        )
        add_box_visual(
            middle_tray,
            name=f"middle_{side_name}_fixed_stop",
            size=(MIDDLE_FIXED_STOP_W, MIDDLE_FIXED_STOP_D, MIDDLE_FIXED_STOP_H),
            xyz=(
                side * (MIDDLE_W / 2.0 - SHEET - MIDDLE_GUIDE_W - MIDDLE_FIXED_STOP_W / 2.0),
                MIDDLE_FIXED_STOP_Y,
                MIDDLE_FIXED_STOP_Z + 0.001,
            ),
            material=middle_color,
        )
    middle_tray.inertial = Inertial.from_geometry(
        Box((MIDDLE_W + 2.0 * MIDDLE_RUNNER_W, MIDDLE_D + 2.0 * MIDDLE_LIP_W, MIDDLE_SIDE_H)),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SIDE_H / 2.0)),
    )

    inner_tray = model.part("inner_tray")
    add_tray_shell_visuals(
        inner_tray,
        prefix="inner",
        width=INNER_W,
        depth=INNER_D,
        side_height=INNER_SIDE_H,
        front_height=INNER_FRONT_H,
        rear_height=INNER_REAR_H,
        lip_width=INNER_LIP_W,
        material=inner_color,
    )
    for side_name, side in (("left", -1.0), ("right", 1.0)):
        add_box_visual(
            inner_tray,
            name=f"inner_{side_name}_runner",
            size=(INNER_RUNNER_W, INNER_RUNNER_LEN, INNER_RUNNER_H),
            xyz=(
                side * (INNER_W / 2.0 + INNER_RUNNER_W / 2.0),
                INNER_RUNNER_Y,
                INNER_RUNNER_Z,
            ),
            material=inner_color,
        )
        add_box_visual(
            inner_tray,
            name=f"inner_{side_name}_stop",
            size=(INNER_STOP_W, INNER_STOP_D, INNER_STOP_H),
            xyz=(
                side * (INNER_W / 2.0 + INNER_STOP_W / 2.0),
                INNER_STOP_Y,
                INNER_RUNNER_Z + INNER_RUNNER_H,
            ),
            material=inner_color,
        )
    inner_tray.inertial = Inertial.from_geometry(
        Box((INNER_W + 2.0 * INNER_RUNNER_W, INNER_D + 2.0 * INNER_LIP_W, INNER_FRONT_H)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, INNER_FRONT_H / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=middle_tray,
        origin=Origin(xyz=(0.0, MID_CLOSED_Y, MID_CLOSED_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.40, lower=0.0, upper=MID_TRAVEL),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_tray,
        child=inner_tray,
        origin=Origin(xyz=(0.0, INNER_CLOSED_Y, INNER_CLOSED_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=INNER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    middle_tray = object_model.get_part("middle_tray")
    inner_tray = object_model.get_part("inner_tray")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    body_left_rail = outer_body.get_visual("body_left_rail")
    body_left_cap = outer_body.get_visual("body_left_cap")
    middle_left_runner = middle_tray.get_visual("middle_left_runner")
    middle_left_moving_stop = middle_tray.get_visual("middle_left_moving_stop")
    middle_left_guide = middle_tray.get_visual("middle_left_guide")
    middle_left_guide_cap = middle_tray.get_visual("middle_left_guide_cap")
    middle_left_fixed_stop = middle_tray.get_visual("middle_left_fixed_stop")
    inner_left_runner = inner_tray.get_visual("inner_left_runner")
    body_left_stop = outer_body.get_visual("body_left_stop")

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
        "serial_prismatic_axes",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (0.0, 1.0, 0.0)
        and tuple(middle_to_inner.axis) == (0.0, 1.0, 0.0)
        and outer_to_middle.parent == outer_body.name
        and outer_to_middle.child == middle_tray.name
        and middle_to_inner.parent == middle_tray.name
        and middle_to_inner.child == inner_tray.name,
        "Expected two serial prismatic stages that extend together along +Y.",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_contact(
            outer_body,
            middle_tray,
            elem_a=body_left_rail,
            elem_b=middle_left_runner,
            name="middle_runner_supported_on_body_rail_closed",
        )
        ctx.expect_contact(
            outer_body,
            middle_tray,
            elem_a=body_left_cap,
            elem_b=middle_left_runner,
            name="middle_runner_captured_under_body_cap_closed",
        )
        ctx.expect_contact(
            middle_tray,
            inner_tray,
            elem_a=middle_left_guide,
            elem_b=inner_left_runner,
            name="inner_runner_supported_on_middle_guide_closed",
        )
        ctx.expect_contact(
            middle_tray,
            inner_tray,
            elem_a=middle_left_guide_cap,
            elem_b=inner_left_runner,
            name="inner_runner_captured_under_middle_cap_closed",
        )
        ctx.expect_within(
            middle_tray,
            outer_body,
            axes="xy",
            margin=0.0,
            name="middle_nested_inside_body_closed",
        )
        ctx.expect_within(
            inner_tray,
            middle_tray,
            axes="xy",
            margin=0.0,
            name="inner_nested_inside_middle_closed",
        )

    with ctx.pose({outer_to_middle: MID_TRAVEL, middle_to_inner: 0.0}):
        ctx.expect_contact(
            outer_body,
            middle_tray,
            elem_a=body_left_rail,
            elem_b=middle_left_runner,
            name="middle_runner_supported_on_body_rail_extended",
        )
        ctx.expect_overlap(
            outer_body,
            middle_tray,
            axes="y",
            min_overlap=0.18,
            name="middle_still_on_body_guides",
        )

    with ctx.pose({outer_to_middle: MID_TRAVEL, middle_to_inner: INNER_TRAVEL}):
        ctx.expect_contact(
            outer_body,
            middle_tray,
            elem_a=body_left_rail,
            elem_b=middle_left_runner,
            name="middle_runner_supported_on_body_rail_full_extension",
        )
        ctx.expect_contact(
            middle_tray,
            inner_tray,
            elem_a=middle_left_guide,
            elem_b=inner_left_runner,
            name="inner_runner_supported_on_middle_guide_full_extension",
        )
        ctx.expect_overlap(
            outer_body,
            middle_tray,
            axes="y",
            min_overlap=0.18,
            name="middle_support_overlap_full_extension",
        )
        ctx.expect_overlap(
            middle_tray,
            inner_tray,
            axes="y",
            min_overlap=0.06,
            name="inner_support_overlap_full_extension",
        )
        ctx.expect_overlap(
            middle_tray,
            inner_tray,
            axes="x",
            min_overlap=0.24,
            name="inner_stays_centered_on_middle_guides",
        )
        ctx.expect_contact(
            outer_body,
            middle_tray,
            elem_a=body_left_stop,
            elem_b=middle_left_moving_stop,
            name="outer_stop_catches_middle_stop_at_full_extension",
        )
        ctx.expect_gap(
            outer_body,
            middle_tray,
            axis="z",
            min_gap=0.001,
            positive_elem="body_left_stop",
            negative_elem="middle_left_runner",
            name="body_stop_stays_above_middle_runner",
        )
        ctx.expect_gap(
            middle_tray,
            inner_tray,
            axis="z",
            min_gap=0.001,
            positive_elem="middle_left_fixed_stop",
            negative_elem="inner_left_runner",
            name="middle_fixed_stop_stays_above_inner_runner",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
