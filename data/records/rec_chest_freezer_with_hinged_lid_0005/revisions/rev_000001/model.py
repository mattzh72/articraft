from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 1.10
BODY_D = 0.66
BODY_H = 0.83
SHELL_T = 0.05
BASE_T = 0.065

CAVITY_W = 0.88
CAVITY_D = 0.44
CAVITY_FLOOR_Z = 0.15
LINER_T = 0.012
COLLAR_T = 0.03

LID_W = 1.12
LID_D = 0.68
LID_TOP_T = 0.028
LID_SIDE_DROP = 0.058
LID_SIDE_T = 0.034
LID_PANEL_W = 0.84
LID_PANEL_D = 0.40
LID_PANEL_T = 0.016
GASKET_T = 0.008
GASKET_BAND = 0.028
HINGE_R = 0.014
HINGE_Y = BODY_D / 2.0 + 0.014
HINGE_Z = BODY_H + HINGE_R
GASKET_OUTER_W = BODY_W - 2.0 * SHELL_T - 0.02
GASKET_OUTER_D = BODY_D - 2.0 * SHELL_T - 0.04
GASKET_CENTER_Y = -HINGE_Y
TOP_BOTTOM_Z = BODY_H + 0.002
TOP_CENTER_Z_LOCAL = TOP_BOTTOM_Z + LID_TOP_T / 2.0 - HINGE_Z
SKIRT_CENTER_Z_LOCAL = TOP_BOTTOM_Z - LID_SIDE_DROP / 2.0 - HINGE_Z
PANEL_CENTER_Z_LOCAL = BODY_H - 0.020 - LID_PANEL_T / 2.0 - HINGE_Z
GASKET_CENTER_Z_LOCAL = BODY_H + GASKET_T / 2.0 - HINGE_Z
INNER_BRACE_T = (TOP_CENTER_Z_LOCAL - LID_TOP_T / 2.0) - (PANEL_CENTER_Z_LOCAL + LID_PANEL_T / 2.0)
INNER_BRACE_Z_LOCAL = PANEL_CENTER_Z_LOCAL + LID_PANEL_T / 2.0 + INNER_BRACE_T / 2.0


def _rpy_y_90() -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deep_freeze_chest", assets=ASSETS)

    enamel = model.material("enamel_white", rgba=(0.94, 0.95, 0.96, 1.0))
    liner = model.material("liner_white", rgba=(0.84, 0.87, 0.89, 1.0))
    trim = model.material("trim_gray", rgba=(0.73, 0.76, 0.79, 1.0))
    gasket = model.material("gasket_gray", rgba=(0.46, 0.48, 0.50, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.62, 0.65, 0.69, 1.0))

    body = model.part("body")
    body.visual(Box((BODY_W, BODY_D, BASE_T)), origin=Origin(xyz=(0.0, 0.0, BASE_T / 2.0)), material=enamel, name="floor")

    wall_h = BODY_H - BASE_T
    wall_z = BASE_T + wall_h / 2.0
    body.visual(
        Box((SHELL_T, BODY_D, wall_h)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SHELL_T / 2.0, 0.0, wall_z)),
        material=enamel,
        name="left_wall",
    )
    body.visual(
        Box((SHELL_T, BODY_D, wall_h)),
        origin=Origin(xyz=(BODY_W / 2.0 - SHELL_T / 2.0, 0.0, wall_z)),
        material=enamel,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, wall_h)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + SHELL_T / 2.0, wall_z)),
        material=enamel,
        name="front_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, wall_h)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - SHELL_T / 2.0, wall_z)),
        material=enamel,
        name="rear_wall",
    )

    liner_top_z = BODY_H - COLLAR_T
    liner_h = liner_top_z - CAVITY_FLOOR_Z
    liner_z = CAVITY_FLOOR_Z + liner_h / 2.0
    liner_outer_w = CAVITY_W + 2.0 * LINER_T
    liner_outer_d = CAVITY_D + 2.0 * LINER_T
    body.visual(
        Box((LINER_T, liner_outer_d, liner_h)),
        origin=Origin(xyz=(-CAVITY_W / 2.0 - LINER_T / 2.0, 0.0, liner_z)),
        material=liner,
        name="left_liner",
    )
    body.visual(
        Box((LINER_T, liner_outer_d, liner_h)),
        origin=Origin(xyz=(CAVITY_W / 2.0 + LINER_T / 2.0, 0.0, liner_z)),
        material=liner,
        name="right_liner",
    )
    body.visual(
        Box((CAVITY_W, LINER_T, liner_h)),
        origin=Origin(xyz=(0.0, -CAVITY_D / 2.0 - LINER_T / 2.0, liner_z)),
        material=liner,
        name="front_liner",
    )
    body.visual(
        Box((CAVITY_W, LINER_T, liner_h)),
        origin=Origin(xyz=(0.0, CAVITY_D / 2.0 + LINER_T / 2.0, liner_z)),
        material=liner,
        name="rear_liner",
    )
    body.visual(
        Box((CAVITY_W, CAVITY_D, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, CAVITY_FLOOR_Z - 0.007)),
        material=liner,
        name="liner_floor",
    )

    shell_inner_x = BODY_W / 2.0 - SHELL_T
    shell_inner_y = BODY_D / 2.0 - SHELL_T
    liner_outer_x = CAVITY_W / 2.0 + LINER_T
    liner_outer_y = CAVITY_D / 2.0 + LINER_T
    collar_z = BODY_H - COLLAR_T / 2.0
    body.visual(
        Box((shell_inner_x - liner_outer_x, liner_outer_d, COLLAR_T)),
        origin=Origin(xyz=(-(shell_inner_x + liner_outer_x) / 2.0, 0.0, collar_z)),
        material=trim,
        name="left_collar",
    )
    body.visual(
        Box((shell_inner_x - liner_outer_x, liner_outer_d, COLLAR_T)),
        origin=Origin(xyz=((shell_inner_x + liner_outer_x) / 2.0, 0.0, collar_z)),
        material=trim,
        name="right_collar",
    )
    body.visual(
        Box((CAVITY_W, shell_inner_y - liner_outer_y, COLLAR_T)),
        origin=Origin(xyz=(0.0, -(shell_inner_y + liner_outer_y) / 2.0, collar_z)),
        material=trim,
        name="front_collar",
    )
    body.visual(
        Box((CAVITY_W, shell_inner_y - liner_outer_y, COLLAR_T)),
        origin=Origin(xyz=(0.0, (shell_inner_y + liner_outer_y) / 2.0, collar_z)),
        material=trim,
        name="rear_collar",
    )
    body.visual(
        Box((BODY_W - 0.08, 0.028, 0.024)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 + 0.012, BODY_H - 0.012)),
        material=hinge_metal,
        name="body_hinge_strip",
    )

    foot = Box((0.09, 0.04, 0.025))
    for foot_name, x, y in (
        ("foot_fl", -BODY_W / 2.0 + 0.12, -BODY_D / 2.0 + 0.08),
        ("foot_fr", BODY_W / 2.0 - 0.12, -BODY_D / 2.0 + 0.08),
        ("foot_rl", -BODY_W / 2.0 + 0.12, BODY_D / 2.0 - 0.09),
        ("foot_rr", BODY_W / 2.0 - 0.12, BODY_D / 2.0 - 0.09),
    ):
        body.visual(foot, origin=Origin(xyz=(x, y, 0.0125)), material=trim, name=foot_name)

    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, LID_TOP_T)),
        origin=Origin(xyz=(0.0, -LID_D / 2.0, TOP_CENTER_Z_LOCAL)),
        material=enamel,
        name="top_shell",
    )
    lid.visual(
        Box((LID_W, LID_SIDE_T, LID_SIDE_DROP)),
        origin=Origin(
            xyz=(0.0, -LID_D - LID_SIDE_T / 2.0, SKIRT_CENTER_Z_LOCAL),
        ),
        material=enamel,
        name="front_skirt",
    )
    lid.visual(
        Box((LID_SIDE_T, LID_D - 2.0 * LID_SIDE_T, LID_SIDE_DROP)),
        origin=Origin(
            xyz=(-(LID_W / 2.0 + LID_SIDE_T / 2.0), -LID_D / 2.0, SKIRT_CENTER_Z_LOCAL),
        ),
        material=enamel,
        name="left_skirt",
    )
    lid.visual(
        Box((LID_SIDE_T, LID_D - 2.0 * LID_SIDE_T, LID_SIDE_DROP)),
        origin=Origin(
            xyz=((LID_W / 2.0 + LID_SIDE_T / 2.0), -LID_D / 2.0, SKIRT_CENTER_Z_LOCAL),
        ),
        material=enamel,
        name="right_skirt",
    )
    lid.visual(
        Box((LID_PANEL_W, LID_PANEL_D, LID_PANEL_T)),
        origin=Origin(xyz=(0.0, GASKET_CENTER_Y, PANEL_CENTER_Z_LOCAL)),
        material=liner,
        name="inner_panel",
    )
    lid.visual(
        Box((0.10, 0.10, INNER_BRACE_T)),
        origin=Origin(xyz=(0.0, GASKET_CENTER_Y, INNER_BRACE_Z_LOCAL)),
        material=liner,
        name="inner_brace",
    )
    lid.visual(
        Box((GASKET_OUTER_W, GASKET_BAND, GASKET_T)),
        origin=Origin(
            xyz=(
                0.0,
                GASKET_CENTER_Y - (GASKET_OUTER_D / 2.0 - GASKET_BAND / 2.0),
                GASKET_CENTER_Z_LOCAL,
            )
        ),
        material=gasket,
        name="front_gasket",
    )
    lid.visual(
        Box((GASKET_OUTER_W, GASKET_BAND, GASKET_T)),
        origin=Origin(
            xyz=(
                0.0,
                GASKET_CENTER_Y + (GASKET_OUTER_D / 2.0 - GASKET_BAND / 2.0),
                GASKET_CENTER_Z_LOCAL,
            )
        ),
        material=gasket,
        name="rear_gasket",
    )
    lid.visual(
        Box((GASKET_BAND, GASKET_OUTER_D - 2.0 * GASKET_BAND, GASKET_T)),
        origin=Origin(
            xyz=(
                -(GASKET_OUTER_W / 2.0 - GASKET_BAND / 2.0),
                GASKET_CENTER_Y,
                GASKET_CENTER_Z_LOCAL,
            )
        ),
        material=gasket,
        name="left_gasket",
    )
    lid.visual(
        Box((GASKET_BAND, GASKET_OUTER_D - 2.0 * GASKET_BAND, GASKET_T)),
        origin=Origin(
            xyz=(
                (GASKET_OUTER_W / 2.0 - GASKET_BAND / 2.0),
                GASKET_CENTER_Y,
                GASKET_CENTER_Z_LOCAL,
            )
        ),
        material=gasket,
        name="right_gasket",
    )
    lid.visual(
        Box((0.30, 0.024, 0.014)),
        origin=Origin(
            xyz=(0.0, -LID_D - 0.012, BODY_H - 0.018 - HINGE_Z),
        ),
        material=trim,
        name="front_handle",
    )
    lid.visual(
        Cylinder(radius=HINGE_R, length=LID_W - 0.05),
        origin=Origin(rpy=_rpy_y_90()),
        material=hinge_metal,
        name="hinge_barrel",
    )

    lid.inertial = Inertial.from_geometry(
        Box((LID_W, LID_D, LID_TOP_T + LID_SIDE_DROP)),
        mass=8.5,
        origin=Origin(
            xyz=(0.0, -LID_D / 2.0, BODY_H + 0.002 + (LID_TOP_T - LID_SIDE_DROP) / 2.0 - HINGE_Z),
        ),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")

    front_collar = body.get_visual("front_collar")
    rear_collar = body.get_visual("rear_collar")
    liner_floor = body.get_visual("liner_floor")
    hinge_strip = body.get_visual("body_hinge_strip")

    front_gasket = lid.get_visual("front_gasket")
    rear_gasket = lid.get_visual("rear_gasket")
    left_gasket = lid.get_visual("left_gasket")
    right_gasket = lid.get_visual("right_gasket")
    inner_panel = lid.get_visual("inner_panel")
    hinge_barrel = lid.get_visual("hinge_barrel")
    top_shell = lid.get_visual("top_shell")
    front_handle = lid.get_visual("front_handle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=8)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    limits = rear_hinge.motion_limits
    ctx.check(
        "rear_hinge_is_revolute",
        rear_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulation_type={rear_hinge.articulation_type}",
    )
    ctx.check(
        "rear_hinge_axis_runs_full_width",
        rear_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"axis={rear_hinge.axis}",
    )
    ctx.check(
        "rear_hinge_has_realistic_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and 1.05 <= limits.upper <= 1.30,
        details=f"limits={limits}",
    )

    ctx.expect_overlap(body, lid, axes="xy", elem_a=liner_floor, elem_b=top_shell, min_overlap=0.40, name="lid_covers_opening")
    ctx.expect_within(
        lid,
        body,
        axes="xy",
        inner_elem=inner_panel,
        outer_elem=liner_floor,
        margin=0.0,
        name="underside_panel_stays_within_cavity",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a=hinge_barrel,
        elem_b=hinge_strip,
        name="visible_rear_hinge_remains_mounted",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="x",
        elem_a=hinge_barrel,
        elem_b=hinge_strip,
        min_overlap=0.95,
        name="rear_hinge_spans_almost_full_width",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0015,
        positive_elem=front_gasket,
        negative_elem=front_collar,
        name="front_gasket_seats_on_collar",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0015,
        positive_elem=rear_gasket,
        negative_elem=rear_collar,
        name="rear_gasket_seats_on_collar",
    )
    ctx.expect_within(
        lid,
        lid,
        axes="x",
        inner_elem=left_gasket,
        outer_elem=top_shell,
        margin=0.0,
        name="gasket_left_edge_stays_under_lid",
    )
    ctx.expect_within(
        lid,
        lid,
        axes="x",
        inner_elem=right_gasket,
        outer_elem=top_shell,
        margin=0.0,
        name="gasket_right_edge_stays_under_lid",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="x",
        elem_a=front_handle,
        elem_b=top_shell,
        min_overlap=0.20,
        name="front_handle_is_centered_on_lid",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({rear_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_hinge_lower_no_floating")
            ctx.expect_contact(
                lid,
                body,
                elem_a=hinge_barrel,
                elem_b=hinge_strip,
                name="hinge_connected_at_rest",
            )
        with ctx.pose({rear_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="rear_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="rear_hinge_upper_no_floating")
            ctx.expect_contact(
                lid,
                body,
                elem_a=hinge_barrel,
                elem_b=hinge_strip,
                name="hinge_connected_when_open",
            )
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                min_gap=0.22,
                positive_elem=front_gasket,
                negative_elem=front_collar,
                name="front_edge_lifts_clear_when_open",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
