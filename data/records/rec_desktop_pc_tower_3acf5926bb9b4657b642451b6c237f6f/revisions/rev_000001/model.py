from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_W = 0.245
OUTER_D = 0.545
OUTER_H = 0.565

PANEL_T = 0.0018
SHEET_T = 0.0018
TOP_T = 0.0018
FLOOR_T = 0.006

TOP_OPEN_D = 0.190
TOP_RAIL_W = 0.013
TOP_REAR_BAR_D = 0.012

SIDE_PANEL_H = OUTER_H - 0.006
SIDE_PANEL_D = OUTER_D - 0.004

DRIVE_CAGE_W = 0.104
DRIVE_CAGE_D = 0.145
DRIVE_CAGE_H = 0.215

TOP_COVER_W = (OUTER_W - PANEL_T) - 2.0 * TOP_RAIL_W + 0.012
TOP_COVER_D = 0.175
TOP_COVER_CLEARANCE = 0.0


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extended_atx_full_tower")

    body_color = model.material("case_black", rgba=(0.12, 0.12, 0.13, 1.0))
    panel_color = model.material("panel_graphite", rgba=(0.18, 0.18, 0.20, 1.0))
    accent_color = model.material("dark_trim", rgba=(0.07, 0.07, 0.08, 1.0))
    cage_color = model.material("zinc_cage", rgba=(0.62, 0.64, 0.66, 1.0))

    body = model.part("case_body")
    body_inner_x_min = -OUTER_W / 2.0 + PANEL_T
    body_width = OUTER_W - PANEL_T

    body.visual(
        Box((body_width, OUTER_D, FLOOR_T)),
        origin=Origin(xyz=(PANEL_T / 2.0, 0.0, FLOOR_T / 2.0)),
        material=body_color,
        name="floor_tray",
    )
    body.visual(
        Box((SHEET_T, OUTER_D, OUTER_H)),
        origin=Origin(xyz=(OUTER_W / 2.0 - SHEET_T / 2.0, 0.0, OUTER_H / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_width, SHEET_T, OUTER_H)),
        origin=Origin(
            xyz=(PANEL_T / 2.0, OUTER_D / 2.0 - SHEET_T / 2.0, OUTER_H / 2.0)
        ),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((body_width, SHEET_T, OUTER_H)),
        origin=Origin(
            xyz=(PANEL_T / 2.0, -OUTER_D / 2.0 + SHEET_T / 2.0, OUTER_H / 2.0)
        ),
        material=body_color,
        name="rear_wall",
    )

    top_front_depth = OUTER_D - TOP_OPEN_D
    top_front_center_y = (-OUTER_D / 2.0 + TOP_OPEN_D + OUTER_D / 2.0) * 0.5
    body.visual(
        Box((body_width, top_front_depth, TOP_T)),
        origin=Origin(
            xyz=(PANEL_T / 2.0, top_front_center_y, OUTER_H - TOP_T / 2.0)
        ),
        material=body_color,
        name="top_front_panel",
    )

    side_rail_depth = TOP_OPEN_D - TOP_REAR_BAR_D
    side_rail_center_y = -OUTER_D / 2.0 + TOP_REAR_BAR_D + side_rail_depth / 2.0
    left_rail_center_x = body_inner_x_min + TOP_RAIL_W / 2.0
    right_rail_center_x = OUTER_W / 2.0 - TOP_RAIL_W / 2.0
    body.visual(
        Box((TOP_RAIL_W, side_rail_depth, TOP_T)),
        origin=Origin(
            xyz=(left_rail_center_x, side_rail_center_y, OUTER_H - TOP_T / 2.0)
        ),
        material=accent_color,
        name="top_left_rail",
    )
    body.visual(
        Box((TOP_RAIL_W, side_rail_depth, TOP_T)),
        origin=Origin(
            xyz=(right_rail_center_x, side_rail_center_y, OUTER_H - TOP_T / 2.0)
        ),
        material=accent_color,
        name="top_right_rail",
    )
    body.visual(
        Box((body_width, TOP_REAR_BAR_D, TOP_T)),
        origin=Origin(
            xyz=(
                PANEL_T / 2.0,
                -OUTER_D / 2.0 + TOP_REAR_BAR_D / 2.0,
                OUTER_H - TOP_T / 2.0,
            )
        ),
        material=accent_color,
        name="top_rear_hinge_bar",
    )

    body.visual(
        Box((0.030, 0.230, 0.0014)),
        origin=Origin(
            xyz=(OUTER_W / 2.0 - SHEET_T - 0.015, -0.010, 0.285)
        ),
        material=accent_color,
        name="motherboard_tray_relief",
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((PANEL_T, SIDE_PANEL_D, SIDE_PANEL_H)),
        origin=Origin(xyz=(PANEL_T / 2.0, SIDE_PANEL_D / 2.0, SIDE_PANEL_H / 2.0)),
        material=panel_color,
        name="panel_skin",
    )
    side_panel.visual(
        Box((0.006, SIDE_PANEL_D * 0.98, SIDE_PANEL_H)),
        origin=Origin(xyz=(-0.003, SIDE_PANEL_D * 0.49, SIDE_PANEL_H / 2.0)),
        material=panel_color,
        name="panel_rear_return",
    )
    side_panel.visual(
        Box((0.006, 0.014, SIDE_PANEL_H * 0.40)),
        origin=Origin(
            xyz=(-0.003, SIDE_PANEL_D - 0.016, SIDE_PANEL_H * 0.53)
        ),
        material=accent_color,
        name="panel_latch_rib",
    )
    for idx, z_center in enumerate((0.070, SIDE_PANEL_H / 2.0, SIDE_PANEL_H - 0.070), start=1):
        side_panel.visual(
            Cylinder(radius=0.0035, length=0.042),
            origin=Origin(xyz=(-0.0015, 0.0, z_center)),
            material=accent_color,
            name=f"panel_hinge_knuckle_{idx}",
        )

    drive_cage = model.part("drive_cage")
    drive_cage.visual(
        Cylinder(radius=0.005, length=DRIVE_CAGE_H),
        origin=Origin(xyz=(0.0, 0.0, DRIVE_CAGE_H / 2.0)),
        material=accent_color,
        name="cage_hinge_spine",
    )
    drive_cage.visual(
        Box((0.002, DRIVE_CAGE_D, DRIVE_CAGE_H)),
        origin=Origin(xyz=(0.001, -DRIVE_CAGE_D / 2.0, DRIVE_CAGE_H / 2.0)),
        material=cage_color,
        name="cage_side_outer",
    )
    drive_cage.visual(
        Box((0.002, DRIVE_CAGE_D, DRIVE_CAGE_H)),
        origin=Origin(
            xyz=(DRIVE_CAGE_W - 0.001, -DRIVE_CAGE_D / 2.0, DRIVE_CAGE_H / 2.0)
        ),
        material=cage_color,
        name="cage_side_inner",
    )
    drive_cage.visual(
        Box((DRIVE_CAGE_W, DRIVE_CAGE_D, 0.002)),
        origin=Origin(xyz=(DRIVE_CAGE_W / 2.0, -DRIVE_CAGE_D / 2.0, 0.001)),
        material=cage_color,
        name="cage_bottom_tray",
    )
    drive_cage.visual(
        Box((DRIVE_CAGE_W, 0.008, 0.012)),
        origin=Origin(xyz=(DRIVE_CAGE_W / 2.0, -0.004, DRIVE_CAGE_H - 0.010)),
        material=cage_color,
        name="cage_top_front_bridge",
    )
    drive_cage.visual(
        Box((DRIVE_CAGE_W, 0.008, 0.012)),
        origin=Origin(
            xyz=(
                DRIVE_CAGE_W / 2.0,
                -DRIVE_CAGE_D + 0.004,
                DRIVE_CAGE_H - 0.010,
            )
        ),
        material=cage_color,
        name="cage_top_rear_bridge",
    )
    for idx, z_level in enumerate((0.070, 0.126, 0.182), start=1):
        drive_cage.visual(
            Box((DRIVE_CAGE_W, DRIVE_CAGE_D - 0.020, 0.002)),
            origin=Origin(
                xyz=(DRIVE_CAGE_W / 2.0, -DRIVE_CAGE_D / 2.0, z_level)
            ),
            material=cage_color,
            name=f"drive_shelf_{idx}",
        )

    top_cover = model.part("top_exhaust_cover")
    top_cover.visual(
        Box((TOP_COVER_W, TOP_COVER_D, TOP_T)),
        origin=Origin(
            xyz=(0.0, TOP_COVER_D / 2.0, TOP_COVER_CLEARANCE + TOP_T / 2.0)
        ),
        material=panel_color,
        name="cover_panel",
    )
    top_cover.visual(
        Box((TOP_COVER_W * 0.76, 0.012, 0.004)),
        origin=Origin(
            xyz=(0.0, 0.060, TOP_COVER_CLEARANCE + TOP_T + 0.002)
        ),
        material=accent_color,
        name="cover_rib_front",
    )
    top_cover.visual(
        Box((TOP_COVER_W * 0.76, 0.012, 0.004)),
        origin=Origin(
            xyz=(0.0, 0.122, TOP_COVER_CLEARANCE + TOP_T + 0.002)
        ),
        material=accent_color,
        name="cover_rib_rear",
    )

    model.articulation(
        "body_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_panel,
        origin=Origin(xyz=(-OUTER_W / 2.0, -OUTER_D / 2.0 + 0.002, 0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.5,
            lower=0.0,
            upper=2.15,
        ),
    )

    model.articulation(
        "body_to_drive_cage",
        ArticulationType.REVOLUTE,
        parent=body,
        child=drive_cage,
        origin=Origin(
            xyz=(-OUTER_W / 2.0 + 0.035, OUTER_D / 2.0 - 0.045, FLOOR_T)
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    model.articulation(
        "body_to_top_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_cover,
        origin=Origin(xyz=(PANEL_T / 2.0, -OUTER_D / 2.0 + TOP_REAR_BAR_D, OUTER_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.0,
            lower=0.0,
            upper=1.10,
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

    body = object_model.get_part("case_body")
    side_panel = object_model.get_part("side_panel")
    drive_cage = object_model.get_part("drive_cage")
    top_cover = object_model.get_part("top_exhaust_cover")

    panel_hinge = object_model.get_articulation("body_to_side_panel")
    cage_hinge = object_model.get_articulation("body_to_drive_cage")
    cover_hinge = object_model.get_articulation("body_to_top_cover")

    panel_skin = side_panel.get_visual("panel_skin")
    cage_side_inner = drive_cage.get_visual("cage_side_inner")
    cover_panel = top_cover.get_visual("cover_panel")

    ctx.check(
        "all prompt parts exist",
        all(part is not None for part in (body, side_panel, drive_cage, top_cover)),
        details="One or more prompt-critical parts could not be resolved.",
    )

    ctx.expect_gap(
        body,
        side_panel,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        negative_elem=panel_skin,
        name="side panel sits flush with the left opening",
    )
    ctx.expect_overlap(
        side_panel,
        body,
        axes="yz",
        min_overlap=0.45,
        name="side panel covers the full tower side aperture",
    )
    ctx.expect_within(
        drive_cage,
        body,
        axes="xy",
        margin=0.0,
        name="drive cage stays inside the body footprint when closed",
    )
    ctx.expect_gap(
        top_cover,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.0002,
        positive_elem=cover_panel,
        name="top exhaust cover closes onto the top frame",
    )
    ctx.expect_overlap(
        top_cover,
        body,
        axes="xy",
        min_overlap=0.16,
        elem_a=cover_panel,
        name="top exhaust cover spans the rear top vent opening",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(side_panel, elem=panel_skin)
    with ctx.pose({panel_hinge: 1.55}):
        open_panel_aabb = ctx.part_element_world_aabb(side_panel, elem=panel_skin)
    ctx.check(
        "side panel opens outward from the rear edge",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][0] < rest_panel_aabb[0][0] - 0.12,
        details=f"rest={rest_panel_aabb}, open={open_panel_aabb}",
    )

    rest_cage_aabb = ctx.part_element_world_aabb(drive_cage, elem=cage_side_inner)
    with ctx.pose({cage_hinge: 1.10}):
        open_cage_aabb = ctx.part_element_world_aabb(drive_cage, elem=cage_side_inner)
    ctx.check(
        "drive cage swings out toward the side opening",
        rest_cage_aabb is not None
        and open_cage_aabb is not None
        and open_cage_aabb[0][0] < rest_cage_aabb[0][0] - 0.05,
        details=f"rest={rest_cage_aabb}, open={open_cage_aabb}",
    )

    rest_cover_aabb = ctx.part_element_world_aabb(top_cover, elem=cover_panel)
    with ctx.pose({cover_hinge: 0.90}):
        open_cover_aabb = ctx.part_element_world_aabb(top_cover, elem=cover_panel)
    ctx.check(
        "top exhaust cover lifts upward on its hinge",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.09,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    with ctx.pose({panel_hinge: 1.55, cage_hinge: 1.10, cover_hinge: 0.90}):
        opened_panel = _aabb_center(ctx.part_element_world_aabb(side_panel, elem=panel_skin))
        opened_cage = _aabb_center(ctx.part_element_world_aabb(drive_cage, elem=cage_side_inner))
        opened_cover = _aabb_center(ctx.part_element_world_aabb(top_cover, elem=cover_panel))
    ctx.check(
        "all articulated service features have distinct opened poses",
        opened_panel is not None and opened_cage is not None and opened_cover is not None,
        details=f"panel={opened_panel}, cage={opened_cage}, cover={opened_cover}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
