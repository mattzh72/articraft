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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 0.28
BODY_H = 0.35
BODY_D = 0.13
BACK_T = 0.006
WALL_T = 0.006
BOTTOM_T = 0.006
ROOF_T = 0.008
ROOF_OVERHANG = 0.022
FRAME_D = 0.012
HEADER_T = 0.012
SILL_T = 0.012
DOOR_T = 0.008
HINGE_R = 0.0045
DOOR_W = BODY_W - (2.0 * WALL_T) - 0.010
DOOR_H = BODY_H - ROOF_T - HEADER_T - SILL_T
HANDLE_W = 0.09
HANDLE_D = 0.014
HANDLE_H = 0.016
DOOR_SWING = math.radians(80.0)


def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: str) -> float:
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mailbox", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.24, 0.29, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.14, 0.17, 1.0))
    satin_handle = model.material("satin_handle", rgba=(0.62, 0.64, 0.66, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BACK_T, BODY_H)),
        origin=Origin(xyz=(0.0, BACK_T / 2.0, BODY_H / 2.0)),
        material=painted_steel,
        name="back_panel",
    )
    body.visual(
        Box((WALL_T, BODY_D - BACK_T, BODY_H - ROOF_T)),
        origin=Origin(
            xyz=(
                (BODY_W / 2.0) - (WALL_T / 2.0),
                BACK_T + ((BODY_D - BACK_T) / 2.0),
                (BODY_H - ROOF_T) / 2.0,
            )
        ),
        material=painted_steel,
        name="right_wall",
    )
    body.visual(
        Box((WALL_T, BODY_D - BACK_T, BODY_H - ROOF_T)),
        origin=Origin(
            xyz=(
                -((BODY_W / 2.0) - (WALL_T / 2.0)),
                BACK_T + ((BODY_D - BACK_T) / 2.0),
                (BODY_H - ROOF_T) / 2.0,
            )
        ),
        material=painted_steel,
        name="left_wall",
    )
    body.visual(
        Box((BODY_W - (2.0 * WALL_T), BODY_D - BACK_T, BOTTOM_T)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_T + ((BODY_D - BACK_T) / 2.0),
                BOTTOM_T / 2.0,
            )
        ),
        material=painted_steel,
        name="bottom_floor",
    )
    body.visual(
        Box((BODY_W, BODY_D + ROOF_OVERHANG, ROOF_T)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_D + ROOF_OVERHANG) / 2.0,
                BODY_H - (ROOF_T / 2.0),
            )
        ),
        material=painted_steel,
        name="roof_panel",
    )
    body.visual(
        Box((BODY_W - (2.0 * WALL_T), FRAME_D, HEADER_T)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D - (FRAME_D / 2.0),
                BODY_H - ROOF_T - (HEADER_T / 2.0),
            )
        ),
        material=dark_trim,
        name="top_header",
    )
    body.visual(
        Box((BODY_W - (2.0 * WALL_T), FRAME_D, SILL_T)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D - (FRAME_D / 2.0),
                SILL_T / 2.0,
            )
        ),
        material=dark_trim,
        name="bottom_sill",
    )
    body.visual(
        Cylinder(radius=HINGE_R, length=DOOR_W - 0.016),
        origin=Origin(
            xyz=(
                0.0,
                BODY_D + (DOOR_T / 2.0),
                SILL_T - HINGE_R,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="hinge_barrel",
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, HINGE_R + (DOOR_H / 2.0))),
        material=dark_trim,
        name="door_panel",
    )
    door.visual(
        Box((HANDLE_W, HANDLE_D, HANDLE_H)),
        origin=Origin(
            xyz=(
                0.0,
                (DOOR_T / 2.0) + (HANDLE_D / 2.0),
                HINGE_R + (DOOR_H * 0.68),
            )
        ),
        material=satin_handle,
        name="pull_handle",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, BODY_D + (DOOR_T / 2.0), SILL_T - HINGE_R)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-DOOR_SWING,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")
    back_panel = body.get_visual("back_panel")
    roof_panel = body.get_visual("roof_panel")
    top_header = body.get_visual("top_header")
    bottom_sill = body.get_visual("bottom_sill")
    hinge_barrel = body.get_visual("hinge_barrel")
    door_panel = door.get_visual("door_panel")
    pull_handle = door.get_visual("pull_handle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "door hinge axis runs left-to-right",
        tuple(round(v, 6) for v in door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis hinge, got {door_hinge.axis}",
    )
    ctx.check(
        "door hinge opens outward about eighty degrees",
        abs(door_hinge.motion_limits.lower + DOOR_SWING) < 1e-6
        and abs(door_hinge.motion_limits.upper) < 1e-6,
        details=(
            f"expected limits [{-DOOR_SWING}, 0.0], "
            f"got [{door_hinge.motion_limits.lower}, {door_hinge.motion_limits.upper}]"
        ),
    )

    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=door_panel,
        negative_elem=bottom_sill,
        name="door closes flush to the front sill",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=door_panel,
        elem_b=hinge_barrel,
        name="door makes physical contact at the bottom hinge edge",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="x",
        min_overlap=DOOR_W - 0.001,
        elem_a=door_panel,
        elem_b=back_panel,
        name="door spans nearly the full mailbox width",
    )
    ctx.expect_within(
        door,
        body,
        axes="x",
        margin=0.0,
        inner_elem=door_panel,
        outer_elem=back_panel,
        name="door stays within the body width",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a=door_panel,
        elem_b=top_header,
        name="door reaches the top header when shut",
    )

    back_panel_aabb = ctx.part_element_world_aabb(body, elem=back_panel)
    roof_panel_aabb = ctx.part_element_world_aabb(body, elem=roof_panel)
    door_panel_aabb = ctx.part_element_world_aabb(door, elem=door_panel)
    pull_handle_aabb = ctx.part_element_world_aabb(door, elem=pull_handle)

    if back_panel_aabb is not None and roof_panel_aabb is not None:
        roof_overhang = roof_panel_aabb[1][1] - BODY_D
        ctx.check(
            "roof projects forward past the box body",
            roof_overhang >= (ROOF_OVERHANG - 0.001),
            details=f"expected at least {ROOF_OVERHANG - 0.001} m overhang, got {roof_overhang}",
        )
        ctx.check(
            "back panel stays flat against the wall side",
            _span(back_panel_aabb, "y") <= BACK_T + 1e-6,
            details=f"back panel depth should stay near {BACK_T}, got {_span(back_panel_aabb, 'y')}",
        )

    if door_panel_aabb is not None:
        door_width = _span(door_panel_aabb, "x")
        door_height = _span(door_panel_aabb, "z")
        width_ratio = door_width / BODY_W
        height_ratio = door_height / (BODY_H - ROOF_T)
        ctx.check(
            "front flap covers most of the front opening",
            width_ratio >= 0.90 and height_ratio >= 0.92,
            details=(
                "door panel should dominate the front face, "
                f"got {door_width} m by {door_height} m "
                f"({width_ratio:.1%} of width, {height_ratio:.1%} of height)"
            ),
        )

    with ctx.pose({door_hinge: -DOOR_SWING}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened door clears the housing")
        open_handle_aabb = ctx.part_element_world_aabb(door, elem=pull_handle)
        body_aabb = ctx.part_world_aabb(body)
        if open_handle_aabb is not None and body_aabb is not None:
            handle_front_gap = open_handle_aabb[0][1] - body_aabb[1][1]
            ctx.check(
                "opened flap swings clearly outward from the body",
                handle_front_gap >= 0.10,
                details=f"expected opened handle to sit at least 0.10 m in front, got {handle_front_gap}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
