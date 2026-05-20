from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


DESK_WIDTH = 1.60
DESK_DEPTH = 0.78
TOP_THICKNESS = 0.035
CARCASS_WIDTH = 1.54
CARCASS_DEPTH = 0.73
CARCASS_HEIGHT = 0.725
PEDESTAL_WIDTH = 0.39
PANEL_THICKNESS = 0.018
BACK_PANEL_THICKNESS = 0.012
BOTTOM_PANEL_THICKNESS = 0.022
MODESTY_PANEL_THICKNESS = 0.015
DRAWER_FRONT_THICKNESS = 0.020
DRAWER_TOTAL_DEPTH = 0.59
DRAWER_SHELL_DEPTH = 0.57
DRAWER_SHELL_WIDTH = 0.312
DRAWER_SIDE_THICKNESS = 0.012
RUNNER_THICKNESS = 0.009
RUNNER_HEIGHT = 0.028
RUNNER_DEPTH = 0.50
BODY_RAIL_THICKNESS = 0.012
BODY_RAIL_DEPTH = 0.52
DRAWER_EXTENSION = 0.24
DRAWER_FRONT_WIDTH = 0.348
DRAWER_CENTER_Y = (CARCASS_DEPTH * 0.5) - (DRAWER_TOTAL_DEPTH * 0.5)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _drawer_front_meshes(width: float, height: float, handle_width: float, handle_height: float, prefix: str):
    panel = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(width, height),
            [superellipse_profile(handle_width, handle_height, exponent=2.0, segments=40)],
            height=DRAWER_FRONT_THICKNESS,
            center=True,
        ).rotate_x(pi / 2.0),
        f"{prefix}_front_panel",
    )
    bezel = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(handle_width + 0.014, handle_height + 0.010, exponent=2.0, segments=40),
            [superellipse_profile(handle_width - 0.016, handle_height - 0.014, exponent=2.0, segments=40)],
            height=0.008,
            center=True,
        ).rotate_x(pi / 2.0),
        f"{prefix}_pull_bezel",
    )
    pocket = mesh_from_geometry(
        ExtrudeGeometry(
            superellipse_profile(handle_width + 0.010, handle_height + 0.008, exponent=2.0, segments=40),
            height=0.018,
            center=True,
        ).rotate_x(pi / 2.0),
        f"{prefix}_pull_pocket",
    )
    return panel, bezel, pocket


def _add_drawer(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    x_center: float,
    z_center: float,
    opening_bottom: float,
    front_height: float,
    shell_height: float,
    front_material,
    case_material,
    runner_material,
    bezel_material,
    pocket_material,
) -> None:
    drawer = model.part(name)
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_WIDTH, DRAWER_TOTAL_DEPTH, front_height)),
        mass=4.0 + 18.0 * front_height,
    )

    handle_width = min(0.125, DRAWER_FRONT_WIDTH * 0.34)
    handle_height = min(0.038, max(0.028, front_height * 0.18))
    front_panel_mesh, bezel_mesh, pocket_mesh = _drawer_front_meshes(
        DRAWER_FRONT_WIDTH,
        front_height,
        handle_width,
        handle_height,
        name,
    )

    front_center_y = (DRAWER_TOTAL_DEPTH * 0.5) - (DRAWER_FRONT_THICKNESS * 0.5)
    shell_front_y = front_center_y - (DRAWER_FRONT_THICKNESS * 0.5)
    side_length = DRAWER_SHELL_DEPTH - BACK_PANEL_THICKNESS
    side_center_y = shell_front_y - (side_length * 0.5)
    back_center_y = shell_front_y - DRAWER_SHELL_DEPTH + (BACK_PANEL_THICKNESS * 0.5)
    runner_local_z = (opening_bottom + 0.055) - z_center

    drawer.visual(
        front_panel_mesh,
        origin=Origin(xyz=(0.0, front_center_y, 0.0)),
        material=front_material,
        name="front_panel",
    )
    drawer.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, front_center_y - 0.006, 0.0)),
        material=bezel_material,
        name="pull_bezel",
    )
    drawer.visual(
        pocket_mesh,
        origin=Origin(xyz=(0.0, front_center_y - 0.019, 0.0)),
        material=pocket_material,
        name="pull_pocket",
    )

    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, side_length, shell_height)),
        origin=Origin(
            xyz=(
                -(DRAWER_SHELL_WIDTH * 0.5) + (DRAWER_SIDE_THICKNESS * 0.5),
                side_center_y,
                0.0,
            )
        ),
        material=case_material,
        name="left_side",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, side_length, shell_height)),
        origin=Origin(
            xyz=(
                (DRAWER_SHELL_WIDTH * 0.5) - (DRAWER_SIDE_THICKNESS * 0.5),
                side_center_y,
                0.0,
            )
        ),
        material=case_material,
        name="right_side",
    )
    drawer.visual(
        Box((DRAWER_SHELL_WIDTH - (2.0 * DRAWER_SIDE_THICKNESS), side_length, DRAWER_SIDE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                side_center_y,
                -(shell_height * 0.5) + (DRAWER_SIDE_THICKNESS * 0.5),
            )
        ),
        material=case_material,
        name="bottom_panel",
    )
    drawer.visual(
        Box((DRAWER_SHELL_WIDTH, BACK_PANEL_THICKNESS, shell_height)),
        origin=Origin(xyz=(0.0, back_center_y, 0.0)),
        material=case_material,
        name="back_panel",
    )

    runner_x = (DRAWER_SHELL_WIDTH * 0.5) + (RUNNER_THICKNESS * 0.5)
    drawer.visual(
        Box((RUNNER_THICKNESS, RUNNER_DEPTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(-runner_x, -0.03, runner_local_z)),
        material=runner_material,
        name="left_runner",
    )
    drawer.visual(
        Box((RUNNER_THICKNESS, RUNNER_DEPTH, RUNNER_HEIGHT)),
        origin=Origin(xyz=(runner_x, -0.03, runner_local_z)),
        material=runner_material,
        name="right_runner",
    )

    model.articulation(
        f"{name}_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(x_center, DRAWER_CENTER_Y, z_center)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=DRAWER_EXTENSION,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_writing_desk")

    walnut = model.material("walnut", rgba=(0.46, 0.30, 0.18, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.34, 0.20, 0.11, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.42, 0.26, 0.16, 1.0))
    runner_metal = model.material("runner_metal", rgba=(0.56, 0.57, 0.58, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.67, 0.58, 0.34, 1.0))
    handle_shadow = model.material("handle_shadow", rgba=(0.10, 0.07, 0.05, 1.0))

    body = model.part("desk_body")
    body.inertial = Inertial.from_geometry(
        Box((DESK_WIDTH, DESK_DEPTH, CARCASS_HEIGHT + TOP_THICKNESS)),
        mass=78.0,
        origin=Origin(xyz=(0.0, 0.0, (CARCASS_HEIGHT + TOP_THICKNESS) * 0.5)),
    )

    body.visual(
        Box((DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CARCASS_HEIGHT + (TOP_THICKNESS * 0.5))),
        material=walnut_dark,
        name="top",
    )

    side_panel_x = (CARCASS_WIDTH * 0.5) - (PANEL_THICKNESS * 0.5)
    divider_x = (0.38 + 0.398) * 0.5
    body.visual(
        Box((PANEL_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(xyz=(-side_panel_x, 0.0, CARCASS_HEIGHT * 0.5)),
        material=walnut,
        name="left_outer_side",
    )
    body.visual(
        Box((PANEL_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(xyz=(side_panel_x, 0.0, CARCASS_HEIGHT * 0.5)),
        material=walnut,
        name="right_outer_side",
    )
    body.visual(
        Box((PANEL_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(xyz=(-divider_x, 0.0, CARCASS_HEIGHT * 0.5)),
        material=walnut,
        name="left_inner_divider",
    )
    body.visual(
        Box((PANEL_THICKNESS, CARCASS_DEPTH, CARCASS_HEIGHT)),
        origin=Origin(xyz=(divider_x, 0.0, CARCASS_HEIGHT * 0.5)),
        material=walnut,
        name="right_inner_divider",
    )

    pedestal_centers = {"left": -0.575, "right": 0.575}
    back_panel_y = -(CARCASS_DEPTH * 0.5) + (BACK_PANEL_THICKNESS * 0.5)
    pedestal_depth_inside = CARCASS_DEPTH - BACK_PANEL_THICKNESS
    pedestal_depth_center = (CARCASS_DEPTH * 0.5 + back_panel_y + (BACK_PANEL_THICKNESS * 0.5)) * 0.5
    pedestal_bottom_z = BOTTOM_PANEL_THICKNESS * 0.5
    plinth_y = (CARCASS_DEPTH * 0.5) - (0.030 * 0.5)
    for side_name, x_center in pedestal_centers.items():
        body.visual(
            Box((PEDESTAL_WIDTH - (2.0 * PANEL_THICKNESS), BACK_PANEL_THICKNESS, CARCASS_HEIGHT)),
            origin=Origin(xyz=(x_center, back_panel_y, CARCASS_HEIGHT * 0.5)),
            material=walnut,
            name=f"{side_name}_back_panel",
        )
        body.visual(
            Box((PEDESTAL_WIDTH - (2.0 * PANEL_THICKNESS), pedestal_depth_inside, BOTTOM_PANEL_THICKNESS)),
            origin=Origin(xyz=(x_center, pedestal_depth_center, pedestal_bottom_z)),
            material=drawer_wood,
            name=f"{side_name}_bottom_panel",
        )
        body.visual(
            Box((PEDESTAL_WIDTH - (2.0 * PANEL_THICKNESS), 0.030, 0.045)),
            origin=Origin(xyz=(x_center, plinth_y, 0.0225)),
            material=walnut_dark,
            name=f"{side_name}_front_plinth",
        )

    body.visual(
        Box((0.760, MODESTY_PANEL_THICKNESS, 0.420)),
        origin=Origin(xyz=(0.0, -(CARCASS_DEPTH * 0.5) + (MODESTY_PANEL_THICKNESS * 0.5), 0.360)),
        material=walnut,
        name="modesty_panel",
    )

    left_openings = [
        ("left_bottom_drawer", 0.045, 0.214, 0.208, 0.178),
        ("left_middle_drawer", 0.277, 0.214, 0.208, 0.178),
        ("left_top_drawer", 0.509, 0.214, 0.208, 0.178),
    ]
    right_openings = [
        ("right_bottom_drawer", 0.045, 0.278, 0.272, 0.236),
        ("right_middle_drawer", 0.341, 0.212, 0.206, 0.176),
        ("right_top_drawer", 0.571, 0.154, 0.148, 0.114),
    ]

    opening_width = PEDESTAL_WIDTH - (2.0 * PANEL_THICKNESS)
    rail_x_offset = (opening_width * 0.5) - (BODY_RAIL_THICKNESS * 0.5)

    for z_bottom in (0.259, 0.491):
        body.visual(
            Box((opening_width, pedestal_depth_inside, PANEL_THICKNESS)),
            origin=Origin(xyz=(-0.575, pedestal_depth_center, z_bottom + (PANEL_THICKNESS * 0.5))),
            material=walnut,
        )
    for z_bottom in (0.323, 0.553):
        body.visual(
            Box((opening_width, pedestal_depth_inside, PANEL_THICKNESS)),
            origin=Origin(xyz=(0.575, pedestal_depth_center, z_bottom + (PANEL_THICKNESS * 0.5))),
            material=walnut,
        )

    for drawer_name, opening_bottom, opening_height, front_height, shell_height in left_openings:
        z_center = opening_bottom + (opening_height * 0.5)
        rail_z = opening_bottom + 0.055
        body.visual(
            Box((BODY_RAIL_THICKNESS, BODY_RAIL_DEPTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(-0.575 - rail_x_offset, -0.04, rail_z)),
            material=runner_metal,
            name=f"{drawer_name}_left_rail",
        )
        body.visual(
            Box((BODY_RAIL_THICKNESS, BODY_RAIL_DEPTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(-0.575 + rail_x_offset, -0.04, rail_z)),
            material=runner_metal,
            name=f"{drawer_name}_right_rail",
        )
        _add_drawer(
            model,
            body,
            name=drawer_name,
            x_center=-0.575,
            z_center=z_center,
            opening_bottom=opening_bottom,
            front_height=front_height,
            shell_height=shell_height,
            front_material=walnut_dark,
            case_material=drawer_wood,
            runner_material=runner_metal,
            bezel_material=aged_brass,
            pocket_material=handle_shadow,
        )

    for drawer_name, opening_bottom, opening_height, front_height, shell_height in right_openings:
        z_center = opening_bottom + (opening_height * 0.5)
        rail_z = opening_bottom + 0.055
        body.visual(
            Box((BODY_RAIL_THICKNESS, BODY_RAIL_DEPTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(0.575 - rail_x_offset, -0.04, rail_z)),
            material=runner_metal,
            name=f"{drawer_name}_left_rail",
        )
        body.visual(
            Box((BODY_RAIL_THICKNESS, BODY_RAIL_DEPTH, RUNNER_HEIGHT)),
            origin=Origin(xyz=(0.575 + rail_x_offset, -0.04, rail_z)),
            material=runner_metal,
            name=f"{drawer_name}_right_rail",
        )
        _add_drawer(
            model,
            body,
            name=drawer_name,
            x_center=0.575,
            z_center=z_center,
            opening_bottom=opening_bottom,
            front_height=front_height,
            shell_height=shell_height,
            front_material=walnut_dark,
            case_material=drawer_wood,
            runner_material=runner_metal,
            bezel_material=aged_brass,
            pocket_material=handle_shadow,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("desk_body")
    drawer_names = [
        "left_top_drawer",
        "left_middle_drawer",
        "left_bottom_drawer",
        "right_top_drawer",
        "right_middle_drawer",
        "right_bottom_drawer",
    ]
    drawer_joints = {
        name: object_model.get_articulation(f"{name}_slide")
        for name in drawer_names
    }
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

    for drawer_name in drawer_names:
        drawer = object_model.get_part(drawer_name)
        joint = drawer_joints[drawer_name]
        ctx.expect_contact(drawer, body, name=f"{drawer_name}_mounted_closed")
        ctx.check(
            f"{drawer_name}_slides_on_y_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"expected prismatic axis (0, 1, 0), got {joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{drawer_name}_has_realistic_extension",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.20 <= limits.upper <= 0.26,
            f"unexpected motion limits: {limits}",
        )

    with ctx.pose({joint: 0.18 for joint in drawer_joints.values()}):
        for drawer_name in drawer_names:
            ctx.expect_contact(
                object_model.get_part(drawer_name),
                body,
                name=f"{drawer_name}_mounted_open",
            )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_drawers_open")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
