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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

COUNTERTOP_WIDTH = 0.92
COUNTERTOP_DEPTH = 0.66
COUNTERTOP_THICKNESS = 0.038

COOKTOP_WIDTH = 0.59
COOKTOP_DEPTH = 0.52
COOKTOP_THICKNESS = 0.006
COOKTOP_CORNER_RADIUS = 0.018

CUTOUT_WIDTH = 0.548
CUTOUT_DEPTH = 0.478
CUTOUT_CORNER_RADIUS = 0.012

MOUNT_SKIRT_WIDTH = 0.536
MOUNT_SKIRT_DEPTH = 0.466
MOUNT_SKIRT_INNER_WIDTH = 0.498
MOUNT_SKIRT_INNER_DEPTH = 0.428
MOUNT_SKIRT_DROP = 0.018

CONTROL_PANEL_OUTER = 0.096
CONTROL_PANEL_INNER = 0.082
CONTROL_PANEL_CORNER_RADIUS = 0.012
CONTROL_PANEL_WALL_HEIGHT = 0.0025
CONTROL_PANEL_TOP_HEIGHT = 0.0015
CONTROL_PANEL_CENTER = (-0.205, -0.165)

BUTTON_RADIUS = 0.010
BUTTON_HEIGHT = 0.0046
BUTTON_BOTTOM_Z = 0.0018
BUTTON_TRAVEL = 0.0018
BUTTON_GRID_OFFSET = 0.015
BUTTON_HIDDEN_FLANGE_RADIUS = 0.0122
BUTTON_HIDDEN_FLANGE_HEIGHT = 0.0005
BUTTON_HIDDEN_FLANGE_TOP_Z = CONTROL_PANEL_WALL_HEIGHT

ZONE_MARK_THICKNESS = 0.0006
ZONE_LINE_WIDTH = 0.003
ZONE_SPECS = (
    ("zone_rear_left", (-0.102, 0.118), 0.092),
    ("zone_rear_right", (0.132, 0.118), 0.090),
    ("zone_front_left", (-0.018, -0.012), 0.088),
    ("zone_front_right", (0.150, -0.078), 0.102),
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _profile_loop_at_z(
    profile: list[tuple[float, float]],
    z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceramic_cooktop", assets=ASSETS)

    countertop_stone = model.material("countertop_stone", rgba=(0.79, 0.78, 0.75, 1.0))
    glass_black = model.material("glass_black", rgba=(0.08, 0.08, 0.09, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.56, 0.58, 0.62, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.16, 0.16, 0.17, 1.0))
    button_ceramic = model.material("button_ceramic", rgba=(0.87, 0.87, 0.85, 1.0))

    countertop = model.part("countertop")
    countertop_profile = rounded_rect_profile(
        COUNTERTOP_WIDTH,
        COUNTERTOP_DEPTH,
        radius=0.020,
        corner_segments=8,
    )
    cutout_profile = rounded_rect_profile(
        CUTOUT_WIDTH,
        CUTOUT_DEPTH,
        radius=CUTOUT_CORNER_RADIUS,
        corner_segments=8,
    )
    countertop.visual(
        _save_mesh(
            "countertop_with_cooktop_cutout.obj",
            ExtrudeWithHolesGeometry(
                countertop_profile,
                [cutout_profile],
                COUNTERTOP_THICKNESS,
                center=False,
            ),
        ),
        material=countertop_stone,
        name="countertop_slab",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((COUNTERTOP_WIDTH, COUNTERTOP_DEPTH, COUNTERTOP_THICKNESS)),
        mass=38.0,
        origin=Origin(
            xyz=(0.0, 0.0, COUNTERTOP_THICKNESS * 0.5),
        ),
    )

    cooktop = model.part("cooktop")
    glass_profile = rounded_rect_profile(
        COOKTOP_WIDTH,
        COOKTOP_DEPTH,
        radius=COOKTOP_CORNER_RADIUS,
        corner_segments=10,
    )
    cooktop.visual(
        _save_mesh(
            "cooktop_glass_panel.obj",
            LoftGeometry(
                [
                    _profile_loop_at_z(glass_profile, 0.0),
                    _profile_loop_at_z(glass_profile, COOKTOP_THICKNESS - 0.0012),
                    _profile_loop_at_z(
                        rounded_rect_profile(
                            COOKTOP_WIDTH - 0.004,
                            COOKTOP_DEPTH - 0.004,
                            radius=COOKTOP_CORNER_RADIUS - 0.002,
                            corner_segments=10,
                        ),
                        COOKTOP_THICKNESS,
                    ),
                ],
                cap=True,
                closed=True,
            ),
        ),
        material=glass_black,
        name="glass_panel",
    )
    skirt_outer = rounded_rect_profile(
        MOUNT_SKIRT_WIDTH,
        MOUNT_SKIRT_DEPTH,
        radius=0.010,
        corner_segments=8,
    )
    skirt_inner = rounded_rect_profile(
        MOUNT_SKIRT_INNER_WIDTH,
        MOUNT_SKIRT_INNER_DEPTH,
        radius=0.006,
        corner_segments=8,
    )
    cooktop.visual(
        _save_mesh(
            "cooktop_mount_skirt.obj",
            ExtrudeWithHolesGeometry(
                skirt_outer,
                [skirt_inner],
                MOUNT_SKIRT_DROP,
                center=False,
            ).translate(0.0, 0.0, -MOUNT_SKIRT_DROP),
        ),
        material=trim_dark,
        name="mount_skirt",
    )
    for zone_name, (zone_x, zone_y), zone_radius in ZONE_SPECS:
        cooktop.visual(
            _save_mesh(
                f"{zone_name}.obj",
                ExtrudeWithHolesGeometry(
                    _circle_profile(zone_radius, segments=56),
                    [_circle_profile(zone_radius - ZONE_LINE_WIDTH, segments=56)],
                    ZONE_MARK_THICKNESS,
                    center=False,
                ).translate(zone_x, zone_y, COOKTOP_THICKNESS),
            ),
            material=zone_mark,
            name=zone_name,
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, COOKTOP_THICKNESS + MOUNT_SKIRT_DROP)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    control_panel = model.part("control_panel")
    control_outer = rounded_rect_profile(
        CONTROL_PANEL_OUTER,
        CONTROL_PANEL_OUTER,
        radius=CONTROL_PANEL_CORNER_RADIUS,
        corner_segments=8,
    )
    control_inner = rounded_rect_profile(
        CONTROL_PANEL_INNER,
        CONTROL_PANEL_INNER,
        radius=CONTROL_PANEL_CORNER_RADIUS * 0.7,
        corner_segments=8,
    )
    button_hole_centers = {
        "button_front_left": (-BUTTON_GRID_OFFSET, -BUTTON_GRID_OFFSET),
        "button_front_right": (BUTTON_GRID_OFFSET, -BUTTON_GRID_OFFSET),
        "button_rear_left": (-BUTTON_GRID_OFFSET, BUTTON_GRID_OFFSET),
        "button_rear_right": (BUTTON_GRID_OFFSET, BUTTON_GRID_OFFSET),
    }
    control_panel.visual(
        _save_mesh(
            "control_panel_wall_ring.obj",
            ExtrudeWithHolesGeometry(
                control_outer,
                [control_inner],
                CONTROL_PANEL_WALL_HEIGHT,
                center=False,
            ),
        ),
        material=trim_dark,
        name="panel_walls",
    )
    control_panel.visual(
        _save_mesh(
            "control_panel_top_plate.obj",
            ExtrudeWithHolesGeometry(
                control_outer,
                [
                    _translate_profile(_circle_profile(BUTTON_RADIUS, segments=36), dx, dy)
                    for dx, dy in button_hole_centers.values()
                ],
                CONTROL_PANEL_TOP_HEIGHT,
                center=False,
            ).translate(0.0, 0.0, CONTROL_PANEL_WALL_HEIGHT),
        ),
        material=trim_dark,
        name="panel_top",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box(
            (
                CONTROL_PANEL_OUTER,
                CONTROL_PANEL_OUTER,
                CONTROL_PANEL_WALL_HEIGHT + CONTROL_PANEL_TOP_HEIGHT,
            )
        ),
        mass=0.45,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (CONTROL_PANEL_WALL_HEIGHT + CONTROL_PANEL_TOP_HEIGHT) * 0.5,
            )
        ),
    )

    button_mesh = _save_mesh(
        "cooktop_button_body.obj",
        LatheGeometry(
            [
                (0.0, BUTTON_BOTTOM_Z),
                (BUTTON_RADIUS * 0.86, BUTTON_BOTTOM_Z),
                (BUTTON_RADIUS, BUTTON_BOTTOM_Z + 0.0008),
                (BUTTON_RADIUS * 0.98, BUTTON_BOTTOM_Z + BUTTON_HEIGHT * 0.55),
                (BUTTON_RADIUS * 0.90, BUTTON_BOTTOM_Z + BUTTON_HEIGHT),
                (0.0, BUTTON_BOTTOM_Z + BUTTON_HEIGHT),
            ],
            segments=48,
            closed=True,
        ),
    )
    button_flange_mesh = _save_mesh(
        "cooktop_button_flange.obj",
        ExtrudeGeometry(
            _circle_profile(BUTTON_HIDDEN_FLANGE_RADIUS, segments=36),
            BUTTON_HIDDEN_FLANGE_HEIGHT,
            center=False,
        ).translate(
            0.0,
            0.0,
            BUTTON_HIDDEN_FLANGE_TOP_Z - BUTTON_HIDDEN_FLANGE_HEIGHT,
        ),
    )
    for button_name in (
        "button_front_left",
        "button_front_right",
        "button_rear_left",
        "button_rear_right",
    ):
        button_part = model.part(button_name)
        button_part.visual(button_mesh, material=button_ceramic, name="button_body")
        button_part.visual(button_flange_mesh, material=button_ceramic, name="button_stop")
        button_part.inertial = Inertial.from_geometry(
            Cylinder(radius=BUTTON_RADIUS, length=BUTTON_HEIGHT),
            mass=0.05,
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    BUTTON_BOTTOM_Z + BUTTON_HEIGHT * 0.5,
                )
            ),
        )

    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, COUNTERTOP_THICKNESS)),
    )
    model.articulation(
        "cooktop_to_control_panel",
        ArticulationType.FIXED,
        parent=cooktop,
        child=control_panel,
        origin=Origin(
            xyz=(
                CONTROL_PANEL_CENTER[0],
                CONTROL_PANEL_CENTER[1],
                COOKTOP_THICKNESS,
            )
        ),
    )

    for button_name, (button_x, button_y) in button_hole_centers.items():
        model.articulation(
            f"control_panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button_name,
            origin=Origin(xyz=(button_x, button_y, 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    control_panel = object_model.get_part("control_panel")
    glass_panel = cooktop.get_visual("glass_panel")
    moving_joint_names = (
        "control_panel_to_button_front_left",
        "control_panel_to_button_front_right",
        "control_panel_to_button_rear_left",
        "control_panel_to_button_rear_right",
    )
    button_names = (
        "button_front_left",
        "button_front_right",
        "button_rear_left",
        "button_rear_right",
    )
    button_joints = {
        name: object_model.get_articulation(name) for name in moving_joint_names
    }
    buttons = {name: object_model.get_part(name) for name in button_names}

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=20,
        name="button_strokes_clearance",
    )

    ctx.expect_contact(
        cooktop,
        countertop,
        elem_a="glass_panel",
        name="cooktop_glass_contacts_countertop",
    )
    ctx.expect_within(
        cooktop,
        countertop,
        axes="xy",
        inner_elem="glass_panel",
        margin=0.0,
        name="cooktop_sits_within_countertop_bounds",
    )
    ctx.expect_contact(
        control_panel,
        cooktop,
        name="control_panel_mounted_on_cooktop",
    )
    ctx.expect_within(
        control_panel,
        cooktop,
        axes="xy",
        margin=0.0,
        name="control_panel_within_glass_planform",
    )

    for zone_name, _, _ in ZONE_SPECS:
        ctx.expect_within(
            cooktop,
            cooktop,
            axes="xy",
            inner_elem=zone_name,
            outer_elem="glass_panel",
            margin=0.0,
            name=f"{zone_name}_inside_glass_panel",
        )

    expected_axes = (0.0, 0.0, -1.0)
    for joint_name, button_name in zip(moving_joint_names, button_names):
        joint = button_joints[joint_name]
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"{joint_name} should be PRISMATIC.",
        )
        ctx.check(
            f"{joint_name}_axis_is_panel_normal",
            tuple(joint.axis) == expected_axes,
            f"{joint_name} axis {joint.axis!r} should be {expected_axes!r}.",
        )
        ctx.check(
            f"{joint_name}_stroke_is_short",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.003,
            f"{joint_name} should have a short downward stroke, got {limits!r}.",
        )
        ctx.expect_contact(
            buttons[button_name],
            control_panel,
            name=f"{button_name}_rest_contacts_control_panel",
        )

    ctx.expect_origin_distance(
        buttons["button_front_left"],
        buttons["button_front_right"],
        axes="x",
        min_dist=0.029,
        max_dist=0.031,
        name="front_button_spacing_x",
    )
    ctx.expect_origin_distance(
        buttons["button_front_left"],
        buttons["button_rear_left"],
        axes="y",
        min_dist=0.029,
        max_dist=0.031,
        name="left_button_spacing_y",
    )

    button_positions = {
        name: ctx.part_world_position(part) for name, part in buttons.items()
    }
    left_front_cluster = all(
        position is not None and position[0] < -0.16 and position[1] < -0.12
        for position in button_positions.values()
    )
    ctx.check(
        "button_cluster_near_left_front_corner",
        left_front_cluster,
        f"Expected all button origins near the left-front corner, got {button_positions!r}.",
    )

    cooktop_aabb = ctx.part_world_aabb(cooktop)
    cooktop_has_no_deep_base = (
        cooktop_aabb is not None and cooktop_aabb[0][2] >= 0.018 and cooktop_aabb[1][2] <= 0.045
    )
    ctx.check(
        "cooktop_has_only_shallow_mounting_skirt",
        cooktop_has_no_deep_base,
        f"Cooktop should be a shallow top-only assembly, got AABB {cooktop_aabb!r}.",
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")
    pressed_pose = {
        joint: joint.motion_limits.upper
        for joint in button_joints.values()
        if joint.motion_limits is not None and joint.motion_limits.upper is not None
    }
    with ctx.pose(pressed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="buttons_pressed_no_floating")
        for button_name in button_names:
            ctx.expect_contact(
                buttons[button_name],
                cooktop,
                name=f"{button_name}_pressed_contacts_glass_stop",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
