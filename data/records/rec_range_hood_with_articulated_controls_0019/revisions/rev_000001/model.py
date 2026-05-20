from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.18
PANEL_THICKNESS = 0.004
CANOPY_TOP_WIDTH = 0.76
CANOPY_TOP_DEPTH = 0.36
CANOPY_TOP_Y = CANOPY_DEPTH * 0.5 - CANOPY_TOP_DEPTH * 0.5
CANOPY_TOP_FRONT_Y = CANOPY_TOP_Y - CANOPY_TOP_DEPTH * 0.5
CONTROL_COLUMN_INNER_X = 0.404
CONTROL_COLUMN_OUTER_X = CANOPY_WIDTH * 0.5

CHIMNEY_WIDTH = 0.32
CHIMNEY_DEPTH = 0.28
CHIMNEY_Y = 0.11
CHIMNEY_Z_MIN = CANOPY_HEIGHT - 0.002
CHIMNEY_Z_MAX = 0.96
CHIMNEY_HEIGHT = CHIMNEY_Z_MAX - CHIMNEY_Z_MIN

BUTTON_COUNT = 6
BUTTON_SIZE = 0.020
BUTTON_DEPTH = 0.012
BUTTON_TRAVEL = 0.004
BUTTON_X_CENTER = 0.426
BUTTON_Y_REST = -0.248
BUTTON_Z_CENTERS = (0.150, 0.126, 0.102, 0.078, 0.054, 0.030)


def _aabb_dims(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(aabb[1][axis] - aabb[0][axis] for axis in range(3))


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _panel_mesh(
    mesh_name: str,
    corners: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    *,
    thickness: float,
):
    p0, p1, p2, p3 = corners
    edge_a = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    edge_b = (p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2])
    normal = (
        edge_a[1] * edge_b[2] - edge_a[2] * edge_b[1],
        edge_a[2] * edge_b[0] - edge_a[0] * edge_b[2],
        edge_a[0] * edge_b[1] - edge_a[1] * edge_b[0],
    )
    normal_length = (normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2) ** 0.5
    offset = (
        normal[0] * thickness * 0.5 / normal_length,
        normal[1] * thickness * 0.5 / normal_length,
        normal[2] * thickness * 0.5 / normal_length,
    )

    mesh = MeshGeometry()
    outer_ids = [
        mesh.add_vertex(point[0] + offset[0], point[1] + offset[1], point[2] + offset[2])
        for point in corners
    ]
    inner_ids = [
        mesh.add_vertex(point[0] - offset[0], point[1] - offset[1], point[2] - offset[2])
        for point in corners
    ]

    _add_quad(mesh, outer_ids[0], outer_ids[1], outer_ids[2], outer_ids[3])
    _add_quad(mesh, inner_ids[3], inner_ids[2], inner_ids[1], inner_ids[0])
    for index in range(4):
        next_index = (index + 1) % 4
        _add_quad(
            mesh,
            outer_ids[index],
            outer_ids[next_index],
            inner_ids[next_index],
            inner_ids[index],
        )

    return mesh_from_geometry(mesh, ASSETS.mesh_path(mesh_name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    shadow_steel = model.material("shadow_steel", rgba=(0.41, 0.44, 0.47, 1.0))
    button_black = model.material("button_black", rgba=(0.13, 0.14, 0.15, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((CANOPY_TOP_WIDTH, CANOPY_TOP_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, CANOPY_TOP_Y, CANOPY_HEIGHT - PANEL_THICKNESS * 0.5)),
        material=stainless,
        name="canopy_top",
    )
    housing.visual(
        _panel_mesh(
            "canopy_left_side.obj",
            (
                (-CANOPY_WIDTH * 0.5 + PANEL_THICKNESS * 0.5, -CANOPY_DEPTH * 0.5, 0.0),
                (-CANOPY_WIDTH * 0.5 + PANEL_THICKNESS * 0.5, CANOPY_DEPTH * 0.5, 0.0),
                (-CANOPY_WIDTH * 0.5 + PANEL_THICKNESS * 0.5, CANOPY_DEPTH * 0.5, CANOPY_HEIGHT - PANEL_THICKNESS),
                (-CANOPY_WIDTH * 0.5 + PANEL_THICKNESS * 0.5, CANOPY_TOP_FRONT_Y, CANOPY_HEIGHT - PANEL_THICKNESS),
            ),
            thickness=PANEL_THICKNESS,
        ),
        material=stainless,
        name="canopy_left_side",
    )
    housing.visual(
        _panel_mesh(
            "canopy_right_side.obj",
            (
                (CANOPY_WIDTH * 0.5 - PANEL_THICKNESS * 0.5, CANOPY_TOP_FRONT_Y, CANOPY_HEIGHT - PANEL_THICKNESS),
                (CANOPY_WIDTH * 0.5 - PANEL_THICKNESS * 0.5, CANOPY_DEPTH * 0.5, CANOPY_HEIGHT - PANEL_THICKNESS),
                (CANOPY_WIDTH * 0.5 - PANEL_THICKNESS * 0.5, CANOPY_DEPTH * 0.5, 0.0),
                (CANOPY_WIDTH * 0.5 - PANEL_THICKNESS * 0.5, -CANOPY_DEPTH * 0.5, 0.0),
            ),
            thickness=PANEL_THICKNESS,
        ),
        material=stainless,
        name="canopy_right_side",
    )
    housing.visual(
        Box((CANOPY_WIDTH, PANEL_THICKNESS, CANOPY_HEIGHT)),
        origin=Origin(xyz=(0.0, CANOPY_DEPTH * 0.5 - PANEL_THICKNESS * 0.5, CANOPY_HEIGHT * 0.5)),
        material=shadow_steel,
        name="canopy_back",
    )
    housing.visual(
        _panel_mesh(
            "canopy_front_left.obj",
            (
                (-CANOPY_WIDTH * 0.5, -CANOPY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5, 0.0),
                (CONTROL_COLUMN_INNER_X, -CANOPY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5, 0.0),
                (CONTROL_COLUMN_INNER_X, CANOPY_TOP_FRONT_Y + PANEL_THICKNESS * 0.5, CANOPY_HEIGHT - PANEL_THICKNESS),
                (-CANOPY_WIDTH * 0.5, CANOPY_TOP_FRONT_Y + PANEL_THICKNESS * 0.5, CANOPY_HEIGHT - PANEL_THICKNESS),
            ),
            thickness=PANEL_THICKNESS,
        ),
        material=stainless,
        name="front_panel_left",
    )
    housing.visual(
        Box((0.012, PANEL_THICKNESS, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_COLUMN_INNER_X + 0.006,
                -CANOPY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5,
                CANOPY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="control_inner_rail",
    )
    housing.visual(
        Box((0.014, PANEL_THICKNESS, CANOPY_HEIGHT)),
        origin=Origin(
            xyz=(
                CONTROL_COLUMN_OUTER_X - 0.007,
                -CANOPY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5,
                CANOPY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="control_outer_rail",
    )
    housing.visual(
        Box((BUTTON_SIZE, PANEL_THICKNESS, 0.020)),
        origin=Origin(xyz=(BUTTON_X_CENTER, -CANOPY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5, 0.170)),
        material=stainless,
        name="control_top_cap",
    )
    housing.visual(
        Box((BUTTON_SIZE, PANEL_THICKNESS, 0.020)),
        origin=Origin(xyz=(BUTTON_X_CENTER, -CANOPY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5, 0.010)),
        material=stainless,
        name="control_bottom_cap",
    )
    for separator_index, separator_z in enumerate((0.138, 0.114, 0.090, 0.066, 0.042), start=1):
        housing.visual(
            Box((BUTTON_SIZE, PANEL_THICKNESS, 0.004)),
            origin=Origin(xyz=(BUTTON_X_CENTER, -CANOPY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5, separator_z)),
            material=stainless,
            name=f"control_separator_{separator_index}",
        )

    housing.visual(
        Box((CHIMNEY_WIDTH, PANEL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_Y - CHIMNEY_DEPTH * 0.5 + PANEL_THICKNESS * 0.5,
                CHIMNEY_Z_MIN + CHIMNEY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH, PANEL_THICKNESS, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CHIMNEY_Y + CHIMNEY_DEPTH * 0.5 - PANEL_THICKNESS * 0.5,
                CHIMNEY_Z_MIN + CHIMNEY_HEIGHT * 0.5,
            )
        ),
        material=shadow_steel,
        name="chimney_back",
    )
    housing.visual(
        Box((PANEL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                -CHIMNEY_WIDTH * 0.5 + PANEL_THICKNESS * 0.5,
                CHIMNEY_Y,
                CHIMNEY_Z_MIN + CHIMNEY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    housing.visual(
        Box((PANEL_THICKNESS, CHIMNEY_DEPTH, CHIMNEY_HEIGHT)),
        origin=Origin(
            xyz=(
                CHIMNEY_WIDTH * 0.5 - PANEL_THICKNESS * 0.5,
                CHIMNEY_Y,
                CHIMNEY_Z_MIN + CHIMNEY_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    housing.visual(
        Box((CHIMNEY_WIDTH, CHIMNEY_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, CHIMNEY_Y, CHIMNEY_Z_MAX - PANEL_THICKNESS * 0.5)),
        material=stainless,
        name="chimney_top",
    )
    housing.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CHIMNEY_Z_MAX)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_Z_MAX * 0.5)),
    )

    filter_panel = model.part("filter_panel")
    filter_panel.visual(
        Box((0.892, 0.220, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=shadow_steel,
        name="underside_baffle_filter",
    )
    filter_panel.inertial = Inertial.from_geometry(
        Box((0.892, 0.220, 0.012)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    model.articulation(
        "housing_to_filter_panel",
        ArticulationType.FIXED,
        parent=housing,
        child=filter_panel,
        origin=Origin(xyz=(0.0, 0.030, 0.004)),
    )

    motor_cover = model.part("motor_cover")
    motor_cover.visual(
        Box((0.120, 0.040, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=shadow_steel,
        name="motor_cover_shell",
    )
    motor_cover.inertial = Inertial.from_geometry(
        Box((0.120, 0.040, 0.048)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    model.articulation(
        "housing_to_motor_cover",
        ArticulationType.FIXED,
        parent=housing,
        child=motor_cover,
        origin=Origin(xyz=(0.0, 0.100, 0.128)),
    )

    for button_index, button_z in enumerate(BUTTON_Z_CENTERS, start=1):
        button = model.part(f"button_{button_index}")
        button.visual(
            Box((BUTTON_SIZE, BUTTON_DEPTH, BUTTON_SIZE)),
            material=button_black,
            name="button_body",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_SIZE, BUTTON_DEPTH, BUTTON_SIZE)),
            mass=0.03,
        )
        model.articulation(
            f"button_{button_index}_plunger",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(BUTTON_X_CENTER, BUTTON_Y_REST, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    housing = object_model.get_part("housing")
    filter_panel = object_model.get_part("filter_panel")
    motor_cover = object_model.get_part("motor_cover")
    canopy_top = housing.get_visual("canopy_top")
    chimney_top = housing.get_visual("chimney_top")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, BUTTON_COUNT + 1)]
    plungers = [object_model.get_articulation(f"button_{index}_plunger") for index in range(1, BUTTON_COUNT + 1)]

    root_names = {part.name for part in object_model.root_parts()}
    ctx.check(
        "single_root_housing",
        root_names == {"housing"},
        details=f"expected only housing as root part, got {sorted(root_names)}",
    )

    housing_aabb = ctx.part_world_aabb(housing)
    if housing_aabb is None:
        ctx.fail("housing_has_geometry", "housing returned no world-space AABB")
    else:
        housing_dims = _aabb_dims(housing_aabb)
        ctx.check(
            "housing_realistic_width",
            0.88 <= housing_dims[0] <= 0.92,
            details=f"expected width near 0.90 m, got {housing_dims[0]:.4f}",
        )
        ctx.check(
            "housing_realistic_depth",
            0.49 <= housing_dims[1] <= 0.51,
            details=f"expected depth near 0.50 m, got {housing_dims[1]:.4f}",
        )
        ctx.check(
            "housing_realistic_height",
            0.94 <= housing_dims[2] <= 0.98,
            details=f"expected height near 0.96 m, got {housing_dims[2]:.4f}",
        )

    canopy_top_aabb = ctx.part_element_world_aabb(housing, elem=canopy_top)
    chimney_top_aabb = ctx.part_element_world_aabb(housing, elem=chimney_top)
    if canopy_top_aabb is None or chimney_top_aabb is None:
        ctx.fail("named_shell_visuals_present", "canopy_top or chimney_top could not be measured")
    else:
        canopy_dims = _aabb_dims(canopy_top_aabb)
        chimney_dims = _aabb_dims(chimney_top_aabb)
        ctx.check(
            "chimney_narrower_than_canopy",
            chimney_dims[0] < canopy_dims[0] * 0.5 and chimney_dims[1] < canopy_dims[1],
            details=f"canopy dims={canopy_dims}, chimney dims={chimney_dims}",
        )

    ctx.expect_contact(filter_panel, housing, name="filter_panel_contacts_housing")
    ctx.expect_contact(motor_cover, housing, name="motor_cover_contacts_housing")
    ctx.expect_within(filter_panel, housing, axes="xy", margin=0.0, name="filter_panel_within_canopy_footprint")
    ctx.expect_within(motor_cover, housing, axes="xy", margin=0.0, name="motor_cover_within_housing")

    rest_positions: list[tuple[float, float, float]] = []
    for index, (button, plunger) in enumerate(zip(buttons, plungers), start=1):
        limits = plunger.motion_limits
        ctx.check(
            f"{plunger.name}_is_prismatic",
            plunger.joint_type == ArticulationType.PRISMATIC,
            details=f"expected PRISMATIC joint, got {plunger.joint_type}",
        )
        ctx.check(
            f"{plunger.name}_axis_inward",
            tuple(plunger.axis) == (0.0, 1.0, 0.0),
            details=f"expected inward +Y axis, got {plunger.axis}",
        )
        ctx.check(
            f"{plunger.name}_short_travel",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0035 <= limits.upper <= 0.0045,
            details=(
                "expected a short inward plunger stroke around 4 mm, "
                f"got limits={limits}"
            ),
        )

        ctx.expect_contact(button, housing, name=f"button_{index}_rest_contact")
        ctx.expect_within(button, housing, axes="xz", margin=0.0, name=f"button_{index}_rest_within_housing")

        rest_position = ctx.part_world_position(button)
        if rest_position is None:
            ctx.fail(f"button_{index}_rest_position_exists", "button returned no world position at rest")
            continue
        rest_positions.append(rest_position)

        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"button_{index}_limits_defined", "button plunger is missing finite motion limits")
            continue

        with ctx.pose({plunger: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{plunger.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{plunger.name}_lower_no_floating")

        with ctx.pose({plunger: limits.upper}):
            ctx.expect_contact(button, housing, name=f"button_{index}_pressed_contact")
            ctx.expect_within(
                button,
                housing,
                axes="xz",
                margin=0.0,
                name=f"button_{index}_pressed_within_housing",
            )

            pressed_position = ctx.part_world_position(button)
            if pressed_position is None:
                ctx.fail(f"button_{index}_pressed_position_exists", "button returned no world position when pressed")
            else:
                ctx.check(
                    f"button_{index}_presses_inward",
                    abs(pressed_position[0] - rest_position[0]) <= 1e-6
                    and abs(pressed_position[2] - rest_position[2]) <= 1e-6
                    and pressed_position[1] - rest_position[1] >= limits.upper - 1e-6,
                    details=f"rest={rest_position}, pressed={pressed_position}, upper={limits.upper}",
                )

            ctx.fail_if_parts_overlap_in_current_pose(name=f"{plunger.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{plunger.name}_upper_no_floating")

    if len(rest_positions) == BUTTON_COUNT:
        x_values = [position[0] for position in rest_positions]
        y_values = [position[1] for position in rest_positions]
        z_values = [position[2] for position in rest_positions]
        z_spacings = [z_values[index] - z_values[index + 1] for index in range(BUTTON_COUNT - 1)]

        ctx.check(
            "buttons_share_single_vertical_column",
            max(x_values) - min(x_values) <= 1e-6 and max(y_values) - min(y_values) <= 1e-6,
            details=f"button positions={rest_positions}",
        )
        ctx.check(
            "buttons_down_right_front_edge",
            x_values[0] > 0.41 and y_values[0] < -0.247,
            details=f"expected column on right front edge, got first button at {rest_positions[0]}",
        )
        ctx.check(
            "buttons_evenly_spaced_vertically",
            all(0.023 <= spacing <= 0.025 for spacing in z_spacings),
            details=f"expected about 24 mm vertical pitch, got spacings={z_spacings}",
        )
        ctx.check(
            "buttons_fill_front_column_height",
            min(z_values) < 0.04 and max(z_values) > 0.14,
            details=f"expected a tall six-button stack, got z positions={z_values}",
        )

    ctx.fail_if_isolated_parts(max_pose_samples=32, name="button_pose_sweep_no_floating")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=32,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
