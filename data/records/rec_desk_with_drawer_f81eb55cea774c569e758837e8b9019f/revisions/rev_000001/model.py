from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


DESK_TOP_THICKNESS = 0.032
DESK_UNDERSIDE_Z = 0.726
DESK_TOP_Z = DESK_UNDERSIDE_Z + DESK_TOP_THICKNESS


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    name: str | None = None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _make_extruded_mesh(profile: list[tuple[float, float]], height: float, name: str):
    return mesh_from_geometry(ExtrudeGeometry.from_z0(profile, height), name)


def _build_drawer_part(
    model: ArticulatedObject,
    *,
    name: str,
    front_material,
    carcass_material,
    rail_material,
) -> None:
    front_width = 0.394
    front_height = 0.208
    front_thickness = 0.018
    body_width = 0.366
    body_depth = 0.460
    body_height = 0.158
    body_bottom_z = -0.086
    body_center_y = front_thickness * 0.5 + body_depth * 0.5
    body_center_z = body_bottom_z + body_height * 0.5
    side_thickness = 0.012
    runner_width = 0.010
    runner_length = 0.300
    runner_height = 0.035
    runner_center_y = 0.280
    runner_center_z = body_center_z - 0.010

    drawer = model.part(name)
    _add_box(
        drawer,
        size=(front_width, front_thickness, front_height),
        xyz=(0.0, 0.0, 0.0),
        material=front_material,
        name="front",
    )
    _add_box(
        drawer,
        size=(0.016, 0.020, 0.016),
        xyz=(-0.055, -0.015, 0.0),
        material=rail_material,
        name="handle_left_post",
    )
    _add_box(
        drawer,
        size=(0.016, 0.020, 0.016),
        xyz=(0.055, -0.015, 0.0),
        material=rail_material,
        name="handle_right_post",
    )
    _add_box(
        drawer,
        size=(0.160, 0.012, 0.014),
        xyz=(0.0, -0.026, 0.0),
        material=rail_material,
        name="handle_bar",
    )
    _add_box(
        drawer,
        size=(side_thickness, body_depth, body_height),
        xyz=(body_width * 0.5 - side_thickness * 0.5, body_center_y, body_center_z),
        material=carcass_material,
        name="right_side",
    )
    _add_box(
        drawer,
        size=(side_thickness, body_depth, body_height),
        xyz=(-body_width * 0.5 + side_thickness * 0.5, body_center_y, body_center_z),
        material=carcass_material,
        name="left_side",
    )
    _add_box(
        drawer,
        size=(body_width - 0.024, body_depth, 0.012),
        xyz=(0.0, body_center_y, body_bottom_z + 0.006),
        material=carcass_material,
        name="bottom",
    )
    _add_box(
        drawer,
        size=(body_width - 0.024, 0.012, body_height),
        xyz=(0.0, front_thickness * 0.5 + body_depth - 0.006, body_center_z),
        material=carcass_material,
        name="back",
    )
    _add_box(
        drawer,
        size=(runner_width, runner_length, runner_height),
        xyz=(body_width * 0.5 + runner_width * 0.5, runner_center_y, runner_center_z),
        material=rail_material,
        name="right_runner",
    )
    _add_box(
        drawer,
        size=(runner_width, runner_length, runner_height),
        xyz=(-body_width * 0.5 - runner_width * 0.5, runner_center_y, runner_center_z),
        material=rail_material,
        name="left_runner",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((front_width, 0.478, front_height)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.235, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_l_desk")

    walnut = model.material("walnut", rgba=(0.60, 0.44, 0.28, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.22, 1.0))
    cabinet_white = model.material("cabinet_white", rgba=(0.88, 0.89, 0.90, 1.0))
    drawer_white = model.material("drawer_white", rgba=(0.95, 0.95, 0.94, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.17, 1.0))

    corner_bracket_profile = [
        (-0.06, 0.0),
        (0.16, 0.0),
        (0.0, -0.17),
    ]

    bracket_mesh = _make_extruded_mesh(
        corner_bracket_profile, 0.010, "mitre_corner_bracket_plate_mesh_v3"
    )

    main_worktop = model.part("main_worktop")
    _add_box(
        main_worktop,
        size=(1.50, 0.75, DESK_TOP_THICKNESS),
        xyz=(-0.05, 0.0, DESK_UNDERSIDE_Z + DESK_TOP_THICKNESS * 0.5),
        material=walnut,
        name="main_top_slab",
    )
    main_worktop.inertial = Inertial.from_geometry(
        Box((1.50, 0.75, DESK_TOP_THICKNESS)),
        mass=24.0,
        origin=Origin(xyz=(-0.05, 0.0, DESK_UNDERSIDE_Z + DESK_TOP_THICKNESS * 0.5)),
    )

    corner_bracket = model.part("corner_bracket")
    corner_bracket.visual(bracket_mesh, material=steel, name="corner_plate")
    corner_bracket.inertial = Inertial.from_geometry(
        Box((0.22, 0.17, 0.010)),
        mass=1.2,
        origin=Origin(xyz=(0.11, -0.085, 0.005)),
    )

    return_worktop = model.part("return_worktop")
    _add_box(
        return_worktop,
        size=(0.60, 1.20, DESK_TOP_THICKNESS),
        xyz=(0.0, 0.0, DESK_TOP_THICKNESS * 0.5),
        material=walnut,
        name="return_top_slab",
    )
    return_worktop.inertial = Inertial.from_geometry(
        Box((0.60, 1.20, DESK_TOP_THICKNESS)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, DESK_TOP_THICKNESS * 0.5)),
    )

    left_panel_leg = model.part("left_panel_leg")
    _add_box(
        left_panel_leg,
        size=(0.036, 0.700, DESK_UNDERSIDE_Z),
        xyz=(0.0, 0.0, DESK_UNDERSIDE_Z * 0.5),
        material=graphite,
        name="left_panel",
    )
    left_panel_leg.inertial = Inertial.from_geometry(
        Box((0.036, 0.700, DESK_UNDERSIDE_Z)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, DESK_UNDERSIDE_Z * 0.5)),
    )

    return_panel_leg = model.part("return_panel_leg")
    _add_box(
        return_panel_leg,
        size=(0.560, 0.036, DESK_UNDERSIDE_Z),
        xyz=(0.0, 0.0, DESK_UNDERSIDE_Z * 0.5),
        material=graphite,
        name="return_panel",
    )
    return_panel_leg.inertial = Inertial.from_geometry(
        Box((0.560, 0.036, DESK_UNDERSIDE_Z)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, DESK_UNDERSIDE_Z * 0.5)),
    )

    right_pedestal = model.part("right_pedestal")
    pedestal_w = 0.420
    pedestal_d = 0.560
    pedestal_h = DESK_UNDERSIDE_Z
    side_t = 0.018
    inner_w = pedestal_w - 2.0 * side_t
    rail_x = inner_w * 0.5 + 0.005
    for sx in (-1.0, 1.0):
        _add_box(
            right_pedestal,
            size=(side_t, pedestal_d, pedestal_h),
            xyz=(sx * (pedestal_w * 0.5 - side_t * 0.5), 0.0, pedestal_h * 0.5),
            material=cabinet_white,
            name=f"side_{'left' if sx < 0.0 else 'right'}",
        )
    _add_box(
        right_pedestal,
        size=(inner_w, 0.012, pedestal_h),
        xyz=(0.0, pedestal_d * 0.5 - 0.006, pedestal_h * 0.5),
        material=cabinet_white,
        name="back_panel",
    )
    _add_box(
        right_pedestal,
        size=(inner_w, pedestal_d - 0.030, 0.018),
        xyz=(0.0, 0.015, 0.089),
        material=cabinet_white,
        name="bottom_panel",
    )
    _add_box(
        right_pedestal,
        size=(inner_w, pedestal_d - 0.030, 0.016),
        xyz=(0.0, 0.015, 0.306),
        material=cabinet_white,
        name="divider_lower",
    )
    _add_box(
        right_pedestal,
        size=(inner_w, pedestal_d - 0.030, 0.016),
        xyz=(0.0, 0.015, 0.515),
        material=cabinet_white,
        name="divider_upper",
    )
    _add_box(
        right_pedestal,
        size=(inner_w, 0.180, 0.018),
        xyz=(0.0, 0.190, pedestal_h - 0.009),
        material=cabinet_white,
        name="top_stretcher",
    )
    _add_box(
        right_pedestal,
        size=(inner_w, 0.012, 0.080),
        xyz=(0.0, -0.215, 0.040),
        material=graphite,
        name="toe_kick",
    )
    for drawer_name, drawer_z in (
        ("lower", 0.184),
        ("middle", 0.393),
        ("upper", 0.602),
    ):
        _add_box(
            right_pedestal,
            size=(0.010, 0.400, 0.035),
            xyz=(-rail_x, -0.080, drawer_z),
            material=steel,
            name=f"{drawer_name}_left_rail",
        )
        _add_box(
            right_pedestal,
            size=(0.010, 0.400, 0.035),
            xyz=(rail_x, -0.080, drawer_z),
            material=steel,
            name=f"{drawer_name}_right_rail",
        )
    right_pedestal.inertial = Inertial.from_geometry(
        Box((pedestal_w, pedestal_d, pedestal_h)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, pedestal_h * 0.5)),
    )

    _build_drawer_part(
        model,
        name="lower_drawer",
        front_material=drawer_white,
        carcass_material=charcoal,
        rail_material=steel,
    )
    _build_drawer_part(
        model,
        name="middle_drawer",
        front_material=drawer_white,
        carcass_material=charcoal,
        rail_material=steel,
    )
    _build_drawer_part(
        model,
        name="upper_drawer",
        front_material=drawer_white,
        carcass_material=charcoal,
        rail_material=steel,
    )

    keyboard_mount = model.part("keyboard_mount")
    _add_box(
        keyboard_mount,
        size=(0.700, 0.030, 0.020),
        xyz=(0.0, 0.125, 0.060),
        material=steel,
        name="rear_crossmember",
    )
    _add_box(
        keyboard_mount,
        size=(0.020, 0.030, 0.080),
        xyz=(-0.340, 0.125, 0.020),
        material=steel,
        name="left_hanger",
    )
    _add_box(
        keyboard_mount,
        size=(0.020, 0.030, 0.080),
        xyz=(0.340, 0.125, 0.020),
        material=steel,
        name="right_hanger",
    )
    _add_box(
        keyboard_mount,
        size=(0.010, 0.300, 0.020),
        xyz=(-0.325, -0.010, 0.000),
        material=steel,
        name="left_fixed_rail",
    )
    _add_box(
        keyboard_mount,
        size=(0.010, 0.300, 0.020),
        xyz=(0.325, -0.010, 0.000),
        material=steel,
        name="right_fixed_rail",
    )
    keyboard_mount.inertial = Inertial.from_geometry(
        Box((0.700, 0.330, 0.080)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.040, 0.030)),
    )

    keyboard_tray = model.part("keyboard_tray")
    _add_box(
        keyboard_tray,
        size=(0.640, 0.300, 0.020),
        xyz=(0.0, 0.0, 0.0),
        material=graphite,
        name="tray_board",
    )
    _add_box(
        keyboard_tray,
        size=(0.640, 0.018, 0.030),
        xyz=(0.0, -0.159, 0.005),
        material=graphite,
        name="tray_front_lip",
    )
    keyboard_tray.inertial = Inertial.from_geometry(
        Box((0.640, 0.320, 0.030)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.009, 0.005)),
    )

    model.articulation(
        "main_to_corner_bracket",
        ArticulationType.FIXED,
        parent=main_worktop,
        child=corner_bracket,
        origin=Origin(xyz=(0.73, 0.17, 0.716)),
    )
    model.articulation(
        "main_to_return_worktop",
        ArticulationType.FIXED,
        parent=main_worktop,
        child=return_worktop,
        origin=Origin(xyz=(1.00, 0.60, DESK_UNDERSIDE_Z)),
    )
    model.articulation(
        "main_to_left_panel",
        ArticulationType.FIXED,
        parent=main_worktop,
        child=left_panel_leg,
        origin=Origin(xyz=(-0.720, 0.0, 0.0)),
    )
    model.articulation(
        "main_to_return_panel",
        ArticulationType.FIXED,
        parent=main_worktop,
        child=return_panel_leg,
        origin=Origin(xyz=(1.00, 1.150, 0.0)),
    )
    model.articulation(
        "main_to_pedestal",
        ArticulationType.FIXED,
        parent=main_worktop,
        child=right_pedestal,
        origin=Origin(xyz=(0.450, -0.100, 0.0)),
    )
    model.articulation(
        "pedestal_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=right_pedestal,
        child="lower_drawer",
        origin=Origin(xyz=(0.0, -0.289, 0.194)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.300),
    )
    model.articulation(
        "pedestal_to_middle_drawer",
        ArticulationType.PRISMATIC,
        parent=right_pedestal,
        child="middle_drawer",
        origin=Origin(xyz=(0.0, -0.289, 0.403)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.300),
    )
    model.articulation(
        "pedestal_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=right_pedestal,
        child="upper_drawer",
        origin=Origin(xyz=(0.0, -0.289, 0.612)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.300),
    )
    model.articulation(
        "main_to_keyboard_mount",
        ArticulationType.FIXED,
        parent=main_worktop,
        child=keyboard_mount,
        origin=Origin(xyz=(-0.190, -0.180, 0.656)),
    )
    model.articulation(
        "keyboard_mount_to_tray",
        ArticulationType.PRISMATIC,
        parent=keyboard_mount,
        child=keyboard_tray,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.40, lower=0.0, upper=0.260),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    main_worktop = object_model.get_part("main_worktop")
    return_worktop = object_model.get_part("return_worktop")
    corner_bracket = object_model.get_part("corner_bracket")
    left_panel_leg = object_model.get_part("left_panel_leg")
    return_panel_leg = object_model.get_part("return_panel_leg")
    right_pedestal = object_model.get_part("right_pedestal")
    lower_drawer = object_model.get_part("lower_drawer")
    middle_drawer = object_model.get_part("middle_drawer")
    upper_drawer = object_model.get_part("upper_drawer")
    keyboard_mount = object_model.get_part("keyboard_mount")
    keyboard_tray = object_model.get_part("keyboard_tray")

    lower_joint = object_model.get_articulation("pedestal_to_lower_drawer")
    middle_joint = object_model.get_articulation("pedestal_to_middle_drawer")
    upper_joint = object_model.get_articulation("pedestal_to_upper_drawer")
    tray_joint = object_model.get_articulation("keyboard_mount_to_tray")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    for joint in (lower_joint, middle_joint, upper_joint, tray_joint):
        ctx.check(
            f"{joint.name}_axis_is_negative_y",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            details=f"Expected -Y prismatic travel axis, got {joint.axis!r}",
        )

    ctx.expect_contact(main_worktop, corner_bracket, name="bracket_touches_main_worktop")
    ctx.expect_contact(return_worktop, corner_bracket, name="bracket_touches_return_worktop")
    ctx.expect_contact(main_worktop, return_worktop, name="worktops_meet_at_mitre")
    ctx.expect_contact(main_worktop, left_panel_leg, name="left_panel_supports_main_top")
    ctx.expect_contact(main_worktop, right_pedestal, name="pedestal_supports_main_top")
    ctx.expect_contact(return_worktop, return_panel_leg, name="return_panel_supports_return_top")
    ctx.expect_contact(main_worktop, keyboard_mount, name="keyboard_mount_attaches_to_main_top")

    ctx.expect_overlap(main_worktop, right_pedestal, axes="xy", min_overlap=0.30)
    ctx.expect_overlap(return_worktop, return_panel_leg, axes="x", min_overlap=0.20)
    ctx.expect_overlap(keyboard_tray, main_worktop, axes="x", min_overlap=0.55)
    ctx.expect_gap(
        main_worktop,
        keyboard_tray,
        axis="z",
        min_gap=0.050,
        max_gap=0.090,
        positive_elem="main_top_slab",
        negative_elem="tray_board",
        name="keyboard_tray_hangs_below_main_top",
    )
    ctx.expect_contact(keyboard_tray, keyboard_mount, name="keyboard_tray_supported_by_mount")

    for drawer in (lower_drawer, middle_drawer, upper_drawer):
        ctx.expect_contact(drawer, right_pedestal, name=f"{drawer.name}_supported_by_rails")
        ctx.expect_within(
            drawer,
            right_pedestal,
            axes="xz",
            margin=0.0,
            name=f"{drawer.name}_stays_within_pedestal_width_and_height",
        )

    def _check_prismatic_slide(part, joint, parent, min_travel: float) -> None:
        limits = joint.motion_limits
        assert limits is not None
        assert limits.lower is not None
        assert limits.upper is not None

        rest = ctx.part_world_position(part)
        assert rest is not None

        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            ctx.expect_contact(part, parent, name=f"{joint.name}_lower_contact")

        with ctx.pose({joint: limits.upper}):
            moved = ctx.part_world_position(part)
            assert moved is not None
            ctx.check(
                f"{joint.name}_slides_forward",
                moved[1] < rest[1] - min_travel,
                details=f"Expected at least {min_travel:.3f} m forward slide; rest={rest}, open={moved}",
            )
            ctx.check(
                f"{joint.name}_keeps_x_alignment",
                abs(moved[0] - rest[0]) <= 1e-6,
                details=f"X drifted during prismatic slide: rest={rest}, open={moved}",
            )
            ctx.check(
                f"{joint.name}_keeps_z_alignment",
                abs(moved[2] - rest[2]) <= 1e-6,
                details=f"Z drifted during prismatic slide: rest={rest}, open={moved}",
            )
            ctx.expect_contact(part, parent, name=f"{joint.name}_upper_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    _check_prismatic_slide(lower_drawer, lower_joint, right_pedestal, 0.28)
    _check_prismatic_slide(middle_drawer, middle_joint, right_pedestal, 0.28)
    _check_prismatic_slide(upper_drawer, upper_joint, right_pedestal, 0.28)
    _check_prismatic_slide(keyboard_tray, tray_joint, keyboard_mount, 0.22)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
