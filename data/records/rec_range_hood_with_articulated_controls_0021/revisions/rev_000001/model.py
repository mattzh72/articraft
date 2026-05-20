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
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    steel = model.material("brushed_steel", rgba=(0.77, 0.79, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.22, 0.24, 1.0))
    black_control = model.material("black_control", rgba=(0.10, 0.10, 0.11, 1.0))
    button_finish = model.material("button_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    indicator = model.material("indicator", rgba=(0.86, 0.18, 0.12, 1.0))

    body = model.part("body")

    canopy_top = body.visual(
        Box((0.60, 0.50, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        material=steel,
        name="canopy_top",
    )
    body.visual(
        Box((0.62, 0.012, 0.288)),
        origin=Origin(xyz=(0.0, -0.244, 0.144)),
        material=steel,
        name="back_panel",
    )

    side_angle = math.atan2(0.15, 0.30)
    side_length = math.hypot(0.30, 0.15) + 0.012
    body.visual(
        Box((0.012, 0.50, side_length)),
        origin=Origin(xyz=(-0.375, 0.0, 0.150), rpy=(0.0, side_angle, 0.0)),
        material=steel,
        name="left_side_shell",
    )
    body.visual(
        Box((0.012, 0.50, side_length)),
        origin=Origin(xyz=(0.375, 0.0, 0.150), rpy=(0.0, -side_angle, 0.0)),
        material=steel,
        name="right_side_shell",
    )

    cheek_height = 0.228
    body.visual(
        Box((0.64, 0.012, 0.248)),
        origin=Origin(xyz=(0.0, 0.244, 0.166)),
        material=steel,
        name="upper_front_panel",
    )
    body.visual(
        Box((0.430, 0.012, 0.048)),
        origin=Origin(xyz=(-0.085, 0.244, 0.024)),
        material=steel,
        name="lower_front_left_panel",
    )

    body.visual(
        Box((0.170, 0.012, 0.013)),
        origin=Origin(xyz=(0.215, 0.244, 0.0065)),
        material=dark_trim,
        name="button_strip_bottom_rail",
    )
    body.visual(
        Box((0.170, 0.012, 0.013)),
        origin=Origin(xyz=(0.215, 0.244, 0.0415)),
        material=dark_trim,
        name="button_strip_top_rail",
    )
    body.visual(
        Box((0.010, 0.012, 0.022)),
        origin=Origin(xyz=(0.135, 0.244, 0.024)),
        material=dark_trim,
        name="button_strip_left_rail",
    )
    body.visual(
        Box((0.010, 0.012, 0.022)),
        origin=Origin(xyz=(0.215, 0.244, 0.024)),
        material=dark_trim,
        name="button_strip_mid_rail",
    )
    body.visual(
        Box((0.010, 0.012, 0.022)),
        origin=Origin(xyz=(0.295, 0.244, 0.024)),
        material=dark_trim,
        name="button_strip_right_rail",
    )

    chimney_center_y = -0.060
    chimney_height = 0.520
    chimney_z = 0.300 + chimney_height / 2.0
    body.visual(
        Box((0.008, 0.200, chimney_height)),
        origin=Origin(xyz=(-0.126, chimney_center_y, chimney_z)),
        material=steel,
        name="chimney_left_wall",
    )
    body.visual(
        Box((0.008, 0.200, chimney_height)),
        origin=Origin(xyz=(0.126, chimney_center_y, chimney_z)),
        material=steel,
        name="chimney_right_wall",
    )
    body.visual(
        Box((0.260, 0.008, chimney_height)),
        origin=Origin(xyz=(0.0, -0.156, chimney_z)),
        material=steel,
        name="chimney_back_wall",
    )
    body.visual(
        Box((0.260, 0.008, chimney_height)),
        origin=Origin(xyz=(0.0, 0.036, chimney_z)),
        material=steel,
        name="chimney_front_wall",
    )
    body.visual(
        Box((0.260, 0.200, 0.008)),
        origin=Origin(xyz=(0.0, chimney_center_y, 0.816)),
        material=steel,
        name="chimney_cap",
    )

    body.inertial = Inertial.from_geometry(
        Box((0.90, 0.50, 0.824)),
        mass=17.0,
        origin=Origin(xyz=(0.0, 0.0, 0.412)),
    )

    left_cheek_panel = model.part("left_cheek_panel")
    left_cheek_panel.visual(
        Box((0.090, 0.010, 0.220)),
        origin=Origin(xyz=(-0.275, 0.255, 0.165)),
        material=steel,
        name="left_cheek_plate",
    )
    left_cheek_panel.inertial = Inertial.from_geometry(
        Box((0.090, 0.010, 0.220)),
        mass=0.35,
        origin=Origin(xyz=(-0.275, 0.255, 0.165)),
    )
    model.articulation(
        "body_to_left_cheek_panel",
        ArticulationType.FIXED,
        parent=body,
        child=left_cheek_panel,
        origin=Origin(),
    )

    right_cheek_panel = model.part("right_cheek_panel")
    right_cheek_panel.visual(
        Box((0.090, 0.010, 0.220)),
        origin=Origin(xyz=(0.275, 0.255, 0.165)),
        material=steel,
        name="right_cheek_plate",
    )
    right_cheek_panel.inertial = Inertial.from_geometry(
        Box((0.090, 0.010, 0.220)),
        mass=0.35,
        origin=Origin(xyz=(0.275, 0.255, 0.165)),
    )
    model.articulation(
        "body_to_right_cheek_panel",
        ArticulationType.FIXED,
        parent=body,
        child=right_cheek_panel,
        origin=Origin(),
    )

    def add_knob(
        name: str,
        *,
        parent_part,
        joint_xyz: tuple[float, float, float],
        joint_rpy: tuple[float, float, float],
    ) -> None:
        knob = model.part(name)
        knob.visual(
            Sphere(radius=0.0045),
            origin=Origin(xyz=(0.0, 0.0, 0.0045)),
            material=black_control,
            name="knob_base",
        )
        knob.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.0105)),
            material=black_control,
            name="knob_body",
        )
        knob.visual(
            Box((0.003, 0.012, 0.003)),
            origin=Origin(xyz=(0.0, 0.0105, 0.0165)),
            material=indicator,
            name="knob_pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Sphere(radius=0.016),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
        )
        model.articulation(
            f"{parent_part.name}_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=parent_part,
            child=knob,
            origin=Origin(xyz=joint_xyz, rpy=joint_rpy),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.3, velocity=10.0),
        )

    cheek_surface_x = -0.275
    cheek_surface_y = 0.260
    cheek_mount_rpy = (-math.pi / 2.0, 0.0, 0.0)

    add_knob(
        "upper_knob",
        parent_part=left_cheek_panel,
        joint_xyz=(cheek_surface_x, cheek_surface_y, 0.185),
        joint_rpy=cheek_mount_rpy,
    )
    add_knob(
        "lower_knob",
        parent_part=left_cheek_panel,
        joint_xyz=(cheek_surface_x, cheek_surface_y, 0.110),
        joint_rpy=cheek_mount_rpy,
    )

    def add_button(name: str, center_x: float) -> None:
        button = model.part(name)
        button.visual(
            Box((0.070, 0.022, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=button_finish,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.070, 0.022, 0.010)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(
                xyz=(center_x, 0.248, 0.024),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.10,
                lower=0.0,
                upper=0.006,
            ),
        )

    add_button("left_button", 0.175)
    add_button("right_button", 0.255)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    upper_knob = object_model.get_part("upper_knob")
    lower_knob = object_model.get_part("lower_knob")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    left_cheek_panel = object_model.get_part("left_cheek_panel")
    upper_knob_joint = object_model.get_articulation("left_cheek_panel_to_upper_knob")
    lower_knob_joint = object_model.get_articulation("left_cheek_panel_to_lower_knob")
    left_button_joint = object_model.get_articulation("body_to_left_button")
    right_button_joint = object_model.get_articulation("body_to_right_button")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_cheek_panel, body, name="left_cheek_panel_mounted_to_body")
    for control in (upper_knob, lower_knob):
        ctx.expect_contact(control, left_cheek_panel, name=f"{control.name}_mounted_to_left_cheek")
    for control in (left_button, right_button):
        ctx.expect_contact(control, body, name=f"{control.name}_mounted_to_body")

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_has_aabb", "Body AABB was unavailable.")
    else:
        width = body_aabb[1][0] - body_aabb[0][0]
        depth = body_aabb[1][1] - body_aabb[0][1]
        height = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "range_hood_realistic_size",
            0.85 <= width <= 0.95 and 0.49 <= depth <= 0.53 and 0.80 <= height <= 0.84,
            f"Unexpected body dimensions width={width:.3f}, depth={depth:.3f}, height={height:.3f}",
        )

    upper_pos = ctx.part_world_position(upper_knob)
    lower_pos = ctx.part_world_position(lower_knob)
    left_button_pos = ctx.part_world_position(left_button)
    right_button_pos = ctx.part_world_position(right_button)
    if None in (upper_pos, lower_pos, left_button_pos, right_button_pos):
        ctx.fail("control_positions_available", "One or more control positions were unavailable.")
    else:
        assert upper_pos is not None
        assert lower_pos is not None
        assert left_button_pos is not None
        assert right_button_pos is not None
        ctx.check(
            "knobs_stacked_vertically",
            abs(upper_pos[0] - lower_pos[0]) <= 0.01
            and abs(upper_pos[1] - lower_pos[1]) <= 0.01
            and upper_pos[2] > lower_pos[2] + 0.05,
            (
                f"Knob centers not stacked on left cheek: "
                f"upper={upper_pos}, lower={lower_pos}"
            ),
        )
        ctx.check(
            "buttons_aligned_horizontally",
            abs(left_button_pos[2] - right_button_pos[2]) <= 0.003
            and right_button_pos[0] > left_button_pos[0] + 0.05,
            (
                f"Buttons are not laid out horizontally on the lower-right strip: "
                f"left={left_button_pos}, right={right_button_pos}"
            ),
        )
        ctx.check(
            "controls_on_correct_sides",
            upper_pos[0] < -0.25
            and lower_pos[0] < -0.25
            and left_button_pos[0] > 0.14
            and right_button_pos[0] > left_button_pos[0],
            (
                f"Control layout incorrect: upper={upper_pos}, lower={lower_pos}, "
                f"left_button={left_button_pos}, right_button={right_button_pos}"
            ),
        )

    with ctx.pose({upper_knob_joint: math.pi / 2.0, lower_knob_joint: -math.pi / 3.0}):
        ctx.expect_contact(upper_knob, left_cheek_panel, name="upper_knob_contact_when_rotated")
        ctx.expect_contact(lower_knob, left_cheek_panel, name="lower_knob_contact_when_rotated")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    with ctx.pose({left_button_joint: 0.006, right_button_joint: 0.006}):
        ctx.expect_contact(left_button, body, name="left_button_contact_when_pressed")
        ctx.expect_contact(right_button, body, name="right_button_contact_when_pressed")
        ctx.fail_if_isolated_parts(name="pressed_buttons_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="pressed_buttons_no_overlap")
        left_pressed = ctx.part_world_position(left_button)
        right_pressed = ctx.part_world_position(right_button)
        if left_rest is not None and right_rest is not None and left_pressed is not None and right_pressed is not None:
            ctx.check(
                "left_button_moves_inward",
                left_pressed[1] < left_rest[1] - 0.004,
                f"Left button did not move inward enough: rest={left_rest}, pressed={left_pressed}",
            )
            ctx.check(
                "right_button_moves_inward",
                right_pressed[1] < right_rest[1] - 0.004,
                f"Right button did not move inward enough: rest={right_rest}, pressed={right_pressed}",
            )

    for joint in (left_button_joint, right_button_joint):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
