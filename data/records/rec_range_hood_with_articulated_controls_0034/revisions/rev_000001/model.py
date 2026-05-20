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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _trapezoid_panel_mesh(
    *,
    filename: str,
    bottom_width: float,
    top_width: float,
    height: float,
    thickness: float,
):
    profile = [
        (-bottom_width / 2.0, 0.0),
        (bottom_width / 2.0, 0.0),
        (top_width / 2.0, height),
        (-top_width / 2.0, height),
    ]
    panel = ExtrudeGeometry(profile, thickness, cap=True, center=True, closed=True)
    panel.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(panel, ASSETS.mesh_path(filename))


def _rounded_button_mesh(
    *,
    filename: str,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
):
    profile = rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    button = ExtrudeGeometry(profile, thickness, cap=True, center=True, closed=True)
    button.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(button, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.35, 0.37, 0.40, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.14, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_button = model.material("satin_button", rgba=(0.62, 0.64, 0.66, 1.0))

    canopy_bottom_width = 0.90
    canopy_top_width = 0.56
    canopy_depth = 0.50
    canopy_height = 0.22
    shell_t = 0.012
    join_t = 0.002

    chimney_width = 0.24
    chimney_depth = 0.18
    chimney_height = 0.56
    chimney_t = 0.008
    chimney_center_y = (-canopy_depth / 2.0) + (chimney_depth / 2.0)
    front_surface_y = canopy_depth / 2.0

    control_strip_width = 0.32
    control_strip_height = 0.15
    control_strip_t = 0.010
    control_strip_center_z = 0.092
    control_strip_outer_y = front_surface_y + control_strip_t

    button_width = 0.074
    button_height = 0.044
    button_t = 0.008

    side_splay = (canopy_bottom_width - canopy_top_width) / 2.0
    side_angle = math.atan2(side_splay, canopy_height)
    side_length = math.hypot(side_splay, canopy_height)
    side_center_x = -(canopy_bottom_width + canopy_top_width) / 4.0

    canopy_panel_mesh = _trapezoid_panel_mesh(
        filename="range_hood_canopy_panel.obj",
        bottom_width=canopy_bottom_width,
        top_width=canopy_top_width,
        height=canopy_height,
        thickness=shell_t,
    )
    button_mesh = _rounded_button_mesh(
        filename="range_hood_button.obj",
        width=button_width,
        height=button_height,
        thickness=button_t,
        corner_radius=0.007,
    )

    hood_body = model.part("hood_body")
    hood_body.visual(
        canopy_panel_mesh,
        origin=Origin(xyz=(0.0, canopy_depth / 2.0 - shell_t / 2.0, 0.0)),
        material=stainless,
        name="canopy_front",
    )
    hood_body.visual(
        canopy_panel_mesh,
        origin=Origin(xyz=(0.0, -canopy_depth / 2.0 + shell_t / 2.0, 0.0)),
        material=stainless,
        name="canopy_back",
    )
    hood_body.visual(
        Box((shell_t, canopy_depth - 2.0 * shell_t + join_t, side_length + join_t)),
        origin=Origin(
            xyz=(side_center_x, 0.0, canopy_height / 2.0),
            rpy=(0.0, side_angle, 0.0),
        ),
        material=stainless,
        name="canopy_left_side",
    )
    hood_body.visual(
        Box((shell_t, canopy_depth - 2.0 * shell_t + join_t, side_length + join_t)),
        origin=Origin(
            xyz=(-side_center_x, 0.0, canopy_height / 2.0),
            rpy=(0.0, -side_angle, 0.0),
        ),
        material=stainless,
        name="canopy_right_side",
    )
    hood_body.visual(
        Box((canopy_top_width + join_t, canopy_depth + join_t, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - shell_t / 2.0)),
        material=stainless,
        name="canopy_top",
    )
    hood_body.visual(
        Box((canopy_bottom_width - 2.0 * shell_t, canopy_depth - 2.0 * shell_t, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=brushed_dark,
        name="grease_filter",
    )
    hood_body.visual(
        Box((control_strip_width, control_strip_t, control_strip_height)),
        origin=Origin(
            xyz=(0.0, front_surface_y + control_strip_t / 2.0, control_strip_center_z)
        ),
        material=charcoal,
        name="control_strip",
    )

    chimney_z = canopy_height + chimney_height / 2.0 - 0.001
    hood_body.visual(
        Box((chimney_width, chimney_t, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y + chimney_depth / 2.0 - chimney_t / 2.0,
                chimney_z,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((chimney_width, chimney_t, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y - chimney_depth / 2.0 + chimney_t / 2.0,
                chimney_z,
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((chimney_t, chimney_depth - 2.0 * chimney_t + join_t, chimney_height)),
        origin=Origin(
            xyz=(chimney_width / 2.0 - chimney_t / 2.0, chimney_center_y, chimney_z)
        ),
        material=stainless,
        name="chimney_right_side",
    )
    hood_body.visual(
        Box((chimney_t, chimney_depth - 2.0 * chimney_t + join_t, chimney_height)),
        origin=Origin(
            xyz=(-chimney_width / 2.0 + chimney_t / 2.0, chimney_center_y, chimney_z)
        ),
        material=stainless,
        name="chimney_left_side",
    )
    hood_body.visual(
        Box((chimney_width - 2.0 * chimney_t, 0.02, 0.024)),
        origin=Origin(xyz=(0.0, chimney_center_y - chimney_depth / 2.0 + 0.018, canopy_height + 0.018)),
        material=charcoal,
        name="duct_shadow",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_bottom_width, canopy_depth, canopy_height + chimney_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + chimney_height) / 2.0)),
    )

    button_specs = (
        ("left_button", -0.065, 0.115),
        ("right_button", 0.065, 0.115),
    )
    knob_specs = (
        ("left_knob", -0.065, 0.058),
        ("right_knob", 0.065, 0.058),
    )

    for part_name, x_pos, z_pos in button_specs:
        button = model.part(part_name)
        button.visual(
            button_mesh,
            origin=Origin(xyz=(0.0, button_t / 2.0, 0.0)),
            material=satin_button,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_width, button_t, button_height)),
            mass=0.03,
            origin=Origin(xyz=(0.0, button_t / 2.0, 0.0)),
        )
        model.articulation(
            f"hood_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(x_pos, control_strip_outer_y, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.06,
                lower=0.0,
                upper=0.004,
            ),
        )

    for part_name, x_pos, z_pos in knob_specs:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_plastic,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="knob_face",
        )
        knob.visual(
            Box((0.0035, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.022, 0.010)),
            material=charcoal,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.016, length=0.014),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"hood_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x_pos, control_strip_outer_y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")

    left_button_joint = object_model.get_articulation("hood_to_left_button")
    right_button_joint = object_model.get_articulation("hood_to_right_button")
    left_knob_joint = object_model.get_articulation("hood_to_left_knob")
    right_knob_joint = object_model.get_articulation("hood_to_right_knob")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(
        left_button,
        hood_body,
        elem_b="control_strip",
        reason="The push-button cap plunges into the shallow switch cavity behind the front control strip.",
    )
    ctx.allow_overlap(
        right_button,
        hood_body,
        elem_b="control_strip",
        reason="The push-button cap plunges into the shallow switch cavity behind the front control strip.",
    )

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

    def _aabb_size(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    def _aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    hood_aabb = ctx.part_world_aabb(hood_body)
    canopy_front_aabb = ctx.part_element_world_aabb(hood_body, elem="canopy_front")
    chimney_front_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_front")
    left_button_pos = ctx.part_world_position(left_button)
    right_button_pos = ctx.part_world_position(right_button)
    left_knob_pos = ctx.part_world_position(left_knob)
    right_knob_pos = ctx.part_world_position(right_knob)

    if hood_aabb is not None:
        hood_size = _aabb_size(hood_aabb)
        ctx.check(
            "hood_overall_proportions",
            0.88 <= hood_size[0] <= 0.92
            and 0.50 <= hood_size[1] <= 0.53
            and 0.77 <= hood_size[2] <= 0.80,
            details=f"overall size was {hood_size}",
        )
    else:
        ctx.fail("hood_overall_proportions", "hood_body AABB was unavailable")

    if canopy_front_aabb is not None and chimney_front_aabb is not None:
        canopy_width = _aabb_size(canopy_front_aabb)[0]
        chimney_width_world = _aabb_size(chimney_front_aabb)[0]
        ctx.check(
            "chimney_narrower_than_canopy",
            chimney_width_world < canopy_width * 0.5,
            details=f"chimney width {chimney_width_world} vs canopy width {canopy_width}",
        )
    else:
        ctx.fail(
            "chimney_narrower_than_canopy",
            "could not evaluate canopy_front and chimney_front AABBs",
        )

    ctx.check(
        "button_joint_types",
        left_button_joint.articulation_type == ArticulationType.PRISMATIC
        and right_button_joint.articulation_type == ArticulationType.PRISMATIC,
        details="Both buttons should be prismatic push-buttons.",
    )
    ctx.check(
        "button_joint_axes",
        tuple(left_button_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(right_button_joint.axis) == (0.0, -1.0, 0.0),
        details=f"button axes were {left_button_joint.axis} and {right_button_joint.axis}",
    )
    ctx.check(
        "knob_joint_types",
        left_knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details="Both knobs should rotate continuously.",
    )
    ctx.check(
        "knob_joint_axes",
        tuple(left_knob_joint.axis) == (0.0, 1.0, 0.0)
        and tuple(right_knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"knob axes were {left_knob_joint.axis} and {right_knob_joint.axis}",
    )

    ctx.expect_contact(left_button, hood_body, elem_b="control_strip")
    ctx.expect_contact(right_button, hood_body, elem_b="control_strip")
    ctx.expect_contact(left_knob, hood_body, elem_b="control_strip")
    ctx.expect_contact(right_knob, hood_body, elem_b="control_strip")

    ctx.expect_overlap(left_button, hood_body, axes="xz", elem_b="control_strip", min_overlap=0.035)
    ctx.expect_overlap(right_button, hood_body, axes="xz", elem_b="control_strip", min_overlap=0.035)
    ctx.expect_overlap(left_knob, hood_body, axes="xz", elem_b="control_strip", min_overlap=0.020)
    ctx.expect_overlap(right_knob, hood_body, axes="xz", elem_b="control_strip", min_overlap=0.020)

    ctx.expect_gap(
        left_button,
        hood_body,
        axis="y",
        negative_elem="control_strip",
        max_gap=0.0005,
        max_penetration=0.0,
        name="left_button_rest_flush",
    )
    ctx.expect_gap(
        right_button,
        hood_body,
        axis="y",
        negative_elem="control_strip",
        max_gap=0.0005,
        max_penetration=0.0,
        name="right_button_rest_flush",
    )
    ctx.expect_gap(
        left_knob,
        hood_body,
        axis="y",
        negative_elem="control_strip",
        max_gap=0.0005,
        max_penetration=0.0,
        name="left_knob_rest_flush",
    )
    ctx.expect_gap(
        right_knob,
        hood_body,
        axis="y",
        negative_elem="control_strip",
        max_gap=0.0005,
        max_penetration=0.0,
        name="right_knob_rest_flush",
    )

    ctx.expect_origin_gap(
        left_button,
        left_knob,
        axis="z",
        min_gap=0.045,
        max_gap=0.065,
        name="left_knob_beneath_left_button",
    )
    ctx.expect_origin_gap(
        right_button,
        right_knob,
        axis="z",
        min_gap=0.045,
        max_gap=0.065,
        name="right_knob_beneath_right_button",
    )

    if (
        left_button_pos is not None
        and right_button_pos is not None
        and left_knob_pos is not None
        and right_knob_pos is not None
    ):
        ctx.check(
            "controls_centered_on_front_strip",
            abs(left_button_pos[0] + right_button_pos[0]) < 0.002
            and abs(left_knob_pos[0] + right_knob_pos[0]) < 0.002
            and abs(left_button_pos[1] - right_button_pos[1]) < 0.001
            and abs(left_knob_pos[1] - right_knob_pos[1]) < 0.001,
            details=(
                f"button positions were {left_button_pos} and {right_button_pos}; "
                f"knob positions were {left_knob_pos} and {right_knob_pos}"
            ),
        )
    else:
        ctx.fail("controls_centered_on_front_strip", "one or more control positions were unavailable")

    left_button_limits = left_button_joint.motion_limits
    right_button_limits = right_button_joint.motion_limits
    if left_button_limits is not None and left_button_limits.upper is not None:
        left_button_rest = ctx.part_world_position(left_button)
        with ctx.pose({left_button_joint: left_button_limits.upper}):
            left_button_pressed = ctx.part_world_position(left_button)
            ctx.fail_if_parts_overlap_in_current_pose(name="left_button_pressed_allowed_overlap_only")
            ctx.fail_if_isolated_parts(name="left_button_pressed_no_floating")
            ctx.expect_gap(
                left_button,
                hood_body,
                axis="y",
                negative_elem="control_strip",
                min_gap=-0.0045,
                max_gap=-0.0035,
                name="left_button_pressed_inward_travel",
            )
            if left_button_rest is not None and left_button_pressed is not None:
                ctx.check(
                    "left_button_world_motion",
                    left_button_pressed[1] < left_button_rest[1] - 0.0035,
                    details=f"rest {left_button_rest}, pressed {left_button_pressed}",
                )
            else:
                ctx.fail("left_button_world_motion", "left button positions were unavailable")

    if right_button_limits is not None and right_button_limits.upper is not None:
        right_button_rest = ctx.part_world_position(right_button)
        with ctx.pose({right_button_joint: right_button_limits.upper}):
            right_button_pressed = ctx.part_world_position(right_button)
            ctx.fail_if_parts_overlap_in_current_pose(name="right_button_pressed_allowed_overlap_only")
            ctx.fail_if_isolated_parts(name="right_button_pressed_no_floating")
            ctx.expect_gap(
                right_button,
                hood_body,
                axis="y",
                negative_elem="control_strip",
                min_gap=-0.0045,
                max_gap=-0.0035,
                name="right_button_pressed_inward_travel",
            )
            if right_button_rest is not None and right_button_pressed is not None:
                ctx.check(
                    "right_button_world_motion",
                    right_button_pressed[1] < right_button_rest[1] - 0.0035,
                    details=f"rest {right_button_rest}, pressed {right_button_pressed}",
                )
            else:
                ctx.fail("right_button_world_motion", "right button positions were unavailable")

    left_indicator_rest = ctx.part_element_world_aabb(left_knob, elem="indicator")
    if left_indicator_rest is not None:
        left_indicator_rest_center = _aabb_center(left_indicator_rest)
        with ctx.pose({left_knob_joint: math.pi / 2.0}):
            left_indicator_turned = ctx.part_element_world_aabb(left_knob, elem="indicator")
            ctx.fail_if_parts_overlap_in_current_pose(name="left_knob_rotated_no_overlap")
            ctx.fail_if_isolated_parts(name="left_knob_rotated_no_floating")
            if left_indicator_turned is not None:
                left_indicator_turned_center = _aabb_center(left_indicator_turned)
                ctx.check(
                    "left_knob_indicator_rotates",
                    left_indicator_turned_center[0] > left_indicator_rest_center[0] + 0.007
                    and left_indicator_turned_center[2] < left_indicator_rest_center[2] - 0.007,
                    details=(
                        f"rest center {left_indicator_rest_center}, "
                        f"turned center {left_indicator_turned_center}"
                    ),
                )
            else:
                ctx.fail("left_knob_indicator_rotates", "left knob indicator AABB was unavailable")
    else:
        ctx.fail("left_knob_indicator_rotates", "left knob rest indicator AABB was unavailable")

    right_indicator_rest = ctx.part_element_world_aabb(right_knob, elem="indicator")
    if right_indicator_rest is not None:
        right_indicator_rest_center = _aabb_center(right_indicator_rest)
        with ctx.pose({right_knob_joint: -math.pi / 2.0}):
            right_indicator_turned = ctx.part_element_world_aabb(right_knob, elem="indicator")
            ctx.fail_if_parts_overlap_in_current_pose(name="right_knob_rotated_no_overlap")
            ctx.fail_if_isolated_parts(name="right_knob_rotated_no_floating")
            if right_indicator_turned is not None:
                right_indicator_turned_center = _aabb_center(right_indicator_turned)
                ctx.check(
                    "right_knob_indicator_rotates",
                    right_indicator_turned_center[0] < right_indicator_rest_center[0] - 0.007
                    and right_indicator_turned_center[2] < right_indicator_rest_center[2] - 0.007,
                    details=(
                        f"rest center {right_indicator_rest_center}, "
                        f"turned center {right_indicator_turned_center}"
                    ),
                )
            else:
                ctx.fail("right_knob_indicator_rotates", "right knob indicator AABB was unavailable")
    else:
        ctx.fail("right_knob_indicator_rotates", "right knob rest indicator AABB was unavailable")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
