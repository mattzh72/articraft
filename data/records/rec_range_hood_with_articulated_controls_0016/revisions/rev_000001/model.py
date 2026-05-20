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


def _add_rectangular_frame(
    part,
    *,
    prefix: str,
    center: tuple[float, float, float],
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    depth: float,
    material,
) -> None:
    cx, cy, cz = center
    outer_x, outer_z = outer_size
    inner_x, inner_z = inner_size
    side_thickness = max((outer_x - inner_x) * 0.5, 0.001)
    top_thickness = max((outer_z - inner_z) * 0.5, 0.001)

    side_center_x = (inner_x * 0.5) + (side_thickness * 0.5)
    top_center_z = (inner_z * 0.5) + (top_thickness * 0.5)

    part.visual(
        Box((side_thickness, depth, outer_z)),
        origin=Origin(xyz=(cx - side_center_x, cy, cz)),
        material=material,
        name=f"{prefix}_left_bar",
    )
    part.visual(
        Box((side_thickness, depth, outer_z)),
        origin=Origin(xyz=(cx + side_center_x, cy, cz)),
        material=material,
        name=f"{prefix}_right_bar",
    )
    part.visual(
        Box((inner_x, depth, top_thickness)),
        origin=Origin(xyz=(cx, cy, cz + top_center_z)),
        material=material,
        name=f"{prefix}_top_bar",
    )
    part.visual(
        Box((inner_x, depth, top_thickness)),
        origin=Origin(xyz=(cx, cy, cz - top_center_z)),
        material=material,
        name=f"{prefix}_bottom_bar",
    )


def _build_button(model: ArticulatedObject, name: str, material) -> object:
    button = model.part(name)
    button.visual(
        Box((0.016, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
        material=material,
        name="plunger",
    )
    button.visual(
        Box((0.014, 0.002, 0.014)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=material,
        name="cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.016, 0.010, 0.016)),
        mass=0.015,
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
    )
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    black_control = model.material("black_control", rgba=(0.10, 0.10, 0.11, 1.0))
    brushed_button = model.material("brushed_button", rgba=(0.72, 0.73, 0.75, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        Box((0.904, 0.504, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        material=stainless,
        name="top_panel",
    )
    canopy.visual(
        Box((0.904, 0.020, 0.144)),
        origin=Origin(xyz=(0.0, -0.241, 0.071)),
        material=stainless,
        name="front_fascia",
    )
    canopy.visual(
        Box((0.904, 0.020, 0.144)),
        origin=Origin(xyz=(0.0, 0.241, 0.071)),
        material=stainless,
        name="rear_panel",
    )
    canopy.visual(
        Box((0.024, 0.504, 0.144)),
        origin=Origin(xyz=(-0.440, 0.0, 0.071)),
        material=stainless,
        name="left_panel",
    )
    canopy.visual(
        Box((0.024, 0.504, 0.144)),
        origin=Origin(xyz=(0.440, 0.0, 0.071)),
        material=stainless,
        name="right_panel",
    )
    canopy.visual(
        Box((0.300, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.247, 0.146)),
        material=satin_steel,
        name="upper_front_trim",
    )
    canopy.visual(
        Box((0.340, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.245, 0.018)),
        material=dark_trim,
        name="lamp_strip",
    )

    controls_z = 0.092
    frame_center_y = -0.256
    frame_depth = 0.012

    _add_rectangular_frame(
        canopy,
        prefix="knob_bezel",
        center=(0.0, frame_center_y, controls_z),
        outer_size=(0.070, 0.070),
        inner_size=(0.056, 0.056),
        depth=frame_depth,
        material=dark_trim,
    )
    for x_pos, prefix in (
        (-0.105, "button_outer_left"),
        (-0.055, "button_inner_left"),
        (0.055, "button_inner_right"),
        (0.105, "button_outer_right"),
    ):
        _add_rectangular_frame(
            canopy,
            prefix=prefix,
            center=(x_pos, frame_center_y, controls_z),
            outer_size=(0.022, 0.022),
            inner_size=(0.016, 0.016),
            depth=frame_depth,
            material=dark_trim,
        )

    canopy.inertial = Inertial.from_geometry(
        Box((0.90, 0.50, 0.16)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    chimney = model.part("chimney_chase")
    chimney.visual(
        Box((0.326, 0.014, 0.742)),
        origin=Origin(xyz=(0.0, -0.133, 0.371)),
        material=satin_steel,
        name="front_chase",
    )
    chimney.visual(
        Box((0.326, 0.014, 0.742)),
        origin=Origin(xyz=(0.0, 0.133, 0.371)),
        material=satin_steel,
        name="rear_chase",
    )
    chimney.visual(
        Box((0.014, 0.286, 0.742)),
        origin=Origin(xyz=(-0.156, 0.0, 0.371)),
        material=satin_steel,
        name="left_chase",
    )
    chimney.visual(
        Box((0.014, 0.286, 0.742)),
        origin=Origin(xyz=(0.156, 0.0, 0.371)),
        material=satin_steel,
        name="right_chase",
    )
    chimney.inertial = Inertial.from_geometry(
        Box((0.32, 0.28, 0.74)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    knob = model.part("main_knob")
    knob.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_control,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="knob_mount",
    )
    knob.visual(
        Box((0.004, 0.002, 0.012)),
        origin=Origin(xyz=(0.0, -0.019, 0.014)),
        material=brushed_button,
        name="indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.050, 0.020, 0.050)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
    )

    model.articulation(
        "canopy_to_main_knob",
        ArticulationType.CONTINUOUS,
        parent=canopy,
        child=knob,
        origin=Origin(xyz=(0.0, -0.250, controls_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    button_specs = (
        ("left_outer_button", -0.105),
        ("left_inner_button", -0.055),
        ("right_inner_button", 0.055),
        ("right_outer_button", 0.105),
    )
    for part_name, x_pos in button_specs:
        button = _build_button(model, part_name, brushed_button)
        model.articulation(
            f"canopy_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(x_pos, -0.255, controls_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.04,
                lower=0.0,
                upper=0.0025,
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

    canopy = object_model.get_part("canopy")
    chimney = object_model.get_part("chimney_chase")
    knob = object_model.get_part("main_knob")
    left_outer = object_model.get_part("left_outer_button")
    left_inner = object_model.get_part("left_inner_button")
    right_inner = object_model.get_part("right_inner_button")
    right_outer = object_model.get_part("right_outer_button")

    chimney_joint = object_model.get_articulation("canopy_to_chimney")
    knob_joint = object_model.get_articulation("canopy_to_main_knob")
    button_joints = [
        object_model.get_articulation("canopy_to_left_outer_button"),
        object_model.get_articulation("canopy_to_left_inner_button"),
        object_model.get_articulation("canopy_to_right_inner_button"),
        object_model.get_articulation("canopy_to_right_outer_button"),
    ]
    buttons = [left_outer, left_inner, right_inner, right_outer]

    def _center_from_aabb(aabb):
        return tuple((aabb[0][idx] + aabb[1][idx]) * 0.5 for idx in range(3))

    ctx.expect_contact(chimney, canopy, name="chimney_seated_on_canopy")
    ctx.expect_gap(chimney, canopy, axis="z", max_gap=0.001, max_penetration=0.0, name="chimney_vertical_seat")
    ctx.expect_within(chimney, canopy, axes="xy", margin=0.31, name="chimney_centered_over_canopy")

    ctx.expect_contact(knob, canopy, name="knob_is_mounted")
    for button in buttons:
        ctx.expect_contact(button, canopy, name=f"{button.name}_is_guided")

    ctx.expect_origin_distance(knob, canopy, axes="x", min_dist=0.0, max_dist=0.001, name="knob_centered")

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "knob_joint_type",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"expected continuous knob, got {knob_joint.articulation_type}",
    )
    ctx.check(
        "knob_joint_axis",
        tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        f"expected knob axis (0,1,0), got {knob_joint.axis}",
    )
    ctx.check(
        "knob_joint_unbounded",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        "continuous knob should have effort/velocity only",
    )

    for joint in button_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_type",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"expected prismatic articulation, got {joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"expected inward button axis (0,1,0), got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_limits",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - 0.0) < 1e-9
            and abs(limits.upper - 0.0025) < 1e-9,
            f"expected 0.0 to 0.0025 m button travel, got {limits}",
        )
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
                ctx.expect_contact(joint.child, canopy, name=f"{joint.name}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                ctx.expect_contact(joint.child, canopy, name=f"{joint.name}_upper_contact")

    rest_positions = {button.name: ctx.part_world_position(button) for button in buttons}
    with ctx.pose({joint: 0.0025 for joint in button_joints}):
        for button in buttons:
            pressed_position = ctx.part_world_position(button)
            rest_position = rest_positions[button.name]
            ok = (
                rest_position is not None
                and pressed_position is not None
                and pressed_position[1] > rest_position[1] + 0.0020
                and abs(pressed_position[0] - rest_position[0]) < 1e-6
                and abs(pressed_position[2] - rest_position[2]) < 1e-6
            )
            ctx.check(
                f"{button.name}_moves_inward_only",
                ok,
                f"rest={rest_position}, pressed={pressed_position}",
            )
        ctx.fail_if_parts_overlap_in_current_pose(name="all_buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="all_buttons_pressed_no_floating")

    knob_indicator_rest = ctx.part_element_world_aabb(knob, elem="indicator")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_indicator_quarter = ctx.part_element_world_aabb(knob, elem="indicator")
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_quarter_turn_no_floating")

    knob_rotates_visibly = False
    if knob_indicator_rest is not None and knob_indicator_quarter is not None:
        rest_center = _center_from_aabb(knob_indicator_rest)
        quarter_center = _center_from_aabb(knob_indicator_quarter)
        knob_rotates_visibly = (
            abs(rest_center[2] - quarter_center[2]) > 0.008
            and abs(rest_center[0] - quarter_center[0]) > 0.008
        )
    ctx.check(
        "knob_indicator_rotates_with_joint",
        knob_rotates_visibly,
        f"indicator aabbs rest={knob_indicator_rest}, quarter_turn={knob_indicator_quarter}",
    )

    left_outer_pos = ctx.part_world_position(left_outer)
    left_inner_pos = ctx.part_world_position(left_inner)
    knob_pos = ctx.part_world_position(knob)
    right_inner_pos = ctx.part_world_position(right_inner)
    right_outer_pos = ctx.part_world_position(right_outer)
    symmetry_ok = (
        left_outer_pos is not None
        and left_inner_pos is not None
        and knob_pos is not None
        and right_inner_pos is not None
        and right_outer_pos is not None
        and abs(knob_pos[0]) < 1e-6
        and abs(left_outer_pos[0] + right_outer_pos[0]) < 1e-6
        and abs(left_inner_pos[0] + right_inner_pos[0]) < 1e-6
        and abs(left_outer_pos[2] - knob_pos[2]) < 1e-6
        and abs(left_inner_pos[2] - knob_pos[2]) < 1e-6
        and abs(right_inner_pos[2] - knob_pos[2]) < 1e-6
        and abs(right_outer_pos[2] - knob_pos[2]) < 1e-6
    )
    ctx.check(
        "five_control_layout_is_symmetric",
        symmetry_ok,
        (
            f"left_outer={left_outer_pos}, left_inner={left_inner_pos}, knob={knob_pos}, "
            f"right_inner={right_inner_pos}, right_outer={right_outer_pos}"
        ),
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

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
