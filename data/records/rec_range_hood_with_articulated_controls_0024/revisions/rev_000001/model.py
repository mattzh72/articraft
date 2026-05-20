from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.09
    panel_t = 0.004

    chimney_width = 0.32
    chimney_depth = 0.28
    chimney_height = 0.84
    chimney_center_y = 0.07

    body = model.part("hood_body")

    body.visual(
        Box((canopy_width, canopy_depth, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - 0.0015)),
        material=stainless,
        name="canopy_top",
    )
    body.visual(
        Box((canopy_width, panel_t, canopy_height)),
        origin=Origin(xyz=(0.0, canopy_depth / 2 - panel_t / 2, canopy_height / 2)),
        material=stainless,
        name="canopy_back",
    )
    body.visual(
        Box((panel_t, canopy_depth, canopy_height)),
        origin=Origin(xyz=(-canopy_width / 2 + panel_t / 2, 0.0, canopy_height / 2)),
        material=stainless,
        name="canopy_left",
    )
    body.visual(
        Box((panel_t, canopy_depth, canopy_height)),
        origin=Origin(xyz=(canopy_width / 2 - panel_t / 2, 0.0, canopy_height / 2)),
        material=stainless,
        name="canopy_right",
    )

    strip_z = 0.065
    strip_height = 0.05
    opening_height = 0.02
    rail_height = (strip_height - opening_height) / 2
    cluster_start = -0.338
    bar_w = 0.010
    button_opening = 0.020
    cluster_width = 4 * bar_w + 3 * button_opening
    front_y = -canopy_depth / 2 + panel_t / 2

    body.visual(
        Box((0.112, panel_t, strip_height)),
        origin=Origin(xyz=(-0.394, front_y, strip_z)),
        material=stainless,
        name="front_left_margin",
    )
    body.visual(
        Box((cluster_width, panel_t, rail_height)),
        origin=Origin(
            xyz=(
                cluster_start + cluster_width / 2,
                front_y,
                strip_z + opening_height / 2 + rail_height / 2,
            )
        ),
        material=stainless,
        name="button_cluster_top_rail",
    )
    body.visual(
        Box((cluster_width, panel_t, rail_height)),
        origin=Origin(
            xyz=(
                cluster_start + cluster_width / 2,
                front_y,
                strip_z - opening_height / 2 - rail_height / 2,
            )
        ),
        material=stainless,
        name="button_cluster_bottom_rail",
    )

    bar_centers = (
        cluster_start + bar_w / 2,
        cluster_start + bar_w + button_opening + bar_w / 2,
        cluster_start + 2 * (bar_w + button_opening) + bar_w / 2,
        cluster_start + 3 * button_opening + 3.5 * bar_w,
    )
    for index, x_center in enumerate(bar_centers, start=1):
        body.visual(
            Box((bar_w, panel_t, opening_height)),
            origin=Origin(xyz=(x_center, front_y, strip_z)),
            material=stainless,
            name=f"button_cluster_bar_{index}",
        )

    body.visual(
        Box((0.238, panel_t, strip_height)),
        origin=Origin(xyz=(-0.119, front_y, strip_z)),
        material=stainless,
        name="front_center_filler",
    )
    body.visual(
        Box((0.45, panel_t, strip_height)),
        origin=Origin(xyz=(0.225, front_y, strip_z)),
        material=stainless,
        name="front_knob_plate",
    )

    body.visual(
        Box((canopy_width - 2 * panel_t, canopy_depth - 2 * panel_t, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=charcoal,
        name="grease_filter",
    )

    chimney_z = canopy_height + chimney_height / 2
    body.visual(
        Box((chimney_width, panel_t, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_center_y - chimney_depth / 2 + panel_t / 2, chimney_z)
        ),
        material=stainless,
        name="chimney_front",
    )
    body.visual(
        Box((chimney_width, panel_t, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_center_y + chimney_depth / 2 - panel_t / 2, chimney_z)
        ),
        material=stainless,
        name="chimney_back",
    )
    body.visual(
        Box((panel_t, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(-chimney_width / 2 + panel_t / 2, chimney_center_y, chimney_z)
        ),
        material=stainless,
        name="chimney_left",
    )
    body.visual(
        Box((panel_t, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(chimney_width / 2 - panel_t / 2, chimney_center_y, chimney_z)
        ),
        material=stainless,
        name="chimney_right",
    )
    body.visual(
        Box((chimney_width, chimney_depth, 0.003)),
        origin=Origin(xyz=(0.0, chimney_center_y, canopy_height + chimney_height - 0.0015)),
        material=stainless,
        name="chimney_top",
    )

    button_x_centers = (-0.318, -0.288, -0.258)
    for index, x_center in enumerate(button_x_centers, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.020, 0.006, 0.020)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=control_black,
            name="button_cap",
        )
        button.visual(
            Box((0.020, 0.012, 0.020)),
            origin=Origin(),
            material=control_black,
            name="button_stem",
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_center, front_y, strip_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.08,
                lower=0.0,
                upper=0.006,
            ),
        )

    for knob_name, x_center in (("fan_knob", 0.14), ("light_knob", 0.26)):
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
            material=control_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.002, 0.010)),
            origin=Origin(xyz=(0.0, -0.011, 0.007)),
            material=charcoal,
            name="knob_pointer",
        )
        model.articulation(
            f"{knob_name}_turn",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x_center, -0.260, strip_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("hood_body")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")
    button_3 = object_model.get_part("button_3")
    fan_knob = object_model.get_part("fan_knob")
    light_knob = object_model.get_part("light_knob")

    button_joints = (
        object_model.get_articulation("button_1_press"),
        object_model.get_articulation("button_2_press"),
        object_model.get_articulation("button_3_press"),
    )
    knob_joints = (
        object_model.get_articulation("fan_knob_turn"),
        object_model.get_articulation("light_knob_turn"),
    )

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    for part_name in (
        "hood_body",
        "button_1",
        "button_2",
        "button_3",
        "fan_knob",
        "light_knob",
    ):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    for joint_name in (
        "button_1_press",
        "button_2_press",
        "button_3_press",
        "fan_knob_turn",
        "light_knob_turn",
    ):
        ctx.check(f"{joint_name}_present", object_model.get_articulation(joint_name) is not None)

    for joint in button_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"expected +Y press axis, got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_limits",
            limits is not None
            and joint.joint_type == ArticulationType.PRISMATIC
            and limits.lower == 0.0
            and limits.upper == 0.006,
            details="button press joints should be short-stroke prismatics",
        )

    for joint in knob_joints:
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"expected front-to-back axis, got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_continuous",
            joint.joint_type == ArticulationType.CONTINUOUS,
            details="knobs should rotate continuously",
        )

    with ctx.pose():
        for control in (button_1, button_2, button_3, fan_knob, light_knob):
            ctx.expect_contact(control, body, name=f"{control.name}_mounted_to_body")
            ctx.expect_within(
                control,
                body,
                axes="xz",
                margin=0.02,
                name=f"{control.name}_within_body_xz",
            )

        button_positions = [
            ctx.part_world_position(button_1),
            ctx.part_world_position(button_2),
            ctx.part_world_position(button_3),
        ]
        knob_positions = [
            ctx.part_world_position(fan_knob),
            ctx.part_world_position(light_knob),
        ]
        ctx.check(
            "buttons_are_left_to_right",
            button_positions[0] is not None
            and button_positions[1] is not None
            and button_positions[2] is not None
            and button_positions[0][0] < button_positions[1][0] < button_positions[2][0] < 0.0,
            details=f"unexpected button x positions: {button_positions}",
        )
        ctx.check(
            "knobs_are_on_right_half",
            knob_positions[0] is not None
            and knob_positions[1] is not None
            and 0.0 < knob_positions[0][0] < knob_positions[1][0],
            details=f"unexpected knob x positions: {knob_positions}",
        )

        canopy_top = ctx.part_element_world_aabb(body, elem="canopy_top")
        chimney_top = ctx.part_element_world_aabb(body, elem="chimney_top")
        ctx.check(
            "chimney_rises_well_above_canopy",
            canopy_top is not None
            and chimney_top is not None
            and chimney_top[1][2] - canopy_top[1][2] > 0.70,
            details="chimney shell should be substantially taller than the canopy",
        )

    for button, joint in zip((button_1, button_2, button_3), button_joints):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                rest_pos = ctx.part_world_position(button)
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
                ctx.expect_contact(button, body, name=f"{joint.name}_lower_contact")
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                ctx.expect_contact(button, body, name=f"{joint.name}_upper_contact")
            ctx.check(
                f"{joint.name}_moves_inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] > rest_pos[1]
                and abs((pressed_pos[1] - rest_pos[1]) - limits.upper) < 1e-6,
                details=f"expected +Y button travel of {limits.upper}, got rest={rest_pos}, pressed={pressed_pos}",
            )

    for knob, joint in zip((fan_knob, light_knob), knob_joints):
        with ctx.pose({joint: 0.0}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_zero_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_zero_no_floating")
            ctx.expect_contact(knob, body, name=f"{joint.name}_zero_contact")
        with ctx.pose({joint: pi / 2}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_quarter_turn_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{joint.name}_quarter_turn_no_floating")
            ctx.expect_contact(knob, body, name=f"{joint.name}_quarter_turn_contact")

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
