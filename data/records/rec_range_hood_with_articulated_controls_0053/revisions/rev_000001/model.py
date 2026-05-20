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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    hood_white = model.material("hood_white", rgba=(0.94, 0.94, 0.92, 1.0))
    filter_aluminum = model.material("filter_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    control_dark = model.material("control_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_white = model.material("lens_white", rgba=(0.90, 0.90, 0.86, 0.85))

    width = 0.76
    depth = 0.50
    height = 0.12
    shell_t = 0.012

    front_y = -depth * 0.5
    rear_y = depth * 0.5
    left_x = -width * 0.5
    right_x = width * 0.5
    top_z = height * 0.5
    bottom_z = -height * 0.5

    pocket_width = 0.20
    pocket_height = 0.078
    pocket_depth = 0.024
    backplate_t = 0.003
    pocket_wall_t = 0.008
    guide_t = 0.002

    hood = model.part("hood_body")

    hood.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, top_z - shell_t * 0.5)),
        material=hood_white,
        name="top_panel",
    )
    hood.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(left_x + shell_t * 0.5, 0.0, 0.0)),
        material=hood_white,
        name="left_side",
    )
    hood.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(right_x - shell_t * 0.5, 0.0, 0.0)),
        material=hood_white,
        name="right_side",
    )
    hood.visual(
        Box((width - 2.0 * shell_t, shell_t, height)),
        origin=Origin(xyz=(0.0, rear_y - shell_t * 0.5, 0.0)),
        material=hood_white,
        name="rear_panel",
    )

    front_side_w = (width - pocket_width) * 0.5
    front_strip_h = (height - pocket_height) * 0.5
    hood.visual(
        Box((front_side_w, shell_t, height)),
        origin=Origin(xyz=(-(pocket_width * 0.5) - front_side_w * 0.5, front_y + shell_t * 0.5, 0.0)),
        material=hood_white,
        name="front_left_band",
    )
    hood.visual(
        Box((front_side_w, shell_t, height)),
        origin=Origin(xyz=((pocket_width * 0.5) + front_side_w * 0.5, front_y + shell_t * 0.5, 0.0)),
        material=hood_white,
        name="front_right_band",
    )
    hood.visual(
        Box((pocket_width, shell_t, front_strip_h)),
        origin=Origin(xyz=(0.0, front_y + shell_t * 0.5, pocket_height * 0.5 + front_strip_h * 0.5)),
        material=hood_white,
        name="front_top_band",
    )
    hood.visual(
        Box((pocket_width, shell_t, front_strip_h)),
        origin=Origin(xyz=(0.0, front_y + shell_t * 0.5, -(pocket_height * 0.5 + front_strip_h * 0.5))),
        material=hood_white,
        name="front_bottom_band",
    )

    cavity_depth = pocket_depth - shell_t
    cavity_y = front_y + shell_t + cavity_depth * 0.5
    hood.visual(
        Box((pocket_wall_t, cavity_depth, pocket_height)),
        origin=Origin(xyz=(-(pocket_width * 0.5) + pocket_wall_t * 0.5, cavity_y, 0.0)),
        material=control_dark,
        name="pocket_left_wall",
    )
    hood.visual(
        Box((pocket_wall_t, cavity_depth, pocket_height)),
        origin=Origin(xyz=((pocket_width * 0.5) - pocket_wall_t * 0.5, cavity_y, 0.0)),
        material=control_dark,
        name="pocket_right_wall",
    )
    hood.visual(
        Box((pocket_width, cavity_depth, pocket_wall_t)),
        origin=Origin(xyz=(0.0, cavity_y, pocket_height * 0.5 - pocket_wall_t * 0.5)),
        material=control_dark,
        name="pocket_top_wall",
    )
    hood.visual(
        Box((pocket_width, cavity_depth, pocket_wall_t)),
        origin=Origin(xyz=(0.0, cavity_y, -(pocket_height * 0.5 - pocket_wall_t * 0.5))),
        material=control_dark,
        name="pocket_bottom_wall",
    )
    hood.visual(
        Box((pocket_width, backplate_t, pocket_height)),
        origin=Origin(
            xyz=(
                0.0,
                front_y + pocket_depth + backplate_t * 0.5,
                0.0,
            )
        ),
        material=control_dark,
        name="control_backplate",
    )

    inner_width = width - 2.0 * shell_t
    inner_depth = depth - 2.0 * shell_t
    front_rail_d = 0.060
    rear_rail_d = 0.050
    center_divider_w = 0.016
    filter_t = 0.006
    filter_depth = inner_depth - front_rail_d - rear_rail_d
    filter_width = (inner_width - center_divider_w) * 0.5
    filter_center_y = (-inner_depth * 0.5 + front_rail_d) + filter_depth * 0.5
    filter_center_z = bottom_z + filter_t * 0.5

    hood.visual(
        Box((inner_width, front_rail_d, shell_t)),
        origin=Origin(
            xyz=(
                0.0,
                -inner_depth * 0.5 + front_rail_d * 0.5,
                bottom_z + shell_t * 0.5,
            )
        ),
        material=hood_white,
        name="front_underside_rail",
    )
    hood.visual(
        Box((inner_width, rear_rail_d, shell_t)),
        origin=Origin(
            xyz=(
                0.0,
                inner_depth * 0.5 - rear_rail_d * 0.5,
                bottom_z + shell_t * 0.5,
            )
        ),
        material=hood_white,
        name="rear_underside_rail",
    )
    hood.visual(
        Box((center_divider_w, filter_depth, shell_t)),
        origin=Origin(xyz=(0.0, filter_center_y, bottom_z + shell_t * 0.5)),
        material=hood_white,
        name="center_filter_divider",
    )
    hood.visual(
        Box((filter_width, filter_depth, filter_t)),
        origin=Origin(
            xyz=(
                -center_divider_w * 0.5 - filter_width * 0.5,
                filter_center_y,
                filter_center_z,
            )
        ),
        material=filter_aluminum,
        name="left_filter",
    )
    hood.visual(
        Box((filter_width, filter_depth, filter_t)),
        origin=Origin(
            xyz=(
                center_divider_w * 0.5 + filter_width * 0.5,
                filter_center_y,
                filter_center_z,
            )
        ),
        material=filter_aluminum,
        name="right_filter",
    )
    hood.visual(
        Box((0.12, 0.045, 0.004)),
        origin=Origin(xyz=(0.0, -0.12, bottom_z + 0.008)),
        material=lens_white,
        name="light_lens",
    )
    hood.inertial = Inertial.from_geometry(Box((width, depth, height)), mass=8.5)

    knob_origin = Origin(xyz=(0.0, front_y, -0.004))
    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_black,
        name="knob_face",
    )
    knob.visual(
        Cylinder(radius=0.006, length=pocket_depth),
        origin=Origin(
            xyz=(0.0, pocket_depth * 0.5, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=control_black,
        name="knob_shaft",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.014),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
    )
    model.articulation(
        "hood_to_knob",
        ArticulationType.CONTINUOUS,
        parent=hood,
        child=knob,
        origin=knob_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    button_w = 0.018
    button_h = 0.012
    button_depth = 0.018
    button_travel = 0.006
    guide_depth = pocket_depth
    guide_center_y = front_y + guide_depth * 0.5

    button_specs = (
        ("button_top_left", -0.026, 0.024),
        ("button_top_right", 0.026, 0.024),
        ("button_left_upper", -0.061, 0.010),
        ("button_left_lower", -0.061, -0.022),
        ("button_right_upper", 0.061, 0.010),
        ("button_right_lower", 0.061, -0.022),
    )

    for button_name, x_pos, z_pos in button_specs:
        hood.visual(
            Box((guide_t, guide_depth, button_h)),
            origin=Origin(xyz=(x_pos - button_w * 0.5 - guide_t * 0.5, guide_center_y, z_pos)),
            material=control_dark,
            name=f"{button_name}_guide_left",
        )
        hood.visual(
            Box((guide_t, guide_depth, button_h)),
            origin=Origin(xyz=(x_pos + button_w * 0.5 + guide_t * 0.5, guide_center_y, z_pos)),
            material=control_dark,
            name=f"{button_name}_guide_right",
        )
        hood.visual(
            Box((button_w, guide_depth, guide_t)),
            origin=Origin(xyz=(x_pos, guide_center_y, z_pos + button_h * 0.5 + guide_t * 0.5)),
            material=control_dark,
            name=f"{button_name}_guide_top",
        )
        hood.visual(
            Box((button_w, guide_depth, guide_t)),
            origin=Origin(xyz=(x_pos, guide_center_y, z_pos - button_h * 0.5 - guide_t * 0.5)),
            material=control_dark,
            name=f"{button_name}_guide_bottom",
        )

        button = model.part(button_name)
        button.visual(
            Box((button_w, button_depth, button_h)),
            origin=Origin(xyz=(0.0, button_depth * 0.5, 0.0)),
            material=control_black,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_w, button_depth, button_h)),
            mass=0.02,
            origin=Origin(xyz=(0.0, button_depth * 0.5, 0.0)),
        )
        model.articulation(
            f"hood_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=hood,
            child=button,
            origin=Origin(xyz=(x_pos, front_y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood = object_model.get_part("hood_body")
    control_backplate = hood.get_visual("control_backplate")
    knob = object_model.get_part("knob")
    knob_joint = object_model.get_articulation("hood_to_knob")

    button_names = (
        "button_top_left",
        "button_top_right",
        "button_left_upper",
        "button_left_lower",
        "button_right_upper",
        "button_right_lower",
    )
    buttons = {name: object_model.get_part(name) for name in button_names}
    button_joints = {
        name: object_model.get_articulation(f"hood_to_{name}")
        for name in button_names
    }

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    hood_aabb = ctx.part_world_aabb(hood)
    ctx.check("hood_aabb_present", hood_aabb is not None, "hood body needs measurable geometry")
    if hood_aabb is not None:
        hood_size = (
            hood_aabb[1][0] - hood_aabb[0][0],
            hood_aabb[1][1] - hood_aabb[0][1],
            hood_aabb[1][2] - hood_aabb[0][2],
        )
        ctx.check(
            "hood_realistic_width",
            0.74 <= hood_size[0] <= 0.78,
            f"expected ~0.76 m width, got {hood_size[0]:.4f}",
        )
        ctx.check(
            "hood_realistic_depth",
            0.48 <= hood_size[1] <= 0.52,
            f"expected ~0.50 m depth, got {hood_size[1]:.4f}",
        )
        ctx.check(
            "hood_realistic_height",
            0.10 <= hood_size[2] <= 0.14,
            f"expected shallow ~0.12 m height, got {hood_size[2]:.4f}",
        )

    knob_limits = knob_joint.motion_limits
    ctx.check(
        "knob_continuous_joint_type",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"knob joint type was {knob_joint.articulation_type}",
    )
    ctx.check(
        "knob_axis_faces_front",
        tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        f"knob axis was {knob_joint.axis}",
    )
    ctx.check(
        "knob_continuous_unbounded",
        knob_limits is not None and knob_limits.lower is None and knob_limits.upper is None,
        "continuous knob should not have lower/upper bounds",
    )
    ctx.expect_contact(knob, hood, elem_b=control_backplate, name="knob_mount_contact")
    ctx.expect_within(knob, hood, axes="xz", outer_elem=control_backplate, margin=0.002, name="knob_in_control_pocket")
    with ctx.pose({knob_joint: math.pi * 1.25}):
        ctx.expect_contact(knob, hood, elem_b=control_backplate, name="knob_rotated_mount_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_rotated_no_floating")

    rest_positions = {name: ctx.part_world_position(part) for name, part in buttons.items()}
    for name, part in buttons.items():
        joint = button_joints[name]
        limits = joint.motion_limits
        ctx.check(
            f"{name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"{name} joint type was {joint.articulation_type}",
        )
        ctx.check(
            f"{name}_axis_is_inward_y",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{name} axis was {joint.axis}",
        )
        ctx.check(
            f"{name}_travel_is_short",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) < 1e-9
            and 0.004 <= limits.upper <= 0.008,
            f"{name} travel limits were {limits}",
        )
        ctx.expect_contact(part, hood, name=f"{name}_rest_contact")
        ctx.expect_within(
            part,
            hood,
            axes="xz",
            outer_elem=control_backplate,
            margin=0.004,
            name=f"{name}_inside_control_cluster",
        )
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_position = ctx.part_world_position(part)
                ctx.check(
                    f"{name}_moves_inward",
                    rest_positions[name] is not None
                    and pressed_position is not None
                    and pressed_position[1] > rest_positions[name][1] + 0.005,
                    f"{name} should move inward along +y",
                )
                ctx.expect_contact(part, hood, name=f"{name}_pressed_contact")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_pressed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_pressed_no_floating")

    top_left = rest_positions["button_top_left"]
    top_right = rest_positions["button_top_right"]
    left_upper = rest_positions["button_left_upper"]
    left_lower = rest_positions["button_left_lower"]
    right_upper = rest_positions["button_right_upper"]
    right_lower = rest_positions["button_right_lower"]
    knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "control_positions_available",
        all(pos is not None for pos in (top_left, top_right, left_upper, left_lower, right_upper, right_lower, knob_pos)),
        "control parts need measurable world positions",
    )
    if all(pos is not None for pos in (top_left, top_right, left_upper, left_lower, right_upper, right_lower, knob_pos)):
        assert top_left is not None
        assert top_right is not None
        assert left_upper is not None
        assert left_lower is not None
        assert right_upper is not None
        assert right_lower is not None
        assert knob_pos is not None
        ctx.check(
            "top_buttons_above_knob",
            top_left[2] > knob_pos[2] + 0.02 and top_right[2] > knob_pos[2] + 0.02,
            "two buttons should sit above the knob",
        )
        ctx.check(
            "left_buttons_left_of_knob",
            left_upper[0] < knob_pos[0] - 0.04 and left_lower[0] < knob_pos[0] - 0.04,
            "left pair should sit to the left of the knob",
        )
        ctx.check(
            "right_buttons_right_of_knob",
            right_upper[0] > knob_pos[0] + 0.04 and right_lower[0] > knob_pos[0] + 0.04,
            "right pair should sit to the right of the knob",
        )
        ctx.check(
            "side_buttons_stack_vertically",
            left_upper[2] > left_lower[2] + 0.02 and right_upper[2] > right_lower[2] + 0.02,
            "each side should have two vertically stacked buttons",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
