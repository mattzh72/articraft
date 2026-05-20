from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="passenger_car_glove_compartment")

    # Overall proportions for a typical passenger-car glove compartment.
    opening_w = 0.36
    opening_h = 0.17
    bin_depth = 0.24
    wall = 0.006
    frame_border = 0.034
    fascia_depth = 0.028
    door_t = 0.018
    door_gap = 0.0015
    shell_front_offset = 0.012

    door_w = opening_w - 2.0 * door_gap
    door_h = opening_h - 2.0 * door_gap

    button_w = 0.038
    button_h = 0.016
    aperture_w = 0.046
    aperture_h = 0.021
    button_z = opening_h * 0.56

    arm_y = opening_w / 2.0 - 0.020
    arm_pivot_x = 0.0435
    arm_pivot_z = 0.030
    arm_tip_vec = (-0.026, 0.0, 0.041)
    arm_tip_len = math.sqrt(arm_tip_vec[0] ** 2 + arm_tip_vec[2] ** 2)
    arm_bar_angle = math.atan2(-arm_tip_vec[2], arm_tip_vec[0])
    arm_tip_unit = (
        arm_tip_vec[0] / arm_tip_len,
        0.0,
        arm_tip_vec[2] / arm_tip_len,
    )

    dash_mat = model.material("dash_trim", color=(0.19, 0.20, 0.22, 1.0))
    bin_mat = model.material("bin_shell", color=(0.11, 0.12, 0.13, 1.0))
    door_mat = model.material("door_panel", color=(0.22, 0.23, 0.25, 1.0))
    inner_mat = model.material("inner_liner", color=(0.12, 0.13, 0.14, 1.0))
    hinge_mat = model.material("hinge_hardware", color=(0.15, 0.15, 0.16, 1.0))
    button_trim = model.material("button_trim", color=(0.70, 0.72, 0.75, 1.0))
    button_face = model.material("button_face", color=(0.14, 0.14, 0.15, 1.0))

    def add_y_cylinder(part, *, radius, length, xyz, material, name):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    bin_part = model.part("dashboard_bin")
    bin_part.inertial = Inertial.from_geometry(
        Box((bin_depth, opening_w + 2.0 * frame_border, opening_h + frame_border)),
        mass=1.9,
        origin=Origin(xyz=(bin_depth / 2.0, 0.0, opening_h / 2.0)),
    )

    # Surrounding dashboard bezel framing the glovebox opening.
    bin_part.visual(
        Box((fascia_depth, opening_w + 2.0 * frame_border, frame_border)),
        origin=Origin(xyz=(0.006, 0.0, opening_h + frame_border / 2.0)),
        material=dash_mat,
        name="fascia_top",
    )
    bin_part.visual(
        Box((fascia_depth, opening_w + 2.0 * frame_border, 0.022)),
        origin=Origin(xyz=(0.006, 0.0, -0.018)),
        material=dash_mat,
        name="fascia_bottom",
    )
    bin_part.visual(
        Box((fascia_depth, frame_border, opening_h + frame_border)),
        origin=Origin(
            xyz=(0.006, opening_w / 2.0 + frame_border / 2.0, opening_h / 2.0)
        ),
        material=dash_mat,
        name="fascia_right",
    )
    bin_part.visual(
        Box((fascia_depth, frame_border, opening_h + frame_border)),
        origin=Origin(
            xyz=(0.006, -opening_w / 2.0 - frame_border / 2.0, opening_h / 2.0)
        ),
        material=dash_mat,
        name="fascia_left",
    )

    # Fixed storage bin shell recessed into the dashboard.
    shell_len = bin_depth - shell_front_offset
    shell_center_x = shell_front_offset + shell_len / 2.0
    bin_part.visual(
        Box((shell_len, wall, opening_h + wall)),
        origin=Origin(xyz=(shell_center_x, opening_w / 2.0 - wall / 2.0, opening_h / 2.0)),
        material=bin_mat,
        name="bin_right_wall",
    )
    bin_part.visual(
        Box((shell_len, wall, opening_h + wall)),
        origin=Origin(xyz=(shell_center_x, -opening_w / 2.0 + wall / 2.0, opening_h / 2.0)),
        material=bin_mat,
        name="bin_left_wall",
    )
    bin_part.visual(
        Box((shell_len, opening_w - 2.0 * wall + 0.004, wall)),
        origin=Origin(xyz=(shell_front_offset + (bin_depth - shell_front_offset) / 2.0, 0.0, -wall / 2.0)),
        material=bin_mat,
        name="bin_floor",
    )
    bin_part.visual(
        Box((shell_len, opening_w - 2.0 * wall + 0.004, wall)),
        origin=Origin(
            xyz=(shell_front_offset + (bin_depth - shell_front_offset) / 2.0, 0.0, opening_h - wall / 2.0)
        ),
        material=bin_mat,
        name="bin_roof",
    )
    bin_part.visual(
        Box((wall, opening_w - 2.0 * wall + 0.004, opening_h)),
        origin=Origin(xyz=(bin_depth - wall / 2.0, 0.0, opening_h / 2.0)),
        material=bin_mat,
        name="bin_back_wall",
    )

    # Hinge support along the lower sill.
    add_y_cylinder(
        bin_part,
        radius=0.007,
        length=0.048,
        xyz=(0.000, -0.130, 0.000),
        material=hinge_mat,
        name="bin_knuckle_left",
    )
    add_y_cylinder(
        bin_part,
        radius=0.007,
        length=0.048,
        xyz=(0.000, 0.000, 0.000),
        material=hinge_mat,
        name="bin_knuckle_center",
    )
    add_y_cylinder(
        bin_part,
        radius=0.007,
        length=0.048,
        xyz=(0.000, 0.130, 0.000),
        material=hinge_mat,
        name="bin_knuckle_right",
    )

    # Stay-arm supports on both inner side walls.
    bin_part.visual(
        Box((0.020, 0.012, 0.028)),
        origin=Origin(xyz=(arm_pivot_x, arm_y + 0.011, arm_pivot_z)),
        material=hinge_mat,
        name="right_arm_support",
    )
    bin_part.visual(
        Box((0.020, 0.012, 0.028)),
        origin=Origin(xyz=(arm_pivot_x, -arm_y - 0.011, arm_pivot_z)),
        material=hinge_mat,
        name="left_arm_support",
    )

    # Latch receiver tucked under the upper inner lip.
    bin_part.visual(
        Box((0.010, 0.052, 0.020)),
        origin=Origin(xyz=(0.033, 0.0, opening_h - 0.016)),
        material=hinge_mat,
        name="latch_receiver",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((door_t, door_w, door_h)),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.0, opening_h / 2.0)),
    )

    aperture_z_min = button_z - aperture_h / 2.0
    aperture_z_max = button_z + aperture_h / 2.0
    overlap = 0.001
    side_strip_w = (door_w - aperture_w) / 2.0 + overlap
    lower_skin_z0 = 0.010

    # Front door skin split around the button opening so the button can slide
    # without interpenetrating the panel.
    door.visual(
        Box((door_t, door_w, door_h - aperture_z_max + overlap)),
        origin=Origin(
            xyz=(0.0, 0.0, aperture_z_max + (door_h - aperture_z_max + overlap) / 2.0)
        ),
        material=door_mat,
        name="door_outer_upper",
    )
    door.visual(
        Box((door_t, door_w, aperture_z_min - lower_skin_z0 + overlap)),
        origin=Origin(
            xyz=(0.0, 0.0, lower_skin_z0 + (aperture_z_min - lower_skin_z0 + overlap) / 2.0)
        ),
        material=door_mat,
        name="door_outer_lower",
    )
    door.visual(
        Box((door_t, side_strip_w, aperture_h + 2.0 * overlap)),
        origin=Origin(
            xyz=(
                0.0,
                -(aperture_w / 2.0 + side_strip_w / 2.0 - overlap / 2.0),
                button_z,
            )
        ),
        material=door_mat,
        name="door_outer_left",
    )
    door.visual(
        Box((door_t, side_strip_w, aperture_h + 2.0 * overlap)),
        origin=Origin(
            xyz=(
                0.0,
                aperture_w / 2.0 + side_strip_w / 2.0 - overlap / 2.0,
                button_z,
            )
        ),
        material=door_mat,
        name="door_outer_right",
    )

    # Interior liner and hinge leaf on the inside of the door.
    door.visual(
        Box((0.010, door_w - 0.042, 0.052)),
        origin=Origin(xyz=(0.013, 0.0, 0.038)),
        material=inner_mat,
        name="door_inner_lower",
    )
    door.visual(
        Box((0.010, door_w - 0.042, 0.043)),
        origin=Origin(xyz=(0.013, 0.0, 0.139)),
        material=inner_mat,
        name="door_inner_upper",
    )
    door.visual(
        Box((0.008, 0.074, 0.022)),
        origin=Origin(xyz=(0.006, -0.066, 0.011)),
        material=hinge_mat,
        name="door_hinge_leaf_left",
    )
    door.visual(
        Box((0.008, 0.074, 0.022)),
        origin=Origin(xyz=(0.006, 0.066, 0.011)),
        material=hinge_mat,
        name="door_hinge_leaf_right",
    )

    add_y_cylinder(
        door,
        radius=0.007,
        length=0.066,
        xyz=(0.000, -0.066, 0.000),
        material=hinge_mat,
        name="door_knuckle_left",
    )
    add_y_cylinder(
        door,
        radius=0.007,
        length=0.066,
        xyz=(0.000, 0.066, 0.000),
        material=hinge_mat,
        name="door_knuckle_right",
    )

    # Fixed latch housing around the center button.
    door.visual(
        Box((0.004, button_w + 0.016, 0.004)),
        origin=Origin(xyz=(-0.005, 0.0, button_z + 0.013)),
        material=button_trim,
        name="latch_bezel_top",
    )
    door.visual(
        Box((0.004, button_w + 0.016, 0.004)),
        origin=Origin(xyz=(-0.005, 0.0, button_z - 0.013)),
        material=button_trim,
        name="latch_bezel_bottom",
    )
    door.visual(
        Box((0.004, 0.004, button_h + 0.010)),
        origin=Origin(xyz=(-0.005, -(button_w / 2.0 + 0.006), button_z)),
        material=button_trim,
        name="latch_bezel_left",
    )
    door.visual(
        Box((0.004, 0.004, button_h + 0.010)),
        origin=Origin(xyz=(-0.005, button_w / 2.0 + 0.006, button_z)),
        material=button_trim,
        name="latch_bezel_right",
    )
    door.visual(
        Box((0.016, 0.006, button_h + 0.010)),
        origin=Origin(xyz=(0.010, -(button_w / 2.0 - 0.003), button_z)),
        material=hinge_mat,
        name="button_guide_left",
    )
    door.visual(
        Box((0.016, 0.006, button_h + 0.010)),
        origin=Origin(xyz=(0.010, button_w / 2.0 - 0.003, button_z)),
        material=hinge_mat,
        name="button_guide_right",
    )
    door.visual(
        Box((0.016, button_w + 0.008, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, button_z + 0.014)),
        material=hinge_mat,
        name="button_guide_top",
    )
    door.visual(
        Box((0.016, button_w + 0.008, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, button_z - 0.014)),
        material=hinge_mat,
        name="button_guide_bottom",
    )

    # Door-side brackets for the stay arms.
    door.visual(
        Box((0.014, 0.016, 0.014)),
        origin=Origin(xyz=(0.014, -arm_y, 0.028)),
        material=hinge_mat,
        name="left_stay_bracket",
    )
    door.visual(
        Box((0.014, 0.016, 0.014)),
        origin=Origin(xyz=(0.014, arm_y, 0.028)),
        material=hinge_mat,
        name="right_stay_bracket",
    )

    # Top latch tongue that reaches toward the receiver when the door is shut.
    door.visual(
        Box((0.014, 0.036, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, opening_h - 0.018)),
        material=hinge_mat,
        name="latch_tongue",
    )

    push_button = model.part("push_button")
    push_button.inertial = Inertial.from_geometry(
        Box((0.012, button_w, button_h)),
        mass=0.05,
        origin=Origin(),
    )
    push_button.visual(
        Box((0.010, button_w, button_h)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=button_face,
        name="button_cap",
    )
    push_button.visual(
        Box((0.008, 0.026, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=button_trim,
        name="button_stem",
    )

    def build_stay_arm(part_name: str) -> None:
        arm = model.part(part_name)
        arm.inertial = Inertial.from_geometry(
            Box((0.050, 0.010, 0.018)),
            mass=0.08,
            origin=Origin(xyz=(-0.020, 0.0, 0.020)),
        )
        add_y_cylinder(
            arm,
            radius=0.006,
            length=0.012,
            xyz=(0.0, 0.0, 0.0),
            material=hinge_mat,
            name="pivot_eye",
        )
        arm.visual(
            Box((arm_tip_len, 0.010, 0.006)),
            origin=Origin(
                xyz=(arm_tip_vec[0] / 2.0, 0.0, arm_tip_vec[2] / 2.0),
                rpy=(0.0, arm_bar_angle, 0.0),
            ),
            material=hinge_mat,
            name="arm_bar",
        )
        tip_center = (
            arm_tip_vec[0] - arm_tip_unit[0] * 0.006,
            0.0,
            arm_tip_vec[2] - arm_tip_unit[2] * 0.006,
        )
        arm.visual(
            Box((0.014, 0.010, 0.010)),
            origin=Origin(xyz=tip_center, rpy=(0.0, arm_bar_angle, 0.0)),
            material=hinge_mat,
            name="arm_tip",
        )

    build_stay_arm("left_stay_arm")
    build_stay_arm("right_stay_arm")

    model.articulation(
        "bin_to_door",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "door_to_push_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=push_button,
        origin=Origin(xyz=(0.0, 0.0, button_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.10,
            lower=0.0,
            upper=0.008,
        ),
    )
    model.articulation(
        "bin_to_left_stay_arm",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=model.get_part("left_stay_arm"),
        origin=Origin(xyz=(arm_pivot_x, -arm_y, arm_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.12,
        ),
    )
    model.articulation(
        "bin_to_right_stay_arm",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=model.get_part("right_stay_arm"),
        origin=Origin(xyz=(arm_pivot_x, arm_y, arm_pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.12,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    bin_part = object_model.get_part("dashboard_bin")
    door = object_model.get_part("door")
    push_button = object_model.get_part("push_button")
    left_arm = object_model.get_part("left_stay_arm")
    right_arm = object_model.get_part("right_stay_arm")

    door_hinge = object_model.get_articulation("bin_to_door")
    button_slide = object_model.get_articulation("door_to_push_button")
    left_arm_joint = object_model.get_articulation("bin_to_left_stay_arm")
    right_arm_joint = object_model.get_articulation("bin_to_right_stay_arm")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) / 2.0,
            (min_y + max_y) / 2.0,
            (min_z + max_z) / 2.0,
        )

    # Closed door stays seated in the dashboard opening without clipping through
    # the back of the latch receiver.
    with ctx.pose(
        {
            door_hinge: 0.0,
            button_slide: 0.0,
            left_arm_joint: 0.0,
            right_arm_joint: 0.0,
        }
    ):
        ctx.expect_gap(
            bin_part,
            door,
            axis="x",
            positive_elem="latch_receiver",
            negative_elem="latch_tongue",
            min_gap=0.002,
            max_gap=0.020,
            name="closed latch tongue sits just ahead of the receiver",
        )

        left_tip_closed = aabb_center(ctx.part_element_world_aabb(left_arm, elem="arm_tip"))
        right_tip_closed = aabb_center(ctx.part_element_world_aabb(right_arm, elem="arm_tip"))
        ctx.check(
            "stay arms start mirrored in the closed pose",
            left_tip_closed is not None
            and right_tip_closed is not None
            and abs(left_tip_closed[0] - right_tip_closed[0]) < 0.001
            and abs(left_tip_closed[2] - right_tip_closed[2]) < 0.001
            and abs(left_tip_closed[1] + right_tip_closed[1]) < 0.002,
            details=f"left={left_tip_closed}, right={right_tip_closed}",
        )

    # Positive hinge motion should swing the door outward into the cabin and
    # downward away from the opening.
    closed_panel = ctx.part_element_world_aabb(door, elem="door_outer_upper")
    closed_panel_center = aabb_center(closed_panel)
    with ctx.pose({door_hinge: 1.10}):
        open_panel = ctx.part_element_world_aabb(door, elem="door_outer_upper")
    open_panel_center = aabb_center(open_panel)
    ctx.check(
        "door opens downward on the lower hinge",
        closed_panel_center is not None
        and open_panel_center is not None
        and open_panel_center[0] < closed_panel_center[0] - 0.050
        and open_panel_center[2] < closed_panel_center[2] - 0.040,
        details=f"closed={closed_panel_center}, open={open_panel_center}",
    )

    # The push button should slide inward into the latch housing.
    with ctx.pose({door_hinge: 0.0, button_slide: 0.0}):
        button_rest = aabb_center(ctx.part_element_world_aabb(push_button, elem="button_cap"))
    with ctx.pose({door_hinge: 0.0, button_slide: 0.008}):
        button_pressed = aabb_center(ctx.part_element_world_aabb(push_button, elem="button_cap"))
    ctx.check(
        "push button slides inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0] > button_rest[0] + 0.006,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    # In the deployed pose the side stays rotate down and remain close to the
    # door-side brackets they visually support.
    deployed_pose = {
        door_hinge: 1.10,
        left_arm_joint: 1.10,
        right_arm_joint: 1.10,
        button_slide: 0.0,
    }
    with ctx.pose(deployed_pose):
        left_tip_open = aabb_center(ctx.part_element_world_aabb(left_arm, elem="arm_tip"))
        right_tip_open = aabb_center(ctx.part_element_world_aabb(right_arm, elem="arm_tip"))
        left_bracket_open = aabb_center(ctx.part_element_world_aabb(door, elem="left_stay_bracket"))
        right_bracket_open = aabb_center(ctx.part_element_world_aabb(door, elem="right_stay_bracket"))

    def center_dist(a, b):
        if a is None or b is None:
            return None
        return math.sqrt(
            (a[0] - b[0]) ** 2 +
            (a[1] - b[1]) ** 2 +
            (a[2] - b[2]) ** 2
        )

    left_open_dist = center_dist(left_tip_open, left_bracket_open)
    right_open_dist = center_dist(right_tip_open, right_bracket_open)
    ctx.check(
        "left stay arm deploys toward the door bracket",
        left_open_dist is not None and left_open_dist < 0.020,
        details=f"left_tip={left_tip_open}, left_bracket={left_bracket_open}, dist={left_open_dist}",
    )
    ctx.check(
        "right stay arm deploys toward the door bracket",
        right_open_dist is not None and right_open_dist < 0.020,
        details=f"right_tip={right_tip_open}, right_bracket={right_bracket_open}, dist={right_open_dist}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
