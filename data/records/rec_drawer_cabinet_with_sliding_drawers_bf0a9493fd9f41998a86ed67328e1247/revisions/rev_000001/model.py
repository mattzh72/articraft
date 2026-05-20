from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="rolling_tool_chest")

    body_red = model.material("body_red", rgba=(0.74, 0.08, 0.08, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.69, 1.0))
    zinc = model.material("zinc", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    top_mat = model.material("top_mat", rgba=(0.13, 0.14, 0.14, 1.0))

    overall_width = 1.08
    overall_depth = 0.56
    body_height = 0.78
    body_base_z = 0.115
    side_t = 0.024
    back_t = 0.018
    front_frame_t = 0.024
    top_t = 0.024
    bottom_t = 0.024
    jamb_w = 0.012
    opening_width = overall_width - 2.0 * (side_t + jamb_w)
    bottom_rail_h = 0.060
    top_rail_h = 0.050
    drawer_face_h = 0.126
    drawer_gap = 0.008
    slide_h = 0.028
    slide_body_t = 0.012
    slide_carriage_t = 0.010
    slide_drawer_t = 0.010
    body_slide_len = 0.440
    moving_slide_len = 0.300
    slide_stage_travel = 0.195
    drawer_origin_y = -(overall_depth * 0.5) + 0.010
    tray_w = opening_width - 0.040
    tray_depth = 0.460
    tray_wall_t = 0.004
    tray_h = 0.096
    tray_y_center = 0.010 + tray_depth * 0.5
    tray_side_z = -0.013
    tray_bottom_z = -0.059
    slide_local_y = 0.170
    slide_local_z = -0.015
    body_slide_y = -0.030
    body_slide_x = overall_width * 0.5 - side_t - slide_body_t * 0.5
    carriage_slide_x = overall_width * 0.5 - side_t - slide_body_t - slide_carriage_t * 0.5
    drawer_slide_x = overall_width * 0.5 - side_t - slide_body_t - slide_carriage_t - slide_drawer_t * 0.5
    handle_bar_w = 0.440

    wheel_radius = 0.050
    wheel_width = 0.026
    hub_width = 0.030
    caster_plate = 0.060
    fork_inside = hub_width
    fork_arm_t = 0.004
    fork_arm_y = 0.012
    wheel_center_drop = body_base_z - wheel_radius
    caster_trail = 0.075

    body = model.part("body")
    body.visual(
        Box((side_t, overall_depth, body_height)),
        origin=Origin(xyz=(-(overall_width - side_t) * 0.5, 0.0, body_base_z + body_height * 0.5)),
        material=body_red,
        name="left_side",
    )
    body.visual(
        Box((side_t, overall_depth, body_height)),
        origin=Origin(xyz=((overall_width - side_t) * 0.5, 0.0, body_base_z + body_height * 0.5)),
        material=body_red,
        name="right_side",
    )
    body.visual(
        Box((overall_width - 2.0 * side_t, overall_depth - front_frame_t - back_t, bottom_t)),
        origin=Origin(
            xyz=(
                0.0,
                ((-overall_depth * 0.5 + front_frame_t) + (overall_depth * 0.5 - back_t)) * 0.5,
                body_base_z + bottom_t * 0.5,
            )
        ),
        material=body_red,
        name="bottom_panel",
    )
    body.visual(
        Box((overall_width, overall_depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, body_base_z + body_height - top_t * 0.5)),
        material=body_red,
        name="top_panel",
    )
    body.visual(
        Box((overall_width - 2.0 * side_t, back_t, body_height)),
        origin=Origin(xyz=(0.0, (overall_depth - back_t) * 0.5, body_base_z + body_height * 0.5)),
        material=body_red,
        name="back_panel",
    )
    body.visual(
        Box((jamb_w, front_frame_t, body_height - top_rail_h - bottom_rail_h)),
        origin=Origin(
            xyz=(
                -(overall_width * 0.5 - side_t - jamb_w * 0.5),
                -(overall_depth - front_frame_t) * 0.5,
                body_base_z + bottom_rail_h + (body_height - top_rail_h - bottom_rail_h) * 0.5,
            )
        ),
        material=body_red,
        name="left_jamb",
    )
    body.visual(
        Box((jamb_w, front_frame_t, body_height - top_rail_h - bottom_rail_h)),
        origin=Origin(
            xyz=(
                overall_width * 0.5 - side_t - jamb_w * 0.5,
                -(overall_depth - front_frame_t) * 0.5,
                body_base_z + bottom_rail_h + (body_height - top_rail_h - bottom_rail_h) * 0.5,
            )
        ),
        material=body_red,
        name="right_jamb",
    )
    body.visual(
        Box((opening_width, front_frame_t, bottom_rail_h)),
        origin=Origin(
            xyz=(
                0.0,
                -(overall_depth - front_frame_t) * 0.5,
                body_base_z + bottom_rail_h * 0.5,
            )
        ),
        material=body_red,
        name="bottom_rail",
    )
    body.visual(
        Box((opening_width, front_frame_t, top_rail_h)),
        origin=Origin(
            xyz=(
                0.0,
                -(overall_depth - front_frame_t) * 0.5,
                body_base_z + body_height - top_rail_h * 0.5,
            )
        ),
        material=body_red,
        name="top_rail",
    )
    body.visual(
        Box((overall_width - 0.080, overall_depth - 0.090, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, body_base_z + body_height + 0.002)),
        material=top_mat,
        name="top_mat",
    )

    body.inertial = Inertial.from_geometry(
        Box((overall_width, overall_depth, body_height)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, body_base_z + body_height * 0.5)),
    )

    drawer_carriages = []
    drawers = []
    drawer_carriage_joints = []
    drawer_joints = []
    for drawer_index in range(5):
        drawer_number = drawer_index + 1
        face_center_z = (
            body_base_z
            + bottom_rail_h
            + drawer_face_h * 0.5
            + drawer_index * (drawer_face_h + drawer_gap)
        )
        body.visual(
            Box((slide_body_t, body_slide_len, slide_h)),
            origin=Origin(xyz=(-body_slide_x, body_slide_y, face_center_z + slide_local_z)),
            material=zinc,
            name=f"left_body_slide_{drawer_number}",
        )
        body.visual(
            Box((slide_body_t, body_slide_len, slide_h)),
            origin=Origin(xyz=(body_slide_x, body_slide_y, face_center_z + slide_local_z)),
            material=zinc,
            name=f"right_body_slide_{drawer_number}",
        )

        carriage = model.part(f"drawer_carriage_{drawer_number}")
        carriage.visual(
            Box((slide_carriage_t, moving_slide_len, slide_h)),
            origin=Origin(xyz=(-carriage_slide_x, slide_local_y, slide_local_z)),
            material=zinc,
            name="left_carriage_slide",
        )
        carriage.visual(
            Box((slide_carriage_t, moving_slide_len, slide_h)),
            origin=Origin(xyz=(carriage_slide_x, slide_local_y, slide_local_z)),
            material=zinc,
            name="right_carriage_slide",
        )
        carriage.visual(
            Box((2.0 * carriage_slide_x, 0.024, 0.006)),
            origin=Origin(xyz=(0.0, 0.332, 0.002)),
            material=dark_steel,
            name="rear_bridge",
        )
        carriage.inertial = Inertial.from_geometry(
            Box((2.0 * carriage_slide_x + slide_carriage_t, moving_slide_len, slide_h)),
            mass=0.9,
            origin=Origin(xyz=(0.0, slide_local_y, slide_local_z)),
        )

        drawer = model.part(f"drawer_{drawer_number}")
        drawer.visual(
            Box((opening_width - 0.016, 0.020, drawer_face_h)),
            origin=Origin(),
            material=body_red,
            name="front_panel",
        )
        drawer.visual(
            Box((tray_w - 2.0 * tray_wall_t, tray_depth - 2.0 * tray_wall_t, tray_wall_t)),
            origin=Origin(xyz=(0.0, tray_y_center, tray_bottom_z)),
            material=steel,
            name="tray_bottom",
        )
        drawer.visual(
            Box((tray_wall_t, tray_depth, tray_h)),
            origin=Origin(xyz=(-(tray_w - tray_wall_t) * 0.5, tray_y_center, tray_side_z)),
            material=steel,
            name="left_tray_side",
        )
        drawer.visual(
            Box((tray_wall_t, tray_depth, tray_h)),
            origin=Origin(xyz=((tray_w - tray_wall_t) * 0.5, tray_y_center, tray_side_z)),
            material=steel,
            name="right_tray_side",
        )
        drawer.visual(
            Box((tray_w, tray_wall_t, tray_h)),
            origin=Origin(xyz=(0.0, 0.010 + tray_depth - tray_wall_t * 0.5, tray_side_z)),
            material=steel,
            name="tray_back",
        )
        drawer.visual(
            Box((handle_bar_w, 0.016, 0.008)),
            origin=Origin(xyz=(0.0, -0.026, 0.004)),
            material=dark_steel,
            name="pull_bar",
        )
        drawer.visual(
            Box((0.016, 0.008, 0.012)),
            origin=Origin(xyz=(-0.170, -0.014, 0.004)),
            material=dark_steel,
            name="left_pull_post",
        )
        drawer.visual(
            Box((0.016, 0.008, 0.012)),
            origin=Origin(xyz=(0.170, -0.014, 0.004)),
            material=dark_steel,
            name="right_pull_post",
        )
        drawer.visual(
            Box((slide_drawer_t, moving_slide_len, slide_h)),
            origin=Origin(xyz=(-drawer_slide_x, slide_local_y, slide_local_z)),
            material=zinc,
            name="left_inner_slide",
        )
        drawer.visual(
            Box((slide_drawer_t, moving_slide_len, slide_h)),
            origin=Origin(xyz=(drawer_slide_x, slide_local_y, slide_local_z)),
            material=zinc,
            name="right_inner_slide",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((opening_width - 0.016, 0.020 + tray_depth, drawer_face_h)),
            mass=5.8,
            origin=Origin(xyz=(0.0, 0.220, -0.004)),
        )

        carriage_joint = model.articulation(
            f"body_to_drawer_carriage_{drawer_number}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=carriage,
            origin=Origin(xyz=(0.0, drawer_origin_y, face_center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=150.0,
                velocity=1.2,
                lower=0.0,
                upper=slide_stage_travel,
            ),
        )
        drawer_joint = model.articulation(
            f"drawer_carriage_{drawer_number}_to_drawer_{drawer_number}",
            ArticulationType.PRISMATIC,
            parent=carriage,
            child=drawer,
            origin=Origin(),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=150.0,
                velocity=1.2,
                lower=0.0,
                upper=slide_stage_travel,
            ),
        )

        drawer_carriages.append(carriage)
        drawers.append(drawer)
        drawer_carriage_joints.append(carriage_joint)
        drawer_joints.append(drawer_joint)

    def add_caster(name: str, *, x: float, y: float) -> tuple[object, object, object, object]:
        yoke = model.part(f"{name}_yoke")
        yoke.visual(
            Box((caster_plate, caster_plate, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=dark_steel,
            name="top_plate",
        )
        yoke.visual(
            Box((0.018, 0.018, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, -0.023)),
            material=dark_steel,
            name="swivel_stem",
        )
        yoke.visual(
            Box((0.016, caster_trail, 0.004)),
            origin=Origin(xyz=(0.0, caster_trail * 0.5, -0.006)),
            material=dark_steel,
            name="caster_spine",
        )
        yoke.visual(
            Box((fork_inside + 2.0 * fork_arm_t, 0.020, 0.008)),
            origin=Origin(xyz=(0.0, caster_trail, -0.006)),
            material=dark_steel,
            name="fork_bridge",
        )
        yoke.visual(
            Box((fork_arm_t, fork_arm_y, 0.060)),
            origin=Origin(
                xyz=(
                    -(fork_inside + fork_arm_t) * 0.5,
                    caster_trail,
                    -0.038,
                )
            ),
            material=dark_steel,
            name="left_fork",
        )
        yoke.visual(
            Box((fork_arm_t, fork_arm_y, 0.060)),
            origin=Origin(
                xyz=(
                    (fork_inside + fork_arm_t) * 0.5,
                    caster_trail,
                    -0.038,
                )
            ),
            material=dark_steel,
            name="right_fork",
        )
        yoke.inertial = Inertial.from_geometry(
            Box((caster_plate, caster_plate + caster_trail, wheel_center_drop + 0.040)),
            mass=1.2,
            origin=Origin(xyz=(0.0, caster_trail * 0.5, -0.040)),
        )

        wheel = model.part(f"{name}_wheel")
        wheel.visual(
            Cylinder(radius=wheel_radius, length=wheel_width),
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.026, length=hub_width),
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
            material=zinc,
            name="hub",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=hub_width),
            mass=0.85,
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        )

        swivel_joint = model.articulation(
            f"body_to_{name}_swivel",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=yoke,
            origin=Origin(xyz=(x, y, body_base_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=30.0, velocity=8.0),
        )
        wheel_joint = model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=yoke,
            child=wheel,
            origin=Origin(xyz=(0.0, caster_trail, -wheel_center_drop)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=20.0),
        )
        return yoke, wheel, swivel_joint, wheel_joint

    caster_specs = (
        ("caster_front_left", -(overall_width * 0.5 - 0.100), -(overall_depth * 0.5 - 0.085)),
        ("caster_front_right", overall_width * 0.5 - 0.100, -(overall_depth * 0.5 - 0.085)),
        ("caster_rear_left", -(overall_width * 0.5 - 0.100), overall_depth * 0.5 - 0.110),
        ("caster_rear_right", overall_width * 0.5 - 0.100, overall_depth * 0.5 - 0.110),
    )
    for caster_name, caster_x, caster_y in caster_specs:
        add_caster(caster_name, x=caster_x, y=caster_y)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    drawer_parts = [object_model.get_part(f"drawer_{index}") for index in range(1, 6)]
    carriage_parts = [object_model.get_part(f"drawer_carriage_{index}") for index in range(1, 6)]
    carriage_joints = [
        object_model.get_articulation(f"body_to_drawer_carriage_{index}") for index in range(1, 6)
    ]
    drawer_joints = [
        object_model.get_articulation(f"drawer_carriage_{index}_to_drawer_{index}") for index in range(1, 6)
    ]

    caster_names = (
        "caster_front_left",
        "caster_front_right",
        "caster_rear_left",
        "caster_rear_right",
    )
    caster_yokes = [object_model.get_part(f"{name}_yoke") for name in caster_names]
    caster_wheels = [object_model.get_part(f"{name}_wheel") for name in caster_names]
    caster_swivels = [object_model.get_articulation(f"body_to_{name}_swivel") for name in caster_names]
    caster_spins = [object_model.get_articulation(f"{name}_spin") for name in caster_names]

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20, name="articulation_overlap_sweep")

    ctx.check("five_drawers_present", len(drawer_parts) == 5, f"found {len(drawer_parts)} drawers")
    ctx.check("four_casters_present", len(caster_yokes) == 4, f"found {len(caster_yokes)} caster yokes")

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_aabb_available", "body world AABB was unavailable")
    else:
        body_min, body_max = body_aabb
        body_size = (
            body_max[0] - body_min[0],
            body_max[1] - body_min[1],
            body_max[2] - body_min[2],
        )
        ctx.check(
            "body_width_realistic",
            1.04 <= body_size[0] <= 1.10,
            f"body width {body_size[0]:.3f} m is outside expected wide tool chest range",
        )
        ctx.check(
            "body_depth_realistic",
            0.54 <= body_size[1] <= 0.57,
            f"body depth {body_size[1]:.3f} m is outside expected rolling chest range",
        )
        ctx.check(
            "body_height_realistic",
            0.77 <= body_size[2] <= 0.81,
            f"body height {body_size[2]:.3f} m is outside expected rolling chest range",
        )

    all_drawers_closed_pose = {}
    for carriage_joint, drawer_joint in zip(carriage_joints, drawer_joints):
        all_drawers_closed_pose[carriage_joint] = 0.0
        all_drawers_closed_pose[drawer_joint] = 0.0

    with ctx.pose(all_drawers_closed_pose):
        front_panel_boxes = []
        for index, drawer in enumerate(drawer_parts, start=1):
            front_panel_box = ctx.part_element_world_aabb(drawer, elem="front_panel")
            if front_panel_box is None:
                ctx.fail(f"drawer_{index}_front_panel_aabb", "front panel AABB unavailable")
            else:
                front_panel_boxes.append(front_panel_box)
        if len(front_panel_boxes) == 5:
            heights = [box_max[2] - box_min[2] for box_min, box_max in front_panel_boxes]
            centers = [(box_min[2] + box_max[2]) * 0.5 for box_min, box_max in front_panel_boxes]
            spacings = [centers[i + 1] - centers[i] for i in range(4)]
            ctx.check(
                "drawer_face_heights_match",
                max(heights) - min(heights) <= 0.001,
                f"drawer face heights vary too much: {heights}",
            )
            ctx.check(
                "drawer_faces_evenly_spaced",
                max(spacings) - min(spacings) <= 0.002,
                f"drawer vertical spacing is uneven: {spacings}",
            )

    for index, (drawer, carriage, carriage_joint, drawer_joint) in enumerate(
        zip(drawer_parts, carriage_parts, carriage_joints, drawer_joints),
        start=1,
    ):
        carriage_limits = carriage_joint.motion_limits
        drawer_limits = drawer_joint.motion_limits
        carriage_upper = 0.0 if carriage_limits is None or carriage_limits.upper is None else carriage_limits.upper
        drawer_upper = 0.0 if drawer_limits is None or drawer_limits.upper is None else drawer_limits.upper

        ctx.check(
            f"drawer_{index}_carriage_axis",
            tuple(round(value, 6) for value in carriage_joint.axis) == (0.0, -1.0, 0.0),
            f"expected carriage slide axis (0, -1, 0), got {carriage_joint.axis}",
        )
        ctx.check(
            f"drawer_{index}_drawer_axis",
            tuple(round(value, 6) for value in drawer_joint.axis) == (0.0, -1.0, 0.0),
            f"expected drawer slide axis (0, -1, 0), got {drawer_joint.axis}",
        )

        with ctx.pose({carriage_joint: 0.0, drawer_joint: 0.0}):
            ctx.expect_contact(
                body,
                carriage,
                elem_a=f"left_body_slide_{index}",
                elem_b="left_carriage_slide",
                name=f"drawer_{index}_closed_left_body_to_carriage_contact",
            )
            ctx.expect_contact(
                body,
                carriage,
                elem_a=f"right_body_slide_{index}",
                elem_b="right_carriage_slide",
                name=f"drawer_{index}_closed_right_body_to_carriage_contact",
            )
            ctx.expect_contact(
                carriage,
                drawer,
                elem_a="left_carriage_slide",
                elem_b="left_inner_slide",
                name=f"drawer_{index}_closed_left_carriage_to_drawer_contact",
            )
            ctx.expect_contact(
                carriage,
                drawer,
                elem_a="right_carriage_slide",
                elem_b="right_inner_slide",
                name=f"drawer_{index}_closed_right_carriage_to_drawer_contact",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="xz",
                min_overlap=0.10,
                name=f"drawer_{index}_closed_sits_in_body_width_height",
            )

        with ctx.pose({carriage_joint: carriage_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"drawer_{index}_carriage_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"drawer_{index}_carriage_upper_no_floating")
            ctx.expect_contact(
                body,
                carriage,
                elem_a=f"left_body_slide_{index}",
                elem_b="left_carriage_slide",
                name=f"drawer_{index}_carriage_upper_left_body_contact",
            )
            ctx.expect_contact(
                body,
                carriage,
                elem_a=f"right_body_slide_{index}",
                elem_b="right_carriage_slide",
                name=f"drawer_{index}_carriage_upper_right_body_contact",
            )

        with ctx.pose({drawer_joint: drawer_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"drawer_{index}_drawer_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"drawer_{index}_drawer_upper_no_floating")
            ctx.expect_contact(
                carriage,
                drawer,
                elem_a="left_carriage_slide",
                elem_b="left_inner_slide",
                name=f"drawer_{index}_drawer_upper_left_carriage_contact",
            )
            ctx.expect_contact(
                carriage,
                drawer,
                elem_a="right_carriage_slide",
                elem_b="right_inner_slide",
                name=f"drawer_{index}_drawer_upper_right_carriage_contact",
            )

        with ctx.pose({carriage_joint: carriage_upper, drawer_joint: drawer_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"drawer_{index}_full_extension_no_overlap")
            ctx.fail_if_isolated_parts(name=f"drawer_{index}_full_extension_no_floating")
            ctx.expect_contact(
                body,
                carriage,
                elem_a=f"left_body_slide_{index}",
                elem_b="left_carriage_slide",
                name=f"drawer_{index}_full_extension_left_body_contact",
            )
            ctx.expect_contact(
                carriage,
                drawer,
                elem_a="left_carriage_slide",
                elem_b="left_inner_slide",
                name=f"drawer_{index}_full_extension_left_drawer_contact",
            )
            ctx.expect_gap(
                body,
                drawer,
                axis="y",
                negative_elem="front_panel",
                min_gap=0.34,
                name=f"drawer_{index}_full_extension_projects_out_front",
            )

    for caster_name, yoke, wheel, swivel_joint, spin_joint in zip(
        caster_names,
        caster_yokes,
        caster_wheels,
        caster_swivels,
        caster_spins,
    ):
        ctx.check(
            f"{caster_name}_swivel_axis",
            tuple(round(value, 6) for value in swivel_joint.axis) == (0.0, 0.0, 1.0),
            f"expected caster swivel axis (0, 0, 1), got {swivel_joint.axis}",
        )
        ctx.check(
            f"{caster_name}_spin_axis",
            tuple(round(value, 6) for value in spin_joint.axis) == (1.0, 0.0, 0.0),
            f"expected wheel spin axis (1, 0, 0), got {spin_joint.axis}",
        )

        ctx.expect_contact(
            body,
            yoke,
            elem_a="bottom_panel",
            elem_b="top_plate",
            name=f"{caster_name}_body_to_top_plate_contact",
        )
        ctx.expect_contact(
            yoke,
            wheel,
            elem_a="left_fork",
            elem_b="hub",
            name=f"{caster_name}_left_fork_to_hub_contact",
        )
        ctx.expect_contact(
            yoke,
            wheel,
            elem_a="right_fork",
            elem_b="hub",
            name=f"{caster_name}_right_fork_to_hub_contact",
        )

        with ctx.pose({swivel_joint: 1.10, spin_joint: 1.57}):
            ctx.fail_if_isolated_parts(name=f"{caster_name}_posed_no_floating")
            ctx.expect_contact(
                body,
                yoke,
                elem_a="bottom_panel",
                elem_b="top_plate",
                name=f"{caster_name}_posed_body_contact",
            )
            ctx.expect_contact(
                yoke,
                wheel,
                elem_a="left_fork",
                elem_b="hub",
                name=f"{caster_name}_posed_left_fork_contact",
            )
            ctx.expect_contact(
                yoke,
                wheel,
                elem_a="right_fork",
                elem_b="hub",
                name=f"{caster_name}_posed_right_fork_contact",
            )

    ctx.fail_if_isolated_parts(max_pose_samples=8, name="sampled_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
