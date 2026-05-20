from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_portable_air_scrubber")

    housing_color = model.material("housing_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    trim_color = model.material("trim_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    grille_color = model.material("grille_black", rgba=(0.12, 0.13, 0.14, 1.0))
    wheel_color = model.material("wheel_black", rgba=(0.08, 0.08, 0.09, 1.0))
    fork_color = model.material("fork_steel", rgba=(0.48, 0.50, 0.54, 1.0))
    filter_frame_color = model.material("filter_frame", rgba=(0.73, 0.75, 0.78, 1.0))
    prefilter_color = model.material("prefilter_green", rgba=(0.44, 0.58, 0.44, 1.0))
    pleat_color = model.material("pleat_white", rgba=(0.90, 0.92, 0.91, 1.0))
    carbon_color = model.material("carbon_dark", rgba=(0.20, 0.21, 0.22, 1.0))
    hepa_color = model.material("hepa_blue", rgba=(0.48, 0.62, 0.78, 1.0))
    accent_color = model.material("accent_orange", rgba=(0.86, 0.47, 0.16, 1.0))

    body = model.part("housing")

    body_width = 0.56
    body_depth = 0.46
    body_height = 0.96
    body_bottom = 0.085
    wall = 0.022
    half_w = body_width / 2.0
    half_d = body_depth / 2.0
    body_top = body_bottom + body_height

    opening_width = 0.474
    opening_height = 0.500
    opening_bottom = body_bottom + 0.055
    opening_top = opening_bottom + opening_height
    lintel_height = 0.045
    jamb_width = 0.032

    left_side_x = -half_w + wall / 2.0
    right_side_x = half_w - wall / 2.0
    back_y = -half_d + wall / 2.0
    front_panel_y = half_d - wall / 2.0

    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(left_side_x, 0.0, body_bottom + body_height / 2.0)),
        material=housing_color,
        name="left_side",
    )
    body.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(right_side_x, 0.0, body_bottom + body_height / 2.0)),
        material=housing_color,
        name="right_side",
    )
    body.visual(
        Box((body_width - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, back_y, body_bottom + body_height / 2.0)),
        material=housing_color,
        name="back_panel",
    )
    body.visual(
        Box((body_width - 2.0 * wall, body_depth - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + wall / 2.0)),
        material=housing_color,
        name="base_pan",
    )
    body.visual(
        Box((body_width - 2.0 * wall, body_depth - 2.0 * wall, wall)),
        origin=Origin(xyz=(0.0, 0.0, body_top - wall / 2.0)),
        material=housing_color,
        name="roof",
    )

    upper_front_height = body_top - (opening_top + lintel_height)
    body.visual(
        Box((body_width - 2.0 * wall, wall, upper_front_height)),
        origin=Origin(
            xyz=(0.0, front_panel_y, opening_top + lintel_height + upper_front_height / 2.0)
        ),
        material=housing_color,
        name="upper_front_panel",
    )
    body.visual(
        Box((body_width - 2.0 * wall, wall, opening_bottom - body_bottom)),
        origin=Origin(
            xyz=(0.0, front_panel_y, body_bottom + (opening_bottom - body_bottom) / 2.0)
        ),
        material=housing_color,
        name="front_sill",
    )
    body.visual(
        Box((jamb_width, wall, opening_height + lintel_height)),
        origin=Origin(
            xyz=(
                -(opening_width / 2.0 + jamb_width / 2.0),
                front_panel_y,
                opening_bottom + (opening_height + lintel_height) / 2.0,
            )
        ),
        material=housing_color,
        name="front_left_jamb",
    )
    body.visual(
        Box((jamb_width, wall, opening_height + lintel_height)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 + jamb_width / 2.0,
                front_panel_y,
                opening_bottom + (opening_height + lintel_height) / 2.0,
            )
        ),
        material=housing_color,
        name="front_right_jamb",
    )
    body.visual(
        Box((opening_width, wall, lintel_height)),
        origin=Origin(xyz=(0.0, front_panel_y, opening_top + lintel_height / 2.0)),
        material=housing_color,
        name="front_lintel",
    )

    vent_width = body_width - 2.0 * wall - 0.040
    vent_slat_thickness = 0.010
    vent_front_y = half_d - vent_slat_thickness / 2.0
    first_slat_z = opening_top + lintel_height + 0.055
    for idx in range(6):
        body.visual(
            Box((vent_width, vent_slat_thickness, 0.014)),
            origin=Origin(xyz=(0.0, vent_front_y, first_slat_z + idx * 0.045)),
            material=grille_color,
            name=f"vent_slat_{idx + 1}",
        )

    body.visual(
        Box((0.16, 0.08, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, body_top + 0.010)),
        material=trim_color,
        name="top_exhaust_plenum",
    )
    body.visual(
        Cylinder(radius=0.075, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, body_top + 0.030)),
        material=trim_color,
        name="duct_collar",
    )

    guide_rail_width = 0.030
    guide_rail_depth = 0.280
    guide_rail_height = 0.020
    guide_rail_center_y = half_d - wall - guide_rail_depth / 2.0 + 0.010
    guide_rail_z = body_bottom + wall + guide_rail_height / 2.0
    guide_rail_x = 0.170
    for side, x_pos in (("left", -guide_rail_x), ("right", guide_rail_x)):
        body.visual(
            Box((guide_rail_width, guide_rail_depth, guide_rail_height)),
            origin=Origin(xyz=(x_pos, guide_rail_center_y, guide_rail_z)),
            material=trim_color,
            name=f"{side}_guide_rail",
        )

    hinge_radius = 0.009
    hinge_x = -opening_width / 2.0 - 0.002
    hinge_y = half_d + 0.013
    for leaf_name, leaf_z, leaf_length in (
        ("body_hinge_leaf_lower", opening_bottom - 0.013, 0.030),
        ("body_hinge_leaf_mid", opening_bottom + 0.142, 0.035),
        ("body_hinge_leaf_upper", opening_bottom + 0.307, 0.045),
    ):
        body.visual(
            Box((0.020, 0.024, leaf_length)),
            origin=Origin(xyz=(hinge_x - 0.008, half_d + 0.008, leaf_z)),
            material=trim_color,
            name=leaf_name,
        )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.030),
        origin=Origin(xyz=(hinge_x, hinge_y, opening_bottom - 0.013)),
        material=trim_color,
        name="body_hinge_knuckle_lower",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.035),
        origin=Origin(xyz=(hinge_x, hinge_y, opening_bottom + 0.142)),
        material=trim_color,
        name="body_hinge_knuckle_mid",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=0.045),
        origin=Origin(xyz=(hinge_x, hinge_y, opening_bottom + 0.307)),
        material=trim_color,
        name="body_hinge_knuckle_upper",
    )

    door = model.part("front_door")
    door_width = opening_width - 0.008
    door_height = opening_height - 0.008
    door_thickness = 0.028
    door_frame_depth = 0.016
    panel_center_y = -door_thickness / 2.0 - 0.013
    panel_inner_face_y = panel_center_y - door_thickness / 2.0

    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width / 2.0,
                panel_center_y,
                door_height / 2.0,
            )
        ),
        material=housing_color,
        name="door_panel",
    )
    door.visual(
        Box((door_width - 0.050, door_frame_depth, door_height - 0.070)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width / 2.0,
                -door_thickness + door_frame_depth / 2.0 - 0.013,
                door_height / 2.0,
            )
        ),
        material=trim_color,
        name="door_inner_stiffener",
    )
    door.visual(
        Box((0.015, 0.022, door_height - 0.020)),
        origin=Origin(xyz=(0.0155, -0.002, door_height / 2.0)),
        material=trim_color,
        name="door_hinge_leaf",
    )
    door.visual(
        Box((0.042, 0.014, 0.220)),
        origin=Origin(xyz=(hinge_radius + door_width - 0.040, -0.006, door_height / 2.0)),
        material=trim_color,
        name="handle_base",
    )
    door.visual(
        Box((0.026, 0.040, 0.180)),
        origin=Origin(
            xyz=(hinge_radius + door_width - 0.040, 0.012, door_height / 2.0)
        ),
        material=accent_color,
        name="door_handle",
    )
    door.visual(
        Box((0.012, 0.010, door_height - 0.080)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width - 0.005,
                panel_inner_face_y + 0.005,
                door_height / 2.0,
            )
        ),
        material=trim_color,
        name="door_overlap_right",
    )
    door.visual(
        Box((door_width - 0.090, 0.010, 0.012)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width / 2.0 + 0.006,
                panel_inner_face_y + 0.005,
                0.490,
            )
        ),
        material=trim_color,
        name="door_overlap_top",
    )
    door.visual(
        Box((door_width - 0.110, 0.010, 0.014)),
        origin=Origin(
            xyz=(
                hinge_radius + door_width / 2.0 + 0.002,
                panel_inner_face_y + 0.005,
                0.003,
            )
        ),
        material=trim_color,
        name="door_overlap_bottom",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=trim_color,
        name="door_hinge_knuckle_lower",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=trim_color,
        name="door_hinge_knuckle_mid",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=trim_color,
        name="door_hinge_knuckle_upper",
    )

    filter_stack = model.part("filter_stack")
    stack_width = 0.430
    stack_depth = 0.320
    stack_height = 0.420
    tray_thickness = 0.014
    side_rail_width = 0.030

    filter_stack.visual(
        Box((stack_width, stack_depth, tray_thickness)),
        origin=Origin(xyz=(0.0, -stack_depth / 2.0, tray_thickness / 2.0)),
        material=trim_color,
        name="tray",
    )
    filter_stack.visual(
        Box((side_rail_width, stack_depth, stack_height - tray_thickness)),
        origin=Origin(
            xyz=(
                -(stack_width / 2.0 - side_rail_width / 2.0),
                -stack_depth / 2.0,
                tray_thickness + (stack_height - tray_thickness) / 2.0,
            )
        ),
        material=filter_frame_color,
        name="left_stack_rail",
    )
    filter_stack.visual(
        Box((side_rail_width, stack_depth, stack_height - tray_thickness)),
        origin=Origin(
            xyz=(
                stack_width / 2.0 - side_rail_width / 2.0,
                -stack_depth / 2.0,
                tray_thickness + (stack_height - tray_thickness) / 2.0,
            )
        ),
        material=filter_frame_color,
        name="right_stack_rail",
    )
    filter_stack.visual(
        Box((stack_width - 0.060, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -stack_depth + 0.010, stack_height - 0.010)),
        material=trim_color,
        name="rear_crossbrace",
    )
    filter_stack.visual(
        Box((0.070, 0.016, 0.210)),
        origin=Origin(xyz=(0.0, -0.008, 0.116)),
        material=trim_color,
        name="pull_upright",
    )
    filter_stack.visual(
        Box((0.150, 0.018, 0.045)),
        origin=Origin(xyz=(0.0, -0.004, 0.215)),
        material=accent_color,
        name="pull_handle",
    )

    stage_specs = (
        ("stage_1", -0.040, 0.018, prefilter_color),
        ("stage_2", -0.100, 0.028, pleat_color),
        ("stage_3", -0.170, 0.036, carbon_color),
        ("stage_4", -0.250, 0.060, hepa_color),
    )
    for stage_name, y_center, depth, media_material in stage_specs:
        filter_stack.visual(
            Box((stack_width - 0.040, depth, 0.360)),
            origin=Origin(xyz=(0.0, y_center, 0.200)),
            material=filter_frame_color,
            name=f"{stage_name}_frame",
        )
        filter_stack.visual(
            Box((stack_width - 0.095, depth * 0.78, 0.305)),
            origin=Origin(xyz=(0.0, y_center, 0.200)),
            material=media_material,
            name=f"{stage_name}_media",
        )

    wheel_radius = 0.040
    wheel_width = 0.022
    fork_arm_width = 0.008
    fork_arm_depth = 0.024
    fork_arm_height = 0.032

    caster_locations = (
        ("front_left_caster", -0.205, 0.165),
        ("front_right_caster", 0.205, 0.165),
        ("rear_left_caster", -0.205, -0.165),
        ("rear_right_caster", 0.205, -0.165),
    )

    for caster_name, caster_x, caster_y in caster_locations:
        caster = model.part(caster_name)
        caster.visual(
            Box((0.055, 0.040, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=fork_color,
            name="mount_plate",
        )
        caster.visual(
            Cylinder(radius=0.009, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=fork_color,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.040, 0.028, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.021)),
            material=fork_color,
            name="fork_top",
        )
        caster.visual(
            Box((fork_arm_width, fork_arm_depth, fork_arm_height)),
            origin=Origin(xyz=(-0.011, 0.0, -0.039)),
            material=fork_color,
            name="left_fork_arm",
        )
        caster.visual(
            Box((fork_arm_width, fork_arm_depth, fork_arm_height)),
            origin=Origin(xyz=(0.011, 0.0, -0.039)),
            material=fork_color,
            name="right_fork_arm",
        )
        caster.visual(
            Cylinder(radius=wheel_radius, length=wheel_width),
            origin=Origin(xyz=(0.0, 0.0, -0.045), rpy=(0.0, 1.57079632679, 0.0)),
            material=wheel_color,
            name="wheel",
        )

        model.articulation(
            f"housing_to_{caster_name}",
            ArticulationType.FIXED,
            parent=body,
            child=caster,
            origin=Origin(xyz=(caster_x, caster_y, body_bottom)),
        )

    model.articulation(
        "housing_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, opening_bottom + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.8,
            lower=0.0,
            upper=2.05,
        ),
    )

    model.articulation(
        "housing_to_filter_stack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_stack,
        origin=Origin(
            xyz=(
                0.0,
                half_d - wall - 0.020,
                body_bottom + wall + guide_rail_height,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=0.220,
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
    housing = object_model.get_part("housing")
    door = object_model.get_part("front_door")
    filter_stack = object_model.get_part("filter_stack")
    door_hinge = object_model.get_articulation("housing_to_front_door")
    stack_slide = object_model.get_articulation("housing_to_filter_stack")

    ctx.check("housing exists", housing is not None)
    ctx.check("front door exists", door is not None)
    ctx.check("filter stack exists", filter_stack is not None)

    ctx.expect_gap(
        housing,
        door,
        axis="x",
        positive_elem="front_right_jamb",
        negative_elem="door_panel",
        min_gap=0.001,
        max_gap=0.010,
        name="door has a small right-edge reveal when closed",
    )
    ctx.expect_gap(
        housing,
        door,
        axis="z",
        positive_elem="front_lintel",
        negative_elem="door_panel",
        min_gap=0.001,
        max_gap=0.010,
        name="door has a small top reveal when closed",
    )
    ctx.expect_within(
        filter_stack,
        housing,
        axes="xz",
        margin=0.030,
        name="retracted filter stack stays centered in the housing cavity",
    )
    ctx.expect_overlap(
        filter_stack,
        housing,
        axes="y",
        min_overlap=0.120,
        name="retracted filter stack remains inserted in the cabinet",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.35}):
        open_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward on the left hinge",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.140,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    retracted_stack_pos = ctx.part_world_position(filter_stack)
    with ctx.pose({stack_slide: 0.220}):
        extended_stack_pos = ctx.part_world_position(filter_stack)
        ctx.expect_within(
            filter_stack,
            housing,
            axes="xz",
            margin=0.030,
            name="extended filter stack remains rail-aligned in x and z",
        )
        ctx.expect_overlap(
            filter_stack,
            housing,
            axes="y",
            min_overlap=0.080,
            name="extended filter stack retains insertion on the guide rails",
        )
    ctx.check(
        "filter stack slides forward out of the lower bay",
        retracted_stack_pos is not None
        and extended_stack_pos is not None
        and extended_stack_pos[1] > retracted_stack_pos[1] + 0.180,
        details=f"retracted={retracted_stack_pos}, extended={extended_stack_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
