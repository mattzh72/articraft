from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_care_overbed_table")

    steel = Material("warm_gray_powder_coated_steel", color=(0.62, 0.62, 0.58, 1.0))
    dark_steel = Material("dark_gray_hardware", color=(0.18, 0.18, 0.17, 1.0))
    black_rubber = Material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    nylon = Material("off_white_nylon_glides", color=(0.86, 0.84, 0.76, 1.0))
    laminate = Material("maple_laminate", color=(0.82, 0.70, 0.50, 1.0))
    edge_band = Material("brown_edge_band", color=(0.35, 0.22, 0.12, 1.0))

    base = model.part("base_frame")
    # Low offset pedestal base: two rails on the patient side, tied at the post side.
    base.visual(Box((0.88, 0.050, 0.035)), origin=Origin(xyz=(0.07, 0.240, 0.105)), material=steel, name="rail_0")
    base.visual(Box((0.88, 0.050, 0.035)), origin=Origin(xyz=(0.07, -0.240, 0.105)), material=steel, name="rail_1")
    base.visual(Box((0.080, 0.530, 0.040)), origin=Origin(xyz=(-0.370, 0.0, 0.108)), material=steel, name="post_side_cross_rail")

    post_x = -0.335
    # Hollow lower height sleeve made from four walls so the moving post can slide
    # with real clearance instead of occupying a solid placeholder.
    base.visual(Box((0.008, 0.078, 0.520)), origin=Origin(xyz=(post_x + 0.037, 0.0, 0.380)), material=steel, name="lower_sleeve_wall_0")
    base.visual(Box((0.008, 0.078, 0.520)), origin=Origin(xyz=(post_x - 0.037, 0.0, 0.380)), material=steel, name="lower_sleeve_wall_1")
    base.visual(Box((0.078, 0.008, 0.520)), origin=Origin(xyz=(post_x, 0.037, 0.380)), material=steel, name="lower_sleeve_wall_2")
    base.visual(Box((0.078, 0.008, 0.520)), origin=Origin(xyz=(post_x, -0.037, 0.380)), material=steel, name="lower_sleeve_wall_3")
    base.visual(Box((0.0115, 0.105, 0.055)), origin=Origin(xyz=(post_x + 0.04675, 0.0, 0.585)), material=dark_steel, name="height_collar_0")
    base.visual(Box((0.0115, 0.105, 0.055)), origin=Origin(xyz=(post_x - 0.04675, 0.0, 0.585)), material=dark_steel, name="height_collar_1")
    base.visual(Box((0.105, 0.0115, 0.055)), origin=Origin(xyz=(post_x, 0.04675, 0.585)), material=dark_steel, name="height_collar_2")
    base.visual(Box((0.105, 0.0115, 0.055)), origin=Origin(xyz=(post_x, -0.04675, 0.585)), material=dark_steel, name="height_collar_3")
    base.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(post_x, -0.065, 0.585), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="knob_hub",
    )

    # Caster forks and short pivot bosses are integrated into the base frame.
    for idx, y in enumerate((0.240, -0.240)):
        sign = 1.0 if y > 0 else -1.0
        base.visual(Box((0.086, 0.008, 0.055)), origin=Origin(xyz=(0.435, y - sign * 0.023, 0.061)), material=steel, name=f"caster_fork_{idx}_0")
        base.visual(Box((0.086, 0.008, 0.055)), origin=Origin(xyz=(0.435, y + sign * 0.023, 0.061)), material=steel, name=f"caster_fork_{idx}_1")
        base.visual(
            Cylinder(radius=0.007, length=0.022),
            origin=Origin(xyz=(0.435, y - sign * 0.028, 0.052), rpy=(pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name=f"caster_axle_stub_{idx}_0",
        )
        base.visual(
            Cylinder(radius=0.007, length=0.022),
            origin=Origin(xyz=(0.435, y + sign * 0.028, 0.052), rpy=(pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name=f"caster_axle_stub_{idx}_1",
        )

    # Brake bar pivots are real brackets on the lower rails; the moving bar
    # spans between them rather than floating below the frame.
    base.visual(Box((0.060, 0.040, 0.055)), origin=Origin(xyz=(0.215, 0.195, 0.068)), material=dark_steel, name="brake_pivot_0")
    base.visual(Box((0.060, 0.040, 0.055)), origin=Origin(xyz=(0.215, -0.195, 0.068)), material=dark_steel, name="brake_pivot_1")

    upper_post = model.part("upper_post")
    upper_post.visual(Box((0.045, 0.045, 0.430)), origin=Origin(xyz=(0.0, 0.0, -0.055)), material=steel, name="sliding_square_tube")
    upper_post.visual(Box((0.0105, 0.020, 0.028)), origin=Origin(xyz=(0.02775, 0.0, -0.230)), material=nylon, name="glide_lower_x_pos")
    upper_post.visual(Box((0.0105, 0.020, 0.028)), origin=Origin(xyz=(-0.02775, 0.0, -0.230)), material=nylon, name="glide_lower_x_neg")
    upper_post.visual(Box((0.020, 0.0105, 0.028)), origin=Origin(xyz=(0.0, 0.02775, -0.230)), material=nylon, name="glide_lower_y_pos")
    upper_post.visual(Box((0.020, 0.0105, 0.028)), origin=Origin(xyz=(0.0, -0.02775, -0.230)), material=nylon, name="glide_lower_y_neg")
    upper_post.visual(Box((0.0105, 0.020, 0.028)), origin=Origin(xyz=(0.02775, 0.0, -0.050)), material=nylon, name="glide_upper_x_pos")
    upper_post.visual(Box((0.0105, 0.020, 0.028)), origin=Origin(xyz=(-0.02775, 0.0, -0.050)), material=nylon, name="glide_upper_x_neg")
    upper_post.visual(Box((0.020, 0.0105, 0.028)), origin=Origin(xyz=(0.0, 0.02775, -0.050)), material=nylon, name="glide_upper_y_pos")
    upper_post.visual(Box((0.020, 0.0105, 0.028)), origin=Origin(xyz=(0.0, -0.02775, -0.050)), material=nylon, name="glide_upper_y_neg")
    upper_post.visual(Box((0.480, 0.045, 0.045)), origin=Origin(xyz=(0.205, 0.0, 0.140)), material=steel, name="underside_cantilever_arm")
    upper_post.visual(Box((0.130, 0.180, 0.010)), origin=Origin(xyz=(0.340, 0.0, 0.158)), material=dark_steel, name="top_mounting_plate")

    tabletop = model.part("tabletop")
    tabletop.visual(Box((0.820, 0.420, 0.030)), origin=Origin(), material=laminate, name="laminate_slab")
    tabletop.visual(Box((0.842, 0.014, 0.036)), origin=Origin(xyz=(0.0, 0.217, 0.0)), material=edge_band, name="edge_band_0")
    tabletop.visual(Box((0.842, 0.014, 0.036)), origin=Origin(xyz=(0.0, -0.217, 0.0)), material=edge_band, name="edge_band_1")
    tabletop.visual(Box((0.014, 0.420, 0.036)), origin=Origin(xyz=(0.417, 0.0, 0.0)), material=edge_band, name="edge_band_2")
    tabletop.visual(Box((0.014, 0.420, 0.036)), origin=Origin(xyz=(-0.417, 0.0, 0.0)), material=edge_band, name="edge_band_3")

    collar_knob = model.part("collar_knob")
    collar_knob.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=black_rubber,
        name="knob_disk",
    )
    collar_knob.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="center_cap",
    )

    brake_bar = model.part("brake_bar")
    brake_bar.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=dark_steel,
        name="bar_tube",
    )
    brake_bar.visual(Box((0.032, 0.055, 0.036)), origin=Origin(xyz=(0.020, 0.0, -0.018)), material=dark_steel, name="pedal_web")
    brake_bar.visual(Box((0.110, 0.200, 0.012)), origin=Origin(xyz=(0.060, 0.0, -0.036)), material=black_rubber, name="foot_pad")

    caster_parts = []
    for idx, y in enumerate((0.240, -0.240)):
        caster = model.part(f"caster_{idx}")
        caster.visual(
            Cylinder(radius=0.040, length=0.032),
            origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
            material=black_rubber,
            name="rubber_wheel",
        )
        caster.visual(
            Cylinder(radius=0.018, length=0.034),
            origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name="wheel_hub",
        )
        caster_parts.append((caster, y))

    model.articulation(
        "base_to_upper_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_post,
        origin=Origin(xyz=(post_x, 0.0, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.12, lower=0.0, upper=0.220),
    )
    model.articulation(
        "upper_post_to_tabletop",
        ArticulationType.FIXED,
        parent=upper_post,
        child=tabletop,
        origin=Origin(xyz=(0.340, 0.0, 0.178)),
    )
    model.articulation(
        "base_to_collar_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=collar_knob,
        origin=Origin(xyz=(post_x, -0.0825, 0.585)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "base_to_brake_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=brake_bar,
        origin=Origin(xyz=(0.215, 0.0, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=0.0, upper=0.35),
    )
    for idx, (caster, y) in enumerate(caster_parts):
        model.articulation(
            f"base_to_caster_{idx}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=(0.435, y, 0.052)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    upper_post = object_model.get_part("upper_post")
    tabletop = object_model.get_part("tabletop")
    knob = object_model.get_part("collar_knob")
    brake = object_model.get_part("brake_bar")
    caster_0 = object_model.get_part("caster_0")
    caster_1 = object_model.get_part("caster_1")

    height_joint = object_model.get_articulation("base_to_upper_post")
    brake_joint = object_model.get_articulation("base_to_brake_bar")
    knob_joint = object_model.get_articulation("base_to_collar_knob")
    caster_joint_0 = object_model.get_articulation("base_to_caster_0")
    caster_joint_1 = object_model.get_articulation("base_to_caster_1")

    ctx.check(
        "primary articulated mechanisms are present",
        height_joint.articulation_type == ArticulationType.PRISMATIC
        and brake_joint.articulation_type == ArticulationType.REVOLUTE
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and caster_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and caster_joint_1.articulation_type == ArticulationType.CONTINUOUS,
    )

    ctx.expect_contact(
        knob,
        base,
        elem_a="knob_disk",
        elem_b="knob_hub",
        contact_tol=0.001,
        name="collar knob seats on threaded hub",
    )
    ctx.expect_contact(
        brake,
        base,
        elem_a="bar_tube",
        elem_b="brake_pivot_0",
        contact_tol=0.001,
        name="brake bar supported by pivot bracket 0",
    )
    ctx.expect_contact(
        brake,
        base,
        elem_a="bar_tube",
        elem_b="brake_pivot_1",
        contact_tol=0.001,
        name="brake bar supported by pivot bracket 1",
    )
    ctx.expect_contact(
        tabletop,
        upper_post,
        elem_a="laminate_slab",
        elem_b="top_mounting_plate",
        contact_tol=0.001,
        name="laminate top is seated on post mounting plate",
    )
    ctx.expect_contact(
        upper_post,
        base,
        elem_a="glide_lower_x_pos",
        elem_b="lower_sleeve_wall_0",
        contact_tol=0.001,
        name="sliding post rides in lower sleeve",
    )

    ctx.expect_origin_gap(
        tabletop,
        base,
        axis="z",
        min_gap=0.72,
        max_gap=0.86,
        name="collapsed tabletop height is home-care scale",
    )

    caster_0_pos = ctx.part_world_position(caster_0)
    caster_1_pos = ctx.part_world_position(caster_1)
    ctx.check(
        "twin front casters are tucked under patient side",
        caster_0_pos is not None
        and caster_1_pos is not None
        and caster_0_pos[0] > 0.40
        and caster_1_pos[0] > 0.40
        and caster_0_pos[2] < 0.07
        and caster_1_pos[2] < 0.07,
        details=f"caster_0={caster_0_pos}, caster_1={caster_1_pos}",
    )

    top_aabb = ctx.part_element_world_aabb(tabletop, elem="laminate_slab")
    ctx.check(
        "rectangular top reaches past the offset pedestal",
        top_aabb is not None and top_aabb[1][0] > 0.40 and top_aabb[0][0] < -0.38,
        details=f"laminate_slab_aabb={top_aabb}",
    )

    rest_top = ctx.part_world_position(tabletop)
    rest_pad_aabb = ctx.part_element_world_aabb(brake, elem="foot_pad")
    with ctx.pose({height_joint: 0.220, brake_joint: 0.35, caster_joint_0: pi, caster_joint_1: -pi, knob_joint: 2 * pi}):
        raised_top = ctx.part_world_position(tabletop)
        pressed_pad_aabb = ctx.part_element_world_aabb(brake, elem="foot_pad")
        ctx.expect_origin_gap(
            tabletop,
            base,
            axis="z",
            min_gap=0.94,
            max_gap=1.08,
            name="raised tabletop remains home-care scale",
        )
        ctx.expect_contact(
            upper_post,
            base,
            elem_a="glide_lower_x_pos",
            elem_b="lower_sleeve_wall_0",
            contact_tol=0.001,
            name="extended post remains captured by sleeve glides",
        )

    ctx.check(
        "height adjustment raises the tabletop",
        rest_top is not None and raised_top is not None and raised_top[2] > rest_top[2] + 0.20,
        details=f"rest={rest_top}, raised={raised_top}",
    )
    ctx.check(
        "brake pedal rotates downward when pressed",
        rest_pad_aabb is not None
        and pressed_pad_aabb is not None
        and pressed_pad_aabb[0][2] < rest_pad_aabb[0][2] - 0.010,
        details=f"rest_pad={rest_pad_aabb}, pressed_pad={pressed_pad_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
