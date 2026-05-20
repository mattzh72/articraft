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
    model = ArticulatedObject(name="bench_guillotine_paper_cutter")

    bed_paint = model.material("bed_paint", rgba=(0.22, 0.28, 0.33, 1.0))
    bed_trim = model.material("bed_trim", rgba=(0.15, 0.17, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.79, 0.81, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.88, 0.89, 0.91, 1.0))
    clamp_dark = model.material("clamp_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    safety_red = model.material("safety_red", rgba=(0.74, 0.12, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.50, 0.36, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=bed_paint,
        name="bed_surface",
    )
    base.visual(
        Box((0.42, 0.012, 0.020)),
        origin=Origin(xyz=(-0.02, 0.164, 0.026)),
        material=bed_trim,
        name="rear_fence",
    )
    base.visual(
        Box((0.010, 0.320, 0.001)),
        origin=Origin(xyz=(0.212, -0.020, 0.0165)),
        material=blade_steel,
        name="cutting_strip",
    )
    base.visual(
        Box((0.044, 0.022, 0.012)),
        origin=Origin(xyz=(0.214, 0.167, 0.016)),
        material=bed_trim,
        name="blade_hinge_block",
    )
    for foot_x, foot_y, foot_name in (
        (-0.205, -0.145, "foot_front_left"),
        (0.205, -0.145, "foot_front_right"),
        (-0.205, 0.145, "foot_rear_left"),
        (0.205, 0.145, "foot_rear_right"),
    ):
        base.visual(
            Box((0.032, 0.024, 0.006)),
            origin=Origin(xyz=(foot_x, foot_y, -0.003)),
            material=rubber,
            name=foot_name,
        )

    for post_y, suffix in ((-0.105, "front"), (0.105, "rear")):
        base.visual(
            Box((0.028, 0.028, 0.006)),
            origin=Origin(xyz=(0.190, post_y, 0.019)),
            material=bed_trim,
            name=f"guide_post_base_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.070),
            origin=Origin(xyz=(0.190, post_y, 0.051)),
            material=steel,
            name=f"guide_post_{suffix}",
        )

    base.visual(
        Box((0.050, 0.030, 0.010)),
        origin=Origin(xyz=(0.080, -0.162, 0.017)),
        material=bed_trim,
        name="clamp_lever_bracket",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.50, 0.36, 0.09)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Box((0.024, 0.500, 0.010)),
        origin=Origin(xyz=(0.0, -0.250, 0.002)),
        material=steel,
        name="arm_beam",
    )
    blade_arm.visual(
        Box((0.032, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, 0.000)),
        material=steel,
        name="hinge_saddle",
    )
    blade_arm.visual(
        Box((0.004, 0.330, 0.004)),
        origin=Origin(xyz=(-0.010, -0.160, -0.006)),
        material=blade_steel,
        name="blade_edge",
    )
    blade_arm.visual(
        Box((0.028, 0.090, 0.022)),
        origin=Origin(xyz=(0.0, -0.455, 0.014)),
        material=safety_red,
        name="handle_mount",
    )
    blade_arm.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=Origin(xyz=(0.0, -0.500, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.050, 0.56, 0.07)),
        mass=1.2,
        origin=Origin(xyz=(0.0, -0.28, 0.01)),
    )

    clamp_bar = model.part("clamp_bar")
    clamp_bar.visual(
        Box((0.014, 0.300, 0.006)),
        origin=Origin(xyz=(-0.051, 0.0, 0.023)),
        material=clamp_dark,
        name="upper_rail",
    )
    clamp_bar.visual(
        Box((0.018, 0.280, 0.010)),
        origin=Origin(xyz=(-0.051, 0.0, -0.019)),
        material=clamp_dark,
        name="clamp_beam",
    )
    clamp_bar.visual(
        Box((0.016, 0.280, 0.004)),
        origin=Origin(xyz=(-0.051, 0.0, -0.026)),
        material=rubber,
        name="clamp_pad",
    )
    for post_y, suffix in ((-0.105, "front"), (0.105, "rear")):
        clamp_bar.visual(
            Box((0.008, 0.018, 0.036)),
            origin=Origin(xyz=(-0.010, post_y, 0.002)),
            material=clamp_dark,
            name=f"guide_shoe_{suffix}",
        )
        clamp_bar.visual(
            Box((0.030, 0.014, 0.012)),
            origin=Origin(xyz=(-0.029, post_y, 0.014)),
            material=clamp_dark,
            name=f"shoe_arm_{suffix}",
        )
        clamp_bar.visual(
            Box((0.014, 0.018, 0.040)),
            origin=Origin(xyz=(-0.051, post_y, 0.000)),
            material=clamp_dark,
            name=f"guide_keeper_{suffix}",
        )
    clamp_bar.inertial = Inertial.from_geometry(
        Box((0.060, 0.30, 0.06)),
        mass=0.45,
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
    )

    clamp_lever = model.part("clamp_lever")
    clamp_lever.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="lever_pivot",
    )
    clamp_lever.visual(
        Box((0.020, 0.110, 0.020)),
        origin=Origin(xyz=(0.0, 0.045, 0.010)),
        material=safety_red,
        name="lever_arm",
    )
    clamp_lever.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.0, 0.102, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="lever_grip",
    )
    clamp_lever.inertial = Inertial.from_geometry(
        Box((0.050, 0.15, 0.12)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.05, 0.05)),
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(0.214, 0.165, 0.027)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "base_to_clamp_bar",
        ArticulationType.PRISMATIC,
        parent=base,
        child=clamp_bar,
        origin=Origin(xyz=(0.190, 0.0, 0.060)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=0.0,
            upper=0.015,
        ),
    )
    model.articulation(
        "base_to_clamp_lever",
        ArticulationType.FIXED,
        parent=base,
        child=clamp_lever,
        origin=Origin(xyz=(0.080, -0.162, 0.028)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    clamp_bar = object_model.get_part("clamp_bar")
    clamp_lever = object_model.get_part("clamp_lever")
    blade_joint = object_model.get_articulation("base_to_blade_arm")
    clamp_slide = object_model.get_articulation("base_to_clamp_bar")

    bed_surface = base.get_visual("bed_surface")
    cutting_strip = base.get_visual("cutting_strip")
    hinge_block = base.get_visual("blade_hinge_block")
    guide_post_front = base.get_visual("guide_post_front")
    guide_post_rear = base.get_visual("guide_post_rear")
    lever_bracket = base.get_visual("clamp_lever_bracket")

    hinge_saddle = blade_arm.get_visual("hinge_saddle")
    blade_edge = blade_arm.get_visual("blade_edge")
    handle_grip = blade_arm.get_visual("handle_grip")

    clamp_pad = clamp_bar.get_visual("clamp_pad")
    guide_shoe_front = clamp_bar.get_visual("guide_shoe_front")
    guide_shoe_rear = clamp_bar.get_visual("guide_shoe_rear")

    lever_pivot = clamp_lever.get_visual("lever_pivot")

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

    ctx.expect_contact(
        blade_arm,
        base,
        elem_a=hinge_saddle,
        elem_b=hinge_block,
        name="blade_arm_hinge_saddle_contacts_hinge_block",
    )
    ctx.expect_contact(
        clamp_bar,
        base,
        elem_a=guide_shoe_front,
        elem_b=guide_post_front,
        name="front_guide_shoe_contacts_front_post",
    )
    ctx.expect_contact(
        clamp_bar,
        base,
        elem_a=guide_shoe_rear,
        elem_b=guide_post_rear,
        name="rear_guide_shoe_contacts_rear_post",
    )
    ctx.expect_contact(
        clamp_lever,
        base,
        elem_a=lever_pivot,
        elem_b=lever_bracket,
        name="clamp_lever_pivot_seated_on_front_bracket",
    )

    with ctx.pose({blade_joint: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem=blade_edge,
            negative_elem=cutting_strip,
            min_gap=0.0,
            max_gap=0.0025,
            name="blade_edge_seats_just_above_cutting_strip",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="y",
            elem_a=blade_edge,
            elem_b=cutting_strip,
            min_overlap=0.28,
            name="blade_edge_runs_along_cutting_strip",
        )

    with ctx.pose({clamp_slide: 0.0}):
        ctx.expect_gap(
            clamp_bar,
            base,
            axis="z",
            positive_elem=clamp_pad,
            negative_elem=bed_surface,
            min_gap=0.015,
            max_gap=0.0175,
            name="clamp_pad_rests_above_bed",
        )
        clamp_pad_rest_aabb = ctx.part_element_world_aabb(clamp_bar, elem=clamp_pad)
        assert clamp_pad_rest_aabb is not None

    with ctx.pose({clamp_slide: 0.015}):
        ctx.expect_gap(
            clamp_bar,
            base,
            axis="z",
            positive_elem=clamp_pad,
            negative_elem=bed_surface,
            min_gap=0.0005,
            max_gap=0.0025,
            name="clamp_pad_lowers_close_to_bed",
        )
        clamp_pad_pressed_aabb = ctx.part_element_world_aabb(clamp_bar, elem=clamp_pad)
        assert clamp_pad_pressed_aabb is not None
        assert clamp_pad_pressed_aabb[0][2] < clamp_pad_rest_aabb[0][2] - 0.013

    handle_rest_aabb = ctx.part_element_world_aabb(blade_arm, elem=handle_grip)
    assert handle_rest_aabb is not None
    with ctx.pose({blade_joint: math.radians(70.0)}):
        handle_open_aabb = ctx.part_element_world_aabb(blade_arm, elem=handle_grip)
        assert handle_open_aabb is not None
        assert handle_open_aabb[1][2] > handle_rest_aabb[1][2] + 0.22

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
