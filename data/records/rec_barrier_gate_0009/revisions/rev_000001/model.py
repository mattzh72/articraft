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
    model = ArticulatedObject(name="checkpoint_barrier_gate")

    concrete = model.material("concrete", rgba=(0.63, 0.63, 0.61, 1.0))
    cabinet_gray = model.material("cabinet_gray", rgba=(0.20, 0.22, 0.25, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.36, 0.38, 0.41, 1.0))
    arm_white = model.material("arm_white", rgba=(0.96, 0.96, 0.93, 1.0))
    signal_red = model.material("signal_red", rgba=(0.78, 0.10, 0.10, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.10, 1.0))

    slab_length = 1.30
    slab_width = 0.70
    slab_height = 0.18

    cabinet_length = 0.50
    cabinet_width = 0.36
    cabinet_height = 0.82
    cabinet_center_x = -0.12
    cabinet_top_z = slab_height + cabinet_height

    pivot_z = 1.14
    bracket_height = 0.16
    bracket_length = 0.10
    bracket_thickness = 0.015
    bracket_y = 0.055

    arm_width = 0.08
    arm_height = 0.10
    segment_length = 0.40
    segment_count = 9
    arm_front_start = 0.03
    rear_stub_length = 0.72
    hub_radius = 0.045
    hub_length = 0.095

    base = model.part("base")
    base.visual(
        Box((slab_length, slab_width, slab_height)),
        origin=Origin(xyz=(cabinet_center_x, 0.0, slab_height / 2.0)),
        material=concrete,
        name="concrete_base",
    )
    base.visual(
        Box((cabinet_length, cabinet_width, cabinet_height)),
        origin=Origin(
            xyz=(cabinet_center_x, 0.0, slab_height + cabinet_height / 2.0)
        ),
        material=cabinet_gray,
        name="operator_cabinet",
    )
    base.visual(
        Box((0.16, 0.24, 0.08)),
        origin=Origin(xyz=(-0.12, 0.0, cabinet_top_z + 0.04)),
        material=steel_gray,
        name="pivot_post",
    )
    base.visual(
        Box((bracket_length, bracket_thickness, bracket_height)),
        origin=Origin(xyz=(0.0, bracket_y, pivot_z)),
        material=steel_gray,
        name="left_bracket",
    )
    base.visual(
        Box((bracket_length, bracket_thickness, bracket_height)),
        origin=Origin(xyz=(0.0, -bracket_y, pivot_z)),
        material=steel_gray,
        name="right_bracket",
    )

    boom = model.part("boom_assembly")
    boom.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=steel_gray,
        name="hinge_hub",
    )
    boom.visual(
        Box((rear_stub_length, arm_width, arm_height)),
        origin=Origin(xyz=(-0.39, 0.0, 0.0)),
        material=steel_gray,
        name="rear_stub",
    )

    for index in range(segment_count):
        segment_center_x = arm_front_start + (segment_length / 2.0) + index * segment_length
        boom.visual(
            Box((segment_length, arm_width, arm_height)),
            origin=Origin(xyz=(segment_center_x, 0.0, 0.0)),
            material=arm_white if index % 2 == 0 else signal_red,
            name=f"arm_segment_{index}",
        )

    boom.visual(
        Box((0.08, 0.09, 0.12)),
        origin=Origin(xyz=(3.67, 0.0, 0.0)),
        material=matte_black,
        name="boom_tip",
    )
    boom.visual(
        Box((0.26, 0.18, 0.18)),
        origin=Origin(xyz=(-0.58, 0.0, -0.14)),
        material=matte_black,
        name="counterweight",
    )

    model.articulation(
        "barrier_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=1.5,
            lower=-1.35,
            upper=0.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    boom = object_model.get_part("boom_assembly")
    hinge = object_model.get_articulation("barrier_hinge")

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

    ctx.check(
        "boom_part_present",
        boom is not None,
        "boom_assembly part should exist",
    )
    ctx.check(
        "hinge_axis_is_lateral",
        hinge.axis == (0.0, 1.0, 0.0),
        f"expected hinge axis (0, 1, 0), got {hinge.axis!r}",
    )
    ctx.check(
        "hinge_has_realistic_travel",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower is not None
        and hinge.motion_limits.upper is not None
        and (hinge.motion_limits.upper - hinge.motion_limits.lower) > 1.2,
        "barrier arm should have a broad up/down travel range",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            boom,
            base,
            elem_a="hinge_hub",
            elem_b="left_bracket",
            name="hinge_hub_contacts_left_bracket",
        )
        ctx.expect_contact(
            boom,
            base,
            elem_a="hinge_hub",
            elem_b="right_bracket",
            name="hinge_hub_contacts_right_bracket",
        )
        ctx.expect_gap(
            boom,
            base,
            axis="z",
            positive_elem="arm_segment_0",
            negative_elem="operator_cabinet",
            min_gap=0.07,
            max_gap=0.11,
            name="closed_boom_clears_cabinet_roof",
        )
        counterweight_aabb = ctx.part_element_world_aabb(boom, elem="counterweight")
        ctx.check(
            "counterweight_sits_behind_pivot",
            counterweight_aabb is not None and counterweight_aabb[1][0] < -0.40,
            f"counterweight should stay behind the pivot, got {counterweight_aabb!r}",
        )

        closed_tip_aabb = ctx.part_element_world_aabb(boom, elem="boom_tip")
        closed_tip_center_z = (
            (closed_tip_aabb[0][2] + closed_tip_aabb[1][2]) / 2.0
            if closed_tip_aabb is not None
            else None
        )

    with ctx.pose({hinge: -1.20}):
        open_tip_aabb = ctx.part_element_world_aabb(boom, elem="boom_tip")
        open_tip_center_z = (
            (open_tip_aabb[0][2] + open_tip_aabb[1][2]) / 2.0
            if open_tip_aabb is not None
            else None
        )
        ctx.check(
            "boom_tip_rises_when_opened",
            closed_tip_center_z is not None
            and open_tip_center_z is not None
            and open_tip_center_z > closed_tip_center_z + 2.8,
            f"expected open tip z to rise well above closed pose; "
            f"closed={closed_tip_center_z!r}, open={open_tip_center_z!r}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
