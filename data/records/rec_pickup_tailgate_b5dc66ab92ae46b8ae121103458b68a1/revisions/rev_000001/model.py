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
    model = ArticulatedObject(name="split_tailgate_barn_doors")

    body_paint = model.material("body_paint", rgba=(0.79, 0.82, 0.84, 1.0))
    liner_dark = model.material("liner_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))

    bed_width = 1.72
    stub_depth = 0.28
    floor_thickness = 0.035
    stub_width = 0.11
    stub_height = 0.38
    opening_half_width = 0.75
    hinge_axis_abs_x = 0.767
    hinge_axis_y = -0.018
    hinge_axis_z = 0.205
    leaf_height = 0.33
    leaf_thickness = 0.034
    leaf_extent = 0.763
    leaf_panel_start = 0.028
    leaf_panel_width = leaf_extent - leaf_panel_start

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((bed_width, stub_depth, floor_thickness)),
        origin=Origin(xyz=(0.0, stub_depth / 2.0, floor_thickness / 2.0)),
        material=liner_dark,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((bed_width - 0.12, 0.050, 0.080)),
        origin=Origin(xyz=(0.0, 0.025, 0.040)),
        material=body_paint,
        name="rear_sill",
    )
    bed_frame.visual(
        Box((stub_width, stub_depth, stub_height)),
        origin=Origin(xyz=(-0.805, stub_depth / 2.0, stub_height / 2.0)),
        material=body_paint,
        name="left_stub_outer",
    )
    bed_frame.visual(
        Box((stub_width, stub_depth, stub_height)),
        origin=Origin(xyz=(0.805, stub_depth / 2.0, stub_height / 2.0)),
        material=body_paint,
        name="right_stub_outer",
    )
    bed_frame.visual(
        Box((0.072, stub_depth - 0.030, stub_height - 0.090)),
        origin=Origin(xyz=(-0.800, (stub_depth + 0.030) / 2.0, (stub_height - 0.020) / 2.0)),
        material=liner_dark,
        name="left_stub_liner",
    )
    bed_frame.visual(
        Box((0.072, stub_depth - 0.030, stub_height - 0.090)),
        origin=Origin(xyz=(0.800, (stub_depth + 0.030) / 2.0, (stub_height - 0.020) / 2.0)),
        material=liner_dark,
        name="right_stub_liner",
    )
    bed_frame.visual(
        Box((0.040, 0.030, 0.320)),
        origin=Origin(xyz=(-0.790, 0.015, 0.200)),
        material=body_paint,
        name="left_rear_jamb",
    )
    bed_frame.visual(
        Box((0.040, 0.030, 0.320)),
        origin=Origin(xyz=(0.790, 0.015, 0.200)),
        material=body_paint,
        name="right_rear_jamb",
    )
    bed_frame.visual(
        Box((0.032, 0.026, 0.310)),
        origin=Origin(xyz=(-hinge_axis_abs_x, 0.007, hinge_axis_z)),
        material=hinge_steel,
        name="left_hinge_pad",
    )
    bed_frame.visual(
        Box((0.032, 0.026, 0.310)),
        origin=Origin(xyz=(hinge_axis_abs_x, 0.007, hinge_axis_z)),
        material=hinge_steel,
        name="right_hinge_pad",
    )
    bed_frame.inertial = Inertial.from_geometry(
        Box((bed_width, stub_depth, stub_height)),
        mass=140.0,
        origin=Origin(xyz=(0.0, stub_depth / 2.0, stub_height / 2.0)),
    )

    def add_leaf(name: str, handedness: float):
        leaf = model.part(name)

        leaf.visual(
            Box((leaf_panel_width, leaf_thickness, leaf_height)),
            origin=Origin(xyz=(handedness * (leaf_panel_start + leaf_panel_width / 2.0), 0.0, 0.0)),
            material=body_paint,
            name="outer_skin",
        )
        leaf.visual(
            Box((0.074, leaf_thickness + 0.004, leaf_height)),
            origin=Origin(xyz=(handedness * 0.064, 0.0, 0.0)),
            material=body_paint,
            name="hinge_stile",
        )
        leaf.visual(
            Box((0.034, leaf_thickness + 0.003, leaf_height)),
            origin=Origin(xyz=(handedness * (leaf_extent - 0.017), 0.0, 0.0)),
            material=body_paint,
            name="meeting_edge_stile",
        )
        leaf.visual(
            Box((0.610, 0.018, 0.215)),
            origin=Origin(xyz=(handedness * 0.396, 0.008, 0.0)),
            material=liner_dark,
            name="inner_recess",
        )
        leaf.visual(
            Box((0.585, 0.024, 0.050)),
            origin=Origin(xyz=(handedness * 0.392, 0.002, 0.120)),
            material=body_paint,
            name="top_rail",
        )
        leaf.visual(
            Box((0.585, 0.024, 0.050)),
            origin=Origin(xyz=(handedness * 0.392, 0.002, -0.120)),
            material=body_paint,
            name="bottom_rail",
        )
        leaf.visual(
            Box((0.080, 0.024, 0.215)),
            origin=Origin(xyz=(handedness * (leaf_extent - 0.040), 0.002, 0.0)),
            material=body_paint,
            name="latch_backing",
        )

        for barrel_name, z_pos in (
            ("hinge_barrel_lower", -0.122),
            ("hinge_barrel_mid", 0.0),
            ("hinge_barrel_upper", 0.122),
        ):
            leaf.visual(
                Cylinder(radius=0.012, length=0.072),
                origin=Origin(xyz=(0.0, 0.0, z_pos)),
                material=hinge_steel,
                name=barrel_name,
            )
            leaf.visual(
                Box((0.036, 0.016, 0.064)),
                origin=Origin(xyz=(handedness * 0.018, 0.0, z_pos)),
                material=hinge_steel,
                name=f"{barrel_name}_strap",
            )

        leaf.inertial = Inertial.from_geometry(
            Box((leaf_extent, leaf_thickness + 0.004, leaf_height)),
            mass=18.0,
            origin=Origin(xyz=(handedness * (leaf_extent / 2.0), 0.0, 0.0)),
        )
        return leaf

    left_leaf = add_leaf("left_leaf", 1.0)
    right_leaf = add_leaf("right_leaf", -1.0)

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(),
        material=handle_black,
        name="pivot_boss",
    )
    latch_handle.visual(
        Box((0.072, 0.014, 0.018)),
        origin=Origin(xyz=(0.036, -0.012, 0.0)),
        material=handle_black,
        name="handle_grip",
    )
    latch_handle.visual(
        Box((0.020, 0.024, 0.026)),
        origin=Origin(xyz=(0.066, -0.015, 0.0)),
        material=handle_black,
        name="finger_pull",
    )
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.090, 0.030, 0.052)),
        mass=0.6,
        origin=Origin(xyz=(0.045, -0.012, 0.0)),
    )

    left_hinge = model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=left_leaf,
        origin=Origin(xyz=(-hinge_axis_abs_x, hinge_axis_y, hinge_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    right_hinge = model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=right_leaf,
        origin=Origin(xyz=(hinge_axis_abs_x, hinge_axis_y, hinge_axis_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=right_leaf,
        child=latch_handle,
        origin=Origin(xyz=(-0.718, -0.029, 0.015)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    bed_frame = object_model.get_part("bed_frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    latch_handle = object_model.get_part("latch_handle")

    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    left_pad = bed_frame.get_visual("left_hinge_pad")
    right_pad = bed_frame.get_visual("right_hinge_pad")
    bed_floor = bed_frame.get_visual("bed_floor")
    left_mid_barrel = left_leaf.get_visual("hinge_barrel_mid")
    right_mid_barrel = right_leaf.get_visual("hinge_barrel_mid")
    right_skin = right_leaf.get_visual("outer_skin")
    handle_boss = latch_handle.get_visual("pivot_boss")

    ctx.check(
        "left_hinge_axis_vertical",
        left_hinge.axis == (0.0, 0.0, 1.0),
        details=f"expected +Z hinge axis, got {left_hinge.axis}",
    )
    ctx.check(
        "right_hinge_axis_vertical",
        right_hinge.axis == (0.0, 0.0, -1.0),
        details=f"expected -Z hinge axis, got {right_hinge.axis}",
    )
    ctx.check(
        "handle_axis_vertical",
        handle_pivot.axis == (0.0, 0.0, 1.0),
        details=f"expected +Z handle axis, got {handle_pivot.axis}",
    )

    ctx.expect_contact(
        left_leaf,
        bed_frame,
        elem_a=left_mid_barrel,
        elem_b=left_pad,
        name="left_leaf_contacts_hinge_pad_closed",
    )
    ctx.expect_contact(
        right_leaf,
        bed_frame,
        elem_a=right_mid_barrel,
        elem_b=right_pad,
        name="right_leaf_contacts_hinge_pad_closed",
    )
    ctx.expect_contact(
        latch_handle,
        right_leaf,
        elem_a=handle_boss,
        elem_b=right_skin,
        name="handle_contacts_right_leaf_closed",
    )
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        min_gap=0.004,
        max_gap=0.012,
        name="center_split_gap_closed",
    )
    ctx.expect_gap(
        left_leaf,
        bed_frame,
        axis="z",
        min_gap=0.004,
        max_gap=0.010,
        negative_elem=bed_floor,
        name="left_leaf_bottom_clearance",
    )
    ctx.expect_gap(
        right_leaf,
        bed_frame,
        axis="z",
        min_gap=0.004,
        max_gap=0.010,
        negative_elem=bed_floor,
        name="right_leaf_bottom_clearance",
    )

    left_rest_aabb = ctx.part_world_aabb(left_leaf)
    right_rest_aabb = ctx.part_world_aabb(right_leaf)
    handle_rest_aabb = ctx.part_world_aabb(latch_handle)
    assert left_rest_aabb is not None
    assert right_rest_aabb is not None
    assert handle_rest_aabb is not None

    with ctx.pose({left_hinge: math.radians(95.0), right_hinge: math.radians(95.0)}):
        left_open_aabb = ctx.part_world_aabb(left_leaf)
        right_open_aabb = ctx.part_world_aabb(right_leaf)
        assert left_open_aabb is not None
        assert right_open_aabb is not None
        assert left_open_aabb[1][1] > left_rest_aabb[1][1] + 0.55
        assert right_open_aabb[1][1] > right_rest_aabb[1][1] + 0.55
        ctx.expect_contact(
            left_leaf,
            bed_frame,
            elem_a=left_mid_barrel,
            elem_b=left_pad,
            name="left_leaf_stays_clipped_to_hinge_open",
        )
        ctx.expect_contact(
            right_leaf,
            bed_frame,
            elem_a=right_mid_barrel,
            elem_b=right_pad,
            name="right_leaf_stays_clipped_to_hinge_open",
        )

    with ctx.pose({handle_pivot: -0.45}):
        handle_rotated_aabb = ctx.part_world_aabb(latch_handle)
        assert handle_rotated_aabb is not None
        assert handle_rotated_aabb[0][1] < handle_rest_aabb[0][1] - 0.006
        ctx.expect_contact(
            latch_handle,
            right_leaf,
            elem_a=handle_boss,
            elem_b=right_skin,
            name="handle_stays_mounted_while_rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
