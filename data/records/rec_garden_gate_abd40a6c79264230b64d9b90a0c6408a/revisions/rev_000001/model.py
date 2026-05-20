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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hinge_knuckle_mesh(name: str, *, inner_radius: float, outer_radius: float, height: float):
    start_angle = math.radians(120.0)
    end_angle = -start_angle
    outer_points = []
    inner_points = []
    segments = 18
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + ((end_angle - start_angle) * t)
        outer_points.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle)))
    for index in range(segments + 1):
        t = index / segments
        angle = end_angle + ((start_angle - end_angle) * t)
        inner_points.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle)))
    profile = outer_points + inner_points
    return mesh_from_geometry(
        ExtrudeGeometry(profile, height, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_gate")

    cedar = model.material("cedar", rgba=(0.55, 0.37, 0.21, 1.0))
    painted_panel = model.material("painted_panel", rgba=(0.84, 0.86, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.23, 0.25, 0.27, 1.0))
    concrete = model.material("concrete", rgba=(0.56, 0.56, 0.54, 1.0))
    black_hardware = model.material("black_hardware", rgba=(0.08, 0.08, 0.09, 1.0))

    post_size = 0.09
    post_top = 1.00
    footing_depth = 0.12
    hinge_axis_x = 0.0125

    gate_bottom = 0.08
    gate_top = 0.90
    gate_thickness = 0.035
    gate_width = 0.922
    stile_width = 0.040
    rail_height = 0.045
    mid_rail_bottom = 0.500
    mid_rail_top = mid_rail_bottom + rail_height
    top_rail_bottom = gate_top - rail_height

    pin_radius = 0.0055
    knuckle_inner_radius = 0.0075
    knuckle_outer_radius = 0.0115
    knuckle_height = 0.090
    upper_hinge_z = 0.720
    lower_hinge_z = 0.180

    right_post_inner_face = 0.950
    right_post_center_x = right_post_inner_face + (post_size * 0.5)
    post_center_z = ((post_top - footing_depth) * 0.5)

    knuckle_mesh = _hinge_knuckle_mesh(
        "gate_hinge_knuckle",
        inner_radius=knuckle_inner_radius,
        outer_radius=knuckle_outer_radius,
        height=knuckle_height,
    )

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((post_size, post_size, post_top + footing_depth)),
        origin=Origin(xyz=(-post_size * 0.5, 0.0, post_center_z)),
        material=cedar,
        name="left_post",
    )
    support_frame.visual(
        Box((post_size, post_size, post_top + footing_depth)),
        origin=Origin(xyz=(right_post_center_x, 0.0, post_center_z)),
        material=cedar,
        name="right_post",
    )
    support_frame.visual(
        Box((right_post_center_x + (post_size * 0.5), 0.060, footing_depth)),
        origin=Origin(
            xyz=((right_post_center_x - (post_size * 0.5)) * 0.5, 0.0, -footing_depth * 0.5)
        ),
        material=concrete,
        name="ground_tie",
    )
    support_frame.visual(
        Box((0.100, 0.100, 0.018)),
        origin=Origin(xyz=(-post_size * 0.5, 0.0, post_top + 0.009)),
        material=cedar,
        name="left_cap",
    )
    support_frame.visual(
        Box((0.100, 0.100, 0.018)),
        origin=Origin(xyz=(right_post_center_x, 0.0, post_top + 0.009)),
        material=cedar,
        name="right_cap",
    )
    for name, z_pos in (("upper", upper_hinge_z), ("lower", lower_hinge_z)):
        support_frame.visual(
            Box((0.012, 0.050, 0.100)),
            origin=Origin(xyz=(-0.006, 0.0, z_pos)),
            material=steel,
            name=f"{name}_hinge_plate",
        )
        support_frame.visual(
            Box((0.015, 0.018, knuckle_height)),
            origin=Origin(xyz=(0.0005, 0.0, z_pos)),
            material=steel,
            name=f"{name}_hinge_boss",
        )
        support_frame.visual(
            Cylinder(radius=pin_radius, length=knuckle_height),
            origin=Origin(xyz=(hinge_axis_x, 0.0, z_pos)),
            material=steel,
            name=f"{name}_pin",
        )
    support_frame.visual(
        Box((0.012, 0.048, 0.120)),
        origin=Origin(xyz=(0.944, 0.0, 0.560)),
        material=steel,
        name="keeper_plate",
    )
    support_frame.visual(
        Box((0.014, 0.028, 0.085)),
        origin=Origin(xyz=(0.937, 0.0, 0.560)),
        material=black_hardware,
        name="striker_block",
    )

    leaf = model.part("leaf")
    leaf.visual(
        Box((stile_width, gate_thickness, gate_top - gate_bottom)),
        origin=Origin(
            xyz=(0.018 + (stile_width * 0.5), 0.0, gate_bottom + ((gate_top - gate_bottom) * 0.5))
        ),
        material=cedar,
        name="left_stile",
    )
    leaf.visual(
        Box((stile_width, gate_thickness, gate_top - gate_bottom)),
        origin=Origin(
            xyz=(gate_width - (stile_width * 0.5), 0.0, gate_bottom + ((gate_top - gate_bottom) * 0.5))
        ),
        material=cedar,
        name="right_stile",
    )
    rail_width = gate_width - 0.076
    rail_center_x = gate_width * 0.5
    leaf.visual(
        Box((rail_width, gate_thickness, rail_height)),
        origin=Origin(xyz=(rail_center_x, 0.0, gate_bottom + (rail_height * 0.5))),
        material=cedar,
        name="bottom_rail",
    )
    leaf.visual(
        Box((rail_width, gate_thickness, rail_height)),
        origin=Origin(xyz=(rail_center_x, 0.0, (mid_rail_bottom + mid_rail_top) * 0.5)),
        material=cedar,
        name="mid_rail",
    )
    leaf.visual(
        Box((rail_width, gate_thickness, rail_height)),
        origin=Origin(xyz=(rail_center_x, 0.0, gate_top - (rail_height * 0.5))),
        material=cedar,
        name="top_rail",
    )
    leaf.visual(
        Box((gate_width - 0.098, 0.018, mid_rail_bottom - gate_bottom - 0.002)),
        origin=Origin(
            xyz=(
                rail_center_x,
                0.0,
                (gate_bottom + rail_height + mid_rail_bottom) * 0.5 - 0.001,
            )
        ),
        material=painted_panel,
        name="lower_panel",
    )

    lattice_bars = (
        ("lattice_rise_0", (0.048, mid_rail_top - 0.003), (0.380, top_rail_bottom + 0.003)),
        ("lattice_rise_1", (0.300, mid_rail_top - 0.003), (0.632, top_rail_bottom + 0.003)),
        ("lattice_rise_2", (0.552, mid_rail_top - 0.003), (0.884, top_rail_bottom + 0.003)),
        ("lattice_fall_0", (0.048, top_rail_bottom + 0.003), (0.380, mid_rail_top - 0.003)),
        ("lattice_fall_1", (0.300, top_rail_bottom + 0.003), (0.632, mid_rail_top - 0.003)),
        ("lattice_fall_2", (0.552, top_rail_bottom + 0.003), (0.884, mid_rail_top - 0.003)),
    )
    for name, start, end in lattice_bars:
        dx = end[0] - start[0]
        dz = end[1] - start[1]
        leaf.visual(
            Box((math.hypot(dx, dz) + 0.012, 0.010, 0.016)),
            origin=Origin(
                xyz=((start[0] + end[0]) * 0.5, 0.0, (start[1] + end[1]) * 0.5),
                rpy=(0.0, -math.atan2(dz, dx), 0.0),
            ),
            material=cedar,
            name=name,
        )

    for name, z_pos in (("upper", upper_hinge_z), ("lower", lower_hinge_z)):
        leaf.visual(
            Box((0.090, gate_thickness, 0.040)),
            origin=Origin(xyz=(0.055, 0.0, z_pos)),
            material=steel,
            name=f"{name}_hinge_strap",
        )
        leaf.visual(
            knuckle_mesh,
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=steel,
            name=f"{name}_knuckle",
        )

    leaf.visual(
        Box((0.050, 0.024, 0.100)),
        origin=Origin(xyz=(0.892, 0.0, 0.560)),
        material=black_hardware,
        name="latch_case",
    )
    leaf.visual(
        Box((0.026, 0.008, 0.090)),
        origin=Origin(xyz=(0.892, (gate_thickness * 0.5) + 0.004, 0.560)),
        material=steel,
        name="latch_plate",
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=black_hardware,
        name="hub",
    )
    latch_handle.visual(
        Box((0.018, 0.006, 0.060)),
        origin=Origin(xyz=(0.012, 0.0, -0.006)),
        material=black_hardware,
        name="turn_bar",
    )
    latch_handle.visual(
        Box((0.028, 0.006, 0.012)),
        origin=Origin(xyz=(0.021, 0.0, -0.031)),
        material=black_hardware,
        name="finger_tab",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=leaf,
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "latch_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=latch_handle,
        origin=Origin(xyz=(0.892, (gate_thickness * 0.5) + 0.012, 0.560)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=-1.1,
            upper=1.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    leaf = object_model.get_part("leaf")
    latch_handle = object_model.get_part("latch_handle")
    leaf_hinge = object_model.get_articulation("leaf_hinge")
    latch_handle_pivot = object_model.get_articulation("latch_handle_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "leaf_hinge_axis_vertical",
        tuple(leaf_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical hinge axis, got {leaf_hinge.axis!r}",
    )
    ctx.check(
        "latch_handle_axis_transverse",
        tuple(latch_handle_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"Expected latch handle axis along gate thickness, got {latch_handle_pivot.axis!r}",
    )

    ctx.expect_within(
        support_frame,
        leaf,
        axes="xy",
        inner_elem="upper_pin",
        outer_elem="upper_knuckle",
        name="upper_hinge_pin_captured",
    )
    ctx.expect_overlap(
        leaf,
        support_frame,
        axes="z",
        elem_a="upper_knuckle",
        elem_b="upper_pin",
        min_overlap=0.08,
        name="upper_hinge_pin_height_overlap",
    )
    ctx.expect_within(
        support_frame,
        leaf,
        axes="xy",
        inner_elem="lower_pin",
        outer_elem="lower_knuckle",
        name="lower_hinge_pin_captured",
    )
    ctx.expect_overlap(
        leaf,
        support_frame,
        axes="z",
        elem_a="lower_knuckle",
        elem_b="lower_pin",
        min_overlap=0.08,
        name="lower_hinge_pin_height_overlap",
    )
    ctx.expect_gap(
        support_frame,
        leaf,
        axis="x",
        positive_elem="right_post",
        negative_elem="right_stile",
        min_gap=0.010,
        max_gap=0.025,
        name="leaf_closing_gap_to_latch_post",
    )
    ctx.expect_contact(
        latch_handle,
        leaf,
        elem_a="hub",
        elem_b="latch_plate",
        name="latch_handle_seated_on_plate",
    )

    open_angle = math.radians(85.0)
    with ctx.pose({leaf_hinge: open_angle}):
        ctx.expect_within(
            support_frame,
            leaf,
            axes="xy",
            inner_elem="upper_pin",
            outer_elem="upper_knuckle",
            name="upper_hinge_stays_engaged_open",
        )
        ctx.expect_within(
            support_frame,
            leaf,
            axes="xy",
            inner_elem="lower_pin",
            outer_elem="lower_knuckle",
            name="lower_hinge_stays_engaged_open",
        )
        open_stile_aabb = ctx.part_element_world_aabb(leaf, elem="right_stile")
        assert open_stile_aabb is not None
        ctx.check(
            "leaf_swings_out_from_posts",
            open_stile_aabb[0][1] > 0.80,
            details=f"Expected opened leaf to sweep into +Y, got AABB {open_stile_aabb!r}",
        )

    rest_bar_aabb = ctx.part_element_world_aabb(latch_handle, elem="turn_bar")
    assert rest_bar_aabb is not None
    rest_bar_dx = rest_bar_aabb[1][0] - rest_bar_aabb[0][0]
    with ctx.pose({latch_handle_pivot: 1.0}):
        turned_bar_aabb = ctx.part_element_world_aabb(latch_handle, elem="turn_bar")
        assert turned_bar_aabb is not None
        turned_bar_dx = turned_bar_aabb[1][0] - turned_bar_aabb[0][0]
        ctx.check(
            "latch_handle_rotates",
            turned_bar_dx > rest_bar_dx + 0.020,
            details=(
                "Expected latch turn bar to widen in X when rotated; "
                f"rest span={rest_bar_dx:.4f}, turned span={turned_bar_dx:.4f}"
            ),
        )
        ctx.expect_contact(
            latch_handle,
            leaf,
            elem_a="hub",
            elem_b="latch_plate",
            name="latch_handle_stays_seated_when_turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
