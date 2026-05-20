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
    TorusGeometry,
    mesh_from_geometry,
)


def _half_arch_height(progress: float, outer_top: float, center_top: float) -> float:
    t = max(0.0, min(1.0, progress))
    return outer_top + (center_top - outer_top) * (2.0 * t - t * t)


def _full_arch_height(progress: float, outer_z: float, center_z: float) -> float:
    t = max(0.0, min(1.0, progress))
    return outer_z + (center_z - outer_z) * 4.0 * t * (1.0 - t)


def _add_box_arch(
    part,
    *,
    x_points: list[float],
    z_points: list[float],
    depth: float,
    height: float,
    material,
    name_prefix: str,
) -> None:
    for index, (x0, x1, z0, z1) in enumerate(zip(x_points[:-1], x_points[1:], z_points[:-1], z_points[1:])):
        dx = x1 - x0
        dz = z1 - z0
        length = math.hypot(dx, dz) + 0.018
        angle = math.atan2(dz, dx)
        part.visual(
            Box((length, depth, height)),
            origin=Origin(xyz=((x0 + x1) * 0.5, 0.0, (z0 + z1) * 0.5), rpy=(0.0, angle, 0.0)),
            material=material,
            name=f"{name_prefix}_{index}",
        )


def _build_gate_leaf(
    model: ArticulatedObject,
    *,
    name: str,
    side_sign: float,
    leaf_width: float,
    leaf_thickness: float,
    stile_width: float,
    rail_height: float,
    leaf_bottom: float,
    outer_top: float,
    center_top: float,
    gate_material,
    hardware_material,
    add_latch: bool,
    add_bolt_guides: bool,
) -> None:
    leaf = model.part(name)

    outer_stile_height = outer_top - leaf_bottom
    meeting_stile_height = center_top - leaf_bottom
    outer_stile_center_x = side_sign * (stile_width * 0.5)
    meeting_stile_center_x = side_sign * (leaf_width - stile_width * 0.5)
    mid_rail_z = 1.02

    leaf.visual(
        Box((stile_width, leaf_thickness, outer_stile_height)),
        origin=Origin(xyz=(outer_stile_center_x, 0.0, leaf_bottom + outer_stile_height * 0.5)),
        material=gate_material,
        name="outer_stile",
    )
    leaf.visual(
        Box((stile_width, leaf_thickness, meeting_stile_height)),
        origin=Origin(xyz=(meeting_stile_center_x, 0.0, leaf_bottom + meeting_stile_height * 0.5)),
        material=gate_material,
        name="meeting_stile",
    )
    leaf.visual(
        Box((leaf_width - stile_width, leaf_thickness, rail_height)),
        origin=Origin(xyz=(side_sign * (leaf_width * 0.5), 0.0, leaf_bottom + rail_height * 0.5)),
        material=gate_material,
        name="bottom_rail",
    )
    leaf.visual(
        Box((leaf_width - stile_width * 1.2, leaf_thickness, 0.035)),
        origin=Origin(xyz=(side_sign * (leaf_width * 0.5), 0.0, mid_rail_z)),
        material=gate_material,
        name="lock_rail",
    )

    arch_samples = [stile_width * 0.5 + frac * (leaf_width - stile_width) for frac in (0.0, 0.18, 0.38, 0.62, 0.82, 1.0)]
    x_points = [side_sign * sample for sample in arch_samples]
    z_points = [
        _half_arch_height(sample / leaf_width, outer_top, center_top) - rail_height * 0.5
        for sample in arch_samples
    ]
    _add_box_arch(
        leaf,
        x_points=x_points,
        z_points=z_points,
        depth=leaf_thickness,
        height=rail_height,
        material=gate_material,
        name_prefix="top_rail",
    )
    for connector_index, (node_x, node_z) in enumerate(zip(x_points[1:-1], z_points[1:-1]), start=1):
        leaf.visual(
            Box((0.040, leaf_thickness, rail_height * 0.92)),
            origin=Origin(xyz=(node_x, 0.0, node_z)),
            material=gate_material,
            name=f"arch_connector_{connector_index}",
        )

    for bar_index, bar_fraction in enumerate((0.23, 0.47, 0.71)):
        bar_x = side_sign * (bar_fraction * leaf_width)
        bar_top = _half_arch_height(abs(bar_x) / leaf_width, outer_top, center_top) - rail_height * 1.02
        bar_bottom = leaf_bottom + rail_height * 0.98
        bar_length = max(0.20, bar_top - bar_bottom)
        leaf.visual(
            Cylinder(radius=0.0075, length=bar_length),
            origin=Origin(xyz=(bar_x, 0.0, bar_bottom + bar_length * 0.5)),
            material=hardware_material,
            name=f"bar_{bar_index}",
        )

    for hinge_z in (0.46, 1.22):
        leaf.visual(
            Box((0.11, 0.010, 0.040)),
            origin=Origin(xyz=(side_sign * 0.055, 0.020, hinge_z)),
            material=hardware_material,
            name=f"hinge_strap_{int(hinge_z * 100)}",
        )

    leaf.visual(
        Cylinder(radius=0.012, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
        material=hardware_material,
        name="hinge_knuckle",
    )

    if add_latch:
        latch_case_x = side_sign * (leaf_width - 0.060)
        leaf.visual(
            Box((0.075, 0.012, 0.050)),
            origin=Origin(xyz=(latch_case_x, 0.021, 1.05)),
            material=hardware_material,
            name="latch_case",
        )
        leaf.visual(
            Box((0.035, 0.012, 0.090)),
            origin=Origin(xyz=(latch_case_x - side_sign * 0.012, 0.021, 1.01)),
            material=hardware_material,
            name="latch_backplate",
        )

    if add_bolt_guides:
        guide_mesh = mesh_from_geometry(
            TorusGeometry(radius=0.014, tube=0.003, radial_segments=18, tubular_segments=36),
            f"{name}_guide_loop",
        )
        meeting_x = side_sign * (leaf_width - 0.028)
        leaf.visual(
            guide_mesh,
            origin=Origin(xyz=(meeting_x, 0.030, 1.16)),
            material=hardware_material,
            name="upper_guide",
        )
        leaf.visual(
            guide_mesh,
            origin=Origin(xyz=(meeting_x, 0.030, 0.62)),
            material=hardware_material,
            name="lower_guide",
        )
        leaf.visual(
            Box((0.045, 0.012, 0.045)),
            origin=Origin(xyz=(side_sign * (leaf_width - 0.024), 0.021, 1.05)),
            material=hardware_material,
            name="latch_keeper",
        )

    leaf.inertial = Inertial.from_geometry(
        Box((leaf_width, 0.08, center_top - leaf_bottom)),
        mass=18.0,
        origin=Origin(xyz=(side_sign * (leaf_width * 0.5), 0.0, leaf_bottom + (center_top - leaf_bottom) * 0.5)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_courtyard_gate")

    frame_black = model.material("frame_black", rgba=(0.14, 0.14, 0.14, 1.0))
    gate_black = model.material("gate_black", rgba=(0.18, 0.18, 0.18, 1.0))
    hardware = model.material("hardware", rgba=(0.30, 0.31, 0.33, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.54, 0.55, 0.57, 1.0))
    latch_bronze = model.material("latch_bronze", rgba=(0.54, 0.46, 0.30, 1.0))

    post_width = 0.12
    post_depth = 0.14
    post_height = 2.10
    opening_width = 1.80
    side_gap = 0.008
    center_gap = 0.008
    leaf_width = (opening_width - 2.0 * side_gap - center_gap) * 0.5
    leaf_thickness = 0.030
    stile_width = 0.045
    rail_height = 0.045
    leaf_bottom = 0.08
    outer_top = 1.62
    center_top = 1.75
    post_center_x = opening_width * 0.5 + post_width * 0.5
    hinge_x = opening_width * 0.5 - side_gap

    frame = model.part("frame")
    frame.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(-post_center_x, 0.0, post_height * 0.5)),
        material=frame_black,
        name="left_post",
    )
    frame.visual(
        Box((post_width, post_depth, post_height)),
        origin=Origin(xyz=(post_center_x, 0.0, post_height * 0.5)),
        material=frame_black,
        name="right_post",
    )
    frame.visual(
        Box((post_width + 0.030, post_depth + 0.030, 0.045)),
        origin=Origin(xyz=(-post_center_x, 0.0, post_height + 0.0225)),
        material=frame_black,
        name="left_cap",
    )
    frame.visual(
        Box((post_width + 0.030, post_depth + 0.030, 0.045)),
        origin=Origin(xyz=(post_center_x, 0.0, post_height + 0.0225)),
        material=frame_black,
        name="right_cap",
    )

    arch_beam_height = 0.090
    arch_start = -opening_width * 0.5 - 0.015
    arch_end = opening_width * 0.5 + 0.015
    arch_samples = [arch_start + (arch_end - arch_start) * frac for frac in (0.0, 0.16, 0.32, 0.50, 0.68, 0.84, 1.0)]
    arch_z = [_full_arch_height((x - arch_start) / (arch_end - arch_start), 1.84, 1.93) for x in arch_samples]
    _add_box_arch(
        frame,
        x_points=arch_samples,
        z_points=arch_z,
        depth=post_depth,
        height=arch_beam_height,
        material=frame_black,
        name_prefix="arch_beam",
    )
    frame.visual(
        Box((0.090, post_depth, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 1.92)),
        material=frame_black,
        name="keystone",
    )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        frame.visual(
            Cylinder(radius=0.014, length=0.24),
            origin=Origin(xyz=(side_sign * (hinge_x + 0.014), 0.0, 0.44)),
            material=hardware,
            name=f"{side_name}_hinge_lower",
        )
        frame.visual(
            Cylinder(radius=0.014, length=0.24),
            origin=Origin(xyz=(side_sign * (hinge_x + 0.014), 0.0, 1.24)),
            material=hardware,
            name=f"{side_name}_hinge_upper",
        )

    frame.inertial = Inertial.from_geometry(
        Box((opening_width + 2.0 * post_width, post_depth, post_height + 0.18)),
        mass=70.0,
        origin=Origin(xyz=(0.0, 0.0, (post_height + 0.18) * 0.5)),
    )

    _build_gate_leaf(
        model,
        name="left_leaf",
        side_sign=1.0,
        leaf_width=leaf_width,
        leaf_thickness=leaf_thickness,
        stile_width=stile_width,
        rail_height=rail_height,
        leaf_bottom=leaf_bottom,
        outer_top=outer_top,
        center_top=center_top,
        gate_material=gate_black,
        hardware_material=hardware,
        add_latch=True,
        add_bolt_guides=False,
    )
    _build_gate_leaf(
        model,
        name="right_leaf",
        side_sign=-1.0,
        leaf_width=leaf_width,
        leaf_thickness=leaf_thickness,
        stile_width=stile_width,
        rail_height=rail_height,
        leaf_bottom=leaf_bottom,
        outer_top=outer_top,
        center_top=center_top,
        gate_material=gate_black,
        hardware_material=hardware,
        add_latch=False,
        add_bolt_guides=True,
    )

    latch_lever = model.part("latch_lever")
    latch_lever.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=hardware,
        name="pivot_boss",
    )
    latch_lever.visual(
        Box((0.050, 0.012, 0.014)),
        origin=Origin(xyz=(0.025, 0.010, 0.0)),
        material=latch_bronze,
        name="lever_bar",
    )
    latch_lever.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.046, 0.010, -0.014)),
        material=latch_bronze,
        name="lever_tip",
    )
    latch_lever.inertial = Inertial.from_geometry(
        Box((0.056, 0.020, 0.040)),
        mass=0.35,
        origin=Origin(xyz=(0.028, 0.010, -0.004)),
    )

    cane_bolt = model.part("cane_bolt")
    cane_bolt.visual(
        Cylinder(radius=0.007, length=1.40),
        origin=Origin(xyz=(0.0, 0.0, -0.48)),
        material=bolt_steel,
        name="bolt_rod",
    )
    cane_bolt.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=hardware,
        name="upper_stop",
    )
    cane_bolt.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.549)),
        material=hardware,
        name="lower_stop",
    )
    cane_bolt.visual(
        Cylinder(radius=0.005, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.20), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=bolt_steel,
        name="bolt_handle",
    )
    cane_bolt.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=1.44),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.48)),
    )

    left_leaf = model.get_part("left_leaf")
    right_leaf = model.get_part("right_leaf")

    model.articulation(
        "left_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_leaf,
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "right_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_leaf,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=1.5,
            lower=-math.radians(105.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=left_leaf,
        child=latch_lever,
        origin=Origin(xyz=(leaf_width - 0.060, 0.027, 1.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-0.45,
            upper=0.35,
        ),
    )
    model.articulation(
        "cane_bolt_slide",
        ArticulationType.PRISMATIC,
        parent=right_leaf,
        child=cane_bolt,
        origin=Origin(xyz=(-(leaf_width - 0.028), 0.030, 1.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.35,
            lower=0.0,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")
    latch_lever = object_model.get_part("latch_lever")
    cane_bolt = object_model.get_part("cane_bolt")

    left_hinge = object_model.get_articulation("left_leaf_hinge")
    right_hinge = object_model.get_articulation("right_leaf_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")
    bolt_slide = object_model.get_articulation("cane_bolt_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("left_hinge_axis_vertical", left_hinge.axis == (0.0, 0.0, 1.0), details=str(left_hinge.axis))
    ctx.check("right_hinge_axis_vertical", right_hinge.axis == (0.0, 0.0, 1.0), details=str(right_hinge.axis))
    ctx.check("latch_axis_out_of_gate_plane", latch_pivot.axis == (0.0, 1.0, 0.0), details=str(latch_pivot.axis))
    ctx.check("bolt_axis_vertical", bolt_slide.axis == (0.0, 0.0, 1.0), details=str(bolt_slide.axis))

    ctx.expect_contact(left_leaf, frame, name="left_leaf_hinged_to_frame")
    ctx.expect_contact(right_leaf, frame, name="right_leaf_hinged_to_frame")
    ctx.expect_contact(latch_lever, left_leaf, name="latch_lever_mounted_to_left_leaf")
    ctx.expect_contact(cane_bolt, right_leaf, name="cane_bolt_captured_on_right_leaf")

    ctx.expect_gap(
        left_leaf,
        frame,
        axis="x",
        positive_elem="outer_stile",
        negative_elem="left_post",
        min_gap=0.006,
        max_gap=0.010,
        name="left_leaf_post_clearance",
    )
    ctx.expect_gap(
        frame,
        right_leaf,
        axis="x",
        positive_elem="right_post",
        negative_elem="outer_stile",
        min_gap=0.006,
        max_gap=0.010,
        name="right_leaf_post_clearance",
    )
    ctx.expect_gap(
        right_leaf,
        left_leaf,
        axis="x",
        positive_elem="meeting_stile",
        negative_elem="meeting_stile",
        min_gap=0.006,
        max_gap=0.010,
        name="meeting_stile_gap",
    )

    ctx.expect_overlap(
        cane_bolt,
        right_leaf,
        axes="xy",
        elem_a="bolt_rod",
        elem_b="upper_guide",
        min_overlap=0.012,
        name="bolt_runs_through_upper_guide",
    )
    ctx.expect_overlap(
        cane_bolt,
        right_leaf,
        axes="xy",
        elem_a="bolt_rod",
        elem_b="lower_guide",
        min_overlap=0.012,
        name="bolt_runs_through_lower_guide",
    )

    left_rest = ctx.part_element_world_aabb(left_leaf, elem="meeting_stile")
    right_rest = ctx.part_element_world_aabb(right_leaf, elem="meeting_stile")
    latch_tip_rest = ctx.part_element_world_aabb(latch_lever, elem="lever_tip")
    bolt_rest = ctx.part_element_world_aabb(cane_bolt, elem="bolt_rod")
    assert left_rest is not None
    assert right_rest is not None
    assert latch_tip_rest is not None
    assert bolt_rest is not None

    left_rest_y = (left_rest[0][1] + left_rest[1][1]) * 0.5
    right_rest_y = (right_rest[0][1] + right_rest[1][1]) * 0.5
    latch_tip_rest_center_x = (latch_tip_rest[0][0] + latch_tip_rest[1][0]) * 0.5
    bolt_rest_bottom = bolt_rest[0][2]

    with ctx.pose({left_hinge: math.radians(80.0)}):
        left_open = ctx.part_element_world_aabb(left_leaf, elem="meeting_stile")
        assert left_open is not None
        left_open_y = (left_open[0][1] + left_open[1][1]) * 0.5
        assert left_open_y > left_rest_y + 0.55
        ctx.expect_contact(left_leaf, frame, name="left_leaf_stays_on_hinges_when_open")

    with ctx.pose({right_hinge: -math.radians(80.0)}):
        right_open = ctx.part_element_world_aabb(right_leaf, elem="meeting_stile")
        assert right_open is not None
        right_open_y = (right_open[0][1] + right_open[1][1]) * 0.5
        assert right_open_y > right_rest_y + 0.55
        ctx.expect_contact(right_leaf, frame, name="right_leaf_stays_on_hinges_when_open")

    with ctx.pose({latch_pivot: 0.30}):
        latch_tip_open = ctx.part_element_world_aabb(latch_lever, elem="lever_tip")
        assert latch_tip_open is not None
        latch_tip_open_center_x = (latch_tip_open[0][0] + latch_tip_open[1][0]) * 0.5
        assert latch_tip_open_center_x < latch_tip_rest_center_x - 0.006
        ctx.expect_contact(latch_lever, left_leaf, name="latch_remains_pinned_during_rotation")

    with ctx.pose({bolt_slide: 0.22}):
        bolt_raised = ctx.part_element_world_aabb(cane_bolt, elem="bolt_rod")
        assert bolt_raised is not None
        assert bolt_raised[0][2] > bolt_rest_bottom + 0.20
        ctx.expect_overlap(
            cane_bolt,
            right_leaf,
            axes="xy",
            elem_a="bolt_rod",
            elem_b="upper_guide",
            min_overlap=0.012,
            name="raised_bolt_stays_in_upper_guide",
        )
        ctx.expect_overlap(
            cane_bolt,
            right_leaf,
            axes="xy",
            elem_a="bolt_rod",
            elem_b="lower_guide",
            min_overlap=0.012,
            name="raised_bolt_stays_in_lower_guide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
