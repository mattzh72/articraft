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
    mesh_from_geometry,
    tube_from_spline_points,
)


IRON = Material("blackened_wrought_iron", rgba=(0.015, 0.014, 0.013, 1.0))
WORN_EDGE = Material("rubbed_dark_steel", rgba=(0.10, 0.095, 0.085, 1.0))
STONE = Material("weathered_courtyard_stone", rgba=(0.43, 0.39, 0.33, 1.0))


def _box(part, name: str, size, xyz, material=IRON) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name: str, radius: float, length: float, xyz, rpy=(0.0, 0.0, 0.0), material=IRON) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_courtyard_gate")

    leaf_width = 0.805
    hinge_x = 0.84
    hinge_y = 0.065
    leaf_plane_y = -hinge_y
    leaf_thickness = 0.045
    stile_w = 0.055

    frame = model.part("fixed_frame")
    _box(frame, "ground_sill", (2.08, 0.080, 0.040), (0.0, 0.0, 0.020), STONE)
    _box(frame, "post_0", (0.120, 0.140, 2.240), (-0.950, 0.0, 1.120), STONE)
    _box(frame, "post_1", (0.120, 0.140, 2.240), (0.950, 0.0, 1.120), STONE)

    fixed_arch = tube_from_spline_points(
        [(-0.955, 0.0, 2.080), (-0.480, 0.0, 2.220), (0.0, 0.0, 2.300), (0.480, 0.0, 2.220), (0.955, 0.0, 2.080)],
        radius=0.040,
        samples_per_segment=10,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(mesh_from_geometry(fixed_arch, "fixed_shallow_arch"), material=STONE, name="fixed_shallow_arch")

    # Fixed hinge knuckles and straps on the outer posts, offset from the gate
    # plane so the leaf knuckles can interleave without intersecting them.
    for side, sx in (("latch", -hinge_x), ("bolt", hinge_x)):
        post_x = -0.950 if sx < 0.0 else 0.950
        strap_center_x = (sx + post_x) / 2.0
        for i, z in enumerate((0.64, 1.50)):
            _cylinder(frame, f"{side}_post_knuckle_{i}", 0.022, 0.155, (sx, hinge_y, z), material=IRON)
            _box(frame, f"{side}_post_strap_{i}", (0.130, 0.026, 0.090), (strap_center_x, hinge_y, z), IRON)

    # A square ground receiver with a clear central hole for the cane bolt.  Its
    # rear bridge overlaps the sill, making it a fixed, supported piece.
    socket_x = hinge_x - 0.775
    socket_y = -0.115
    socket_z = 0.006
    _box(frame, "ground_socket_front", (0.075, 0.020, 0.012), (socket_x, socket_y - 0.030, socket_z), WORN_EDGE)
    _box(frame, "ground_socket_back", (0.075, 0.085, 0.012), (socket_x, socket_y + 0.075, socket_z), WORN_EDGE)
    _box(frame, "ground_socket_side_0", (0.015, 0.080, 0.012), (socket_x - 0.030, socket_y, socket_z), WORN_EDGE)
    _box(frame, "ground_socket_side_1", (0.015, 0.080, 0.012), (socket_x + 0.030, socket_y, socket_z), WORN_EDGE)

    latch_leaf = model.part("latch_leaf")
    _box(latch_leaf, "hinge_stile", (stile_w, leaf_thickness, 1.900), (0.0, leaf_plane_y, 0.985), IRON)
    _box(latch_leaf, "meeting_stile", (stile_w, leaf_thickness, 2.000), (leaf_width, leaf_plane_y, 1.040), IRON)
    _box(latch_leaf, "bottom_rail", (leaf_width, leaf_thickness, 0.055), (leaf_width / 2.0, leaf_plane_y, 0.135), IRON)
    _box(latch_leaf, "middle_rail", (leaf_width, 0.038, 0.045), (leaf_width / 2.0, leaf_plane_y, 1.015), IRON)
    _box(latch_leaf, "kick_plate", (0.38, 0.018, 0.30), (0.53, leaf_plane_y - 0.016, 0.36), WORN_EDGE)
    _box(latch_leaf, "latch_pivot_plate", (0.115, 0.045, 0.135), (0.700, leaf_plane_y - 0.025, 1.045), WORN_EDGE)
    _cylinder(latch_leaf, "latch_pivot_pin", 0.012, 0.090, (0.700, leaf_plane_y - 0.075, 1.045), rpy=(pi / 2.0, 0.0, 0.0), material=WORN_EDGE)
    for i, x in enumerate((0.18, 0.34, 0.50, 0.66)):
        _cylinder(latch_leaf, f"picket_{i}", 0.012, 1.720, (x, leaf_plane_y, 0.980), material=IRON)
    for i, z in enumerate((0.95, 1.83)):
        _cylinder(latch_leaf, f"hinge_knuckle_{i}", 0.020, 0.145, (0.0, 0.0, z), material=IRON)
        _box(latch_leaf, f"hinge_strap_{i}", (0.150, 0.078, 0.065), (0.065, leaf_plane_y / 2.0, z), IRON)
    latch_arch = tube_from_spline_points(
        [(0.02, leaf_plane_y, 1.885), (0.33, leaf_plane_y, 1.950), (0.62, leaf_plane_y, 2.005), (leaf_width - 0.02, leaf_plane_y, 2.035)],
        radius=0.020,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    latch_leaf.visual(mesh_from_geometry(latch_arch, "latch_leaf_arched_top_rail"), material=IRON, name="arched_top_rail")

    bolt_leaf = model.part("bolt_leaf")
    _box(bolt_leaf, "hinge_stile", (stile_w, leaf_thickness, 1.900), (0.0, leaf_plane_y, 0.985), IRON)
    _box(bolt_leaf, "meeting_stile", (stile_w, leaf_thickness, 2.000), (-leaf_width, leaf_plane_y, 1.040), IRON)
    _box(bolt_leaf, "bottom_rail", (leaf_width, leaf_thickness, 0.055), (-leaf_width / 2.0, leaf_plane_y, 0.135), IRON)
    _box(bolt_leaf, "middle_rail", (leaf_width, 0.038, 0.045), (-leaf_width / 2.0, leaf_plane_y, 1.015), IRON)
    _box(bolt_leaf, "kick_plate", (0.38, 0.018, 0.30), (-0.53, leaf_plane_y - 0.016, 0.36), WORN_EDGE)
    for i, x in enumerate((-0.18, -0.34, -0.50, -0.66)):
        _cylinder(bolt_leaf, f"picket_{i}", 0.012, 1.720, (x, leaf_plane_y, 0.980), material=IRON)
    for i, z in enumerate((0.95, 1.83)):
        _cylinder(bolt_leaf, f"hinge_knuckle_{i}", 0.020, 0.145, (0.0, 0.0, z), material=IRON)
        _box(bolt_leaf, f"hinge_strap_{i}", (0.150, 0.078, 0.065), (-0.065, leaf_plane_y / 2.0, z), IRON)
    bolt_arch = tube_from_spline_points(
        [(-0.02, leaf_plane_y, 1.885), (-0.33, leaf_plane_y, 1.950), (-0.62, leaf_plane_y, 2.005), (-leaf_width + 0.02, leaf_plane_y, 2.035)],
        radius=0.020,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )
    bolt_leaf.visual(mesh_from_geometry(bolt_arch, "bolt_leaf_arched_top_rail"), material=IRON, name="arched_top_rail")

    # Receiver fork for the latch lever; the slot is deliberately clear around
    # the lever bar at the closed pose.
    _box(bolt_leaf, "latch_keeper_back", (0.150, 0.035, 0.135), (-0.790, leaf_plane_y - 0.040, 1.045), WORN_EDGE)
    _box(bolt_leaf, "latch_keeper_upper", (0.145, 0.050, 0.014), (-0.790, leaf_plane_y - 0.070, 1.086), WORN_EDGE)
    _box(bolt_leaf, "latch_keeper_lower", (0.145, 0.050, 0.014), (-0.790, leaf_plane_y - 0.070, 1.004), WORN_EDGE)

    rod_x = -0.775
    rod_y = leaf_plane_y - 0.115
    loop_inner_x = 0.024
    loop_inner_y = 0.090
    loop_t = 0.013
    loop_h = 0.060
    for loop_name, z in (("lower_loop", 0.62), ("upper_loop", 1.24)):
        loop_front_y = rod_y - loop_inner_y / 2.0 - loop_t
        loop_back_y = leaf_plane_y - leaf_thickness / 2.0 - loop_t / 2.0
        loop_span_y = (loop_back_y - loop_front_y) + loop_t
        loop_center_y = (loop_front_y + loop_back_y) / 2.0
        _box(
            bolt_leaf,
            f"{loop_name}_front",
            (loop_inner_x + 2 * loop_t, loop_t * 2.0, loop_h),
            (rod_x, loop_front_y, z),
            WORN_EDGE,
        )
        _box(
            bolt_leaf,
            f"{loop_name}_back",
            (loop_inner_x + 2 * loop_t, 0.030, loop_h),
            (rod_x, loop_back_y, z),
            WORN_EDGE,
        )
        _box(
            bolt_leaf,
            f"{loop_name}_side_0",
            (loop_t, loop_span_y, loop_h),
            (rod_x - loop_inner_x / 2.0 - loop_t / 2.0, loop_center_y, z),
            WORN_EDGE,
        )
        _box(
            bolt_leaf,
            f"{loop_name}_side_1",
            (loop_t, loop_span_y, loop_h),
            (rod_x + loop_inner_x / 2.0 + loop_t / 2.0, loop_center_y, z),
            WORN_EDGE,
        )

    latch_lever = model.part("latch_lever")
    _box(latch_lever, "lever_bar", (0.300, 0.024, 0.034), (0.055, 0.0, 0.0), WORN_EDGE)
    _cylinder(latch_lever, "pivot_disc", 0.040, 0.018, (0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0), material=WORN_EDGE)
    _cylinder(latch_lever, "round_grip", 0.018, 0.080, (-0.105, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0), material=WORN_EDGE)

    cane_bolt = model.part("cane_bolt")
    _cylinder(cane_bolt, "bolt_rod", 0.012, 1.600, (0.0, 0.0, 0.0), material=WORN_EDGE)
    _cylinder(cane_bolt, "pull_handle", 0.010, 0.135, (0.0, -0.068, 0.250), rpy=(pi / 2.0, 0.0, 0.0), material=WORN_EDGE)
    _box(cane_bolt, "handle_collar", (0.040, 0.024, 0.050), (0.0, 0.0, 0.250), WORN_EDGE)
    _cylinder(cane_bolt, "tapered_tip", 0.010, 0.070, (0.0, 0.0, -0.815), material=WORN_EDGE)

    model.articulation(
        "frame_to_latch_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=latch_leaf,
        origin=Origin(xyz=(-hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "frame_to_bolt_leaf",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bolt_leaf,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=0.0, upper=1.25),
    )
    model.articulation(
        "latch_leaf_to_latch_lever",
        ArticulationType.REVOLUTE,
        parent=latch_leaf,
        child=latch_lever,
        origin=Origin(xyz=(0.700, leaf_plane_y - 0.075, 1.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.35, upper=0.85),
    )
    model.articulation(
        "bolt_leaf_to_cane_bolt",
        ArticulationType.PRISMATIC,
        parent=bolt_leaf,
        child=cane_bolt,
        origin=Origin(xyz=(rod_x, rod_y, 0.800)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.25),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    latch_leaf = object_model.get_part("latch_leaf")
    bolt_leaf = object_model.get_part("bolt_leaf")
    latch_lever = object_model.get_part("latch_lever")
    cane_bolt = object_model.get_part("cane_bolt")
    latch_hinge = object_model.get_articulation("frame_to_latch_leaf")
    bolt_hinge = object_model.get_articulation("frame_to_bolt_leaf")
    lever_joint = object_model.get_articulation("latch_leaf_to_latch_lever")
    bolt_slide = object_model.get_articulation("bolt_leaf_to_cane_bolt")

    ctx.allow_overlap(
        latch_leaf,
        latch_lever,
        elem_a="latch_pivot_pin",
        elem_b="pivot_disc",
        reason="The latch lever is captured on a small pivot pin passing through its round boss.",
    )
    ctx.allow_overlap(
        latch_leaf,
        latch_lever,
        elem_a="latch_pivot_pin",
        elem_b="lever_bar",
        reason="The pivot pin passes through the lever bar at its pivot hole.",
    )
    ctx.expect_contact(
        latch_leaf,
        latch_lever,
        elem_a="latch_pivot_pin",
        elem_b="pivot_disc",
        contact_tol=0.010,
        name="latch lever boss is seated on its pivot pin",
    )

    ctx.expect_gap(
        bolt_leaf,
        latch_leaf,
        axis="x",
        positive_elem="meeting_stile",
        negative_elem="meeting_stile",
        min_gap=0.006,
        max_gap=0.035,
        name="closed leaves meet with a narrow center reveal",
    )
    ctx.expect_overlap(
        latch_leaf,
        bolt_leaf,
        axes="z",
        elem_a="meeting_stile",
        elem_b="meeting_stile",
        min_overlap=1.80,
        name="meeting stiles share a tall closing line",
    )

    latch_rest = ctx.part_element_world_aabb(latch_leaf, elem="meeting_stile")
    bolt_rest = ctx.part_element_world_aabb(bolt_leaf, elem="meeting_stile")
    with ctx.pose({latch_hinge: 0.95, bolt_hinge: 0.95}):
        latch_open = ctx.part_element_world_aabb(latch_leaf, elem="meeting_stile")
        bolt_open = ctx.part_element_world_aabb(bolt_leaf, elem="meeting_stile")
    ctx.check(
        "both leaves swing inward on their post hinges",
        latch_rest is not None
        and bolt_rest is not None
        and latch_open is not None
        and bolt_open is not None
        and (latch_open[0][1] + latch_open[1][1]) / 2.0 > (latch_rest[0][1] + latch_rest[1][1]) / 2.0 + 0.35
        and (bolt_open[0][1] + bolt_open[1][1]) / 2.0 > (bolt_rest[0][1] + bolt_rest[1][1]) / 2.0 + 0.35,
        details=f"rest={latch_rest},{bolt_rest} open={latch_open},{bolt_open}",
    )

    lever_rest = ctx.part_element_world_aabb(latch_lever, elem="lever_bar")
    with ctx.pose({lever_joint: 0.65}):
        lever_raised = ctx.part_element_world_aabb(latch_lever, elem="lever_bar")
    ctx.check(
        "latch lever rotates upward about its small pivot",
        lever_rest is not None
        and lever_raised is not None
        and lever_raised[1][2] > lever_rest[1][2] + 0.09,
        details=f"rest={lever_rest}, raised={lever_raised}",
    )

    for loop in ("lower_loop", "upper_loop"):
        ctx.expect_overlap(
            cane_bolt,
            bolt_leaf,
            axes="z",
            elem_a="bolt_rod",
            elem_b=f"{loop}_front",
            min_overlap=0.045,
            name=f"cane bolt passes through {loop} height",
        )
        ctx.expect_gap(
            cane_bolt,
            bolt_leaf,
            axis="y",
            positive_elem="bolt_rod",
            negative_elem=f"{loop}_front",
            min_gap=0.004,
            name=f"cane bolt clears front strap of {loop}",
        )
        ctx.expect_gap(
            bolt_leaf,
            cane_bolt,
            axis="y",
            positive_elem=f"{loop}_back",
            negative_elem="bolt_rod",
            min_gap=0.004,
            name=f"cane bolt clears back strap of {loop}",
        )
        ctx.expect_contact(
            cane_bolt,
            bolt_leaf,
            elem_a="bolt_rod",
            elem_b=f"{loop}_side_1",
            contact_tol=0.002,
            name=f"cane bolt is laterally captured by {loop}",
        )

    bolt_rest = ctx.part_world_position(cane_bolt)
    with ctx.pose({bolt_slide: 0.25}):
        bolt_raised = ctx.part_world_position(cane_bolt)
        for loop in ("lower_loop", "upper_loop"):
            ctx.expect_overlap(
                cane_bolt,
                bolt_leaf,
                axes="z",
                elem_a="bolt_rod",
                elem_b=f"{loop}_front",
                min_overlap=0.045,
                name=f"raised cane bolt remains captured in {loop}",
            )
    ctx.check(
        "cane bolt slides upward in its guide loops",
        bolt_rest is not None and bolt_raised is not None and bolt_raised[2] > bolt_rest[2] + 0.20,
        details=f"rest={bolt_rest}, raised={bolt_raised}",
    )

    return ctx.report()


object_model = build_object_model()
