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
    model = ArticulatedObject(name="cantilever_sliding_garden_gate")

    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.63, 1.0))
    steel = model.material("steel", rgba=(0.30, 0.32, 0.34, 1.0))
    gate_green = model.material("gate_green", rgba=(0.20, 0.29, 0.20, 1.0))
    roller_black = model.material("roller_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hardware = model.material("hardware", rgba=(0.14, 0.14, 0.15, 1.0))

    def add_box(
        part,
        name: str,
        size: tuple[float, float, float],
        xyz: tuple[float, float, float],
        material,
        *,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_cylinder(
        part,
        name: str,
        radius: float,
        length: float,
        xyz: tuple[float, float, float],
        material,
        *,
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_diagonal_member(
        part,
        name: str,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        *,
        depth_y: float,
        depth_z: float,
        material,
    ) -> None:
        dx = end[0] - start[0]
        dz = end[2] - start[2]
        length = math.hypot(dx, dz)
        angle = math.atan2(dz, dx)
        center = (
            0.5 * (start[0] + end[0]),
            0.5 * (start[1] + end[1]),
            0.5 * (start[2] + end[2]),
        )
        add_box(
            part,
            name,
            (length, depth_y, depth_z),
            center,
            material,
            rpy=(0.0, -angle, 0.0),
        )

    support = model.part("support_assembly")
    add_box(
        support,
        "foundation_beam",
        (5.35, 0.48, 0.18),
        (0.425, -0.08, 0.09),
        concrete,
    )
    add_box(
        support,
        "guide_post",
        (0.12, 0.12, 1.95),
        (0.0, -0.16, 0.975),
        steel,
    )
    add_box(
        support,
        "latch_post",
        (0.12, 0.12, 1.95),
        (2.95, -0.16, 0.975),
        steel,
    )
    add_box(
        support,
        "guide_bracket_arm",
        (0.10, 0.06, 0.06),
        (0.02, -0.11, 1.89),
        steel,
    )
    add_box(
        support,
        "guide_bracket_crosshead",
        (0.08, 0.22, 0.03),
        (0.04, 0.0, 1.92),
        steel,
    )
    add_box(
        support,
        "guide_plate_inboard",
        (0.08, 0.02, 0.24),
        (0.04, -0.095, 1.79),
        steel,
    )
    add_box(
        support,
        "guide_plate_outboard",
        (0.08, 0.02, 0.24),
        (0.04, 0.095, 1.79),
        steel,
    )
    add_cylinder(
        support,
        "guide_roller_inboard",
        0.03,
        0.09,
        (0.04, -0.058, 1.79),
        roller_black,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    add_cylinder(
        support,
        "guide_roller_outboard",
        0.03,
        0.09,
        (0.04, 0.058, 1.79),
        roller_black,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    add_box(
        support,
        "latch_receiver",
        (0.025, 0.16, 0.10),
        (2.9125, -0.05, 0.92),
        hardware,
    )

    truck_specs = (("rear_truck", -0.80), ("front_truck", -0.15))
    for prefix, truck_x in truck_specs:
        add_box(
            support,
            f"{prefix}_pedestal",
            (0.30, 0.26, 0.08),
            (truck_x, -0.02, 0.22),
            steel,
        )
        add_box(
            support,
            f"{prefix}_cheek_inboard",
            (0.20, 0.01, 0.18),
            (truck_x, -0.075, 0.31),
            steel,
        )
        add_box(
            support,
            f"{prefix}_cheek_outboard",
            (0.20, 0.01, 0.18),
            (truck_x, 0.075, 0.31),
            steel,
        )
        add_cylinder(
            support,
            f"{prefix}_rear_roller",
            0.045,
            0.08,
            (truck_x - 0.06, 0.0, 0.375),
            roller_black,
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        )
        add_box(
            support,
            f"{prefix}_rear_carrier",
            (0.035, 0.04, 0.15),
            (truck_x - 0.06, 0.0, 0.295),
            steel,
        )
        add_cylinder(
            support,
            f"{prefix}_front_roller",
            0.045,
            0.08,
            (truck_x + 0.06, 0.0, 0.375),
            roller_black,
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        )
        add_box(
            support,
            f"{prefix}_front_carrier",
            (0.035, 0.04, 0.15),
            (truck_x + 0.06, 0.0, 0.295),
            steel,
        )

    support.inertial = Inertial.from_geometry(
        Box((5.35, 0.48, 1.95)),
        mass=280.0,
        origin=Origin(xyz=(0.425, -0.08, 0.975)),
    )

    leaf = model.part("gate_leaf")
    add_box(leaf, "track_top", (4.40, 0.14, 0.02), (-2.20, 0.0, 0.17), gate_green)
    add_box(leaf, "track_side_outboard", (4.40, 0.02, 0.14), (-2.20, 0.06, 0.09), gate_green)
    add_box(leaf, "track_side_inboard", (4.40, 0.02, 0.14), (-2.20, -0.06, 0.09), gate_green)
    add_box(leaf, "track_lip_outboard", (4.40, 0.03, 0.02), (-2.20, 0.04, 0.01), gate_green)
    add_box(leaf, "track_lip_inboard", (4.40, 0.03, 0.02), (-2.20, -0.04, 0.01), gate_green)

    add_box(leaf, "leading_stile", (0.06, 0.05, 1.32), (-0.03, 0.0, 0.84), gate_green)
    add_box(leaf, "guide_side_stile", (0.06, 0.05, 1.32), (-2.90, 0.0, 0.84), gate_green)
    add_box(leaf, "rear_stile", (0.06, 0.05, 1.32), (-4.37, 0.0, 0.84), gate_green)
    add_box(leaf, "top_rail", (4.34, 0.05, 0.06), (-2.20, 0.0, 1.53), gate_green)
    add_box(leaf, "mid_rail", (2.84, 0.04, 0.05), (-1.45, 0.0, 0.92), gate_green)
    add_box(leaf, "handle_mount", (0.05, 0.02, 0.08), (-0.03, 0.035, 0.92), hardware)

    for idx, picket_x in enumerate((-0.42, -0.82, -1.22, -1.62, -2.02, -2.42, -2.72), start=1):
        add_box(
            leaf,
            f"picket_{idx}",
            (0.03, 0.025, 1.32),
            (picket_x, 0.0, 0.84),
            gate_green,
        )

    add_diagonal_member(
        leaf,
        "main_span_brace",
        (-2.90, 0.0, 0.20),
        (-0.06, 0.0, 1.48),
        depth_y=0.04,
        depth_z=0.04,
        material=gate_green,
    )
    add_diagonal_member(
        leaf,
        "counterbalance_brace_upper",
        (-4.34, 0.0, 0.20),
        (-2.93, 0.0, 1.48),
        depth_y=0.04,
        depth_z=0.04,
        material=gate_green,
    )
    add_diagonal_member(
        leaf,
        "counterbalance_brace_lower",
        (-4.34, 0.0, 1.48),
        (-2.93, 0.0, 0.20),
        depth_y=0.04,
        depth_z=0.04,
        material=gate_green,
    )

    leaf.inertial = Inertial.from_geometry(
        Box((4.40, 0.16, 1.30)),
        mass=95.0,
        origin=Origin(xyz=(-2.20, 0.0, 0.83)),
    )

    handle = model.part("latch_handle")
    add_cylinder(
        handle,
        "pivot_spindle",
        0.006,
        0.012,
        (0.0, 0.0, 0.0),
        hardware,
        rpy=(-math.pi / 2.0, 0.0, 0.0),
    )
    add_cylinder(
        handle,
        "pivot_collar",
        0.016,
        0.006,
        (0.0, 0.003, 0.0),
        hardware,
        rpy=(-math.pi / 2.0, 0.0, 0.0),
    )
    add_box(handle, "lever_hub", (0.030, 0.018, 0.030), (-0.015, 0.009, -0.008), hardware)
    add_box(handle, "lever_bar", (0.072, 0.014, 0.020), (-0.051, 0.010, -0.018), hardware)
    add_cylinder(
        handle,
        "grip",
        0.008,
        0.082,
        (-0.087, 0.010, -0.052),
        hardware,
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.12, 0.04, 0.12)),
        mass=0.6,
        origin=Origin(xyz=(-0.045, 0.010, -0.035)),
    )

    model.articulation(
        "support_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=support,
        child=leaf,
        origin=Origin(xyz=(2.90, 0.0, 0.26)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.5,
            lower=-2.82,
            upper=0.0,
        ),
    )

    model.articulation(
        "leaf_to_handle",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=handle,
        origin=Origin(xyz=(-0.015, 0.045, 0.92)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=-0.35,
            upper=0.40,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_assembly")
    leaf = object_model.get_part("gate_leaf")
    handle = object_model.get_part("latch_handle")

    slide = object_model.get_articulation("support_to_gate_leaf")
    handle_joint = object_model.get_articulation("leaf_to_handle")

    track_top = leaf.get_visual("track_top")
    top_rail = leaf.get_visual("top_rail")
    leading_stile = leaf.get_visual("leading_stile")
    handle_mount = leaf.get_visual("handle_mount")

    front_truck_front_roller = support.get_visual("front_truck_front_roller")
    rear_truck_rear_roller = support.get_visual("rear_truck_rear_roller")
    guide_roller_inboard = support.get_visual("guide_roller_inboard")
    guide_roller_outboard = support.get_visual("guide_roller_outboard")
    latch_receiver = support.get_visual("latch_receiver")
    pivot_spindle = handle.get_visual("pivot_spindle")
    pivot_collar = handle.get_visual("pivot_collar")
    grip = handle.get_visual("grip")

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
    ctx.allow_overlap(
        handle,
        leaf,
        elem_a=pivot_spindle,
        elem_b=handle_mount,
        reason="The latch spindle intentionally passes through the small mounting plate on the leading stile.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slide_axis_matches_fence_line",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"slide axis was {tuple(slide.axis)}",
    )
    ctx.check(
        "handle_axis_matches_pivot",
        tuple(handle_joint.axis) == (0.0, 1.0, 0.0),
        details=f"handle axis was {tuple(handle_joint.axis)}",
    )

    ctx.expect_gap(
        leaf,
        support,
        axis="z",
        positive_elem=track_top,
        negative_elem=front_truck_front_roller,
        min_gap=0.0,
        max_gap=0.002,
        name="front_truck_carries_track",
    )
    ctx.expect_gap(
        leaf,
        support,
        axis="z",
        positive_elem=track_top,
        negative_elem=rear_truck_rear_roller,
        min_gap=0.0,
        max_gap=0.002,
        name="rear_truck_carries_track",
    )
    ctx.expect_gap(
        support,
        leaf,
        axis="y",
        positive_elem=guide_roller_outboard,
        negative_elem=top_rail,
        min_gap=0.0,
        max_gap=0.006,
        name="outboard_top_guide_clearance",
    )
    ctx.expect_gap(
        leaf,
        support,
        axis="y",
        positive_elem=top_rail,
        negative_elem=guide_roller_inboard,
        min_gap=0.0,
        max_gap=0.006,
        name="inboard_top_guide_clearance",
    )
    ctx.expect_gap(
        support,
        leaf,
        axis="x",
        positive_elem=latch_receiver,
        negative_elem=leading_stile,
        min_gap=0.0,
        max_gap=0.002,
        name="closed_leaf_meets_receiver",
    )
    ctx.expect_contact(
        handle,
        leaf,
        elem_a=pivot_collar,
        elem_b=handle_mount,
        name="handle_collar_seats_on_mount",
    )

    rest_leaf_pos = ctx.part_world_position(leaf)
    if rest_leaf_pos is None:
        ctx.fail("leaf_rest_position_available", "Gate leaf world position was unavailable in the rest pose.")

    slide_open = slide.motion_limits.lower
    if slide_open is None:
        ctx.fail("slide_has_open_limit", "Sliding articulation was missing its open travel limit.")
    elif rest_leaf_pos is not None:
        with ctx.pose({slide: slide_open}):
            open_leaf_pos = ctx.part_world_position(leaf)
            if open_leaf_pos is None:
                ctx.fail("leaf_open_position_available", "Gate leaf world position was unavailable in the open pose.")
            else:
                ctx.check(
                    "leaf_translates_open_along_fence_line",
                    open_leaf_pos[0] < rest_leaf_pos[0] - 2.7,
                    details=f"rest={rest_leaf_pos}, open={open_leaf_pos}",
                )
                ctx.check(
                    "leaf_keeps_lateral_alignment",
                    abs(open_leaf_pos[1] - rest_leaf_pos[1]) < 1e-6,
                    details=f"rest={rest_leaf_pos}, open={open_leaf_pos}",
                )
                ctx.check(
                    "leaf_keeps_height_on_trucks",
                    abs(open_leaf_pos[2] - rest_leaf_pos[2]) < 1e-6,
                    details=f"rest={rest_leaf_pos}, open={open_leaf_pos}",
                )

            ctx.expect_gap(
                leaf,
                support,
                axis="z",
                positive_elem=track_top,
                negative_elem=front_truck_front_roller,
                min_gap=0.0,
                max_gap=0.002,
                name="front_truck_still_under_track_open",
            )
            ctx.expect_gap(
                leaf,
                support,
                axis="z",
                positive_elem=track_top,
                negative_elem=rear_truck_rear_roller,
                min_gap=0.0,
                max_gap=0.002,
                name="rear_truck_still_under_track_open",
            )
            ctx.expect_gap(
                support,
                leaf,
                axis="y",
                positive_elem=guide_roller_outboard,
                negative_elem=top_rail,
                min_gap=0.0,
                max_gap=0.006,
                name="outboard_guide_clearance_open",
            )
            ctx.expect_gap(
                leaf,
                support,
                axis="y",
                positive_elem=top_rail,
                negative_elem=guide_roller_inboard,
                min_gap=0.0,
                max_gap=0.006,
                name="inboard_guide_clearance_open",
            )

    rest_grip_aabb = ctx.part_element_world_aabb(handle, elem=grip)
    if rest_grip_aabb is None:
        ctx.fail("handle_grip_rest_aabb_available", "Grip element AABB was unavailable in the rest pose.")
    else:
        handle_turn = handle_joint.motion_limits.upper
        if handle_turn is None:
            ctx.fail("handle_has_turn_limit", "Handle articulation was missing its turned pose limit.")
        else:
            with ctx.pose({handle_joint: handle_turn}):
                turned_grip_aabb = ctx.part_element_world_aabb(handle, elem=grip)
                if turned_grip_aabb is None:
                    ctx.fail("handle_grip_open_aabb_available", "Grip element AABB was unavailable in the turned pose.")
                else:
                    ctx.check(
                        "handle_grip_lifts_when_turned",
                        turned_grip_aabb[1][2] > rest_grip_aabb[1][2] + 0.02,
                        details=f"rest={rest_grip_aabb}, turned={turned_grip_aabb}",
                    )
                    ctx.expect_contact(
                        handle,
                        leaf,
                        elem_a=pivot_collar,
                        elem_b=handle_mount,
                        name="handle_stays_seated_while_rotating",
                    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
