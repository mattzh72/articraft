from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate_utility", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.21, 0.23, 0.25, 1.0))
    gate_paint = model.material("gate_paint", rgba=(0.30, 0.37, 0.42, 1.0))
    hardware_finish = model.material("hardware_finish", rgba=(0.62, 0.64, 0.66, 1.0))
    roller_finish = model.material("roller_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    warning_finish = model.material("warning_finish", rgba=(0.86, 0.67, 0.12, 1.0))

    def box_visual(part, name, size, xyz, *, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def cyl_visual(part, name, radius, length, xyz, *, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((2.55, 0.20, 1.95)),
        mass=140.0,
        origin=Origin(xyz=(-0.45, 0.0, 0.975)),
    )

    # Ground track and support line.
    box_visual(frame, "track_base", (2.40, 0.12, 0.04), (-0.50, 0.0, 0.02), material=frame_paint)
    box_visual(frame, "ground_rail", (2.40, 0.03, 0.032), (-0.50, 0.0, 0.056), material=hardware_finish)
    box_visual(frame, "track_stop_left", (0.03, 0.12, 0.08), (-1.675, 0.0, 0.08), material=warning_finish)
    box_visual(frame, "track_stop_right", (0.03, 0.12, 0.08), (0.675, 0.0, 0.08), material=warning_finish)

    # Structural posts and header.
    post_size = (0.10, 0.12, 1.90)
    box_visual(frame, "post_stack", post_size, (-1.52, 0.0, 1.00), material=frame_paint)
    box_visual(frame, "post_left", post_size, (-0.62, 0.0, 1.00), material=frame_paint)
    box_visual(frame, "post_right", post_size, (0.62, 0.0, 1.00), material=frame_paint)
    box_visual(frame, "header_beam", (2.50, 0.14, 0.12), (-0.45, 0.0, 1.79), material=frame_paint)

    # Reinforced sleeves at the header joints.
    box_visual(frame, "sleeve_stack", (0.18, 0.14, 0.06), (-1.52, 0.0, 1.75), material=hardware_finish)
    box_visual(frame, "sleeve_left", (0.18, 0.14, 0.06), (-0.62, 0.0, 1.75), material=hardware_finish)
    box_visual(frame, "sleeve_right", (0.18, 0.14, 0.06), (0.62, 0.0, 1.75), material=hardware_finish)

    # Base plates and anchor bolts.
    for prefix, x_center in (("stack", -1.52), ("left", -0.62), ("right", 0.62)):
        box_visual(frame, f"{prefix}_base_plate", (0.16, 0.14, 0.01), (x_center, 0.0, 0.045), material=hardware_finish)
        for i, (dx, dy) in enumerate(((-0.045, -0.04), (-0.045, 0.04), (0.045, -0.04), (0.045, 0.04)), start=1):
            cyl_visual(
                frame,
                f"{prefix}_anchor_{i}",
                0.006,
                0.012,
                (x_center + dx, dy, 0.056),
                material=hardware_finish,
            )

    # Suspended top guide shoe that keeps the gate legibly constrained.
    box_visual(frame, "guide_shoe_top", (0.28, 0.11, 0.03), (-0.48, 0.0, 1.715), material=hardware_finish)
    box_visual(frame, "guide_shoe_front", (0.28, 0.02, 0.14), (-0.48, 0.04, 1.63), material=hardware_finish)
    box_visual(frame, "guide_shoe_back", (0.28, 0.02, 0.14), (-0.48, -0.04, 1.63), material=hardware_finish)
    for i, x_center in enumerate((-0.53, -0.43), start=1):
        cyl_visual(
            frame,
            f"guide_bolt_{i}",
            0.006,
            0.02,
            (x_center, 0.0, 1.73),
            material=hardware_finish,
            rpy=(0.0, math.pi / 2.0, 0.0),
        )

    # Latch keeper fork on the closing post.
    box_visual(frame, "keeper_upper", (0.04, 0.012, 0.025), (0.585, 0.018, 1.00), material=hardware_finish)
    box_visual(frame, "keeper_lower", (0.04, 0.012, 0.025), (0.585, -0.018, 1.00), material=hardware_finish)
    box_visual(frame, "keeper_back", (0.01, 0.048, 0.05), (0.605, 0.0, 1.00), material=hardware_finish)
    cyl_visual(
        frame,
        "keeper_fastener_top",
        0.006,
        0.014,
        (0.573, 0.0, 1.018),
        material=hardware_finish,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    cyl_visual(
        frame,
        "keeper_fastener_bottom",
        0.006,
        0.014,
        (0.573, 0.0, 0.982),
        material=hardware_finish,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    gate = model.part("gate_leaf")
    gate.inertial = Inertial.from_geometry(
        Box((1.12, 0.08, 1.55)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
    )

    # Heavy rectangular tube frame.
    box_visual(gate, "bottom_rail", (1.09, 0.05, 0.07), (0.0, 0.0, 0.195), material=gate_paint)
    box_visual(gate, "top_rail", (1.09, 0.05, 0.07), (0.0, 0.0, 1.62), material=gate_paint)
    box_visual(gate, "left_stile", (0.07, 0.05, 1.495), (-0.51, 0.0, 0.9075), material=gate_paint)
    box_visual(gate, "right_stile", (0.07, 0.05, 1.495), (0.51, 0.0, 0.9075), material=gate_paint)
    box_visual(gate, "mid_rail", (1.01, 0.04, 0.05), (0.0, 0.0, 0.94), material=gate_paint)

    # Reinforcement and anti-racking members.
    brace_angle = math.atan2(1.20, 0.82)
    box_visual(
        gate,
        "diagonal_brace",
        (1.46, 0.018, 0.04),
        (-0.04, 0.0, 0.89),
        material=hardware_finish,
        rpy=(0.0, brace_angle, 0.0),
    )
    box_visual(
        gate,
        "gusset_lower_left",
        (0.11, 0.012, 0.11),
        (-0.44, 0.0, 0.27),
        material=hardware_finish,
        rpy=(0.0, math.radians(45.0), 0.0),
    )
    box_visual(
        gate,
        "gusset_upper_right",
        (0.11, 0.012, 0.11),
        (0.44, 0.0, 1.55),
        material=hardware_finish,
        rpy=(0.0, math.radians(-45.0), 0.0),
    )

    # Security infill bars.
    for i, x_center in enumerate((-0.34, -0.17, 0.0, 0.17, 0.34), start=1):
        box_visual(gate, f"picket_{i}", (0.03, 0.018, 1.355), (x_center, 0.0, 0.9075), material=hardware_finish)

    # Rolling carriage hardware.
    for prefix, x_center in (("rear", -0.30), ("front", 0.30)):
        box_visual(gate, f"{prefix}_hanger_left", (0.018, 0.01, 0.06), (x_center, 0.018, 0.13), material=hardware_finish)
        box_visual(gate, f"{prefix}_hanger_right", (0.018, 0.01, 0.06), (x_center, -0.018, 0.13), material=hardware_finish)
        cyl_visual(
            gate,
            f"{prefix}_axle",
            0.007,
            0.036,
            (x_center, 0.0, 0.11),
            material=hardware_finish,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
        cyl_visual(
            gate,
            f"{prefix}_wheel",
            0.038,
            0.024,
            (x_center, 0.0, 0.11),
            material=roller_finish,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
        cyl_visual(
            gate,
            f"{prefix}_bolt_head_front",
            0.0055,
            0.006,
            (x_center, 0.026, 0.11),
            material=hardware_finish,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )
        cyl_visual(
            gate,
            f"{prefix}_bolt_head_back",
            0.0055,
            0.006,
            (x_center, -0.026, 0.11),
            material=hardware_finish,
            rpy=(math.pi / 2.0, 0.0, 0.0),
        )

    # Wear strip where the guide shoe rides over the top rail.
    box_visual(gate, "guide_wear_strip", (0.24, 0.022, 0.016), (-0.20, 0.0, 1.663), material=hardware_finish)

    # Latch hardware and pull.
    box_visual(gate, "latch_housing", (0.08, 0.028, 0.07), (0.50, 0.0, 1.00), material=hardware_finish)
    box_visual(gate, "latch_bolt", (0.026, 0.012, 0.018), (0.553, 0.0, 1.00), material=hardware_finish)
    cyl_visual(gate, "handle_grip", 0.009, 0.15, (0.46, 0.032, 1.00), material=hardware_finish)
    cyl_visual(
        gate,
        "handle_post_top",
        0.006,
        0.05,
        (0.46, 0.014, 1.055),
        material=hardware_finish,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    cyl_visual(
        gate,
        "handle_post_bottom",
        0.006,
        0.05,
        (0.46, 0.014, 0.945),
        material=hardware_finish,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )
    cyl_visual(
        gate,
        "handle_fastener_top",
        0.005,
        0.012,
        (0.46, 0.0, 1.055),
        material=hardware_finish,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    cyl_visual(
        gate,
        "handle_fastener_bottom",
        0.005,
        0.012,
        (0.46, 0.0, 0.945),
        material=hardware_finish,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.8, lower=-0.96, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    gate = object_model.get_part("gate_leaf")
    gate_slide = object_model.get_articulation("gate_slide")

    ground_rail = frame.get_visual("ground_rail")
    guide_shoe_top = frame.get_visual("guide_shoe_top")
    guide_shoe_front = frame.get_visual("guide_shoe_front")
    guide_shoe_back = frame.get_visual("guide_shoe_back")
    keeper_back = frame.get_visual("keeper_back")
    front_wheel = gate.get_visual("front_wheel")
    rear_wheel = gate.get_visual("rear_wheel")
    top_rail = gate.get_visual("top_rail")
    guide_wear_strip = gate.get_visual("guide_wear_strip")
    latch_bolt = gate.get_visual("latch_bolt")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_origin_distance(gate, frame, axes="yz", max_dist=1e-6, name="gate_stays_on_slide_axis")
    ctx.expect_origin_distance(gate, frame, axes="x", max_dist=1e-6, name="gate_closed_position")
    ctx.expect_contact(gate, frame, elem_a=front_wheel, elem_b=ground_rail, contact_tol=0.0015, name="front_wheel_on_ground_rail")
    ctx.expect_contact(gate, frame, elem_a=rear_wheel, elem_b=ground_rail, contact_tol=0.0015, name="rear_wheel_on_ground_rail")
    ctx.expect_overlap(gate, frame, axes="x", elem_a=top_rail, elem_b=guide_shoe_top, min_overlap=0.20, name="top_rail_runs_through_guide_shoe")
    ctx.expect_gap(
        frame,
        gate,
        axis="y",
        positive_elem=guide_shoe_front,
        negative_elem=guide_wear_strip,
        min_gap=0.004,
        max_gap=0.02,
        name="front_guide_clearance",
    )
    ctx.expect_gap(
        gate,
        frame,
        axis="y",
        positive_elem=guide_wear_strip,
        negative_elem=guide_shoe_back,
        min_gap=0.004,
        max_gap=0.02,
        name="rear_guide_clearance",
    )
    ctx.expect_overlap(gate, frame, axes="yz", elem_a=latch_bolt, elem_b=keeper_back, min_overlap=0.012, name="latch_bolt_aligned_with_keeper")
    ctx.expect_gap(
        frame,
        gate,
        axis="x",
        positive_elem=keeper_back,
        negative_elem=latch_bolt,
        min_gap=0.03,
        max_gap=0.07,
        name="latch_bolt_seats_inside_keeper_depth",
    )

    with ctx.pose({gate_slide: -0.96}):
        ctx.expect_origin_distance(gate, frame, axes="yz", max_dist=1e-6, name="gate_open_pose_stays_on_slide_axis")
        ctx.expect_origin_gap(frame, gate, axis="x", min_gap=0.95, max_gap=0.97, name="gate_full_open_travel")
        ctx.expect_contact(gate, frame, elem_a=front_wheel, elem_b=ground_rail, contact_tol=0.0015, name="front_wheel_stays_on_ground_rail_open")
        ctx.expect_contact(gate, frame, elem_a=rear_wheel, elem_b=ground_rail, contact_tol=0.0015, name="rear_wheel_stays_on_ground_rail_open")
        ctx.expect_overlap(gate, frame, axes="x", elem_a=top_rail, elem_b=guide_shoe_top, min_overlap=0.17, name="top_rail_remains_captured_open")
        ctx.expect_gap(
            frame,
            gate,
            axis="x",
            positive_elem=keeper_back,
            negative_elem=latch_bolt,
            min_gap=0.98,
            name="latch_clears_keeper_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
