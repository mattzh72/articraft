from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_aframe_step_ladder")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.65, 1.0))
    dark_steel = model.material("dark_hinge_steel", rgba=(0.13, 0.14, 0.14, 1.0))
    safety_yellow = model.material("safety_yellow_tread_nosing", rgba=(0.98, 0.72, 0.06, 1.0))
    black_rubber = model.material("replaceable_black_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    brace_orange = model.material("painted_spread_brace_orange", rgba=(0.90, 0.32, 0.08, 1.0))
    bolt_zinc = model.material("zinc_plated_fasteners", rgba=(0.78, 0.80, 0.78, 1.0))

    front = model.part("front_frame")

    def add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def add_cylinder_x(part, name, radius, length, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def beam_yz(part, name, x, y0, z0, y1, z1, size_x, size_y, material):
        dy = y1 - y0
        dz = z1 - z0
        length = math.sqrt(dy * dy + dz * dz)
        roll = math.atan2(-dy, dz)
        add_box(
            part,
            name,
            (size_x, size_y, length),
            (x, (y0 + y1) / 2.0, (z0 + z1) / 2.0),
            material,
            (roll, 0.0, 0.0),
        )

    # Front ladder section: two chunky side rails tied together by deep boxed treads.
    front_top_z = 1.32
    front_foot_y = -0.53
    rail_top_y = 0.0
    rail_half_width = 0.40
    for side, x in (("0", -rail_half_width), ("1", rail_half_width)):
        beam_yz(front, f"front_rail_{side}", x, front_foot_y, 0.075, rail_top_y, front_top_z, 0.072, 0.055, galvanized)
        add_box(front, f"front_foot_cap_{side}", (0.18, 0.145, 0.080), (x, front_foot_y - 0.02, 0.040), black_rubber)
        add_box(front, f"side_wear_shoe_{side}", (0.105, 0.035, 0.020), (x, front_foot_y - 0.088, 0.063), black_rubber)

    tread_specs = [
        (0, -0.405, 0.32, 0.27),
        (1, -0.295, 0.60, 0.25),
        (2, -0.185, 0.88, 0.225),
        (3, -0.075, 1.16, 0.205),
    ]
    for idx, y, z, depth in tread_specs:
        add_box(front, f"tread_{idx}", (0.92, depth, 0.055), (0.0, y, z), galvanized)
        add_box(front, f"front_nosing_{idx}", (0.94, 0.035, 0.070), (0.0, y - depth / 2.0 - 0.006, z + 0.006), safety_yellow)
        for groove in range(3):
            gy = y - depth * 0.25 + groove * depth * 0.25
            add_box(front, f"grip_strip_{idx}_{groove}", (0.80, 0.016, 0.008), (0.0, gy, z + 0.030), black_rubber)
        add_box(front, f"tread_bolt_row_{idx}", (0.86, 0.018, 0.014), (0.0, y + depth * 0.34, z + 0.032), bolt_zinc)

    add_box(front, "top_platform", (0.96, 0.22, 0.060), (0.0, -0.155, 1.31), galvanized)
    add_box(front, "top_tool_lip", (1.00, 0.060, 0.090), (0.0, -0.245, 1.355), safety_yellow)
    add_box(front, "open_service_slot", (0.54, 0.030, 0.012), (0.0, -0.055, 1.345), dark_steel)
    add_cylinder_x(front, "top_pivot_pin", 0.026, 0.94, (0.0, 0.0, front_top_z), dark_steel)
    add_box(front, "top_hinge_backer", (0.98, 0.060, 0.080), (0.0, -0.100, front_top_z - 0.010), galvanized)

    # Side plates and pin bosses for the serviceable spread-limit braces.
    brace_anchor_y = -0.305
    brace_anchor_z = 0.525
    for idx, x in enumerate((-0.490, 0.490)):
        sign = 1.0 if x > 0.0 else -1.0
        inner_x = x - sign * 0.055
        outer_x = x + sign * 0.055
        add_box(front, f"front_brace_plate_{idx}", (0.030, 0.105, 0.095), (inner_x, brace_anchor_y, brace_anchor_z), dark_steel)
        add_box(front, f"front_clevis_outer_{idx}", (0.030, 0.105, 0.095), (outer_x, brace_anchor_y, brace_anchor_z), dark_steel)
        add_cylinder_x(front, f"front_brace_pin_{idx}", 0.018, 0.145, (x, brace_anchor_y, brace_anchor_z), bolt_zinc)
        add_box(front, f"front_inspection_tab_{idx}", (0.045, 0.065, 0.026), (inner_x, brace_anchor_y - 0.062, brace_anchor_z + 0.047), safety_yellow)

    # Rear support frame, hinged at the top cross pin.  Its local frame is the hinge line.
    rear = model.part("rear_frame")
    rear_foot_y = 0.58
    rear_foot_z = -1.320
    for side, x in (("0", -0.30), ("1", 0.30)):
        beam_yz(rear, f"rear_leg_{side}", x, rear_foot_y, rear_foot_z, 0.0, 0.0, 0.070, 0.055, galvanized)
        add_box(rear, f"rear_foot_cap_{side}", (0.18, 0.150, 0.080), (x, rear_foot_y + 0.015, rear_foot_z + 0.040), black_rubber)
    add_cylinder_x(rear, "rear_hinge_barrel", 0.034, 0.60, (0.0, 0.0, 0.0), dark_steel)
    add_box(rear, "rear_upper_crossbar", (0.86, 0.055, 0.060), (0.0, 0.085, -0.160), galvanized)
    add_box(rear, "rear_mid_crossbar", (0.83, 0.050, 0.055), (0.0, 0.330, -0.650), galvanized)
    add_box(rear, "rear_foot_bar", (0.83, 0.055, 0.050), (0.0, rear_foot_y + 0.010, rear_foot_z + 0.070), galvanized)
    for idx, x in enumerate((-0.490, 0.490)):
        sign = 1.0 if x > 0.0 else -1.0
        stop_x = x + sign * 0.065
        add_box(rear, f"rear_brace_plate_{idx}", (0.052, 0.090, 0.190), (stop_x, 0.333, -0.795), dark_steel)
        add_cylinder_x(rear, f"rear_brace_stop_pin_{idx}", 0.018, 0.550, (stop_x, 0.333, -0.705), bolt_zinc)

    rear_joint = model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, front_top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.0, lower=0.0, upper=0.62),
        motion_properties=MotionProperties(damping=1.2, friction=0.6),
    )

    # Paired spread-limit braces.  They fold with the rear section but, at the
    # open stop, each hooked end bears against a rear stop plate.
    brace_length = 0.713
    for idx, x in enumerate((-0.490, 0.490)):
        brace = model.part(f"brace_{idx}")
        add_cylinder_x(brace, "front_eye", 0.026, 0.070, (0.0, 0.0, 0.0), brace_orange)
        add_box(brace, "flat_bar", (0.034, brace_length, 0.032), (0.0, brace_length / 2.0, 0.036), brace_orange)
        add_cylinder_x(brace, "rear_eye", 0.026, 0.070, (0.0, brace_length, 0.0), brace_orange)
        add_box(brace, "rear_hook", (0.052, 0.085, 0.075), (0.0, brace_length + 0.014, 0.0), dark_steel)
        add_box(brace, "replaceable_bushing", (0.040, 0.022, 0.055), (0.0, brace_length + 0.056, 0.0), black_rubber)
        model.articulation(
            f"front_to_brace_{idx}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=brace,
            origin=Origin(xyz=(x, brace_anchor_y, brace_anchor_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=28.0, velocity=1.3, lower=0.0, upper=0.78),
            motion_properties=MotionProperties(damping=0.7, friction=0.4),
            mimic=Mimic(joint=rear_joint.name, multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    brace_0 = object_model.get_part("brace_0")
    brace_1 = object_model.get_part("brace_1")
    rear_joint = object_model.get_articulation("front_to_rear")

    ctx.allow_overlap(
        front,
        rear,
        elem_a="top_pivot_pin",
        elem_b="rear_hinge_barrel",
        reason="The rear hinge barrel intentionally captures the serviceable top pivot shaft.",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="x",
        elem_a="top_pivot_pin",
        elem_b="rear_hinge_barrel",
        min_overlap=0.55,
        name="top hinge shaft is retained across the rear barrel",
    )
    for leg_idx in range(2):
        ctx.allow_overlap(
            front,
            rear,
            elem_a="top_pivot_pin",
            elem_b=f"rear_leg_{leg_idx}",
            reason="The serviceable top pivot shaft intentionally passes through the bushed rear-leg hinge lug.",
        )
        ctx.expect_overlap(
            front,
            rear,
            axes="x",
            elem_a="top_pivot_pin",
            elem_b=f"rear_leg_{leg_idx}",
            min_overlap=0.055,
            name=f"rear leg {leg_idx} is captured on the top hinge shaft",
        )

    for idx, brace in enumerate((brace_0, brace_1)):
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"front_brace_pin_{idx}",
            elem_b="front_eye",
            reason="The spread brace eye is intentionally captured around its removable front pin.",
        )
        ctx.expect_overlap(
            front,
            brace,
            axes="x",
            elem_a=f"front_brace_pin_{idx}",
            elem_b="front_eye",
            min_overlap=0.055,
            name=f"brace {idx} front eye is retained on its pin",
        )
        ctx.expect_contact(
            brace,
            rear,
            elem_a="rear_hook",
            elem_b=f"rear_brace_plate_{idx}",
            contact_tol=0.035,
            name=f"brace {idx} hook bears on rear stop plate in the open state",
        )

    ctx.expect_gap(
        rear,
        front,
        axis="y",
        positive_elem="rear_foot_bar",
        negative_elem="front_foot_cap_0",
        min_gap=0.75,
        name="open stance spreads rear feet behind front feet",
    )

    open_rear_aabb = ctx.part_element_world_aabb(rear, elem="rear_foot_bar")
    open_brace_aabb = ctx.part_element_world_aabb(brace_0, elem="rear_hook")
    with ctx.pose({rear_joint: 0.62}):
        closed_rear_aabb = ctx.part_element_world_aabb(rear, elem="rear_foot_bar")
        closed_brace_aabb = ctx.part_element_world_aabb(brace_0, elem="rear_hook")
    ctx.check(
        "rear frame folds toward the front stop",
        open_rear_aabb is not None
        and closed_rear_aabb is not None
        and closed_rear_aabb[0][1] < open_rear_aabb[0][1] - 0.45,
        details=f"open={open_rear_aabb}, closed={closed_rear_aabb}",
    )
    ctx.check(
        "spread brace folds upward with the rear frame",
        open_brace_aabb is not None
        and closed_brace_aabb is not None
        and closed_brace_aabb[1][2] > open_brace_aabb[1][2] + 0.25,
        details=f"open={open_brace_aabb}, closed={closed_brace_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
