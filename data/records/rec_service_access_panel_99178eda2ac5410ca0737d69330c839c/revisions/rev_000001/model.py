from __future__ import annotations

import math

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
    model = ArticulatedObject(name="industrial_service_access_panel")

    frame_steel = model.material("dark_phosphate_steel", rgba=(0.09, 0.10, 0.10, 1.0))
    door_paint = model.material("powder_coated_panel_blue", rgba=(0.10, 0.20, 0.28, 1.0))
    safety_yellow = model.material("safety_yellow_guard", rgba=(1.0, 0.72, 0.05, 1.0))
    red_lockout = model.material("red_lockout_marking", rgba=(0.82, 0.04, 0.02, 1.0))
    black_rubber = model.material("black_rubber_stop", rgba=(0.01, 0.01, 0.008, 1.0))
    bare_steel = model.material("worn_bare_steel", rgba=(0.58, 0.58, 0.55, 1.0))
    bolt_dark = model.material("dark_socket_head_bolts", rgba=(0.015, 0.015, 0.014, 1.0))

    def box(part, name: str, size, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl(part, name: str, radius: float, length: float, xyz, material: Material, rpy=(0.0, 0.0, 0.0)) -> None:
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Root support: a welded, box-section frame around a real empty service opening.
    frame = model.part("frame")
    box(frame, "left_rail", (0.12, 0.10, 1.80), (-0.59, 0.0, 0.90), frame_steel)
    box(frame, "right_rail", (0.12, 0.10, 1.80), (0.59, 0.0, 0.90), frame_steel)
    box(frame, "top_rail", (1.30, 0.10, 0.12), (0.0, 0.0, 1.74), frame_steel)
    box(frame, "bottom_rail", (1.30, 0.10, 0.12), (0.0, 0.0, 0.06), frame_steel)
    box(frame, "inner_lip_top", (1.02, 0.020, 0.040), (0.0, 0.051, 1.64), black_rubber)
    box(frame, "inner_lip_bottom", (1.02, 0.020, 0.040), (0.0, 0.051, 0.16), black_rubber)
    box(frame, "inner_lip_hinge", (0.040, 0.020, 1.44), (-0.46, 0.051, 0.90), black_rubber)
    box(frame, "inner_lip_latch", (0.040, 0.020, 1.44), (0.50, 0.051, 0.90), black_rubber)

    # Hinge-side load path: guard plate, interleaved hinge knuckles, and a captured pin.
    box(frame, "hinge_guard", (0.050, 0.130, 1.64), (-0.665, 0.085, 0.90), safety_yellow)
    cyl(frame, "hinge_pin", 0.012, 1.60, (-0.53, 0.085, 0.90), bare_steel)
    for i, z in enumerate((0.185, 0.675, 1.125, 1.615)):
        cyl(frame, f"frame_hinge_barrel_{i}", 0.038, 0.160, (-0.53, 0.085, z), bare_steel)
        box(frame, f"frame_hinge_leaf_{i}", (0.11, 0.030, 0.150), (-0.585, 0.063, z), frame_steel)

    # Latch side receiver with guarded slot.  The slot is open between the upper and lower lugs.
    box(frame, "keeper_backer", (0.13, 0.080, 0.28), (0.605, 0.125, 0.90), frame_steel)
    box(frame, "keeper_upper_lug", (0.19, 0.080, 0.035), (0.53, 0.185, 0.975), safety_yellow)
    box(frame, "keeper_lower_lug", (0.19, 0.080, 0.035), (0.53, 0.185, 0.825), safety_yellow)
    box(frame, "keeper_side_guard", (0.040, 0.120, 0.28), (0.705, 0.185, 0.90), safety_yellow)
    box(frame, "keeper_weld_web", (0.080, 0.100, 0.22), (0.60, 0.075, 0.90), frame_steel)
    box(frame, "keeper_bridge_upper", (0.060, 0.040, 0.035), (0.655, 0.185, 0.975), safety_yellow)
    box(frame, "keeper_bridge_lower", (0.060, 0.040, 0.035), (0.655, 0.185, 0.825), safety_yellow)

    # Closed stops, open over-travel stop, and rubber faces backed by the frame.
    box(frame, "closed_stop_upper", (0.10, 0.018, 0.080), (0.49, 0.052, 1.54), black_rubber)
    box(frame, "closed_stop_lower", (0.10, 0.018, 0.080), (0.49, 0.052, 0.26), black_rubber)
    box(frame, "open_stop_bracket", (0.080, 0.120, 0.22), (-0.64, 0.190, 1.48), frame_steel)
    box(frame, "open_stop_face", (0.050, 0.030, 0.18), (-0.615, 0.255, 1.48), black_rubber)

    # Visible socket-head bolt pattern on load-carrying plates.
    for side_x in (-0.59, 0.59):
        for z in (0.28, 0.58, 0.90, 1.22, 1.52):
            cyl(frame, f"rail_bolt_{side_x:+.2f}_{z:.2f}", 0.018, 0.014, (side_x, 0.056, z), bolt_dark, rpy=(-math.pi / 2.0, 0.0, 0.0))
    for z in (0.80, 1.00):
        cyl(frame, f"keeper_bolt_{z:.2f}", 0.016, 0.014, (0.595, 0.160, z), bolt_dark, rpy=(-math.pi / 2.0, 0.0, 0.0))

    # Swinging service panel.  Its local frame is the hinge line; closed panel extends along +X.
    door = model.part("door")
    box(door, "panel_plate", (1.02, 0.045, 1.42), (0.55, 0.0, 0.0), door_paint)
    box(door, "hinge_doubler", (0.14, 0.032, 1.30), (0.105, 0.037, 0.0), frame_steel)
    box(door, "latch_doubler", (0.14, 0.032, 1.20), (0.97, 0.037, 0.0), frame_steel)
    box(door, "top_stiffener", (0.88, 0.035, 0.060), (0.56, 0.038, 0.60), frame_steel)
    box(door, "bottom_stiffener", (0.88, 0.035, 0.060), (0.56, 0.038, -0.60), frame_steel)
    box(door, "diagonal_brace_0", (1.265, 0.035, 0.055), (0.52, 0.046, 0.0), safety_yellow, rpy=(0.0, -0.965, 0.0))
    box(door, "diagonal_brace_1", (1.265, 0.035, 0.055), (0.52, 0.046, 0.0), safety_yellow, rpy=(0.0, 0.965, 0.0))

    for i, z in enumerate((-0.45, 0.0, 0.45)):
        cyl(door, f"door_hinge_barrel_{i}", 0.034, 0.280, (0.0, 0.0, z), bare_steel)
        box(door, f"door_hinge_leaf_{i}", (0.14, 0.050, 0.280), (0.085, 0.025, z), frame_steel)

    box(door, "latch_bearing_plate", (0.22, 0.032, 0.18), (0.86, 0.056, 0.0), frame_steel)
    box(door, "lockout_upper_ear", (0.085, 0.028, 0.090), (0.75, 0.064, 0.125), red_lockout)
    box(door, "lockout_lower_ear", (0.085, 0.028, 0.090), (0.75, 0.064, -0.125), red_lockout)
    cyl(door, "upper_lockout_hole", 0.017, 0.006, (0.75, 0.073, 0.125), bolt_dark, rpy=(-math.pi / 2.0, 0.0, 0.0))
    cyl(door, "lower_lockout_hole", 0.017, 0.006, (0.75, 0.073, -0.125), bolt_dark, rpy=(-math.pi / 2.0, 0.0, 0.0))

    # Guard rails around the handle: tied back to the bearing plate, not decorative floating bars.
    box(door, "latch_guard_top", (0.46, 0.040, 0.040), (0.90, 0.125, 0.245), safety_yellow)
    box(door, "latch_guard_bottom", (0.46, 0.040, 0.040), (0.90, 0.125, -0.430), safety_yellow)
    box(door, "latch_guard_left_foot", (0.050, 0.125, 0.10), (0.69, 0.074, 0.245), safety_yellow)
    box(door, "latch_guard_right_foot", (0.050, 0.125, 0.10), (1.11, 0.074, 0.245), safety_yellow)
    box(door, "latch_guard_left_foot_lower", (0.050, 0.125, 0.10), (0.69, 0.074, -0.430), safety_yellow)
    box(door, "latch_guard_right_foot_lower", (0.050, 0.125, 0.10), (1.11, 0.074, -0.430), safety_yellow)

    for x, z in ((0.77, 0.07), (0.95, 0.07), (0.77, -0.07), (0.95, -0.07)):
        cyl(door, f"bearing_bolt_{x:.2f}_{z:+.2f}", 0.014, 0.010, (x, 0.068, z), bolt_dark, rpy=(-math.pi / 2.0, 0.0, 0.0))
    for x in (0.18, 0.50, 0.85):
        for z in (0.60, -0.60):
            cyl(door, f"stiffener_bolt_{x:.2f}_{z:+.2f}", 0.014, 0.010, (x, 0.060, z), bolt_dark, rpy=(-math.pi / 2.0, 0.0, 0.0))

    door_hinge = model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-0.53, 0.085, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.75, lower=0.0, upper=1.75),
    )
    door_hinge.meta["description"] = "Vertical hinge; positive motion swings the latch side outward from the frame."

    # A separate rotary latch: the yellow bar closes into the frame keeper and lifts to release.
    latch = model.part("latch")
    cyl(latch, "pivot_shaft", 0.018, 0.055, (0.0, -0.020, 0.0), bare_steel, rpy=(-math.pi / 2.0, 0.0, 0.0))
    cyl(latch, "pivot_hub", 0.055, 0.045, (0.0, 0.025, 0.0), bare_steel, rpy=(-math.pi / 2.0, 0.0, 0.0))
    box(latch, "latch_bar", (0.34, 0.035, 0.045), (0.17, 0.020, 0.0), safety_yellow)
    box(latch, "handle_grip", (0.065, 0.040, 0.30), (-0.065, 0.020, -0.17), frame_steel)
    box(latch, "lockout_tab", (0.090, 0.026, 0.070), (-0.005, 0.023, -0.335), red_lockout)
    cyl(latch, "tab_lockout_hole", 0.018, 0.006, (-0.005, 0.039, -0.335), bolt_dark, rpy=(-math.pi / 2.0, 0.0, 0.0))

    latch_joint = model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.86, 0.090, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=0.0, upper=1.05),
    )
    latch_joint.meta["description"] = "Positive motion lifts the latch bar out of the keeper slot."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("frame_to_door")
    latch_joint = object_model.get_articulation("door_to_latch")

    for barrel_name in ("door_hinge_barrel_0", "door_hinge_barrel_1", "door_hinge_barrel_2"):
        ctx.allow_overlap(
            frame,
            door,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            reason="The hinge pin is intentionally captured inside the service-door hinge barrel.",
        )
        ctx.expect_within(
            frame,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=barrel_name,
            margin=0.002,
            name=f"{barrel_name} contains the hinge pin",
        )
        ctx.expect_overlap(
            frame,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=barrel_name,
            min_overlap=0.24,
            name=f"{barrel_name} has retained pin engagement",
        )

    ctx.allow_overlap(
        door,
        latch,
        elem_a="latch_bearing_plate",
        elem_b="pivot_shaft",
        reason="The latch shaft intentionally passes through the reinforced bearing plate.",
    )
    ctx.expect_within(
        latch,
        door,
        axes="xz",
        inner_elem="pivot_shaft",
        outer_elem="latch_bearing_plate",
        margin=0.002,
        name="latch shaft is centered in the bearing plate",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="y",
        elem_a="pivot_shaft",
        elem_b="latch_bearing_plate",
        min_overlap=0.020,
        name="latch shaft passes through bearing plate thickness",
    )

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            door,
            frame,
            axis="y",
            positive_elem="panel_plate",
            negative_elem="right_rail",
            min_gap=0.010,
            max_gap=0.020,
            name="closed door sits proud of the latch-side frame",
        )
        ctx.expect_gap(
            frame,
            latch,
            axis="z",
            positive_elem="keeper_upper_lug",
            negative_elem="latch_bar",
            min_gap=0.020,
            max_gap=0.060,
            name="closed latch bar clears upper keeper lug",
        )
        ctx.expect_gap(
            latch,
            frame,
            axis="z",
            positive_elem="latch_bar",
            negative_elem="keeper_lower_lug",
            min_gap=0.020,
            max_gap=0.060,
            name="closed latch bar clears lower keeper lug",
        )
        ctx.expect_overlap(
            latch,
            frame,
            axes="xy",
            elem_a="latch_bar",
            elem_b="keeper_upper_lug",
            min_overlap=0.030,
            name="latch bar reaches into keeper footprint",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(door, elem="panel_plate")
        closed_bar_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")

    with ctx.pose({door_hinge: 1.20, latch_joint: 0.0}):
        swung_panel_aabb = ctx.part_element_world_aabb(door, elem="panel_plate")

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.75}):
        raised_bar_aabb = ctx.part_element_world_aabb(latch, elem="latch_bar")

    ctx.check(
        "door swing moves latch side outward",
        closed_panel_aabb is not None
        and swung_panel_aabb is not None
        and swung_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.40,
        details=f"closed={closed_panel_aabb}, swung={swung_panel_aabb}",
    )
    ctx.check(
        "latch release lifts the keeper end",
        closed_bar_aabb is not None
        and raised_bar_aabb is not None
        and raised_bar_aabb[1][2] > closed_bar_aabb[1][2] + 0.12,
        details=f"closed={closed_bar_aabb}, raised={raised_bar_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
