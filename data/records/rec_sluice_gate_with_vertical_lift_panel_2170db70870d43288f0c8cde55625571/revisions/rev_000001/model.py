from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_sluice_gate")

    concrete = model.material("weathered_concrete", rgba=(0.46, 0.45, 0.40, 1.0))
    mortar = model.material("dark_mortar", rgba=(0.22, 0.22, 0.20, 1.0))
    guide_steel = model.material("blackened_guide_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    gate_steel = model.material("galvanized_gate_steel", rgba=(0.42, 0.49, 0.52, 1.0))
    rib_steel = model.material("darker_welded_ribs", rgba=(0.25, 0.31, 0.34, 1.0))
    gearbox_paint = model.material("industrial_green_gearbox", rgba=(0.08, 0.22, 0.17, 1.0))
    cover_paint = model.material("slightly_lighter_cover", rgba=(0.10, 0.28, 0.22, 1.0))
    wheel_red = model.material("red_crank_wheel", rgba=(0.72, 0.05, 0.03, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.01, 0.01, 0.01, 1.0))
    water = model.material("muted_water", rgba=(0.10, 0.32, 0.45, 0.45))
    white = model.material("painted_scale_white", rgba=(0.88, 0.88, 0.82, 1.0))
    black = model.material("scale_mark_black", rgba=(0.02, 0.02, 0.02, 1.0))

    frame = model.part("masonry_frame")

    # Masonry water-channel frame: two piers, a sill, and a lintel leave the
    # central channel open instead of representing the wall as a solid slab.
    frame.visual(Box((0.50, 0.70, 2.15)), origin=Origin(xyz=(-1.05, 0.0, 1.375)), material=concrete, name="left_pier")
    frame.visual(Box((0.50, 0.70, 2.15)), origin=Origin(xyz=(1.05, 0.0, 1.375)), material=concrete, name="right_pier")
    frame.visual(Box((2.60, 0.75, 0.30)), origin=Origin(xyz=(0.0, 0.0, 0.15)), material=concrete, name="sill")
    frame.visual(Box((2.60, 0.75, 0.35)), origin=Origin(xyz=(0.0, 0.0, 2.625)), material=concrete, name="lintel")
    frame.visual(Box((1.46, 0.62, 0.08)), origin=Origin(xyz=(0.0, 0.03, 0.04)), material=mortar, name="wet_channel_floor")
    frame.visual(Box((1.42, 0.60, 1.16)), origin=Origin(xyz=(0.0, 0.02, 0.62)), material=water, name="water_volume")

    # Mortar course lines and individual block cues, embedded in the masonry.
    for z in (0.64, 0.98, 1.32, 1.66, 2.00):
        frame.visual(Box((0.51, 0.012, 0.018)), origin=Origin(xyz=(-1.05, -0.356, z)), material=mortar, name=f"left_mortar_{z:.2f}")
        frame.visual(Box((0.51, 0.012, 0.018)), origin=Origin(xyz=(1.05, -0.356, z)), material=mortar, name=f"right_mortar_{z:.2f}")
    for x in (-0.65, 0.0, 0.65):
        frame.visual(Box((0.022, 0.012, 0.28)), origin=Origin(xyz=(x, -0.381, 2.62)), material=mortar, name=f"lintel_joint_{x:.2f}")

    # Steel guide jambs are U-channel-like assemblies bolted to the masonry
    # face, leaving a clear slot for the gate plate.
    for side, sx in (("left", -1.0), ("right", 1.0)):
        web_x = sx * 0.69
        lip_x = sx * 0.63
        frame.visual(Box((0.08, 0.20, 2.15)), origin=Origin(xyz=(web_x, -0.45, 1.375)), material=guide_steel, name=f"{side}_guide_web")
        frame.visual(Box((0.16, 0.045, 2.15)), origin=Origin(xyz=(lip_x, -0.5725, 1.375)), material=guide_steel, name=f"{side}_front_lip")
        frame.visual(Box((0.16, 0.045, 2.15)), origin=Origin(xyz=(lip_x, -0.38, 1.375)), material=guide_steel, name=f"{side}_rear_lip")
        frame.visual(Box((0.24, 0.05, 0.12)), origin=Origin(xyz=(web_x, -0.50, 0.32)), material=guide_steel, name=f"{side}_guide_foot")
        frame.visual(Box((0.24, 0.05, 0.12)), origin=Origin(xyz=(web_x, -0.50, 2.42)), material=guide_steel, name=f"{side}_guide_head")
        for z in (0.55, 1.05, 1.55, 2.05):
            frame.visual(Cylinder(radius=0.025, length=0.014), origin=Origin(xyz=(web_x, -0.565, z), rpy=(math.pi / 2, 0.0, 0.0)), material=gate_steel, name=f"{side}_jamb_bolt_{z:.2f}")

    # Lift-height scale fixed to the service side of one pier.
    frame.visual(Box((0.055, 0.008, 1.45)), origin=Origin(xyz=(0.90, -0.365, 1.18)), material=white, name="lift_scale_strip")
    for i in range(9):
        tick_z = 0.50 + i * 0.16
        tick_width = 0.050 if i % 2 == 0 else 0.032
        frame.visual(Box((tick_width, 0.010, 0.010)), origin=Origin(xyz=(0.90, -0.372, tick_z)), material=black, name=f"scale_tick_{i}")

    # Compact gearbox perched on the lintel with an exposed side input boss.
    frame.visual(Box((0.55, 0.38, 0.38)), origin=Origin(xyz=(0.0, -0.45, 3.02)), material=gearbox_paint, name="gearbox_body")
    frame.visual(Box((0.68, 0.46, 0.08)), origin=Origin(xyz=(0.0, -0.45, 2.84)), material=gearbox_paint, name="gearbox_foot")
    frame.visual(Cylinder(radius=0.055, length=0.10), origin=Origin(xyz=(0.0, -0.58, 2.78)), material=guide_steel, name="output_collar")
    frame.visual(Cylinder(radius=0.070, length=0.08), origin=Origin(xyz=(0.315, -0.45, 3.05), rpy=(0.0, math.pi / 2, 0.0)), material=guide_steel, name="input_bearing")
    for x in (-0.25, 0.25):
        for y in (-0.59, -0.31):
            frame.visual(Cylinder(radius=0.028, length=0.018), origin=Origin(xyz=(x, y, 2.89)), material=guide_steel, name=f"gearbox_anchor_{x:.2f}_{y:.2f}")

    # Fixed hinge knuckles for the service cover. The middle knuckle belongs to
    # the cover part below, so the side hinge remains visually articulated.
    frame.visual(Cylinder(radius=0.018, length=0.10), origin=Origin(xyz=(-0.31, -0.72, 2.89)), material=guide_steel, name="cover_hinge_lower")
    frame.visual(Cylinder(radius=0.018, length=0.10), origin=Origin(xyz=(-0.31, -0.72, 3.15)), material=guide_steel, name="cover_hinge_upper")
    frame.visual(Box((0.036, 0.090, 0.105)), origin=Origin(xyz=(-0.337, -0.680, 2.89)), material=guide_steel, name="lower_hinge_leaf")
    frame.visual(Box((0.036, 0.090, 0.105)), origin=Origin(xyz=(-0.337, -0.680, 3.15)), material=guide_steel, name="upper_hinge_leaf")
    frame.visual(Box((0.090, 0.030, 0.105)), origin=Origin(xyz=(-0.305, -0.635, 2.89)), material=guide_steel, name="lower_hinge_bridge")
    frame.visual(Box((0.090, 0.030, 0.105)), origin=Origin(xyz=(-0.305, -0.635, 3.15)), material=guide_steel, name="upper_hinge_bridge")

    gate = model.part("lift_panel")
    gate.visual(Box((1.06, 0.08, 1.35)), origin=Origin(xyz=(0.0, 0.0, 0.675)), material=gate_steel, name="gate_plate")
    gate.visual(Box((1.10, 0.10, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.023)), material=rubber, name="bottom_seal")
    gate.visual(Box((1.10, 0.11, 0.055)), origin=Origin(xyz=(0.0, 0.0, 1.362)), material=rib_steel, name="top_cap")
    for z in (0.34, 0.72, 1.10):
        gate.visual(Box((1.00, 0.045, 0.060)), origin=Origin(xyz=(0.0, -0.055, z)), material=rib_steel, name=f"horizontal_rib_{z:.2f}")
    for x in (-0.34, 0.0, 0.34):
        gate.visual(Box((0.055, 0.045, 1.18)), origin=Origin(xyz=(x, -0.055, 0.72)), material=rib_steel, name=f"vertical_stiffener_{x:.2f}")
    gate.visual(Box((0.12, 0.12, 1.20)), origin=Origin(xyz=(-0.59, 0.0, 0.72)), material=rib_steel, name="left_guide_shoe")
    gate.visual(Box((0.12, 0.12, 1.20)), origin=Origin(xyz=(0.59, 0.0, 0.72)), material=rib_steel, name="right_guide_shoe")
    gate.visual(Box((0.22, 0.075, 0.10)), origin=Origin(xyz=(0.0, -0.075, 1.40)), material=rib_steel, name="lifting_lug")
    gate.visual(Cylinder(radius=0.030, length=0.76), origin=Origin(xyz=(0.0, -0.10, 1.77)), material=guide_steel, name="lift_stem")
    gate.visual(Box((0.070, 0.035, 0.16)), origin=Origin(xyz=(0.545, -0.045, 0.78)), material=black, name="scale_pointer")

    cover = model.part("gearbox_cover")
    cover.visual(Box((0.55, 0.045, 0.36)), origin=Origin(xyz=(0.295, 0.0175, 0.0)), material=cover_paint, name="cover_plate")
    cover.visual(Cylinder(radius=0.018, length=0.10), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=guide_steel, name="cover_hinge_knuckle")
    cover.visual(Box((0.030, 0.055, 0.10)), origin=Origin(xyz=(0.018, 0.030, 0.0)), material=guide_steel, name="cover_hinge_leaf")
    for x, z in ((0.11, 0.12), (0.47, 0.12), (0.11, -0.12), (0.47, -0.12)):
        cover.visual(Cylinder(radius=0.017, length=0.018), origin=Origin(xyz=(x, -0.012, z), rpy=(math.pi / 2, 0.0, 0.0)), material=guide_steel, name=f"cover_bolt_{x:.2f}_{z:.2f}")
    cover.visual(Cylinder(radius=0.008, length=0.080), origin=Origin(xyz=(0.43, -0.043, 0.07), rpy=(math.pi / 2, 0.0, 0.0)), material=guide_steel, name="handle_post_top")
    cover.visual(Cylinder(radius=0.008, length=0.080), origin=Origin(xyz=(0.43, -0.043, -0.07), rpy=(math.pi / 2, 0.0, 0.0)), material=guide_steel, name="handle_post_bottom")
    cover.visual(Cylinder(radius=0.012, length=0.14), origin=Origin(xyz=(0.43, -0.082, 0.0)), material=guide_steel, name="service_handle")

    wheel = model.part("crank_wheel")
    handwheel = WheelGeometry(
        0.26,
        0.035,
        rim=WheelRim(inner_radius=0.215, flange_height=0.012, flange_thickness=0.008),
        hub=WheelHub(radius=0.055, width=0.070, cap_style="domed"),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.014, window_radius=0.045),
        bore=WheelBore(style="round", diameter=0.034),
    )
    wheel.visual(mesh_from_geometry(handwheel, "red_handwheel"), origin=Origin(xyz=(0.24, 0.0, 0.0)), material=wheel_red, name="handwheel")
    wheel.visual(Cylinder(radius=0.030, length=0.24), origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)), material=guide_steel, name="input_shaft")
    wheel.visual(Cylinder(radius=0.026, length=0.070), origin=Origin(xyz=(0.255, 0.0, 0.23), rpy=(0.0, math.pi / 2, 0.0)), material=guide_steel, name="rim_grip")

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate,
        origin=Origin(xyz=(0.0, -0.48, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.08, lower=0.0, upper=0.70),
    )
    model.articulation(
        "frame_to_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(-0.31, -0.72, 3.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.8, lower=0.0, upper=1.75),
    )
    model.articulation(
        "frame_to_crank",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.355, -0.45, 3.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("masonry_frame")
    gate = object_model.get_part("lift_panel")
    cover = object_model.get_part("gearbox_cover")
    crank = object_model.get_part("crank_wheel")
    panel_slide = object_model.get_articulation("frame_to_panel")
    cover_hinge = object_model.get_articulation("frame_to_cover")
    crank_joint = object_model.get_articulation("frame_to_crank")

    with ctx.pose({panel_slide: 0.0}):
        ctx.expect_gap(gate, frame, axis="z", positive_elem="bottom_seal", negative_elem="sill", min_gap=0.0, max_gap=0.003, name="closed gate seal seats on sill")
        ctx.expect_gap(frame, gate, axis="x", positive_elem="right_guide_web", negative_elem="gate_plate", min_gap=0.08, max_gap=0.18, name="right guide flanks the gate")
        ctx.expect_gap(gate, frame, axis="x", positive_elem="gate_plate", negative_elem="left_guide_web", min_gap=0.08, max_gap=0.18, name="left guide flanks the gate")

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({panel_slide: 0.70}):
        raised_pos = ctx.part_world_position(gate)
        ctx.expect_gap(gate, frame, axis="z", positive_elem="bottom_seal", negative_elem="sill", min_gap=0.68, max_gap=0.72, name="lift panel opens upward")
    ctx.check(
        "panel prismatic motion is vertical",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.65,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    ctx.expect_contact(crank, frame, elem_a="input_shaft", elem_b="input_bearing", contact_tol=0.002, name="crank shaft meets gearbox bearing")
    ctx.check(
        "crank is continuous",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_joint.articulation_type}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    with ctx.pose({cover_hinge: 1.25}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_plate")
    closed_cover_y = None if closed_cover_aabb is None else (closed_cover_aabb[0][1] + closed_cover_aabb[1][1]) / 2.0
    open_cover_y = None if open_cover_aabb is None else (open_cover_aabb[0][1] + open_cover_aabb[1][1]) / 2.0
    ctx.check(
        "cover swings outward on side hinge",
        closed_cover_y is not None and open_cover_y is not None and open_cover_y < closed_cover_y - 0.15,
        details=f"closed_y={closed_cover_y}, open_y={open_cover_y}",
    )

    return ctx.report()


object_model = build_object_model()
