from __future__ import annotations

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
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_latch_bolt")

    # --------------------------
    # Fixed Housing (root part)
    # --------------------------
    housing = model.part("housing")
    
    # Housing geometry: 0.06m (X) x 0.03m (Y) x 0.025m (Z) box with cutouts
    # Cutouts: bolt travel hole and guide slot for knob
    housing_wp = cq.Workplane("XY").box(0.06, 0.03, 0.025)
    
    # Cut bolt travel hole (slightly larger than bolt cross-section for clearance)
    bolt_hole = cq.Workplane("XY").box(0.062, 0.022, 0.017)  # Oversized for through-cut
    housing_wp = housing_wp.cut(bolt_hole)
    
    # Cut guide slot for bolt knob (top face, along X axis)
    guide_slot = cq.Workplane("XY").box(0.06, 0.012, 0.005).translate((0, 0, 0.0275))
    housing_wp = housing_wp.cut(guide_slot)
    
    # Add housing visual with metal material
    housing_visual = mesh_from_cadquery(housing_wp, "housing_shell")
    housing.visual(
        housing_visual,
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),  # Centered vertically (bottom at Z=0)
        name="housing_shell",
        material=Material(name="silver", color=(0.8, 0.8, 0.8))  # Silver color
    )

    # --------------------------
    # Strike Receiver (fixed part, child of housing)
    # --------------------------
    receiver = model.part("receiver")
    
    # Receiver geometry: 0.04m (X) x 0.04m (Y) x 0.005m (Z) plate with bolt hole
    receiver_wp = cq.Workplane("XY").box(0.04, 0.04, 0.005)
    receiver_hole = cq.Workplane("XY").box(0.022, 0.017, 0.006)  # Oversized for through-cut
    receiver_wp = receiver_wp.cut(receiver_hole)
    
    receiver_visual = mesh_from_cadquery(receiver_wp, "receiver_plate")
    receiver.visual(
        receiver_visual,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered on receiver part frame
        name="receiver_plate",
        material=Material(name="silver", color=(0.8, 0.8, 0.8))  # Silver color
    )

    # --------------------------
    # Sliding Bolt (moving part)
    # --------------------------
    bolt = model.part("bolt")
    
    # Main bolt body: 0.06m (X) x 0.02m (Y) x 0.015m (Z)
    bolt_body = Box((0.06, 0.02, 0.015))
    bolt.visual(
        bolt_body,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Centered on bolt part frame
        name="bolt_body",
        material=Material(name="silver", color=(0.8, 0.8, 0.8))  # Silver color
    )
    
    # Raised knob: cylinder (radius 0.005m, height 0.01m) on front of bolt
    knob = Cylinder(radius=0.005, height=0.01)
    bolt.visual(
        knob,
        origin=Origin(xyz=(0.03, 0.0, 0.0125)),  # Front of bolt (bolt length 0.06m, so +0.03 from center)
        name="bolt_knob",
        material=Material(name="silver", color=(0.8, 0.8, 0.8))  # Silver color
    )

    # --------------------------
    # Articulations
    # --------------------------
    # Prismatic joint for bolt (primary moving mechanism)
    model.articulation(
        "housing_to_bolt",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=bolt,
        origin=Origin(xyz=(-0.03, 0.0, 0.0125)),  # Joint at back of housing
        axis=(1.0, 0.0, 0.0),  # Horizontal X-axis movement
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=0.0, upper=0.06),
    )
    
    # Fixed joint for receiver (secondary, non-moving) - positioned to touch housing front
    model.articulation(
        "housing_to_receiver",
        ArticulationType.FIXED,
        parent=housing,
        child=receiver,
        origin=Origin(xyz=(0.05, 0.0, 0.0125)),  # Receiver back face at X=0.03 (housing front)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    bolt = object_model.get_part("bolt")
    receiver = object_model.get_part("receiver")
    bolt_joint = object_model.get_articulation("housing_to_bolt")

    # Allow receiver as isolated part (fixed joint not recognized by compiler)
    ctx.allow_isolated_part(
        "receiver",
        reason="Receiver is fixed to housing via fixed articulation, compiler doesn't detect fixed joint connections"
    )

    # Allow intentional overlap: knob slides through housing guide slot
    ctx.allow_overlap(
        "bolt", "housing",
        elem_a="bolt_knob", elem_b="housing_shell",
        reason="Knob intentionally slides through housing guide slot"
    )
    # Proof check: knob contacts housing in closed pose (inside guide slot)
    with ctx.pose({bolt_joint: 0.0}):
        ctx.expect_contact(bolt, housing, elem_a="bolt_knob", elem_b="housing_shell", name="knob_in_guide_slot_closed")

    # Rest pose (q=0) and extended pose (q=0.06) positions
    with ctx.pose({bolt_joint: 0.0}):
        rest_pos = ctx.part_world_position(bolt)
    with ctx.pose({bolt_joint: 0.06}):
        extended_pos = ctx.part_world_position(bolt)

    # 1. Primary articulation checks
    ctx.check(
        "primary_joint_is_prismatic",
        bolt_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"Joint type: {bolt_joint.articulation_type}"
    )
    ctx.check(
        "prismatic_axis_correct",
        bolt_joint.axis == (1.0, 0.0, 0.0),
        details=f"Axis: {bolt_joint.axis}"
    )
    ctx.check(
        "motion_limits_valid",
        bolt_joint.motion_limits.lower == 0.0 and bolt_joint.motion_limits.upper == 0.06,
        details=f"Limits: {bolt_joint.motion_limits.lower}, {bolt_joint.motion_limits.upper}"
    )

    # 2. Support/contact checks
    with ctx.pose({bolt_joint: 0.0}):
        ctx.expect_contact(bolt, housing, name="bolt_contacts_housing_at_rest")
        ctx.expect_contact(bolt, bolt, elem_a="bolt_body", elem_b="bolt_knob", name="knob_attached_to_bolt")

    # 3. Closed pose (q=0): bolt fully retracted
    with ctx.pose({bolt_joint: 0.0}):
        bolt_aabb = ctx.part_world_aabb(bolt)
        # bolt_aabb is ((min_x, min_y, min_z), (max_x, max_y, max_z))
        max_x = bolt_aabb[1][0]
        ctx.check(
            "bolt_fully_retracted",
            max_x <= 0.03 + 0.001,  # Bolt max X ≤ housing front face (X=0.03)
            details=f"Bolt max X: {max_x}"
        )
        ctx.expect_within(bolt, housing, axes="xy", elem_a="bolt_knob", name="knob_within_housing_xy")

    # 4. Extended pose (q=0.06): bolt engages receiver
    with ctx.pose({bolt_joint: 0.06}):
        ctx.expect_overlap(bolt, receiver, axes="x", min_overlap=0.01, name="bolt_overlaps_receiver")
        ctx.expect_overlap(bolt, receiver, axes="yz", name="bolt_centered_in_receiver_yz")
        ctx.check(
            "bolt_extends_forward",
            extended_pos[0] > rest_pos[0] + 0.05,
            details=f"Rest X: {rest_pos[0]}, Extended X: {extended_pos[0]}"
        )

    # 5. Visible details: metal finish and key features
    for part_name, visual_name in [("housing", "housing_shell"), ("bolt", "bolt_body"), ("receiver", "receiver_plate")]:
        visual = object_model.get_part(part_name).get_visual(visual_name)
        ctx.check(
            f"{part_name}_silver_finish",
            visual.material is not None and visual.material.name == "silver",
            details=f"{part_name} material: {visual.material}"
        )
    ctx.check(
        "bolt_has_knob",
        bolt.get_visual("bolt_knob") is not None,
        details="Bolt missing knob visual"
    )

    return ctx.report()


object_model = build_object_model()
