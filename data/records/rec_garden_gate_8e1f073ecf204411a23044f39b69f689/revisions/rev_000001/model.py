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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="picket_garden_gate")

    post_w = 0.09
    post_d = 0.09
    post_h = 1.35
    opening = 1.00

    gate_w = 0.985
    gate_t = 0.04
    gate_h = 1.10
    gate_clearance = 0.06

    stile_w = 0.045
    rail_h = 0.09

    lower_rail_z = gate_clearance + 0.12
    mid_rail_z = gate_clearance + 0.46
    upper_rail_z = gate_clearance + 0.72

    ring_x = 0.84
    ring_y = 0.0335
    ring_pivot_z = gate_clearance + 0.79

    bolt_x = 0.945
    bolt_visual_y = 0.028
    bolt_origin_z = gate_clearance + 0.345
    upper_guide_z = gate_clearance + 0.34
    lower_guide_z = gate_clearance + 0.16

    receiver_x = -bolt_visual_y
    receiver_y = bolt_x

    wood = model.material("wood", rgba=(0.66, 0.52, 0.34, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.59, 0.47, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    iron = model.material("iron", rgba=(0.14, 0.15, 0.16, 1.0))
    stone = model.material("stone", rgba=(0.63, 0.62, 0.58, 1.0))

    def mesh_named(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def make_picket_mesh(name: str, width: float, height: float, thickness: float, tip_height: float):
        profile = [
            (-width / 2.0, 0.0),
            (width / 2.0, 0.0),
            (width / 2.0, height - tip_height),
            (0.0, height),
            (-width / 2.0, height - tip_height),
        ]
        geom = ExtrudeGeometry.centered(profile, thickness)
        geom.rotate_x(math.pi / 2.0)
        return mesh_named(name, geom)

    picket_mesh = make_picket_mesh(
        "garden_gate_picket",
        width=0.085,
        height=0.98,
        thickness=0.018,
        tip_height=0.11,
    )

    ring_geom = TorusGeometry(radius=0.045, tube=0.0055, radial_segments=16, tubular_segments=36)
    ring_geom.rotate_x(math.pi / 2.0)
    ring_mesh = mesh_named("garden_gate_ring", ring_geom)

    gate_frame = model.part("gate_frame")
    gate_frame.visual(
        Box((opening + 2.0 * post_w, 0.16, 0.03)),
        origin=Origin(xyz=(opening / 2.0, 0.0, -0.015)),
        material=stone,
        name="threshold_sill",
    )
    gate_frame.visual(
        Box((0.10, receiver_y, 0.025)),
        origin=Origin(xyz=(receiver_x, receiver_y / 2.0, -0.0125)),
        material=stone,
        name="open_hold_path",
    )
    gate_frame.visual(
        Box((post_w, post_d, post_h)),
        origin=Origin(xyz=(-post_w / 2.0, 0.0, post_h / 2.0)),
        material=weathered_wood,
        name="hinge_post",
    )
    gate_frame.visual(
        Box((post_w, post_d, post_h)),
        origin=Origin(xyz=(opening + post_w / 2.0, 0.0, post_h / 2.0)),
        material=weathered_wood,
        name="latch_post",
    )
    gate_frame.visual(
        Box((0.015, gate_t, 0.80)),
        origin=Origin(xyz=(gate_w + 0.0075, 0.0, gate_clearance + 0.43)),
        material=weathered_wood,
        name="closing_stop",
    )
    gate_frame.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(-0.012, 0.0, gate_clearance + 0.78), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="upper_hinge_knuckle",
    )
    gate_frame.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(-0.012, 0.0, gate_clearance + 0.26), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lower_hinge_knuckle",
    )
    gate_frame.visual(
        Box((0.028, 0.008, 0.11)),
        origin=Origin(xyz=(-0.014, 0.0, gate_clearance + 0.78)),
        material=steel,
        name="upper_hinge_plate",
    )
    gate_frame.visual(
        Box((0.028, 0.008, 0.11)),
        origin=Origin(xyz=(-0.014, 0.0, gate_clearance + 0.26)),
        material=steel,
        name="lower_hinge_plate",
    )
    gate_frame.visual(
        Box((0.020, 0.010, 0.12)),
        origin=Origin(xyz=(opening - 0.005, 0.045, gate_clearance + 0.73)),
        material=iron,
        name="keeper_plate",
    )
    gate_frame.visual(
        Box((0.012, 0.020, 0.035)),
        origin=Origin(xyz=(opening - 0.006, 0.035, gate_clearance + 0.73)),
        material=iron,
        name="keeper_lip",
    )
    gate_frame.visual(
        Box((0.055, 0.055, 0.008)),
        origin=Origin(xyz=(receiver_x, receiver_y, -0.004)),
        material=steel,
        name="receiver_pad",
    )
    gate_frame.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(receiver_x, receiver_y - 0.009, 0.010)),
        material=steel,
        name="receiver_back",
    )
    gate_frame.visual(
        Box((0.022, 0.004, 0.020)),
        origin=Origin(xyz=(receiver_x, receiver_y + 0.009, 0.010)),
        material=steel,
        name="receiver_front",
    )
    gate_frame.visual(
        Box((0.004, 0.018, 0.020)),
        origin=Origin(xyz=(receiver_x - 0.009, receiver_y, 0.010)),
        material=steel,
        name="receiver_left",
    )
    gate_frame.visual(
        Box((0.004, 0.018, 0.020)),
        origin=Origin(xyz=(receiver_x + 0.009, receiver_y, 0.010)),
        material=steel,
        name="receiver_right",
    )
    gate_frame.inertial = Inertial.from_geometry(
        Box((opening + 2.0 * post_w, receiver_y + 0.08, post_h + 0.03)),
        mass=40.0,
        origin=Origin(xyz=(opening / 2.0, receiver_y / 2.0, post_h / 2.0)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((stile_w, gate_t, gate_h)),
        origin=Origin(xyz=(stile_w / 2.0, 0.0, gate_clearance + gate_h / 2.0)),
        material=wood,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((stile_w, gate_t, gate_h)),
        origin=Origin(xyz=(gate_w - stile_w / 2.0, 0.0, gate_clearance + gate_h / 2.0)),
        material=wood,
        name="latch_stile",
    )

    rail_width = gate_w - 2.0 * stile_w
    rail_center_x = gate_w / 2.0
    gate_leaf.visual(
        Box((rail_width, gate_t, rail_h)),
        origin=Origin(xyz=(rail_center_x, 0.0, lower_rail_z)),
        material=wood,
        name="lower_rail",
    )
    gate_leaf.visual(
        Box((rail_width, gate_t, rail_h)),
        origin=Origin(xyz=(rail_center_x, 0.0, mid_rail_z)),
        material=wood,
        name="mid_rail",
    )
    gate_leaf.visual(
        Box((rail_width, gate_t, rail_h)),
        origin=Origin(xyz=(rail_center_x, 0.0, upper_rail_z)),
        material=wood,
        name="upper_rail",
    )

    brace_dx = 0.79
    brace_dz = upper_rail_z - lower_rail_z
    gate_leaf.visual(
        Box((math.hypot(brace_dx, brace_dz), 0.028, 0.028)),
        origin=Origin(
            xyz=(0.48, -0.006, (lower_rail_z + upper_rail_z) / 2.0),
            rpy=(0.0, math.atan2(brace_dz, brace_dx), 0.0),
        ),
        material=weathered_wood,
        name="diagonal_brace",
    )

    picket_base_z = gate_clearance + 0.10
    picket_centers = [0.105 + index * 0.110 for index in range(8)]
    for index, center_x in enumerate(picket_centers):
        gate_leaf.visual(
            picket_mesh,
            origin=Origin(xyz=(center_x, 0.011, picket_base_z)),
            material=wood,
            name=f"picket_{index}",
        )

    gate_leaf.visual(
        Box((0.24, 0.008, 0.045)),
        origin=Origin(xyz=(0.125, 0.016, gate_clearance + 0.78)),
        material=steel,
        name="upper_hinge_strap",
    )
    gate_leaf.visual(
        Box((0.24, 0.008, 0.045)),
        origin=Origin(xyz=(0.125, 0.016, gate_clearance + 0.26)),
        material=steel,
        name="lower_hinge_strap",
    )
    gate_leaf.visual(
        Box((0.16, 0.008, 0.20)),
        origin=Origin(xyz=(ring_x, 0.024, gate_clearance + 0.73)),
        material=iron,
        name="latch_plate",
    )
    gate_leaf.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(gate_w - 0.026, 0.024, gate_clearance + 0.73)),
        material=iron,
        name="latch_bar",
    )

    guide_specs = (
        ("upper", upper_guide_z),
        ("lower", lower_guide_z),
    )
    for prefix, guide_z in guide_specs:
        gate_leaf.visual(
            Box((0.006, 0.018, 0.018)),
            origin=Origin(xyz=(bolt_x - 0.011, 0.028, guide_z)),
            material=steel,
            name=f"{prefix}_guide_left",
        )
        gate_leaf.visual(
            Box((0.006, 0.018, 0.018)),
            origin=Origin(xyz=(bolt_x + 0.011, 0.028, guide_z)),
            material=steel,
            name=f"{prefix}_guide_right",
        )
        gate_leaf.visual(
            Box((0.016, 0.003, 0.018)),
            origin=Origin(xyz=(bolt_x, 0.0205, guide_z)),
            material=steel,
            name=f"{prefix}_guide_back",
        )
        gate_leaf.visual(
            Box((0.016, 0.003, 0.018)),
            origin=Origin(xyz=(bolt_x, 0.0355, guide_z)),
            material=steel,
            name=f"{prefix}_guide_front",
        )

    gate_leaf.inertial = Inertial.from_geometry(
        Box((gate_w, gate_t, gate_h)),
        mass=18.0,
        origin=Origin(xyz=(gate_w / 2.0, 0.0, gate_clearance + gate_h / 2.0)),
    )

    ring_latch = model.part("ring_latch")
    ring_latch.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, ring_y, -0.048)),
        material=iron,
        name="pull_ring",
    )
    ring_latch.visual(
        Cylinder(radius=0.0045, length=0.09),
        origin=Origin(xyz=(0.0, ring_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="ring_spindle",
    )
    ring_latch.inertial = Inertial.from_geometry(
        Box((0.10, 0.020, 0.11)),
        mass=0.35,
        origin=Origin(xyz=(0.0, ring_y, -0.045)),
    )

    drop_bolt = model.part("drop_bolt")
    drop_bolt.visual(
        Cylinder(radius=0.006, length=0.44),
        origin=Origin(xyz=(0.0, bolt_visual_y, -0.08)),
        material=steel,
        name="bolt_rod",
    )
    drop_bolt.visual(
        Cylinder(radius=0.0065, length=0.05),
        origin=Origin(xyz=(0.0, bolt_visual_y, -0.31)),
        material=steel,
        name="bolt_tip",
    )
    drop_bolt.visual(
        Cylinder(radius=0.007, length=0.10),
        origin=Origin(xyz=(0.0, bolt_visual_y, 0.14), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="bolt_handle",
    )
    drop_bolt.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, bolt_visual_y, 0.11)),
        material=iron,
        name="bolt_knob",
    )
    drop_bolt.inertial = Inertial.from_geometry(
        Box((0.10, 0.020, 0.52)),
        mass=0.55,
        origin=Origin(xyz=(0.0, bolt_visual_y, -0.06)),
    )

    model.articulation(
        "gate_swing",
        ArticulationType.REVOLUTE,
        parent=gate_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )
    model.articulation(
        "ring_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=ring_latch,
        origin=Origin(xyz=(ring_x, 0.0, ring_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=-0.20,
            upper=0.90,
        ),
    )
    model.articulation(
        "drop_bolt_slide",
        ArticulationType.PRISMATIC,
        parent=gate_leaf,
        child=drop_bolt,
        origin=Origin(xyz=(bolt_x, 0.0, bolt_origin_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.20,
            lower=0.0,
            upper=0.07,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    gate_frame = object_model.get_part("gate_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    ring_latch = object_model.get_part("ring_latch")
    drop_bolt = object_model.get_part("drop_bolt")

    gate_swing = object_model.get_articulation("gate_swing")
    ring_latch_pivot = object_model.get_articulation("ring_latch_pivot")
    drop_bolt_slide = object_model.get_articulation("drop_bolt_slide")

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

    ctx.check(
        "gate_swing_axis_vertical",
        tuple(gate_swing.axis) == (0.0, 0.0, 1.0),
        details=f"axis={gate_swing.axis}",
    )
    ctx.check(
        "ring_latch_axis_horizontal",
        tuple(ring_latch_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"axis={ring_latch_pivot.axis}",
    )
    ctx.check(
        "drop_bolt_axis_vertical",
        tuple(drop_bolt_slide.axis) == (0.0, 0.0, -1.0),
        details=f"axis={drop_bolt_slide.axis}",
    )

    ctx.expect_contact(gate_leaf, gate_frame, elem_a="latch_stile", elem_b="closing_stop")
    ctx.expect_contact(ring_latch, gate_leaf, elem_a="pull_ring", elem_b="latch_plate", contact_tol=0.0005)
    ctx.expect_contact(drop_bolt, gate_leaf, elem_a="bolt_rod", elem_b="upper_guide_back", contact_tol=0.0005)
    ctx.expect_contact(drop_bolt, gate_leaf, elem_a="bolt_rod", elem_b="lower_guide_back", contact_tol=0.0005)

    closed_latch_aabb = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")
    assert closed_latch_aabb is not None
    with ctx.pose({gate_swing: math.pi / 2.0}):
        open_latch_aabb = ctx.part_element_world_aabb(gate_leaf, elem="latch_stile")
        assert open_latch_aabb is not None
        ctx.check(
            "gate_leaf_swings_clear_of_opening",
            open_latch_aabb[1][1] > 0.90 and open_latch_aabb[1][0] < 0.08,
            details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
        )

    ring_rest_aabb = ctx.part_world_aabb(ring_latch)
    assert ring_rest_aabb is not None
    with ctx.pose({ring_latch_pivot: 0.70}):
        ring_lifted_aabb = ctx.part_world_aabb(ring_latch)
        assert ring_lifted_aabb is not None
        ctx.check(
            "ring_latch_lifts_off_plate",
            ring_lifted_aabb[1][1] > ring_rest_aabb[1][1] + 0.02,
            details=f"rest={ring_rest_aabb}, lifted={ring_lifted_aabb}",
        )

    bolt_rest_position = ctx.part_world_position(drop_bolt)
    assert bolt_rest_position is not None
    with ctx.pose({drop_bolt_slide: 0.07}):
        bolt_lowered_position = ctx.part_world_position(drop_bolt)
        assert bolt_lowered_position is not None
        ctx.check(
            "drop_bolt_slides_downward",
            bolt_lowered_position[2] < bolt_rest_position[2] - 0.06,
            details=f"rest={bolt_rest_position}, lowered={bolt_lowered_position}",
        )

    with ctx.pose({gate_swing: math.pi / 2.0, drop_bolt_slide: 0.07}):
        ctx.expect_contact(drop_bolt, gate_frame, elem_a="bolt_tip", elem_b="receiver_pad", contact_tol=0.001)
        ctx.expect_overlap(drop_bolt, gate_frame, elem_a="bolt_tip", elem_b="receiver_pad", axes="xy", min_overlap=0.01)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
