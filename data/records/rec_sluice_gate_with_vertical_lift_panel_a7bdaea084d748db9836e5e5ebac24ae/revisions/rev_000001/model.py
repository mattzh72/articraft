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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_sluice_gate")

    weathered_concrete = model.material("weathered_concrete", rgba=(0.57, 0.56, 0.52, 1.0))
    damp_stain = model.material("damp_stain", rgba=(0.28, 0.33, 0.30, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.06, 0.18, 0.25, 1.0))
    worn_gate_steel = model.material("worn_gate_steel", rgba=(0.16, 0.25, 0.30, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.06, 0.07, 0.08, 1.0))
    gearbox_gray = model.material("gearbox_gray", rgba=(0.37, 0.40, 0.38, 1.0))
    grease_black = model.material("grease_black", rgba=(0.015, 0.015, 0.014, 1.0))
    safety_red = model.material("safety_red", rgba=(0.70, 0.08, 0.04, 1.0))

    handwheel_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.28, tube=0.030, radial_segments=18, tubular_segments=80),
        "handwheel_ring",
    )

    masonry_frame = model.part("masonry_frame")
    # Canal-gate scale masonry portal: clear opening is about 2.7 m wide by 3.25 m tall.
    masonry_frame.visual(
        Box((0.65, 0.90, 3.70)),
        origin=Origin(xyz=(-1.675, 0.0, 1.80)),
        material=weathered_concrete,
        name="side_pier_0",
    )
    masonry_frame.visual(
        Box((0.65, 0.90, 3.70)),
        origin=Origin(xyz=(1.675, 0.0, 1.80)),
        material=weathered_concrete,
        name="side_pier_1",
    )
    masonry_frame.visual(
        Box((4.00, 1.00, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=weathered_concrete,
        name="sill_block",
    )
    masonry_frame.visual(
        Box((4.00, 1.00, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 3.45)),
        material=weathered_concrete,
        name="lintel_beam",
    )
    masonry_frame.visual(
        Box((2.70, 0.035, 0.055)),
        origin=Origin(xyz=(0.0, -0.455, 0.10)),
        material=damp_stain,
        name="waterline_stain",
    )

    # Steel U-channel guide jambs.  The flanges wrap the moving panel edges with
    # visible clearance so the lift panel reads as captured, not floating.
    masonry_frame.visual(
        Box((0.10, 0.30, 3.25)),
        origin=Origin(xyz=(-1.30, -0.035, 1.625)),
        material=painted_steel,
        name="left_guide_web",
    )
    masonry_frame.visual(
        Box((0.34, 0.040, 3.25)),
        origin=Origin(xyz=(-1.13, -0.150, 1.625)),
        material=painted_steel,
        name="left_front_lip",
    )
    masonry_frame.visual(
        Box((0.34, 0.040, 3.25)),
        origin=Origin(xyz=(-1.13, 0.070, 1.625)),
        material=painted_steel,
        name="left_rear_lip",
    )
    masonry_frame.visual(
        Box((0.42, 0.08, 0.10)),
        origin=Origin(xyz=(-1.13, -0.150, 3.30)),
        material=painted_steel,
        name="left_top_keeper",
    )
    masonry_frame.visual(
        Box((0.10, 0.30, 3.25)),
        origin=Origin(xyz=(1.30, -0.035, 1.625)),
        material=painted_steel,
        name="right_guide_web",
    )
    masonry_frame.visual(
        Box((0.34, 0.040, 3.25)),
        origin=Origin(xyz=(1.13, -0.150, 1.625)),
        material=painted_steel,
        name="right_front_lip",
    )
    masonry_frame.visual(
        Box((0.34, 0.040, 3.25)),
        origin=Origin(xyz=(1.13, 0.070, 1.625)),
        material=painted_steel,
        name="right_rear_lip",
    )
    masonry_frame.visual(
        Box((0.42, 0.08, 0.10)),
        origin=Origin(xyz=(1.13, -0.150, 3.30)),
        material=painted_steel,
        name="right_top_keeper",
    )
    # Prominent bolted anchor heads on the front guide lips.
    for index, z in enumerate((0.45, 1.25, 2.05, 2.85)):
        masonry_frame.visual(
            Cylinder(radius=0.035, length=0.020),
            origin=Origin(xyz=(-1.13, -0.180, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"left_guide_bolt_{index}",
        )
        masonry_frame.visual(
            Cylinder(radius=0.035, length=0.020),
            origin=Origin(xyz=(1.13, -0.180, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"right_guide_bolt_{index}",
        )

    lift_panel = model.part("lift_panel")
    lift_panel.visual(
        Box((2.12, 0.090, 2.35)),
        origin=Origin(xyz=(0.0, -0.030, 1.175)),
        material=worn_gate_steel,
        name="flat_plate",
    )
    lift_panel.visual(
        Box((0.20, 0.140, 2.45)),
        origin=Origin(xyz=(-1.13, -0.020, 1.225)),
        material=painted_steel,
        name="left_shoe",
    )
    lift_panel.visual(
        Box((0.20, 0.140, 2.45)),
        origin=Origin(xyz=(1.13, -0.020, 1.225)),
        material=painted_steel,
        name="right_shoe",
    )
    lift_panel.visual(
        Box((2.05, 0.045, 0.16)),
        origin=Origin(xyz=(0.0, -0.088, 2.16)),
        material=dark_metal,
        name="top_stiffener",
    )
    lift_panel.visual(
        Box((2.05, 0.045, 0.16)),
        origin=Origin(xyz=(0.0, -0.088, 0.36)),
        material=dark_metal,
        name="lower_stiffener",
    )
    lift_panel.visual(
        Box((0.24, 0.10, 0.28)),
        origin=Origin(xyz=(0.0, -0.100, 2.47)),
        material=dark_metal,
        name="lift_lug",
    )

    operator_housing = model.part("operator_housing")
    operator_housing.visual(
        Box((0.95, 0.65, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_metal,
        name="mounting_base",
    )
    operator_housing.visual(
        Box((0.66, 0.48, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=gearbox_gray,
        name="gearbox_case",
    )
    operator_housing.visual(
        Cylinder(radius=0.18, length=0.16),
        origin=Origin(xyz=(0.0, -0.320, 0.32), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="input_boss",
    )
    operator_housing.visual(
        Cylinder(radius=0.14, length=0.20),
        origin=Origin(xyz=(0.0, 0.255, 0.30), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="output_boss",
    )
    operator_housing.visual(
        Box((0.48, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.12, 0.59)),
        material=gearbox_gray,
        name="worm_cover",
    )
    operator_housing.visual(
        Box((0.18, 0.22, 0.32)),
        origin=Origin(xyz=(0.38, -0.335, 0.50)),
        material=dark_metal,
        name="pawl_mount",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        handwheel_ring_mesh,
        origin=Origin(xyz=(0.0, -0.100, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="outer_ring",
    )
    handwheel.visual(
        Cylinder(radius=0.070, length=0.14),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    for index, angle in enumerate((0.0, math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0)):
        handwheel.visual(
            Box((0.50, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(0.0, angle, 0.0)),
            material=dark_metal,
            name=f"spoke_{index}",
        )
    handwheel.visual(
        Cylinder(radius=0.035, length=0.11),
        origin=Origin(xyz=(0.0, -0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grease_black,
        name="shaft_stub",
    )

    locking_pawl = model.part("locking_pawl")
    locking_pawl.visual(
        Cylinder(radius=0.050, length=0.070),
        origin=Origin(xyz=(0.0, -0.035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_pin",
    )
    locking_pawl.visual(
        Box((0.30, 0.035, 0.060)),
        origin=Origin(xyz=(0.150, -0.055, -0.020)),
        material=safety_red,
        name="pawl_arm",
    )
    locking_pawl.visual(
        Box((0.070, 0.040, 0.090)),
        origin=Origin(xyz=(0.335, -0.055, -0.045)),
        material=dark_metal,
        name="pawl_tooth",
    )

    model.articulation(
        "panel_lift",
        ArticulationType.PRISMATIC,
        parent=masonry_frame,
        child=lift_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160000.0, velocity=0.18, lower=0.0, upper=0.75),
    )
    model.articulation(
        "frame_to_operator",
        ArticulationType.FIXED,
        parent=masonry_frame,
        child=operator_housing,
        origin=Origin(xyz=(0.0, -0.350, 3.70)),
    )
    model.articulation(
        "handwheel_spin",
        ArticulationType.CONTINUOUS,
        parent=operator_housing,
        child=handwheel,
        origin=Origin(xyz=(0.0, -0.400, 0.32)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=6.0),
    )
    model.articulation(
        "pawl_pivot",
        ArticulationType.REVOLUTE,
        parent=operator_housing,
        child=locking_pawl,
        origin=Origin(xyz=(0.38, -0.430, 0.50)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.45, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("masonry_frame")
    panel = object_model.get_part("lift_panel")
    housing = object_model.get_part("operator_housing")
    handwheel = object_model.get_part("handwheel")
    pawl = object_model.get_part("locking_pawl")
    panel_lift = object_model.get_articulation("panel_lift")
    handwheel_spin = object_model.get_articulation("handwheel_spin")
    pawl_pivot = object_model.get_articulation("pawl_pivot")

    ctx.check(
        "panel uses vertical prismatic travel",
        panel_lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(panel_lift.axis) == (0.0, 0.0, 1.0)
        and panel_lift.motion_limits is not None
        and panel_lift.motion_limits.upper is not None
        and panel_lift.motion_limits.upper >= 0.70,
        details=f"type={panel_lift.articulation_type}, axis={panel_lift.axis}, limits={panel_lift.motion_limits}",
    )
    ctx.check(
        "handwheel is continuous on input shaft",
        handwheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(handwheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={handwheel_spin.articulation_type}, axis={handwheel_spin.axis}",
    )
    ctx.check(
        "pawl has a limited pivot beside the wheel",
        pawl_pivot.articulation_type == ArticulationType.REVOLUTE
        and pawl_pivot.motion_limits is not None
        and pawl_pivot.motion_limits.lower is not None
        and pawl_pivot.motion_limits.upper is not None
        and pawl_pivot.motion_limits.lower < 0.0 < pawl_pivot.motion_limits.upper,
        details=f"type={pawl_pivot.articulation_type}, limits={pawl_pivot.motion_limits}",
    )
    ctx.allow_overlap(
        pawl,
        housing,
        elem_a="pivot_pin",
        elem_b="pawl_mount",
        reason="The locking pawl's short pivot pin is intentionally captured through the operator-housing mount.",
    )

    # The panel side shoes sit in the guide jamb U-channels with small real
    # clearances; this proves the moving slab is visibly captured by the jambs.
    ctx.expect_gap(
        panel,
        frame,
        axis="x",
        positive_elem="left_shoe",
        negative_elem="left_guide_web",
        max_gap=0.050,
        max_penetration=0.0,
        name="left shoe runs beside guide web",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="x",
        positive_elem="right_guide_web",
        negative_elem="right_shoe",
        max_gap=0.050,
        max_penetration=0.0,
        name="right shoe runs beside guide web",
    )
    ctx.expect_gap(
        panel,
        frame,
        axis="y",
        positive_elem="left_shoe",
        negative_elem="left_front_lip",
        max_gap=0.040,
        max_penetration=0.0,
        name="front guide lip retains panel shoe",
    )
    ctx.expect_gap(
        frame,
        panel,
        axis="y",
        positive_elem="left_rear_lip",
        negative_elem="left_shoe",
        max_gap=0.040,
        max_penetration=0.0,
        name="rear guide lip retains panel shoe",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="z",
        elem_a="left_shoe",
        elem_b="left_guide_web",
        min_overlap=2.20,
        name="closed panel remains engaged in guide height",
    )

    rest_pos = ctx.part_world_position(panel)
    with ctx.pose({panel_lift: 0.75}):
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="left_shoe",
            elem_b="left_guide_web",
            min_overlap=2.20,
            name="raised panel remains engaged in guide height",
        )
        raised_pos = ctx.part_world_position(panel)
    ctx.check(
        "panel lifts upward along jambs",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.70,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    ctx.expect_contact(
        housing,
        frame,
        elem_a="mounting_base",
        elem_b="lintel_beam",
        contact_tol=0.002,
        name="gearbox base is perched on lintel",
    )
    ctx.expect_contact(
        handwheel,
        housing,
        elem_a="hub",
        elem_b="input_boss",
        contact_tol=0.002,
        name="handwheel hub seats on input boss",
    )
    ctx.expect_contact(
        pawl,
        housing,
        elem_a="pivot_pin",
        elem_b="pawl_mount",
        contact_tol=0.002,
        name="pawl pivot is mounted to operator housing",
    )

    def _center_z(aabb):
        return 0.5 * (aabb[0][2] + aabb[1][2]) if aabb is not None else None

    tooth_rest = ctx.part_element_world_aabb(pawl, elem="pawl_tooth")
    with ctx.pose({pawl_pivot: 0.65}):
        tooth_rotated = ctx.part_element_world_aabb(pawl, elem="pawl_tooth")
    ctx.check(
        "pawl tooth moves when pivoted",
        tooth_rest is not None
        and tooth_rotated is not None
        and abs(_center_z(tooth_rotated) - _center_z(tooth_rest)) > 0.045,
        details=f"rest={tooth_rest}, rotated={tooth_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
