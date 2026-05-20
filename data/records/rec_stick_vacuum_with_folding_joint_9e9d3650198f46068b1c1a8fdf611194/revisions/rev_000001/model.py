from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_folding_stick_vacuum")

    molded_olive = model.material("molded_olive", rgba=(0.24, 0.28, 0.20, 1.0))
    graphite = model.material("graphite_molded", rgba=(0.055, 0.060, 0.060, 1.0))
    rubber = model.material("black_rubber", rgba=(0.010, 0.011, 0.010, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.53, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.34, 0.06, 1.0))
    dark_clear = model.material("smoked_clear_window", rgba=(0.12, 0.16, 0.18, 0.55))
    hose_gray = model.material("flex_hose_gray", rgba=(0.12, 0.13, 0.12, 1.0))

    # Root frame is the exposed folding-joint centerline.  The upper assembly
    # extends upward, while the lower wand child extends downward from here.
    upper = model.part("upper_assembly")

    upper.visual(
        Cylinder(radius=0.019, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=graphite,
        name="upper_wand",
    )
    upper.visual(
        Box((0.030, 0.030, 0.590)),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=graphite,
        name="reinforced_spine",
    )
    upper.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=molded_olive,
        name="upper_hinge_collar",
    )
    upper.visual(
        Cylinder(radius=0.027, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=molded_olive,
        name="motor_collar",
    )

    motor_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.060, length=0.230, radial_segments=32, height_segments=8),
        "slim_motor_capsule",
    )
    upper.visual(
        motor_mesh,
        origin=Origin(xyz=(0.055, 0.0, 0.520), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=molded_olive,
        name="motor_body",
    )
    upper.visual(
        Box((0.115, 0.105, 0.090)),
        origin=Origin(xyz=(-0.045, 0.0, 0.485)),
        material=graphite,
        name="battery_pack",
    )
    upper.visual(
        Box((0.105, 0.095, 0.014)),
        origin=Origin(xyz=(-0.045, 0.0, 0.538)),
        material=rubber,
        name="battery_sole",
    )

    vent_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.125, 0.060),
            0.004,
            slot_size=(0.025, 0.0045),
            pitch=(0.034, 0.014),
            frame=0.008,
            corner_radius=0.004,
            stagger=True,
        ),
        "side_filter_slots",
    )
    for side, y in (("0", 0.052), ("1", -0.052)):
        upper.visual(
            vent_mesh,
            origin=Origin(
                xyz=(0.055, y, 0.520),
                rpy=(-math.pi / 2.0 if y > 0 else math.pi / 2.0, 0.0, 0.0),
            ),
            material=graphite,
            name=f"filter_grille_{side}",
        )

    handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.035, 0.0, 0.560),
                (-0.130, 0.0, 0.560),
                (-0.150, 0.0, 0.740),
                (-0.070, 0.0, 0.830),
                (0.030, 0.0, 0.730),
            ],
            radius=0.014,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=True,
        ),
        "rear_handle_loop",
    )
    upper.visual(handle_mesh, material=graphite, name="handle_loop")
    upper.visual(
        Box((0.040, 0.052, 0.038)),
        origin=Origin(xyz=(-0.032, 0.0, 0.558)),
        material=graphite,
        name="handle_lower_anchor",
    )
    upper.visual(
        Box((0.045, 0.050, 0.038)),
        origin=Origin(xyz=(0.030, 0.0, 0.716)),
        material=graphite,
        name="handle_upper_anchor",
    )
    upper.visual(
        Cylinder(radius=0.021, length=0.115),
        origin=Origin(xyz=(-0.141, 0.0, 0.680), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rubber_grip",
    )

    # Folding joint: two heavy side plates on the upper assembly, with the child
    # central lug and axle nested through them.
    upper.visual(
        Box((0.082, 0.017, 0.108)),
        origin=Origin(xyz=(0.0, 0.046, -0.006)),
        material=molded_olive,
        name="fold_side_plate_0",
    )
    upper.visual(
        Box((0.082, 0.017, 0.108)),
        origin=Origin(xyz=(0.0, -0.046, -0.006)),
        material=molded_olive,
        name="fold_side_plate_1",
    )
    upper.visual(
        Cylinder(radius=0.0075, length=0.007),
        origin=Origin(xyz=(-0.022, 0.05382, 0.027), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_bolt_upper_0",
    )
    upper.visual(
        Cylinder(radius=0.0075, length=0.007),
        origin=Origin(xyz=(0.024, 0.05382, -0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_bolt_lower_0",
    )
    upper.visual(
        Cylinder(radius=0.0075, length=0.007),
        origin=Origin(xyz=(-0.022, -0.05382, 0.027), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_bolt_upper_1",
    )
    upper.visual(
        Cylinder(radius=0.0075, length=0.007),
        origin=Origin(xyz=(0.024, -0.05382, -0.034), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_bolt_lower_1",
    )
    upper.visual(
        Box((0.096, 0.104, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=molded_olive,
        name="fold_top_bridge",
    )
    upper.visual(
        Box((0.050, 0.012, 0.040)),
        origin=Origin(xyz=(0.060, 0.0, 0.025)),
        material=safety_orange,
        name="fold_lock_tab",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.0065, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trigger_pin",
    )
    trigger.visual(
        Box((0.016, 0.035, 0.070)),
        origin=Origin(xyz=(0.018, 0.0, -0.036), rpy=(0.0, -0.18, 0.0)),
        material=safety_orange,
        name="trigger_paddle",
    )
    upper.visual(
        Box((0.040, 0.014, 0.040)),
        origin=Origin(xyz=(-0.105, 0.031, 0.660)),
        material=graphite,
        name="trigger_housing",
    )
    upper.visual(
        Box((0.040, 0.014, 0.040)),
        origin=Origin(xyz=(-0.105, -0.031, 0.660)),
        material=graphite,
        name="trigger_housing_side",
    )
    upper.visual(
        Box((0.040, 0.076, 0.014)),
        origin=Origin(xyz=(-0.105, 0.0, 0.684)),
        material=graphite,
        name="trigger_housing_bridge",
    )

    lower = model.part("lower_wand")
    lower.visual(
        Cylinder(radius=0.017, length=0.700),
        origin=Origin(xyz=(0.0, 0.0, -0.395)),
        material=graphite,
        name="main_tube",
    )
    lower.visual(
        Box((0.012, 0.006, 0.610)),
        origin=Origin(xyz=(0.020, 0.0, -0.395)),
        material=molded_olive,
        name="front_rib",
    )
    lower.visual(
        Box((0.010, 0.006, 0.560)),
        origin=Origin(xyz=(-0.020, 0.0, -0.420)),
        material=molded_olive,
        name="rear_rib",
    )
    lower.visual(
        Box((0.058, 0.054, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=molded_olive,
        name="fold_center_lug",
    )
    lower.visual(
        Cylinder(radius=0.012, length=0.126),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="fold_axle",
    )
    lower.visual(
        Cylinder(radius=0.028, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=molded_olive,
        name="lower_hinge_collar",
    )
    lower.visual(
        Cylinder(radius=0.026, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.715)),
        material=molded_olive,
        name="neck_collar",
    )
    lower.visual(
        Cylinder(radius=0.012, length=0.215),
        origin=Origin(xyz=(-0.029, 0.0, -0.410), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hose_gray,
        name="service_conduit",
    )

    # Fork hardware at the floor-head end.  The head carries the steel pivot
    # axle; these cheek plates show the receiving yoke and service bolts.
    lower.visual(
        Box((0.074, 0.018, 0.090)),
        origin=Origin(xyz=(0.030, 0.065, -0.785)),
        material=molded_olive,
        name="head_fork_cheek_0",
    )
    lower.visual(
        Box((0.074, 0.018, 0.090)),
        origin=Origin(xyz=(0.030, -0.065, -0.785)),
        material=molded_olive,
        name="head_fork_cheek_1",
    )
    lower.visual(
        Cylinder(radius=0.0065, length=0.007),
        origin=Origin(xyz=(0.003, 0.07605, -0.760), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="head_bolt_upper_0",
    )
    lower.visual(
        Cylinder(radius=0.0065, length=0.007),
        origin=Origin(xyz=(0.056, 0.07605, -0.808), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="head_bolt_lower_0",
    )
    lower.visual(
        Cylinder(radius=0.0065, length=0.007),
        origin=Origin(xyz=(0.003, -0.07605, -0.760), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="head_bolt_upper_1",
    )
    lower.visual(
        Cylinder(radius=0.0065, length=0.007),
        origin=Origin(xyz=(0.056, -0.07605, -0.808), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="head_bolt_lower_1",
    )
    lower.visual(
        Box((0.055, 0.120, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, -0.740)),
        material=molded_olive,
        name="head_fork_bridge",
    )

    head = model.part("floor_head")
    head.visual(
        Cylinder(radius=0.014, length=0.152),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="head_axle",
    )
    head.visual(
        Box((0.062, 0.090, 0.052)),
        origin=Origin(xyz=(0.025, 0.0, -0.028)),
        material=molded_olive,
        name="pivot_knuckle",
    )
    head.visual(
        Box((0.315, 0.310, 0.070)),
        origin=Origin(xyz=(0.130, 0.0, -0.105)),
        material=molded_olive,
        name="head_shell",
    )
    head.visual(
        Box((0.055, 0.340, 0.040)),
        origin=Origin(xyz=(0.265, 0.0, -0.105)),
        material=rubber,
        name="front_bumper",
    )
    head.visual(
        Box((0.260, 0.248, 0.018)),
        origin=Origin(xyz=(0.135, 0.0, -0.062)),
        material=dark_clear,
        name="brush_window",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.235),
        origin=Origin(xyz=(0.155, 0.0, -0.121), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_orange,
        name="brush_roll",
    )
    for idx, y in enumerate((0.162, -0.162)):
        head.visual(
            Cylinder(radius=0.028, length=0.018),
            origin=Origin(xyz=(-0.010, y, -0.128), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"rear_wheel_{idx}",
        )
    for idx, y in enumerate((0.120, -0.120)):
        head.visual(
            Box((0.070, 0.035, 0.024)),
            origin=Origin(xyz=(0.040, y, -0.062)),
            material=graphite,
            name=f"link_anchor_{idx}",
        )

    fold_joint = model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=lower,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.3, lower=0.0, upper=1.55),
    )
    # The floor-head pivot is placed at the bottom yoke centerline, in the lower
    # wand frame.  The head's own frame is the steel axle center.
    model.articulation(
        "head_pivot",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=head,
        origin=Origin(xyz=(0.030, 0.0, -0.785)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0, lower=-0.55, upper=0.65),
    )
    model.articulation(
        "trigger_pivot",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=trigger,
        origin=Origin(xyz=(-0.105, 0.0, 0.660)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=0.32),
    )

    # Pose samples include a partly folded wand and both ends of head tilt so the
    # QC harness sees the actual usable mechanism envelope.
    fold_joint.meta["qc_samples"] = [0.0, 0.85, 1.30]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper = object_model.get_part("upper_assembly")
    lower = object_model.get_part("lower_wand")
    head = object_model.get_part("floor_head")
    trigger = object_model.get_part("trigger")
    fold_joint = object_model.get_articulation("fold_joint")
    head_pivot = object_model.get_articulation("head_pivot")
    trigger_pivot = object_model.get_articulation("trigger_pivot")

    for plate in ("fold_side_plate_0", "fold_side_plate_1"):
        ctx.allow_overlap(
            upper,
            lower,
            elem_a=plate,
            elem_b="fold_axle",
            reason="The steel folding-joint axle is intentionally captured through the solid proxy side plate.",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="xz",
            elem_a=plate,
            elem_b="fold_axle",
            min_overlap=0.010,
            name=f"{plate} captures fold axle in projection",
        )
    for cheek in ("head_fork_cheek_0", "head_fork_cheek_1"):
        ctx.allow_overlap(
            lower,
            head,
            elem_a=cheek,
            elem_b="head_axle",
            reason="The floor-head pivot axle is represented as passing through the yoke cheek bore.",
        )
        ctx.expect_overlap(
            lower,
            head,
            axes="xz",
            elem_a=cheek,
            elem_b="head_axle",
            min_overlap=0.010,
            name=f"{cheek} captures head axle in projection",
        )
    for boss in ("trigger_housing", "trigger_housing_side"):
        ctx.allow_overlap(
            upper,
            trigger,
            elem_a=boss,
            elem_b="trigger_pin",
            reason="The trigger pin is intentionally captured in the separated side boss of the trigger housing.",
        )
        ctx.expect_overlap(
            upper,
            trigger,
            axes="z",
            elem_a=boss,
            elem_b="trigger_pin",
            min_overlap=0.010,
            name=f"{boss} retains trigger pin vertically",
        )
        ctx.expect_overlap(
            upper,
            trigger,
            axes="y",
            elem_a=boss,
            elem_b="trigger_pin",
            min_overlap=0.004,
            name=f"{boss} overlaps trigger pin end",
        )

    ctx.expect_overlap(
        upper,
        lower,
        axes="y",
        elem_a="fold_side_plate_0",
        elem_b="fold_axle",
        min_overlap=0.004,
        name="fold axle spans through upper hinge plate",
    )
    ctx.expect_overlap(
        lower,
        head,
        axes="y",
        elem_a="head_fork_cheek_0",
        elem_b="head_axle",
        min_overlap=0.004,
        name="floor head axle spans through yoke plate",
    )

    rest_lower_aabb = ctx.part_element_world_aabb(lower, elem="main_tube")
    with ctx.pose({fold_joint: 1.20}):
        folded_lower_aabb = ctx.part_element_world_aabb(lower, elem="main_tube")
    ctx.check(
        "fold joint swings lower wand rearward",
        rest_lower_aabb is not None
        and folded_lower_aabb is not None
        and folded_lower_aabb[0][0] < rest_lower_aabb[0][0] - 0.18,
        details=f"rest={rest_lower_aabb}, folded={folded_lower_aabb}",
    )

    with ctx.pose({head_pivot: 0.55}):
        nose_down = ctx.part_element_world_aabb(head, elem="front_bumper")
    with ctx.pose({head_pivot: -0.45}):
        nose_up = ctx.part_element_world_aabb(head, elem="front_bumper")
    ctx.check(
        "floor head pivot visibly tilts the nose",
        nose_down is not None
        and nose_up is not None
        and nose_down[0][2] < nose_up[0][2] - 0.07,
        details=f"nose_down={nose_down}, nose_up={nose_up}",
    )

    rest_trigger = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_pivot: 0.25}):
        squeezed_trigger = ctx.part_world_aabb(trigger)
    ctx.check(
        "trigger rotates under handle",
        rest_trigger is not None
        and squeezed_trigger is not None
        and squeezed_trigger[0][2] < rest_trigger[0][2] - 0.002,
        details=f"rest={rest_trigger}, squeezed={squeezed_trigger}",
    )

    return ctx.report()


object_model = build_object_model()
