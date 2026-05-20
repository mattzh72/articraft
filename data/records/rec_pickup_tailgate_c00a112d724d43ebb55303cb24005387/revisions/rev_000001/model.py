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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate")

    body_blue = Material("painted_blue", rgba=(0.05, 0.18, 0.42, 1.0))
    dark_blue = Material("shadowed_blue", rgba=(0.025, 0.07, 0.16, 1.0))
    black = Material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    steel = Material("dark_zinc_steel", rgba=(0.32, 0.34, 0.35, 1.0))
    rubber = Material("rubber_black", rgba=(0.015, 0.014, 0.012, 1.0))

    model.materials.extend([body_blue, dark_blue, black, steel, rubber])

    # Static rear bed opening: a short floor/sill and the two rear jambs make the
    # truck-side structure that the gate closes against.
    bed = model.part("bed_frame")
    bed.visual(
        Box((1.92, 0.24, 0.12)),
        origin=Origin(xyz=(0.0, 0.08, 0.64)),
        material=body_blue,
        name="lower_sill",
    )
    bed.visual(
        Box((1.78, 0.65, 0.045)),
        origin=Origin(xyz=(0.0, 0.41, 0.7225)),
        material=dark_blue,
        name="bed_floor",
    )
    for side, x in enumerate((-0.93, 0.93)):
        bed.visual(
            Box((0.12, 0.34, 0.755)),
            origin=Origin(xyz=(x, 0.08, 1.0725)),
            material=body_blue,
            name=f"rear_jamb_{side}",
        )
        bed.visual(
            Box((0.12, 0.72, 0.11)),
            origin=Origin(xyz=(x, 0.43, 1.335)),
            material=body_blue,
            name=f"bed_rail_{side}",
        )
        bed.visual(
            Box((0.08, 0.70, 0.34)),
            origin=Origin(xyz=(x, 0.43, 0.91)),
            material=dark_blue,
            name=f"side_wall_{side}",
        )
        # The lower corner hinge brackets are fixed to the bed jambs.  They stop
        # just outboard of the moving hinge barrels, leaving a visible side gap.
        bed.visual(
            Box((0.06, 0.10, 0.14)),
            origin=Origin(xyz=(x * 0.9452, -0.07, 0.70)),
            material=steel,
            name=f"hinge_bracket_{side}",
        )
        bed.visual(
            Cylinder(radius=0.020, length=0.007),
            origin=Origin(xyz=(x * 0.9452, -0.121, 0.70), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name=f"hinge_hole_{side}",
        )
        bed.visual(
            Cylinder(radius=0.018, length=0.09),
            origin=Origin(xyz=(x * 0.90, -0.020, 1.225), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"striker_pin_{side}",
        )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((1.68, 0.075, 0.60)),
        origin=Origin(xyz=(0.0, -0.040, 0.300)),
        material=body_blue,
        name="outer_panel",
    )
    tailgate.visual(
        Box((1.70, 0.09, 0.060)),
        origin=Origin(xyz=(0.0, -0.038, 0.600)),
        material=body_blue,
        name="top_cap",
    )
    tailgate.visual(
        Box((1.62, 0.014, 0.035)),
        origin=Origin(xyz=(0.0, -0.0845, 0.540)),
        material=body_blue,
        name="upper_stamping",
    )
    tailgate.visual(
        Box((1.48, 0.014, 0.035)),
        origin=Origin(xyz=(0.0, -0.0845, 0.075)),
        material=body_blue,
        name="lower_stamping",
    )
    for side, x in enumerate((-0.775, 0.775)):
        tailgate.visual(
            Box((0.055, 0.014, 0.46)),
            origin=Origin(xyz=(x, -0.0845, 0.305)),
            material=body_blue,
            name=f"side_stamping_{side}",
        )
        tailgate.visual(
            Box((0.055, 0.030, 0.110)),
            origin=Origin(xyz=(x * 1.045, -0.014, 0.515)),
            material=steel,
            name=f"latch_pocket_{side}",
        )
    tailgate.visual(
        Box((0.80, 0.010, 0.235)),
        origin=Origin(xyz=(0.0, -0.090, 0.305)),
        material=dark_blue,
        name="center_recess",
    )
    tailgate.visual(
        Box((0.365, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, -0.083, 0.490)),
        material=black,
        name="handle_pocket",
    )
    tailgate.visual(
        Box((0.420, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, -0.090, 0.585)),
        material=body_blue,
        name="handle_bezel_top",
    )
    tailgate.visual(
        Box((0.420, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, -0.090, 0.395)),
        material=body_blue,
        name="handle_bezel_bottom",
    )
    for side, x in enumerate((-0.212, 0.212)):
        tailgate.visual(
            Box((0.030, 0.014, 0.190)),
            origin=Origin(xyz=(x, -0.090, 0.490)),
            material=body_blue,
            name=f"handle_bezel_side_{side}",
        )
    # Moving half of the two lower corner hinges, centered on the same horizontal
    # axis as the tailgate joint.
    for side, x in enumerate((-0.805, 0.805)):
        tailgate.visual(
            Cylinder(radius=0.026, length=0.090),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"hinge_barrel_{side}",
        )
        tailgate.visual(
            Box((0.075, 0.030, 0.115)),
            origin=Origin(xyz=(x, -0.030, 0.055)),
            material=steel,
            name=f"hinge_strap_{side}",
        )
    tailgate.visual(
        Box((1.58, 0.012, 0.035)),
        origin=Origin(xyz=(0.0, -0.002, 0.055)),
        material=rubber,
        name="lower_seal",
    )

    handle = model.part("latch_handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.300),
        origin=Origin(xyz=(0.0, -0.011, -0.004), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="handle_pivot",
    )
    handle.visual(
        Box((0.260, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, -0.009, -0.055)),
        material=black,
        name="handle_paddle",
    )
    handle.visual(
        Box((0.220, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, -0.015, -0.095)),
        material=black,
        name="handle_grip_lip",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, -0.070, 0.700)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "tailgate_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=handle,
        origin=Origin(xyz=(0.0, -0.089, 0.535)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    handle = object_model.get_part("latch_handle")
    gate_hinge = object_model.get_articulation("bed_to_tailgate")
    handle_pivot = object_model.get_articulation("tailgate_to_latch_handle")

    with ctx.pose({gate_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_overlap(
            tailgate,
            bed,
            axes="x",
            min_overlap=1.60,
            elem_a="outer_panel",
            elem_b="lower_sill",
            name="tailgate spans bed opening",
        )
        ctx.expect_gap(
            tailgate,
            handle,
            axis="y",
            positive_elem="handle_pocket",
            negative_elem="handle_paddle",
            max_penetration=0.0005,
            max_gap=0.0015,
            name="latch handle sits flush in pocket",
        )

    closed_gate = ctx.part_world_aabb(tailgate)
    with ctx.pose({gate_hinge: 1.45, handle_pivot: 0.0}):
        dropped_gate = ctx.part_world_aabb(tailgate)
    ctx.check(
        "tailgate drops outward and downward",
        closed_gate is not None
        and dropped_gate is not None
        and dropped_gate[0][1] < closed_gate[0][1] - 0.30
        and dropped_gate[1][2] < closed_gate[1][2] - 0.30,
        details=f"closed={closed_gate}, dropped={dropped_gate}",
    )

    closed_lip = ctx.part_element_world_aabb(handle, elem="handle_grip_lip")
    with ctx.pose({handle_pivot: 0.45}):
        pulled_lip = ctx.part_element_world_aabb(handle, elem="handle_grip_lip")
    ctx.check(
        "latch handle pulls outward",
        closed_lip is not None
        and pulled_lip is not None
        and pulled_lip[0][1] < closed_lip[0][1] - 0.025,
        details=f"closed={closed_lip}, pulled={pulled_lip}",
    )

    return ctx.report()


object_model = build_object_model()
