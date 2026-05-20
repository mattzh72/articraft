from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_pickup_tailgate")

    body_red = model.material("body_red", rgba=(0.62, 0.06, 0.04, 1.0))
    dark_bedliner = model.material("dark_bedliner", rgba=(0.035, 0.038, 0.040, 1.0))
    black_trim = model.material("black_trim", rgba=(0.015, 0.016, 0.018, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.28, 0.29, 0.30, 1.0))
    latch_black = model.material("latch_black", rgba=(0.02, 0.02, 0.022, 1.0))
    reflector_red = model.material("reflector_red", rgba=(0.85, 0.04, 0.03, 1.0))

    bed = model.part("bed")
    # A short pickup-box context makes the tailgate hinges read as mounted to
    # the side structure instead of as isolated flaps.
    bed.visual(
        Box((1.25, 1.72, 0.08)),
        origin=Origin(xyz=(0.625, 0.0, 0.18)),
        material=body_red,
        name="bed_floor",
    )
    bed.visual(
        Box((1.18, 1.52, 0.018)),
        origin=Origin(xyz=(0.66, 0.0, 0.226)),
        material=dark_bedliner,
        name="floor_liner",
    )
    for side_y in (-0.90, 0.90):
        bed.visual(
            Box((1.28, 0.08, 0.71)),
            origin=Origin(xyz=(0.64, side_y, 0.57)),
            material=body_red,
            name=f"side_wall_{side_y:+.0f}",
        )
        bed.visual(
            Box((1.30, 0.115, 0.055)),
            origin=Origin(xyz=(0.64, side_y, 0.94)),
            material=black_trim,
            name=f"rail_cap_{side_y:+.0f}",
        )
        bed.visual(
            Box((0.08, 0.13, 0.25)),
            origin=Origin(xyz=(0.02, side_y, 0.34)),
            material=hinge_steel,
            name=f"rear_hinge_post_{side_y:+.0f}",
        )
    bed.visual(
        Box((0.08, 1.88, 0.71)),
        origin=Origin(xyz=(1.28, 0.0, 0.57)),
        material=body_red,
        name="front_wall",
    )
    bed.visual(
        Box((0.09, 1.90, 0.055)),
        origin=Origin(xyz=(1.28, 0.0, 0.94)),
        material=black_trim,
        name="front_rail_cap",
    )
    # Alternating lower hinge knuckles on the truck side.  Leaf plates sit
    # under the gate bottom so the closed gate has real clearance.
    lower_parent_segments = [(-0.72, 0.20), (0.0, 0.26), (0.72, 0.20)]
    for i, (y_pos, length) in enumerate(lower_parent_segments):
        bed.visual(
            Box((0.13, length, 0.024)),
            origin=Origin(xyz=(-0.006, y_pos, 0.227)),
            material=hinge_steel,
            name=f"lower_leaf_{i}",
        )
        bed.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(-0.055, y_pos, 0.240), rpy=(1.5707963268, 0.0, 0.0)),
            material=hinge_steel,
            name=f"lower_bed_barrel_{i}",
        )
    bed.visual(
        Cylinder(radius=0.009, length=1.64),
        origin=Origin(xyz=(-0.055, 0.0, 0.240), rpy=(1.5707963268, 0.0, 0.0)),
        material=hinge_steel,
        name="lower_hinge_pin",
    )

    main_gate = model.part("main_gate")
    main_gate.visual(
        Box((0.060, 1.64, 0.500)),
        origin=Origin(xyz=(0.055, 0.0, 0.250)),
        material=body_red,
        name="main_skin",
    )
    main_gate.visual(
        Box((0.012, 1.48, 0.370)),
        origin=Origin(xyz=(0.019, 0.0, 0.285)),
        material=body_red,
        name="stamped_recess",
    )
    main_gate.visual(
        Box((0.014, 1.56, 0.050)),
        origin=Origin(xyz=(0.018, 0.0, 0.055)),
        material=body_red,
        name="lower_rail",
    )
    main_gate.visual(
        Box((0.014, 1.56, 0.045)),
        origin=Origin(xyz=(0.018, 0.0, 0.478)),
        material=black_trim,
        name="upper_seal",
    )
    for y_pos in (-0.68, 0.68):
        main_gate.visual(
            Box((0.016, 0.075, 0.165)),
            origin=Origin(xyz=(0.017, y_pos, 0.315)),
            material=reflector_red,
            name=f"reflector_{y_pos:+.0f}",
        )
    main_gate.visual(
        Box((0.016, 0.26, 0.055)),
        origin=Origin(xyz=(0.017, 0.0, 0.430)),
        material=latch_black,
        name="main_pull",
    )
    for y_pos in (-0.40, 0.0, 0.40):
        main_gate.visual(
            Box((0.014, 0.035, 0.400)),
            origin=Origin(xyz=(0.018, y_pos, 0.270)),
            material=body_red,
            name=f"vertical_rib_{y_pos:+.0f}",
        )
    lower_gate_segments = [(-0.38, 0.34), (0.38, 0.34)]
    for i, (y_pos, length) in enumerate(lower_gate_segments):
        main_gate.visual(
            Box((0.040, length, 0.050)),
            origin=Origin(xyz=(0.015, y_pos, 0.036)),
            material=hinge_steel,
            name=f"lower_gate_leaf_{i}",
        )
        main_gate.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(1.5707963268, 0.0, 0.0)),
            material=hinge_steel,
            name=f"lower_gate_barrel_{i}",
        )
    # Parent-side barrels for the second, top-edge hinge.
    upper_parent_segments = [(-0.51, 0.30), (0.51, 0.30)]
    for i, (y_pos, length) in enumerate(upper_parent_segments):
        main_gate.visual(
            Box((0.036, length, 0.032)),
            origin=Origin(xyz=(0.018, y_pos, 0.482)),
            material=hinge_steel,
            name=f"upper_main_leaf_{i}",
        )
        main_gate.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(0.0, y_pos, 0.500), rpy=(1.5707963268, 0.0, 0.0)),
            material=hinge_steel,
            name=f"upper_main_barrel_{i}",
        )

    upper_flap = model.part("upper_flap")
    upper_flap.visual(
        Box((0.055, 1.60, 0.180)),
        origin=Origin(xyz=(0.055, 0.0, 0.090)),
        material=body_red,
        name="flap_skin",
    )
    upper_flap.visual(
        Box((0.014, 1.48, 0.090)),
        origin=Origin(xyz=(0.023, 0.0, 0.105)),
        material=body_red,
        name="flap_recess",
    )
    upper_flap.visual(
        Box((0.014, 1.50, 0.030)),
        origin=Origin(xyz=(0.023, 0.0, 0.164)),
        material=black_trim,
        name="top_seal",
    )
    upper_flap.visual(
        Box((0.018, 0.34, 0.040)),
        origin=Origin(xyz=(0.024, 0.0, 0.105)),
        material=latch_black,
        name="flap_pull",
    )
    upper_flap.visual(
        Box((0.014, 1.50, 0.024)),
        origin=Origin(xyz=(0.020, 0.0, 0.016)),
        material=black_trim,
        name="hinge_seal",
    )
    upper_child_segments = [(-0.75, 0.18), (-0.16, 0.26), (0.16, 0.26), (0.75, 0.18)]
    for i, (y_pos, length) in enumerate(upper_child_segments):
        upper_flap.visual(
            Box((0.040, length, 0.036)),
            origin=Origin(xyz=(0.020, y_pos, 0.018)),
            material=hinge_steel,
            name=f"upper_flap_leaf_{i}",
        )
        upper_flap.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(1.5707963268, 0.0, 0.0)),
            material=hinge_steel,
            name=f"upper_flap_barrel_{i}",
        )

    model.articulation(
        "bed_to_main_gate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=main_gate,
        origin=Origin(xyz=(-0.055, 0.0, 0.240)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "main_gate_to_upper_flap",
        ArticulationType.REVOLUTE,
        parent=main_gate,
        child=upper_flap,
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed")
    main_gate = object_model.get_part("main_gate")
    upper_flap = object_model.get_part("upper_flap")
    main_hinge = object_model.get_articulation("bed_to_main_gate")
    flap_hinge = object_model.get_articulation("main_gate_to_upper_flap")

    ctx.check(
        "main gate uses lower transverse hinge",
        tuple(main_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={main_hinge.axis}",
    )
    ctx.check(
        "upper flap uses top transverse hinge",
        tuple(flap_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )
    for barrel_name in ("lower_gate_barrel_0", "lower_gate_barrel_1"):
        ctx.allow_overlap(
            bed,
            main_gate,
            elem_a="lower_hinge_pin",
            elem_b=barrel_name,
            reason="The lower hinge pin is intentionally captured inside the gate-side hinge barrel.",
        )
        ctx.expect_overlap(
            bed,
            main_gate,
            axes="xyz",
            elem_a="lower_hinge_pin",
            elem_b=barrel_name,
            min_overlap=0.015,
            name=f"{barrel_name} is captured on lower hinge pin",
        )

    with ctx.pose({main_hinge: 0.0, flap_hinge: 0.0}):
        ctx.expect_gap(
            upper_flap,
            main_gate,
            axis="z",
            positive_elem="flap_skin",
            negative_elem="main_skin",
            max_gap=0.001,
            max_penetration=0.0,
            name="upper flap sits on top edge of main gate",
        )
        ctx.expect_overlap(
            upper_flap,
            main_gate,
            axes="y",
            elem_a="flap_skin",
            elem_b="main_skin",
            min_overlap=1.50,
            name="upper flap spans same transverse tailgate opening",
        )
        ctx.expect_gap(
            main_gate,
            bed,
            axis="z",
            positive_elem="main_skin",
            negative_elem="bed_floor",
            max_gap=0.025,
            max_penetration=0.0,
            name="main gate bottom aligns with bed floor height",
        )

    closed_main = ctx.part_element_world_aabb(main_gate, elem="main_skin")
    closed_flap = ctx.part_element_world_aabb(upper_flap, elem="flap_skin")
    with ctx.pose({main_hinge: 1.20, flap_hinge: 0.0}):
        lowered_main = ctx.part_element_world_aabb(main_gate, elem="main_skin")
    with ctx.pose({main_hinge: 0.0, flap_hinge: 1.20}):
        folded_flap = ctx.part_element_world_aabb(upper_flap, elem="flap_skin")

    ctx.check(
        "main gate lowers rearward from bottom hinge",
        closed_main is not None
        and lowered_main is not None
        and lowered_main[0][0] < closed_main[0][0] - 0.35
        and lowered_main[1][2] < closed_main[1][2] - 0.12,
        details=f"closed={closed_main}, lowered={lowered_main}",
    )
    ctx.check(
        "upper flap folds rearward from top hinge",
        closed_flap is not None
        and folded_flap is not None
        and folded_flap[0][0] < closed_flap[0][0] - 0.10
        and folded_flap[1][2] < closed_flap[1][2] - 0.03,
        details=f"closed={closed_flap}, folded={folded_flap}",
    )

    return ctx.report()


object_model = build_object_model()
