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
    model = ArticulatedObject(name="under_shelf_monitor_mount")

    satin_black = Material("satin_black_powdercoat", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_edge = Material("dark_rubber_edge", rgba=(0.005, 0.005, 0.006, 1.0))
    brushed_pin = Material("brushed_steel_pins", rgba=(0.62, 0.62, 0.58, 1.0))
    screw_dark = Material("black_socket_screws", rgba=(0.0, 0.0, 0.0, 1.0))

    # Root frame is on the shoulder pivot axis at the underside of the shelf plate.
    top_plate = model.part("top_plate")
    top_plate.visual(
        Box((0.320, 0.160, 0.012)),
        origin=Origin(xyz=(0.140, 0.0, 0.006)),
        material=satin_black,
        name="plate",
    )
    # Shallow formed lips keep the plate from reading as a simple flat slab.
    top_plate.visual(
        Box((0.320, 0.010, 0.018)),
        origin=Origin(xyz=(0.140, 0.075, -0.003)),
        material=satin_black,
        name="side_lip_0",
    )
    top_plate.visual(
        Box((0.320, 0.010, 0.018)),
        origin=Origin(xyz=(0.140, -0.075, -0.003)),
        material=satin_black,
        name="side_lip_1",
    )
    # Four exposed socket-head screws on the underside mounting plate.
    for i, (x, y) in enumerate(((-0.005, -0.052), (-0.005, 0.052), (0.250, -0.052), (0.250, 0.052))):
        top_plate.visual(
            Cylinder(radius=0.009, length=0.003),
            origin=Origin(xyz=(x, y, -0.0015)),
            material=screw_dark,
            name=f"screw_head_{i}",
        )

    # The shoulder pivot is supported by two cheeks hanging from the top plate.
    top_plate.visual(
        Box((0.078, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, 0.032, -0.020)),
        material=satin_black,
        name="shoulder_cheek_0",
    )
    top_plate.visual(
        Box((0.078, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, -0.032, -0.020)),
        material=satin_black,
        name="shoulder_cheek_1",
    )
    top_plate.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=brushed_pin,
        name="shoulder_upper_pin_cap",
    )

    base_arm = model.part("base_arm")
    # Upper folded link, a wide low profile stamped bar.
    base_arm.visual(
        Box((0.205, 0.046, 0.012)),
        origin=Origin(xyz=(0.1025, 0.0, 0.0)),
        material=satin_black,
        name="upper_bar",
    )
    base_arm.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="shoulder_eye",
    )
    base_arm.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=brushed_pin,
        name="shoulder_pin",
    )
    # A supported elbow fork at the far end of the first link.
    base_arm.visual(
        Box((0.066, 0.086, 0.008)),
        origin=Origin(xyz=(0.205, 0.0, -0.006)),
        material=satin_black,
        name="elbow_bridge",
    )
    base_arm.visual(
        Box((0.060, 0.014, 0.048)),
        origin=Origin(xyz=(0.205, 0.034, -0.023)),
        material=satin_black,
        name="elbow_cheek_0",
    )
    base_arm.visual(
        Box((0.060, 0.014, 0.048)),
        origin=Origin(xyz=(0.205, -0.034, -0.023)),
        material=satin_black,
        name="elbow_cheek_1",
    )
    for i, y in enumerate((-0.045, 0.045)):
        base_arm.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(0.205, y, -0.023), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed_pin,
            name=f"elbow_pin_cap_{i}",
        )

    forearm = model.part("forearm")
    # Lower link folds back underneath the upper link at q=0, keeping the stack shallow.
    forearm.visual(
        Box((0.185, 0.040, 0.012)),
        origin=Origin(xyz=(-0.0925, 0.0, -0.020)),
        material=satin_black,
        name="lower_bar",
    )
    forearm.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=satin_black,
        name="elbow_eye",
    )
    forearm.visual(
        Box((0.044, 0.058, 0.010)),
        origin=Origin(xyz=(-0.185, 0.0, -0.020)),
        material=satin_black,
        name="pan_mount_pad",
    )
    forearm.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.185, 0.0, -0.020)),
        material=brushed_pin,
        name="pan_lower_bushing",
    )

    pan_carriage = model.part("pan_carriage")
    # Independent pan stage: a compact turntable below the lower arm.
    pan_carriage.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=satin_black,
        name="pan_turntable",
    )
    pan_carriage.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=brushed_pin,
        name="pan_spindle",
    )
    pan_carriage.visual(
        Box((0.030, 0.034, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, -0.018)),
        material=satin_black,
        name="pitch_block",
    )
    pan_carriage.visual(
        Box((0.012, 0.118, 0.050)),
        origin=Origin(xyz=(0.025, 0.0, -0.030)),
        material=satin_black,
        name="pitch_back_wall",
    )
    pan_carriage.visual(
        Box((0.030, 0.014, 0.050)),
        origin=Origin(xyz=(0.045, 0.052, -0.030)),
        material=satin_black,
        name="pitch_cheek_0",
    )
    pan_carriage.visual(
        Box((0.030, 0.014, 0.050)),
        origin=Origin(xyz=(0.045, -0.052, -0.030)),
        material=satin_black,
        name="pitch_cheek_1",
    )

    head_frame = model.part("head_frame")
    # A compact rectangular VESA-style head frame, modeled without a screen.
    # Its zero pitch pose is folded nearly parallel to the shelf plate.
    head_frame.visual(
        Cylinder(radius=0.011, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="pitch_barrel",
    )
    head_frame.visual(
        Box((0.090, 0.010, 0.010)),
        origin=Origin(xyz=(0.060, 0.037, 0.0)),
        material=satin_black,
        name="frame_rail_0",
    )
    head_frame.visual(
        Box((0.090, 0.010, 0.010)),
        origin=Origin(xyz=(0.060, -0.037, 0.0)),
        material=satin_black,
        name="frame_rail_1",
    )
    head_frame.visual(
        Box((0.012, 0.084, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=satin_black,
        name="hinge_side_rail",
    )
    head_frame.visual(
        Box((0.012, 0.084, 0.010)),
        origin=Origin(xyz=(0.098, 0.0, 0.0)),
        material=satin_black,
        name="outer_side_rail",
    )
    head_frame.visual(
        Box((0.010, 0.084, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=satin_black,
        name="center_crossbar",
    )
    for i, (x, y) in enumerate(((0.034, -0.037), (0.034, 0.037), (0.086, -0.037), (0.086, 0.037))):
        head_frame.visual(
            Cylinder(radius=0.006, length=0.002),
            origin=Origin(xyz=(x, y, -0.006)),
            material=dark_edge,
            name=f"vesa_pad_{i}",
        )

    shoulder = model.articulation(
        "plate_to_base_arm",
        ArticulationType.REVOLUTE,
        parent=top_plate,
        child=base_arm,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "base_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=base_arm,
        child=forearm,
        origin=Origin(xyz=(0.205, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-2.70, upper=0.25),
    )
    model.articulation(
        "forearm_to_pan_carriage",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=pan_carriage,
        origin=Origin(xyz=(-0.185, 0.0, -0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "pan_carriage_to_head_frame",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=head_frame,
        origin=Origin(xyz=(0.054, 0.0, -0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.5, lower=-1.25, upper=0.20),
    )

    # Record representative folded/open poses for downstream QC sampling.
    shoulder.meta["qc_samples"] = [0.0, 1.1, -1.1]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    top_plate = object_model.get_part("top_plate")
    base_arm = object_model.get_part("base_arm")
    forearm = object_model.get_part("forearm")
    pan_carriage = object_model.get_part("pan_carriage")
    head_frame = object_model.get_part("head_frame")
    shoulder = object_model.get_articulation("plate_to_base_arm")
    elbow = object_model.get_articulation("base_arm_to_forearm")
    pan = object_model.get_articulation("forearm_to_pan_carriage")
    pitch = object_model.get_articulation("pan_carriage_to_head_frame")

    ctx.check(
        "four articulated user joints",
        len(object_model.articulations) == 4,
        details=f"found {len(object_model.articulations)} articulations",
    )
    for joint, lower, upper in (
        (shoulder, -1.75, 1.75),
        (elbow, -2.70, 0.25),
        (pan, -1.57, 1.57),
        (pitch, -1.25, 0.20),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has realistic stops",
            limits is not None and limits.lower == lower and limits.upper == upper,
            details=f"limits={limits}",
        )

    with ctx.pose({shoulder: 0.0, elbow: 0.0, pan: 0.0, pitch: 0.0}):
        ctx.expect_within(
            head_frame,
            top_plate,
            axes="xy",
            margin=0.075,
            name="folded head stays near plate footprint",
        )
        ctx.expect_gap(
            top_plate,
            head_frame,
            axis="z",
            min_gap=0.045,
            max_gap=0.120,
            positive_elem="plate",
            name="folded stack is shallow under the shelf plate",
        )
        ctx.expect_overlap(
            base_arm,
            forearm,
            axes="x",
            min_overlap=0.100,
            name="folded links nest over each other in plan",
        )

    folded_aabb = ctx.part_world_aabb(head_frame)
    with ctx.pose({shoulder: 1.05, elbow: -1.75, pan: 0.65, pitch: -1.05}):
        ctx.expect_origin_distance(
            head_frame,
            top_plate,
            axes="xy",
            min_dist=0.130,
            name="articulated arm carries head away from plate",
        )
        lowered_aabb = ctx.part_world_aabb(head_frame)
    ctx.check(
        "pitch hinge can drop the head frame",
        folded_aabb is not None
        and lowered_aabb is not None
        and lowered_aabb[0][2] < folded_aabb[0][2] - 0.020,
        details=f"folded_aabb={folded_aabb}, lowered_aabb={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
