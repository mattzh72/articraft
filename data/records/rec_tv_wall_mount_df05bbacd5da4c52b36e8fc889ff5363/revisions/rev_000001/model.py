from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_tv_wall_mount")

    powder_black = model.material("powder_black", rgba=(0.025, 0.027, 0.030, 1.0))
    satin_black = model.material("satin_black", rgba=(0.045, 0.048, 0.055, 1.0))
    dark_recess = model.material("dark_recess", rgba=(0.004, 0.004, 0.005, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.012, 0.012, 0.014, 1.0))
    parkerized = model.material("parkerized_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    rubbed_edges = model.material("rubbed_edges", rgba=(0.34, 0.35, 0.36, 1.0))
    safety_red = model.material("safety_red", rgba=(0.75, 0.05, 0.035, 1.0))

    # Root frame: X is wall width, +Y projects out from the wall, Z is vertical.
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.42, 0.035, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder_black,
        name="main_plate",
    )
    wall_plate.visual(
        Box((0.080, 0.020, 0.58)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=satin_black,
        name="center_rib",
    )
    for index, x in enumerate((-0.165, 0.165)):
        wall_plate.visual(
            Box((0.030, 0.014, 0.60)),
            origin=Origin(xyz=(x, 0.024, 0.0)),
            material=satin_black,
            name=f"side_rib_{index}",
        )
    for index, z in enumerate((-0.270, 0.270)):
        wall_plate.visual(
            Box((0.37, 0.010, 0.038)),
            origin=Origin(xyz=(0.0, 0.028, z)),
            material=satin_black,
            name=f"cross_stiffener_{index}",
        )
    slot_positions = [(-0.120, -0.255), (0.120, -0.255), (-0.120, 0.255), (0.120, 0.255)]
    for index, (x, z) in enumerate(slot_positions):
        wall_plate.visual(
            Box((0.080, 0.005, 0.020)),
            origin=Origin(xyz=(x, 0.020, z)),
            material=dark_recess,
            name=f"lag_slot_{index}",
        )
        wall_plate.visual(
            Cylinder(radius=0.018, length=0.009),
            origin=Origin(xyz=(x, 0.026, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=parkerized,
            name=f"lag_head_{index}",
        )
    wall_plate.visual(
        Box((0.135, 0.060, 0.36)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=satin_black,
        name="wall_pivot_spine",
    )
    wall_plate.visual(
        Cylinder(radius=0.046, length=0.285),
        origin=Origin(xyz=(0.0, 0.100, 0.0)),
        material=parkerized,
        name="wall_pivot_barrel",
    )
    wall_plate.visual(
        Cylinder(radius=0.064, length=0.028),
        origin=Origin(xyz=(0.0, 0.100, 0.158)),
        material=rubbed_edges,
        name="upper_bearing_cap",
    )
    wall_plate.visual(
        Cylinder(radius=0.064, length=0.028),
        origin=Origin(xyz=(0.0, 0.100, -0.158)),
        material=rubbed_edges,
        name="lower_bearing_cap",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.42, 0.13, 0.70)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )

    swing_arm = model.part("swing_arm")
    swing_arm.visual(
        Cylinder(radius=0.039, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=parkerized,
        name="wall_socket_barrel",
    )
    swing_arm.visual(
        Cylinder(radius=0.038, length=0.180),
        origin=Origin(xyz=(0.0, 0.340, 0.0)),
        material=parkerized,
        name="elbow_barrel",
    )
    for index, z in enumerate((-0.064, 0.064)):
        swing_arm.visual(
            Box((0.066, 0.240, 0.028)),
            origin=Origin(xyz=(0.0, 0.180, z)),
            material=satin_black,
            name=f"main_link_{index}",
        )
    swing_arm.visual(
        Box((0.040, 0.235, 0.018)),
        origin=Origin(xyz=(0.0, 0.180, 0.0), rpy=(math.radians(21.0), 0.0, 0.0)),
        material=powder_black,
        name="diagonal_web",
    )
    swing_arm.visual(
        Box((0.095, 0.038, 0.145)),
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        material=satin_black,
        name="rear_clevis_bridge",
    )
    swing_arm.visual(
        Box((0.090, 0.040, 0.135)),
        origin=Origin(xyz=(0.0, 0.300, 0.0)),
        material=satin_black,
        name="front_clevis_bridge",
    )
    swing_arm.inertial = Inertial.from_geometry(
        Box((0.12, 0.38, 0.18)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.17, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.036, length=0.172),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=parkerized,
        name="elbow_socket_barrel",
    )
    forearm.visual(
        Cylinder(radius=0.036, length=0.172),
        origin=Origin(xyz=(0.0, 0.300, 0.0)),
        material=parkerized,
        name="nose_barrel",
    )
    for index, z in enumerate((-0.055, 0.055)):
        forearm.visual(
            Box((0.058, 0.210, 0.026)),
            origin=Origin(xyz=(0.0, 0.155, z)),
            material=satin_black,
            name=f"fore_link_{index}",
        )
    forearm.visual(
        Box((0.034, 0.205, 0.016)),
        origin=Origin(xyz=(0.0, 0.155, 0.0), rpy=(math.radians(-19.0), 0.0, 0.0)),
        material=powder_black,
        name="forearm_web",
    )
    forearm.visual(
        Box((0.082, 0.038, 0.125)),
        origin=Origin(xyz=(0.0, 0.0545, 0.0)),
        material=satin_black,
        name="elbow_bridge",
    )
    forearm.visual(
        Box((0.082, 0.038, 0.125)),
        origin=Origin(xyz=(0.0, 0.2455, 0.0)),
        material=satin_black,
        name="nose_bridge",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.11, 0.34, 0.16)),
        mass=1.75,
        origin=Origin(xyz=(0.0, 0.15, 0.0)),
    )

    screen_head = model.part("screen_head")
    screen_head.visual(
        Cylinder(radius=0.037, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=parkerized,
        name="nose_socket_barrel",
    )
    screen_head.visual(
        Box((0.245, 0.057, 0.034)),
        origin=Origin(xyz=(0.0, 0.064, 0.082)),
        material=satin_black,
        name="head_backbone",
    )
    screen_head.visual(
        Box((0.245, 0.057, 0.034)),
        origin=Origin(xyz=(0.0, 0.064, -0.082)),
        material=satin_black,
        name="lower_head_bridge",
    )
    screen_head.visual(
        Box((0.028, 0.055, 0.225)),
        origin=Origin(xyz=(-0.112, 0.080, 0.0)),
        material=satin_black,
        name="tilt_cheek_0",
    )
    screen_head.visual(
        Cylinder(radius=0.048, length=0.016),
        origin=Origin(xyz=(-0.112, 0.080, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubbed_edges,
        name="tilt_washer_0",
    )
    screen_head.visual(
        Box((0.028, 0.055, 0.225)),
        origin=Origin(xyz=(0.112, 0.080, 0.0)),
        material=satin_black,
        name="tilt_cheek_1",
    )
    screen_head.visual(
        Cylinder(radius=0.048, length=0.016),
        origin=Origin(xyz=(0.112, 0.080, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubbed_edges,
        name="tilt_washer_1",
    )
    screen_head.visual(
        Box((0.245, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.055, 0.132)),
        material=satin_black,
        name="upper_yoke_bridge",
    )
    screen_head.visual(
        Box((0.245, 0.030, 0.045)),
        origin=Origin(xyz=(0.0, 0.055, -0.132)),
        material=satin_black,
        name="lower_yoke_bridge",
    )
    screen_head.inertial = Inertial.from_geometry(
        Box((0.25, 0.13, 0.28)),
        mass=1.25,
        origin=Origin(xyz=(0.0, 0.06, 0.0)),
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Cylinder(radius=0.026, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=parkerized,
        name="trunnion_barrel",
    )
    tilt_frame.visual(
        Box((0.118, 0.110, 0.050)),
        origin=Origin(xyz=(0.0, 0.063, 0.0)),
        material=satin_black,
        name="tilt_neck",
    )
    tilt_frame.visual(
        Box((0.130, 0.100, 0.075)),
        origin=Origin(xyz=(0.0, 0.130, 0.0)),
        material=satin_black,
        name="central_standoff",
    )
    tilt_frame.visual(
        Box((0.330, 0.030, 0.370)),
        origin=Origin(xyz=(0.0, 0.177, 0.0)),
        material=powder_black,
        name="vesa_plate",
    )
    tilt_frame.visual(
        Box((0.640, 0.030, 0.047)),
        origin=Origin(xyz=(0.0, 0.184, 0.180)),
        material=satin_black,
        name="upper_crossbar",
    )
    tilt_frame.visual(
        Box((0.640, 0.030, 0.047)),
        origin=Origin(xyz=(0.0, 0.184, -0.180)),
        material=satin_black,
        name="lower_crossbar",
    )
    for index, x in enumerate((-0.225, 0.225)):
        tilt_frame.visual(
            Box((0.052, 0.034, 0.490)),
            origin=Origin(xyz=(x, 0.190, 0.0)),
            material=satin_black,
            name=f"tv_rail_{index}",
        )
        tilt_frame.visual(
            Box((0.072, 0.043, 0.045)),
            origin=Origin(xyz=(x, 0.205, 0.240)),
            material=powder_black,
            name=f"top_hook_{index}",
        )
        tilt_frame.visual(
            Box((0.068, 0.040, 0.040)),
            origin=Origin(xyz=(x, 0.205, -0.240)),
            material=powder_black,
            name=f"lower_hook_{index}",
        )
    for index, (x, z) in enumerate(((-0.100, -0.075), (0.100, -0.075), (-0.100, 0.075), (0.100, 0.075))):
        tilt_frame.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(x, 0.191, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_recess,
            name=f"vesa_bore_{index}",
        )
        tilt_frame.visual(
            Cylinder(radius=0.020, length=0.008),
            origin=Origin(xyz=(x, 0.195, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=parkerized,
            name=f"vesa_screw_{index}",
        )
    tilt_frame.visual(
        Box((0.225, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, 0.190, -0.208)),
        material=satin_black,
        name="latch_housing",
    )
    tilt_frame.visual(
        Box((0.180, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.202, -0.235)),
        material=dark_recess,
        name="latch_slot",
    )
    tilt_frame.inertial = Inertial.from_geometry(
        Box((0.68, 0.23, 0.55)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.16, 0.0)),
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=parkerized,
        name="knob_stem",
    )
    tilt_knob.visual(
        Cylinder(radius=0.040, length=0.022),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="ribbed_handwheel",
    )
    tilt_knob.visual(
        Cylinder(radius=0.026, length=0.026),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_recess,
        name="recessed_knob_face",
    )
    tilt_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.04, length=0.06),
        mass=0.16,
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
    )

    safety_latch = model.part("safety_latch")
    safety_latch.visual(
        Box((0.165, 0.012, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=parkerized,
        name="sliding_latch",
    )
    safety_latch.visual(
        Box((0.060, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.007, -0.024)),
        material=safety_red,
        name="release_tab",
    )
    safety_latch.visual(
        Cylinder(radius=0.0035, length=0.094),
        origin=Origin(xyz=(0.0, 0.010, -0.077)),
        material=safety_red,
        name="pull_cord",
    )
    safety_latch.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.010, -0.130), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_red,
        name="cord_pull",
    )
    safety_latch.inertial = Inertial.from_geometry(
        Box((0.17, 0.02, 0.15)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.006, -0.060)),
    )

    model.articulation(
        "wall_swing",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=swing_arm,
        origin=Origin(xyz=(0.0, 0.100, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.9, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "elbow_swing",
        ArticulationType.REVOLUTE,
        parent=swing_arm,
        child=forearm,
        origin=Origin(xyz=(0.0, 0.340, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=-2.15, upper=2.15),
    )
    model.articulation(
        "screen_swivel",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=screen_head,
        origin=Origin(xyz=(0.0, 0.300, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "screen_tilt",
        ArticulationType.REVOLUTE,
        parent=screen_head,
        child=tilt_frame,
        origin=Origin(xyz=(0.0, 0.080, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.65,
            lower=math.radians(-15.0),
            upper=math.radians(12.0),
        ),
    )
    model.articulation(
        "tilt_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=screen_head,
        child=tilt_knob,
        origin=Origin(xyz=(0.145, 0.080, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "latch_release",
        ArticulationType.PRISMATIC,
        parent=tilt_frame,
        child=safety_latch,
        origin=Origin(xyz=(0.0, 0.200, -0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.10, lower=0.0, upper=0.035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_plate")
    swing = object_model.get_part("swing_arm")
    forearm = object_model.get_part("forearm")
    head = object_model.get_part("screen_head")
    frame = object_model.get_part("tilt_frame")
    knob = object_model.get_part("tilt_knob")
    latch = object_model.get_part("safety_latch")

    wall_swing = object_model.get_articulation("wall_swing")
    elbow_swing = object_model.get_articulation("elbow_swing")
    screen_swivel = object_model.get_articulation("screen_swivel")
    screen_tilt = object_model.get_articulation("screen_tilt")
    latch_release = object_model.get_articulation("latch_release")

    ctx.allow_overlap(
        wall,
        swing,
        elem_a="wall_pivot_barrel",
        elem_b="wall_socket_barrel",
        reason="The swing-arm bearing collar is intentionally captured around the fixed vertical wall pivot.",
    )
    ctx.allow_overlap(
        wall,
        swing,
        elem_a="wall_pivot_barrel",
        elem_b="rear_clevis_bridge",
        reason="The rear clevis bridge is welded into the same compact wall-pivot bearing stack.",
    )
    ctx.allow_overlap(
        wall,
        swing,
        elem_a="wall_pivot_spine",
        elem_b="wall_socket_barrel",
        reason="The fixed spine wraps close to the rotating bearing sleeve in the simplified wall-pivot stack.",
    )
    ctx.expect_overlap(
        wall,
        swing,
        axes="z",
        elem_a="wall_pivot_barrel",
        elem_b="wall_socket_barrel",
        min_overlap=0.16,
        name="wall pivot collar retains vertical engagement",
    )
    ctx.expect_overlap(
        wall,
        swing,
        axes="z",
        elem_a="wall_pivot_barrel",
        elem_b="rear_clevis_bridge",
        min_overlap=0.12,
        name="rear clevis bridge is inside pivot stack height",
    )
    ctx.expect_overlap(
        wall,
        swing,
        axes="z",
        elem_a="wall_pivot_spine",
        elem_b="wall_socket_barrel",
        min_overlap=0.18,
        name="wall pivot spine supports sleeve height",
    )

    ctx.allow_overlap(
        swing,
        forearm,
        elem_a="elbow_barrel",
        elem_b="elbow_socket_barrel",
        reason="The elbow bearing barrels are represented as nested steel sleeves around the same pin.",
    )
    ctx.allow_overlap(
        swing,
        forearm,
        elem_a="front_clevis_bridge",
        elem_b="elbow_socket_barrel",
        reason="The forearm bushing is captured inside the compact front clevis bridge at the elbow pin.",
    )
    ctx.expect_overlap(
        swing,
        forearm,
        axes="z",
        elem_a="elbow_barrel",
        elem_b="elbow_socket_barrel",
        min_overlap=0.15,
        name="elbow nested sleeves stay engaged",
    )
    ctx.expect_overlap(
        swing,
        forearm,
        axes="z",
        elem_a="front_clevis_bridge",
        elem_b="elbow_socket_barrel",
        min_overlap=0.12,
        name="elbow bushing sits within clevis bridge height",
    )

    ctx.allow_overlap(
        forearm,
        head,
        elem_a="nose_barrel",
        elem_b="nose_socket_barrel",
        reason="The screen swivel bearing is a captured vertical sleeve, simplified as coincident cylindrical bushings.",
    )
    ctx.expect_overlap(
        forearm,
        head,
        axes="z",
        elem_a="nose_barrel",
        elem_b="nose_socket_barrel",
        min_overlap=0.15,
        name="screen swivel bearing is captured",
    )

    for cheek in ("tilt_cheek_0", "tilt_cheek_1"):
        ctx.allow_overlap(
            head,
            frame,
            elem_a=cheek,
            elem_b="trunnion_barrel",
            reason="The tilt trunnion passes through the cheek bore of the yoke.",
        )
    for washer in ("tilt_washer_0", "tilt_washer_1"):
        ctx.allow_overlap(
            head,
            frame,
            elem_a=washer,
            elem_b="trunnion_barrel",
            reason="The broad friction washer is drawn as a solid disk around the tilt trunnion proxy.",
        )
    ctx.expect_overlap(
        head,
        frame,
        axes="x",
        elem_a="tilt_cheek_0",
        elem_b="trunnion_barrel",
        min_overlap=0.010,
        name="tilt trunnion crosses one cheek",
    )
    ctx.expect_overlap(
        head,
        frame,
        axes="x",
        elem_a="tilt_cheek_1",
        elem_b="trunnion_barrel",
        min_overlap=0.010,
        name="tilt trunnion crosses opposite cheek",
    )
    ctx.expect_overlap(
        head,
        frame,
        axes="x",
        elem_a="tilt_washer_1",
        elem_b="trunnion_barrel",
        min_overlap=0.010,
        name="tilt washer surrounds trunnion end",
    )

    ctx.allow_overlap(
        head,
        knob,
        elem_a="tilt_cheek_1",
        elem_b="knob_stem",
        reason="The tightening knob stem threads through the side cheek and bears on the tilt trunnion.",
    )
    ctx.allow_overlap(
        head,
        knob,
        elem_a="tilt_washer_1",
        elem_b="knob_stem",
        reason="The handwheel stem passes through the outer friction washer before clamping the tilt joint.",
    )
    ctx.allow_overlap(
        frame,
        knob,
        elem_a="trunnion_barrel",
        elem_b="knob_stem",
        reason="The tightening stem bottoms into the trunnion clamp proxy at the tilt axis.",
    )
    ctx.expect_overlap(
        head,
        knob,
        axes="x",
        elem_a="tilt_cheek_1",
        elem_b="knob_stem",
        min_overlap=0.008,
        name="tilt knob stem enters cheek",
    )
    ctx.expect_overlap(
        frame,
        knob,
        axes="x",
        elem_a="trunnion_barrel",
        elem_b="knob_stem",
        min_overlap=0.010,
        name="tilt knob bears on trunnion",
    )

    ctx.allow_overlap(
        frame,
        latch,
        elem_a="latch_slot",
        elem_b="sliding_latch",
        reason="The spring safety latch is intentionally represented as sliding in a simplified solid slot.",
    )
    ctx.allow_overlap(
        frame,
        latch,
        elem_a="latch_housing",
        elem_b="sliding_latch",
        reason="The latch housing is a simplified surrounding guide channel around the sliding latch tongue.",
    )
    ctx.expect_contact(
        frame,
        latch,
        elem_a="latch_slot",
        elem_b="sliding_latch",
        contact_tol=0.004,
        name="safety latch rides in lower crossbar slot",
    )
    ctx.expect_overlap(
        frame,
        latch,
        axes="xz",
        elem_a="latch_housing",
        elem_b="sliding_latch",
        min_overlap=0.030,
        name="sliding latch is retained by guide housing",
    )

    ctx.expect_overlap(
        frame,
        wall,
        axes="x",
        elem_a="upper_crossbar",
        elem_b="main_plate",
        min_overlap=0.35,
        name="screen rails are broader than the wall plate",
    )

    rest_frame_aabb = ctx.part_world_aabb(frame)
    with ctx.pose({wall_swing: 0.80}):
        swung_frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "wall swing moves screen bracket sideways",
        rest_frame_aabb is not None
        and swung_frame_aabb is not None
        and swung_frame_aabb[1][0] < rest_frame_aabb[1][0] - 0.12,
        details=f"rest={rest_frame_aabb}, swung={swung_frame_aabb}",
    )

    with ctx.pose({elbow_swing: -1.10, screen_swivel: 0.45}):
        posed = ctx.part_world_aabb(frame)
    ctx.check(
        "elbow and screen swivel produce compound articulation",
        rest_frame_aabb is not None
        and posed is not None
        and posed[1][1] < rest_frame_aabb[1][1] - 0.04,
        details=f"rest={rest_frame_aabb}, posed={posed}",
    )

    top_at_rest = ctx.part_element_world_aabb(frame, elem="upper_crossbar")
    with ctx.pose({screen_tilt: math.radians(-15.0)}):
        top_tilted = ctx.part_element_world_aabb(frame, elem="upper_crossbar")
    ctx.check(
        "down tilt tips top of bracket away from wall",
        top_at_rest is not None
        and top_tilted is not None
        and top_tilted[0][1] > top_at_rest[0][1] + 0.030,
        details=f"rest={top_at_rest}, tilted={top_tilted}",
    )

    latch_rest = ctx.part_world_aabb(latch)
    with ctx.pose({latch_release: 0.035}):
        latch_released = ctx.part_world_aabb(latch)
    ctx.check(
        "safety latch slides upward to release hooks",
        latch_rest is not None
        and latch_released is not None
        and latch_released[0][2] > latch_rest[0][2] + 0.030,
        details=f"rest={latch_rest}, released={latch_released}",
    )

    return ctx.report()


object_model = build_object_model()
