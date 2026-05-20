from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


ROLL_TO_Y = (-math.pi / 2.0, 0.0, 0.0)
PITCH_TO_X = (0.0, math.pi / 2.0, 0.0)


def _rounded_pad(length: float, width: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(length, width, radius, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_adjustable_bench")

    steel = model.material("satin_black_powdercoat", color=(0.015, 0.016, 0.018, 1.0))
    worn_steel = model.material("brushed_welded_steel", color=(0.33, 0.35, 0.36, 1.0))
    vinyl = model.material("black_grained_vinyl", color=(0.025, 0.025, 0.023, 1.0))
    seam = model.material("stitched_vinyl_seam", color=(0.09, 0.09, 0.085, 1.0))
    rubber = model.material("matte_rubber", color=(0.006, 0.006, 0.005, 1.0))
    rim_mat = model.material("dark_plastic_rim", color=(0.055, 0.058, 0.062, 1.0))
    cap_mat = model.material("zinc_axle_cap", color=(0.55, 0.56, 0.52, 1.0))

    pad_back = _rounded_pad(0.92, 0.36, 0.075, 0.060, "backrest_rounded_pad")
    pad_seat = _rounded_pad(0.42, 0.38, 0.075, 0.055, "seat_rounded_pad")

    wheel_rim = mesh_from_geometry(
        WheelGeometry(
            0.052,
            0.044,
            rim=WheelRim(inner_radius=0.033, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(radius=0.020, width=0.036, cap_style="flat"),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.020),
        ),
        "transport_wheel_rim",
    )
    wheel_tire = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.052,
            inner_radius=0.053,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.03),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.60),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "transport_wheel_tire",
    )

    frame = model.part("frame")
    # Welded rectangular tube base: long rails, heavy crossmembers, hinge towers,
    # axle stubs, and a front carry handle all intersect at weld-like joints.
    for y, name in ((-0.235, "side_rail_0"), (0.235, "side_rail_1")):
        frame.visual(
            Box((1.42, 0.055, 0.065)),
            origin=Origin(xyz=(0.02, y, 0.18)),
            material=steel,
            name=name,
        )
    frame.visual(
        Box((1.52, 0.070, 0.075)),
        origin=Origin(xyz=(0.02, 0.0, 0.205)),
        material=steel,
        name="center_spine",
    )
    for x, name in ((-0.72, "front_crossmember"), (0.72, "rear_crossmember")):
        frame.visual(
            Box((0.095, 0.70, 0.075)),
            origin=Origin(xyz=(x, 0.0, 0.18)),
            material=steel,
            name=name,
        )
    for x, name in ((-0.72, "front_floor_foot"), (0.72, "rear_floor_foot")):
        frame.visual(
            Box((0.13, 0.82, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.065)),
            material=steel,
            name=name,
        )
        for y in (-0.235, 0.235):
            frame.visual(
                Box((0.060, 0.060, 0.130)),
                origin=Origin(xyz=(x, y, 0.118)),
                material=steel,
                name=f"{name}_post_{0 if y < 0 else 1}",
            )
        for y in (-0.43, 0.43):
            frame.visual(
                Box((0.16, 0.060, 0.035)),
                origin=Origin(xyz=(x, y, 0.044)),
                material=rubber,
                name=f"{name}_rubber_{0 if y < 0 else 1}",
            )

    # Forward seat hinge tower and rear/backrest hinge tower.
    for x, y, prefix in (
        (-0.53, -0.175, "seat_hinge_post_0"),
        (-0.53, 0.175, "seat_hinge_post_1"),
        (-0.05, -0.215, "backrest_hinge_post_0"),
        (-0.05, 0.215, "backrest_hinge_post_1"),
    ):
        frame.visual(
            Box((0.060, 0.075, 0.330)),
            origin=Origin(xyz=(x, y, 0.345)),
            material=steel,
            name=prefix,
        )
    frame.visual(
        Cylinder(radius=0.014, length=0.48),
        origin=Origin(xyz=(-0.05, 0.0, 0.50), rpy=ROLL_TO_Y),
        material=cap_mat,
        name="backrest_hinge_pin",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.40),
        origin=Origin(xyz=(-0.53, 0.0, 0.50), rpy=ROLL_TO_Y),
        material=cap_mat,
        name="seat_hinge_pin",
    )

    # Lower pivot brackets for the wide rear support arm.
    for y, name in ((-0.215, "support_pivot_bracket_0"), (0.215, "support_pivot_bracket_1")):
        frame.visual(
            Box((0.075, 0.050, 0.180)),
            origin=Origin(xyz=(0.30, y, 0.300)),
            material=steel,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.012, length=0.46),
        origin=Origin(xyz=(0.30, 0.0, 0.31), rpy=ROLL_TO_Y),
        material=cap_mat,
        name="support_pivot_pin",
    )

    # Small transport-wheel axle mounts and axle stubs.
    for y, name, stub_name in (
        (-0.365, "wheel_mount_0", "axle_stub_0"),
        (0.365, "wheel_mount_1", "axle_stub_1"),
    ):
        frame.visual(
            Box((0.105, 0.050, 0.115)),
            origin=Origin(xyz=(-0.95, y * 0.75, 0.115)),
            material=steel,
            name=name,
        )
        frame.visual(
            Box((0.260, 0.040, 0.040)),
            origin=Origin(xyz=(-0.85, y * 0.78, 0.135)),
            material=steel,
            name=f"axle_fork_{0 if y < 0 else 1}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.125),
            origin=Origin(xyz=(-0.95, y, 0.115), rpy=ROLL_TO_Y),
            material=cap_mat,
            name=stub_name,
        )

    # The carry handle is welded into the front crossmember and projects between
    # the two transport wheels instead of floating ahead of them.
    for y, name in ((-0.095, "handle_side_0"), (0.095, "handle_side_1")):
        frame.visual(
            Cylinder(radius=0.018, length=0.285),
            origin=Origin(xyz=(-0.875, y, 0.205), rpy=PITCH_TO_X),
            material=steel,
            name=name,
        )
    frame.visual(
        Cylinder(radius=0.020, length=0.230),
        origin=Origin(xyz=(-1.015, 0.0, 0.205), rpy=ROLL_TO_Y),
        material=rubber,
        name="front_carry_handle",
    )

    backrest = model.part("backrest")
    backrest.visual(
        pad_back,
        origin=Origin(xyz=(0.48, 0.0, 0.046)),
        material=vinyl,
        name="pad",
    )
    for x in (0.22, 0.48, 0.74):
        backrest.visual(
            Box((0.012, 0.330, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.086)),
            material=seam,
            name=f"seam_{int(x * 100)}",
        )
    backrest.visual(
        Box((0.84, 0.245, 0.026)),
        origin=Origin(xyz=(0.43, 0.0, -0.003)),
        material=worn_steel,
        name="backrest_plate",
    )
    backrest.visual(
        Box((0.68, 0.050, 0.020)),
        origin=Origin(xyz=(0.47, 0.0, -0.006)),
        material=steel,
        name="center_channel",
    )
    for i, x in enumerate((0.22, 0.34, 0.46, 0.58, 0.70)):
        backrest.visual(
            Box((0.040, 0.082, 0.018)),
            origin=Origin(xyz=(x, 0.0, -0.020)),
            material=worn_steel,
            name=f"adjust_notch_{i}",
        )
    backrest.visual(
        Cylinder(radius=0.032, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, -0.005), rpy=ROLL_TO_Y),
        material=worn_steel,
        name="hinge_barrel",
    )

    seat = model.part("seat")
    seat.visual(
        pad_seat,
        origin=Origin(xyz=(0.225, 0.0, 0.046)),
        material=vinyl,
        name="pad",
    )
    seat.visual(
        Box((0.012, 0.345, 0.006)),
        origin=Origin(xyz=(0.225, 0.0, 0.086)),
        material=seam,
        name="center_seam",
    )
    seat.visual(
        Box((0.36, 0.260, 0.026)),
        origin=Origin(xyz=(0.205, 0.0, -0.003)),
        material=worn_steel,
        name="seat_plate",
    )
    seat.visual(
        Cylinder(radius=0.028, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, -0.004), rpy=ROLL_TO_Y),
        material=worn_steel,
        name="hinge_barrel",
    )

    support = model.part("support_arm")
    for y, name in ((-0.155, "side_tube_0"), (0.155, "side_tube_1")):
        support.visual(
            Box((0.430, 0.038, 0.045)),
            origin=Origin(xyz=(0.215, y, -0.035)),
            material=steel,
            name=name,
        )
    for x, name in ((0.040, "lower_bridge"), (0.390, "upper_bridge")):
        support.visual(
            Box((0.045, 0.350, 0.045)),
            origin=Origin(xyz=(x, 0.0, -0.035)),
            material=steel,
            name=name,
        )
    support.visual(
        Cylinder(radius=0.019, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ROLL_TO_Y),
        material=worn_steel,
        name="pivot_barrel",
    )
    support.visual(
        Cylinder(radius=0.014, length=0.365),
        origin=Origin(xyz=(0.425, 0.0, -0.020), rpy=ROLL_TO_Y),
        material=cap_mat,
        name="top_roller",
    )

    for y, name in ((-0.365, "wheel_0"), (0.365, "wheel_1")):
        wheel = model.part(name)
        wheel.visual(
            wheel_tire,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_rim,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_mat,
            name="rim",
        )
        model.articulation(
            f"{name}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.95, y, 0.115)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.05, 0.0, 0.50)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=0.0, upper=1.28),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.53, 0.0, 0.50)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=0.48),
    )
    model.articulation(
        "support_arm_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support,
        origin=Origin(xyz=(0.30, 0.0, 0.31), rpy=(0.0, -0.38, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=0.58),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support = object_model.get_part("support_arm")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    backrest_hinge = object_model.get_articulation("backrest_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    support_pivot = object_model.get_articulation("support_arm_pivot")

    ctx.allow_overlap(
        backrest,
        frame,
        elem_a="hinge_barrel",
        elem_b="backrest_hinge_pin",
        reason="The backrest hinge barrel is intentionally captured around the transverse hinge pin.",
    )
    ctx.expect_overlap(
        backrest,
        frame,
        axes="y",
        min_overlap=0.25,
        elem_a="hinge_barrel",
        elem_b="backrest_hinge_pin",
        name="backrest hinge barrel is retained on pin",
    )
    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_pin",
        elem_b="hinge_barrel",
        reason="The seat hinge barrel is intentionally captured around the forward hinge pin.",
    )
    ctx.expect_overlap(
        seat,
        frame,
        axes="y",
        min_overlap=0.20,
        elem_a="hinge_barrel",
        elem_b="seat_hinge_pin",
        name="seat hinge barrel is retained on pin",
    )
    ctx.allow_overlap(
        frame,
        support,
        elem_a="support_pivot_pin",
        elem_b="pivot_barrel",
        reason="The wide rear support arm rotates around a lower frame pivot pin nested in its barrel.",
    )
    ctx.expect_overlap(
        support,
        frame,
        axes="y",
        min_overlap=0.30,
        elem_a="pivot_barrel",
        elem_b="support_pivot_pin",
        name="support arm pivot barrel is retained on pin",
    )
    ctx.allow_overlap(
        frame,
        wheel_0,
        elem_a="axle_stub_0",
        elem_b="rim",
        reason="The transport wheel rim is intentionally captured on its axle stub so the wheel can spin.",
    )
    ctx.expect_overlap(
        wheel_0,
        frame,
        axes="y",
        min_overlap=0.035,
        elem_a="rim",
        elem_b="axle_stub_0",
        name="wheel 0 rim is retained across axle width",
    )
    ctx.allow_overlap(
        frame,
        wheel_1,
        elem_a="axle_stub_1",
        elem_b="rim",
        reason="The transport wheel rim is intentionally captured on its axle stub so the wheel can spin.",
    )
    ctx.expect_overlap(
        wheel_1,
        frame,
        axes="y",
        min_overlap=0.035,
        elem_a="rim",
        elem_b="axle_stub_1",
        name="wheel 1 rim is retained across axle width",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is not None:
        length = frame_aabb[1][0] - frame_aabb[0][0]
        width = frame_aabb[1][1] - frame_aabb[0][1]
        height = frame_aabb[1][2] - frame_aabb[0][2]
    else:
        length = width = height = 0.0
    ctx.check(
        "commercial gym-floor scale",
        1.55 <= length <= 1.95 and 0.78 <= width <= 0.95 and 0.45 <= height <= 0.55,
        details=f"frame extents length={length:.3f}, width={width:.3f}, height={height:.3f}",
    )

    ctx.expect_gap(
        backrest,
        seat,
        axis="x",
        min_gap=0.045,
        max_gap=0.120,
        positive_elem="pad",
        negative_elem="pad",
        name="split pads have visible gap",
    )
    ctx.expect_gap(
        backrest,
        support,
        axis="z",
        min_gap=0.002,
        max_gap=0.040,
        positive_elem="backrest_plate",
        negative_elem="top_roller",
        name="support arm sits below separate backrest plate",
    )
    ctx.expect_overlap(
        backrest,
        support,
        axes="xy",
        min_overlap=0.015,
        elem_a="backrest_plate",
        elem_b="top_roller",
        name="support arm aligns under backrest plate",
    )

    ctx.check(
        "transport wheel joints are continuous",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in ("wheel_0_spin", "wheel_1_spin")
        ),
    )

    backrest_rest = ctx.part_element_world_aabb(backrest, elem="pad")
    with ctx.pose({backrest_hinge: 1.0}):
        backrest_raised = ctx.part_element_world_aabb(backrest, elem="pad")
    ctx.check(
        "backrest hinge raises pad",
        backrest_rest is not None
        and backrest_raised is not None
        and backrest_raised[1][2] > backrest_rest[1][2] + 0.35,
        details=f"rest={backrest_rest}, raised={backrest_raised}",
    )

    seat_rest = ctx.part_element_world_aabb(seat, elem="pad")
    with ctx.pose({seat_hinge: 0.42}):
        seat_raised = ctx.part_element_world_aabb(seat, elem="pad")
    ctx.check(
        "seat hinge tips rear upward",
        seat_rest is not None
        and seat_raised is not None
        and seat_raised[1][2] > seat_rest[1][2] + 0.09,
        details=f"rest={seat_rest}, raised={seat_raised}",
    )

    support_rest = ctx.part_element_world_aabb(support, elem="top_roller")
    with ctx.pose({support_pivot: 0.45}):
        support_raised = ctx.part_element_world_aabb(support, elem="top_roller")
    ctx.check(
        "rear support arm rotates upward",
        support_rest is not None
        and support_raised is not None
        and support_raised[1][2] > support_rest[1][2] + 0.06,
        details=f"rest={support_rest}, raised={support_raised}",
    )

    ctx.expect_overlap(
        wheel_0,
        frame,
        axes="xz",
        min_overlap=0.018,
        elem_a="rim",
        elem_b="axle_stub_0",
        name="wheel 0 is captured by axle mount",
    )
    ctx.expect_overlap(
        wheel_1,
        frame,
        axes="xz",
        min_overlap=0.018,
        elem_a="rim",
        elem_b="axle_stub_1",
        name="wheel 1 is captured by axle mount",
    )

    return ctx.report()


object_model = build_object_model()
