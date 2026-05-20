from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
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
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _bar_between(part, name, start, end, thickness, material):
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = -math.asin(dz / length)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _rounded_box_mesh(size, radius, name):
    sx, sy, sz = size
    shape = cq.Workplane("XY").box(sx, sy, sz).edges().fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.001, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_adjustable_bench")

    steel = model.material("black_powder_coat", rgba=(0.015, 0.016, 0.017, 1.0))
    vinyl = model.material("charcoal_vinyl", rgba=(0.055, 0.055, 0.050, 1.0))
    seam = model.material("stitched_seam", rgba=(0.18, 0.18, 0.16, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    metal = model.material("brushed_hardware", rgba=(0.62, 0.60, 0.55, 1.0))
    red = model.material("red_pop_pin", rgba=(0.82, 0.035, 0.025, 1.0))

    frame = model.part("main_frame")
    # Welded commercial frame: a central spine, wide floor stabilizers, hinge
    # towers, and rear wheel brackets all deliberately intersect as welded tube.
    frame.visual(
        Box((1.72, 0.075, 0.075)),
        origin=Origin(xyz=(-0.08, 0.0, 0.205)),
        material=steel,
        name="center_spine",
    )
    frame.visual(
        Box((0.15, 0.94, 0.07)),
        origin=Origin(xyz=(-0.90, 0.0, 0.115)),
        material=steel,
        name="rear_floor_tube",
    )
    frame.visual(
        Box((0.14, 0.62, 0.065)),
        origin=Origin(xyz=(0.66, 0.0, 0.115)),
        material=steel,
        name="front_floor_tube",
    )
    _bar_between(frame, "rear_down_strut", (-0.82, 0.0, 0.235), (-0.90, 0.0, 0.115), 0.06, steel)
    _bar_between(frame, "front_down_strut", (0.53, 0.0, 0.235), (0.66, 0.0, 0.115), 0.055, steel)
    _bar_between(frame, "back_hinge_tower_0", (-0.28, -0.18, 0.235), (-0.28, -0.18, 0.58), 0.052, steel)
    _bar_between(frame, "back_hinge_tower_1", (-0.28, 0.18, 0.235), (-0.28, 0.18, 0.58), 0.052, steel)
    frame.visual(
        Box((0.18, 0.43, 0.050)),
        origin=Origin(xyz=(-0.28, 0.0, 0.255)),
        material=steel,
        name="back_hinge_base_yoke",
    )
    frame.visual(
        Box((0.060, 0.33, 0.052)),
        origin=Origin(xyz=(-0.28, 0.0, 0.555)),
        material=steel,
        name="back_hinge_crossmember",
    )
    frame.visual(
        Cylinder(radius=0.031, length=0.72),
        origin=Origin(xyz=(-0.28, 0.0, 0.58), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="back_hinge_pin",
    )
    frame.visual(
        Box((0.13, 0.09, 0.035)),
        origin=Origin(xyz=(-0.28, -0.36, 0.58)),
        material=steel,
        name="back_hinge_lug_0",
    )
    frame.visual(
        Box((0.13, 0.09, 0.035)),
        origin=Origin(xyz=(-0.28, 0.36, 0.58)),
        material=steel,
        name="back_hinge_lug_1",
    )
    _bar_between(frame, "seat_hinge_tower_0", (0.35, -0.15, 0.235), (0.35, -0.15, 0.53), 0.047, steel)
    _bar_between(frame, "seat_hinge_tower_1", (0.35, 0.15, 0.235), (0.35, 0.15, 0.53), 0.047, steel)
    frame.visual(
        Box((0.18, 0.37, 0.048)),
        origin=Origin(xyz=(0.35, 0.0, 0.255)),
        material=steel,
        name="seat_hinge_base_yoke",
    )
    frame.visual(
        Box((0.055, 0.34, 0.047)),
        origin=Origin(xyz=(0.35, 0.0, 0.505)),
        material=steel,
        name="seat_hinge_crossmember",
    )
    frame.visual(
        Cylinder(radius=0.027, length=0.46),
        origin=Origin(xyz=(0.35, 0.0, 0.53), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="seat_hinge_pin",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.36),
        origin=Origin(xyz=(-0.63, 0.0, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="support_pivot_pin",
    )
    frame.visual(
        Box((0.16, 0.43, 0.040)),
        origin=Origin(xyz=(-0.63, 0.0, 0.235)),
        material=steel,
        name="support_pivot_base_yoke",
    )
    _bar_between(frame, "support_pivot_stand_0", (-0.63, -0.19, 0.235), (-0.63, -0.19, 0.30), 0.05, steel)
    _bar_between(frame, "support_pivot_stand_1", (-0.63, 0.19, 0.235), (-0.63, 0.19, 0.30), 0.05, steel)
    # Rear transport-wheel fork mounts, welded to the rear stabilizer.
    for suffix, y in (("0", -0.47), ("1", 0.47)):
        frame.visual(
            Box((0.035, 0.050, 0.155)),
            origin=Origin(xyz=(-0.93, y, 0.145)),
            material=steel,
            name=f"wheel_fork_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.016, length=0.075),
            origin=Origin(xyz=(-0.93, y + (0.035 if y > 0.0 else -0.035), 0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"wheel_axle_stub_{suffix}",
        )

    backrest = model.part("backrest")
    backrest.visual(
        _rounded_box_mesh((0.98, 0.49, 0.085), 0.025, "back_pad"),
        origin=Origin(xyz=(-0.50, 0.0, 0.070)),
        material=vinyl,
        name="back_pad",
    )
    backrest.visual(
        Box((0.88, 0.040, 0.010)),
        origin=Origin(xyz=(-0.51, -0.205, 0.108)),
        material=seam,
        name="back_side_seam_0",
    )
    backrest.visual(
        Box((0.88, 0.040, 0.010)),
        origin=Origin(xyz=(-0.51, 0.205, 0.108)),
        material=seam,
        name="back_side_seam_1",
    )
    backrest.visual(
        Box((0.82, 0.46, 0.045)),
        origin=Origin(xyz=(-0.47, 0.0, 0.006)),
        material=steel,
        name="back_plate",
    )
    backrest.visual(
        Box((0.70, 0.075, 0.038)),
        origin=Origin(xyz=(-0.50, 0.0, -0.035)),
        material=steel,
        name="angle_selector_rail",
    )
    for i, x in enumerate((-0.76, -0.64, -0.52, -0.40, -0.28)):
        backrest.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(x, -0.035, -0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"selector_hole_{i}",
        )
    for suffix, y in (("0", -0.260), ("1", 0.260)):
        backrest.visual(
            Cylinder(radius=0.034, length=0.105),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"back_hinge_barrel_{suffix}",
        )
        backrest.visual(
            Box((0.13, 0.070, 0.070)),
            origin=Origin(xyz=(-0.055, y, -0.015)),
            material=steel,
            name=f"back_hinge_tab_{suffix}",
        )

    seat = model.part("seat")
    seat.visual(
        _rounded_box_mesh((0.54, 0.47, 0.082), 0.022, "seat_pad"),
        origin=Origin(xyz=(-0.28, 0.0, 0.060)),
        material=vinyl,
        name="seat_pad",
    )
    seat.visual(
        Box((0.47, 0.44, 0.038)),
        origin=Origin(xyz=(-0.27, 0.0, -0.001)),
        material=steel,
        name="seat_plate",
    )
    seat.visual(
        Box((0.035, 0.37, 0.012)),
        origin=Origin(xyz=(-0.03, 0.0, 0.099)),
        material=seam,
        name="front_seam",
    )
    for suffix, y in (("0", -0.230), ("1", 0.230)):
        seat.visual(
            Cylinder(radius=0.029, length=0.105),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"seat_hinge_barrel_{suffix}",
        )
        seat.visual(
            Box((0.11, 0.065, 0.065)),
            origin=Origin(xyz=(-0.052, y, -0.010)),
            material=steel,
            name=f"seat_hinge_tab_{suffix}",
        )

    support_arm = model.part("rear_support_arm")
    support_arm.visual(
        Box((0.68, 0.24, 0.055)),
        origin=Origin(xyz=(-0.34, 0.0, 0.0)),
        material=steel,
        name="wide_support_plate",
    )
    support_arm.visual(
        Box((0.58, 0.030, 0.025)),
        origin=Origin(xyz=(-0.36, -0.122, 0.035)),
        material=steel,
        name="side_rib_0",
    )
    support_arm.visual(
        Box((0.58, 0.030, 0.025)),
        origin=Origin(xyz=(-0.36, 0.122, 0.035)),
        material=steel,
        name="side_rib_1",
    )
    support_arm.visual(
        Cylinder(radius=0.035, length=0.29),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="lower_pivot_barrel",
    )
    support_arm.visual(
        Cylinder(radius=0.025, length=0.22),
        origin=Origin(xyz=(-0.66, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="upper_roller",
    )
    support_arm.visual(
        Box((0.090, 0.052, 0.070)),
        origin=Origin(xyz=(-0.43, -0.125, 0.005)),
        material=steel,
        name="pop_pin_boss",
    )

    selector_knob = model.part("selector_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.062,
            0.035,
            body_style="lobed",
            base_diameter=0.045,
            top_diameter=0.058,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=8, depth=0.002),
            bore=KnobBore(style="round", diameter=0.012),
        ),
        "pop_pin_knob",
    )
    selector_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="knob_cap",
    )
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.085),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pin_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="spring_collar",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.050,
            0.030,
            rim=WheelRim(inner_radius=0.031, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.016,
                width=0.020,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.021, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=4, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "transport_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.066,
            0.034,
            inner_radius=0.049,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.03),
            shoulder=TireShoulder(width=0.004, radius=0.0015),
        ),
        "transport_tire",
    )
    for i in range(2):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(wheel_mesh, material=metal, name="wheel_hub")
        wheel.visual(tire_mesh, material=rubber, name="tire")

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.28, 0.0, 0.58), rpy=(0.0, 0.42, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=1.0, lower=-0.42, upper=0.78),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.35, 0.0, 0.53), rpy=(0.0, 0.08, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=-0.08, upper=0.35),
    )
    model.articulation(
        "frame_to_rear_support_arm",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=support_arm,
        origin=Origin(xyz=(-0.63, 0.0, 0.30), rpy=(0.0, 0.35, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.30, upper=0.55),
    )
    model.articulation(
        "support_arm_to_selector_knob",
        ArticulationType.PRISMATIC,
        parent=support_arm,
        child=selector_knob,
        origin=Origin(xyz=(-0.43, -0.157, 0.010)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.20, lower=0.0, upper=0.035),
    )
    for i, y in enumerate((-0.52, 0.52)):
        model.articulation(
            f"frame_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=f"wheel_{i}",
            origin=Origin(xyz=(-0.93, y, 0.115), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("main_frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    support_arm = object_model.get_part("rear_support_arm")
    knob = object_model.get_part("selector_knob")
    back_joint = object_model.get_articulation("frame_to_backrest")
    seat_joint = object_model.get_articulation("frame_to_seat")
    support_joint = object_model.get_articulation("frame_to_rear_support_arm")
    knob_joint = object_model.get_articulation("support_arm_to_selector_knob")

    # Hinge pins, barrels, and welded boss seats are intentionally coaxial or
    # inserted. Keep the allowances scoped to those named mechanism elements.
    for elem in ("back_hinge_barrel_0", "back_hinge_barrel_1"):
        ctx.allow_overlap(
            frame,
            backrest,
            elem_a="back_hinge_pin",
            elem_b=elem,
            reason="The backrest hinge barrels are modeled around the transverse hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            backrest,
            axes="yz",
            elem_a="back_hinge_pin",
            elem_b=elem,
            min_overlap=0.02,
            name=f"{elem} captures the back hinge pin",
        )
    for elem in ("back_hinge_tab_0", "back_hinge_tab_1"):
        ctx.allow_overlap(
            frame,
            backrest,
            elem_a="back_hinge_pin",
            elem_b=elem,
            reason="The backrest hinge tabs are pierced by the same transverse hinge pin as the barrels.",
        )
        ctx.expect_overlap(
            frame,
            backrest,
            axes="yz",
            elem_a="back_hinge_pin",
            elem_b=elem,
            min_overlap=0.015,
            name=f"{elem} is pinned to the back hinge",
        )
    for elem in ("seat_hinge_barrel_0", "seat_hinge_barrel_1"):
        ctx.allow_overlap(
            frame,
            seat,
            elem_a="seat_hinge_pin",
            elem_b=elem,
            reason="The seat hinge barrels are modeled around the forward hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            seat,
            axes="yz",
            elem_a="seat_hinge_pin",
            elem_b=elem,
            min_overlap=0.018,
            name=f"{elem} captures the seat hinge pin",
        )
    for elem in ("seat_hinge_tab_0", "seat_hinge_tab_1"):
        ctx.allow_overlap(
            frame,
            seat,
            elem_a="seat_hinge_pin",
            elem_b=elem,
            reason="The seat hinge tabs are pierced by the forward hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            seat,
            axes="yz",
            elem_a="seat_hinge_pin",
            elem_b=elem,
            min_overlap=0.015,
            name=f"{elem} is pinned to the seat hinge",
        )
    ctx.allow_overlap(
        frame,
        support_arm,
        elem_a="support_pivot_pin",
        elem_b="lower_pivot_barrel",
        reason="The wide rear support arm rotates around the lower frame pivot pin.",
    )
    ctx.expect_overlap(
        frame,
        support_arm,
        axes="yz",
        elem_a="support_pivot_pin",
        elem_b="lower_pivot_barrel",
        min_overlap=0.02,
        name="support arm captures the lower pivot pin",
    )
    ctx.allow_overlap(
        frame,
        support_arm,
        elem_a="support_pivot_pin",
        elem_b="wide_support_plate",
        reason="The lower pivot pin passes through the wide welded support arm plate.",
    )
    ctx.expect_overlap(
        frame,
        support_arm,
        axes="yz",
        elem_a="support_pivot_pin",
        elem_b="wide_support_plate",
        min_overlap=0.020,
        name="support pivot pin passes through wide plate",
    )
    ctx.allow_overlap(
        support_arm,
        knob,
        elem_a="pop_pin_boss",
        elem_b="pin_shaft",
        reason="The selector pin shaft is intentionally inserted through the welded support-arm boss.",
    )
    ctx.expect_overlap(
        support_arm,
        knob,
        axes="y",
        elem_a="pop_pin_boss",
        elem_b="pin_shaft",
        min_overlap=0.015,
        name="selector pin remains inserted in boss",
    )
    ctx.allow_overlap(
        support_arm,
        knob,
        elem_a="wide_support_plate",
        elem_b="pin_shaft",
        reason="The pop-pin shaft continues through the support arm plate behind the welded boss.",
    )
    ctx.expect_overlap(
        support_arm,
        knob,
        axes="y",
        elem_a="wide_support_plate",
        elem_b="pin_shaft",
        min_overlap=0.010,
        name="selector pin passes through support arm plate",
    )
    ctx.expect_gap(
        support_arm,
        knob,
        axis="y",
        positive_elem="wide_support_plate",
        negative_elem="knob_cap",
        min_gap=0.010,
        name="selector knob cap is visibly separate from support arm",
    )
    ctx.expect_gap(
        backrest,
        support_arm,
        axis="z",
        positive_elem="back_plate",
        negative_elem="wide_support_plate",
        min_gap=0.025,
        name="support arm is separate below the backrest plate",
    )

    for i in range(2):
        wheel = object_model.get_part(f"wheel_{i}")
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=f"wheel_axle_stub_{i}",
            elem_b="wheel_hub",
            reason="The transport wheel hub is captured on the small axle stub so it can spin.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="yz",
            elem_a=f"wheel_axle_stub_{i}",
            elem_b="wheel_hub",
            min_overlap=0.010,
            name=f"wheel_{i} hub stays on axle",
        )

    def _elem_max_z(part, elem):
        box = ctx.part_element_world_aabb(part, elem=elem)
        return None if box is None else box[1][2]

    rest_back = _elem_max_z(backrest, "back_pad")
    with ctx.pose({back_joint: 0.55}):
        raised_back = _elem_max_z(backrest, "back_pad")
    ctx.check(
        "backrest pivots upward about transverse hinge",
        rest_back is not None and raised_back is not None and raised_back > rest_back + 0.10,
        details=f"rest={rest_back}, raised={raised_back}",
    )

    rest_seat = _elem_max_z(seat, "seat_pad")
    with ctx.pose({seat_joint: 0.28}):
        raised_seat = _elem_max_z(seat, "seat_pad")
    ctx.check(
        "seat pivots upward about forward hinge",
        rest_seat is not None and raised_seat is not None and raised_seat > rest_seat + 0.030,
        details=f"rest={rest_seat}, raised={raised_seat}",
    )

    rest_arm = _elem_max_z(support_arm, "upper_roller")
    with ctx.pose({support_joint: 0.30}):
        raised_arm = _elem_max_z(support_arm, "upper_roller")
    ctx.check(
        "rear support arm rotates on lower frame pivot",
        rest_arm is not None and raised_arm is not None and abs(raised_arm - rest_arm) > 0.035,
        details=f"rest={rest_arm}, raised={raised_arm}",
    )

    rest_knob = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: 0.030}):
        pulled_knob = ctx.part_world_position(knob)
    ctx.check(
        "pop pin knob pulls outward along local short axis",
        rest_knob is not None and pulled_knob is not None and pulled_knob[1] < rest_knob[1] - 0.020,
        details=f"rest={rest_knob}, pulled={pulled_knob}",
    )

    return ctx.report()


object_model = build_object_model()
