from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _rounded_rect_loop(length: float, width: float, z: float, radius: float, segments: int = 5):
    """Return a closed rounded-rectangle loop in the local XY plane."""
    half_x = length * 0.5
    half_y = width * 0.5
    radius = min(radius, half_x * 0.45, half_y * 0.45)
    centers = (
        (half_x - radius, half_y - radius, 0.0, math.pi / 2.0),
        (-half_x + radius, half_y - radius, math.pi / 2.0, math.pi),
        (-half_x + radius, -half_y + radius, math.pi, 3.0 * math.pi / 2.0),
        (half_x - radius, -half_y + radius, 3.0 * math.pi / 2.0, 2.0 * math.pi),
    )
    points = []
    for cx, cy, start, end in centers:
        for index in range(segments + 1):
            if points and index == 0:
                continue
            t = index / segments
            angle = start + (end - start) * t
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle), z))
    return points


def _cushion_mesh(name: str, *, length: float, width: float, height: float):
    """Chamfered, softly rounded vinyl pad with a subtly crowned top."""
    lower = _rounded_rect_loop(length * 0.98, width * 0.98, 0.0, 0.035)
    shoulder = _rounded_rect_loop(length, width, height * 0.35, 0.045)
    top = _rounded_rect_loop(length * 0.96, width * 0.94, height, 0.060)
    return mesh_from_geometry(LoftGeometry([lower, shoulder, top], cap=True, closed=True), name)


def _beam_xz(part, start, end, *, width: float, thickness: float, material, name: str):
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dz = ez - sz
    length = math.hypot(dx, dz)
    pitch = math.atan2(-dz, dx)
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_decline_bench")

    frame_finish = model.material("black_powder_coat", rgba=(0.035, 0.038, 0.042, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_pin = model.material("brushed_pin", rgba=(0.62, 0.64, 0.66, 1.0))
    vinyl = model.material("black_vinyl", rgba=(0.025, 0.026, 0.028, 1.0))
    seam_gray = model.material("vinyl_seam_gray", rgba=(0.30, 0.31, 0.32, 1.0))
    foam = model.material("dense_foam", rgba=(0.045, 0.047, 0.050, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.020, 0.021, 0.023, 1.0))
    wheel_plastic = model.material("wheel_plastic_gray", rgba=(0.42, 0.43, 0.45, 1.0))
    hole_black = model.material("selector_hole_black", rgba=(0.0, 0.0, 0.0, 1.0))

    frame = model.part("frame")

    # Low, full-scale tube frame with transport wheels at the rear end.
    _beam_xz(
        frame,
        (-0.84, 0.0, 0.155),
        (0.70, 0.0, 0.155),
        width=0.060,
        thickness=0.055,
        material=frame_finish,
        name="main_spine",
    )
    frame.visual(
        Box((0.11, 0.56, 0.060)),
        origin=Origin(xyz=(0.68, 0.0, 0.035)),
        material=frame_finish,
        name="front_floor_bar",
    )
    frame.visual(
        Box((0.12, 0.38, 0.055)),
        origin=Origin(xyz=(-0.80, 0.0, 0.040)),
        material=frame_finish,
        name="rear_floor_bar",
    )
    _beam_xz(
        frame,
        (0.58, 0.0, 0.155),
        (0.68, 0.0, 0.055),
        width=0.060,
        thickness=0.050,
        material=frame_finish,
        name="front_down_leg",
    )
    _beam_xz(
        frame,
        (-0.76, 0.0, 0.055),
        (-0.64, 0.0, 0.155),
        width=0.060,
        thickness=0.050,
        material=frame_finish,
        name="rear_rake_leg",
    )
    _beam_xz(
        frame,
        (-0.82, 0.0, 0.065),
        (-0.90, 0.0, 0.080),
        width=0.052,
        thickness=0.040,
        material=frame_finish,
        name="wheel_axle_yoke",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.54),
        origin=Origin(xyz=(-0.90, 0.0, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="transport_axle",
    )

    # Pad hinge pedestal and front roller outrigger.
    frame.visual(
        Box((0.060, 0.075, 0.350)),
        origin=Origin(xyz=(-0.055, 0.0, 0.325)),
        material=frame_finish,
        name="back_hinge_mast",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.340),
        origin=Origin(xyz=(-0.055, 0.0, 0.500), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="back_hinge_pin",
    )
    frame.visual(
        Box((0.050, 0.070, 0.330)),
        origin=Origin(xyz=(0.085, 0.0, 0.320)),
        material=frame_finish,
        name="seat_hinge_mast",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.320),
        origin=Origin(xyz=(0.085, 0.0, 0.485), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="seat_hinge_pin",
    )
    _beam_xz(
        frame,
        (0.55, 0.0, 0.155),
        (0.80, 0.0, 0.405),
        width=0.065,
        thickness=0.050,
        material=frame_finish,
        name="front_roller_strut",
    )
    frame.visual(
        Box((0.055, 0.070, 0.130)),
        origin=Origin(xyz=(0.80, 0.0, 0.405)),
        material=frame_finish,
        name="roller_upright",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(0.80, 0.0, 0.455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="roller_axle_center",
    )

    # Pop-pin selector plate under the backrest, with visible indexing holes.
    frame.visual(
        Box((0.055, 0.070, 0.125)),
        origin=Origin(xyz=(-0.58, -0.030, 0.205)),
        material=frame_finish,
        name="selector_base_tab",
    )
    _beam_xz(
        frame,
        (-0.58, -0.055, 0.230),
        (-0.18, -0.055, 0.430),
        width=0.036,
        thickness=0.070,
        material=dark_steel,
        name="support_arm",
    )
    for index, t in enumerate((0.18, 0.36, 0.54, 0.72)):
        x = -0.58 + 0.40 * t
        z = 0.230 + 0.200 * t
        frame.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(x, -0.075, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hole_black,
            name=f"selector_hole_{index}",
        )

    # Long back pad hinged on the central horizontal pivot.
    back_pad = model.part("back_pad")
    back_pad.visual(
        _cushion_mesh("long_back_pad_cushion", length=0.96, width=0.35, height=0.078),
        origin=Origin(xyz=(-0.515, 0.0, 0.058)),
        material=vinyl,
        name="cushion",
    )
    back_pad.visual(
        Box((0.82, 0.018, 0.010)),
        origin=Origin(xyz=(-0.50, -0.115, 0.135)),
        material=seam_gray,
        name="seam_0",
    )
    back_pad.visual(
        Box((0.82, 0.018, 0.004)),
        origin=Origin(xyz=(-0.50, 0.115, 0.135)),
        material=seam_gray,
        name="seam_1",
    )
    back_pad.visual(
        Box((0.86, 0.055, 0.026)),
        origin=Origin(xyz=(-0.48, 0.0, 0.045)),
        material=frame_finish,
        name="underside_rail",
    )
    for index, y in enumerate((-0.105, 0.105)):
        back_pad.visual(
            Box((0.070, 0.170, 0.040)),
            origin=Origin(xyz=(-0.040, y, 0.045)),
            material=dark_steel,
            name=f"hinge_side_plate_{index}",
        )
    for index, y in enumerate((-0.2025, 0.2025)):
        back_pad.visual(
            Cylinder(radius=0.025, length=0.065),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"hinge_barrel_{index}",
        )
        back_pad.visual(
            Box((0.080, 0.032, 0.090)),
            origin=Origin(xyz=(-0.035, y, 0.040)),
            material=dark_steel,
            name=f"hinge_web_{index}",
        )

    # Smaller seat pad hinged ahead of the back pad.
    seat = model.part("seat")
    seat.visual(
        _cushion_mesh("articulated_seat_cushion", length=0.46, width=0.34, height=0.075),
        origin=Origin(xyz=(0.255, 0.0, 0.058)),
        material=vinyl,
        name="cushion",
    )
    seat.visual(
        Box((0.36, 0.052, 0.024)),
        origin=Origin(xyz=(0.25, 0.0, 0.046)),
        material=frame_finish,
        name="underside_plate",
    )
    for index, y in enumerate((-0.100, 0.100)):
        seat.visual(
            Box((0.070, 0.160, 0.035)),
            origin=Origin(xyz=(0.055, y, 0.042)),
            material=dark_steel,
            name=f"hinge_side_plate_{index}",
        )
    for index, y in enumerate((-0.190, 0.190)):
        seat.visual(
            Cylinder(radius=0.023, length=0.060),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"hinge_barrel_{index}",
        )
        seat.visual(
            Box((0.080, 0.030, 0.080)),
            origin=Origin(xyz=(0.038, y, 0.035)),
            material=dark_steel,
            name=f"hinge_web_{index}",
        )

    # Front leg rollers: two independent foam pads on the same cross-axis.
    for index, y in enumerate((-0.1625, 0.1625)):
        roller = model.part(f"roller_{index}")
        roller.visual(
            Cylinder(radius=0.072, length=0.205),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=foam,
            name="foam_roll",
        )
        roller.visual(
            Cylinder(radius=0.024, length=0.216),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_pin,
            name="end_cap_core",
        )
        model.articulation(
            f"roller_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=roller,
            origin=Origin(xyz=(0.80, y, 0.455)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=12.0),
        )

    # Low transport wheels at the rear. WheelGeometry spins about its local X axis.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.050,
            0.034,
            rim=WheelRim(inner_radius=0.030, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.017,
                width=0.026,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.022, hole_diameter=0.0028),
            ),
            face=WheelFace(dish_depth=0.002, front_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "transport_wheel_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.070,
            0.044,
            inner_radius=0.052,
            tread=TireTread(style="circumferential", depth=0.0025, count=3),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
        ),
        "transport_wheel_tire",
    )
    for index, y in enumerate((-0.235, 0.235)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(wheel_mesh, material=wheel_plastic, name="hub")
        wheel.visual(tire_mesh, material=wheel_rubber, name="tire")
        model.articulation(
            f"wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.90, y, 0.080), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=18.0),
        )

    # Separate pop-pin selector knob sliding outward from the support arm.
    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.0075, length=0.080),
        origin=Origin(xyz=(0.0, -0.040, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="pin_shaft",
    )
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.026,
                body_style="lobed",
                top_diameter=0.042,
                crown_radius=0.002,
                bore=KnobBore(style="round", diameter=0.010),
            ),
            "selector_lobed_knob",
        ),
        origin=Origin(xyz=(0.0, -0.092, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=frame_finish,
        name="knob_cap",
    )
    selector_knob.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_pin,
        name="collar",
    )

    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(-0.055, 0.0, 0.500)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.085, 0.0, 0.485)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=0.42),
    )
    model.articulation(
        "selector_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=selector_knob,
        origin=Origin(xyz=(-0.365, -0.077, 0.338)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.035),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    back_pad = object_model.get_part("back_pad")
    seat = object_model.get_part("seat")
    selector_knob = object_model.get_part("selector_knob")
    back_hinge = object_model.get_articulation("back_hinge")
    seat_hinge = object_model.get_articulation("seat_hinge")
    selector_slide = object_model.get_articulation("selector_slide")

    for name in ("roller_spin_0", "roller_spin_1", "wheel_spin_0", "wheel_spin_1"):
        joint = object_model.get_articulation(name)
        ctx.check(
            f"{name}_is_continuous",
            joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{name} should be a continuous spin joint.",
        )

    for wheel_name in ("wheel_0", "wheel_1"):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="transport_axle",
            elem_b="hub",
            reason="The transport axle is intentionally captured through the wheel hub bore.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="transport_axle",
            elem_b="hub",
            min_overlap=0.010,
            name=f"{wheel_name} retained on axle",
        )

    for roller_name in ("roller_0", "roller_1"):
        roller = object_model.get_part(roller_name)
        ctx.allow_overlap(
            frame,
            roller,
            elem_a="roller_axle_center",
            elem_b="end_cap_core",
            reason="The leg roller's metal core is intentionally captured on the support axle.",
        )
        ctx.expect_overlap(
            frame,
            roller,
            axes="y",
            elem_a="roller_axle_center",
            elem_b="end_cap_core",
            min_overlap=0.004,
            name=f"{roller_name} retained on support axle",
        )

    ctx.check(
        "back_pad_longer_than_seat",
        back_pad is not None
        and seat is not None
        and ctx.part_world_aabb(back_pad) is not None
        and ctx.part_world_aabb(seat) is not None
        and (ctx.part_world_aabb(back_pad)[1][0] - ctx.part_world_aabb(back_pad)[0][0])
        > 1.8 * (ctx.part_world_aabb(seat)[1][0] - ctx.part_world_aabb(seat)[0][0]),
        details="The back pad should be visibly long relative to the articulated seat.",
    )
    ctx.expect_gap(
        frame,
        selector_knob,
        axis="y",
        min_gap=0.030,
        positive_elem="support_arm",
        negative_elem="knob_cap",
        name="selector knob stands off from support arm",
    )
    ctx.expect_gap(
        frame,
        selector_knob,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem="support_arm",
        negative_elem="pin_shaft",
        name="selector pin seats at support arm face",
    )

    rest_back_aabb = ctx.part_world_aabb(back_pad)
    with ctx.pose({back_hinge: 0.35}):
        raised_back_aabb = ctx.part_world_aabb(back_pad)
    with ctx.pose({back_hinge: -0.35}):
        declined_back_aabb = ctx.part_world_aabb(back_pad)
    ctx.check(
        "back hinge raises and declines pad",
        rest_back_aabb is not None
        and raised_back_aabb is not None
        and declined_back_aabb is not None
        and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.12
        and declined_back_aabb[0][2] < rest_back_aabb[0][2] - 0.12,
        details=f"rest={rest_back_aabb}, raised={raised_back_aabb}, declined={declined_back_aabb}",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({seat_hinge: 0.42}):
        raised_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat hinge lifts front cushion",
        rest_seat_aabb is not None
        and raised_seat_aabb is not None
        and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.10,
        details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
    )

    rest_knob_pos = ctx.part_world_position(selector_knob)
    with ctx.pose({selector_slide: 0.035}):
        pulled_knob_pos = ctx.part_world_position(selector_knob)
    ctx.check(
        "selector knob pulls outward",
        rest_knob_pos is not None
        and pulled_knob_pos is not None
        and pulled_knob_pos[1] < rest_knob_pos[1] - 0.030,
        details=f"rest={rest_knob_pos}, pulled={pulled_knob_pos}",
    )

    return ctx.report()


object_model = build_object_model()
