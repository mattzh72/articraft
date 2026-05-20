from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireTread,
    TestContext,
    TestReport,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    ExtrudeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _tube_pose(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("tube endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5), rpy=(0.0, pitch, yaw)), length


def _add_tube(part, name: str, start, end, radius: float, material: Material) -> None:
    origin, length = _tube_pose(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _rounded_pad_mesh(name: str, length: float, width: float, thickness: float, radius: float):
    profile = rounded_rect_profile(length, width, radius, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry(profile, thickness, center=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    steel = Material("satin_black_steel", color=(0.03, 0.035, 0.035, 1.0))
    dark_steel = Material("dark_powdercoat", color=(0.01, 0.012, 0.012, 1.0))
    rubber = Material("matte_black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    vinyl = Material("black_grained_vinyl", color=(0.02, 0.022, 0.024, 1.0))
    seam = Material("slightly_gloss_seam", color=(0.0, 0.0, 0.0, 1.0))
    metal = Material("brushed_pin_metal", color=(0.65, 0.66, 0.63, 1.0))
    red = Material("red_pop_pin_knob", color=(0.85, 0.04, 0.025, 1.0))

    # Root assembly: a compact low tubular home-gym frame with fixed brackets,
    # front transport axle, hinge axles, and the adjustment ladder under the back pad.
    frame = model.part("center_frame")
    rail_z = 0.075
    for y, suffix in ((-0.19, "0"), (0.19, "1")):
        _add_tube(frame, f"base_rail_{suffix}", (-0.68, y, rail_z), (0.61, y, rail_z), 0.022, steel)
    _add_tube(frame, "rear_cross_tube", (-0.65, -0.23, rail_z), (-0.65, 0.23, rail_z), 0.023, steel)
    _add_tube(frame, "front_cross_tube", (0.57, -0.23, rail_z), (0.57, 0.23, rail_z), 0.023, steel)
    _add_tube(frame, "center_spine", (-0.50, 0.0, 0.12), (0.43, 0.0, 0.255), 0.023, steel)
    _add_tube(frame, "rear_mast", (-0.44, 0.0, rail_z), (-0.125, 0.0, 0.365), 0.022, steel)
    _add_tube(frame, "front_mast", (0.40, 0.0, rail_z), (-0.035, 0.0, 0.395), 0.022, steel)
    _add_tube(frame, "rear_mast_crossbar", (-0.44, -0.19, rail_z), (-0.44, 0.19, rail_z), 0.018, steel)
    _add_tube(frame, "front_mast_crossbar", (0.40, -0.19, rail_z), (0.40, 0.19, rail_z), 0.018, steel)
    _add_tube(frame, "mid_cross_tube", (-0.18, -0.19, 0.315), (-0.18, 0.19, 0.315), 0.018, steel)
    frame.visual(
        Box((0.090, 0.430, 0.045)),
        origin=Origin(xyz=(-0.50, 0.0, 0.098)),
        material=dark_steel,
        name="spine_saddle_plate",
    )

    # Hinge and pivot hardware carried by the fixed frame.
    _add_tube(frame, "back_hinge_axle", (-0.09, -0.235, 0.43), (-0.09, 0.235, 0.43), 0.011, metal)
    _add_tube(frame, "seat_hinge_axle", (-0.035, -0.215, 0.43), (-0.035, 0.215, 0.43), 0.010, metal)
    _add_tube(frame, "support_pivot_axle", (-0.64, -0.13, 0.130), (-0.64, 0.13, 0.130), 0.010, metal)
    _add_tube(frame, "front_wheel_axle", (0.69, -0.325, 0.085), (0.69, 0.325, 0.085), 0.009, metal)
    for y, suffix in ((-0.238, "0"), (0.238, "1")):
        _add_tube(frame, f"back_hinge_side_support_{suffix}", (-0.18, y * 0.80, 0.315), (-0.09, y, 0.430), 0.012, steel)
    for y, suffix in ((-0.218, "0"), (0.218, "1")):
        _add_tube(frame, f"seat_hinge_side_support_{suffix}", (0.03, y * 0.88, 0.330), (-0.035, y, 0.430), 0.012, steel)
    _add_tube(frame, "wheel_axle_bracket", (0.57, 0.0, rail_z), (0.69, 0.0, 0.085), 0.011, steel)
    _add_tube(frame, "seat_support_crossbar", (0.03, -0.192, 0.330), (0.03, 0.192, 0.330), 0.014, steel)
    for y, suffix in ((-0.13, "0"), (0.13, "1")):
        frame.visual(
            Box((0.060, 0.018, 0.080)),
            origin=Origin(xyz=(-0.64, y, 0.098)),
            material=steel,
            name=f"support_pivot_lug_{suffix}",
        )

    # Rubber foot pads are small fixed caps on the cross tubes.
    for x, tag in ((-0.65, "rear"), (0.57, "front")):
        for y, side in ((-0.245, "0"), (0.245, "1")):
            frame.visual(
                Box((0.075, 0.035, 0.024)),
                origin=Origin(xyz=(x, y, 0.040)),
                material=rubber,
                name=f"{tag}_foot_{side}",
            )

    # The angle selector ladder is a slotted-looking fixed bar under the backrest.
    _add_tube(frame, "selector_ladder", (-0.50, -0.165, 0.215), (-0.18, -0.165, 0.36), 0.014, dark_steel)
    for i, (x, z) in enumerate(((-0.45, 0.238), (-0.37, 0.274), (-0.29, 0.310), (-0.21, 0.346))):
        frame.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(xyz=(x, -0.165, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"selector_hole_{i}",
        )
    frame.visual(
        Box((0.060, 0.045, 0.050)),
        origin=Origin(xyz=(-0.18, -0.168, 0.336)),
        material=dark_steel,
        name="selector_mount_tab",
    )

    # Backrest: independent pad, underside support bracket, hinge sleeve, and the
    # side pop-pin socket.  Its local frame is on the transverse hinge line.
    backrest = model.part("backrest")
    backrest.visual(
        _rounded_pad_mesh("backrest_pad_mesh", 0.78, 0.31, 0.075, 0.045),
        origin=Origin(xyz=(-0.405, 0.0, 0.060)),
        material=vinyl,
        name="back_pad",
    )
    backrest.visual(
        Box((0.70, 0.014, 0.008)),
        origin=Origin(xyz=(-0.405, 0.118, 0.0995)),
        material=seam,
        name="back_seam_0",
    )
    backrest.visual(
        Box((0.70, 0.014, 0.008)),
        origin=Origin(xyz=(-0.405, -0.118, 0.0995)),
        material=seam,
        name="back_seam_1",
    )
    for y, suffix in ((-0.105, "0"), (0.105, "1")):
        _add_tube(backrest, f"back_under_rail_{suffix}", (-0.745, y, 0.012), (-0.025, y, 0.012), 0.014, steel)
    _add_tube(backrest, "back_cross_bracket", (-0.52, -0.125, 0.012), (-0.52, 0.125, 0.012), 0.012, steel)
    _add_tube(backrest, "back_hinge_sleeve", (0.0, -0.205, 0.0), (0.0, 0.205, 0.0), 0.018, steel)
    for y, suffix in ((-0.105, "0"), (0.105, "1")):
        backrest.visual(
            Box((0.036, 0.034, 0.020)),
            origin=Origin(xyz=(-0.026, y, 0.025)),
            material=steel,
            name=f"back_hinge_tab_{suffix}",
        )
    backrest.visual(
        Box((0.28, 0.014, 0.055)),
        origin=Origin(xyz=(-0.28, -0.168, -0.020)),
        material=dark_steel,
        name="pop_pin_plate",
    )
    _add_tube(backrest, "pop_pin_standoff", (-0.28, -0.105, -0.010), (-0.28, -0.168, -0.010), 0.010, steel)
    backrest.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(-0.28, -0.178, -0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pop_pin_boss",
    )

    # Seat pad: a separate short pivoting pad on its own hinge and support bracket.
    seat = model.part("seat_pad")
    seat.visual(
        _rounded_pad_mesh("seat_pad_mesh", 0.46, 0.31, 0.075, 0.045),
        origin=Origin(xyz=(0.245, 0.0, 0.058)),
        material=vinyl,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.37, 0.014, 0.008)),
        origin=Origin(xyz=(0.255, 0.118, 0.0995)),
        material=seam,
        name="seat_seam_0",
    )
    seat.visual(
        Box((0.37, 0.014, 0.008)),
        origin=Origin(xyz=(0.255, -0.118, 0.0995)),
        material=seam,
        name="seat_seam_1",
    )
    for y, suffix in ((-0.100, "0"), (0.100, "1")):
        _add_tube(seat, f"seat_under_rail_{suffix}", (0.025, y, 0.012), (0.435, y, 0.012), 0.014, steel)
    _add_tube(seat, "seat_cross_bracket", (0.25, -0.12, 0.012), (0.25, 0.12, 0.012), 0.012, steel)
    _add_tube(seat, "seat_hinge_sleeve", (0.0, -0.185, 0.0), (0.0, 0.185, 0.0), 0.017, steel)
    for y, suffix in ((-0.100, "0"), (0.100, "1")):
        seat.visual(
            Box((0.040, 0.032, 0.020)),
            origin=Origin(xyz=(0.020, y, 0.022)),
            material=steel,
            name=f"seat_hinge_tab_{suffix}",
        )

    # Rear support link: a pivoting pair of steel straps/tubes tied by pivot barrels.
    rear_support = model.part("rear_support")
    _add_tube(rear_support, "lower_bushing", (0.0, -0.115, 0.0), (0.0, 0.115, 0.0), 0.018, steel)
    for y, suffix in ((-0.075, "0"), (0.075, "1")):
        rear_support.visual(
            Box((0.032, 0.020, 0.018)),
            origin=Origin(xyz=(0.020, y, 0.020)),
            material=steel,
            name=f"lower_link_tab_{suffix}",
        )
        _add_tube(rear_support, f"side_link_{suffix}", (0.018, y, 0.018), (0.245, y, 0.245), 0.013, steel)
    _add_tube(rear_support, "upper_roller", (0.245, -0.105, 0.245), (0.245, 0.105, 0.245), 0.017, metal)

    # Transport wheels, each a spinning wheel-and-tire part on the fixed axle.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.066,
            0.044,
            inner_radius=0.043,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="ribbed", depth=0.003, count=18, land_ratio=0.62),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "front_transport_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.043,
            0.042,
            rim=WheelRim(inner_radius=0.029, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.018,
                width=0.034,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.023, hole_diameter=0.003),
            ),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.022),
        ),
        "front_transport_wheel",
    )
    for y, suffix in ((-0.285, "0"), (0.285, "1")):
        wheel = model.part(f"front_wheel_{suffix}")
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=rubber, name="tire")
        wheel.visual(wheel_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=metal, name="rim")
        wheel.visual(
            Cylinder(radius=0.013, length=0.052),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="axle_bushing",
        )
        model.articulation(
            f"frame_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.69, y, 0.085)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=18.0),
        )

    # Separate pop-pin knob and shaft, translating outward along the local side axis.
    pop_pin = model.part("pop_pin")
    pop_pin.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.0, -0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pin_shaft",
    )
    pop_pin.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.026,
                body_style="lobed",
                top_diameter=0.036,
                grip=KnobGrip(style="ribbed", count=8, depth=0.002),
            ),
            "pop_pin_knob_mesh",
        ),
        origin=Origin(xyz=(0.0, -0.066, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="knob_cap",
    )

    back_hinge = model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.09, 0.0, 0.43)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.4, lower=0.0, upper=1.20),
    )
    seat_hinge = model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.035, 0.0, 0.43)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.2, lower=0.0, upper=0.45),
    )
    support_pivot = model.articulation(
        "frame_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_support,
        origin=Origin(xyz=(-0.64, 0.0, 0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=-0.30, upper=0.85),
    )
    pin_slide = model.articulation(
        "backrest_to_pop_pin",
        ArticulationType.PRISMATIC,
        parent=backrest,
        child=pop_pin,
        origin=Origin(xyz=(-0.28, -0.182, -0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.035),
    )

    # Keep handles for tests without changing the authored joint semantics.
    model.meta["primary_joints"] = (
        back_hinge.name,
        seat_hinge.name,
        support_pivot.name,
        pin_slide.name,
        "frame_to_wheel_0",
        "frame_to_wheel_1",
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("center_frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat_pad")
    support = object_model.get_part("rear_support")
    pop_pin = object_model.get_part("pop_pin")
    wheel_0 = object_model.get_part("front_wheel_0")
    wheel_1 = object_model.get_part("front_wheel_1")

    back_hinge = object_model.get_articulation("frame_to_backrest")
    seat_hinge = object_model.get_articulation("frame_to_seat")
    support_pivot = object_model.get_articulation("frame_to_rear_support")
    pin_slide = object_model.get_articulation("backrest_to_pop_pin")
    wheel_joint_0 = object_model.get_articulation("frame_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("frame_to_wheel_1")

    # Captured metal pins and hinge sleeves intentionally share the same physical
    # pivot volumes.  The scoped allowances correspond to real bearing/axle fits.
    ctx.allow_overlap(
        frame,
        backrest,
        elem_a="back_hinge_axle",
        elem_b="back_hinge_sleeve",
        reason="The backrest hinge sleeve is modeled around the fixed transverse hinge axle.",
    )
    ctx.expect_overlap(
        frame,
        backrest,
        axes="y",
        elem_a="back_hinge_axle",
        elem_b="back_hinge_sleeve",
        min_overlap=0.20,
        name="backrest hinge sleeve spans the transverse axle",
    )
    ctx.allow_overlap(
        frame,
        seat,
        elem_a="seat_hinge_axle",
        elem_b="seat_hinge_sleeve",
        reason="The seat hinge sleeve is modeled around its own fixed hinge axle.",
    )
    ctx.expect_overlap(
        frame,
        seat,
        axes="y",
        elem_a="seat_hinge_axle",
        elem_b="seat_hinge_sleeve",
        min_overlap=0.18,
        name="seat hinge sleeve spans the transverse axle",
    )
    ctx.allow_overlap(
        frame,
        support,
        elem_a="support_pivot_axle",
        elem_b="lower_bushing",
        reason="The rear support link lower bushing rotates around the fixed lower pivot axle.",
    )
    ctx.expect_overlap(
        frame,
        support,
        axes="y",
        elem_a="support_pivot_axle",
        elem_b="lower_bushing",
        min_overlap=0.11,
        name="rear support bushing is retained on lower pivot",
    )
    ctx.allow_overlap(
        backrest,
        pop_pin,
        elem_a="pop_pin_boss",
        elem_b="pin_shaft",
        reason="The pop-pin shaft passes through the spring boss before pulling outward.",
    )
    ctx.expect_overlap(
        backrest,
        pop_pin,
        axes="y",
        elem_a="pop_pin_boss",
        elem_b="pin_shaft",
        min_overlap=0.005,
        name="pop-pin shaft remains captured by the boss",
    )
    for wheel, suffix in ((wheel_0, "0"), (wheel_1, "1")):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="front_wheel_axle",
            elem_b="axle_bushing",
            reason=f"Transport wheel {suffix} spins around the fixed axle through its hub bushing.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="front_wheel_axle",
            elem_b="axle_bushing",
            min_overlap=0.020,
            name=f"wheel {suffix} bushing is retained on axle",
        )

    ctx.check(
        "front wheels are continuous spin joints",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.expect_gap(
        backrest,
        frame,
        axis="z",
        positive_elem="back_pad",
        negative_elem="back_hinge_axle",
        min_gap=0.002,
        max_gap=0.060,
        name="back pad sits above hinge frame",
    )
    ctx.expect_gap(
        seat,
        frame,
        axis="z",
        positive_elem="seat_cushion",
        negative_elem="seat_hinge_axle",
        min_gap=0.002,
        max_gap=0.060,
        name="seat pad sits above hinge frame",
    )

    rest_back_aabb = ctx.part_world_aabb(backrest)
    with ctx.pose({back_hinge: 0.90}):
        raised_back_aabb = ctx.part_world_aabb(backrest)
    ctx.check(
        "backrest raises at incline setting",
        rest_back_aabb is not None
        and raised_back_aabb is not None
        and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.20,
        details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
    )

    rest_seat_aabb = ctx.part_world_aabb(seat)
    with ctx.pose({seat_hinge: 0.40}):
        raised_seat_aabb = ctx.part_world_aabb(seat)
    ctx.check(
        "seat pad pivots upward on front edge",
        rest_seat_aabb is not None
        and raised_seat_aabb is not None
        and raised_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.04,
        details=f"rest={rest_seat_aabb}, raised={raised_seat_aabb}",
    )

    rest_support = ctx.part_element_world_aabb(support, elem="upper_roller")
    with ctx.pose({support_pivot: 0.55}):
        moved_support = ctx.part_element_world_aabb(support, elem="upper_roller")
    ctx.check(
        "rear support link swings on lower pivot",
        rest_support is not None
        and moved_support is not None
        and abs(moved_support[0][0] - rest_support[0][0]) > 0.035,
        details=f"rest={rest_support}, moved={moved_support}",
    )

    rest_pin = ctx.part_world_position(pop_pin)
    with ctx.pose({pin_slide: 0.030}):
        pulled_pin = ctx.part_world_position(pop_pin)
    ctx.check(
        "pop-pin knob pulls outward along side axis",
        rest_pin is not None and pulled_pin is not None and pulled_pin[1] < rest_pin[1] - 0.020,
        details=f"rest={rest_pin}, pulled={pulled_pin}",
    )

    ctx.expect_overlap(wheel_0, frame, axes="z", elem_a="tire", elem_b="front_wheel_axle", min_overlap=0.005, name="wheel 0 centered on axle height")
    ctx.expect_overlap(wheel_1, frame, axes="z", elem_a="tire", elem_b="front_wheel_axle", min_overlap=0.005, name="wheel 1 centered on axle height")

    return ctx.report()


object_model = build_object_model()
