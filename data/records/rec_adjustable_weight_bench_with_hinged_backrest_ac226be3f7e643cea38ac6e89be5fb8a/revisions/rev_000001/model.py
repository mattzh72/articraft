from __future__ import annotations

from math import acos, atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_adjustable_weight_bench")

    steel = model.material("satin_black_steel", rgba=(0.025, 0.027, 0.030, 1.0))
    hinge_steel = model.material("brushed_pivot_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    vinyl = model.material("black_textured_vinyl", rgba=(0.015, 0.014, 0.013, 1.0))
    seam = model.material("slightly_raised_pad_seams", rgba=(0.05, 0.05, 0.055, 1.0))
    foam = model.material("dark_gray_foam", rgba=(0.08, 0.085, 0.09, 1.0))
    tire_rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    wheel_plastic = model.material("dark_wheel_hub", rgba=(0.12, 0.12, 0.13, 1.0))

    def cyl_x(part, name, center, length, radius, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_y(part, name, center, length, radius, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(-pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_z(part, name, center, length, radius, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center),
            material=material,
            name=name,
        )

    def cylinder_between(part, name, p0, p1, radius, material):
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dz = p1[2] - p0[2]
        length = sqrt(dx * dx + dy * dy + dz * dz)
        pitch = acos(dz / length)
        yaw = atan2(dy, dx)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0),
                rpy=(0.0, pitch, yaw),
            ),
            material=material,
            name=name,
        )

    transport_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.058,
            0.038,
            inner_radius=0.043,
            tread=TireTread(style="ribbed", depth=0.003, count=18, land_ratio=0.62),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "transport_wheel_tire",
    )
    transport_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.043,
            0.034,
            rim=WheelRim(inner_radius=0.030, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.018,
                width=0.026,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.022, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.022),
        ),
        "transport_wheel_hub",
    )
    roller_mesh = mesh_from_geometry(
        TireGeometry(
            0.062,
            0.180,
            inner_radius=0.012,
            tread=TireTread(style="smooth", depth=0.001, count=1, land_ratio=1.0),
            sidewall=TireSidewall(style="rounded", bulge=0.08),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        "front_foam_roller",
    )

    frame = model.part("center_frame")
    cyl_x(frame, "main_spine", (0.00, 0.00, 0.140), 1.38, 0.018, steel)
    cyl_x(frame, "side_rail_0", (-0.02, -0.18, 0.120), 1.18, 0.016, steel)
    cyl_x(frame, "side_rail_1", (-0.02, 0.18, 0.120), 1.18, 0.016, steel)
    for idx, x in enumerate((-0.55, -0.45, -0.10, -0.04, 0.25, 0.55)):
        cyl_y(frame, f"cross_tube_{idx}", (x, 0.0, 0.120), 0.43, 0.016, steel)

    # Rear stabilizer foot and an adjustment rack for the back support link.
    cyl_y(frame, "rear_floor_foot", (-0.66, 0.0, 0.075), 0.56, 0.020, steel)
    cyl_x(frame, "rear_foot_strut_0", (-0.605, -0.18, 0.090), 0.13, 0.014, steel)
    cyl_x(frame, "rear_foot_strut_1", (-0.605, 0.18, 0.090), 0.13, 0.014, steel)
    cyl_z(frame, "rack_post_0", (-0.60, 0.0, 0.190), 0.140, 0.012, steel)
    cyl_z(frame, "rack_post_1", (-0.26, 0.0, 0.190), 0.140, 0.012, steel)
    frame.visual(
        Box((0.40, 0.045, 0.024)),
        origin=Origin(xyz=(-0.43, 0.0, 0.258)),
        material=steel,
        name="incline_rack",
    )
    for idx, x in enumerate((-0.56, -0.49, -0.42, -0.35, -0.28)):
        frame.visual(
            Box((0.026, 0.050, 0.026)),
            origin=Origin(xyz=(x, 0.0, 0.283), rpy=(0.0, -0.35, 0.0)),
            material=hinge_steel,
            name=f"rack_tooth_{idx}",
        )

    # Hinge towers and side cheeks for the split backrest and seat pad.
    for idx, y in enumerate((-0.185, 0.185)):
        cyl_z(frame, f"back_hinge_post_{idx}", (-0.10, y, 0.275), 0.310, 0.014, steel)
        frame.visual(
            Box((0.058, 0.035, 0.090)),
            origin=Origin(xyz=(-0.10, y, 0.430)),
            material=steel,
            name=f"back_hinge_cheek_{idx}",
        )
        cyl_z(frame, f"seat_hinge_post_{idx}", (-0.04, y, 0.270), 0.300, 0.014, steel)
        frame.visual(
            Box((0.054, 0.035, 0.080)),
            origin=Origin(xyz=(-0.04, y, 0.420)),
            material=steel,
            name=f"seat_hinge_cheek_{idx}",
        )
    cyl_y(frame, "back_hinge_axle", (-0.10, 0.0, 0.430), 0.420, 0.011, hinge_steel)
    cyl_y(frame, "seat_hinge_axle", (-0.04, 0.0, 0.420), 0.390, 0.010, hinge_steel)

    # Lower pivot for the folding rear support link.
    for idx, y in enumerate((-0.082, 0.082)):
        cyl_z(frame, f"rear_link_pivot_post_{idx}", (-0.45, y, 0.160), 0.100, 0.011, steel)
        frame.visual(
            Box((0.060, 0.025, 0.070)),
            origin=Origin(xyz=(-0.45, y, 0.205)),
            material=steel,
            name=f"rear_link_pivot_cheek_{idx}",
        )
    cyl_y(frame, "rear_link_pivot_axle", (-0.45, 0.0, 0.205), 0.190, 0.011, hinge_steel)

    # Front transport wheel axle and the rigid yoke that carries the foam rollers.
    cyl_x(frame, "front_axle_arm_0", (0.635, -0.18, 0.100), 0.25, 0.012, steel)
    cyl_x(frame, "front_axle_arm_1", (0.635, 0.18, 0.100), 0.25, 0.012, steel)
    cyl_y(frame, "transport_axle", (0.735, 0.0, 0.095), 0.66, 0.013, hinge_steel)
    cyl_z(frame, "front_yoke_upright", (0.62, 0.0, 0.230), 0.230, 0.016, steel)
    cyl_x(frame, "front_yoke_neck", (0.725, 0.0, 0.342), 0.230, 0.015, steel)
    frame.visual(
        Box((0.045, 0.100, 0.125)),
        origin=Origin(xyz=(0.835, 0.0, 0.305)),
        material=steel,
        name="front_yoke_plate",
    )
    cyl_y(frame, "roller_axle", (0.835, 0.0, 0.342), 0.54, 0.012, hinge_steel)

    backrest = model.part("backrest")
    cyl_y(backrest, "hinge_barrel", (0.0, 0.0, 0.0), 0.255, 0.024, hinge_steel)
    backrest.visual(
        Box((0.720, 0.310, 0.075)),
        origin=Origin(xyz=(-0.380, 0.0, 0.065)),
        material=vinyl,
        name="pad",
    )
    backrest.visual(
        Box((0.725, 0.318, 0.010)),
        origin=Origin(xyz=(-0.380, 0.0, 0.107)),
        material=seam,
        name="top_seam_panel",
    )
    for idx, y in enumerate((-0.100, 0.100)):
        backrest.visual(
            Box((0.570, 0.024, 0.028)),
            origin=Origin(xyz=(-0.425, y, 0.015)),
            material=steel,
            name=f"support_rail_{idx}",
        )
    backrest.visual(
        Box((0.070, 0.250, 0.030)),
        origin=Origin(xyz=(-0.105, 0.0, 0.015)),
        material=steel,
        name="hinge_cross_brace",
    )
    backrest.visual(
        Box((0.100, 0.220, 0.024)),
        origin=Origin(xyz=(-0.040, 0.0, 0.034)),
        material=steel,
        name="barrel_mount_tab",
    )

    seat = model.part("seat")
    cyl_y(seat, "hinge_barrel", (0.0, 0.0, 0.0), 0.235, 0.021, hinge_steel)
    seat.visual(
        Box((0.420, 0.310, 0.070)),
        origin=Origin(xyz=(0.230, 0.0, 0.060)),
        material=vinyl,
        name="pad",
    )
    seat.visual(
        Box((0.425, 0.318, 0.009)),
        origin=Origin(xyz=(0.230, 0.0, 0.099)),
        material=seam,
        name="top_seam_panel",
    )
    for idx, y in enumerate((-0.092, 0.092)):
        seat.visual(
            Box((0.390, 0.023, 0.026)),
            origin=Origin(xyz=(0.205, y, 0.012)),
            material=steel,
            name=f"support_rail_{idx}",
        )
    seat.visual(
        Box((0.055, 0.240, 0.028)),
        origin=Origin(xyz=(0.070, 0.0, 0.012)),
        material=steel,
        name="hinge_cross_brace",
    )

    rear_link = model.part("rear_link")
    cyl_y(rear_link, "lower_bushing", (0.0, 0.0, 0.0), 0.110, 0.022, hinge_steel)
    top_pin = (-0.220, 0.0, 0.230)
    for idx, y in enumerate((-0.035, 0.035)):
        cylinder_between(rear_link, f"side_strap_{idx}", (-0.020, y, 0.015), (top_pin[0], y, top_pin[2]), 0.010, steel)
        rear_link.visual(
            Box((0.038, 0.026, 0.014)),
            origin=Origin(xyz=(-0.020, y, 0.026)),
            material=steel,
            name=f"lower_weld_tab_{idx}",
        )
    cyl_y(rear_link, "upper_roller_pin", top_pin, 0.115, 0.017, hinge_steel)

    for idx, y in enumerate((-0.290, 0.290)):
        wheel = model.part(f"transport_wheel_{idx}")
        wheel.visual(transport_tire_mesh, material=tire_rubber, name="rubber_tire")
        wheel.visual(transport_wheel_mesh, material=wheel_plastic, name="spoked_hub")
        model.articulation(
            f"frame_to_transport_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(0.735, y, 0.095), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=30.0),
        )

    for idx, y in enumerate((-0.145, 0.145)):
        roller = model.part(f"front_roller_{idx}")
        roller.visual(roller_mesh, material=foam, name="foam_sleeve")
        model.articulation(
            f"frame_to_front_roller_{idx}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=roller,
            origin=Origin(xyz=(0.835, y, 0.342), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    model.articulation(
        "frame_to_backrest",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=backrest,
        origin=Origin(xyz=(-0.10, 0.0, 0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=0.0, upper=1.10),
    )
    model.articulation(
        "frame_to_seat",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(-0.04, 0.0, 0.420)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=0.38),
    )
    model.articulation(
        "frame_to_rear_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_link,
        origin=Origin(xyz=(-0.45, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.35, upper=0.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("center_frame")
    backrest = object_model.get_part("backrest")
    seat = object_model.get_part("seat")
    rear_link = object_model.get_part("rear_link")
    back_joint = object_model.get_articulation("frame_to_backrest")
    seat_joint = object_model.get_articulation("frame_to_seat")
    link_joint = object_model.get_articulation("frame_to_rear_link")

    ctx.allow_overlap(
        backrest,
        frame,
        elem_a="hinge_barrel",
        elem_b="back_hinge_axle",
        reason="The backrest hinge barrel is intentionally captured around the transverse steel axle.",
    )
    ctx.expect_overlap(
        backrest,
        frame,
        axes="y",
        min_overlap=0.20,
        elem_a="hinge_barrel",
        elem_b="back_hinge_axle",
        name="backrest hinge barrel is retained on its axle",
    )
    ctx.allow_overlap(
        seat,
        frame,
        elem_a="hinge_barrel",
        elem_b="seat_hinge_axle",
        reason="The seat hinge barrel is intentionally represented as a sleeve around its hinge axle.",
    )
    ctx.expect_overlap(
        seat,
        frame,
        axes="y",
        min_overlap=0.18,
        elem_a="hinge_barrel",
        elem_b="seat_hinge_axle",
        name="seat hinge barrel is retained on its axle",
    )
    ctx.allow_overlap(
        rear_link,
        frame,
        elem_a="lower_bushing",
        elem_b="rear_link_pivot_axle",
        reason="The folding support link lower bushing is captured on the lower frame pivot axle.",
    )
    ctx.expect_overlap(
        rear_link,
        frame,
        axes="y",
        min_overlap=0.09,
        elem_a="lower_bushing",
        elem_b="rear_link_pivot_axle",
        name="rear support link bushing is retained on its pivot axle",
    )

    for idx in (0, 1):
        wheel = object_model.get_part(f"transport_wheel_{idx}")
        ctx.allow_overlap(
            wheel,
            frame,
            elem_a="spoked_hub",
            elem_b="transport_axle",
            reason="The transport wheel hub bore is intentionally represented around the wheel axle.",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="y",
            min_overlap=0.025,
            elem_a="spoked_hub",
            elem_b="transport_axle",
            name=f"transport wheel {idx} hub is retained on the axle",
        )

        roller = object_model.get_part(f"front_roller_{idx}")
        ctx.allow_overlap(
            roller,
            frame,
            elem_a="foam_sleeve",
            elem_b="roller_axle",
            reason="The foam foot roller sleeve is intentionally captured around the shared support axle.",
        )
        ctx.expect_overlap(
            roller,
            frame,
            axes="y",
            min_overlap=0.15,
            elem_a="foam_sleeve",
            elem_b="roller_axle",
            name=f"front roller {idx} is retained on the shared axle",
        )

    ctx.check(
        "primary mechanisms are articulated",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "frame_to_backrest",
                "frame_to_seat",
                "frame_to_rear_link",
                "frame_to_transport_wheel_0",
                "frame_to_transport_wheel_1",
                "frame_to_front_roller_0",
                "frame_to_front_roller_1",
            )
        ),
        details="Missing one of the requested pad, support-link, wheel, or roller joints.",
    )

    ctx.expect_gap(
        backrest,
        frame,
        axis="z",
        min_gap=0.22,
        positive_elem="pad",
        negative_elem="main_spine",
        name="back pad sits above the low frame",
    )
    ctx.expect_gap(
        seat,
        frame,
        axis="z",
        min_gap=0.21,
        positive_elem="pad",
        negative_elem="main_spine",
        name="seat pad sits above the low frame",
    )
    ctx.expect_gap(
        seat,
        backrest,
        axis="x",
        min_gap=0.040,
        max_gap=0.140,
        positive_elem="pad",
        negative_elem="pad",
        name="split pads retain a visible gap",
    )
    ctx.expect_overlap(
        rear_link,
        frame,
        axes="xy",
        min_overlap=0.040,
        elem_a="lower_bushing",
        elem_b="incline_rack",
        name="rear support link is aligned with the adjustment rack",
    )

    rest_back_aabb = ctx.part_element_world_aabb(backrest, elem="pad")
    with ctx.pose({back_joint: 1.0}):
        raised_back_aabb = ctx.part_element_world_aabb(backrest, elem="pad")
    ctx.check(
        "backrest hinge raises the back pad",
        rest_back_aabb is not None
        and raised_back_aabb is not None
        and raised_back_aabb[1][2] > rest_back_aabb[1][2] + 0.28,
        details=f"rest={rest_back_aabb}, raised={raised_back_aabb}",
    )

    rest_seat_aabb = ctx.part_element_world_aabb(seat, elem="pad")
    with ctx.pose({seat_joint: 0.35}):
        tilted_seat_aabb = ctx.part_element_world_aabb(seat, elem="pad")
    ctx.check(
        "seat hinge tips the front of the seat upward",
        rest_seat_aabb is not None
        and tilted_seat_aabb is not None
        and tilted_seat_aabb[1][2] > rest_seat_aabb[1][2] + 0.09,
        details=f"rest={rest_seat_aabb}, tilted={tilted_seat_aabb}",
    )

    rest_link_aabb = ctx.part_world_aabb(rear_link)
    with ctx.pose({link_joint: 0.75}):
        raised_link_aabb = ctx.part_world_aabb(rear_link)
    ctx.check(
        "rear support link swings upward from its lower pivot",
        rest_link_aabb is not None
        and raised_link_aabb is not None
        and raised_link_aabb[1][2] > rest_link_aabb[1][2] + 0.045,
        details=f"rest={rest_link_aabb}, raised={raised_link_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
