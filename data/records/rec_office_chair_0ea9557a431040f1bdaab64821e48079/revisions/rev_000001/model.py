from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
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
    superellipse_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mesh_back_office_chair")

    black = model.material("mat_black_fabric", rgba=(0.02, 0.022, 0.024, 1.0))
    mesh = model.material("mat_translucent_mesh", rgba=(0.02, 0.025, 0.026, 0.58))
    charcoal = model.material("mat_charcoal_frame", rgba=(0.08, 0.085, 0.09, 1.0))
    dark_plastic = model.material("mat_dark_plastic", rgba=(0.015, 0.015, 0.017, 1.0))
    rubber = model.material("mat_rubber", rgba=(0.01, 0.01, 0.011, 1.0))
    chrome = model.material("mat_satin_chrome", rgba=(0.62, 0.64, 0.62, 1.0))

    base = model.part("base")

    # Five-star base, central gas-lift pedestal, and the fixed lower half of
    # the under-seat tilt bracket.
    base.visual(
        Cylinder(radius=0.075, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=charcoal,
        name="center_hub",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=chrome,
        name="gas_lift",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=charcoal,
        name="pedestal_collar",
    )
    base.visual(
        Box((0.220, 0.180, 0.025)),
        origin=Origin(xyz=(0.050, 0.0, 0.5125)),
        material=charcoal,
        name="top_plate",
    )
    base.visual(
        Box((0.100, 0.320, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
        material=charcoal,
        name="cross_member",
    )
    for side, y in enumerate((-0.142, 0.142)):
        base.visual(
            Box((0.130, 0.044, 0.025)),
            origin=Origin(xyz=(-0.050, y, 0.515)),
            material=charcoal,
            name=f"clevis_rib_{side}",
        )
        base.visual(
            Box((0.065, 0.044, 0.075)),
            origin=Origin(xyz=(-0.100, y, 0.540)),
            material=charcoal,
            name=f"tilt_clevis_{side}",
        )

    caster_radius = 0.055
    caster_width = 0.044
    wheel_center_z = caster_radius + 0.005
    leg_radius = 0.420
    wheel_radius_from_center = 0.455

    for i in range(5):
        theta = math.radians(90.0 + i * 72.0)
        c, s = math.cos(theta), math.sin(theta)
        radial = (c, s, 0.0)
        tangent = (-s, c, 0.0)

        leg = tube_from_spline_points(
            [
                (0.0, 0.0, 0.120),
                (0.155 * c, 0.155 * s, 0.125),
                (leg_radius * c, leg_radius * s, 0.145),
            ],
            radius=0.024,
            samples_per_segment=10,
            radial_segments=18,
        )
        base.visual(
            mesh_from_geometry(leg, f"star_leg_{i}"),
            material=charcoal,
            name=f"star_leg_{i}",
        )

        wheel_center = (
            wheel_radius_from_center * radial[0],
            wheel_radius_from_center * radial[1],
            wheel_center_z,
        )
        yaw = theta
        prong_offset = caster_width / 2.0 + 0.016
        for side_sign in (-1.0, 1.0):
            base.visual(
                Box((0.050, 0.012, 0.090)),
                origin=Origin(
                    xyz=(
                        wheel_center[0] + side_sign * prong_offset * tangent[0],
                        wheel_center[1] + side_sign * prong_offset * tangent[1],
                        0.095,
                    ),
                    rpy=(0.0, 0.0, yaw),
                ),
                material=charcoal,
                name=f"caster_fork_{i}_{0 if side_sign < 0 else 1}",
            )
        base.visual(
            Box((0.064, 0.082, 0.016)),
            origin=Origin(xyz=(wheel_center[0], wheel_center[1], 0.145), rpy=(0.0, 0.0, yaw)),
            material=charcoal,
            name=f"caster_bridge_{i}",
        )
        base.visual(
            Box((0.110, 0.046, 0.034)),
            origin=Origin(
                xyz=(0.410 * radial[0], 0.410 * radial[1], 0.145),
                rpy=(0.0, 0.0, yaw),
            ),
            material=charcoal,
            name=f"caster_socket_{i}",
        )
        base.visual(
            Cylinder(radius=0.013, length=0.070),
            origin=Origin(xyz=(wheel_center[0], wheel_center[1], 0.167)),
            material=charcoal,
            name=f"caster_stem_{i}",
        )
        base.visual(
            Cylinder(radius=0.005, length=0.076),
            origin=Origin(
                xyz=wheel_center,
                rpy=(0.0, math.pi / 2.0, theta + math.pi / 2.0),
            ),
            material=chrome,
            name=f"caster_axle_{i}",
        )

    seat_back = model.part("seat_back")

    seat_cushion = ExtrudeGeometry(
        superellipse_profile(0.540, 0.490, exponent=3.0, segments=64),
        0.080,
        center=True,
    )
    seat_back.visual(
        mesh_from_geometry(seat_cushion, "seat_cushion"),
        origin=Origin(xyz=(0.120, 0.0, 0.0875)),
        material=black,
        name="seat_cushion",
    )
    seat_back.visual(
        Box((0.480, 0.210, 0.035)),
        origin=Origin(xyz=(0.120, 0.0, 0.030)),
        material=dark_plastic,
        name="seat_pan",
    )
    seat_back.visual(
        Cylinder(radius=0.025, length=0.240),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    seat_back.visual(
        Box((0.205, 0.230, 0.030)),
        origin=Origin(xyz=(-0.010, 0.0, 0.012)),
        material=charcoal,
        name="tilt_carriage",
    )

    back_frame = tube_from_spline_points(
        [
            (-0.170, -0.245, 0.200),
            (-0.255, -0.270, 0.500),
            (-0.342, -0.225, 0.810),
            (-0.360, 0.000, 0.860),
            (-0.342, 0.225, 0.810),
            (-0.255, 0.270, 0.500),
            (-0.170, 0.245, 0.200),
        ],
        radius=0.017,
        samples_per_segment=14,
        radial_segments=20,
        closed_spline=True,
    )
    seat_back.visual(
        mesh_from_geometry(back_frame, "back_frame"),
        material=charcoal,
        name="back_frame",
    )
    for side, y in enumerate((-0.180, 0.180)):
        support = tube_from_spline_points(
            [
                (-0.060, y, 0.030),
                (-0.120, y, 0.115),
                (-0.174, y * 1.20, 0.212),
            ],
            radius=0.016,
            samples_per_segment=10,
            radial_segments=18,
        )
        seat_back.visual(
            mesh_from_geometry(support, f"rear_support_{side}"),
            material=charcoal,
            name=f"rear_support_{side}",
        )

    back_mesh = PerforatedPanelGeometry(
        (0.435, 0.585),
        0.006,
        hole_diameter=0.014,
        pitch=(0.028, 0.030),
        frame=0.014,
        corner_radius=0.040,
        stagger=True,
    )
    recline = 0.18
    seat_back.visual(
        mesh_from_geometry(back_mesh, "back_mesh"),
        origin=Origin(
            xyz=(-0.265, 0.0, 0.510),
            rpy=(math.pi / 2.0 - recline, 0.0, math.pi / 2.0),
        ),
        material=mesh,
        name="back_mesh",
    )
    for side, y in enumerate((-0.245, 0.245)):
        seat_back.visual(
            Box((0.065, 0.075, 0.052)),
            origin=Origin(xyz=(-0.255, y, 0.505)),
            material=charcoal,
            name=f"mesh_clip_{side}",
        )

    # Paired armrests are rigid with the tilting seat/back shell.  Numbered
    # names avoid imposing a handedness convention on the symmetric chair.
    for side, y in enumerate((-0.315, 0.315)):
        seat_back.visual(
            Box((0.260, 0.220, 0.036)),
            origin=Origin(xyz=(0.170, y * 0.665, 0.047)),
            material=charcoal,
            name=f"arm_bracket_{side}",
        )
        seat_back.visual(
            Box((0.335, 0.066, 0.048)),
            origin=Origin(xyz=(0.112, y, 0.270)),
            material=black,
            name=f"arm_pad_{side}",
        )
        for post, x in enumerate((0.075, 0.235)):
            seat_back.visual(
                Box((0.038, 0.038, 0.230)),
                origin=Origin(xyz=(x, y, 0.145)),
                material=charcoal,
                name=f"arm_post_{side}_{post}",
            )

    tilt = model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=seat_back,
        origin=Origin(xyz=(-0.100, 0.0, 0.540)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.4, lower=0.0, upper=math.radians(15.0)),
    )
    tilt.meta["description"] = "simple under-seat tilt axis; positive motion reclines the seat-back assembly"

    for i in range(5):
        theta = math.radians(90.0 + i * 72.0)
        wheel_center = (
            wheel_radius_from_center * math.cos(theta),
            wheel_radius_from_center * math.sin(theta),
            wheel_center_z,
        )
        caster = model.part(f"caster_wheel_{i}")
        wheel_core = WheelGeometry(
            0.039,
            caster_width * 0.72,
            rim=WheelRim(inner_radius=0.025, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(radius=0.017, width=0.026, cap_style="domed"),
            face=WheelFace(dish_depth=0.002, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.008),
        )
        tire = TireGeometry(
            caster_radius,
            caster_width,
            inner_radius=0.038,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.06),
            tread=TireTread(style="ribbed", depth=0.0035, count=18, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        )
        caster.visual(mesh_from_geometry(wheel_core, f"caster_core_{i}"), material=chrome, name="wheel_core")
        caster.visual(mesh_from_geometry(tire, f"caster_tire_{i}"), material=rubber, name="tire")
        model.articulation(
            f"caster_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(xyz=wheel_center, rpy=(0.0, 0.0, theta + math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    seat_back = object_model.get_part("seat_back")
    tilt = object_model.get_articulation("tilt")
    caster_joints = [object_model.get_articulation(f"caster_spin_{i}") for i in range(5)]
    caster_parts = [object_model.get_part(f"caster_wheel_{i}") for i in range(5)]

    for i, caster in enumerate(caster_parts):
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"caster_axle_{i}",
            elem_b="wheel_core",
            reason="The fixed caster axle intentionally passes through the wheel hub bore.",
        )
        ctx.expect_contact(
            base,
            caster,
            elem_a=f"caster_axle_{i}",
            elem_b="wheel_core",
            contact_tol=0.001,
            name=f"caster axle {i} is captured in wheel hub",
        )

    ctx.check(
        "five continuous caster axles",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in caster_joints),
        details=str([j.articulation_type for j in caster_joints]),
    )
    ctx.check(
        "tilt limit is fifteen degrees",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower == 0.0
        and abs(tilt.motion_limits.upper - math.radians(15.0)) < 1e-6,
        details=str(tilt.motion_limits),
    )
    ctx.expect_gap(
        seat_back,
        base,
        axis="z",
        positive_elem="seat_pan",
        negative_elem="top_plate",
        min_gap=0.020,
        name="seat pan sits above pedestal top plate",
    )
    ctx.expect_gap(
        base,
        seat_back,
        axis="y",
        positive_elem="tilt_clevis_1",
        negative_elem="hinge_barrel",
        min_gap=0.0,
        max_gap=0.002,
        name="tilt clevis captures hinge barrel side",
    )

    closed_back = ctx.part_element_world_aabb(seat_back, elem="back_frame")
    with ctx.pose({tilt: math.radians(15.0)}):
        reclined_back = ctx.part_element_world_aabb(seat_back, elem="back_frame")
    ctx.check(
        "tilt reclines back frame rearward",
        closed_back is not None
        and reclined_back is not None
        and reclined_back[0][0] < closed_back[0][0] - 0.045
        and reclined_back[1][2] < closed_back[1][2] - 0.030,
        details=f"closed={closed_back}, reclined={reclined_back}",
    )

    return ctx.report()


object_model = build_object_model()
