from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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


REAR_WHEEL_RADIUS = 0.300
CASTER_WHEEL_RADIUS = 0.075


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    polished_aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.71, 0.72, 1.0))
    dark_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.06, 0.065, 0.070, 1.0))
    navy_fabric = model.material("navy_sling_fabric", rgba=(0.02, 0.09, 0.19, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.03, 0.03, 0.035, 1.0))

    frame = model.part("frame")

    def tube_x(name: str, xyz: tuple[float, float, float], length: float, radius: float) -> None:
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_aluminum,
            name=name,
        )

    def tube_y(name: str, xyz: tuple[float, float, float], length: float, radius: float) -> None:
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=polished_aluminum,
            name=name,
        )

    def tube_z(name: str, xyz: tuple[float, float, float], length: float, radius: float) -> None:
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz),
            material=polished_aluminum,
            name=name,
        )

    def tube_xz(
        name: str,
        p0: tuple[float, float, float],
        p1: tuple[float, float, float],
        radius: float,
    ) -> None:
        dx = p1[0] - p0[0]
        dz = p1[2] - p0[2]
        length = math.hypot(dx, dz)
        center = ((p0[0] + p1[0]) * 0.5, p0[1], (p0[2] + p1[2]) * 0.5)
        frame.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=(0.0, math.atan2(dx, dz), 0.0)),
            material=polished_aluminum,
            name=name,
        )

    # Fabric sling and back panel.
    frame.visual(
        Box((0.50, 0.48, 0.045)),
        origin=Origin(xyz=(0.07, 0.0, 0.475)),
        material=navy_fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.040, 0.50, 0.50)),
        origin=Origin(xyz=(-0.220, 0.0, 0.720)),
        material=navy_fabric,
        name="back_sling",
    )

    # Main tubular chair structure.
    tube_y("rear_axle", (-0.18, 0.0, 0.340), 0.660, 0.018)
    tube_y("front_crossbar", (0.330, 0.0, 0.240), 0.540, 0.014)
    tube_x("seat_front_rail", (0.070, 0.245, 0.445), 0.510, 0.014)
    tube_x("seat_rear_rail", (0.070, -0.245, 0.445), 0.510, 0.014)
    tube_x("lower_rail_0", (0.075, 0.245, 0.240), 0.510, 0.014)
    tube_x("lower_rail_1", (0.075, -0.245, 0.240), 0.510, 0.014)
    tube_z("rear_post_0", (-0.180, 0.245, 0.650), 0.620, 0.015)
    tube_z("rear_post_1", (-0.180, -0.245, 0.650), 0.620, 0.015)
    tube_z("front_post_0", (0.320, 0.245, 0.345), 0.220, 0.014)
    tube_z("front_post_1", (0.320, -0.245, 0.345), 0.220, 0.014)
    tube_y("back_top_rail", (-0.180, 0.0, 0.960), 0.510, 0.014)
    tube_x("push_handle_0", (-0.255, 0.245, 0.960), 0.150, 0.013)
    tube_x("push_handle_1", (-0.255, -0.245, 0.960), 0.150, 0.013)

    # Front caster sockets are static sleeves on the frame; the caster stems touch
    # their lower faces and swivel below them.
    tube_z("caster_socket_0", (0.330, 0.245, 0.260), 0.080, 0.018)
    tube_z("caster_socket_1", (0.330, -0.245, 0.260), 0.080, 0.018)

    # Arm pads and simple footrests.
    for i, y in enumerate((0.295, -0.295)):
        frame.visual(
            Box((0.350, 0.055, 0.040)),
            origin=Origin(xyz=(0.060, y, 0.670)),
            material=black_vinyl,
            name=f"arm_pad_{i}",
        )
        tube_z(f"arm_support_{i}_0", (-0.090, y, 0.565), 0.210, 0.010)
        tube_z(f"arm_support_{i}_1", (0.205, y, 0.555), 0.190, 0.010)
        rail_y = 0.245 if y > 0.0 else -0.245
        bridge_y = (rail_y + y) * 0.5
        tube_y(f"arm_bridge_{i}_0", (-0.090, bridge_y, 0.456), abs(y - rail_y), 0.010)
        tube_y(f"arm_bridge_{i}_1", (0.205, bridge_y, 0.456), abs(y - rail_y), 0.010)

    for i, y in enumerate((0.120, -0.120)):
        tube_xz(f"footrest_tube_{i}", (0.330, y, 0.240), (0.500, y, 0.128), 0.010)
        frame.visual(
            Box((0.055, 0.090, 0.040)),
            origin=Origin(xyz=(0.505, y, 0.128)),
            material=graphite,
            name=f"footplate_socket_{i}",
        )
        frame.visual(
            Box((0.205, 0.140, 0.025)),
            origin=Origin(xyz=(0.565, y, 0.115), rpy=(0.0, -0.12, 0.0)),
            material=graphite,
            name=f"footplate_{i}",
        )

    rear_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.255,
            0.042,
            rim=WheelRim(inner_radius=0.205, flange_height=0.009, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.042,
                width=0.055,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.050, hole_diameter=0.005),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.002),
            spokes=WheelSpokes(style="straight", count=18, thickness=0.0025, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "rear_spoked_wheel",
    )
    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            REAR_WHEEL_RADIUS,
            0.050,
            inner_radius=0.257,
            tread=TireTread(style="circumferential", depth=0.0025, count=4),
            grooves=(TireGroove(center_offset=0.0, width=0.005, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
        ),
        "rear_black_tire",
    )
    handrim_mesh = mesh_from_geometry(
        TireGeometry(
            0.262,
            0.012,
            inner_radius=0.250,
            sidewall=TireSidewall(style="rounded", bulge=0.02),
        ),
        "push_handrim",
    )

    for i, (y, handrim_offset) in enumerate(((0.390, 0.058), (-0.390, -0.058))):
        wheel = model.part(f"rear_wheel_{i}")
        wheel.visual(rear_wheel_mesh, material=polished_aluminum, name="spoked_wheel")
        wheel.visual(rear_tire_mesh, material=dark_rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.034, length=0.120),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_aluminum,
            name="hub",
        )
        wheel.visual(
            handrim_mesh,
            origin=Origin(xyz=(handrim_offset, 0.0, 0.0)),
            material=polished_aluminum,
            name="push_rim",
        )
        for stay in range(6):
            theta = stay * math.tau / 6.0
            wheel.visual(
                Cylinder(radius=0.0035, length=abs(handrim_offset)),
                origin=Origin(
                    xyz=(handrim_offset * 0.5, 0.252 * math.cos(theta), 0.252 * math.sin(theta)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=polished_aluminum,
                name=f"push_rim_stay_{stay}",
            )
        model.articulation(
            f"rear_wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.180, y, 0.340), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=12.0),
        )

    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.054,
            0.030,
            rim=WheelRim(inner_radius=0.038, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(radius=0.018, width=0.026, cap_style="flat"),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.005),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        "front_caster_wheel",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            CASTER_WHEEL_RADIUS,
            0.034,
            inner_radius=0.055,
            tread=TireTread(style="circumferential", depth=0.002, count=2),
            sidewall=TireSidewall(style="rounded", bulge=0.03),
        ),
        "front_caster_tire",
    )

    for i, y in enumerate((0.245, -0.245)):
        caster = model.part(f"front_caster_{i}")
        caster.visual(
            Cylinder(radius=0.012, length=0.120),
            origin=Origin(xyz=(0.0, 0.0, -0.060)),
            material=polished_aluminum,
            name="stem",
        )
        caster.visual(
            Box((0.090, 0.086, 0.020)),
            origin=Origin(xyz=(0.055, 0.0, -0.035)),
            material=polished_aluminum,
            name="fork_crown",
        )
        for side, yy in enumerate((0.036, -0.036)):
            caster.visual(
                Box((0.020, 0.010, 0.165)),
                origin=Origin(xyz=(0.095, yy, -0.107)),
                material=polished_aluminum,
                name=f"fork_blade_{side}",
            )
        caster.visual(
            Cylinder(radius=0.007, length=0.040),
            origin=Origin(xyz=(0.095, 0.050, -0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=polished_aluminum,
            name="axle_stub_0",
        )
        caster.visual(
            Cylinder(radius=0.007, length=0.040),
            origin=Origin(xyz=(0.095, -0.050, -0.125), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=polished_aluminum,
            name="axle_stub_1",
        )
        model.articulation(
            f"front_caster_swivel_{i}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(0.330, y, 0.220)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )

        caster_wheel = model.part(f"caster_wheel_{i}")
        caster_wheel.visual(caster_wheel_mesh, material=polished_aluminum, name="wheel")
        caster_wheel.visual(caster_tire_mesh, material=dark_rubber, name="tire")
        caster_wheel.visual(
            Cylinder(radius=0.019, length=0.060),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_aluminum,
            name="hub",
        )
        model.articulation(
            f"caster_wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=caster_wheel,
            origin=Origin(xyz=(0.095, 0.0, -0.125), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    continuous_joints = [
        "rear_wheel_spin_0",
        "rear_wheel_spin_1",
        "front_caster_swivel_0",
        "front_caster_swivel_1",
        "caster_wheel_spin_0",
        "caster_wheel_spin_1",
    ]
    ctx.check(
        "six continuous wheel and caster joints",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.CONTINUOUS
            for name in continuous_joints
        ),
        details="Rear wheels, front caster swivels, and caster wheels should all be continuous joints.",
    )

    ctx.expect_contact(
        "rear_wheel_0",
        "frame",
        elem_a="hub",
        elem_b="rear_axle",
        contact_tol=0.002,
        name="rear wheel 0 hub meets axle",
    )
    ctx.expect_contact(
        "rear_wheel_1",
        "frame",
        elem_a="hub",
        elem_b="rear_axle",
        contact_tol=0.002,
        name="rear wheel 1 hub meets axle",
    )

    rear_aabb = ctx.part_world_aabb("rear_wheel_0")
    caster_aabb = ctx.part_world_aabb("caster_wheel_0")
    if rear_aabb is not None and caster_aabb is not None:
        rear_size = tuple(float(rear_aabb[1][i] - rear_aabb[0][i]) for i in range(3))
        caster_size = tuple(float(caster_aabb[1][i] - caster_aabb[0][i]) for i in range(3))
        rear_diameter = max(rear_size[0], rear_size[2])
        caster_diameter = max(caster_size[0], caster_size[2])
        ctx.check(
            "rear wheels larger than casters",
            rear_diameter > 3.0 * caster_diameter,
            details=f"rear_diameter={rear_diameter:.3f}, caster_diameter={caster_diameter:.3f}",
        )

    rear_pos = ctx.part_world_position("rear_wheel_0")
    with ctx.pose(rear_wheel_spin_0=1.25):
        spun_rear_pos = ctx.part_world_position("rear_wheel_0")
    ctx.check(
        "rear wheel spins about fixed axle",
        rear_pos is not None
        and spun_rear_pos is not None
        and all(abs(rear_pos[i] - spun_rear_pos[i]) < 1e-6 for i in range(3)),
        details=f"rest={rear_pos}, spun={spun_rear_pos}",
    )

    caster_pos = ctx.part_world_position("caster_wheel_0")
    with ctx.pose(front_caster_swivel_0=1.0):
        swivel_pos = ctx.part_world_position("caster_wheel_0")
    ctx.check(
        "caster wheel follows swivel trail",
        caster_pos is not None
        and swivel_pos is not None
        and abs(caster_pos[0] - swivel_pos[0]) + abs(caster_pos[1] - swivel_pos[1]) > 0.020,
        details=f"rest={caster_pos}, swivel={swivel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
