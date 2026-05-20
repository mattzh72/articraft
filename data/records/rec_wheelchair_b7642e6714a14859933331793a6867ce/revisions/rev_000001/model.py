from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    wire_from_points,
)


def _tube_mesh(points, radius: float, name: str):
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=radius * 2.4,
            corner_segments=8,
        ),
        name,
    )


def _straight_tube_mesh(a, b, radius: float, name: str):
    return mesh_from_geometry(
        wire_from_points(
            [a, b],
            radius=radius,
            radial_segments=18,
            cap_ends=True,
            corner_mode="miter",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_fabric = model.material("dark_navy_fabric", rgba=(0.03, 0.055, 0.09, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    graphite = model.material("graphite", rgba=(0.09, 0.095, 0.10, 1.0))
    footplate_finish = model.material("ribbed_black_footplates", rgba=(0.025, 0.025, 0.025, 1.0))

    frame = model.part("frame")

    # Slim tubular side frames under the seat, with tall rear push-handle posts.
    for suffix, y in (("0", -0.23), ("1", 0.23)):
        frame.visual(
            _tube_mesh(
                [
                    (-0.285, y, 0.305),
                    (-0.245, y, 0.480),
                    (0.285, y, 0.480),
                    (0.345, y, 0.240),
                    (0.170, y, 0.240),
                    (-0.285, y, 0.305),
                ],
                0.014,
                f"side_frame_{suffix}",
            ),
            material=satin_aluminum,
            name=f"side_frame_{suffix}",
        )
        frame.visual(
            _tube_mesh(
                [
                    (-0.250, y, 0.470),
                    (-0.290, y, 0.730),
                    (-0.335, y, 1.020),
                    (-0.425, y, 1.045),
                ],
                0.014,
                f"rear_post_{suffix}",
            ),
            material=satin_aluminum,
            name=f"rear_post_{suffix}",
        )
        frame.visual(
            _straight_tube_mesh(
                (-0.425, y, 1.045),
                (-0.515, y, 1.045),
                0.018,
                f"push_grip_{suffix}",
            ),
            material=black_rubber,
            name=f"push_grip_{suffix}",
        )

    # Cross tubes tie the narrow frame footprint together and provide wheel/caster mounts.
    frame.visual(
        _straight_tube_mesh((-0.285, -0.365, 0.305), (-0.285, 0.365, 0.305), 0.016, "rear_axle"),
        material=satin_aluminum,
        name="rear_axle",
    )
    for name, x, z, radius in (
        ("rear_seat_rail", -0.225, 0.480, 0.012),
        ("front_seat_rail", 0.285, 0.480, 0.012),
        ("front_caster_rail", 0.345, 0.240, 0.012),
        ("backrest_crossbar", -0.320, 0.870, 0.011),
        ("handle_crossbar", -0.370, 1.010, 0.010),
    ):
        frame.visual(
            _straight_tube_mesh((x, -0.245, z), (x, 0.245, z), radius, name),
            material=satin_aluminum,
            name=name,
        )
    frame.visual(
        _straight_tube_mesh((-0.370, -0.240, 1.010), (-0.390, -0.230, 1.040), 0.011, "handle_bridge_0"),
        material=satin_aluminum,
        name="handle_bridge_0",
    )
    frame.visual(
        _straight_tube_mesh((-0.370, 0.240, 1.010), (-0.390, 0.230, 1.040), 0.011, "handle_bridge_1"),
        material=satin_aluminum,
        name="handle_bridge_1",
    )

    frame.visual(
        _straight_tube_mesh((0.345, -0.240, 0.240), (0.425, -0.240, 0.235), 0.012, "caster_socket_0"),
        material=satin_aluminum,
        name="caster_socket_0",
    )
    frame.visual(
        _straight_tube_mesh((0.345, 0.240, 0.240), (0.425, 0.240, 0.235), 0.012, "caster_socket_1"),
        material=satin_aluminum,
        name="caster_socket_1",
    )

    # Seat, back sling, arm rests, and footrests are fixed to the same supported frame.
    frame.visual(
        Box((0.465, 0.485, 0.045)),
        origin=Origin(xyz=(0.035, 0.0, 0.505)),
        material=dark_fabric,
        name="seat_cushion",
    )
    frame.visual(
        Box((0.045, 0.485, 0.405)),
        origin=Origin(xyz=(-0.285, 0.0, 0.705), rpy=(0.0, -0.08, 0.0)),
        material=dark_fabric,
        name="backrest_sling",
    )
    for suffix, y in (("0", -0.285), ("1", 0.285)):
        frame.visual(
            Box((0.330, 0.050, 0.035)),
            origin=Origin(xyz=(0.025, y, 0.670)),
            material=black_rubber,
            name=f"arm_pad_{suffix}",
        )
        inner_y = -0.235 if y < 0.0 else 0.235
        frame.visual(
            _straight_tube_mesh((-0.135, y, 0.650), (-0.155, inner_y, 0.490), 0.010, f"arm_support_{suffix}_0"),
            material=satin_aluminum,
            name=f"arm_support_{suffix}_0",
        )
        frame.visual(
            _straight_tube_mesh((0.175, y, 0.650), (0.205, inner_y, 0.490), 0.010, f"arm_support_{suffix}_1"),
            material=satin_aluminum,
            name=f"arm_support_{suffix}_1",
        )

    for suffix, y in (("0", -0.145), ("1", 0.145)):
        frame.visual(
            _tube_mesh(
                [
                    (0.255, y, 0.455),
                    (0.365, y, 0.320),
                    (0.505, y, 0.190),
                    (0.535, y, 0.160),
                ],
                0.011,
                f"footrest_hanger_{suffix}",
            ),
            material=satin_aluminum,
            name=f"footrest_hanger_{suffix}",
        )
        frame.visual(
            _straight_tube_mesh((0.535, y, 0.160), (0.570, y, 0.138), 0.012, f"footplate_socket_{suffix}"),
            material=satin_aluminum,
            name=f"footplate_socket_{suffix}",
        )
        frame.visual(
            Box((0.175, 0.135, 0.018)),
            origin=Origin(xyz=(0.570, y, 0.135), rpy=(0.0, 0.10, 0.0)),
            material=footplate_finish,
            name=f"footplate_{suffix}",
        )
        frame.visual(
            Box((0.115, 0.006, 0.006)),
            origin=Origin(xyz=(0.575, y - 0.045, 0.142), rpy=(0.0, 0.10, 0.0)),
            material=graphite,
            name=f"footplate_rib_{suffix}_0",
        )
        frame.visual(
            Box((0.115, 0.006, 0.006)),
            origin=Origin(xyz=(0.575, y, 0.142), rpy=(0.0, 0.10, 0.0)),
            material=graphite,
            name=f"footplate_rib_{suffix}_1",
        )
        frame.visual(
            Box((0.115, 0.006, 0.006)),
            origin=Origin(xyz=(0.575, y + 0.045, 0.142), rpy=(0.0, 0.10, 0.0)),
            material=graphite,
            name=f"footplate_rib_{suffix}_2",
        )

    rear_wheel_specs = (("rear_wheel_0", -0.340, -1.0), ("rear_wheel_1", 0.340, 1.0))
    for part_name, y, handrim_side in rear_wheel_specs:
        wheel = model.part(part_name)
        wheel.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.262,
                    0.034,
                    rim=WheelRim(inner_radius=0.205, flange_height=0.007, flange_thickness=0.003),
                    hub=WheelHub(
                        radius=0.035,
                        width=0.036,
                        cap_style="domed",
                        bolt_pattern=BoltPattern(count=6, circle_diameter=0.045, hole_diameter=0.004),
                    ),
                    face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                    spokes=WheelSpokes(style="straight", count=24, thickness=0.0024, window_radius=0.012),
                    bore=WheelBore(style="round", diameter=0.030),
                ),
                f"{part_name}_rim",
            ),
            material=satin_aluminum,
            name="rim",
        )
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.310,
                    0.045,
                    inner_radius=0.264,
                    tread=TireTread(style="circumferential", depth=0.003, count=5),
                    grooves=(
                        TireGroove(center_offset=-0.010, width=0.004, depth=0.002),
                        TireGroove(center_offset=0.010, width=0.004, depth=0.002),
                    ),
                    sidewall=TireSidewall(style="rounded", bulge=0.045),
                ),
                f"{part_name}_tire",
            ),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(
                TorusGeometry(0.278, 0.0055, radial_segments=18, tubular_segments=72).rotate_y(math.pi / 2.0),
                f"{part_name}_handrim",
            ),
            origin=Origin(xyz=(handrim_side * 0.034, 0.0, 0.0)),
            material=satin_aluminum,
            name="handrim",
        )
        for i in range(6):
            theta = i * math.tau / 6.0
            radial_y = 0.278 * math.cos(theta)
            radial_z = 0.278 * math.sin(theta)
            wheel.visual(
                _straight_tube_mesh(
                    (handrim_side * 0.034, radial_y, radial_z),
                    (handrim_side * 0.002, 0.258 * math.cos(theta), 0.258 * math.sin(theta)),
                    0.004,
                    f"{part_name}_handrim_standoff_{i}",
                ),
                material=satin_aluminum,
                name=f"handrim_standoff_{i}",
            )
        model.articulation(
            f"frame_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(-0.285, y, 0.305), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=18.0),
        )

    for suffix, y in (("0", -0.240), ("1", 0.240)):
        caster = model.part(f"caster_fork_{suffix}")
        caster.visual(
            Cylinder(radius=0.012, length=0.150),
            origin=Origin(xyz=(0.0, 0.0, -0.075)),
            material=satin_aluminum,
            name="swivel_stem",
        )
        caster.visual(
            Cylinder(radius=0.060, length=0.018),
            origin=Origin(xyz=(-0.045, 0.0, -0.092)),
            material=graphite,
            name="fork_crown",
        )
        for side, yy in (("0", -0.032), ("1", 0.032)):
            caster.visual(
                Box((0.050, 0.007, 0.118)),
                origin=Origin(xyz=(-0.095, yy, -0.154)),
                material=graphite,
                name=f"fork_plate_{side}",
            )
        caster.visual(
            Cylinder(radius=0.008, length=0.080),
            origin=Origin(xyz=(-0.095, 0.0, -0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_aluminum,
            name="caster_axle",
        )
        model.articulation(
            f"frame_to_caster_fork_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(0.425, y, 0.235)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=10.0),
        )

        wheel = model.part(f"caster_wheel_{suffix}")
        wheel.visual(
            mesh_from_geometry(
                WheelGeometry(
                    0.047,
                    0.026,
                    rim=WheelRim(inner_radius=0.026, flange_height=0.003, flange_thickness=0.0015),
                    hub=WheelHub(radius=0.014, width=0.024, cap_style="flat"),
                    face=WheelFace(dish_depth=0.002),
                    spokes=WheelSpokes(style="straight", count=6, thickness=0.002, window_radius=0.004),
                    bore=WheelBore(style="round", diameter=0.012),
                ),
                f"caster_wheel_{suffix}_rim",
            ),
            material=satin_aluminum,
            name="rim",
        )
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.064,
                    0.034,
                    inner_radius=0.047,
                    tread=TireTread(style="circumferential", depth=0.002, count=3),
                    sidewall=TireSidewall(style="rounded", bulge=0.035),
                ),
                f"caster_wheel_{suffix}_tire",
            ),
            material=black_rubber,
            name="tire",
        )
        model.articulation(
            f"caster_fork_{suffix}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(-0.095, 0.0, -0.160), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")

    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    caster_fork_0 = object_model.get_part("caster_fork_0")
    caster_fork_1 = object_model.get_part("caster_fork_1")
    caster_wheel_0 = object_model.get_part("caster_wheel_0")
    caster_wheel_1 = object_model.get_part("caster_wheel_1")

    for joint_name in (
        "frame_to_rear_wheel_0",
        "frame_to_rear_wheel_1",
        "frame_to_caster_fork_0",
        "frame_to_caster_fork_1",
        "caster_fork_0_to_wheel",
        "caster_fork_1_to_wheel",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_continuous",
            joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} should be a continuous rotation joint.",
        )

    ctx.allow_overlap(
        caster_fork_0,
        caster_wheel_0,
        elem_a="caster_axle",
        elem_b="rim",
        reason="The caster axle is intentionally captured through the wheel hub/rim bore.",
    )
    ctx.allow_overlap(
        caster_fork_1,
        caster_wheel_1,
        elem_a="caster_axle",
        elem_b="rim",
        reason="The caster axle is intentionally captured through the wheel hub/rim bore.",
    )
    ctx.allow_overlap(
        caster_fork_0,
        frame,
        elem_a="swivel_stem",
        elem_b="caster_socket_0",
        reason="The vertical caster stem is intentionally seated inside the frame socket.",
    )
    ctx.allow_overlap(
        caster_fork_1,
        frame,
        elem_a="swivel_stem",
        elem_b="caster_socket_1",
        reason="The vertical caster stem is intentionally seated inside the frame socket.",
    )
    ctx.allow_overlap(
        frame,
        rear_wheel_0,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The fixed rear axle intentionally passes through the rotating wheel hub bore.",
    )
    ctx.allow_overlap(
        frame,
        rear_wheel_1,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The fixed rear axle intentionally passes through the rotating wheel hub bore.",
    )
    ctx.expect_overlap(
        caster_fork_0,
        caster_wheel_0,
        axes="xyz",
        min_overlap=0.010,
        elem_a="caster_axle",
        elem_b="rim",
        name="caster_0_axle_captured",
    )
    ctx.expect_overlap(
        caster_fork_1,
        caster_wheel_1,
        axes="xyz",
        min_overlap=0.010,
        elem_a="caster_axle",
        elem_b="rim",
        name="caster_1_axle_captured",
    )
    ctx.expect_overlap(
        caster_fork_0,
        frame,
        axes="xy",
        min_overlap=0.006,
        elem_a="swivel_stem",
        elem_b="caster_socket_0",
        name="caster_0_stem_seated",
    )
    ctx.expect_overlap(
        caster_fork_1,
        frame,
        axes="xy",
        min_overlap=0.006,
        elem_a="swivel_stem",
        elem_b="caster_socket_1",
        name="caster_1_stem_seated",
    )
    ctx.expect_overlap(
        frame,
        rear_wheel_0,
        axes="xyz",
        min_overlap=0.010,
        elem_a="rear_axle",
        elem_b="rim",
        name="rear_wheel_0_axle_captured",
    )
    ctx.expect_overlap(
        frame,
        rear_wheel_1,
        axes="xyz",
        min_overlap=0.010,
        elem_a="rear_axle",
        elem_b="rim",
        name="rear_wheel_1_axle_captured",
    )

    ctx.expect_overlap(rear_wheel_0, frame, axes="yz", min_overlap=0.010)
    ctx.expect_overlap(rear_wheel_1, frame, axes="yz", min_overlap=0.010)
    ctx.expect_overlap(caster_wheel_0, caster_fork_0, axes="xz", min_overlap=0.020)
    ctx.expect_overlap(caster_wheel_1, caster_fork_1, axes="xz", min_overlap=0.020)

    rest_pos = ctx.part_world_position(caster_wheel_0)
    with ctx.pose({"frame_to_caster_fork_0": math.pi / 2.0, "caster_fork_0_to_wheel": math.pi}):
        swiveled_pos = ctx.part_world_position(caster_wheel_0)
    ctx.check(
        "front caster swivels around vertical stem",
        rest_pos is not None
        and swiveled_pos is not None
        and abs(swiveled_pos[1] - rest_pos[1]) > 0.025
        and abs(swiveled_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, swiveled={swiveled_pos}",
    )

    return ctx.report()


object_model = build_object_model()
