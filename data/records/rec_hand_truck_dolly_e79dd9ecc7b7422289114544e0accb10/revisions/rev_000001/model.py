from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    ExtrudeGeometry,
    Material,
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
    tube_from_spline_points,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_hand_truck")

    red = model.material("powder_coated_red", rgba=(0.86, 0.08, 0.03, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_steel = model.material("dark_axle_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    hub_gray = model.material("cast_aluminum", rgba=(0.78, 0.78, 0.72, 1.0))

    frame = model.part("frame")

    # A low, rounded-corner steel toe plate, broad enough for boxes and thin
    # enough to read as a real load blade instead of a block.
    toe_plate = ExtrudeGeometry(
        rounded_rect_profile(0.36, 0.46, 0.035, corner_segments=8),
        0.025,
        cap=True,
        center=True,
    )
    frame.visual(
        mesh_from_geometry(toe_plate, "toe_plate_mesh"),
        origin=Origin(xyz=(0.15, 0.0, 0.025)),
        material=steel,
        name="toe_plate",
    )
    frame.visual(
        Box((0.038, 0.43, 0.13)),
        origin=Origin(xyz=(-0.035, 0.0, 0.088)),
        material=steel,
        name="back_lip",
    )
    frame.visual(
        Box((0.035, 0.39, 0.018)),
        origin=Origin(xyz=(0.315, 0.0, 0.043)),
        material=steel,
        name="front_bevel",
    )

    tube_radius = 0.017
    rail_half_width = 0.18
    for index, y in enumerate((-rail_half_width, rail_half_width)):
        side_rail = wire_from_points(
            [
                (0.02, y, 0.045),
                (-0.11, y, 0.125),
                (-0.18, y, 0.19),
                (-0.135, y, 0.56),
                (-0.115, y, 0.88),
                (-0.145, y, 1.13),
            ],
            radius=tube_radius,
            radial_segments=20,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.035,
            corner_segments=10,
        )
        frame.visual(
            mesh_from_geometry(side_rail, f"side_rail_{index}_mesh"),
            material=red,
            name=f"side_rail_{index}",
        )

        top_loop = tube_from_spline_points(
            [
                (-0.145, y, 1.08),
                (-0.225, y, 1.10),
                (-0.262, y, 1.20),
                (-0.210, y, 1.30),
                (-0.095, y, 1.285),
                (-0.075, y, 1.175),
                (-0.125, y, 1.085),
            ],
            radius=0.015,
            samples_per_segment=14,
            closed_spline=True,
            radial_segments=20,
        )
        frame.visual(
            mesh_from_geometry(top_loop, f"loop_handle_{index}_mesh"),
            material=red,
            name=f"loop_handle_{index}",
        )

        rubber_grip = tube_from_spline_points(
            [
                (-0.225, y, 1.235),
                (-0.185, y, 1.285),
                (-0.112, y, 1.275),
            ],
            radius=0.019,
            samples_per_segment=14,
            radial_segments=20,
            cap_ends=True,
        )
        frame.visual(
            mesh_from_geometry(rubber_grip, f"handle_grip_{index}_mesh"),
            material=rubber,
            name=f"handle_grip_{index}",
        )

    for z, x, name in (
        (0.245, -0.168, "lower_crossbar"),
        (0.64, -0.130, "middle_crossbar"),
        (1.035, -0.135, "upper_crossbar"),
    ):
        crossbar = wire_from_points(
            [(x, -0.23, z), (x, 0.23, z)],
            radius=0.015,
            radial_segments=20,
            cap_ends=True,
        )
        frame.visual(
            mesh_from_geometry(crossbar, f"{name}_mesh"),
            material=red,
            name=name,
        )

    frame.visual(
        Box((0.065, 0.028, 0.86)),
        origin=Origin(xyz=(-0.150, 0.0, 0.640)),
        material=red,
        name="center_back_spine",
    )

    # Diagonal toe-plate braces tie the flat blade into the wheel/rail assembly.
    for index, y in enumerate((-0.145, 0.145)):
        brace = wire_from_points(
            [(0.055, y, 0.04), (-0.18, y, 0.19)],
            radius=0.012,
            radial_segments=16,
            cap_ends=True,
        )
        frame.visual(
            mesh_from_geometry(brace, f"toe_brace_{index}_mesh"),
            material=red,
            name=f"toe_brace_{index}",
        )

    # Fixed axle spanning through the two independently spinning wheels.
    frame.visual(
        Cylinder(radius=0.019, length=0.76),
        origin=Origin(xyz=(-0.18, 0.0, 0.19), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    for index, y in enumerate((-0.235, 0.235)):
        frame.visual(
            Box((0.070, 0.030, 0.115)),
            origin=Origin(xyz=(-0.18, y, 0.215)),
            material=red,
            name=f"axle_bracket_{index}",
        )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.128,
            0.060,
            rim=WheelRim(
                inner_radius=0.078,
                flange_height=0.009,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.035,
                width=0.052,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.046,
                    hole_diameter=0.005,
                ),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.048),
        ),
        "wheel_rim_mesh",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.185,
            0.078,
            inner_radius=0.126,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.009, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.007, depth=0.003),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.009, radius=0.004),
        ),
        "wheel_tire_mesh",
    )

    wheel_positions = [(-0.18, -0.305, 0.185), (-0.18, 0.305, 0.185)]
    wheel_axes = [(0.0, -1.0, 0.0), (0.0, 1.0, 0.0)]
    wheel_rpys = [(0.0, 0.0, -pi / 2.0), (0.0, 0.0, pi / 2.0)]
    for index, (position, axis, rpy) in enumerate(zip(wheel_positions, wheel_axes, wheel_rpys)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=rpy),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=rpy),
            material=hub_gray,
            name="rim",
        )
        model.articulation(
            f"axle_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=position),
            axis=axis,
            motion_limits=MotionLimits(effort=35.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    joint_0 = object_model.get_articulation("axle_to_wheel_0")
    joint_1 = object_model.get_articulation("axle_to_wheel_1")

    for wheel in (wheel_0, wheel_1):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason=(
                "The fixed axle is intentionally modeled as captured through "
                "the wheel hub/bore proxy while the wheel spins continuously."
            ),
        )
        ctx.expect_within(
            frame,
            wheel,
            axes="xz",
            inner_elem="axle",
            outer_elem="rim",
            margin=0.0,
            name=f"{wheel.name} axle is concentric with the hub",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="y",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.055,
            name=f"{wheel.name} hub retains axle insertion",
        )

    ctx.check(
        "only the two wheels articulate",
        len(object_model.articulations) == 2
        and joint_0.articulation_type == ArticulationType.CONTINUOUS
        and joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.expect_origin_distance(
        wheel_0,
        wheel_1,
        axes="y",
        min_dist=0.58,
        max_dist=0.64,
        name="wheels sit at opposite lower corners",
    )
    wheel_center = ctx.part_world_position(wheel_0)
    ctx.check(
        "wheel centers sit on the fixed axle height",
        wheel_center is not None and abs(wheel_center[2] - 0.185) < 1e-6,
        details=f"wheel_0 position={wheel_center}",
    )

    rest_0 = ctx.part_world_position(wheel_0)
    rest_1 = ctx.part_world_position(wheel_1)
    with ctx.pose({joint_0: 1.75, joint_1: -2.25}):
        spun_0 = ctx.part_world_position(wheel_0)
        spun_1 = ctx.part_world_position(wheel_1)
    ctx.check(
        "wheel spin keeps hubs on axle",
        rest_0 is not None
        and rest_1 is not None
        and spun_0 is not None
        and spun_1 is not None
        and max(abs(rest_0[i] - spun_0[i]) for i in range(3)) < 1e-6
        and max(abs(rest_1[i] - spun_1[i]) for i in range(3)) < 1e-6,
        details=f"rest=({rest_0}, {rest_1}), spun=({spun_0}, {spun_1})",
    )

    return ctx.report()


object_model = build_object_model()
