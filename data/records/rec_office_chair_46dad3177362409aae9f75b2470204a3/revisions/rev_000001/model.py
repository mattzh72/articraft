from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c)


def _yz_section(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)]


def _make_seat_cushion_mesh():
    profile = [
        (0.0, 0.006),
        (0.070, 0.004),
        (0.145, 0.006),
        (0.176, 0.014),
        (0.188, 0.028),
        (0.184, 0.045),
        (0.162, 0.055),
        (0.090, 0.058),
        (0.0, 0.055),
    ]
    return _mesh("studio_chair_seat_cushion", LatheGeometry(profile, segments=64))


def _make_backrest_pad_mesh():
    return _mesh(
        "studio_chair_backrest_pad",
        section_loft(
            [
                _yz_section(-0.020, width=0.255, height=0.145, radius=0.030),
                _yz_section(0.000, width=0.275, height=0.160, radius=0.034),
                _yz_section(0.020, width=0.255, height=0.145, radius=0.030),
            ]
        ),
    )


def _add_caster(
    model: ArticulatedObject,
    pedestal,
    *,
    index: int,
    angle: float,
    base_radius: float,
    caster_z: float,
    metal,
    dark_metal,
    rubber,
) -> None:
    swivel = model.part(f"caster_{index}_swivel")
    swivel.visual(
        Cylinder(radius=0.0065, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=dark_metal,
        name="stem",
    )
    swivel.visual(
        Box((0.024, 0.032, 0.014)),
        origin=Origin(xyz=(-0.006, 0.0, -0.030)),
        material=dark_metal,
        name="fork_head",
    )
    swivel.visual(
        Box((0.026, 0.0045, 0.040)),
        origin=Origin(xyz=(-0.026, 0.013, -0.050)),
        material=dark_metal,
        name="left_fork_leg",
    )
    swivel.visual(
        Box((0.026, 0.0045, 0.040)),
        origin=Origin(xyz=(-0.026, -0.013, -0.050)),
        material=dark_metal,
        name="right_fork_leg",
    )
    swivel.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.075)),
        mass=0.10,
        origin=Origin(xyz=(-0.008, 0.0, -0.035)),
    )

    wheel = model.part(f"caster_{index}_wheel")
    wheel.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="wheel_tire",
    )
    wheel.visual(
        Cylinder(radius=0.015, length=0.021),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.018),
        mass=0.14,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    tip_x, tip_y = _rotate_xy(base_radius, 0.0, angle)
    model.articulation(
        f"pedestal_to_caster_{index}_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=swivel,
        origin=Origin(xyz=(tip_x, tip_y, caster_z), rpy=(0.0, 0.0, angle)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )
    model.articulation(
        f"caster_{index}_swivel_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=swivel,
        child=wheel,
        origin=Origin(xyz=(-0.026, 0.0, -0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_chair")

    polished_aluminum = model.material("polished_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.61, 0.65, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    black_vinyl = model.material("black_vinyl", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    seat_cushion_mesh = _make_seat_cushion_mesh()
    backrest_pad_mesh = _make_backrest_pad_mesh()

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.050, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=satin_steel,
        name="base_hub",
    )
    pedestal.visual(
        Cylinder(radius=0.044, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=polished_aluminum,
        name="outer_column",
    )
    pedestal.visual(
        Cylinder(radius=0.026, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.421)),
        material=satin_steel,
        name="inner_column",
    )
    pedestal.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.492)),
        material=dark_metal,
        name="top_bearing_plate",
    )

    arm_radius = 0.018
    arm_tip_radius = 0.300
    caster_mount_z = 0.084
    for arm_index in range(5):
        angle = 2.0 * math.pi * arm_index / 5.0
        arm_points = [
            (0.030, 0.0, 0.084),
            (0.145, 0.0, 0.078),
            (0.272, 0.0, 0.076),
        ]
        rotated_arm = [_rotate_xy(px, py, angle) + (pz,) for px, py, pz in arm_points]
        pedestal.visual(
            _mesh(
                f"studio_chair_arm_{arm_index}",
                tube_from_spline_points(
                    rotated_arm,
                    radius=arm_radius,
                    samples_per_segment=10,
                    radial_segments=18,
                    cap_ends=True,
                ),
            ),
            material=polished_aluminum,
            name=f"arm_{arm_index}",
        )
        socket_x, socket_y = _rotate_xy(arm_tip_radius, 0.0, angle)
        pad_x, pad_y = _rotate_xy(0.267, 0.0, angle)
        pedestal.visual(
            Box((0.040, 0.028, 0.024)),
            origin=Origin(xyz=(pad_x, pad_y, 0.082), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name=f"caster_mount_pad_{arm_index}",
        )
        pedestal.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(socket_x, socket_y, caster_mount_z + 0.005)),
            material=dark_metal,
            name=f"caster_socket_{arm_index}",
        )

    pedestal.inertial = Inertial.from_geometry(
        Box((0.65, 0.65, 0.52)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )

    seat = model.part("seat")
    seat.visual(seat_cushion_mesh, material=black_vinyl, name="seat_cushion")
    seat.visual(
        Cylinder(radius=0.118, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_metal,
        name="seat_plate",
    )
    seat.visual(
        Box((0.040, 0.170, 0.008)),
        origin=Origin(xyz=(0.124, 0.0, 0.010)),
        material=dark_metal,
        name="front_hinge_bridge",
    )
    seat.visual(
        Box((0.016, 0.016, 0.032)),
        origin=Origin(xyz=(0.145, 0.078, -0.002)),
        material=dark_metal,
        name="front_hinge_left_bracket",
    )
    seat.visual(
        Box((0.016, 0.016, 0.032)),
        origin=Origin(xyz=(0.145, -0.078, -0.002)),
        material=dark_metal,
        name="front_hinge_right_bracket",
    )
    seat.visual(
        Box((0.040, 0.146, 0.014)),
        origin=Origin(xyz=(-0.138, 0.0, 0.020)),
        material=dark_metal,
        name="rear_hinge_bridge",
    )
    seat.visual(
        Box((0.018, 0.016, 0.060)),
        origin=Origin(xyz=(-0.184, 0.064, 0.050)),
        material=dark_metal,
        name="rear_hinge_left_bracket",
    )
    seat.visual(
        Box((0.018, 0.016, 0.060)),
        origin=Origin(xyz=(-0.184, -0.064, 0.050)),
        material=dark_metal,
        name="rear_hinge_right_bracket",
    )
    seat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.190, length=0.060),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="backrest_hinge_tube",
    )
    backrest.visual(
        Box((0.036, 0.014, 0.135)),
        origin=Origin(xyz=(-0.018, 0.036, 0.064)),
        material=satin_steel,
        name="left_backrest_upright",
    )
    backrest.visual(
        Box((0.036, 0.014, 0.135)),
        origin=Origin(xyz=(-0.018, -0.036, 0.064)),
        material=satin_steel,
        name="right_backrest_upright",
    )
    backrest.visual(
        Box((0.030, 0.110, 0.014)),
        origin=Origin(xyz=(-0.030, 0.0, 0.126)),
        material=satin_steel,
        name="backrest_crossbar",
    )
    backrest.visual(
        backrest_pad_mesh,
        origin=Origin(xyz=(-0.042, 0.0, 0.205)),
        material=black_vinyl,
        name="back_pad",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.080, 0.280, 0.340)),
        mass=1.6,
        origin=Origin(xyz=(-0.025, 0.0, 0.165)),
    )

    footrest = model.part("footrest")
    footrest.visual(
        Cylinder(radius=0.0065, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="footrest_hinge_tube",
    )
    footrest.visual(
        Box((0.024, 0.012, 0.084)),
        origin=Origin(xyz=(0.000, 0.050, -0.042)),
        material=satin_steel,
        name="footrest_left_arm",
    )
    footrest.visual(
        Box((0.024, 0.012, 0.084)),
        origin=Origin(xyz=(0.000, -0.050, -0.042)),
        material=satin_steel,
        name="footrest_right_arm",
    )
    footrest.visual(
        Cylinder(radius=0.012, length=0.210),
        origin=Origin(xyz=(0.000, 0.0, -0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_aluminum,
        name="foot_bar",
    )
    footrest.inertial = Inertial.from_geometry(
        Box((0.095, 0.220, 0.090)),
        mass=0.8,
        origin=Origin(xyz=(0.012, 0.0, -0.048)),
    )

    model.articulation(
        "pedestal_to_seat_spin",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.498)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.184, 0.0, 0.056)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "seat_to_footrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=footrest,
        origin=Origin(xyz=(0.146, 0.0, -0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
    )

    for caster_index in range(5):
        _add_caster(
            model,
            pedestal,
            index=caster_index,
            angle=2.0 * math.pi * caster_index / 5.0,
            base_radius=arm_tip_radius,
            caster_z=caster_mount_z,
            metal=polished_aluminum,
            dark_metal=dark_metal,
            rubber=rubber,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    footrest = object_model.get_part("footrest")
    caster_swivel = object_model.get_articulation("pedestal_to_caster_0_swivel")
    caster_wheel_spin = object_model.get_articulation("caster_0_swivel_to_wheel")
    seat_spin = object_model.get_articulation("pedestal_to_seat_spin")
    backrest_hinge = object_model.get_articulation("seat_to_backrest")
    footrest_hinge = object_model.get_articulation("seat_to_footrest")
    caster_wheel = object_model.get_part("caster_0_wheel")

    ctx.expect_origin_gap(
        seat,
        pedestal,
        axis="z",
        min_gap=0.45,
        max_gap=0.55,
        name="seat is mounted above the pedestal column",
    )

    ctx.check(
        "seat swivel uses a vertical continuous joint",
        seat_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(seat_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={seat_spin.articulation_type}, axis={seat_spin.axis}",
    )
    ctx.check(
        "backrest hinge pitches about the seat rear",
        backrest_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(backrest_hinge.axis) == (0.0, -1.0, 0.0)
        and backrest_hinge.motion_limits is not None
        and backrest_hinge.motion_limits.upper is not None
        and backrest_hinge.motion_limits.upper > 0.30,
        details=f"type={backrest_hinge.articulation_type}, axis={backrest_hinge.axis}, limits={backrest_hinge.motion_limits}",
    )
    ctx.check(
        "footrest hinge folds down from the seat front",
        footrest_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(footrest_hinge.axis) == (0.0, -1.0, 0.0)
        and footrest_hinge.motion_limits is not None
        and footrest_hinge.motion_limits.upper is not None
        and footrest_hinge.motion_limits.upper >= 1.0,
        details=f"type={footrest_hinge.articulation_type}, axis={footrest_hinge.axis}, limits={footrest_hinge.motion_limits}",
    )
    ctx.check(
        "caster has swivel and wheel spin joints",
        caster_swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(caster_swivel.axis) == (0.0, 0.0, 1.0)
        and caster_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(caster_wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"swivel={caster_swivel.articulation_type, caster_swivel.axis}, spin={caster_wheel_spin.articulation_type, caster_wheel_spin.axis}",
    )

    rest_back_pad = ctx.part_element_world_aabb(backrest, elem="back_pad")
    with ctx.pose({backrest_hinge: 0.38}):
        reclined_back_pad = ctx.part_element_world_aabb(backrest, elem="back_pad")
    ctx.check(
        "backrest reclines backward",
        rest_back_pad is not None
        and reclined_back_pad is not None
        and ((reclined_back_pad[0][0] + reclined_back_pad[1][0]) * 0.5) < ((rest_back_pad[0][0] + rest_back_pad[1][0]) * 0.5) - 0.020,
        details=f"rest={rest_back_pad}, reclined={reclined_back_pad}",
    )

    rest_foot_bar = ctx.part_element_world_aabb(footrest, elem="foot_bar")
    with ctx.pose({footrest_hinge: 1.0}):
        deployed_foot_bar = ctx.part_element_world_aabb(footrest, elem="foot_bar")
    ctx.check(
        "footrest bar swings forward when deployed",
        rest_foot_bar is not None
        and deployed_foot_bar is not None
        and ((deployed_foot_bar[0][0] + deployed_foot_bar[1][0]) * 0.5) > ((rest_foot_bar[0][0] + rest_foot_bar[1][0]) * 0.5) + 0.040,
        details=f"rest={rest_foot_bar}, deployed={deployed_foot_bar}",
    )

    rest_back_pad = ctx.part_element_world_aabb(backrest, elem="back_pad")
    with ctx.pose({seat_spin: math.pi / 2.0}):
        spun_back_pad = ctx.part_element_world_aabb(backrest, elem="back_pad")
    ctx.check(
        "upper chair rotates about the pedestal",
        rest_back_pad is not None
        and spun_back_pad is not None
        and abs(((spun_back_pad[0][1] + spun_back_pad[1][1]) * 0.5)) > 0.10,
        details=f"rest={rest_back_pad}, spun={spun_back_pad}",
    )

    rest_wheel_pos = ctx.part_world_position(caster_wheel)
    with ctx.pose({caster_swivel: math.pi / 2.0}):
        swivel_wheel_pos = ctx.part_world_position(caster_wheel)
    ctx.check(
        "caster swivel changes wheel trail direction",
        rest_wheel_pos is not None
        and swivel_wheel_pos is not None
        and abs(swivel_wheel_pos[1] - rest_wheel_pos[1]) > 0.020,
        details=f"rest={rest_wheel_pos}, swivel={swivel_wheel_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
