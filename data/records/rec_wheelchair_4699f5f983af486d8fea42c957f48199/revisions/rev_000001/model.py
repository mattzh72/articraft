from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
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
)


def _axis_to_rpy(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = axis
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 0.0:
        raise ValueError("axis must be non-zero")
    x, y, z = x / length, y / length, z / length
    return (0.0, math.atan2(math.sqrt(x * x + y * y), z), math.atan2(y, x))


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str,
):
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=_axis_to_rpy((vx, vy, vz)),
        ),
        material=material,
        name=name,
    )


def _push_rim_mesh(name: str):
    rim = TorusGeometry(0.258, 0.006, radial_segments=24, tubular_segments=96)
    rim.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(rim, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    polished_aluminum = model.material("polished_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.03, 0.03, 0.035, 1.0))
    blue_fabric = model.material("blue_fabric", rgba=(0.05, 0.16, 0.34, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.018, 1.0))
    gray_hub = model.material("brushed_gray_hub", rgba=(0.58, 0.61, 0.64, 1.0))

    rear_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.268,
            0.045,
            rim=WheelRim(inner_radius=0.202, flange_height=0.010, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.042,
                width=0.047,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.054, hole_diameter=0.0045),
            ),
            face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
            spokes=WheelSpokes(style="straight", count=18, thickness=0.0032, window_radius=0.014),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "rear_wheel_rim",
    )
    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.320,
            0.055,
            inner_radius=0.270,
            tread=TireTread(style="circumferential", depth=0.004, count=4),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.05),
            shoulder=TireShoulder(width=0.008, radius=0.004),
        ),
        "rear_tire",
    )
    caster_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.053,
            0.032,
            rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(radius=0.017, width=0.034, cap_style="flat"),
            face=WheelFace(dish_depth=0.002, front_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        "caster_wheel_rim",
    )
    caster_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.042,
            inner_radius=0.055,
            tread=TireTread(style="block", depth=0.003, count=16, land_ratio=0.60),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )
    right_push_rim = _push_rim_mesh("right_push_rim")
    left_push_rim = _push_rim_mesh("left_push_rim")

    frame = model.part("frame")

    # Upholstery and rigid support boards are fixed to the welded frame.
    frame.visual(
        Box((0.50, 0.50, 0.065)),
        origin=Origin(xyz=(0.0, 0.075, 0.510)),
        material=blue_fabric,
        name="seat_cushion",
    )
    frame.visual(
        Box((0.50, 0.055, 0.53)),
        origin=Origin(xyz=(0.0, -0.245, 0.785)),
        material=blue_fabric,
        name="backrest",
    )

    # Seat rails, cross tubes, uprights, and diagonal braces.
    for side, x in (("left", 0.265), ("right", -0.265)):
        _cylinder_between(frame, (x, -0.210, 0.475), (x, 0.370, 0.475), radius=0.017, material=polished_aluminum, name=f"{side}_seat_rail")
        _cylinder_between(frame, (x, -0.215, 0.305), (x, -0.215, 1.050), radius=0.018, material=polished_aluminum, name=f"{side}_back_post")
        _cylinder_between(frame, (x, 0.365, 0.255), (x, 0.365, 0.490), radius=0.017, material=polished_aluminum, name=f"{side}_front_post")
        _cylinder_between(frame, (x, -0.205, 0.315), (x, 0.370, 0.485), radius=0.014, material=polished_aluminum, name=f"{side}_diagonal_brace")
        _cylinder_between(frame, (x, -0.215, 1.035), (x, -0.405, 1.110), radius=0.016, material=polished_aluminum, name=f"{side}_push_handle")
        frame.visual(
            Cylinder(radius=0.025, length=0.090),
            origin=Origin(xyz=(x, -0.410, 1.112), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_plastic,
            name=f"{side}_hand_grip",
        )
        _cylinder_between(frame, (x, 0.365, 0.300), (x * 0.65, 0.365, 0.300), radius=0.012, material=polished_aluminum, name=f"{side}_footrest_mount")
        _cylinder_between(frame, (x * 0.65, 0.365, 0.300), (x * 0.64, 0.680, 0.120), radius=0.012, material=polished_aluminum, name=f"{side}_footrest_tube")
        frame.visual(
            Box((0.175, 0.135, 0.018)),
            origin=Origin(xyz=(x * 0.58, 0.725, 0.112), rpy=(0.0, -0.16 if x > 0 else 0.16, 0.0)),
            material=black_plastic,
            name=f"{side}_footplate",
        )
        _cylinder_between(frame, (x, 0.370, 0.300), (x, 0.450, 0.300), radius=0.017, material=polished_aluminum, name=f"{side}_caster_reach")
        _cylinder_between(frame, (x, 0.450, 0.255), (x, 0.450, 0.325), radius=0.021, material=polished_aluminum, name=f"{side}_caster_socket")

    _cylinder_between(frame, (-0.285, 0.370, 0.475), (0.285, 0.370, 0.475), radius=0.017, material=polished_aluminum, name="front_cross_tube")
    _cylinder_between(frame, (-0.285, -0.210, 0.475), (0.285, -0.210, 0.475), radius=0.017, material=polished_aluminum, name="rear_seat_cross_tube")
    _cylinder_between(frame, (-0.390, -0.160, 0.320), (0.390, -0.160, 0.320), radius=0.020, material=polished_aluminum, name="rear_axle_tube")
    _cylinder_between(frame, (-0.220, -0.225, 0.990), (0.220, -0.225, 0.990), radius=0.015, material=polished_aluminum, name="back_cross_tube")
    _cylinder_between(frame, (-0.250, -0.235, 0.600), (0.250, -0.235, 0.600), radius=0.014, material=polished_aluminum, name="back_lumbar_tube")

    # Short axle noses end at the wheel hub faces; they visually explain the wheel bearings.
    _cylinder_between(frame, (0.388, -0.160, 0.320), (0.448, -0.160, 0.320), radius=0.017, material=polished_aluminum, name="left_axle_nose")
    _cylinder_between(frame, (-0.388, -0.160, 0.320), (-0.448, -0.160, 0.320), radius=0.017, material=polished_aluminum, name="right_axle_nose")

    rear_positions = {
        "left": (0.460, -0.160, 0.320, left_push_rim, 0.040),
        "right": (-0.460, -0.160, 0.320, right_push_rim, -0.040),
    }
    for side, (x, y, z, push_rim_mesh, push_offset) in rear_positions.items():
        wheel = model.part(f"{side}_rear_wheel")
        wheel.visual(rear_wheel_mesh, material=gray_hub, name="rim_and_spokes")
        wheel.visual(rear_tire_mesh, material=dark_rubber, name="tire")
        wheel.visual(push_rim_mesh, origin=Origin(xyz=(push_offset, 0.0, 0.0)), material=polished_aluminum, name="push_rim")
        for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
            wheel.visual(
                Box((0.028, 0.008, 0.008)),
                origin=Origin(xyz=(push_offset * 0.74, 0.258 * math.cos(angle), 0.258 * math.sin(angle))),
                material=polished_aluminum,
                name=f"push_rim_standoff_{index}",
            )
        model.articulation(
            f"{side}_rear_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=20.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    for side, x in (("left", 0.265), ("right", -0.265)):
        caster = model.part(f"{side}_caster")
        caster.visual(
            Cylinder(radius=0.014, length=0.105),
            origin=Origin(xyz=(0.0, 0.0, -0.0525)),
            material=polished_aluminum,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.090, 0.070, 0.018)),
            origin=Origin(xyz=(0.0, -0.0175, -0.095)),
            material=polished_aluminum,
            name="fork_crown",
        )
        for fork_side, fx in (("outer", 0.032), ("inner", -0.032)):
            caster.visual(
                Box((0.012, 0.018, 0.150)),
                origin=Origin(xyz=(fx, -0.035, -0.145)),
                material=polished_aluminum,
                name=f"{fork_side}_fork_leg",
            )
            caster.visual(
                Cylinder(radius=0.017, length=0.010),
                origin=Origin(xyz=(fx, -0.035, -0.180), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=polished_aluminum,
                name=f"{fork_side}_axle_boss",
            )
        model.articulation(
            f"{side}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=caster,
            origin=Origin(xyz=(x, 0.450, 0.255)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
            motion_properties=MotionProperties(damping=0.05, friction=0.02),
        )

        caster_wheel = model.part(f"{side}_caster_wheel")
        caster_wheel.visual(caster_wheel_mesh, material=gray_hub, name="rim")
        caster_wheel.visual(caster_tire_mesh, material=dark_rubber, name="tire")
        model.articulation(
            f"{side}_caster_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, -0.035, -0.180)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=20.0),
            motion_properties=MotionProperties(damping=0.02, friction=0.01),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_rear = object_model.get_part("left_rear_wheel")
    right_rear = object_model.get_part("right_rear_wheel")
    left_caster = object_model.get_part("left_caster")
    left_caster_wheel = object_model.get_part("left_caster_wheel")

    expected_continuous = (
        "left_rear_spin",
        "right_rear_spin",
        "left_caster_swivel",
        "right_caster_swivel",
        "left_caster_spin",
        "right_caster_spin",
    )
    for joint_name in expected_continuous:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_is_continuous",
            joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} should be a continuous wheelchair rolling/swivel joint.",
        )

    ctx.expect_origin_distance(left_rear, right_rear, axes="x", min_dist=0.88, name="rear_wheels_have_wide_stance")
    ctx.expect_overlap(left_rear, right_rear, axes="y", min_overlap=0.40, name="rear_wheels_share_axle_line")
    for side, wheel_name, nose_name in (
        ("left", "left_rear_wheel", "left_axle_nose"),
        ("right", "right_rear_wheel", "right_axle_nose"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=nose_name,
            elem_b="rim_and_spokes",
            reason="The fixed axle nose is intentionally inserted into the wheel hub bearing.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="x",
            min_overlap=0.006,
            elem_a=nose_name,
            elem_b="rim_and_spokes",
            name=f"{side}_axle_inserted_in_hub",
        )
    ctx.expect_gap(
        left_rear,
        frame,
        axis="x",
        min_gap=0.15,
        positive_elem="tire",
        negative_elem="seat_cushion",
        name="left_tire_outboard_of_seat",
    )
    ctx.expect_gap(
        frame,
        right_rear,
        axis="x",
        min_gap=0.15,
        positive_elem="seat_cushion",
        negative_elem="tire",
        name="right_tire_outboard_of_seat",
    )

    # The front caster wheel trails the vertical stem, so changing swivel pose should move its axle.
    swivel = object_model.get_articulation("left_caster_swivel")
    rest_pos = ctx.part_world_position(left_caster_wheel)
    with ctx.pose({swivel: 1.0}):
        turned_pos = ctx.part_world_position(left_caster_wheel)
    ctx.check(
        "caster_swivel_moves_trailing_wheel",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) > 0.015
        and abs(rest_pos[1] - turned_pos[1]) > 0.004,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    ctx.expect_overlap(left_caster, left_caster_wheel, axes="z", min_overlap=0.050, name="caster_fork_surrounds_wheel_height")

    return ctx.report()


object_model = build_object_model()
