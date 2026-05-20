from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
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
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _rounded_pad(width: float, height: float, thickness: float, radius: float, name: str):
    return _mesh(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            thickness,
            center=True,
        ),
        name,
    )


def _back_pad(width: float, height: float, thickness: float, radius: float, name: str):
    geometry = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
        center=True,
    )
    geometry.rotate_x(pi / 2.0)
    return _mesh(geometry, name)


def _rear_wheel_visuals(part, prefix: str, *, side_sign: float, metal, rubber) -> None:
    part.visual(
        _mesh(
            WheelGeometry(
                0.255,
                0.040,
                rim=WheelRim(inner_radius=0.205, flange_height=0.010, flange_thickness=0.004),
                hub=WheelHub(radius=0.046, width=0.060, cap_style="domed"),
                face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.003),
                spokes=WheelSpokes(style="straight", count=12, thickness=0.004, window_radius=0.012),
                bore=WheelBore(style="round", diameter=0.018),
            ),
            f"{prefix}_rim",
        ),
        material=metal,
        name="rim",
    )
    part.visual(
        _mesh(
            TireGeometry(
                0.310,
                0.052,
                inner_radius=0.265,
                carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
                tread=TireTread(style="circumferential", depth=0.004, count=4),
                grooves=(
                    TireGroove(center_offset=-0.012, width=0.004, depth=0.002),
                    TireGroove(center_offset=0.012, width=0.004, depth=0.002),
                ),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
            ),
            f"{prefix}_tire",
        ),
        material=rubber,
        name="tire",
    )
    hand_rim = TorusGeometry(0.278, 0.006, radial_segments=18, tubular_segments=72).rotate_y(pi / 2.0)
    part.visual(
        _mesh(hand_rim, f"{prefix}_handrim"),
        origin=Origin(xyz=(side_sign * 0.044, 0.0, 0.0)),
        material=metal,
        name="handrim",
    )
    for index, (rod_y, rod_z) in enumerate(
        (
            (0.000, 0.266),
            (0.230, 0.133),
            (0.230, -0.133),
            (0.000, -0.266),
            (-0.230, -0.133),
            (-0.230, 0.133),
        )
    ):
        part.visual(
            Cylinder(radius=0.010, length=0.040),
            origin=Origin(xyz=(side_sign * 0.028, rod_y, rod_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name=f"handrim_standoff_{index}",
        )


def _caster_wheel_visuals(part, prefix: str, *, metal, rubber) -> None:
    part.visual(
        _mesh(
            WheelGeometry(
                0.064,
                0.038,
                rim=WheelRim(inner_radius=0.043, flange_height=0.005, flange_thickness=0.002),
                hub=WheelHub(radius=0.020, width=0.046, cap_style="flat"),
                face=WheelFace(dish_depth=0.003, front_inset=0.002),
                spokes=WheelSpokes(style="straight", count=5, thickness=0.003, window_radius=0.006),
                bore=WheelBore(style="round", diameter=0.010),
            ),
            f"{prefix}_rim",
        ),
        material=metal,
        name="rim",
    )
    part.visual(
        _mesh(
            TireGeometry(
                0.095,
                0.050,
                inner_radius=0.066,
                tread=TireTread(style="circumferential", depth=0.003, count=3),
                sidewall=TireSidewall(style="rounded", bulge=0.04),
            ),
            f"{prefix}_tire",
        ),
        material=rubber,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    powder_coat = model.material("powder_coat_black", rgba=(0.06, 0.065, 0.07, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.12, 0.13, 0.14, 1.0))
    cushion = model.material("black_vinyl", rgba=(0.025, 0.025, 0.030, 1.0))
    webbing = model.material("back_webbing", rgba=(0.045, 0.047, 0.052, 1.0))
    metal = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.74, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.010, 0.010, 0.011, 1.0))
    foot_grip = model.material("grippy_footplates", rgba=(0.08, 0.085, 0.09, 1.0))

    frame = model.part("frame")

    # Seat and back upholstery are mounted to, and slightly embedded into, the
    # welded tube frame so the static root part remains one supported assembly.
    frame.visual(
        _rounded_pad(0.500, 0.430, 0.058, 0.045, "seat_cushion"),
        origin=Origin(xyz=(0.0, 0.020, 0.492)),
        material=cushion,
        name="seat_cushion",
    )
    frame.visual(
        _back_pad(0.500, 0.520, 0.040, 0.045, "backrest_pad"),
        origin=Origin(xyz=(0.0, -0.245, 0.790)),
        material=webbing,
        name="backrest_pad",
    )
    frame.visual(
        Box((0.540, 0.410, 0.050)),
        origin=Origin(xyz=(0.0, 0.020, 0.438)),
        material=dark_frame,
        name="underseat_box",
    )
    frame.visual(
        Cylinder(radius=0.030, length=0.560),
        origin=Origin(xyz=(0.0, 0.250, 0.435), rpy=(0.0, pi / 2.0, 0.0)),
        material=powder_coat,
        name="front_seat_tube",
    )
    frame.visual(
        Cylinder(radius=0.032, length=0.610),
        origin=Origin(xyz=(0.0, -0.195, 0.310), rpy=(0.0, pi / 2.0, 0.0)),
        material=powder_coat,
        name="rear_axle_tube",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.485),
        origin=Origin(xyz=(0.295, 0.030, 0.430), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=powder_coat,
        name="side_rail_0",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.485),
        origin=Origin(xyz=(-0.295, 0.030, 0.430), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=powder_coat,
        name="side_rail_1",
    )

    for side, x in (("left", -0.300), ("right", 0.300)):
        side_sign = -1.0 if side == "left" else 1.0
        frame.visual(
            Cylinder(radius=0.029, length=0.730),
            origin=Origin(xyz=(x, -0.240, 0.775)),
            material=powder_coat,
            name=f"{side}_back_post",
        )
        frame.visual(
            Cylinder(radius=0.028, length=0.330),
            origin=Origin(xyz=(x, 0.245, 0.300)),
            material=powder_coat,
            name=f"{side}_front_leg",
        )
        frame.visual(
            Box((0.080, 0.090, 0.085)),
            origin=Origin(xyz=(x + side_sign * 0.030, -0.195, 0.310)),
            material=dark_frame,
            name=f"{side}_axle_block",
        )
        frame.visual(
            Cylinder(radius=0.046, length=0.040),
            origin=Origin(
                xyz=(x + side_sign * 0.064, -0.195, 0.310),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=metal,
            name=f"{side}_axle_boss",
        )
        frame.visual(
            Cylinder(radius=0.020, length=0.120),
            origin=Origin(
                xyz=(side_sign * 0.405, -0.195, 0.310),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=metal,
            name=f"{side}_axle_pin",
        )
        frame.visual(
            Cylinder(radius=0.024, length=0.170),
            origin=Origin(xyz=(x, -0.195, 0.375)),
            material=powder_coat,
            name=f"{side}_rear_dropout",
        )
        frame.visual(
            Box((0.065, 0.290, 0.050)),
            origin=Origin(xyz=(x, 0.020, 0.695)),
            material=dark_frame,
            name=f"{side}_arm_pad",
        )
        frame.visual(
            Cylinder(radius=0.020, length=0.250),
            origin=Origin(xyz=(x, 0.100, 0.575)),
            material=powder_coat,
            name=f"{side}_arm_support_0",
        )
        frame.visual(
            Cylinder(radius=0.020, length=0.250),
            origin=Origin(xyz=(x, -0.105, 0.575)),
            material=powder_coat,
            name=f"{side}_arm_support_1",
        )
        frame.visual(
            _mesh(
                tube_from_spline_points(
                    [
                        (x, 0.230, 0.455),
                        (x + side_sign * 0.020, 0.395, 0.375),
                        (x + side_sign * 0.075, 0.430, 0.385),
                    ],
                    radius=0.024,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
                f"{side}_caster_strut",
            ),
            material=powder_coat,
            name=f"{side}_caster_strut",
        )
        frame.visual(
            _mesh(
                tube_from_spline_points(
                    [
                        (x, 0.250, 0.420),
                        (x - side_sign * 0.160, 0.540, 0.310),
                        (x - side_sign * 0.170, 0.630, 0.240),
                    ],
                    radius=0.021,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
                f"{side}_footrest_hanger",
            ),
            material=powder_coat,
            name=f"{side}_footrest_hanger",
        )
        frame.visual(
            Box((0.215, 0.160, 0.030)),
            origin=Origin(xyz=(side_sign * 0.115, 0.675, 0.205)),
            material=foot_grip,
            name=f"{side}_footplate",
        )
        frame.visual(
            Cylinder(radius=0.020, length=0.120),
            origin=Origin(
                xyz=(side_sign * 0.190, 0.605, 0.220),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=powder_coat,
            name=f"{side}_footplate_hinge",
        )
        frame.visual(
            _mesh(
                tube_from_spline_points(
                    [
                        (x, -0.240, 1.110),
                        (x, -0.300, 1.170),
                        (x, -0.395, 1.175),
                    ],
                    radius=0.022,
                    samples_per_segment=12,
                    radial_segments=16,
                ),
                f"{side}_push_handle",
            ),
            material=powder_coat,
            name=f"{side}_push_handle",
        )
        frame.visual(
            Cylinder(radius=0.025, length=0.115),
            origin=Origin(xyz=(x, -0.435, 1.175), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"{side}_hand_grip",
        )

    frame.visual(
        Cylinder(radius=0.019, length=0.560),
        origin=Origin(xyz=(0.0, -0.245, 1.035), rpy=(0.0, pi / 2.0, 0.0)),
        material=powder_coat,
        name="back_cross_tube",
    )
    frame.visual(
        Cylinder(radius=0.021, length=0.550),
        origin=Origin(xyz=(0.0, 0.430, 0.385), rpy=(0.0, pi / 2.0, 0.0)),
        material=powder_coat,
        name="front_caster_crossbar",
    )

    left_rear_wheel = model.part("left_rear_wheel")
    _rear_wheel_visuals(left_rear_wheel, "left_rear_wheel", side_sign=-1.0, metal=metal, rubber=rubber)

    right_rear_wheel = model.part("right_rear_wheel")
    _rear_wheel_visuals(right_rear_wheel, "right_rear_wheel", side_sign=1.0, metal=metal, rubber=rubber)

    left_caster_yoke = model.part("left_caster_yoke")
    right_caster_yoke = model.part("right_caster_yoke")
    for side, yoke in (("left", left_caster_yoke), ("right", right_caster_yoke)):
        yoke.visual(
            Cylinder(radius=0.026, length=0.145),
            origin=Origin(xyz=(0.0, 0.0, -0.070)),
            material=metal,
            name="swivel_stem",
        )
        yoke.visual(
            Cylinder(radius=0.050, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.128)),
            material=dark_frame,
            name="bearing_collar",
        )
        yoke.visual(
            Box((0.130, 0.060, 0.034)),
            origin=Origin(xyz=(0.0, 0.0, -0.130)),
            material=dark_frame,
            name="fork_bridge",
        )
        yoke.visual(
            Box((0.026, 0.052, 0.154)),
            origin=Origin(xyz=(-0.055, 0.0, -0.223)),
            material=dark_frame,
            name="fork_tine_0",
        )
        yoke.visual(
            Box((0.026, 0.052, 0.154)),
            origin=Origin(xyz=(0.055, 0.0, -0.223)),
            material=dark_frame,
            name="fork_tine_1",
        )
        yoke.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(-0.051, 0.0, -0.255), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name="outer_axle_cap_0",
        )
        yoke.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(0.051, 0.0, -0.255), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name="outer_axle_cap_1",
        )
        yoke.visual(
            Cylinder(radius=0.010, length=0.112),
            origin=Origin(xyz=(0.0, 0.0, -0.255), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name="axle_pin",
        )

    left_caster_wheel = model.part("left_caster_wheel")
    _caster_wheel_visuals(left_caster_wheel, "left_caster_wheel", metal=metal, rubber=rubber)
    right_caster_wheel = model.part("right_caster_wheel")
    _caster_wheel_visuals(right_caster_wheel, "right_caster_wheel", metal=metal, rubber=rubber)

    model.articulation(
        "left_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.435, -0.195, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "right_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(0.435, -0.195, 0.310)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster_yoke,
        origin=Origin(xyz=(-0.285, 0.430, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster_yoke,
        origin=Origin(xyz=(0.285, 0.430, 0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )
    model.articulation(
        "left_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster_yoke,
        child=left_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "right_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster_yoke,
        child=right_caster_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    required_parts = (
        "frame",
        "left_rear_wheel",
        "right_rear_wheel",
        "left_caster_yoke",
        "right_caster_yoke",
        "left_caster_wheel",
        "right_caster_wheel",
    )
    for part_name in required_parts:
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    continuous_x = ("left_rear_spin", "right_rear_spin", "left_caster_spin", "right_caster_spin")
    continuous_z = ("left_caster_swivel", "right_caster_swivel")
    for joint_name in continuous_x:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_continuous_x",
            joint is not None
            and joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
        )
    for joint_name in continuous_z:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_continuous_z",
            joint is not None
            and joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
        )

    captured_axles = (
        ("frame", "left_rear_wheel", "left_axle_pin", "rim", 0.025),
        ("frame", "right_rear_wheel", "right_axle_pin", "rim", 0.025),
        ("left_caster_yoke", "left_caster_wheel", "axle_pin", "rim", 0.035),
        ("right_caster_yoke", "right_caster_wheel", "axle_pin", "rim", 0.035),
    )
    for parent_name, wheel_name, pin_elem, rim_elem, min_insert in captured_axles:
        ctx.allow_overlap(
            parent_name,
            wheel_name,
            elem_a=pin_elem,
            elem_b=rim_elem,
            reason="A real wheel axle pin is intentionally captured inside the rotating hub bore.",
        )
        ctx.expect_overlap(
            parent_name,
            wheel_name,
            axes="x",
            elem_a=pin_elem,
            elem_b=rim_elem,
            min_overlap=min_insert,
            name=f"{pin_elem}_retained_in_{wheel_name}",
        )
        ctx.expect_within(
            parent_name,
            wheel_name,
            axes="yz",
            inner_elem=pin_elem,
            outer_elem=rim_elem,
            margin=0.004,
            name=f"{pin_elem}_centered_in_{wheel_name}",
        )

    rear = object_model.get_part("left_rear_wheel")
    caster = object_model.get_part("left_caster_wheel")
    frame = object_model.get_part("frame")
    if rear is not None:
        aabb = ctx.part_world_aabb(rear)
        if aabb is not None:
            mins, maxs = aabb
            diameter = max(float(maxs[1] - mins[1]), float(maxs[2] - mins[2]))
            ctx.check("large_rear_wheel_diameter", 0.60 <= diameter <= 0.64, f"diameter={diameter}")
    if caster is not None:
        aabb = ctx.part_world_aabb(caster)
        if aabb is not None:
            mins, maxs = aabb
            diameter = max(float(maxs[1] - mins[1]), float(maxs[2] - mins[2]))
            ctx.check("small_front_caster_diameter", 0.18 <= diameter <= 0.21, f"diameter={diameter}")
    if frame is not None and rear is not None:
        frame_aabb = ctx.part_world_aabb(frame)
        rear_aabb = ctx.part_world_aabb(rear)
        if frame_aabb is not None and rear_aabb is not None:
            ctx.check(
                "backrest_above_rear_wheel",
                float(frame_aabb[1][2]) > float(rear_aabb[1][2]) + 0.45,
                f"frame={frame_aabb}, rear={rear_aabb}",
            )

    return ctx.report()


object_model = build_object_model()
