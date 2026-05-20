from __future__ import annotations

from math import atan2, cos, pi, sin, sqrt

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
    tube_from_spline_points,
)


def _cylinder_between(part, p0, p1, *, radius, material, name):
    """Add a cylinder whose local +Z axis runs from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return
    nx, ny, nz = dx / length, dy / length, dz / length
    pitch = atan2(sqrt(nx * nx + ny * ny), nz)
    yaw = atan2(ny, nx)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _make_rear_wheel(part, *, prefix: str, outward: float, metal, rubber) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.266,
                0.036,
                rim=WheelRim(inner_radius=0.210, flange_height=0.007, flange_thickness=0.003),
                hub=WheelHub(
                    radius=0.044,
                    width=0.042,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=6, circle_diameter=0.050, hole_diameter=0.004),
                ),
                face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=16, thickness=0.0025, window_radius=0.016),
                bore=WheelBore(style="round", diameter=0.014),
            ),
            f"{prefix}_rim",
        ),
        material=metal,
        name="rim_spokes",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.305,
                0.046,
                inner_radius=0.268,
                tread=TireTread(style="circumferential", depth=0.0025, count=4),
                grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
                sidewall=TireSidewall(style="rounded", bulge=0.035),
            ),
            f"{prefix}_tire",
        ),
        material=rubber,
        name="tire",
    )

    # Wheelchair hand rim: a separate outer circular tube carried by short
    # standoffs, offset to the user's outside of each wheel.
    hoop_radius = 0.258
    hoop_x = outward * 0.043
    points = [
        (hoop_x, hoop_radius * cos(2.0 * pi * i / 48.0), hoop_radius * sin(2.0 * pi * i / 48.0))
        for i in range(48)
    ]
    part.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                points,
                radius=0.006,
                closed_spline=True,
                samples_per_segment=4,
                radial_segments=12,
            ),
            f"{prefix}_handrim",
        ),
        material=metal,
        name="handrim",
    )
    for i, angle in enumerate((0.0, pi / 3.0, 2.0 * pi / 3.0, pi, 4.0 * pi / 3.0, 5.0 * pi / 3.0)):
        _cylinder_between(
            part,
            (hoop_x, hoop_radius * cos(angle), hoop_radius * sin(angle)),
            (0.0, 0.045 * cos(angle), 0.045 * sin(angle)),
            radius=0.004,
            material=metal,
            name=f"handrim_standoff_{i}",
        )


def _make_caster_yoke(part, *, metal, dark) -> None:
    part.visual(
        Cylinder(radius=0.017, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=metal,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark,
        name="bearing_cap",
    )
    part.visual(
        Box((0.078, 0.080, 0.026)),
        origin=Origin(xyz=(-0.026, 0.0, -0.039)),
        material=dark,
        name="fork_crown",
    )
    for side, y in (("outer", 0.020), ("inner", -0.020)):
        part.visual(
            Box((0.034, 0.008, 0.116)),
            origin=Origin(xyz=(-0.040, y, -0.110)),
            material=metal,
            name=f"{side}_fork_plate",
        )
        part.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(-0.040, y * 1.18, -0.132), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=f"{side}_axle_cap",
        )


def _make_caster_wheel(part, *, prefix: str, metal, rubber) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.052,
                0.026,
                rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.002),
                hub=WheelHub(radius=0.018, width=0.024, cap_style="flat"),
                spokes=WheelSpokes(style="straight", count=5, thickness=0.002, window_radius=0.004),
                bore=WheelBore(style="round", diameter=0.007),
            ),
            f"{prefix}_rim",
        ),
        material=metal,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.074,
                0.032,
                inner_radius=0.053,
                tread=TireTread(style="circumferential", depth=0.002, count=2),
                sidewall=TireSidewall(style="rounded", bulge=0.04),
            ),
            f"{prefix}_tire",
        ),
        material=rubber,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_wheelchair")

    black_rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.026, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    blue_vinyl = model.material("blue_vinyl", rgba=(0.05, 0.16, 0.42, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.055, 0.055, 0.060, 1.0))

    frame = model.part("frame")

    # Low, wide support frame. X is forward, Y is left, Z is up.
    side_y = 0.245
    for side_name, y in (("left", side_y), ("right", -side_y)):
        _cylinder_between(frame, (-0.225, y, 0.310), (0.420, y, 0.235), radius=0.015, material=satin_metal, name=f"{side_name}_lower_rail")
        _cylinder_between(frame, (-0.225, y, 0.430), (0.270, y, 0.430), radius=0.015, material=satin_metal, name=f"{side_name}_seat_rail")
        _cylinder_between(frame, (0.270, y, 0.430), (0.420, y, 0.235), radius=0.014, material=satin_metal, name=f"{side_name}_front_strut")
        _cylinder_between(frame, (-0.225, y, 0.310), (-0.245, y, 0.975), radius=0.014, material=satin_metal, name=f"{side_name}_back_cane")
        _cylinder_between(frame, (-0.245, y, 0.975), (-0.385, y, 0.975), radius=0.014, material=satin_metal, name=f"{side_name}_push_handle_bar")
        frame.visual(
            Cylinder(radius=0.019, length=0.100),
            origin=Origin(xyz=(-0.430, y, 0.975), rpy=(0.0, pi / 2.0, 0.0)),
            material=black_rubber,
            name=f"{side_name}_push_grip",
        )
        _cylinder_between(frame, (-0.060, y, 0.420), (-0.225, y, 0.310), radius=0.012, material=satin_metal, name=f"{side_name}_rear_brace")
        _cylinder_between(frame, (0.330, y, 0.340), (0.555, y * 0.58, 0.123), radius=0.013, material=satin_metal, name=f"{side_name}_footrest_hanger")
        _cylinder_between(frame, (0.420, y, 0.205), (0.420, y, 0.235), radius=0.021, material=dark_metal, name=f"{side_name}_caster_socket")
        frame.visual(
            Box((0.165, 0.145, 0.026)),
            origin=Origin(xyz=(0.590, y * 0.58, 0.110), rpy=(0.0, 0.10, 0.0)),
            material=black_plastic,
            name=f"{side_name}_foot_plate",
        )

    for name, x, z, radius in (
        ("rear_axle_tube", -0.225, 0.310, 0.018),
        ("front_seat_tube", 0.270, 0.430, 0.014),
        ("mid_cross_tube", 0.030, 0.405, 0.012),
        ("back_cross_tube", -0.250, 0.785, 0.012),
    ):
        _cylinder_between(frame, (x, -0.270, z), (x, 0.270, z), radius=radius, material=satin_metal, name=name)

    _cylinder_between(frame, (-0.150, -0.205, 0.335), (0.270, 0.205, 0.415), radius=0.010, material=satin_metal, name="folding_brace_0")
    _cylinder_between(frame, (-0.150, 0.205, 0.335), (0.270, -0.205, 0.415), radius=0.010, material=satin_metal, name="folding_brace_1")

    # Axle collars stop just short of the wheel hubs so the spin joints read as
    # supported without burying the wheel visuals in the frame.
    _cylinder_between(frame, (-0.225, 0.258, 0.310), (-0.225, 0.299, 0.310), radius=0.022, material=dark_metal, name="left_axle_collar")
    _cylinder_between(frame, (-0.225, -0.258, 0.310), (-0.225, -0.299, 0.310), radius=0.022, material=dark_metal, name="right_axle_collar")

    frame.visual(
        Box((0.480, 0.455, 0.055)),
        origin=Origin(xyz=(0.030, 0.0, 0.455)),
        material=blue_vinyl,
        name="seat_cushion",
    )
    frame.visual(
        Box((0.060, 0.455, 0.500)),
        origin=Origin(xyz=(-0.270, 0.0, 0.715), rpy=(0.0, -0.08, 0.0)),
        material=blue_vinyl,
        name="backrest_pad",
    )
    frame.visual(
        Box((0.060, 0.470, 0.040)),
        origin=Origin(xyz=(-0.245, 0.0, 0.475)),
        material=black_plastic,
        name="backrest_base_band",
    )

    left_rear_wheel = model.part("left_rear_wheel")
    _make_rear_wheel(left_rear_wheel, prefix="left_rear_wheel", outward=1.0, metal=satin_metal, rubber=black_rubber)
    right_rear_wheel = model.part("right_rear_wheel")
    _make_rear_wheel(right_rear_wheel, prefix="right_rear_wheel", outward=-1.0, metal=satin_metal, rubber=black_rubber)

    left_caster = model.part("left_caster")
    _make_caster_yoke(left_caster, metal=satin_metal, dark=dark_metal)
    right_caster = model.part("right_caster")
    _make_caster_yoke(right_caster, metal=satin_metal, dark=dark_metal)

    left_caster_wheel = model.part("left_caster_wheel")
    _make_caster_wheel(left_caster_wheel, prefix="left_caster_wheel", metal=satin_metal, rubber=black_rubber)
    right_caster_wheel = model.part("right_caster_wheel")
    _make_caster_wheel(right_caster_wheel, prefix="right_caster_wheel", metal=satin_metal, rubber=black_rubber)

    wheel_spin_limits = MotionLimits(effort=6.0, velocity=18.0)
    model.articulation(
        "left_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.225, 0.375, 0.310), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=wheel_spin_limits,
    )
    model.articulation(
        "right_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.225, -0.375, 0.310), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=wheel_spin_limits,
    )

    caster_swivel_limits = MotionLimits(effort=2.0, velocity=8.0)
    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_caster,
        origin=Origin(xyz=(0.420, 0.245, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=caster_swivel_limits,
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_caster,
        origin=Origin(xyz=(0.420, -0.245, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=caster_swivel_limits,
    )

    model.articulation(
        "left_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=left_caster,
        child=left_caster_wheel,
        origin=Origin(xyz=(-0.040, 0.0, -0.132), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=wheel_spin_limits,
    )
    model.articulation(
        "right_caster_spin",
        ArticulationType.CONTINUOUS,
        parent=right_caster,
        child=right_caster_wheel,
        origin=Origin(xyz=(-0.040, 0.0, -0.132), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=wheel_spin_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_rear = object_model.get_part("left_rear_wheel")
    right_rear = object_model.get_part("right_rear_wheel")
    left_caster = object_model.get_part("left_caster")
    left_caster_wheel = object_model.get_part("left_caster_wheel")

    for joint_name in (
        "left_rear_spin",
        "right_rear_spin",
        "left_caster_swivel",
        "right_caster_swivel",
        "left_caster_spin",
        "right_caster_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_continuous",
            joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{joint_name} should be a continuous wheelchair wheel/caster joint.",
        )

    ctx.expect_origin_distance(left_rear, right_rear, axes="y", min_dist=0.70, name="wide rear wheel track")
    ctx.expect_origin_gap(left_caster, frame, axis="x", min_gap=0.30, name="casters sit forward of rear axle frame")
    ctx.expect_overlap(left_caster_wheel, left_caster, axes="z", min_overlap=0.050, name="caster fork surrounds wheel vertically")

    frame_aabb = ctx.part_world_aabb(frame)
    left_wheel_aabb = ctx.part_world_aabb(left_rear)
    if frame_aabb is not None and left_wheel_aabb is not None:
        frame_mins, frame_maxs = frame_aabb
        wheel_mins, wheel_maxs = left_wheel_aabb
        overall_height = max(frame_maxs[2], wheel_maxs[2]) - min(frame_mins[2], wheel_mins[2])
        rear_diameter = wheel_maxs[2] - wheel_mins[2]
        ctx.check("low wheelchair stance", frame_maxs[2] <= 1.04 and overall_height <= 1.06, f"height={overall_height}")
        ctx.check("large rear wheels", 0.58 <= rear_diameter <= 0.64, f"rear_diameter={rear_diameter}")

    return ctx.report()


object_model = build_object_model()
