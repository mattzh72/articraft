from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    rounded_rect_profile,
)


WHEEL_RADIUS = 0.095
WHEEL_WIDTH = 0.052
FRONT_WHEEL_Y = 0.245
REAR_WHEEL_Y = 0.310
FRONT_AXLE_X = 0.50
REAR_AXLE_X = -0.42
AXLE_Z = WHEEL_RADIUS
KNEE_PIVOT = (-0.07, 0.0, 0.560)


def _wheel_visuals(part, prefix: str, wheel_metal, rubber) -> None:
    """Add a small pneumatic scooter wheel whose spin axis is the part's +Y."""
    wheel_origin = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.071,
                WHEEL_WIDTH * 0.74,
                rim=WheelRim(inner_radius=0.047, flange_height=0.006, flange_thickness=0.0025),
                hub=WheelHub(
                    radius=0.022,
                    width=WHEEL_WIDTH * 0.64,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.003),
                ),
                face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.007),
                bore=WheelBore(style="round", diameter=0.010),
            ),
            f"{prefix}_rim",
        ),
        origin=wheel_origin,
        material=wheel_metal,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=0.073,
                tread=TireTread(style="circumferential", depth=0.003, count=4),
                grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
            ),
            f"{prefix}_tire",
        ),
        origin=wheel_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.078),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_metal,
        name="bearing_sleeve",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    frame_paint = model.material("satin_blue_frame", rgba=(0.05, 0.22, 0.45, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    wheel_metal = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    pad_vinyl = model.material("black_vinyl_pad", rgba=(0.06, 0.055, 0.050, 1.0))

    frame = model.part("frame")
    # Low rectangular-tube chassis, with a visible split rear axle: each rear
    # wheel is carried by its own short outboard stub rather than by one
    # uninterrupted shaft.
    frame.visual(
        Box((0.87, 0.055, 0.045)),
        origin=Origin(xyz=(0.030, 0.0, 0.235)),
        material=frame_paint,
        name="main_spine",
    )
    frame.visual(
        Box((0.080, 0.540, 0.040)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, 0.145)),
        material=frame_paint,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.070, 0.075, 0.130)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, 0.190)),
        material=frame_paint,
        name="rear_drop_link",
    )
    for side_index, side in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.070, 0.116, 0.030)),
            origin=Origin(xyz=(REAR_AXLE_X, side * 0.223, AXLE_Z)),
            material=dark_steel,
            name=f"rear_half_axle_{side_index}",
        )
        frame.visual(
            Box((0.060, 0.012, 0.074)),
            origin=Origin(xyz=(REAR_AXLE_X, side * 0.260, AXLE_Z)),
            material=dark_steel,
            name=f"rear_dropout_{side_index}",
        )

    frame.visual(
        Cylinder(radius=0.045, length=0.220),
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, 0.275)),
        material=dark_steel,
        name="head_tube",
    )

    # Knee support post and side-supported tilt bracket.
    frame.visual(
        Cylinder(radius=0.023, length=0.300),
        origin=Origin(xyz=(-0.070, 0.0, 0.385)),
        material=frame_paint,
        name="knee_post",
    )
    frame.visual(
        Box((0.170, 0.270, 0.028)),
        origin=Origin(xyz=(-0.070, 0.0, 0.505)),
        material=frame_paint,
        name="tilt_bracket_base",
    )
    frame.visual(
        Box((0.130, 0.025, 0.110)),
        origin=Origin(xyz=(-0.070, -0.130, 0.555)),
        material=frame_paint,
        name="pivot_side_0",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(-0.070, -0.141, 0.560), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_boss_0",
    )
    frame.visual(
        Box((0.130, 0.025, 0.110)),
        origin=Origin(xyz=(-0.070, 0.130, 0.555)),
        material=frame_paint,
        name="pivot_side_1",
    )
    frame.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(-0.070, 0.141, 0.560), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_boss_1",
    )

    steering = model.part("steering")
    steering.visual(
        Cylinder(radius=0.024, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=dark_steel,
        name="steering_stem",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.770)),
        material=dark_steel,
        name="handlebar_riser",
    )
    steering.visual(
        Box((0.090, 0.380, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=frame_paint,
        name="fork_bridge",
    )
    for side_index, side in enumerate((-1.0, 1.0)):
        steering.visual(
            Box((0.050, 0.020, 0.140)),
            origin=Origin(xyz=(0.0, side * 0.195, 0.060)),
            material=frame_paint,
            name=f"fork_leg_{side_index}",
        )
        steering.visual(
            Box((0.038, 0.012, 0.030)),
            origin=Origin(xyz=(0.0, side * 0.205, 0.0)),
            material=dark_steel,
            name=f"front_axle_stub_{side_index}",
        )
    steering.visual(
        Cylinder(radius=0.016, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.805), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handlebar",
    )
    for side_index, side in enumerate((-1.0, 1.0)):
        steering.visual(
            Cylinder(radius=0.021, length=0.105),
            origin=Origin(xyz=(0.0, side * 0.330, 0.805), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{side_index}",
        )

    knee_platform = model.part("knee_platform")
    knee_platform.visual(
        Cylinder(radius=0.018, length=0.292),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_barrel",
    )
    knee_platform.visual(
        Box((0.130, 0.085, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_steel,
        name="support_web",
    )
    knee_platform.visual(
        Box((0.340, 0.180, 0.016)),
        origin=Origin(xyz=(0.030, 0.0, 0.060)),
        material=dark_steel,
        name="pad_plate",
    )
    knee_platform.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.390, 0.215, 0.045, corner_segments=10),
                0.046,
                center=True,
            ),
            "rounded_knee_pad",
        ),
        origin=Origin(xyz=(0.035, 0.0, 0.090)),
        material=pad_vinyl,
        name="knee_pad",
    )

    front_wheel_0 = model.part("front_wheel_0")
    front_wheel_1 = model.part("front_wheel_1")
    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_1 = model.part("rear_wheel_1")
    for wheel_part, prefix in (
        (front_wheel_0, "front_wheel_0"),
        (front_wheel_1, "front_wheel_1"),
        (rear_wheel_0, "rear_wheel_0"),
        (rear_wheel_1, "rear_wheel_1"),
    ):
        _wheel_visuals(wheel_part, prefix, wheel_metal, rubber)

    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering,
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, AXLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_spin_0",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_wheel_0,
        origin=Origin(xyz=(0.0, -FRONT_WHEEL_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=28.0),
    )
    model.articulation(
        "front_spin_1",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_wheel_1,
        origin=Origin(xyz=(0.0, FRONT_WHEEL_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=28.0),
    )
    model.articulation(
        "rear_spin_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_0,
        origin=Origin(xyz=(REAR_AXLE_X, -REAR_WHEEL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=28.0),
    )
    model.articulation(
        "rear_spin_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_1,
        origin=Origin(xyz=(REAR_AXLE_X, REAR_WHEEL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=28.0),
    )
    model.articulation(
        "knee_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=knee_platform,
        origin=Origin(xyz=KNEE_PIVOT),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.4, lower=-0.28, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    steering = object_model.get_part("steering")
    knee_platform = object_model.get_part("knee_platform")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    steering_yaw = object_model.get_articulation("steering_yaw")
    knee_tilt = object_model.get_articulation("knee_tilt")

    ctx.allow_overlap(
        frame,
        steering,
        elem_a="head_tube",
        elem_b="steering_stem",
        reason="The steering stem intentionally passes through the fixed head tube as a yaw bearing.",
    )
    ctx.expect_within(
        steering,
        frame,
        axes="xy",
        inner_elem="steering_stem",
        outer_elem="head_tube",
        margin=0.0,
        name="steering stem is centered in head tube",
    )
    ctx.expect_overlap(
        steering,
        frame,
        axes="z",
        elem_a="steering_stem",
        elem_b="head_tube",
        min_overlap=0.16,
        name="steering stem remains retained in head tube",
    )
    for wheel_name, axle_name in (
        ("front_wheel_0", "front_axle_stub_0"),
        ("front_wheel_1", "front_axle_stub_1"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            wheel,
            steering,
            elem_a="bearing_sleeve",
            elem_b=axle_name,
            reason="The wheel bearing sleeve intentionally surrounds the fixed front axle stub.",
        )
        ctx.expect_within(
            wheel,
            steering,
            axes="xz",
            inner_elem="bearing_sleeve",
            outer_elem=axle_name,
            margin=0.0,
            name=f"{wheel_name} bearing is centered on its front axle",
        )
        ctx.expect_overlap(
            wheel,
            steering,
            axes="y",
            elem_a="bearing_sleeve",
            elem_b=axle_name,
            min_overlap=0.004,
            name=f"{wheel_name} bearing remains captured on front axle",
        )
    for wheel_name, axle_name in (
        ("rear_wheel_0", "rear_half_axle_0"),
        ("rear_wheel_1", "rear_half_axle_1"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a=axle_name,
            elem_b="bearing_sleeve",
            reason="The wheel bearing sleeve intentionally surrounds the fixed split rear axle stub.",
        )
        ctx.expect_within(
            wheel,
            frame,
            axes="xz",
            inner_elem="bearing_sleeve",
            outer_elem=axle_name,
            margin=0.0,
            name=f"{wheel_name} bearing is centered on its rear half axle",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="y",
            elem_a="bearing_sleeve",
            elem_b=axle_name,
            min_overlap=0.006,
            name=f"{wheel_name} bearing remains captured on rear half axle",
        )

    for joint_name in ("front_spin_0", "front_spin_1", "rear_spin_0", "rear_spin_1"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_is_continuous",
            joint is not None and joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} should be a continuous wheel-spin joint.",
        )

    with ctx.pose({steering_yaw: 0.0}):
        straight_pos = ctx.part_world_position(front_wheel_1)
    with ctx.pose({steering_yaw: 0.50}):
        yawed_pos = ctx.part_world_position(front_wheel_1)
    ctx.check(
        "front steering assembly yaws",
        straight_pos is not None
        and yawed_pos is not None
        and abs(yawed_pos[0] - straight_pos[0]) > 0.060,
        details=f"straight={straight_pos}, yawed={yawed_pos}",
    )

    with ctx.pose({knee_tilt: -0.25}):
        lower_aabb = ctx.part_element_world_aabb(knee_platform, elem="knee_pad")
    with ctx.pose({knee_tilt: 0.25}):
        upper_aabb = ctx.part_element_world_aabb(knee_platform, elem="knee_pad")
    if lower_aabb is not None and upper_aabb is not None:
        lower_center_x = (lower_aabb[0][0] + lower_aabb[1][0]) * 0.5
        upper_center_x = (upper_aabb[0][0] + upper_aabb[1][0]) * 0.5
    else:
        lower_center_x = upper_center_x = None
    ctx.check(
        "knee platform tilts on transverse pivot",
        lower_center_x is not None
        and upper_center_x is not None
        and abs(upper_center_x - lower_center_x) > 0.025,
        details=f"lower_center_x={lower_center_x}, upper_center_x={upper_center_x}",
    )
    ctx.allow_overlap(
        frame,
        knee_platform,
        elem_a="pivot_side_0",
        elem_b="pivot_barrel",
        reason="The knee platform pivot barrel intentionally passes through the side bracket bore proxy.",
    )
    ctx.allow_overlap(
        frame,
        knee_platform,
        elem_a="pivot_side_1",
        elem_b="pivot_barrel",
        reason="The knee platform pivot barrel intentionally passes through the side bracket bore proxy.",
    )
    ctx.allow_overlap(
        frame,
        knee_platform,
        elem_a="pivot_boss_0",
        elem_b="pivot_barrel",
        reason="The dark pivot boss is a bearing cap around the rotating knee platform barrel.",
    )
    ctx.expect_overlap(
        knee_platform,
        frame,
        axes="xyz",
        elem_a="pivot_barrel",
        elem_b="pivot_boss_0",
        min_overlap=0.008,
        name="pivot barrel is captured by lower boss",
    )
    ctx.allow_overlap(
        frame,
        knee_platform,
        elem_a="pivot_boss_1",
        elem_b="pivot_barrel",
        reason="The dark pivot boss is a bearing cap around the rotating knee platform barrel.",
    )
    ctx.expect_overlap(
        knee_platform,
        frame,
        axes="xyz",
        elem_a="pivot_barrel",
        elem_b="pivot_boss_1",
        min_overlap=0.008,
        name="pivot barrel is captured by upper boss",
    )
    ctx.expect_overlap(
        knee_platform,
        frame,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="pivot_side_0",
        min_overlap=0.020,
        name="pivot barrel is aligned with side bracket",
    )
    ctx.expect_overlap(
        knee_platform,
        frame,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="pivot_side_1",
        min_overlap=0.020,
        name="pivot barrel is aligned with opposite side bracket",
    )

    return ctx.report()


object_model = build_object_model()
