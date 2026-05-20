from __future__ import annotations

from math import pi, radians

import cadquery as cq
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
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _union_all(shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _hollow_tube(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    center: tuple[float, float, float],
):
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return tube.translate((center[0], center[1], center[2] - length / 2.0))


def _rounded_pad(size: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .box(*size)
        .edges("|Z")
        .fillet(0.035)
        .edges(">Z")
        .fillet(0.012)
        .edges("<Z")
        .fillet(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_knee_walker")

    frame_mat = model.material("powder_coated_black", rgba=(0.02, 0.022, 0.024, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    dark_metal_mat = model.material("dark_hardware", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    rim_mat = model.material("silver_wheel_rims", rgba=(0.72, 0.72, 0.70, 1.0))
    pad_mat = model.material("soft_blue_pad", rgba=(0.08, 0.18, 0.36, 1.0))

    # Wheel and tire meshes are authored around local X so they can spin on an
    # axle after the wheel joint frame is yawed 90 degrees.
    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.090,
            0.046,
            inner_radius=0.058,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.045),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        "rear_utility_tire",
    )
    rear_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.060,
            0.040,
            rim=WheelRim(inner_radius=0.041, flange_height=0.005, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.020,
                width=0.032,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.027, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0028, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "rear_spoked_rim",
    )
    front_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.075,
            0.042,
            inner_radius=0.048,
            carcass=TireCarcass(belt_width_ratio=0.68, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.003, count=22, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.004, radius=0.0025),
        ),
        "front_caster_tire",
    )
    front_rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.050,
            0.036,
            rim=WheelRim(inner_radius=0.034, flange_height=0.004, flange_thickness=0.0025),
            hub=WheelHub(radius=0.018, width=0.030, cap_style="domed"),
            face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "front_caster_rim",
    )

    chassis = model.part("chassis")

    def frame_box(name: str, size: tuple[float, float, float], center: tuple[float, float, float]):
        chassis.visual(Box(size), origin=Origin(xyz=center), material=frame_mat, name=name)

    frame_box("side_rail_0", (0.72, 0.036, 0.034), (0.000, 0.160, 0.180))
    frame_box("side_rail_1", (0.72, 0.036, 0.034), (0.000, -0.160, 0.180))
    frame_box("rear_crossmember", (0.060, 0.405, 0.040), (-0.345, 0.000, 0.175))
    frame_box("front_crossmember", (0.075, 0.355, 0.040), (0.340, 0.000, 0.176))
    frame_box("rear_spine", (0.170, 0.046, 0.036), (-0.265, 0.000, 0.205))
    frame_box("front_spine", (0.420, 0.052, 0.036), (0.130, 0.000, 0.205))
    frame_box("axle_drop_0", (0.050, 0.025, 0.135), (-0.345, 0.160, 0.120))
    frame_box("axle_drop_1", (0.050, 0.025, 0.135), (-0.345, -0.160, 0.120))
    frame_box("mast_clamp_0", (0.120, 0.025, 0.070), (-0.120, 0.0365, 0.245))
    frame_box("mast_clamp_1", (0.120, 0.025, 0.070), (-0.120, -0.0365, 0.245))
    frame_box("head_lug_0", (0.085, 0.018, 0.060), (0.410, 0.040, 0.212))
    frame_box("head_lug_1", (0.085, 0.018, 0.060), (0.410, -0.040, 0.212))
    chassis.visual(
        Cylinder(radius=0.032, length=0.300),
        origin=Origin(xyz=(-0.120, 0.000, 0.310)),
        material=metal_mat,
        name="height_sleeve",
    )
    chassis.visual(
        Cylinder(radius=0.034, length=0.125),
        origin=Origin(xyz=(0.410, 0.000, 0.220)),
        material=metal_mat,
        name="steering_head_tube",
    )
    chassis.visual(
        Cylinder(radius=0.009, length=0.500),
        origin=Origin(xyz=(-0.345, 0.000, 0.090), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal_mat,
        name="rear_axle",
    )
    chassis.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(-0.120, 0.060, 0.370), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal_mat,
        name="locking_pin",
    )

    knee_platform = model.part("knee_platform")
    knee_platform.visual(
        Cylinder(radius=0.0225, length=0.430),
        origin=Origin(xyz=(0.000, 0.000, -0.080)),
        material=metal_mat,
        name="inner_mast",
    )
    knee_platform.visual(
        Box((0.210, 0.060, 0.035)),
        origin=Origin(xyz=(-0.045, 0.000, 0.095)),
        material=dark_metal_mat,
        name="pad_support_plate",
    )
    knee_platform.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(xyz=(-0.120, 0.000, 0.085), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal_mat,
        name="cross_brace",
    )
    knee_platform.visual(
        mesh_from_cadquery(_rounded_pad((0.330, 0.185, 0.060)), "contoured_knee_pad", tolerance=0.001),
        origin=Origin(xyz=(-0.060, 0.000, 0.145)),
        material=pad_mat,
        name="cushion",
    )

    steering_fork = model.part("steering_fork")
    steering_fork.visual(
        Cylinder(radius=0.0215, length=0.900),
        origin=Origin(xyz=(0.000, 0.000, 0.355)),
        material=metal_mat,
        name="steering_column",
    )
    steering_fork.visual(
        Box((0.090, 0.050, 0.035)),
        origin=Origin(xyz=(0.055, 0.000, -0.085)),
        material=dark_metal_mat,
        name="lower_crown",
    )
    steering_fork.visual(
        Box((0.030, 0.042, 0.070)),
        origin=Origin(xyz=(0.095, 0.000, -0.052)),
        material=dark_metal_mat,
        name="fork_neck",
    )
    steering_fork.visual(
        Box((0.055, 0.390, 0.030)),
        origin=Origin(xyz=(0.095, 0.000, -0.040)),
        material=dark_metal_mat,
        name="fork_crossbar",
    )
    steering_fork.visual(
        Box((0.028, 0.020, 0.165)),
        origin=Origin(xyz=(0.095, 0.188, -0.098)),
        material=dark_metal_mat,
        name="fork_arm_0",
    )
    steering_fork.visual(
        Box((0.028, 0.020, 0.165)),
        origin=Origin(xyz=(0.095, -0.188, -0.098)),
        material=dark_metal_mat,
        name="fork_arm_1",
    )
    steering_fork.visual(
        Cylinder(radius=0.007, length=0.390),
        origin=Origin(xyz=(0.095, 0.000, -0.125), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal_mat,
        name="front_axle",
    )
    steering_fork.visual(
        Cylinder(radius=0.015, length=0.470),
        origin=Origin(xyz=(0.000, 0.000, 0.785), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="handlebar",
    )
    steering_fork.visual(
        Cylinder(radius=0.020, length=0.105),
        origin=Origin(xyz=(0.000, 0.280, 0.785), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rubber_mat,
        name="grip_0",
    )
    steering_fork.visual(
        Cylinder(radius=0.020, length=0.105),
        origin=Origin(xyz=(0.000, -0.280, 0.785), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=rubber_mat,
        name="grip_1",
    )

    def add_wheel_part(name: str, tire_mesh, rim_mesh, *, tire_material: Material, rim_material: Material):
        wheel = model.part(name)
        wheel.visual(tire_mesh, material=tire_material, name="tire")
        wheel.visual(rim_mesh, material=rim_material, name="rim")
        return wheel

    left_rear_wheel = add_wheel_part("left_rear_wheel", rear_tire_mesh, rear_rim_mesh, tire_material=rubber_mat, rim_material=rim_mat)
    right_rear_wheel = add_wheel_part("right_rear_wheel", rear_tire_mesh, rear_rim_mesh, tire_material=rubber_mat, rim_material=rim_mat)
    left_front_wheel = add_wheel_part("left_front_wheel", front_tire_mesh, front_rim_mesh, tire_material=rubber_mat, rim_material=rim_mat)
    right_front_wheel = add_wheel_part("right_front_wheel", front_tire_mesh, front_rim_mesh, tire_material=rubber_mat, rim_material=rim_mat)

    model.articulation(
        "knee_height_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=knee_platform,
        origin=Origin(xyz=(-0.120, 0.000, 0.405)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    model.articulation(
        "steer_yaw",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=steering_fork,
        origin=Origin(xyz=(0.410, 0.000, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-radians(40.0), upper=radians(40.0)),
    )
    model.articulation(
        "left_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=left_rear_wheel,
        origin=Origin(xyz=(-0.345, 0.232, 0.090), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "right_rear_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.345, -0.232, 0.090), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "left_front_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=left_front_wheel,
        origin=Origin(xyz=(0.095, 0.120, -0.125), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )
    model.articulation(
        "right_front_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=right_front_wheel,
        origin=Origin(xyz=(0.095, -0.120, -0.125), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    knee_platform = object_model.get_part("knee_platform")
    left_rear_wheel = object_model.get_part("left_rear_wheel")
    right_rear_wheel = object_model.get_part("right_rear_wheel")
    left_front_wheel = object_model.get_part("left_front_wheel")
    right_front_wheel = object_model.get_part("right_front_wheel")

    height_slide = object_model.get_articulation("knee_height_slide")
    steer = object_model.get_articulation("steer_yaw")

    ctx.allow_overlap(
        chassis,
        knee_platform,
        elem_a="height_sleeve",
        elem_b="inner_mast",
        reason=(
            "The height-adjustable mast is intentionally represented as a close "
            "sliding bearing inside the hollow sleeve."
        ),
    )
    ctx.allow_overlap(
        chassis,
        object_model.get_part("steering_fork"),
        elem_a="steering_head_tube",
        elem_b="steering_column",
        reason="The steering shaft is intentionally captured in the head-tube bearing.",
    )
    ctx.expect_overlap(
        object_model.get_part("steering_fork"),
        chassis,
        axes="z",
        elem_a="steering_column",
        elem_b="steering_head_tube",
        min_overlap=0.10,
        name="steering column remains captured in the head tube",
    )
    for wheel in (left_rear_wheel, right_rear_wheel):
        ctx.allow_overlap(
            chassis,
            wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The rear axle is intentionally captured through the wheel rim bearing bore.",
        )
        ctx.expect_overlap(
            wheel,
            chassis,
            axes="xz",
            elem_a="rim",
            elem_b="rear_axle",
            min_overlap=0.015,
            name=f"{wheel.name} is centered on the rear axle",
        )

    ctx.expect_within(
        knee_platform,
        chassis,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="height_sleeve",
        margin=0.001,
        name="knee mast stays inside the height sleeve",
    )
    ctx.expect_overlap(
        knee_platform,
        chassis,
        axes="z",
        elem_a="inner_mast",
        elem_b="height_sleeve",
        min_overlap=0.12,
        name="lower knee mast remains inserted",
    )

    for name in ("left_rear_spin", "right_rear_spin", "left_front_spin", "right_front_spin"):
        joint = object_model.get_articulation(name)
        ctx.check(
            f"{name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{name} has type {joint.articulation_type}",
        )

    limits = steer.motion_limits
    ctx.check(
        "steering range is about forty degrees each way",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + radians(40.0)) < 1e-6
        and abs(limits.upper - radians(40.0)) < 1e-6,
        details=f"limits={limits}",
    )

    rest_knee_z = ctx.part_world_position(knee_platform)[2]
    rest_front_y = ctx.part_world_position(left_front_wheel)[1]
    with ctx.pose({height_slide: 0.160}):
        ctx.expect_overlap(
            knee_platform,
            chassis,
            axes="z",
            elem_a="inner_mast",
            elem_b="height_sleeve",
            min_overlap=0.10,
            name="raised knee mast retains insertion",
        )
        raised_knee_z = ctx.part_world_position(knee_platform)[2]

    with ctx.pose({steer: radians(40.0)}):
        steered_front_y = ctx.part_world_position(left_front_wheel)[1]

    ctx.check(
        "height slide raises the knee platform",
        raised_knee_z > rest_knee_z + 0.14,
        details=f"rest_z={rest_knee_z}, raised_z={raised_knee_z}",
    )
    ctx.check(
        "front wheel pair follows steering yaw",
        steered_front_y > rest_front_y + 0.02,
        details=f"rest_y={rest_front_y}, steered_y={steered_front_y}",
    )

    return ctx.report()


object_model = build_object_model()
