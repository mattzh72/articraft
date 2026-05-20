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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_mounted_device")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.11, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.07, 0.08, 0.09, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.20, 0.32, 0.42, 0.92))

    crown_height = 1.28

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.080, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, crown_height + 0.066)),
        material=graphite,
        name="crown_top_plate",
    )
    crown.visual(
        Cylinder(radius=0.058, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, crown_height + 0.006)),
        material=graphite,
        name="crown_core",
    )
    crown.visual(
        Cylinder(radius=0.041, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, crown_height + 0.090)),
        material=anodized_black,
        name="pan_bearing_coller",
    )
    crown.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, crown_height + 0.125)),
        material=aluminum,
        name="pan_thrust_ring",
    )
    for index, azimuth in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        cx = math.cos(azimuth)
        sy = math.sin(azimuth)
        crown.visual(
            Box((0.040, 0.028, 0.040)),
            origin=Origin(
                xyz=(cx * 0.076, sy * 0.076, crown_height + 0.010),
                rpy=(0.0, 0.0, azimuth),
            ),
            material=graphite,
            name=f"leg_lug_{index}",
        )
        crown.visual(
            Box((0.060, 0.022, 0.028)),
            origin=Origin(
                xyz=(cx * 0.044, sy * 0.044, crown_height - 0.010),
                rpy=(0.0, 0.0, azimuth),
            ),
            material=graphite,
            name=f"crown_spider_{index}",
        )
    crown.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.22)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, crown_height + 0.04)),
    )

    leg_pitch = math.radians(155.0)
    leg_upper_axis_x = math.sin(leg_pitch)
    leg_upper_axis_z = math.cos(leg_pitch)
    for index, azimuth in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.044, 0.024, 0.050)),
            origin=Origin(xyz=(0.018, 0.0, -0.026)),
            material=graphite,
            name="leg_shoulder",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.540),
            origin=Origin(xyz=(0.122, 0.0, -0.270), rpy=(0.0, leg_pitch, 0.0)),
            material=matte_black,
            name="upper_tube",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.480),
            origin=Origin(xyz=(0.257, 0.0, -0.575), rpy=(0.0, leg_pitch, 0.0)),
            material=matte_black,
            name="lower_tube",
        )
        leg.visual(
            Sphere(radius=0.018),
            origin=Origin(
                xyz=(
                    0.257 + leg_upper_axis_x * 0.240,
                    0.0,
                    -0.575 + leg_upper_axis_z * 0.240,
                )
            ),
            material=rubber,
            name="foot_tip",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.16, 0.06, 0.92)),
            mass=0.62,
            origin=Origin(xyz=(0.230, 0.0, -0.430)),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.052, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=anodized_black,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.030, length=0.078),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=anodized_black,
        name="pan_column",
    )
    pan_head.visual(
        Box((0.060, 0.106, 0.028)),
        origin=Origin(xyz=(-0.010, 0.0, 0.122)),
        material=graphite,
        name="tilt_saddle",
    )
    pan_head.visual(
        Box((0.018, 0.014, 0.118)),
        origin=Origin(xyz=(-0.030, -0.056, 0.182)),
        material=graphite,
        name="left_support",
    )
    pan_head.visual(
        Box((0.018, 0.014, 0.118)),
        origin=Origin(xyz=(-0.030, 0.056, 0.182)),
        material=graphite,
        name="right_support",
    )
    pan_head.visual(
        Box((0.028, 0.132, 0.020)),
        origin=Origin(xyz=(-0.038, 0.0, 0.242)),
        material=graphite,
        name="support_bridge",
    )
    pan_head.visual(
        Box((0.026, 0.022, 0.026)),
        origin=Origin(xyz=(-0.028, -0.018, 0.138)),
        material=graphite,
        name="handle_boss",
    )
    pan_head.visual(
        Cylinder(radius=0.007, length=0.240),
        origin=Origin(xyz=(-0.140, -0.018, 0.148), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="pan_handle",
    )
    pan_head.visual(
        Cylinder(radius=0.011, length=0.090),
        origin=Origin(xyz=(-0.270, -0.018, 0.148), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="pan_grip",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.36, 0.16, 0.28)),
        mass=1.2,
        origin=Origin(xyz=(-0.07, 0.0, 0.130)),
    )

    tilt_stage = model.part("tilt_stage")
    tilt_stage.visual(
        Cylinder(radius=0.009, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="tilt_axle",
    )
    tilt_stage.visual(
        Box((0.084, 0.058, 0.042)),
        origin=Origin(xyz=(0.028, 0.0, 0.000)),
        material=graphite,
        name="tilt_body",
    )
    tilt_stage.visual(
        Box((0.114, 0.064, 0.010)),
        origin=Origin(xyz=(0.030, 0.0, 0.024)),
        material=anodized_black,
        name="device_plate",
    )
    tilt_stage.visual(
        Box((0.038, 0.016, 0.012)),
        origin=Origin(xyz=(0.000, 0.0, -0.022)),
        material=graphite,
        name="tilt_knuckle",
    )
    tilt_stage.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(-0.012, -0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_trunnion",
    )
    tilt_stage.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(-0.012, 0.039, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_trunnion",
    )
    tilt_stage.inertial = Inertial.from_geometry(
        Box((0.13, 0.08, 0.07)),
        mass=0.55,
        origin=Origin(xyz=(0.032, 0.0, 0.016)),
    )

    device = model.part("device")
    device.visual(
        Box((0.040, 0.028, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, 0.006)),
        material=anodized_black,
        name="mounting_foot",
    )
    device.visual(
        Box((0.156, 0.078, 0.102)),
        origin=Origin(xyz=(0.004, 0.0, 0.063)),
        material=matte_black,
        name="camera_body",
    )
    device.visual(
        Box((0.050, 0.050, 0.028)),
        origin=Origin(xyz=(-0.036, 0.0, 0.110)),
        material=matte_black,
        name="viewfinder_housing",
    )
    device.visual(
        Cylinder(radius=0.033, length=0.082),
        origin=Origin(xyz=(0.122, 0.0, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="lens_barrel",
    )
    device.visual(
        Cylinder(radius=0.027, length=0.016),
        origin=Origin(xyz=(0.171, 0.0, 0.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens_front",
    )
    device.visual(
        Box((0.042, 0.012, 0.020)),
        origin=Origin(xyz=(-0.052, 0.045, 0.078)),
        material=matte_black,
        name="side_grip",
    )
    device.inertial = Inertial.from_geometry(
        Box((0.22, 0.09, 0.14)),
        mass=0.95,
        origin=Origin(xyz=(0.030, 0.0, 0.070)),
    )

    for index, azimuth in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=f"leg_{index}",
            origin=Origin(
                xyz=(
                    math.cos(azimuth) * 0.106,
                    math.sin(azimuth) * 0.106,
                    crown_height + 0.010,
                ),
                rpy=(0.0, 0.0, azimuth),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.6,
                lower=0.0,
                upper=math.radians(78.0),
            ),
        )

    model.articulation(
        "crown_to_pan",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, crown_height + 0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.4),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=math.radians(-70.0),
            upper=math.radians(90.0),
        ),
    )
    model.articulation(
        "stage_to_device",
        ArticulationType.FIXED,
        parent=tilt_stage,
        child=device,
        origin=Origin(xyz=(0.055, 0.0, 0.029)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pan_head = object_model.get_part("pan_head")
    tilt_stage = object_model.get_part("tilt_stage")
    device = object_model.get_part("device")
    leg_0 = object_model.get_part("leg_0")

    crown_to_pan = object_model.get_articulation("crown_to_pan")
    pan_to_tilt = object_model.get_articulation("pan_to_tilt")
    crown_to_leg_0 = object_model.get_articulation("crown_to_leg_0")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            0.5 * (lower[0] + upper[0]),
            0.5 * (lower[1] + upper[1]),
            0.5 * (lower[2] + upper[2]),
        )

    ctx.expect_contact(
        device,
        tilt_stage,
        elem_a="mounting_foot",
        elem_b="device_plate",
        name="device mounting foot sits on the tilt plate",
    )
    ctx.expect_contact(
        pan_head,
        tilt_stage,
        elem_a="left_support",
        elem_b="left_trunnion",
        name="left fork support captures the left trunnion",
    )
    ctx.expect_contact(
        pan_head,
        tilt_stage,
        elem_a="right_support",
        elem_b="right_trunnion",
        name="right fork support captures the right trunnion",
    )

    rest_foot = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_tip"))
    with ctx.pose({crown_to_leg_0: math.radians(70.0)}):
        folded_foot = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_tip"))
    ctx.check(
        "front leg folds upward toward the crown",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[2] > rest_foot[2] + 0.60
        and folded_foot[0] > rest_foot[0] + 0.35,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    rest_handle = aabb_center(ctx.part_element_world_aabb(pan_head, elem="pan_handle"))
    with ctx.pose({crown_to_pan: math.pi / 2.0}):
        panned_handle = aabb_center(ctx.part_element_world_aabb(pan_head, elem="pan_handle"))
    ctx.check(
        "pan head yaws around the vertical axis",
        rest_handle is not None
        and panned_handle is not None
        and abs(panned_handle[1] - rest_handle[1]) > 0.12
        and abs(panned_handle[2] - rest_handle[2]) < 0.01,
        details=f"rest_handle={rest_handle}, panned_handle={panned_handle}",
    )

    rest_device_pos = ctx.part_world_position(device)
    with ctx.pose({pan_to_tilt: math.radians(40.0)}):
        tilted_device_pos = ctx.part_world_position(device)
    ctx.check(
        "device tilts upward about the horizontal stage axis",
        rest_device_pos is not None
        and tilted_device_pos is not None
        and tilted_device_pos[2] > rest_device_pos[2] + 0.015,
        details=f"rest_device_pos={rest_device_pos}, tilted_device_pos={tilted_device_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
