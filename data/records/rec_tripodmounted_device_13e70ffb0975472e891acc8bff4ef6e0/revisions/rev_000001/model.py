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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_camera_mount")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.23, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    metal = model.material("metal", rgba=(0.62, 0.64, 0.67, 1.0))
    camera_black = model.material("camera_black", rgba=(0.10, 0.10, 0.11, 1.0))

    support = model.part("support")
    support.visual(
        Cylinder(radius=0.056, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=dark_grey,
        name="tripod_crown",
    )
    support.visual(
        Cylinder(radius=0.026, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.374)),
        material=dark_grey,
        name="crown_neck",
    )
    support.visual(
        Cylinder(radius=0.019, length=0.942),
        origin=Origin(xyz=(0.0, 0.0, 0.8855)),
        material=matte_black,
        name="center_column",
    )
    support.visual(
        Cylinder(radius=0.030, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 1.391)),
        material=dark_grey,
        name="top_receiver",
    )
    support.visual(
        Cylinder(radius=0.012, length=0.040),
        origin=Origin(
            xyz=(0.040, 0.0, 1.415),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="column_lock_knob",
    )
    support.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(
            xyz=(0.058, 0.0, 1.415),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="column_lock_pad",
    )

    hinge_radius = 0.012
    hinge_mount_size = (0.028, 0.024, 0.052)
    hinge_mount_center = 0.058
    hinge_radius_center = 0.084
    hinge_z = 0.306
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        support.visual(
            Box(hinge_mount_size),
            origin=Origin(
                xyz=(
                    hinge_mount_center * math.cos(angle),
                    hinge_mount_center * math.sin(angle),
                    hinge_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_grey,
            name=f"leg_{index}_hinge_lug",
        )

    support.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 1.48)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
    )

    deployed_leg_pitch = math.radians(65.0)

    def leg_axis_point(distance: float) -> tuple[float, float, float]:
        return (
            math.sin(deployed_leg_pitch) * distance,
            0.0,
            -math.cos(deployed_leg_pitch) * distance,
        )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=hinge_radius, length=0.030),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="hinge_knuckle",
        )
        leg.visual(
            Box((0.032, 0.024, 0.048)),
            origin=Origin(
                xyz=leg_axis_point(0.026),
                rpy=(0.0, -deployed_leg_pitch, 0.0),
            ),
            material=dark_grey,
            name="hinge_clamp",
        )
        leg.visual(
            Box((0.028, 0.012, 0.340)),
            origin=Origin(
                xyz=leg_axis_point(0.210),
                rpy=(0.0, -deployed_leg_pitch, 0.0),
            ),
            material=matte_black,
            name="upper_tube",
        )
        leg.visual(
            Box((0.022, 0.010, 0.380)),
            origin=Origin(
                xyz=leg_axis_point(0.500),
                rpy=(0.0, -deployed_leg_pitch, 0.0),
            ),
            material=dark_grey,
            name="lower_tube",
        )
        leg.visual(
            Box((0.055, 0.026, 0.032)),
            origin=Origin(xyz=(0.619, 0.0, -0.290)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.72, 0.04, 0.09)),
            mass=0.55,
            origin=Origin(xyz=(0.360, 0.0, -0.170)),
        )
        model.articulation(
            f"support_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=leg,
            origin=Origin(
                xyz=(
                    hinge_radius_center * math.cos(angle),
                    hinge_radius_center * math.sin(angle),
                    hinge_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.4,
                lower=0.0,
                upper=1.35,
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_grey,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.040, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=matte_black,
        name="pan_stage",
    )
    pan_head.visual(
        Box((0.094, 0.064, 0.026)),
        origin=Origin(xyz=(0.022, 0.0, 0.058)),
        material=dark_grey,
        name="tilt_bridge",
    )
    for side, y in (("left", 0.034), ("right", -0.034)):
        pan_head.visual(
            Box((0.034, 0.012, 0.136)),
            origin=Origin(xyz=(0.056, y, 0.120)),
            material=dark_grey,
            name=f"{side}_tilt_cheek",
        )
    pan_head.visual(
        Box((0.032, 0.086, 0.022)),
        origin=Origin(xyz=(0.032, 0.0, 0.182)),
        material=dark_grey,
        name="cheek_crossbar",
    )
    pan_head.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(
            xyz=(-0.014, 0.0, 0.072),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="pan_handle_socket",
    )
    pan_head.visual(
        Cylinder(radius=0.008, length=0.170),
        origin=Origin(
            xyz=(-0.094, 0.0, 0.072),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=matte_black,
        name="pan_handle",
    )
    pan_head.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(
            xyz=(-0.184, 0.0, 0.072),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="pan_handle_grip",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.26, 0.11, 0.18)),
        mass=1.0,
        origin=Origin(xyz=(-0.030, 0.0, 0.090)),
    )

    camera_mount = model.part("camera_mount")
    camera_mount.visual(
        Box((0.022, 0.056, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=metal,
        name="tilt_axle",
    )
    camera_mount.visual(
        Box((0.028, 0.052, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_grey,
        name="tilt_block",
    )
    camera_mount.visual(
        Box((0.128, 0.050, 0.010)),
        origin=Origin(xyz=(0.068, 0.0, 0.052)),
        material=metal,
        name="device_plate",
    )
    camera_mount.visual(
        Box((0.120, 0.070, 0.060)),
        origin=Origin(xyz=(0.074, 0.0, 0.084)),
        material=camera_black,
        name="camera_body",
    )
    camera_mount.visual(
        Box((0.080, 0.040, 0.018)),
        origin=Origin(xyz=(0.082, 0.0, 0.122)),
        material=camera_black,
        name="top_housing",
    )
    camera_mount.visual(
        Cylinder(radius=0.028, length=0.070),
        origin=Origin(
            xyz=(0.158, 0.0, 0.092),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=matte_black,
        name="lens_barrel",
    )
    camera_mount.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(
            xyz=(0.202, 0.0, 0.092),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber,
        name="lens_ring",
    )
    camera_mount.visual(
        Box((0.040, 0.016, 0.024)),
        origin=Origin(xyz=(0.016, 0.0, 0.110)),
        material=camera_black,
        name="rear_screen_housing",
    )
    camera_mount.inertial = Inertial.from_geometry(
        Box((0.25, 0.08, 0.15)),
        mass=1.2,
        origin=Origin(xyz=(0.090, 0.0, 0.075)),
    )

    model.articulation(
        "support_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.435)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0),
    )

    model.articulation(
        "pan_head_to_camera_mount",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_mount,
        origin=Origin(xyz=(0.056, 0.0, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=math.radians(-65.0),
            upper=math.radians(85.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    pan_head = object_model.get_part("pan_head")
    camera_mount = object_model.get_part("camera_mount")
    legs = [object_model.get_part(f"leg_{index}") for index in range(3)]
    leg_joints = [object_model.get_articulation(f"support_to_leg_{index}") for index in range(3)]
    pan_joint = object_model.get_articulation("support_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_camera_mount")

    ctx.expect_origin_gap(
        pan_head,
        support,
        axis="z",
        min_gap=1.35,
        max_gap=1.50,
        name="pan head sits high above the floor plane",
    )
    ctx.expect_gap(
        camera_mount,
        pan_head,
        axis="z",
        min_gap=0.030,
        max_gap=0.080,
        positive_elem="tilt_block",
        negative_elem="tilt_bridge",
        name="tilt block sits above the yoke bridge",
    )

    rest_foot_aabbs = [
        ctx.part_element_world_aabb(leg, elem="foot_pad")
        for leg in legs
    ]
    with ctx.pose({joint: 1.10 for joint in leg_joints}):
        folded_foot_aabbs = [
            ctx.part_element_world_aabb(leg, elem="foot_pad")
            for leg in legs
        ]
    for index, (rest_aabb, folded_aabb) in enumerate(zip(rest_foot_aabbs, folded_foot_aabbs)):
        rest_radius = None
        folded_radius = None
        if rest_aabb is not None:
            rest_center = (
                (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0,
                (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0,
                (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0,
            )
            rest_radius = math.hypot(rest_center[0], rest_center[1])
        if folded_aabb is not None:
            folded_center = (
                (folded_aabb[0][0] + folded_aabb[1][0]) / 2.0,
                (folded_aabb[0][1] + folded_aabb[1][1]) / 2.0,
                (folded_aabb[0][2] + folded_aabb[1][2]) / 2.0,
            )
            folded_radius = math.hypot(folded_center[0], folded_center[1])
        ctx.check(
            f"leg {index} folds upward toward the center column",
            rest_aabb is not None
            and folded_aabb is not None
            and folded_aabb[0][2] > rest_aabb[0][2] + 0.45
            and folded_radius is not None
            and rest_radius is not None
            and folded_radius < rest_radius - 0.075,
            details=(
                f"rest_aabb={rest_aabb}, folded_aabb={folded_aabb}, "
                f"rest_radius={rest_radius}, folded_radius={folded_radius}"
            ),
        )

    rest_camera_pos = ctx.part_world_position(camera_mount)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_camera_pos = ctx.part_world_position(camera_mount)
    ctx.check(
        "pan stage swings the device around the vertical axis",
        rest_camera_pos is not None
        and panned_camera_pos is not None
        and abs(panned_camera_pos[1]) > abs(rest_camera_pos[1]) + 0.030,
        details=f"rest={rest_camera_pos}, panned={panned_camera_pos}",
    )

    rest_aabb = ctx.part_world_aabb(camera_mount)
    with ctx.pose({tilt_joint: math.radians(45.0)}):
        tilted_aabb = ctx.part_world_aabb(camera_mount)
    ctx.check(
        "positive tilt raises the front of the device",
        rest_aabb is not None
        and tilted_aabb is not None
        and tilted_aabb[1][2] > rest_aabb[1][2] + 0.040,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
