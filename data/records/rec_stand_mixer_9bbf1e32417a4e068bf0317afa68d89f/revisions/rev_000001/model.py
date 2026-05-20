from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="streamlined_stand_mixer")

    body_enamel = model.material("body_enamel", rgba=(0.93, 0.93, 0.90, 1.0))
    stainless = model.material("stainless", rgba=(0.86, 0.88, 0.90, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))

    def yz_section(
        x: float,
        *,
        width: float,
        height: float,
        radius: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_center)
            for z, y in rounded_rect_profile(
                height,
                width,
                radius,
                corner_segments=5,
            )
        ]

    base = model.part("base")

    base_geom = ExtrudeGeometry(
        rounded_rect_profile(0.27, 0.18, 0.048, corner_segments=5),
        0.03,
    ).translate(0.015, 0.0, 0.015)
    base_geom.merge(
        BoxGeometry((0.116, 0.094, 0.012)).translate(0.088, 0.0, 0.036)
    )
    base_geom.merge(
        BoxGeometry((0.082, 0.016, 0.008)).translate(0.092, 0.026, 0.046)
    )
    base_geom.merge(
        BoxGeometry((0.082, 0.016, 0.008)).translate(0.092, -0.026, 0.046)
    )

    neck_geom = section_loft(
        [
            yz_section(-0.082, width=0.118, height=0.118, radius=0.038, z_center=0.089),
            yz_section(-0.058, width=0.106, height=0.188, radius=0.036, z_center=0.125),
            yz_section(-0.034, width=0.092, height=0.258, radius=0.032, z_center=0.161),
            yz_section(-0.012, width=0.078, height=0.302, radius=0.028, z_center=0.185),
        ]
    )
    base_geom.merge(neck_geom)

    hinge_cheek = CylinderGeometry(radius=0.027, height=0.018).rotate_x(math.pi / 2.0)
    base_geom.merge(hinge_cheek.copy().translate(0.002, 0.029, 0.322))
    base_geom.merge(hinge_cheek.copy().translate(0.002, -0.029, 0.322))
    base_geom.merge(
        CylinderGeometry(radius=0.012, height=0.012)
        .rotate_x(math.pi / 2.0)
        .translate(-0.050, 0.054, 0.188)
    )
    base_geom.merge(BoxGeometry((0.034, 0.012, 0.014)).translate(-0.063, 0.052, 0.282))
    base_geom.merge(BoxGeometry((0.024, 0.020, 0.030)).translate(-0.052, 0.044, 0.268))

    base.visual(
        mesh_from_geometry(base_geom, "mixer_base_shell"),
        material=body_enamel,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.31, 0.20, 0.35)),
        mass=7.6,
        origin=Origin(xyz=(0.01, 0.0, 0.175)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_outer = [
        (0.016, 0.0),
        (0.045, 0.006),
        (0.071, 0.038),
        (0.079, 0.084),
        (0.082, 0.096),
    ]
    bowl_inner = [
        (0.0, 0.004),
        (0.040, 0.012),
        (0.066, 0.038),
        (0.074, 0.092),
    ]
    bowl_geom = LatheGeometry.from_shell_profiles(bowl_outer, bowl_inner, segments=40)
    bowl_geom.translate(0.054, 0.0, 0.032)

    carriage_geom = BoxGeometry((0.074, 0.066, 0.010)).translate(0.037, 0.0, 0.005)
    carriage_geom.merge(BoxGeometry((0.016, 0.050, 0.018)).translate(0.008, 0.0, 0.009))
    carriage_geom.merge(CylinderGeometry(radius=0.040, height=0.024).translate(0.054, 0.0, 0.012))
    carriage_geom.merge(CylinderGeometry(radius=0.050, height=0.008).translate(0.054, 0.0, 0.028))
    carriage_geom.merge(bowl_geom)

    bowl_carriage.visual(
        mesh_from_geometry(carriage_geom, "bowl_carriage_shell"),
        material=stainless,
        name="bowl_shell",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.16, 0.17, 0.14)),
        mass=1.2,
        origin=Origin(xyz=(0.054, 0.0, 0.07)),
    )

    model.articulation(
        "base_to_bowl_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.09, 0.0, 0.05)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.08,
            lower=0.0,
            upper=0.032,
        ),
    )

    head = model.part("head")
    head_geom = section_loft(
        [
            yz_section(0.026, width=0.034, height=0.050, radius=0.014, z_center=0.010),
            yz_section(0.058, width=0.078, height=0.094, radius=0.030, z_center=0.002),
            yz_section(0.126, width=0.108, height=0.128, radius=0.042, z_center=-0.014),
            yz_section(0.176, width=0.098, height=0.116, radius=0.034, z_center=-0.024),
            yz_section(0.228, width=0.054, height=0.082, radius=0.018, z_center=-0.034),
        ]
    )
    head_geom.merge(
        CylinderGeometry(radius=0.016, height=0.032)
        .rotate_x(math.pi / 2.0)
        .translate(0.010, 0.0, 0.014)
    )
    head_geom.merge(CylinderGeometry(radius=0.026, height=0.040).translate(0.164, 0.0, -0.058))
    head_geom.merge(CylinderGeometry(radius=0.017, height=0.020).translate(0.164, 0.0, -0.088))

    head.visual(
        mesh_from_geometry(head_geom, "mixer_head_shell"),
        material=body_enamel,
        name="head_shell",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.24, 0.13, 0.16)),
        mass=3.9,
        origin=Origin(xyz=(0.12, 0.0, -0.02)),
    )

    model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.002, 0.0, 0.322)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.005, height=0.028).translate(0.0, 0.0, -0.014)
    whisk_geom.merge(CylinderGeometry(radius=0.008, height=0.018).translate(0.0, 0.0, -0.037))
    whisk_geom.merge(CylinderGeometry(radius=0.011, height=0.016).translate(0.0, 0.0, -0.054))

    for index in range(4):
        angle = index * math.pi / 2.0
        c = math.cos(angle)
        s = math.sin(angle)
        whisk_geom.merge(
            tube_from_spline_points(
                [
                    (0.008 * c, 0.008 * s, -0.052),
                    (0.018 * c, 0.018 * s, -0.072),
                    (0.031 * c, 0.031 * s, -0.102),
                    (0.037 * c, 0.037 * s, -0.124),
                    (0.0, 0.0, -0.138),
                    (-0.037 * c, -0.037 * s, -0.124),
                    (-0.031 * c, -0.031 * s, -0.102),
                    (-0.018 * c, -0.018 * s, -0.072),
                    (-0.008 * c, -0.008 * s, -0.052),
                ],
                radius=0.0014,
                samples_per_segment=10,
                radial_segments=12,
            )
        )

    whisk.visual(
        mesh_from_geometry(whisk_geom, "balloon_whisk"),
        material=stainless,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.15),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
    )

    model.articulation(
        "head_to_whisk_drive",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.164, 0.0, -0.098)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="speed_hub",
    )
    speed_control.visual(
        Box((0.036, 0.008, 0.010)),
        origin=Origin(xyz=(0.020, 0.010, 0.0)),
        material=dark_trim,
        name="speed_handle",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.042, 0.016, 0.018)),
        mass=0.06,
        origin=Origin(xyz=(0.020, 0.009, 0.0)),
    )

    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(-0.050, 0.060, 0.188)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.30,
            upper=0.75,
        ),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.020, 0.008, 0.010)),
        origin=Origin(xyz=(0.010, 0.004, 0.0)),
        material=dark_trim,
        name="lock_button",
    )
    head_lock.visual(
        Box((0.014, 0.004, 0.006)),
        origin=Origin(xyz=(0.007, 0.002, 0.0)),
        material=dark_trim,
        name="lock_stem",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.024, 0.010, 0.012)),
        mass=0.03,
        origin=Origin(xyz=(0.010, 0.004, 0.0)),
    )

    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.075, 0.058, 0.282)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.04,
            lower=0.0,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl_slide = object_model.get_articulation("base_to_bowl_slide")
    head_tilt = object_model.get_articulation("base_to_head_tilt")
    whisk_drive = object_model.get_articulation("head_to_whisk_drive")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    base = object_model.get_part("base")
    bowl_carriage = object_model.get_part("bowl_carriage")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    expected_layout = {
        "base_to_bowl_slide": ArticulationType.PRISMATIC,
        "base_to_head_tilt": ArticulationType.REVOLUTE,
        "head_to_whisk_drive": ArticulationType.CONTINUOUS,
        "base_to_speed_control": ArticulationType.REVOLUTE,
        "base_to_head_lock": ArticulationType.PRISMATIC,
    }
    actual_layout = {
        articulation.name: articulation.articulation_type
        for articulation in object_model.articulations
    }
    ctx.check(
        "canonical articulation layout only",
        actual_layout == expected_layout,
        details=f"actual_layout={actual_layout}",
    )

    bowl_rest = ctx.part_world_position(bowl_carriage)
    whisk_rest = ctx.part_world_position(whisk)
    lock_rest = ctx.part_world_position(head_lock)
    speed_rest = ctx.part_element_world_aabb(speed_control, elem="speed_handle")
    bowl_aabb = ctx.part_world_aabb(bowl_carriage)
    whisk_aabb = ctx.part_world_aabb(whisk)

    bowl_upper = bowl_slide.motion_limits.upper
    head_upper = head_tilt.motion_limits.upper
    speed_upper = speed_joint.motion_limits.upper
    lock_upper = lock_joint.motion_limits.upper

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple(
            0.5 * (min_corner[index] + max_corner[index])
            for index in range(3)
        )

    bowl_center = aabb_center(bowl_aabb)
    whisk_center = aabb_center(whisk_aabb)
    ctx.check(
        "whisk is centered over the bowl",
        bowl_center is not None
        and whisk_center is not None
        and abs(whisk_center[0] - bowl_center[0]) < 0.025
        and abs(whisk_center[1] - bowl_center[1]) < 0.01,
        details=f"bowl_center={bowl_center}, whisk_center={whisk_center}",
    )

    with ctx.pose({bowl_slide: bowl_upper}):
        bowl_extended = ctx.part_world_position(bowl_carriage)
    ctx.check(
        "bowl carriage slides forward",
        bowl_rest is not None
        and bowl_extended is not None
        and bowl_extended[0] > bowl_rest[0] + 0.02,
        details=f"rest={bowl_rest}, extended={bowl_extended}",
    )

    with ctx.pose({head_tilt: head_upper}):
        whisk_lifted = ctx.part_world_position(whisk)
        whisk_lifted_aabb = ctx.part_world_aabb(whisk)
        bowl_pose_aabb = ctx.part_world_aabb(bowl_carriage)
    ctx.check(
        "tilted head lifts the whisk clear of the bowl",
        whisk_lifted_aabb is not None
        and bowl_pose_aabb is not None
        and whisk_lifted_aabb[0][2] > bowl_pose_aabb[1][2] + 0.01,
        details=f"whisk_aabb={whisk_lifted_aabb}, bowl_aabb={bowl_pose_aabb}",
    )

    with ctx.pose({speed_joint: speed_upper}):
        speed_rotated = ctx.part_element_world_aabb(speed_control, elem="speed_handle")

    ctx.check(
        "speed control rotates upward",
        aabb_center(speed_rest) is not None
        and aabb_center(speed_rotated) is not None
        and aabb_center(speed_rotated)[2] > aabb_center(speed_rest)[2] + 0.004,
        details=f"rest={speed_rest}, rotated={speed_rotated}",
    )

    with ctx.pose({lock_joint: lock_upper}):
        lock_extended = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock slider translates forward",
        lock_rest is not None
        and lock_extended is not None
        and lock_extended[0] > lock_rest[0] + 0.006,
        details=f"rest={lock_rest}, extended={lock_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
