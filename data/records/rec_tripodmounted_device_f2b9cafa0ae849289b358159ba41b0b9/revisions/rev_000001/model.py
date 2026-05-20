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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_pan_tilt_device")

    dark_anodized = model.material("dark_anodized", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.044, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=dark_anodized,
        name="crown_core",
    )
    crown.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=dark_anodized,
        name="pan_bearing_base",
    )
    crown.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=satin_black,
        name="underslung_ballast",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    hinge_radius = 0.060
    hinge_height = 0.062
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        radial = (c, s, 0.0)
        tangent = (-s, c, 0.0)

        crown.visual(
            Box((0.020, 0.006, 0.020)),
            origin=Origin(
                xyz=(
                    hinge_radius * radial[0] + 0.010 * tangent[0],
                    hinge_radius * radial[1] + 0.010 * tangent[1],
                    hinge_height,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_anodized,
            name=f"hinge_ear_{index}_a",
        )
        crown.visual(
            Box((0.020, 0.006, 0.020)),
            origin=Origin(
                xyz=(
                    hinge_radius * radial[0] - 0.010 * tangent[0],
                    hinge_radius * radial[1] - 0.010 * tangent[1],
                    hinge_height,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_anodized,
            name=f"hinge_ear_{index}_b",
        )
        crown.visual(
            Box((0.034, 0.012, 0.006)),
            origin=Origin(
                xyz=(
                    (hinge_radius - 0.012) * radial[0],
                    (hinge_radius - 0.012) * radial[1],
                    hinge_height - 0.012,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_anodized,
            name=f"hinge_pad_{index}",
        )
        crown.visual(
            Box((0.040, 0.016, 0.010)),
            origin=Origin(
                xyz=(
                    0.034 * radial[0],
                    0.034 * radial[1],
                    0.058,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_anodized,
            name=f"spider_rib_{index}",
        )

        leg = model.part(f"leg_{index}")
        leg_mesh = tube_from_spline_points(
            [
                (0.026, 0.0, -0.008),
                (0.120, 0.0, -0.028),
                (0.220, 0.0, -0.042),
                (0.305, 0.0, -0.050),
            ],
            radius=0.009,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        leg.visual(
            Cylinder(radius=0.0055, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.032, 0.012, 0.012)),
            origin=Origin(xyz=(0.014, 0.0, -0.006)),
            material=satin_black,
            name="knuckle_block",
        )
        leg.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=satin_black,
            name="lower_tube",
        )
        leg.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(0.305, 0.0, -0.056)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.340, 0.040, 0.075)),
            mass=0.30,
            origin=Origin(xyz=(0.170, 0.0, -0.028)),
        )

        model.articulation(
            f"crown_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * radial[0], hinge_radius * radial[1], hinge_height),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.2,
                lower=0.0,
                upper=math.radians(72.0),
            ),
        )

    crown.inertial = Inertial.from_geometry(
        Box((0.140, 0.140, 0.110)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.029, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_anodized,
        name="pan_drum",
    )
    pan_head.visual(
        Box((0.018, 0.050, 0.012)),
        origin=Origin(xyz=(-0.008, 0.0, 0.020)),
        material=dark_anodized,
        name="tilt_bridge",
    )
    pan_head.visual(
        Box((0.014, 0.040, 0.020)),
        origin=Origin(xyz=(-0.008, 0.0, 0.024)),
        material=satin_black,
        name="rear_stanchion",
    )
    pan_head.visual(
        Box((0.020, 0.008, 0.032)),
        origin=Origin(xyz=(0.006, 0.025, 0.031)),
        material=dark_anodized,
        name="left_cheek",
    )
    pan_head.visual(
        Box((0.020, 0.008, 0.032)),
        origin=Origin(xyz=(0.006, -0.025, 0.031)),
        material=dark_anodized,
        name="right_cheek",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.070)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5),
    )

    device_plate = model.part("device_plate")
    device_plate.visual(
        Cylinder(radius=0.0048, length=0.038),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="tilt_trunnion",
    )
    device_plate.visual(
        Box((0.014, 0.038, 0.016)),
        origin=Origin(xyz=(0.006, 0.0, -0.002)),
        material=satin_black,
        name="tilt_block",
    )
    device_plate.visual(
        Box((0.086, 0.032, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, -0.010)),
        material=dark_anodized,
        name="device_plate",
    )
    device_plate.visual(
        Box((0.100, 0.038, 0.038)),
        origin=Origin(xyz=(0.060, 0.0, 0.016)),
        material=satin_black,
        name="sensor_body",
    )
    device_plate.visual(
        Box((0.028, 0.024, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.030)),
        material=dark_anodized,
        name="rear_screen_housing",
    )
    device_plate.visual(
        Box((0.034, 0.024, 0.006)),
        origin=Origin(xyz=(0.054, 0.0, 0.038)),
        material=dark_anodized,
        name="top_cover",
    )
    device_plate.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(xyz=(0.114, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_anodized,
        name="lens",
    )
    device_plate.inertial = Inertial.from_geometry(
        Box((0.135, 0.042, 0.060)),
        mass=0.55,
        origin=Origin(xyz=(0.060, 0.0, 0.016)),
    )

    model.articulation(
        "head_to_device_plate",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=device_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=math.radians(-30.0),
            upper=math.radians(65.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    device_plate = object_model.get_part("device_plate")
    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("head_to_device_plate")
    leg_joints = [object_model.get_articulation(f"crown_to_leg_{index}") for index in range(3)]

    def elem_center(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.expect_gap(
        pan_head,
        crown,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan head seats on the crown bearing",
    )
    ctx.expect_gap(
        device_plate,
        crown,
        axis="z",
        min_gap=0.012,
        name="device mass stays above the crown",
    )

    foot_centers = [elem_center(f"leg_{index}", "foot_pad") for index in range(3)]
    for index, foot_center in enumerate(foot_centers):
        foot_aabb = ctx.part_element_world_aabb(f"leg_{index}", elem="foot_pad")
        foot_bottom = None if foot_aabb is None else foot_aabb[0][2]
        radius_xy = None if foot_center is None else math.hypot(foot_center[0], foot_center[1])
        ctx.check(
            f"leg {index} foot sits on the mounting plane",
            foot_bottom is not None and abs(foot_bottom) <= 0.003,
            details=f"foot_bottom={foot_bottom}",
        )
        ctx.check(
            f"leg {index} reaches a wide support radius",
            radius_xy is not None and radius_xy >= 0.29,
            details=f"radius_xy={radius_xy}",
        )

    rest_foot = foot_centers[0]
    with ctx.pose({leg_joints[0]: leg_joints[0].motion_limits.upper}):
        folded_foot = elem_center("leg_0", "foot_pad")
    rest_radius = None if rest_foot is None else math.hypot(rest_foot[0], rest_foot[1])
    folded_radius = None if folded_foot is None else math.hypot(folded_foot[0], folded_foot[1])
    ctx.check(
        "one tripod leg folds upward toward the crown",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[2] > rest_foot[2] + 0.18
        and folded_radius < rest_radius - 0.12,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    rest_lens = elem_center("device_plate", "lens")
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_lens = elem_center("device_plate", "lens")
    ctx.check(
        "pan head rotates the device about vertical",
        rest_lens is not None
        and panned_lens is not None
        and abs(rest_lens[2] - panned_lens[2]) <= 0.003
        and abs(rest_lens[0] - panned_lens[0]) >= 0.07
        and abs(rest_lens[1] - panned_lens[1]) >= 0.07,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        tilted_lens = elem_center("device_plate", "lens")
        ctx.expect_gap(
            device_plate,
            crown,
            axis="z",
            min_gap=0.024,
            name="up-tilted device clears the tripod crown",
        )
    ctx.check(
        "device tilts upward around the horizontal axis",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] > rest_lens[2] + 0.07
        and tilted_lens[0] < rest_lens[0] - 0.02,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
