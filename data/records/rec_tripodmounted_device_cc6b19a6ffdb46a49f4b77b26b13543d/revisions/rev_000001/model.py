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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_pan_tilt_device")

    matte_black = model.material("matte_black", rgba=(0.15, 0.16, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass", rgba=(0.10, 0.12, 0.16, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=graphite,
        name="crown_body",
    )
    crown.visual(
        Cylinder(radius=0.048, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=aluminum,
        name="top_cap",
    )
    crown.visual(
        Cylinder(radius=0.024, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.662)),
        material=matte_black,
        name="pan_support_pedestal",
    )
    crown.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.623)),
        material=graphite,
        name="pedestal_shoulder",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    leg_names = ("front_leg", "left_leg", "right_leg")
    hinge_radius = 0.090
    hinge_height = 0.566

    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        hinge_xyz = (hinge_radius * c, hinge_radius * s, hinge_height)
        crown.visual(
            Box((0.028, 0.008, 0.028)),
            origin=Origin(
                xyz=(
                    hinge_xyz[0] - 0.023 * c + 0.020 * -s,
                    hinge_xyz[1] - 0.023 * s + 0.020 * c,
                    hinge_xyz[2] - 0.010,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"{leg_names[index]}_hinge_rib",
        )
        crown.visual(
            Box((0.028, 0.008, 0.028)),
            origin=Origin(
                xyz=(
                    hinge_xyz[0] - 0.023 * c - 0.020 * -s,
                    hinge_xyz[1] - 0.023 * s - 0.020 * c,
                    hinge_xyz[2] - 0.010,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=graphite,
            name=f"{leg_names[index]}_ear_backer",
        )
        crown.visual(
            Box((0.020, 0.008, 0.036)),
            origin=Origin(
                xyz=(
                    hinge_xyz[0] + 0.020 * -s,
                    hinge_xyz[1] + 0.020 * c,
                    hinge_xyz[2],
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=aluminum,
            name=f"{leg_names[index]}_inner_ear",
        )
        crown.visual(
            Box((0.020, 0.008, 0.036)),
            origin=Origin(
                xyz=(
                    hinge_xyz[0] - 0.020 * -s,
                    hinge_xyz[1] - 0.020 * c,
                    hinge_xyz[2],
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=aluminum,
            name=f"{leg_names[index]}_outer_ear",
        )

    crown.inertial = Inertial.from_geometry(
        Box((0.86, 0.86, 0.78)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
    )

    for index, angle in enumerate(leg_angles):
        leg = model.part(leg_names[index])
        leg.visual(
            Cylinder(radius=0.009, length=0.032),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name="hinge_knuckle",
        )
        leg.visual(
            Box((0.022, 0.014, 0.016)),
            origin=Origin(xyz=(0.013, 0.0, -0.008)),
            material=graphite,
            name="hinge_yoke",
        )
        leg.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.014, 0.0, -0.010),
                        (0.125, 0.0, -0.182),
                        (0.240, 0.0, -0.370),
                        (0.340, 0.0, -0.540),
                    ],
                    radius=0.012,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"{leg_names[index]}_tube",
            ),
            material=matte_black,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=0.019),
            origin=Origin(xyz=(0.340, 0.0, -0.540)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.380, 0.050, 0.580)),
            mass=0.65,
            origin=Origin(xyz=(0.170, 0.0, -0.270)),
        )

        model.articulation(
            f"crown_to_{leg_names[index]}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), hinge_height), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.4,
                lower=-1.05,
                upper=0.20,
            ),
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.086, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=aluminum,
        name="pan_turntable",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=matte_black,
        name="head_hub",
    )
    head.visual(
        Box((0.052, 0.112, 0.010)),
        origin=Origin(xyz=(0.026, 0.0, 0.028)),
        material=graphite,
        name="cradle_base",
    )
    head.visual(
        Box((0.022, 0.012, 0.090)),
        origin=Origin(xyz=(0.026, 0.054, 0.078)),
        material=graphite,
        name="left_cheek",
    )
    head.visual(
        Box((0.022, 0.012, 0.090)),
        origin=Origin(xyz=(0.026, -0.054, 0.078)),
        material=graphite,
        name="right_cheek",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.100),
        origin=Origin(xyz=(0.026, 0.0, 0.118), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="cheek_crossbar",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.180, 0.130, 0.130)),
        mass=1.1,
        origin=Origin(xyz=(0.010, 0.0, 0.055)),
    )

    model.articulation(
        "crown_to_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.704)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )

    device = model.part("device")
    device.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, 0.041, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="left_trunnion",
    )
    device.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.0, -0.041, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="right_trunnion",
    )
    device.visual(
        Box((0.026, 0.076, 0.050)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=graphite,
        name="rear_housing",
    )
    device.visual(
        Box((0.112, 0.076, 0.066)),
        origin=Origin(xyz=(0.072, 0.0, 0.008)),
        material=matte_black,
        name="device_body",
    )
    device.visual(
        Box((0.130, 0.070, 0.006)),
        origin=Origin(xyz=(0.070, 0.0, -0.036)),
        material=aluminum,
        name="device_plate",
    )
    device.visual(
        Box((0.050, 0.038, 0.018)),
        origin=Origin(xyz=(0.055, 0.0, -0.027)),
        material=graphite,
        name="plate_riser",
    )
    device.visual(
        Cylinder(radius=0.019, length=0.036),
        origin=Origin(xyz=(0.144, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_barrel",
    )
    device.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.165, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="lens_bezel",
    )
    device.visual(
        Box((0.050, 0.040, 0.003)),
        origin=Origin(xyz=(0.050, 0.0, 0.042)),
        material=glass,
        name="top_display",
    )
    device.inertial = Inertial.from_geometry(
        Box((0.180, 0.090, 0.090)),
        mass=0.9,
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
    )

    model.articulation(
        "head_to_device",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device,
        origin=Origin(xyz=(0.030, 0.0, 0.086)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-0.70,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crown = object_model.get_part("crown")
    head = object_model.get_part("head")
    device = object_model.get_part("device")
    front_leg = object_model.get_part("front_leg")

    pan = object_model.get_articulation("crown_to_head")
    tilt = object_model.get_articulation("head_to_device")
    front_leg_hinge = object_model.get_articulation("crown_to_front_leg")

    def aabb_center(aabb):
        if aabb is None:
            return None
        aabb_min, aabb_max = aabb
        return tuple((aabb_min[i] + aabb_max[i]) * 0.5 for i in range(3))

    ctx.expect_contact(
        head,
        crown,
        elem_a="pan_turntable",
        elem_b="pan_support_pedestal",
        name="pan head sits on the crown pedestal",
    )
    ctx.expect_overlap(
        head,
        crown,
        axes="xy",
        elem_a="pan_turntable",
        elem_b="pan_support_pedestal",
        min_overlap=0.045,
        name="pan head remains broadly supported by the crown",
    )

    rest_device_origin = ctx.part_world_position(device)
    with ctx.pose({pan: math.pi / 2.0}):
        panned_device_origin = ctx.part_world_position(device)
    ctx.check(
        "pan joint rotates the device around the vertical axis",
        rest_device_origin is not None
        and panned_device_origin is not None
        and abs(panned_device_origin[1] - rest_device_origin[1]) > 0.025
        and abs(panned_device_origin[2] - rest_device_origin[2]) < 1e-6,
        details=f"rest={rest_device_origin}, panned={panned_device_origin}",
    )

    rest_lens_center = aabb_center(ctx.part_element_world_aabb(device, elem="lens_bezel"))
    with ctx.pose({tilt: 0.75}):
        tilted_lens_center = aabb_center(ctx.part_element_world_aabb(device, elem="lens_bezel"))
    ctx.check(
        "tilt joint lifts the device nose upward",
        rest_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] > rest_lens_center[2] + 0.08,
        details=f"rest={rest_lens_center}, tilted={tilted_lens_center}",
    )

    rest_foot_center = aabb_center(ctx.part_element_world_aabb(front_leg, elem="foot_pad"))
    with ctx.pose({front_leg_hinge: -1.0}):
        folded_foot_center = aabb_center(ctx.part_element_world_aabb(front_leg, elem="foot_pad"))
    ctx.check(
        "front leg folds upward toward the crown",
        rest_foot_center is not None
        and folded_foot_center is not None
        and folded_foot_center[2] > rest_foot_center[2] + 0.40,
        details=f"rest={rest_foot_center}, folded={folded_foot_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
