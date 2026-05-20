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


def _center_of_aabb(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="solar_halpha_refractor_altaz")

    tripod_black = model.material("tripod_black", rgba=(0.12, 0.12, 0.13, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.31, 0.33, 0.36, 1.0))
    tube_gold = model.material("tube_gold", rgba=(0.78, 0.64, 0.28, 1.0))
    etalon_red = model.material("etalon_red", rgba=(0.68, 0.10, 0.09, 1.0))
    focuser_black = model.material("focuser_black", rgba=(0.08, 0.08, 0.09, 1.0))
    anodized_silver = model.material("anodized_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.026, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.538)),
        material=tripod_black,
        name="center_column",
    )
    tripod_base.visual(
        Cylinder(radius=0.048, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.871)),
        material=tripod_black,
        name="upper_collar",
    )
    tripod_base.visual(
        Cylinder(radius=0.072, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.917)),
        material=mount_gray,
        name="bearing_cap",
    )
    tripod_base.visual(
        Cylinder(radius=0.036, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.564)),
        material=mount_gray,
        name="spreader_hub",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.032 * c, 0.032 * s, 0.848),
                (0.180 * c, 0.180 * s, 0.500),
                (0.430 * c, 0.430 * s, 0.030),
            ],
            radius=0.014,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        tripod_base.visual(
            mesh_from_geometry(leg_mesh, f"tripod_leg_{index}"),
            material=tripod_black,
            name=f"leg_{index}",
        )

        spreader_mesh = tube_from_spline_points(
            [
                (0.024 * c, 0.024 * s, 0.564),
                (0.110 * c, 0.110 * s, 0.505),
                (0.175 * c, 0.175 * s, 0.460),
            ],
            radius=0.006,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        )
        tripod_base.visual(
            mesh_from_geometry(spreader_mesh, f"tripod_spreader_{index}"),
            material=mount_gray,
            name=f"spreader_{index}",
        )
        tripod_base.visual(
            Sphere(radius=0.022),
            origin=Origin(xyz=(0.430 * c, 0.430 * s, 0.030)),
            material=rubber,
            name=f"foot_{index}",
        )

    tripod_base.inertial = Inertial.from_geometry(
        Box((0.92, 0.92, 0.96)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
    )

    mount_arm = model.part("mount_arm")
    mount_arm.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=mount_gray,
        name="azimuth_drum",
    )
    mount_arm.visual(
        Cylinder(radius=0.085, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=mount_gray,
        name="azimuth_plate",
    )
    mount_arm.visual(
        Box((0.150, 0.110, 0.098)),
        origin=Origin(xyz=(0.055, -0.030, 0.131)),
        material=mount_gray,
        name="arm_base_block",
    )
    mount_arm.visual(
        Box((0.080, 0.080, 0.270)),
        origin=Origin(xyz=(0.100, -0.040, 0.320)),
        material=mount_gray,
        name="upright_arm",
    )
    mount_arm.visual(
        Box((0.210, 0.048, 0.050)),
        origin=Origin(xyz=(0.050, -0.030, 0.275), rpy=(0.0, 0.92, 0.0)),
        material=mount_gray,
        name="rear_brace",
    )
    mount_arm.visual(
        Cylinder(radius=0.045, length=0.080),
        origin=Origin(xyz=(0.100, -0.040, 0.500), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=focuser_black,
        name="alt_bearing_housing",
    )
    mount_arm.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.100, -0.110, 0.500), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=focuser_black,
        name="alt_clutch_knob",
    )
    mount_arm.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.62)),
        mass=3.2,
        origin=Origin(xyz=(0.045, -0.030, 0.310)),
    )

    telescope = model.part("telescope")
    telescope.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_silver,
        name="trunnion_disc",
    )
    telescope.visual(
        Box((0.120, 0.060, 0.070)),
        origin=Origin(xyz=(0.020, 0.050, 0.0)),
        material=anodized_silver,
        name="saddle_block",
    )
    telescope.visual(
        Cylinder(radius=0.038, length=0.280),
        origin=Origin(xyz=(0.080, 0.050, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_gold,
        name="main_tube",
    )
    telescope.visual(
        Cylinder(radius=0.045, length=0.080),
        origin=Origin(xyz=(0.260, 0.050, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=etalon_red,
        name="etalon_tube",
    )
    telescope.visual(
        Cylinder(radius=0.048, length=0.015),
        origin=Origin(xyz=(0.3075, 0.050, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=focuser_black,
        name="etalon_cell",
    )
    telescope.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(-0.085, 0.050, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=focuser_black,
        name="rear_adapter",
    )
    telescope.visual(
        Box((0.110, 0.050, 0.050)),
        origin=Origin(xyz=(-0.165, 0.050, 0.0)),
        material=focuser_black,
        name="drawtube_body",
    )
    telescope.visual(
        Box((0.040, 0.042, 0.042)),
        origin=Origin(xyz=(-0.240, 0.050, 0.0)),
        material=focuser_black,
        name="drawtube_nose",
    )
    telescope.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(-0.320, 0.050, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_silver,
        name="eyepiece_tube",
    )
    telescope.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(xyz=(-0.400, 0.050, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=focuser_black,
        name="eyepiece_barrel",
    )
    telescope.inertial = Inertial.from_geometry(
        Box((0.74, 0.12, 0.11)),
        mass=2.8,
        origin=Origin(xyz=(-0.040, 0.050, 0.0)),
    )

    model.articulation(
        "tripod_to_mount",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=mount_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.940)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5),
    )
    model.articulation(
        "mount_to_telescope",
        ArticulationType.REVOLUTE,
        parent=mount_arm,
        child=telescope,
        origin=Origin(xyz=(0.100, 0.0, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.25,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    mount_arm = object_model.get_part("mount_arm")
    telescope = object_model.get_part("telescope")
    azimuth = object_model.get_articulation("tripod_to_mount")
    altitude = object_model.get_articulation("mount_to_telescope")

    ctx.expect_gap(
        mount_arm,
        tripod_base,
        axis="z",
        positive_elem="azimuth_drum",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=1e-6,
        name="azimuth drum seats on the tripod bearing",
    )
    ctx.expect_gap(
        telescope,
        mount_arm,
        axis="y",
        positive_elem="trunnion_disc",
        negative_elem="alt_bearing_housing",
        max_gap=0.001,
        max_penetration=0.0,
        name="telescope trunnion seats against the altitude housing",
    )

    rest_position = ctx.part_world_position(telescope)
    with ctx.pose({azimuth: math.pi / 2.0}):
        turned_position = ctx.part_world_position(telescope)
    ctx.check(
        "azimuth joint swings the arm around the vertical axis",
        rest_position is not None
        and turned_position is not None
        and abs(rest_position[0]) > 0.05
        and abs(turned_position[1]) > 0.05
        and abs(math.hypot(rest_position[0], rest_position[1]) - math.hypot(turned_position[0], turned_position[1])) < 0.01,
        details=f"rest={rest_position}, turned={turned_position}",
    )

    with ctx.pose({altitude: altitude.motion_limits.upper}):
        front_center = _center_of_aabb(ctx.part_element_world_aabb(telescope, elem="etalon_cell"))
        rear_center = _center_of_aabb(ctx.part_element_world_aabb(telescope, elem="eyepiece_barrel"))
        ctx.check(
            "positive altitude raises the objective above the eyepiece end",
            front_center is not None
            and rear_center is not None
            and front_center[2] > rear_center[2] + 0.18,
            details=f"front={front_center}, rear={rear_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
