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
    model = ArticulatedObject(name="artisan_tilt_head_stand_mixer")

    body_red = model.material("body_red", rgba=(0.72, 0.10, 0.12, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.92, 0.92, 0.94, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.11, 0.11, 0.12, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.74, 0.76, 0.78, 1.0))

    def yz_section(
        x: float,
        width: float,
        height: float,
        radius: float,
        *,
        z_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_shift) for z, y in rounded_rect_profile(height, width, radius)]

    base = model.part("base")

    pedestal_plate = ExtrudeGeometry(rounded_rect_profile(0.36, 0.26, 0.060), 0.050)
    base.visual(
        mesh_from_geometry(pedestal_plate, "base_plate"),
        origin=Origin(xyz=(0.020, 0.0, 0.025)),
        material=body_red,
        name="base_plate",
    )

    pedestal_shell = section_loft(
        [
            yz_section(0.0, 0.15, 0.16, 0.040, z_shift=0.000),
            yz_section(0.04, 0.13, 0.23, 0.036, z_shift=0.026),
            yz_section(0.08, 0.10, 0.29, 0.028, z_shift=0.044),
        ]
    )
    base.visual(
        mesh_from_geometry(pedestal_shell, "pedestal_shell"),
        origin=Origin(xyz=(-0.145, 0.0, 0.170)),
        material=body_red,
        name="pedestal_shell",
    )

    base.visual(
        Box((0.240, 0.130, 0.028)),
        origin=Origin(xyz=(0.055, 0.0, 0.064)),
        material=body_red,
        name="carriage_deck",
    )
    base.visual(
        Box((0.040, 0.136, 0.040)),
        origin=Origin(xyz=(-0.075, 0.0, 0.290)),
        material=body_red,
        name="hinge_block",
    )
    base.visual(
        Box((0.026, 0.012, 0.040)),
        origin=Origin(xyz=(-0.043, 0.074, 0.314)),
        material=body_red,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.026, 0.012, 0.040)),
        origin=Origin(xyz=(-0.043, -0.074, 0.314)),
        material=body_red,
        name="right_hinge_cheek",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.28, 0.34)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.130, 0.095, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=body_red,
        name="slide_block",
    )
    bowl_carriage.visual(
        Box((0.050, 0.060, 0.014)),
        origin=Origin(xyz=(-0.035, 0.0, 0.037)),
        material=body_red,
        name="bowl_saddle",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.050, length=0.008),
        origin=Origin(xyz=(0.020, 0.0, 0.034)),
        material=satin_steel,
        name="mount_plate",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.14, 0.10, 0.08)),
        mass=1.2,
        origin=Origin(xyz=(0.010, 0.0, 0.040)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.090, 0.0, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.08, lower=0.0, upper=0.035),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [(0.030, 0.000), (0.055, 0.010), (0.094, 0.055), (0.110, 0.126), (0.114, 0.160)],
        [(0.000, 0.005), (0.048, 0.016), (0.089, 0.056), (0.104, 0.156)],
        segments=64,
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "mixing_bowl_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=polished_steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.034, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=polished_steel,
        name="bowl_foot",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.116, length=0.198),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.038, 0.0, 0.036)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            yz_section(0.040, 0.105, 0.115, 0.030, z_shift=0.028),
            yz_section(0.155, 0.170, 0.166, 0.050, z_shift=0.024),
            yz_section(0.278, 0.165, 0.150, 0.045, z_shift=0.020),
            yz_section(0.364, 0.120, 0.116, 0.032, z_shift=0.016),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "head_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=body_red,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.136),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_red,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.056, 0.100, 0.032)),
        origin=Origin(xyz=(0.038, 0.0, -0.006)),
        material=body_red,
        name="hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.080),
        origin=Origin(xyz=(0.250, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_red,
        name="drive_nose",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.032),
        origin=Origin(xyz=(0.172, 0.0, -0.039)),
        material=dark_trim,
        name="planetary_hub",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.36, 0.19, 0.19)),
        mass=4.4,
        origin=Origin(xyz=(0.190, 0.0, -0.005)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.040, 0.0, 0.314)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.006, height=0.024).translate(0.0, 0.0, -0.033)
    whisk_geom.merge(CylinderGeometry(radius=0.010, height=0.018).translate(0.0, 0.0, -0.051))
    whisk_geom.merge(CylinderGeometry(radius=0.013, height=0.012).translate(0.0, 0.0, -0.064))
    for i in range(10):
        angle = i * math.pi / 10.0
        c = math.cos(angle)
        s = math.sin(angle)
        loop = tube_from_spline_points(
            [
                (0.009 * c, 0.009 * s, -0.038),
                (0.018 * c, 0.018 * s, -0.054),
                (0.030 * c, 0.030 * s, -0.076),
                (0.041 * c, 0.041 * s, -0.096),
                (0.0, 0.0, -0.120),
                (-0.041 * c, -0.041 * s, -0.096),
                (-0.030 * c, -0.030 * s, -0.076),
                (-0.018 * c, -0.018 * s, -0.054),
                (-0.009 * c, -0.009 * s, -0.038),
            ],
            radius=0.0016,
            samples_per_segment=14,
            radial_segments=14,
        )
        whisk_geom.merge(loop)
    whisk.visual(
        mesh_from_geometry(whisk_geom, "balloon_whisk"),
        material=polished_steel,
        name="whisk_shell",
    )
    whisk.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=satin_steel,
        name="whisk_coupler",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.122),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, -0.061)),
    )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.172, 0.0, -0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
