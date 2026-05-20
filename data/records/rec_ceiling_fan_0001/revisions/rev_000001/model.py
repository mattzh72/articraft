from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba):
    for kwargs in (
        {"name": name, "rgba": rgba},
        {"name": name, "diffuse": rgba},
        {"name": name, "color": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            continue
    try:
        return Material(name, rgba)
    except TypeError:
        return Material(name=name)


def _build_canopy_mesh():
    canopy_profile = [
        (0.0, 0.000),
        (0.074, 0.000),
        (0.071, -0.006),
        (0.064, -0.016),
        (0.052, -0.028),
        (0.036, -0.041),
        (0.020, -0.053),
        (0.0, -0.056),
    ]
    return mesh_from_geometry(
        LatheGeometry(canopy_profile, segments=72),
        ASSETS.mesh_path("ceiling_fan_canopy.obj"),
    )


def _build_rotor_shell_mesh():
    shell_geom = LatheGeometry(
        [
            (0.0, -0.016),
            (0.035, -0.016),
            (0.048, -0.020),
            (0.076, -0.028),
            (0.096, -0.045),
            (0.108, -0.068),
            (0.105, -0.089),
            (0.093, -0.105),
            (0.076, -0.113),
            (0.0, -0.113),
        ],
        segments=72,
    )
    shell_geom.merge(
        LatheGeometry(
            [
                (0.0, -0.113),
                (0.060, -0.113),
                (0.066, -0.121),
                (0.062, -0.143),
                (0.054, -0.162),
                (0.043, -0.177),
                (0.030, -0.188),
                (0.0, -0.190),
            ],
            segments=72,
        )
    )
    shell_geom.merge(
        CylinderGeometry(radius=0.111, height=0.004, radial_segments=48).translate(0.0, 0.0, -0.034)
    )
    shell_geom.merge(
        CylinderGeometry(radius=0.095, height=0.004, radial_segments=48).translate(0.0, 0.0, -0.103)
    )
    shell_geom.merge(
        CylinderGeometry(radius=0.021, height=0.010, radial_segments=32).translate(0.0, 0.0, -0.194)
    )
    return mesh_from_geometry(
        shell_geom,
        ASSETS.mesh_path("ceiling_fan_rotor_shell.obj"),
    )


def _build_blade_arm_mesh(blade_count: int):
    arm_template = BoxGeometry((0.106, 0.028, 0.008)).translate(0.110, 0.0, -0.084)
    arm_template.merge(
        CylinderGeometry(radius=0.020, height=0.006, radial_segments=28).translate(
            0.060, 0.0, -0.084
        )
    )
    arm_template.merge(
        CylinderGeometry(radius=0.017, height=0.006, radial_segments=28).translate(
            0.160, 0.0, -0.084
        )
    )
    arms_geom = arm_template.copy()
    for blade_index in range(1, blade_count):
        arms_geom.merge(arm_template.copy().rotate_z(blade_index * (2.0 * pi / blade_count)))
    return mesh_from_geometry(
        arms_geom,
        ASSETS.mesh_path("ceiling_fan_blade_arms.obj"),
    )


def _build_blade_mesh(blade_count: int):
    blade_outline = sample_catmull_rom_spline_2d(
        [
            (0.000, 0.030),
            (0.075, 0.046),
            (0.220, 0.062),
            (0.410, 0.054),
            (0.530, 0.028),
            (0.560, 0.000),
            (0.530, -0.028),
            (0.410, -0.054),
            (0.220, -0.062),
            (0.075, -0.046),
            (0.000, -0.030),
        ],
        samples_per_segment=12,
        closed=True,
    )
    if blade_outline and blade_outline[0] == blade_outline[-1]:
        blade_outline = blade_outline[:-1]

    blade_template = ExtrudeGeometry(
        blade_outline,
        height=0.008,
        cap=True,
        center=True,
        closed=True,
    )
    blades_geom = blade_template.copy().rotate_x(0.22).translate(0.145, 0.0, -0.090)
    for blade_index in range(1, blade_count):
        blades_geom.merge(
            blade_template.copy()
            .rotate_x(0.22)
            .translate(0.145, 0.0, -0.090)
            .rotate_z(blade_index * (2.0 * pi / blade_count))
        )
    return mesh_from_geometry(
        blades_geom,
        ASSETS.mesh_path("ceiling_fan_blades.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_fan", assets=ASSETS)

    brushed_nickel = _make_material("brushed_nickel", (0.76, 0.77, 0.79, 1.0))
    satin_black = _make_material("satin_black", (0.12, 0.12, 0.13, 1.0))
    dark_walnut = _make_material("dark_walnut", (0.35, 0.22, 0.13, 1.0))
    model.materials.extend([brushed_nickel, satin_black, dark_walnut])

    blade_count = 5

    mount = model.part("mount")
    mount.visual(_build_canopy_mesh(), material=brushed_nickel, name="canopy")
    mount.visual(
        Cylinder(radius=0.076, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=satin_black,
        name="ceiling_gasket",
    )
    mount.visual(
        Cylinder(radius=0.020, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.063)),
        material=satin_black,
        name="downrod_collar",
    )
    mount.visual(
        Cylinder(radius=0.011, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=brushed_nickel,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        material=brushed_nickel,
        name="motor_coupler",
    )
    mount.visual(
        Cylinder(radius=0.028, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.321)),
        material=brushed_nickel,
        name="hanger_yoke",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.350),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=satin_black,
        name="rotor_collar",
    )
    rotor.visual(
        _build_rotor_shell_mesh(),
        material=brushed_nickel,
        name="motor_housing",
    )
    rotor.visual(
        _build_blade_arm_mesh(blade_count),
        material=satin_black,
        name="blade_arms",
    )
    rotor.visual(
        _build_blade_mesh(blade_count),
        material=dark_walnut,
        name="blades",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.630, length=0.210),
        mass=6.6,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
    )

    model.articulation(
        "mount_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent="mount",
        child="rotor",
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    for angle in (0.0, pi / 4.0, pi / 2.0):
        with ctx.pose({"mount_to_rotor_spin": angle}):
            ctx.expect_origin_distance("mount", "rotor", axes="xy", max_dist=0.01)
            ctx.expect_aabb_overlap("mount", "rotor", axes="xy", min_overlap=0.12)
            ctx.expect_aabb_gap("mount", "rotor", axis="z", max_gap=0.006, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
