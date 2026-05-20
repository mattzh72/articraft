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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _rotate_xz(x: float, z: float, angle: float) -> tuple[float, float]:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (cosine * x) + (sine * z),
        (-sine * x) + (cosine * z),
    )


def _blade_station(y_pos: float, chord: float, thickness: float, pitch: float) -> list[tuple[float, float, float]]:
    profile = [
        (0.48 * chord, 0.0),
        (0.12 * chord, 0.52 * thickness),
        (-0.52 * chord, 0.02 * thickness),
        (-0.10 * chord, -0.46 * thickness),
    ]
    points: list[tuple[float, float, float]] = []
    for x_pos, z_pos in profile:
        rot_x, rot_z = _rotate_xz(x_pos, z_pos, pitch)
        points.append((rot_x, y_pos, rot_z))
    return points


def _build_blade_mesh():
    sections = [
        _blade_station(0.018, 0.060, 0.0075, 0.60),
        _blade_station(0.120, 0.052, 0.0055, 0.38),
        _blade_station(0.225, 0.034, 0.0030, 0.18),
    ]
    return repair_loft(section_loft(sections))


def _build_canopy_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.020, -0.006),
            (0.044, -0.015),
            (0.070, -0.040),
            (0.086, -0.082),
        ],
        [
            (0.014, -0.006),
            (0.036, -0.016),
            (0.058, -0.040),
            (0.074, -0.080),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _add_fan_head(
    model: ArticulatedObject,
    *,
    name: str,
    housing_material,
    blade_material,
    blade_mesh,
) -> None:
    head = model.part(name)

    head.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_material,
        name="trunnion",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_material,
        name="neck",
    )
    head.visual(
        Cylinder(radius=0.060, length=0.180),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_material,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.070, length=0.032),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_material,
        name="motor_bell",
    )
    head.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(-0.028, 0.0, 0.0)),
        material=housing_material,
        name="rear_cap",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.080),
        origin=Origin(xyz=(0.168, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_material,
        name="blade_hub",
    )
    head.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.212, 0.0, 0.0)),
        material=housing_material,
        name="nose_cap",
    )

    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0), start=1):
        head.visual(
            blade_mesh,
            origin=Origin(xyz=(0.168, 0.0, 0.0), rpy=(angle, 0.0, 0.0)),
            material=blade_material,
            name=f"blade_{index}",
        )

    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.220, length=0.280),
        mass=4.0,
        origin=Origin(xyz=(0.140, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_head_ceiling_fan")

    brass = model.material("brass", rgba=(0.64, 0.55, 0.30, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.22, 0.18, 0.14, 1.0))
    blade_wood = model.material("blade_wood", rgba=(0.45, 0.30, 0.18, 1.0))

    canopy_shell = mesh_from_geometry(_build_canopy_shell(), "canopy_shell")
    blade_mesh = mesh_from_geometry(_build_blade_mesh(), "fan_blade")

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=0.062, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=brass,
        name="ceiling_plate",
    )
    canopy.visual(
        canopy_shell,
        material=brass,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=dark_bronze,
        name="canopy_collar",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.086, length=0.082),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    downrod = model.part("downrod")
    downrod.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=dark_bronze,
        name="upper_coupler",
    )
    downrod.visual(
        Cylinder(radius=0.012, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, -0.330)),
        material=dark_bronze,
        name="rod",
    )
    downrod.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.630)),
        material=brass,
        name="lower_collar",
    )
    downrod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.660),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.330)),
    )

    crossbar = model.part("crossbar")
    crossbar.visual(
        Cylinder(radius=0.055, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=brass,
        name="swivel_hub",
    )
    crossbar.visual(
        Box((0.900, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=dark_bronze,
        name="bar",
    )
    for side, x_pos in (("right", 0.450), ("left", -0.450)):
        crossbar.visual(
            Box((0.040, 0.010, 0.110)),
            origin=Origin(xyz=(x_pos, 0.030, -0.090)),
            material=dark_bronze,
            name=f"{side}_yoke_front",
        )
        crossbar.visual(
            Box((0.040, 0.010, 0.110)),
            origin=Origin(xyz=(x_pos, -0.030, -0.090)),
            material=dark_bronze,
            name=f"{side}_yoke_rear",
        )
    crossbar.inertial = Inertial.from_geometry(
        Box((0.980, 0.130, 0.180)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    _add_fan_head(
        model,
        name="left_head",
        housing_material=dark_bronze,
        blade_material=blade_wood,
        blade_mesh=blade_mesh,
    )
    _add_fan_head(
        model,
        name="right_head",
        housing_material=dark_bronze,
        blade_material=blade_wood,
        blade_mesh=blade_mesh,
    )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
    )
    model.articulation(
        "downrod_to_crossbar",
        ArticulationType.REVOLUTE,
        parent=downrod,
        child=crossbar,
        origin=Origin(xyz=(0.0, 0.0, -0.660)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=-1.05,
            upper=1.05,
        ),
    )
    model.articulation(
        "crossbar_to_left_head",
        ArticulationType.REVOLUTE,
        parent=crossbar,
        child="left_head",
        origin=Origin(xyz=(-0.470, 0.0, -0.090), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.85,
            upper=0.55,
        ),
    )
    model.articulation(
        "crossbar_to_right_head",
        ArticulationType.REVOLUTE,
        parent=crossbar,
        child="right_head",
        origin=Origin(xyz=(0.470, 0.0, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.85,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    crossbar = object_model.get_part("crossbar")
    left_head = object_model.get_part("left_head")
    right_head = object_model.get_part("right_head")

    swivel = object_model.get_articulation("downrod_to_crossbar")
    left_tilt = object_model.get_articulation("crossbar_to_left_head")
    right_tilt = object_model.get_articulation("crossbar_to_right_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "fan_parts_present",
        all(part is not None for part in (canopy, downrod, crossbar, left_head, right_head)),
        "Expected canopy, downrod, crossbar, and both fan heads.",
    )
    ctx.check(
        "crossbar_swivel_axis_is_vertical",
        swivel.axis == (0.0, 0.0, 1.0),
        f"Expected vertical swivel axis, got {swivel.axis}.",
    )
    ctx.check(
        "head_pitch_axes_are_horizontal",
        left_tilt.axis == (0.0, 1.0, 0.0) and right_tilt.axis == (0.0, 1.0, 0.0),
        f"Expected both head tilt axes to be horizontal y axes, got {left_tilt.axis} and {right_tilt.axis}.",
    )

    ctx.expect_contact(downrod, canopy, name="downrod_mount_contacts_canopy")
    ctx.expect_contact(crossbar, downrod, name="crossbar_hub_contacts_downrod")
    ctx.expect_contact(left_head, crossbar, name="left_head_trunnion_contacts_yoke")
    ctx.expect_contact(right_head, crossbar, name="right_head_trunnion_contacts_yoke")
    ctx.expect_origin_distance(
        crossbar,
        canopy,
        axes="z",
        min_dist=0.64,
        max_dist=0.70,
        name="crossbar_hangs_below_canopy",
    )
    ctx.expect_origin_distance(
        left_head,
        right_head,
        axes="x",
        min_dist=0.92,
        max_dist=0.96,
        name="fan_heads_span_crossbar",
    )
    ctx.expect_origin_distance(
        left_head,
        right_head,
        axes="yz",
        max_dist=0.001,
        name="fan_heads_share_mounting_plane",
    )

    with ctx.pose(
        {
            swivel: 0.65,
            left_tilt: -0.45,
            right_tilt: 0.30,
        }
    ):
        ctx.expect_contact(crossbar, downrod, name="swiveled_crossbar_remains_mounted")
        ctx.expect_contact(left_head, crossbar, name="pitched_left_head_remains_mounted")
        ctx.expect_contact(right_head, crossbar, name="pitched_right_head_remains_mounted")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
