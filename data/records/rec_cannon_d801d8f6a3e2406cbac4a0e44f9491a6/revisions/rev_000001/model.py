from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin, tau

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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _circle_points(radius: float, *, count: int = 16) -> list[tuple[float, float, float]]:
    return [
        (radius * cos((tau * index) / count), 0.0, radius * sin((tau * index) / count))
        for index in range(count)
    ]


def _cheek_mesh(*, thickness: float, notch_center_x: float, trunnion_radius: float) -> MeshGeometry:
    notch_left = notch_center_x - trunnion_radius * 1.35
    notch_right = notch_center_x + trunnion_radius * 1.35
    notch_floor = 1.04 - trunnion_radius
    profile = [
        (0.82, 0.66),
        (0.84, 0.84),
        (0.82, 1.02),
        (0.36, 1.03),
        (notch_right, 1.02),
        (notch_right, notch_floor),
        (notch_left, notch_floor),
        (notch_left, 1.02),
        (-0.02, 0.96),
        (-0.18, 0.88),
        (-0.28, 0.74),
        (-0.30, 0.66),
        (-0.16, 0.62),
        (0.18, 0.62),
        (0.60, 0.64),
    ]
    return ExtrudeGeometry(profile, thickness, center=True).rotate_x(pi / 2.0)


def _trail_beam_mesh(points: list[tuple[float, float, float]]) -> MeshGeometry:
    return sweep_profile_along_spline(
        points,
        profile=rounded_rect_profile(0.11, 0.18, radius=0.018, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def _wheel_mesh(
    *,
    wheel_radius: float,
    rim_radius: float,
    hub_radius: float,
    hub_length: float,
    flange_radius: float,
    flange_thickness: float,
    spoke_radius: float,
    spoke_count: int,
) -> MeshGeometry:
    wheel = MeshGeometry()
    wheel.merge(
        tube_from_spline_points(
            _circle_points(wheel_radius, count=18),
            radius=rim_radius,
            closed_spline=True,
            cap_ends=False,
            samples_per_segment=6,
            radial_segments=14,
            up_hint=(0.0, 1.0, 0.0),
        )
    )
    wheel.merge(CylinderGeometry(radius=hub_radius, height=hub_length, radial_segments=28).rotate_x(pi / 2.0))
    wheel.merge(
        CylinderGeometry(radius=flange_radius, height=flange_thickness, radial_segments=24)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.5 * (hub_length - flange_thickness), 0.0)
    )
    wheel.merge(
        CylinderGeometry(radius=flange_radius, height=flange_thickness, radial_segments=24)
        .rotate_x(pi / 2.0)
        .translate(0.0, -0.5 * (hub_length - flange_thickness), 0.0)
    )

    spoke_start = flange_radius * 0.92
    spoke_end = wheel_radius - rim_radius * 1.4
    for index in range(spoke_count):
        angle = (tau * index) / spoke_count
        spoke_points = [
            (spoke_start * cos(angle), 0.0, spoke_start * sin(angle)),
            (spoke_end * cos(angle), 0.0, spoke_end * sin(angle)),
        ]
        wheel.merge(
            tube_from_spline_points(
                spoke_points,
                radius=spoke_radius,
                samples_per_segment=1,
                radial_segments=10,
                cap_ends=True,
                up_hint=(0.0, 1.0, 0.0),
            )
        )
    return wheel


def _barrel_mesh() -> MeshGeometry:
    profile = [
        (0.0, -0.62),
        (0.045, -0.60),
        (0.072, -0.57),
        (0.060, -0.53),
        (0.095, -0.47),
        (0.126, -0.41),
        (0.166, -0.30),
        (0.175, -0.12),
        (0.182, 0.00),
        (0.174, 0.22),
        (0.160, 0.58),
        (0.145, 0.98),
        (0.128, 1.26),
        (0.116, 1.40),
        (0.102, 1.48),
        (0.038, 1.48),
        (0.0, -0.15),
        (0.038, -0.12),
    ]
    shell = LatheGeometry(profile, segments=88).rotate_y(pi / 2.0)

    trunnion = CylinderGeometry(radius=0.045, height=0.54, radial_segments=28).rotate_x(pi / 2.0)
    breech_ring = CylinderGeometry(radius=0.188, height=0.085, radial_segments=40).rotate_y(pi / 2.0).translate(-0.20, 0.0, 0.0)
    reinforce_ring = CylinderGeometry(radius=0.176, height=0.070, radial_segments=40).rotate_y(pi / 2.0).translate(-0.02, 0.0, 0.0)
    muzzle_astragal = CylinderGeometry(radius=0.120, height=0.030, radial_segments=32).rotate_y(pi / 2.0).translate(1.34, 0.0, 0.0)
    sight_block = BoxGeometry((0.055, 0.020, 0.018)).translate(-0.30, 0.0, 0.182)

    return _merge_geometries([shell, trunnion, breech_ring, reinforce_ring, muzzle_astragal, sight_block])


def _carriage_wood_mesh() -> MeshGeometry:
    cheek_thickness = 0.07
    notch_center_x = 0.18
    trunnion_radius = 0.045
    left_cheek = _cheek_mesh(
        thickness=cheek_thickness,
        notch_center_x=notch_center_x,
        trunnion_radius=trunnion_radius,
    ).translate(0.0, 0.23, 0.0)
    right_cheek = _cheek_mesh(
        thickness=cheek_thickness,
        notch_center_x=notch_center_x,
        trunnion_radius=trunnion_radius,
    ).translate(0.0, -0.23, 0.0)

    axle_bed = BoxGeometry((0.32, 0.50, 0.12)).translate(-0.02, 0.0, 0.73)
    front_transom = BoxGeometry((0.20, 0.44, 0.14)).translate(0.58, 0.0, 0.76)
    rear_transom = BoxGeometry((0.22, 0.40, 0.12)).translate(-0.10, 0.0, 0.67)
    trail_yoke = BoxGeometry((0.34, 0.26, 0.10)).translate(-0.42, 0.0, 0.58)
    trail_floor = BoxGeometry((0.86, 0.12, 0.06)).translate(-0.98, 0.0, 0.43)
    tail_block = BoxGeometry((0.12, 0.16, 0.16)).translate(-1.58, 0.0, 0.36)
    breech_bed = BoxGeometry((0.24, 0.22, 0.09)).translate(-0.24, 0.0, 0.79)

    left_beam = _trail_beam_mesh(
        [(-0.22, 0.14, 0.63), (-0.68, 0.11, 0.54), (-1.18, 0.08, 0.45), (-1.56, 0.05, 0.36)]
    )
    right_beam = _trail_beam_mesh(
        [(-0.22, -0.14, 0.63), (-0.68, -0.11, 0.54), (-1.18, -0.08, 0.45), (-1.56, -0.05, 0.36)]
    )

    return _merge_geometries(
        [
            left_cheek,
            right_cheek,
            axle_bed,
            front_transom,
            rear_transom,
            trail_yoke,
            trail_floor,
            tail_block,
            breech_bed,
            left_beam,
            right_beam,
        ]
    )


def _carriage_iron_mesh() -> MeshGeometry:
    axle = CylinderGeometry(radius=0.055, height=0.92, radial_segments=28).rotate_x(pi / 2.0).translate(-0.05, 0.0, 0.72)
    left_washer = CylinderGeometry(radius=0.078, height=0.035, radial_segments=24).rotate_x(pi / 2.0).translate(-0.05, 0.4425, 0.72)
    right_washer = CylinderGeometry(radius=0.078, height=0.035, radial_segments=24).rotate_x(pi / 2.0).translate(-0.05, -0.4425, 0.72)
    left_trunnion_cap = BoxGeometry((0.18, 0.018, 0.026)).translate(0.18, 0.23, 1.099)
    right_trunnion_cap = BoxGeometry((0.18, 0.018, 0.026)).translate(0.18, -0.23, 1.099)
    pintle_plate = BoxGeometry((0.22, 0.10, 0.008)).translate(-1.58, 0.0, 0.44)
    pintle_hook = wire_from_points(
        [(-1.64, 0.0, 0.44), (-1.60, 0.0, 0.50), (-1.54, 0.0, 0.50), (-1.50, 0.0, 0.44)],
        radius=0.010,
        radial_segments=12,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.018,
        corner_segments=8,
        up_hint=(0.0, 1.0, 0.0),
    )
    return _merge_geometries(
        [
            axle,
            left_washer,
            right_washer,
            left_trunnion_cap,
            right_trunnion_cap,
            pintle_plate,
            pintle_hook,
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civil_war_3in_ordnance_rifle")

    oak = model.material("oak", rgba=(0.53, 0.37, 0.20, 1.0))
    wrought_iron = model.material("wrought_iron", rgba=(0.22, 0.24, 0.27, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.52, 0.55, 0.58, 1.0))

    carriage = model.part("carriage")
    carriage.visual(_save_mesh("carriage_wood", _carriage_wood_mesh()), material=oak, name="carriage_wood")
    carriage.visual(_save_mesh("carriage_iron", _carriage_iron_mesh()), material=dark_iron, name="carriage_iron")
    carriage.visual(
        Box((0.22, 0.10, 0.008)),
        origin=Origin(xyz=(-1.58, 0.0, 0.44)),
        material=dark_iron,
        name="pintle_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((2.50, 1.10, 1.10)),
        mass=620.0,
        origin=Origin(xyz=(-0.38, 0.0, 0.62)),
    )

    barrel = model.part("barrel")
    barrel.visual(_save_mesh("ordnance_rifle_barrel", _barrel_mesh()), material=wrought_iron, name="barrel_shell")
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=2.10),
        mass=410.0,
        origin=Origin(xyz=(0.38, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    wheel_mesh = _save_mesh(
        "ordnance_rifle_wheel",
        _wheel_mesh(
            wheel_radius=0.72,
            rim_radius=0.015,
            hub_radius=0.095,
            hub_length=0.54,
            flange_radius=0.135,
            flange_thickness=0.050,
            spoke_radius=0.010,
            spoke_count=14,
        ),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(wheel_mesh, material=dark_iron, name="wheel_assembly")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=0.54),
        mass=84.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(wheel_mesh, material=dark_iron, name="wheel_assembly")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=0.54),
        mass=84.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.18, 0.0, 1.04)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.45,
            lower=-0.18,
            upper=0.42,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(-0.05, 0.73, 0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=8.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(-0.05, -0.73, 0.72)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    barrel_elevation = object_model.get_articulation("barrel_elevation")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    carriage.get_visual("pintle_plate")
    barrel.get_visual("barrel_shell")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.003)
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

    ctx.expect_contact(barrel, carriage, contact_tol=0.003, name="barrel_seated_on_trunnions")
    ctx.expect_contact(left_wheel, carriage, contact_tol=0.001, name="left_wheel_seated_on_axle")
    ctx.expect_contact(right_wheel, carriage, contact_tol=0.001, name="right_wheel_seated_on_axle")
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="y", min_dist=1.40, max_dist=1.50, name="wheel_track_width")

    ctx.check(
        "barrel_joint_axis_is_lateral",
        tuple(barrel_elevation.axis) == (0.0, 1.0, 0.0),
        f"Expected barrel elevation axis (0,1,0), got {barrel_elevation.axis}.",
    )
    ctx.check(
        "wheel_axes_are_lateral",
        tuple(left_wheel_spin.axis) == (0.0, 1.0, 0.0) and tuple(right_wheel_spin.axis) == (0.0, 1.0, 0.0),
        f"Wheel axes were {left_wheel_spin.axis} and {right_wheel_spin.axis}.",
    )

    barrel_rest_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    assert barrel_rest_aabb is not None
    with ctx.pose({barrel_elevation: 0.35}):
        barrel_up_aabb = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
        assert barrel_up_aabb is not None
        ctx.check(
            "barrel_elevates_muzzle",
            barrel_up_aabb[1][2] > barrel_rest_aabb[1][2] + 0.19,
            f"Barrel max z only changed from {barrel_rest_aabb[1][2]:.3f} to {barrel_up_aabb[1][2]:.3f}.",
        )

    with ctx.pose({left_wheel_spin: 1.1, right_wheel_spin: -0.9}):
        ctx.expect_contact(left_wheel, carriage, contact_tol=0.001, name="left_wheel_remains_on_axle_when_spun")
        ctx.expect_contact(right_wheel, carriage, contact_tol=0.001, name="right_wheel_remains_on_axle_when_spun")

    carriage_aabb = ctx.part_world_aabb(carriage)
    pintle_aabb = ctx.part_element_world_aabb(carriage, elem="pintle_plate")
    assert carriage_aabb is not None and pintle_aabb is not None
    ctx.check(
        "pintle_plate_is_at_trail_end",
        pintle_aabb[0][0] <= carriage_aabb[0][0] + 0.08,
        f"Pintle plate min x {pintle_aabb[0][0]:.3f} was not near carriage trail end {carriage_aabb[0][0]:.3f}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
