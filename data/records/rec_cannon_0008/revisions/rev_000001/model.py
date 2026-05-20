from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent
SCALE = 3.0


def _s(value: float) -> float:
    return SCALE * value


def _sv(values: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(SCALE * value for value in values)


def _scaled_origin(
    *, xyz: tuple[float, float, float] = (0.0, 0.0, 0.0), rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
) -> Origin:
    return Origin(xyz=_sv(xyz), rpy=rpy)


def _scaled_profile(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(SCALE * radius, SCALE * z_pos) for radius, z_pos in points]


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _add_cheek(part, *, side_y: float, material) -> None:
    for x_pos, z_pos, length, height, angle_deg, name in [
        (-0.24, 0.23, 0.34, 0.06, 0.0, "tail_lower"),
        (-0.06, 0.30, 0.38, 0.06, 8.0, "tail_rise"),
        (0.11, 0.39, 0.30, 0.055, 18.0, "mid_rise"),
        (0.23, 0.46, 0.16, 0.050, 28.0, "trunnion_cheek"),
        (0.27, 0.31, 0.09, 0.16, 0.0, "front_upright"),
        (-0.33, 0.25, 0.08, 0.12, 0.0, "rear_upright"),
    ]:
        part.visual(
            Box(_sv((length, 0.05, height))),
            origin=_scaled_origin(
                xyz=(x_pos, side_y, z_pos),
                rpy=(0.0, 0.0, math.radians(angle_deg)),
            ),
            material=material,
            name=f"{'left' if side_y > 0.0 else 'right'}_{name}",
        )


def _add_wheel(part, *, material_wood, material_iron, inner_side_sign: float) -> None:
    felloe_radius = _s(0.155)
    tire_radius = _s(0.174)
    for angle_deg in range(0, 360, 30):
        angle = math.radians(angle_deg)
        tangent_angle = angle + (math.pi / 2.0)
        wood_x = math.cos(angle) * felloe_radius
        wood_z = math.sin(angle) * felloe_radius
        tire_x = math.cos(angle) * tire_radius
        tire_z = math.sin(angle) * tire_radius
        part.visual(
            Box(_sv((0.094, 0.040, 0.032))),
            origin=Origin(xyz=(wood_x, 0.0, wood_z), rpy=(0.0, tangent_angle, 0.0)),
            material=material_wood,
            name=f"felloe_segment_{angle_deg}",
        )
        part.visual(
            Box(_sv((0.096, 0.048, 0.012))),
            origin=Origin(xyz=(tire_x, 0.0, tire_z), rpy=(0.0, tangent_angle, 0.0)),
            material=material_iron,
            name=f"tire_segment_{angle_deg}",
        )
    for angle_deg in range(0, 360, 30):
        angle = math.radians(angle_deg)
        part.visual(
            Box(_sv((0.29, 0.020, 0.026))),
            origin=Origin(rpy=(0.0, angle, 0.0)),
            material=material_wood,
            name=f"spoke_{angle_deg}",
        )
    part.visual(
        Cylinder(radius=_s(0.033), length=_s(0.070)),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material_wood,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=_s(0.024), length=_s(0.045)),
        origin=_scaled_origin(xyz=(0.0, inner_side_sign * 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material_iron,
        name="inner_hub",
    )
    part.visual(
        Cylinder(radius=_s(0.024), length=_s(0.035)),
        origin=_scaled_origin(xyz=(0.0, -inner_side_sign * 0.038, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material_iron,
        name="outer_hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="siege_cannon", assets=ASSETS)

    iron_dark = model.material("iron_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    iron_mid = model.material("iron_mid", rgba=(0.31, 0.33, 0.36, 1.0))
    timber = model.material("timber", rgba=(0.47, 0.31, 0.17, 1.0))
    timber_dark = model.material("timber_dark", rgba=(0.34, 0.23, 0.12, 1.0))
    band_iron = model.material("band_iron", rgba=(0.43, 0.45, 0.47, 1.0))

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box(_sv((1.05, 0.48, 0.34))),
        mass=2600.0,
        origin=_scaled_origin(xyz=(-0.02, 0.0, 0.28)),
    )
    _add_cheek(carriage, side_y=0.165, material=timber)
    _add_cheek(carriage, side_y=-0.165, material=timber)
    carriage.visual(
        Box(_sv((0.56, 0.28, 0.060))),
        origin=_scaled_origin(xyz=(-0.08, 0.0, 0.250)),
        material=timber_dark,
        name="bed",
    )
    carriage.visual(
        Box(_sv((0.10, 0.32, 0.11))),
        origin=_scaled_origin(xyz=(0.25, 0.0, 0.275)),
        material=timber_dark,
        name="front_transom",
    )
    carriage.visual(
        Box(_sv((0.12, 0.32, 0.10))),
        origin=_scaled_origin(xyz=(-0.28, 0.0, 0.245)),
        material=timber_dark,
        name="rear_transom",
    )
    carriage.visual(
        Box(_sv((0.24, 0.18, 0.07))),
        origin=_scaled_origin(xyz=(-0.42, 0.0, 0.225)),
        material=timber_dark,
        name="trail_block",
    )
    carriage.visual(
        Box(_sv((0.16, 0.30, 0.06))),
        origin=_scaled_origin(xyz=(0.04, 0.0, 0.205)),
        material=timber_dark,
        name="front_chassis_beam",
    )
    carriage.visual(
        Box(_sv((0.18, 0.30, 0.06))),
        origin=_scaled_origin(xyz=(-0.18, 0.0, 0.205)),
        material=timber_dark,
        name="rear_chassis_beam",
    )
    carriage.visual(
        Cylinder(radius=_s(0.035), length=_s(0.35)),
        origin=_scaled_origin(xyz=(0.20, 0.0, 0.174), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=timber_dark,
        name="front_axle",
    )
    carriage.visual(
        Cylinder(radius=_s(0.035), length=_s(0.35)),
        origin=_scaled_origin(xyz=(-0.26, 0.0, 0.174), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=timber_dark,
        name="rear_axle",
    )
    carriage.visual(
        Box(_sv((0.09, 0.30, 0.08))),
        origin=_scaled_origin(xyz=(0.142, 0.0, 0.190)),
        material=timber_dark,
        name="front_axle_saddle",
    )
    for name, x_pos, y_pos in [
        ("front_left_axle_stub", 0.20, 0.195),
        ("front_right_axle_stub", 0.20, -0.195),
        ("rear_left_axle_stub", -0.26, 0.195),
        ("rear_right_axle_stub", -0.26, -0.195),
    ]:
        carriage.visual(
            Cylinder(radius=_s(0.024), length=_s(0.045)),
            origin=_scaled_origin(xyz=(x_pos, y_pos, 0.174), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=band_iron,
            name=name,
        )
    carriage.visual(
        Box(_sv((0.07, 0.030, 0.070))),
        origin=_scaled_origin(xyz=(0.0, 0.167, 0.470)),
        material=band_iron,
        name="left_trunnion_seat",
    )
    carriage.visual(
        Box(_sv((0.07, 0.030, 0.070))),
        origin=_scaled_origin(xyz=(0.0, -0.167, 0.470)),
        material=band_iron,
        name="right_trunnion_seat",
    )
    carriage.visual(
        Box(_sv((0.14, 0.03, 0.09))),
        origin=_scaled_origin(xyz=(0.09, 0.167, 0.390)),
        material=timber_dark,
        name="left_trunnion_bridge",
    )
    carriage.visual(
        Box(_sv((0.14, 0.03, 0.09))),
        origin=_scaled_origin(xyz=(0.09, -0.167, 0.390)),
        material=timber_dark,
        name="right_trunnion_bridge",
    )

    barrel_shell = _save_mesh(
        "barrel_shell.obj",
        LatheGeometry.from_shell_profiles(
            _scaled_profile(
                [
                    (0.088, 0.00),
                    (0.098, 0.06),
                    (0.094, 0.16),
                    (0.086, 0.42),
                    (0.078, 0.76),
                    (0.070, 1.05),
                    (0.073, 1.12),
                ]
            ),
            _scaled_profile(
                [
                    (0.022, 0.10),
                    (0.031, 0.16),
                    (0.034, 0.52),
                    (0.038, 1.02),
                    (0.040, 1.12),
                ]
            ),
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
    )

    barrel = model.part("barrel")
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=_s(0.090), length=_s(1.25)),
        mass=5200.0,
        origin=_scaled_origin(xyz=(0.38, 0.0, 0.02), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    barrel.visual(
        barrel_shell,
        origin=_scaled_origin(xyz=(-0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_dark,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=_s(0.029), length=_s(0.304)),
        origin=_scaled_origin(xyz=(0.0, 0.0, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron_mid,
        name="trunnion_axle",
    )
    barrel.visual(
        Cylinder(radius=_s(0.100), length=_s(0.110)),
        origin=_scaled_origin(xyz=(-0.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="breech_reinforce",
    )
    barrel.visual(
        Cylinder(radius=_s(0.087), length=_s(0.120)),
        origin=_scaled_origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="first_reinforce_band",
    )
    barrel.visual(
        Cylinder(radius=_s(0.078), length=_s(0.160)),
        origin=_scaled_origin(xyz=(0.37, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="second_reinforce_band",
    )
    barrel.visual(
        Cylinder(radius=_s(0.076), length=_s(0.070)),
        origin=_scaled_origin(xyz=(0.84, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=_s(0.026), length=_s(0.100)),
        origin=_scaled_origin(xyz=(-0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron_mid,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=_s(0.048)),
        origin=_scaled_origin(xyz=(-0.34, 0.0, 0.0)),
        material=iron_mid,
        name="cascabel",
    )

    def add_wheel(name: str, *, inner_side_sign: float) -> None:
        wheel = model.part(name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=_s(0.174), length=_s(0.070)),
            mass=240.0,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        _add_wheel(wheel, material_wood=timber, material_iron=band_iron, inner_side_sign=inner_side_sign)

    add_wheel("front_left_wheel", inner_side_sign=-1.0)
    add_wheel("front_right_wheel", inner_side_sign=1.0)
    add_wheel("rear_left_wheel", inner_side_sign=-1.0)
    add_wheel("rear_right_wheel", inner_side_sign=1.0)

    model.articulation(
        "barrel_pitch",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=_scaled_origin(xyz=(0.0, 0.0, 0.470)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=32000.0, velocity=0.55, lower=-0.06, upper=0.28),
    )
    for joint_name, child_name, x_pos, y_pos in [
        ("front_left_wheel_spin", "front_left_wheel", 0.20, 0.270),
        ("front_right_wheel_spin", "front_right_wheel", 0.20, -0.270),
        ("rear_left_wheel_spin", "rear_left_wheel", -0.26, 0.270),
        ("rear_right_wheel_spin", "rear_right_wheel", -0.26, -0.270),
    ]:
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=carriage,
            child=child_name,
            origin=_scaled_origin(xyz=(x_pos, y_pos, 0.174)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1600.0, velocity=12.0, lower=-math.tau, upper=math.tau),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    barrel_pitch = object_model.get_articulation("barrel_pitch")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")

    left_trunnion_seat = carriage.get_visual("left_trunnion_seat")
    right_trunnion_seat = carriage.get_visual("right_trunnion_seat")
    bed = carriage.get_visual("bed")
    front_transom = carriage.get_visual("front_transom")
    front_left_axle_stub = carriage.get_visual("front_left_axle_stub")
    front_right_axle_stub = carriage.get_visual("front_right_axle_stub")
    rear_left_axle_stub = carriage.get_visual("rear_left_axle_stub")
    rear_right_axle_stub = carriage.get_visual("rear_right_axle_stub")

    trunnion_axle = barrel.get_visual("trunnion_axle")
    cascabel = barrel.get_visual("cascabel")
    muzzle_band = barrel.get_visual("muzzle_band")

    front_left_inner_hub = front_left_wheel.get_visual("inner_hub")
    front_right_inner_hub = front_right_wheel.get_visual("inner_hub")
    rear_left_inner_hub = rear_left_wheel.get_visual("inner_hub")
    rear_right_inner_hub = rear_right_wheel.get_visual("inner_hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=12)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check(
        "barrel_pitch_axis",
        barrel_pitch.axis == (0.0, -1.0, 0.0),
        details=f"Expected barrel pitch axis (0,-1,0), got {barrel_pitch.axis!r}.",
    )
    for joint_obj in (
        front_left_wheel_spin,
        front_right_wheel_spin,
        rear_left_wheel_spin,
        rear_right_wheel_spin,
    ):
        ctx.check(
            f"{joint_obj.name}_axis",
            joint_obj.axis == (0.0, 1.0, 0.0),
            details=f"Expected axle axis (0,1,0), got {joint_obj.axis!r}.",
        )

    carriage_aabb = ctx.part_world_aabb(carriage)
    barrel_aabb = ctx.part_world_aabb(barrel)
    front_left_aabb = ctx.part_world_aabb(front_left_wheel)
    front_right_aabb = ctx.part_world_aabb(front_right_wheel)
    rear_left_aabb = ctx.part_world_aabb(rear_left_wheel)
    rear_right_aabb = ctx.part_world_aabb(rear_right_wheel)

    if carriage_aabb is not None and barrel_aabb is not None and front_left_aabb is not None:
        carriage_length = carriage_aabb[1][0] - carriage_aabb[0][0]
        barrel_length = barrel_aabb[1][0] - barrel_aabb[0][0]
        wheel_diameter = front_left_aabb[1][2] - front_left_aabb[0][2]
        ctx.check(
            "siege_scale",
            carriage_length > 2.4 and barrel_length > 3.4 and wheel_diameter > 1.0,
            details=(
                f"Expected large siege-gun scale, got carriage_length={carriage_length:.3f}, "
                f"barrel_length={barrel_length:.3f}, wheel_diameter={wheel_diameter:.3f}."
            ),
        )

    wheel_min_zs = []
    for wheel_aabb in (front_left_aabb, front_right_aabb, rear_left_aabb, rear_right_aabb):
        if wheel_aabb is not None:
            wheel_min_zs.append(wheel_aabb[0][2])
    if len(wheel_min_zs) == 4:
        ctx.check(
            "wheels_share_ground_plane",
            max(abs(value) for value in wheel_min_zs) < 0.08 and (max(wheel_min_zs) - min(wheel_min_zs)) < 0.01,
            details=f"Wheel ground-plane z minima were {wheel_min_zs!r}.",
        )

    ctx.expect_overlap(barrel, carriage, axes="xy", min_overlap=_s(0.12))
    ctx.expect_origin_distance(barrel, carriage, axes="y", max_dist=0.02)
    ctx.expect_gap(
        carriage,
        barrel,
        axis="y",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem=left_trunnion_seat,
        negative_elem=trunnion_axle,
        name="left_trunnion_bearing_seat",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="y",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem=trunnion_axle,
        negative_elem=right_trunnion_seat,
        name="right_trunnion_bearing_seat",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        min_gap=_s(0.12),
        positive_elem=cascabel,
        negative_elem=bed,
        name="cascabel_clears_bed_at_rest",
    )

    ctx.expect_gap(
        front_left_wheel,
        carriage,
        axis="y",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem=front_left_inner_hub,
        negative_elem=front_left_axle_stub,
        name="front_left_wheel_seated_on_stub",
    )
    ctx.expect_gap(
        carriage,
        front_right_wheel,
        axis="y",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem=front_right_axle_stub,
        negative_elem=front_right_inner_hub,
        name="front_right_wheel_seated_on_stub",
    )
    ctx.expect_gap(
        rear_left_wheel,
        carriage,
        axis="y",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem=rear_left_inner_hub,
        negative_elem=rear_left_axle_stub,
        name="rear_left_wheel_seated_on_stub",
    )
    ctx.expect_gap(
        carriage,
        rear_right_wheel,
        axis="y",
        max_gap=0.004,
        max_penetration=0.001,
        positive_elem=rear_right_axle_stub,
        negative_elem=rear_right_inner_hub,
        name="rear_right_wheel_seated_on_stub",
    )

    barrel_limits = barrel_pitch.motion_limits
    ctx.check(
        "barrel_pitch_limits_present",
        barrel_limits is not None and barrel_limits.lower is not None and barrel_limits.upper is not None,
        details="Barrel pitch needs finite depression and elevation limits.",
    )

    rest_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
    rest_muzzle_top = rest_muzzle_aabb[1][2] if rest_muzzle_aabb is not None else None

    if barrel_limits is not None and barrel_limits.lower is not None and barrel_limits.upper is not None:
        with ctx.pose({barrel_pitch: barrel_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="barrel_pitch_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="barrel_pitch_upper_no_floating")
            ctx.expect_gap(
                carriage,
                barrel,
                axis="y",
                max_gap=0.004,
                max_penetration=0.001,
                positive_elem=left_trunnion_seat,
                negative_elem=trunnion_axle,
                name="left_trunnion_contact_at_full_elevation",
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="y",
                max_gap=0.004,
                max_penetration=0.001,
                positive_elem=trunnion_axle,
                negative_elem=right_trunnion_seat,
                name="right_trunnion_contact_at_full_elevation",
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                min_gap=_s(0.03),
                positive_elem=cascabel,
                negative_elem=bed,
                name="elevated_cascabel_still_clears_bed",
            )
            elevated_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
            if elevated_muzzle_aabb is not None and rest_muzzle_top is not None:
                elevated_muzzle_top = elevated_muzzle_aabb[1][2]
                ctx.check(
                    "muzzle_rises_when_elevated",
                    elevated_muzzle_top > rest_muzzle_top + _s(0.10),
                    details=(
                        f"Rest muzzle top={rest_muzzle_top:.3f}, "
                        f"elevated muzzle top={elevated_muzzle_top:.3f}."
                    ),
                )

        with ctx.pose({barrel_pitch: barrel_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="barrel_pitch_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="barrel_pitch_lower_no_floating")
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                min_gap=_s(0.008),
                positive_elem=muzzle_band,
                negative_elem=front_transom,
                name="depressed_muzzle_clears_front_transom",
            )
            lowered_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
            if lowered_muzzle_aabb is not None and rest_muzzle_top is not None:
                lowered_muzzle_top = lowered_muzzle_aabb[1][2]
                ctx.check(
                    "muzzle_drops_when_depressed",
                    lowered_muzzle_top < rest_muzzle_top - _s(0.03),
                    details=(
                        f"Rest muzzle top={rest_muzzle_top:.3f}, "
                        f"lowered muzzle top={lowered_muzzle_top:.3f}."
                    ),
                )

    with ctx.pose(
        {
            front_left_wheel_spin: 1.7,
            front_right_wheel_spin: -2.4,
            rear_left_wheel_spin: 0.9,
            rear_right_wheel_spin: -1.2,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.expect_gap(
            front_left_wheel,
            carriage,
            axis="y",
            max_gap=0.004,
            max_penetration=0.001,
            positive_elem=front_left_inner_hub,
            negative_elem=front_left_axle_stub,
            name="front_left_wheel_stays_seated_when_spun",
        )
        ctx.expect_gap(
            carriage,
            front_right_wheel,
            axis="y",
            max_gap=0.004,
            max_penetration=0.001,
            positive_elem=front_right_axle_stub,
            negative_elem=front_right_inner_hub,
            name="front_right_wheel_stays_seated_when_spun",
        )
        ctx.expect_gap(
            rear_left_wheel,
            carriage,
            axis="y",
            max_gap=0.004,
            max_penetration=0.001,
            positive_elem=rear_left_inner_hub,
            negative_elem=rear_left_axle_stub,
            name="rear_left_wheel_stays_seated_when_spun",
        )
        ctx.expect_gap(
            carriage,
            rear_right_wheel,
            axis="y",
            max_gap=0.004,
            max_penetration=0.001,
            positive_elem=rear_right_axle_stub,
            negative_elem=rear_right_inner_hub,
            name="rear_right_wheel_stays_seated_when_spun",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
