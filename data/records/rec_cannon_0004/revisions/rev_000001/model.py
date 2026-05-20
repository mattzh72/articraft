from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _ring_mesh(name: str, *, outer_radius: float, inner_radius: float, width: float):
    geometry = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        height=width,
        center=True,
    )
    geometry.rotate_x(math.pi / 2.0)
    return _save_mesh(name, geometry)


def _build_barrel_shell_mesh():
    outer_profile = [
        (0.180, -0.46),
        (0.186, -0.30),
        (0.178, -0.10),
        (0.168, 0.12),
        (0.158, 0.72),
        (0.149, 1.20),
        (0.160, 1.40),
        (0.150, 1.56),
    ]
    inner_profile = [
        (0.000, -0.46),
        (0.028, -0.36),
        (0.082, -0.10),
        (0.086, 0.90),
        (0.089, 1.53),
    ]
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    geometry.rotate_y(math.pi / 2.0)
    return geometry


def _add_wheel_visuals(part, *, prefix: str, wheel_wood, iron) -> None:
    part.visual(
        _ring_mesh(
            f"{prefix}_outer_tire.obj",
            outer_radius=0.76,
            inner_radius=0.72,
            width=0.048,
        ),
        material=iron,
        name="outer_tire",
    )
    part.visual(
        _ring_mesh(
            f"{prefix}_felloe.obj",
            outer_radius=0.72,
            inner_radius=0.58,
            width=0.10,
        ),
        material=wheel_wood,
        name="felloe",
    )
    part.visual(
        _ring_mesh(
            f"{prefix}_hub.obj",
            outer_radius=0.125,
            inner_radius=0.062,
            width=0.26,
        ),
        material=wheel_wood,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.0625, length=0.32),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub_sleeve",
    )
    part.visual(
        _ring_mesh(
            f"{prefix}_hub_band_outer.obj",
            outer_radius=0.150,
            inner_radius=0.124,
            width=0.038,
        ),
        origin=Origin(xyz=(0.0, 0.11, 0.0)),
        material=iron,
        name="hub_band_outer",
    )
    part.visual(
        _ring_mesh(
            f"{prefix}_hub_band_inner.obj",
            outer_radius=0.150,
            inner_radius=0.124,
            width=0.038,
        ),
        origin=Origin(xyz=(0.0, -0.11, 0.0)),
        material=iron,
        name="hub_band_inner",
    )
    for spoke_index in range(14):
        angle = 2.0 * math.pi * spoke_index / 14.0
        start_radius = 0.09
        end_radius = 0.62
        start_x = math.cos(angle) * start_radius
        start_z = math.sin(angle) * start_radius
        end_x = math.cos(angle) * end_radius
        end_z = math.sin(angle) * end_radius
        dx = end_x - start_x
        dz = end_z - start_z
        part.visual(
            Cylinder(radius=0.017, length=math.hypot(dx, dz) + 0.05),
            origin=Origin(
                xyz=((start_x + end_x) * 0.5, 0.0, (start_z + end_z) * 0.5),
                rpy=(0.0, math.atan2(dx, dz), 0.0),
            ),
            material=wheel_wood,
            name=f"spoke_{spoke_index:02d}",
        )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((lower + upper) * 0.5 for lower, upper in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civil_war_field_cannon", assets=ASSETS)

    barrel_iron = model.material("barrel_iron", rgba=(0.16, 0.17, 0.19, 1.0))
    wrought_iron = model.material("wrought_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    carriage_green = model.material("carriage_green", rgba=(0.36, 0.41, 0.23, 1.0))
    wheel_wood = model.material("wheel_wood", rgba=(0.57, 0.44, 0.26, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        Box((1.00, 0.12, 0.44)),
        origin=Origin(xyz=(-0.12, 0.34, 1.02)),
        material=carriage_green,
        name="left_cheek",
    )
    carriage.visual(
        Box((1.00, 0.12, 0.44)),
        origin=Origin(xyz=(-0.12, -0.34, 1.02)),
        material=carriage_green,
        name="right_cheek",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=1.70),
        origin=Origin(xyz=(-0.58, 0.0, 0.76), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wrought_iron,
        name="axle",
    )
    carriage.visual(
        Box((0.22, 0.68, 0.18)),
        origin=Origin(xyz=(0.14, 0.0, 0.76)),
        material=carriage_green,
        name="front_transom",
    )
    carriage.visual(
        Box((0.28, 0.78, 0.18)),
        origin=Origin(xyz=(-0.70, 0.0, 0.72)),
        material=carriage_green,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.30, 0.22, 0.10)),
        origin=Origin(xyz=(-0.56, 0.0, 0.72)),
        material=wheel_wood,
        name="elevating_block",
    )
    carriage.visual(
        Box((1.25, 0.12, 0.18)),
        origin=Origin(xyz=(-1.32, 0.16, 0.60)),
        material=carriage_green,
        name="left_trail_rail",
    )
    carriage.visual(
        Box((1.25, 0.12, 0.18)),
        origin=Origin(xyz=(-1.32, -0.16, 0.60)),
        material=carriage_green,
        name="right_trail_rail",
    )
    carriage.visual(
        Box((1.20, 0.30, 0.24)),
        origin=Origin(xyz=(-1.45, 0.0, 0.42)),
        material=carriage_green,
        name="trail_stock",
    )
    carriage.visual(
        Box((0.34, 0.28, 0.18)),
        origin=Origin(xyz=(-1.95, 0.0, 0.24)),
        material=carriage_green,
        name="trail_end",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((2.45, 1.80, 1.30)),
        mass=520.0,
        origin=Origin(xyz=(-0.80, 0.0, 0.64)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("barrel_shell.obj", _build_barrel_shell_mesh()),
        material=barrel_iron,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.190, length=0.14),
        origin=Origin(xyz=(-0.18, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_iron,
        name="breech_band",
    )
    barrel.visual(
        Cylinder(radius=0.172, length=0.08),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_iron,
        name="trunnion_reinforce",
    )
    barrel.visual(
        Cylinder(radius=0.162, length=0.12),
        origin=Origin(xyz=(1.40, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_iron,
        name="muzzle_band",
    )
    barrel.visual(
        Cylinder(radius=0.150, length=0.05),
        origin=Origin(xyz=(1.53, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_iron,
        name="muzzle_ring",
    )
    barrel.visual(
        Cylinder(radius=0.075, length=0.16),
        origin=Origin(xyz=(-0.54, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=barrel_iron,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.09),
        origin=Origin(xyz=(-0.66, 0.0, 0.0)),
        material=barrel_iron,
        name="cascabel",
    )
    barrel.visual(
        Cylinder(radius=0.055, length=0.27),
        origin=Origin(xyz=(0.0, 0.295, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=barrel_iron,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.055, length=0.27),
        origin=Origin(xyz=(0.0, -0.295, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=barrel_iron,
        name="right_trunnion",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.18, length=2.16),
        mass=760.0,
        origin=Origin(xyz=(0.54, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(left_wheel, prefix="left_wheel", wheel_wood=wheel_wood, iron=wrought_iron)
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.76, length=0.12),
        mass=135.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(right_wheel, prefix="right_wheel", wheel_wood=wheel_wood, iron=wrought_iron)
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.76, length=0.12),
        mass=135.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.7,
            lower=0.0,
            upper=math.radians(15.0),
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(-0.58, 0.84, 0.76)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=10.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(-0.58, -0.84, 0.76)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    barrel_elevation = object_model.get_articulation("barrel_elevation")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    axle = carriage.get_visual("axle")
    left_cheek = carriage.get_visual("left_cheek")
    right_cheek = carriage.get_visual("right_cheek")
    elevating_block = carriage.get_visual("elevating_block")
    front_transom = carriage.get_visual("front_transom")
    left_hub_sleeve = left_wheel.get_visual("hub_sleeve")
    right_hub_sleeve = right_wheel.get_visual("hub_sleeve")
    left_tire = left_wheel.get_visual("outer_tire")
    right_tire = right_wheel.get_visual("outer_tire")
    left_trunnion = barrel.get_visual("left_trunnion")
    right_trunnion = barrel.get_visual("right_trunnion")
    breech_band = barrel.get_visual("breech_band")
    muzzle_band = barrel.get_visual("muzzle_band")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        barrel,
        carriage,
        elem_a=left_trunnion,
        elem_b=left_cheek,
        reason="the left trunnion is carried in the left cheek bearing of the wooden carriage",
    )
    ctx.allow_overlap(
        barrel,
        carriage,
        elem_a=right_trunnion,
        elem_b=right_cheek,
        reason="the right trunnion is carried in the right cheek bearing of the wooden carriage",
    )
    ctx.allow_overlap(
        left_wheel,
        carriage,
        elem_a=left_hub_sleeve,
        elem_b=axle,
        reason="the left hub sleeve envelopes the axle spindle as a simplified bearing surface",
    )
    ctx.allow_overlap(
        right_wheel,
        carriage,
        elem_a=right_hub_sleeve,
        elem_b=axle,
        reason="the right hub sleeve envelopes the axle spindle as a simplified bearing surface",
    )

    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=48)

    ctx.check(
        "barrel_elevation_joint_type",
        barrel_elevation.articulation_type == ArticulationType.REVOLUTE,
        "barrel elevation should be a revolute trunnion joint",
    )
    ctx.check(
        "wheel_spin_joint_types",
        left_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_wheel_spin.articulation_type == ArticulationType.CONTINUOUS,
        "wheel axles should support continuous spinning motion",
    )
    ctx.check(
        "joint_axes_oriented_correctly",
        barrel_elevation.axis == (0.0, -1.0, 0.0)
        and left_wheel_spin.axis == (0.0, 1.0, 0.0)
        and right_wheel_spin.axis == (0.0, 1.0, 0.0),
        "barrel should elevate about the trunnion axis and both wheels should rotate about the axle line",
    )

    ctx.expect_within(
        barrel,
        carriage,
        axes="xz",
        inner_elem=left_trunnion,
        outer_elem=left_cheek,
    )
    ctx.expect_within(
        barrel,
        carriage,
        axes="xz",
        inner_elem=right_trunnion,
        outer_elem=right_cheek,
    )
    ctx.expect_within(
        carriage,
        left_wheel,
        axes="xz",
        inner_elem=axle,
        outer_elem=left_hub_sleeve,
    )
    ctx.expect_within(
        carriage,
        right_wheel,
        axes="xz",
        inner_elem=axle,
        outer_elem=right_hub_sleeve,
    )
    ctx.expect_gap(
        left_wheel,
        carriage,
        axis="y",
        min_gap=0.30,
        positive_elem=left_tire,
        negative_elem=left_cheek,
    )
    ctx.expect_gap(
        carriage,
        right_wheel,
        axis="y",
        min_gap=0.30,
        positive_elem=right_cheek,
        negative_elem=right_tire,
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="x",
        min_gap=1.05,
        positive_elem=muzzle_band,
        negative_elem=front_transom,
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        min_gap=0.04,
        max_gap=0.14,
        positive_elem=breech_band,
        negative_elem=elevating_block,
    )

    rest_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
    rest_left_spoke_aabb = ctx.part_element_world_aabb(left_wheel, elem="spoke_00")
    rest_right_spoke_aabb = ctx.part_element_world_aabb(right_wheel, elem="spoke_00")

    barrel_upper = barrel_elevation.motion_limits.upper if barrel_elevation.motion_limits is not None else None
    if barrel_upper is None:
        ctx.fail("barrel_upper_limit_present", "barrel elevation joint is missing an upper motion limit")
    else:
        with ctx.pose({barrel_elevation: barrel_upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="barrel_elevated_no_overlap")
            ctx.fail_if_isolated_parts(name="barrel_elevated_no_floating")
            ctx.expect_within(
                barrel,
                carriage,
                axes="xz",
                inner_elem=left_trunnion,
                outer_elem=left_cheek,
            )
            ctx.expect_within(
                barrel,
                carriage,
                axes="xz",
                inner_elem=right_trunnion,
                outer_elem=right_cheek,
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                min_gap=0.32,
                positive_elem=muzzle_band,
                negative_elem=front_transom,
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                min_gap=0.015,
                positive_elem=breech_band,
                negative_elem=elevating_block,
            )
            elevated_muzzle_aabb = ctx.part_element_world_aabb(barrel, elem=muzzle_band)
            if rest_muzzle_aabb is None or elevated_muzzle_aabb is None:
                ctx.fail("barrel_muzzle_pose_measurement", "could not resolve muzzle band AABB for pose test")
            else:
                ctx.check(
                    "barrel_elevation_raises_muzzle",
                    elevated_muzzle_aabb[1][2] > rest_muzzle_aabb[1][2] + 0.32,
                    "muzzle band should move upward substantially when the barrel elevates",
                )

    with ctx.pose({left_wheel_spin: math.pi / 2.0, right_wheel_spin: -math.pi / 3.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheels_spun_no_overlap")
        ctx.fail_if_isolated_parts(name="wheels_spun_no_floating")
        ctx.expect_within(
            carriage,
            left_wheel,
            axes="xz",
            inner_elem=axle,
            outer_elem=left_hub_sleeve,
        )
        ctx.expect_within(
            carriage,
            right_wheel,
            axes="xz",
            inner_elem=axle,
            outer_elem=right_hub_sleeve,
        )
        spun_left_spoke_aabb = ctx.part_element_world_aabb(left_wheel, elem="spoke_00")
        spun_right_spoke_aabb = ctx.part_element_world_aabb(right_wheel, elem="spoke_00")
        if rest_left_spoke_aabb is None or spun_left_spoke_aabb is None:
            ctx.fail("left_wheel_pose_measurement", "could not resolve left wheel spoke AABB for spin test")
        else:
            rest_left_center = _aabb_center(rest_left_spoke_aabb)
            spun_left_center = _aabb_center(spun_left_spoke_aabb)
            ctx.check(
                "left_wheel_spin_moves_spokes",
                abs(spun_left_center[0] - rest_left_center[0]) > 0.20
                and abs(spun_left_center[2] - rest_left_center[2]) > 0.20,
                "left wheel spoke should move around the axle when the wheel spins",
            )
        if rest_right_spoke_aabb is None or spun_right_spoke_aabb is None:
            ctx.fail("right_wheel_pose_measurement", "could not resolve right wheel spoke AABB for spin test")
        else:
            rest_right_center = _aabb_center(rest_right_spoke_aabb)
            spun_right_center = _aabb_center(spun_right_spoke_aabb)
            ctx.check(
                "right_wheel_spin_moves_spokes",
                abs(spun_right_center[0] - rest_right_center[0]) > 0.12
                and abs(spun_right_center[2] - rest_right_center[2]) > 0.12,
                "right wheel spoke should move around the axle when the wheel spins",
            )

    with ctx.pose({barrel_elevation: 0.30}):
        ctx.expect_gap(
            barrel,
            carriage,
            axis="z",
            min_gap=0.28,
            positive_elem=muzzle_band,
            negative_elem=front_transom,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
