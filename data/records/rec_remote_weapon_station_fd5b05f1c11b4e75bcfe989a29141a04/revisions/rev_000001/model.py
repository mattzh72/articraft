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
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_coord, z_center + z_coord)
        for y_coord, z_coord in rounded_rect_profile(width, height, radius)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_remote_weapon_station")

    deck_paint = model.material("deck_paint", rgba=(0.23, 0.24, 0.26, 1.0))
    turret_gray = model.material("turret_gray", rgba=(0.32, 0.34, 0.36, 1.0))
    armor_green = model.material("armor_green", rgba=(0.36, 0.40, 0.30, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.10, 0.18, 0.20, 0.65))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.42, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=deck_paint,
        name="deck_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.17, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=turret_gray,
        name="support_column",
    )
    pedestal.visual(
        Cylinder(radius=0.24, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=turret_gray,
        name="bearing_pedestal",
    )
    pedestal.visual(
        Cylinder(radius=0.305, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
        material=steel_dark,
        name="slew_race",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.84, 0.84, 0.58)),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    turret_fork = model.part("turret_fork")
    ring_shell = LatheGeometry.from_shell_profiles(
        [
            (0.305, -0.340),
            (0.330, -0.285),
            (0.332, -0.252),
            (0.314, -0.228),
            (0.280, -0.220),
        ],
        [
            (0.214, -0.332),
            (0.214, -0.250),
            (0.206, -0.220),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    turret_fork.visual(
        mesh_from_geometry(ring_shell, "turret_ring_shell"),
        material=turret_gray,
        name="ring_shell",
    )
    turret_fork.visual(
        Box((0.42, 0.06, 0.44)),
        origin=Origin(xyz=(0.04, 0.235, 0.0)),
        material=turret_gray,
        name="left_arm",
    )
    turret_fork.visual(
        Box((0.42, 0.06, 0.44)),
        origin=Origin(xyz=(0.04, -0.235, 0.0)),
        material=turret_gray,
        name="right_arm",
    )
    turret_fork.visual(
        Box((0.22, 0.04, 0.24)),
        origin=Origin(xyz=(-0.07, 0.277, -0.07)),
        material=turret_gray,
        name="left_gusset",
    )
    turret_fork.visual(
        Box((0.22, 0.04, 0.24)),
        origin=Origin(xyz=(-0.07, -0.277, -0.07)),
        material=turret_gray,
        name="right_gusset",
    )
    turret_fork.visual(
        Box((0.16, 0.46, 0.05)),
        origin=Origin(xyz=(-0.12, 0.0, 0.235)),
        material=turret_gray,
        name="rear_bridge",
    )
    turret_fork.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.01, 0.255, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_bearing_housing",
    )
    turret_fork.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.01, -0.255, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_bearing_housing",
    )
    turret_fork.inertial = Inertial.from_geometry(
        Box((0.66, 0.66, 0.58)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    weapon_housing = model.part("weapon_housing")
    weapon_shell = section_loft(
        [
            _yz_section(-0.20, width=0.22, height=0.22, radius=0.035, z_center=0.02),
            _yz_section(0.00, width=0.32, height=0.30, radius=0.045, z_center=0.03),
            _yz_section(0.34, width=0.30, height=0.26, radius=0.040, z_center=0.00),
            _yz_section(0.66, width=0.18, height=0.18, radius=0.026, z_center=-0.03),
        ]
    )
    weapon_housing.visual(
        mesh_from_geometry(weapon_shell, "weapon_housing_shell"),
        material=armor_green,
        name="housing_shell",
    )
    weapon_housing.visual(
        Box((0.34, 0.22, 0.14)),
        origin=Origin(xyz=(0.10, 0.0, -0.13)),
        material=armor_green,
        name="breech_block",
    )
    weapon_housing.visual(
        Box((0.10, 0.08, 0.06)),
        origin=Origin(xyz=(0.34, 0.0, 0.145)),
        material=glass_dark,
        name="optic_block",
    )
    weapon_housing.visual(
        Box((0.20, 0.16, 0.008)),
        origin=Origin(xyz=(0.09, 0.0, 0.176)),
        material=armor_green,
        name="hatch_seat",
    )
    weapon_housing.visual(
        Cylinder(radius=0.060, length=0.06),
        origin=Origin(xyz=(0.02, 0.175, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_trunnion",
    )
    weapon_housing.visual(
        Cylinder(radius=0.060, length=0.06),
        origin=Origin(xyz=(0.02, -0.175, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_trunnion",
    )
    weapon_housing.visual(
        Cylinder(radius=0.070, length=0.24),
        origin=Origin(xyz=(0.78, 0.0, -0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="barrel_shroud",
    )
    weapon_housing.visual(
        Cylinder(radius=0.030, length=0.58),
        origin=Origin(xyz=(1.19, 0.0, -0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="muzzle_tube",
    )
    weapon_housing.inertial = Inertial.from_geometry(
        Box((1.68, 0.32, 0.40)),
        mass=145.0,
        origin=Origin(xyz=(0.62, 0.0, -0.02)),
    )

    access_hatch = model.part("access_hatch")
    access_hatch.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="hinge_barrel",
    )
    access_hatch.visual(
        Box((0.18, 0.12, 0.018)),
        origin=Origin(xyz=(0.09, 0.0, 0.009)),
        material=armor_green,
        name="hatch_panel",
    )
    access_hatch.visual(
        Box((0.05, 0.012, 0.010)),
        origin=Origin(xyz=(0.12, 0.0, 0.023)),
        material=steel_dark,
        name="hatch_handle",
    )
    access_hatch.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.034)),
        mass=8.0,
        origin=Origin(xyz=(0.09, 0.0, 0.017)),
    )

    model.articulation(
        "pedestal_to_turret",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turret_fork,
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=320.0, velocity=0.8),
    )
    model.articulation(
        "turret_to_weapon",
        ArticulationType.REVOLUTE,
        parent=turret_fork,
        child=weapon_housing,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.7,
            lower=math.radians(-18.0),
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "weapon_to_hatch",
        ArticulationType.REVOLUTE,
        parent=weapon_housing,
        child=access_hatch,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
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

    pedestal = object_model.get_part("pedestal")
    turret_fork = object_model.get_part("turret_fork")
    weapon_housing = object_model.get_part("weapon_housing")
    access_hatch = object_model.get_part("access_hatch")

    slew = object_model.get_articulation("pedestal_to_turret")
    elevation = object_model.get_articulation("turret_to_weapon")
    hatch_hinge = object_model.get_articulation("weapon_to_hatch")

    ctx.check(
        "all required parts are present",
        all(part is not None for part in (pedestal, turret_fork, weapon_housing, access_hatch)),
    )
    ctx.check(
        "articulation stack matches prompt",
        slew.articulation_type == ArticulationType.CONTINUOUS
        and tuple(slew.axis) == (0.0, 0.0, 1.0)
        and elevation.articulation_type == ArticulationType.REVOLUTE
        and tuple(elevation.axis) == (0.0, -1.0, 0.0)
        and hatch_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hatch_hinge.axis) == (0.0, -1.0, 0.0),
        details=(
            f"slew={slew.articulation_type, slew.axis}, "
            f"elevation={elevation.articulation_type, elevation.axis}, "
            f"hatch={hatch_hinge.articulation_type, hatch_hinge.axis}"
        ),
    )

    with ctx.pose({slew: 0.0, elevation: 0.0, hatch_hinge: 0.0}):
        ctx.expect_contact(
            weapon_housing,
            turret_fork,
            elem_a="left_trunnion",
            elem_b="left_arm",
            contact_tol=0.0005,
            name="left trunnion bears on the left fork arm",
        )
        ctx.expect_contact(
            weapon_housing,
            turret_fork,
            elem_a="right_trunnion",
            elem_b="right_arm",
            contact_tol=0.0005,
            name="right trunnion bears on the right fork arm",
        )
        ctx.expect_gap(
            access_hatch,
            weapon_housing,
            axis="z",
            positive_elem="hatch_panel",
            negative_elem="hatch_seat",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed hatch seats on roof coaming",
        )
        ctx.expect_overlap(
            access_hatch,
            weapon_housing,
            axes="xy",
            elem_a="hatch_panel",
            elem_b="hatch_seat",
            min_overlap=0.10,
            name="closed hatch covers the access opening",
        )

        muzzle_rest = _aabb_center(ctx.part_element_world_aabb(weapon_housing, elem="muzzle_tube"))

    with ctx.pose({slew: math.pi / 2.0, elevation: 0.0, hatch_hinge: 0.0}):
        muzzle_yawed = _aabb_center(ctx.part_element_world_aabb(weapon_housing, elem="muzzle_tube"))

    ctx.check(
        "turret yaw swings muzzle around vertical axis",
        muzzle_rest is not None
        and muzzle_yawed is not None
        and abs(muzzle_rest[0]) > 1.0
        and abs(muzzle_yawed[1]) > 1.0
        and abs(muzzle_yawed[0]) < 0.08,
        details=f"rest={muzzle_rest}, yawed={muzzle_yawed}",
    )

    with ctx.pose({slew: 0.0, elevation: elevation.motion_limits.upper, hatch_hinge: 0.0}):
        muzzle_elevated = _aabb_center(ctx.part_element_world_aabb(weapon_housing, elem="muzzle_tube"))

    ctx.check(
        "weapon elevation raises the muzzle",
        muzzle_rest is not None
        and muzzle_elevated is not None
        and muzzle_elevated[2] > muzzle_rest[2] + 0.40,
        details=f"rest={muzzle_rest}, elevated={muzzle_elevated}",
    )

    with ctx.pose({slew: 0.0, elevation: 0.0, hatch_hinge: hatch_hinge.motion_limits.upper}):
        hatch_open = _aabb_center(ctx.part_element_world_aabb(access_hatch, elem="hatch_panel"))

    with ctx.pose({slew: 0.0, elevation: 0.0, hatch_hinge: 0.0}):
        hatch_closed = _aabb_center(ctx.part_element_world_aabb(access_hatch, elem="hatch_panel"))

    ctx.check(
        "access hatch opens upward",
        hatch_closed is not None
        and hatch_open is not None
        and hatch_open[2] > hatch_closed[2] + 0.06,
        details=f"closed={hatch_closed}, open={hatch_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
