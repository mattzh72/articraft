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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_remote_weapon_station", assets=ASSETS)

    armor = model.material("armor", rgba=(0.43, 0.47, 0.35, 1.0))
    dark_armor = model.material("dark_armor", rgba=(0.28, 0.31, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.46, 0.48, 0.50, 1.0))
    black = model.material("black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.22, 0.26, 0.9))

    pedestal = model.part("pedestal")
    pedestal.inertial = Inertial.from_geometry(
        Box((0.92, 1.04, 1.08)),
        mass=720.0,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
    )
    pedestal.visual(
        Box((0.92, 1.04, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_armor,
        name="base_plate",
    )
    pedestal.visual(
        Box((0.58, 0.70, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=armor,
        name="mount_frame",
    )

    pedestal_shell = section_loft(
        [
            _xy_section(0.44, 0.54, 0.06, 0.00),
            _xy_section(0.40, 0.49, 0.06, 0.26),
            _xy_section(0.36, 0.44, 0.05, 0.58),
            _xy_section(0.31, 0.37, 0.05, 0.78),
        ]
    )
    pedestal.visual(
        _save_mesh("rws_pedestal_shell.obj", pedestal_shell),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=armor,
        name="column_shell",
    )
    pedestal.visual(
        Box((0.16, 0.30, 0.34)),
        origin=Origin(xyz=(-0.10, 0.0, 0.32)),
        material=dark_armor,
        name="service_box",
    )
    pedestal.visual(
        Cylinder(radius=0.16, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        material=steel,
        name="bearing_support",
    )
    pedestal.visual(
        Cylinder(radius=0.29, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material=steel,
        name="lower_ring",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.72, 0.54, 0.76)),
        mass=360.0,
        origin=Origin(xyz=(0.02, 0.0, 0.36)),
    )
    yaw_stage.visual(
        Cylinder(radius=0.30, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=steel,
        name="upper_ring",
    )
    yaw_stage.visual(
        Cylinder(radius=0.20, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=dark_armor,
        name="slew_drum",
    )
    yaw_stage.visual(
        Box((0.38, 0.34, 0.09)),
        origin=Origin(xyz=(-0.02, 0.0, 0.325)),
        material=armor,
        name="deck_box",
    )
    yaw_stage.visual(
        Box((0.20, 0.30, 0.14)),
        origin=Origin(xyz=(-0.11, 0.0, 0.26)),
        material=dark_armor,
        name="drive_pack",
    )

    cheek_profile = [
        (-0.12, 0.18),
        (0.00, 0.18),
        (0.14, 0.22),
        (0.28, 0.34),
        (0.29, 0.54),
        (0.17, 0.63),
        (-0.02, 0.61),
        (-0.10, 0.42),
    ]
    cheek_mesh = _save_mesh(
        "rws_side_cheek.obj",
        ExtrudeGeometry(cheek_profile, 0.03, center=True).rotate_x(math.pi / 2.0),
    )
    yaw_stage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, 0.165, 0.0)),
        material=armor,
        name="left_cheek",
    )
    yaw_stage.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -0.165, 0.0)),
        material=armor,
        name="right_cheek",
    )
    yaw_stage.visual(
        Cylinder(radius=0.058, length=0.04),
        origin=Origin(xyz=(0.10, 0.165, 0.44), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion_boss",
    )
    yaw_stage.visual(
        Cylinder(radius=0.058, length=0.04),
        origin=Origin(xyz=(0.10, -0.165, 0.44), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion_boss",
    )
    yaw_stage.visual(
        Box((0.12, 0.34, 0.08)),
        origin=Origin(xyz=(-0.06, 0.0, 0.52)),
        material=dark_armor,
        name="top_crossmember",
    )
    yaw_stage.visual(
        Box((0.24, 0.015, 0.26)),
        origin=Origin(
            xyz=(0.12, 0.1875, 0.34),
            rpy=(0.0, math.radians(-18.0), 0.0),
        ),
        material=armor,
        name="left_shield",
    )
    yaw_stage.visual(
        Box((0.24, 0.015, 0.26)),
        origin=Origin(
            xyz=(0.12, -0.1875, 0.34),
            rpy=(0.0, math.radians(-18.0), 0.0),
        ),
        material=armor,
        name="right_shield",
    )
    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.inertial = Inertial.from_geometry(
        Box((1.56, 0.32, 0.38)),
        mass=290.0,
        origin=Origin(xyz=(0.68, 0.0, 0.10)),
    )
    weapon_cradle.visual(
        Cylinder(radius=0.035, length=0.21),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_shaft",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(xyz=(0.0, 0.125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion_pin",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(xyz=(0.0, -0.125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion_pin",
    )
    weapon_cradle.visual(
        Box((0.18, 0.21, 0.12)),
        origin=Origin(xyz=(0.04, 0.0, 0.04)),
        material=dark_armor,
        name="cradle_block",
    )
    weapon_cradle.visual(
        Box((0.30, 0.04, 0.10)),
        origin=Origin(xyz=(0.19, 0.083, 0.06)),
        material=armor,
        name="left_cradle_arm",
    )
    weapon_cradle.visual(
        Box((0.30, 0.04, 0.10)),
        origin=Origin(xyz=(0.19, -0.083, 0.06)),
        material=armor,
        name="right_cradle_arm",
    )
    weapon_cradle.visual(
        Box((0.38, 0.16, 0.18)),
        origin=Origin(xyz=(0.38, 0.0, 0.07)),
        material=dark_armor,
        name="receiver_box",
    )
    weapon_cradle.visual(
        Box((0.24, 0.14, 0.06)),
        origin=Origin(xyz=(0.34, 0.0, 0.19)),
        material=armor,
        name="feed_cover",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.042, length=0.34),
        origin=Origin(xyz=(0.74, 0.0, 0.07), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_armor,
        name="barrel_shroud",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.017, length=0.86),
        origin=Origin(xyz=(1.14, 0.0, 0.07), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="barrel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.026, length=0.08),
        origin=Origin(xyz=(1.61, 0.0, 0.07), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="muzzle_brake",
    )
    weapon_cradle.visual(
        Box((0.24, 0.12, 0.18)),
        origin=Origin(xyz=(0.30, -0.02, 0.22)),
        material=armor,
        name="ammo_box",
    )
    weapon_cradle.visual(
        Box((0.16, 0.10, 0.14)),
        origin=Origin(xyz=(0.26, 0.05, 0.25)),
        material=black,
        name="sensor_box",
    )
    weapon_cradle.visual(
        Box((0.08, 0.03, 0.07)),
        origin=Origin(xyz=(0.33, 0.095, 0.25)),
        material=glass,
        name="eo_window",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.032, length=0.04),
        origin=Origin(xyz=(0.31, 0.092, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="thermal_window",
    )

    model.articulation(
        "azimuth_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=weapon_cradle,
        origin=Origin(xyz=(0.10, 0.0, 0.44)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.1,
            lower=math.radians(-10.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    yaw_stage = object_model.get_part("yaw_stage")
    weapon_cradle = object_model.get_part("weapon_cradle")
    azimuth_axis = object_model.get_articulation("azimuth_axis")
    elevation_axis = object_model.get_articulation("elevation_axis")

    lower_ring = pedestal.get_visual("lower_ring")
    upper_ring = yaw_stage.get_visual("upper_ring")
    deck_box = yaw_stage.get_visual("deck_box")
    left_cheek = yaw_stage.get_visual("left_cheek")
    right_cheek = yaw_stage.get_visual("right_cheek")
    left_trunnion_pin = weapon_cradle.get_visual("left_trunnion_pin")
    right_trunnion_pin = weapon_cradle.get_visual("right_trunnion_pin")
    cradle_block = weapon_cradle.get_visual("cradle_block")
    barrel = weapon_cradle.get_visual("barrel")
    sensor_box = weapon_cradle.get_visual("sensor_box")
    ammo_box = weapon_cradle.get_visual("ammo_box")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=16,
        name="articulated pairs stay clear through sampled motion",
    )

    ctx.expect_contact(
        yaw_stage,
        pedestal,
        elem_a=upper_ring,
        elem_b=lower_ring,
        name="upper slewing ring contacts lower ring",
    )
    ctx.expect_gap(
        yaw_stage,
        pedestal,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_ring,
        negative_elem=lower_ring,
        name="slewing ring seats without penetration",
    )
    ctx.expect_overlap(
        yaw_stage,
        pedestal,
        axes="xy",
        min_overlap=0.40,
        elem_a=upper_ring,
        elem_b=lower_ring,
        name="slewing rings share the same footprint",
    )
    ctx.expect_contact(
        weapon_cradle,
        yaw_stage,
        elem_a=left_trunnion_pin,
        elem_b=yaw_stage.get_visual("left_trunnion_boss"),
        name="left trunnion bears on left cheek",
    )
    ctx.expect_contact(
        weapon_cradle,
        yaw_stage,
        elem_a=right_trunnion_pin,
        elem_b=yaw_stage.get_visual("right_trunnion_boss"),
        name="right trunnion bears on right cheek",
    )
    ctx.expect_gap(
        weapon_cradle,
        yaw_stage,
        axis="z",
        min_gap=0.015,
        positive_elem=cradle_block,
        negative_elem=deck_box,
        name="cradle block clears the rotating deck",
    )

    left_cheek_aabb = ctx.part_element_world_aabb(yaw_stage, elem=left_cheek)
    right_cheek_aabb = ctx.part_element_world_aabb(yaw_stage, elem=right_cheek)
    sensor_aabb = ctx.part_element_world_aabb(weapon_cradle, elem=sensor_box)
    ammo_aabb = ctx.part_element_world_aabb(weapon_cradle, elem=ammo_box)
    assert left_cheek_aabb is not None
    assert right_cheek_aabb is not None
    assert sensor_aabb is not None
    assert ammo_aabb is not None
    inner_left_y = left_cheek_aabb[0][1]
    inner_right_y = right_cheek_aabb[1][1]
    ctx.check(
        "sensor package stays between side cheeks",
        sensor_aabb[0][1] >= inner_right_y - 0.002 and sensor_aabb[1][1] <= inner_left_y + 0.002,
        details=f"sensor y-span={sensor_aabb[0][1]:.3f}..{sensor_aabb[1][1]:.3f}, cheek gap={inner_right_y:.3f}..{inner_left_y:.3f}",
    )
    ctx.check(
        "ammo box stays between side cheeks",
        ammo_aabb[0][1] >= inner_right_y - 0.002 and ammo_aabb[1][1] <= inner_left_y + 0.002,
        details=f"ammo y-span={ammo_aabb[0][1]:.3f}..{ammo_aabb[1][1]:.3f}, cheek gap={inner_right_y:.3f}..{inner_left_y:.3f}",
    )

    barrel_rest = ctx.part_element_world_aabb(weapon_cradle, elem=barrel)
    trunnion_rest = ctx.part_world_position(weapon_cradle)
    assert barrel_rest is not None
    assert trunnion_rest is not None
    barrel_rest_center = _aabb_center(barrel_rest)

    with ctx.pose({elevation_axis: math.radians(60.0)}):
        barrel_high = ctx.part_element_world_aabb(weapon_cradle, elem=barrel)
        assert barrel_high is not None
        barrel_high_center = _aabb_center(barrel_high)
        ctx.check(
            "elevation raises the barrel strongly",
            barrel_high_center[2] > barrel_rest_center[2] + 0.55,
            details=f"rest z={barrel_rest_center[2]:.3f}, raised z={barrel_high_center[2]:.3f}",
        )
        ctx.check(
            "elevation pulls barrel rearward in x",
            barrel_high_center[0] < barrel_rest_center[0] - 0.18,
            details=f"rest x={barrel_rest_center[0]:.3f}, raised x={barrel_high_center[0]:.3f}",
        )
        ctx.expect_gap(
            weapon_cradle,
            pedestal,
            axis="z",
            min_gap=0.25,
            name="raised weapon stays clear above pedestal",
        )

    with ctx.pose({elevation_axis: math.radians(-10.0)}):
        barrel_low = ctx.part_element_world_aabb(weapon_cradle, elem=barrel)
        assert barrel_low is not None
        barrel_low_center = _aabb_center(barrel_low)
        ctx.check(
            "depression lowers the barrel",
            barrel_low_center[2] < barrel_rest_center[2] - 0.10,
            details=f"rest z={barrel_rest_center[2]:.3f}, depressed z={barrel_low_center[2]:.3f}",
        )
        ctx.expect_gap(
            weapon_cradle,
            pedestal,
            axis="z",
            min_gap=0.16,
            name="depressed weapon still clears pedestal",
        )

    yaw_rest = ctx.part_world_position(weapon_cradle)
    assert yaw_rest is not None
    with ctx.pose({azimuth_axis: math.pi / 2.0}):
        yaw_quarter = ctx.part_world_position(weapon_cradle)
        assert yaw_quarter is not None
        ctx.check(
            "azimuth rotates the upper assembly about vertical axis",
            abs(yaw_quarter[0]) < 0.02 and abs(yaw_quarter[1] - yaw_rest[0]) < 0.02,
            details=f"rest={yaw_rest}, quarter_turn={yaw_quarter}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
