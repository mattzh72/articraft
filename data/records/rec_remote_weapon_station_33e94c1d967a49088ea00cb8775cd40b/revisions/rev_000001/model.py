from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)


def _base_ring_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.58, 0.00),
            (0.70, 0.02),
            (0.72, 0.06),
            (0.68, 0.09),
        ],
        [
            (0.46, 0.00),
            (0.56, 0.02),
            (0.58, 0.06),
            (0.54, 0.09),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _turret_section(
    x_pos: float,
    width: float,
    shoulder_z: float,
    roof_z: float,
) -> tuple[tuple[float, float, float], ...]:
    half_width = width * 0.5
    shoulder_width = width * 0.37
    roof_width = width * 0.18
    return (
        (x_pos, -half_width, 0.0),
        (x_pos, -shoulder_width, shoulder_z),
        (x_pos, -roof_width, roof_z),
        (x_pos, roof_width, roof_z),
        (x_pos, shoulder_width, shoulder_z),
        (x_pos, half_width, 0.0),
    )


def _turret_shell():
    return repair_loft(
        section_loft(
            [
                _turret_section(-0.52, 1.02, 0.10, 0.22),
                _turret_section(-0.14, 1.12, 0.13, 0.28),
                _turret_section(0.16, 0.96, 0.12, 0.24),
                _turret_section(0.32, 0.66, 0.09, 0.18),
            ]
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vehicle_weapon_station")

    ring_mat = model.material("ring_paint", rgba=(0.16, 0.17, 0.18, 1.0))
    armor_mat = model.material("armor_paint", rgba=(0.31, 0.34, 0.29, 1.0))
    steel_mat = model.material("gun_steel", rgba=(0.12, 0.12, 0.14, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        mesh_from_geometry(_base_ring_shell(), "base_ring_shell"),
        material=ring_mat,
        name="ring_shell",
    )

    turret = model.part("turret")
    turret.visual(
        mesh_from_geometry(_turret_shell(), "turret_shell"),
        material=armor_mat,
        name="turret_shell",
    )
    turret.visual(
        Box((0.34, 0.14, 0.24)),
        origin=Origin(xyz=(0.39, 0.28, 0.16)),
        material=armor_mat,
        name="left_cheek",
    )
    turret.visual(
        Box((0.34, 0.14, 0.24)),
        origin=Origin(xyz=(0.39, -0.28, 0.16)),
        material=armor_mat,
        name="right_cheek",
    )
    turret.visual(
        Box((0.20, 0.18, 0.08)),
        origin=Origin(xyz=(-0.10, 0.18, 0.27), rpy=(0.0, -0.18, 0.0)),
        material=armor_mat,
        name="sensor_blister",
    )

    gun_cradle = model.part("gun_cradle")
    gun_cradle.visual(
        Box((0.50, 0.24, 0.22)),
        origin=Origin(xyz=(0.24, 0.0, 0.03)),
        material=armor_mat,
        name="receiver",
    )
    gun_cradle.visual(
        Box((0.42, 0.24, 0.28)),
        origin=Origin(xyz=(0.22, -0.22, 0.10)),
        material=armor_mat,
        name="feed_housing",
    )
    gun_cradle.visual(
        Box((0.34, 0.22, 0.06)),
        origin=Origin(xyz=(0.24, 0.0, 0.18), rpy=(0.0, -0.20, 0.0)),
        material=armor_mat,
        name="receiver_top_cover",
    )
    gun_cradle.visual(
        Box((0.28, 0.22, 0.05)),
        origin=Origin(xyz=(0.22, -0.22, 0.24), rpy=(0.0, -0.24, 0.0)),
        material=armor_mat,
        name="feed_housing_cap",
    )
    gun_cradle.visual(
        Cylinder(radius=0.055, length=1.18),
        origin=Origin(xyz=(0.82, 0.0, 0.03), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mat,
        name="barrel",
    )
    gun_cradle.visual(
        Cylinder(radius=0.075, length=0.12),
        origin=Origin(xyz=(1.47, 0.0, 0.03), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel_mat,
        name="muzzle_brake",
    )
    gun_cradle.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, 0.16, 0.03), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="left_trunnion",
    )
    gun_cradle.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(xyz=(0.0, -0.16, 0.03), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="right_trunnion",
    )

    reload_door = model.part("reload_door")
    reload_door.visual(
        Box((0.34, 0.018, 0.24)),
        origin=Origin(xyz=(0.17, -0.009, 0.0)),
        material=armor_mat,
        name="door_panel",
    )
    reload_door.visual(
        Cylinder(radius=0.012, length=0.06),
        origin=Origin(xyz=(0.0, -0.015, 0.07)),
        material=steel_mat,
        name="upper_hinge_knuckle",
    )
    reload_door.visual(
        Cylinder(radius=0.012, length=0.06),
        origin=Origin(xyz=(0.0, -0.015, -0.07)),
        material=steel_mat,
        name="lower_hinge_knuckle",
    )
    reload_door.visual(
        Box((0.08, 0.01, 0.05)),
        origin=Origin(xyz=(0.23, -0.022, 0.0)),
        material=steel_mat,
        name="door_handle",
    )

    turret_yaw = model.articulation(
        "base_to_turret",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6),
    )
    turret_yaw.meta["qc_samples"] = [0.0, 0.6, -0.9]

    cradle_pitch = model.articulation(
        "turret_to_gun_cradle",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=gun_cradle,
        origin=Origin(xyz=(0.605, 0.0, 0.18)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.0,
            lower=-0.30,
            upper=1.05,
        ),
    )

    reload_hinge = model.articulation(
        "cradle_to_reload_door",
        ArticulationType.REVOLUTE,
        parent=gun_cradle,
        child=reload_door,
        origin=Origin(xyz=(0.02, -0.34, 0.10)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
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

    base_ring = object_model.get_part("base_ring")
    turret = object_model.get_part("turret")
    gun_cradle = object_model.get_part("gun_cradle")
    reload_door = object_model.get_part("reload_door")

    turret_yaw = object_model.get_articulation("base_to_turret")
    cradle_pitch = object_model.get_articulation("turret_to_gun_cradle")
    reload_hinge = object_model.get_articulation("cradle_to_reload_door")

    with ctx.pose({reload_hinge: 0.0}):
        ctx.expect_gap(
            gun_cradle,
            reload_door,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="feed_housing",
            negative_elem="door_panel",
            name="reload door closes flush to the feed housing side",
        )
        ctx.expect_overlap(
            reload_door,
            gun_cradle,
            axes="xz",
            min_overlap=0.18,
            elem_a="door_panel",
            elem_b="feed_housing",
            name="closed reload door covers the feed opening",
        )

    rest_cradle_pos = ctx.part_world_position(gun_cradle)
    with ctx.pose({turret_yaw: 0.6}):
        yawed_cradle_pos = ctx.part_world_position(gun_cradle)
    ctx.check(
        "turret yaw rotates the gun cradle around the base",
        rest_cradle_pos is not None
        and yawed_cradle_pos is not None
        and yawed_cradle_pos[1] > rest_cradle_pos[1] + 0.10,
        details=f"rest={rest_cradle_pos}, yawed={yawed_cradle_pos}",
    )

    rest_barrel_aabb = ctx.part_element_world_aabb(gun_cradle, elem="barrel")
    with ctx.pose({cradle_pitch: 0.7}):
        elevated_barrel_aabb = ctx.part_element_world_aabb(gun_cradle, elem="barrel")
    ctx.check(
        "gun cradle positive pitch elevates the barrel",
        rest_barrel_aabb is not None
        and elevated_barrel_aabb is not None
        and elevated_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.20,
        details=f"rest={rest_barrel_aabb}, elevated={elevated_barrel_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(reload_door, elem="door_panel")
    with ctx.pose({reload_hinge: 1.0}):
        open_door_aabb = ctx.part_element_world_aabb(reload_door, elem="door_panel")
    ctx.check(
        "reload door opens outward from the feed housing",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    ctx.expect_contact(
        turret,
        base_ring,
        elem_a="turret_shell",
        elem_b="ring_shell",
        contact_tol=0.001,
        name="turret body seats on the rotating base ring",
    )
    ctx.expect_contact(
        gun_cradle,
        turret,
        elem_a="left_trunnion",
        elem_b="left_cheek",
        contact_tol=0.03,
        name="left trunnion stays mounted between the turret cheeks",
    )
    ctx.expect_contact(
        gun_cradle,
        turret,
        elem_a="right_trunnion",
        elem_b="right_cheek",
        contact_tol=0.03,
        name="right trunnion stays mounted between the turret cheeks",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
