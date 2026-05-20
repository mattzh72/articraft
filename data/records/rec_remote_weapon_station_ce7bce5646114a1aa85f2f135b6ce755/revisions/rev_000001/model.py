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
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _turret_section(
    x: float,
    *,
    bottom_width: float,
    shoulder_width: float,
    roof_width: float,
    z_bottom: float,
    z_shoulder: float,
    z_roof: float,
) -> tuple[tuple[float, float, float], ...]:
    return (
        (x, -bottom_width * 0.5, z_bottom),
        (x, -shoulder_width * 0.5, z_shoulder),
        (x, -roof_width * 0.5, z_roof),
        (x, roof_width * 0.5, z_roof),
        (x, shoulder_width * 0.5, z_shoulder),
        (x, bottom_width * 0.5, z_bottom),
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((low + high) * 0.5 for low, high in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_naval_remote_station")

    naval_grey = model.material("naval_grey", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.24, 0.26, 0.28, 1.0))
    black = model.material("black", rgba=(0.08, 0.09, 0.10, 1.0))
    sight_glass = model.material("sight_glass", rgba=(0.22, 0.34, 0.39, 0.45))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.24, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_grey,
        name="deck_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.145, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=naval_grey,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=dark_grey,
        name="yaw_race",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.48, 0.48, 0.32)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    turret_body = model.part("turret_body")
    body_sections = [
        _turret_section(
            -0.12,
            bottom_width=0.34,
            shoulder_width=0.30,
            roof_width=0.20,
            z_bottom=0.00,
            z_shoulder=0.18,
            z_roof=0.29,
        ),
        _turret_section(
            0.02,
            bottom_width=0.40,
            shoulder_width=0.35,
            roof_width=0.24,
            z_bottom=0.00,
            z_shoulder=0.20,
            z_roof=0.32,
        ),
        _turret_section(
            0.16,
            bottom_width=0.34,
            shoulder_width=0.28,
            roof_width=0.18,
            z_bottom=0.03,
            z_shoulder=0.20,
            z_roof=0.26,
        ),
        _turret_section(
            0.22,
            bottom_width=0.22,
            shoulder_width=0.18,
            roof_width=0.10,
            z_bottom=0.06,
            z_shoulder=0.17,
            z_roof=0.19,
        ),
    ]
    turret_body.visual(
        mesh_from_geometry(section_loft(body_sections), "turret_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=naval_grey,
        name="body_shell",
    )
    turret_body.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_grey,
        name="turret_ring",
    )
    turret_body.visual(
        Box((0.09, 0.27, 0.06)),
        origin=Origin(xyz=(0.165, 0.0, 0.08)),
        material=naval_grey,
        name="gun_support_bridge",
    )
    turret_body.visual(
        Box((0.09, 0.045, 0.18)),
        origin=Origin(xyz=(0.195, 0.1525, 0.15)),
        material=naval_grey,
        name="right_trunnion_cheek",
    )
    turret_body.visual(
        Box((0.09, 0.045, 0.18)),
        origin=Origin(xyz=(0.195, -0.1525, 0.15)),
        material=naval_grey,
        name="left_trunnion_cheek",
    )
    turret_body.visual(
        Box((0.192, 0.012, 0.18)),
        origin=Origin(xyz=(0.086, 0.184, 0.16)),
        material=dark_grey,
        name="sensor_bay",
    )
    turret_body.visual(
        Box((0.014, 0.012, 0.20)),
        origin=Origin(xyz=(-0.013, 0.2002, 0.16)),
        material=dark_grey,
        name="sensor_hinge_bracket",
    )
    turret_body.visual(
        Box((0.010, 0.012, 0.20)),
        origin=Origin(xyz=(-0.011, 0.1882, 0.16)),
        material=dark_grey,
        name="sensor_hinge_web",
    )
    turret_body.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=Origin(xyz=(0.09, 0.195, 0.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=sight_glass,
        name="sensor_lens",
    )
    turret_body.visual(
        Box((0.10, 0.08, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, 0.33)),
        material=dark_grey,
        name="roof_sight_housing",
    )
    turret_body.inertial = Inertial.from_geometry(
        Box((0.46, 0.40, 0.36)),
        mass=115.0,
        origin=Origin(xyz=(0.08, 0.0, 0.18)),
    )

    gun_mount = model.part("gun_mount")
    gun_mount.visual(
        Cylinder(radius=0.035, length=0.26),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="trunnion_sleeve",
    )
    gun_mount.visual(
        Box((0.24, 0.14, 0.11)),
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        material=dark_grey,
        name="receiver",
    )
    gun_mount.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=(0.17, 0.0, 0.085)),
        material=naval_grey,
        name="recoil_guard",
    )
    gun_mount.visual(
        Box((0.11, 0.06, 0.15)),
        origin=Origin(xyz=(0.13, -0.10, 0.01)),
        material=naval_grey,
        name="ammo_box",
    )
    gun_mount.visual(
        Cylinder(radius=0.028, length=0.62),
        origin=Origin(xyz=(0.60, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="barrel",
    )
    gun_mount.visual(
        Cylinder(radius=0.036, length=0.08),
        origin=Origin(xyz=(0.95, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="muzzle_brake",
    )
    gun_mount.inertial = Inertial.from_geometry(
        Box((1.04, 0.28, 0.18)),
        mass=62.0,
        origin=Origin(xyz=(0.52, 0.0, 0.02)),
    )

    sensor_cover = model.part("sensor_cover")
    sensor_cover.visual(
        Box((0.19, 0.032, 0.19)),
        origin=Origin(xyz=(0.095, 0.016, 0.0)),
        material=naval_grey,
        name="cover_shell",
    )
    sensor_cover.visual(
        Cylinder(radius=0.006, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_grey,
        name="cover_hinge_barrel",
    )
    sensor_cover.visual(
        Box((0.025, 0.038, 0.20)),
        origin=Origin(xyz=(0.1775, 0.019, 0.0)),
        material=naval_grey,
        name="cover_nose",
    )
    sensor_cover.visual(
        Box((0.05, 0.006, 0.03)),
        origin=Origin(xyz=(0.11, 0.035, 0.0)),
        material=dark_grey,
        name="cover_handle",
    )
    sensor_cover.inertial = Inertial.from_geometry(
        Box((0.20, 0.04, 0.20)),
        mass=9.0,
        origin=Origin(xyz=(0.10, 0.02, 0.0)),
    )

    model.articulation(
        "pedestal_to_turret_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=turret_body,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )
    model.articulation(
        "turret_to_gun_pitch",
        ArticulationType.REVOLUTE,
        parent=turret_body,
        child=gun_mount,
        origin=Origin(xyz=(0.24, 0.0, 0.15)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=math.radians(-20.0),
            upper=math.radians(65.0),
        ),
    )
    model.articulation(
        "turret_to_sensor_cover",
        ArticulationType.REVOLUTE,
        parent=turret_body,
        child=sensor_cover,
        origin=Origin(xyz=(0.0, 0.2002, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(115.0),
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
    turret_body = object_model.get_part("turret_body")
    gun_mount = object_model.get_part("gun_mount")
    sensor_cover = object_model.get_part("sensor_cover")

    yaw_joint = object_model.get_articulation("pedestal_to_turret_yaw")
    pitch_joint = object_model.get_articulation("turret_to_gun_pitch")
    cover_joint = object_model.get_articulation("turret_to_sensor_cover")

    ctx.expect_contact(
        turret_body,
        pedestal,
        elem_a="turret_ring",
        elem_b="yaw_race",
        contact_tol=0.0005,
        name="turret ring sits on pedestal yaw race",
    )
    ctx.expect_overlap(
        turret_body,
        pedestal,
        axes="xy",
        elem_a="turret_ring",
        elem_b="yaw_race",
        min_overlap=0.30,
        name="turret ring stays centered over the pedestal",
    )
    ctx.expect_contact(
        gun_mount,
        turret_body,
        elem_a="trunnion_sleeve",
        elem_b="right_trunnion_cheek",
        contact_tol=0.001,
        name="gun trunnion is carried by the side cheek",
    )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_gap(
            sensor_cover,
            turret_body,
            axis="y",
            positive_elem="cover_shell",
            negative_elem="body_shell",
            min_gap=0.0,
            max_gap=0.004,
            name="sensor cover closes just proud of the body shell",
        )
        ctx.expect_overlap(
            sensor_cover,
            turret_body,
            axes="xz",
            elem_a="cover_shell",
            elem_b="body_shell",
            min_overlap=0.14,
            name="sensor cover spans the sensor opening when shut",
        )

    barrel_rest = ctx.part_element_world_aabb(gun_mount, elem="barrel")
    with ctx.pose({yaw_joint: 0.7}):
        barrel_yawed = ctx.part_element_world_aabb(gun_mount, elem="barrel")
    with ctx.pose({pitch_joint: math.radians(35.0)}):
        barrel_pitched = ctx.part_element_world_aabb(gun_mount, elem="barrel")

    cover_closed = ctx.part_element_world_aabb(sensor_cover, elem="cover_shell")
    with ctx.pose({cover_joint: math.radians(70.0)}):
        cover_open = ctx.part_element_world_aabb(sensor_cover, elem="cover_shell")

    barrel_rest_center = _aabb_center(barrel_rest)
    barrel_yawed_center = _aabb_center(barrel_yawed)
    barrel_pitched_center = _aabb_center(barrel_pitched)
    cover_closed_center = _aabb_center(cover_closed)
    cover_open_center = _aabb_center(cover_open)

    ctx.check(
        "positive yaw swings the gun around the vertical axis",
        barrel_rest_center is not None
        and barrel_yawed_center is not None
        and barrel_yawed_center[1] > barrel_rest_center[1] + 0.22
        and barrel_yawed_center[0] < barrel_rest_center[0] - 0.10,
        details=f"rest={barrel_rest_center}, yawed={barrel_yawed_center}",
    )
    ctx.check(
        "positive pitch raises the barrel",
        barrel_rest_center is not None
        and barrel_pitched_center is not None
        and barrel_pitched_center[2] > barrel_rest_center[2] + 0.18,
        details=f"rest={barrel_rest_center}, pitched={barrel_pitched_center}",
    )
    ctx.check(
        "sensor cover opens outward from the side bay",
        cover_closed_center is not None
        and cover_open_center is not None
        and cover_open_center[1] > cover_closed_center[1] + 0.06,
        details=f"closed={cover_closed_center}, open={cover_open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
