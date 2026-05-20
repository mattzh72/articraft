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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _pod_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    center_y: float,
    center_z: float = 0.0,
    segments: int = 32,
) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(width, height, exponent=3.2, segments=segments)
    return [(x_pos, center_y + y, center_z + z) for y, z in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vehicle_roof_launcher")

    roof_paint = model.material("roof_paint", rgba=(0.34, 0.36, 0.39, 1.0))
    launcher_paint = model.material("launcher_paint", rgba=(0.46, 0.50, 0.42, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    pod_paint = model.material("pod_paint", rgba=(0.55, 0.58, 0.50, 1.0))
    optic_glass = model.material("optic_glass", rgba=(0.16, 0.20, 0.22, 1.0))

    roof = model.part("roof_module")
    roof_length = 1.50
    roof_width = 1.10
    roof_thickness = 0.04
    opening_radius = 0.24
    ring_outer_radius = 0.31
    ring_height = 0.08

    roof_panel_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(roof_length, roof_width, 0.08),
        [_circle_profile(opening_radius, segments=56)],
        roof_thickness,
        center=True,
    ).translate(0.0, 0.0, roof_thickness * 0.5)
    roof.visual(
        mesh_from_geometry(roof_panel_geom, "roof_panel"),
        material=roof_paint,
        name="roof_panel",
    )

    turret_ring_geom = ExtrudeWithHolesGeometry(
        _circle_profile(ring_outer_radius, segments=64),
        [_circle_profile(opening_radius, segments=64)],
        ring_height,
        center=True,
    ).translate(0.0, 0.0, roof_thickness + (ring_height * 0.5))
    roof.visual(
        mesh_from_geometry(turret_ring_geom, "turret_ring"),
        material=dark_metal,
        name="turret_ring",
    )
    roof.inertial = Inertial.from_geometry(
        Box((roof_length, roof_width, roof_thickness + ring_height)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, (roof_thickness + ring_height) * 0.5)),
    )

    turret = model.part("turret_head")
    turret.visual(
        Cylinder(radius=0.28, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_metal,
        name="turntable",
    )
    turret.visual(
        Cylinder(radius=0.085, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=launcher_paint,
        name="center_pedestal",
    )
    turret.visual(
        Box((0.11, 0.15, 0.21)),
        origin=Origin(xyz=(0.035, -0.215, 0.125)),
        material=launcher_paint,
        name="side_stanchion",
    )
    turret.visual(
        Box((0.12, 0.022, 0.08)),
        origin=Origin(xyz=(0.13, -0.255, 0.20)),
        material=launcher_paint,
        name="yoke_arm",
    )
    turret.visual(
        Cylinder(radius=0.03, length=0.05),
        origin=Origin(xyz=(0.14, -0.245, 0.22), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="yoke_boss",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.56, 0.32, 0.26)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.04, 0.13)),
    )

    pod = model.part("launch_pod")
    pod_center_y = 0.10
    pod_sections = [
        _pod_section(-0.02, width=0.05, height=0.055, center_y=pod_center_y),
        _pod_section(0.08, width=0.11, height=0.10, center_y=pod_center_y),
        _pod_section(0.30, width=0.15, height=0.13, center_y=pod_center_y),
        _pod_section(0.78, width=0.14, height=0.12, center_y=pod_center_y),
        _pod_section(1.04, width=0.04, height=0.045, center_y=pod_center_y),
    ]
    pod_body_geom = section_loft(pod_sections)
    pod.visual(
        mesh_from_geometry(pod_body_geom, "launch_pod_body"),
        material=pod_paint,
        name="pod_body",
    )
    pod.visual(
        Cylinder(radius=0.03, length=0.08),
        origin=Origin(xyz=(0.0, 0.04, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pod_trunnion",
    )
    pod.visual(
        Box((0.09, 0.075, 0.075)),
        origin=Origin(xyz=(-0.005, pod_center_y, 0.0)),
        material=launcher_paint,
        name="rear_pack",
    )
    pod.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(1.035, pod_center_y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=optic_glass,
        name="nose_cap",
    )
    pod.inertial = Inertial.from_geometry(
        Box((1.10, 0.18, 0.15)),
        mass=22.0,
        origin=Origin(xyz=(0.52, pod_center_y, 0.0)),
    )

    model.articulation(
        "roof_to_turret_yaw",
        ArticulationType.CONTINUOUS,
        parent=roof,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, roof_thickness + ring_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6),
    )

    model.articulation(
        "turret_to_pod_pitch",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=pod,
        origin=Origin(xyz=(0.16, -0.22, 0.22)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.2,
            lower=math.radians(-18.0),
            upper=math.radians(58.0),
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

    roof = object_model.get_part("roof_module")
    turret = object_model.get_part("turret_head")
    pod = object_model.get_part("launch_pod")
    yaw = object_model.get_articulation("roof_to_turret_yaw")
    pitch = object_model.get_articulation("turret_to_pod_pitch")

    ctx.check(
        "parts and articulations exist",
        all(item is not None for item in (roof, turret, pod, yaw, pitch)),
        details="Expected roof, turret, pod, yaw articulation, and pitch articulation.",
    )

    with ctx.pose({yaw: 0.0, pitch: 0.0}):
        ctx.expect_contact(
            turret,
            roof,
            elem_a="turntable",
            elem_b="turret_ring",
            contact_tol=1e-5,
            name="turntable seats on turret ring",
        )
        ctx.expect_overlap(
            turret,
            roof,
            axes="xy",
            elem_a="turntable",
            elem_b="turret_ring",
            min_overlap=0.50,
            name="turntable stays centered over the roof ring",
        )
        ctx.expect_gap(
            pod,
            turret,
            axis="y",
            positive_elem="pod_trunnion",
            negative_elem="yoke_boss",
            max_gap=0.001,
            max_penetration=0.0,
            name="pod trunnion meets the side yoke boss",
        )
        ctx.expect_gap(
            pod,
            roof,
            axis="z",
            positive_elem="pod_body",
            negative_elem="turret_ring",
            min_gap=0.08,
            name="launch pod clears the roof ring vertically",
        )

        rest_pod_pos = ctx.part_world_position(pod)
        rest_nose_aabb = ctx.part_element_world_aabb(pod, elem="nose_cap")

    with ctx.pose({yaw: math.pi / 2.0, pitch: 0.0}):
        yawed_pod_pos = ctx.part_world_position(pod)

    ctx.check(
        "yaw rotates pod around roof center",
        rest_pod_pos is not None
        and yawed_pod_pos is not None
        and yawed_pod_pos[0] > rest_pod_pos[0] + 0.02
        and yawed_pod_pos[1] > 0.05,
        details=f"rest={rest_pod_pos}, yawed={yawed_pod_pos}",
    )

    with ctx.pose({yaw: 0.0, pitch: math.radians(45.0)}):
        pitched_nose_aabb = ctx.part_element_world_aabb(pod, elem="nose_cap")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_nose_z = _aabb_center_z(rest_nose_aabb)
    pitched_nose_z = _aabb_center_z(pitched_nose_aabb)
    ctx.check(
        "positive pitch raises launch pod nose",
        rest_nose_z is not None
        and pitched_nose_z is not None
        and pitched_nose_z > rest_nose_z + 0.18,
        details=f"rest_nose_z={rest_nose_z}, pitched_nose_z={pitched_nose_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
