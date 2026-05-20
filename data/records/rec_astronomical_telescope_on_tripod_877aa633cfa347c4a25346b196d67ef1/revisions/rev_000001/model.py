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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _open_tube_shell_geometry(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 56,
) -> MeshGeometry:
    geom = MeshGeometry()
    half_length = length * 0.5

    outer_back: list[int] = []
    outer_front: list[int] = []
    inner_back: list[int] = []
    inner_front: list[int] = []

    for index in range(segments):
        angle = (2.0 * math.pi * index) / segments
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        outer_back.append(geom.add_vertex(outer_radius * cos_a, outer_radius * sin_a, -half_length))
        outer_front.append(geom.add_vertex(outer_radius * cos_a, outer_radius * sin_a, half_length))
        inner_back.append(geom.add_vertex(inner_radius * cos_a, inner_radius * sin_a, -half_length))
        inner_front.append(geom.add_vertex(inner_radius * cos_a, inner_radius * sin_a, half_length))

    for index in range(segments):
        nxt = (index + 1) % segments

        geom.add_face(outer_back[index], outer_back[nxt], outer_front[nxt])
        geom.add_face(outer_back[index], outer_front[nxt], outer_front[index])

        geom.add_face(inner_back[index], inner_front[nxt], inner_back[nxt])
        geom.add_face(inner_back[index], inner_front[index], inner_front[nxt])

        geom.add_face(outer_back[index], inner_back[nxt], outer_back[nxt])
        geom.add_face(outer_back[index], inner_back[index], inner_back[nxt])

        geom.add_face(outer_front[index], outer_front[nxt], inner_front[nxt])
        geom.add_face(outer_front[index], inner_front[nxt], inner_front[index])

    return geom


def _tube_shell_mesh(
    name: str,
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 56,
):
    return mesh_from_geometry(
        _open_tube_shell_geometry(
            outer_radius,
            inner_radius,
            length,
            segments=segments,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="video_astronomy_telescope")

    tripod_dark = model.material("tripod_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    aluminum = model.material("aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    mount_dark = model.material("mount_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    tube_white = model.material("tube_white", rgba=(0.92, 0.93, 0.94, 1.0))
    optics_gray = model.material("optics_gray", rgba=(0.56, 0.59, 0.63, 1.0))
    camera_black = model.material("camera_black", rgba=(0.07, 0.08, 0.09, 1.0))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.135, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=tripod_dark,
        name="head_plate",
    )
    tripod.visual(
        Cylinder(radius=0.072, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=tripod_dark,
        name="azimuth_pier",
    )
    tripod.visual(
        Cylinder(radius=0.048, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=tripod_dark,
        name="center_hub",
    )

    leg_length = 0.95
    leg_radius = 0.019
    top_radius = 0.108
    foot_radius = 0.52
    top_z = -0.026
    foot_z = -0.94
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        top = (top_radius * math.cos(angle), top_radius * math.sin(angle), top_z)
        foot = (foot_radius * math.cos(angle), foot_radius * math.sin(angle), foot_z)
        ux = top[0] - foot[0]
        uy = top[1] - foot[1]
        uz = top[2] - foot[2]
        yaw = math.atan2(uy, ux)
        pitch = math.atan2(math.hypot(ux, uy), uz)

        tripod.visual(
            Cylinder(radius=leg_radius, length=leg_length),
            origin=Origin(
                xyz=(
                    0.5 * (top[0] + foot[0]),
                    0.5 * (top[1] + foot[1]),
                    0.5 * (top[2] + foot[2]),
                ),
                rpy=(0.0, pitch, yaw),
            ),
            material=aluminum,
            name=f"leg_{index}_tube",
        )
        tripod.visual(
            Box((0.065, 0.038, 0.03)),
            origin=Origin(
                xyz=(
                    (top_radius + 0.014) * math.cos(angle),
                    (top_radius + 0.014) * math.sin(angle),
                    -0.031,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=tripod_dark,
            name=f"leg_clamp_{index}",
        )

    mount_arm = model.part("mount_arm")
    mount_arm.visual(
        Cylinder(radius=0.105, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=mount_dark,
        name="azimuth_turntable",
    )
    mount_arm.visual(
        Box((0.16, 0.20, 0.16)),
        origin=Origin(xyz=(0.05, 0.0, 0.10)),
        material=mount_dark,
        name="drive_body",
    )
    mount_arm.visual(
        Box((0.08, 0.13, 0.36)),
        origin=Origin(xyz=(0.08, 0.0, 0.26)),
        material=mount_dark,
        name="single_arm",
    )
    mount_arm.visual(
        Cylinder(radius=0.036, length=0.24),
        origin=Origin(xyz=(0.04, 0.0, 0.18), rpy=(0.0, 0.32, 0.0)),
        material=mount_dark,
        name="arm_brace",
    )
    mount_arm.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.085, 0.0, 0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mount_dark,
        name="altitude_hub",
    )
    mount_arm.visual(
        Box((0.07, 0.24, 0.05)),
        origin=Origin(xyz=(0.035, 0.0, 0.43)),
        material=mount_dark,
        name="saddle_block",
    )

    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        _tube_shell_mesh("ota_shell", 0.115, 0.105, 0.68),
        origin=Origin(xyz=(-0.154, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=tube_white,
        name="ota_shell",
    )
    optical_tube.visual(
        Box((0.026, 0.40, 0.018)),
        origin=Origin(xyz=(-0.013, 0.0, 0.0)),
        material=mount_dark,
        name="ota_dovetail",
    )
    optical_tube.visual(
        _tube_shell_mesh("front_ring_band", 0.127, 0.118, 0.028),
        origin=Origin(xyz=(-0.154, -0.14, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_dark,
        name="front_ring_band",
    )
    optical_tube.visual(
        _tube_shell_mesh("rear_ring_band", 0.127, 0.118, 0.028),
        origin=Origin(xyz=(-0.154, 0.14, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_dark,
        name="rear_ring_band",
    )
    optical_tube.visual(
        Box((0.078, 0.08, 0.03)),
        origin=Origin(xyz=(-0.039, -0.14, 0.0)),
        material=mount_dark,
        name="front_ring_block",
    )
    optical_tube.visual(
        Box((0.078, 0.08, 0.03)),
        origin=Origin(xyz=(-0.039, 0.14, 0.0)),
        material=mount_dark,
        name="rear_ring_block",
    )
    optical_tube.visual(
        _tube_shell_mesh("primary_cell", 0.112, 0.100, 0.055),
        origin=Origin(xyz=(-0.154, 0.31, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=mount_dark,
        name="primary_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.102, length=0.018),
        origin=Origin(xyz=(-0.154, 0.302, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=optics_gray,
        name="primary_mirror",
    )
    optical_tube.visual(
        Box((0.216, 0.004, 0.006)),
        origin=Origin(xyz=(-0.154, -0.19, 0.0)),
        material=mount_dark,
        name="secondary_spider_x",
    )
    optical_tube.visual(
        Box((0.006, 0.004, 0.216)),
        origin=Origin(xyz=(-0.154, -0.19, 0.0)),
        material=mount_dark,
        name="secondary_spider_z",
    )
    optical_tube.visual(
        Cylinder(radius=0.015, length=0.04),
        origin=Origin(xyz=(-0.154, -0.19, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_dark,
        name="secondary_holder",
    )
    optical_tube.visual(
        Box((0.068, 0.052, 0.03)),
        origin=Origin(xyz=(-0.154, -0.09, 0.111)),
        material=mount_dark,
        name="focuser_base",
    )
    optical_tube.visual(
        Cylinder(radius=0.027, length=0.092),
        origin=Origin(xyz=(-0.154, -0.09, 0.172)),
        material=mount_dark,
        name="focuser_drawtube",
    )
    optical_tube.visual(
        Cylinder(radius=0.034, length=0.112),
        origin=Origin(xyz=(-0.154, -0.09, 0.252)),
        material=camera_black,
        name="camera_body",
    )
    optical_tube.visual(
        Box((0.078, 0.062, 0.032)),
        origin=Origin(xyz=(-0.154, -0.09, 0.318)),
        material=camera_black,
        name="camera_heat_sink",
    )

    model.articulation(
        "tripod_to_mount",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=mount_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2),
    )
    model.articulation(
        "mount_to_optical_tube",
        ArticulationType.REVOLUTE,
        parent=mount_arm,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.8,
            lower=-0.45,
            upper=1.40,
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

    tripod = object_model.get_part("tripod")
    mount_arm = object_model.get_part("mount_arm")
    optical_tube = object_model.get_part("optical_tube")
    azimuth = object_model.get_articulation("tripod_to_mount")
    altitude = object_model.get_articulation("mount_to_optical_tube")

    ctx.check(
        "azimuth is continuous around vertical axis",
        azimuth.articulation_type == ArticulationType.CONTINUOUS and azimuth.axis == (0.0, 0.0, 1.0),
        details=f"type={azimuth.articulation_type}, axis={azimuth.axis}",
    )
    ctx.check(
        "altitude raises around horizontal axis",
        altitude.articulation_type == ArticulationType.REVOLUTE and altitude.axis == (1.0, 0.0, 0.0),
        details=f"type={altitude.articulation_type}, axis={altitude.axis}",
    )

    with ctx.pose({altitude: 0.0}):
        ctx.expect_contact(
            optical_tube,
            mount_arm,
            elem_a="ota_dovetail",
            elem_b="saddle_block",
            contact_tol=0.001,
            name="dovetail rail seats in the saddle",
        )
        ctx.expect_gap(
            mount_arm,
            optical_tube,
            axis="x",
            positive_elem="saddle_block",
            negative_elem="ota_shell",
            min_gap=0.02,
            name="tube shell clears the single arm",
        )

        rest_aabb = ctx.part_element_world_aabb(optical_tube, elem="ota_shell")

    with ctx.pose({altitude: 1.0}):
        raised_aabb = ctx.part_element_world_aabb(optical_tube, elem="ota_shell")

    ctx.check(
        "positive altitude lifts the telescope toward zenith",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.12,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    ctx.expect_gap(
        mount_arm,
        tripod,
        axis="z",
        positive_elem="azimuth_turntable",
        negative_elem="head_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="azimuth bearing sits on the flat tripod head",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
