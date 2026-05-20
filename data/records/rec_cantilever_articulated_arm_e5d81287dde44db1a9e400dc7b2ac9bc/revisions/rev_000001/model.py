from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 36,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _capsule_profile(
    length_between_centers: float,
    radius: float,
    *,
    segments_per_end: int = 24,
) -> list[tuple[float, float]]:
    """Capsule in local XY with pivot centers at x=0 and x=length."""

    pts: list[tuple[float, float]] = []
    for i in range(segments_per_end + 1):
        angle = -math.pi / 2.0 + math.pi * i / segments_per_end
        pts.append(
            (
                length_between_centers + radius * math.cos(angle),
                radius * math.sin(angle),
            )
        )
    for i in range(segments_per_end + 1):
        angle = math.pi / 2.0 + math.pi * i / segments_per_end
        pts.append((radius * math.cos(angle), radius * math.sin(angle)))
    return pts


def _holed_capsule_mesh(
    name: str,
    *,
    length_between_centers: float,
    outer_radius: float,
    bore_radius: float,
    thickness: float,
):
    outer = _capsule_profile(length_between_centers, outer_radius)
    holes = [
        _circle_profile(bore_radius, center=(0.0, 0.0), segments=36),
        _circle_profile(bore_radius, center=(length_between_centers, 0.0), segments=36),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, thickness, center=True),
        name,
    )


def _holed_disc_mesh(name: str, *, outer_radius: float, bore_radius: float, thickness: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=48),
            [_circle_profile(bore_radius, segments=36)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_cantilever_arm")

    powder_gray = model.material("powder_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.015, 0.016, 0.018, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.95, 0.42, 0.10, 1.0))
    blue_link = model.material("blue_link", rgba=(0.10, 0.26, 0.50, 1.0))
    face_aluminum = model.material("face_aluminum", rgba=(0.72, 0.74, 0.70, 1.0))

    shoulder_x = 0.145
    shoulder_z = 0.350
    upper_len = 0.430
    forearm_len = 0.335

    support = model.part("support")
    support.visual(
        Box((0.035, 0.280, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        material=powder_gray,
        name="backplate",
    )
    support.visual(
        Box((0.085, 0.104, 0.130)),
        origin=Origin(xyz=(0.055, 0.0, shoulder_z)),
        material=powder_gray,
        name="side_standoff",
    )
    support.visual(
        Box((0.180, 0.150, 0.018)),
        origin=Origin(xyz=(0.098, 0.0, shoulder_z - 0.045)),
        material=powder_gray,
        name="lower_bearing_shelf",
    )
    support.visual(
        Box((0.180, 0.150, 0.018)),
        origin=Origin(xyz=(0.098, 0.0, shoulder_z + 0.045)),
        material=powder_gray,
        name="upper_bearing_shelf",
    )
    support.visual(
        Box((0.055, 0.150, 0.108)),
        origin=Origin(xyz=(0.032, 0.0, shoulder_z)),
        material=powder_gray,
        name="shelf_bridge",
    )
    support.visual(
        Cylinder(radius=0.013, length=0.112),
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z)),
        material=dark_metal,
        name="shoulder_pin",
    )
    support.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z - 0.028)),
        material=dark_metal,
        name="lower_shoulder_boss",
    )
    support.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z + 0.028)),
        material=dark_metal,
        name="upper_shoulder_boss",
    )
    for y in (-0.095, 0.095):
        for z in (shoulder_z - 0.165, shoulder_z + 0.165):
            support.visual(
                Cylinder(radius=0.014, length=0.008),
                origin=Origin(xyz=(0.019, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_metal,
                name=f"wall_bolt_{len(support.visuals)}",
            )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        _holed_capsule_mesh(
            "upper_arm_shell",
            length_between_centers=upper_len,
            outer_radius=0.045,
            bore_radius=0.018,
            thickness=0.030,
        ),
        material=safety_orange,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Box((upper_len - 0.140, 0.020, 0.008)),
        origin=Origin(xyz=(upper_len / 2.0, 0.0, 0.019)),
        material=dark_metal,
        name="upper_arm_rib",
    )
    upper_arm.visual(
        Cylinder(radius=0.0115, length=0.096),
        origin=Origin(xyz=(upper_len, 0.0, 0.024)),
        material=dark_metal,
        name="elbow_pin",
    )
    upper_arm.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(upper_len, 0.0, 0.019)),
        material=dark_metal,
        name="elbow_collar",
    )

    forearm = model.part("forearm")
    forearm.visual(
        _holed_capsule_mesh(
            "forearm_shell",
            length_between_centers=forearm_len,
            outer_radius=0.038,
            bore_radius=0.015,
            thickness=0.030,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=blue_link,
        name="forearm_shell",
    )
    forearm.visual(
        Box((forearm_len - 0.120, 0.017, 0.007)),
        origin=Origin(xyz=(forearm_len / 2.0, 0.0, 0.062)),
        material=dark_metal,
        name="forearm_rib",
    )
    forearm.visual(
        Cylinder(radius=0.010, length=0.092),
        origin=Origin(xyz=(forearm_len, 0.0, 0.068)),
        material=dark_metal,
        name="wrist_pin",
    )
    forearm.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(forearm_len, 0.0, 0.066)),
        material=dark_metal,
        name="wrist_collar",
    )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(
        _holed_disc_mesh(
            "wrist_turntable",
            outer_radius=0.045,
            bore_radius=0.014,
            thickness=0.026,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=face_aluminum,
        name="wrist_turntable",
    )
    wrist_face.visual(
        Box((0.060, 0.050, 0.038)),
        origin=Origin(xyz=(0.058, 0.0, 0.092)),
        material=face_aluminum,
        name="wrist_neck",
    )
    wrist_face.visual(
        Box((0.018, 0.120, 0.100)),
        origin=Origin(xyz=(0.086, 0.0, 0.092)),
        material=face_aluminum,
        name="mounting_face",
    )
    for y in (-0.036, 0.036):
        for z in (0.062, 0.122):
            wrist_face.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(0.098, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bearing_black,
                name=f"face_socket_{len(wrist_face.visuals)}",
            )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=support,
        child=upper_arm,
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.8, lower=-2.20, upper=2.20),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_face,
        origin=Origin(xyz=(forearm_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=-2.80, upper=2.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "three supported revolute joints",
        len(revolute_joints) == 3,
        details=f"revolute={[joint.name for joint in revolute_joints]}",
    )
    ctx.check(
        "root support remains to the side of the chain",
        shoulder.origin.xyz[0] > 0.12,
        details=f"shoulder_origin={shoulder.origin.xyz}",
    )
    ctx.allow_overlap(
        "forearm",
        "upper_arm",
        elem_a="forearm_shell",
        elem_b="elbow_pin",
        reason="The steel elbow pin is intentionally shown passing through the forearm bearing bore.",
    )
    ctx.allow_overlap(
        "support",
        "upper_arm",
        elem_a="shoulder_pin",
        elem_b="upper_arm_shell",
        reason="The shoulder shaft is intentionally shown captured through the upper-arm bearing bore.",
    )
    ctx.allow_overlap(
        "forearm",
        "wrist_face",
        elem_a="wrist_pin",
        elem_b="wrist_turntable",
        reason="The wrist pin is intentionally shown captured through the turntable bearing bore.",
    )

    ctx.expect_overlap(
        "upper_arm",
        "support",
        axes="xy",
        min_overlap=0.035,
        elem_a="upper_arm_shell",
        elem_b="upper_bearing_shelf",
        name="shoulder link captured between side shelves",
    )
    ctx.expect_overlap(
        "support",
        "upper_arm",
        axes="xy",
        min_overlap=0.020,
        elem_a="shoulder_pin",
        elem_b="upper_arm_shell",
        name="shoulder pin stays centered in the arm bore",
    )
    ctx.expect_overlap(
        "forearm",
        "upper_arm",
        axes="xy",
        min_overlap=0.030,
        elem_a="forearm_shell",
        elem_b="upper_arm_shell",
        name="elbow bosses share a supported pivot footprint",
    )
    ctx.expect_overlap(
        "forearm",
        "upper_arm",
        axes="xy",
        min_overlap=0.018,
        elem_a="forearm_shell",
        elem_b="elbow_pin",
        name="elbow pin stays centered in the forearm bore",
    )
    ctx.expect_overlap(
        "wrist_face",
        "forearm",
        axes="xy",
        min_overlap=0.026,
        elem_a="wrist_turntable",
        elem_b="forearm_shell",
        name="wrist turntable centered on distal pivot",
    )
    ctx.expect_overlap(
        "forearm",
        "wrist_face",
        axes="xy",
        min_overlap=0.018,
        elem_a="wrist_pin",
        elem_b="wrist_turntable",
        name="wrist pin stays centered in the turntable bore",
    )

    rest_wrist_aabb = ctx.part_world_aabb("wrist_face")
    with ctx.pose({shoulder: 0.55, elbow: -0.80}):
        posed_wrist_aabb = ctx.part_world_aabb("wrist_face")
    ctx.check(
        "shoulder and elbow swing the cantilevered wrist in plan",
        rest_wrist_aabb is not None
        and posed_wrist_aabb is not None
        and abs(((posed_wrist_aabb[0][1] + posed_wrist_aabb[1][1]) / 2.0) - ((rest_wrist_aabb[0][1] + rest_wrist_aabb[1][1]) / 2.0)) > 0.10,
        details=f"rest={rest_wrist_aabb}, posed={posed_wrist_aabb}",
    )

    with ctx.pose({wrist: 1.35}):
        yawed_wrist_aabb = ctx.part_world_aabb("wrist_face")
    ctx.check(
        "wrist joint yaws the compact mounting face",
        rest_wrist_aabb is not None
        and yawed_wrist_aabb is not None
        and abs(((yawed_wrist_aabb[0][1] + yawed_wrist_aabb[1][1]) / 2.0) - ((rest_wrist_aabb[0][1] + rest_wrist_aabb[1][1]) / 2.0)) > 0.020,
        details=f"rest={rest_wrist_aabb}, wrist_yaw={yawed_wrist_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
