from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


BODY_LENGTH = 0.230
BODY_WIDTH = 0.160
BODY_HEIGHT = 0.036
HINGE_X = -0.068
HINGE_Z = 0.226
BOWL_CENTER_X = 0.055
BOWL_SEAT_TOP = 0.046
WHISK_MOUNT_X = 0.123
WHISK_MOUNT_Z = -0.058
HEAD_OPEN_ANGLE = math.radians(62.0)
HEAD_SHELL_OFFSET_X = 0.030
HEAD_SHELL_OFFSET_Z = 0.006


def yz_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    z_center: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width_y, height_z, radius, corner_segments=corner_segments)
    return [(x, y, z + z_center) for y, z in profile]


def build_body_geometry():
    base = ExtrudeGeometry.from_z0(
        rounded_rect_profile(BODY_LENGTH, BODY_WIDTH, 0.040, corner_segments=10),
        BODY_HEIGHT,
    )

    tower = section_loft(
        [
            yz_section(-0.088, 0.096, 0.120, 0.020, z_center=0.096),
            yz_section(-0.082, 0.092, 0.158, 0.022, z_center=0.115),
            yz_section(-0.074, 0.084, 0.184, 0.022, z_center=0.129),
            yz_section(-0.066, 0.074, 0.180, 0.020, z_center=0.136),
        ]
    )

    seat = CylinderGeometry(0.056, 0.010).translate(BOWL_CENTER_X, 0.0, 0.041)
    hinge_pod = CylinderGeometry(0.030, 0.084).rotate_x(-math.pi / 2.0).translate(HINGE_X, 0.0, HINGE_Z)

    base.merge(tower)
    base.merge(seat)
    base.merge(hinge_pod)
    return base


def build_head_geometry():
    head = section_loft(
        [
            yz_section(0.000, 0.086, 0.106, 0.022, z_center=0.014),
            yz_section(0.050, 0.112, 0.132, 0.028, z_center=0.022),
            yz_section(0.118, 0.130, 0.146, 0.032, z_center=0.026),
            yz_section(0.186, 0.110, 0.120, 0.026, z_center=0.014),
            yz_section(0.240, 0.082, 0.086, 0.018, z_center=0.000),
        ]
    )

    drive_collar = CylinderGeometry(0.019, 0.0254).translate(WHISK_MOUNT_X - HEAD_SHELL_OFFSET_X, 0.0, -0.0593)
    knob_boss = (
        CylinderGeometry(0.011, 0.020)
        .rotate_x(-math.pi / 2.0)
        .translate(0.082 - HEAD_SHELL_OFFSET_X, 0.055, 0.026)
    )
    head.merge(drive_collar)
    head.merge(knob_boss)
    return head


def build_bowl_geometry():
    return LatheGeometry.from_shell_profiles(
        [
            (0.028, 0.000),
            (0.042, 0.010),
            (0.078, 0.040),
            (0.095, 0.086),
            (0.101, 0.100),
        ],
        [
            (0.000, 0.006),
            (0.025, 0.013),
            (0.071, 0.042),
            (0.088, 0.090),
        ],
        segments=56,
    )


def build_whisk_geometry():
    whisk = CylinderGeometry(0.0045, 0.024).translate(0.0, 0.0, -0.012)
    whisk.merge(CylinderGeometry(0.0090, 0.022).translate(0.0, 0.0, -0.034))

    loop_count = 6
    for idx in range(loop_count):
        angle = idx * math.pi / loop_count
        c = math.cos(angle)
        s = math.sin(angle)
        whisk.merge(
            tube_from_spline_points(
                [
                    (0.007 * c, 0.007 * s, -0.040),
                    (0.019 * c, 0.019 * s, -0.053),
                    (0.032 * c, 0.032 * s, -0.071),
                    (0.026 * c, 0.026 * s, -0.084),
                    (-0.004 * c, -0.004 * s, -0.100),
                    (-0.026 * c, -0.026 * s, -0.084),
                    (-0.032 * c, -0.032 * s, -0.071),
                    (-0.019 * c, -0.019 * s, -0.053),
                    (-0.007 * c, -0.007 * s, -0.040),
                ],
                radius=0.00135,
                samples_per_segment=12,
                radial_segments=14,
            )
        )

    return whisk


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    enamel = model.material("enamel", rgba=(0.84, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.87, 0.88, 0.89, 1.0))
    trim = model.material("trim", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(build_body_geometry(), "body_shell"),
        material=enamel,
        name="body_shell",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(build_bowl_geometry(), "bowl_shell"),
        material=steel,
        name="bowl_shell",
    )

    model.articulation(
        "body_to_bowl",
        ArticulationType.FIXED,
        parent=body,
        child=bowl,
        origin=Origin(xyz=(BOWL_CENTER_X, 0.0, BOWL_SEAT_TOP)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(build_head_geometry(), "head_shell"),
        origin=Origin(xyz=(HEAD_SHELL_OFFSET_X, 0.0, HEAD_SHELL_OFFSET_Z)),
        material=enamel,
        name="head_shell",
    )

    tilt_head = model.articulation(
        "body_to_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.2,
            lower=0.0,
            upper=HEAD_OPEN_ANGLE,
        ),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="knob_shaft",
    )
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.018,
                body_style="skirted",
                top_diameter=0.028,
                skirt=KnobSkirt(0.038, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=14, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="knob_cap",
    )

    model.articulation(
        "head_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=speed_knob,
        origin=Origin(xyz=(0.082, 0.0650, 0.026)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    whisk = model.part("whisk")
    whisk.visual(
        mesh_from_geometry(build_whisk_geometry(), "whisk_shell"),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=steel,
        name="whisk_shell",
    )

    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(WHISK_MOUNT_X, 0.0, WHISK_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    tilt_head.meta["qc_samples"] = [0.0, HEAD_OPEN_ANGLE]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    speed_knob = object_model.get_part("speed_knob")

    tilt_head = object_model.get_articulation("body_to_head")
    knob_joint = object_model.get_articulation("head_to_speed_knob")
    whisk_joint = object_model.get_articulation("head_to_whisk")

    ctx.expect_contact(
        bowl,
        body,
        contact_tol=0.0015,
        name="bowl sits on the mixer base",
    )
    ctx.expect_overlap(
        whisk,
        bowl,
        axes="xy",
        min_overlap=0.050,
        name="closed whisk is centered over the bowl",
    )
    ctx.expect_gap(
        speed_knob,
        head,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        name="speed knob sits on the head side surface",
    )
    ctx.expect_overlap(
        speed_knob,
        head,
        axes="xz",
        min_overlap=0.020,
        name="speed knob aligns with the head side panel",
    )

    whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({tilt_head: HEAD_OPEN_ANGLE}):
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            min_gap=0.030,
            name="open head lifts the whisk clear of the bowl",
        )
        whisk_open = ctx.part_world_position(whisk)

    ctx.check(
        "head tilt raises whisk origin",
        whisk_rest is not None and whisk_open is not None and whisk_open[2] > whisk_rest[2] + 0.060,
        details=f"rest={whisk_rest}, open={whisk_open}",
    )

    ctx.check(
        "speed knob is continuous",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=f"limits={knob_joint.motion_limits!r}",
    )
    ctx.check(
        "whisk is continuous",
        whisk_joint.motion_limits is not None
        and whisk_joint.motion_limits.lower is None
        and whisk_joint.motion_limits.upper is None,
        details=f"limits={whisk_joint.motion_limits!r}",
    )

    return ctx.report()


object_model = build_object_model()
