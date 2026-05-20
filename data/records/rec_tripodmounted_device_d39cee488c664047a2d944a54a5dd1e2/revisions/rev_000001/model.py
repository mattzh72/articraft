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
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_points(radius: float, z: float, count: int = 8) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / count),
            radius * math.sin((2.0 * math.pi * i) / count),
            z,
        )
        for i in range(count)
    ]


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    yaw = math.atan2(dy, dx)
    center = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _leg_segment_origin(start: float, length: float, splay: float) -> Origin:
    center = start + 0.5 * length
    return Origin(
        xyz=(center * math.sin(splay), 0.0, -center * math.cos(splay)),
        rpy=(0.0, math.pi - splay, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_device_mount")

    anodized = model.material("anodized_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    device_gray = model.material("device_gray", rgba=(0.48, 0.5, 0.54, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    accent = model.material("accent", rgba=(0.18, 0.24, 0.34, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.055, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=anodized,
        name="hub_shell",
    )
    crown.visual(
        Cylinder(radius=0.04, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="pan_bearing",
    )
    for idx, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        shoulder_x = 0.054 * math.cos(theta)
        shoulder_y = 0.054 * math.sin(theta)
        rib_x = 0.093 * math.cos(theta)
        rib_y = 0.093 * math.sin(theta)
        hinge_x = 0.122 * math.cos(theta)
        hinge_y = 0.122 * math.sin(theta)
        fork_offset = 0.018
        crown.visual(
            Box((0.05, 0.032, 0.028)),
            origin=Origin(xyz=(shoulder_x, shoulder_y, -0.042), rpy=(0.0, 0.0, theta)),
            material=anodized,
            name=f"leg_shoulder_{idx}",
        )
        crown.visual(
            Box((0.056, 0.006, 0.018)),
            origin=Origin(
                xyz=(
                    rib_x - fork_offset * math.sin(theta),
                    rib_y + fork_offset * math.cos(theta),
                    -0.045,
                ),
                rpy=(0.0, 0.0, theta),
            ),
            material=anodized,
            name=f"leg_rib_left_{idx}",
        )
        crown.visual(
            Box((0.056, 0.006, 0.018)),
            origin=Origin(
                xyz=(
                    rib_x + fork_offset * math.sin(theta),
                    rib_y - fork_offset * math.cos(theta),
                    -0.045,
                ),
                rpy=(0.0, 0.0, theta),
            ),
            material=anodized,
            name=f"leg_rib_right_{idx}",
        )
        crown.visual(
            Box((0.028, 0.008, 0.044)),
            origin=Origin(
                xyz=(
                    hinge_x - fork_offset * math.sin(theta),
                    hinge_y + fork_offset * math.cos(theta),
                    -0.036,
                ),
                rpy=(0.0, 0.0, theta),
            ),
            material=steel,
            name=f"leg_fork_left_{idx}",
        )
        crown.visual(
            Box((0.028, 0.008, 0.044)),
            origin=Origin(
                xyz=(
                    hinge_x + fork_offset * math.sin(theta),
                    hinge_y - fork_offset * math.cos(theta),
                    -0.036,
                ),
                rpy=(0.0, 0.0, theta),
            ),
            material=steel,
            name=f"leg_fork_right_{idx}",
        )
        crown.visual(
            Cylinder(radius=0.009, length=0.046),
            origin=Origin(xyz=(hinge_x, hinge_y, -0.05), rpy=(math.pi / 2.0, 0.0, theta)),
            material=steel,
            name=f"leg_pin_{idx}",
        )

    hoop = tube_from_spline_points(
        _circle_points(radius=0.145, z=0.11, count=12),
        radius=0.006,
        closed_spline=True,
        cap_ends=False,
        samples_per_segment=10,
        radial_segments=18,
    )
    crown.visual(
        mesh_from_geometry(hoop, "guard_hoop"),
        material=steel,
        name="guard_hoop",
    )
    for idx, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        post_x = 0.138 * math.cos(theta)
        post_y = 0.138 * math.sin(theta)
        crown.visual(
            Cylinder(radius=0.006, length=0.12),
            origin=Origin(xyz=(post_x, post_y, 0.05)),
            material=steel,
            name=f"guard_post_{idx}",
        )
        strut_origin, strut_length = _segment_pose(
            (0.045 * math.cos(theta), 0.045 * math.sin(theta), -0.002),
            (post_x, post_y, 0.012),
        )
        crown.visual(
            Cylinder(radius=0.005, length=strut_length),
            origin=strut_origin,
            material=steel,
            name=f"guard_strut_{idx}",
        )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=accent,
        name="rotor_base",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=accent,
        name="pan_column",
    )
    head.visual(
        Box((0.07, 0.12, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=accent,
        name="yoke_base",
    )
    head.visual(
        Box((0.09, 0.014, 0.09)),
        origin=Origin(xyz=(0.0, 0.056, 0.125)),
        material=accent,
        name="left_cheek",
    )
    head.visual(
        Box((0.09, 0.014, 0.09)),
        origin=Origin(xyz=(0.0, -0.056, 0.125)),
        material=accent,
        name="right_cheek",
    )
    head.visual(
        Box((0.02, 0.112, 0.016)),
        origin=Origin(xyz=(-0.03, 0.0, 0.148)),
        material=accent,
        name="rear_tie_bar",
    )

    device = model.part("device")
    device.visual(
        Cylinder(radius=0.007, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_shaft",
    )
    device.visual(
        Box((0.14, 0.08, 0.008)),
        origin=Origin(xyz=(0.035, 0.0, -0.044)),
        material=steel,
        name="device_plate",
    )
    device.visual(
        Box((0.12, 0.08, 0.06)),
        origin=Origin(xyz=(0.04, 0.0, -0.014)),
        material=device_gray,
        name="device_body",
    )
    device.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(xyz=(0.115, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="lens_barrel",
    )

    leg_splay = 0.42
    upper_start = 0.05
    upper_len = 0.37
    lower_len = 0.34
    lower_start = upper_start + upper_len - 0.03
    foot_start = lower_start + lower_len - 0.03
    leg_axis = (math.sin(leg_splay), 0.0, -math.cos(leg_splay))
    upper_origin = _leg_segment_origin(upper_start, upper_len, leg_splay)
    lower_origin = _leg_segment_origin(lower_start, lower_len, leg_splay)
    foot_origin = _leg_segment_origin(foot_start, 0.06, leg_splay)
    for idx, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        leg = model.part(f"leg_{idx}")
        leg.visual(
            Cylinder(radius=0.016, length=0.024),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hinge_sleeve",
        )
        for brace_name, side in (("left", 0.012), ("right", -0.012)):
            brace_origin, brace_len = _segment_pose(
                (0.015 * leg_axis[0], side, 0.015 * leg_axis[2]),
                (0.078 * leg_axis[0], side, 0.078 * leg_axis[2]),
            )
            leg.visual(
                Cylinder(radius=0.0045, length=brace_len),
                origin=brace_origin,
                material=steel,
                name=f"upper_brace_{brace_name}",
            )
        leg.visual(
            Cylinder(radius=0.018, length=upper_len),
            origin=upper_origin,
            material=anodized,
            name="upper_tube",
        )
        leg.visual(
            Cylinder(radius=0.014, length=lower_len),
            origin=lower_origin,
            material=steel,
            name="lower_tube",
        )
        leg.visual(
            Cylinder(radius=0.016, length=0.06),
            origin=foot_origin,
            material=rubber,
            name="foot_tip",
        )
        model.articulation(
            f"crown_to_leg_{idx}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(0.122 * math.cos(theta), 0.122 * math.sin(theta), -0.05),
                rpy=(0.0, 0.0, theta),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=1.18, effort=10.0, velocity=1.2),
        )

    model.articulation(
        "crown_to_head",
        ArticulationType.REVOLUTE,
        parent=crown,
        child=head,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=8.0, velocity=1.8),
    )
    model.articulation(
        "head_to_device",
        ArticulationType.REVOLUTE,
        parent=head,
        child=device,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=0.85, effort=6.0, velocity=1.4),
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
    crown = object_model.get_part("crown")
    head = object_model.get_part("head")
    device = object_model.get_part("device")
    pan = object_model.get_articulation("crown_to_head")
    tilt = object_model.get_articulation("head_to_device")

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    for idx in (1, 2, 3):
        ctx.allow_overlap(
            crown,
            object_model.get_part(f"leg_{idx}"),
            elem_a=f"leg_pin_{idx}",
            elem_b="hinge_sleeve",
            reason="Each tripod leg rotates on a crown hinge pin modeled as a solid pin passing through a hinge sleeve proxy.",
        )

    ctx.expect_overlap(
        head,
        crown,
        axes="xy",
        elem_a="rotor_base",
        elem_b="pan_bearing",
        min_overlap=0.05,
        name="pan head remains centered over the crown bearing",
    )
    ctx.expect_gap(
        device,
        crown,
        axis="z",
        positive_elem="device_plate",
        negative_elem="pan_bearing",
        min_gap=0.05,
        name="device plate clears the crown bearing stage",
    )

    rest_lens_z = _aabb_center_z(ctx.part_element_world_aabb(device, elem="lens_barrel"))
    with ctx.pose({tilt: tilt.motion_limits.upper}):
        tilted_lens_z = _aabb_center_z(ctx.part_element_world_aabb(device, elem="lens_barrel"))
    ctx.check(
        "tilt joint lifts the device at upper limit",
        rest_lens_z is not None and tilted_lens_z is not None and tilted_lens_z > rest_lens_z + 0.04,
        details=f"rest_z={rest_lens_z}, tilted_z={tilted_lens_z}",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({pan: math.pi / 2.0}):
        panned_head_pos = ctx.part_world_position(head)
    ctx.check(
        "pan joint rotates in place about the vertical axis",
        rest_head_pos is not None
        and panned_head_pos is not None
        and abs(panned_head_pos[0] - rest_head_pos[0]) < 1e-6
        and abs(panned_head_pos[1] - rest_head_pos[1]) < 1e-6
        and abs(panned_head_pos[2] - rest_head_pos[2]) < 1e-6,
        details=f"rest={rest_head_pos}, panned={panned_head_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
