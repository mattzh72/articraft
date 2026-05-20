from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _yz_section(width: float, height: float, radius: float, x: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for z, y in rounded_rect_profile(height, width, radius, corner_segments=10)]


def _circle_profile(radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _build_tower_body_geometry():
    base_plate = ExtrudeGeometry(
        rounded_rect_profile(0.34, 0.26, 0.045, corner_segments=10),
        0.035,
    ).translate(0.0, 0.0, 0.0175)
    base_plate.merge(
        ExtrudeGeometry(
            rounded_rect_profile(0.20, 0.16, 0.030, corner_segments=8),
            0.055,
        ).translate(0.0, 0.0, 0.0625)
    )
    base_plate.merge(
        section_loft(
            [
                _xy_section(0.16, 0.12, 0.022, 0.090),
                _xy_section(0.128, 0.095, 0.020, 0.620),
                _xy_section(0.108, 0.080, 0.017, 1.020),
            ]
        )
    )
    base_plate.merge(
        CylinderGeometry(radius=0.062, height=0.060, radial_segments=36).translate(0.0, 0.0, 1.050)
    )
    return base_plate


def _build_pan_body_geometry():
    yoke = CylinderGeometry(radius=0.068, height=0.045, radial_segments=36).translate(0.0, 0.0, 0.0225)
    yoke.merge(CylinderGeometry(radius=0.090, height=0.018, radial_segments=36).translate(0.0, 0.0, 0.054))
    yoke.merge(BoxGeometry((0.050, 0.112, 0.080)).translate(-0.006, 0.0, 0.103))
    yoke.merge(BoxGeometry((0.080, 0.180, 0.035)).translate(0.020, 0.0, 0.080))

    arm_profile = rounded_rect_profile(0.030, 0.046, radius=0.010, corner_segments=6)
    left_arm = sweep_profile_along_spline(
        [
            (0.006, 0.076, 0.080),
            (0.018, 0.090, 0.118),
            (0.040, 0.108, 0.182),
            (0.062, 0.116, 0.235),
        ],
        profile=arm_profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    right_arm = sweep_profile_along_spline(
        [
            (0.006, -0.076, 0.080),
            (0.018, -0.090, 0.118),
            (0.040, -0.108, 0.182),
            (0.062, -0.116, 0.235),
        ],
        profile=arm_profile,
        samples_per_segment=14,
        cap_profile=True,
    )
    yoke.merge(left_arm)
    yoke.merge(right_arm)
    return yoke


def _build_head_shell_geometry():
    return section_loft(
        [
            _yz_section(0.104, 0.128, 0.026, -0.065),
            _yz_section(0.154, 0.182, 0.044, 0.015),
            _yz_section(0.184, 0.216, 0.054, 0.165),
            _yz_section(0.174, 0.198, 0.050, 0.285),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_searchlight_tower")

    painted_metal = model.material("painted_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    deep_graphite = model.material("deep_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    elastomer_black = model.material("elastomer_black", rgba=(0.06, 0.06, 0.07, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.78, 0.90, 0.96, 0.38))

    tower = model.part("tower")
    tower.visual(
        mesh_from_geometry(_build_tower_body_geometry(), "searchlight_tower_body"),
        material=painted_metal,
        name="tower_body",
    )
    tower.visual(
        Cylinder(radius=0.078, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.090)),
        material=polymer_dark,
        name="tower_bearing_trim",
    )
    foot_positions = ((0.120, 0.090), (0.120, -0.090), (-0.120, 0.090), (-0.120, -0.090))
    for index, (x_pos, y_pos) in enumerate(foot_positions):
        tower.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, -0.005)),
            material=elastomer_black,
            name=f"foot_pad_{index}",
        )
    tower.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, 1.11)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        mesh_from_geometry(_build_pan_body_geometry(), "searchlight_pan_yoke"),
        material=painted_metal,
        name="pan_body",
    )
    pan_stage.visual(
        Box((0.046, 0.026, 0.054)),
        origin=Origin(xyz=(0.112, 0.119, 0.235)),
        material=polymer_dark,
        name="left_bearing_block",
    )
    pan_stage.visual(
        Box((0.046, 0.026, 0.054)),
        origin=Origin(xyz=(0.112, -0.119, 0.235)),
        material=polymer_dark,
        name="right_bearing_block",
    )
    pan_stage.visual(
        Box((0.022, 0.024, 0.064)),
        origin=Origin(xyz=(0.087, 0.119, 0.222)),
        material=painted_metal,
        name="left_bearing_strut",
    )
    pan_stage.visual(
        Box((0.022, 0.024, 0.064)),
        origin=Origin(xyz=(0.087, -0.119, 0.222)),
        material=painted_metal,
        name="right_bearing_strut",
    )
    pan_stage.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.112, 0.133, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=deep_graphite,
        name="left_bearing_cap",
    )
    pan_stage.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.112, -0.133, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=deep_graphite,
        name="right_bearing_cap",
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.12, 0.28, 0.30)),
        mass=5.6,
        origin=Origin(xyz=(0.030, 0.0, 0.140)),
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_build_head_shell_geometry(), "searchlight_head_shell"),
        material=painted_metal,
        name="head_shell",
    )
    bezel_ring = ExtrudeWithHolesGeometry(
        _circle_profile(0.086, segments=40),
        [_circle_profile(0.076, segments=40)],
        0.024,
    )
    head.visual(
        mesh_from_geometry(bezel_ring, "searchlight_front_bezel"),
        origin=Origin(xyz=(0.297, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=deep_graphite,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.076, length=0.006),
        origin=Origin(xyz=(0.306, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="lens",
    )
    head.visual(
        Box((0.028, 0.108, 0.124)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0)),
        material=polymer_dark,
        name="rear_service_cap",
    )
    head.visual(
        Box((0.056, 0.036, 0.072)),
        origin=Origin(xyz=(0.010, 0.078, 0.0)),
        material=polymer_dark,
        name="left_trunnion_mount",
    )
    head.visual(
        Box((0.056, 0.036, 0.072)),
        origin=Origin(xyz=(0.010, -0.078, 0.0)),
        material=polymer_dark,
        name="right_trunnion_mount",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=deep_graphite,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, -0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=deep_graphite,
        name="right_trunnion",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.38, 0.20, 0.22)),
        mass=6.8,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2),
    )

    model.articulation(
        "pan_to_head",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=head,
        origin=Origin(xyz=(0.110, 0.0, 0.235)),
        # The head projects along local +X from the tilt axis.
        # -Y makes positive q pitch the beam upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=math.radians(-25.0),
            upper=math.radians(68.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_stage = object_model.get_part("pan_stage")
    head = object_model.get_part("head")
    pan_joint = object_model.get_articulation("tower_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_head")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.check(
        "core_parts_present",
        all(part is not None for part in (tower, pan_stage, head)),
        "Tower, pan stage, and head must all exist.",
    )
    ctx.check(
        "pan_axis_vertical",
        tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical pan axis, got {pan_joint.axis}.",
    )
    ctx.check(
        "tilt_axis_crosshead",
        tuple(tilt_joint.axis) == (0.0, -1.0, 0.0),
        f"Expected tilt axis along -Y, got {tilt_joint.axis}.",
    )

    ctx.expect_contact(
        pan_stage,
        tower,
        elem_a="pan_body",
        elem_b="tower_bearing_trim",
        name="pan_bearing_is_seated",
    )
    ctx.expect_contact(
        head,
        pan_stage,
        elem_a="left_trunnion",
        elem_b="left_bearing_block",
        name="left_trunnion_carried_by_bearing_block",
    )
    ctx.expect_contact(
        head,
        pan_stage,
        elem_a="right_trunnion",
        elem_b="right_bearing_block",
        name="right_trunnion_carried_by_bearing_block",
    )

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_gap(head, tower, axis="z", min_gap=0.10, name="neutral_head_clears_tower")

    with ctx.pose({pan_joint: 0.0, tilt_joint: tilt_joint.motion_limits.lower}):
        ctx.expect_gap(head, tower, axis="z", min_gap=0.012, name="down_tilt_clears_tower")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        neutral_bezel_center = _center_from_aabb(ctx.part_element_world_aabb(head, elem="front_bezel"))
    with ctx.pose({pan_joint: 0.0, tilt_joint: math.radians(45.0)}):
        raised_bezel_center = _center_from_aabb(ctx.part_element_world_aabb(head, elem="front_bezel"))
    with ctx.pose({pan_joint: math.pi / 2.0, tilt_joint: 0.0}):
        panned_bezel_center = _center_from_aabb(ctx.part_element_world_aabb(head, elem="front_bezel"))

    tilt_ok = (
        neutral_bezel_center is not None
        and raised_bezel_center is not None
        and raised_bezel_center[2] > neutral_bezel_center[2] + 0.12
    )
    ctx.check(
        "positive_tilt_raises_bezel",
        tilt_ok,
        f"Neutral bezel center={neutral_bezel_center}, raised bezel center={raised_bezel_center}.",
    )

    pan_ok = (
        neutral_bezel_center is not None
        and panned_bezel_center is not None
        and panned_bezel_center[1] > neutral_bezel_center[1] + 0.18
    )
    ctx.check(
        "positive_pan_swings_bezel_toward_positive_y",
        pan_ok,
        f"Neutral bezel center={neutral_bezel_center}, panned bezel center={panned_bezel_center}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
