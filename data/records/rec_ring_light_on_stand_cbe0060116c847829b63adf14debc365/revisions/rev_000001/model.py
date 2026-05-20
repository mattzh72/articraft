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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rotate_point(
    point: tuple[float, float, float], yaw: float
) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (x * c - y * s, x * s + y * c, z)


def _aabb_center(aabb) -> tuple[float, float, float]:
    return (
        0.5 * (aabb[0][0] + aabb[1][0]),
        0.5 * (aabb[0][1] + aabb[1][1]),
        0.5 * (aabb[0][2] + aabb[1][2]),
    )


def _radial_xy(point: tuple[float, float, float]) -> float:
    return math.hypot(point[0], point[1])


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_shift) for z, y in rounded_rect_profile(height, width, radius)]


def _build_leg_mesh():
    sections = [
        _yz_section(0.026, 0.014, 0.004, 0.010, z_shift=-0.004),
        _yz_section(0.024, 0.012, 0.004, 0.032, z_shift=-0.006),
        _yz_section(0.020, 0.010, 0.003, 0.092, z_shift=-0.010),
        _yz_section(0.018, 0.008, 0.003, 0.142, z_shift=-0.013),
        _yz_section(0.028, 0.006, 0.002, 0.160, z_shift=-0.015),
    ]
    return mesh_from_geometry(section_loft(sections), "ring_light_tripod_leg")


def _build_hub_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.018, 0.000),
                (0.056, 0.000),
                (0.060, 0.006),
                (0.060, 0.020),
                (0.054, 0.028),
                (0.018, 0.028),
            ],
            [
                (0.017, 0.002),
                (0.034, 0.002),
                (0.039, 0.006),
                (0.039, 0.020),
                (0.034, 0.024),
                (0.017, 0.024),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        "ring_light_hub_shell",
    )


def _build_sleeve_shell():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.021, 0.000), (0.021, 0.130)],
            [(0.0155, 0.002), (0.0155, 0.128)],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        "ring_light_mast_sleeve",
    )


def _build_ring_body():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.102, -0.013),
                (0.124, -0.016),
                (0.145, -0.012),
                (0.148, 0.000),
                (0.145, 0.012),
                (0.124, 0.016),
                (0.102, 0.013),
            ],
            [
                (0.099, -0.010),
                (0.120, -0.012),
                (0.139, -0.009),
                (0.142, 0.000),
                (0.139, 0.009),
                (0.120, 0.012),
                (0.099, 0.010),
            ],
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ),
        "ring_light_head_body",
    )


def _build_ring_diffuser():
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.105, 0.0130),
                (0.141, 0.0130),
                (0.141, 0.0160),
                (0.105, 0.0160),
            ],
            [
                (0.108, 0.0135),
                (0.138, 0.0135),
                (0.138, 0.0155),
                (0.108, 0.0155),
            ],
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ),
        "ring_light_head_diffuser",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_ring_light")

    satin_black = model.material("satin_black", rgba=(0.16, 0.17, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.96, 0.97, 0.98, 0.92))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    ring_body_mesh = _build_ring_body()
    ring_diffuser_mesh = _build_ring_diffuser()

    base_hub = model.part("base_hub")
    base_hub.visual(
        Cylinder(radius=0.062, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=charcoal,
        name="hub_disc",
    )
    base_hub.visual(
        Cylinder(radius=0.022, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=satin_black,
        name="sleeve",
    )
    for index, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0), start=1):
        clip_pos = _rotate_point((0.072, 0.0085, 0.013), theta)
        clip_neg = _rotate_point((0.072, -0.0085, 0.013), theta)
        clip_cap = _rotate_point((0.072, 0.0, 0.022), theta)
        base_hub.visual(
            Box((0.014, 0.003, 0.014)),
            origin=Origin(xyz=clip_pos, rpy=(0.0, 0.0, theta)),
            material=charcoal,
            name=f"leg_{index}_clip_pos",
        )
        base_hub.visual(
            Box((0.014, 0.003, 0.014)),
            origin=Origin(xyz=clip_neg, rpy=(0.0, 0.0, theta)),
            material=charcoal,
            name=f"leg_{index}_clip_neg",
        )
        base_hub.visual(
            Box((0.020, 0.020, 0.004)),
            origin=Origin(xyz=clip_cap, rpy=(0.0, 0.0, theta)),
            material=satin_black,
            name=f"leg_{index}_clip_cap",
        )

    mast_inner = model.part("mast_inner")
    mast_inner.visual(
        Cylinder(radius=0.0165, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=aluminum,
        name="guide_collar",
    )
    mast_inner.visual(
        Cylinder(radius=0.0135, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=aluminum,
        name="mast_tube",
    )
    mast_inner.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material=charcoal,
        name="top_collar",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.028, 0.024, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=charcoal,
        name="clamp_block",
    )
    yoke.visual(
        Box((0.020, 0.018, 0.120)),
        origin=Origin(xyz=(-0.010, 0.0, 0.060)),
        material=charcoal,
        name="rear_neck",
    )
    yoke.visual(
        Box((0.060, 0.018, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.118)),
        material=charcoal,
        name="axis_bridge",
    )
    _add_member(
        yoke,
        (0.050, 0.0, 0.118),
        (0.048, 0.146, 0.118),
        0.0045,
        charcoal,
        name="left_arm",
    )
    _add_member(
        yoke,
        (0.050, 0.0, 0.118),
        (0.048, -0.146, 0.118),
        0.0045,
        charcoal,
        name="right_arm",
    )
    yoke.visual(
        Box((0.012, 0.014, 0.020)),
        origin=Origin(xyz=(0.054, 0.153, 0.118)),
        material=charcoal,
        name="left_pad",
    )
    yoke.visual(
        Box((0.012, 0.014, 0.020)),
        origin=Origin(xyz=(0.054, -0.153, 0.118)),
        material=charcoal,
        name="right_pad",
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        ring_body_mesh,
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="ring_body",
    )
    ring_head.visual(
        ring_diffuser_mesh,
        origin=Origin(xyz=(0.095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=diffuser_white,
        name="diffuser",
    )
    ring_head.visual(
        Cylinder(radius=0.010, length=0.292),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="axle_bar",
    )
    ring_head.visual(
        Box((0.020, 0.220, 0.070)),
        origin=Origin(xyz=(0.078, 0.0, -0.032)),
        material=charcoal,
        name="rear_frame",
    )
    ring_head.visual(
        Box((0.034, 0.054, 0.130)),
        origin=Origin(xyz=(0.072, 0.0, -0.108)),
        material=charcoal,
        name="rear_housing",
    )

    leg_parts = []
    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, theta in enumerate(leg_angles, start=1):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, theta)),
            material=aluminum,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.020, 0.012, 0.012)),
            origin=Origin(
                xyz=_rotate_point((0.016, 0.0, -0.004), theta),
                rpy=(0.0, 0.0, theta),
            ),
            material=satin_black,
            name="root_block",
        )
        leg.visual(
            Box((0.116, 0.018, 0.020)),
            origin=Origin(
                xyz=_rotate_point((0.074, 0.0, -0.014), theta),
                rpy=(0.0, 0.0, theta),
            ),
            material=satin_black,
            name="leg_body",
        )
        leg.visual(
            Box((0.028, 0.020, 0.006)),
            origin=Origin(
                xyz=_rotate_point((0.134, 0.0, -0.020), theta),
                rpy=(0.0, 0.0, theta),
            ),
            material=rubber,
            name="foot_pad",
        )
        leg_parts.append(leg)

    model.articulation(
        "hub_to_mast",
        ArticulationType.PRISMATIC,
        parent=base_hub,
        child=mast_inner,
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.15, lower=0.0, upper=0.120),
    )
    model.articulation(
        "mast_to_yoke",
        ArticulationType.FIXED,
        parent=mast_inner,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
    )
    model.articulation(
        "yoke_to_ring",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.75,
            upper=0.55,
        ),
    )
    for index, theta in enumerate(leg_angles, start=1):
        model.articulation(
            f"hub_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=base_hub,
            child=f"leg_{index}",
            origin=Origin(xyz=_rotate_point((0.072, 0.0, 0.013), theta)),
            axis=(-math.sin(theta), math.cos(theta), 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.5,
                lower=-1.10,
                upper=0.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_hub = object_model.get_part("base_hub")
    mast_inner = object_model.get_part("mast_inner")
    yoke = object_model.get_part("yoke")
    ring_head = object_model.get_part("ring_head")
    legs = [object_model.get_part(f"leg_{index}") for index in range(1, 4)]

    mast_joint = object_model.get_articulation("hub_to_mast")
    tilt_joint = object_model.get_articulation("yoke_to_ring")
    leg_joints = [
        object_model.get_articulation(f"hub_to_leg_{index}") for index in range(1, 4)
    ]

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

    ctx.expect_contact(mast_inner, base_hub, elem_a="mast_tube", elem_b="sleeve")
    ctx.expect_within(
        mast_inner,
        base_hub,
        axes="xy",
        inner_elem="guide_collar",
        outer_elem="sleeve",
    )
    ctx.expect_contact(yoke, mast_inner, elem_a="clamp_block", elem_b="top_collar")
    ctx.expect_contact(ring_head, yoke, elem_a="axle_bar", elem_b="left_pad")
    ctx.expect_contact(ring_head, yoke, elem_a="axle_bar", elem_b="right_pad")

    for index, leg in enumerate(legs, start=1):
        ctx.expect_contact(
            leg,
            base_hub,
            name=f"leg_{index}_hinge_is_clipped",
        )
        ctx.expect_gap(
            base_hub,
            leg,
            axis="z",
            positive_elem="hub_disc",
            negative_elem="foot_pad",
            min_gap=0.003,
            max_gap=0.020,
            name=f"leg_{index}_opens_below_hub",
        )

    ring_rest = ctx.part_world_position(ring_head)
    assert ring_rest is not None
    with ctx.pose({mast_joint: mast_joint.motion_limits.upper}):
        ring_extended = ctx.part_world_position(ring_head)
        assert ring_extended is not None
        ctx.expect_within(
            mast_inner,
            base_hub,
            axes="xy",
            inner_elem="guide_collar",
            outer_elem="sleeve",
            name="mast_stays_guided_when_extended",
        )
    ctx.check(
        "mast_extension_raises_ring",
        ring_extended[2] > ring_rest[2] + 0.10,
        details=(
            f"Ring center z changed from {ring_rest[2]:.3f} m to "
            f"{ring_extended[2]:.3f} m; expected more than 0.10 m of lift."
        ),
    )

    rear_rest_box = ctx.part_element_world_aabb(ring_head, elem="rear_housing")
    assert rear_rest_box is not None
    rear_rest_center = _aabb_center(rear_rest_box)
    tilt_centers = []
    for pose_value in (tilt_joint.motion_limits.lower, tilt_joint.motion_limits.upper):
        if pose_value is None or abs(pose_value) < 1e-6:
            continue
        with ctx.pose({tilt_joint: pose_value}):
            rear_box = ctx.part_element_world_aabb(ring_head, elem="rear_housing")
            assert rear_box is not None
            tilt_centers.append(_aabb_center(rear_box))
    best_tilt_center = max(
        tilt_centers,
        key=lambda center: abs(center[2] - rear_rest_center[2]),
    )
    ctx.check(
        "ring_head_tilts_on_horizontal_axis",
        abs(best_tilt_center[2] - rear_rest_center[2]) > 0.03
        and abs(best_tilt_center[0] - rear_rest_center[0]) > 0.02,
        details=(
            f"Rear housing moved from {rear_rest_center} to {best_tilt_center}; "
            "expected substantial x/z travel from tilt motion about a horizontal axis."
        ),
    )

    for index, (leg, leg_joint) in enumerate(zip(legs, leg_joints), start=1):
        foot_rest_box = ctx.part_element_world_aabb(leg, elem="foot_pad")
        assert foot_rest_box is not None
        foot_rest_center = _aabb_center(foot_rest_box)
        folded_samples: list[tuple[float, tuple[float, float, float]]] = []
        for pose_value in (leg_joint.motion_limits.lower, leg_joint.motion_limits.upper):
            if pose_value is None or abs(pose_value) < 1e-6:
                continue
            with ctx.pose({leg_joint: pose_value}):
                foot_box = ctx.part_element_world_aabb(leg, elem="foot_pad")
                assert foot_box is not None
                folded_samples.append((pose_value, _aabb_center(foot_box)))
        folded_pose, folded_center = min(
            folded_samples,
            key=lambda item: _radial_xy(item[1]),
        )
        with ctx.pose({leg_joint: folded_pose}):
            ctx.expect_contact(
                leg,
                base_hub,
                name=f"leg_{index}_stays_clipped_when_folded",
            )
        ctx.check(
            f"leg_{index}_folds_inward_for_storage",
            _radial_xy(folded_center) < _radial_xy(foot_rest_center) - 0.05
            and folded_center[2] > foot_rest_center[2] + 0.04,
            details=(
                f"Foot pad moved from {foot_rest_center} to {folded_center}; "
                "expected the folded pose to draw the foot inward and upward."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
