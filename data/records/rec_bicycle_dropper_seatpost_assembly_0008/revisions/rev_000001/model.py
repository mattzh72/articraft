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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    radius: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _tube_shell_mesh(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    name: str,
    *,
    segments: int = 72,
):
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _yz_section(x_pos: float, width: float, thickness: float, z_offset: float):
    corner_radius = max(0.0015, min(width * 0.22, thickness * 0.42))
    profile = rounded_rect_profile(width, thickness, corner_radius, corner_segments=8)
    return [(x_pos, y, z + z_offset) for y, z in profile]


def _build_saddle_body_mesh(
    name: str,
    *,
    width_delta: float = 0.0,
    thickness_scale: float = 1.0,
    z_shift: float = 0.0,
):
    section_specs = [
        (-0.138, 0.090, 0.018, 0.040),
        (-0.102, 0.145, 0.040, 0.034),
        (-0.050, 0.138, 0.042, 0.028),
        (0.005, 0.098, 0.033, 0.022),
        (0.060, 0.058, 0.024, 0.018),
        (0.108, 0.032, 0.015, 0.016),
        (0.135, 0.018, 0.010, 0.015),
    ]
    sections = [
        _yz_section(
            x_pos,
            max(0.012, width + width_delta),
            max(0.007, thickness * thickness_scale),
            z_pos + z_shift,
        )
        for x_pos, width, thickness, z_pos in section_specs
    ]
    return _save_mesh(name, repair_loft(section_loft(sections)))


def _mirror_y(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(x, -y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_seatpost")

    post_black = model.material("post_black", rgba=(0.10, 0.11, 0.12, 1.0))
    seal_black = model.material("seal_black", rgba=(0.06, 0.06, 0.07, 1.0))
    hard_anodized = model.material("hard_anodized", rgba=(0.18, 0.19, 0.20, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.14, 0.14, 0.15, 1.0))
    saddle_cover = model.material("saddle_cover", rgba=(0.08, 0.08, 0.09, 1.0))
    saddle_shell = model.material("saddle_shell", rgba=(0.17, 0.18, 0.19, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.70, 0.71, 0.74, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.62, 0.63, 0.66, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        _tube_shell_mesh(0.0160, 0.0143, 0.0, 0.278, "outer_sleeve_shell"),
        material=post_black,
        name="outer_sleeve_shell",
    )
    outer_sleeve.visual(
        _tube_shell_mesh(0.0168, 0.0133, 0.278, 0.292, "outer_sleeve_bushing_v2"),
        material=seal_black,
        name="guide_bushing",
    )
    outer_sleeve.visual(
        _tube_shell_mesh(0.0170, 0.0148, 0.000, 0.026, "outer_sleeve_lower_collar"),
        material=hard_anodized,
        name="lower_collar",
    )
    outer_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0168, length=0.292),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        _tube_shell_mesh(0.0127, 0.0108, 0.0, 0.228, "inner_tube_shell"),
        material=hard_anodized,
        name="inner_tube_shell",
    )
    inner_tube.visual(
        Cylinder(radius=0.0152, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=hard_anodized,
        name="travel_collar",
    )
    inner_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0127, length=0.228),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=clamp_black,
        name="head_pivot",
    )
    saddle_clamp.visual(
        Box((0.032, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=clamp_black,
        name="clamp_base",
    )
    saddle_clamp.visual(
        Box((0.010, 0.012, 0.032)),
        origin=Origin(xyz=(0.014, 0.0, 0.026)),
        material=clamp_black,
        name="front_tower",
    )
    saddle_clamp.visual(
        Box((0.010, 0.012, 0.032)),
        origin=Origin(xyz=(-0.014, 0.0, 0.026)),
        material=clamp_black,
        name="rear_tower",
    )
    saddle_clamp.visual(
        Box((0.010, 0.060, 0.004)),
        origin=Origin(xyz=(0.014, 0.0, 0.0185)),
        material=clamp_black,
        name="front_lower_bar",
    )
    saddle_clamp.visual(
        Box((0.010, 0.070, 0.004)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0185)),
        material=clamp_black,
        name="rear_lower_bar",
    )
    for x_pos, bolt_name in ((0.014, "front"), (-0.014, "rear")):
        saddle_clamp.visual(
            Cylinder(radius=0.0032, length=0.028),
            origin=Origin(xyz=(x_pos, 0.0, 0.030)),
            material=bolt_steel,
            name=f"{bolt_name}_bolt",
        )
        saddle_clamp.visual(
            Box((0.010, 0.010, 0.004)),
            origin=Origin(xyz=(x_pos, 0.0, 0.046)),
            material=bolt_steel,
            name=f"{bolt_name}_nut",
        )
    saddle_clamp.inertial = Inertial.from_geometry(
        Box((0.050, 0.052, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        _build_saddle_body_mesh("saddle_pad_v2"),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=saddle_cover,
        name="saddle_pad",
    )
    saddle.visual(
        _build_saddle_body_mesh(
            "saddle_base_shell_v2",
            width_delta=-0.018,
            thickness_scale=0.55,
            z_shift=-0.011,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=saddle_shell,
        name="saddle_base",
    )
    for idx, xyz in enumerate(
        [
            (-0.096, 0.032, 0.039),
            (-0.096, -0.032, 0.039),
            (0.082, 0.022, 0.041),
            (0.082, -0.022, 0.041),
        ]
    ):
        saddle.visual(
            Box((0.014, 0.010, 0.010)),
            origin=Origin(xyz=xyz),
            material=saddle_shell,
            name=f"rail_receiver_{idx}",
        )
    left_rail_points = [
        (-0.104, 0.026, 0.010),
        (-0.088, 0.024, 0.004),
        (-0.058, 0.022, 0.000),
        (-0.010, 0.022, 0.000),
        (0.040, 0.021, 0.001),
        (0.078, 0.017, 0.008),
        (0.100, 0.010, 0.018),
    ]
    saddle.visual(
        _save_mesh(
            "left_saddle_rail_v3",
            tube_from_spline_points(
                left_rail_points,
                radius=0.003,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=rail_steel,
        name="left_rail",
    )
    saddle.visual(
        _save_mesh(
            "right_saddle_rail_v3",
            tube_from_spline_points(
                _mirror_y(left_rail_points),
                radius=0.003,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=rail_steel,
        name="right_rail",
    )
    for idx, (a, b) in enumerate(
        [
            ((-0.096, 0.026, 0.010), (-0.096, 0.032, 0.034)),
            ((-0.096, -0.026, 0.010), (-0.096, -0.032, 0.034)),
            ((0.082, 0.015, 0.010), (0.082, 0.022, 0.036)),
            ((0.082, -0.015, 0.010), (0.082, -0.022, 0.036)),
        ]
    ):
        _add_member(
            saddle,
            a,
            b,
            radius=0.0034,
            material=saddle_shell,
            name=f"rail_mount_{idx}",
        )
    saddle.inertial = Inertial.from_geometry(
        Box((0.276, 0.150, 0.060)),
        mass=0.31,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    model.articulation(
        "dropper_travel",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.060),
    )
    model.articulation(
        "clamp_mount",
        ArticulationType.FIXED,
        parent=inner_tube,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
    )
    model.articulation(
        "saddle_mount",
        ArticulationType.FIXED,
        parent=saddle_clamp,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_tube = object_model.get_part("inner_tube")
    saddle_clamp = object_model.get_part("saddle_clamp")
    saddle = object_model.get_part("saddle")
    dropper = object_model.get_articulation("dropper_travel")

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
        "dropper_joint_axis_is_vertical",
        tuple(dropper.axis) == (0.0, 0.0, 1.0),
        f"axis={dropper.axis}",
    )
    limits = dropper.motion_limits
    ctx.check(
        "dropper_joint_limits_are_60mm",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and abs(limits.upper - 0.060) < 1e-9,
        f"limits={limits}",
    )

    ctx.expect_within(
        inner_tube,
        outer_sleeve,
        axes="xy",
        margin=0.0,
        inner_elem="inner_tube_shell",
        outer_elem="outer_sleeve_shell",
    )
    ctx.expect_overlap(
        inner_tube,
        outer_sleeve,
        axes="xy",
        min_overlap=0.024,
        elem_a="inner_tube_shell",
        elem_b="outer_sleeve_shell",
    )
    ctx.expect_contact(saddle_clamp, inner_tube)
    ctx.expect_contact(saddle, saddle_clamp)

    saddle_aabb = ctx.part_world_aabb(saddle)
    assert saddle_aabb is not None
    saddle_len = saddle_aabb[1][0] - saddle_aabb[0][0]
    saddle_width = saddle_aabb[1][1] - saddle_aabb[0][1]
    ctx.check(
        "saddle_has_standard_scale",
        saddle_len > 0.25 and saddle_width > 0.13,
        f"length={saddle_len:.4f}, width={saddle_width:.4f}",
    )

    saddle_rest = ctx.part_world_position(saddle)
    assert saddle_rest is not None
    with ctx.pose({dropper: 0.060}):
        saddle_extended = ctx.part_world_position(saddle)
        assert saddle_extended is not None
        ctx.expect_within(
            inner_tube,
            outer_sleeve,
            axes="xy",
            margin=0.0,
            inner_elem="inner_tube_shell",
            outer_elem="outer_sleeve_shell",
        )
        ctx.check(
            "saddle_rises_by_60mm",
            abs((saddle_extended[2] - saddle_rest[2]) - 0.060) < 0.002,
            f"travel={saddle_extended[2] - saddle_rest[2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
