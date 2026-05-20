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
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mountain_bike_dropper_post")

    satin_alloy = model.material("satin_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    hardcoat_black = model.material("hardcoat_black", rgba=(0.14, 0.15, 0.17, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.25, 0.27, 0.30, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.58, 0.60, 0.64, 1.0))
    saddle_skin = model.material("saddle_skin", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    def _yz_superellipse_section(
        x: float,
        *,
        width: float,
        thickness: float,
        center_z: float,
        exponent: float = 2.25,
        segments: int = 40,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + center_z)
            for y, z in superellipse_profile(width, thickness, exponent=exponent, segments=segments)
        ]

    body_shell = _save_mesh(
        "dropper_outer_body",
        LatheGeometry.from_shell_profiles(
            [
                (0.0149, 0.000),
                (0.0158, 0.012),
                (0.0158, 0.255),
                (0.0176, 0.262),
                (0.0188, 0.269),
                (0.0188, 0.289),
                (0.0172, 0.294),
                (0.0172, 0.302),
            ],
            [
                (0.0139, 0.003),
                (0.0148, 0.012),
                (0.0148, 0.258),
                (0.0155, 0.266),
                (0.0155, 0.289),
                (0.0138, 0.294),
                (0.0138, 0.302),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=10,
        ),
    )
    saddle_shell = _save_mesh(
        "trail_saddle_shell",
        section_loft(
            [
                _yz_superellipse_section(-0.145, width=0.126, thickness=0.026, center_z=0.060),
                _yz_superellipse_section(-0.098, width=0.146, thickness=0.038, center_z=0.056),
                _yz_superellipse_section(-0.040, width=0.140, thickness=0.036, center_z=0.050),
                _yz_superellipse_section(0.025, width=0.118, thickness=0.030, center_z=0.045),
                _yz_superellipse_section(0.082, width=0.076, thickness=0.022, center_z=0.042),
                _yz_superellipse_section(0.122, width=0.038, thickness=0.014, center_z=0.041),
                _yz_superellipse_section(0.146, width=0.016, thickness=0.008, center_z=0.040),
            ]
        ),
    )
    cable_housing = _save_mesh(
        "dropper_cable_housing_curve",
        tube_from_spline_points(
            [
                (-0.039, 0.0, 0.008),
                (-0.058, 0.0, -0.006),
                (-0.073, 0.0, -0.020),
                (-0.083, 0.0, -0.040),
            ],
            radius=0.0038,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    left_rail = _save_mesh(
        "saddle_left_rail",
        tube_from_spline_points(
            [
                (-0.110, 0.023, 0.010),
                (-0.075, 0.023, 0.004),
                (-0.040, 0.023, 0.000),
                (0.042, 0.023, 0.000),
                (0.092, 0.022, 0.007),
                (0.118, 0.018, 0.020),
            ],
            radius=0.0035,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    right_rail = _save_mesh(
        "saddle_right_rail",
        tube_from_spline_points(
            [
                (-0.110, -0.023, 0.010),
                (-0.075, -0.023, 0.004),
                (-0.040, -0.023, 0.000),
                (0.042, -0.023, 0.000),
                (0.092, -0.022, 0.007),
                (0.118, -0.018, 0.020),
            ],
            radius=0.0035,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    outer_body = model.part("outer_body")
    outer_body.visual(body_shell, material=satin_alloy, name="body_shell")
    for axis, sign in (("x", -1.0), ("x", 1.0), ("y", -1.0), ("y", 1.0)):
        if axis == "x":
            xyz = (sign * 0.0162, 0.0, 0.297)
            size = (0.0032, 0.010, 0.010)
        else:
            xyz = (0.0, sign * 0.0162, 0.297)
            size = (0.010, 0.0032, 0.010)
        outer_body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=rubber,
            name=f"wiper_pad_{axis}_{'pos' if sign > 0 else 'neg'}",
        )
    outer_body.visual(
        Cylinder(radius=0.0141, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=gunmetal,
        name="base_bushing",
    )
    outer_body.visual(
        Box((0.012, 0.014, 0.020)),
        origin=Origin(xyz=(-0.024, 0.0, 0.278)),
        material=satin_alloy,
        name="actuator_boss",
    )
    outer_body.visual(
        Cylinder(radius=0.0042, length=0.010),
        origin=Origin(xyz=(-0.035, 0.0, 0.278), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="actuator_cap",
    )
    outer_body.inertial = Inertial.from_geometry(
        Box((0.038, 0.038, 0.302)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
    )

    inner_shaft = model.part("inner_shaft")
    inner_shaft.visual(
        Cylinder(radius=0.0131, length=0.500),
        material=hardcoat_black,
        name="lower_stanchion",
    )
    inner_shaft.visual(
        Cylinder(radius=0.0142, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.243)),
        material=gunmetal,
        name="head_spigot",
    )
    inner_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0135, length=0.500),
        mass=0.46,
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        Box((0.032, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=satin_alloy,
        name="main_head_block",
    )
    for side in (-1.0, 1.0):
        clamp_head.visual(
            Box((0.032, 0.014, 0.009)),
            origin=Origin(xyz=(0.0, side * 0.017, 0.014)),
            material=satin_alloy,
            name=f"lower_saddle_{'left' if side > 0 else 'right'}",
        )
        clamp_head.visual(
            Box((0.028, 0.012, 0.007)),
            origin=Origin(xyz=(0.0, side * 0.017, 0.029)),
            material=satin_alloy,
            name=f"upper_plate_{'left' if side > 0 else 'right'}",
        )
        clamp_head.visual(
            Box((0.008, 0.010, 0.026)),
            origin=Origin(xyz=(0.0, side * 0.014, 0.013)),
            material=satin_alloy,
            name=f"side_web_{'left' if side > 0 else 'right'}",
        )
    for x_pos in (-0.010, 0.010):
        clamp_head.visual(
            Cylinder(radius=0.0025, length=0.060),
            origin=Origin(xyz=(x_pos, 0.0, 0.029), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gunmetal,
            name=f"clamp_bolt_{'rear' if x_pos < 0 else 'front'}",
        )
    clamp_head.visual(
        Box((0.018, 0.014, 0.010)),
        origin=Origin(xyz=(-0.023, 0.0, 0.008)),
        material=satin_alloy,
        name="cable_port_base",
    )
    clamp_head.visual(
        Cylinder(radius=0.0050, length=0.018),
        origin=Origin(xyz=(-0.034, 0.0, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="cable_port",
    )
    clamp_head.visual(cable_housing, material=rubber, name="cable_housing")
    clamp_head.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.070)),
        mass=0.22,
        origin=Origin(xyz=(-0.010, 0.0, 0.016)),
    )

    saddle = model.part("saddle")
    saddle.visual(saddle_shell, material=saddle_skin, name="saddle_shell")
    saddle.visual(left_rail, material=rail_steel, name="left_rail")
    saddle.visual(right_rail, material=rail_steel, name="right_rail")
    for x_pos in (-0.030, 0.030):
        for side in (-1.0, 1.0):
            saddle.visual(
                Cylinder(radius=0.0050, length=0.032),
                origin=Origin(xyz=(x_pos, side * 0.023, 0.019)),
                material=gunmetal,
                name=f"rail_mount_{'rear' if x_pos < 0 else 'front'}_{'left' if side > 0 else 'right'}",
            )
    saddle.visual(
        Box((0.054, 0.096, 0.018)),
        origin=Origin(xyz=(-0.072, 0.0, 0.046)),
        material=saddle_skin,
        name="rear_platform_pad",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.292, 0.146, 0.068)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
    )

    model.articulation(
        "dropper_travel",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=inner_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.302)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.28,
            lower=0.0,
            upper=0.170,
        ),
    )
    model.articulation(
        "shaft_to_clamp",
        ArticulationType.FIXED,
        parent=inner_shaft,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
    )
    model.articulation(
        "clamp_to_saddle",
        ArticulationType.FIXED,
        parent=clamp_head,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_body = object_model.get_part("outer_body")
    inner_shaft = object_model.get_part("inner_shaft")
    clamp_head = object_model.get_part("clamp_head")
    saddle = object_model.get_part("saddle")
    dropper_travel = object_model.get_articulation("dropper_travel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.001)
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
        "dropper axis is vertical",
        tuple(dropper_travel.axis) == (0.0, 0.0, 1.0),
        f"expected vertical prismatic axis, got {dropper_travel.axis}",
    )
    limits = dropper_travel.motion_limits
    ctx.check(
        "dropper travel range is realistic",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and 0.15 <= limits.upper <= 0.18,
        f"unexpected travel limits: {limits}",
    )

    ctx.expect_within(
        inner_shaft,
        outer_body,
        axes="xy",
        inner_elem="lower_stanchion",
        outer_elem="body_shell",
        margin=0.0,
    )
    ctx.expect_contact(inner_shaft, clamp_head)
    ctx.expect_contact(clamp_head, saddle)
    ctx.expect_gap(saddle, outer_body, axis="z", min_gap=0.18)

    rest_pos = ctx.part_world_position(inner_shaft)
    rest_saddle = ctx.part_world_position(saddle)
    assert rest_pos is not None
    assert rest_saddle is not None

    with ctx.pose({dropper_travel: 0.170}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fail_if_parts_overlap_at_full_extension")
        ctx.expect_within(
            inner_shaft,
            outer_body,
            axes="xy",
            inner_elem="lower_stanchion",
            outer_elem="body_shell",
            margin=0.0,
            name="stanchion_stays_centered_at_full_extension",
        )
        ctx.expect_contact(inner_shaft, clamp_head, name="clamp_head_remains_seated_on_shaft")
        ctx.expect_contact(clamp_head, saddle, name="saddle_remains_clamped_to_head")
        extended_pos = ctx.part_world_position(inner_shaft)
        extended_saddle = ctx.part_world_position(saddle)
        assert extended_pos is not None
        assert extended_saddle is not None
        ctx.check(
            "inner shaft rises by full travel",
            extended_pos[2] > rest_pos[2] + 0.165,
            f"inner shaft rest z={rest_pos[2]:.4f}, extended z={extended_pos[2]:.4f}",
        )
        ctx.check(
            "saddle rises with dropper motion",
            extended_saddle[2] > rest_saddle[2] + 0.165,
            f"saddle rest z={rest_saddle[2]:.4f}, extended z={extended_saddle[2]:.4f}",
        )

    # Keep pose-specific checks lean.
    # Do not add blanket lower/upper pose sweeps or
    # `fail_if_parts_overlap_in_sampled_poses(...)` by default.
    # Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is
    # genuinely uncertain or mechanically important.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
