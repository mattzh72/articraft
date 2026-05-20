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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _profile_at_z(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


def _annulus_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius),
            [_circle_profile(inner_radius)],
            height,
            center=False,
        ),
        name,
    )


def _rounded_rect_ring_mesh(
    outer_size: tuple[float, float],
    outer_radius: float,
    hole_profile: list[tuple[float, float]],
    height: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_size[0], outer_size[1], outer_radius),
            [hole_profile],
            height,
            center=False,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cold_press_smoothie_blender")

    base_dark = model.material("base_dark", rgba=(0.13, 0.14, 0.16, 1.0))
    base_trim = model.material("base_trim", rgba=(0.07, 0.08, 0.09, 1.0))
    polycarbonate = model.material("polycarbonate", rgba=(0.80, 0.91, 0.97, 0.34))
    smoky_clear = model.material("smoky_clear", rgba=(0.35, 0.40, 0.44, 0.85))
    stainless = model.material("stainless", rgba=(0.82, 0.84, 0.87, 1.0))
    cap_dark = model.material("cap_dark", rgba=(0.19, 0.21, 0.23, 1.0))
    control_accent = model.material("control_accent", rgba=(0.84, 0.62, 0.18, 1.0))

    base = model.part("base")
    jar = model.part("jar")
    blade_assembly = model.part("blade_assembly")
    fill_cap = model.part("fill_cap")

    base_body_sections = [
        _profile_at_z(rounded_rect_profile(0.240, 0.180, 0.024), 0.0),
        _profile_at_z(rounded_rect_profile(0.214, 0.158, 0.021), 0.064),
        _profile_at_z(rounded_rect_profile(0.182, 0.130, 0.016), 0.086),
    ]
    base.visual(
        mesh_from_geometry(section_loft(base_body_sections), "base_body"),
        material=base_dark,
        name="base_body",
    )
    base.visual(
        _annulus_mesh(0.067, 0.050, 0.014, "base_coupling"),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=base_trim,
        name="base_coupling",
    )
    base.visual(
        Cylinder(radius=0.029, length=0.018),
        origin=Origin(xyz=(0.0, -0.090, 0.041), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_accent,
        name="control_dial",
    )

    jar.visual(
        _annulus_mesh(0.064, 0.050, 0.028, "jar_collar"),
        material=base_trim,
        name="jar_collar",
    )
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radius = 0.067
        jar.visual(
            Box((0.018, 0.012, 0.008)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.012),
                rpy=(0.0, 0.0, angle),
            ),
            material=base_trim,
            name=f"bayonet_lug_{idx}",
        )

    jar.visual(
        _rounded_rect_ring_mesh(
            (0.132, 0.120),
            0.018,
            _circle_profile(0.012),
            0.010,
            "jar_bottom",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=base_trim,
        name="jar_bottom",
    )
    jar.visual(
        _rounded_rect_ring_mesh(
            (0.142, 0.130),
            0.020,
            rounded_rect_profile(0.126, 0.114, 0.015),
            0.190,
            "jar_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=polycarbonate,
        name="jar_shell",
    )
    jar.visual(
        _rounded_rect_ring_mesh(
            (0.146, 0.134),
            0.020,
            _circle_profile(0.046),
            0.008,
            "jar_shoulder",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
        material=polycarbonate,
        name="jar_shoulder",
    )
    jar.visual(
        _annulus_mesh(0.060, 0.046, 0.024, "neck_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.236)),
        material=polycarbonate,
        name="neck_ring",
    )
    jar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.066, 0.0, 0.212),
                    (0.108, 0.0, 0.198),
                    (0.118, 0.0, 0.150),
                    (0.112, 0.0, 0.096),
                    (0.070, 0.0, 0.060),
                ],
                radius=0.008,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "jar_handle",
        ),
        material=smoky_clear,
        name="jar_handle",
    )

    blade_assembly.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=stainless,
        name="blade_spindle",
    )
    blade_assembly.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=stainless,
        name="blade_hub",
    )
    blade_specs = (
        (0.0, 0.008, 0.22),
        (math.pi / 3.0, 0.010, -0.28),
        (2.0 * math.pi / 3.0, 0.016, 0.34),
    )
    for idx, (yaw, z, pitch) in enumerate(blade_specs):
        blade_assembly.visual(
            Box((0.066, 0.014, 0.0026)),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(0.0, pitch, yaw)),
            material=stainless,
            name=f"blade_bar_{idx}",
        )

    fill_cap.visual(
        Cylinder(radius=0.062, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=cap_dark,
        name="cap_top",
    )
    fill_cap.visual(
        _annulus_mesh(0.079, 0.0615, 0.024, "cap_skirt"),
        material=cap_dark,
        name="cap_skirt",
    )
    for idx, angle in enumerate(i * math.pi / 4.0 for i in range(8)):
        radius = 0.074
        fill_cap.visual(
            Box((0.015, 0.010, 0.022)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.012),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_dark,
            name=f"cap_rib_{idx}",
        )

    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )
    model.articulation(
        "jar_to_blades",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=35.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "jar_to_cap",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=fill_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade_assembly = object_model.get_part("blade_assembly")
    fill_cap = object_model.get_part("fill_cap")
    base_to_jar = object_model.get_articulation("base_to_jar")
    jar_to_blades = object_model.get_articulation("jar_to_blades")
    jar_to_cap = object_model.get_articulation("jar_to_cap")

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
        "all prompt-critical parts exist",
        all(part is not None for part in (base, jar, blade_assembly, fill_cap)),
        details="Expected base, jar, blade assembly, and fill cap parts.",
    )
    ctx.check(
        "jar is mounted by a fixed bayonet seat",
        base_to_jar.articulation_type == ArticulationType.FIXED,
        details=f"base_to_jar type={base_to_jar.articulation_type}",
    )
    ctx.check(
        "blade assembly spins on a vertical revolute axis",
        jar_to_blades.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in jar_to_blades.axis) == (0.0, 0.0, 1.0),
        details=f"jar_to_blades type={jar_to_blades.articulation_type}, axis={jar_to_blades.axis}",
    )
    ctx.check(
        "fill cap twists on a vertical sealing axis",
        jar_to_cap.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in jar_to_cap.axis) == (0.0, 0.0, 1.0),
        details=f"jar_to_cap type={jar_to_cap.articulation_type}, axis={jar_to_cap.axis}",
    )
    ctx.check(
        "blade joint allows a broad spin range",
        jar_to_blades.motion_limits is not None
        and jar_to_blades.motion_limits.lower is not None
        and jar_to_blades.motion_limits.upper is not None
        and jar_to_blades.motion_limits.lower <= -3.0
        and jar_to_blades.motion_limits.upper >= 3.0,
        details=f"blade limits={jar_to_blades.motion_limits}",
    )
    ctx.check(
        "cap screw range is realistic",
        jar_to_cap.motion_limits is not None
        and jar_to_cap.motion_limits.lower == 0.0
        and jar_to_cap.motion_limits.upper is not None
        and 1.0 <= jar_to_cap.motion_limits.upper <= 3.2,
        details=f"cap limits={jar_to_cap.motion_limits}",
    )

    ctx.expect_origin_gap(
        jar,
        base,
        axis="z",
        min_gap=0.099,
        max_gap=0.101,
        name="jar frame sits on top of the base coupling plane",
    )
    ctx.expect_origin_gap(
        blade_assembly,
        jar,
        axis="z",
        min_gap=0.039,
        max_gap=0.041,
        name="blade spindle sits at the jar floor axis",
    )
    ctx.expect_contact(
        jar,
        base,
        elem_a="jar_collar",
        elem_b="base_coupling",
        name="jar collar seats on the base coupling",
    )
    ctx.expect_overlap(
        jar,
        base,
        axes="xy",
        elem_a="jar_collar",
        elem_b="base_coupling",
        min_overlap=0.100,
        name="jar collar overlaps the bayonet coupling footprint",
    )
    ctx.expect_within(
        blade_assembly,
        jar,
        axes="xy",
        outer_elem="jar_shell",
        margin=0.0,
        name="blade assembly stays within the jar body footprint",
    )

    with ctx.pose({jar_to_cap: 0.0}):
        ctx.expect_contact(
            fill_cap,
            jar,
            elem_a="cap_top",
            elem_b="neck_ring",
            name="fill cap seals against the jar neck",
        )
        ctx.expect_overlap(
            fill_cap,
            jar,
            axes="xy",
            elem_a="cap_top",
            elem_b="neck_ring",
            min_overlap=0.090,
            name="fill cap covers the wide mouth opening",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
