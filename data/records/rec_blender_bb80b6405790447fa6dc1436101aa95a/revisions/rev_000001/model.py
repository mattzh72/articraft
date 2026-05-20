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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _annular_shell(
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
        ),
        name,
    )


def _blade_loop(
    x: float,
    *,
    chord: float,
    thickness: float,
    z_center: float,
    sweep_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    return [
        (x, sweep_y - half_chord, z_center - 0.35 * half_thickness),
        (x, sweep_y + 0.48 * half_chord, z_center - half_thickness),
        (x, sweep_y + half_chord, z_center + 0.30 * half_thickness),
        (x, sweep_y - 0.42 * half_chord, z_center + half_thickness),
    ]


def _build_blade_assembly_mesh():
    lower_fin = BoxGeometry((0.050, 0.008, 0.0016)).translate(0.025, 0.0, 0.0092)
    upper_fin = BoxGeometry((0.040, 0.007, 0.0016)).translate(0.020, 0.0, 0.0130)
    mesh = lower_fin
    for angle in (2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        mesh.merge(lower_fin.copy().rotate_z(angle))
    for angle in (math.pi / 3.0, math.pi, 5.0 * math.pi / 3.0):
        mesh.merge(upper_fin.copy().rotate_z(angle))
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vacuum_blender")

    base_charcoal = model.material("base_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_glass = model.material("panel_glass", rgba=(0.12, 0.16, 0.18, 0.55))
    collar_black = model.material("collar_black", rgba=(0.09, 0.10, 0.11, 1.0))
    jar_smoke = model.material("jar_smoke", rgba=(0.70, 0.82, 0.90, 0.33))
    lid_black = model.material("lid_black", rgba=(0.12, 0.13, 0.14, 1.0))
    seal_black = model.material("seal_black", rgba=(0.06, 0.06, 0.07, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.81, 0.84, 1.0))

    base = model.part("base")
    base_shell = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.290, 0.220, 0.030, 0.000, y_shift=-0.003),
                _xy_section(0.274, 0.206, 0.029, 0.052, y_shift=0.002),
                _xy_section(0.252, 0.186, 0.026, 0.108, y_shift=0.010),
            ]
        ),
        "blender_base_shell",
    )
    base.visual(base_shell, material=base_charcoal, name="base_shell")
    base.visual(
        Box((0.166, 0.006, 0.054)),
        origin=Origin(xyz=(0.0, -0.103, 0.040), rpy=(math.radians(18.0), 0.0, 0.0)),
        material=panel_glass,
        name="control_panel",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.010),
        origin=Origin(xyz=(0.0, 0.008, 0.103)),
        material=base_charcoal,
        name="top_plinth",
    )

    collar = model.part("collar")
    collar.visual(
        _annular_shell(
            outer_profile=[
                (0.090, 0.000),
                (0.096, 0.004),
                (0.097, 0.016),
                (0.097, 0.028),
                (0.093, 0.032),
            ],
            inner_profile=[
                (0.082, 0.000),
                (0.086, 0.004),
                (0.087, 0.015),
                (0.087, 0.026),
                (0.084, 0.031),
            ],
            name="blender_collar_shell",
        ),
        material=collar_black,
        name="collar_shell",
    )
    tab_radius = 0.091
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        collar.visual(
            Box((0.032, 0.010, 0.008)),
            origin=Origin(
                xyz=(tab_radius * math.cos(angle), tab_radius * math.sin(angle), 0.026),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=collar_black,
            name=f"bayonet_tab_{index}",
        )

    jar = model.part("jar")
    jar.visual(
        _annular_shell(
            outer_profile=[
                (0.090, 0.000),
                (0.091, 0.006),
                (0.091, 0.028),
                (0.088, 0.040),
            ],
            inner_profile=[
                (0.082, 0.000),
                (0.083, 0.006),
                (0.083, 0.028),
                (0.078, 0.040),
            ],
            name="blender_jar_coupler",
        ),
        material=collar_black,
        name="jar_coupler",
    )
    jar.visual(
        _annular_shell(
            outer_profile=[
                (0.088, 0.040),
                (0.086, 0.072),
                (0.088, 0.090),
                (0.088, 0.224),
                (0.092, 0.252),
            ],
            inner_profile=[
                (0.078, 0.040),
                (0.080, 0.072),
                (0.084, 0.090),
                (0.084, 0.252),
            ],
            name="blender_jar_shell",
        ),
        material=jar_smoke,
        name="jar_shell",
    )
    jar.visual(
        Cylinder(radius=0.082, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=jar_smoke,
        name="jar_floor",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        jar.visual(
            Box((0.032, 0.010, 0.008)),
            origin=Origin(
                xyz=(0.091 * math.cos(angle), 0.091 * math.sin(angle), 0.002),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            material=collar_black,
            name=f"jar_lug_{index}",
        )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.015, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=blade_steel,
        name="blade_hub",
    )
    blade_assembly.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=blade_steel,
        name="blade_spindle",
    )
    blade_assembly.visual(
        mesh_from_geometry(_build_blade_assembly_mesh(), "blender_blade_fins"),
        material=blade_steel,
        name="blade_fins",
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.089, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=lid_black,
        name="lid_flange",
    )
    lid.visual(
        Cylinder(radius=0.073, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=lid_black,
        name="lid_cap",
    )
    lid.visual(
        Cylinder(radius=0.079, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=seal_black,
        name="seal_skirt",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=lid_black,
        name="valve_tower",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=seal_black,
        name="vacuum_port",
    )

    valve_flap = model.part("valve_flap")
    valve_flap.visual(
        Cylinder(radius=0.0045, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_black,
        name="hinge_barrel",
    )
    valve_flap.visual(
        Box((0.040, 0.028, 0.004)),
        origin=Origin(xyz=(0.014, 0.0, 0.002)),
        material=lid_black,
        name="flap_leaf",
    )
    valve_flap.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.026, 0.0, 0.007)),
        material=lid_black,
        name="lift_grip",
    )

    model.articulation(
        "base_to_collar",
        ArticulationType.FIXED,
        parent=base,
        child=collar,
        origin=Origin(xyz=(0.0, 0.008, 0.108)),
    )
    model.articulation(
        "collar_to_jar",
        ArticulationType.FIXED,
        parent=collar,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
    )
    model.articulation(
        "jar_to_blades",
        ArticulationType.REVOLUTE,
        parent=jar,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=40.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "jar_to_lid",
        ArticulationType.FIXED,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.252)),
    )
    model.articulation(
        "lid_to_valve_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=valve_flap,
        origin=Origin(xyz=(-0.012, 0.0, 0.032)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    collar = object_model.get_part("collar")
    jar = object_model.get_part("jar")
    blade_assembly = object_model.get_part("blade_assembly")
    lid = object_model.get_part("lid")
    valve_flap = object_model.get_part("valve_flap")

    blade_joint = object_model.get_articulation("jar_to_blades")
    flap_joint = object_model.get_articulation("lid_to_valve_flap")

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

    ctx.expect_contact(collar, base, name="collar is seated on motor base")
    ctx.expect_contact(jar, collar, name="jar is seated in bayonet collar")
    ctx.expect_contact(lid, jar, name="vacuum lid seals against jar rim")
    ctx.expect_contact(valve_flap, lid, name="valve flap closes onto lid")

    ctx.expect_overlap(jar, collar, axes="xy", min_overlap=0.150, name="jar aligns over collar")
    ctx.expect_overlap(lid, jar, axes="xy", min_overlap=0.160, name="lid covers jar mouth")
    ctx.expect_within(
        blade_assembly,
        jar,
        axes="xy",
        margin=0.0,
        name="blade sweep stays inside jar footprint",
    )
    ctx.expect_gap(
        blade_assembly,
        jar,
        axis="z",
        positive_elem="blade_fins",
        negative_elem="jar_floor",
        min_gap=0.0015,
        max_gap=0.020,
        name="blade fins clear jar floor",
    )
    ctx.expect_contact(
        blade_assembly,
        jar,
        elem_a="blade_hub",
        elem_b="jar_floor",
        name="blade hub mounts to jar floor",
    )

    def _axis_close(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-6 for a, b in zip(axis, target))

    ctx.check(
        "blade articulation uses vertical axis",
        _axis_close(blade_joint.axis, (0.0, 0.0, 1.0)),
        details=f"expected (0, 0, 1), got {blade_joint.axis}",
    )
    ctx.check(
        "valve flap articulation uses transverse hinge axis",
        _axis_close(flap_joint.axis, (0.0, -1.0, 0.0)),
        details=f"expected (0, -1, 0), got {flap_joint.axis}",
    )

    closed_flap_aabb = ctx.part_world_aabb(valve_flap)
    assert closed_flap_aabb is not None
    with ctx.pose({flap_joint: math.radians(72.0)}):
        open_flap_aabb = ctx.part_world_aabb(valve_flap)
        assert open_flap_aabb is not None
        ctx.check(
            "valve flap lifts above closed position",
            open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.012,
            details=f"closed max z {closed_flap_aabb[1][2]:.4f}, open max z {open_flap_aabb[1][2]:.4f}",
        )

    with ctx.pose({blade_joint: math.pi / 3.0}):
        ctx.expect_within(
            blade_assembly,
            jar,
            axes="xy",
            margin=0.0,
            name="blade sweep stays inside jar footprint when rotated",
        )
        ctx.expect_gap(
            blade_assembly,
            jar,
            axis="z",
            positive_elem="blade_fins",
            negative_elem="jar_floor",
            min_gap=0.0015,
            max_gap=0.020,
            name="blade fins clear jar floor when rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
