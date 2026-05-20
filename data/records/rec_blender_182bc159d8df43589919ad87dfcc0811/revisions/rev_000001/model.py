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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_countertop_blender")

    base_body = model.material("base_body", rgba=(0.16, 0.17, 0.19, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    smoked_poly = model.material("smoked_poly", rgba=(0.76, 0.88, 0.96, 0.42))
    lid_rubber = model.material("lid_rubber", rgba=(0.13, 0.14, 0.15, 1.0))
    cap_plastic = model.material("cap_plastic", rgba=(0.24, 0.25, 0.28, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.34, 0.36, 0.39, 1.0))

    base = model.part("base")
    base_shell = section_loft(
        [
            _section(0.24, 0.20, 0.030, 0.0),
            _section(0.228, 0.188, 0.028, 0.058),
            _section(0.186, 0.154, 0.024, 0.095),
        ]
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_housing"),
        material=base_body,
        name="housing_shell",
    )
    base.visual(
        Box((0.114, 0.114, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=accent_gray,
        name="top_pad",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=steel,
        name="drive_coupler",
    )
    base.visual(
        Box((0.130, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, 0.096, 0.040)),
        material=accent_gray,
        name="front_panel",
    )
    for name, x, y in (
        ("foot_fl", 0.078, 0.064),
        ("foot_fr", -0.078, 0.064),
        ("foot_rl", 0.078, -0.064),
        ("foot_rr", -0.078, -0.064),
    ):
        base.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber_black,
            name=name,
        )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.20, 0.110)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    jar = model.part("jar")
    jar_body = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.146, 0.146, 0.020, corner_segments=8),
        [rounded_rect_profile(0.128, 0.128, 0.015, corner_segments=8)],
        height=0.280,
        cap=False,
        center=True,
    )
    jar.visual(
        mesh_from_geometry(jar_body, "jar_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.159)),
        material=smoked_poly,
        name="body_shell",
    )
    jar_rim = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.156, 0.156, 0.022, corner_segments=8),
        [rounded_rect_profile(0.128, 0.128, 0.015, corner_segments=8)],
        height=0.018,
        center=True,
    )
    jar.visual(
        mesh_from_geometry(jar_rim, "jar_top_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.299)),
        material=lid_rubber,
        name="top_rim",
    )
    jar_collar = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.146, 0.146, 0.016, corner_segments=8),
        [rounded_rect_profile(0.062, 0.062, 0.008, corner_segments=8)],
        height=0.028,
        center=True,
    )
    jar.visual(
        mesh_from_geometry(jar_collar, "jar_collar_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=accent_gray,
        name="collar_shell",
    )
    jar.visual(
        Box((0.130, 0.130, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=smoked_poly,
        name="bottom_floor",
    )
    jar.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=accent_gray,
        name="bearing_boss",
    )
    handle_geom = tube_from_spline_points(
        [
            (0.070, 0.028, 0.062),
            (0.094, 0.034, 0.092),
            (0.108, 0.020, 0.156),
            (0.108, -0.010, 0.220),
            (0.094, -0.026, 0.272),
            (0.070, -0.026, 0.292),
        ],
        radius=0.007,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    jar.visual(
        mesh_from_geometry(handle_geom, "jar_handle"),
        material=accent_gray,
        name="handle",
    )
    jar.inertial = Inertial.from_geometry(
        Box((0.170, 0.170, 0.310)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel,
        name="shaft",
    )
    blade_assembly.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=steel,
        name="hub",
    )
    blade_assembly.visual(
        Box((0.040, 0.010, 0.003)),
        origin=Origin(xyz=(0.026, 0.0, 0.023), rpy=(0.0, math.radians(18.0), 0.0)),
        material=steel,
        name="blade_east",
    )
    blade_assembly.visual(
        Box((0.040, 0.010, 0.003)),
        origin=Origin(xyz=(-0.026, 0.0, 0.025), rpy=(0.0, math.radians(-18.0), math.pi)),
        material=steel,
        name="blade_west",
    )
    blade_assembly.visual(
        Box((0.040, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.026, 0.023), rpy=(0.0, math.radians(-16.0), math.pi / 2.0)),
        material=steel,
        name="blade_north",
    )
    blade_assembly.visual(
        Box((0.040, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, -0.026, 0.025), rpy=(0.0, math.radians(16.0), -math.pi / 2.0)),
        material=steel,
        name="blade_south",
    )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.032),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "jar_to_blade",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=40.0),
    )

    lid = model.part("lid")
    lid_flange = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.160, 0.160, 0.022, corner_segments=8),
        [rounded_rect_profile(0.074, 0.074, 0.012, corner_segments=8)],
        height=0.018,
        center=True,
    )
    lid.visual(
        mesh_from_geometry(lid_flange, "lid_flange"),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=lid_rubber,
        name="flange",
    )
    lid_plug = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.126, 0.126, 0.015, corner_segments=8),
        [rounded_rect_profile(0.074, 0.074, 0.012, corner_segments=8)],
        height=0.018,
        center=True,
    )
    lid.visual(
        mesh_from_geometry(lid_plug, "lid_plug"),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=lid_rubber,
        name="plug",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.064),
        origin=Origin(xyz=(0.0, -0.037, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_plastic,
        name="hinge_rod",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.162, 0.162, 0.040)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "jar_to_lid",
        ArticulationType.FIXED,
        parent=jar,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
    )

    fill_cap = model.part("fill_cap")
    fill_cap.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_plastic,
        name="cap_hinge_barrel",
    )
    fill_cap.visual(
        Box((0.082, 0.076, 0.009)),
        origin=Origin(xyz=(0.0, 0.000, -0.0045)),
        material=cap_plastic,
        name="cap_plate",
    )
    fill_cap.visual(
        Box((0.056, 0.056, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, -0.0095)),
        material=cap_plastic,
        name="cap_insert",
    )
    fill_cap.visual(
        Box((0.024, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.040, -0.003)),
        material=cap_plastic,
        name="cap_tab",
    )
    fill_cap.inertial = Inertial.from_geometry(
        Box((0.082, 0.090, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.008, -0.006)),
    )

    model.articulation(
        "lid_to_fill_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=fill_cap,
        origin=Origin(xyz=(0.0, -0.037, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade_assembly = object_model.get_part("blade_assembly")
    lid = object_model.get_part("lid")
    fill_cap = object_model.get_part("fill_cap")

    jar_to_blade = object_model.get_articulation("jar_to_blade")
    lid_to_fill_cap = object_model.get_articulation("lid_to_fill_cap")

    base.get_visual("drive_coupler")
    jar.get_visual("body_shell")
    jar.get_visual("bottom_floor")
    blade_assembly.get_visual("blade_east")
    lid.get_visual("flange")
    fill_cap.get_visual("cap_plate")
    fill_cap.get_visual("cap_tab")

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
    ctx.allow_overlap(
        jar,
        blade_assembly,
        reason="The blade shaft passes through the sealed jar-bottom bearing boss.",
    )
    ctx.allow_overlap(
        lid,
        fill_cap,
        reason="The hinged fill cap is represented with coaxial pin-and-barrel hinge geometry.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(jar, base, name="jar_contacts_base")
    ctx.expect_gap(
        jar,
        base,
        axis="z",
        positive_elem="collar_shell",
        negative_elem="top_pad",
        max_gap=0.001,
        max_penetration=1e-6,
        name="jar_seats_on_base",
    )
    ctx.expect_overlap(jar, base, axes="xy", min_overlap=0.090, name="jar_over_base_pad")

    ctx.expect_contact(lid, jar, name="lid_contacts_jar")
    ctx.expect_overlap(lid, jar, axes="xy", min_overlap=0.110, name="lid_covers_jar_opening")

    ctx.expect_within(
        blade_assembly,
        jar,
        axes="xy",
        margin=0.012,
        name="blade_stays_within_jar_footprint",
    )
    ctx.expect_gap(
        lid,
        blade_assembly,
        axis="z",
        min_gap=0.240,
        positive_elem="plug",
        negative_elem="hub",
        name="blade_sits_well_below_lid",
    )

    with ctx.pose({lid_to_fill_cap: 0.0}):
        ctx.expect_contact(fill_cap, lid, name="fill_cap_closed_contact")
        ctx.expect_within(
            fill_cap,
            lid,
            axes="xy",
            inner_elem="cap_insert",
            outer_elem="plug",
            margin=0.010,
            name="fill_cap_insert_within_lid_opening",
        )

    blade_rest = ctx.part_element_world_aabb(blade_assembly, elem="blade_east")
    assert blade_rest is not None
    with ctx.pose({jar_to_blade: math.pi / 2.0}):
        blade_quarter_turn = ctx.part_element_world_aabb(blade_assembly, elem="blade_east")
        assert blade_quarter_turn is not None
        assert blade_quarter_turn[1][1] > blade_rest[1][1] + 0.015
        ctx.expect_within(
            blade_assembly,
            jar,
            axes="xy",
            margin=0.012,
            name="blade_stays_within_jar_footprint_quarter_turn",
        )

    cap_rest = ctx.part_element_world_aabb(fill_cap, elem="cap_tab")
    assert cap_rest is not None
    cap_hinge_limits = lid_to_fill_cap.motion_limits
    assert cap_hinge_limits is not None
    assert cap_hinge_limits.lower is not None
    assert cap_hinge_limits.upper is not None
    with ctx.pose({lid_to_fill_cap: cap_hinge_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="fill_cap_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="fill_cap_lower_no_floating")
    with ctx.pose({lid_to_fill_cap: cap_hinge_limits.upper}):
        cap_open = ctx.part_element_world_aabb(fill_cap, elem="cap_tab")
        assert cap_open is not None
        assert cap_open[0][2] > cap_rest[0][2] + 0.020
        ctx.fail_if_parts_overlap_in_current_pose(name="fill_cap_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="fill_cap_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
