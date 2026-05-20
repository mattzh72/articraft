from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_toy_model_plane")

    body_paint = model.material("body_paint", rgba=(0.90, 0.92, 0.94, 1.0))
    accent_aluminum = model.material("accent_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))
    stand_metal = model.material("stand_metal", rgba=(0.43, 0.46, 0.50, 1.0))
    smoke_polymer = model.material("smoke_polymer", rgba=(0.18, 0.21, 0.25, 0.72))
    matte_polymer = model.material("matte_polymer", rgba=(0.10, 0.11, 0.12, 1.0))
    elastomer = model.material("elastomer", rgba=(0.05, 0.05, 0.06, 1.0))

    def yz_superellipse_section(
        x: float,
        width: float,
        height: float,
        z_center: float,
        *,
        exponent: float = 2.7,
        segments: int = 40,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z_center + z)
            for y, z in superellipse_profile(
                width,
                height,
                exponent=exponent,
                segments=segments,
            )
        ]

    def xz_section(
        y: float,
        chord: float,
        thickness: float,
        *,
        x_center: float = 0.0,
        z_center: float = 0.0,
        radius_scale: float = 0.42,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_center + x, y, z_center + z)
            for x, z in rounded_rect_profile(
                chord,
                thickness,
                radius=max(0.0008, thickness * radius_scale),
                corner_segments=6,
            )
        ]

    def xy_section(
        z: float,
        chord: float,
        thickness_y: float,
        *,
        x_center: float = 0.0,
        y_center: float = 0.0,
        radius_scale: float = 0.42,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_center + x, y_center + y, z)
            for x, y in rounded_rect_profile(
                chord,
                thickness_y,
                radius=max(0.0008, thickness_y * radius_scale),
                corner_segments=6,
            )
        ]

    stand_base = model.part("stand_base")
    base_profile = rounded_rect_profile(0.24, 0.14, 0.028, corner_segments=10)
    base_shell = mesh_from_geometry(
        ExtrudeGeometry(base_profile, 0.018),
        "stand_base_shell",
    )
    stand_base.visual(
        base_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=stand_metal,
        name="base_shell",
    )
    for name, x_pos, y_pos in (
        ("foot_fl", 0.070, 0.038),
        ("foot_fr", 0.070, -0.038),
        ("foot_rl", -0.070, 0.038),
        ("foot_rr", -0.070, -0.038),
    ):
        stand_base.visual(
            Box((0.032, 0.020, 0.004)),
            origin=Origin(xyz=(x_pos, y_pos, 0.002)),
            material=elastomer,
            name=name,
        )
    stand_base.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(-0.024, 0.0, 0.019)),
        material=stand_metal,
        name="mast_pedestal",
    )
    stand_base.inertial = Inertial.from_geometry(
        Box((0.24, 0.14, 0.026)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    stand_mast = model.part("stand_mast")
    mast_geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.006),
            (-0.006, 0.0, -0.024),
            (-0.014, 0.0, -0.056),
            (-0.022, 0.0, -0.084),
            (-0.024, 0.0, -0.102),
        ],
        radius=0.0048,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    stand_mast.visual(
        mesh_from_geometry(mast_geom, "stand_mast_tube"),
        material=stand_metal,
        name="mast_tube",
    )
    stand_mast.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(-0.024, 0.0, -0.102)),
        material=stand_metal,
        name="base_collar",
    )
    stand_mast.visual(
        Box((0.014, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.013, -0.006)),
        material=stand_metal,
        name="fork_left",
    )
    stand_mast.visual(
        Box((0.014, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -0.013, -0.006)),
        material=stand_metal,
        name="fork_right",
    )
    stand_mast.visual(
        Box((0.014, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=stand_metal,
        name="fork_bridge",
    )
    stand_mast.inertial = Inertial.from_geometry(
        Box((0.05, 0.03, 0.12)),
        mass=0.18,
        origin=Origin(xyz=(-0.012, 0.0, -0.060)),
    )

    stand_head = model.part("stand_head")
    stand_head.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stand_metal,
        name="pivot_barrel",
    )
    stand_head.visual(
        Box((0.028, 0.012, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, -0.009)),
        material=stand_metal,
        name="cradle_block",
    )
    stand_head.visual(
        Box((0.024, 0.018, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, -0.002)),
        material=elastomer,
        name="saddle_pad",
    )
    stand_head.inertial = Inertial.from_geometry(
        Box((0.03, 0.022, 0.018)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    body = model.part("body")
    fuselage_sections = [
        yz_superellipse_section(-0.190, 0.006, 0.016, 0.029, exponent=2.2),
        yz_superellipse_section(-0.154, 0.018, 0.032, 0.030, exponent=2.4),
        yz_superellipse_section(-0.094, 0.034, 0.046, 0.031, exponent=2.7),
        yz_superellipse_section(-0.015, 0.054, 0.056, 0.031, exponent=2.9),
        yz_superellipse_section(0.050, 0.048, 0.044, 0.030, exponent=2.8),
        yz_superellipse_section(0.110, 0.040, 0.040, 0.030, exponent=2.6),
        yz_superellipse_section(0.148, 0.024, 0.032, 0.029, exponent=2.3),
        yz_superellipse_section(0.158, 0.014, 0.022, 0.028, exponent=2.1),
    ]
    body.visual(
        mesh_from_geometry(section_loft(fuselage_sections), "fuselage_shell"),
        material=body_paint,
        name="fuselage_shell",
    )
    body.visual(
        Box((0.024, 0.018, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, 0.005)),
        material=accent_aluminum,
        name="mount_boss",
    )
    body.visual(
        Box((0.072, 0.036, 0.004)),
        origin=Origin(xyz=(0.046, 0.0, 0.058)),
        material=accent_aluminum,
        name="canopy_deck",
    )
    body.visual(
        Box((0.014, 0.012, 0.012)),
        origin=Origin(xyz=(-0.006, -0.029, 0.033)),
        material=body_paint,
        name="left_wing_pad",
    )
    body.visual(
        Box((0.014, 0.012, 0.012)),
        origin=Origin(xyz=(-0.006, 0.029, 0.033)),
        material=body_paint,
        name="right_wing_pad",
    )
    body.visual(
        Box((0.012, 0.012, 0.012)),
        origin=Origin(xyz=(-0.149, -0.015, 0.034)),
        material=body_paint,
        name="left_tail_pad",
    )
    body.visual(
        Box((0.012, 0.012, 0.012)),
        origin=Origin(xyz=(-0.149, 0.015, 0.034)),
        material=body_paint,
        name="right_tail_pad",
    )
    body.visual(
        Box((0.018, 0.012, 0.014)),
        origin=Origin(xyz=(-0.160, 0.0, 0.049)),
        material=body_paint,
        name="fin_pad",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.156, 0.0, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_aluminum,
        name="nose_face",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.35, 0.065, 0.075)),
        mass=0.55,
        origin=Origin(xyz=(-0.010, 0.0, 0.032)),
    )

    left_wing = model.part("left_wing")
    left_wing.visual(
        Box((0.014, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        material=body_paint,
        name="root_tongue",
    )
    left_wing.visual(
        mesh_from_geometry(
            section_loft(
                [
                    xz_section(-0.004, 0.108, 0.012, x_center=0.002, z_center=0.0),
                    xz_section(-0.096, 0.080, 0.009, x_center=-0.008, z_center=0.008),
                    xz_section(-0.200, 0.048, 0.006, x_center=-0.028, z_center=0.016),
                ]
            ),
            "left_wing_shell",
        ),
        material=body_paint,
        name="wing_shell",
    )
    left_wing.inertial = Inertial.from_geometry(
        Box((0.12, 0.20, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(-0.015, -0.100, 0.008)),
    )

    right_wing = model.part("right_wing")
    right_wing.visual(
        Box((0.014, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=body_paint,
        name="root_tongue",
    )
    right_wing.visual(
        mesh_from_geometry(
            section_loft(
                [
                    xz_section(0.004, 0.108, 0.012, x_center=0.002, z_center=0.0),
                    xz_section(0.096, 0.080, 0.009, x_center=-0.008, z_center=0.008),
                    xz_section(0.200, 0.048, 0.006, x_center=-0.028, z_center=0.016),
                ]
            ),
            "right_wing_shell",
        ),
        material=body_paint,
        name="wing_shell",
    )
    right_wing.inertial = Inertial.from_geometry(
        Box((0.12, 0.20, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(-0.015, 0.100, 0.008)),
    )

    left_stabilizer = model.part("left_stabilizer")
    left_stabilizer.visual(
        Box((0.010, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        material=body_paint,
        name="root_tongue",
    )
    left_stabilizer.visual(
        mesh_from_geometry(
            section_loft(
                [
                    xz_section(-0.004, 0.056, 0.008, x_center=-0.004, z_center=0.0),
                    xz_section(-0.040, 0.040, 0.006, x_center=-0.012, z_center=0.003),
                    xz_section(-0.076, 0.024, 0.004, x_center=-0.020, z_center=0.006),
                ]
            ),
            "left_stabilizer_shell",
        ),
        material=body_paint,
        name="stabilizer_shell",
    )
    left_stabilizer.inertial = Inertial.from_geometry(
        Box((0.07, 0.08, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(-0.012, -0.040, 0.003)),
    )

    right_stabilizer = model.part("right_stabilizer")
    right_stabilizer.visual(
        Box((0.010, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=body_paint,
        name="root_tongue",
    )
    right_stabilizer.visual(
        mesh_from_geometry(
            section_loft(
                [
                    xz_section(0.004, 0.056, 0.008, x_center=-0.004, z_center=0.0),
                    xz_section(0.040, 0.040, 0.006, x_center=-0.012, z_center=0.003),
                    xz_section(0.076, 0.024, 0.004, x_center=-0.020, z_center=0.006),
                ]
            ),
            "right_stabilizer_shell",
        ),
        material=body_paint,
        name="stabilizer_shell",
    )
    right_stabilizer.inertial = Inertial.from_geometry(
        Box((0.07, 0.08, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(-0.012, 0.040, 0.003)),
    )

    fin = model.part("fin")
    fin.visual(
        Box((0.016, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=body_paint,
        name="root_tab",
    )
    fin.visual(
        mesh_from_geometry(
            section_loft(
                [
                    xy_section(0.008, 0.050, 0.006, x_center=-0.002),
                    xy_section(0.044, 0.032, 0.004, x_center=-0.014),
                    xy_section(0.078, 0.010, 0.002, x_center=-0.028),
                ]
            ),
            "fin_shell",
        ),
        material=body_paint,
        name="fin_shell",
    )
    fin.inertial = Inertial.from_geometry(
        Box((0.06, 0.012, 0.09)),
        mass=0.035,
        origin=Origin(xyz=(-0.015, 0.0, 0.040)),
    )

    canopy = model.part("canopy")
    canopy.visual(
        Box((0.072, 0.032, 0.010)),
        origin=Origin(xyz=(0.048, 0.0, 0.057)),
        material=matte_polymer,
        name="canopy_skirt",
    )
    canopy_sections = [
        yz_superellipse_section(0.018, 0.018, 0.012, 0.062, exponent=3.0, segments=28),
        yz_superellipse_section(0.040, 0.030, 0.022, 0.066, exponent=3.2, segments=28),
        yz_superellipse_section(0.066, 0.030, 0.024, 0.066, exponent=3.2, segments=28),
        yz_superellipse_section(0.086, 0.018, 0.012, 0.062, exponent=3.0, segments=28),
    ]
    canopy.visual(
        mesh_from_geometry(section_loft(canopy_sections), "canopy_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=smoke_polymer,
        name="canopy_shell",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.08, 0.04, 0.03)),
        mass=0.02,
        origin=Origin(xyz=(0.050, 0.0, 0.064)),
    )

    propeller = model.part("propeller")
    blade_profile = [
        (-0.004, 0.000),
        (-0.011, 0.014),
        (-0.010, 0.042),
        (-0.006, 0.068),
        (-0.003, 0.088),
        (0.003, 0.088),
        (0.006, 0.068),
        (0.010, 0.042),
        (0.011, 0.014),
        (0.004, 0.000),
    ]
    blade_geom = ExtrudeGeometry(blade_profile, 0.0026).rotate_y(math.pi / 2.0)
    blade_a = blade_geom.copy().rotate_x(math.radians(7.0))
    blade_b = blade_geom.copy().rotate_z(math.pi).rotate_x(math.radians(7.0))
    spinner_geom = (
        ConeGeometry(radius=0.016, height=0.028, radial_segments=28, closed=True)
        .rotate_y(math.pi / 2.0)
        .translate(0.017, 0.0, 0.0)
    )
    hub_geom = CylinderGeometry(radius=0.0075, height=0.008, radial_segments=24).rotate_y(
        math.pi / 2.0
    )
    backplate_geom = CylinderGeometry(radius=0.0105, height=0.003, radial_segments=24).rotate_y(
        math.pi / 2.0
    ).translate(0.0015, 0.0, 0.0)
    prop_geom = spinner_geom.copy().merge(hub_geom).merge(backplate_geom).merge(blade_a).merge(blade_b)
    propeller.visual(
        mesh_from_geometry(prop_geom, "propeller_spinner"),
        material=accent_aluminum,
        name="spinner_and_blades",
    )
    propeller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.05, length=0.04),
        mass=0.03,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.FIXED,
        parent=stand_base,
        child=stand_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
    )

    model.articulation(
        "mast_to_head",
        ArticulationType.REVOLUTE,
        parent=stand_mast,
        child=stand_head,
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=1.0,
            lower=math.radians(-16.0),
            upper=math.radians(20.0),
        ),
    )

    model.articulation(
        "head_to_body",
        ArticulationType.FIXED,
        parent=stand_head,
        child=body,
        origin=Origin(),
    )

    model.articulation(
        "body_to_left_wing",
        ArticulationType.FIXED,
        parent=body,
        child=left_wing,
        origin=Origin(xyz=(-0.006, -0.034, 0.033)),
    )
    model.articulation(
        "body_to_right_wing",
        ArticulationType.FIXED,
        parent=body,
        child=right_wing,
        origin=Origin(xyz=(-0.006, 0.034, 0.033)),
    )
    model.articulation(
        "body_to_left_stabilizer",
        ArticulationType.FIXED,
        parent=body,
        child=left_stabilizer,
        origin=Origin(xyz=(-0.149, -0.017, 0.034)),
    )
    model.articulation(
        "body_to_right_stabilizer",
        ArticulationType.FIXED,
        parent=body,
        child=right_stabilizer,
        origin=Origin(xyz=(-0.149, 0.017, 0.034)),
    )
    model.articulation(
        "body_to_fin",
        ArticulationType.FIXED,
        parent=body,
        child=fin,
        origin=Origin(xyz=(-0.160, 0.0, 0.056)),
    )
    model.articulation(
        "body_to_canopy",
        ArticulationType.FIXED,
        parent=body,
        child=canopy,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "body_to_propeller",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=propeller,
        origin=Origin(xyz=(0.158, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand_base = object_model.get_part("stand_base")
    stand_mast = object_model.get_part("stand_mast")
    stand_head = object_model.get_part("stand_head")
    body = object_model.get_part("body")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    left_stabilizer = object_model.get_part("left_stabilizer")
    right_stabilizer = object_model.get_part("right_stabilizer")
    fin = object_model.get_part("fin")
    canopy = object_model.get_part("canopy")
    propeller = object_model.get_part("propeller")
    stand_tilt = object_model.get_articulation("mast_to_head")

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

    ctx.expect_contact(
        stand_mast,
        stand_base,
        elem_a="base_collar",
        elem_b="mast_pedestal",
        name="mast is seated on base pedestal",
    )
    ctx.expect_contact(
        stand_head,
        stand_mast,
        elem_a="pivot_barrel",
        elem_b="fork_left",
        name="head barrel touches left fork cheek",
    )
    ctx.expect_contact(
        stand_head,
        stand_mast,
        elem_a="pivot_barrel",
        elem_b="fork_right",
        name="head barrel touches right fork cheek",
    )
    ctx.expect_contact(
        body,
        stand_head,
        elem_a="mount_boss",
        elem_b="saddle_pad",
        name="body is mounted on stand saddle",
    )
    ctx.expect_contact(
        canopy,
        body,
        elem_a="canopy_skirt",
        elem_b="canopy_deck",
        name="canopy is seated on cockpit deck",
    )
    ctx.expect_contact(
        left_wing,
        body,
        elem_a="root_tongue",
        elem_b="left_wing_pad",
        name="left wing root is mounted",
    )
    ctx.expect_contact(
        right_wing,
        body,
        elem_a="root_tongue",
        elem_b="right_wing_pad",
        name="right wing root is mounted",
    )
    ctx.expect_contact(
        left_stabilizer,
        body,
        elem_a="root_tongue",
        elem_b="left_tail_pad",
        name="left stabilizer root is mounted",
    )
    ctx.expect_contact(
        right_stabilizer,
        body,
        elem_a="root_tongue",
        elem_b="right_tail_pad",
        name="right stabilizer root is mounted",
    )
    ctx.expect_contact(
        fin,
        body,
        elem_a="root_tab",
        elem_b="fin_pad",
        name="fin root is mounted",
    )
    ctx.expect_contact(
        propeller,
        body,
        elem_a="spinner_and_blades",
        elem_b="nose_face",
        name="propeller hub is mounted to nose face",
    )

    with ctx.pose({stand_tilt: 0.0}):
        neutral_prop_z = ctx.part_world_position(propeller)[2]
    with ctx.pose({stand_tilt: min(stand_tilt.motion_limits.upper, math.radians(16.0))}):
        pitched_prop_z = ctx.part_world_position(propeller)[2]
        ctx.expect_gap(
            left_wing,
            stand_base,
            axis="z",
            min_gap=0.090,
            name="left wing clears base at nose-up tilt",
        )
        ctx.expect_gap(
            right_wing,
            stand_base,
            axis="z",
            min_gap=0.090,
            name="right wing clears base at nose-up tilt",
        )
    ctx.check(
        "positive stand tilt raises nose",
        pitched_prop_z > neutral_prop_z + 0.010,
        details=f"neutral z={neutral_prop_z:.4f}, pitched z={pitched_prop_z:.4f}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
