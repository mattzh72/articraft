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
    CylinderGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    def merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
        merged = MeshGeometry()
        for geometry in geometries:
            merged.merge(geometry)
        return merged

    def bridge_loops(
        geometry: MeshGeometry,
        outer_loop: list[int],
        inner_loop: list[int],
        *,
        reverse: bool = False,
    ) -> None:
        segments = len(outer_loop)
        for index in range(segments):
            next_index = (index + 1) % segments
            a = outer_loop[index]
            b = outer_loop[next_index]
            c = inner_loop[next_index]
            d = inner_loop[index]
            if reverse:
                geometry.add_face(a, c, b)
                geometry.add_face(a, d, c)
            else:
                geometry.add_face(a, b, c)
                geometry.add_face(a, c, d)

    def revolve_shell(
        outer_profile: list[tuple[float, float]],
        inner_profile: list[tuple[float, float]],
        *,
        segments: int = 64,
    ) -> MeshGeometry:
        shell = MeshGeometry()

        def add_profile_rows(profile: list[tuple[float, float]]) -> list[list[int]]:
            rows: list[list[int]] = []
            for radius, z_pos in profile:
                row: list[int] = []
                for segment in range(segments):
                    angle = (math.tau * segment) / segments
                    row.append(
                        shell.add_vertex(
                            radius * math.cos(angle),
                            radius * math.sin(angle),
                            z_pos,
                        )
                    )
                rows.append(row)
            return rows

        outer_rows = add_profile_rows(outer_profile)
        inner_rows = add_profile_rows(inner_profile)

        for row_index in range(len(outer_rows) - 1):
            bridge_loops(shell, outer_rows[row_index], outer_rows[row_index + 1])
        for row_index in range(len(inner_rows) - 1):
            bridge_loops(
                shell,
                inner_rows[row_index + 1],
                inner_rows[row_index],
                reverse=True,
            )

        bridge_loops(shell, outer_rows[0], inner_rows[0], reverse=True)
        bridge_loops(shell, outer_rows[-1], inner_rows[-1])
        return shell

    def blade_loop(
        x_pos: float,
        width: float,
        thickness: float,
        *,
        camber: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        half_width = width * 0.5
        half_thickness = thickness * 0.5
        return [
            (x_pos, -1.00 * half_width, 0.00),
            (x_pos, -0.72 * half_width, 0.78 * half_thickness),
            (x_pos, -0.18 * half_width, 1.00 * half_thickness + camber),
            (x_pos, 0.56 * half_width, 0.54 * half_thickness),
            (x_pos, 1.00 * half_width, 0.00),
            (x_pos, 0.52 * half_width, -0.42 * half_thickness),
            (x_pos, -0.20 * half_width, -0.74 * half_thickness),
            (x_pos, -0.82 * half_width, -0.28 * half_thickness),
        ]

    def build_blade_geometry() -> MeshGeometry:
        return section_loft(
            [
                blade_loop(0.028, 0.138, 0.010, camber=0.0008),
                blade_loop(0.190, 0.128, 0.009, camber=0.0007),
                blade_loop(0.355, 0.106, 0.007, camber=0.0005),
                blade_loop(0.520, 0.082, 0.005, camber=0.0003),
            ]
        )

    def build_canopy_geometry() -> MeshGeometry:
        return LatheGeometry(
            [
                (0.0, 0.000),
                (0.055, 0.000),
                (0.074, -0.018),
                (0.082, -0.052),
                (0.062, -0.084),
                (0.028, -0.103),
                (0.018, -0.108),
                (0.0, -0.108),
            ],
            segments=64,
        )

    def build_rotor_mesh() -> MeshGeometry:
        top_coupler = CylinderGeometry(radius=0.030, height=0.055, radial_segments=40).translate(
            0.0,
            0.0,
            -0.0275,
        )
        top_cap = CylinderGeometry(radius=0.114, height=0.024, radial_segments=56).translate(
            0.0,
            0.0,
            -0.047,
        )
        motor_drum = CylinderGeometry(radius=0.107, height=0.110, radial_segments=64).translate(
            0.0,
            0.0,
            -0.107,
        )
        lower_band = CylinderGeometry(radius=0.116, height=0.014, radial_segments=56).translate(
            0.0,
            0.0,
            -0.160,
        )
        switch_housing = CylinderGeometry(
            radius=0.084,
            height=0.066,
            radial_segments=56,
        ).translate(0.0, 0.0, -0.188)
        light_ring = CylinderGeometry(radius=0.102, height=0.022, radial_segments=56).translate(
            0.0,
            0.0,
            -0.220,
        )
        stem = CylinderGeometry(radius=0.009, height=0.062, radial_segments=28).translate(
            0.0,
            0.0,
            -0.262,
        )
        socket = CylinderGeometry(radius=0.014, height=0.026, radial_segments=28).translate(
            0.0,
            0.0,
            -0.306,
        )
        return merge_geometries(
            top_coupler,
            top_cap,
            motor_drum,
            lower_band,
            switch_housing,
            light_ring,
            stem,
            socket,
        )

    canopy_mesh = mesh_from_geometry(build_canopy_geometry(), "ceiling_fan_canopy")
    rotor_mesh = mesh_from_geometry(build_rotor_mesh(), "ceiling_fan_rotor")
    blade_mesh = mesh_from_geometry(build_blade_geometry(), "ceiling_fan_blade")
    diffuser_mesh = mesh_from_geometry(
        revolve_shell(
            [
                (0.014, -0.160),
                (0.038, -0.154),
                (0.078, -0.126),
                (0.102, -0.080),
                (0.106, -0.040),
                (0.094, 0.000),
            ],
            [
                (0.000, -0.148),
                (0.026, -0.144),
                (0.066, -0.118),
                (0.094, -0.074),
                (0.098, -0.040),
                (0.086, -0.008),
            ],
            segments=72,
        ),
        "ceiling_fan_diffuser",
    )

    model = ArticulatedObject(name="ceiling_fan_with_light_kit")

    brushed_nickel = model.material("brushed_nickel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.24, 0.25, 0.28, 1.0))
    walnut = model.material("walnut", rgba=(0.47, 0.33, 0.21, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.96, 0.97, 0.99, 0.58))
    warm_light = model.material("warm_light", rgba=(0.95, 0.90, 0.76, 0.92))

    canopy = model.part("canopy")
    canopy.visual(
        Box((0.170, 0.170, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=brushed_nickel,
        name="ceiling_plate",
    )
    canopy.visual(
        canopy_mesh,
        material=brushed_nickel,
        name="canopy_shell",
    )
    canopy.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.114)),
        material=dark_trim,
        name="canopy_collar",
    )

    downrod = model.part("downrod")
    downrod.visual(
        Cylinder(radius=0.011, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=brushed_nickel,
        name="rod_shaft",
    )
    downrod.visual(
        Cylinder(radius=0.018, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.301)),
        material=dark_trim,
        name="lower_coupling",
    )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, -0.126)),
    )

    rotor = model.part("rotor_assembly")
    rotor.visual(
        rotor_mesh,
        material=brushed_nickel,
        name="motor_shell",
    )
    rotor.visual(
        Cylinder(radius=0.102, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.220)),
        material=dark_trim,
        name="light_ring",
    )
    rotor.visual(
        Cylinder(radius=0.009, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, -0.262)),
        material=dark_trim,
        name="light_stem",
    )
    rotor.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.306)),
        material=dark_trim,
        name="bulb_socket",
    )
    rotor.visual(
        Sphere(radius=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.343)),
        material=warm_light,
        name="bulb_globe",
    )

    fan_spin = model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=downrod,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=15.0),
    )

    blade_mount_radius = 0.114
    blade_mount_z = -0.052
    for index in range(5):
        angle = index * (math.tau / 5.0)
        rotor.visual(
            Box((0.022, 0.040, 0.014)),
            origin=Origin(
                xyz=(
                    0.106 * math.cos(angle),
                    0.106 * math.sin(angle),
                    -0.056,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_trim,
            name=f"blade_root_{index + 1}",
        )
        rotor.visual(
            Box((0.082, 0.024, 0.012)),
            origin=Origin(
                xyz=(
                    0.155 * math.cos(angle),
                    0.155 * math.sin(angle),
                    -0.060,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_trim,
            name=f"blade_arm_{index + 1}",
        )
        rotor.visual(
            Box((0.064, 0.118, 0.014)),
            origin=Origin(
                xyz=(
                    0.218 * math.cos(angle),
                    0.218 * math.sin(angle),
                    -0.068,
                ),
                rpy=(0.18, 0.0, angle),
            ),
            material=dark_trim,
            name=f"blade_bracket_{index + 1}",
        )
        rotor.visual(
            blade_mesh,
            origin=Origin(
                xyz=(
                    blade_mount_radius * math.cos(angle),
                    blade_mount_radius * math.sin(angle),
                    blade_mount_z,
                ),
                rpy=(0.18, 0.0, angle),
            ),
            material=walnut,
            name=f"blade_panel_{index + 1}",
        )

    diffuser = model.part("diffuser_cover")
    diffuser.visual(
        diffuser_mesh,
        material=frosted_glass,
        name="glass_shell",
    )
    diffuser.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(-0.099, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="left_hinge_pin",
    )
    diffuser.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.099, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="right_hinge_pin",
    )
    diffuser.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.151)),
        material=brushed_nickel,
        name="finial_flange",
    )
    diffuser.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.159)),
        material=brushed_nickel,
        name="diffuser_finial",
    )

    model.articulation(
        "diffuser_hinge",
        ArticulationType.REVOLUTE,
        parent=rotor,
        child=diffuser,
        origin=Origin(xyz=(0.0, 0.0, -0.236)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    downrod = object_model.get_part("downrod")
    rotor = object_model.get_part("rotor_assembly")
    diffuser = object_model.get_part("diffuser_cover")
    fan_spin = object_model.get_articulation("fan_spin")
    diffuser_hinge = object_model.get_articulation("diffuser_hinge")
    light_ring = rotor.get_visual("light_ring")
    glass_shell = diffuser.get_visual("glass_shell")
    blade_panel_1 = rotor.get_visual("blade_panel_1")

    def aabb_center(aabb):
        return (
            (aabb[0][0] + aabb[1][0]) * 0.5,
            (aabb[0][1] + aabb[1][1]) * 0.5,
            (aabb[0][2] + aabb[1][2]) * 0.5,
        )

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

    ctx.expect_contact(downrod, canopy, contact_tol=0.001, name="downrod seats in canopy")
    ctx.expect_overlap(
        downrod,
        canopy,
        axes="xy",
        min_overlap=0.020,
        name="downrod centered under canopy",
    )
    ctx.expect_contact(rotor, downrod, contact_tol=0.001, name="rotor couples to downrod")
    ctx.expect_overlap(
        rotor,
        downrod,
        axes="xy",
        min_overlap=0.020,
        name="rotor centered on downrod",
    )

    ctx.expect_overlap(
        diffuser,
        rotor,
        axes="xy",
        min_overlap=0.170,
        elem_a=glass_shell,
        elem_b=light_ring,
        name="diffuser centered under light ring",
    )
    ctx.expect_gap(
        rotor,
        diffuser,
        axis="z",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem=light_ring,
        negative_elem=glass_shell,
        name="closed diffuser sits just below light ring",
    )

    ctx.check(
        "fan articulation axis is vertical",
        fan_spin.axis == (0.0, 0.0, 1.0),
        details=f"axis={fan_spin.axis}",
    )
    ctx.check(
        "diffuser hinge axis is lateral",
        diffuser_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={diffuser_hinge.axis}",
    )

    rotor_origin = ctx.part_world_position(rotor)
    blade_rest = ctx.part_element_world_aabb(rotor, elem=blade_panel_1)
    assert rotor_origin is not None
    assert blade_rest is not None
    blade_rest_center = aabb_center(blade_rest)
    ctx.check(
        "blade panel sits at realistic radius",
        0.34 <= math.hypot(blade_rest_center[0] - rotor_origin[0], blade_rest_center[1] - rotor_origin[1]) <= 0.46,
        details=f"blade_center={blade_rest_center} rotor_origin={rotor_origin}",
    )
    blade_rest_radius = math.hypot(
        blade_rest_center[0] - rotor_origin[0],
        blade_rest_center[1] - rotor_origin[1],
    )
    with ctx.pose({fan_spin: 1.0}):
        blade_spun = ctx.part_element_world_aabb(rotor, elem=blade_panel_1)
        assert blade_spun is not None
        blade_spun_center = aabb_center(blade_spun)
        blade_spun_radius = math.hypot(
            blade_spun_center[0] - rotor_origin[0],
            blade_spun_center[1] - rotor_origin[1],
        )
        ctx.check(
            "fan spin carries blade around hub",
            (
                blade_spun_center[1] > blade_rest_center[1] + 0.070
                and abs(blade_spun_radius - blade_rest_radius) < 0.010
                and abs(blade_spun_center[2] - blade_rest_center[2]) < 0.002
            ),
            details=f"rest={blade_rest_center}, spun={blade_spun_center}",
        )

    glass_rest_aabb = ctx.part_element_world_aabb(diffuser, elem=glass_shell)
    assert glass_rest_aabb is not None
    glass_rest_center = aabb_center(glass_rest_aabb)
    with ctx.pose({diffuser_hinge: 1.05}):
        glass_open_aabb = ctx.part_element_world_aabb(diffuser, elem=glass_shell)
        assert glass_open_aabb is not None
        glass_open_center = aabb_center(glass_open_aabb)
        ctx.check(
            "diffuser swings open for bulb access",
            (
                glass_open_center[1] > glass_rest_center[1] + 0.060
                and glass_open_center[2] > glass_rest_center[2] + 0.040
            ),
            details=f"closed={glass_rest_center}, open={glass_open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
