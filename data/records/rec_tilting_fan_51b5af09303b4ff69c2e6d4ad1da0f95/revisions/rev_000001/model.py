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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _torus_mesh(major_radius: float, tube_radius: float, name: str):
    geom = TorusGeometry(major_radius, tube_radius, radial_segments=18, tubular_segments=44)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def _blade_sections():
    def loop(z: float, x_front: float, x_back: float, y_half: float, camber: float):
        mid_front = 0.62 * x_front + 0.38 * x_back
        mid_back = 0.28 * x_front + 0.72 * x_back
        return [
            (x_back, -y_half, z),
            (mid_back, -0.95 * y_half, z),
            (x_front, -0.22 * y_half, z),
            (x_front + camber, 0.20 * y_half, z),
            (mid_front + 0.6 * camber, 0.88 * y_half, z),
            (x_back + 0.25 * camber, y_half, z),
        ]

    return [
        loop(0.014, 0.010, -0.010, 0.009, 0.0008),
        loop(0.058, 0.015, -0.008, 0.0075, 0.0018),
        loop(0.102, 0.020, -0.004, 0.0050, 0.0030),
        loop(0.142, 0.024, 0.001, 0.0032, 0.0045),
    ]


def _blade_mesh(name: str, angle: float):
    geom = section_loft(_blade_sections())
    geom.rotate_x(angle)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_tilting_fan")

    body = model.material("body_powdercoat", rgba=(0.22, 0.24, 0.26, 1.0))
    guard = model.material("guard_powdercoat", rgba=(0.72, 0.75, 0.78, 1.0))
    hardware = model.material("stainless_hardware", rgba=(0.80, 0.82, 0.84, 1.0))
    blade = model.material("composite_blade", rgba=(0.58, 0.62, 0.66, 1.0))
    seal = model.material("sealed_gasket", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.21, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=body,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=body,
        name="ballast_cover",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, 0.407)),
        material=body,
        name="mast",
    )
    base.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=seal,
        name="mast_collar",
    )
    base.visual(
        Box((0.06, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        material=body,
        name="yoke_core",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        base.visual(
            Box((0.05, 0.045, 0.06)),
            origin=Origin(xyz=(-0.05, sign * 0.09, 0.710)),
            material=body,
            name=f"{side}_rear_brace",
        )
        base.visual(
            Box((0.20, 0.03, 0.26)),
            origin=Origin(xyz=(-0.08, sign * 0.125, 0.80)),
            material=body,
            name=f"{side}_arm",
        )
        base.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(
                xyz=(0.0, sign * 0.117, 0.80),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"{side}_bushing",
        )
        base.visual(
            Cylinder(radius=0.030, length=0.032),
            origin=Origin(
                xyz=(0.0, sign * 0.156, 0.80),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"{side}_tilt_knob",
        )
    base.visual(
        Box((0.06, 0.30, 0.03)),
        origin=Origin(xyz=(-0.05, 0.0, 0.920)),
        material=body,
        name="top_bridge",
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.018, length=0.164),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_shaft",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(
            xyz=(0.0, 0.096, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=seal,
        name="left_pivot_collar",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(
            xyz=(0.0, -0.096, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=seal,
        name="right_pivot_collar",
    )
    head.visual(
        Box((0.074, 0.170, 0.044)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=body,
        name="pivot_fairing",
    )
    head.visual(
        Cylinder(radius=0.065, length=0.090),
        origin=Origin(xyz=(0.105, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body,
        name="motor_housing",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body,
        name="nose_block",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.182, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="spindle_nose",
    )
    head.visual(
        Box((0.080, 0.080, 0.018)),
        origin=Origin(xyz=(0.140, 0.0, 0.049)),
        material=body,
        name="rain_visor",
    )
    head.visual(
        Box((0.014, 0.140, 0.020)),
        origin=Origin(xyz=(0.168, 0.105, 0.0)),
        material=body,
        name="left_mount_ear",
    )
    head.visual(
        Box((0.014, 0.140, 0.020)),
        origin=Origin(xyz=(0.168, -0.105, 0.0)),
        material=body,
        name="right_mount_ear",
    )

    front_guard = model.part("front_guard")
    front_guard.visual(
        _torus_mesh(0.215, 0.008, "front_guard_outer_rim"),
        origin=Origin(xyz=(0.252, 0.0, 0.0)),
        material=guard,
        name="outer_rim",
    )
    front_guard.visual(
        _torus_mesh(0.120, 0.005, "front_guard_inner_rim"),
        origin=Origin(xyz=(0.252, 0.0, 0.0)),
        material=guard,
        name="inner_rim",
    )
    front_guard.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(0.252, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=guard,
        name="center_badge",
    )
    front_guard.visual(
        Box((0.006, 0.012, 0.080)),
        origin=Origin(xyz=(0.252, 0.0, 0.080)),
        material=guard,
        name="center_spoke_top",
    )
    front_guard.visual(
        Box((0.006, 0.012, 0.080)),
        origin=Origin(xyz=(0.252, 0.0, -0.080)),
        material=guard,
        name="center_spoke_bottom",
    )
    front_guard.visual(
        Box((0.006, 0.070, 0.012)),
        origin=Origin(xyz=(0.252, 0.080, 0.0)),
        material=guard,
        name="center_spoke_left",
    )
    front_guard.visual(
        Box((0.006, 0.070, 0.012)),
        origin=Origin(xyz=(0.252, -0.080, 0.0)),
        material=guard,
        name="center_spoke_right",
    )
    front_guard.visual(
        Box((0.006, 0.018, 0.100)),
        origin=Origin(xyz=(0.252, 0.0, 0.157)),
        material=guard,
        name="outer_spoke_top",
    )
    front_guard.visual(
        Box((0.006, 0.018, 0.100)),
        origin=Origin(xyz=(0.252, 0.0, -0.157)),
        material=guard,
        name="outer_spoke_bottom",
    )
    front_guard.visual(
        Box((0.006, 0.085, 0.018)),
        origin=Origin(xyz=(0.252, 0.1675, 0.0)),
        material=guard,
        name="outer_spoke_left",
    )
    front_guard.visual(
        Box((0.006, 0.085, 0.018)),
        origin=Origin(xyz=(0.252, -0.1675, 0.0)),
        material=guard,
        name="outer_spoke_right",
    )
    front_guard.visual(
        Box((0.097, 0.018, 0.012)),
        origin=Origin(xyz=(0.2235, 0.0, 0.1675)),
        material=hardware,
        name="clamp_top",
    )
    front_guard.visual(
        Box((0.097, 0.018, 0.012)),
        origin=Origin(xyz=(0.2235, 0.0, -0.1675)),
        material=hardware,
        name="clamp_bottom",
    )
    front_guard.visual(
        Box((0.097, 0.012, 0.018)),
        origin=Origin(xyz=(0.2235, 0.1675, 0.0)),
        material=hardware,
        name="clamp_left",
    )
    front_guard.visual(
        Box((0.097, 0.012, 0.018)),
        origin=Origin(xyz=(0.2235, -0.1675, 0.0)),
        material=hardware,
        name="clamp_right",
    )

    rear_guard = model.part("rear_guard")
    rear_guard.visual(
        _torus_mesh(0.205, 0.008, "rear_guard_outer_rim"),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=guard,
        name="outer_rim",
    )
    rear_guard.visual(
        _torus_mesh(0.100, 0.005, "rear_guard_inner_rim"),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=guard,
        name="inner_rim",
    )
    rear_guard.visual(
        Box((0.006, 0.018, 0.100)),
        origin=Origin(xyz=(0.155, 0.0, 0.152)),
        material=guard,
        name="top_link",
    )
    rear_guard.visual(
        Box((0.006, 0.018, 0.100)),
        origin=Origin(xyz=(0.155, 0.0, -0.152)),
        material=guard,
        name="bottom_link",
    )
    rear_guard.visual(
        Box((0.006, 0.100, 0.018)),
        origin=Origin(xyz=(0.158, 0.152, 0.0)),
        material=guard,
        name="left_link",
    )
    rear_guard.visual(
        Box((0.006, 0.100, 0.018)),
        origin=Origin(xyz=(0.158, -0.152, 0.0)),
        material=guard,
        name="right_link",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="hub_sleeve",
    )
    rotor.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=hardware,
        name="hub_cap",
    )
    for idx in range(3):
        rotor.visual(
            _blade_mesh(f"rotor_blade_{idx}", idx * math.tau / 3.0),
            material=blade,
            name=f"blade_{idx}",
        )

    tilt = model.articulation(
        "base_to_head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.3,
            lower=-0.25,
            upper=0.55,
        ),
    )
    model.articulation(
        "head_to_front_guard",
        ArticulationType.FIXED,
        parent=head,
        child=front_guard,
        origin=Origin(),
    )
    model.articulation(
        "head_to_rear_guard",
        ArticulationType.FIXED,
        parent=head,
        child=rear_guard,
        origin=Origin(),
    )
    spin = model.articulation(
        "head_to_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.190, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )

    model.meta["primary_articulations"] = [tilt.name, spin.name]
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    front_guard = object_model.get_part("front_guard")
    rear_guard = object_model.get_part("rear_guard")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head_tilt")
    spin = object_model.get_articulation("head_to_rotor_spin")

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
        "tilt_joint_axis_matches_side_pivot",
        tilt.axis == (0.0, -1.0, 0.0),
        details=f"expected (0, -1, 0), got {tilt.axis}",
    )
    ctx.check(
        "rotor_spin_axis_matches_hub_axis",
        spin.axis == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {spin.axis}",
    )

    ctx.expect_contact(base, head, elem_a="left_bushing", elem_b="left_pivot_collar")
    ctx.expect_contact(base, head, elem_a="right_bushing", elem_b="right_pivot_collar")
    ctx.expect_contact(front_guard, head)
    ctx.expect_contact(rear_guard, head)
    ctx.expect_contact(rotor, head, elem_a="hub_sleeve", elem_b="spindle_nose")

    ctx.expect_gap(front_guard, rotor, axis="x", positive_elem="outer_rim", min_gap=0.004, max_gap=0.045)
    ctx.expect_gap(rotor, rear_guard, axis="x", min_gap=0.010, max_gap=0.060)
    ctx.expect_within(rotor, front_guard, axes="yz", margin=0.012)

    with ctx.pose({tilt: 0.0}):
        rest_aabb = ctx.part_element_world_aabb(front_guard, elem="outer_rim")
    with ctx.pose({tilt: 0.45}):
        tilt_aabb = ctx.part_element_world_aabb(front_guard, elem="outer_rim")

    if rest_aabb is None or tilt_aabb is None:
        ctx.fail("tilt_pose_measurements_available", "front guard rim AABB was unavailable in one of the checked poses")
    else:
        rest_z = 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
        tilt_z = 0.5 * (tilt_aabb[0][2] + tilt_aabb[1][2])
        ctx.check(
            "positive_tilt_lifts_front_of_head",
            tilt_z > rest_z + 0.03,
            details=f"rest_z={rest_z:.4f}, tilt_z={tilt_z:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
