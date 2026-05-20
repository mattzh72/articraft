from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _add_wheel_visuals(part, *, mesh_prefix: str, wood_material, tire_material) -> None:
    felloe_radius = 0.63
    felloe_tube = 0.060
    hub_radius = 0.095
    hub_length = 0.28
    flange_radius = 0.118
    flange_length = 0.042
    part.visual(
        _save_mesh(
            f"{mesh_prefix}_felloe",
            TorusGeometry(
                radius=felloe_radius,
                tube=felloe_tube,
                radial_segments=18,
                tubular_segments=64,
            ).rotate_x(pi / 2.0),
        ),
        material=wood_material,
        name="wood_felloe",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wood_material,
        name="hub_body",
    )
    part.visual(
        Cylinder(radius=flange_radius, length=flange_length),
        origin=Origin(xyz=(0.0, 0.119, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wood_material,
        name="outer_flange",
    )
    part.visual(
        Cylinder(radius=flange_radius, length=flange_length),
        origin=Origin(xyz=(0.0, -0.119, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=wood_material,
        name="inner_flange",
    )

    spoke_inner = 0.070
    spoke_outer = 0.600
    spoke_length = spoke_outer - spoke_inner
    spoke_center_radius = 0.5 * (spoke_inner + spoke_outer)
    for index in range(12):
        angle = 2.0 * pi * index / 12.0
        part.visual(
            Box((spoke_length, 0.090, 0.065)),
            origin=Origin(
                xyz=(cos(angle) * spoke_center_radius, 0.0, sin(angle) * spoke_center_radius),
                rpy=(0.0, angle, 0.0),
            ),
            material=wood_material,
            name=f"spoke_{index:02d}",
        )

    part.visual(
        _save_mesh(
            f"{mesh_prefix}_tire",
            TorusGeometry(
                radius=0.685,
                tube=0.035,
                radial_segments=18,
                tubular_segments=72,
            ).rotate_x(pi / 2.0),
        ),
        material=tire_material,
        name="iron_tire",
    )


def _build_barrel_shell_mesh() -> MeshGeometry:
    outer_profile = [
        (0.0, -0.54),
        (0.052, -0.50),
        (0.090, -0.45),
        (0.061, -0.40),
        (0.150, -0.35),
        (0.198, -0.25),
        (0.222, -0.10),
        (0.228, 0.10),
        (0.220, 0.34),
        (0.206, 0.74),
        (0.188, 1.12),
        (0.175, 1.34),
        (0.184, 1.50),
        (0.196, 1.58),
    ]
    inner_profile = [
        (0.020, -0.10),
        (0.042, -0.02),
        (0.056, 0.40),
        (0.0585, 1.18),
        (0.060, 1.52),
        (0.060, 1.58),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(pi / 2.0)


def _aabb_center_z(aabb) -> float | None:
    if aabb is None:
        return None
    return 0.5 * (aabb[0][2] + aabb[1][2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="napoleonic_12_pounder_field_cannon")

    bronze = model.material("bronze", rgba=(0.68, 0.51, 0.24, 1.0))
    oiled_oak = model.material("oiled_oak", rgba=(0.50, 0.33, 0.16, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.59, 0.43, 0.24, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.12, 0.12, 0.13, 1.0))

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.84, 0.06, 0.56)),
        origin=Origin(xyz=(0.22, 0.36, 0.02)),
        material=oiled_oak,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.84, 0.06, 0.56)),
        origin=Origin(xyz=(0.22, -0.36, 0.02)),
        material=oiled_oak,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.30, 0.78, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, -0.12)),
        material=oiled_oak,
        name="axle_body",
    )
    carriage.visual(
        Cylinder(radius=0.055, length=1.12),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="axle_shaft",
    )
    carriage.visual(
        Box((0.24, 0.74, 0.12)),
        origin=Origin(xyz=(0.54, 0.0, -0.14)),
        material=oiled_oak,
        name="front_transom",
    )
    carriage.visual(
        Box((1.46, 0.22, 0.22)),
        origin=Origin(xyz=(-0.82, 0.0, -0.20)),
        material=oiled_oak,
        name="trail_beam_lower",
    )
    carriage.visual(
        Box((1.16, 0.14, 0.18)),
        origin=Origin(xyz=(-0.98, 0.0, -0.03)),
        material=weathered_wood,
        name="trail_beam_upper",
    )
    carriage.visual(
        Box((0.20, 0.26, 0.08)),
        origin=Origin(xyz=(-1.46, 0.0, -0.24)),
        material=oiled_oak,
        name="trail_rear_cap",
    )
    carriage.visual(
        Box((0.16, 0.16, 0.02)),
        origin=Origin(xyz=(-1.50, 0.0, -0.33)),
        material=dark_iron,
        name="trail_skid_shoe",
    )
    carriage.visual(
        Box((0.10, 0.22, 0.06)),
        origin=Origin(xyz=(-1.56, 0.0, -0.31)),
        material=dark_iron,
        name="tow_hook_bracket_body",
    )
    carriage.visual(
        Box((0.08, 0.04, 0.09)),
        origin=Origin(xyz=(-1.62, 0.07, -0.34)),
        material=dark_iron,
        name="tow_hook_left_lug",
    )
    carriage.visual(
        Box((0.08, 0.04, 0.09)),
        origin=Origin(xyz=(-1.62, -0.07, -0.34)),
        material=dark_iron,
        name="tow_hook_right_lug",
    )
    carriage.visual(
        Box((0.12, 0.02, 0.12)),
        origin=Origin(xyz=(0.34, 0.40, 0.20)),
        material=dark_iron,
        name="left_trunnion_cap_plate",
    )
    carriage.visual(
        Box((0.12, 0.02, 0.12)),
        origin=Origin(xyz=(0.34, -0.40, 0.20)),
        material=dark_iron,
        name="right_trunnion_cap_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((3.10, 1.10, 1.00)),
        mass=540.0,
        origin=Origin(xyz=(-0.20, 0.0, -0.06)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("napoleon_barrel_shell", _build_barrel_shell_mesh()),
        material=bronze,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.068, length=0.12),
        origin=Origin(xyz=(0.0, 0.27, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.068, length=0.12),
        origin=Origin(xyz=(0.0, -0.27, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bronze,
        name="right_trunnion",
    )
    barrel.visual(
        Box((0.050, 0.018, 0.018)),
        origin=Origin(xyz=(1.48, 0.0, 0.193)),
        material=bronze,
        name="front_sight",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=2.12),
        mass=620.0,
        origin=Origin(xyz=(0.42, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    left_wheel = model.part("left_wheel")
    _add_wheel_visuals(
        left_wheel,
        mesh_prefix="left_wheel",
        wood_material=weathered_wood,
        tire_material=blackened_steel,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=0.28),
        mass=92.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _add_wheel_visuals(
        right_wheel,
        mesh_prefix="right_wheel",
        wood_material=weathered_wood,
        tire_material=blackened_steel,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=0.28),
        mass=92.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    tow_hook_plate = model.part("tow_hook_plate")
    tow_hook_plate.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="hinge_barrel",
    )
    tow_hook_plate.visual(
        Box((0.12, 0.022, 0.055)),
        origin=Origin(xyz=(-0.06, 0.0, -0.028)),
        material=dark_iron,
        name="hook_plate",
    )
    tow_hook_plate.visual(
        Box((0.14, 0.022, 0.045)),
        origin=Origin(xyz=(-0.18, 0.0, -0.065)),
        material=dark_iron,
        name="hook_shank",
    )
    tow_hook_plate.visual(
        Box((0.028, 0.022, 0.10)),
        origin=Origin(xyz=(-0.25, 0.0, -0.148)),
        material=dark_iron,
        name="ring_strap",
    )
    tow_hook_plate.visual(
        _save_mesh(
            "tow_hook_lunette",
            TorusGeometry(
                radius=0.075,
                tube=0.012,
                radial_segments=14,
                tubular_segments=40,
            ).rotate_y(pi / 2.0),
        ),
        origin=Origin(xyz=(-0.25, 0.0, -0.11), rpy=(0.0, -0.08, 0.0)),
        material=dark_iron,
        name="lunette_ring",
    )
    tow_hook_plate.inertial = Inertial.from_geometry(
        Box((0.32, 0.14, 0.14)),
        mass=18.0,
        origin=Origin(xyz=(-0.16, 0.0, -0.03)),
    )

    model.articulation(
        "carriage_to_barrel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(0.34, 0.0, 0.28)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4200.0,
            velocity=0.8,
            lower=-0.15,
            upper=0.52,
        ),
    )
    model.articulation(
        "carriage_to_left_wheel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=left_wheel,
        origin=Origin(xyz=(0.0, 0.70, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=8.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )
    model.articulation(
        "carriage_to_right_wheel",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=right_wheel,
        origin=Origin(xyz=(0.0, -0.70, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=8.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )
    model.articulation(
        "carriage_to_tow_hook_plate",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tow_hook_plate,
        origin=Origin(xyz=(-1.66, 0.0, -0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    tow_hook_plate = object_model.get_part("tow_hook_plate")

    barrel_joint = object_model.get_articulation("carriage_to_barrel")
    left_wheel_joint = object_model.get_articulation("carriage_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("carriage_to_right_wheel")
    tow_hook_joint = object_model.get_articulation("carriage_to_tow_hook_plate")

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

    ctx.expect_contact(left_wheel, carriage, name="left wheel bears on the axle")
    ctx.expect_contact(right_wheel, carriage, name="right wheel bears on the axle")
    ctx.expect_contact(barrel, carriage, name="barrel trunnions seat in the cheeks")
    ctx.expect_contact(tow_hook_plate, carriage, name="tow hook plate is pinned to the trail bracket")

    ctx.check(
        "barrel uses a realistic elevation hinge axis",
        barrel_joint.articulation_type == ArticulationType.REVOLUTE
        and barrel_joint.axis == (0.0, -1.0, 0.0),
        details=f"type={barrel_joint.articulation_type}, axis={barrel_joint.axis}",
    )
    ctx.check(
        "wheel axles run across the carriage",
        left_wheel_joint.articulation_type == ArticulationType.REVOLUTE
        and right_wheel_joint.articulation_type == ArticulationType.REVOLUTE
        and left_wheel_joint.axis == (0.0, 1.0, 0.0)
        and right_wheel_joint.axis == (0.0, 1.0, 0.0),
        details=f"left={left_wheel_joint.axis}, right={right_wheel_joint.axis}",
    )
    ctx.check(
        "tow hook plate swings on a transverse pin",
        tow_hook_joint.articulation_type == ArticulationType.REVOLUTE
        and tow_hook_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={tow_hook_joint.articulation_type}, axis={tow_hook_joint.axis}",
    )

    rest_sight = ctx.part_element_world_aabb(barrel, elem="front_sight")
    with ctx.pose({barrel_joint: 0.35}):
        raised_sight = ctx.part_element_world_aabb(barrel, elem="front_sight")
    rest_sight_z = _aabb_center_z(rest_sight)
    raised_sight_z = _aabb_center_z(raised_sight)
    ctx.check(
        "positive barrel elevation lifts the muzzle",
        rest_sight_z is not None and raised_sight_z is not None and raised_sight_z > rest_sight_z + 0.08,
        details=f"rest_z={rest_sight_z}, raised_z={raised_sight_z}",
    )

    rest_ring = ctx.part_element_world_aabb(tow_hook_plate, elem="lunette_ring")
    with ctx.pose({tow_hook_joint: 0.70}):
        lifted_ring = ctx.part_element_world_aabb(tow_hook_plate, elem="lunette_ring")
    rest_ring_z = _aabb_center_z(rest_ring)
    lifted_ring_z = _aabb_center_z(lifted_ring)
    ctx.check(
        "tow hook plate can swing upward for limbering",
        rest_ring_z is not None and lifted_ring_z is not None and lifted_ring_z > rest_ring_z + 0.08,
        details=f"rest_z={rest_ring_z}, lifted_z={lifted_ring_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
