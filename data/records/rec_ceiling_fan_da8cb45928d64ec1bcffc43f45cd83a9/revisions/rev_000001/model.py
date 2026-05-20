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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _regular_polygon_loop(
    *,
    sides: int,
    circumradius: float,
    z: float,
    angle_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (
            circumradius * math.cos(angle_offset + (2.0 * math.pi * index / sides)),
            circumradius * math.sin(angle_offset + (2.0 * math.pi * index / sides)),
            z,
        )
        for index in range(sides)
    ]


def _hex_loop(flat_to_flat: float, z: float) -> list[tuple[float, float, float]]:
    apothem = flat_to_flat * 0.5
    circumradius = apothem / math.cos(math.pi / 6.0)
    # Start at 30° so the housing presents a flat face on +X for the pull-chain boss.
    return _regular_polygon_loop(
        sides=6,
        circumradius=circumradius,
        z=z,
        angle_offset=math.pi / 6.0,
    )


def _blade_outline(
    *,
    length: float,
    root_width: float,
    mid_width: float,
    tip_width: float,
) -> list[tuple[float, float]]:
    root_half = root_width * 0.5
    mid_half = mid_width * 0.5
    tip_half = tip_width * 0.5
    return [
        (0.000, -root_half),
        (0.040, -root_half * 1.02),
        (0.170, -mid_half),
        (0.340, -mid_half * 0.96),
        (length - 0.050, -tip_half * 1.02),
        (length, -tip_half * 0.40),
        (length, tip_half * 0.40),
        (length - 0.050, tip_half * 1.02),
        (0.340, mid_half * 0.96),
        (0.170, mid_half),
        (0.040, root_half * 1.02),
        (0.000, root_half),
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def _aabb_max_planar_radius(aabb, *, origin_xy: tuple[float, float]) -> float | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    ox, oy = origin_xy
    max_radius = 0.0
    for x in (mins[0], maxs[0]):
        for y in (mins[1], maxs[1]):
            max_radius = max(max_radius, math.hypot(x - ox, y - oy))
    return max_radius


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farmhouse_ceiling_fan")

    canopy_black = model.material("canopy_black", rgba=(0.17, 0.16, 0.15, 1.0))
    iron_black = model.material("iron_black", rgba=(0.14, 0.14, 0.14, 1.0))
    aged_steel = model.material("aged_steel", rgba=(0.34, 0.34, 0.33, 1.0))
    reclaimed_wood = model.material("reclaimed_wood", rgba=(0.46, 0.34, 0.24, 1.0))
    pull_brass = model.material("pull_brass", rgba=(0.63, 0.56, 0.38, 1.0))

    canopy = model.part("ceiling_canopy")
    canopy_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.000, 0.000),
                (0.054, 0.000),
                (0.077, 0.010),
                (0.082, 0.030),
                (0.075, 0.052),
                (0.058, 0.072),
                (0.020, 0.086),
                (0.000, 0.086),
            ],
            segments=64,
        ),
        "canopy_shell",
    )
    canopy.visual(canopy_mesh, material=canopy_black, name="canopy_shell")
    canopy.visual(
        Cylinder(radius=0.086, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=canopy_black,
        name="ceiling_plate",
    )
    canopy.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=aged_steel,
        name="canopy_collar",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((0.170, 0.170, 0.086)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
    )

    downrod = model.part("downrod")
    downrod.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=aged_steel,
        name="upper_collar",
    )
    downrod.visual(
        Cylinder(radius=0.011, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        material=aged_steel,
        name="rod",
    )
    downrod.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=aged_steel,
        name="lower_collar",
    )
    downrod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.342),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
    )

    motor_housing = model.part("motor_housing")
    housing_mesh = mesh_from_geometry(
        section_loft(
            [
                _hex_loop(0.220, 0.012),
                _hex_loop(0.280, 0.038),
                _hex_loop(0.280, 0.142),
                _hex_loop(0.240, 0.180),
            ]
        ),
        "motor_housing_hex_shell",
    )
    motor_housing.visual(housing_mesh, material=canopy_black, name="hex_shell")
    motor_housing.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=aged_steel,
        name="top_yoke",
    )
    motor_housing.visual(
        Cylinder(radius=0.082, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=iron_black,
        name="bottom_cap",
    )
    motor_housing.visual(
        Cylinder(radius=0.064, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.169)),
        material=aged_steel,
        name="bearing_collar",
    )
    motor_housing.visual(
        Box((0.016, 0.022, 0.022)),
        origin=Origin(xyz=(0.148, 0.0, 0.122)),
        material=aged_steel,
        name="switch_pivot_boss",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Box((0.320, 0.320, 0.180)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    blade_assembly = model.part("blade_assembly")
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            _blade_outline(
                length=0.540,
                root_width=0.145,
                mid_width=0.132,
                tip_width=0.112,
            ),
            0.012,
            cap=True,
            closed=True,
        ),
        "farmhouse_blade",
    )
    blade_assembly.visual(
        Cylinder(radius=0.064, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=aged_steel,
        name="rotor_cap",
    )
    blade_assembly.visual(
        Cylinder(radius=0.096, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=iron_black,
        name="hub_drum",
    )
    blade_assembly.visual(
        Cylinder(radius=0.112, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=aged_steel,
        name="spider_plate",
    )
    blade_assembly.visual(
        Cylinder(radius=0.090, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=iron_black,
        name="lower_hub_cover",
    )

    blade_angles_deg = (36.0, 108.0, 180.0, 252.0, 324.0)
    for blade_index, angle_deg in enumerate(blade_angles_deg):
        angle = math.radians(angle_deg)
        blade_assembly.visual(
            Box((0.170, 0.028, 0.010)),
            origin=Origin(xyz=(0.136, 0.0, 0.030), rpy=(0.0, 0.0, angle)),
            material=aged_steel,
            name=f"blade_iron_arm_{blade_index}",
        )
        blade_assembly.visual(
            Box((0.090, 0.102, 0.006)),
            origin=Origin(xyz=(0.205, 0.0, 0.039), rpy=(0.0, 0.0, angle)),
            material=aged_steel,
            name=f"blade_iron_plate_{blade_index}",
        )
        blade_assembly.visual(
            Box((0.050, 0.016, 0.026)),
            origin=Origin(xyz=(0.102, 0.0, 0.039), rpy=(0.0, 0.0, angle)),
            material=aged_steel,
            name=f"blade_iron_gusset_{blade_index}",
        )
        blade_assembly.visual(
            blade_mesh,
            origin=Origin(
                xyz=(0.125, 0.0, 0.036),
                rpy=(math.radians(6.0), 0.0, angle),
            ),
            material=reclaimed_wood,
            name=f"blade_{blade_index}",
        )
    blade_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.670, length=0.110),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    pull_chain = model.part("pull_chain")
    pull_chain.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aged_steel,
        name="pivot_barrel",
    )
    pull_chain.visual(
        Box((0.012, 0.004, 0.024)),
        origin=Origin(xyz=(0.010, 0.0, 0.012)),
        material=aged_steel,
        name="switch_link",
    )
    pull_chain.visual(
        Cylinder(radius=0.0017, length=0.138),
        origin=Origin(xyz=(0.012, 0.0, 0.091)),
        material=pull_brass,
        name="chain_run",
    )
    pull_chain.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.012, 0.0, 0.164)),
        material=pull_brass,
        name="chain_knot",
    )
    pull_chain.visual(
        Cylinder(radius=0.0022, length=0.020),
        origin=Origin(xyz=(0.012, 0.0, 0.178)),
        material=pull_brass,
        name="handle_stem",
    )
    pull_chain.visual(
        Cylinder(radius=0.0045, length=0.028),
        origin=Origin(xyz=(0.012, 0.0, 0.196)),
        material=pull_brass,
        name="pull_handle",
    )
    pull_chain.inertial = Inertial.from_geometry(
        Box((0.024, 0.018, 0.224)),
        mass=0.08,
        origin=Origin(xyz=(0.012, 0.0, 0.112)),
    )

    model.articulation(
        "canopy_to_downrod",
        ArticulationType.FIXED,
        parent=canopy,
        child=downrod,
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
    )
    model.articulation(
        "downrod_to_motor_housing",
        ArticulationType.FIXED,
        parent=downrod,
        child=motor_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.342)),
    )
    model.articulation(
        "motor_to_blade_spin",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=blade_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=20.0),
    )
    model.articulation(
        "housing_to_pull_chain",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=pull_chain,
        origin=Origin(xyz=(0.160, 0.0, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(38.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("ceiling_canopy")
    downrod = object_model.get_part("downrod")
    motor_housing = object_model.get_part("motor_housing")
    blade_assembly = object_model.get_part("blade_assembly")
    pull_chain = object_model.get_part("pull_chain")

    blade_spin = object_model.get_articulation("motor_to_blade_spin")
    chain_pivot = object_model.get_articulation("housing_to_pull_chain")

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
        downrod,
        canopy,
        elem_a="upper_collar",
        elem_b="canopy_collar",
        name="downrod seats into canopy collar",
    )
    ctx.expect_contact(
        motor_housing,
        downrod,
        elem_a="top_yoke",
        elem_b="lower_collar",
        name="motor housing hangs from downrod collar",
    )
    ctx.expect_contact(
        blade_assembly,
        motor_housing,
        elem_a="rotor_cap",
        elem_b="bearing_collar",
        name="blade rotor meets housing bearing collar",
    )
    ctx.expect_contact(
        pull_chain,
        motor_housing,
        elem_a="pivot_barrel",
        elem_b="switch_pivot_boss",
        name="pull chain is mounted to the side switch boss",
    )

    blade_limits = blade_spin.motion_limits
    ctx.check(
        "blade assembly uses a continuous vertical axle",
        blade_spin.joint_type == ArticulationType.CONTINUOUS
        and blade_spin.axis == (0.0, 0.0, 1.0)
        and blade_limits is not None
        and blade_limits.lower is None
        and blade_limits.upper is None,
        details=(
            f"type={blade_spin.joint_type}, axis={blade_spin.axis}, "
            f"limits={blade_limits}"
        ),
    )

    chain_limits = chain_pivot.motion_limits
    ctx.check(
        "pull chain pivots from the housing side",
        chain_pivot.joint_type == ArticulationType.REVOLUTE
        and chain_pivot.axis == (0.0, 1.0, 0.0)
        and chain_limits is not None
        and chain_limits.lower == 0.0
        and chain_limits.upper is not None
        and chain_limits.upper > 0.5,
        details=(
            f"type={chain_pivot.joint_type}, axis={chain_pivot.axis}, "
            f"limits={chain_limits}"
        ),
    )

    blade_spin_center = ctx.part_world_position(blade_assembly)
    blade_radii = []
    if blade_spin_center is not None:
        for blade_index in range(5):
            blade_radii.append(
                _aabb_max_planar_radius(
                    ctx.part_element_world_aabb(blade_assembly, elem=f"blade_{blade_index}"),
                    origin_xy=(blade_spin_center[0], blade_spin_center[1]),
                )
            )
    valid_blade_radii = [radius for radius in blade_radii if radius is not None]
    max_blade_radius = max(valid_blade_radii) if valid_blade_radii else None
    ctx.check(
        "fan has realistic five-blade span",
        len(valid_blade_radii) == 5
        and max_blade_radius is not None
        and 0.64 <= max_blade_radius <= 0.76,
        details=f"blade_radii={blade_radii}, max_radius={max_blade_radius}",
    )

    rest_blade_center = _aabb_center(
        ctx.part_element_world_aabb(blade_assembly, elem="blade_0")
    )
    with ctx.pose({blade_spin: math.pi / 2.0}):
        quarter_turn_center = _aabb_center(
            ctx.part_element_world_aabb(blade_assembly, elem="blade_0")
        )
    ctx.check(
        "blade visuals move under spin articulation",
        rest_blade_center is not None
        and quarter_turn_center is not None
        and abs(rest_blade_center[0] - quarter_turn_center[0]) > 0.18
        and abs(rest_blade_center[1] - quarter_turn_center[1]) > 0.18,
        details=f"rest={rest_blade_center}, quarter_turn={quarter_turn_center}",
    )

    rest_handle_center = _aabb_center(
        ctx.part_element_world_aabb(pull_chain, elem="pull_handle")
    )
    with ctx.pose({chain_pivot: chain_limits.upper if chain_limits is not None else 0.0}):
        swung_handle_center = _aabb_center(
            ctx.part_element_world_aabb(pull_chain, elem="pull_handle")
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no overlaps when pull chain swings outward"
        )
    ctx.check(
        "pull handle swings outward from housing",
        rest_handle_center is not None
        and swung_handle_center is not None
        and swung_handle_center[0] > rest_handle_center[0] + 0.04
        and swung_handle_center[2] < rest_handle_center[2] - 0.01,
        details=f"rest={rest_handle_center}, swung={swung_handle_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
