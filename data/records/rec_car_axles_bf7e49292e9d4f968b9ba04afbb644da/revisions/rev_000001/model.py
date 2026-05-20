from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _x_aligned_cylinder(
    radius: float,
    length: float,
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _build_stub_axle(
    model: ArticulatedObject,
    side: str,
    side_sign: float,
    steel,
    machined_steel,
) -> None:
    stub = model.part(f"{side}_stub_axle")
    dir_x = side_sign

    geom, origin = _x_aligned_cylinder(0.056, 0.030, xyz=(dir_x * 0.015, 0.0, 0.0))
    stub.visual(geom, origin=origin, material=machined_steel, name="inner_flange")

    geom, origin = _x_aligned_cylinder(0.038, 0.220, xyz=(dir_x * 0.140, 0.0, 0.0))
    stub.visual(geom, origin=origin, material=steel, name="shaft_tube")

    geom, origin = _x_aligned_cylinder(0.046, 0.022, xyz=(dir_x * 0.236, 0.0, 0.0))
    stub.visual(geom, origin=origin, material=machined_steel, name="outer_collar")

    geom, origin = _x_aligned_cylinder(0.029, 0.040, xyz=(dir_x * 0.270, 0.0, 0.0))
    stub.visual(geom, origin=origin, material=machined_steel, name="outer_spindle")

    stub.visual(
        Box((0.026, 0.050, 0.036)),
        origin=Origin(xyz=(dir_x * 0.095, 0.0, 0.050)),
        material=machined_steel,
        name="rotation_ear",
    )

    stub.inertial = Inertial.from_geometry(
        Box((0.310, 0.120, 0.140)),
        mass=18.0,
        origin=Origin(xyz=(dir_x * 0.155, 0.0, 0.0)),
    )


def _build_portal_hub(
    model: ArticulatedObject,
    side: str,
    side_sign: float,
    cast_iron,
    machined_steel,
) -> None:
    hub = model.part(f"{side}_portal_hub")
    dir_x = side_sign

    geom, origin = _x_aligned_cylinder(0.040, 0.030, xyz=(dir_x * 0.015, 0.0, 0.0))
    hub.visual(geom, origin=origin, material=machined_steel, name="input_collar")

    hub.visual(
        Box((0.064, 0.150, 0.220)),
        origin=Origin(xyz=(dir_x * 0.052, 0.0, 0.110)),
        material=cast_iron,
        name="upright_case",
    )
    hub.visual(
        Box((0.150, 0.230, 0.170)),
        origin=Origin(xyz=(dir_x * 0.135, 0.0, 0.165)),
        material=cast_iron,
        name="reduction_case",
    )

    geom, origin = _x_aligned_cylinder(0.102, 0.082, xyz=(dir_x * 0.196, 0.0, 0.165))
    hub.visual(geom, origin=origin, material=cast_iron, name="gear_cover")

    geom, origin = _x_aligned_cylinder(0.066, 0.024, xyz=(dir_x * 0.245, 0.0, 0.165))
    hub.visual(geom, origin=origin, material=machined_steel, name="wheel_flange")

    hub.visual(
        Box((0.090, 0.120, 0.050)),
        origin=Origin(xyz=(dir_x * 0.095, 0.0, 0.255)),
        material=cast_iron,
        name="top_cap",
    )

    hub.inertial = Inertial.from_geometry(
        Box((0.290, 0.260, 0.320)),
        mass=42.0,
        origin=Origin(xyz=(dir_x * 0.145, 0.0, 0.155)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_axle")

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.29, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.40, 0.42, 0.44, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.63, 0.65, 0.67, 1.0))

    housing = model.part("differential_housing")
    housing.visual(
        Box((0.540, 0.260, 0.240)),
        material=cast_iron,
        name="main_case",
    )
    housing.visual(
        Box((0.260, 0.200, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=cast_iron,
        name="top_cover",
    )
    housing.visual(
        Box((0.360, 0.080, 0.170)),
        origin=Origin(xyz=(0.0, 0.170, 0.000)),
        material=machined_steel,
        name="front_cover",
    )
    housing.visual(
        Box((0.400, 0.055, 0.210)),
        origin=Origin(xyz=(0.0, -0.1575, -0.005)),
        material=steel,
        name="rear_rib",
    )
    housing.visual(
        Box((0.260, 0.180, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material=cast_iron,
        name="bottom_sump",
    )
    geom, origin = _x_aligned_cylinder(0.082, 0.080, xyz=(0.310, 0.0, 0.0))
    housing.visual(geom, origin=origin, material=machined_steel, name="left_side_flange")
    geom, origin = _x_aligned_cylinder(0.082, 0.080, xyz=(-0.310, 0.0, 0.0))
    housing.visual(geom, origin=origin, material=machined_steel, name="right_side_flange")
    housing.visual(
        Box((0.100, 0.100, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=machined_steel,
        name="breather_cap",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=machined_steel,
        name="breather_neck",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.760, 0.360, 0.500)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    _build_stub_axle(model, "left", 1.0, steel, machined_steel)
    _build_stub_axle(model, "right", -1.0, steel, machined_steel)
    _build_portal_hub(model, "left", 1.0, cast_iron, machined_steel)
    _build_portal_hub(model, "right", -1.0, cast_iron, machined_steel)

    spin_limits = MotionLimits(effort=800.0, velocity=12.0, lower=-2.0 * pi, upper=2.0 * pi)

    model.articulation(
        "housing_to_left_stub",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="left_stub_axle",
        origin=Origin(xyz=(0.350, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "housing_to_right_stub",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="right_stub_axle",
        origin=Origin(xyz=(-0.350, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "left_stub_to_left_portal_hub",
        ArticulationType.REVOLUTE,
        parent="left_stub_axle",
        child="left_portal_hub",
        origin=Origin(xyz=(0.290, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=spin_limits,
    )
    model.articulation(
        "right_stub_to_right_portal_hub",
        ArticulationType.REVOLUTE,
        parent="right_stub_axle",
        child="right_portal_hub",
        origin=Origin(xyz=(-0.290, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=spin_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("differential_housing")
    left_stub = object_model.get_part("left_stub_axle")
    right_stub = object_model.get_part("right_stub_axle")
    left_hub = object_model.get_part("left_portal_hub")
    right_hub = object_model.get_part("right_portal_hub")

    left_stub_spin = object_model.get_articulation("housing_to_left_stub")
    right_stub_spin = object_model.get_articulation("housing_to_right_stub")
    left_hub_spin = object_model.get_articulation("left_stub_to_left_portal_hub")
    right_hub_spin = object_model.get_articulation("right_stub_to_right_portal_hub")

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

    for joint_name, joint in (
        ("housing_to_left_stub", left_stub_spin),
        ("housing_to_right_stub", right_stub_spin),
        ("left_stub_to_left_portal_hub", left_hub_spin),
        ("right_stub_to_right_portal_hub", right_hub_spin),
    ):
        axis = joint.axis
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} uses a horizontal axle axis",
            axis is not None and abs(abs(axis[0]) - 1.0) < 1e-9 and abs(axis[1]) < 1e-9 and abs(axis[2]) < 1e-9,
            details=f"axis={axis}",
        )
        ctx.check(
            f"{joint_name} has spin range through zero",
            limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < 0.0 < limits.upper,
            details=f"limits={limits}",
        )

    ctx.expect_contact(
        left_stub,
        housing,
        elem_a="inner_flange",
        elem_b="left_side_flange",
        name="left stub axle mounts to the left housing flange",
    )
    ctx.expect_contact(
        right_stub,
        housing,
        elem_a="inner_flange",
        elem_b="right_side_flange",
        name="right stub axle mounts to the right housing flange",
    )
    ctx.expect_contact(
        left_hub,
        left_stub,
        elem_a="input_collar",
        elem_b="outer_spindle",
        name="left portal hub seats on the left stub spindle",
    )
    ctx.expect_contact(
        right_hub,
        right_stub,
        elem_a="input_collar",
        elem_b="outer_spindle",
        name="right portal hub seats on the right stub spindle",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))

    left_flange_center = _aabb_center(ctx.part_element_world_aabb(left_hub, elem="wheel_flange"))
    right_flange_center = _aabb_center(ctx.part_element_world_aabb(right_hub, elem="wheel_flange"))
    left_spindle_center = _aabb_center(ctx.part_element_world_aabb(left_stub, elem="outer_spindle"))
    right_spindle_center = _aabb_center(ctx.part_element_world_aabb(right_stub, elem="outer_spindle"))

    ctx.check(
        "left portal output sits above the left stub axle centerline",
        left_flange_center is not None
        and left_spindle_center is not None
        and left_flange_center[2] > left_spindle_center[2] + 0.12,
        details=f"left_flange_center={left_flange_center}, left_spindle_center={left_spindle_center}",
    )
    ctx.check(
        "right portal output sits above the right stub axle centerline",
        right_flange_center is not None
        and right_spindle_center is not None
        and right_flange_center[2] > right_spindle_center[2] + 0.12,
        details=f"right_flange_center={right_flange_center}, right_spindle_center={right_spindle_center}",
    )

    left_stub_ear_rest = _aabb_center(ctx.part_element_world_aabb(left_stub, elem="rotation_ear"))
    with ctx.pose({left_stub_spin: pi / 2.0}):
        left_stub_ear_quarter_turn = _aabb_center(ctx.part_element_world_aabb(left_stub, elem="rotation_ear"))
    ctx.check(
        "left stub axle visibly rotates about its own longitudinal axis",
        left_stub_ear_rest is not None
        and left_stub_ear_quarter_turn is not None
        and abs(left_stub_ear_rest[2] - left_stub_ear_quarter_turn[2]) > 0.035
        and abs(left_stub_ear_rest[1] - left_stub_ear_quarter_turn[1]) > 0.035,
        details=f"rest={left_stub_ear_rest}, quarter_turn={left_stub_ear_quarter_turn}",
    )

    left_hub_flange_rest = _aabb_center(ctx.part_element_world_aabb(left_hub, elem="wheel_flange"))
    with ctx.pose({left_hub_spin: pi / 2.0}):
        left_hub_flange_quarter_turn = _aabb_center(ctx.part_element_world_aabb(left_hub, elem="wheel_flange"))
    ctx.check(
        "left portal hub visibly spins around the outer stub axle",
        left_hub_flange_rest is not None
        and left_hub_flange_quarter_turn is not None
        and abs(left_hub_flange_rest[2] - left_hub_flange_quarter_turn[2]) > 0.10
        and abs(left_hub_flange_rest[1] - left_hub_flange_quarter_turn[1]) > 0.10,
        details=f"rest={left_hub_flange_rest}, quarter_turn={left_hub_flange_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
