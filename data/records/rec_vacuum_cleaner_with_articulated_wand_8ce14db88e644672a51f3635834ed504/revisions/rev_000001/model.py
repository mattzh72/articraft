from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _yz_section(
    *,
    x: float,
    width: float,
    height: float,
    radius: float,
    center_z: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + center_z) for y, z in rounded_rect_profile(width, height, radius)]


def _wheel_part(model: ArticulatedObject, name: str, tire_material, hub_material):
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.095, length=0.050),
        origin=Origin(rpy=(radians(90.0), 0.0, 0.0)),
        material=tire_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.052, length=0.056),
        origin=Origin(rpy=(radians(90.0), 0.0, 0.0)),
        material=hub_material,
        name="hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.050),
        mass=1.1,
        origin=Origin(rpy=(radians(90.0), 0.0, 0.0)),
    )
    return wheel


def _tube_along_x(radius: float, length: float, *, material, name: str | None = None):
    return {
        "geometry": Cylinder(radius=radius, length=length),
        "origin": Origin(xyz=(length * 0.5, 0.0, 0.0), rpy=(0.0, radians(90.0), 0.0)),
        "material": material,
        "name": name,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    body_plastic = model.material("body_plastic", rgba=(0.72, 0.14, 0.10, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.29, 0.31, 0.34, 1.0))
    steel = model.material("steel", rgba=(0.75, 0.77, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    bin_tint = model.material("bin_tint", rgba=(0.54, 0.72, 0.82, 0.45))

    body = model.part("body")
    shell_geom = section_loft(
        [
            _yz_section(x=-0.22, width=0.24, height=0.22, radius=0.045, center_z=0.180),
            _yz_section(x=-0.06, width=0.29, height=0.26, radius=0.055, center_z=0.186),
            _yz_section(x=0.10, width=0.29, height=0.24, radius=0.050, center_z=0.172),
            _yz_section(x=0.25, width=0.19, height=0.17, radius=0.040, center_z=0.145),
        ]
    )
    body.visual(mesh_from_geometry(shell_geom, "vacuum_body_shell"), material=body_plastic, name="shell")
    body.visual(
        Box((0.34, 0.23, 0.050)),
        origin=Origin(xyz=(-0.02, 0.0, 0.075)),
        material=dark_trim,
        name="base_skid",
    )
    body.visual(
        Cylinder(radius=0.078, length=0.175),
        origin=Origin(xyz=(0.03, 0.0, 0.255), rpy=(radians(90.0), 0.0, 0.0)),
        material=bin_tint,
        name="dust_bin",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.070),
        origin=Origin(xyz=(0.305, 0.0, 0.225), rpy=(0.0, radians(90.0), 0.0)),
        material=graphite,
        name="intake_collar",
    )
    body.visual(
        Box((0.150, 0.120, 0.085)),
        origin=Origin(xyz=(0.230, 0.0, 0.208)),
        material=graphite,
        name="front_motor_housing",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.072),
        origin=Origin(xyz=(-0.175, 0.0, 0.286), rpy=(0.0, radians(90.0), 0.0)),
        material=graphite,
        name="rear_handle_bridge",
    )
    carry_handle = tube_from_spline_points(
        [
            (-0.205, -0.060, 0.240),
            (-0.200, -0.055, 0.287),
            (-0.155, -0.020, 0.322),
            (-0.095, 0.0, 0.334),
            (-0.035, 0.020, 0.322),
            (0.015, 0.055, 0.286),
            (0.022, 0.060, 0.243),
        ],
        radius=0.012,
        samples_per_segment=14,
        radial_segments=16,
    )
    body.visual(mesh_from_geometry(carry_handle, "vacuum_carry_handle"), material=dark_trim, name="carry_handle")
    body.visual(
        Box((0.070, 0.084, 0.074)),
        origin=Origin(xyz=(-0.055, 0.142, 0.095)),
        material=graphite,
        name="left_wheel_pod",
    )
    body.visual(
        Box((0.070, 0.084, 0.074)),
        origin=Origin(xyz=(-0.055, -0.142, 0.095)),
        material=graphite,
        name="right_wheel_pod",
    )
    body.visual(
        Box((0.082, 0.092, 0.040)),
        origin=Origin(xyz=(0.246, 0.0, 0.074)),
        material=graphite,
        name="front_roller_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.56, 0.30, 0.33)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
    )

    left_wheel = _wheel_part(model, "left_wheel", rubber, steel)
    right_wheel = _wheel_part(model, "right_wheel", rubber, steel)

    front_roller = model.part("front_roller")
    front_roller.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.028), rpy=(radians(90.0), 0.0, 0.0)),
        material=rubber,
        name="roller",
    )
    front_roller.visual(
        Box((0.018, 0.110, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=graphite,
        name="fork",
    )
    front_roller.visual(
        Box((0.030, 0.056, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=graphite,
        name="hanger",
    )
    front_roller.inertial = Inertial.from_geometry(
        Box((0.09, 0.12, 0.11)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    primary_wand = model.part("primary_wand")
    primary_wand.visual(
        geometry=Cylinder(radius=0.020, length=0.515),
        origin=Origin(xyz=(0.2575, 0.0, 0.0), rpy=(0.0, radians(90.0), 0.0)),
        material=steel,
        name="primary_tube",
    )
    primary_wand.visual(
        Cylinder(radius=0.029, length=0.056),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, radians(90.0), 0.0)),
        material=graphite,
        name="proximal_cuff",
    )
    primary_wand.visual(
        Cylinder(radius=0.024, length=0.028),
        origin=Origin(xyz=(0.506, 0.0, 0.0), rpy=(0.0, radians(90.0), 0.0)),
        material=graphite,
        name="distal_socket",
    )
    primary_wand.visual(
        Box((0.032, 0.010, 0.050)),
        origin=Origin(xyz=(0.524, 0.020, 0.0)),
        material=graphite,
        name="left_elbow_cheek",
    )
    primary_wand.visual(
        Box((0.032, 0.010, 0.050)),
        origin=Origin(xyz=(0.524, -0.020, 0.0)),
        material=graphite,
        name="right_elbow_cheek",
    )
    primary_wand.inertial = Inertial.from_geometry(
        Box((0.58, 0.07, 0.07)),
        mass=1.1,
        origin=Origin(xyz=(0.29, 0.0, 0.0)),
    )

    secondary_wand = model.part("secondary_wand")
    secondary_wand.visual(**_tube_along_x(0.018, 0.246, material=steel, name="secondary_tube"))
    secondary_wand.visual(
        Box((0.032, 0.024, 0.042)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=graphite,
        name="elbow_knuckle",
    )
    secondary_wand.visual(
        Cylinder(radius=0.011, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(radians(90.0), 0.0, 0.0)),
        material=graphite,
        name="elbow_pin_barrel",
    )
    secondary_wand.visual(
        Box((0.056, 0.012, 0.040)),
        origin=Origin(xyz=(0.242, 0.030, 0.0)),
        material=graphite,
        name="left_yoke_cheek",
    )
    secondary_wand.visual(
        Box((0.056, 0.012, 0.040)),
        origin=Origin(xyz=(0.242, -0.030, 0.0)),
        material=graphite,
        name="right_yoke_cheek",
    )
    secondary_wand.visual(
        Box((0.050, 0.060, 0.014)),
        origin=Origin(xyz=(0.228, 0.0, 0.017)),
        material=graphite,
        name="yoke_bridge",
    )
    secondary_wand.inertial = Inertial.from_geometry(
        Box((0.31, 0.08, 0.08)),
        mass=0.65,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Box((0.300, 0.100, 0.028)),
        origin=Origin(xyz=(0.188, 0.0, -0.034)),
        material=graphite,
        name="head_shell",
    )
    nozzle.visual(
        Box((0.315, 0.074, 0.010)),
        origin=Origin(xyz=(0.198, 0.0, -0.049)),
        material=dark_trim,
        name="suction_lip",
    )
    nozzle.visual(
        Box((0.092, 0.032, 0.054)),
        origin=Origin(xyz=(0.052, 0.0, -0.004)),
        material=graphite,
        name="neck_block",
    )
    nozzle.visual(
        Cylinder(radius=0.012, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(radians(90.0), 0.0, 0.0)),
        material=graphite,
        name="hinge_pin",
    )
    nozzle.visual(
        Box((0.034, 0.030, 0.020)),
        origin=Origin(xyz=(0.016, 0.0, 0.016)),
        material=graphite,
        name="hinge_knuckle",
    )
    nozzle.visual(
        Box((0.128, 0.050, 0.016)),
        origin=Origin(xyz=(0.102, 0.0, -0.022)),
        material=dark_trim,
        name="air_channel",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.35, 0.10, 0.08)),
        mass=0.95,
        origin=Origin(xyz=(0.175, 0.0, -0.015)),
    )

    model.articulation(
        "body_to_left_wheel",
        ArticulationType.FIXED,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(-0.055, 0.212, 0.095)),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.FIXED,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-0.055, -0.212, 0.095)),
    )
    model.articulation(
        "body_to_front_roller",
        ArticulationType.FIXED,
        parent=body,
        child=front_roller,
        origin=Origin(xyz=(0.246, 0.0, 0.054)),
    )
    model.articulation(
        "body_to_primary_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=primary_wand,
        origin=Origin(xyz=(0.343, 0.0, 0.225), rpy=(0.0, 0.17, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=-0.45, upper=0.75),
    )
    model.articulation(
        "primary_to_secondary_wand",
        ArticulationType.REVOLUTE,
        parent=primary_wand,
        child=secondary_wand,
        origin=Origin(xyz=(0.540, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=11.0, velocity=2.0, lower=-0.65, upper=1.05),
    )
    model.articulation(
        "secondary_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=secondary_wand,
        child=nozzle,
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(0.0, -0.18, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-0.45, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    primary_wand = object_model.get_part("primary_wand")
    secondary_wand = object_model.get_part("secondary_wand")
    nozzle = object_model.get_part("nozzle")
    shoulder = object_model.get_articulation("body_to_primary_wand")
    elbow = object_model.get_articulation("primary_to_secondary_wand")
    nozzle_pitch = object_model.get_articulation("secondary_to_nozzle")

    def _aabb_center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def _aabb_center_x(aabb) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    body_aabb = ctx.part_world_aabb(body)
    nozzle_aabb = ctx.part_world_aabb(nozzle)
    body_center_z = _aabb_center_z(body_aabb)
    nozzle_center_z = _aabb_center_z(nozzle_aabb)

    ctx.check(
        "canister body rides higher than the floor nozzle at rest",
        body_center_z is not None
        and nozzle_center_z is not None
        and body_center_z > nozzle_center_z + 0.08,
        details=f"body_center_z={body_center_z}, nozzle_center_z={nozzle_center_z}",
    )
    ctx.expect_gap(
        nozzle,
        body,
        axis="x",
        min_gap=0.120,
        name="floor nozzle projects ahead of the canister body",
    )
    ctx.expect_gap(
        primary_wand,
        body,
        axis="x",
        min_gap=-0.020,
        max_gap=0.120,
        name="primary wand starts near the body intake",
    )

    rest_nozzle = ctx.part_world_position(nozzle)
    with ctx.pose({shoulder: 0.55, elbow: 0.0, nozzle_pitch: 0.35}):
        extended_secondary_aabb = ctx.part_world_aabb(secondary_wand)
    with ctx.pose({shoulder: 0.55, elbow: 0.85, nozzle_pitch: 0.35}):
        lifted_nozzle = ctx.part_world_position(nozzle)
        folded_secondary_aabb = ctx.part_world_aabb(secondary_wand)

    ctx.check(
        "wand joints lift the floor nozzle",
        rest_nozzle is not None
        and lifted_nozzle is not None
        and lifted_nozzle[2] > rest_nozzle[2] + 0.18,
        details=f"rest_nozzle={rest_nozzle}, lifted_nozzle={lifted_nozzle}",
    )
    ctx.check(
        "folded wand pulls the short secondary link rearward",
        _aabb_center_x(extended_secondary_aabb) is not None
        and _aabb_center_x(folded_secondary_aabb) is not None
        and _aabb_center_x(folded_secondary_aabb) < _aabb_center_x(extended_secondary_aabb) - 0.05,
        details=(
            f"extended_secondary_center_x={_aabb_center_x(extended_secondary_aabb)}, "
            f"folded_secondary_center_x={_aabb_center_x(folded_secondary_aabb)}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
