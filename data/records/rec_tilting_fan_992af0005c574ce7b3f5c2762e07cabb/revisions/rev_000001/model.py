from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    FanRotorShroud,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


PIVOT_Z = 1.02
FRONT_GUARD_X = 0.100
REAR_GUARD_X = -0.105
GUARD_OUTER_R = 0.340
ROTOR_R = 0.280


def _ring_x(x: float, radius: float, tube: float, *, segments: int = 88) -> MeshGeometry:
    """A torus in the fan guard plane, with its normal along local +X."""

    return (
        TorusGeometry(radius, tube, radial_segments=16, tubular_segments=segments)
        .rotate_y(math.pi / 2.0)
        .translate(x, 0.0, 0.0)
    )


def _tube(points, radius: float, *, samples: int = 2, radial_segments: int = 10) -> MeshGeometry:
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples,
        radial_segments=radial_segments,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )


def _radial_guard(x: float, *, count: int, inner: float, outer: float, tube: float, phase: float) -> MeshGeometry:
    geom = MeshGeometry()
    for idx in range(count):
        a = phase + idx * 2.0 * math.pi / count
        geom.merge(
            _tube(
                [
                    (x, inner * math.cos(a), inner * math.sin(a)),
                    (x, outer * math.cos(a), outer * math.sin(a)),
                ],
                tube,
                samples=1,
                radial_segments=8,
            )
        )
    return geom


def _front_guard_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    geom.merge(_ring_x(FRONT_GUARD_X, GUARD_OUTER_R, 0.010))
    geom.merge(_ring_x(FRONT_GUARD_X, 0.238, 0.006, segments=72))
    geom.merge(_ring_x(FRONT_GUARD_X, 0.105, 0.006, segments=60))
    geom.merge(_radial_guard(FRONT_GUARD_X, count=16, inner=0.105, outer=GUARD_OUTER_R, tube=0.0048, phase=0.0))
    return geom


def _rear_guard_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    geom.merge(_ring_x(REAR_GUARD_X, GUARD_OUTER_R, 0.009))
    geom.merge(_ring_x(REAR_GUARD_X, 0.245, 0.0055, segments=72))
    geom.merge(_ring_x(REAR_GUARD_X, 0.118, 0.006, segments=60))
    geom.merge(_radial_guard(REAR_GUARD_X, count=12, inner=0.118, outer=GUARD_OUTER_R, tube=0.0045, phase=math.pi / 12.0))
    return geom


def _guard_tie_geometry() -> MeshGeometry:
    geom = MeshGeometry()
    for idx in range(8):
        a = idx * 2.0 * math.pi / 8.0 + math.pi / 8.0
        y = GUARD_OUTER_R * math.cos(a)
        z = GUARD_OUTER_R * math.sin(a)
        geom.merge(
            _tube(
                [(REAR_GUARD_X, y, z), (FRONT_GUARD_X, y, z)],
                0.006,
                samples=1,
                radial_segments=8,
            )
        )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_tilting_fan")

    model.material("dark_powdercoat", rgba=(0.035, 0.040, 0.038, 1.0))
    model.material("galvanized_steel", rgba=(0.58, 0.61, 0.58, 1.0))
    model.material("safety_yellow", rgba=(1.0, 0.70, 0.05, 1.0))
    model.material("rubber_black", rgba=(0.012, 0.012, 0.010, 1.0))
    model.material("bronze_wear", rgba=(0.76, 0.48, 0.20, 1.0))
    model.material("service_blue", rgba=(0.05, 0.18, 0.36, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.86, 0.56, 0.060)),
        origin=Origin(xyz=(0.00, 0.00, 0.045)),
        material="dark_powdercoat",
        name="skid_plate",
    )
    base.visual(
        Box((0.92, 0.070, 0.060)),
        origin=Origin(xyz=(0.00, 0.235, 0.035)),
        material="dark_powdercoat",
        name="rail_0",
    )
    base.visual(
        Box((0.92, 0.070, 0.060)),
        origin=Origin(xyz=(0.00, -0.235, 0.035)),
        material="dark_powdercoat",
        name="rail_1",
    )
    for i, (x, y) in enumerate(((-0.34, -0.21), (-0.34, 0.21), (0.34, -0.21), (0.34, 0.21))):
        base.visual(
            Cylinder(radius=0.055, length=0.025),
            origin=Origin(xyz=(x, y, 0.0025)),
            material="rubber_black",
            name=f"foot_{i}",
        )
    for idx, y in enumerate((-0.345, 0.345)):
        base.visual(
            Box((0.240, 0.180, 0.050)),
            origin=Origin(xyz=(-0.020, y, 0.100)),
            material="dark_powdercoat",
            name=f"outrigger_{idx}",
        )

    # Welded yoke cheeks and a cross tube make the tilt pivot a real structure.
    for idx, y in enumerate((-0.410, 0.410)):
        base.visual(
            Box((0.110, 0.080, 0.950)),
            origin=Origin(xyz=(-0.020, y, 0.550)),
            material="dark_powdercoat",
            name=f"yoke_cheek_{idx}",
        )
        base.visual(
            Cylinder(radius=0.100, length=0.080),
            origin=Origin(xyz=(-0.020, y, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="galvanized_steel",
            name=f"bearing_boss_{idx}",
        )
        base.visual(
            Box((0.145, 0.012, 0.145)),
            origin=Origin(xyz=(-0.020, -0.376 if y < 0.0 else 0.376, PIVOT_Z)),
            material="bronze_wear",
            name="wear_pad_0" if idx == 0 else "wear_pad_1",
        )
        base.visual(
            Box((0.190, 0.050, 0.100)),
            origin=Origin(xyz=(0.120, y, 0.230)),
            material="dark_powdercoat",
            name=f"gusset_foot_{idx}",
        )
    base.visual(
        Box((0.095, 0.760, 0.075)),
        origin=Origin(xyz=(-0.070, 0.0, 0.500)),
        material="dark_powdercoat",
        name="lower_crossbar",
    )
    base.visual(
        Box((0.300, 0.200, 0.050)),
        origin=Origin(xyz=(0.230, 0.0, 0.100)),
        material="service_blue",
        name="service_tray",
    )
    base.visual(
        Box((0.230, 0.030, 0.030)),
        origin=Origin(xyz=(0.230, 0.085, 0.140)),
        material="galvanized_steel",
        name="spare_rail_0",
    )
    base.visual(
        Box((0.230, 0.030, 0.030)),
        origin=Origin(xyz=(0.230, -0.085, 0.140)),
        material="galvanized_steel",
        name="spare_rail_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_geometry(_front_guard_geometry(), "front_guard"),
        material="galvanized_steel",
        name="front_guard",
    )
    head.visual(
        mesh_from_geometry(_rear_guard_geometry(), "rear_guard"),
        material="galvanized_steel",
        name="rear_guard",
    )
    head.visual(
        mesh_from_geometry(_guard_tie_geometry(), "guard_ties"),
        material="galvanized_steel",
        name="guard_ties",
    )
    head.visual(
        Cylinder(radius=0.132, length=0.220),
        origin=Origin(xyz=(-0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_powdercoat",
        name="motor_can",
    )
    head.visual(
        Cylinder(radius=0.058, length=0.045),
        origin=Origin(xyz=(-0.0575, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="galvanized_steel",
        name="bearing_nose",
    )
    head.visual(
        Cylinder(radius=0.118, length=0.022),
        origin=Origin(xyz=(-0.311, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="service_blue",
        name="service_cover",
    )
    for idx, (y, z) in enumerate(((0.070, 0.070), (-0.070, 0.070), (0.070, -0.070), (-0.070, -0.070))):
        head.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(-0.329, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="galvanized_steel",
            name=f"cover_screw_{idx}",
        )

    for idx, y in enumerate((-0.330, 0.330)):
        head.visual(
            Cylinder(radius=0.056, length=0.080),
            origin=Origin(xyz=(-0.020, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="galvanized_steel",
            name="side_pin_0" if idx == 0 else "side_pin_1",
        )
        head.visual(
            Box((0.270, 0.060, 0.095)),
            origin=Origin(xyz=(-0.010, y, 0.0)),
            material="dark_powdercoat",
            name=f"side_bridge_{idx}",
        )
        head.visual(
            Cylinder(radius=0.080, length=0.020),
            origin=Origin(xyz=(-0.020, -0.360 if y < 0.0 else 0.360, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="bronze_wear",
            name=f"pivot_washer_{idx}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                ROTOR_R,
                0.072,
                5,
                thickness=0.055,
                blade_pitch_deg=34.0,
                blade_sweep_deg=28.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=16.0, camber=0.18),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.022, rear_collar_radius=0.065, bore_diameter=0.026),
                shroud=FanRotorShroud(thickness=0.012, depth=0.042, clearance=0.004, lip_depth=0.004),
            ),
            "rotor_blades",
        ),
        material="safety_yellow",
        name="rotor_blades",
    )
    rotor.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material="galvanized_steel",
        name="drive_shaft",
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.65, lower=-0.45, upper=0.55),
        motion_properties=MotionProperties(damping=4.0, friction=1.5),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=90.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("base_to_head")
    spin = object_model.get_articulation("head_to_rotor")

    ctx.check("tilt joint is revolute", tilt.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("rotor joint is continuous", spin.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check(
        "tilt has serviceable range",
        tilt.motion_limits is not None and tilt.motion_limits.lower <= -0.40 and tilt.motion_limits.upper >= 0.50,
        details=f"limits={tilt.motion_limits}",
    )

    ctx.expect_gap(
        base,
        head,
        axis="y",
        positive_elem="wear_pad_1",
        negative_elem="side_pin_1",
        max_gap=0.002,
        max_penetration=0.0,
        name="positive side pivot seats against wear pad",
    )
    ctx.expect_gap(
        head,
        base,
        axis="y",
        positive_elem="side_pin_0",
        negative_elem="wear_pad_0",
        max_gap=0.002,
        max_penetration=0.0,
        name="negative side pivot seats against wear pad",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="rotor_blades",
        outer_elem="front_guard",
        margin=0.005,
        name="rotor diameter is guarded in front projection",
    )
    ctx.expect_gap(
        head,
        rotor,
        axis="x",
        positive_elem="front_guard",
        negative_elem="rotor_blades",
        min_gap=0.030,
        name="front guard clears spinning rotor",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="rotor_blades",
        negative_elem="rear_guard",
        min_gap=0.025,
        name="rear guard clears spinning rotor",
    )

    front_rest = ctx.part_element_world_aabb(head, elem="front_guard")
    with ctx.pose({tilt: 0.45}):
        front_tilted = ctx.part_element_world_aabb(head, elem="front_guard")
    if front_rest is not None and front_tilted is not None:
        rest_z = (front_rest[0][2] + front_rest[1][2]) * 0.5
        tilted_z = (front_tilted[0][2] + front_tilted[1][2]) * 0.5
    else:
        rest_z = tilted_z = None
    ctx.check(
        "positive tilt lifts guarded nose",
        rest_z is not None and tilted_z is not None and tilted_z > rest_z + 0.025,
        details=f"rest_z={rest_z}, tilted_z={tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()
