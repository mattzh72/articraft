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
    LatheGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


HEAD_Z = 1.30


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _cyl_x(center: tuple[float, float, float], *, radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(center: tuple[float, float, float], *, radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _cyl_z(center: tuple[float, float, float], *, radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(xyz=center)


def _radial_spoke_origin(
    angle: float,
    *,
    inner_radius: float,
    outer_radius: float,
    y: float,
) -> tuple[Cylinder, Origin]:
    length = outer_radius - inner_radius
    mid = (inner_radius + outer_radius) * 0.5
    x = math.cos(angle) * mid
    z = math.sin(angle) * mid
    # Cylinder local +Z is rotated into the fan-head XZ radial direction.
    return (
        Cylinder(radius=0.0042, length=length),
        Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0 - angle, 0.0)),
    )


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_tilting_fan")

    powder = _mat(model, "dark_powder_coated_aluminum", (0.09, 0.12, 0.12, 1.0))
    guard_finish = _mat(model, "graphite_polymer_guard", (0.02, 0.025, 0.025, 1.0))
    rubber = _mat(model, "black_epdm_seals", (0.005, 0.005, 0.004, 1.0))
    stainless = _mat(model, "brushed_stainless_hardware", (0.70, 0.72, 0.69, 1.0))
    blade = _mat(model, "matte_safety_yellow_blades", (0.88, 0.68, 0.12, 1.0))
    motor = _mat(model, "sealed_motor_black", (0.015, 0.017, 0.018, 1.0))
    warning = _mat(model, "amber_weatherproof_label", (1.0, 0.56, 0.10, 1.0))

    base = model.part("base")
    _add_box(base, "anchor_plinth", (0.62, 0.46, 0.060), (0.0, 0.0, 0.030), powder)
    _add_box(base, "drip_skirt", (0.68, 0.52, 0.022), (0.0, 0.0, 0.071), powder)
    _add_box(base, "raised_pedestal", (0.24, 0.22, 0.080), (0.0, 0.0, 0.122), powder)

    mast_geom, mast_origin = _cyl_z((0.0, 0.0, 0.530), radius=0.043, length=0.800)
    base.visual(mast_geom, origin=mast_origin, material=powder, name="sealed_mast")
    collar_geom, collar_origin = _cyl_z((0.0, 0.0, 0.160), radius=0.063, length=0.035)
    base.visual(collar_geom, origin=collar_origin, material=rubber, name="mast_gasket")
    top_collar_geom, top_collar_origin = _cyl_z((0.0, 0.0, 0.905), radius=0.058, length=0.050)
    base.visual(top_collar_geom, origin=top_collar_origin, material=stainless, name="upper_clamp")

    _add_box(base, "waterproof_control_box", (0.145, 0.070, 0.205), (0.0, -0.071, 0.470), powder)
    _add_box(base, "control_lid_gasket", (0.157, 0.012, 0.217), (0.0, -0.112, 0.470), rubber)
    _add_box(base, "sealed_switch_boot", (0.070, 0.014, 0.038), (0.0, -0.122, 0.515), rubber)
    gland_geom, gland_origin = _cyl_y((0.0, -0.123, 0.405), radius=0.022, length=0.035)
    base.visual(gland_geom, origin=gland_origin, material=rubber, name="cable_gland")
    _add_box(base, "warning_label", (0.090, 0.006, 0.030), (0.0, -0.121, 0.565), warning)

    # Outdoor yoke: all side pivot structure is welded back to the mast, so the
    # fan head has visible hard continuity through the trunnions and bearings.
    _add_box(base, "yoke_bridge", (0.86, 0.090, 0.055), (0.0, 0.0, 0.925), powder)
    for suffix, x in (("0", -0.430), ("1", 0.430)):
        _add_box(base, f"yoke_arm_{suffix}", (0.055, 0.092, 0.550), (x, 0.0, 1.075), powder)
        geom, org = _cyl_x((x * 0.935, 0.0, HEAD_Z), radius=0.058, length=0.082)
        base.visual(geom, origin=org, material=stainless, name=f"yoke_bearing_{suffix}")
        seal_geom, seal_org = _cyl_x((x * 0.905, 0.0, HEAD_Z), radius=0.063, length=0.012)
        base.visual(seal_geom, origin=seal_org, material=rubber, name=f"pivot_seal_{suffix}")

    for x in (-0.245, 0.245):
        for y in (-0.165, 0.165):
            washer, washer_origin = _cyl_z((x, y, 0.086), radius=0.024, length=0.008)
            base.visual(washer, origin=washer_origin, material=stainless, name=f"anchor_washer_{x}_{y}")
            bolt, bolt_origin = _cyl_z((x, y, 0.097), radius=0.014, length=0.014)
            base.visual(bolt, origin=bolt_origin, material=stainless, name=f"anchor_bolt_{x}_{y}")

    head = model.part("fan_head")
    # Shallow weatherproof duct with an overhanging front rain lip, built as a
    # lathed hollow shell and rotated so its axis is the fan's local +Y axis.
    shroud = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.300, -0.105),
            (0.325, -0.095),
            (0.325, 0.082),
            (0.348, 0.102),
            (0.342, 0.120),
        ],
        inner_profile=[
            (0.270, -0.085),
            (0.286, -0.070),
            (0.292, 0.075),
            (0.302, 0.102),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
    )
    head.visual(
        mesh_from_geometry(shroud, "rain_lipped_shroud"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=powder,
        name="rain_lipped_shroud",
    )

    for name, y, radius, tube in (
        ("front_guard_outer", 0.118, 0.302, 0.0065),
        ("front_guard_mid", 0.118, 0.205, 0.0045),
        ("front_guard_inner", 0.118, 0.078, 0.0045),
        ("rear_guard_outer", -0.092, 0.292, 0.0055),
        ("rear_guard_mid", -0.092, 0.198, 0.0040),
        ("rear_guard_inner", -0.092, 0.084, 0.0040),
    ):
        head.visual(
            mesh_from_geometry(TorusGeometry(radius, tube, radial_segments=20, tubular_segments=96), name),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=guard_finish,
            name=name,
        )

    for i in range(16):
        angle = 2.0 * math.pi * i / 16.0
        geom, org = _radial_spoke_origin(angle, inner_radius=0.062, outer_radius=0.314, y=0.118)
        head.visual(geom, origin=org, material=guard_finish, name=f"front_spoke_{i}")
    for i in range(12):
        angle = 2.0 * math.pi * (i + 0.5) / 12.0
        geom, org = _radial_spoke_origin(angle, inner_radius=0.084, outer_radius=0.288, y=-0.092)
        head.visual(geom, origin=org, material=guard_finish, name=f"rear_spoke_{i}")

    motor_geom, motor_origin = _cyl_y((0.0, -0.158, 0.0), radius=0.094, length=0.145)
    head.visual(motor_geom, origin=motor_origin, material=motor, name="sealed_motor_pod")
    hub_flange_geom, hub_flange_origin = _cyl_y((0.0, -0.072, 0.0), radius=0.070, length=0.028)
    head.visual(hub_flange_geom, origin=hub_flange_origin, material=stainless, name="motor_flange")
    shaft_geom, shaft_origin = _cyl_y((0.0, -0.021, 0.0), radius=0.017, length=0.082)
    head.visual(shaft_geom, origin=shaft_origin, material=stainless, name="drive_stub")

    for i, angle in enumerate((math.radians(55), math.radians(125), math.radians(235), math.radians(305))):
        r0 = 0.092
        r1 = 0.232
        mid = 0.5 * (r0 + r1)
        length = r1 - r0
        x = math.cos(angle) * mid
        z = math.sin(angle) * mid
        head.visual(
            Cylinder(radius=0.006, length=length),
            origin=Origin(xyz=(x, -0.112, z), rpy=(0.0, math.pi / 2.0 - angle, 0.0)),
            material=powder,
            name=f"motor_strut_{i}",
        )

    _add_box(head, "rain_visor", (0.730, 0.260, 0.022), (0.0, 0.020, 0.356), powder)
    _add_box(head, "front_drip_lip", (0.715, 0.022, 0.060), (0.0, 0.160, 0.320), powder)
    _add_box(head, "rear_drip_lip", (0.675, 0.020, 0.040), (0.0, -0.100, 0.330), powder)
    _add_box(head, "visor_rib_0", (0.026, 0.210, 0.180), (-0.250, 0.020, 0.265), powder)
    _add_box(head, "visor_rib_1", (0.026, 0.210, 0.180), (0.250, 0.020, 0.265), powder)

    for suffix, sign in (("0", -1.0), ("1", 1.0)):
        trunnion_geom, trunnion_origin = _cyl_x((sign * 0.390, 0.0, 0.0), radius=0.038, length=0.205)
        head.visual(trunnion_geom, origin=trunnion_origin, material=stainless, name=f"trunnion_{suffix}")
        boss_geom, boss_origin = _cyl_x((sign * 0.315, 0.0, 0.0), radius=0.056, length=0.050)
        head.visual(boss_geom, origin=boss_origin, material=powder, name=f"pivot_boss_{suffix}")
        knob_geom, knob_origin = _cyl_x((sign * 0.490, 0.0, 0.0), radius=0.052, length=0.038)
        head.visual(knob_geom, origin=knob_origin, material=guard_finish, name=f"tilt_knob_{suffix}")

    rotor = model.part("rotor")
    rotor_mesh = FanRotorGeometry(
        outer_radius=0.255,
        hub_radius=0.060,
        blade_count=5,
        thickness=0.052,
        blade_pitch_deg=32.0,
        blade_sweep_deg=24.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.15, tip_clearance=0.010),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.020, rear_collar_radius=0.050, bore_diameter=0.040),
    )
    rotor.visual(
        mesh_from_geometry(rotor_mesh, "rotor_blades"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blade,
        name="rotor_blades",
    )
    nose_geom, nose_origin = _cyl_y((0.0, 0.032, 0.0), radius=0.047, length=0.024)
    rotor.visual(nose_geom, origin=nose_origin, material=stainless, name="hub_nose")

    model.articulation(
        "tilt_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, HEAD_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.55, lower=-0.55, upper=0.55),
        motion_properties=MotionProperties(damping=0.9, friction=0.25),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=75.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("fan_head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("tilt_pivot")
    spin = object_model.get_articulation("rotor_spin")

    for suffix in ("0", "1"):
        ctx.allow_overlap(
            base,
            head,
            elem_a=f"yoke_bearing_{suffix}",
            elem_b=f"trunnion_{suffix}",
            reason="The stainless side trunnion is intentionally captured inside the solid bearing sleeve proxy.",
        )
        ctx.allow_overlap(
            base,
            head,
            elem_a=f"pivot_seal_{suffix}",
            elem_b=f"trunnion_{suffix}",
            reason="The EPDM pivot seal is intentionally compressed around the trunnion for weatherproofing.",
        )
        ctx.allow_overlap(
            base,
            head,
            elem_a=f"yoke_arm_{suffix}",
            elem_b=f"trunnion_{suffix}",
            reason="The yoke cheek is a simplified solid plate; the trunnion is intentionally shown passing through its bored pivot zone.",
        )
        ctx.expect_within(
            head,
            base,
            axes="yz",
            inner_elem=f"trunnion_{suffix}",
            outer_elem=f"yoke_bearing_{suffix}",
            margin=0.003,
            name=f"trunnion_{suffix}_centered_in_bearing",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="x",
            elem_a=f"trunnion_{suffix}",
            elem_b=f"yoke_bearing_{suffix}",
            min_overlap=0.030,
            name=f"trunnion_{suffix}_retained_by_bearing",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="x",
            elem_a=f"trunnion_{suffix}",
            elem_b=f"pivot_seal_{suffix}",
            min_overlap=0.008,
            name=f"trunnion_{suffix}_passes_through_seal",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="x",
            elem_a=f"trunnion_{suffix}",
            elem_b=f"yoke_arm_{suffix}",
            min_overlap=0.040,
            name=f"trunnion_{suffix}_passes_through_yoke_cheek",
        )

    ctx.expect_within(
        rotor,
        head,
        axes="xz",
        inner_elem="rotor_blades",
        outer_elem="rain_lipped_shroud",
        margin=0.015,
        name="rotor_clears_stationary_guard_radius",
    )
    ctx.expect_overlap(
        rotor,
        head,
        axes="y",
        elem_a="rotor_blades",
        elem_b="rain_lipped_shroud",
        min_overlap=0.040,
        name="rotor_sits_within_guard_depth",
    )

    ctx.check(
        "tilt_pivot_is_side_axis_revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower == -0.55
        and tilt.motion_limits.upper == 0.55,
        details=f"type={tilt.articulation_type} axis={tilt.axis} limits={tilt.motion_limits}",
    )
    ctx.check(
        "rotor_spin_is_continuous_hub_axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={spin.articulation_type} axis={spin.axis}",
    )

    rest_front = ctx.part_element_world_aabb(head, elem="front_guard_outer")
    with ctx.pose({tilt: 0.45}):
        tilted_front = ctx.part_element_world_aabb(head, elem="front_guard_outer")
        ctx.expect_within(
            rotor,
            head,
            axes="xz",
            inner_elem="rotor_blades",
            outer_elem="rain_lipped_shroud",
            margin=0.020,
            name="tilted_rotor_stays_inside_guard",
        )
    ctx.check(
        "positive_tilt_raises_front_guard",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[1][2] > rest_front[1][2] + 0.020,
        details=f"rest={rest_front}, tilted={tilted_front}",
    )

    return ctx.report()


object_model = build_object_model()
