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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _envelope_mesh():
    profile = [
        (0.0, -16.0),
        (0.35, -15.4),
        (0.95, -14.6),
        (2.10, -13.0),
        (3.25, -10.0),
        (4.00, -5.6),
        (4.25, -0.5),
        (4.18, 4.8),
        (3.80, 9.5),
        (2.85, 13.1),
        (1.55, 15.1),
        (0.50, 15.8),
        (0.0, 16.0),
    ]
    return LatheGeometry(profile, segments=88).rotate_y(math.pi / 2.0)


def _pylon_arm_mesh(side_sign: float):
    profile = rounded_rect_profile(0.28, 0.20, 0.05, corner_segments=6)
    return sweep_profile_along_spline(
        [
            (-0.42, -1.56 * side_sign, 0.72),
            (-0.18, -0.94 * side_sign, 0.38),
            (0.02, -0.18 * side_sign, -0.06),
        ],
        profile=profile,
        samples_per_segment=14,
        cap_profile=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_blimp")

    envelope_white = model.material("envelope_white", rgba=(0.88, 0.89, 0.86, 1.0))
    gondola_gray = model.material("gondola_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    structure_gray = model.material("structure_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    engine_orange = model.material("engine_orange", rgba=(0.77, 0.39, 0.12, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.12, 0.18, 0.24, 1.0))
    prop_black = model.material("prop_black", rgba=(0.08, 0.09, 0.10, 1.0))

    envelope_mesh = _save_mesh("blimp_envelope", _envelope_mesh())
    right_pylon_arm_mesh = _save_mesh("right_pylon_arm", _pylon_arm_mesh(1.0))
    left_pylon_arm_mesh = _save_mesh("left_pylon_arm", _pylon_arm_mesh(-1.0))
    pylon_yoke_mesh = _save_mesh(
        "pylon_yoke",
        TrunnionYokeGeometry(
            (0.44, 0.40, 0.42),
            span_width=0.22,
            trunnion_diameter=0.08,
            trunnion_center_z=0.26,
            base_thickness=0.10,
            corner_radius=0.03,
            center=False,
        ),
    )
    propeller_mesh = _save_mesh(
        "utility_propeller",
        FanRotorGeometry(
            0.72,
            0.16,
            3,
            thickness=0.14,
            blade_pitch_deg=34.0,
            blade_sweep_deg=19.0,
            blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=18.0, camber=0.14),
            hub=FanRotorHub(style="spinner", rear_collar_height=0.035, rear_collar_radius=0.13),
            center=False,
        ),
    )

    hull = model.part("hull")
    hull.visual(envelope_mesh, material=envelope_white, name="envelope")
    hull.visual(
        Box((7.0, 1.40, 0.90)),
        origin=Origin(xyz=(0.1, 0.0, -4.00)),
        material=structure_gray,
        name="keel_beam",
    )
    hull.visual(
        Box((3.2, 0.52, 3.60)),
        origin=Origin(xyz=(-12.7, 0.0, 2.10)),
        material=envelope_white,
        name="rear_fin",
    )
    hull.visual(
        Box((3.0, 6.20, 0.48)),
        origin=Origin(xyz=(-12.8, 0.0, 0.52)),
        material=envelope_white,
        name="tailplane",
    )
    hull.visual(
        Box((2.3, 0.44, 2.10)),
        origin=Origin(xyz=(-13.1, 0.0, -1.55)),
        material=envelope_white,
        name="ventral_fin",
    )

    for index, (x_pos, y_pos) in enumerate(((-1.35, -0.48), (-1.35, 0.48), (1.35, -0.48), (1.35, 0.48))):
        hull.visual(
            Cylinder(radius=0.085, length=1.45),
            origin=Origin(xyz=(x_pos, y_pos, -5.175)),
            material=structure_gray,
            name=f"support_strut_{index}",
        )

    gondola = model.part("gondola")
    gondola.visual(
        Box((4.60, 1.85, 1.60)),
        material=gondola_gray,
        name="cabin_body",
    )
    gondola.visual(
        Box((0.90, 1.50, 0.96)),
        origin=Origin(xyz=(2.55, 0.0, 0.10)),
        material=gondola_gray,
        name="nose_block",
    )
    gondola.visual(
        Box((1.10, 1.20, 0.18)),
        origin=Origin(xyz=(0.55, 0.0, 0.85)),
        material=gondola_gray,
        name="roof_fairing",
    )
    gondola.visual(
        Box((1.55, 0.04, 0.52)),
        origin=Origin(xyz=(1.15, 0.93, 0.22)),
        material=glass_dark,
        name="starboard_window",
    )
    gondola.visual(
        Box((1.55, 0.04, 0.52)),
        origin=Origin(xyz=(1.15, -0.93, 0.22)),
        material=glass_dark,
        name="port_window",
    )
    gondola.visual(
        Box((0.92, 0.70, 0.48)),
        origin=Origin(xyz=(2.74, 0.0, 0.22)),
        material=glass_dark,
        name="windshield",
    )
    gondola.visual(
        Box((1.70, 0.20, 0.12)),
        origin=Origin(xyz=(-0.4, 0.72, -0.78)),
        material=structure_gray,
        name="skid_0",
    )
    gondola.visual(
        Box((1.70, 0.20, 0.12)),
        origin=Origin(xyz=(-0.4, -0.72, -0.78)),
        material=structure_gray,
        name="skid_1",
    )

    right_pylon = model.part("right_pylon")
    right_pylon.visual(
        Box((0.64, 0.18, 0.56)),
        origin=Origin(xyz=(-0.42, -1.56, 0.72)),
        material=structure_gray,
        name="mount_pad",
    )
    right_pylon.visual(right_pylon_arm_mesh, material=structure_gray, name="arm")
    right_pylon.visual(
        pylon_yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
        material=structure_gray,
        name="yoke",
    )

    left_pylon = model.part("left_pylon")
    left_pylon.visual(
        Box((0.64, 0.18, 0.56)),
        origin=Origin(xyz=(-0.42, 1.56, 0.72)),
        material=structure_gray,
        name="mount_pad",
    )
    left_pylon.visual(left_pylon_arm_mesh, material=structure_gray, name="arm")
    left_pylon.visual(
        pylon_yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.26)),
        material=structure_gray,
        name="yoke",
    )

    right_engine_pod = model.part("right_engine_pod")
    right_engine_pod.visual(
        Cylinder(radius=0.085, length=0.20),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structure_gray,
        name="pivot_collar",
    )
    right_engine_pod.visual(
        Box((0.36, 0.18, 0.16)),
        origin=Origin(xyz=(0.17, 0.0, -0.10)),
        material=structure_gray,
        name="pivot_neck",
    )
    right_engine_pod.visual(
        Cylinder(radius=0.32, length=1.26),
        origin=Origin(xyz=(0.92, 0.0, -0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=engine_orange,
        name="pod_body",
    )
    right_engine_pod.visual(
        Cylinder(radius=0.21, length=0.44),
        origin=Origin(xyz=(1.76, 0.0, -0.14), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=engine_orange,
        name="nose_cap",
    )
    right_engine_pod.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(2.05, 0.0, -0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=structure_gray,
        name="prop_shaft_stub",
    )

    left_engine_pod = model.part("left_engine_pod")
    left_engine_pod.visual(
        Cylinder(radius=0.085, length=0.20),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=structure_gray,
        name="pivot_collar",
    )
    left_engine_pod.visual(
        Box((0.36, 0.18, 0.16)),
        origin=Origin(xyz=(0.17, 0.0, -0.10)),
        material=structure_gray,
        name="pivot_neck",
    )
    left_engine_pod.visual(
        Cylinder(radius=0.32, length=1.26),
        origin=Origin(xyz=(0.92, 0.0, -0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=engine_orange,
        name="pod_body",
    )
    left_engine_pod.visual(
        Cylinder(radius=0.21, length=0.44),
        origin=Origin(xyz=(1.76, 0.0, -0.14), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=engine_orange,
        name="nose_cap",
    )
    left_engine_pod.visual(
        Cylinder(radius=0.055, length=0.18),
        origin=Origin(xyz=(2.05, 0.0, -0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=structure_gray,
        name="prop_shaft_stub",
    )

    right_propeller = model.part("right_propeller")
    right_propeller.visual(
        propeller_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )
    right_propeller.visual(
        Cylinder(radius=0.045, length=0.20),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=structure_gray,
        name="hub_shaft",
    )

    left_propeller = model.part("left_propeller")
    left_propeller.visual(
        propeller_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=prop_black,
        name="rotor",
    )
    left_propeller.visual(
        Cylinder(radius=0.045, length=0.20),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=structure_gray,
        name="hub_shaft",
    )

    rudder = model.part("rudder")
    rudder.visual(
        Box((1.60, 0.18, 2.20)),
        origin=Origin(xyz=(-0.80, 0.0, 0.0)),
        material=envelope_white,
        name="surface",
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        Box((1.28, 2.52, 0.18)),
        origin=Origin(xyz=(-0.64, 0.0, 0.0)),
        material=envelope_white,
        name="surface",
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        Box((1.28, 2.52, 0.18)),
        origin=Origin(xyz=(-0.64, 0.0, 0.0)),
        material=envelope_white,
        name="surface",
    )

    model.articulation(
        "hull_to_gondola",
        ArticulationType.FIXED,
        parent=hull,
        child=gondola,
        origin=Origin(xyz=(0.0, 0.0, -6.70)),
    )
    model.articulation(
        "hull_to_right_pylon",
        ArticulationType.FIXED,
        parent=hull,
        child=right_pylon,
        origin=Origin(xyz=(0.20, 2.35, -5.35)),
    )
    model.articulation(
        "hull_to_left_pylon",
        ArticulationType.FIXED,
        parent=hull,
        child=left_pylon,
        origin=Origin(xyz=(0.20, -2.35, -5.35)),
    )
    model.articulation(
        "right_pod_vector",
        ArticulationType.REVOLUTE,
        parent=right_pylon,
        child=right_engine_pod,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=-1.05,
            upper=0.85,
        ),
    )
    model.articulation(
        "left_pod_vector",
        ArticulationType.REVOLUTE,
        parent=left_pylon,
        child=left_engine_pod,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=1.2,
            lower=-1.05,
            upper=0.85,
        ),
    )
    model.articulation(
        "right_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=right_engine_pod,
        child=right_propeller,
        origin=Origin(xyz=(2.14, 0.0, -0.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=40.0),
    )
    model.articulation(
        "left_prop_spin",
        ArticulationType.CONTINUOUS,
        parent=left_engine_pod,
        child=left_propeller,
        origin=Origin(xyz=(2.14, 0.0, -0.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=40.0),
    )
    model.articulation(
        "rudder_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-14.30, 0.0, 2.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.9,
            lower=-0.50,
            upper=0.50,
        ),
    )
    model.articulation(
        "left_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(-14.30, -2.80, 0.52)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.9,
            lower=-0.40,
            upper=0.48,
        ),
    )
    model.articulation(
        "right_elevator_hinge",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(-14.30, 2.80, 0.52)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.9,
            lower=-0.40,
            upper=0.48,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    gondola = object_model.get_part("gondola")
    right_pylon = object_model.get_part("right_pylon")
    left_pylon = object_model.get_part("left_pylon")
    right_engine_pod = object_model.get_part("right_engine_pod")
    left_engine_pod = object_model.get_part("left_engine_pod")
    right_propeller = object_model.get_part("right_propeller")
    left_propeller = object_model.get_part("left_propeller")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")

    left_pod_vector = object_model.get_articulation("left_pod_vector")
    rudder_hinge = object_model.get_articulation("rudder_hinge")
    left_elevator_hinge = object_model.get_articulation("left_elevator_hinge")

    ctx.expect_contact(gondola, hull, name="gondola is physically supported by the hull struts")
    ctx.expect_origin_gap(
        hull,
        gondola,
        axis="z",
        min_gap=6.0,
        max_gap=7.2,
        name="gondola hangs well below the hull centerline",
    )
    ctx.allow_overlap(
        left_engine_pod,
        left_pylon,
        elem_a="pivot_neck",
        elem_b="yoke",
        reason="The short trunnion neck intentionally sits inside the vectoring yoke mount.",
    )
    ctx.allow_overlap(
        right_engine_pod,
        right_pylon,
        elem_a="pivot_neck",
        elem_b="yoke",
        reason="The short trunnion neck intentionally sits inside the vectoring yoke mount.",
    )
    ctx.expect_contact(right_engine_pod, right_pylon, name="right engine pod is carried by its pylon")
    ctx.expect_contact(left_engine_pod, left_pylon, name="left engine pod is carried by its pylon")
    ctx.expect_contact(right_propeller, right_engine_pod, name="right propeller sits on the pod shaft")
    ctx.expect_contact(left_propeller, left_engine_pod, name="left propeller sits on the pod shaft")

    left_elevator_rest = ctx.part_element_world_aabb(left_elevator, elem="surface")
    with ctx.pose({left_elevator_hinge: 0.40}):
        left_elevator_up = ctx.part_element_world_aabb(left_elevator, elem="surface")
    ctx.check(
        "left elevator raises at positive deflection",
        left_elevator_rest is not None
        and left_elevator_up is not None
        and left_elevator_up[1][2] > left_elevator_rest[1][2] + 0.18,
        details=f"rest={left_elevator_rest}, up={left_elevator_up}",
    )

    left_prop_rest = ctx.part_world_aabb(left_propeller)
    with ctx.pose({left_pod_vector: 0.70}):
        left_prop_vectored = ctx.part_world_aabb(left_propeller)
    ctx.check(
        "left engine pod vectors upward at positive pitch",
        left_prop_rest is not None
        and left_prop_vectored is not None
        and left_prop_vectored[1][2] > left_prop_rest[1][2] + 0.35,
        details=f"rest={left_prop_rest}, vectored={left_prop_vectored}",
    )

    rudder_rest = ctx.part_element_world_aabb(rudder, elem="surface")
    with ctx.pose({rudder_hinge: 0.35}):
        rudder_deflected = ctx.part_element_world_aabb(rudder, elem="surface")
    ctx.check(
        "rudder swings laterally from the rear fin hinge",
        rudder_rest is not None
        and rudder_deflected is not None
        and rudder_deflected[0][1] < rudder_rest[0][1] - 0.20,
        details=f"rest={rudder_rest}, deflected={rudder_deflected}",
    )

    return ctx.report()


object_model = build_object_model()
