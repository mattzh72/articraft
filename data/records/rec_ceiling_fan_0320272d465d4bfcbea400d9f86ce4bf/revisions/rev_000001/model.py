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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_motor_ceiling_fan")

    satin_white = model.material("satin_white", rgba=(0.88, 0.86, 0.80, 1.0))
    dark_vent = model.material("dark_vent", rgba=(0.04, 0.045, 0.045, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.55, 0.55, 0.52, 1.0))
    blade_ivory = model.material("blade_ivory", rgba=(0.92, 0.89, 0.78, 1.0))

    housing = model.part("motor_housing")
    housing.visual(
        Box((0.70, 0.24, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_white,
        name="rectangular_case",
    )
    housing.visual(
        Box((0.15, 0.105, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=satin_white,
        name="rod_saddle",
    )
    housing.visual(
        Cylinder(radius=0.024, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=brushed_metal,
        name="downrod",
    )
    housing.visual(
        Cylinder(radius=0.115, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.82)),
        material=satin_white,
        name="ceiling_canopy",
    )

    # Bearing collars and short axle stubs make the two horizontal rotor axes
    # visible at the opposed motor ends.
    housing.visual(
        Cylinder(radius=0.082, length=0.080),
        origin=Origin(xyz=(-0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="end_collar_0",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(-0.446, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="shaft_stub_0",
    )
    housing.visual(
        Cylinder(radius=0.082, length=0.080),
        origin=Origin(xyz=(0.390, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="end_collar_1",
    )
    housing.visual(
        Cylinder(radius=0.012, length=0.052),
        origin=Origin(xyz=(0.446, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="shaft_stub_1",
    )

    # Shallow dark insets read as ventilation slots in the rectangular motor box.
    for side_y in (-1.0, 1.0):
        for i, x in enumerate((-0.245, -0.195, -0.145, 0.145, 0.195, 0.245)):
            housing.visual(
                Box((0.033, 0.006, 0.011)),
                origin=Origin(xyz=(x, side_y * 0.121, 0.035)),
                material=dark_vent,
                name=f"side_vent_{'neg' if side_y < 0 else 'pos'}_{i}",
            )
    for i, x in enumerate((-0.22, -0.15, 0.15, 0.22)):
        housing.visual(
            Box((0.040, 0.070, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.081)),
            material=dark_vent,
            name=f"top_vent_{i}",
        )

    rotor_specs = [
        ("rotor_0", -0.482, -1.0, (0.0, -math.pi / 2.0, 0.0), -24.0),
        ("rotor_1", 0.482, 1.0, (0.0, math.pi / 2.0, 0.0), 24.0),
    ]
    for part_name, x, axis_x, visual_rpy, pitch in rotor_specs:
        rotor = model.part(part_name)
        rotor_mesh = mesh_from_geometry(
            FanRotorGeometry(
                outer_radius=0.275,
                hub_radius=0.055,
                blade_count=3,
                thickness=0.040,
                blade_pitch_deg=pitch,
                blade_sweep_deg=26.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=pitch * 0.45, camber=0.10),
                hub=FanRotorHub(style="spinner", bore_diameter=0.030),
            ),
            f"{part_name}_blade_set",
        )
        rotor.visual(
            rotor_mesh,
            origin=Origin(rpy=visual_rpy),
            material=blade_ivory,
            name="blade_set",
        )
        rotor.visual(
            Cylinder(radius=0.060, length=0.030),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_metal,
            name="hub_band",
        )
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=rotor,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            axis=(axis_x, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("motor_housing")
    rotor_0 = object_model.get_part("rotor_0")
    rotor_1 = object_model.get_part("rotor_1")
    joint_0 = object_model.get_articulation("housing_to_rotor_0")
    joint_1 = object_model.get_articulation("housing_to_rotor_1")

    ctx.check(
        "both blade sets are continuous rotors",
        joint_0.articulation_type == ArticulationType.CONTINUOUS
        and joint_1.articulation_type == ArticulationType.CONTINUOUS,
    )
    ctx.check(
        "rotor axes are horizontal and opposed",
        tuple(joint_0.axis) == (-1.0, 0.0, 0.0) and tuple(joint_1.axis) == (1.0, 0.0, 0.0),
    )
    ctx.allow_overlap(
        housing,
        rotor_0,
        elem_a="shaft_stub_0",
        elem_b="hub_band",
        reason="The left rotor hub is intentionally captured on the short motor shaft.",
    )
    ctx.allow_overlap(
        housing,
        rotor_1,
        elem_a="shaft_stub_1",
        elem_b="hub_band",
        reason="The right rotor hub is intentionally captured on the short motor shaft.",
    )
    ctx.expect_overlap(
        housing,
        rotor_0,
        axes="x",
        elem_a="shaft_stub_0",
        elem_b="hub_band",
        min_overlap=0.003,
        name="left hub remains seated on the shaft",
    )
    ctx.expect_overlap(
        housing,
        rotor_1,
        axes="x",
        elem_a="shaft_stub_1",
        elem_b="hub_band",
        min_overlap=0.003,
        name="right hub remains seated on the shaft",
    )
    ctx.expect_within(
        housing,
        rotor_0,
        axes="yz",
        inner_elem="shaft_stub_0",
        outer_elem="hub_band",
        margin=0.002,
        name="left shaft is centered inside the hub",
    )
    ctx.expect_within(
        housing,
        rotor_1,
        axes="yz",
        inner_elem="shaft_stub_1",
        outer_elem="hub_band",
        margin=0.002,
        name="right shaft is centered inside the hub",
    )

    rest_0 = ctx.part_world_position(rotor_0)
    rest_1 = ctx.part_world_position(rotor_1)
    with ctx.pose({joint_0: math.pi / 2.0, joint_1: -math.pi / 2.0}):
        spun_0 = ctx.part_world_position(rotor_0)
        spun_1 = ctx.part_world_position(rotor_1)
    ctx.check(
        "continuous rotation keeps hubs on their horizontal motor axes",
        rest_0 is not None
        and rest_1 is not None
        and spun_0 is not None
        and spun_1 is not None
        and max(abs(rest_0[i] - spun_0[i]) for i in range(3)) < 1e-6
        and max(abs(rest_1[i] - spun_1[i]) for i in range(3)) < 1e-6,
        details=f"rest_0={rest_0}, spun_0={spun_0}, rest_1={rest_1}, spun_1={spun_1}",
    )

    return ctx.report()


object_model = build_object_model()
