from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="liquid_cooled_gpu")

    pcb_green = model.material("pcb_green", rgba=(0.03, 0.22, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_gunmetal = model.material("satin_gunmetal", rgba=(0.20, 0.22, 0.24, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.055, 0.060, 0.066, 1.0))
    nickel = model.material("nickel", rgba=(0.72, 0.70, 0.66, 1.0))
    rubber = model.material("rubber_black", rgba=(0.006, 0.006, 0.007, 1.0))
    copper = model.material("copper", rgba=(0.76, 0.35, 0.13, 1.0))
    fan_gray = model.material("fan_gray", rgba=(0.08, 0.09, 0.10, 1.0))

    body = model.part("body")

    body.visual(
        Box((0.310, 0.115, 0.0024)),
        origin=Origin(xyz=(0.000, 0.000, 0.0012)),
        material=pcb_green,
        name="circuit_board",
    )
    # Gold PCIe edge fingers and rear bracket cues anchor the flat GPU board.
    body.visual(
        Box((0.105, 0.007, 0.0012)),
        origin=Origin(xyz=(-0.082, -0.0605, 0.0028)),
        material=copper,
        name="pcie_fingers",
    )
    body.visual(
        Box((0.006, 0.118, 0.030)),
        origin=Origin(xyz=(-0.158, 0.000, 0.0165)),
        material=nickel,
        name="io_bracket",
    )

    shroud_profile = rounded_rect_profile(0.255, 0.086, 0.010, corner_segments=8)
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(shroud_profile, 0.012),
            "long_shroud_cover",
        ),
        origin=Origin(xyz=(-0.008, 0.000, 0.0030)),
        material=matte_black,
        name="long_cover",
    )
    body.visual(
        Box((0.210, 0.006, 0.0020)),
        origin=Origin(xyz=(-0.012, 0.0415, 0.0115)),
        material=satin_gunmetal,
        name="upper_trim",
    )
    body.visual(
        Box((0.150, 0.005, 0.0020)),
        origin=Origin(xyz=(-0.040, -0.0415, 0.0115)),
        material=satin_gunmetal,
        name="lower_trim",
    )

    pump_profile = rounded_rect_profile(0.072, 0.058, 0.012, corner_segments=10)
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(pump_profile, 0.014),
            "pump_block_cover",
        ),
        origin=Origin(xyz=(-0.067, 0.001, 0.0142)),
        material=dark_graphite,
        name="pump_cover",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.004),
        origin=Origin(xyz=(-0.067, 0.001, 0.0302)),
        material=satin_gunmetal,
        name="pump_cap",
    )
    for y in (-0.017, 0.017):
        body.visual(
            Cylinder(radius=0.0062, length=0.040),
            origin=Origin(xyz=(-0.108, y, 0.0245), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=nickel,
            name=f"coolant_fitting_{'low' if y < 0 else 'high'}",
        )
        body.visual(
            Cylinder(radius=0.0074, length=0.048),
            origin=Origin(xyz=(-0.137, y, 0.0245), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"hose_stub_{'low' if y < 0 else 'high'}",
        )

    # Rear-corner circular assist-fan pod and raised guard.  The fan rotor itself
    # is a child link so it can spin freely inside this stationary frame.
    body.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.058, 0.058),
                (0.083, 0.083),
                0.0075,
                opening_shape="circle",
                outer_shape="circle",
                face=BezelFace(style="radiused_step", front_lip=0.0015, fillet=0.0012),
                center=False,
            ),
            "rear_fan_pod_frame",
        ),
        origin=Origin(xyz=(0.104, 0.023, 0.0140)),
        material=matte_black,
        name="fan_pod_frame",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.003),
        origin=Origin(xyz=(0.104, 0.023, 0.0145)),
        material=dark_graphite,
        name="fan_well",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.011),
        origin=Origin(xyz=(0.104, 0.023, 0.0215)),
        material=nickel,
        name="fan_axle",
    )
    for x, y in (
        (0.070, 0.023),
        (0.138, 0.023),
        (0.104, -0.011),
        (0.104, 0.057),
    ):
        body.visual(
            Cylinder(radius=0.0024, length=0.009),
            origin=Origin(xyz=(x, y, 0.0243)),
            material=satin_gunmetal,
            name=f"guard_post_{x:.3f}_{y:.3f}",
        )
    body.visual(
        Box((0.076, 0.0042, 0.0022)),
        origin=Origin(xyz=(0.104, 0.023, 0.0280)),
        material=satin_gunmetal,
        name="fan_guard_cross",
    )
    body.visual(
        Box((0.0042, 0.076, 0.0022)),
        origin=Origin(xyz=(0.104, 0.023, 0.0280)),
        material=satin_gunmetal,
        name="fan_guard_bar",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.0024),
        origin=Origin(xyz=(0.104, 0.023, 0.0281)),
        material=satin_gunmetal,
        name="guard_center",
    )

    body.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.018, -0.046, 0.0120), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_gunmetal,
        name="selector_socket",
    )

    fan = model.part("assist_fan")
    fan.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.0245,
                0.0075,
                9,
                thickness=0.006,
                blade_pitch_deg=33.0,
                blade_sweep_deg=28.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.0015),
            ),
            "assist_fan_rotor",
        ),
        origin=Origin(),
        material=fan_gray,
        name="fan_rotor",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.019,
                0.012,
                body_style="faceted",
                base_diameter=0.020,
                top_diameter=0.016,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=14, depth=0.0007, width=0.0012),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_knob_cap",
        ),
        # Knob geometry is built on local Z; rotate it so the short shaft points
        # out through the lower shroud edge.
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_gunmetal,
        name="knob_cap",
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=fan,
        origin=Origin(xyz=(0.104, 0.023, 0.0224)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=180.0),
    )
    model.articulation(
        "selector_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(-0.018, -0.0490, 0.0120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    fan = object_model.get_part("assist_fan")
    knob = object_model.get_part("selector_knob")
    fan_joint = object_model.get_articulation("fan_spin")
    knob_joint = object_model.get_articulation("selector_turn")

    ctx.allow_overlap(
        body,
        fan,
        elem_a="fan_axle",
        elem_b="fan_rotor",
        reason="The assist fan hub is intentionally captured on a small central axle/bearing.",
    )
    ctx.expect_overlap(
        body,
        fan,
        axes="z",
        elem_a="fan_axle",
        elem_b="fan_rotor",
        min_overlap=0.004,
        name="assist fan hub remains seated on axle",
    )
    ctx.check(
        "assist fan uses continuous spin joint",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={fan_joint.articulation_type}",
    )
    ctx.check(
        "selector knob uses limited rotary joint",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower < 0.0
        and knob_joint.motion_limits.upper > 0.0,
        details=f"type={knob_joint.articulation_type}, limits={knob_joint.motion_limits}",
    )
    ctx.expect_within(
        fan,
        body,
        axes="xy",
        inner_elem="fan_rotor",
        outer_elem="fan_pod_frame",
        margin=0.001,
        name="assist fan is centered inside rear pod",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_cap",
        elem_b="selector_socket",
        contact_tol=0.0015,
        name="selector knob seats on edge socket",
    )

    rest_knob_pos = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: 1.1, fan_joint: math.pi}):
        turned_knob_pos = ctx.part_world_position(knob)
        ctx.expect_within(
            fan,
            body,
            axes="xy",
            inner_elem="fan_rotor",
            outer_elem="fan_pod_frame",
            margin=0.001,
            name="spinning fan stays inside pod",
        )

    ctx.check(
        "selector knob turns about fixed short shaft",
        rest_knob_pos is not None
        and turned_knob_pos is not None
        and max(abs(a - b) for a, b in zip(rest_knob_pos, turned_knob_pos)) < 1e-6,
        details=f"rest={rest_knob_pos}, turned={turned_knob_pos}",
    )

    return ctx.report()


object_model = build_object_model()
