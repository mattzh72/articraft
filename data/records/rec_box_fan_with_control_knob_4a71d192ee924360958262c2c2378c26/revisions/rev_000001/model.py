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
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desk_box_fan")

    white = model.material("warm_white_plastic", rgba=(0.94, 0.94, 0.90, 1.0))
    grille_white = model.material("slightly_glossy_white", rgba=(0.98, 0.98, 0.95, 1.0))
    blade_gray = model.material("smoky_translucent_gray", rgba=(0.42, 0.46, 0.50, 1.0))
    dark_print = model.material("dark_print", rgba=(0.03, 0.035, 0.04, 1.0))
    knob_gray = model.material("cool_gray_plastic", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")

    # Box-fan shell: a real open frame, not a solid block.  The object frame is
    # at the rotor shaft center; X is front/back depth, Y is width, Z is height.
    body.visual(Box((0.165, 0.320, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.1525)), material=white, name="top_wall")
    body.visual(Box((0.165, 0.320, 0.035)), origin=Origin(xyz=(0.0, 0.0, -0.1525)), material=white, name="bottom_wall")
    body.visual(Box((0.165, 0.035, 0.340)), origin=Origin(xyz=(0.0, 0.1425, 0.0)), material=white, name="right_wall")
    body.visual(Box((0.165, 0.035, 0.340)), origin=Origin(xyz=(0.0, -0.1425, 0.0)), material=white, name="left_wall")

    # Low, broad feet integrated into the bottom of the plastic frame.
    body.visual(Box((0.135, 0.075, 0.034)), origin=Origin(xyz=(0.0, -0.080, -0.187)), material=white, name="foot_0")
    body.visual(Box((0.135, 0.075, 0.034)), origin=Origin(xyz=(0.0, 0.080, -0.187)), material=white, name="foot_1")

    # Front protective cage: concentric round rings and radial spokes, all
    # intersecting the white shell so the grille reads as one molded front.
    front_x = -0.082
    body.visual(
        mesh_from_geometry(
            TorusGeometry(0.132, 0.0036, radial_segments=14, tubular_segments=72),
            "front_ring_0",
        ),
        origin=Origin(xyz=(front_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_white,
        name="front_ring_0",
    )
    for index, ring_radius in enumerate((0.104, 0.076, 0.048), start=1):
        body.visual(
            mesh_from_geometry(
                TorusGeometry(ring_radius, 0.0036, radial_segments=14, tubular_segments=72),
                f"front_ring_{index}",
            ),
            origin=Origin(xyz=(front_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grille_white,
            name=f"front_ring_{index}",
        )

    for index in range(10):
        angle = index * math.pi / 10.0
        body.visual(
            Box((0.0070, 0.268, 0.0042)),
            origin=Origin(xyz=(front_x, 0.0, 0.0), rpy=(angle, 0.0, 0.0)),
            material=grille_white,
            name=f"front_spoke_{index}",
        )

    body.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(front_x - 0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_white,
        name="front_center_cap",
    )

    # Rear grille and central motor pod.  The pod is supported by four broad
    # struts so the spinning rotor has a believable fixed bearing behind it.
    rear_x = 0.080
    body.visual(
        mesh_from_geometry(
            TorusGeometry(0.124, 0.0035, radial_segments=14, tubular_segments=72),
            "rear_outer_ring",
        ),
        origin=Origin(xyz=(rear_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_white,
        name="rear_outer_ring",
    )
    body.visual(
        mesh_from_geometry(
            TorusGeometry(0.072, 0.0032, radial_segments=12, tubular_segments=60),
            "rear_inner_ring",
        ),
        origin=Origin(xyz=(rear_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_white,
        name="rear_inner_ring",
    )
    for index in range(6):
        angle = index * math.pi / 6.0
        body.visual(
            Box((0.009, 0.250, 0.005)),
            origin=Origin(xyz=(rear_x, 0.0, 0.0), rpy=(angle, 0.0, 0.0)),
            material=grille_white,
            name=f"rear_spoke_{index}",
        )

    body.visual(
        Cylinder(radius=0.041, length=0.052),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="motor_pod",
    )
    body.visual(Box((0.017, 0.126, 0.014)), origin=Origin(xyz=(0.014, 0.095, 0.0)), material=white, name="motor_strut_0")
    body.visual(Box((0.017, 0.126, 0.014)), origin=Origin(xyz=(0.014, -0.095, 0.0)), material=white, name="motor_strut_1")
    body.visual(Box((0.017, 0.014, 0.136)), origin=Origin(xyz=(0.014, 0.0, 0.096)), material=white, name="motor_strut_2")
    body.visual(Box((0.017, 0.014, 0.136)), origin=Origin(xyz=(0.014, 0.0, -0.096)), material=white, name="motor_strut_3")

    # Side speed-control boss and three printed detent dots on the right face.
    knob_center = (-0.030, 0.0, 0.075)
    body.visual(
        Cylinder(radius=0.034, length=0.011),
        origin=Origin(xyz=(knob_center[0], 0.1655, knob_center[2]), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="knob_boss",
    )
    detent_radius = 0.041
    for index, detent_angle in enumerate((-0.78, 0.0, 0.78)):
        body.visual(
            Cylinder(radius=0.0038, length=0.0016),
            origin=Origin(
                xyz=(
                    knob_center[0] + detent_radius * math.sin(detent_angle),
                    0.1606,
                    knob_center[2] + detent_radius * math.cos(detent_angle),
                ),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_print,
            name=f"speed_dot_{index}",
        )

    rotor = model.part("rotor")
    rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.113,
                0.028,
                5,
                thickness=0.020,
                blade_pitch_deg=29.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.003),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.006, rear_collar_radius=0.021),
            ),
            "five_blade_rotor",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_gray,
        name="five_blade_rotor",
    )
    rotor.visual(
        Cylinder(radius=0.0085, length=0.070),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blade_gray,
        name="shaft",
    )

    model.articulation(
        "body_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=45.0),
    )

    knob = model.part("speed_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.047,
                0.024,
                body_style="cylindrical",
                edge_radius=0.0012,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_gray,
        name="knob_cap",
    )

    model.articulation(
        "body_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(knob_center[0], 0.1710, knob_center[2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=2.0 * math.pi / 3.0),
        meta={"detents": (0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rotor = object_model.get_part("rotor")
    knob = object_model.get_part("speed_knob")
    rotor_joint = object_model.get_articulation("body_to_rotor")
    knob_joint = object_model.get_articulation("body_to_speed_knob")

    ctx.check("five blade rotor part present", rotor is not None, "Expected a separate rotor part.")
    ctx.check("speed knob part present", knob is not None, "Expected a separate speed knob part.")
    ctx.check("rotor spins about box fan axis", rotor_joint.axis == (1.0, 0.0, 0.0), f"axis={rotor_joint.axis!r}")
    ctx.check(
        "speed knob has three-position travel",
        knob_joint.motion_limits is not None
        and abs(float(knob_joint.motion_limits.upper) - 2.0 * math.pi / 3.0) < 1e-6
        and abs(float(knob_joint.motion_limits.lower)) < 1e-6,
        f"limits={knob_joint.motion_limits!r}",
    )
    ctx.allow_overlap(
        rotor,
        body,
        elem_a="shaft",
        elem_b="motor_pod",
        reason="The rotor shaft is intentionally captured inside the fixed motor-pod bearing.",
    )
    ctx.expect_within(
        rotor,
        body,
        axes="yz",
        inner_elem="shaft",
        outer_elem="motor_pod",
        margin=0.001,
        name="rotor shaft is centered in the motor bearing",
    )
    ctx.expect_overlap(
        rotor,
        body,
        axes="x",
        elem_a="shaft",
        elem_b="motor_pod",
        min_overlap=0.035,
        name="rotor shaft remains inserted in the motor bearing",
    )
    ctx.expect_within(
        rotor,
        body,
        axes="yz",
        inner_elem="five_blade_rotor",
        outer_elem="front_ring_0",
        margin=0.001,
        name="rotor disk sits inside the front grille ring",
    )
    ctx.expect_gap(
        body,
        rotor,
        axis="x",
        positive_elem="motor_pod",
        negative_elem="five_blade_rotor",
        min_gap=0.001,
        max_gap=0.030,
        name="rotor clears the fixed motor pod",
    )
    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_cap",
        elem_b="knob_boss",
        contact_tol=0.003,
        name="speed knob seats on the side boss",
    )

    return ctx.report()


object_model = build_object_model()
