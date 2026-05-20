from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


FAN_RADIUS = 0.235
ROTOR_RADIUS = 0.185
FRONT_GRILLE_X = 0.070
REAR_GRILLE_X = -0.055
PIVOT_X = -0.035
PIVOT_Z = 0.730


def _cylinder_to_x() -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0, 0.0)


def _cylinder_to_y() -> tuple[float, float, float]:
    return (-math.pi / 2.0, 0.0, 0.0)


def _radial_spoke_pose(x: float, angle: float, radius: float) -> Origin:
    return Origin(
        xyz=(x, 0.5 * radius * math.cos(angle), 0.5 * radius * math.sin(angle)),
        rpy=(angle - math.pi / 2.0, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_household_fan")

    warm_plastic = model.material("warm_plastic", rgba=(0.88, 0.86, 0.80, 1.0))
    dark_plastic = model.material("charcoal_plastic", rgba=(0.08, 0.085, 0.09, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    metal = model.material("brushed_steel", rgba=(0.72, 0.74, 0.74, 1.0))
    grille_wire = model.material("pale_gray_wire", rgba=(0.82, 0.84, 0.84, 1.0))
    blade_mat = model.material("translucent_smoke_blade", rgba=(0.42, 0.55, 0.62, 0.58))
    label_blue = model.material("speed_blue", rgba=(0.10, 0.25, 0.55, 1.0))

    support = model.part("support")

    base_plate = ExtrudeGeometry.from_z0(
        superellipse_profile(0.54, 0.36, exponent=2.7, segments=72),
        0.045,
    )
    support.visual(
        mesh_from_geometry(base_plate, "weighted_oval_base"),
        material=warm_plastic,
        name="weighted_oval_base",
    )
    support.visual(
        Cylinder(radius=0.235, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="rubber_foot_ring",
    )

    control_panel = ExtrudeGeometry.from_z0(
        superellipse_profile(0.205, 0.086, exponent=3.0, segments=48),
        0.008,
    )
    support.visual(
        mesh_from_geometry(control_panel, "control_panel"),
        origin=Origin(xyz=(0.105, 0.0, 0.045)),
        material=dark_plastic,
        name="control_panel",
    )

    support.visual(
        Cylinder(radius=0.024, length=0.470),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.280)),
        material=metal,
        name="upright_pole",
    )
    support.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.070)),
        material=warm_plastic,
        name="lower_collar",
    )
    support.visual(
        Cylinder(radius=0.036, length=0.055),
        origin=Origin(xyz=(PIVOT_X, 0.0, 0.502)),
        material=warm_plastic,
        name="upper_collar",
    )
    support.visual(
        Box((0.018, 0.026, 0.120)),
        origin=Origin(xyz=(-0.215, 0.0, 0.570)),
        material=warm_plastic,
        name="rear_fork_stem",
    )
    support.visual(
        Box((0.180, 0.026, 0.018)),
        origin=Origin(xyz=(-0.125, 0.0, 0.520)),
        material=warm_plastic,
        name="rear_offset_neck",
    )
    support.visual(
        Cylinder(radius=0.018, length=0.610),
        origin=Origin(xyz=(-0.215, 0.0, 0.615), rpy=_cylinder_to_y()),
        material=metal,
        name="yoke_crossbar",
    )
    for side, y in enumerate((-0.285, 0.285)):
        support.visual(
            Box((0.180, 0.030, 0.030)),
            origin=Origin(xyz=(-0.125, y, 0.615)),
            material=warm_plastic,
            name=f"yoke_side_bridge_{side}",
        )
        support.visual(
            Box((0.040, 0.030, 0.220)),
            origin=Origin(xyz=(PIVOT_X, y, 0.665)),
            material=warm_plastic,
            name=f"yoke_arm_{side}",
        )
        support.visual(
            Cylinder(radius=0.042, length=0.034),
            origin=Origin(xyz=(PIVOT_X, y, PIVOT_Z), rpy=_cylinder_to_y()),
            material=warm_plastic,
            name=f"pivot_bushing_{side}",
        )

    head = model.part("head")

    motor_shell = LatheGeometry(
        [
            (0.000, -0.182),
            (0.042, -0.178),
            (0.086, -0.150),
            (0.112, -0.095),
            (0.114, -0.052),
            (0.096, -0.024),
            (0.044, -0.014),
            (0.000, -0.014),
        ],
        segments=72,
    )
    head.visual(
        mesh_from_geometry(motor_shell, "motor_shell"),
        origin=Origin(rpy=_cylinder_to_x()),
        material=warm_plastic,
        name="motor_shell",
    )
    head.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(-0.043, 0.0, 0.0), rpy=_cylinder_to_x()),
        material=warm_plastic,
        name="guard_mount",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.485),
        origin=Origin(xyz=(-0.040, 0.0, 0.0), rpy=_cylinder_to_y()),
        material=metal,
        name="pivot_pin",
    )
    for side, y in enumerate((-0.250, 0.250)):
        head.visual(
            Cylinder(radius=0.044, length=0.034),
            origin=Origin(xyz=(-0.040, y, 0.0), rpy=_cylinder_to_y()),
            material=dark_plastic,
            name=f"pivot_cap_{side}",
        )

    rear_handle = tube_from_spline_points(
        [
            (-0.165, -0.040, 0.080),
            (-0.145, 0.000, 0.136),
            (-0.095, 0.040, 0.136),
            (-0.055, 0.046, 0.082),
        ],
        radius=0.0065,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    head.visual(
        mesh_from_geometry(rear_handle, "rear_carry_handle"),
        material=warm_plastic,
        name="rear_carry_handle",
    )

    head.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(-0.187, 0.0, 0.0), rpy=_cylinder_to_x()),
        material=dark_plastic,
        name="speed_socket",
    )

    # Protective guard: double grille, outer cage, radial spokes, and a center badge.
    for label, x in (("front", FRONT_GRILLE_X), ("rear", REAR_GRILLE_X)):
        for ring_index, radius in enumerate((FAN_RADIUS, 0.195, 0.150, 0.105, 0.062)):
            tube = 0.0060 if ring_index == 0 else 0.0028
            head.visual(
                mesh_from_geometry(
                    TorusGeometry(radius=radius, tube=tube, radial_segments=18, tubular_segments=96),
                    f"{label}_ring_{ring_index}",
                ),
                origin=Origin(xyz=(x, 0.0, 0.0), rpy=_cylinder_to_x()),
                material=grille_wire,
                name=f"{label}_ring_{ring_index}",
            )
        spoke_count = 24 if label == "front" else 16
        for i in range(spoke_count):
            angle = 2.0 * math.pi * i / spoke_count
            head.visual(
                Cylinder(radius=0.0019, length=FAN_RADIUS),
                origin=_radial_spoke_pose(x, angle, FAN_RADIUS),
                material=grille_wire,
                name=f"{label}_spoke_{i}",
            )

    for i in range(20):
        angle = 2.0 * math.pi * i / 20.0
        head.visual(
            Cylinder(radius=0.0021, length=FRONT_GRILLE_X - REAR_GRILLE_X),
            origin=Origin(
                xyz=(
                    0.5 * (FRONT_GRILLE_X + REAR_GRILLE_X),
                    FAN_RADIUS * math.cos(angle),
                    FAN_RADIUS * math.sin(angle),
                ),
                rpy=_cylinder_to_x(),
            ),
            material=grille_wire,
            name=f"cage_wire_{i}",
        )

    head.visual(
        Cylinder(radius=0.036, length=0.010),
        origin=Origin(xyz=(FRONT_GRILLE_X + 0.002, 0.0, 0.0), rpy=_cylinder_to_x()),
        material=dark_plastic,
        name="front_badge",
    )
    head.visual(
        Box((0.030, 0.006, 0.006)),
        origin=Origin(xyz=(FRONT_GRILLE_X + 0.008, 0.0, 0.0)),
        material=label_blue,
        name="badge_mark",
    )

    rotor = model.part("rotor")
    rotor_geom = FanRotorGeometry(
        outer_radius=ROTOR_RADIUS,
        hub_radius=0.046,
        blade_count=5,
        thickness=0.028,
        blade_pitch_deg=31.0,
        blade_sweep_deg=28.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.18, tip_clearance=0.010),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.044, bore_diameter=0.010),
    )
    rotor.visual(
        mesh_from_geometry(rotor_geom, "blade_assembly"),
        origin=Origin(rpy=_cylinder_to_x()),
        material=blade_mat,
        name="blade_assembly",
    )
    rotor.visual(
        Cylinder(radius=0.009, length=0.056),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=_cylinder_to_x()),
        material=metal,
        name="shaft_stub",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.043,
                0.026,
                body_style="skirted",
                top_diameter=0.034,
                grip=KnobGrip(style="fluted", count=18, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_cap",
        ),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="speed_cap",
    )

    button_positions = (0.055, 0.105, 0.155)
    for i, x in enumerate(button_positions):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.0145, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=label_blue if i == 1 else warm_plastic,
            name="button_cap",
        )
        model.articulation(
            f"support_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=support,
            child=button,
            origin=Origin(xyz=(x, 0.0, 0.053)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    model.articulation(
        "support_to_head",
        ArticulationType.REVOLUTE,
        parent=support,
        child=head,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.8, lower=-0.35, upper=0.52),
    )
    model.articulation(
        "head_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=80.0),
    )
    model.articulation(
        "head_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=head,
        child=speed_knob,
        origin=Origin(xyz=(-0.193, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=4.7),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    head = object_model.get_part("head")
    rotor = object_model.get_part("rotor")
    speed_knob = object_model.get_part("speed_knob")
    tilt = object_model.get_articulation("support_to_head")
    spin = object_model.get_articulation("head_to_rotor")
    knob_joint = object_model.get_articulation("head_to_speed_knob")

    ctx.expect_gap(
        head,
        rotor,
        axis="x",
        positive_elem="front_ring_0",
        negative_elem="blade_assembly",
        min_gap=0.025,
        name="front grille clears spinning blades",
    )
    ctx.expect_gap(
        rotor,
        head,
        axis="x",
        positive_elem="blade_assembly",
        negative_elem="rear_ring_0",
        min_gap=0.018,
        name="rear grille clears spinning blades",
    )
    ctx.expect_within(
        rotor,
        head,
        axes="yz",
        inner_elem="blade_assembly",
        outer_elem="front_ring_0",
        margin=0.004,
        name="blade sweep fits inside protective grille",
    )
    ctx.expect_gap(
        head,
        speed_knob,
        axis="x",
        positive_elem="speed_socket",
        negative_elem="speed_cap",
        max_gap=0.001,
        max_penetration=0.001,
        name="speed knob seats on rear socket",
    )

    rest_front = ctx.part_element_world_aabb(head, elem="front_ring_0")
    with ctx.pose({tilt: 0.42}):
        raised_front = ctx.part_element_world_aabb(head, elem="front_ring_0")
    ctx.check(
        "tilt joint raises the fan face",
        rest_front is not None
        and raised_front is not None
        and 0.5 * (raised_front[0][2] + raised_front[1][2]) > 0.5 * (rest_front[0][2] + rest_front[1][2]) + 0.020,
        details=f"rest={rest_front}, raised={raised_front}",
    )

    ctx.check(
        "spin axis matches fan shaft",
        tuple(round(v, 3) for v in spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={spin.axis}",
    )
    ctx.check(
        "speed knob has a finite rotary range",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower == 0.0
        and knob_joint.motion_limits.upper is not None
        and knob_joint.motion_limits.upper > 4.0,
        details=f"limits={knob_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
