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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_personal_box_fan")

    cream = model.material("warm_ivory_plastic", rgba=(0.86, 0.84, 0.76, 1.0))
    dark = model.material("dark_graphite", rgba=(0.04, 0.045, 0.05, 1.0))
    satin = model.material("satin_black", rgba=(0.01, 0.012, 0.014, 1.0))
    blue = model.material("translucent_smoke_blade", rgba=(0.38, 0.54, 0.66, 0.62))
    steel = model.material("brushed_steel", rgba=(0.60, 0.60, 0.57, 1.0))

    yaw_z = 0.355
    tilt_z = 0.245

    # Root: weighted table base, pedestal, and fixed bearing for oscillation.
    base = model.part("base")
    base.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cream,
        name="round_foot",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.310),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=cream,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=dark,
        name="oscillation_bearing",
    )

    # Rotating yoke: U-shaped trunnion stand that carries the tilting fan head.
    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.038, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark,
        name="turntable_collar",
    )
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.440, 0.055, 0.265),
                span_width=0.390,
                trunnion_diameter=0.028,
                trunnion_center_z=0.220,
                base_thickness=0.035,
                corner_radius=0.006,
                center=False,
            ),
            "tilt_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cream,
        name="tilt_yoke",
    )

    # Tilting fan housing/cage.  The local +Y direction is the front airflow axis.
    housing = model.part("housing")
    ring_mesh = mesh_from_geometry(TorusGeometry(0.176, 0.011, radial_segments=24, tubular_segments=96), "guard_outer_ring")
    inner_ring_mesh = mesh_from_geometry(TorusGeometry(0.095, 0.0035, radial_segments=12, tubular_segments=72), "guard_inner_ring")
    for name, y in (("front_ring", 0.060), ("rear_ring", -0.060)):
        housing.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name=name,
        )
    housing.visual(
        inner_ring_mesh,
        origin=Origin(xyz=(0.0, 0.063, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_inner_ring",
    )
    housing.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.0, 0.066, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="front_badge",
    )
    for i in range(6):
        angle = i * math.pi / 6.0
        housing.visual(
            Box((0.360, 0.0045, 0.0045)),
            origin=Origin(xyz=(0.0, 0.066, 0.0), rpy=(0.0, -angle, 0.0)),
            material=dark,
            name=f"front_spoke_{i}",
        )
    for i in range(4):
        angle = i * math.pi / 4.0
        housing.visual(
            Box((0.360, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(0.0, -angle, 0.0)),
            material=dark,
            name=f"rear_spoke_{i}",
        )
    for i in range(8):
        angle = i * math.tau / 8.0
        housing.visual(
            Cylinder(radius=0.0038, length=0.120),
            origin=Origin(
                xyz=(0.176 * math.cos(angle), 0.0, 0.176 * math.sin(angle)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark,
            name=f"cage_rod_{i}",
        )
    housing.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, -0.082, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="motor_bulge",
    )
    housing.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bearing_socket",
    )
    for side, x in (("left", -0.207), ("right", 0.207)):
        housing.visual(
            Cylinder(radius=0.009, length=0.060),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{side}_tilt_pin",
        )
        cap_x = -0.2245 if x < 0.0 else 0.2245
        housing.visual(
            Cylinder(radius=0.022, length=0.009),
            origin=Origin(xyz=(cap_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"{side}_pivot_cap",
        )

    # Rotating impeller with a small shaft visibly captured by the rear bearing.
    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.132,
                0.030,
                5,
                thickness=0.018,
                blade_pitch_deg=30.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=13.0, camber=0.12, tip_clearance=0.004),
                hub=FanRotorHub(style="spinner", bore_diameter=0.006),
            ),
            "fan_blade_rotor",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="blade_rotor",
    )
    blade.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="blade_shaft",
    )

    # Separate user-facing speed control knob on the base.
    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.020,
                body_style="skirted",
                top_diameter=0.034,
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_control_knob",
        ),
        material=dark,
        name="knob_cap",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, yaw_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.9),
    )
    model.articulation(
        "yoke_to_housing",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, tilt_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=30.0),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.085, -0.080, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    yoke_joint = object_model.get_articulation("base_to_yoke")
    tilt_joint = object_model.get_articulation("yoke_to_housing")
    blade_joint = object_model.get_articulation("housing_to_blade")
    knob_joint = object_model.get_articulation("base_to_speed_knob")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    yoke = object_model.get_part("yoke")
    speed_knob = object_model.get_part("speed_knob")
    base = object_model.get_part("base")

    ctx.allow_overlap(
        housing,
        blade,
        elem_a="bearing_socket",
        elem_b="blade_shaft",
        reason="The rotating blade shaft is intentionally captured inside the stationary rear bearing socket.",
    )
    ctx.expect_within(
        blade,
        housing,
        axes="xz",
        inner_elem="blade_shaft",
        outer_elem="bearing_socket",
        margin=0.0,
        name="blade shaft is centered in rear bearing",
    )
    ctx.expect_overlap(
        blade,
        housing,
        axes="y",
        elem_a="blade_shaft",
        elem_b="bearing_socket",
        min_overlap=0.015,
        name="blade shaft remains inserted in bearing",
    )

    ctx.check(
        "oscillation joint is continuous vertical yaw",
        yoke_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(yoke_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={yoke_joint.articulation_type}, axis={yoke_joint.axis}",
    )
    ctx.check(
        "blade joint is continuous about airflow axis",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(blade_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={blade_joint.articulation_type}, axis={blade_joint.axis}",
    )
    ctx.check(
        "speed knob rotates continuously",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(knob_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )
    ctx.check(
        "tilt hinge has realistic desk fan range",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower <= -0.35
        and tilt_joint.motion_limits.upper >= 0.45,
        details=f"type={tilt_joint.articulation_type}, limits={tilt_joint.motion_limits}",
    )

    ctx.expect_within(
        blade,
        housing,
        axes="xz",
        inner_elem="blade_rotor",
        outer_elem="front_ring",
        margin=0.0,
        name="fan blade disk fits within round guard",
    )
    ctx.expect_gap(
        housing,
        blade,
        axis="y",
        positive_elem="front_ring",
        negative_elem="blade_rotor",
        min_gap=0.020,
        name="front grille stands ahead of spinning blade",
    )
    ctx.expect_contact(speed_knob, base, elem_a="knob_cap", elem_b="round_foot", contact_tol=0.003, name="speed knob sits on base")
    ctx.expect_contact(yoke, base, elem_a="turntable_collar", elem_b="oscillation_bearing", contact_tol=0.001, name="oscillation collar seats on bearing")

    rest_aabb = ctx.part_element_world_aabb(housing, elem="front_ring")
    with ctx.pose({tilt_joint: 0.45}):
        tilted_aabb = ctx.part_element_world_aabb(housing, elem="front_ring")
    if rest_aabb is not None and tilted_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        tilted_center_z = (tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5
        ctx.check(
            "positive tilt raises front grille",
            tilted_center_z > rest_center_z + 0.015,
            details=f"rest_z={rest_center_z:.3f}, tilted_z={tilted_center_z:.3f}",
        )
    else:
        ctx.fail("positive tilt raises front grille", "front_ring AABBs were unavailable")

    return ctx.report()


object_model = build_object_model()
