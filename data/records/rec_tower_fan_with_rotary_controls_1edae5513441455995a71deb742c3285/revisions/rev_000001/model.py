from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_tower_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.80, 1.0))
    dark = model.material("charcoal_grille", rgba=(0.035, 0.040, 0.045, 1.0))
    satin_gray = model.material("satin_gray_plastic", rgba=(0.48, 0.50, 0.52, 1.0))
    metal = model.material("dark_bushed_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    blue_gray = model.material("blue_gray_rotor", rgba=(0.22, 0.31, 0.36, 1.0))
    black = model.material("black_markings", rgba=(0.005, 0.005, 0.004, 1.0))

    def rounded_prism_mesh(width: float, depth: float, height: float, radius: float, name: str):
        return mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(width, depth, radius, corner_segments=8),
                height,
                cap=True,
                center=True,
            ),
            name,
        )

    # Root: a circular weighted stand and turntable, sized like a desktop-to-floor
    # tower fan base.
    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.180, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_gray,
        name="base_disc",
    )
    stand.visual(
        Cylinder(radius=0.120, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=warm_white,
        name="raised_foot",
    )
    stand.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=warm_white,
        name="pedestal_neck",
    )
    stand.visual(
        Cylinder(radius=0.075, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.091)),
        material=satin_gray,
        name="turntable",
    )

    # Oscillating tower body.  The housing is built as a hollow rail-and-grille
    # assembly so the rotating cross-flow blower has real clearance inside.
    body = model.part("body")
    body.visual(
        Cylinder(radius=0.055, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=warm_white,
        name="lower_collar",
    )
    body.visual(
        rounded_prism_mesh(0.170, 0.120, 0.055, 0.030, "bottom_cap_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=warm_white,
        name="bottom_cap",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=dark,
        name="lower_bearing",
    )
    for x, name in ((-0.073, "side_rail_0"), (0.073, "side_rail_1")):
        body.visual(
            rounded_prism_mesh(0.028, 0.120, 0.670, 0.012, f"{name}_mesh"),
            origin=Origin(xyz=(x, 0.0, 0.425)),
            material=warm_white,
            name=name,
        )
    body.visual(
        rounded_prism_mesh(0.145, 0.010, 0.670, 0.004, "rear_panel_mesh"),
        origin=Origin(xyz=(0.0, 0.058, 0.425)),
        material=warm_white,
        name="rear_panel",
    )
    body.visual(
        rounded_prism_mesh(0.180, 0.130, 0.082, 0.034, "top_cap_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.799)),
        material=warm_white,
        name="top_cap",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.756)),
        material=dark,
        name="upper_bearing",
    )
    body.visual(
        Box((0.006, 0.007, 0.610)),
        origin=Origin(xyz=(0.0, -0.061, 0.425)),
        material=dark,
        name="front_spine",
    )
    for i in range(26):
        z = 0.130 + i * 0.023
        body.visual(
            Box((0.145, 0.007, 0.008)),
            origin=Origin(xyz=(0.0, -0.061, z)),
            material=dark,
            name=f"grille_slat_{i}",
        )

    # Tick marks around the two top-cap controls.  They sit proud of the cap but
    # slightly penetrate it so they read as molded/printed features.
    for prefix, cx in (("timer", -0.045), ("speed", 0.045)):
        for i, angle in enumerate((-120, -75, -30, 30, 75, 120)):
            a = math.radians(angle)
            r = 0.030
            body.visual(
                Box((0.0022, 0.0090, 0.0012)),
                origin=Origin(
                    xyz=(cx + r * math.sin(a), -0.010 + r * math.cos(a), 0.8402),
                    rpy=(0.0, 0.0, -a),
                ),
                material=black,
                name=f"{prefix}_tick_{i}",
            )

    model.articulation(
        "stand_to_body",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.85, upper=0.85),
    )

    # Cross-flow blower wheel rotating continuously inside the hollow tower.
    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.045,
                inner_radius=0.023,
                width=0.580,
                blade_count=32,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "blower_wheel_mesh",
        ),
        origin=Origin(),
        material=blue_gray,
        name="blower_wheel",
    )
    blower.visual(
        Cylinder(radius=0.005, length=0.658),
        origin=Origin(),
        material=metal,
        name="shaft",
    )
    for z, name in ((-0.286, "lower_hub"), (0.286, "upper_hub")):
        blower.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=metal,
            name=name,
        )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=40.0),
    )

    timer_knob = model.part("timer_knob")
    speed_knob = model.part("speed_knob")
    knob_geom = KnobGeometry(
        0.038,
        0.024,
        body_style="skirted",
        top_diameter=0.032,
        skirt=KnobSkirt(0.044, 0.006, flare=0.07, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=18, depth=0.0011),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005),
        center=False,
    )
    for knob, mesh_name in ((timer_knob, "timer_knob_mesh"), (speed_knob, "speed_knob_mesh")):
        knob.visual(
            mesh_from_geometry(knob_geom, mesh_name),
            origin=Origin(),
            material=satin_gray,
            name="cap",
        )
        knob.visual(
            Box((0.004, 0.018, 0.002)),
            origin=Origin(xyz=(0.0, 0.007, 0.0245)),
            material=black,
            name="pointer",
        )

    model.articulation(
        "body_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(-0.045, -0.010, 0.840)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0, lower=0.0, upper=2.0 * math.pi),
    )
    model.articulation(
        "body_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=speed_knob,
        origin=Origin(xyz=(0.045, -0.010, 0.840)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=4.0, lower=0.0, upper=2.7),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower")
    timer_knob = object_model.get_part("timer_knob")
    speed_knob = object_model.get_part("speed_knob")
    osc = object_model.get_articulation("stand_to_body")
    blower_spin = object_model.get_articulation("body_to_blower")
    timer_spin = object_model.get_articulation("body_to_timer_knob")
    speed_spin = object_model.get_articulation("body_to_speed_knob")

    def joint_kind(joint) -> str:
        kind = joint.articulation_type
        return getattr(kind, "name", str(kind)).lower()

    def aabb_center(part, elem: str):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.check(
        "body oscillates on vertical revolute",
        "revolute" in joint_kind(osc)
        and osc.axis == (0.0, 0.0, 1.0)
        and osc.motion_limits.lower < 0.0
        and osc.motion_limits.upper > 0.0,
        details=f"type={osc.articulation_type}, axis={osc.axis}, limits={osc.motion_limits}",
    )
    ctx.check(
        "blower has continuous spin joint",
        "continuous" in joint_kind(blower_spin) and blower_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={blower_spin.articulation_type}, axis={blower_spin.axis}",
    )
    ctx.check(
        "top knobs have independent revolute joints",
        "revolute" in joint_kind(timer_spin)
        and "revolute" in joint_kind(speed_spin)
        and timer_spin.child != speed_spin.child,
        details=f"timer={timer_spin.articulation_type}, speed={speed_spin.articulation_type}",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body has measurable upright proportions", "missing body AABB")
    else:
        lo, hi = body_aabb
        dx, dy, dz = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
        ctx.check(
            "body has narrow upright tower proportions",
            dz > 0.78 and dx < 0.22 and dy < 0.17 and dz > 4.0 * dx,
            details=f"body dims dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}",
        )

    ctx.expect_gap(
        body,
        stand,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="turntable",
        max_gap=0.001,
        max_penetration=0.0,
        name="oscillating body is seated on the turntable",
    )
    ctx.expect_within(
        blower,
        body,
        axes="xy",
        inner_elem="blower_wheel",
        margin=0.0,
        name="blower wheel sits inside the tower footprint",
    )
    ctx.expect_gap(
        body,
        blower,
        axis="y",
        positive_elem="rear_panel",
        negative_elem="blower_wheel",
        min_gap=0.005,
        name="blower clears the rear shell",
    )
    ctx.expect_gap(
        blower,
        body,
        axis="y",
        positive_elem="blower_wheel",
        negative_elem="front_spine",
        min_gap=0.008,
        name="blower clears the front grille",
    )
    ctx.expect_contact(
        blower,
        body,
        elem_a="shaft",
        elem_b="lower_bearing",
        contact_tol=0.001,
        name="blower shaft seats in lower bearing",
    )
    ctx.expect_contact(
        blower,
        body,
        elem_a="shaft",
        elem_b="upper_bearing",
        contact_tol=0.001,
        name="blower shaft seats in upper bearing",
    )
    ctx.expect_gap(
        timer_knob,
        body,
        axis="z",
        positive_elem="cap",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.00001,
        name="timer knob sits on top cap",
    )
    ctx.expect_gap(
        speed_knob,
        body,
        axis="z",
        positive_elem="cap",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.00001,
        name="speed knob sits on top cap",
    )

    rest_front = aabb_center(body, "front_spine")
    with ctx.pose({osc: osc.motion_limits.upper}):
        swung_front = aabb_center(body, "front_spine")
    ctx.check(
        "oscillation pose swings the whole tower face",
        rest_front is not None
        and swung_front is not None
        and math.dist(rest_front[:2], swung_front[:2]) > 0.025,
        details=f"rest={rest_front}, swung={swung_front}",
    )

    timer_rest = aabb_center(timer_knob, "pointer")
    speed_rest = aabb_center(speed_knob, "pointer")
    with ctx.pose({timer_spin: math.pi / 2.0}):
        timer_turned = aabb_center(timer_knob, "pointer")
        speed_still = aabb_center(speed_knob, "pointer")
    ctx.check(
        "timer knob rotates without driving speed knob",
        timer_rest is not None
        and timer_turned is not None
        and speed_rest is not None
        and speed_still is not None
        and math.dist(timer_rest[:2], timer_turned[:2]) > 0.006
        and math.dist(speed_rest[:2], speed_still[:2]) < 0.001,
        details=f"timer_rest={timer_rest}, timer_turned={timer_turned}, speed_rest={speed_rest}, speed_still={speed_still}",
    )

    speed_rest = aabb_center(speed_knob, "pointer")
    with ctx.pose({speed_spin: speed_spin.motion_limits.upper}):
        speed_turned = aabb_center(speed_knob, "pointer")
    ctx.check(
        "speed knob rotates independently",
        speed_rest is not None
        and speed_turned is not None
        and math.dist(speed_rest[:2], speed_turned[:2]) > 0.006,
        details=f"speed_rest={speed_rest}, speed_turned={speed_turned}",
    )

    return ctx.report()


object_model = build_object_model()
