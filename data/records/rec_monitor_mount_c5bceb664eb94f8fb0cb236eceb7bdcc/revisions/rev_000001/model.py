from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobRelief,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BLACK = Material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
GRAPHITE = Material("satin_graphite", rgba=(0.11, 0.115, 0.12, 1.0))
STEEL = Material("brushed_steel", rgba=(0.55, 0.57, 0.58, 1.0))
RUBBER = Material("dark_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
SCREEN = Material("dark_display_back", rgba=(0.03, 0.035, 0.04, 1.0))


def _rounded_plate(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            height,
        ),
        name,
    )


def _lobed_knob(diameter: float, height: float, name: str):
    return mesh_from_geometry(
        KnobGeometry(
            diameter,
            height,
            body_style="lobed",
            base_diameter=diameter * 0.72,
            top_diameter=diameter * 0.92,
            crown_radius=0.0015,
            bore=KnobBore(style="round", diameter=diameter * 0.18),
            grip=KnobGrip(style="ribbed", count=8, depth=0.0012),
            body_reliefs=(KnobRelief(style="top_recess", width=diameter * 0.34, depth=0.0012),),
        ),
        name,
    )


def _cyl_y(part, radius: float, length: float, xyz, *, material, name: str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_x(part, radius: float, length: float, xyz, *, material, name: str):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="engineered_monitor_mount")
    model.material("matte_black", rgba=BLACK.rgba)
    model.material("satin_graphite", rgba=GRAPHITE.rgba)
    model.material("brushed_steel", rgba=STEEL.rgba)
    model.material("dark_rubber", rgba=RUBBER.rgba)
    model.material("dark_display_back", rgba=SCREEN.rgba)

    base_plate_mesh = _rounded_plate(0.260, 0.200, 0.030, 0.035, "rounded_desk_plate")
    display_corner_mesh = _rounded_plate(0.360, 0.580, 0.018, 0.030, "rounded_monitor_back")
    vesa_plate_mesh = _rounded_plate(0.180, 0.180, 0.016, 0.018, "rounded_vesa_plate")

    desk_clamp = model.part("desk_clamp")
    desk_clamp.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=BLACK,
        name="rounded_top_plate",
    )
    desk_clamp.visual(
        Box((0.205, 0.145, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.032)),
        material=RUBBER,
        name="desk_grip_pad",
    )
    desk_clamp.visual(
        Box((0.028, 0.170, 0.182)),
        origin=Origin(xyz=(-0.115, 0.0, -0.058)),
        material=BLACK,
        name="rear_clamp_wall",
    )
    desk_clamp.visual(
        Box((0.180, 0.150, 0.022)),
        origin=Origin(xyz=(-0.042, 0.0, -0.151)),
        material=BLACK,
        name="lower_clamp_jaw",
    )
    desk_clamp.visual(
        Cylinder(radius=0.012, length=0.140),
        origin=Origin(xyz=(0.040, 0.0, -0.205)),
        material=STEEL,
        name="clamp_screw",
    )
    desk_clamp.visual(
        Cylinder(radius=0.042, length=0.014),
        origin=Origin(xyz=(0.040, 0.0, -0.130)),
        material=RUBBER,
        name="pressure_pad",
    )
    desk_clamp.visual(
        Cylinder(radius=0.036, length=0.026),
        origin=Origin(xyz=(0.040, 0.0, -0.286)),
        material=RUBBER,
        name="hand_knob",
    )
    desk_clamp.visual(
        Cylinder(radius=0.032, length=0.362),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=STEEL,
        name="fixed_post",
    )
    desk_clamp.visual(
        Cylinder(radius=0.038, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.384)),
        material=BLACK,
        name="post_cap",
    )

    base_swivel = model.part("base_swivel")
    base_swivel.visual(
        Cylinder(radius=0.045, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=GRAPHITE,
        name="swivel_collar",
    )
    base_swivel.visual(
        Cylinder(radius=0.055, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=BLACK,
        name="turret_cap",
    )
    base_swivel.visual(
        Box((0.110, 0.100, 0.032)),
        origin=Origin(xyz=(0.045, 0.0, 0.062)),
        material=BLACK,
        name="shoulder_bridge",
    )
    for y, suffix in ((0.043, "0"), (-0.043, "1")):
        base_swivel.visual(
            Box((0.045, 0.018, 0.090)),
            origin=Origin(xyz=(0.100, y, 0.115)),
            material=BLACK,
            name=f"shoulder_cheek_{suffix}",
        )
    _cyl_y(base_swivel, 0.0215, 0.018, (0.100, -0.061, 0.118), material=RUBBER, name="shoulder_tension_knob")

    lower_arm = model.part("lower_arm")
    _cyl_y(lower_arm, 0.026, 0.052, (0.0, 0.0, 0.0), material=GRAPHITE, name="shoulder_hub")
    for y, suffix in ((0.032, "0"), (-0.032, "1")):
        lower_arm.visual(
            Box((0.310, 0.022, 0.032)),
            origin=Origin(xyz=(0.190, y, 0.016)),
            material=GRAPHITE,
            name=f"parallel_tube_{suffix}",
        )
        lower_arm.visual(
            Box((0.055, 0.022, 0.032)),
            origin=Origin(xyz=(0.036, y, 0.016)),
            material=GRAPHITE,
            name=f"shoulder_link_{suffix}",
        )
    lower_arm.visual(
        Box((0.285, 0.074, 0.018)),
        origin=Origin(xyz=(0.195, 0.0, 0.045)),
        material=BLACK,
        name="upper_cover",
    )
    _cyl_x(lower_arm, 0.012, 0.220, (0.170, 0.0, -0.025), material=STEEL, name="gas_spring_body")
    _cyl_x(lower_arm, 0.007, 0.155, (0.290, 0.0, -0.025), material=STEEL, name="gas_spring_rod")
    for x, suffix in ((0.060, "front"), (0.295, "rear")):
        lower_arm.visual(
            Box((0.026, 0.020, 0.070)),
            origin=Origin(xyz=(x, 0.0, 0.002)),
            material=BLACK,
            name=f"{suffix}_spring_link",
        )
    for y, suffix in ((0.043, "0"), (-0.043, "1")):
        lower_arm.visual(
            Box((0.055, 0.018, 0.080)),
            origin=Origin(xyz=(0.360, y, 0.025)),
            material=GRAPHITE,
            name=f"elbow_cheek_{suffix}",
        )
    _cyl_y(lower_arm, 0.028, 0.006, (0.360, 0.054, 0.025), material=BLACK, name="elbow_cap_0")
    _cyl_y(lower_arm, 0.028, 0.006, (0.360, -0.054, 0.025), material=BLACK, name="elbow_cap_1")

    upper_arm = model.part("upper_arm")
    _cyl_y(upper_arm, 0.026, 0.052, (0.0, 0.0, 0.0), material=GRAPHITE, name="elbow_hub")
    upper_arm.visual(
        Box((0.290, 0.058, 0.050)),
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        material=GRAPHITE,
        name="main_beam",
    )
    upper_arm.visual(
        Box((0.270, 0.046, 0.018)),
        origin=Origin(xyz=(0.190, 0.0, 0.032)),
        material=BLACK,
        name="top_insert",
    )
    upper_arm.visual(
        Box((0.230, 0.018, 0.018)),
        origin=Origin(xyz=(0.185, 0.037, -0.033)),
        material=RUBBER,
        name="cable_clip",
    )
    _cyl_y(upper_arm, 0.0215, 0.018, (0.150, 0.038, 0.036), material=RUBBER, name="elbow_tension_knob")
    for y, suffix in ((0.043, "0"), (-0.043, "1")):
        upper_arm.visual(
            Box((0.060, 0.024, 0.020)),
            origin=Origin(xyz=(0.305, y * 0.79, 0.037)),
            material=GRAPHITE,
            name=f"wrist_strap_{suffix}",
        )
        upper_arm.visual(
            Box((0.055, 0.018, 0.080)),
            origin=Origin(xyz=(0.340, y, 0.0)),
            material=GRAPHITE,
            name=f"wrist_cheek_{suffix}",
        )
    _cyl_y(upper_arm, 0.008, 0.088, (0.340, 0.0, 0.0), material=STEEL, name="wrist_pin")

    wrist = model.part("wrist")
    _cyl_y(wrist, 0.026, 0.052, (0.0, 0.0, 0.0), material=GRAPHITE, name="wrist_hub")
    wrist.visual(
        Box((0.075, 0.050, 0.045)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=GRAPHITE,
        name="tilt_neck",
    )
    _cyl_x(wrist, 0.032, 0.020, (0.100, 0.0, 0.0), material=BLACK, name="roll_bearing_face")
    _cyl_y(wrist, 0.0215, 0.018, (0.065, -0.033, 0.0), material=RUBBER, name="tilt_tension_knob")

    display_bracket = model.part("display_bracket")
    _cyl_x(display_bracket, 0.038, 0.020, (0.010, 0.0, 0.0), material=GRAPHITE, name="roll_boss")
    display_bracket.visual(
        vesa_plate_mesh,
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=BLACK,
        name="vesa_plate",
    )
    for rz, ry, angle, suffix in (
        (0.046, 0.046, -0.78, "0"),
        (0.046, -0.046, 0.78, "1"),
        (-0.046, 0.046, 0.78, "2"),
        (-0.046, -0.046, -0.78, "3"),
    ):
        display_bracket.visual(
            Box((0.014, 0.024, 0.145)),
            origin=Origin(xyz=(0.024, ry, rz), rpy=(angle, 0.0, 0.0)),
            material=GRAPHITE,
            name=f"vesa_spoke_{suffix}",
        )
    for y in (-0.050, 0.050):
        for z in (-0.050, 0.050):
            _cyl_x(
                display_bracket,
                0.0075,
                0.010,
                (0.017, y, z),
                material=STEEL,
                name=f"vesa_screw_{'p' if y > 0 else 'n'}y_{'p' if z > 0 else 'n'}z",
            )
            _cyl_x(
                display_bracket,
                0.010,
                0.020,
                (0.049, y, z),
                material=BLACK,
                name=f"standoff_{'p' if y > 0 else 'n'}y_{'p' if z > 0 else 'n'}z",
            )
    display_bracket.visual(
        display_corner_mesh,
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=SCREEN,
        name="monitor_back",
    )
    display_bracket.visual(
        Box((0.004, 0.545, 0.315)),
        origin=Origin(xyz=(0.0765, 0.0, 0.0)),
        material=Material("dark_glass", rgba=(0.005, 0.007, 0.010, 1.0)),
        name="thin_front_glass",
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=desk_clamp,
        child=base_swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=-pi, upper=pi),
        motion_properties=MotionProperties(damping=0.15, friction=0.08),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=base_swivel,
        child=lower_arm,
        origin=Origin(xyz=(0.110, 0.0, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.9, lower=-0.65, upper=0.55),
        motion_properties=MotionProperties(damping=0.35, friction=0.18),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.360, 0.0, 0.025)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.0, lower=-0.95, upper=0.95),
        motion_properties=MotionProperties(damping=0.28, friction=0.12),
    )
    model.articulation(
        "wrist_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=wrist,
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.2, lower=-0.75, upper=0.75),
        motion_properties=MotionProperties(damping=0.18, friction=0.10),
    )
    model.articulation(
        "display_roll",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=display_bracket,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-pi / 2.0, upper=pi / 2.0),
        motion_properties=MotionProperties(damping=0.10, friction=0.12),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("desk_clamp")
    swivel = object_model.get_part("base_swivel")
    upper = object_model.get_part("upper_arm")
    bracket = object_model.get_part("display_bracket")
    wrist = object_model.get_part("wrist")

    ctx.allow_overlap(
        upper,
        wrist,
        elem_a="wrist_pin",
        elem_b="wrist_hub",
        reason="The steel wrist pin is intentionally captured inside the tilt hub to make the load-bearing hinge read correctly.",
    )
    ctx.expect_overlap(
        upper,
        wrist,
        axes="y",
        min_overlap=0.045,
        elem_a="wrist_pin",
        elem_b="wrist_hub",
        name="wrist pin spans through the captured hub",
    )
    ctx.expect_within(
        upper,
        wrist,
        axes="xz",
        margin=0.002,
        inner_elem="wrist_pin",
        outer_elem="wrist_hub",
        name="wrist pin is centered inside the tilt hub",
    )

    ctx.expect_gap(
        swivel,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0005,
        positive_elem="swivel_collar",
        negative_elem="post_cap",
        name="swivel collar is seated on the fixed post",
    )
    ctx.expect_overlap(
        swivel,
        base,
        axes="xy",
        min_overlap=0.060,
        elem_a="swivel_collar",
        elem_b="fixed_post",
        name="swivel collar is centered over the support post",
    )
    ctx.expect_origin_gap(
        bracket,
        base,
        axis="x",
        min_gap=0.65,
        name="arm carries the display well out from the desk clamp",
    )
    ctx.expect_gap(
        bracket,
        base,
        axis="z",
        min_gap=0.18,
        positive_elem="monitor_back",
        negative_elem="rounded_top_plate",
        name="display bracket sits at monitor height above the desk base",
    )

    joints = {
        name: object_model.get_articulation(name)
        for name in ("base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_tilt", "display_roll")
    }
    for name, joint in joints.items():
        limits = joint.motion_limits
        ctx.check(
            f"{name} has realistic bounded motion",
            limits is not None and limits.lower is not None and limits.upper is not None and limits.upper > limits.lower,
            details=f"limits={limits}",
        )

    rest_display = ctx.part_world_position(bracket)
    with ctx.pose({"base_yaw": 0.65}):
        yawed_display = ctx.part_world_position(bracket)
    ctx.check(
        "base yaw swings the supported display laterally",
        rest_display is not None
        and yawed_display is not None
        and abs(yawed_display[1] - rest_display[1]) > 0.35,
        details=f"rest={rest_display}, yawed={yawed_display}",
    )

    rest_wrist = ctx.part_world_position(wrist)
    with ctx.pose({"shoulder_pitch": -0.35, "elbow_pitch": 0.55, "wrist_tilt": -0.20}):
        posed_wrist = ctx.part_world_position(wrist)
        posed_bracket = ctx.part_world_position(bracket)
    ctx.check(
        "pitch joints change the arm reach without detaching the display mount",
        rest_wrist is not None
        and posed_wrist is not None
        and posed_bracket is not None
        and abs(posed_wrist[2] - rest_wrist[2]) > 0.025
        and posed_bracket[0] > 0.45,
        details=f"rest_wrist={rest_wrist}, posed_wrist={posed_wrist}, posed_bracket={posed_bracket}",
    )

    return ctx.report()


object_model = build_object_model()
