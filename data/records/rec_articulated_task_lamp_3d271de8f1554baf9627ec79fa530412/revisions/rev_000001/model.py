from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cyl_x(part, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, radius: float, length: float, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _spring_mesh(length: float, *, turns: int, radius: float, wire_radius: float, z: float, name: str):
    points: list[tuple[float, float, float]] = [(0.0, 0.0, 0.010), (0.035, 0.0, z)]
    samples = turns * 18
    start = 0.055
    end = length - 0.055
    for i in range(samples + 1):
        t = i / samples
        x = start + (end - start) * t
        a = 2.0 * math.pi * turns * t
        points.append((x, radius * math.cos(a), z + radius * math.sin(a)))
    points.extend([(length - 0.035, 0.0, z), (length, 0.0, 0.010)])
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=wire_radius,
            samples_per_segment=4,
            radial_segments=10,
            cap_ends=True,
        ),
        name,
    )


def _add_arm_segment(part, *, length: float, rod_radius: float, half_width: float, material, spring_material, prefix: str):
    if prefix == "lower":
        pivot_name, tip_name, spring_name = "lower_pivot", "lower_tip", "lower_spring"
    elif prefix == "upper":
        pivot_name, tip_name, spring_name = "upper_pivot", "upper_tip", "upper_spring"
    else:
        pivot_name, tip_name, spring_name = f"{prefix}_pivot", f"{prefix}_tip", f"{prefix}_spring"
    rod_length = length - 0.050
    _cyl_x(part, rod_radius, rod_length, (length / 2.0, half_width, 0.0), material, f"{prefix}_rod_0")
    _cyl_x(part, rod_radius, rod_length, (length / 2.0, -half_width, 0.0), material, f"{prefix}_rod_1")
    pivot_length = half_width * 2.0
    _cyl_y(part, 0.018, pivot_length, (0.0, 0.0, 0.0), material, pivot_name)
    _cyl_y(part, 0.018, pivot_length, (length, 0.0, 0.0), material, tip_name)
    part.visual(
        Box((0.070, half_width * 2.0 + 0.020, 0.014)),
        origin=Origin(xyz=(0.045, 0.0, -0.006)),
        material=material,
        name=f"{prefix}_root_bridge",
    )
    part.visual(
        Box((0.070, half_width * 2.0 + 0.020, 0.014)),
        origin=Origin(xyz=(length - 0.045, 0.0, -0.006)),
        material=material,
        name=f"{prefix}_tip_bridge",
    )
    part.visual(
        _spring_mesh(
            length,
            turns=10 if length > 0.34 else 8,
            radius=0.010,
            wire_radius=0.0024,
            z=0.033,
            name=f"{prefix}_spring_mesh",
        ),
        material=spring_material,
        name=spring_name,
    )


def _make_knob_mesh(name: str):
    return mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.020,
            body_style="lobed",
            base_diameter=0.034,
            top_diameter=0.040,
            crown_radius=0.0015,
            grip=KnobGrip(style="ribbed", count=8, depth=0.0014),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        ),
        name,
    )


def _shade_shell(name: str):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.030, -0.012),
                (0.043, -0.040),
                (0.060, -0.135),
                (0.067, -0.150),
            ],
            inner_profile=[
                (0.022, -0.018),
                (0.034, -0.045),
                (0.053, -0.134),
                (0.060, -0.143),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=6,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_task_lamp")

    graphite = model.material("satin_graphite", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_edge = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    warm_steel = model.material("warm_brushed_steel", rgba=(0.82, 0.78, 0.68, 1.0))
    reflector = model.material("soft_reflector", rgba=(0.92, 0.88, 0.78, 1.0))
    brass = model.material("brass_bushings", rgba=(0.78, 0.58, 0.24, 1.0))
    warm_light = model.material("warm_lamp_glow", rgba=(1.0, 0.82, 0.43, 0.88))

    lower_len = 0.380
    upper_len = 0.335

    base = model.part("base")
    base.visual(Cylinder(radius=0.145, length=0.034), origin=Origin(xyz=(0.0, 0.0, 0.017)), material=graphite, name="weighted_disk")
    base.visual(Cylinder(radius=0.151, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.006)), material=dark_edge, name="rubber_ring")
    base.visual(Cylinder(radius=0.060, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.039)), material=steel, name="top_trim")
    for i, (x, y) in enumerate(((0.090, 0.070), (-0.090, 0.070), (0.090, -0.070), (-0.090, -0.070))):
        base.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(x, y, 0.002)),
            material=dark_edge,
            name=f"foot_{i}",
        )

    turntable = model.part("turntable")
    turntable.visual(Cylinder(radius=0.046, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=graphite, name="swivel_plate")
    turntable.visual(Cylinder(radius=0.027, length=0.082), origin=Origin(xyz=(0.0, 0.0, 0.050)), material=graphite, name="pedestal")
    turntable.visual(Box((0.076, 0.104, 0.020)), origin=Origin(xyz=(0.012, 0.0, 0.082)), material=graphite, name="yoke_bridge")
    turntable.visual(Box((0.060, 0.012, 0.085)), origin=Origin(xyz=(0.012, 0.048, 0.120)), material=graphite, name="yoke_cheek_0")
    turntable.visual(Box((0.060, 0.012, 0.085)), origin=Origin(xyz=(0.012, -0.048, 0.120)), material=graphite, name="yoke_cheek_1")
    _cyl_y(turntable, 0.014, 0.120, (0.0, 0.0, 0.135), steel, "shoulder_pin")
    _cyl_y(turntable, 0.020, 0.020, (0.0, 0.059, 0.135), brass, "shoulder_bushing")

    lower_arm = model.part("lower_arm")
    _add_arm_segment(lower_arm, length=lower_len, rod_radius=0.0085, half_width=0.026, material=steel, spring_material=warm_steel, prefix="lower")
    lower_arm.visual(Box((0.048, 0.062, 0.012)), origin=Origin(xyz=(lower_len - 0.055, 0.0, -0.018)), material=graphite, name="elbow_plate")

    upper_arm = model.part("upper_arm")
    _add_arm_segment(upper_arm, length=upper_len, rod_radius=0.0075, half_width=0.023, material=steel, spring_material=warm_steel, prefix="upper")
    upper_arm.visual(Box((0.052, 0.056, 0.012)), origin=Origin(xyz=(0.090, 0.0, -0.007)), material=graphite, name="inner_elbow_plate")
    upper_arm.visual(Box((0.054, 0.052, 0.014)), origin=Origin(xyz=(upper_len - 0.035, 0.0, -0.018)), material=graphite, name="wrist_mount")

    wrist = model.part("wrist")
    wrist.visual(Cylinder(radius=0.023, length=0.034), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=graphite, name="swivel_hub")
    _cyl_y(wrist, 0.014, 0.070, (0.040, 0.0, 0.0), steel, "head_pin")
    wrist.visual(Box((0.045, 0.011, 0.052)), origin=Origin(xyz=(0.040, 0.031, -0.004)), material=graphite, name="head_cheek_0")
    wrist.visual(Box((0.045, 0.011, 0.052)), origin=Origin(xyz=(0.040, -0.031, -0.004)), material=graphite, name="head_cheek_1")
    wrist.visual(Box((0.018, 0.070, 0.014)), origin=Origin(xyz=(0.022, 0.0, -0.028)), material=graphite, name="fork_bridge")

    head = model.part("head")
    head.visual(_shade_shell("shade_shell"), origin=Origin(xyz=(0.0, 0.0, -0.060)), material=graphite, name="shade_shell")
    head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.064, tube=0.004, radial_segments=18, tubular_segments=72), "front_lip"),
        origin=Origin(xyz=(0.0, 0.0, -0.207)),
        material=dark_edge,
        name="front_lip",
    )
    head.visual(Cylinder(radius=0.024, length=0.038), origin=Origin(xyz=(0.0, 0.0, -0.098)), material=reflector, name="lamp_socket")
    head.visual(Sphere(radius=0.024), origin=Origin(xyz=(0.0, 0.0, -0.136)), material=warm_light, name="bulb")
    _cyl_y(head, 0.018, 0.048, (0.0, 0.0, 0.0), steel, "tilt_barrel")
    head.visual(Cylinder(radius=0.024, length=0.078), origin=Origin(xyz=(0.0, 0.0, -0.039)), material=graphite, name="neck_spigot")
    head.visual(Cylinder(radius=0.034, length=0.020), origin=Origin(xyz=(0.0, 0.0, -0.071)), material=graphite, name="rear_cap")

    shoulder_knob = model.part("shoulder_knob")
    shoulder_knob.visual(_make_knob_mesh("shoulder_knob_mesh"), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=graphite, name="knob_cap")
    _cyl_y(shoulder_knob, 0.006, 0.054, (0.0, 0.027, 0.0), steel, "knob_stem")

    elbow_knob = model.part("elbow_knob")
    elbow_knob.visual(_make_knob_mesh("elbow_knob_mesh"), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=graphite, name="knob_cap")
    _cyl_y(elbow_knob, 0.0055, 0.052, (0.0, 0.026, 0.0), steel, "knob_stem")

    wrist_knob = model.part("wrist_knob")
    wrist_knob.visual(_make_knob_mesh("wrist_knob_mesh"), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=graphite, name="knob_cap")
    _cyl_y(wrist_knob, 0.005, 0.044, (0.0, 0.022, 0.0), steel, "knob_stem")

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.4, lower=-2.8, upper=2.8),
    )
    model.articulation(
        "turntable_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.135), rpy=(0.0, -math.radians(55.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.0, lower=-0.65, upper=0.95),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(lower_len, 0.0, 0.0), rpy=(0.0, math.radians(55.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.1, lower=-0.95, upper=1.10),
    )
    model.articulation(
        "upper_arm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=wrist,
        origin=Origin(xyz=(upper_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "wrist_to_head",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=head,
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, -math.radians(30.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.2, lower=-0.50, upper=0.80),
    )
    model.articulation(
        "turntable_to_shoulder_knob",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=shoulder_knob,
        origin=Origin(xyz=(0.0, -0.078, 0.135)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "lower_arm_to_elbow_knob",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=elbow_knob,
        origin=Origin(xyz=(lower_len, -0.055, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "wrist_to_wrist_knob",
        ArticulationType.REVOLUTE,
        parent=wrist,
        child=wrist_knob,
        origin=Origin(xyz=(0.040, -0.050, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    wrist = object_model.get_part("wrist")
    head = object_model.get_part("head")
    shoulder_knob = object_model.get_part("shoulder_knob")
    elbow_knob = object_model.get_part("elbow_knob")
    wrist_knob = object_model.get_part("wrist_knob")

    shoulder = object_model.get_articulation("turntable_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    wrist_swivel = object_model.get_articulation("upper_arm_to_wrist")
    head_tilt = object_model.get_articulation("wrist_to_head")

    ctx.allow_overlap(turntable, lower_arm, reason="The shoulder is a compact pinned yoke with captured bushings and a spring hook.")
    ctx.allow_overlap(lower_arm, upper_arm, reason="The elbow is a stacked hinge with coaxial collars, washers, and spring hooks.")
    ctx.allow_overlap(upper_arm, wrist, reason="The wrist hub is seated in a close saddle at the upper arm tip.")
    ctx.allow_overlap(wrist, head, reason="The head trunnion is captured inside the wrist fork.")
    ctx.allow_overlap(turntable, shoulder_knob, reason="The shoulder thumb wheel bears on and threads through the yoke hardware.")
    ctx.allow_overlap(lower_arm, shoulder_knob, reason="The shoulder knob stem clamps the lower arm pivot collar.")
    ctx.allow_overlap(lower_arm, elbow_knob, reason="The elbow knob stem clamps the lower arm side of the hinge.")
    ctx.allow_overlap(upper_arm, elbow_knob, reason="The elbow knob stem clamps the upper arm side of the hinge.")
    ctx.allow_overlap(wrist, wrist_knob, reason="The wrist knob stem passes through the fork cheek and pin hardware.")
    ctx.allow_overlap(head, wrist_knob, reason="The wrist knob stem clamps the head neck at the tilt joint.")

    ctx.expect_gap(turntable, base, axis="z", max_gap=0.002, max_penetration=0.0, positive_elem="swivel_plate", negative_elem="top_trim", name="turntable sits on weighted base")
    ctx.expect_contact(turntable, lower_arm, contact_tol=0.010, name="shoulder joint is physically captured")
    ctx.expect_contact(lower_arm, upper_arm, contact_tol=0.010, name="elbow joint is physically captured")
    ctx.expect_contact(upper_arm, wrist, contact_tol=0.010, name="wrist joint is physically captured")
    ctx.expect_contact(wrist, head, contact_tol=0.010, name="head tilt joint is physically captured")
    ctx.expect_contact(shoulder_knob, turntable, contact_tol=0.010, name="shoulder knob bears on yoke")
    ctx.expect_contact(shoulder_knob, lower_arm, contact_tol=0.010, name="shoulder knob clamps pivot collar")
    ctx.expect_contact(elbow_knob, lower_arm, contact_tol=0.010, name="elbow knob bears on lower collar")
    ctx.expect_contact(elbow_knob, upper_arm, contact_tol=0.010, name="elbow knob clamps upper collar")
    ctx.expect_contact(wrist_knob, wrist, contact_tol=0.010, name="wrist knob bears on fork")
    ctx.expect_contact(wrist_knob, head, contact_tol=0.010, name="wrist knob clamps head neck")
    ctx.expect_contact(head, head, elem_a="lamp_socket", elem_b="bulb", contact_tol=0.003, name="bulb seats in socket")

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({shoulder: -0.45, elbow: -0.55, wrist_swivel: 0.60, head_tilt: 0.45}):
        raised_head_pos = ctx.part_world_position(head)
        ctx.expect_contact(wrist, head, contact_tol=0.010, name="head remains pinned while aimed")
    ctx.check(
        "arm raises the focused head",
        rest_head_pos is not None and raised_head_pos is not None and raised_head_pos[2] > rest_head_pos[2] + 0.035,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    return ctx.report()


object_model = build_object_model()
