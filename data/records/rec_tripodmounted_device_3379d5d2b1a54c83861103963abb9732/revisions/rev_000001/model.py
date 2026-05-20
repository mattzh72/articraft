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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="surveying_tripod")

    yellow = model.material("tripod_yellow", rgba=(0.95, 0.72, 0.20, 1.0))
    dark = model.material("matte_black", rgba=(0.02, 0.025, 0.025, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    metal = model.material("brushed_metal", rgba=(0.56, 0.58, 0.57, 1.0))
    green = model.material("instrument_green", rgba=(0.11, 0.34, 0.27, 1.0))
    glass = model.material("blue_glass", rgba=(0.20, 0.42, 0.68, 0.82))
    white = model.material("white_mark", rgba=(0.9, 0.88, 0.78, 1.0))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.105, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.130)),
        material=dark,
        name="crown_plate",
    )
    crown.visual(
        Cylinder(radius=0.045, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 1.225)),
        material=metal,
        name="center_column",
    )
    crown.visual(
        Cylinder(radius=0.090, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 1.2875)),
        material=dark,
        name="top_collar",
    )

    hinge_radius = 0.130
    hinge_height = 1.130
    arm_length = 0.080
    arm_center_radius = 0.065
    for i in range(3):
        yaw = i * 2.0 * math.pi / 3.0
        crown.visual(
            Box((arm_length, 0.035, 0.035)),
            origin=Origin(
                xyz=(
                    math.cos(yaw) * arm_center_radius,
                    math.sin(yaw) * arm_center_radius,
                    hinge_height,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark,
            name=f"hinge_arm_{i}",
        )

    leg_dx = 0.550
    leg_dz = -1.120
    leg_length = math.hypot(leg_dx, leg_dz)
    leg_pitch = math.atan2(leg_dx, leg_dz)
    for i in range(3):
        leg = model.part(f"leg_{i}")
        leg.visual(
            Cylinder(radius=0.025, length=0.090),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="hinge_barrel",
        )
        leg.visual(
            Cylinder(radius=0.019, length=leg_length),
            origin=Origin(
                xyz=(leg_dx * 0.5, 0.0, leg_dz * 0.5),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=yellow,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.029, length=0.075),
            origin=Origin(
                xyz=(leg_dx * 0.48, 0.0, leg_dz * 0.48),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=dark,
            name="leg_clamp",
        )
        leg.visual(
            Cylinder(radius=0.034, length=0.045),
            origin=Origin(
                xyz=(leg_dx, 0.0, leg_dz),
                rpy=(0.0, leg_pitch, 0.0),
            ),
            material=rubber,
            name="foot",
        )
        yaw = i * 2.0 * math.pi / 3.0
        model.articulation(
            f"crown_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(
                xyz=(math.cos(yaw) * hinge_radius, math.sin(yaw) * hinge_radius, hinge_height),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=50.0, velocity=1.2, lower=-0.28, upper=0.42),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.118, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.095, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=metal,
        name="graduated_ring",
    )
    pan_head.visual(
        Box((0.125, 0.165, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=green,
        name="pedestal",
    )
    pan_head.visual(
        Box((0.165, 0.340, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=green,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.090, 0.050, 0.200)),
        origin=Origin(xyz=(0.0, 0.145, 0.230)),
        material=green,
        name="yoke_cheek_0",
    )
    pan_head.visual(
        Box((0.090, 0.050, 0.200)),
        origin=Origin(xyz=(0.0, -0.145, 0.230)),
        material=green,
        name="yoke_cheek_1",
    )
    pan_head.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.0, 0.134, 0.250), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pivot_boss_0",
    )
    pan_head.visual(
        Cylinder(radius=0.036, length=0.028),
        origin=Origin(xyz=(0.0, -0.134, 0.250), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pivot_boss_1",
    )
    model.articulation(
        "crown_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.055, length=0.560),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=green,
        name="scope_tube",
    )
    body.visual(
        Box((0.240, 0.150, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=green,
        name="tilt_housing",
    )
    body.visual(
        Cylinder(radius=0.070, length=0.045),
        origin=Origin(xyz=(0.3025, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="objective_rim",
    )
    body.visual(
        Cylinder(radius=0.054, length=0.006),
        origin=Origin(xyz=(0.328, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    body.visual(
        Cylinder(radius=0.037, length=0.070),
        origin=Origin(xyz=(-0.315, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="eyepiece",
    )
    body.visual(
        Box((0.060, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, 0.091, 0.0)),
        material=metal,
        name="trunnion_block_0",
    )
    body.visual(
        Box((0.060, 0.032, 0.052)),
        origin=Origin(xyz=(0.0, -0.091, 0.0)),
        material=metal,
        name="trunnion_block_1",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.048),
        origin=Origin(xyz=(0.0, 0.096, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="trunnion_0",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.048),
        origin=Origin(xyz=(0.0, -0.096, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="trunnion_1",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.0885),
        origin=Origin(xyz=(0.110, 0.11825, -0.022), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="leveling_shaft",
    )
    body.visual(
        Box((0.075, 0.010, 0.018)),
        origin=Origin(xyz=(-0.070, 0.0, 0.075)),
        material=white,
        name="bubble_level",
    )
    model.articulation(
        "pan_head_to_body",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.55, upper=0.55),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.035,
            body_style="faceted",
            top_diameter=0.050,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0018),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        ),
        "leveling_knob_mesh",
    )
    leveling_knob = model.part("leveling_knob")
    leveling_knob.visual(
        knob_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="knob_cap",
    )
    leveling_knob.visual(
        Sphere(radius=0.0045),
        origin=Origin(xyz=(0.0, 0.019, 0.018)),
        material=white,
        name="knob_marker",
    )
    model.articulation(
        "body_to_leveling_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=leveling_knob,
        origin=Origin(xyz=(0.110, 0.180, -0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    body = object_model.get_part("body")
    knob = object_model.get_part("leveling_knob")

    leg_joints = [object_model.get_articulation(f"crown_to_leg_{i}") for i in range(3)]
    pan_joint = object_model.get_articulation("crown_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_body")
    knob_joint = object_model.get_articulation("body_to_leveling_knob")

    ctx.check(
        "three crown leg hinges",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in leg_joints),
        details="All three tripod legs should be separate revolute hinge links at the crown.",
    )
    ctx.check(
        "continuous pan and knob joints",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"pan={pan_joint.articulation_type}, knob={knob_joint.articulation_type}",
    )
    ctx.check(
        "limited body tilt hinge",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.motion_limits is not None
        and tilt_joint.motion_limits.lower < 0.0
        and tilt_joint.motion_limits.upper > 0.0,
        details="The surveying body should tilt both up and down on a bounded horizontal hinge.",
    )

    ctx.expect_contact(
        pan_head,
        crown,
        elem_a="pan_base",
        elem_b="top_collar",
        contact_tol=0.001,
        name="pan bearing sits on crown collar",
    )
    for i in range(3):
        ctx.expect_contact(
            object_model.get_part(f"leg_{i}"),
            crown,
            elem_a="hinge_barrel",
            elem_b=f"hinge_arm_{i}",
            contact_tol=0.002,
            name=f"leg_{i} hinge barrel meets crown arm",
        )
    ctx.expect_gap(
        pan_head,
        body,
        axis="y",
        positive_elem="yoke_cheek_0",
        negative_elem="trunnion_0",
        min_gap=0.0,
        max_gap=0.008,
        name="positive trunnion is close to yoke cheek",
    )
    ctx.expect_gap(
        body,
        pan_head,
        axis="y",
        positive_elem="trunnion_1",
        negative_elem="yoke_cheek_1",
        min_gap=0.0,
        max_gap=0.008,
        name="negative trunnion is close to yoke cheek",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="y",
        positive_elem="knob_cap",
        negative_elem="leveling_shaft",
        min_gap=0.0,
        max_gap=0.002,
        name="leveling knob seats against its short shaft",
    )

    def elem_center_z(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[2] + hi[2]) * 0.5

    rest_lens_z = elem_center_z(body, "objective_lens")
    with ctx.pose({tilt_joint: 0.35}):
        raised_lens_z = elem_center_z(body, "objective_lens")
    ctx.check(
        "positive tilt raises objective end",
        rest_lens_z is not None and raised_lens_z is not None and raised_lens_z > rest_lens_z + 0.04,
        details=f"rest_z={rest_lens_z}, raised_z={raised_lens_z}",
    )

    def elem_center_x(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[0] + hi[0]) * 0.5

    leg_0 = object_model.get_part("leg_0")
    rest_foot_x = elem_center_x(leg_0, "foot")
    with ctx.pose({leg_joints[0]: 0.30}):
        hinged_foot_x = elem_center_x(leg_0, "foot")
    ctx.check(
        "leg hinge swings foot in radial plane",
        rest_foot_x is not None and hinged_foot_x is not None and abs(hinged_foot_x - rest_foot_x) > 0.05,
        details=f"rest_x={rest_foot_x}, hinged_x={hinged_foot_x}",
    )

    marker_rest = ctx.part_element_world_aabb(knob, elem="knob_marker")
    with ctx.pose({knob_joint: 1.0}):
        marker_rotated = ctx.part_element_world_aabb(knob, elem="knob_marker")
    if marker_rest is not None and marker_rotated is not None:
        rest_center = tuple((marker_rest[0][i] + marker_rest[1][i]) * 0.5 for i in range(3))
        rotated_center = tuple((marker_rotated[0][i] + marker_rotated[1][i]) * 0.5 for i in range(3))
        moved = abs(rest_center[0] - rotated_center[0]) + abs(rest_center[2] - rotated_center[2])
    else:
        rest_center = rotated_center = None
        moved = 0.0
    ctx.check(
        "leveling knob visibly rotates about side shaft",
        moved > 0.010,
        details=f"rest={rest_center}, rotated={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()
