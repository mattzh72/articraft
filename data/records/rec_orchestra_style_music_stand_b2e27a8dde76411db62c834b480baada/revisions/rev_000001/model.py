from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rpy_for_z_to_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return an RPY that points a local +Z cylinder along ``vector``."""
    x, y, z = vector
    planar = math.hypot(x, y)
    yaw = math.atan2(y, x) if planar > 1e-9 else 0.0
    pitch = math.atan2(planar, z)
    return (0.0, pitch, yaw)


def _cylinder_between(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material,
):
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    midpoint = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=midpoint, rpy=_rpy_for_z_to_vector((vx, vy, vz))),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_tripod_orchestra_stand")

    black = model.material("black_powder_coat", color=(0.015, 0.014, 0.013, 1.0))
    satin = model.material("satin_black_steel", color=(0.03, 0.032, 0.034, 1.0))
    worn = model.material("worn_edge_steel", color=(0.38, 0.37, 0.34, 1.0))
    rubber = model.material("black_rubber", color=(0.005, 0.005, 0.005, 1.0))
    knob_mat = model.material("matte_plastic_knob", color=(0.02, 0.02, 0.022, 1.0))

    lower_mast = model.part("lower_mast")
    lower_mast.visual(
        Cylinder(radius=0.021, length=0.73),
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
        material=satin,
        name="outer_tube",
    )
    lower_mast.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=black,
        name="folding_hub",
    )
    lower_mast.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.055)),
        material=black,
        name="collar_ring",
    )
    lower_mast.visual(
        Cylinder(radius=0.012, length=0.046),
        origin=Origin(xyz=(0.055, 0.0, 1.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="threaded_boss",
    )

    # Three folding tripod legs are pinned around the low hub.  Their part
    # frames sit on the hinge pins; the visible tubes angle down to the rubber
    # feet in the deployed rest pose.
    leg_joints = []
    hinge_radius = 0.064
    for index, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        radial = (math.cos(angle), math.sin(angle), 0.0)
        tangent = (-math.sin(angle), math.cos(angle), 0.0)
        leg = model.part(f"leg_{index}")
        _cylinder_between(
            leg,
            "hinge_barrel",
            (-0.027 * tangent[0], -0.027 * tangent[1], 0.0),
            (0.027 * tangent[0], 0.027 * tangent[1], 0.0),
            0.009,
            worn,
        )
        _cylinder_between(
            leg,
            "leg_socket",
            (0.0, 0.0, 0.0),
            (0.055 * radial[0], 0.055 * radial[1], -0.035),
            0.011,
            black,
        )
        leg_start = (0.055 * radial[0], 0.055 * radial[1], -0.035)
        leg_end = (0.62 * radial[0], 0.62 * radial[1], -0.395)
        _cylinder_between(leg, "splayed_tube", leg_start, leg_end, 0.0125, satin)
        leg.visual(
            Cylinder(radius=0.036, length=0.014),
            origin=Origin(xyz=(leg_end[0], leg_end[1], leg_end[2] - 0.006)),
            material=rubber,
            name="rubber_foot",
        )

        joint = model.articulation(
            f"lower_mast_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=lower_mast,
            child=leg,
            origin=Origin(xyz=(hinge_radius * radial[0], hinge_radius * radial[1], 0.43)),
            axis=(math.sin(angle), -math.cos(angle), 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.35),
        )
        leg_joints.append(joint)

    height_stage = model.part("height_stage")
    height_stage.visual(
        Cylinder(radius=0.0145, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=satin,
        name="sliding_tube",
    )
    height_stage.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=black,
        name="top_plug",
    )
    slide_joint = model.articulation(
        "lower_mast_to_height_stage",
        ArticulationType.PRISMATIC,
        parent=lower_mast,
        child=height_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.22, lower=0.0, upper=0.32),
    )

    collar_knob = model.part("collar_knob")
    _cylinder_between(collar_knob, "threaded_stud", (0.0, 0.0, 0.0), (0.040, 0.0, 0.0), 0.006, worn)
    collar_knob.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="knob_washer",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.056,
            0.028,
            body_style="lobed",
            base_diameter=0.040,
            top_diameter=0.050,
            crown_radius=0.002,
            bore=KnobBore(style="round", diameter=0.008),
        ),
        "collar_lobed_knob",
    )
    collar_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )
    knob_joint = model.articulation(
        "lower_mast_to_collar_knob",
        ArticulationType.CONTINUOUS,
        parent=lower_mast,
        child=collar_knob,
        origin=Origin(xyz=(0.078, 0.0, 1.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0),
    )

    desk_support = model.part("desk_support")
    desk_support.visual(
        Cylinder(radius=0.023, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black,
        name="head_socket",
    )
    _cylinder_between(desk_support, "support_arm", (0.0, 0.0, 0.052), (0.0, -0.078, 0.094), 0.012, satin)
    _cylinder_between(desk_support, "tilt_pin", (-0.14, -0.078, 0.094), (0.14, -0.078, 0.094), 0.008, worn)
    desk_support.visual(
        Box((0.040, 0.020, 0.050)),
        origin=Origin(xyz=(-0.105, -0.078, 0.070)),
        material=black,
        name="yoke_cheek_0",
    )
    desk_support.visual(
        Box((0.040, 0.020, 0.050)),
        origin=Origin(xyz=(0.105, -0.078, 0.070)),
        material=black,
        name="yoke_cheek_1",
    )
    model.articulation(
        "height_stage_to_desk_support",
        ArticulationType.FIXED,
        parent=height_stage,
        child=desk_support,
        origin=Origin(xyz=(0.0, 0.0, 0.534)),
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.560, 0.010, 0.360)),
        origin=Origin(xyz=(0.0, -0.030, -0.185)),
        material=black,
        name="stamped_back",
    )
    desk.visual(
        Box((0.560, 0.085, 0.014)),
        origin=Origin(xyz=(0.0, -0.063, -0.362)),
        material=black,
        name="lower_shelf",
    )
    desk.visual(
        Box((0.560, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, -0.103, -0.333)),
        material=black,
        name="deep_lower_lip",
    )
    desk.visual(
        Box((0.540, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.030, -0.006)),
        material=black,
        name="rolled_top_edge",
    )
    desk.visual(
        Box((0.018, 0.018, 0.330)),
        origin=Origin(xyz=(-0.278, -0.030, -0.185)),
        material=black,
        name="side_flange_0",
    )
    desk.visual(
        Box((0.018, 0.018, 0.330)),
        origin=Origin(xyz=(0.278, -0.030, -0.185)),
        material=black,
        name="side_flange_1",
    )
    for i, x in enumerate((-0.18, 0.0, 0.18)):
        desk.visual(
            Box((0.020, 0.006, 0.245)),
            origin=Origin(xyz=(x, -0.038, -0.185)),
            material=satin,
            name=f"pressed_rib_{i}",
        )
    desk.visual(
        Box((0.038, 0.038, 0.075)),
        origin=Origin(xyz=(-0.060, -0.010, -0.028)),
        material=black,
        name="hinge_strap_0",
    )
    desk.visual(
        Box((0.038, 0.038, 0.075)),
        origin=Origin(xyz=(0.060, -0.010, -0.028)),
        material=black,
        name="hinge_strap_1",
    )
    desk_hinge = model.articulation(
        "desk_support_to_desk",
        ArticulationType.REVOLUTE,
        parent=desk_support,
        child=desk,
        origin=Origin(xyz=(0.0, -0.078, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.6, lower=-0.35, upper=0.70),
    )

    # Keep references alive for linters and to document the three primary
    # mechanisms: folding legs, height slide, collar knob, and desk tilt.
    model.meta["mechanisms"] = {
        "leg_joints": [joint.name for joint in leg_joints],
        "height_slide": slide_joint.name,
        "collar_knob": knob_joint.name,
        "desk_tilt": desk_hinge.name,
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_mast = object_model.get_part("lower_mast")
    height_stage = object_model.get_part("height_stage")
    desk_support = object_model.get_part("desk_support")
    desk = object_model.get_part("desk")

    slide = object_model.get_articulation("lower_mast_to_height_stage")
    knob = object_model.get_articulation("lower_mast_to_collar_knob")
    desk_hinge = object_model.get_articulation("desk_support_to_desk")
    leg_hinge = object_model.get_articulation("lower_mast_to_leg_0")

    ctx.allow_overlap(
        lower_mast,
        height_stage,
        elem_a="outer_tube",
        elem_b="sliding_tube",
        reason="The inner height tube is intentionally retained inside the lower mast sleeve.",
    )
    ctx.allow_overlap(
        lower_mast,
        height_stage,
        elem_a="collar_ring",
        elem_b="sliding_tube",
        reason="The clamp collar surrounds the sliding height tube at the mast lock.",
    )
    ctx.allow_overlap(
        desk_support,
        desk,
        elem_a="tilt_pin",
        elem_b="hinge_strap_0",
        reason="The desk hinge strap is represented as captured around the horizontal tilt pin.",
    )
    ctx.allow_overlap(
        desk_support,
        desk,
        elem_a="tilt_pin",
        elem_b="hinge_strap_1",
        reason="The second desk hinge strap is represented as captured around the same tilt pin.",
    )

    ctx.expect_within(
        height_stage,
        lower_mast,
        axes="xy",
        inner_elem="sliding_tube",
        outer_elem="outer_tube",
        margin=0.004,
        name="sliding tube stays centered in lower mast",
    )
    ctx.expect_overlap(
        height_stage,
        lower_mast,
        axes="z",
        elem_a="sliding_tube",
        elem_b="outer_tube",
        min_overlap=0.16,
        name="collapsed height stage remains inserted",
    )
    ctx.expect_within(
        height_stage,
        lower_mast,
        axes="xy",
        inner_elem="sliding_tube",
        outer_elem="collar_ring",
        margin=0.003,
        name="collar encircles height stage",
    )
    ctx.expect_overlap(
        height_stage,
        lower_mast,
        axes="z",
        elem_a="sliding_tube",
        elem_b="collar_ring",
        min_overlap=0.045,
        name="collar covers the sliding tube",
    )

    rest_height = ctx.part_world_position(height_stage)
    with ctx.pose({slide: 0.32}):
        ctx.expect_within(
            height_stage,
            lower_mast,
            axes="xy",
            inner_elem="sliding_tube",
            outer_elem="outer_tube",
            margin=0.004,
            name="extended height stage remains centered",
        )
        ctx.expect_overlap(
            height_stage,
            lower_mast,
            axes="z",
            elem_a="sliding_tube",
            elem_b="outer_tube",
            min_overlap=0.035,
            name="extended height stage keeps retained insertion",
        )
        extended_height = ctx.part_world_position(height_stage)
    ctx.check(
        "height stage slides upward",
        rest_height is not None and extended_height is not None and extended_height[2] > rest_height[2] + 0.30,
        details=f"rest={rest_height}, extended={extended_height}",
    )

    ctx.check(
        "collar knob is continuous rotary clamp",
        knob.articulation_type == ArticulationType.CONTINUOUS and tuple(knob.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob.articulation_type}, axis={knob.axis}",
    )

    ctx.expect_overlap(
        desk,
        desk_support,
        axes="xyz",
        elem_a="hinge_strap_0",
        elem_b="tilt_pin",
        min_overlap=0.006,
        name="desk hinge strap captures the horizontal pin",
    )
    ctx.expect_overlap(
        desk,
        desk_support,
        axes="xyz",
        elem_a="hinge_strap_1",
        elem_b="tilt_pin",
        min_overlap=0.006,
        name="second desk hinge strap captures the horizontal pin",
    )
    ctx.expect_contact(
        desk_support,
        height_stage,
        elem_a="head_socket",
        elem_b="top_plug",
        contact_tol=0.002,
        name="separate desk support sits on height-stage top plug",
    )
    rest_desk_aabb = ctx.part_world_aabb(desk)
    with ctx.pose({desk_hinge: 0.55}):
        tilted_desk_aabb = ctx.part_world_aabb(desk)
    ctx.check(
        "desk tilt changes music desk angle",
        rest_desk_aabb is not None
        and tilted_desk_aabb is not None
        and tilted_desk_aabb[1][1] > rest_desk_aabb[1][1] + 0.06,
        details=f"rest={rest_desk_aabb}, tilted={tilted_desk_aabb}",
    )

    leg = object_model.get_part("leg_0")
    rest_leg_aabb = ctx.part_world_aabb(leg)
    with ctx.pose({leg_hinge: 1.05}):
        folded_leg_aabb = ctx.part_world_aabb(leg)
    ctx.check(
        "tripod leg folds upward from deployed stance",
        rest_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[0][2] > rest_leg_aabb[0][2] + 0.16,
        details=f"rest={rest_leg_aabb}, folded={folded_leg_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
