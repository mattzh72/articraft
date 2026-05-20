from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


TRIPOD_HINGE_Z = 0.335
PAN_AXIS_Z = 1.240
TILT_AXIS_Z_IN_YOKE = 0.192
CAN_FRONT_Y = 0.136
VERTICAL_LEAF_OPEN = 1.38
SIDE_LEAF_OPEN = 1.10


def make_can_shell():
    shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.082, -0.116),
            (0.088, -0.096),
            (0.096, -0.018),
            (0.099, 0.072),
            (0.104, 0.120),
            (0.110, 0.128),
        ],
        inner_profile=[
            (0.070, -0.102),
            (0.073, -0.034),
            (0.078, 0.082),
            (0.091, 0.128),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_x(-math.pi / 2.0)
    return shell


def make_leg_mesh():
    return tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.110, 0.0, -0.050),
            (0.330, 0.0, -0.190),
            (0.520, 0.0, -0.315),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_tripod")

    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.49, 0.52, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.051, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=charcoal,
        name="crown",
    )
    stand.visual(
        Cylinder(radius=0.022, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=charcoal,
        name="lower_spine",
    )
    stand.visual(
        Cylinder(radius=0.018, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.780)),
        material=black,
        name="mast",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.210)),
        material=charcoal,
        name="head_socket",
    )
    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            mesh_from_geometry(make_leg_mesh(), f"spotlight_leg_{index}"),
            material=black,
            name="tube",
        )
        leg.visual(
            Cylinder(radius=0.010, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="knuckle",
        )
        leg.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.530, 0.0, -0.323)),
            material=rubber,
            name="foot",
        )

        model.articulation(
            f"crown_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=stand,
            child=leg,
            origin=Origin(
                xyz=(0.061 * math.cos(angle), 0.061 * math.sin(angle), TRIPOD_HINGE_Z),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.4,
                lower=-0.10,
                upper=0.82,
            ),
        )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.330, 0.096, 0.278),
                span_width=0.225,
                trunnion_diameter=0.024,
                trunnion_center_z=TILT_AXIS_Z_IN_YOKE,
                base_thickness=0.020,
                corner_radius=0.006,
                center=False,
            ),
            "spotlight_yoke",
        ),
        material=charcoal,
        name="body",
    )
    yoke.visual(
        Cylinder(radius=0.030, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=steel,
        name="pan_collar",
    )

    model.articulation(
        "stand_pan",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.6,
            lower=-1.8,
            upper=1.8,
        ),
    )

    can = model.part("can")
    can.visual(
        mesh_from_geometry(make_can_shell(), "spotlight_can_shell"),
        material=black,
        name="shell",
    )
    can.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.100, -0.008, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    can.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.100, -0.008, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    can.visual(
        Cylinder(radius=0.013, length=0.060),
        origin=Origin(xyz=(0.120, -0.082, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="focus_housing",
    )
    can.visual(
        Box((0.174, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.124, 0.084)),
        material=charcoal,
        name="front_frame_top",
    )
    can.visual(
        Box((0.174, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.124, -0.084)),
        material=charcoal,
        name="front_frame_bottom",
    )
    can.visual(
        Box((0.014, 0.016, 0.174)),
        origin=Origin(xyz=(0.084, 0.132, 0.0)),
        material=charcoal,
        name="front_frame_right",
    )
    can.visual(
        Box((0.014, 0.016, 0.174)),
        origin=Origin(xyz=(-0.084, 0.132, 0.0)),
        material=charcoal,
        name="front_frame_left",
    )

    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, TILT_AXIS_Z_IN_YOKE)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.9,
            upper=0.7,
        ),
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.054,
                0.018,
                body_style="cylindrical",
                crown_radius=0.002,
                edge_radius=0.002,
                center=False,
            ),
            "focus_knob",
        ),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="cap",
    )

    model.articulation(
        "focus_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=can,
        child=knob,
        origin=Origin(xyz=(0.150, -0.082, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=10.0),
    )

    top_leaf = model.part("top_leaf")
    top_leaf.visual(
        Cylinder(radius=0.0045, length=0.162),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge",
    )
    top_leaf.visual(
        Box((0.150, 0.010, 0.018)),
        origin=Origin(
            xyz=(0.0, 0.003, -0.009),
            rpy=(VERTICAL_LEAF_OPEN, 0.0, 0.0),
        ),
        material=black,
        name="strap",
    )
    top_leaf.visual(
        Box((0.146, 0.004, 0.032)),
        origin=Origin(
            xyz=(0.0, 0.006, -0.020),
            rpy=(VERTICAL_LEAF_OPEN, 0.0, 0.0),
        ),
        material=black,
        name="panel",
    )

    bottom_leaf = model.part("bottom_leaf")
    bottom_leaf.visual(
        Cylinder(radius=0.0045, length=0.162),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge",
    )
    bottom_leaf.visual(
        Box((0.150, 0.010, 0.018)),
        origin=Origin(
            xyz=(0.0, 0.003, 0.009),
            rpy=(-VERTICAL_LEAF_OPEN, 0.0, 0.0),
        ),
        material=black,
        name="strap",
    )
    bottom_leaf.visual(
        Box((0.146, 0.004, 0.032)),
        origin=Origin(
            xyz=(0.0, 0.006, 0.020),
            rpy=(-VERTICAL_LEAF_OPEN, 0.0, 0.0),
        ),
        material=black,
        name="panel",
    )

    left_leaf = model.part("left_leaf")
    left_leaf.visual(
        Cylinder(radius=0.0045, length=0.162),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="hinge",
    )
    left_leaf.visual(
        Box((0.018, 0.010, 0.150)),
        origin=Origin(
            xyz=(0.009, 0.006, 0.0),
            rpy=(0.0, 0.0, SIDE_LEAF_OPEN),
        ),
        material=black,
        name="strap",
    )
    left_leaf.visual(
        Box((0.032, 0.004, 0.146)),
        origin=Origin(
            xyz=(0.022, 0.009, 0.0),
            rpy=(0.0, 0.0, SIDE_LEAF_OPEN),
        ),
        material=black,
        name="panel",
    )

    right_leaf = model.part("right_leaf")
    right_leaf.visual(
        Cylinder(radius=0.0045, length=0.162),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="hinge",
    )
    right_leaf.visual(
        Box((0.018, 0.010, 0.150)),
        origin=Origin(
            xyz=(-0.009, 0.006, 0.0),
            rpy=(0.0, 0.0, -SIDE_LEAF_OPEN),
        ),
        material=black,
        name="strap",
    )
    right_leaf.visual(
        Box((0.032, 0.004, 0.146)),
        origin=Origin(
            xyz=(-0.022, 0.009, 0.0),
            rpy=(0.0, 0.0, -SIDE_LEAF_OPEN),
        ),
        material=black,
        name="panel",
    )

    model.articulation(
        "top_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=top_leaf,
        origin=Origin(xyz=(0.0, CAN_FRONT_Y, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.45,
            upper=0.55,
        ),
    )
    model.articulation(
        "bottom_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=bottom_leaf,
        origin=Origin(xyz=(0.0, CAN_FRONT_Y, -0.090)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.45,
            upper=0.55,
        ),
    )
    model.articulation(
        "left_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=left_leaf,
        origin=Origin(xyz=(-0.090, CAN_FRONT_Y + 0.008, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.45,
            upper=0.55,
        ),
    )
    model.articulation(
        "right_leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=can,
        child=right_leaf,
        origin=Origin(xyz=(0.090, CAN_FRONT_Y + 0.008, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.45,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    can = object_model.get_part("can")
    knob = object_model.get_part("knob")
    leg_0 = object_model.get_part("leg_0")
    top_leaf = object_model.get_part("top_leaf")
    bottom_leaf = object_model.get_part("bottom_leaf")
    left_leaf = object_model.get_part("left_leaf")
    right_leaf = object_model.get_part("right_leaf")

    leg_joint = object_model.get_articulation("crown_leg_0")
    pan_joint = object_model.get_articulation("stand_pan")
    tilt_joint = object_model.get_articulation("yoke_tilt")
    knob_joint = object_model.get_articulation("focus_knob_spin")
    top_joint = object_model.get_articulation("top_leaf_hinge")

    ctx.allow_overlap(
        top_leaf,
        left_leaf,
        elem_a="panel",
        elem_b="panel",
        reason="Adjacent barndoor leaves are shown in a ready pose; the simplified flat-panel proxies omit the small real hinge-plane stagger at the front corners.",
    )
    ctx.allow_overlap(
        top_leaf,
        right_leaf,
        elem_a="panel",
        elem_b="panel",
        reason="Adjacent barndoor leaves are shown in a ready pose; the simplified flat-panel proxies omit the small real hinge-plane stagger at the front corners.",
    )
    ctx.allow_overlap(
        bottom_leaf,
        left_leaf,
        elem_a="panel",
        elem_b="panel",
        reason="Adjacent barndoor leaves are shown in a ready pose; the simplified flat-panel proxies omit the small real hinge-plane stagger at the front corners.",
    )
    ctx.allow_overlap(
        bottom_leaf,
        right_leaf,
        elem_a="panel",
        elem_b="panel",
        reason="Adjacent barndoor leaves are shown in a ready pose; the simplified flat-panel proxies omit the small real hinge-plane stagger at the front corners.",
    )

    def elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    ctx.expect_origin_gap(
        can,
        stand,
        axis="z",
        min_gap=0.20,
        name="lamp head sits well above the tripod crown",
    )
    ctx.expect_origin_distance(
        knob,
        can,
        axes="x",
        min_dist=0.12,
        max_dist=0.20,
        name="focus knob projects from the can side",
    )
    ctx.check(
        "focus knob uses a continuous shaft joint",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None,
        details=(
            f"type={knob_joint.articulation_type}, "
            f"limits={knob_joint.motion_limits}"
        ),
    )

    rest_foot = elem_center(leg_0, "foot")
    with ctx.pose({leg_joint: 0.72}):
        folded_foot = elem_center(leg_0, "foot")
    ctx.check(
        "tripod leg folds inward toward the mast",
        rest_foot is not None
        and folded_foot is not None
        and math.hypot(rest_foot[0], rest_foot[1]) > math.hypot(folded_foot[0], folded_foot[1]) + 0.25,
        details=f"rest={rest_foot}, folded={folded_foot}",
    )

    rest_front = elem_center(can, "front_frame_top")
    with ctx.pose({pan_joint: 1.0}):
        panned_front = elem_center(can, "front_frame_top")
    ctx.check(
        "yoke pan turns the beam direction around the mast",
        rest_front is not None
        and panned_front is not None
        and math.hypot(rest_front[0] - panned_front[0], rest_front[1] - panned_front[1]) > 0.10,
        details=f"rest={rest_front}, panned={panned_front}",
    )

    rest_top = elem_center(can, "front_frame_top")
    with ctx.pose({tilt_joint: 0.45}):
        tilted_top = elem_center(can, "front_frame_top")
    ctx.check(
        "lamp can tilts upward in the yoke",
        rest_top is not None
        and tilted_top is not None
        and tilted_top[2] > rest_top[2] + 0.03,
        details=f"rest={rest_top}, tilted={tilted_top}",
    )

    rest_leaf = elem_center(top_leaf, "panel")
    rest_right_leaf = elem_center(right_leaf, "panel")
    with ctx.pose({top_joint: -0.90}):
        closed_leaf = elem_center(top_leaf, "panel")
        unchanged_right_leaf = elem_center(right_leaf, "panel")
    ctx.check(
        "top barndoor leaf closes independently of the side leaves",
        rest_leaf is not None
        and closed_leaf is not None
        and rest_right_leaf is not None
        and unchanged_right_leaf is not None
        and closed_leaf[1] < rest_leaf[1] - 0.015
        and abs(unchanged_right_leaf[1] - rest_right_leaf[1]) < 0.001,
        details=(
            f"top_rest={rest_leaf}, top_closed={closed_leaf}, "
            f"right_rest={rest_right_leaf}, right_same={unchanged_right_leaf}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
