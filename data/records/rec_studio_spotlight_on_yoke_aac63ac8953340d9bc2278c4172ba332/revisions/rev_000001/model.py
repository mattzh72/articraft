from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TrunnionYokeGeometry,
    WheelGeometry,
    mesh_from_geometry,
)


def _aligned_cylinder(radius: float, length: float, *, along: str, xyz: tuple[float, float, float]):
    if along == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    elif along == "y":
        rpy = (-math.pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=rpy)


def _spotlight_housing_mesh():
    outer_profile = [
        (0.114, -0.170),
        (0.111, -0.128),
        (0.108, -0.020),
        (0.109, 0.118),
        (0.115, 0.166),
    ]
    inner_profile = [
        (0.000, -0.170),
        (0.062, -0.154),
        (0.094, -0.122),
        (0.100, -0.020),
        (0.102, 0.120),
        (0.103, 0.156),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=80,
        ),
        "spotlight_housing",
    )


def _tire_mesh():
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.039,
            0.022,
            inner_radius=0.028,
        ),
        "studio_tire",
    )
    return tire_mesh


def _add_leaf(
    model: ArticulatedObject,
    *,
    name: str,
    parent,
    hinge_name: str,
    hinge_xyz: tuple[float, float, float],
    axis: tuple[float, float, float],
    panel_size: tuple[float, float, float],
    panel_xyz: tuple[float, float, float],
    barrel_length: float,
    barrel_xyz: tuple[float, float, float],
    barrel_axis: str,
    material,
    metal,
) -> None:
    leaf = model.part(name)
    leaf.visual(
        Box(panel_size),
        origin=Origin(xyz=panel_xyz),
        material=material,
        name="panel",
    )
    barrel_geom, barrel_origin = _aligned_cylinder(
        0.0055,
        barrel_length,
        along=barrel_axis,
        xyz=barrel_xyz,
    )
    leaf.visual(
        barrel_geom,
        origin=barrel_origin,
        material=metal,
        name="hinge_barrel",
    )
    model.articulation(
        hinge_name,
        ArticulationType.REVOLUTE,
        parent=parent,
        child=leaf,
        origin=Origin(xyz=hinge_xyz),
        axis=axis,
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight")

    stand_finish = model.material("stand_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    body_finish = model.material("body_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    door_finish = model.material("door_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    metal = model.material("metal", rgba=(0.55, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.63, 0.76, 0.84, 0.42))
    control_finish = model.material("control_finish", rgba=(0.18, 0.18, 0.19, 1.0))

    base = model.part("base")
    rail_size = (0.032, 0.420, 0.028)
    brace_size = (0.350, 0.032, 0.028)
    deck_size = (0.320, 0.110, 0.018)
    rail_z = 0.096
    fork_bridge_size = (0.040, 0.036, 0.010)
    fork_leg_size = (0.006, 0.036, 0.056)
    wheel_centers = [
        (-0.160, -0.205, 0.040),
        (0.160, -0.205, 0.040),
        (-0.160, 0.205, 0.040),
        (0.160, 0.205, 0.040),
    ]

    for index, rail_x in enumerate((-0.160, 0.160)):
        base.visual(
            Box(rail_size),
            origin=Origin(xyz=(rail_x, 0.0, rail_z)),
            material=stand_finish,
            name=f"rail_{index}",
        )
    for index, brace_y in enumerate((-0.110, 0.110)):
        base.visual(
            Box(brace_size),
            origin=Origin(xyz=(0.0, brace_y, rail_z)),
            material=stand_finish,
            name=f"brace_{index}",
        )
    base.visual(
        Box(deck_size),
        origin=Origin(xyz=(0.0, 0.0, 0.116)),
        material=stand_finish,
        name="deck",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.148),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=stand_finish,
        name="post",
    )

    for index, (wheel_x, wheel_y, _) in enumerate(wheel_centers):
        base.visual(
            Box(fork_bridge_size),
            origin=Origin(xyz=(wheel_x, wheel_y, 0.086)),
            material=stand_finish,
            name=f"fork_bridge_{index}",
        )
        for side_index, offset_x in enumerate((-0.014, 0.014)):
            base.visual(
                Box(fork_leg_size),
                origin=Origin(xyz=(wheel_x + offset_x, wheel_y, 0.054)),
                material=stand_finish,
                name=f"fork_leg_{index}_{side_index}",
            )

    tire_mesh = _tire_mesh()
    for index, (wheel_x, wheel_y, wheel_z) in enumerate(wheel_centers):
        wheel = model.part(f"wheel_{index}")
        core_geom, core_origin = _aligned_cylinder(
            0.029,
            0.018,
            along="x",
            xyz=(0.0, 0.0, 0.0),
        )
        wheel.visual(
            core_geom,
            origin=core_origin,
            material=metal,
            name="wheel",
        )
        wheel.visual(
            tire_mesh,
            material=rubber,
            name="tire",
        )
        model.articulation(
            f"base_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=wheel,
            origin=Origin(xyz=(wheel_x, wheel_y, wheel_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=18.0,
            ),
        )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.300, 0.075, 0.220),
                span_width=0.228,
                trunnion_diameter=0.018,
                trunnion_center_z=0.148,
                base_thickness=0.018,
                center=False,
            ),
            "spotlight_yoke",
        ),
        material=stand_finish,
        name="yoke_body",
    )
    yoke.visual(
        Cylinder(radius=0.048, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=metal,
        name="pan_hub",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=-1.35,
            upper=1.35,
        ),
    )

    can = model.part("can")
    can.visual(
        _spotlight_housing_mesh(),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_finish,
        name="housing",
    )
    for index, boss_x in enumerate((-0.114, 0.114)):
        trunnion_geom, trunnion_origin = _aligned_cylinder(
            0.009,
            0.018,
            along="x",
            xyz=(boss_x, 0.0, 0.0),
        )
        can.visual(
            trunnion_geom,
            origin=trunnion_origin,
            material=metal,
            name=f"trunnion_{index}",
        )
    rear_boss_geom, rear_boss_origin = _aligned_cylinder(
        0.028,
        0.020,
        along="y",
        xyz=(0.054, -0.164, -0.028),
    )
    can.visual(
        rear_boss_geom,
        origin=rear_boss_origin,
        material=body_finish,
        name="rear_boss",
    )
    handle_geom, handle_origin = _aligned_cylinder(
        0.010,
        0.070,
        along="x",
        xyz=(0.0, -0.040, 0.118),
    )
    can.visual(
        handle_geom,
        origin=handle_origin,
        material=metal,
        name="top_handle",
    )

    model.articulation(
        "yoke_to_can",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-0.85,
            upper=1.05,
        ),
    )

    lens_geom, lens_origin = _aligned_cylinder(
        0.101,
        0.008,
        along="y",
        xyz=(0.0, 0.0, 0.0),
    )
    can.visual(
        lens_geom,
        origin=Origin(
            xyz=(0.0, 0.142, 0.0),
            rpy=lens_origin.rpy,
        ),
        material=lens_glass,
        name="lens_glass",
    )
    can.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.198, 0.198),
                (0.210, 0.210),
                0.006,
                opening_shape="circle",
                outer_shape="circle",
                center=False,
            ),
            "lens_ring",
        ),
        origin=Origin(xyz=(0.0, 0.136, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="lens_ring",
    )

    frame = model.part("barndoor_frame")
    frame.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.208, 0.208),
                (0.290, 0.290),
                0.018,
                opening_shape="circle",
                outer_shape="rounded_rect",
                outer_corner_radius=0.018,
                center=False,
            ),
            "barndoor_frame",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=door_finish,
        name="frame",
    )
    model.articulation(
        "can_to_barndoor_frame",
        ArticulationType.FIXED,
        parent=can,
        child=frame,
        origin=Origin(xyz=(0.0, 0.164, 0.0)),
    )
    _add_leaf(
        model,
        name="top_leaf",
        parent=frame,
        hinge_name="frame_to_top_leaf",
        hinge_xyz=(0.0, 0.0235, 0.145),
        axis=(1.0, 0.0, 0.0),
        panel_size=(0.252, 0.003, 0.060),
        panel_xyz=(0.0, 0.0035, -0.030),
        barrel_length=0.252,
        barrel_xyz=(0.0, 0.0, 0.0),
        barrel_axis="x",
        material=door_finish,
        metal=metal,
    )
    _add_leaf(
        model,
        name="bottom_leaf",
        parent=frame,
        hinge_name="frame_to_bottom_leaf",
        hinge_xyz=(0.0, 0.0235, -0.145),
        axis=(-1.0, 0.0, 0.0),
        panel_size=(0.252, 0.003, 0.060),
        panel_xyz=(0.0, 0.0035, 0.030),
        barrel_length=0.252,
        barrel_xyz=(0.0, 0.0, 0.0),
        barrel_axis="x",
        material=door_finish,
        metal=metal,
    )
    _add_leaf(
        model,
        name="side_leaf_0",
        parent=frame,
        hinge_name="frame_to_side_leaf_0",
        hinge_xyz=(-0.145, 0.0235, 0.0),
        axis=(0.0, 0.0, 1.0),
        panel_size=(0.060, 0.003, 0.242),
        panel_xyz=(0.030, -0.0035, 0.0),
        barrel_length=0.242,
        barrel_xyz=(0.0, 0.0, 0.0),
        barrel_axis="z",
        material=door_finish,
        metal=metal,
    )
    _add_leaf(
        model,
        name="side_leaf_1",
        parent=frame,
        hinge_name="frame_to_side_leaf_1",
        hinge_xyz=(0.145, 0.0235, 0.0),
        axis=(0.0, 0.0, -1.0),
        panel_size=(0.060, 0.003, 0.242),
        panel_xyz=(-0.030, -0.0035, 0.0),
        barrel_length=0.242,
        barrel_xyz=(0.0, 0.0, 0.0),
        barrel_axis="z",
        material=door_finish,
        metal=metal,
    )

    rear_knob = model.part("rear_knob")
    shaft_geom, shaft_origin = _aligned_cylinder(
        0.006,
        0.018,
        along="y",
        xyz=(0.0, -0.009, 0.0),
    )
    rear_knob.visual(
        shaft_geom,
        origin=shaft_origin,
        material=metal,
        name="shaft",
    )
    rear_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.028,
                body_style="skirted",
                top_diameter=0.040,
                base_diameter=0.054,
                crown_radius=0.003,
                edge_radius=0.0015,
                center=False,
            ),
            "rear_knob",
        ),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="knob",
    )
    model.articulation(
        "can_to_rear_knob",
        ArticulationType.CONTINUOUS,
        parent=can,
        child=rear_knob,
        origin=Origin(xyz=(0.054, -0.174, -0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
        ),
    )

    return model


def _joint_type_name(joint) -> str:
    articulation_type = getattr(joint, "articulation_type", None)
    if articulation_type is None:
        return ""
    return getattr(articulation_type, "name", str(articulation_type))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap(
        "can",
        "yoke",
        elem_a="trunnion_0",
        elem_b="yoke_body",
        reason="The can's trunnion stub is intentionally seated inside the yoke cheek bore proxy.",
    )
    ctx.allow_overlap(
        "can",
        "yoke",
        elem_a="trunnion_1",
        elem_b="yoke_body",
        reason="The can's trunnion stub is intentionally seated inside the yoke cheek bore proxy.",
    )

    frame = object_model.get_part("barndoor_frame")
    top_leaf = object_model.get_part("top_leaf")
    side_leaf_0 = object_model.get_part("side_leaf_0")
    pan = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_can")
    top_hinge = object_model.get_articulation("frame_to_top_leaf")
    side_hinge = object_model.get_articulation("frame_to_side_leaf_0")
    knob_joint = object_model.get_articulation("can_to_rear_knob")

    wheel_joint_names = [f"base_to_wheel_{index}" for index in range(4)]
    wheel_joints = [object_model.get_articulation(name) for name in wheel_joint_names]

    rest_frame_pos = ctx.part_world_position(frame)
    with ctx.pose({pan: 0.85}):
        panned_frame_pos = ctx.part_world_position(frame)
    ctx.check(
        "pan swings the head around the post",
        rest_frame_pos is not None
        and panned_frame_pos is not None
        and abs(panned_frame_pos[0] - rest_frame_pos[0]) > 0.08,
        details=f"rest={rest_frame_pos}, panned={panned_frame_pos}",
    )

    with ctx.pose({tilt: 0.70}):
        tilted_frame_pos = ctx.part_world_position(frame)
    ctx.check(
        "positive tilt raises the light front",
        rest_frame_pos is not None
        and tilted_frame_pos is not None
        and tilted_frame_pos[2] > rest_frame_pos[2] + 0.08,
        details=f"rest={rest_frame_pos}, tilted={tilted_frame_pos}",
    )

    rest_top_aabb = ctx.part_world_aabb(top_leaf)
    with ctx.pose({top_hinge: 1.0}):
        open_top_aabb = ctx.part_world_aabb(top_leaf)
    ctx.check(
        "top leaf opens forward",
        rest_top_aabb is not None
        and open_top_aabb is not None
        and open_top_aabb[1][1] > rest_top_aabb[1][1] + 0.035,
        details=f"rest={rest_top_aabb}, open={open_top_aabb}",
    )

    rest_side_aabb = ctx.part_world_aabb(side_leaf_0)
    with ctx.pose({side_hinge: 1.0}):
        open_side_aabb = ctx.part_world_aabb(side_leaf_0)
    ctx.check(
        "side leaf opens forward",
        rest_side_aabb is not None
        and open_side_aabb is not None
        and open_side_aabb[1][1] > rest_side_aabb[1][1] + 0.035,
        details=f"rest={rest_side_aabb}, open={open_side_aabb}",
    )

    ctx.check(
        "rear knob uses continuous rotation",
        _joint_type_name(knob_joint).endswith("CONTINUOUS"),
        details=f"type={_joint_type_name(knob_joint)}",
    )
    ctx.check(
        "all base wheels use continuous spin joints",
        all(_joint_type_name(joint).endswith("CONTINUOUS") for joint in wheel_joints),
        details=str([_joint_type_name(joint) for joint in wheel_joints]),
    )

    return ctx.report()


object_model = build_object_model()
