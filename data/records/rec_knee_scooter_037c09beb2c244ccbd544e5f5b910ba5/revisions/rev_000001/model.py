from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
    ExtrudeGeometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    black = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_metal = model.material("dark_powder_coat", rgba=(0.08, 0.09, 0.095, 1.0))
    silver = model.material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    pad_mat = model.material("soft_black_pad", rgba=(0.01, 0.012, 0.012, 1.0))
    tire_mat = model.material("dark_polyurethane", rgba=(0.015, 0.014, 0.013, 1.0))
    hub_mat = model.material("light_hub_plastic", rgba=(0.86, 0.86, 0.80, 1.0))
    basket_mat = model.material("black_wire_basket", rgba=(0.01, 0.012, 0.012, 1.0))

    def tube(part, pts, radius, name, material=dark_metal, segments=16):
        geom = wire_from_points(
            pts,
            radius=radius,
            radial_segments=segments,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=radius * 1.25,
        )
        part.visual(mesh_from_geometry(geom, name), material=material, name=name)

    # Fixed frame: rear axle, lower frame, knee-pad supports, bearing shell,
    # and the stationary half of the parking-stand hinge.
    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.026, length=1.10),
        origin=Origin(xyz=(-0.02, 0.0, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="lower_tube",
    )
    frame.visual(
        Cylinder(radius=0.018, length=0.56),
        origin=Origin(xyz=(-0.50, 0.0, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_axle",
    )
    tube(frame, [(-0.50, 0.0, 0.095), (-0.42, 0.0, 0.22)], 0.015, "rear_axle_strut", silver)
    tube(frame, [(-0.35, -0.10, 0.25), (-0.12, -0.10, 0.50)], 0.018, "pad_side_strut_0")
    tube(frame, [(-0.35, 0.10, 0.25), (-0.12, 0.10, 0.50)], 0.018, "pad_side_strut_1")
    frame.visual(
        Cylinder(radius=0.023, length=0.31),
        origin=Origin(xyz=(-0.12, 0.0, 0.395)),
        material=dark_metal,
        name="pad_post",
    )
    frame.visual(
        Box((0.34, 0.18, 0.022)),
        origin=Origin(xyz=(-0.12, 0.0, 0.515)),
        material=silver,
        name="pad_plate",
    )
    pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.42, 0.23, 0.045, corner_segments=10), 0.060, center=True),
        "knee_pad_rounded",
    )
    frame.visual(
        pad_mesh,
        origin=Origin(xyz=(-0.12, 0.0, 0.570)),
        material=pad_mat,
        name="knee_pad",
    )

    tube(frame, [(0.32, 0.0, 0.25), (0.462, 0.0, 0.342)], 0.020, "head_tube_down_brace")
    frame.visual(
        Cylinder(radius=0.042, length=0.075),
        origin=Origin(xyz=(0.50, 0.0, 0.385)),
        material=dark_metal,
        name="lower_bearing_collar",
    )
    frame.visual(
        Cylinder(radius=0.042, length=0.075),
        origin=Origin(xyz=(0.50, 0.0, 0.565)),
        material=dark_metal,
        name="upper_bearing_collar",
    )
    frame.visual(
        Box((0.018, 0.020, 0.190)),
        origin=Origin(xyz=(0.50, 0.050, 0.475)),
        material=dark_metal,
        name="head_tube_web",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.225),
        origin=Origin(xyz=(0.045, 0.0, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="stand_pin",
    )
    frame.visual(
        Box((0.050, 0.028, 0.070)),
        origin=Origin(xyz=(0.045, -0.118, 0.180)),
        material=dark_metal,
        name="stand_lug_0",
    )
    frame.visual(
        Box((0.050, 0.028, 0.070)),
        origin=Origin(xyz=(0.045, 0.118, 0.180)),
        material=dark_metal,
        name="stand_lug_1",
    )
    frame.visual(
        Box((0.035, 0.100, 0.030)),
        origin=Origin(xyz=(0.045, -0.073, 0.230)),
        material=dark_metal,
        name="stand_bridge_0",
    )
    frame.visual(
        Box((0.035, 0.100, 0.030)),
        origin=Origin(xyz=(0.045, 0.073, 0.230)),
        material=dark_metal,
        name="stand_bridge_1",
    )

    # Steering/fork assembly.  The front axle belongs to this yawing part, so
    # the two front wheels steer together.
    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.026, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=silver,
        name="steerer_stem",
    )
    front_fork.visual(
        Cylinder(radius=0.024, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        material=dark_metal,
        name="handle_post",
    )
    front_fork.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.73), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar",
    )
    front_fork.visual(
        Cylinder(radius=0.022, length=0.12),
        origin=Origin(xyz=(0.0, -0.31, 0.73), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip_0",
    )
    front_fork.visual(
        Cylinder(radius=0.022, length=0.12),
        origin=Origin(xyz=(0.0, 0.31, 0.73), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip_1",
    )
    front_fork.visual(
        Box((0.075, 0.200, 0.045)),
        origin=Origin(xyz=(0.025, 0.0, -0.124)),
        material=dark_metal,
        name="fork_crown",
    )
    tube(front_fork, [(0.0, -0.070, -0.124), (0.045, -0.120, -0.220), (0.085, -0.145, -0.325)], 0.017, "fork_leg_0")
    tube(front_fork, [(0.0, 0.070, -0.124), (0.045, 0.120, -0.220), (0.085, 0.145, -0.325)], 0.017, "fork_leg_1")
    front_fork.visual(
        Box((0.050, 0.055, 0.055)),
        origin=Origin(xyz=(0.085, -0.145, -0.325)),
        material=dark_metal,
        name="dropout_0",
    )
    front_fork.visual(
        Box((0.050, 0.055, 0.055)),
        origin=Origin(xyz=(0.085, 0.145, -0.325)),
        material=dark_metal,
        name="dropout_1",
    )
    front_fork.visual(
        Cylinder(radius=0.014, length=0.45),
        origin=Origin(xyz=(0.085, 0.0, -0.325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="front_axle",
    )

    # Front basket is rigidly attached to the steering/fork assembly with two
    # welded support arms.  Thin panels/slats leave the top open like a basket.
    basket_center = (0.245, 0.0, 0.295)
    front_fork.visual(
        Box((0.30, 0.40, 0.018)),
        origin=Origin(xyz=(basket_center[0], basket_center[1], basket_center[2] - 0.110)),
        material=basket_mat,
        name="basket_floor",
    )
    front_fork.visual(
        Box((0.018, 0.40, 0.220)),
        origin=Origin(xyz=(basket_center[0] + 0.150, 0.0, basket_center[2])),
        material=basket_mat,
        name="basket_front_wall",
    )
    front_fork.visual(
        Box((0.018, 0.40, 0.165)),
        origin=Origin(xyz=(basket_center[0] - 0.150, 0.0, basket_center[2] - 0.028)),
        material=basket_mat,
        name="basket_back_wall",
    )
    front_fork.visual(
        Box((0.30, 0.018, 0.220)),
        origin=Origin(xyz=(basket_center[0], -0.200, basket_center[2])),
        material=basket_mat,
        name="basket_side_0",
    )
    front_fork.visual(
        Box((0.30, 0.018, 0.220)),
        origin=Origin(xyz=(basket_center[0], 0.200, basket_center[2])),
        material=basket_mat,
        name="basket_side_1",
    )
    front_fork.visual(
        Box((0.030, 0.430, 0.030)),
        origin=Origin(xyz=(basket_center[0] + 0.010, 0.0, basket_center[2] + 0.115)),
        material=basket_mat,
        name="basket_top_rim",
    )
    for i, y in enumerate((-0.145, 0.145)):
        front_fork.visual(
            Box((0.180, 0.018, 0.018)),
            origin=Origin(xyz=(0.075, y, 0.205)),
            material=dark_metal,
            name=f"basket_arm_{i}",
        )
    front_fork.visual(
        Box((0.044, 0.330, 0.030)),
        origin=Origin(xyz=(0.020, 0.0, 0.205)),
        material=dark_metal,
        name="basket_mount_clamp",
    )
    for i, y in enumerate((-0.12, 0.0, 0.12)):
        front_fork.visual(
            Box((0.010, 0.012, 0.190)),
            origin=Origin(xyz=(basket_center[0] + 0.160, y, basket_center[2] - 0.010)),
            material=silver,
            name=f"basket_front_slat_{i}",
        )
    for i, z in enumerate((basket_center[2] - 0.045, basket_center[2] + 0.045)):
        front_fork.visual(
            Box((0.014, 0.405, 0.010)),
            origin=Origin(xyz=(basket_center[0] + 0.160, 0.0, z)),
            material=silver,
            name=f"basket_front_rail_{i}",
        )

    model.articulation(
        "frame_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.50, 0.0, 0.445)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-0.70, upper=0.70),
    )

    # Shared detailed tire visual plus simple axle-centered hub cylinders.
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.085,
            0.052,
            inner_radius=0.060,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.04),
            tread=TireTread(style="ribbed", depth=0.0035, count=16, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "small_polyurethane_tire",
    )
    wheel_specs = (
        ("front_wheel_0", front_fork, (0.085, -0.225, -0.325)),
        ("front_wheel_1", front_fork, (0.085, 0.225, -0.325)),
        ("rear_wheel_0", frame, (-0.50, -0.245, 0.095)),
        ("rear_wheel_1", frame, (-0.50, 0.245, 0.095)),
    )
    for wheel_name, parent, xyz in wheel_specs:
        wheel = model.part(wheel_name)
        wheel.visual(tire_mesh, origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)), material=tire_mat, name="tire")
        wheel.visual(
            Cylinder(radius=0.057, length=0.046),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hub_mat,
            name="hub",
        )
        model.articulation(
            f"{parent.name}_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=30.0),
        )

    # Short parking stand: the hinge sleeve is concentric with the frame's pin,
    # and the U-shaped foot folds down around the lateral pivot.
    stand = model.part("parking_stand")
    stand.visual(
        Cylinder(radius=0.027, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="stand_sleeve",
    )
    stand.visual(
        Box((0.235, 0.018, 0.020)),
        origin=Origin(xyz=(-0.118, -0.072, -0.030)),
        material=dark_metal,
        name="stand_leg_0",
    )
    stand.visual(
        Box((0.235, 0.018, 0.020)),
        origin=Origin(xyz=(-0.118, 0.072, -0.030)),
        material=dark_metal,
        name="stand_leg_1",
    )
    stand.visual(
        Box((0.034, 0.210, 0.022)),
        origin=Origin(xyz=(-0.238, 0.0, -0.032)),
        material=black,
        name="stand_foot",
    )
    model.articulation(
        "frame_to_parking_stand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=stand,
        origin=Origin(xyz=(0.045, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=0.76),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fork = object_model.get_part("front_fork")
    stand = object_model.get_part("parking_stand")
    steer_joint = object_model.get_articulation("frame_to_front_fork")
    stand_joint = object_model.get_articulation("frame_to_parking_stand")

    # Captured rotating hardware is intentionally represented with solid pins
    # nested inside simplified hub/bearing sleeves.
    for collar in ("lower_bearing_collar", "upper_bearing_collar"):
        ctx.allow_overlap(
            frame,
            fork,
            elem_a=collar,
            elem_b="steerer_stem",
            reason="The steerable fork's stem is captured inside the fixed head-tube bearing collar.",
        )
        ctx.expect_within(
            fork,
            frame,
            axes="xy",
            inner_elem="steerer_stem",
            outer_elem=collar,
            margin=0.002,
            name=f"steerer centered in {collar}",
        )
        ctx.expect_overlap(
            fork,
            frame,
            axes="z",
            elem_a="steerer_stem",
            elem_b=collar,
            min_overlap=0.040,
            name=f"steerer retained by {collar}",
        )

    ctx.allow_overlap(
        frame,
        stand,
        elem_a="stand_pin",
        elem_b="stand_sleeve",
        reason="The parking stand sleeve rotates around the frame-mounted hinge pin.",
    )
    ctx.expect_within(
        frame,
        stand,
        axes="xz",
        inner_elem="stand_pin",
        outer_elem="stand_sleeve",
        margin=0.002,
        name="stand pin stays inside hinge sleeve",
    )
    ctx.expect_overlap(
        frame,
        stand,
        axes="y",
        elem_a="stand_pin",
        elem_b="stand_sleeve",
        min_overlap=0.140,
        name="stand sleeve spans the hinge pin",
    )

    for wheel_name, axle_part, axle_elem in (
        ("front_wheel_0", fork, "front_axle"),
        ("front_wheel_1", fork, "front_axle"),
        ("rear_wheel_0", frame, "rear_axle"),
        ("rear_wheel_1", frame, "rear_axle"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            axle_part,
            wheel,
            elem_a=axle_elem,
            elem_b="hub",
            reason="A small wheel hub is captured around its axle so the wheel stays mounted while spinning.",
        )
        ctx.expect_within(
            axle_part,
            wheel,
            axes="xz",
            inner_elem=axle_elem,
            outer_elem="hub",
            margin=0.004,
            name=f"{wheel_name} axle runs through hub center",
        )
        ctx.expect_overlap(
            axle_part,
            wheel,
            axes="y",
            elem_a=axle_elem,
            elem_b="hub",
            min_overlap=0.020,
            name=f"{wheel_name} hub retained on axle",
        )

    wheel_joints = [
        object_model.get_articulation("front_fork_to_front_wheel_0"),
        object_model.get_articulation("front_fork_to_front_wheel_1"),
        object_model.get_articulation("frame_to_rear_wheel_0"),
        object_model.get_articulation("frame_to_rear_wheel_1"),
    ]
    ctx.check(
        "four wheel spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details="All four small wheels should be continuous spin joints on their axles.",
    )

    front_wheel = object_model.get_part("front_wheel_0")
    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steer_joint: 0.55}):
        yawed_front = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork yaws front axle",
        rest_front is not None
        and yawed_front is not None
        and abs(yawed_front[1] - rest_front[1]) > 0.030,
        details=f"rest={rest_front}, yawed={yawed_front}",
    )

    stowed_foot = ctx.part_element_world_aabb(stand, elem="stand_foot")
    with ctx.pose({stand_joint: 0.76}):
        parked_foot = ctx.part_element_world_aabb(stand, elem="stand_foot")
        ctx.expect_within(
            frame,
            stand,
            axes="xz",
            inner_elem="stand_pin",
            outer_elem="stand_sleeve",
            margin=0.002,
            name="folded stand remains clipped to hinge",
        )
    ctx.check(
        "parking stand folds down",
        stowed_foot is not None
        and parked_foot is not None
        and parked_foot[0][2] < stowed_foot[0][2] - 0.12,
        details=f"stowed={stowed_foot}, parked={parked_foot}",
    )

    return ctx.report()


object_model = build_object_model()
