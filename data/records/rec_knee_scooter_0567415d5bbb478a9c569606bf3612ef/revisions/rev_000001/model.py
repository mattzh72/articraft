from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _rounded_knee_pad() -> object:
    """Broad, softly rounded rectangular cushion authored in meters."""
    return (
        cq.Workplane("XY")
        .box(0.56, 0.34, 0.08)
        .edges("|Z")
        .fillet(0.045)
        .edges(">Z")
        .fillet(0.018)
    )


def _add_box(part, size, xyz, material, name):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="all_terrain_knee_scooter")

    metal = _mat(model, "powder_coated_dark_metal", (0.08, 0.10, 0.12, 1.0))
    accent = _mat(model, "blue_fork_accent", (0.05, 0.18, 0.42, 1.0))
    rubber = _mat(model, "matte_black_rubber", (0.01, 0.01, 0.01, 1.0))
    rim_mat = _mat(model, "brushed_aluminum_rim", (0.72, 0.74, 0.72, 1.0))
    pad_mat = _mat(model, "textured_black_vinyl_pad", (0.025, 0.022, 0.020, 1.0))
    grip_mat = _mat(model, "soft_grip_rubber", (0.015, 0.015, 0.014, 1.0))

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.118,
            0.066,
            rim=WheelRim(
                inner_radius=0.078,
                flange_height=0.008,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.036,
                width=0.050,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=0.048,
                    hole_diameter=0.006,
                ),
            ),
            face=WheelFace(dish_depth=0.007, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.015),
            bore=WheelBore(style="round", diameter=0.046),
        ),
        "all_terrain_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.165,
            0.076,
            inner_radius=0.116,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.10),
            tread=TireTread(style="block", depth=0.010, count=22, land_ratio=0.54),
            grooves=(
                TireGroove(center_offset=-0.018, width=0.006, depth=0.003),
                TireGroove(center_offset=0.018, width=0.006, depth=0.003),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.07),
            shoulder=TireShoulder(width=0.010, radius=0.005),
        ),
        "all_terrain_pneumatic_tire",
    )

    main_frame = model.part("main_frame")
    # Rectangular chassis: two side rails, crossbars, a center spine, and fixed rear axle supports.
    _add_box(main_frame, (0.98, 0.040, 0.045), (0.02, -0.17, 0.340), metal, "frame_rail_0")
    _add_box(main_frame, (0.98, 0.040, 0.045), (0.02, 0.17, 0.340), metal, "frame_rail_1")
    _add_box(main_frame, (0.070, 0.470, 0.050), (-0.43, 0.0, 0.340), metal, "rear_crossbar")
    _add_box(main_frame, (0.085, 0.470, 0.050), (0.51, 0.0, 0.340), metal, "front_crossbar")
    _add_box(main_frame, (0.92, 0.052, 0.040), (0.02, 0.0, 0.345), metal, "center_spine")

    # Knee-pad platform and four short riser posts tie the broad cushion into the frame.
    _add_box(main_frame, (0.58, 0.33, 0.035), (-0.12, 0.0, 0.4325), metal, "pad_mount_plate")
    for ix, x in enumerate((-0.31, 0.06)):
        for iy, y in enumerate((-0.12, 0.12)):
            _add_box(main_frame, (0.052, 0.070, 0.070), (x, y, 0.385), metal, f"pad_post_{ix}_{iy}")

    # Fixed rear wheel supports and stub axles.
    for suffix, ysign in (("0", -1.0), ("1", 1.0)):
        y_support = ysign * 0.215
        y_axle = ysign * 0.285
        y_cap = ysign * 0.349
        _add_box(main_frame, (0.080, 0.050, 0.205), (-0.43, y_support, 0.255), metal, f"rear_dropout_{suffix}")
        _add_box(main_frame, (0.024, 0.185, 0.024), (-0.43, y_axle, 0.165), metal, f"rear_axle_{suffix}")
        _add_box(main_frame, (0.052, 0.026, 0.052), (-0.43, y_cap, 0.165), metal, f"rear_retainer_{suffix}")

    # Front head bracket: split cheeks leave clearance for the rotating steering shaft.
    _add_box(main_frame, (0.170, 0.180, 0.042), (0.52, 0.0, 0.335), metal, "head_base")
    _add_box(main_frame, (0.090, 0.035, 0.185), (0.53, -0.060, 0.435), metal, "head_cheek_0")
    _add_box(main_frame, (0.090, 0.035, 0.185), (0.53, 0.060, 0.435), metal, "head_cheek_1")
    main_frame.visual(
        Cylinder(radius=0.045, length=0.130),
        origin=Origin(xyz=(0.53, 0.0, 0.430)),
        material=metal,
        name="head_tube",
    )

    knee_pad = model.part("knee_pad")
    knee_pad.visual(
        mesh_from_cadquery(_rounded_knee_pad(), "rounded_knee_pad", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=pad_mat,
        name="cushion",
    )

    steering_fork = model.part("steering_fork")
    # The child frame sits on the vertical steering axis at the head bracket.
    steering_fork.visual(
        Cylinder(radius=0.022, length=0.65),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=accent,
        name="steering_column",
    )
    steering_fork.visual(
        Cylinder(radius=0.018, length=0.54),
        origin=Origin(xyz=(0.0, 0.0, 0.585), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="handlebar")
    for suffix, y in (("0", -0.300), ("1", 0.300)):
        steering_fork.visual(
            Cylinder(radius=0.024, length=0.115),
            origin=Origin(xyz=(0.0, y, 0.585), rpy=(pi / 2.0, 0.0, 0.0)),
            material=grip_mat,
            name=f"grip_{suffix}",
        )
    steering_fork.visual(
        Box((0.180, 0.050, 0.040)),
        origin=Origin(xyz=(0.080, 0.0, -0.045)),
        material=accent,
        name="stem_bridge",
    )
    _add_box(steering_fork, (0.175, 0.410, 0.052), (0.160, 0.0, -0.060), accent, "fork_crown")
    _add_box(steering_fork, (0.060, 0.032, 0.255), (0.160, -0.190, -0.185), accent, "fork_leg_0")
    _add_box(steering_fork, (0.060, 0.032, 0.255), (0.160, 0.190, -0.185), accent, "fork_leg_1")
    steering_fork.visual(
        Box((0.024, 0.455, 0.024)),
        origin=Origin(xyz=(0.160, 0.0, -0.265)),
        material=accent,
        name="front_axle",
    )
    _add_box(steering_fork, (0.040, 0.020, 0.056), (0.160, -0.235, -0.265), accent, "front_retainer_0")
    _add_box(steering_fork, (0.040, 0.020, 0.056), (0.160, 0.235, -0.265), accent, "front_retainer_1")
    _add_box(steering_fork, (0.032, 0.070, 0.032), (0.160, 0.0, -0.265), accent, "front_center_spacer")

    def add_wheel(name: str):
        wheel = model.part(name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=rim_mat,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.030, length=0.070),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=rim_mat,
            name="bearing_sleeve",
        )
        return wheel

    rear_wheel_0 = add_wheel("rear_wheel_0")
    rear_wheel_1 = add_wheel("rear_wheel_1")
    front_wheel_0 = add_wheel("front_wheel_0")
    front_wheel_1 = add_wheel("front_wheel_1")

    model.articulation(
        "frame_to_pad",
        ArticulationType.FIXED,
        parent=main_frame,
        child=knee_pad,
        origin=Origin(xyz=(-0.12, 0.0, 0.450)),
    )
    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=main_frame,
        child=steering_fork,
        origin=Origin(xyz=(0.53, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    for suffix, wheel, y in (
        ("0", rear_wheel_0, -0.285),
        ("1", rear_wheel_1, 0.285),
    ):
        model.articulation(
            f"rear_wheel_spin_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=main_frame,
            child=wheel,
            origin=Origin(xyz=(-0.43, y, 0.165)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=30.0),
        )
    for suffix, wheel, y in (
        ("0", front_wheel_0, -0.105),
        ("1", front_wheel_1, 0.105),
    ):
        model.articulation(
            f"front_wheel_spin_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=steering_fork,
            child=wheel,
            origin=Origin(xyz=(0.160, y, -0.265)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=30.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    steering = object_model.get_articulation("steering_yaw")
    fork = object_model.get_part("steering_fork")

    ctx.check(
        "front fork has realistic yaw limits",
        steering.motion_limits is not None
        and steering.motion_limits.lower is not None
        and steering.motion_limits.upper is not None
        and steering.motion_limits.lower < -0.6
        and steering.motion_limits.upper > 0.6,
        details=f"limits={steering.motion_limits}",
    )

    ctx.allow_overlap(
        "main_frame",
        "steering_fork",
        elem_a="head_tube",
        elem_b="steering_column",
        reason="The steering column is intentionally captured inside the frame head tube bearing.",
    )
    ctx.allow_overlap(
        "main_frame",
        "steering_fork",
        elem_a="head_tube",
        elem_b="stem_bridge",
        reason="The lower steering lug is seated through the head tube bearing so the fork remains captured while yawing.",
    )
    ctx.expect_overlap(
        "main_frame",
        "steering_fork",
        axes="xyz",
        elem_a="head_tube",
        elem_b="steering_column",
        min_overlap=0.035,
        name="steering column remains captured in head tube",
    )
    ctx.expect_overlap(
        "main_frame",
        "steering_fork",
        axes="xyz",
        elem_a="head_tube",
        elem_b="stem_bridge",
        min_overlap=0.035,
        name="steering lug remains captured in head tube",
    )

    wheel_joint_names = (
        "rear_wheel_spin_0",
        "rear_wheel_spin_1",
        "front_wheel_spin_0",
        "front_wheel_spin_1",
    )
    for joint_name in wheel_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name} spins on a lateral axle",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    # The front pair is captured by the steerable fork: the fork axle crosses each
    # front wheel hub projection, while the wheels remain separate spinning parts.
    for wheel_name in ("front_wheel_0", "front_wheel_1"):
        ctx.allow_overlap(
            fork,
            wheel_name,
            elem_a="front_axle",
            elem_b="bearing_sleeve",
            reason="The front axle intentionally passes through the wheel bearing sleeve so the pair stays clipped into the steerable fork.",
        )
        ctx.expect_overlap(
            wheel_name,
            fork,
            axes="xyz",
            elem_a="bearing_sleeve",
            elem_b="front_axle",
            min_overlap=0.018,
            name=f"{wheel_name} bearing is retained on front axle",
        )

    for suffix, wheel_name in (("0", "rear_wheel_0"), ("1", "rear_wheel_1")):
        ctx.allow_overlap(
            "main_frame",
            wheel_name,
            elem_a=f"rear_axle_{suffix}",
            elem_b="bearing_sleeve",
            reason="The fixed rear axle intentionally passes through the wheel bearing sleeve.",
        )
        ctx.expect_overlap(
            "main_frame",
            wheel_name,
            axes="xyz",
            elem_a=f"rear_axle_{suffix}",
            elem_b="bearing_sleeve",
            min_overlap=0.018,
            name=f"{wheel_name} bearing is retained on fixed axle",
        )

    front_before = ctx.part_world_position("front_wheel_1")
    rear_before = ctx.part_world_position("rear_wheel_1")
    with ctx.pose({steering: 0.60}):
        front_after = ctx.part_world_position("front_wheel_1")
        rear_after = ctx.part_world_position("rear_wheel_1")
    ctx.check(
        "front pair follows steering yaw",
        front_before is not None
        and front_after is not None
        and abs(front_after[0] - front_before[0]) > 0.020
        and abs(front_after[1] - front_before[1]) > 0.005,
        details=f"front_before={front_before}, front_after={front_after}",
    )
    ctx.check(
        "rear wheels stay fixed during steering",
        rear_before is not None
        and rear_after is not None
        and abs(rear_after[0] - rear_before[0]) < 1e-6
        and abs(rear_after[1] - rear_before[1]) < 1e-6,
        details=f"rear_before={rear_before}, rear_after={rear_after}",
    )

    return ctx.report()


object_model = build_object_model()
