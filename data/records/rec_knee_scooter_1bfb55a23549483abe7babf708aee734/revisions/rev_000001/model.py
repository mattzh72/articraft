from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    ExtrudeGeometry,
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
    superellipse_profile,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """Open-bore round tube with real clearance for telescoping members."""
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)
    return mesh_from_cadquery(tube, name)


def _pad_mesh(name: str):
    pad_profile = superellipse_profile(0.38, 0.20, exponent=3.4, segments=64)
    return mesh_from_geometry(ExtrudeGeometry.centered(pad_profile, 0.055), name)


def _add_wheel_visuals(part, radius: float, width: float, name_prefix: str, tire_mat: Material, rim_mat: Material):
    rim_radius = radius * 0.64
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            radius,
            width,
            inner_radius=rim_radius * 0.94,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.08),
            tread=TireTread(style="block", depth=radius * 0.035, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=width * 0.12, depth=radius * 0.012),),
            sidewall=TireSidewall(style="rounded", bulge=0.06),
            shoulder=TireShoulder(width=width * 0.13, radius=radius * 0.035),
        ),
        f"{name_prefix}_tire_mesh",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            rim_radius,
            width * 0.82,
            rim=WheelRim(
                inner_radius=rim_radius * 0.58,
                flange_height=radius * 0.035,
                flange_thickness=width * 0.09,
                bead_seat_depth=radius * 0.015,
            ),
            hub=WheelHub(
                radius=radius * 0.23,
                width=width * 0.70,
                cap_style="flat",
                bolt_pattern=BoltPattern(
                    count=5,
                    circle_diameter=radius * 0.26,
                    hole_diameter=radius * 0.035,
                ),
            ),
            face=WheelFace(dish_depth=width * 0.08, front_inset=width * 0.06, rear_inset=width * 0.04),
            spokes=WheelSpokes(style="split_y", count=5, thickness=width * 0.055, window_radius=radius * 0.10),
            bore=WheelBore(style="round", diameter=0.026 if radius > 0.09 else 0.020),
        ),
        f"{name_prefix}_rim_mesh",
    )
    part.visual(tire_mesh, material=tire_mat, name="tire")
    part.visual(wheel_mesh, material=rim_mat, name="rim")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_knee_scooter")

    satin_black = model.material("satin_black", rgba=(0.02, 0.025, 0.025, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.05, 0.06, 0.07, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    blue_vinyl = model.material("blue_vinyl", rgba=(0.02, 0.05, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.465, 0.060, 0.055)),
        origin=Origin(xyz=(0.1375, 0.0, 0.260)),
        material=dark_frame,
        name="front_spine",
    )
    frame.visual(
        Box((0.225, 0.060, 0.055)),
        origin=Origin(xyz=(-0.2575, 0.0, 0.260)),
        material=dark_frame,
        name="rear_spine",
    )
    frame.visual(
        _tube_mesh(0.055, 0.038, 0.160, "head_collar_mesh"),
        origin=Origin(xyz=(0.420, 0.0, 0.222)),
        material=brushed_metal,
        name="head_collar",
    )
    frame.visual(
        Cylinder(radius=0.017, length=0.460),
        origin=Origin(xyz=(-0.340, 0.0, 0.260), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_frame,
        name="rear_top_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.560),
        origin=Origin(xyz=(-0.360, 0.0, 0.110), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="rear_axle",
    )
    for idx, y in enumerate((-0.185, 0.185)):
        frame.visual(
            Box((0.040, 0.032, 0.155)),
            origin=Origin(xyz=(-0.360, y, 0.180)),
            material=dark_frame,
            name=f"rear_support_{idx}",
        )
    frame.visual(
        _tube_mesh(0.034, 0.023, 0.250, "knee_sleeve_mesh"),
        origin=Origin(xyz=(-0.120, 0.0, 0.230)),
        material=brushed_metal,
        name="knee_sleeve",
    )
    for idx, y in enumerate((-0.042, 0.042)):
        frame.visual(
            Box((0.120, 0.018, 0.020)),
            origin=Origin(xyz=(-0.120, y, 0.242)),
            material=dark_frame,
            name=f"knee_mount_lug_{idx}",
        )
    frame.visual(
        _tube_mesh(0.045, 0.024, 0.035, "knee_clamp_collar_mesh"),
        origin=Origin(xyz=(-0.120, 0.0, 0.4675)),
        material=satin_black,
        name="knee_clamp_collar",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        _tube_mesh(0.028, 0.019, 0.495, "steering_sleeve_mesh"),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=brushed_metal,
        name="steering_sleeve",
    )
    front_fork.visual(
        Cylinder(radius=0.038, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=brushed_metal,
        name="steerer_bearing",
    )
    front_fork.visual(
        Box((0.110, 0.255, 0.035)),
        origin=Origin(xyz=(0.020, 0.0, -0.060)),
        material=dark_frame,
        name="fork_crown",
    )
    for idx, y in enumerate((-0.120, 0.120)):
        front_fork.visual(
            Box((0.040, 0.026, 0.230)),
            origin=Origin(xyz=(0.020, y, -0.125)),
            material=dark_frame,
            name=f"fork_leg_{idx}",
        )
    front_fork.visual(
        Cylinder(radius=0.010, length=0.292),
        origin=Origin(xyz=(0.020, 0.0, -0.170), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="front_axle",
    )

    knee_post = model.part("knee_post")
    knee_post.visual(
        Cylinder(radius=0.017, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=brushed_metal,
        name="knee_inner_post",
    )
    knee_post.visual(
        Cylinder(radius=0.023, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
        material=satin_black,
        name="knee_guide_bushing",
    )
    knee_post.visual(
        Box((0.210, 0.120, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
        material=satin_black,
        name="pad_plate",
    )
    knee_post.visual(
        _pad_mesh("knee_pad_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material=blue_vinyl,
        name="knee_pad",
    )

    handle_stem = model.part("handle_stem")
    handle_stem.visual(
        Cylinder(radius=0.015, length=0.580),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed_metal,
        name="handle_inner_stem",
    )
    handle_stem.visual(
        Cylinder(radius=0.019, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
        material=satin_black,
        name="handle_guide_bushing",
    )
    handle_stem.visual(
        Cylinder(radius=0.014, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.360), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="handlebar",
    )
    for idx, y in enumerate((-0.300, 0.300)):
        handle_stem.visual(
            Cylinder(radius=0.023, length=0.145),
            origin=Origin(xyz=(0.0, y, 0.360), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{idx}",
        )

    rear_wheel_positions = ((-0.360, -0.230, 0.110), (-0.360, 0.230, 0.110))
    for idx, xyz in enumerate(rear_wheel_positions):
        wheel = model.part(f"rear_wheel_{idx}")
        _add_wheel_visuals(wheel, 0.110, 0.055, f"rear_wheel_{idx}", rubber, brushed_metal)
        model.articulation(
            f"rear_wheel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=wheel,
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=20.0),
        )

    front_wheel_positions = ((0.020, -0.065, -0.170), (0.020, 0.065, -0.170))
    for idx, xyz in enumerate(front_wheel_positions):
        wheel = model.part(f"front_wheel_{idx}")
        _add_wheel_visuals(wheel, 0.085, 0.042, f"front_wheel_{idx}", rubber, brushed_metal)
        model.articulation(
            f"front_wheel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=front_fork,
            child=wheel,
            origin=Origin(xyz=xyz, rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=22.0),
        )

    model.articulation(
        "fork_yaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.420, 0.0, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "knee_post_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=knee_post,
        origin=Origin(xyz=(-0.120, 0.0, 0.470)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.140),
    )
    model.articulation(
        "handle_stem_slide",
        ArticulationType.PRISMATIC,
        parent=front_fork,
        child=handle_stem,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.20, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    front_fork = object_model.get_part("front_fork")
    knee_post = object_model.get_part("knee_post")
    handle_stem = object_model.get_part("handle_stem")
    knee_slide = object_model.get_articulation("knee_post_slide")
    handle_slide = object_model.get_articulation("handle_stem_slide")
    fork_yaw = object_model.get_articulation("fork_yaw")

    ctx.allow_overlap(
        frame,
        front_fork,
        elem_a="head_collar",
        elem_b="steerer_bearing",
        reason="The steering bearing proxy is intentionally captured inside the frame head collar.",
    )
    ctx.expect_overlap(
        frame,
        front_fork,
        axes="z",
        elem_a="head_collar",
        elem_b="steerer_bearing",
        min_overlap=0.12,
        name="steerer bearing is retained in head collar",
    )
    ctx.allow_overlap(
        frame,
        knee_post,
        elem_a="knee_sleeve",
        elem_b="knee_guide_bushing",
        reason="The low-friction guide bushing is intentionally clipped against the knee height sleeve bore.",
    )
    ctx.expect_overlap(
        frame,
        knee_post,
        axes="z",
        elem_a="knee_sleeve",
        elem_b="knee_guide_bushing",
        min_overlap=0.019,
        name="knee guide bushing is inside sleeve",
    )
    ctx.allow_overlap(
        front_fork,
        handle_stem,
        elem_a="steering_sleeve",
        elem_b="handle_guide_bushing",
        reason="The handle guide bushing is intentionally fitted against the telescoping steering sleeve bore.",
    )
    ctx.expect_overlap(
        front_fork,
        handle_stem,
        axes="z",
        elem_a="steering_sleeve",
        elem_b="handle_guide_bushing",
        min_overlap=0.020,
        name="handle guide bushing is inside steering sleeve",
    )

    for wheel_name in ("rear_wheel_0", "rear_wheel_1"):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The rear axle shaft is intentionally captured through the wheel hub bearing proxy.",
        )
        ctx.expect_overlap(
            frame,
            wheel,
            axes="xyz",
            elem_a="rear_axle",
            elem_b="rim",
            min_overlap=0.020,
            name=f"{wheel_name} hub is retained on rear axle",
        )
    for wheel_name in ("front_wheel_0", "front_wheel_1"):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            front_fork,
            wheel,
            elem_a="front_axle",
            elem_b="rim",
            reason="The front axle shaft is intentionally captured through the small wheel hub bearing proxy.",
        )
        ctx.expect_overlap(
            front_fork,
            wheel,
            axes="xyz",
            elem_a="front_axle",
            elem_b="rim",
            min_overlap=0.016,
            name=f"{wheel_name} hub is retained on fork axle",
        )

    ctx.expect_within(
        knee_post,
        frame,
        axes="xy",
        inner_elem="knee_inner_post",
        outer_elem="knee_sleeve",
        margin=0.001,
        name="knee post centered in height sleeve",
    )
    ctx.expect_overlap(
        knee_post,
        frame,
        axes="z",
        elem_a="knee_inner_post",
        elem_b="knee_sleeve",
        min_overlap=0.18,
        name="collapsed knee post remains deeply captured",
    )
    with ctx.pose({knee_slide: 0.140}):
        ctx.expect_within(
            knee_post,
            frame,
            axes="xy",
            inner_elem="knee_inner_post",
            outer_elem="knee_sleeve",
            margin=0.001,
            name="raised knee post stays inside sleeve bore",
        )
        ctx.expect_overlap(
            knee_post,
            frame,
            axes="z",
            elem_a="knee_inner_post",
            elem_b="knee_sleeve",
            min_overlap=0.085,
            name="raised knee post retains captured insertion",
        )

    ctx.expect_within(
        handle_stem,
        front_fork,
        axes="xy",
        inner_elem="handle_inner_stem",
        outer_elem="steering_sleeve",
        margin=0.001,
        name="handle stem centered in steering sleeve",
    )
    ctx.expect_overlap(
        handle_stem,
        front_fork,
        axes="z",
        elem_a="handle_inner_stem",
        elem_b="steering_sleeve",
        min_overlap=0.22,
        name="collapsed handle stem remains inserted",
    )
    with ctx.pose({handle_slide: 0.180}):
        ctx.expect_within(
            handle_stem,
            front_fork,
            axes="xy",
            inner_elem="handle_inner_stem",
            outer_elem="steering_sleeve",
            margin=0.001,
            name="extended handle stem stays centered",
        )
        ctx.expect_overlap(
            handle_stem,
            front_fork,
            axes="z",
            elem_a="handle_inner_stem",
            elem_b="steering_sleeve",
            min_overlap=0.045,
            name="extended handle stem retains insertion",
        )

    rest_wheel_pos = ctx.part_world_position(object_model.get_part("front_wheel_0"))
    with ctx.pose({fork_yaw: 0.55}):
        yawed_wheel_pos = ctx.part_world_position(object_model.get_part("front_wheel_0"))
    ctx.check(
        "front fork yaw moves wheel about steering axis",
        rest_wheel_pos is not None
        and yawed_wheel_pos is not None
        and abs(yawed_wheel_pos[1] - rest_wheel_pos[1]) > 0.010,
        details=f"rest={rest_wheel_pos}, yawed={yawed_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
