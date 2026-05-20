from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
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
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _hollow_tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
    """Closed annular tube centered on local Z, with a real through bore."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _rounded_plate_mesh(name: str, length: float, width: float, thickness: float, radius: float):
    profile = rounded_rect_profile(length, width, radius, corner_segments=10)
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knee_scooter")

    steel = model.material("satin_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    charcoal_foam = model.material("charcoal_foam", rgba=(0.035, 0.035, 0.040, 1.0))
    matte_gray = model.material("matte_gray", rgba=(0.22, 0.24, 0.25, 1.0))

    # Reusable visible hardware meshes.
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.070,
            0.044,
            rim=WheelRim(inner_radius=0.044, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(radius=0.024, width=0.038, cap_style="domed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.008),
        ),
        "gray_mag_wheel",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.095,
            0.050,
            inner_radius=0.068,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.004, count=22, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.005, radius=0.002),
        ),
        "black_scooter_tire",
    )
    knee_pad_mesh = _rounded_plate_mesh("rounded_knee_pad", 0.54, 0.245, 0.055, 0.060)
    head_tube_mesh = _hollow_tube_mesh("head_tube_bearing_shell", 0.045, 0.033, 0.250)
    clamp_band_mesh = _hollow_tube_mesh("handlebar_clamp_band", 0.040, 0.020, 0.046)
    sleeve_mesh = _hollow_tube_mesh("telescoping_handlebar_sleeve", 0.027, 0.018, 0.350)

    frame = model.part("frame")
    # Knee platform: a metal deck with a shaped, padded foam cushion and a shallow center groove.
    frame.visual(
        Box((0.58, 0.245, 0.035)),
        origin=Origin(xyz=(-0.10, 0.0, 0.373)),
        material=steel,
        name="deck_plate",
    )
    frame.visual(
        knee_pad_mesh,
        origin=Origin(xyz=(-0.10, 0.0, 0.418)),
        material=charcoal_foam,
        name="knee_pad",
    )
    frame.visual(
        Box((0.48, 0.012, 0.006)),
        origin=Origin(xyz=(-0.10, 0.0, 0.448)),
        material=dark_steel,
        name="pad_center_groove",
    )
    frame.visual(
        Box((0.46, 0.036, 0.012)),
        origin=Origin(xyz=(-0.10, 0.078, 0.451)),
        material=charcoal_foam,
        name="pad_side_bolster_0",
    )
    frame.visual(
        Box((0.46, 0.036, 0.012)),
        origin=Origin(xyz=(-0.10, -0.078, 0.451)),
        material=charcoal_foam,
        name="pad_side_bolster_1",
    )

    # Under-deck tubular frame.
    for y, name in ((0.090, "rail_0"), (-0.090, "rail_1")):
        frame.visual(
            Cylinder(radius=0.016, length=0.820),
            origin=Origin(xyz=(0.000, y, 0.340), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=name,
        )
    for x, name in ((-0.360, "deck_crossbar_0"), (0.255, "deck_crossbar_1")):
        frame.visual(
            Cylinder(radius=0.014, length=0.240),
            origin=Origin(xyz=(x, 0.0, 0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=name,
        )

    # Rear axle carrier: split bearing caps and cheeks touch the axle tangent
    # line without enclosing it as a simplified solid block.
    frame.visual(
        Cylinder(radius=0.016, length=0.340),
        origin=Origin(xyz=(-0.420, 0.0, 0.198), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_hanger_crossbar",
    )
    for y, name in ((0.135, "rear_bearing_0"), (-0.135, "rear_bearing_1")):
        frame.visual(
            Box((0.058, 0.036, 0.018)),
            origin=Origin(xyz=(-0.420, y, 0.116)),
            material=matte_gray,
            name=name,
        )
        for dx, cheek_name in ((0.024, "outer"), (-0.024, "inner")):
            frame.visual(
                Box((0.010, 0.036, 0.050)),
                origin=Origin(xyz=(-0.420 + dx, y, 0.095)),
                material=matte_gray,
                name=f"{name}_{cheek_name}_cheek",
            )
        frame.visual(
            Box((0.036, 0.034, 0.078)),
            origin=Origin(xyz=(-0.420, y, 0.156)),
            material=steel,
            name=f"rear_drop_link_{0 if y > 0 else 1}",
        )
    for y, name in ((0.090, "rear_upright_0"), (-0.090, "rear_upright_1")):
        frame.visual(
            Box((0.040, 0.026, 0.160)),
            origin=Origin(xyz=(-0.395, y, 0.270)),
            material=steel,
            name=name,
        )

    # Front steering head and gusseted deck connection.
    for y, name in ((0.060, "front_head_block_0"), (-0.060, "front_head_block_1")):
        frame.visual(
            Box((0.145, 0.050, 0.070)),
            origin=Origin(xyz=(0.390, y, 0.360)),
            material=steel,
            name=name,
        )
    frame.visual(
        head_tube_mesh,
        origin=Origin(xyz=(0.420, 0.0, 0.490)),
        material=matte_gray,
        name="head_tube",
    )
    # Four small bearing pads protrude into the headset bore to give the steering
    # stem a real, non-floating contact path while preserving steering rotation.
    for dx, dy, sx, sy, name in (
        (0.034, 0.0, 0.014, 0.014, "steering_bearing_pad_x0"),
        (-0.034, 0.0, 0.014, 0.014, "steering_bearing_pad_x1"),
        (0.0, 0.034, 0.014, 0.014, "steering_bearing_pad_y0"),
        (0.0, -0.034, 0.014, 0.014, "steering_bearing_pad_y1"),
    ):
        frame.visual(
            Box((sx, sy, 0.105)),
            origin=Origin(xyz=(0.420 + dx, dy, 0.500)),
            material=dark_steel,
            name=name,
        )
    for y, name in ((0.060, "front_gusset_0"), (-0.060, "front_gusset_1")):
        frame.visual(
            Box((0.090, 0.020, 0.150)),
            origin=Origin(xyz=(0.355, y, 0.430), rpy=(0.0, -0.28, 0.0)),
            material=steel,
            name=name,
        )

    rear_wheels = model.part("rear_wheels")
    rear_wheels.visual(
        Cylinder(radius=0.012, length=0.500),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle_shaft",
    )
    for y, side in ((0.195, "0"), (-0.195, "1")):
        rear_wheels.visual(
            wheel_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=matte_gray,
            name=f"rear_wheel_{side}",
        )
        rear_wheels.visual(
            tire_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_rubber,
            name=f"rear_tire_{side}",
        )

    front_fork = model.part("front_fork")
    # Child frame is on the steering axis at the center of the head tube.
    front_fork.visual(
        Cylinder(radius=0.020, length=0.345),
        origin=Origin(xyz=(0.0, 0.0, -0.222), rpy=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="steering_stem",
    )
    front_fork.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=matte_gray,
        name="handlebar_sleeve",
    )
    for x, y, sx, sy, name in (
        (0.017, 0.0, 0.006, 0.010, "post_guide_x0"),
        (-0.017, 0.0, 0.006, 0.010, "post_guide_x1"),
        (0.0, 0.017, 0.010, 0.006, "post_guide_y0"),
        (0.0, -0.017, 0.010, 0.006, "post_guide_y1"),
    ):
        front_fork.visual(
            Box((sx, sy, 0.280)),
            origin=Origin(xyz=(x, y, 0.095)),
            material=dark_steel,
            name=name,
        )
    front_fork.visual(
        clamp_band_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=dark_steel,
        name="sleeve_clamp_band",
    )
    front_fork.visual(
        Box((0.120, 0.360, 0.036)),
        origin=Origin(xyz=(0.030, 0.0, -0.185)),
        material=steel,
        name="fork_crown",
    )
    for y, name in ((0.155, "fork_leg_0"), (-0.155, "fork_leg_1")):
        front_fork.visual(
            Cylinder(radius=0.014, length=0.248),
            origin=Origin(xyz=(0.030, y, -0.241)),
            material=steel,
            name=name,
        )
        front_fork.visual(
            Box((0.058, 0.036, 0.018)),
            origin=Origin(xyz=(0.030, y, -0.374)),
            material=matte_gray,
            name=f"front_axle_bearing_{0 if y > 0 else 1}",
        )
        for dx, cheek_name in ((0.024, "outer"), (-0.024, "inner")):
            front_fork.visual(
                Box((0.010, 0.036, 0.050)),
                origin=Origin(xyz=(0.030 + dx, y, -0.395)),
                material=matte_gray,
                name=f"front_axle_bearing_{0 if y > 0 else 1}_{cheek_name}_cheek",
            )

    front_wheels = model.part("front_wheels")
    front_wheels.visual(
        Cylinder(radius=0.012, length=0.340),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_axle_shaft",
    )
    for y, side in ((0.075, "0"), (-0.075, "1")):
        front_wheels.visual(
            wheel_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=matte_gray,
            name=f"front_wheel_{side}",
        )
        front_wheels.visual(
            tire_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_rubber,
            name=f"front_tire_{side}",
        )

    handlebar_post = model.part("handlebar_post")
    handlebar_post.visual(
        Cylinder(radius=0.014, length=0.640),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="inner_post",
    )
    handlebar_post.visual(
        Cylinder(radius=0.018, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.350), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handlebar_crossbar",
    )
    for y, name in ((0.285, "grip_0"), (-0.285, "grip_1")):
        handlebar_post.visual(
            Cylinder(radius=0.022, length=0.105),
            origin=Origin(xyz=(0.0, y, 0.350), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=name,
        )

    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheels,
        origin=Origin(xyz=(-0.420, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=12.0),
    )
    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.420, 0.0, 0.490)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheels,
        origin=Origin(xyz=(0.030, 0.0, -0.395)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=12.0),
    )
    model.articulation(
        "handlebar_height",
        ArticulationType.PRISMATIC,
        parent=front_fork,
        child=handlebar_post,
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_fork = object_model.get_part("front_fork")
    front_wheels = object_model.get_part("front_wheels")
    handlebar_post = object_model.get_part("handlebar_post")
    rear_wheels = object_model.get_part("rear_wheels")
    steering = object_model.get_articulation("steering")
    handlebar_height = object_model.get_articulation("handlebar_height")
    rear_axle = object_model.get_articulation("rear_axle")

    ctx.check(
        "steering joint is vertical",
        tuple(round(v, 3) for v in steering.axis) == (0.0, 0.0, 1.0),
        details=f"axis={steering.axis}",
    )
    ctx.check(
        "rear axle spins across scooter width",
        tuple(round(v, 3) for v in rear_axle.axis) == (0.0, 1.0, 0.0),
        details=f"axis={rear_axle.axis}",
    )

    ctx.expect_overlap(
        handlebar_post,
        front_fork,
        axes="z",
        elem_a="inner_post",
        elem_b="handlebar_sleeve",
        min_overlap=0.250,
        name="collapsed handlebar remains deeply inserted",
    )
    ctx.expect_within(
        handlebar_post,
        front_fork,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="handlebar_sleeve",
        margin=0.002,
        name="handlebar post is centered inside sleeve",
    )

    rest_pos = ctx.part_world_position(handlebar_post)
    with ctx.pose({handlebar_height: 0.160}):
        raised_pos = ctx.part_world_position(handlebar_post)
        ctx.expect_overlap(
            handlebar_post,
            front_fork,
            axes="z",
            elem_a="inner_post",
            elem_b="handlebar_sleeve",
            min_overlap=0.110,
            name="raised handlebar still has retained insertion",
        )
        ctx.expect_within(
            handlebar_post,
            front_fork,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="handlebar_sleeve",
            margin=0.002,
            name="raised handlebar stays centered in sleeve",
        )
    ctx.check(
        "height joint raises handlebar upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.150,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    front_rest = ctx.part_world_position(front_wheels)
    with ctx.pose({steering: 0.55}):
        front_steered = ctx.part_world_position(front_wheels)
    ctx.check(
        "steering yaw swings front wheel fork",
        front_rest is not None
        and front_steered is not None
        and abs(front_steered[1] - front_rest[1]) > 0.012,
        details=f"rest={front_rest}, steered={front_steered}",
    )
    ctx.expect_origin_gap(
        handlebar_post,
        rear_wheels,
        axis="z",
        min_gap=0.550,
        name="handlebar stands well above rear axle",
    )

    return ctx.report()


object_model = build_object_model()
