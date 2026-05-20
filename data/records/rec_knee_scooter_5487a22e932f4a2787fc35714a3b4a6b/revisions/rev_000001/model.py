from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_knee_scooter")

    frame_blue = model.material("powder_coated_blue", rgba=(0.05, 0.18, 0.48, 1.0))
    satin_metal = model.material("satin_aluminum", rgba=(0.72, 0.75, 0.77, 1.0))
    dark_metal = model.material("black_hardware", rgba=(0.015, 0.015, 0.018, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    pad_vinyl = model.material("navy_padded_vinyl", rgba=(0.015, 0.04, 0.10, 1.0))

    axle_rpy = (math.pi / 2.0, 0.0, 0.0)
    wheel_frame_rpy = (0.0, 0.0, math.pi / 2.0)

    frame = model.part("frame")
    frame.visual(
        Box((0.75, 0.12, 0.035)),
        origin=Origin(xyz=(-0.05, 0.0, 0.245)),
        material=frame_blue,
        name="narrow_deck",
    )
    frame.visual(
        Box((0.22, 0.10, 0.045)),
        origin=Origin(xyz=(0.35, 0.0, 0.275)),
        material=frame_blue,
        name="front_neck",
    )
    frame.visual(
        Box((0.36, 0.10, 0.090)),
        origin=Origin(xyz=(-0.10, 0.0, 0.305)),
        material=frame_blue,
        name="pad_pedestal",
    )
    frame.visual(
        Box((0.45, 0.22, 0.070)),
        origin=Origin(xyz=(-0.11, 0.0, 0.385)),
        material=pad_vinyl,
        name="knee_pad",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.430),
        origin=Origin(xyz=(-0.43, 0.0, 0.100), rpy=axle_rpy),
        material=satin_metal,
        name="rear_axle",
    )
    for suffix, y in (("0", -0.055), ("1", 0.055)):
        frame.visual(
            Box((0.040, 0.025, 0.160)),
            origin=Origin(xyz=(-0.43, y, 0.170)),
            material=frame_blue,
            name=f"rear_dropout_{suffix}",
        )
    frame.visual(
        Box((0.075, 0.018, 0.110)),
        origin=Origin(xyz=(0.43, -0.084, 0.340)),
        material=frame_blue,
        name="front_hinge_cheek_0",
    )
    frame.visual(
        Box((0.075, 0.018, 0.110)),
        origin=Origin(xyz=(0.43, 0.084, 0.340)),
        material=frame_blue,
        name="front_hinge_cheek_1",
    )
    for suffix, y in (("0", -0.059), ("1", 0.059)):
        frame.visual(
            Box((0.075, 0.050, 0.030)),
            origin=Origin(xyz=(0.43, y, 0.300)),
            material=frame_blue,
            name=f"hinge_side_bridge_{suffix}",
        )

    steering_base = model.part("steering_base")
    steering_base.visual(
        Cylinder(radius=0.016, length=0.150),
        origin=Origin(rpy=axle_rpy),
        material=dark_metal,
        name="fold_barrel",
    )
    for suffix, y in (("0", -0.035), ("1", 0.035)):
        steering_base.visual(
            Box((0.055, 0.012, 0.065)),
            origin=Origin(xyz=(0.0, y, 0.035)),
            material=frame_blue,
            name=f"hinge_web_{suffix}",
        )

    # A square, open steering-head bearing block.  The yawing steerer shaft runs
    # through the clear center instead of being hidden inside a solid proxy.
    head_outer = 0.075
    head_inner = 0.052
    head_wall = (head_outer - head_inner) / 2.0
    head_wall_offset = head_inner / 2.0 + head_wall / 2.0
    head_height = 0.220
    for name, xyz, size in (
        ("head_x_wall_pos", (head_wall_offset, 0.0, 0.130), (head_wall, head_outer, head_height)),
        ("head_x_wall_neg", (-head_wall_offset, 0.0, 0.130), (head_wall, head_outer, head_height)),
        ("head_y_wall_pos", (0.0, head_wall_offset, 0.130), (head_outer, head_wall, head_height)),
        ("head_y_wall_neg", (0.0, -head_wall_offset, 0.130), (head_outer, head_wall, head_height)),
    ):
        steering_base.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=frame_blue,
            name=name,
        )

    lower_column = model.part("lower_column")
    lower_column.visual(
        Cylinder(radius=0.017, length=0.215),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=satin_metal,
        name="steerer_shaft",
    )
    lower_column.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=dark_metal,
        name="upper_bearing_race",
    )
    lower_column.visual(
        Box((0.040, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=satin_metal,
        name="mast_bottom_stop",
    )
    for name, xyz, size in (
        ("collar_x_wall_pos", (0.040, 0.0, 0.140), (0.020, 0.090, 0.030)),
        ("collar_x_wall_neg", (-0.040, 0.0, 0.140), (0.020, 0.090, 0.030)),
        ("collar_y_wall_pos", (0.0, 0.040, 0.140), (0.090, 0.020, 0.030)),
        ("collar_y_wall_neg", (0.0, -0.040, 0.140), (0.090, 0.020, 0.030)),
    ):
        lower_column.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=satin_metal,
            name=name,
        )

    sleeve_outer = 0.060
    sleeve_inner = 0.040
    sleeve_wall = (sleeve_outer - sleeve_inner) / 2.0
    sleeve_offset = sleeve_inner / 2.0 + sleeve_wall / 2.0
    sleeve_height = 0.330
    sleeve_center_z = 0.295
    for name, xyz, size in (
        ("sleeve_x_wall_pos", (sleeve_offset, 0.0, sleeve_center_z), (sleeve_wall, sleeve_outer, sleeve_height)),
        ("sleeve_x_wall_neg", (-sleeve_offset, 0.0, sleeve_center_z), (sleeve_wall, sleeve_outer, sleeve_height)),
        ("sleeve_y_wall_pos", (0.0, sleeve_offset, sleeve_center_z), (sleeve_outer, sleeve_wall, sleeve_height)),
        ("sleeve_y_wall_neg", (0.0, -sleeve_offset, sleeve_center_z), (sleeve_outer, sleeve_wall, sleeve_height)),
    ):
        lower_column.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=satin_metal,
            name=name,
        )

    lower_column.visual(
        Box((0.130, 0.220, 0.040)),
        origin=Origin(xyz=(0.105, 0.0, -0.215)),
        material=frame_blue,
        name="fork_crown",
    )
    lower_column.visual(
        Box((0.040, 0.034, 0.350)),
        origin=Origin(xyz=(0.070, 0.0, -0.030)),
        material=frame_blue,
        name="fork_upright",
    )
    for suffix, y in (("0", -0.105), ("1", 0.105)):
        lower_column.visual(
            Box((0.034, 0.026, 0.300)),
            origin=Origin(xyz=(0.070, y, -0.265)),
            material=frame_blue,
            name=f"fork_leg_{suffix}",
        )
    lower_column.visual(
        Cylinder(radius=0.012, length=0.430),
        origin=Origin(xyz=(0.070, 0.0, -0.380), rpy=axle_rpy),
        material=satin_metal,
        name="front_axle",
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.032, 0.032, 0.750)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=satin_metal,
        name="inner_mast",
    )
    for name, xyz, size in (
        ("glide_pad_x_pos", (0.01775, 0.0, -0.160), (0.0045, 0.018, 0.040)),
        ("glide_pad_x_neg", (-0.01775, 0.0, -0.160), (0.0045, 0.018, 0.040)),
        ("glide_pad_y_pos", (0.0, 0.01775, -0.160), (0.018, 0.0045, 0.040)),
        ("glide_pad_y_neg", (0.0, -0.01775, -0.160), (0.018, 0.0045, 0.040)),
    ):
        mast.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=dark_metal,
            name=name,
        )
    mast.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.438), rpy=axle_rpy),
        material=satin_metal,
        name="handlebar_crossbar",
    )
    for suffix, y in (("0", -0.255), ("1", 0.255)):
        mast.visual(
            Cylinder(radius=0.019, length=0.110),
            origin=Origin(xyz=(0.0, y, 0.438), rpy=axle_rpy),
            material=rubber,
            name=f"grip_{suffix}",
        )

    tire_geometry = TireGeometry(
        0.085,
        0.046,
        inner_radius=0.057,
        tread=TireTread(style="block", depth=0.004, count=16, land_ratio=0.60),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    wheel_geometry = WheelGeometry(
        0.058,
        0.040,
        rim=WheelRim(inner_radius=0.038, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.025,
            width=0.044,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.035, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=5, thickness=0.004, window_radius=0.012),
        bore=WheelBore(style="round", diameter=0.024),
    )

    wheel_specs = (
        ("rear_wheel_0", "rear", (-0.43, -0.185, 0.090)),
        ("rear_wheel_1", "rear", (-0.43, 0.185, 0.090)),
        ("front_wheel_0", "front", (0.070, -0.185, -0.380)),
        ("front_wheel_1", "front", (0.070, 0.185, -0.380)),
    )
    for index, (part_name, _, _) in enumerate(wheel_specs):
        wheel = model.part(part_name)
        wheel.visual(
            mesh_from_geometry(tire_geometry, f"{part_name}_tire"),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(wheel_geometry, f"{part_name}_rim"),
            material=satin_metal,
            name="rim",
        )

    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering_base,
        origin=Origin(xyz=(0.43, 0.0, 0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5, lower=-1.15, upper=0.0),
    )
    model.articulation(
        "steering_yaw",
        ArticulationType.REVOLUTE,
        parent=steering_base,
        child=lower_column,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=3.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=lower_column,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.30, lower=0.0, upper=0.20),
    )

    for part_name, location, xyz in wheel_specs:
        parent = frame if location == "rear" else lower_column
        model.articulation(
            f"{part_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=part_name,
            origin=Origin(xyz=xyz, rpy=wheel_frame_rpy),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=25.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lower_column = object_model.get_part("lower_column")
    mast = object_model.get_part("mast")
    steering = object_model.get_articulation("steering_yaw")
    slide = object_model.get_articulation("mast_slide")
    fold = object_model.get_articulation("fold_hinge")

    wheel_joints = [
        object_model.get_articulation("rear_wheel_0_spin"),
        object_model.get_articulation("rear_wheel_1_spin"),
        object_model.get_articulation("front_wheel_0_spin"),
        object_model.get_articulation("front_wheel_1_spin"),
    ]
    ctx.check(
        "four independent wheel spin joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in wheel_joints),
        details=str([j.name for j in wheel_joints]),
    )
    for wheel_name, parent, axle_elem in (
        ("rear_wheel_0", frame, "rear_axle"),
        ("rear_wheel_1", frame, "rear_axle"),
        ("front_wheel_0", lower_column, "front_axle"),
        ("front_wheel_1", lower_column, "front_axle"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            parent,
            wheel,
            elem_a=axle_elem,
            elem_b="rim",
            reason="The metal axle is intentionally captured through the wheel hub/bore proxy.",
        )
        ctx.expect_overlap(
            wheel,
            parent,
            axes="y",
            elem_a="rim",
            elem_b=axle_elem,
            min_overlap=0.035,
            name=f"{wheel_name} hub is retained on axle",
        )

    # The inner mast is a real separate sliding part with clearance inside the
    # square lower sleeve, not a fused decoration on the steering column.
    ctx.expect_gap(
        lower_column,
        mast,
        axis="x",
        positive_elem="sleeve_x_wall_pos",
        negative_elem="inner_mast",
        min_gap=0.001,
        max_gap=0.010,
        name="mast clears positive x sleeve wall",
    )
    ctx.expect_gap(
        mast,
        lower_column,
        axis="x",
        positive_elem="inner_mast",
        negative_elem="sleeve_x_wall_neg",
        min_gap=0.001,
        max_gap=0.010,
        name="mast clears negative x sleeve wall",
    )
    ctx.expect_gap(
        lower_column,
        mast,
        axis="y",
        positive_elem="sleeve_y_wall_pos",
        negative_elem="inner_mast",
        min_gap=0.001,
        max_gap=0.010,
        name="mast clears positive y sleeve wall",
    )
    ctx.expect_gap(
        mast,
        lower_column,
        axis="y",
        positive_elem="inner_mast",
        negative_elem="sleeve_y_wall_neg",
        min_gap=0.001,
        max_gap=0.010,
        name="mast clears negative y sleeve wall",
    )
    ctx.expect_overlap(
        mast,
        lower_column,
        axes="z",
        elem_a="inner_mast",
        elem_b="sleeve_x_wall_pos",
        min_overlap=0.25,
        name="collapsed mast remains deeply inserted",
    )

    rest_mast_position = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.20}):
        extended_mast_position = ctx.part_world_position(mast)
        ctx.expect_overlap(
            mast,
            lower_column,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_x_wall_pos",
            min_overlap=0.10,
            name="extended mast retains insertion",
        )
    ctx.check(
        "mast slide raises the handlebar stage",
        rest_mast_position is not None
        and extended_mast_position is not None
        and extended_mast_position[2] > rest_mast_position[2] + 0.18,
        details=f"rest={rest_mast_position}, extended={extended_mast_position}",
    )

    front_wheel = object_model.get_part("front_wheel_1")
    rest_front_position = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.55}):
        yawed_front_position = ctx.part_world_position(front_wheel)
    ctx.check(
        "front fork yaws about the steering head",
        rest_front_position is not None
        and yawed_front_position is not None
        and math.hypot(
            yawed_front_position[0] - rest_front_position[0],
            yawed_front_position[1] - rest_front_position[1],
        )
        > 0.04,
        details=f"rest={rest_front_position}, yawed={yawed_front_position}",
    )

    rest_mast_aabb = ctx.part_world_aabb(mast)
    with ctx.pose({fold: -0.85}):
        folded_mast_aabb = ctx.part_world_aabb(mast)
    ctx.check(
        "fold hinge lays the upright rearward",
        rest_mast_aabb is not None
        and folded_mast_aabb is not None
        and folded_mast_aabb[0][0] < rest_mast_aabb[0][0] - 0.15,
        details=f"rest={rest_mast_aabb}, folded={folded_mast_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
