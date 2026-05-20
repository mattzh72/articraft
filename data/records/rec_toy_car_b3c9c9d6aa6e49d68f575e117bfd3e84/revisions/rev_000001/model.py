from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_utility_wagon")

    body_orange = model.material("body_orange", rgba=(0.86, 0.35, 0.16, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    tire_black = model.material("tire_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rim_gray = model.material("rim_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")

    body.visual(
        Box((0.390, 0.152, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        material=trim_gray,
        name="body_floor",
    )
    body.visual(
        Box((0.145, 0.014, 0.038)),
        origin=Origin(xyz=(0.108, 0.060, 0.084)),
        material=body_orange,
        name="left_hood_rail",
    )
    body.visual(
        Box((0.145, 0.014, 0.038)),
        origin=Origin(xyz=(0.108, -0.060, 0.084)),
        material=body_orange,
        name="right_hood_rail",
    )
    body.visual(
        Box((0.020, 0.132, 0.022)),
        origin=Origin(xyz=(0.041, 0.000, 0.097)),
        material=body_orange,
        name="cowl_bridge",
    )
    body.visual(
        Box((0.014, 0.132, 0.022)),
        origin=Origin(xyz=(0.089, 0.000, 0.083)),
        material=body_orange,
        name="windshield_base",
    )
    body.visual(
        Box((0.014, 0.132, 0.032)),
        origin=Origin(xyz=(0.185, 0.000, 0.078)),
        material=body_orange,
        name="front_grille",
    )
    body.visual(
        Box((0.018, 0.136, 0.014)),
        origin=Origin(xyz=(0.198, 0.000, 0.060)),
        material=trim_gray,
        name="front_bumper",
    )
    body.visual(
        Box((0.125, 0.016, 0.045)),
        origin=Origin(xyz=(0.018, 0.068, 0.089)),
        material=body_orange,
        name="left_cabin_side",
    )
    body.visual(
        Box((0.125, 0.016, 0.045)),
        origin=Origin(xyz=(0.018, -0.068, 0.089)),
        material=body_orange,
        name="right_cabin_side",
    )
    body.visual(
        Box((0.012, 0.016, 0.056)),
        origin=Origin(xyz=(0.082, 0.068, 0.133)),
        material=body_orange,
        name="left_front_pillar",
    )
    body.visual(
        Box((0.012, 0.016, 0.056)),
        origin=Origin(xyz=(0.082, -0.068, 0.133)),
        material=body_orange,
        name="right_front_pillar",
    )
    body.visual(
        Box((0.012, 0.016, 0.060)),
        origin=Origin(xyz=(-0.028, 0.068, 0.131)),
        material=body_orange,
        name="left_rear_pillar",
    )
    body.visual(
        Box((0.012, 0.016, 0.060)),
        origin=Origin(xyz=(-0.028, -0.068, 0.131)),
        material=body_orange,
        name="right_rear_pillar",
    )
    body.visual(
        Box((0.124, 0.136, 0.012)),
        origin=Origin(xyz=(0.027, 0.000, 0.162)),
        material=body_orange,
        name="roof_panel",
    )
    body.visual(
        Box((0.014, 0.136, 0.102)),
        origin=Origin(xyz=(-0.035, 0.000, 0.117)),
        material=body_orange,
        name="cab_rear_wall",
    )
    body.visual(
        Box((0.160, 0.016, 0.045)),
        origin=Origin(xyz=(-0.110, 0.068, 0.089)),
        material=body_orange,
        name="left_bed_side",
    )
    body.visual(
        Box((0.160, 0.016, 0.045)),
        origin=Origin(xyz=(-0.110, -0.068, 0.089)),
        material=body_orange,
        name="right_bed_side",
    )
    body.visual(
        Box((0.012, 0.132, 0.020)),
        origin=Origin(xyz=(-0.188, 0.000, 0.076)),
        material=trim_gray,
        name="rear_sill",
    )
    body.visual(
        Box((0.096, 0.032, 0.026)),
        origin=Origin(xyz=(0.110, 0.091, 0.105)),
        material=body_orange,
        name="front_left_fender",
    )
    body.visual(
        Box((0.096, 0.032, 0.026)),
        origin=Origin(xyz=(0.110, -0.091, 0.105)),
        material=body_orange,
        name="front_right_fender",
    )
    body.visual(
        Box((0.096, 0.032, 0.026)),
        origin=Origin(xyz=(-0.110, 0.091, 0.105)),
        material=body_orange,
        name="rear_left_fender",
    )
    body.visual(
        Box((0.096, 0.032, 0.026)),
        origin=Origin(xyz=(-0.110, -0.091, 0.105)),
        material=body_orange,
        name="rear_right_fender",
    )
    body.visual(
        Box((0.028, 0.016, 0.030)),
        origin=Origin(xyz=(0.110, 0.068, 0.048)),
        material=trim_gray,
        name="front_left_axle_support",
    )
    body.visual(
        Box((0.016, 0.011, 0.016)),
        origin=Origin(xyz=(0.110, 0.0815, 0.045)),
        material=trim_gray,
        name="front_left_axle_stub",
    )
    body.visual(
        Box((0.028, 0.016, 0.030)),
        origin=Origin(xyz=(0.110, -0.068, 0.048)),
        material=trim_gray,
        name="front_right_axle_support",
    )
    body.visual(
        Box((0.016, 0.011, 0.016)),
        origin=Origin(xyz=(0.110, -0.0815, 0.045)),
        material=trim_gray,
        name="front_right_axle_stub",
    )
    body.visual(
        Box((0.028, 0.016, 0.030)),
        origin=Origin(xyz=(-0.110, 0.068, 0.048)),
        material=trim_gray,
        name="rear_left_axle_support",
    )
    body.visual(
        Box((0.016, 0.011, 0.016)),
        origin=Origin(xyz=(-0.110, 0.0815, 0.045)),
        material=trim_gray,
        name="rear_left_axle_stub",
    )
    body.visual(
        Box((0.028, 0.016, 0.030)),
        origin=Origin(xyz=(-0.110, -0.068, 0.048)),
        material=trim_gray,
        name="rear_right_axle_support",
    )
    body.visual(
        Box((0.016, 0.011, 0.016)),
        origin=Origin(xyz=(-0.110, -0.0815, 0.045)),
        material=trim_gray,
        name="rear_right_axle_stub",
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.135, 0.118, 0.008)),
        origin=Origin(xyz=(0.0675, 0.000, 0.005)),
        material=body_orange,
        name="hood_skin",
    )
    hood.visual(
        Box((0.135, 0.008, 0.024)),
        origin=Origin(xyz=(0.0675, 0.056, 0.012)),
        material=body_orange,
        name="hood_left_skirt",
    )
    hood.visual(
        Box((0.135, 0.008, 0.024)),
        origin=Origin(xyz=(0.0675, -0.056, 0.012)),
        material=body_orange,
        name="hood_right_skirt",
    )
    hood.visual(
        Box((0.010, 0.118, 0.024)),
        origin=Origin(xyz=(0.130, 0.000, 0.012)),
        material=body_orange,
        name="hood_front_lip",
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.010, 0.118, 0.040)),
        origin=Origin(xyz=(-0.005, 0.000, 0.020)),
        material=body_orange,
        name="gate_panel",
    )
    tailgate.visual(
        Box((0.010, 0.118, 0.010)),
        origin=Origin(xyz=(-0.005, 0.000, 0.035)),
        material=trim_gray,
        name="gate_top_rail",
    )

    wheel_specs = [
        ("front_left_wheel", (0.110, 0.101, 0.045), "wheel_tire", "wheel_hub"),
        ("front_right_wheel", (0.110, -0.101, 0.045), "wheel_tire", "wheel_hub"),
        ("rear_left_wheel", (-0.110, 0.101, 0.045), "wheel_tire", "wheel_hub"),
        ("rear_right_wheel", (-0.110, -0.101, 0.045), "wheel_tire", "wheel_hub"),
    ]

    wheel_parts = {}
    for wheel_name, _, tire_name, hub_name in wheel_specs:
        wheel = model.part(wheel_name)
        wheel.visual(
            Cylinder(radius=0.043, length=0.028),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=tire_black,
            name=tire_name,
        )
        wheel.visual(
            Cylinder(radius=0.022, length=0.032),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=rim_gray,
            name=hub_name,
        )
        wheel_parts[wheel_name] = wheel

    model.articulation(
        "body_to_hood",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.040, 0.000, 0.109)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "body_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tailgate,
        origin=Origin(xyz=(-0.195, 0.000, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.30),
    )

    for wheel_name, wheel_xyz, _, _ in wheel_specs:
        model.articulation(
            f"body_to_{wheel_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_parts[wheel_name],
            origin=Origin(xyz=wheel_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
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

    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    tailgate = object_model.get_part("tailgate")
    front_left_wheel = object_model.get_part("front_left_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")

    hood_hinge = object_model.get_articulation("body_to_hood")
    tailgate_hinge = object_model.get_articulation("body_to_tailgate")
    wheel_joint_names = (
        "body_to_front_left_wheel",
        "body_to_front_right_wheel",
        "body_to_rear_left_wheel",
        "body_to_rear_right_wheel",
    )

    hood_skin = hood.get_visual("hood_skin")
    hood_front_lip = hood.get_visual("hood_front_lip")
    cowl_bridge = body.get_visual("cowl_bridge")
    rear_sill = body.get_visual("rear_sill")
    gate_panel = tailgate.get_visual("gate_panel")
    front_left_fender = body.get_visual("front_left_fender")
    rear_left_fender = body.get_visual("rear_left_fender")

    ctx.check(
        "hood hinge uses rear horizontal axis",
        hood_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={hood_hinge.axis}",
    )
    ctx.check(
        "tailgate hinge uses lower horizontal axis",
        tailgate_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={tailgate_hinge.axis}",
    )

    for joint_name in wheel_joint_names:
        wheel_joint = object_model.get_articulation(joint_name)
        limits = wheel_joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={wheel_joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name} spins around a wheel axle",
            abs(wheel_joint.axis[1]) > 0.99
            and abs(wheel_joint.axis[0]) < 1e-9
            and abs(wheel_joint.axis[2]) < 1e-9,
            details=f"axis={wheel_joint.axis}",
        )
        ctx.check(
            f"{joint_name} has no rotation stops",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits}",
        )

    with ctx.pose({hood_hinge: 0.0, tailgate_hinge: 0.0}):
        ctx.expect_overlap(
            hood,
            body,
            axes="xy",
            min_overlap=0.11,
            elem_a=hood_skin,
            name="hood covers the front bay when closed",
        )
        ctx.expect_gap(
            hood,
            body,
            axis="z",
            max_gap=0.010,
            max_penetration=0.0,
            positive_elem=hood_skin,
            negative_elem=cowl_bridge,
            name="closed hood clears the cowl",
        )
        ctx.expect_gap(
            body,
            tailgate,
            axis="x",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem=rear_sill,
            negative_elem=gate_panel,
            name="closed tailgate sits just behind the rear sill",
        )
        ctx.expect_within(
            tailgate,
            body,
            axes="y",
            margin=0.002,
            inner_elem=gate_panel,
            outer_elem=rear_sill,
            name="tailgate fits between the bed sides",
        )
        ctx.expect_overlap(
            front_left_wheel,
            body,
            axes="x",
            min_overlap=0.070,
            elem_b=front_left_fender,
            name="front left wheel sits under its fender in plan",
        )
        ctx.expect_gap(
            body,
            front_left_wheel,
            axis="z",
            max_gap=0.010,
            max_penetration=0.0,
            positive_elem=front_left_fender,
            name="front left fender clears the wheel",
        )
        ctx.expect_overlap(
            rear_left_wheel,
            body,
            axes="x",
            min_overlap=0.070,
            elem_b=rear_left_fender,
            name="rear left wheel sits under its fender in plan",
        )
        ctx.expect_gap(
            body,
            rear_left_wheel,
            axis="z",
            max_gap=0.010,
            max_penetration=0.0,
            positive_elem=rear_left_fender,
            name="rear left fender clears the wheel",
        )

    hood_closed = ctx.part_element_world_aabb(hood, elem=hood_front_lip)
    with ctx.pose({hood_hinge: 1.00}):
        hood_open = ctx.part_element_world_aabb(hood, elem=hood_front_lip)
    ctx.check(
        "hood opens upward",
        hood_closed is not None
        and hood_open is not None
        and hood_open[1][2] > hood_closed[1][2] + 0.045,
        details=f"closed={hood_closed}, open={hood_open}",
    )

    gate_closed = ctx.part_element_world_aabb(tailgate, elem=gate_panel)
    with ctx.pose({tailgate_hinge: 1.20}):
        gate_open = ctx.part_element_world_aabb(tailgate, elem=gate_panel)
    ctx.check(
        "tailgate folds downward and rearward",
        gate_closed is not None
        and gate_open is not None
        and gate_open[0][0] < gate_closed[0][0] - 0.020
        and gate_open[1][2] < gate_closed[1][2] - 0.015,
        details=f"closed={gate_closed}, open={gate_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
