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
    model = ArticulatedObject(name="toy_pickup_truck")

    body_red = model.material("body_red", rgba=(0.80, 0.14, 0.10, 1.0))
    window_tint = model.material("window_tint", rgba=(0.35, 0.55, 0.85, 0.45))
    tire_black = model.material("tire_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    liner_black = model.material("liner_black", rgba=(0.16, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.31, 0.14, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=body_red,
        name="chassis",
    )
    body.visual(
        Box((0.23, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.062, 0.068)),
        material=body_red,
        name="left_rocker",
    )
    body.visual(
        Box((0.23, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, -0.062, 0.068)),
        material=body_red,
        name="right_rocker",
    )
    body.visual(
        Box((0.14, 0.12, 0.008)),
        origin=Origin(xyz=(-0.075, 0.0, 0.084)),
        material=liner_black,
        name="bed_floor",
    )
    body.visual(
        Box((0.14, 0.012, 0.052)),
        origin=Origin(xyz=(-0.075, 0.064, 0.11)),
        material=body_red,
        name="bed_left_wall",
    )
    body.visual(
        Box((0.14, 0.012, 0.052)),
        origin=Origin(xyz=(-0.075, -0.064, 0.11)),
        material=body_red,
        name="bed_right_wall",
    )
    body.visual(
        Box((0.012, 0.12, 0.052)),
        origin=Origin(xyz=(-0.005, 0.0, 0.11)),
        material=body_red,
        name="bed_front_wall",
    )
    body.visual(
        Box((0.10, 0.016, 0.05)),
        origin=Origin(xyz=(-0.095, 0.062, 0.105)),
        material=body_red,
        name="left_rear_quarter",
    )
    body.visual(
        Box((0.10, 0.016, 0.05)),
        origin=Origin(xyz=(-0.095, -0.062, 0.105)),
        material=body_red,
        name="right_rear_quarter",
    )
    body.visual(
        Box((0.10, 0.14, 0.045)),
        origin=Origin(xyz=(0.02, 0.0, 0.1025)),
        material=body_red,
        name="cab_body",
    )
    body.visual(
        Box((0.012, 0.14, 0.06)),
        origin=Origin(xyz=(-0.024, 0.0, 0.13)),
        material=body_red,
        name="cab_rear_wall",
    )
    body.visual(
        Box((0.09, 0.14, 0.018)),
        origin=Origin(xyz=(0.021, 0.0, 0.167)),
        material=body_red,
        name="roof",
    )
    body.visual(
        Box((0.014, 0.132, 0.048)),
        origin=Origin(xyz=(0.05, 0.0, 0.147), rpy=(0.0, -0.52, 0.0)),
        material=window_tint,
        name="windshield",
    )
    body.visual(
        Box((0.062, 0.006, 0.034)),
        origin=Origin(xyz=(0.012, 0.067, 0.145)),
        material=window_tint,
        name="left_side_window",
    )
    body.visual(
        Box((0.062, 0.006, 0.034)),
        origin=Origin(xyz=(0.012, -0.067, 0.145)),
        material=window_tint,
        name="right_side_window",
    )
    body.visual(
        Box((0.006, 0.082, 0.03)),
        origin=Origin(xyz=(-0.024, 0.0, 0.144)),
        material=window_tint,
        name="rear_window",
    )
    body.visual(
        Box((0.02, 0.14, 0.012)),
        origin=Origin(xyz=(0.074, 0.0, 0.126)),
        material=body_red,
        name="cowl",
    )
    body.visual(
        Box((0.094, 0.018, 0.05)),
        origin=Origin(xyz=(0.103, 0.061, 0.1)),
        material=body_red,
        name="left_front_fender",
    )
    body.visual(
        Box((0.094, 0.018, 0.05)),
        origin=Origin(xyz=(0.103, -0.061, 0.1)),
        material=body_red,
        name="right_front_fender",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(0.1, 0.07, 0.05), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="front_left_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(0.1, -0.07, 0.05), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="front_right_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(-0.1, 0.07, 0.05), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rear_left_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.02),
        origin=Origin(xyz=(-0.1, -0.07, 0.05), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="rear_right_axle_stub",
    )
    body.visual(
        Box((0.09, 0.118, 0.02)),
        origin=Origin(xyz=(0.125, 0.0, 0.09)),
        material=liner_black,
        name="engine_bay",
    )
    body.visual(
        Box((0.05, 0.14, 0.018)),
        origin=Origin(xyz=(0.136, 0.0, 0.112)),
        material=body_red,
        name="nose_top",
    )
    body.visual(
        Box((0.02, 0.14, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, 0.07)),
        material=hub_gray,
        name="front_grille",
    )

    hood = model.part("hood")
    hood.visual(
        Box((0.092, 0.14, 0.018)),
        origin=Origin(xyz=(0.046, 0.0, 0.009)),
        material=body_red,
        name="hood_panel",
    )
    hood.visual(
        Box((0.092, 0.006, 0.012)),
        origin=Origin(xyz=(0.046, 0.067, 0.006)),
        material=body_red,
        name="hood_left_lip",
    )
    hood.visual(
        Box((0.092, 0.006, 0.012)),
        origin=Origin(xyz=(0.046, -0.067, 0.006)),
        material=body_red,
        name="hood_right_lip",
    )
    hood.visual(
        Box((0.012, 0.14, 0.016)),
        origin=Origin(xyz=(0.086, 0.0, 0.008)),
        material=body_red,
        name="hood_front_edge",
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((0.012, 0.12, 0.05)),
        origin=Origin(xyz=(-0.006, 0.0, 0.025)),
        material=body_red,
        name="tailgate_panel",
    )

    def add_wheel(part_name: str) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.05, length=0.03),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=tire_black,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.032, length=0.031),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=hub_gray,
            name="hub",
        )

    add_wheel("front_left_wheel")
    add_wheel("front_right_wheel")
    add_wheel("rear_left_wheel")
    add_wheel("rear_right_wheel")

    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.084, 0.0, 0.132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tailgate,
        origin=Origin(xyz=(-0.145, 0.0, 0.088)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_front_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="front_left_wheel",
        origin=Origin(xyz=(0.1, 0.095, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "body_to_front_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="front_right_wheel",
        origin=Origin(xyz=(0.1, -0.095, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "body_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="rear_left_wheel",
        origin=Origin(xyz=(-0.1, 0.095, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "body_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child="rear_right_wheel",
        origin=Origin(xyz=(-0.1, -0.095, 0.05)),
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
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    hood_hinge = object_model.get_articulation("hood_hinge")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")
    wheel_joints = (
        object_model.get_articulation("body_to_front_left_wheel"),
        object_model.get_articulation("body_to_front_right_wheel"),
        object_model.get_articulation("body_to_rear_left_wheel"),
        object_model.get_articulation("body_to_rear_right_wheel"),
    )

    ctx.check(
        "all major parts exist",
        all(
            part is not None
            for part in (
                body,
                hood,
                tailgate,
                front_left_wheel,
                front_right_wheel,
                rear_left_wheel,
                rear_right_wheel,
            )
        ),
    )
    ctx.check(
        "hood hinge is an upward-opening revolute joint",
        hood_hinge.articulation_type == ArticulationType.REVOLUTE
        and hood_hinge.axis == (0.0, -1.0, 0.0)
        and hood_hinge.motion_limits is not None
        and hood_hinge.motion_limits.lower == 0.0
        and hood_hinge.motion_limits.upper is not None
        and hood_hinge.motion_limits.upper >= 1.0,
        details=str(
            {
                "type": hood_hinge.articulation_type,
                "axis": hood_hinge.axis,
                "limits": hood_hinge.motion_limits,
            }
        ),
    )
    ctx.check(
        "tailgate hinge is a lower rear revolute joint",
        tailgate_hinge.articulation_type == ArticulationType.REVOLUTE
        and tailgate_hinge.axis == (0.0, -1.0, 0.0)
        and tailgate_hinge.motion_limits is not None
        and tailgate_hinge.motion_limits.lower == 0.0
        and tailgate_hinge.motion_limits.upper is not None
        and tailgate_hinge.motion_limits.upper >= 1.2,
        details=str(
            {
                "type": tailgate_hinge.articulation_type,
                "axis": tailgate_hinge.axis,
                "limits": tailgate_hinge.motion_limits,
            }
        ),
    )
    ctx.check(
        "all wheel joints are continuous axle joints",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in wheel_joints
        ),
        details=str(
            [
                {
                    "name": joint.name,
                    "type": joint.articulation_type,
                    "axis": joint.axis,
                    "limits": joint.motion_limits,
                }
                for joint in wheel_joints
            ]
        ),
    )

    with ctx.pose({hood_hinge: 0.0}):
        ctx.expect_gap(
            hood,
            body,
            axis="z",
            positive_elem="hood_panel",
            negative_elem="cowl",
            min_gap=0.0,
            max_gap=0.02,
            name="hood rests just above the cowl when closed",
        )
        ctx.expect_overlap(
            hood,
            body,
            axes="xy",
            elem_a="hood_panel",
            elem_b="engine_bay",
            min_overlap=0.08,
            name="hood covers the engine bay footprint when closed",
        )

    hood_closed = ctx.part_element_world_aabb(hood, elem="hood_panel")
    with ctx.pose({hood_hinge: 1.0}):
        hood_open = ctx.part_element_world_aabb(hood, elem="hood_panel")
    ctx.check(
        "hood opens upward",
        hood_closed is not None
        and hood_open is not None
        and hood_open[1][2] > hood_closed[1][2] + 0.04,
        details=f"closed={hood_closed}, open={hood_open}",
    )

    tailgate_closed = ctx.part_element_world_aabb(tailgate, elem="tailgate_panel")
    with ctx.pose({tailgate_hinge: 1.2}):
        tailgate_open = ctx.part_element_world_aabb(tailgate, elem="tailgate_panel")
    ctx.check(
        "tailgate folds downward",
        tailgate_closed is not None
        and tailgate_open is not None
        and tailgate_open[1][2] < tailgate_closed[1][2] - 0.03
        and tailgate_open[0][0] < tailgate_closed[0][0] - 0.01,
        details=f"closed={tailgate_closed}, open={tailgate_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
