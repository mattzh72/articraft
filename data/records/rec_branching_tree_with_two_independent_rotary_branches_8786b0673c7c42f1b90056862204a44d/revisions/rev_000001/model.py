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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_branch_positioning_fixture")

    painted_steel = model.material("painted_steel", rgba=(0.24, 0.29, 0.34, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.78, 0.55, 0.26, 1.0))
    arm_blue = model.material("arm_blue", rgba=(0.15, 0.32, 0.60, 1.0))
    pad_urethane = model.material("pad_urethane", rgba=(0.05, 0.05, 0.045, 1.0))

    hub_x = 0.115
    upper_z = 0.62
    lower_z = 0.32
    bearing_y = 0.080
    bearing_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.023, tube=0.006, radial_segments=24, tubular_segments=48).rotate_x(
            math.pi / 2.0
        ),
        "bronze_bearing_ring",
    )

    spine = model.part("spine")
    spine.visual(
        Box((0.34, 0.26, 0.028)),
        origin=Origin(xyz=(0.035, 0.0, 0.014)),
        material=dark_steel,
        name="floor_plate",
    )
    spine.visual(
        Box((0.070, 0.220, 0.760)),
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        material=painted_steel,
        name="central_spine",
    )
    spine.visual(
        Box((0.110, 0.200, 0.030)),
        origin=Origin(xyz=(0.030, 0.0, 0.780)),
        material=dark_steel,
        name="top_cap",
    )

    def add_supported_bearing_set(prefix: str, z: float) -> None:
        spine.visual(
            Box((0.055, 0.190, 0.085)),
            origin=Origin(xyz=(0.050, 0.0, z)),
            material=painted_steel,
            name=f"{prefix}_spine_boss",
        )
        for index, y in enumerate((-bearing_y, bearing_y)):
            spine.visual(
                Box((0.050, 0.020, 0.070)),
                origin=Origin(xyz=(0.058, y, z)),
                material=painted_steel,
                name=f"{prefix}_support_rib_{index}",
            )
            spine.visual(
                Box((0.110, 0.020, 0.016)),
                origin=Origin(xyz=(hub_x, y, z + 0.032)),
                material=dark_steel,
                name=f"{prefix}_cheek_top_{index}",
            )
            spine.visual(
                Box((0.110, 0.020, 0.016)),
                origin=Origin(xyz=(hub_x, y, z - 0.032)),
                material=dark_steel,
                name=f"{prefix}_cheek_bottom_{index}",
            )
            spine.visual(
                Box((0.016, 0.020, 0.088)),
                origin=Origin(xyz=(hub_x - 0.034, y, z)),
                material=dark_steel,
                name=f"{prefix}_cheek_inner_{index}",
            )
            spine.visual(
                Box((0.016, 0.020, 0.088)),
                origin=Origin(xyz=(hub_x + 0.034, y, z)),
                material=dark_steel,
                name=f"{prefix}_cheek_outer_{index}",
            )
            spine.visual(
                bearing_ring_mesh,
                origin=Origin(xyz=(hub_x, y, z)),
                material=bearing_bronze,
                name=f"{prefix}_bearing_ring_{index}",
            )

    add_supported_bearing_set("upper", upper_z)
    add_supported_bearing_set("lower", lower_z)

    def add_branch(name: str, arm_length: float, pad_x: float, pad_z: float) -> None:
        branch = model.part(name)
        branch.visual(
            Cylinder(radius=0.017, length=0.215),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="shaft",
        )
        branch.visual(
            Cylinder(radius=0.032, length=0.060),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=painted_steel,
            name="hub_barrel",
        )
        branch.visual(
            Box((arm_length, 0.040, 0.034)),
            origin=Origin(xyz=(arm_length / 2.0 + 0.030, 0.0, 0.0)),
            material=arm_blue,
            name="branch_beam",
        )
        branch.visual(
            Box((0.060, 0.050, 0.050)),
            origin=Origin(xyz=(pad_x - 0.028, 0.0, pad_z + 0.012)),
            material=arm_blue,
            name="pad_neck",
        )
        branch.visual(
            Cylinder(radius=0.042, length=0.016),
            origin=Origin(xyz=(pad_x, 0.0, pad_z), rpy=(0.0, 0.0, 0.0)),
            material=pad_urethane,
            name="tooling_pad",
        )

    add_branch("upper_branch", arm_length=0.330, pad_x=0.405, pad_z=-0.040)
    add_branch("lower_branch", arm_length=0.285, pad_x=0.360, pad_z=-0.040)

    model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=spine,
        child="upper_branch",
        origin=Origin(xyz=(hub_x, 0.0, upper_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=math.radians(-75.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=spine,
        child="lower_branch",
        origin=Origin(xyz=(hub_x, 0.0, lower_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=math.radians(-75.0),
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    spine = object_model.get_part("spine")
    upper_branch = object_model.get_part("upper_branch")
    lower_branch = object_model.get_part("lower_branch")
    upper_pivot = object_model.get_articulation("upper_pivot")
    lower_pivot = object_model.get_articulation("lower_pivot")

    ctx.check(
        "pivots are independent parallel revolutes",
        upper_pivot.articulation_type == ArticulationType.REVOLUTE
        and lower_pivot.articulation_type == ArticulationType.REVOLUTE
        and upper_pivot.child == "upper_branch"
        and lower_pivot.child == "lower_branch"
        and tuple(upper_pivot.axis) == tuple(lower_pivot.axis),
    )

    for branch, prefix in ((upper_branch, "upper"), (lower_branch, "lower")):
        for index in range(2):
            ring_name = f"{prefix}_bearing_ring_{index}"
            ctx.allow_overlap(
                branch,
                spine,
                elem_a="shaft",
                elem_b=ring_name,
                reason=(
                    "The rotating shaft is intentionally seated through the bronze bearing ring "
                    "with a tiny modeled interference so the hub reads as mechanically captured."
                ),
            )
            ctx.expect_within(
                branch,
                spine,
                axes="xz",
                inner_elem="shaft",
                outer_elem=ring_name,
                margin=0.001,
                name=f"{prefix} shaft centered in bearing {index}",
            )
            ctx.expect_overlap(
                branch,
                spine,
                axes="y",
                elem_a="shaft",
                elem_b=ring_name,
                min_overlap=0.010,
                name=f"{prefix} shaft passes through bearing {index}",
            )

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    upper_pad_rest = element_center(upper_branch, "tooling_pad")
    lower_pad_rest = element_center(lower_branch, "tooling_pad")
    with ctx.pose({upper_pivot: math.radians(45.0)}):
        upper_pad_moved = element_center(upper_branch, "tooling_pad")
        lower_pad_under_upper_motion = element_center(lower_branch, "tooling_pad")

    ctx.check(
        "upper pivot moves only upper pad",
        upper_pad_rest is not None
        and upper_pad_moved is not None
        and lower_pad_rest is not None
        and lower_pad_under_upper_motion is not None
        and abs(upper_pad_moved[2] - upper_pad_rest[2]) > 0.080
        and abs(lower_pad_under_upper_motion[2] - lower_pad_rest[2]) < 0.001,
        details=f"upper_rest={upper_pad_rest}, upper_moved={upper_pad_moved}, lower={lower_pad_under_upper_motion}",
    )

    with ctx.pose({lower_pivot: math.radians(-40.0)}):
        lower_pad_moved = element_center(lower_branch, "tooling_pad")
        upper_pad_under_lower_motion = element_center(upper_branch, "tooling_pad")

    ctx.check(
        "lower pivot moves only lower pad",
        lower_pad_rest is not None
        and lower_pad_moved is not None
        and upper_pad_rest is not None
        and upper_pad_under_lower_motion is not None
        and abs(lower_pad_moved[2] - lower_pad_rest[2]) > 0.070
        and abs(upper_pad_under_lower_motion[2] - upper_pad_rest[2]) < 0.001,
        details=f"lower_rest={lower_pad_rest}, lower_moved={lower_pad_moved}, upper={upper_pad_under_lower_motion}",
    )

    return ctx.report()


object_model = build_object_model()
