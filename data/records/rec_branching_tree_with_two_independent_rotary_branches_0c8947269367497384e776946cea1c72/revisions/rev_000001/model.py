from __future__ import annotations

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
    model = ArticulatedObject(name="pedestal_tooling_tree")

    cast_iron = model.material("cast_iron", color=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", color=(0.62, 0.66, 0.67, 1.0))
    clamp_black = model.material("clamp_black", color=(0.035, 0.037, 0.040, 1.0))
    painted_arm = model.material("painted_arm", color=(0.93, 0.52, 0.08, 1.0))
    plate_blue = model.material("plate_blue", color=(0.10, 0.22, 0.42, 1.0))
    screw_dark = model.material("screw_dark", color=(0.01, 0.01, 0.012, 1.0))

    pedestal = model.part("pedestal")

    # A broad, low foot gives the freestanding tooling tree credible mass.
    pedestal.visual(
        Cylinder(radius=0.28, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=cast_iron,
        name="round_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.105, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0925)),
        material=cast_iron,
        name="base_boss",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=1.08),
        origin=Origin(xyz=(0.0, 0.0, 0.655)),
        material=brushed_steel,
        name="vertical_column",
    )

    clamp_size = (0.16, 0.14, 0.10)
    shelf_size = (0.08, 0.11, 0.025)
    block_zs = {"lower": 0.55, "upper": 0.90}

    for prefix, block_z, side in (
        ("lower", block_zs["lower"], -1.0),
        ("upper", block_zs["upper"], 1.0),
    ):
        pedestal.visual(
            Box(clamp_size),
            origin=Origin(xyz=(0.0, 0.0, block_z)),
            material=clamp_black,
            name=f"{prefix}_clamp",
        )
        # A narrow split line and bolt heads make each block read as a real clamp,
        # not as a generic cube slipped over the column.
        pedestal.visual(
            Box((0.006, 0.006, 0.092)),
            origin=Origin(xyz=(0.0, -0.073, block_z)),
            material=screw_dark,
            name=f"{prefix}_split_line",
        )
        for ix, x in enumerate((-0.052, 0.052)):
            for iz, zoff in enumerate((-0.026, 0.026)):
                pedestal.visual(
                    Cylinder(radius=0.009, length=0.010),
                    origin=Origin(
                        xyz=(x, -0.075, block_z + zoff),
                        rpy=(pi / 2.0, 0.0, 0.0),
                    ),
                    material=screw_dark,
                    name=f"{prefix}_bolt_{ix}_{iz}",
                )

        joint_z = block_z + 0.075
        shelf_x = side * 0.11
        pedestal.visual(
            Box(shelf_size),
            origin=Origin(xyz=(shelf_x, 0.0, joint_z - 0.040)),
            material=clamp_black,
            name=f"{prefix}_thrust_shelf",
        )
        pedestal.visual(
            Cylinder(radius=0.043, length=0.010),
            origin=Origin(xyz=(shelf_x, 0.0, joint_z - 0.0225)),
            material=brushed_steel,
            name="lower_thrust_washer" if prefix == "lower" else "upper_thrust_washer",
        )

    def add_arm(part_name: str) -> None:
        arm = model.part(part_name)
        arm.visual(
            Cylinder(radius=0.040, length=0.035),
            origin=Origin(),
            material=brushed_steel,
            name="hub",
        )
        arm.visual(
            Box((0.50, 0.036, 0.036)),
            origin=Origin(xyz=(0.27, 0.0, 0.0)),
            material=painted_arm,
            name="arm_beam",
        )
        arm.visual(
            Box((0.22, 0.020, 0.026)),
            origin=Origin(xyz=(0.19, 0.0, 0.027)),
            material=painted_arm,
            name="top_rib",
        )
        arm.visual(
            Box((0.025, 0.12, 0.085)),
            origin=Origin(xyz=(0.505, 0.0, 0.0)),
            material=plate_blue,
            name="faceplate",
        )
        for iy, y in enumerate((-0.040, 0.040)):
            for iz, z in enumerate((-0.027, 0.027)):
                arm.visual(
                    Cylinder(radius=0.0065, length=0.006),
                    origin=Origin(
                        xyz=(0.520, y, z),
                        rpy=(0.0, pi / 2.0, 0.0),
                    ),
                    material=screw_dark,
                    name=f"face_screw_{iy}_{iz}",
                )

    add_arm("lower_arm")
    add_arm("upper_arm")

    model.articulation(
        "lower_pivot",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child="lower_arm",
        origin=Origin(xyz=(-0.11, 0.0, block_zs["lower"] + 0.075), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-2.25, upper=2.25),
    )
    model.articulation(
        "upper_pivot",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child="upper_arm",
        origin=Origin(xyz=(0.11, 0.0, block_zs["upper"] + 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-2.25, upper=2.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lower_pivot = object_model.get_articulation("lower_pivot")
    upper_pivot = object_model.get_articulation("upper_pivot")

    ctx.check(
        "two independent branch pivots",
        lower_pivot.parent == "pedestal"
        and upper_pivot.parent == "pedestal"
        and lower_pivot.child == "lower_arm"
        and upper_pivot.child == "upper_arm"
        and lower_pivot.mimic is None
        and upper_pivot.mimic is None,
        details=f"lower={lower_pivot.parent}->{lower_pivot.child}, upper={upper_pivot.parent}->{upper_pivot.child}",
    )
    ctx.check(
        "rotary arms use vertical revolute axes",
        lower_pivot.articulation_type == ArticulationType.REVOLUTE
        and upper_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(lower_pivot.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_pivot.axis) == (0.0, 0.0, 1.0),
        details=f"lower_axis={lower_pivot.axis}, upper_axis={upper_pivot.axis}",
    )

    ctx.expect_gap(
        lower_arm,
        pedestal,
        axis="z",
        positive_elem="hub",
        negative_elem="lower_thrust_washer",
        max_gap=0.001,
        max_penetration=0.0,
        name="lower hub rests on its clamp washer",
    )
    ctx.expect_gap(
        upper_arm,
        pedestal,
        axis="z",
        positive_elem="hub",
        negative_elem="upper_thrust_washer",
        max_gap=0.001,
        max_penetration=0.0,
        name="upper hub rests on its clamp washer",
    )
    ctx.expect_overlap(
        lower_arm,
        pedestal,
        axes="xy",
        elem_a="hub",
        elem_b="lower_thrust_washer",
        min_overlap=0.065,
        name="lower hub is centered over its support block",
    )
    ctx.expect_overlap(
        upper_arm,
        pedestal,
        axes="xy",
        elem_a="hub",
        elem_b="upper_thrust_washer",
        min_overlap=0.065,
        name="upper hub is centered over its support block",
    )

    def faceplate_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    lower_rest = faceplate_center(lower_arm, "faceplate")
    upper_rest = faceplate_center(upper_arm, "faceplate")
    with ctx.pose({upper_pivot: 0.65}):
        upper_moved = faceplate_center(upper_arm, "faceplate")
        lower_still = faceplate_center(lower_arm, "faceplate")
    with ctx.pose({lower_pivot: 0.65}):
        lower_moved = faceplate_center(lower_arm, "faceplate")
        upper_still = faceplate_center(upper_arm, "faceplate")

    ctx.check(
        "upper pivot moves only the upper faceplate",
        upper_rest is not None
        and upper_moved is not None
        and lower_rest is not None
        and lower_still is not None
        and abs(upper_moved[1] - upper_rest[1]) > 0.20
        and abs(lower_still[1] - lower_rest[1]) < 0.001,
        details=f"upper_rest={upper_rest}, upper_moved={upper_moved}, lower_rest={lower_rest}, lower_still={lower_still}",
    )
    ctx.check(
        "lower pivot moves only the lower faceplate",
        lower_rest is not None
        and lower_moved is not None
        and upper_rest is not None
        and upper_still is not None
        and abs(lower_moved[1] - lower_rest[1]) > 0.20
        and abs(upper_still[1] - upper_rest[1]) < 0.001,
        details=f"lower_rest={lower_rest}, lower_moved={lower_moved}, upper_rest={upper_rest}, upper_still={upper_still}",
    )

    return ctx.report()


object_model = build_object_model()
