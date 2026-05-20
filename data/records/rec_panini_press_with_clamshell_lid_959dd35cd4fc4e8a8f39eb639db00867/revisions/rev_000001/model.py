from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panini_press")

    housing_black = model.material("housing_black", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    grill_dark = model.material("grill_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    handle_black = model.material("handle_black", rgba=(0.12, 0.12, 0.13, 1.0))
    indicator_red = model.material("indicator_red", rgba=(0.73, 0.14, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.340, 0.295, 0.076)),
        origin=Origin(xyz=(0.000, 0.000, 0.042)),
        material=housing_black,
        name="housing_main",
    )
    base.visual(
        Box((0.230, 0.020, 0.012)),
        origin=Origin(xyz=(0.028, -0.120, 0.086)),
        material=trim_black,
        name="left_top_rail",
    )
    base.visual(
        Box((0.230, 0.020, 0.012)),
        origin=Origin(xyz=(0.028, 0.120, 0.086)),
        material=trim_black,
        name="right_top_rail",
    )
    base.visual(
        Box((0.020, 0.240, 0.012)),
        origin=Origin(xyz=(0.151, 0.000, 0.086)),
        material=trim_black,
        name="front_top_rail",
    )
    base.visual(
        Box((0.055, 0.026, 0.012)),
        origin=Origin(xyz=(-0.128, -0.123, 0.085)),
        material=housing_black,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.055, 0.026, 0.012)),
        origin=Origin(xyz=(-0.128, 0.123, 0.085)),
        material=housing_black,
        name="right_hinge_cheek",
    )
    base.visual(
        Box((0.255, 0.215, 0.010)),
        origin=Origin(xyz=(0.012, 0.000, 0.073)),
        material=grill_dark,
        name="lower_grill_plate",
    )
    ridge_y_positions = (-0.078, -0.046, -0.014, 0.018, 0.050, 0.082)
    for index, ridge_y in enumerate(ridge_y_positions):
        base.visual(
            Box((0.228, 0.018, 0.004)),
            origin=Origin(xyz=(0.012, ridge_y, 0.079)),
            material=grill_dark,
            name=f"lower_ridge_{index}",
        )
    base.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(
            xyz=(0.095, 0.144, 0.050),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="knob_mount_boss",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.085),
        origin=Origin(
            xyz=(-0.135, -0.093, 0.096),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.085),
        origin=Origin(
            xyz=(-0.135, 0.093, 0.096),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="right_hinge_barrel",
    )
    for index, (foot_x, foot_y) in enumerate(
        ((-0.120, -0.105), (-0.120, 0.105), (0.120, -0.105), (0.120, 0.105))
    ):
        base.visual(
            Box((0.048, 0.038, 0.004)),
            origin=Origin(xyz=(foot_x, foot_y, 0.002)),
            material=trim_black,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.340, 0.295, 0.096)),
        mass=5.2,
        origin=Origin(xyz=(0.000, 0.000, 0.048)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.280, 0.260, 0.038)),
        origin=Origin(xyz=(0.158, 0.000, 0.020)),
        material=housing_black,
        name="upper_shell",
    )
    lid.visual(
        Box((0.180, 0.230, 0.016)),
        origin=Origin(xyz=(0.095, 0.000, 0.046)),
        material=housing_black,
        name="rear_shell_crown",
    )
    lid.visual(
        Box((0.230, 0.215, 0.010)),
        origin=Origin(xyz=(0.147, 0.000, -0.008)),
        material=grill_dark,
        name="upper_grill_plate",
    )
    for index, ridge_y in enumerate(ridge_y_positions):
        lid.visual(
            Box((0.202, 0.016, 0.003)),
            origin=Origin(xyz=(0.147, ridge_y, -0.0125)),
            material=grill_dark,
            name=f"upper_ridge_{index}",
        )
    lid.visual(
        Box((0.236, 0.012, 0.010)),
        origin=Origin(xyz=(0.147, -0.108, -0.001)),
        material=trim_black,
        name="left_inner_wall",
    )
    lid.visual(
        Box((0.236, 0.012, 0.010)),
        origin=Origin(xyz=(0.147, 0.108, -0.001)),
        material=trim_black,
        name="right_inner_wall",
    )
    lid.visual(
        Box((0.016, 0.216, 0.012)),
        origin=Origin(xyz=(0.254, 0.000, -0.001)),
        material=trim_black,
        name="front_inner_wall",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.090),
        origin=Origin(
            xyz=(0.000, 0.000, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.024, 0.086, 0.016)),
        origin=Origin(xyz=(0.012, 0.000, 0.010)),
        material=trim_black,
        name="hinge_bridge",
    )
    lid.visual(
        Box((0.032, 0.016, 0.014)),
        origin=Origin(xyz=(0.287, -0.048, 0.010)),
        material=handle_black,
        name="left_handle_standoff",
    )
    lid.visual(
        Box((0.032, 0.016, 0.014)),
        origin=Origin(xyz=(0.287, 0.048, 0.010)),
        material=handle_black,
        name="right_handle_standoff",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(
            xyz=(0.305, 0.000, 0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_black,
        name="handle_bar",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.305, 0.260, 0.070)),
        mass=2.6,
        origin=Origin(xyz=(0.152, 0.000, 0.020)),
    )

    knob = model.part("temperature_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=handle_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.010, 0.007)),
        origin=Origin(xyz=(0.000, 0.012, 0.0185)),
        material=indicator_red,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.020),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(-0.135, 0.000, 0.096)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.25,
        ),
    )
    knob_joint = model.articulation(
        "temperature_knob_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=knob,
        origin=Origin(
            xyz=(0.095, 0.1475, 0.050),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=5.0,
            lower=-2.35,
            upper=2.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("temperature_knob")
    lid_hinge = object_model.get_articulation("lid_hinge")
    knob_joint = object_model.get_articulation("temperature_knob_joint")

    lid_limits = lid_hinge.motion_limits
    knob_limits = knob_joint.motion_limits

    if lid_limits is not None and lid_limits.lower is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.lower}):
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem="upper_grill_plate",
                negative_elem="lower_grill_plate",
                min_gap=0.004,
                max_gap=0.012,
                name="closed lid leaves a shallow cooking gap",
            )
            ctx.expect_overlap(
                lid,
                base,
                axes="xy",
                elem_a="upper_grill_plate",
                elem_b="lower_grill_plate",
                min_overlap=0.180,
                name="upper and lower grill plates align in plan",
            )
            handle_closed = ctx.part_element_world_aabb(lid, elem="handle_bar")

        with ctx.pose({lid_hinge: lid_limits.upper}):
            handle_open = ctx.part_element_world_aabb(lid, elem="handle_bar")

        ctx.check(
            "lid handle rises when opened",
            handle_closed is not None
            and handle_open is not None
            and handle_open[0][2] > handle_closed[1][2] + 0.12,
            details=f"closed={handle_closed}, open={handle_open}",
        )

    ctx.expect_gap(
        knob,
        base,
        axis="y",
        positive_elem="knob_body",
        negative_elem="housing_main",
        max_gap=0.002,
        max_penetration=0.0,
        name="temperature knob is flush-mounted on the side wall",
    )
    ctx.expect_origin_gap(
        knob,
        base,
        axis="y",
        min_gap=0.140,
        name="temperature knob sits on the right side of the press",
    )
    ctx.check(
        "temperature knob has a broad adjustment sweep",
        knob_limits is not None
        and knob_limits.lower is not None
        and knob_limits.upper is not None
        and (knob_limits.upper - knob_limits.lower) >= 4.5,
        details=f"limits={knob_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
