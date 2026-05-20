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
    model = ArticulatedObject(name="telescopic_range_hood")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.24, 1.0))
    black = model.material("dial_black", rgba=(0.08, 0.08, 0.09, 1.0))
    mark = model.material("dial_mark", rgba=(0.86, 0.18, 0.12, 1.0))

    width = 0.600
    depth = 0.290
    height = 0.180
    wall = 0.012

    dial_x = 0.200
    dial_z = 0.138
    shaft_opening_w = 0.020
    shaft_opening_h = 0.020
    front_panel_bottom = 0.110

    housing = model.part("outer_housing")

    housing.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall / 2.0)),
        material=stainless,
        name="top_shell",
    )
    housing.visual(
        Box((wall, depth, height - wall)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, (height - wall) / 2.0)),
        material=stainless,
        name="left_wall",
    )
    housing.visual(
        Box((wall, depth, height - wall)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, (height - wall) / 2.0)),
        material=stainless,
        name="right_wall",
    )
    housing.visual(
        Box((width - 2.0 * wall, wall, height - wall)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, (height - wall) / 2.0)),
        material=stainless,
        name="back_wall",
    )

    opening_bottom = dial_z - shaft_opening_h / 2.0
    opening_top = dial_z + shaft_opening_h / 2.0
    opening_left = dial_x - shaft_opening_w / 2.0
    opening_right = dial_x + shaft_opening_w / 2.0

    housing.visual(
        Box((width, wall, height - opening_top)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2.0 - wall / 2.0,
                (opening_top + height) / 2.0,
            )
        ),
        material=stainless,
        name="control_panel_top_strip",
    )
    housing.visual(
        Box((width, wall, opening_bottom - front_panel_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2.0 - wall / 2.0,
                (front_panel_bottom + opening_bottom) / 2.0,
            )
        ),
        material=stainless,
        name="control_panel_bottom_strip",
    )
    housing.visual(
        Box((opening_left + width / 2.0, wall, shaft_opening_h)),
        origin=Origin(
            xyz=(
                (-width / 2.0 + opening_left) / 2.0,
                depth / 2.0 - wall / 2.0,
                dial_z,
            )
        ),
        material=stainless,
        name="control_panel_left_strip",
    )
    housing.visual(
        Box((width / 2.0 - opening_right, wall, shaft_opening_h)),
        origin=Origin(
            xyz=(
                (opening_right + width / 2.0) / 2.0,
                depth / 2.0 - wall / 2.0,
                dial_z,
            )
        ),
        material=stainless,
        name="control_panel_right_strip",
    )

    housing.visual(
        Box((0.330, 0.150, 0.072)),
        origin=Origin(xyz=(0.0, -0.068, 0.132)),
        material=brushed_steel,
        name="blower_plenum",
    )
    housing.visual(
        Cylinder(radius=0.055, length=0.060),
        origin=Origin(xyz=(0.0, -0.070, height + 0.030)),
        material=brushed_steel,
        name="exhaust_collar",
    )
    housing.visual(
        Box((0.008, 0.220, 0.010)),
        origin=Origin(xyz=(-0.284, -0.020, 0.087)),
        material=brushed_steel,
        name="left_guide",
    )
    housing.visual(
        Box((0.008, 0.220, 0.010)),
        origin=Origin(xyz=(0.284, -0.020, 0.087)),
        material=brushed_steel,
        name="right_guide",
    )
    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height + 0.060)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, (height + 0.060) / 2.0)),
    )

    tray = model.part("intake_tray")
    tray.visual(
        Box((0.568, 0.015, 0.060)),
        origin=Origin(xyz=(0.0, 0.0425, 0.065)),
        material=stainless,
        name="tray_fascia",
    )
    tray.visual(
        Box((0.320, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.050, 0.040)),
        material=brushed_steel,
        name="tray_handle",
    )
    tray.visual(
        Box((0.014, 0.256, 0.074)),
        origin=Origin(xyz=(-0.272, -0.092, 0.037)),
        material=stainless,
        name="left_side_rail",
    )
    tray.visual(
        Box((0.014, 0.256, 0.074)),
        origin=Origin(xyz=(0.272, -0.092, 0.037)),
        material=stainless,
        name="right_side_rail",
    )
    tray.visual(
        Box((0.008, 0.238, 0.008)),
        origin=Origin(xyz=(-0.283, -0.091, 0.078)),
        material=brushed_steel,
        name="left_runner",
    )
    tray.visual(
        Box((0.008, 0.238, 0.008)),
        origin=Origin(xyz=(0.283, -0.091, 0.078)),
        material=brushed_steel,
        name="right_runner",
    )
    tray.visual(
        Box((0.530, 0.014, 0.074)),
        origin=Origin(xyz=(0.0, -0.213, 0.037)),
        material=stainless,
        name="rear_bridge",
    )
    tray.visual(
        Box((0.530, 0.201, 0.008)),
        origin=Origin(xyz=(0.0, -0.0645, 0.010)),
        material=stainless,
        name="tray_bottom_panel",
    )
    tray.visual(
        Box((0.156, 0.158, 0.002)),
        origin=Origin(xyz=(-0.174, -0.077, 0.005)),
        material=charcoal,
        name="left_filter_panel",
    )
    tray.visual(
        Box((0.156, 0.158, 0.002)),
        origin=Origin(xyz=(0.0, -0.077, 0.005)),
        material=charcoal,
        name="center_filter_panel",
    )
    tray.visual(
        Box((0.156, 0.158, 0.002)),
        origin=Origin(xyz=(0.174, -0.077, 0.005)),
        material=charcoal,
        name="right_filter_panel",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.572, 0.270, 0.095)),
        mass=2.3,
        origin=Origin(xyz=(0.0, -0.085, 0.0475)),
    )

    model.articulation(
        "housing_to_intake_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tray,
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.170,
        ),
    )

    dial = model.part("fan_dial")
    dial.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.000, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="dial_shaft",
    )
    dial.visual(
        Box((0.050, 0.002, 0.050)),
        origin=Origin(xyz=(0.0, 0.001, 0.0)),
        material=brushed_steel,
        name="dial_rosette",
    )
    dial.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_skirt",
    )
    dial.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="dial_body",
    )
    dial.visual(
        Box((0.006, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, 0.028, 0.020)),
        material=mark,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.062, 0.034, 0.062)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
    )

    model.articulation(
        "housing_to_fan_dial",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=dial,
        origin=Origin(xyz=(dial_x, depth / 2.0, dial_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-2.4,
            upper=2.4,
        ),
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

    housing = object_model.get_part("outer_housing")
    tray = object_model.get_part("intake_tray")
    dial = object_model.get_part("fan_dial")
    tray_slide = object_model.get_articulation("housing_to_intake_tray")
    dial_joint = object_model.get_articulation("housing_to_fan_dial")

    tray_extension = tray_slide.motion_limits.upper if tray_slide.motion_limits is not None else 0.0

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_gap(
            housing,
            tray,
            axis="z",
            positive_elem="control_panel_bottom_strip",
            negative_elem="tray_fascia",
            min_gap=0.010,
            max_gap=0.040,
            name="tray fascia sits below the control strip",
        )
        ctx.expect_contact(
            tray,
            housing,
            elem_a="left_runner",
            elem_b="left_guide",
            name="left runner seats on the left guide",
        )
        ctx.expect_contact(
            tray,
            housing,
            elem_a="right_runner",
            elem_b="right_guide",
            name="right runner seats on the right guide",
        )
        ctx.expect_overlap(
            tray,
            housing,
            axes="y",
            elem_a="left_runner",
            elem_b="left_guide",
            min_overlap=0.190,
            name="left runner remains deeply inserted when closed",
        )
        rest_position = ctx.part_world_position(tray)

    with ctx.pose({tray_slide: tray_extension}):
        ctx.expect_contact(
            tray,
            housing,
            elem_a="left_runner",
            elem_b="left_guide",
            name="left runner still rides the guide when extended",
        )
        ctx.expect_contact(
            tray,
            housing,
            elem_a="right_runner",
            elem_b="right_guide",
            name="right runner still rides the guide when extended",
        )
        ctx.expect_overlap(
            tray,
            housing,
            axes="y",
            elem_a="left_runner",
            elem_b="left_guide",
            min_overlap=0.030,
            name="left runner keeps retained insertion at full extension",
        )
        ctx.expect_overlap(
            tray,
            housing,
            axes="y",
            elem_a="right_runner",
            elem_b="right_guide",
            min_overlap=0.030,
            name="right runner keeps retained insertion at full extension",
        )
        extended_position = ctx.part_world_position(tray)

    ctx.check(
        "intake tray extends forward",
        rest_position is not None
        and extended_position is not None
        and extended_position[1] > rest_position[1] + 0.120,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({dial_joint: 0.0}):
        pointer_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))

    with ctx.pose({dial_joint: 1.1}):
        pointer_turned = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_pointer"))

    ctx.check(
        "dial pointer sweeps as the knob rotates",
        pointer_rest is not None
        and pointer_turned is not None
        and abs(pointer_turned[0] - pointer_rest[0]) > 0.012
        and abs(pointer_turned[2] - pointer_rest[2]) > 0.006,
        details=f"rest={pointer_rest}, turned={pointer_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
