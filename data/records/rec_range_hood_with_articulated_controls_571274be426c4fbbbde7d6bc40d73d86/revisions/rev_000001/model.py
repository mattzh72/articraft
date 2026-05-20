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


def _add_rotary_knob(
    model: ArticulatedObject,
    *,
    part_name: str,
    joint_name: str,
    parent,
    x: float,
    y: float,
    z: float,
    knob_material: str,
    accent_material: str,
):
    knob = model.part(part_name)
    knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.0175, length=0.015),
        origin=Origin(xyz=(0.0, 0.0195, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.019, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0285, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_material,
        name="front_rim",
    )
    knob.visual(
        Box((0.004, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.0305, 0.012,)),
        material=accent_material,
        name="index_tick",
    )

    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=parent,
        child=knob,
        origin=Origin(xyz=(x, y, z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=6.0,
            lower=-pi,
            upper=pi,
        ),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood")

    body_metal = model.material("body_metal", rgba=(0.82, 0.84, 0.86, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    filter_mesh = model.material("filter_mesh", rgba=(0.34, 0.36, 0.38, 1.0))
    indicator = model.material("indicator", rgba=(0.87, 0.88, 0.90, 1.0))

    width = 0.76
    depth = 0.50
    height = 0.16
    wall = 0.015
    top_thickness = 0.015
    front_lip_depth = 0.07
    front_lip_height = 0.045
    opening_width = 0.67
    opening_depth = 0.36
    opening_rear_y = -0.18
    opening_front_y = opening_rear_y + opening_depth

    canopy = model.part("canopy")
    canopy.visual(
        Box((width, depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, height - top_thickness / 2.0)),
        material=body_metal,
        name="top_panel",
    )
    canopy.visual(
        Box((wall, depth, height - top_thickness)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, (height - top_thickness) / 2.0)),
        material=body_metal,
        name="right_wall",
    )
    canopy.visual(
        Box((wall, depth, height - top_thickness)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, (height - top_thickness) / 2.0)),
        material=body_metal,
        name="left_wall",
    )
    canopy.visual(
        Box((width - 2.0 * wall, wall, height - top_thickness)),
        origin=Origin(
            xyz=(0.0, -depth / 2.0 + wall / 2.0, (height - top_thickness) / 2.0)
        ),
        material=body_metal,
        name="back_wall",
    )
    canopy.visual(
        Box((width - 2.0 * wall, 0.012, 0.11)),
        origin=Origin(xyz=(0.0, 0.208, 0.095), rpy=(0.52, 0.0, 0.0)),
        material=body_metal,
        name="visor_face",
    )
    canopy.visual(
        Box((width, front_lip_depth, front_lip_height)),
        origin=Origin(
            xyz=(0.0, opening_front_y + front_lip_depth / 2.0, front_lip_height / 2.0)
        ),
        material=body_metal,
        name="front_lip",
    )
    canopy.visual(
        Box((opening_width, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, opening_rear_y - 0.01, 0.01)),
        material=body_metal,
        name="rear_rail",
    )
    canopy.visual(
        Box((0.025, opening_depth, 0.02)),
        origin=Origin(xyz=(opening_width / 2.0 + 0.0125, 0.0, 0.01)),
        material=body_metal,
        name="right_filter_rail",
    )
    canopy.visual(
        Box((0.025, opening_depth, 0.02)),
        origin=Origin(xyz=(-opening_width / 2.0 - 0.0125, 0.0, 0.01)),
        material=body_metal,
        name="left_filter_rail",
    )
    canopy.visual(
        Box((0.16, 0.018, 0.006)),
        origin=Origin(xyz=(0.205, opening_front_y + 0.009, 0.041)),
        material=dark_trim,
        name="control_strip",
    )

    filter_panel = model.part("filter_panel")
    filter_panel.visual(
        Box((0.66, opening_depth, 0.012)),
        origin=Origin(xyz=(0.0, opening_depth / 2.0, -0.010)),
        material=body_metal,
        name="filter_frame",
    )
    filter_panel.visual(
        Box((0.58, 0.30, 0.004)),
        origin=Origin(xyz=(0.0, opening_depth / 2.0, -0.009)),
        material=filter_mesh,
        name="mesh_insert",
    )
    filter_panel.visual(
        Box((0.66, 0.02, 0.018)),
        origin=Origin(xyz=(0.0, 0.01, -0.009)),
        material=body_metal,
        name="hinge_strip",
    )
    filter_panel.visual(
        Box((0.12, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, opening_depth - 0.01, -0.017)),
        material=dark_trim,
        name="pull_tab",
    )

    model.articulation(
        "canopy_to_filter_panel",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=filter_panel,
        origin=Origin(xyz=(0.0, opening_rear_y, 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    _add_rotary_knob(
        model,
        part_name="fan_knob",
        joint_name="canopy_to_fan_knob",
        parent=canopy,
        x=0.17,
        y=0.25,
        z=0.024,
        knob_material=dark_trim.name,
        accent_material=indicator.name,
    )
    _add_rotary_knob(
        model,
        part_name="light_knob",
        joint_name="canopy_to_light_knob",
        parent=canopy,
        x=0.245,
        y=0.25,
        z=0.024,
        knob_material=dark_trim.name,
        accent_material=indicator.name,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    canopy = object_model.get_part("canopy")
    filter_panel = object_model.get_part("filter_panel")
    fan_knob = object_model.get_part("fan_knob")
    light_knob = object_model.get_part("light_knob")

    filter_hinge = object_model.get_articulation("canopy_to_filter_panel")
    fan_joint = object_model.get_articulation("canopy_to_fan_knob")
    light_joint = object_model.get_articulation("canopy_to_light_knob")

    ctx.expect_gap(
        canopy,
        filter_panel,
        axis="y",
        positive_elem="front_lip",
        negative_elem="filter_frame",
        min_gap=0.0,
        max_gap=0.001,
        name="closed filter panel nests just behind the front lip",
    )
    ctx.expect_gap(
        filter_panel,
        canopy,
        axis="y",
        positive_elem="filter_frame",
        negative_elem="rear_rail",
        min_gap=0.0,
        max_gap=0.001,
        name="closed filter panel seats to the rear hinge rail",
    )
    ctx.expect_within(
        filter_panel,
        canopy,
        axes="x",
        inner_elem="filter_frame",
        outer_elem="rear_rail",
        margin=0.006,
        name="filter frame width stays inside the intake opening",
    )

    ctx.expect_contact(
        fan_knob,
        canopy,
        elem_a="shaft",
        elem_b="front_lip",
        contact_tol=0.0005,
        name="fan knob shaft seats on the front lip",
    )
    ctx.expect_contact(
        light_knob,
        canopy,
        elem_a="shaft",
        elem_b="front_lip",
        contact_tol=0.0005,
        name="light knob shaft seats on the front lip",
    )

    ctx.check(
        "filter hinge axis runs left-right",
        abs(filter_hinge.axis[0]) > 0.99
        and abs(filter_hinge.axis[1]) < 1e-6
        and abs(filter_hinge.axis[2]) < 1e-6,
        details=f"axis={filter_hinge.axis}",
    )
    ctx.check(
        "control knob joints spin front-to-back",
        abs(fan_joint.axis[1]) > 0.99
        and abs(light_joint.axis[1]) > 0.99
        and abs(fan_joint.axis[0]) < 1e-6
        and abs(light_joint.axis[0]) < 1e-6
        and abs(fan_joint.axis[2]) < 1e-6
        and abs(light_joint.axis[2]) < 1e-6,
        details=f"fan_axis={fan_joint.axis}, light_axis={light_joint.axis}",
    )

    pull_rest = ctx.part_element_world_aabb(filter_panel, elem="pull_tab")
    with ctx.pose({filter_hinge: 1.0}):
        pull_open = ctx.part_element_world_aabb(filter_panel, elem="pull_tab")
        ctx.expect_contact(
            fan_knob,
            canopy,
            elem_a="shaft",
            elem_b="front_lip",
            contact_tol=0.0005,
            name="fan knob shaft stays seated while rotating",
        )
    ctx.check(
        "filter panel folds downward from the rear hinge",
        pull_rest is not None
        and pull_open is not None
        and pull_open[0][2] < pull_rest[0][2] - 0.12,
        details=f"rest={pull_rest}, open={pull_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
