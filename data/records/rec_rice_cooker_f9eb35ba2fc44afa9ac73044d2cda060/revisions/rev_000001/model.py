from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box_mesh(
    size: tuple[float, float, float],
    name: str,
    *,
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
    fillet: float = 0.004,
):
    """Small CadQuery rounded box helper, authored in meters."""

    shape = cq.Workplane("XY").box(*size)
    if fillet > 0.0:
        shape = shape.edges().fillet(fillet)
    if translate != (0.0, 0.0, 0.0):
        shape = shape.translate(translate)
    return mesh_from_cadquery(shape, name, tolerance=0.0006, angular_tolerance=0.08)


def _housing_shell_mesh():
    depth, width, height = 0.220, 0.235, 0.125
    cavity_depth, cavity_width, cavity_height = 0.155, 0.165, 0.096

    outer = (
        cq.Workplane("XY")
        .box(depth, width, height)
        .translate((0.0, 0.0, height / 2.0))
        .edges("|Z")
        .fillet(0.023)
        .edges(">Z")
        .fillet(0.006)
    )
    cutter = (
        cq.Workplane("XY")
        .box(cavity_depth, cavity_width, cavity_height)
        .translate((0.0, 0.0, height - cavity_height / 2.0 + 0.002))
        .edges("|Z")
        .fillet(0.022)
    )
    shell = outer.cut(cutter)
    return mesh_from_cadquery(shell, "housing_shell", tolerance=0.0008, angular_tolerance=0.08)


def _inner_bowl_mesh():
    outer_radius = 0.074
    inner_radius = 0.066
    height = 0.079
    wall = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(height)
        .cut(
            cq.Workplane("XY")
            .circle(inner_radius)
            .extrude(height + 0.004)
            .translate((0.0, 0.0, 0.007))
        )
        .edges(">Z")
        .fillet(0.003)
    )
    return mesh_from_cadquery(wall, "inner_bowl", tolerance=0.0007, angular_tolerance=0.08)


def _lid_shell_mesh():
    depth, width, height = 0.210, 0.224, 0.031
    shell = (
        cq.Workplane("XY")
        .box(depth, width, height)
        .translate((0.012 + depth / 2.0, 0.0, 0.002 + height / 2.0))
        .edges("|Z")
        .fillet(0.020)
        .edges(">Z")
        .fillet(0.008)
    )
    return mesh_from_cadquery(shell, "lid_shell", tolerance=0.0007, angular_tolerance=0.08)


def _button_mesh(size: tuple[float, float, float], name: str):
    thickness, width, height = size
    return _rounded_box_mesh(
        size,
        name,
        translate=(thickness / 2.0, 0.0, 0.0),
        fillet=min(width, height) * 0.22,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_rice_cooker")

    warm_white = model.material("warm_white_plastic", rgba=(0.92, 0.88, 0.78, 1.0))
    soft_white = model.material("soft_white_lid", rgba=(0.96, 0.94, 0.88, 1.0))
    graphite = model.material("graphite_panel", rgba=(0.05, 0.055, 0.060, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    satin_metal = model.material("satin_inner_bowl", rgba=(0.72, 0.72, 0.68, 1.0))
    latch_red = model.material("muted_latch_red", rgba=(0.72, 0.16, 0.12, 1.0))
    menu_blue = model.material("menu_button_blue", rgba=(0.35, 0.48, 0.62, 1.0))
    menu_gray = model.material("menu_button_gray", rgba=(0.66, 0.68, 0.70, 1.0))

    housing = model.part("housing")
    housing.visual(_housing_shell_mesh(), material=warm_white, name="housing_shell")
    housing.visual(
        _inner_bowl_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0308)),
        material=satin_metal,
        name="inner_bowl",
    )
    housing.visual(
        Box((0.006, 0.148, 0.062)),
        origin=Origin(xyz=(0.112, 0.0, 0.058)),
        material=graphite,
        name="front_panel",
    )
    housing.visual(
        Box((0.014, 0.045, 0.010)),
        origin=Origin(xyz=(-0.115, 0.089, 0.122)),
        material=warm_white,
        name="hinge_leaf_0",
    )
    housing.visual(
        Box((0.014, 0.045, 0.010)),
        origin=Origin(xyz=(-0.115, -0.089, 0.122)),
        material=warm_white,
        name="hinge_leaf_1",
    )
    for idx, y in enumerate((0.089, -0.089)):
        housing.visual(
            Cylinder(radius=0.009, length=0.045),
            origin=Origin(xyz=(-0.121, y, 0.126), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_white,
            name=f"fixed_hinge_barrel_{idx}",
        )
    for idx, (x, y) in enumerate(((0.068, 0.075), (0.068, -0.075), (-0.068, 0.075), (-0.068, -0.075))):
        housing.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(xyz=(x, y, 0.005)),
            material=dark_rubber,
            name=f"foot_{idx}",
        )

    lid = model.part("lid")
    lid.visual(_lid_shell_mesh(), material=soft_white, name="lid_shell")
    lid.visual(
        Box((0.018, 0.098, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, 0.006)),
        material=soft_white,
        name="hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_white,
        name="lid_hinge_barrel",
    )
    lid.visual(
        _rounded_box_mesh((0.070, 0.055, 0.010), "lid_handle", translate=(0.132, 0.0, 0.038), fillet=0.008),
        material=soft_white,
        name="lid_handle",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.003),
        origin=Origin(xyz=(0.060, -0.060, 0.0345)),
        material=graphite,
        name="steam_vent",
    )

    model.articulation(
        "housing_to_lid",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lid,
        origin=Origin(xyz=(-0.121, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.2, lower=0.0, upper=1.25),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        _button_mesh((0.010, 0.072, 0.019), "latch_button_cap"),
        material=latch_red,
        name="latch_cap",
    )
    model.articulation(
        "housing_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=latch_button,
        origin=Origin(xyz=(0.115, 0.0, 0.076)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.10, lower=0.0, upper=0.0045),
    )

    button_specs = (
        ("menu_button_0", "housing_to_menu_button_0", -0.032, menu_blue),
        ("menu_button_1", "housing_to_menu_button_1", 0.032, menu_gray),
    )
    for part_name, joint_name, y, material in button_specs:
        button = model.part(part_name)
        button.visual(
            _button_mesh((0.008, 0.035, 0.018), f"{part_name}_cap"),
            material=material,
            name="menu_cap",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(0.115, y, 0.043)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.0035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch_button")
    menu_0 = object_model.get_part("menu_button_0")
    menu_1 = object_model.get_part("menu_button_1")

    lid_joint = object_model.get_articulation("housing_to_lid")
    latch_joint = object_model.get_articulation("housing_to_latch_button")
    menu_0_joint = object_model.get_articulation("housing_to_menu_button_0")
    menu_1_joint = object_model.get_articulation("housing_to_menu_button_1")

    ctx.expect_gap(
        lid,
        housing,
        axis="z",
        min_gap=0.001,
        max_gap=0.008,
        positive_elem="lid_shell",
        negative_elem="housing_shell",
        name="closed lid sits just above the housing rim",
    )
    for button, elem, check_name in (
        (latch, "latch_cap", "latch button seats on front panel"),
        (menu_0, "menu_cap", "first menu button seats on front panel"),
        (menu_1, "menu_cap", "second menu button seats on front panel"),
    ):
        ctx.expect_gap(
            button,
            housing,
            axis="x",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem=elem,
            negative_elem="front_panel",
            name=check_name,
        )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.15}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge lifts the lid upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_latch = ctx.part_world_position(latch)
    rest_menu_0 = ctx.part_world_position(menu_0)
    rest_menu_1 = ctx.part_world_position(menu_1)
    with ctx.pose({latch_joint: 0.0045, menu_0_joint: 0.0035}):
        depressed_latch = ctx.part_world_position(latch)
        depressed_menu_0 = ctx.part_world_position(menu_0)
        stationary_menu_1 = ctx.part_world_position(menu_1)
    ctx.check(
        "latch and first menu button depress inward independently",
        rest_latch is not None
        and depressed_latch is not None
        and rest_menu_0 is not None
        and depressed_menu_0 is not None
        and rest_menu_1 is not None
        and stationary_menu_1 is not None
        and depressed_latch[0] < rest_latch[0] - 0.003
        and depressed_menu_0[0] < rest_menu_0[0] - 0.002
        and abs(stationary_menu_1[0] - rest_menu_1[0]) < 0.0005,
        details=(
            f"latch {rest_latch}->{depressed_latch}, "
            f"menu0 {rest_menu_0}->{depressed_menu_0}, menu1 {rest_menu_1}->{stationary_menu_1}"
        ),
    )

    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "countertop travel scale",
        housing_aabb is not None
        and 0.18 <= (housing_aabb[1][0] - housing_aabb[0][0]) <= 0.27
        and 0.18 <= (housing_aabb[1][1] - housing_aabb[0][1]) <= 0.29
        and 0.10 <= (housing_aabb[1][2] - housing_aabb[0][2]) <= 0.18,
        details=f"housing_aabb={housing_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
