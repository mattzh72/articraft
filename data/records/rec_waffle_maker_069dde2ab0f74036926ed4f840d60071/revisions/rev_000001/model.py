from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    section_loft,
)


HINGE_X = -0.205
HINGE_Z = 0.110
LID_CENTER_X = 0.205
LID_OPEN_LIMIT = 1.45
BUTTON_TRAVEL = 0.006


def _circle_section(
    radius: float,
    z: float,
    *,
    center_x: float = 0.0,
    segments: int = 64,
) -> list[tuple[float, float, float]]:
    return [
        (
            center_x + radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
            z,
        )
        for index in range(segments)
    ]


def _lower_body_geometry():
    return section_loft(
        [
            _circle_section(0.158, 0.000),
            _circle_section(0.182, 0.007),
            _circle_section(0.190, 0.035),
            _circle_section(0.180, 0.066),
            _circle_section(0.145, 0.078),
        ]
    )


def _lid_dome_geometry():
    return section_loft(
        [
            _circle_section(0.152, -0.006, center_x=LID_CENTER_X),
            _circle_section(0.174, 0.001, center_x=LID_CENTER_X),
            _circle_section(0.178, 0.018, center_x=LID_CENTER_X),
            _circle_section(0.158, 0.046, center_x=LID_CENTER_X),
            _circle_section(0.104, 0.070, center_x=LID_CENTER_X),
            _circle_section(0.012, 0.078, center_x=LID_CENTER_X),
        ]
    )


def _rounded_box_cq(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    fillet: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2])
        .edges("|Z")
        .fillet(fillet)
        .translate(center)
    )


def _rib_lengths(span: float, pitch: float) -> list[tuple[float, float]]:
    offsets = [index * pitch for index in range(-3, 4)]
    return [(offset, 2.0 * math.sqrt(max(span * span - offset * offset, 0.0))) for offset in offsets]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_belgian_waffle_maker")

    body_black = model.material("satin_black", rgba=(0.035, 0.037, 0.040, 1.0))
    rubber_black = model.material("black_rubber", rgba=(0.010, 0.011, 0.012, 1.0))
    dark_plate = model.material("nonstick_dark", rgba=(0.060, 0.058, 0.052, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.66, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.36, 0.36, 0.34, 1.0))
    button_green = model.material("green_program_button", rgba=(0.10, 0.44, 0.25, 1.0))
    button_amber = model.material("amber_program_button", rgba=(0.86, 0.52, 0.14, 1.0))

    lower = model.part("lower_housing")
    lower.visual(
        mesh_from_geometry(_lower_body_geometry(), "lower_round_housing"),
        material=body_black,
        name="lower_body",
    )
    lower.visual(
        Cylinder(radius=0.138, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=dark_plate,
        name="lower_plate",
    )
    for index, (offset, length) in enumerate(_rib_lengths(0.126, 0.036)):
        lower.visual(
            Box((length, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, offset, 0.088)),
            material=dark_plate,
            name=f"lower_grid_x_{index}",
        )
        lower.visual(
            Box((0.006, length, 0.006)),
            origin=Origin(xyz=(offset, 0.0, 0.088)),
            material=dark_plate,
            name=f"lower_grid_y_{index}",
        )
    lower.visual(
        mesh_from_cadquery(
            _rounded_box_cq((0.092, 0.074, 0.012), (0.145, -0.130, 0.082), 0.010),
            "front_control_pod",
            tolerance=0.0008,
        ),
        material=body_black,
        name="control_pod",
    )
    for y_center, name in ((-0.095, "base_hinge_barrel_0"), (0.095, "base_hinge_barrel_1")):
        lower.visual(
            Box((0.075, 0.052, 0.036)),
            origin=Origin(xyz=(-0.150, y_center, 0.078)),
            material=body_black,
            name=f"{name}_saddle",
        )
        lower.visual(
            Box((0.066, 0.050, 0.020)),
            origin=Origin(xyz=(-0.178, y_center, 0.095)),
            material=body_black,
            name=f"{name}_strap",
        )
        lower.visual(
            Cylinder(radius=0.014, length=0.044),
            origin=Origin(xyz=(HINGE_X, y_center, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=name,
        )

    top = model.part("top_half")
    top.visual(
        mesh_from_geometry(_lid_dome_geometry(), "domed_top_plate"),
        material=brushed_steel,
        name="top_dome",
    )
    top.visual(
        Cylinder(radius=0.130, length=0.010),
        origin=Origin(xyz=(LID_CENTER_X, 0.0, -0.011)),
        material=dark_plate,
        name="upper_plate",
    )
    for index, (offset, length) in enumerate(_rib_lengths(0.118, 0.034)):
        top.visual(
            Box((length, 0.005, 0.006)),
            origin=Origin(xyz=(LID_CENTER_X, offset, -0.016)),
            material=dark_plate,
            name=f"upper_grid_x_{index}",
        )
        top.visual(
            Box((0.005, length, 0.006)),
            origin=Origin(xyz=(LID_CENTER_X + offset, 0.0, -0.016)),
            material=dark_plate,
            name=f"upper_grid_y_{index}",
        )
    top.visual(
        Cylinder(radius=0.012, length=0.112),
        origin=Origin(xyz=(0.387, 0.0, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="front_grip",
    )
    for y_center in (-0.047, 0.047):
        top.visual(
            Box((0.026, 0.018, 0.040)),
            origin=Origin(xyz=(0.365, y_center, 0.041)),
            material=rubber_black,
            name=f"front_handle_post_{0 if y_center < 0 else 1}",
        )
    top.visual(
        Box((0.066, 0.104, 0.018)),
        origin=Origin(xyz=(0.028, 0.0, 0.002)),
        material=body_black,
        name="top_hinge_leaf",
    )
    top.visual(
        Cylinder(radius=0.012, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="top_hinge_barrel",
    )

    model.articulation(
        "lower_to_top",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=top,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=LID_OPEN_LIMIT),
    )

    button_mesh = mesh_from_geometry(
        KnobGeometry(0.025, 0.012, body_style="domed", edge_radius=0.001, center=False),
        "raised_program_button",
    )
    button_specs = [
        ("program_button_0", (0.166, -0.112, 0.088), button_green),
        ("program_button_1", (0.132, -0.146, 0.088), button_amber),
    ]
    for index, (part_name, xyz, material) in enumerate(button_specs):
        button = model.part(part_name)
        button.visual(
            button_mesh,
            origin=Origin(),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=lower,
            child=button,
            origin=Origin(xyz=xyz),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_housing")
    top = object_model.get_part("top_half")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    top_joint = object_model.get_articulation("lower_to_top")
    button_joint_0 = object_model.get_articulation("housing_to_button_0")
    button_joint_1 = object_model.get_articulation("housing_to_button_1")

    with ctx.pose({top_joint: 0.0, button_joint_0: 0.0, button_joint_1: 0.0}):
        ctx.expect_overlap(
            top,
            lower,
            axes="xy",
            min_overlap=0.135,
            elem_a="top_dome",
            elem_b="lower_body",
            name="closed round lid overlaps circular lower housing",
        )
        ctx.expect_gap(
            top,
            lower,
            axis="z",
            min_gap=0.010,
            positive_elem="top_dome",
            negative_elem="lower_plate",
            name="closed dome clears lower waffle plate",
        )
        ctx.expect_overlap(
            top,
            lower,
            axes="xz",
            min_overlap=0.010,
            elem_a="top_hinge_barrel",
            elem_b="base_hinge_barrel_0",
            name="visible hinge barrels share the rear hinge line",
        )
        for button in (button_0, button_1):
            ctx.expect_gap(
                button,
                lower,
                axis="z",
                max_gap=0.0015,
                max_penetration=0.0005,
                positive_elem="button_cap",
                negative_elem="control_pod",
                name=f"{button.name} sits proud on the discrete front pod",
            )

    rest_aabb = ctx.part_world_aabb(top)
    with ctx.pose({top_joint: LID_OPEN_LIMIT}):
        open_aabb = ctx.part_world_aabb(top)
    ctx.check(
        "top half rotates upward on rear hinge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > rest_aabb[1][2] + 0.12,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: BUTTON_TRAVEL}):
        pressed_button_0 = ctx.part_world_position(button_0)
        unchanged_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_1: BUTTON_TRAVEL}):
        pressed_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "program_button_0 depresses independently",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and rest_button_1 is not None
        and unchanged_button_1 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.004
        and abs(unchanged_button_1[2] - rest_button_1[2]) < 1.0e-5,
        details=f"rest0={rest_button_0}, pressed0={pressed_button_0}, other={unchanged_button_1}",
    )
    ctx.check(
        "program_button_1 depresses independently",
        rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_1[2] < rest_button_1[2] - 0.004,
        details=f"rest1={rest_button_1}, pressed1={pressed_button_1}",
    )

    return ctx.report()


object_model = build_object_model()
