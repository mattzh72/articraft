from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _lathe_mesh(profile: list[tuple[float, float]], name: str):
    return mesh_from_geometry(LatheGeometry(profile, segments=72, closed=True), name)


def _rounded_panel_mesh(width: float, depth: float, height: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_belgian_waffle_maker")

    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.056, 0.060, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    nonstick = model.material("nonstick_plate", rgba=(0.045, 0.047, 0.050, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.66, 0.68, 0.69, 1.0))
    graphite = model.material("graphite_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    button_grey = model.material("button_grey", rgba=(0.30, 0.31, 0.33, 1.0))
    label_white = model.material("label_white", rgba=(0.92, 0.92, 0.88, 1.0))
    program_red = model.material("program_red", rgba=(0.76, 0.10, 0.08, 1.0))
    program_amber = model.material("program_amber", rgba=(0.95, 0.55, 0.12, 1.0))
    program_green = model.material("program_green", rgba=(0.18, 0.62, 0.25, 1.0))

    base_body_mesh = _lathe_mesh(
        [
            (0.000, 0.000),
            (0.170, 0.000),
            (0.193, 0.010),
            (0.200, 0.038),
            (0.190, 0.066),
            (0.000, 0.066),
        ],
        "lower_round_housing",
    )
    top_dome_mesh = _lathe_mesh(
        [
            (0.000, 0.088),
            (0.058, 0.085),
            (0.118, 0.063),
            (0.156, 0.028),
            (0.168, 0.000),
            (0.000, 0.000),
        ],
        "domed_top_shell",
    )
    control_panel_mesh = _rounded_panel_mesh(0.205, 0.088, 0.052, 0.020, "front_control_cluster")
    button_mesh = _rounded_panel_mesh(0.032, 0.021, 0.010, 0.006, "program_button_cap")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.045,
            0.026,
            body_style="skirted",
            top_diameter=0.036,
            skirt=KnobSkirt(0.053, 0.006, flare=0.06, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=20.0),
            bore=KnobBore(style="round", diameter=0.007),
            center=False,
        ),
        "thermostat_fluted_knob",
    )

    lower_housing = model.part("lower_housing")
    lower_housing.visual(
        base_body_mesh,
        material=dark_plastic,
        name="round_body",
    )
    lower_housing.visual(
        control_panel_mesh,
        origin=Origin(xyz=(0.0, -0.226, 0.018)),
        material=graphite,
        name="front_cluster",
    )
    lower_housing.visual(
        Cylinder(radius=0.158, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=nonstick,
        name="lower_plate",
    )
    # Raised Belgian waffle grid on the lower cooking plate.
    for index, offset in enumerate((-0.090, -0.045, 0.0, 0.045, 0.090)):
        lower_housing.visual(
            Box((0.008, 0.218, 0.004)),
            origin=Origin(xyz=(offset, 0.0, 0.0775)),
            material=nonstick,
            name=f"waffle_x_rib_{index}",
        )
        lower_housing.visual(
            Box((0.218, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, offset, 0.0775)),
            material=nonstick,
            name=f"waffle_y_rib_{index}",
        )

    # Vented side-wall insets, slightly proud/embedded on each side of the round housing.
    for side, x_pos in (("side_0", -0.195), ("side_1", 0.195)):
        for row, z_pos in enumerate((0.028, 0.039, 0.050)):
            for column, y_pos in enumerate((-0.065, -0.025, 0.015, 0.055)):
                lower_housing.visual(
                    Box((0.020, 0.028, 0.005)),
                    origin=Origin(xyz=(x_pos, y_pos, z_pos)),
                    material=satin_black,
                    name=f"{side}_vent_{row}_{column}",
                )

    # Rear hinge support: two heavy brackets and a captured steel-colored pin.
    lower_housing.visual(
        Box((0.335, 0.046, 0.022)),
        origin=Origin(xyz=(0.0, 0.170, 0.064)),
        material=dark_plastic,
        name="rear_hinge_plinth",
    )
    for index, x_pos in enumerate((-0.140, 0.140)):
        lower_housing.visual(
            Box((0.035, 0.036, 0.055)),
            origin=Origin(xyz=(x_pos, 0.180, 0.0825)),
            material=dark_plastic,
            name=f"hinge_bracket_{index}",
        )
    lower_housing.visual(
        Cylinder(radius=0.006, length=0.340),
        origin=Origin(xyz=(0.0, 0.180, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_pin",
    )

    top_half = model.part("top_half")
    top_half.visual(
        top_dome_mesh,
        origin=Origin(xyz=(0.0, -0.180, -0.018)),
        material=stainless,
        name="domed_shell",
    )
    top_half.visual(
        Cylinder(radius=0.147, length=0.008),
        origin=Origin(xyz=(0.0, -0.170, -0.020)),
        material=nonstick,
        name="upper_plate",
    )
    top_half.visual(
        Cylinder(radius=0.018, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="top_barrel",
    )
    top_half.visual(
        Box((0.090, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, -0.020, -0.002)),
        material=dark_plastic,
        name="hinge_leaf",
    )
    # A molded front handle carried by two posts and tied into the domed shell.
    for index, x_pos in enumerate((-0.058, 0.058)):
        top_half.visual(
            Box((0.022, 0.021, 0.070)),
            origin=Origin(xyz=(x_pos, -0.335, 0.025)),
            material=satin_black,
            name=f"handle_post_{index}",
        )
    top_half.visual(
        Box((0.165, 0.036, 0.018)),
        origin=Origin(xyz=(0.0, -0.335, 0.064)),
        material=satin_black,
        name="front_handle",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_housing,
        child=top_half,
        origin=Origin(xyz=(0.0, 0.180, 0.105)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.4, lower=0.0, upper=1.22),
    )

    knob = model.part("thermostat_knob")
    knob.visual(
        knob_mesh,
        material=satin_black,
        name="knob_shell",
    )
    knob.visual(
        Box((0.004, 0.020, 0.0014)),
        origin=Origin(xyz=(0.0, 0.008, 0.0268)),
        material=label_white,
        name="pointer_mark",
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=lower_housing,
        child=knob,
        origin=Origin(xyz=(0.058, -0.226, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=7.0),
    )

    button_origins = [(-0.060, -0.226, 0.070), (-0.025, -0.226, 0.070), (0.010, -0.226, 0.070)]
    button_mats = [program_red, program_amber, program_green]
    for index, (button_origin, indicator_mat) in enumerate(zip(button_origins, button_mats)):
        button = model.part(f"program_button_{index}")
        button.visual(
            button_mesh,
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0042, length=0.0012),
            origin=Origin(xyz=(0.0, 0.0, 0.0106)),
            material=indicator_mat,
            name="program_dot",
        )
        model.articulation(
            f"button_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=lower_housing,
            child=button,
            origin=Origin(xyz=button_origin),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.055, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_housing")
    top = object_model.get_part("top_half")
    hinge = object_model.get_articulation("rear_hinge")
    knob_joint = object_model.get_articulation("knob_spin")

    ctx.allow_overlap(
        lower,
        top,
        elem_a="hinge_pin",
        elem_b="top_barrel",
        reason="The hinge pin is intentionally captured inside the top barrel proxy.",
    )
    ctx.expect_within(
        lower,
        top,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="top_barrel",
        margin=0.001,
        name="hinge pin centered in barrel",
    )
    ctx.expect_overlap(
        lower,
        top,
        axes="x",
        elem_a="hinge_pin",
        elem_b="top_barrel",
        min_overlap=0.10,
        name="hinge barrel surrounds pin along axis",
    )
    ctx.expect_gap(
        top,
        lower,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.001,
        max_gap=0.006,
        name="closed waffle plates have small cooking gap",
    )

    rest_aabb = ctx.part_world_aabb(top)
    with ctx.pose({hinge: 1.05}):
        open_aabb = ctx.part_world_aabb(top)
    ctx.check(
        "top half opens upward",
        rest_aabb is not None
        and open_aabb is not None
        and float(open_aabb[1][2]) > float(rest_aabb[1][2]) + 0.045,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    ctx.check(
        "thermostat knob uses continuous rotation",
        knob_joint is not None and str(knob_joint.articulation_type).endswith("CONTINUOUS"),
        details=f"joint={knob_joint}",
    )
    ctx.expect_gap(
        "thermostat_knob",
        lower,
        axis="z",
        positive_elem="knob_shell",
        negative_elem="front_cluster",
        min_gap=0.0,
        max_gap=0.002,
        name="thermostat knob sits on front cluster",
    )

    for index in range(3):
        button_name = f"program_button_{index}"
        slide = object_model.get_articulation(f"button_slide_{index}")
        ctx.expect_gap(
            button_name,
            lower,
            axis="z",
            positive_elem="button_cap",
            negative_elem="front_cluster",
            min_gap=0.0,
            max_gap=0.002,
            name=f"program button {index} mounted on cluster",
        )
        rest_pos = ctx.part_world_position(button_name)
        with ctx.pose({slide: 0.004}):
            pressed_pos = ctx.part_world_position(button_name)
        ctx.check(
            f"program button {index} pushes downward",
            rest_pos is not None
            and pressed_pos is not None
            and float(pressed_pos[2]) < float(rest_pos[2]) - 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
