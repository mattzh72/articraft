from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.320
BODY_DEPTH = 0.245
LOWER_HEIGHT = 0.305
HEAD_HEIGHT = 0.120
TOTAL_HEIGHT = LOWER_HEIGHT + HEAD_HEIGHT

BIN_WIDTH = 0.286
BIN_DEPTH = 0.195
BIN_HEIGHT = 0.288
BIN_WALL = 0.004
BIN_TRAVEL = 0.115
PANEL_FACE_Y = -0.1225
CUTTER_Z = 0.338
FRONT_CUTTER_Y = -0.028
REAR_CUTTER_Y = 0.002


def _housing_shape() -> cq.Workplane:
    lower = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, LOWER_HEIGHT).translate((0.0, 0.0, LOWER_HEIGHT / 2.0))
    head = cq.Workplane("XY").box(BODY_WIDTH, 0.225, HEAD_HEIGHT).translate((0.0, -0.004, LOWER_HEIGHT + HEAD_HEIGHT / 2.0))
    housing = lower.union(head)

    bin_cavity = (
        cq.Workplane("XY")
        .box(0.292, 0.206, 0.294)
        .translate((0.0, -0.024, 0.014 + 0.294 / 2.0))
    )
    head_chamber = (
        cq.Workplane("XY")
        .box(0.288, 0.108, 0.074)
        .translate((0.0, -0.012, 0.304 + 0.074 / 2.0))
    )
    paper_slot = (
        cq.Workplane("XY")
        .box(0.226, 0.009, 0.060)
        .translate((0.0, -0.014, TOTAL_HEIGHT - 0.018))
    )

    return housing.cut(bin_cavity).cut(head_chamber).cut(paper_slot)


def _bin_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(BIN_WIDTH, BIN_DEPTH, BIN_HEIGHT)
        .translate((0.0, BIN_DEPTH / 2.0, BIN_HEIGHT / 2.0))
    )
    inner = (
        cq.Workplane("XY")
        .box(BIN_WIDTH - 2.0 * BIN_WALL, BIN_DEPTH - 2.0 * BIN_WALL, BIN_HEIGHT - BIN_WALL)
        .translate(
            (
                0.0,
                BIN_WALL + (BIN_DEPTH - 2.0 * BIN_WALL) / 2.0,
                BIN_WALL + (BIN_HEIGHT - BIN_WALL) / 2.0,
            )
        )
    )
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_paper_shredder")

    housing_dark = model.material("housing_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    bin_smoke = model.material("bin_smoke", rgba=(0.47, 0.58, 0.62, 0.34))
    handle_dark = model.material("handle_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    top_button_finish = model.material("top_button_finish", rgba=(0.36, 0.63, 0.36, 1.0))
    middle_button_finish = model.material("middle_button_finish", rgba=(0.72, 0.74, 0.77, 1.0))
    bottom_button_finish = model.material("bottom_button_finish", rgba=(0.71, 0.20, 0.19, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "shredder_housing"),
        material=housing_dark,
        name="body_shell",
    )
    housing.visual(
        Box((0.192, 0.006, 0.074)),
        origin=Origin(xyz=(0.022, -0.1195, 0.358)),
        material=panel_dark,
        name="control_panel",
    )
    housing.visual(
        Box((BODY_WIDTH - 0.020, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - 0.005, 0.070)),
        material=housing_dark,
        name="rear_spine",
    )
    for side, x_pos in (("left", -0.1445), ("right", 0.1445)):
        housing.visual(
            Box((0.003, 0.160, 0.014)),
            origin=Origin(xyz=(x_pos, -0.020, 0.060)),
            material=panel_dark,
            name=f"{side}_runner",
        )
    for name, y_pos in (("front", FRONT_CUTTER_Y), ("rear", REAR_CUTTER_Y)):
        for side, x_pos in (("left", -0.142), ("right", 0.142)):
            housing.visual(
                Box((0.004, 0.016, 0.018)),
                origin=Origin(xyz=(x_pos, y_pos, CUTTER_Z)),
                material=panel_dark,
                name=f"{name}_{side}_bearing",
            )
    housing.inertial = None

    bin_part = model.part("bin")
    bin_part.visual(
        mesh_from_cadquery(_bin_shape(), "shredder_bin"),
        material=bin_smoke,
        name="bin_shell",
    )
    bin_part.visual(
        Box((0.150, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, -0.007, 0.236)),
        material=handle_dark,
        name="pull_handle",
    )

    model.articulation(
        "housing_to_bin",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=bin_part,
        origin=Origin(xyz=(0.0, -BODY_DEPTH / 2.0, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=BIN_TRAVEL),
    )

    mode_knob = model.part("mode_knob")
    mode_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.024,
                body_style="skirted",
                top_diameter=0.030,
                skirt=KnobSkirt(0.046, 0.004, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "shredder_mode_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_dark,
        name="knob_shell",
    )
    model.articulation(
        "housing_to_mode_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=mode_knob,
        origin=Origin(xyz=(-0.042, PANEL_FACE_Y, 0.356)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    button_specs = (
        ("top_button", top_button_finish, 0.022, 0.010, 0.016, 0.374),
        ("middle_button", middle_button_finish, 0.018, 0.010, 0.015, 0.356),
        ("bottom_button", bottom_button_finish, 0.024, 0.010, 0.014, 0.338),
    )
    for part_name, material, width, depth, height, z_pos in button_specs:
        button = model.part(part_name)
        button.visual(
            Box((width, depth, height)),
            origin=Origin(xyz=(0.0, -depth / 2.0, 0.0)),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(0.052, PANEL_FACE_Y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.003),
        )

    safety_flap = model.part("safety_flap")
    safety_flap.visual(
        Box((0.238, 0.024, 0.002)),
        origin=Origin(xyz=(0.0, -0.012, -0.001)),
        material=panel_dark,
        name="flap_panel",
    )
    safety_flap.visual(
        Cylinder(radius=0.0016, length=0.238),
        origin=Origin(xyz=(0.0, 0.0005, -0.0016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=panel_dark,
        name="hinge_barrel",
    )
    model.articulation(
        "housing_to_safety_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=safety_flap,
        origin=Origin(xyz=(0.0, -0.0095, TOTAL_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=0.0, upper=1.0),
    )

    cutter_positions = (
        ("front_cutter", FRONT_CUTTER_Y, (-0.108, -0.081, -0.054, -0.027, 0.0, 0.027, 0.054, 0.081, 0.108)),
        ("rear_cutter", REAR_CUTTER_Y, (-0.0945, -0.0675, -0.0405, -0.0135, 0.0135, 0.0405, 0.0675, 0.0945)),
    )
    for part_name, y_pos, disc_centers in cutter_positions:
        cutter = model.part(part_name)
        cutter.visual(
            Cylinder(radius=0.0045, length=0.280),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="shaft_core",
        )
        for index, x_pos in enumerate(disc_centers):
            radius = 0.0115 if index % 2 == 0 else 0.0100
            cutter.visual(
                Cylinder(radius=radius, length=0.010),
                origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"cutter_disc_{index}",
            )
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=cutter,
            origin=Origin(xyz=(0.0, y_pos, CUTTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=30.0),
            mimic=Mimic(joint="housing_to_front_cutter", multiplier=-1.0) if part_name == "rear_cutter" else None,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    bin_part = object_model.get_part("bin")
    bin_joint = object_model.get_articulation("housing_to_bin")
    mode_knob = object_model.get_part("mode_knob")
    top_button = object_model.get_part("top_button")
    middle_button = object_model.get_part("middle_button")
    bottom_button = object_model.get_part("bottom_button")
    top_button_joint = object_model.get_articulation("housing_to_top_button")
    middle_button_joint = object_model.get_articulation("housing_to_middle_button")
    bottom_button_joint = object_model.get_articulation("housing_to_bottom_button")
    flap = object_model.get_part("safety_flap")
    flap_joint = object_model.get_articulation("housing_to_safety_flap")
    front_cutter = object_model.get_part("front_cutter")
    rear_cutter = object_model.get_part("rear_cutter")

    ctx.expect_within(
        bin_part,
        housing,
        axes="x",
        inner_elem="bin_shell",
        outer_elem="body_shell",
        margin=0.010,
        name="bin remains laterally inside the housing",
    )

    with ctx.pose({bin_joint: 0.0}):
        ctx.expect_overlap(
            bin_part,
            housing,
            axes="y",
            elem_a="bin_shell",
            elem_b="body_shell",
            min_overlap=0.150,
            name="closed bin remains deeply inserted in the housing",
        )

    with ctx.pose({bin_joint: BIN_TRAVEL}):
        ctx.expect_overlap(
            bin_part,
            housing,
            axes="y",
            elem_a="bin_shell",
            elem_b="body_shell",
            min_overlap=0.060,
            name="extended bin still keeps retained insertion",
        )
        open_pos = ctx.part_world_position(bin_part)

    with ctx.pose({bin_joint: 0.0}):
        closed_pos = ctx.part_world_position(bin_part)

    ctx.check(
        "bin pulls forward when opened",
        closed_pos is not None and open_pos is not None and open_pos[1] < closed_pos[1] - 0.080,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    ctx.expect_contact(
        mode_knob,
        housing,
        elem_a="knob_shell",
        elem_b="control_panel",
        contact_tol=0.0015,
        name="mode knob mounts on the front control panel",
    )

    for button in (top_button, middle_button, bottom_button):
        ctx.expect_contact(
            button,
            housing,
            elem_a="button_cap",
            elem_b="control_panel",
            contact_tol=1e-6,
            name=f"{button.name} sits flush on the front control panel",
        )

    for button, joint in (
        (top_button, top_button_joint),
        (middle_button, middle_button_joint),
        (bottom_button, bottom_button_joint),
    ):
        with ctx.pose({joint: 0.0}):
            rest_position = ctx.part_world_position(button)
        with ctx.pose({joint: 0.003}):
            pressed_position = ctx.part_world_position(button)
        ctx.check(
            f"{button.name} presses inward",
            rest_position is not None and pressed_position is not None and pressed_position[1] > rest_position[1] + 0.0025,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    flap_upper = flap_joint.motion_limits.upper if flap_joint.motion_limits is not None else None
    if flap_upper is not None:
        with ctx.pose({flap_joint: 0.0}):
            flap_closed = ctx.part_element_world_aabb(flap, elem="flap_panel")
        with ctx.pose({flap_joint: flap_upper}):
            flap_open = ctx.part_element_world_aabb(flap, elem="flap_panel")
        ctx.check(
            "safety flap lifts above the paper slot",
            flap_closed is not None
            and flap_open is not None
            and float(flap_open[1][2]) > float(flap_closed[1][2]) + 0.015,
            details=f"closed={flap_closed}, open={flap_open}",
        )

    ctx.expect_origin_distance(
        front_cutter,
        rear_cutter,
        axes="z",
        max_dist=0.002,
        name="cutter shafts share a matched axle height",
    )
    ctx.expect_origin_gap(
        rear_cutter,
        front_cutter,
        axis="y",
        min_gap=0.020,
        max_gap=0.040,
        name="cutter shafts are staggered front to rear under the slot",
    )

    return ctx.report()


object_model = build_object_model()
