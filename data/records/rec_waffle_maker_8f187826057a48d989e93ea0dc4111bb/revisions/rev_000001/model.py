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


BODY_RADIUS = 0.145
BODY_HEIGHT = 0.066
BODY_WALL = 0.004
LOWER_PLATE_TOP = 0.043

CLUSTER_WIDTH = 0.126
CLUSTER_DEPTH = 0.046
CLUSTER_HEIGHT = 0.044
CLUSTER_OVERLAP = 0.012
CLUSTER_CENTER_Y = -(BODY_RADIUS + CLUSTER_DEPTH / 2.0 - CLUSTER_OVERLAP)
CLUSTER_FRONT_Y = CLUSTER_CENTER_Y - CLUSTER_DEPTH / 2.0

HINGE_Y = 0.138
HINGE_Z = 0.066

LID_RADIUS = 0.147
LID_CENTER_Y = -HINGE_Y
LID_OUTER_HEIGHT = 0.056
LID_RIM_HEIGHT = 0.018
LID_PLATE_THICKNESS = 0.012
LID_SPHERE_RADIUS = 0.224
LID_SPHERE_CENTER_Z = LID_OUTER_HEIGHT - LID_SPHERE_RADIUS
LID_INNER_SPHERE_RADIUS = LID_SPHERE_RADIUS - 0.005
LID_INNER_SPHERE_CENTER_Z = LID_OUTER_HEIGHT - 0.006 - LID_INNER_SPHERE_RADIUS

GRID_SPAN = 0.214
GRID_WIDTH = 0.006
GRID_DEPTH = 0.0035
GRID_POSITIONS = (-0.072, -0.048, -0.024, 0.0, 0.024, 0.048, 0.072)

BUTTON_XS = (-0.040, -0.016, 0.008)
BUTTON_Z = 0.026
BUTTON_SIZE = (0.016, 0.006, 0.012)
BUTTON_ORIGIN_Y = CLUSTER_FRONT_Y - 0.0035
BUTTON_TRAVEL = 0.004

KNOB_X = 0.044
KNOB_Z = 0.026
KNOB_ORIGIN_Y = CLUSTER_FRONT_Y
KNOB_RADIUS = 0.017
KNOB_DEPTH = 0.018


def _raised_box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2], centered=(True, True, False))
        .translate(xyz)
    )


def _cut_waffle_pattern(
    shape: cq.Workplane,
    *,
    center_y: float,
    z0: float,
    depth: float,
) -> cq.Workplane:
    for x in GRID_POSITIONS:
        shape = shape.cut(
            _raised_box(
                (GRID_WIDTH, GRID_SPAN, depth + 0.002),
                (x, center_y, z0),
            )
        )
    for local_y in GRID_POSITIONS:
        shape = shape.cut(
            _raised_box(
                (GRID_SPAN, GRID_WIDTH, depth + 0.002),
                (0.0, center_y + local_y, z0),
            )
        )
    return shape


def _build_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").circle(BODY_RADIUS).extrude(BODY_HEIGHT)
    body = body.union(
        cq.Workplane("XY")
        .box(CLUSTER_WIDTH, CLUSTER_DEPTH, CLUSTER_HEIGHT)
        .translate((0.0, CLUSTER_CENTER_Y, CLUSTER_HEIGHT / 2.0))
    )

    body = body.cut(
        cq.Workplane("XY")
        .circle(BODY_RADIUS - BODY_WALL)
        .extrude(BODY_HEIGHT - LOWER_PLATE_TOP)
        .translate((0.0, 0.0, LOWER_PLATE_TOP))
    )
    body = _cut_waffle_pattern(
        body,
        center_y=0.0,
        z0=LOWER_PLATE_TOP - GRID_DEPTH,
        depth=GRID_DEPTH,
    )

    panel_depth = 0.005
    body = body.cut(
        cq.Workplane("XY")
        .box(0.108, panel_depth, 0.028)
        .translate((0.002, CLUSTER_FRONT_Y + panel_depth / 2.0, BUTTON_Z))
    )
    body = body.cut(
        cq.Workplane("XZ")
        .transformed(offset=(0.0, CLUSTER_FRONT_Y, 0.0))
        .center(KNOB_X, KNOB_Z)
        .circle(0.0105)
        .extrude(0.016)
    )
    for x_pos in BUTTON_XS:
        body = body.cut(
            cq.Workplane("XY")
            .box(0.011, 0.014, 0.009)
            .translate((x_pos, CLUSTER_FRONT_Y + 0.005, BUTTON_Z))
        )

    for side in (-1.0, 1.0):
        for z_pos in (0.024, 0.034, 0.044):
            body = body.cut(
                cq.Workplane("XY")
                .box(0.018, 0.052, 0.0045)
                .translate((side * (BODY_RADIUS - 0.004), 0.0, z_pos))
            )

    return body


def _build_lid_shape() -> cq.Workplane:
    lid = (
        cq.Workplane("XY", origin=(0.0, LID_CENTER_Y, 0.0))
        .circle(LID_RADIUS)
        .workplane(offset=0.018)
        .circle(0.142)
        .workplane(offset=0.020)
        .circle(0.118)
        .workplane(offset=0.018)
        .circle(0.090)
        .loft(combine=True)
    )
    lid = lid.cut(
        cq.Workplane("XY")
        .circle(LID_RADIUS - 0.014)
        .extrude(LID_OUTER_HEIGHT - LID_PLATE_THICKNESS)
        .translate((0.0, LID_CENTER_Y, LID_PLATE_THICKNESS))
    )
    lid = _cut_waffle_pattern(
        lid,
        center_y=LID_CENTER_Y,
        z0=0.0,
        depth=GRID_DEPTH,
    )
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="belgian_waffle_maker")

    body_mat = model.material("body_matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    lid_mat = model.material("lid_satin_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    handle_mat = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    button_mat = model.material("button_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    knob_mat = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_mat = model.material("accent_silver", rgba=(0.78, 0.79, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_body_shape(), "waffle_base"),
        material=body_mat,
        name="body_shell",
    )
    base.visual(
        Box((0.132, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, HINGE_Y, 0.041)),
        material=body_mat,
        name="hinge_bridge",
    )
    for index, x_pos in enumerate((-0.050, 0.050)):
        base.visual(
            Box((0.024, 0.022, 0.034)),
            origin=Origin(xyz=(x_pos, HINGE_Y + 0.002, 0.042)),
            material=body_mat,
            name=f"hinge_ear_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shape(), "waffle_lid"),
        material=lid_mat,
        name="lid_shell",
    )
    lid.visual(
        Box((0.124, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, -0.006, 0.010)),
        material=lid_mat,
        name="hinge_pad",
    )
    for index, x_pos in enumerate((-0.040, 0.040)):
        lid.visual(
            Box((0.020, 0.050, 0.026)),
            origin=Origin(xyz=(x_pos, -0.296, 0.015)),
            material=handle_mat,
            name=f"handle_post_{index}",
        )
    lid.visual(
        Box((0.110, 0.032, 0.018)),
        origin=Origin(xyz=(0.0, -0.334, 0.028)),
        material=handle_mat,
        name="handle",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent_mat,
        name="knob_cap",
    )
    knob.visual(
        Box((0.003, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -0.017, 0.009)),
        material=accent_mat,
        name="pointer",
    )

    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box(BUTTON_SIZE),
            origin=Origin(xyz=(0.0, -BUTTON_SIZE[1] / 2.0, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.009, 0.006, 0.009)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=button_mat,
            name="button_neck",
        )
        button.visual(
            Box((0.011, 0.008, 0.009)),
            origin=Origin(xyz=(0.0, 0.008, 0.0)),
            material=button_mat,
            name="button_stem",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_ORIGIN_Y, BUTTON_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(KNOB_X, KNOB_ORIGIN_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("knob")
    hinge = object_model.get_articulation("base_to_lid")
    knob_joint = object_model.get_articulation("base_to_knob")

    ctx.check(
        "lid uses a rear hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE and hinge.axis == (-1.0, 0.0, 0.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.check(
        "thermostat knob rotates continuously",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS and knob_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )

    for index in range(3):
        ctx.allow_overlap(
            base,
            f"button_{index}",
            elem_a="body_shell",
            elem_b="button_stem",
            reason="Each program button is represented with a guided stem captured inside the simplified front control cluster body.",
        )

    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        min_gap=0.0,
        max_gap=0.004,
        name="closed lid seats on the lower housing",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.22,
        name="closed lid covers the circular cooking body",
    )

    closed_handle = ctx.part_element_world_aabb(lid, elem="handle")
    upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    if upper is not None:
        with ctx.pose({hinge: upper}):
            opened_handle = ctx.part_element_world_aabb(lid, elem="handle")
        ctx.check(
            "lid opens upward from the rear hinge",
            closed_handle is not None
            and opened_handle is not None
            and opened_handle[0][2] > closed_handle[0][2] + 0.14,
            details=f"closed={closed_handle}, opened={opened_handle}",
        )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"base_to_button_{index}")
        ctx.check(
            f"button_{index} is prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC and button_joint.axis == (0.0, 1.0, 0.0),
            details=f"type={button_joint.articulation_type}, axis={button_joint.axis}",
        )
        ctx.expect_gap(
            base,
            button,
            axis="y",
            positive_elem="body_shell",
            negative_elem="button_cap",
            min_gap=0.003,
            max_gap=0.0045,
            name=f"button_{index} stands proud of the control cluster",
        )
        upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
        rest_pos = ctx.part_world_position(button)
        if upper is not None:
            with ctx.pose({button_joint: upper}):
                ctx.expect_gap(
                    base,
                    button,
                    axis="y",
                    positive_elem="body_shell",
                    negative_elem="button_cap",
                    max_penetration=0.0006,
                    max_gap=0.001,
                    name=f"button_{index} can be pressed nearly flush",
                )
                pressed_pos = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} presses inward",
                rest_pos is not None
                and pressed_pos is not None
                and pressed_pos[1] > rest_pos[1] + 0.003,
                details=f"rest={rest_pos}, pressed={pressed_pos}",
            )

    ctx.expect_gap(
        base,
        knob,
        axis="y",
        positive_elem="body_shell",
        negative_elem="knob_body",
        max_penetration=1e-5,
        max_gap=0.001,
        name="knob seats against the front control face",
    )

    return ctx.report()


object_model = build_object_model()
