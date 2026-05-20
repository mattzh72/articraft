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
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.088
BODY_D = 0.048
BODY_H = 0.170
CORE_W = 0.084
CORE_D = 0.042
CORE_H = 0.166


def _body_core_mesh():
    shape = cq.Workplane("XY").box(CORE_W, CORE_D, CORE_H).edges().fillet(0.004)
    return mesh_from_cadquery(shape, "multimeter_body_core")


def _overmold_mesh():
    outer = cq.Workplane("XY").box(0.096, 0.054, 0.182).edges().fillet(0.007)
    inner = cq.Workplane("XY").box(0.080, 0.070, 0.160).edges().fillet(0.003)
    return mesh_from_cadquery(outer.cut(inner), "multimeter_overmold")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_multimeter")

    shell = model.material("shell", rgba=(0.24, 0.26, 0.29, 1.0))
    overmold = model.material("overmold", rgba=(0.88, 0.74, 0.15, 1.0))
    trim = model.material("trim", rgba=(0.08, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.19, 0.36, 0.30, 0.45))
    jack_red = model.material("jack_red", rgba=(0.65, 0.08, 0.10, 1.0))

    body = model.part("body")
    body.visual(_body_core_mesh(), material=shell, name="core")
    body.visual(_overmold_mesh(), material=overmold, name="overmold")
    body.visual(
        Box((0.056, 0.004, 0.042)),
        origin=Origin(xyz=(0.0, -CORE_D / 2.0 + 0.001, 0.049)),
        material=trim,
        name="display_bezel",
    )
    body.visual(
        Box((0.048, 0.0015, 0.032)),
        origin=Origin(xyz=(0.0, -CORE_D / 2.0 + 0.0012, 0.049)),
        material=glass,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.027, length=0.003),
        origin=Origin(
            xyz=(0.0, -CORE_D / 2.0 + 0.0015, -0.006),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=shell,
        name="dial_plate",
    )
    body.visual(
        Box((0.066, 0.003, 0.036)),
        origin=Origin(xyz=(0.0, -CORE_D / 2.0 + 0.0015, -0.060)),
        material=trim,
        name="input_panel",
    )
    body.visual(
        Box((0.054, 0.004, 0.094)),
        origin=Origin(xyz=(0.0, CORE_D / 2.0 + 0.0005, -0.002)),
        material=shell,
        name="back_landing",
    )
    body.visual(
        Box((0.070, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, -CORE_D / 2.0 - 0.0005, -0.041)),
        material=shell,
        name="button_panel",
    )
    body.visual(
        Box((0.004, 0.020, 0.032)),
        origin=Origin(xyz=(CORE_W / 2.0 + 0.001, 0.0, 0.028)),
        material=shell,
        name="side_switch_plate",
    )
    for index, x_pos in enumerate((-0.019, 0.0, 0.019)):
        body.visual(
            Cylinder(radius=0.0053, length=0.004),
            origin=Origin(
                xyz=(x_pos, -CORE_D / 2.0 + 0.0015, -0.064),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=jack_red if index == 2 else trim,
            name=f"jack_{index}",
        )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.018,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.047, 0.0045, flare=0.06),
                grip=KnobGrip(style="knurled", count=40, depth=0.0008, helix_angle_deg=22.0),
                center=False,
            ),
            "multimeter_selector",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="selector_knob",
    )

    rocker = model.part("power_rocker")
    rocker.visual(
        Cylinder(radius=0.0022, length=0.022),
        origin=Origin(xyz=(0.0022, 0.0, 0.0)),
        material=trim,
        name="pivot_barrel",
    )
    rocker.visual(
        Box((0.0045, 0.016, 0.028)),
        origin=Origin(xyz=(0.0023, 0.0, 0.0)),
        material=jack_red,
        name="rocker_cap",
    )
    rocker.visual(
        Box((0.0018, 0.006, 0.020)),
        origin=Origin(xyz=(0.0048, 0.0032, 0.0)),
        material=jack_red,
        name="rocker_nose",
    )

    button_x_positions = (-0.019, 0.0, 0.019)
    for index, x_pos in enumerate(button_x_positions):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.014, 0.0025, 0.009)),
            origin=Origin(xyz=(0.0, -0.00125, 0.0)),
            material=trim,
            name="button_cap",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, -CORE_D / 2.0 - 0.002, -0.041)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0015,
            ),
        )

    support = model.part("rear_support")
    support.visual(
        Box((0.066, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, 0.005)),
        material=trim,
        name="pivot_bar",
    )
    support.visual(
        Box((0.066, 0.003, 0.098)),
        origin=Origin(xyz=(0.0, 0.0035, 0.055)),
        material=trim,
        name="support_panel",
    )
    support.visual(
        Box((0.048, 0.009, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, 0.030)),
        material=trim,
        name="center_rib",
    )
    support.visual(
        Box((0.056, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.007, 0.094)),
        material=trim,
        name="top_foot",
    )
    for index, x_pos in enumerate((-0.027, 0.027)):
        support.visual(
            Cylinder(radius=0.004, length=0.012),
            origin=Origin(
                xyz=(x_pos, 0.004, 0.004),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=trim,
            name=f"pivot_{index}",
        )

    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0, -CORE_D / 2.0, -0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    model.articulation(
        "body_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=body,
        child=support,
        origin=Origin(xyz=(0.0, CORE_D / 2.0 + 0.0005, -0.076)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.12,
        ),
    )
    model.articulation(
        "body_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(CORE_W / 2.0 + 0.003, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-0.30,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector")
    support = object_model.get_part("rear_support")
    rocker = object_model.get_part("power_rocker")
    selector_joint = object_model.get_articulation("body_to_selector")
    support_joint = object_model.get_articulation("body_to_rear_support")
    rocker_joint = object_model.get_articulation("body_to_power_rocker")

    ctx.expect_contact(
        selector,
        body,
        elem_a="selector_knob",
        elem_b="core",
        name="selector seats on front housing",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="xz",
        min_overlap=0.028,
        elem_a="selector_knob",
        elem_b="core",
        name="selector remains centered on front face",
    )
    ctx.check(
        "selector articulation is continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS
        and selector_joint.motion_limits is not None
        and selector_joint.motion_limits.lower is None
        and selector_joint.motion_limits.upper is None,
        details=str(selector_joint.motion_limits),
    )

    with ctx.pose({support_joint: 0.0}):
        ctx.expect_gap(
            support,
            body,
            axis="y",
            max_gap=0.012,
            max_penetration=0.0,
            positive_elem="support_panel",
            negative_elem="back_landing",
            name="rear support folds nearly flush to the back",
        )
        closed_aabb = ctx.part_element_world_aabb(support, elem="top_foot")

    with ctx.pose({support_joint: 1.0}):
        open_aabb = ctx.part_element_world_aabb(support, elem="top_foot")

    ctx.check(
        "rear support swings outward behind the body",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.03,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    ctx.expect_contact(
        rocker,
        body,
        elem_a="pivot_barrel",
        elem_b="side_switch_plate",
        name="power rocker is mounted on its side pivot plate",
    )
    with ctx.pose({rocker_joint: 0.0}):
        rocker_rest = ctx.part_element_world_aabb(rocker, elem="rocker_nose")
    with ctx.pose({rocker_joint: 0.30}):
        rocker_tipped = ctx.part_element_world_aabb(rocker, elem="rocker_nose")
    ctx.check(
        "power rocker tips on its local side pivot",
        rocker_rest is not None
        and rocker_tipped is not None
        and ((rocker_tipped[0][1] + rocker_tipped[1][1]) * 0.5)
        > ((rocker_rest[0][1] + rocker_rest[1][1]) * 0.5) + 0.001,
        details=f"rest={rocker_rest}, tipped={rocker_tipped}",
    )

    for index in range(3):
        button = object_model.get_part(f"mode_button_{index}")
        button_joint = object_model.get_articulation(f"body_to_mode_button_{index}")
        ctx.expect_contact(
            button,
            body,
            elem_a="button_cap",
            elem_b="button_panel",
            name=f"mode button {index} seats on the front button panel",
        )
        rest_pos = ctx.part_world_position(button)
        limits = button_joint.motion_limits
        pressed_pos = None
        if limits is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"mode button {index} depresses inward independently",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
