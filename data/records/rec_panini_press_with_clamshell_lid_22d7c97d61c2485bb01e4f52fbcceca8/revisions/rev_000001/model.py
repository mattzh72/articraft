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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _body_shell() -> object:
    profile = [
        (-0.230, 0.000),
        (0.230, 0.000),
        (0.230, 0.084),
        (0.160, 0.088),
        (-0.040, 0.091),
        (-0.155, 0.094),
        (-0.230, 0.095),
    ]
    body = cq.Workplane("XZ").polyline(profile).close().extrude(0.200, both=True)
    timer_recess = (
        cq.Workplane("XY")
        .box(0.034, 0.112, 0.068, centered=(True, True, True))
        .translate((0.213, 0.000, 0.055))
    )
    body = body.cut(timer_recess)
    body = body.edges("|Y").fillet(0.008)
    return body


def _platen_shell() -> object:
    profile = [
        (-0.016, -0.030),
        (0.360, -0.030),
        (0.360, 0.038),
        (0.330, 0.066),
        (0.155, 0.078),
        (0.025, 0.072),
        (-0.016, 0.038),
    ]
    shell = cq.Workplane("XZ").polyline(profile).close().extrude(0.170, both=True)
    shell = shell.edges("|Y").fillet(0.008)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_contact_grill")

    housing_metal = model.material("housing_metal", rgba=(0.69, 0.70, 0.72, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.17, 0.17, 0.18, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.11, 0.11, 0.12, 1.0))
    control_dark = model.material("control_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell"),
        material=housing_metal,
        name="housing",
    )
    body.visual(
        Box((0.338, 0.286, 0.018)),
        origin=Origin(xyz=(0.015, 0.000, 0.087)),
        material=dark_cast,
        name="lower_plate",
    )
    body.visual(
        Box((0.004, 0.096, 0.050)),
        origin=Origin(xyz=(0.198, 0.000, 0.055)),
        material=dark_cast,
        name="timer_panel",
    )
    for side in (-1.0, 1.0):
        body.visual(
            Box((0.042, 0.040, 0.056)),
            origin=Origin(xyz=(-0.205, side * 0.195, 0.120)),
            material=housing_metal,
            name=f"hinge_bracket_{0 if side < 0.0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.014, length=0.038),
            origin=Origin(
                xyz=(-0.188, side * 0.193, 0.132),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=housing_metal,
            name=f"hinge_barrel_{0 if side < 0.0 else 1}",
        )
        body.visual(
            Box((0.048, 0.050, 0.012)),
            origin=Origin(xyz=((0.172), side * 0.150, 0.006)),
            material=dark_cast,
            name=f"foot_{0 if side < 0.0 else 1}",
        )

    platen = model.part("platen")
    platen.visual(
        mesh_from_cadquery(_platen_shell(), "platen_shell"),
        material=housing_metal,
        name="shell",
    )
    platen.visual(
        Box((0.336, 0.286, 0.018)),
        origin=Origin(xyz=(0.182, 0.000, -0.027)),
        material=dark_cast,
        name="upper_plate",
    )
    for side in (-1.0, 1.0):
        platen.visual(
            Box((0.040, 0.028, 0.060)),
            origin=Origin(xyz=(0.012, side * 0.143, 0.018)),
            material=housing_metal,
            name=f"support_arm_{0 if side < 0.0 else 1}",
        )
        platen.visual(
            Cylinder(radius=0.013, length=0.044),
            origin=Origin(
                xyz=(0.000, side * 0.148, 0.000),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=housing_metal,
            name=f"hinge_eye_{0 if side < 0.0 else 1}",
        )
        platen.visual(
            Cylinder(radius=0.0105, length=0.050),
            origin=Origin(xyz=(0.292, side * 0.104, 0.074)),
            material=handle_dark,
            name=f"handle_post_{0 if side < 0.0 else 1}",
        )
    platen.visual(
        Cylinder(radius=0.012, length=0.250),
        origin=Origin(
            xyz=(0.304, 0.000, 0.101),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_dark,
        name="handle_bar",
    )

    browning_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.022,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.048, 0.004, flare=0.05),
            grip=KnobGrip(style="fluted", count=14, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            center=False,
        ),
        "browning_knob",
    )
    browning_knob = model.part("browning_knob")
    browning_knob.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.000, 0.005, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="browning_shaft",
    )
    browning_knob.visual(
        browning_knob_mesh,
        origin=Origin(xyz=(0.000, 0.008, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="browning_body",
    )

    timer_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.054,
            0.024,
            body_style="skirted",
            top_diameter=0.044,
            skirt=KnobSkirt(0.061, 0.005, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        "timer_knob",
    )
    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.006, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_dark,
        name="timer_shaft",
    )
    timer_knob.visual(
        timer_knob_mesh,
        origin=Origin(xyz=(0.008, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_dark,
        name="timer_body",
    )

    model.articulation(
        "body_to_platen",
        ArticulationType.REVOLUTE,
        parent=body,
        child=platen,
        origin=Origin(xyz=(-0.188, 0.000, 0.132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "body_to_browning_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=browning_knob,
        origin=Origin(xyz=(0.110, 0.200, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=10.0),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(0.198, 0.000, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    platen = object_model.get_part("platen")
    browning_knob = object_model.get_part("browning_knob")
    timer_knob = object_model.get_part("timer_knob")
    hinge = object_model.get_articulation("body_to_platen")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            platen,
            body,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            max_gap=0.0015,
            max_penetration=0.0,
            name="grill plates meet cleanly when closed",
        )
        ctx.expect_overlap(
            platen,
            body,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.260,
            name="upper and lower plates align in plan",
        )

    rest_aabb = ctx.part_element_world_aabb(platen, elem="handle_bar")
    upper_limit = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    if upper_limit is not None:
        with ctx.pose({hinge: upper_limit}):
            opened_aabb = ctx.part_element_world_aabb(platen, elem="handle_bar")
            ctx.expect_gap(
                platen,
                body,
                axis="z",
                positive_elem="handle_bar",
                negative_elem="lower_plate",
                min_gap=0.120,
                name="handle rises clear of the base when opened",
            )
        ctx.check(
            "platen opens upward",
            rest_aabb is not None
            and opened_aabb is not None
            and opened_aabb[0][2] > rest_aabb[0][2] + 0.090,
            details=f"rest={rest_aabb}, opened={opened_aabb}",
        )

    body_aabb = ctx.part_world_aabb(body)
    timer_origin = ctx.part_world_position(timer_knob)
    timer_body_aabb = ctx.part_element_world_aabb(timer_knob, elem="timer_body")
    ctx.check(
        "timer knob sits in a true front recess",
        body_aabb is not None
        and timer_origin is not None
        and timer_body_aabb is not None
        and timer_origin[0] < body_aabb[1][0] - 0.020
        and timer_body_aabb[1][0] <= body_aabb[1][0] + 0.008,
        details=f"body={body_aabb}, timer_origin={timer_origin}, timer_body={timer_body_aabb}",
    )

    browning_origin = ctx.part_world_position(browning_knob)
    housing_aabb = ctx.part_element_world_aabb(body, elem="housing")
    ctx.check(
        "browning knob mounts on the body side",
        housing_aabb is not None
        and browning_origin is not None
        and browning_origin[1] >= housing_aabb[1][1] - 0.002,
        details=f"housing={housing_aabb}, browning_origin={browning_origin}",
    )

    return ctx.report()


object_model = build_object_model()
