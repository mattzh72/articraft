from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_dropper_seatpost")

    black_anodized = model.material("black_anodized", color=(0.01, 0.012, 0.014, 1.0))
    satin_alloy = model.material("satin_alloy", color=(0.72, 0.72, 0.68, 1.0))
    polished_tube = model.material("polished_inner_tube", color=(0.86, 0.88, 0.86, 1.0))
    dark_rubber = model.material("dark_port_liner", color=(0.0, 0.0, 0.0, 1.0))
    steel = model.material("brushed_steel", color=(0.50, 0.52, 0.52, 1.0))

    outer_height = 0.330
    collar_top = 0.355
    stanchion_top = 0.230

    outer_tube = model.part("outer_tube")
    outer_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.01545, 0.000),
            (0.01545, outer_height),
        ],
        inner_profile=[
            (0.01260, 0.000),
            (0.01260, outer_height),
        ],
        segments=64,
    )
    outer_tube.visual(
        mesh_from_geometry(outer_shell, "outer_sleeve"),
        material=black_anodized,
        name="outer_sleeve",
    )

    # A separate knurled collar visual sits around the lip like the threaded
    # service collar on a real mechanical dropper.  The bore is just proud of
    # the moving stanchion and lightly bites into the fixed sleeve visual so the
    # authored lower assembly reads as one connected piece.
    collar = KnobGeometry(
        0.046,
        0.040,
        body_style="cylindrical",
        edge_radius=0.0012,
        grip=KnobGrip(style="knurled", count=42, depth=0.0011, helix_angle_deg=28.0),
        bore=KnobBore(style="round", diameter=0.0295),
    )
    outer_tube.visual(
        mesh_from_geometry(collar, "knurled_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=satin_alloy,
        name="knurled_collar",
    )
    wiper_seal = KnobGeometry(
        0.030,
        0.010,
        body_style="cylindrical",
        edge_radius=0.0007,
        bore=KnobBore(style="round", diameter=0.0216),
    )
    outer_tube.visual(
        mesh_from_geometry(wiper_seal, "wiper_seal"),
        origin=Origin(xyz=(0.0, 0.0, collar_top - 0.002)),
        material=dark_rubber,
        name="wiper_seal",
    )
    outer_tube.visual(
        Cylinder(radius=0.0162, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=satin_alloy,
        name="lower_stop_ring",
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        Cylinder(radius=0.0108, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=polished_tube,
        name="stanchion",
    )
    inner_tube.visual(
        Cylinder(radius=0.0120, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, stanchion_top - 0.005)),
        material=steel,
        name="top_press_ring",
    )

    model.articulation(
        "tube_slide",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, collar_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.120),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Cylinder(radius=0.0160, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=black_anodized,
        name="head_socket",
    )
    saddle_clamp.visual(
        Box((0.068, 0.027, 0.016)),
        origin=Origin(xyz=(-0.030, 0.0, 0.038)),
        material=black_anodized,
        name="setback_bridge",
    )
    saddle_clamp.visual(
        Box((0.082, 0.050, 0.014)),
        origin=Origin(xyz=(-0.064, 0.0, 0.052)),
        material=satin_alloy,
        name="lower_cradle",
    )
    saddle_clamp.visual(
        Box((0.074, 0.042, 0.010)),
        origin=Origin(xyz=(-0.064, 0.0, 0.075)),
        material=satin_alloy,
        name="top_plate",
    )
    for index, x in enumerate((-0.091, -0.037)):
        saddle_clamp.visual(
            Cylinder(radius=0.0040, length=0.034),
            origin=Origin(xyz=(x, 0.0, 0.064)),
            material=steel,
            name=f"clamp_bolt_{index}",
        )
    for index, y in enumerate((-0.017, 0.017)):
        saddle_clamp.visual(
            Cylinder(radius=0.0032, length=0.106),
            origin=Origin(xyz=(-0.064, y, 0.0815), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"rail_groove_{index}",
        )

    # Cable/lever actuation port: an angled housing entry boss on the underside
    # of the setback head, with a dark liner visible in the mouth.
    saddle_clamp.visual(
        Cylinder(radius=0.0070, length=0.026),
        origin=Origin(xyz=(-0.028, -0.024, 0.033), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_anodized,
        name="lever_port_boss",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0044, length=0.004),
        origin=Origin(xyz=(-0.028, -0.038, 0.033), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="lever_port_liner",
    )

    model.articulation(
        "tube_to_clamp",
        ArticulationType.FIXED,
        parent=inner_tube,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, stanchion_top)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    inner_tube = object_model.get_part("inner_tube")
    saddle_clamp = object_model.get_part("saddle_clamp")
    slide = object_model.get_articulation("tube_slide")

    ctx.allow_overlap(
        inner_tube,
        outer_tube,
        elem_a="stanchion",
        elem_b="wiper_seal",
        reason=(
            "The black rubber wiper seal is intentionally modeled with slight "
            "compression around the sliding stanchion."
        ),
    )

    ctx.expect_within(
        inner_tube,
        outer_tube,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="stanchion remains coaxial with sleeve",
    )
    ctx.expect_overlap(
        inner_tube,
        outer_tube,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_sleeve",
        min_overlap=0.120,
        name="collapsed stanchion has deep retained insertion",
    )
    ctx.expect_overlap(
        inner_tube,
        outer_tube,
        axes="z",
        elem_a="stanchion",
        elem_b="wiper_seal",
        min_overlap=0.006,
        name="stanchion passes through compressed wiper seal",
    )
    ctx.expect_contact(
        saddle_clamp,
        inner_tube,
        elem_a="head_socket",
        elem_b="top_press_ring",
        contact_tol=0.001,
        name="saddle head is seated on inner tube",
    )

    rest_pos = ctx.part_world_position(inner_tube)
    with ctx.pose({slide: 0.120}):
        ctx.expect_within(
            inner_tube,
            outer_tube,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="outer_sleeve",
            margin=0.0,
            name="extended stanchion remains coaxial",
        )
        ctx.expect_overlap(
            inner_tube,
            outer_tube,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_sleeve",
            min_overlap=0.040,
            name="extended stanchion stays captured in sleeve",
        )
        extended_pos = ctx.part_world_position(inner_tube)

    ctx.check(
        "prismatic tube travel lifts saddle clamp",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.115,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    cradle_box = ctx.part_element_world_aabb(saddle_clamp, elem="lower_cradle")
    socket_box = ctx.part_element_world_aabb(saddle_clamp, elem="head_socket")
    if cradle_box is not None and socket_box is not None:
        cradle_center_x = 0.5 * (cradle_box[0][0] + cradle_box[1][0])
        socket_center_x = 0.5 * (socket_box[0][0] + socket_box[1][0])
        ctx.check(
            "saddle clamp is visibly set back",
            cradle_center_x < socket_center_x - 0.035,
            details=f"cradle_x={cradle_center_x}, socket_x={socket_center_x}",
        )
    else:
        ctx.fail("saddle clamp is visibly set back", "missing cradle or socket geometry")

    ctx.check(
        "lever actuation port is modeled",
        saddle_clamp.get_visual("lever_port_boss") is not None
        and saddle_clamp.get_visual("lever_port_liner") is not None,
    )

    return ctx.report()


object_model = build_object_model()
