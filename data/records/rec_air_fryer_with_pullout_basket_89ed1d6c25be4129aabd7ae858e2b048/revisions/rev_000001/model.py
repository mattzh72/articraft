from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _loft_ellipses(sections: list[tuple[float, float, float]]):
    z0, rx0, ry0 = sections[0]
    wp = cq.Workplane("XY").workplane(offset=z0).ellipse(rx0, ry0)
    prev_z = z0
    for z, rx, ry in sections[1:]:
        wp = wp.workplane(offset=z - prev_z).ellipse(rx, ry)
        prev_z = z
    return wp.loft(combine=True)


def _body_shell_shape():
    outer = _loft_ellipses(
        [
            (0.000, 0.128, 0.132),
            (0.085, 0.157, 0.145),
            (0.205, 0.150, 0.140),
            (0.305, 0.108, 0.112),
        ]
    )
    inner = _loft_ellipses(
        [
            (0.015, 0.117, 0.121),
            (0.090, 0.145, 0.132),
            (0.205, 0.138, 0.128),
            (0.287, 0.096, 0.100),
        ]
    )
    shell = outer.cut(inner)
    drawer_opening = cq.Workplane("XY").box(0.220, 0.218, 0.126).translate((0.180, 0.0, 0.082))
    drawer_pocket = cq.Workplane("XY").box(0.186, 0.224, 0.142).translate((0.085, 0.0, 0.082))
    return shell.cut(drawer_opening).cut(drawer_pocket)


def _drawer_shell_shape():
    outer = cq.Workplane("XY").box(0.184, 0.212, 0.048).translate((-0.072, 0.0, 0.024))
    inner = cq.Workplane("XY").box(0.160, 0.188, 0.038).translate((-0.072, 0.0, 0.025))
    shell = outer.cut(inner)
    top_opening = cq.Workplane("XY").box(0.200, 0.194, 0.028).translate((-0.072, 0.0, 0.038))
    front_panel = cq.Workplane("XY").box(0.030, 0.226, 0.128).translate((0.022, 0.0, 0.064))
    return shell.cut(top_opening).union(front_panel)


def _union_workplanes(items: list):
    merged = items[0]
    for item in items[1:]:
        merged = merged.union(item)
    return merged


def _basket_shell_shape():
    outer = cq.Workplane("XY").box(0.158, 0.188, 0.096).translate((-0.072, 0.0, 0.048))
    inner = cq.Workplane("XY").box(0.146, 0.176, 0.086).translate((-0.072, 0.0, 0.050))
    shell = outer.cut(inner)
    top_opening = cq.Workplane("XY").box(0.170, 0.180, 0.055).translate((-0.072, 0.0, 0.077))
    shell = shell.cut(top_opening)

    perforation_cutters = []
    for x in (-0.120, -0.090, -0.060, -0.030):
        for y in (-0.054, -0.018, 0.018, 0.054):
            perforation_cutters.append(
                cq.Workplane("XY").box(0.014, 0.014, 0.020).translate((x, y, 0.008))
            )
    for x in (-0.110, -0.072, -0.034):
        for z in (0.026, 0.044):
            perforation_cutters.append(
                cq.Workplane("XY").box(0.022, 0.020, 0.010).translate((x, 0.090, z))
            )
            perforation_cutters.append(
                cq.Workplane("XY").box(0.022, 0.020, 0.010).translate((x, -0.090, z))
            )
    for y in (-0.048, 0.0, 0.048):
        perforation_cutters.append(
            cq.Workplane("XY").box(0.018, 0.026, 0.012).translate((-0.143, y, 0.036))
        )

    return shell.cut(_union_workplanes(perforation_cutters))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_round_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.21, 0.22, 0.23, 1.0))
    control_finish = model.material("control_finish", rgba=(0.08, 0.08, 0.09, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.30, 0.31, 0.33, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.74, 0.76, 0.79, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "air_fryer_body_shell"),
        material=body_finish,
        name="shell",
    )
    body.visual(
        Box((0.236, 0.212, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=body_finish,
        name="base_plinth",
    )
    body.visual(
        Box((0.010, 0.118, 0.132)),
        origin=Origin(xyz=(0.156, 0.0, 0.205)),
        material=control_finish,
        name="control_strip",
    )
    body.visual(
        Box((0.128, 0.030, 0.022)),
        origin=Origin(xyz=(0.030, 0.118, 0.053)),
        material=body_finish,
        name="guide_0",
    )
    body.visual(
        Box((0.128, 0.030, 0.022)),
        origin=Origin(xyz=(0.030, -0.118, 0.053)),
        material=body_finish,
        name="guide_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shell_shape(), "air_fryer_drawer_shell"),
        material=drawer_finish,
        name="outer_shell",
    )
    drawer.visual(
        Box((0.160, 0.018, 0.012)),
        origin=Origin(xyz=(-0.073, 0.100, 0.046)),
        material=drawer_finish,
        name="runner_0",
    )
    drawer.visual(
        Box((0.160, 0.018, 0.012)),
        origin=Origin(xyz=(-0.073, -0.100, 0.046)),
        material=drawer_finish,
        name="runner_1",
    )
    drawer.visual(
        Box((0.060, 0.094, 0.022)),
        origin=Origin(xyz=(0.066, 0.0, 0.058)),
        material=control_finish,
        name="handle_grip",
    )
    drawer.visual(
        Box((0.020, 0.026, 0.030)),
        origin=Origin(xyz=(0.036, 0.030, 0.056)),
        material=control_finish,
        name="handle_post_0",
    )
    drawer.visual(
        Box((0.020, 0.026, 0.030)),
        origin=Origin(xyz=(0.036, -0.030, 0.056)),
        material=control_finish,
        name="handle_post_1",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shell_shape(), "air_fryer_basket_shell"),
        material=basket_finish,
        name="basket_shell",
    )
    basket.visual(
        Box((0.118, 0.040, 0.006)),
        origin=Origin(xyz=(-0.074, 0.074, 0.018)),
        material=basket_finish,
        name="plate_ledge_0",
    )
    basket.visual(
        Box((0.118, 0.040, 0.006)),
        origin=Origin(xyz=(-0.074, -0.074, 0.018)),
        material=basket_finish,
        name="plate_ledge_1",
    )

    crisping_plate = model.part("crisping_plate")
    crisping_plate.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.124, 0.150),
                0.0025,
                hole_diameter=0.0055,
                pitch=(0.014, 0.014),
                frame=0.010,
                corner_radius=0.006,
                stagger=True,
            ),
            "air_fryer_crisping_plate",
        ),
        material=plate_finish,
        name="plate_panel",
    )
    for row, x in enumerate((-0.042, 0.042)):
        for col, y in enumerate((-0.052, 0.052)):
            crisping_plate.visual(
                Box((0.010, 0.010, 0.006)),
                origin=Origin(xyz=(x, y, -0.00425)),
                material=plate_finish,
                name=f"plate_foot_{row}_{col}",
            )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.024,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.050, 0.005, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "air_fryer_timer_knob",
        ),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=control_finish,
        name="knob_shell",
    )

    button_parts = []
    for index in range(3):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.006, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=control_finish,
            name="button_stem",
        )
        button.visual(
            Box((0.012, 0.024, 0.014)),
            origin=Origin(xyz=(0.006, 0.0, 0.0)),
            material=control_finish,
            name="button_cap",
        )
        button_parts.append(button)

    latch = model.part("release_latch")
    latch.visual(
        Box((0.030, 0.034, 0.008)),
        origin=Origin(xyz=(0.015, 0.0, 0.004)),
        material=control_finish,
        name="latch_slider",
    )

    drawer_slide = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.110, 0.0, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.085),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(-0.008, 0.0, 0.016)),
    )
    model.articulation(
        "basket_to_crisping_plate",
        ArticulationType.FIXED,
        parent=basket,
        child=crisping_plate,
        origin=Origin(xyz=(-0.072, 0.0, 0.022)),
    )
    model.articulation(
        "body_to_timer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(0.161, 0.0, 0.232)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    button_y = (-0.036, 0.0, 0.036)
    for index, (button, y_pos) in enumerate(zip(button_parts, button_y)):
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.164, y_pos, 0.168)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=0.0025),
        )

    model.articulation(
        "drawer_to_release_latch",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=latch,
        origin=Origin(xyz=(0.048, 0.0, 0.069)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    crisping_plate = object_model.get_part("crisping_plate")
    latch = object_model.get_part("release_latch")
    timer_knob = object_model.get_part("timer_knob")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    knob_spin = object_model.get_articulation("body_to_timer_knob")
    latch_slide = object_model.get_articulation("drawer_to_release_latch")
    button_joints = [
        object_model.get_articulation(f"body_to_mode_button_{index}") for index in range(3)
    ]
    button_parts = [object_model.get_part(f"mode_button_{index}") for index in range(3)]

    ctx.check(
        "drawer_is_prismatic",
        drawer_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={drawer_slide.articulation_type!r}",
    )
    ctx.check(
        "timer_knob_is_continuous",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type!r}",
    )
    for index, joint in enumerate(button_joints):
        ctx.check(
            f"mode_button_{index}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type!r}",
        )

    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        inner_elem="basket_shell",
        outer_elem="outer_shell",
        margin=0.008,
        name="basket stays nested within drawer",
    )
    ctx.expect_within(
        crisping_plate,
        basket,
        axes="xy",
        inner_elem="plate_panel",
        outer_elem="basket_shell",
        margin=0.006,
        name="crisping plate stays inside the basket cavity",
    )
    ctx.allow_overlap(
        drawer,
        basket,
        elem_a="outer_shell",
        elem_b="basket_shell",
        reason="The carrier drawer shell is simplified while visually representing the inner basket retained inside it.",
    )
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="guide_0",
        elem_b="outer_shell",
        reason="The hidden body guide is represented as a simple rail that embeds slightly into the drawer shell proxy.",
    )
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="guide_1",
        elem_b="outer_shell",
        reason="The hidden body guide is represented as a simple rail that embeds slightly into the drawer shell proxy.",
    )

    drawer_rest = ctx.part_world_position(drawer)
    drawer_extended = None
    if drawer_slide.motion_limits is not None and drawer_slide.motion_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="yz",
                elem_a="outer_shell",
                elem_b="shell",
                min_overlap=0.100,
                name="drawer remains aligned with the body opening",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="runner_0",
                elem_b="guide_0",
                min_overlap=0.040,
                name="upper drawer guide retains insertion",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="runner_1",
                elem_b="guide_1",
                min_overlap=0.040,
                name="lower drawer guide retains insertion",
            )
            drawer_extended = ctx.part_world_position(drawer)

    ctx.check(
        "drawer_extends_forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.060,
        details=f"rest={drawer_rest!r}, extended={drawer_extended!r}",
    )

    latch_rest = ctx.part_world_position(latch)
    latch_extended = None
    if latch_slide.motion_limits is not None and latch_slide.motion_limits.upper is not None:
        with ctx.pose({latch_slide: latch_slide.motion_limits.upper}):
            latch_extended = ctx.part_world_position(latch)
    ctx.check(
        "release_latch_slides_forward",
        latch_rest is not None
        and latch_extended is not None
        and latch_extended[0] > latch_rest[0] + 0.008,
        details=f"rest={latch_rest!r}, extended={latch_extended!r}",
    )

    for index, (joint, button) in enumerate(zip(button_joints, button_parts)):
        rest = ctx.part_world_position(button)
        pressed = None
        if joint.motion_limits is not None and joint.motion_limits.upper is not None:
            with ctx.pose({joint: joint.motion_limits.upper}):
                pressed = ctx.part_world_position(button)
        ctx.check(
            f"mode_button_{index}_presses_inward",
            rest is not None and pressed is not None and pressed[0] < rest[0] - 0.0015,
            details=f"rest={rest!r}, pressed={pressed!r}",
        )

    ctx.check(
        "timer_knob_faces_forward",
        tuple(float(v) for v in knob_spin.axis) == (1.0, 0.0, 0.0),
        details=f"axis={knob_spin.axis!r}",
    )
    ctx.check("timer_knob_part_present", timer_knob is not None, details="Timer knob part missing.")

    return ctx.report()


object_model = build_object_model()
