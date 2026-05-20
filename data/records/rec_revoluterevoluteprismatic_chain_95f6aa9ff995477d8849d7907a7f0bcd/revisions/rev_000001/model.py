from __future__ import annotations

import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_sliding_nose_arm")

    painted_steel = model.material("painted_steel", rgba=(0.58, 0.62, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    link_blue = model.material("link_blue", rgba=(0.12, 0.25, 0.72, 1.0))
    link_orange = model.material("link_orange", rgba=(0.95, 0.48, 0.12, 1.0))
    slider_chrome = model.material("slider_chrome", rgba=(0.78, 0.78, 0.74, 1.0))
    nose_red = model.material("nose_red", rgba=(0.85, 0.08, 0.04, 1.0))

    cyl_y = (-math.pi / 2.0, 0.0, 0.0)
    cyl_x = (0.0, math.pi / 2.0, 0.0)

    backplate = model.part("backplate")
    backplate.visual(
        Box((0.28, 0.040, 0.52)),
        origin=Origin(xyz=(0.0, -0.085, 0.0)),
        material=painted_steel,
        name="wall_plate",
    )
    backplate.visual(
        Cylinder(radius=0.064, length=0.051),
        origin=Origin(xyz=(0.0, -0.0395, 0.0), rpy=cyl_y),
        material=dark_steel,
        name="shoulder_standoff",
    )
    backplate.visual(
        Cylinder(radius=0.018, length=0.027),
        origin=Origin(xyz=(0.0, -0.0005, 0.0), rpy=cyl_y),
        material=dark_steel,
        name="shoulder_pin",
    )
    for i, (x, z) in enumerate(
        ((-0.095, 0.185), (0.095, 0.185), (-0.095, -0.185), (0.095, -0.185))
    ):
        backplate.visual(
            Cylinder(radius=0.013, length=0.007),
            origin=Origin(xyz=(x, -0.0615, z), rpy=cyl_y),
            material=dark_steel,
            name=f"mount_screw_{i}",
        )

    proximal = model.part("proximal_link")
    proximal.visual(
        Box((0.30, 0.022, 0.044)),
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        material=link_blue,
        name="proximal_bar",
    )
    proximal.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=link_blue,
        name="shoulder_hub",
    )
    proximal.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(0.36, 0.0, 0.0), rpy=cyl_y),
        material=link_blue,
        name="elbow_hub",
    )
    proximal.visual(
        Cylinder(radius=0.016, length=0.045),
        origin=Origin(xyz=(0.36, 0.035, 0.0), rpy=cyl_y),
        material=dark_steel,
        name="elbow_pin",
    )

    distal = model.part("distal_link")
    distal.visual(
        Cylinder(radius=0.045, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cyl_y),
        material=link_orange,
        name="elbow_hub",
    )
    distal.visual(
        Box((0.34, 0.026, 0.018)),
        origin=Origin(xyz=(0.17, 0.0, 0.021)),
        material=link_orange,
        name="top_guide_rail",
    )
    distal.visual(
        Box((0.34, 0.026, 0.018)),
        origin=Origin(xyz=(0.17, 0.0, -0.021)),
        material=link_orange,
        name="bottom_guide_rail",
    )
    nose = model.part("nose_slider")
    nose.visual(
        Box((0.220, 0.018, 0.024)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=slider_chrome,
        name="slider_shank",
    )
    nose.visual(
        Cylinder(radius=0.015, length=0.035),
        origin=Origin(xyz=(0.1525, 0.0, 0.0), rpy=cyl_x),
        material=nose_red,
        name="rounded_nose",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.95, upper=0.95, effort=45.0, velocity=1.6),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=distal,
        origin=Origin(xyz=(0.36, 0.045, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=35.0, velocity=1.8),
    )
    model.articulation(
        "nose_slide",
        ArticulationType.PRISMATIC,
        parent=distal,
        child=nose,
        origin=Origin(xyz=(0.25, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.12, effort=70.0, velocity=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    backplate = object_model.get_part("backplate")
    proximal = object_model.get_part("proximal_link")
    distal = object_model.get_part("distal_link")
    nose = object_model.get_part("nose_slider")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    slide = object_model.get_articulation("nose_slide")

    ctx.allow_overlap(
        backplate,
        proximal,
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        reason="The shoulder pin is intentionally captured inside the proximal hub bore.",
    )
    ctx.expect_within(
        backplate,
        proximal,
        axes="xz",
        inner_elem="shoulder_pin",
        outer_elem="shoulder_hub",
        margin=0.001,
        name="shoulder pin centered inside proximal hub",
    )
    ctx.expect_overlap(
        backplate,
        proximal,
        axes="y",
        elem_a="shoulder_pin",
        elem_b="shoulder_hub",
        min_overlap=0.020,
        name="shoulder pin passes through proximal hub thickness",
    )

    ctx.allow_overlap(
        proximal,
        distal,
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        reason="The elbow pin is intentionally captured inside the distal hub bore.",
    )
    ctx.expect_within(
        proximal,
        distal,
        axes="xz",
        inner_elem="elbow_pin",
        outer_elem="elbow_hub",
        margin=0.001,
        name="elbow pin centered inside distal hub",
    )
    ctx.expect_overlap(
        proximal,
        distal,
        axes="y",
        elem_a="elbow_pin",
        elem_b="elbow_hub",
        min_overlap=0.020,
        name="elbow pin passes through distal hub thickness",
    )

    ctx.expect_contact(
        nose,
        distal,
        elem_a="slider_shank",
        elem_b="top_guide_rail",
        contact_tol=0.001,
        name="slider bears against upper guide rail",
    )
    ctx.expect_contact(
        nose,
        distal,
        elem_a="slider_shank",
        elem_b="bottom_guide_rail",
        contact_tol=0.001,
        name="slider bears against lower guide rail",
    )
    ctx.expect_overlap(
        nose,
        distal,
        axes="x",
        elem_a="slider_shank",
        elem_b="top_guide_rail",
        min_overlap=0.10,
        name="collapsed slider remains supported in guide",
    )

    rest_pos = ctx.part_world_position(nose)
    with ctx.pose({slide: 0.12}):
        ctx.expect_overlap(
            nose,
            distal,
            axes="x",
            elem_a="slider_shank",
            elem_b="top_guide_rail",
            min_overlap=0.030,
            name="extended slider retains guide engagement",
        )
        extended_pos = ctx.part_world_position(nose)

    ctx.check(
        "prismatic nose extends outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.10,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({shoulder: 0.45, elbow: -0.70, slide: 0.08}):
        posed_pos = ctx.part_world_position(nose)
    ctx.check(
        "revolute shoulder and elbow reposition the nose",
        rest_pos is not None
        and posed_pos is not None
        and abs(posed_pos[2] - rest_pos[2]) > 0.04,
        details=f"rest={rest_pos}, posed={posed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
