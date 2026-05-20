from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _dovetail_prism_y(length: float, bottom_width: float, top_width: float, height: float) -> MeshGeometry:
    """Trapezoidal prism running along local Y, with its lower face at z=0."""
    y0 = -length / 2.0
    y1 = length / 2.0
    xb = bottom_width / 2.0
    xt = top_width / 2.0
    vertices = [
        (-xb, y0, 0.0),
        (xb, y0, 0.0),
        (xt, y0, height),
        (-xt, y0, height),
        (-xb, y1, 0.0),
        (xb, y1, 0.0),
        (xt, y1, height),
        (-xt, y1, height),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def _dovetail_prism_x(length: float, bottom_width: float, top_width: float, height: float) -> MeshGeometry:
    """Trapezoidal prism running along local X, with its lower face at z=0."""
    x0 = -length / 2.0
    x1 = length / 2.0
    yb = bottom_width / 2.0
    yt = top_width / 2.0
    vertices = [
        (x0, -yb, 0.0),
        (x0, yb, 0.0),
        (x0, yt, height),
        (x0, -yt, height),
        (x1, -yb, 0.0),
        (x1, yb, 0.0),
        (x1, yt, height),
        (x1, -yt, height),
    ]
    faces = [
        (0, 1, 2),
        (0, 2, 3),
        (4, 6, 5),
        (4, 7, 6),
        (0, 4, 5),
        (0, 5, 1),
        (1, 5, 6),
        (1, 6, 2),
        (2, 6, 7),
        (2, 7, 3),
        (3, 7, 4),
        (3, 4, 0),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_slide_milling_vise")

    model.material("painted_cast_iron", rgba=(0.16, 0.20, 0.22, 1.0))
    model.material("dark_cast_iron", rgba=(0.09, 0.10, 0.11, 1.0))
    model.material("machined_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    model.material("blued_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("black_handle", rgba=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.62, 0.36, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material="painted_cast_iron",
        name="base_plate",
    )
    for x_pos, name in [(-0.16, "base_way_0"), (0.16, "base_way_1")]:
        base.visual(
            mesh_from_geometry(_dovetail_prism_y(0.39, 0.060, 0.110, 0.050), name),
            origin=Origin(xyz=(x_pos, 0.0, 0.045)),
            material="machined_steel",
            name=name,
        )
    # Low bolt pads and slots make the root read as a real milling-vise base.
    for x_pos in (-0.24, 0.24):
        for y_pos in (-0.125, 0.125):
            base.visual(
                Box((0.095, 0.055, 0.010)),
                origin=Origin(xyz=(x_pos, y_pos, 0.050)),
                material="dark_cast_iron",
                name=f"clamp_pad_{x_pos}_{y_pos}",
            )
            base.visual(
                Box((0.055, 0.018, 0.004)),
                origin=Origin(xyz=(x_pos, y_pos, 0.056)),
                material="blued_steel",
                name=f"bolt_slot_{x_pos}_{y_pos}",
            )

    cross_slide = model.part("cross_slide")
    for x_pos, name in [(-0.16, "lower_saddle_0"), (0.16, "lower_saddle_1")]:
        cross_slide.visual(
            Box((0.115, 0.305, 0.012)),
            origin=Origin(xyz=(x_pos, 0.0, 0.006)),
            material="dark_cast_iron",
            name=name,
        )
    cross_slide.visual(
        Box((0.49, 0.305, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material="painted_cast_iron",
        name="cross_plate",
    )
    for y_pos, name in [(-0.062, "bed_bar_0"), (0.062, "bed_bar_1")]:
        cross_slide.visual(
            Box((0.43, 0.070, 0.058)),
            origin=Origin(xyz=(-0.015, y_pos, 0.075)),
            material="painted_cast_iron",
            name=name,
        )
    for y_pos, name in [(-0.064, "jaw_way_0"), (0.064, "jaw_way_1")]:
        cross_slide.visual(
            mesh_from_geometry(_dovetail_prism_x(0.39, 0.040, 0.064, 0.030), name),
            origin=Origin(xyz=(-0.025, y_pos, 0.104)),
            material="machined_steel",
            name=name,
        )

    cross_slide.visual(
        Box((0.105, 0.205, 0.130)),
        origin=Origin(xyz=(0.145, 0.0, 0.234)),
        material="painted_cast_iron",
        name="rear_jaw_block",
    )
    cross_slide.visual(
        Box((0.105, 0.205, 0.072)),
        origin=Origin(xyz=(0.145, 0.0, 0.135)),
        material="painted_cast_iron",
        name="rear_jaw_pedestal",
    )
    cross_slide.visual(
        Box((0.012, 0.180, 0.078)),
        origin=Origin(xyz=(0.087, 0.0, 0.242)),
        material="machined_steel",
        name="rear_jaw_face",
    )
    for y_pos in (-0.055, 0.0, 0.055):
        cross_slide.visual(
            Box((0.004, 0.006, 0.070)),
            origin=Origin(xyz=(0.079, y_pos, 0.242)),
            material="blued_steel",
            name=f"rear_face_groove_{y_pos}",
        )

    # A forked front screw bearing leaves a visible clearance hole for the screw.
    for y_pos, name in [(-0.048, "screw_post_0"), (0.048, "screw_post_1")]:
        cross_slide.visual(
            Box((0.032, 0.026, 0.088)),
            origin=Origin(xyz=(-0.242, y_pos, 0.091)),
            material="painted_cast_iron",
            name=name,
        )
    cross_slide.visual(
        Box((0.040, 0.070, 0.042)),
        origin=Origin(xyz=(-0.242, 0.0, 0.083)),
        material="painted_cast_iron",
        name="screw_bearing",
    )
    cross_slide.visual(
        Box((0.032, 0.122, 0.023)),
        origin=Origin(xyz=(-0.242, 0.0, 0.145)),
        material="painted_cast_iron",
        name="screw_bridge",
    )

    leadscrew = model.part("leadscrew")
    leadscrew.visual(
        Cylinder(radius=0.009, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="screw_shaft",
    )
    for i, x_pos in enumerate((-0.150, -0.105, -0.060, -0.015, 0.030, 0.075, 0.120, 0.165)):
        leadscrew.visual(
            Cylinder(radius=0.0105, length=0.004),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="blued_steel",
            name=f"thread_crest_{i}",
        )
    leadscrew.visual(
        Cylinder(radius=0.015, length=0.080),
        origin=Origin(xyz=(-0.230, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="machined_steel",
        name="wheel_hub",
    )
    leadscrew.visual(
        Cylinder(radius=0.048, length=0.015),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="blued_steel",
        name="handwheel",
    )
    leadscrew.visual(
        Cylinder(radius=0.0045, length=0.100),
        origin=Origin(xyz=(-0.275, 0.022, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="wheel_spoke",
    )
    leadscrew.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(-0.275, 0.064, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_handle",
        name="spinner_knob",
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        Box((0.155, 0.182, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material="dark_cast_iron",
        name="jaw_saddle",
    )
    front_jaw.visual(
        Box((0.094, 0.205, 0.128)),
        origin=Origin(xyz=(0.0, 0.0, 0.098)),
        material="painted_cast_iron",
        name="front_jaw_block",
    )
    front_jaw.visual(
        Box((0.012, 0.180, 0.078)),
        origin=Origin(xyz=(0.053, 0.0, 0.108)),
        material="machined_steel",
        name="front_jaw_face",
    )
    for y_pos in (-0.055, 0.0, 0.055):
        front_jaw.visual(
            Box((0.004, 0.006, 0.070)),
            origin=Origin(xyz=(0.060, y_pos, 0.108)),
            material="blued_steel",
            name=f"front_face_groove_{y_pos}",
        )

    model.articulation(
        "cross_travel",
        ArticulationType.PRISMATIC,
        parent=base,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.18, lower=-0.060, upper=0.060),
    )
    model.articulation(
        "leadscrew_turn",
        ArticulationType.REVOLUTE,
        parent=cross_slide,
        child=leadscrew,
        origin=Origin(xyz=(-0.105, 0.0, 0.083)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=8.0),
    )
    model.articulation(
        "jaw_travel",
        ArticulationType.PRISMATIC,
        parent=cross_slide,
        child=front_jaw,
        origin=Origin(xyz=(-0.140, 0.0, 0.134)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.05, lower=0.0, upper=0.080),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    cross_slide = object_model.get_part("cross_slide")
    front_jaw = object_model.get_part("front_jaw")
    cross = object_model.get_articulation("cross_travel")
    screw = object_model.get_articulation("leadscrew_turn")
    jaw = object_model.get_articulation("jaw_travel")

    ctx.allow_overlap(
        cross_slide,
        "leadscrew",
        elem_a="screw_bearing",
        elem_b="screw_shaft",
        reason="The visible leadscrew shaft is intentionally captured in a simplified solid bearing boss.",
    )
    ctx.expect_within(
        "leadscrew",
        cross_slide,
        axes="yz",
        inner_elem="screw_shaft",
        outer_elem="screw_bearing",
        margin=0.001,
        name="leadscrew shaft is centered inside the front bearing boss",
    )
    ctx.expect_overlap(
        "leadscrew",
        cross_slide,
        axes="x",
        elem_a="screw_shaft",
        elem_b="screw_bearing",
        min_overlap=0.030,
        name="leadscrew remains captured through the bearing length",
    )

    ctx.expect_gap(
        cross_slide,
        base,
        axis="z",
        positive_elem="lower_saddle_0",
        negative_elem="base_way_0",
        max_gap=0.003,
        max_penetration=0.0,
        name="cross slide is seated just above its dovetail way",
    )
    ctx.expect_overlap(
        cross_slide,
        base,
        axes="y",
        elem_a="lower_saddle_0",
        elem_b="base_way_0",
        min_overlap=0.18,
        name="lower saddle remains engaged along cross way",
    )
    ctx.expect_gap(
        front_jaw,
        cross_slide,
        axis="z",
        positive_elem="jaw_saddle",
        negative_elem="jaw_way_0",
        max_gap=0.003,
        max_penetration=0.0,
        name="moving jaw saddle rides on the longitudinal way",
    )
    ctx.expect_overlap(
        front_jaw,
        cross_slide,
        axes="yz",
        elem_a="front_jaw_face",
        elem_b="rear_jaw_face",
        min_overlap=0.070,
        name="opposing jaw plates align across vise width",
    )

    rest_cross_pos = ctx.part_world_position(cross_slide)
    with ctx.pose({cross: 0.050}):
        moved_cross_pos = ctx.part_world_position(cross_slide)
    ctx.check(
        "cross slide moves perpendicular to jaw travel",
        rest_cross_pos is not None
        and moved_cross_pos is not None
        and moved_cross_pos[1] > rest_cross_pos[1] + 0.045,
        details=f"rest={rest_cross_pos}, moved={moved_cross_pos}",
    )

    rest_jaw_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({jaw: 0.080, screw: 8.0}):
        driven_jaw_pos = ctx.part_world_position(front_jaw)
        ctx.expect_gap(
            cross_slide,
            front_jaw,
            axis="x",
            positive_elem="rear_jaw_face",
            negative_elem="front_jaw_face",
            min_gap=0.035,
            max_gap=0.095,
            name="leadscrew-driven jaw closes toward rear jaw without collision",
        )
    ctx.check(
        "jaw slide moves along the leadscrew axis",
        rest_jaw_pos is not None
        and driven_jaw_pos is not None
        and driven_jaw_pos[0] > rest_jaw_pos[0] + 0.070,
        details=f"rest={rest_jaw_pos}, driven={driven_jaw_pos}",
    )
    ctx.check(
        "central leadscrew is a revolute x-axis drive",
        tuple(round(v, 3) for v in screw.axis) == (1.0, 0.0, 0.0)
        and screw.motion_limits is not None
        and screw.motion_limits.upper is not None
        and screw.motion_limits.upper >= 8.0,
        details=f"axis={screw.axis}, limits={screw.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
