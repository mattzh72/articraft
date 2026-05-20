from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _base_plate_mesh() -> object:
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.360, 0.300, 0.026, corner_segments=10),
            0.016,
            cap=True,
            closed=True,
        ),
        "stand_base_plate",
    )


def _lower_sleeve_mesh() -> object:
    outer_profile = [
        (0.026, 0.016),
        (0.026, 0.594),
        (0.031, 0.594),
        (0.031, 0.666),
    ]
    inner_profile = [
        (0.0215, 0.016),
        (0.0215, 0.666),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "stand_lower_sleeve",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_orchestra_stand")

    satin_black = model.material("satin_black", rgba=(0.15, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.34, 0.35, 0.37, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(_base_plate_mesh(), material=satin_black, name="base_plate")
    base.visual(
        Box((0.076, 0.072, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_black,
        name="tube_plinth",
    )
    base.visual(
        _lower_sleeve_mesh(),
        material=graphite,
        name="lower_sleeve",
    )
    base.visual(
        Box((0.020, 0.068, 0.062)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=satin_black,
        name="rear_gusset",
    )
    base.visual(
        Cylinder(radius=0.0085, length=0.022),
        origin=Origin(
            xyz=(0.040, 0.0, 0.628),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="clamp_boss",
    )
    base.visual(
        Box((0.016, 0.020, 0.034)),
        origin=Origin(xyz=(-0.032, 0.0, 0.628)),
        material=graphite,
        name="collar_backing",
    )
    for index, x_sign in enumerate((-1.0, 1.0)):
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.145 * x_sign, 0.110, 0.003)),
            material=rubber,
            name=f"front_pad_{index}",
        )
        base.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.145 * x_sign, -0.110, 0.003)),
            material=rubber,
            name=f"rear_pad_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.360, 0.300, 0.682)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.341)),
    )

    upper_post = model.part("upper_post")
    upper_post.visual(
        Cylinder(radius=0.019, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=dark_grey,
        name="inner_tube",
    )
    upper_post.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=satin_black,
        name="height_stop_ring",
    )
    upper_post.visual(
        Box((0.060, 0.048, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.692)),
        material=satin_black,
        name="head_block",
    )
    upper_post.visual(
        Box((0.014, 0.022, 0.046)),
        origin=Origin(xyz=(-0.036, -0.008, 0.731)),
        material=satin_black,
        name="left_bracket_cheek",
    )
    upper_post.visual(
        Box((0.014, 0.022, 0.046)),
        origin=Origin(xyz=(0.036, -0.008, 0.731)),
        material=satin_black,
        name="right_bracket_cheek",
    )
    upper_post.visual(
        Box((0.078, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.011, 0.711)),
        material=satin_black,
        name="hinge_bridge",
    )
    upper_post.inertial = Inertial.from_geometry(
        Box((0.090, 0.050, 0.930)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
    )

    model.articulation(
        "base_to_upper_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_post,
        origin=Origin(xyz=(0.0, 0.0, 0.666)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=85.0,
            velocity=0.18,
            lower=0.0,
            upper=0.160,
        ),
    )

    desk = model.part("desk")
    desk.visual(
        Cylinder(radius=0.0064, length=0.058),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="center_hinge_barrel",
    )
    desk.visual(
        Box((0.420, 0.008, 0.290)),
        origin=Origin(xyz=(0.0, -0.004, 0.182)),
        material=graphite,
        name="back_panel",
    )
    desk.visual(
        Box((0.014, 0.044, 0.290)),
        origin=Origin(xyz=(-0.203, 0.014, 0.182)),
        material=graphite,
        name="left_flange",
    )
    desk.visual(
        Box((0.014, 0.044, 0.290)),
        origin=Origin(xyz=(0.203, 0.014, 0.182)),
        material=graphite,
        name="right_flange",
    )
    desk.visual(
        Box((0.126, 0.024, 0.018)),
        origin=Origin(xyz=(-0.129, 0.010, 0.028)),
        material=satin_black,
        name="left_bottom_rail",
    )
    desk.visual(
        Box((0.126, 0.024, 0.018)),
        origin=Origin(xyz=(0.129, 0.010, 0.028)),
        material=satin_black,
        name="right_bottom_rail",
    )
    desk.visual(
        Box((0.384, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, 0.040, 0.022)),
        material=graphite,
        name="score_shelf",
    )
    desk.visual(
        Box((0.360, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.066, 0.026)),
        material=graphite,
        name="music_lip",
    )
    desk.visual(
        Box((0.028, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, -0.001, 0.026)),
        material=satin_black,
        name="center_web",
    )
    desk.visual(
        Box((0.030, 0.024, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, 0.016)),
        material=satin_black,
        name="center_brace",
    )
    desk.visual(
        Box((0.016, 0.014, 0.084)),
        origin=Origin(xyz=(-0.110, 0.000, 0.042)),
        material=satin_black,
        name="left_stiffener",
    )
    desk.visual(
        Box((0.016, 0.014, 0.084)),
        origin=Origin(xyz=(0.110, 0.000, 0.042)),
        material=satin_black,
        name="right_stiffener",
    )
    desk.visual(
        Box((0.400, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.004, 0.312)),
        material=satin_black,
        name="top_rail",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.430, 0.080, 0.330)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.014, 0.165)),
    )

    model.articulation(
        "upper_post_to_desk",
        ArticulationType.REVOLUTE,
        parent=upper_post,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.724), rpy=(0.250, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.380,
            upper=0.320,
        ),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.0045, length=0.022),
        origin=Origin(
            xyz=(0.011, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_grey,
        name="threaded_shaft",
    )
    clamp_knob.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(
            xyz=(0.028, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="knob_core",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        clamp_knob.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(0.028, 0.012 * math.cos(angle), 0.012 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_black,
            name=f"grip_lobe_{index}",
        )
    clamp_knob.visual(
        Box((0.006, 0.003, 0.010)),
        origin=Origin(xyz=(0.031, 0.0, 0.017)),
        material=rubber,
        name="knob_indicator",
    )
    clamp_knob.inertial = Inertial.from_geometry(
        Box((0.050, 0.035, 0.035)),
        mass=0.12,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=clamp_knob,
        origin=Origin(xyz=(0.051, 0.0, 0.628)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    upper_post = object_model.get_part("upper_post")
    desk = object_model.get_part("desk")
    clamp_knob = object_model.get_part("clamp_knob")

    slide = object_model.get_articulation("base_to_upper_post")
    tilt = object_model.get_articulation("upper_post_to_desk")
    knob_spin = object_model.get_articulation("base_to_clamp_knob")

    slide_limits = slide.motion_limits
    tilt_limits = tilt.motion_limits

    ctx.check(
        "all primary parts exist",
        all(part is not None for part in (base, upper_post, desk, clamp_knob)),
    )
    ctx.check(
        "post slides vertically",
        slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "desk hinge is horizontal",
        tilt.axis == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.check(
        "clamp knob spins on a horizontal shaft",
        knob_spin.axis == (1.0, 0.0, 0.0),
        details=f"axis={knob_spin.axis}",
    )

    with ctx.pose({slide: 0.0, tilt: 0.0}):
        ctx.expect_contact(
            upper_post,
            desk,
            elem_a="left_bracket_cheek",
            elem_b="center_hinge_barrel",
            name="left hinge cheek contacts the desk barrel",
        )
        ctx.expect_contact(
            upper_post,
            desk,
            elem_a="right_bracket_cheek",
            elem_b="center_hinge_barrel",
            name="right hinge cheek contacts the desk barrel",
        )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            upper_post,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_sleeve",
            margin=0.003,
            name="inner tube stays centered in the lower sleeve at rest",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_sleeve",
            min_overlap=0.20,
            name="inner tube has deep retained insertion at rest",
        )

    rest_post_pos = ctx.part_world_position(upper_post)
    with ctx.pose({slide: slide_limits.upper if slide_limits is not None else 0.16}):
        ctx.expect_within(
            upper_post,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_sleeve",
            margin=0.003,
            name="extended inner tube stays centered in the lower sleeve",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_sleeve",
            min_overlap=0.05,
            name="extended inner tube keeps retained insertion",
        )
        extended_post_pos = ctx.part_world_position(upper_post)

    ctx.check(
        "upper post extends upward",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.10,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    with ctx.pose({tilt: tilt_limits.lower if tilt_limits is not None else -0.3}):
        forward_panel_aabb = ctx.part_element_world_aabb(desk, elem="back_panel")
    with ctx.pose({tilt: tilt_limits.upper if tilt_limits is not None else 0.3}):
        back_panel_aabb = ctx.part_element_world_aabb(desk, elem="back_panel")

    forward_panel_center_y = None
    back_panel_center_y = None
    if forward_panel_aabb is not None:
        forward_panel_center_y = (
            forward_panel_aabb[0][1] + forward_panel_aabb[1][1]
        ) / 2.0
    if back_panel_aabb is not None:
        back_panel_center_y = (back_panel_aabb[0][1] + back_panel_aabb[1][1]) / 2.0
    ctx.check(
        "desk tilt moves the score panel fore and aft",
        forward_panel_center_y is not None
        and back_panel_center_y is not None
        and forward_panel_center_y > back_panel_center_y + 0.05,
        details=f"forward_y={forward_panel_center_y}, back_y={back_panel_center_y}",
    )

    with ctx.pose({knob_spin: 0.0}):
        knob_indicator_rest = ctx.part_element_world_aabb(clamp_knob, elem="knob_indicator")
    with ctx.pose({knob_spin: math.pi / 2.0}):
        knob_indicator_quarter_turn = ctx.part_element_world_aabb(
            clamp_knob,
            elem="knob_indicator",
        )

    rest_indicator_center = None
    turned_indicator_center = None
    if knob_indicator_rest is not None:
        rest_indicator_center = tuple(
            (knob_indicator_rest[0][axis] + knob_indicator_rest[1][axis]) / 2.0
            for axis in range(3)
        )
    if knob_indicator_quarter_turn is not None:
        turned_indicator_center = tuple(
            (
                knob_indicator_quarter_turn[0][axis]
                + knob_indicator_quarter_turn[1][axis]
            )
            / 2.0
            for axis in range(3)
        )
    ctx.check(
        "clamp knob indicator rotates around the threaded shaft",
        rest_indicator_center is not None
        and turned_indicator_center is not None
        and abs(turned_indicator_center[1] - rest_indicator_center[1]) > 0.010
        and abs(turned_indicator_center[2] - rest_indicator_center[2]) > 0.010,
        details=f"rest={rest_indicator_center}, turned={turned_indicator_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
