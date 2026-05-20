from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    axis: str = "z",
):
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length / 2.0), (outer_radius, length / 2.0)],
        [(inner_radius, -length / 2.0), (inner_radius, length / 2.0)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    if axis == "x":
        shell.rotate_y(pi / 2.0)
    elif axis == "y":
        shell.rotate_x(pi / 2.0)
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_on_welding_vise")

    cast_steel = model.material("cast_steel", rgba=(0.20, 0.23, 0.27, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    jaw_insert = model.material("jaw_insert", rgba=(0.46, 0.31, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.040, 0.110, 0.136)),
        origin=Origin(xyz=(-0.040, 0.000, 0.058)),
        material=cast_steel,
        name="rear_spine",
    )
    frame.visual(
        Box((0.055, 0.085, 0.030)),
        origin=Origin(xyz=(-0.023, 0.000, 0.128)),
        material=cast_steel,
        name="top_head",
    )
    frame.visual(
        Box((0.022, 0.125, 0.055)),
        origin=Origin(xyz=(-0.011, 0.000, 0.147)),
        material=cast_steel,
        name="fixed_jaw_block",
    )
    frame.visual(
        Box((0.004, 0.114, 0.050)),
        origin=Origin(xyz=(-0.002, 0.000, 0.147)),
        material=jaw_insert,
        name="fixed_jaw_face",
    )
    frame.visual(
        Box((0.090, 0.070, 0.012)),
        origin=Origin(xyz=(0.012, 0.000, 0.072)),
        material=cast_steel,
        name="upper_clamp_pad",
    )
    frame.visual(
        Box((0.070, 0.018, 0.016)),
        origin=Origin(xyz=(0.012, 0.026, -0.002)),
        material=cast_steel,
        name="lower_clamp_bar_left",
    )
    frame.visual(
        Box((0.070, 0.018, 0.016)),
        origin=Origin(xyz=(0.012, -0.026, -0.002)),
        material=cast_steel,
        name="lower_clamp_bar_right",
    )
    frame.visual(
        _tube_mesh(
            outer_radius=0.018,
            inner_radius=0.0105,
            length=0.024,
            name="lower_thread_boss",
        ),
        origin=Origin(xyz=(0.012, 0.000, -0.002)),
        material=cast_steel,
        name="thread_boss",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.190),
        origin=Origin(xyz=(0.093, 0.033, 0.123), rpy=(0.000, pi / 2.0, 0.000)),
        material=machined_steel,
        name="guide_rod_left",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.190),
        origin=Origin(xyz=(0.093, -0.033, 0.123), rpy=(0.000, pi / 2.0, 0.000)),
        material=machined_steel,
        name="guide_rod_right",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.189, 0.033, 0.123), rpy=(0.000, pi / 2.0, 0.000)),
        material=cast_steel,
        name="rod_stop_left",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.189, -0.033, 0.123), rpy=(0.000, pi / 2.0, 0.000)),
        material=cast_steel,
        name="rod_stop_right",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.255, 0.130, 0.190)),
        mass=8.5,
        origin=Origin(xyz=(0.040, 0.000, 0.070)),
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        Box((0.022, 0.120, 0.042)),
        origin=Origin(xyz=(-0.019, 0.000, 0.030)),
        material=cast_steel,
        name="front_jaw_plate",
    )
    front_jaw.visual(
        Box((0.004, 0.112, 0.036)),
        origin=Origin(xyz=(-0.028, 0.000, 0.030)),
        material=jaw_insert,
        name="front_jaw_face",
    )
    front_jaw.visual(
        Box((0.042, 0.096, 0.014)),
        origin=Origin(xyz=(0.010, 0.000, 0.018)),
        material=cast_steel,
        name="front_carriage_top",
    )
    front_jaw.visual(
        Box((0.060, 0.088, 0.010)),
        origin=Origin(xyz=(0.010, 0.000, -0.012)),
        material=cast_steel,
        name="front_carriage_bridge",
    )
    front_jaw.visual(
        Box((0.060, 0.010, 0.028)),
        origin=Origin(xyz=(0.010, 0.046, 0.000)),
        material=cast_steel,
        name="front_guide_web_left",
    )
    front_jaw.visual(
        Box((0.060, 0.010, 0.028)),
        origin=Origin(xyz=(0.010, -0.046, 0.000)),
        material=cast_steel,
        name="front_guide_web_right",
    )
    front_jaw.visual(
        Box((0.060, 0.020, 0.010)),
        origin=Origin(xyz=(0.010, 0.033, 0.012)),
        material=cast_steel,
        name="front_guide_cap_left",
    )
    front_jaw.visual(
        Box((0.060, 0.020, 0.010)),
        origin=Origin(xyz=(0.010, -0.033, 0.012)),
        material=cast_steel,
        name="front_guide_cap_right",
    )
    front_jaw.inertial = Inertial.from_geometry(
        Box((0.070, 0.125, 0.060)),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
    )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.008, length=0.090),
        origin=Origin(xyz=(0.000, 0.000, 0.015)),
        material=machined_steel,
        name="screw_shaft",
    )
    clamp_screw.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.012)),
        material=cast_steel,
        name="screw_collar",
    )
    clamp_screw.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.058)),
        material=machined_steel,
        name="screw_pad",
    )
    clamp_screw.visual(
        Cylinder(radius=0.004, length=0.120),
        origin=Origin(xyz=(0.000, 0.000, -0.029), rpy=(pi / 2.0, 0.000, 0.000)),
        material=machined_steel,
        name="screw_handle",
    )
    clamp_screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.100),
        mass=0.9,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
    )

    model.articulation(
        "frame_to_front_jaw",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=front_jaw,
        origin=Origin(xyz=(0.055, 0.000, 0.123)),
        axis=(1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.12,
            lower=0.000,
            upper=0.100,
        ),
    )
    model.articulation(
        "frame_to_clamp_screw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=clamp_screw,
        origin=Origin(xyz=(0.012, 0.000, -0.002)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=4.0,
            lower=-2.0 * pi,
            upper=2.0 * pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    front_jaw = object_model.get_part("front_jaw")
    clamp_screw = object_model.get_part("clamp_screw")
    jaw_slide = object_model.get_articulation("frame_to_front_jaw")
    screw_joint = object_model.get_articulation("frame_to_clamp_screw")

    jaw_limits = jaw_slide.motion_limits
    screw_limits = screw_joint.motion_limits

    ctx.check(
        "front jaw slides on the X axis",
        jaw_slide.axis == (1.0, 0.0, 0.0),
        details=f"axis={jaw_slide.axis}",
    )
    ctx.check(
        "clamp screw rotates on the vertical axis",
        screw_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={screw_joint.axis}",
    )
    ctx.check(
        "front jaw has meaningful travel",
        jaw_limits is not None and jaw_limits.upper is not None and jaw_limits.upper >= 0.10,
        details=f"limits={jaw_limits}",
    )
    ctx.check(
        "screw has multiple turns of motion",
        screw_limits is not None
        and screw_limits.lower is not None
        and screw_limits.upper is not None
        and screw_limits.upper - screw_limits.lower >= 4.0 * pi,
        details=f"limits={screw_limits}",
    )

    ctx.expect_gap(
        front_jaw,
        frame,
        axis="x",
        positive_elem="front_jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.020,
        max_gap=0.030,
        name="closed jaws keep a realistic small working gap",
    )
    ctx.expect_gap(
        frame,
        clamp_screw,
        axis="z",
        positive_elem="upper_clamp_pad",
        negative_elem="screw_pad",
        min_gap=0.004,
        max_gap=0.008,
        name="clamp screw pad sits just below the bench seat",
    )
    ctx.expect_overlap(
        frame,
        front_jaw,
        axes="x",
        elem_a="guide_rod_left",
        elem_b="front_guide_cap_left",
        min_overlap=0.050,
        name="left guide cap stays engaged on the left rod at rest",
    )
    ctx.expect_overlap(
        frame,
        front_jaw,
        axes="x",
        elem_a="guide_rod_right",
        elem_b="front_guide_cap_right",
        min_overlap=0.050,
        name="right guide cap stays engaged on the right rod at rest",
    )
    ctx.expect_gap(
        frame,
        front_jaw,
        axis="z",
        positive_elem="guide_rod_left",
        negative_elem="front_carriage_bridge",
        max_penetration=1e-6,
        max_gap=0.0002,
        name="left guide rod rests on the lower carriage bridge",
    )
    ctx.expect_gap(
        front_jaw,
        frame,
        axis="z",
        positive_elem="front_guide_cap_left",
        negative_elem="guide_rod_left",
        max_penetration=1e-6,
        max_gap=0.0002,
        name="left guide cap captures the left rod from above",
    )

    rest_jaw_pos = ctx.part_world_position(front_jaw)
    rest_screw_pos = ctx.part_world_position(clamp_screw)
    with ctx.pose({jaw_slide: jaw_limits.upper}):
        extended_jaw_pos = ctx.part_world_position(front_jaw)
        ctx.expect_gap(
            front_jaw,
            frame,
            axis="x",
            positive_elem="front_jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.120,
            max_gap=0.130,
            name="extended jaw opens wide enough for stock",
        )
        ctx.expect_overlap(
            frame,
            front_jaw,
            axes="x",
            elem_a="guide_rod_left",
            elem_b="front_guide_cap_left",
            min_overlap=0.040,
            name="left guide cap retains insertion at full opening",
        )
        ctx.expect_overlap(
            frame,
            front_jaw,
            axes="x",
            elem_a="guide_rod_right",
            elem_b="front_guide_cap_right",
            min_overlap=0.040,
            name="right guide cap retains insertion at full opening",
        )
        ctx.expect_gap(
            frame,
            front_jaw,
            axis="z",
            positive_elem="guide_rod_left",
            negative_elem="front_carriage_bridge",
            max_penetration=1e-6,
            max_gap=0.0002,
            name="left guide rod stays seated on the bridge at full opening",
        )

    ctx.check(
        "front jaw moves forward when opened",
        rest_jaw_pos is not None
        and extended_jaw_pos is not None
        and extended_jaw_pos[0] > rest_jaw_pos[0] + 0.090,
        details=f"rest={rest_jaw_pos}, extended={extended_jaw_pos}",
    )

    with ctx.pose({screw_joint: pi}):
        turned_screw_pos = ctx.part_world_position(clamp_screw)
        ctx.expect_gap(
            frame,
            clamp_screw,
            axis="z",
            positive_elem="upper_clamp_pad",
            negative_elem="screw_pad",
            min_gap=0.004,
            max_gap=0.008,
            name="rotating the screw keeps the pressure pad on the clamp axis",
        )

    ctx.check(
        "screw joint is rotational rather than translating",
        rest_screw_pos is not None
        and turned_screw_pos is not None
        and abs(turned_screw_pos[0] - rest_screw_pos[0]) < 1e-6
        and abs(turned_screw_pos[1] - rest_screw_pos[1]) < 1e-6
        and abs(turned_screw_pos[2] - rest_screw_pos[2]) < 1e-6,
        details=f"rest={rest_screw_pos}, turned={turned_screw_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
