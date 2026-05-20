from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _safe_fillet(shape: cq.Workplane, radius: float) -> cq.Workplane:
    """Apply a small cast-metal edge radius when CadQuery can solve it."""
    try:
        return shape.edges().fillet(radius)
    except Exception:
        return shape


def _frame_casting_mesh() -> cq.Workplane:
    """C-shaped clamp frame with a clear screw hole through the lower arm."""
    back = cq.Workplane("XY").box(0.080, 0.080, 0.485).translate((-0.170, 0.0, 0.0))
    upper = cq.Workplane("XY").box(0.295, 0.080, 0.078).translate((-0.055, 0.0, 0.225))
    lower = cq.Workplane("XY").box(0.295, 0.080, 0.078).translate((-0.055, 0.0, -0.225))

    casting = back.union(upper).union(lower)

    # Cast triangular ribs on both cheeks make the open C look like a portable vise
    # frame rather than three plain bars.
    rib_profile = cq.Workplane("XZ").polyline(
        [(-0.150, -0.185), (-0.095, -0.030), (-0.150, 0.185)]
    ).close()
    rib = rib_profile.extrude(0.010, both=True).translate((0.0, 0.045, 0.0))
    casting = casting.union(rib).union(rib.mirror("XY"))

    # Clearance bore for the screw through the lower arm.  The separate lower boss
    # visual is a hollow tube around the same axis.
    screw_hole = (
        cq.Workplane("XY")
        .circle(0.024)
        .extrude(0.180, both=True)
        .translate((0.045, 0.0, -0.225))
    )
    casting = casting.cut(screw_hole)
    return _safe_fillet(casting, 0.006)


def _lower_boss_mesh() -> cq.Workplane:
    """Hollow threaded-looking boss cast into the lower arm."""
    boss = (
        cq.Workplane("XY")
        .circle(0.046)
        .circle(0.020)
        .extrude(0.132, both=True)
        .translate((0.045, 0.0, -0.205))
    )
    return _safe_fillet(boss, 0.004)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="c_clamp_portable_vise")

    cast_steel = model.material("dark_cast_steel", rgba=(0.09, 0.10, 0.11, 1.0))
    bright_steel = model.material("polished_jaw_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    screw_steel = model.material("oiled_screw_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    black_grip = model.material("black_handle_knobs", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_casting_mesh(), "frame_casting", tolerance=0.001),
        material=cast_steel,
        name="frame_casting",
    )
    frame.visual(
        mesh_from_cadquery(_lower_boss_mesh(), "lower_boss", tolerance=0.001),
        material=cast_steel,
        name="lower_boss",
    )
    frame.visual(
        Box((0.092, 0.070, 0.026)),
        origin=Origin(xyz=(0.042, 0.0, 0.174)),
        material=bright_steel,
        name="upper_jaw",
    )
    for index, x in enumerate((-0.028, -0.014, 0.000, 0.014, 0.028)):
        frame.visual(
            Box((0.0055, 0.064, 0.004)),
            origin=Origin(xyz=(0.042 + x, 0.0, 0.159)),
            material=bright_steel,
            name=f"upper_tooth_{index}",
        )

    jaw_pad = model.part("jaw_pad")
    jaw_pad.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=Origin(),
        material=bright_steel,
        name="jaw_disk",
    )
    jaw_pad.visual(
        Cylinder(radius=0.020, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=bright_steel,
        name="pad_socket",
    )
    for index, x in enumerate((-0.024, -0.012, 0.000, 0.012, 0.024)):
        jaw_pad.visual(
            Box((0.0045, 0.058, 0.003)),
            origin=Origin(xyz=(x, 0.0, 0.011)),
            material=bright_steel,
            name=f"pad_tooth_{index}",
        )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.0125, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
        material=screw_steel,
        name="screw_shaft",
    )
    screw_handle.visual(
        Cylinder(radius=0.021, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=screw_steel,
        name="threaded_section",
    )
    # Thin collars on the rod read as screw threads while staying within the
    # hollow lower boss clearance.
    for index in range(16):
        screw_handle.visual(
            Cylinder(radius=0.0155, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.025 - index * 0.022)),
            material=screw_steel,
            name=f"thread_{index}",
        )
    screw_handle.visual(
        Cylinder(radius=0.010, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, -0.455), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screw_steel,
        name="cross_handle",
    )
    screw_handle.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(-0.091, 0.0, -0.455)),
        material=black_grip,
        name="handle_knob_0",
    )
    screw_handle.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.091, 0.0, -0.455)),
        material=black_grip,
        name="handle_knob_1",
    )

    model.articulation(
        "frame_to_jaw_pad",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=jaw_pad,
        origin=Origin(xyz=(0.045, 0.0, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.04, lower=0.0, upper=0.120),
    )
    model.articulation(
        "jaw_pad_to_screw_handle",
        ArticulationType.CONTINUOUS,
        parent=jaw_pad,
        child=screw_handle,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    jaw_pad = object_model.get_part("jaw_pad")
    screw_handle = object_model.get_part("screw_handle")
    slide = object_model.get_articulation("frame_to_jaw_pad")
    spin = object_model.get_articulation("jaw_pad_to_screw_handle")

    ctx.allow_overlap(
        screw_handle,
        frame,
        elem_a="threaded_section",
        elem_b="lower_boss",
        reason=(
            "The screw's thread crests are intentionally seated in the nut-like "
            "lower boss to show the load-bearing threaded engagement."
        ),
    )

    ctx.check(
        "jaw pad has upward prismatic travel",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and abs((slide.motion_limits.upper or 0.0) - 0.120) < 1e-6,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )
    ctx.check(
        "screw handle spins continuously",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_overlap(
        frame,
        jaw_pad,
        axes="xy",
        min_overlap=0.050,
        elem_a="upper_jaw",
        elem_b="jaw_disk",
        name="moving jaw pad stays centered below fixed upper jaw",
    )
    ctx.expect_overlap(
        screw_handle,
        frame,
        axes="z",
        min_overlap=0.090,
        elem_a="threaded_section",
        elem_b="lower_boss",
        name="screw remains engaged in lower threaded boss",
    )
    ctx.expect_gap(
        frame,
        jaw_pad,
        axis="z",
        min_gap=0.120,
        max_gap=0.140,
        positive_elem="upper_jaw",
        negative_elem="jaw_disk",
        name="open vise gap at rest",
    )

    rest_pos = ctx.part_world_position(jaw_pad)
    with ctx.pose({slide: 0.120, spin: math.pi / 2.0}):
        ctx.expect_overlap(
            frame,
            jaw_pad,
            axes="xy",
            min_overlap=0.050,
            elem_a="upper_jaw",
            elem_b="jaw_disk",
            name="raised jaw remains aligned to fixed jaw",
        )
        ctx.expect_gap(
            frame,
            jaw_pad,
            axis="z",
            min_gap=0.004,
            max_gap=0.020,
            positive_elem="upper_jaw",
            negative_elem="jaw_disk",
            name="raised jaw approaches the fixed jaw without collision",
        )
        ctx.expect_overlap(
            screw_handle,
            frame,
            axes="z",
            min_overlap=0.070,
            elem_a="threaded_section",
            elem_b="lower_boss",
            name="raised screw still retained by threaded boss",
        )
        raised_pos = ctx.part_world_position(jaw_pad)

    ctx.check(
        "prismatic screw drive raises lower jaw",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.10,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
