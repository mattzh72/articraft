from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobRelief,
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_tube(outer_radius: float, inner_radius: float, length: float):
    """CadQuery annular tube with its lower face on local z=0."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )


def _rounded_plate(size_x: float, size_y: float, thickness: float):
    """Low, heavy rectangular base plate with softened vertical corners."""
    return (
        cq.Workplane("XY")
        .box(size_x, size_y, thickness)
        .edges("|Z")
        .fillet(0.035)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="perforated_rehearsal_music_stand")

    black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_metal = model.material("dark_powder_coated_steel", rgba=(0.03, 0.032, 0.034, 1.0))
    worn_edges = model.material("worn_black_edges", rgba=(0.07, 0.07, 0.065, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.004, 1.0))

    # Root: the weighted floor plate, lower sleeve, and external height collar.
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_plate(0.46, 0.34, 0.026), "flat_base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=black,
        name="flat_base_plate",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=dark_metal,
        name="post_socket",
    )
    base.visual(
        mesh_from_cadquery(_annular_tube(0.025, 0.0185, 0.64), "lower_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_metal,
        name="lower_tube",
    )
    base.visual(
        mesh_from_cadquery(_annular_tube(0.039, 0.0245, 0.060), "height_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=worn_edges,
        name="height_collar",
    )
    base.visual(
        Box((0.070, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, -0.041, 0.640)),
        material=worn_edges,
        name="collar_lugs",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.060),
        origin=Origin(xyz=(0.0, -0.070, 0.640), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="collar_screw",
    )
    for i, (x, y) in enumerate(((-0.175, -0.120), (-0.175, 0.120), (0.175, -0.120), (0.175, 0.120))):
        base.visual(
            Cylinder(radius=0.026, length=0.008),
            origin=Origin(xyz=(x, y, 0.004)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    # Sliding inner post.  Its frame is at the upper mouth of the lower tube.
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.015, length=0.99),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_metal,
        name="inner_post",
    )
    mast.visual(
        Cylinder(radius=0.0190, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=worn_edges,
        name="lower_glide",
    )
    mast.visual(
        Cylinder(radius=0.020, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.575)),
        material=worn_edges,
        name="head_collar",
    )
    mast.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.130, 0.060, 0.090),
                span_width=0.066,
                trunnion_diameter=0.020,
                trunnion_center_z=0.058,
                base_thickness=0.018,
                corner_radius=0.004,
                center=False,
            ),
            "tilt_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        material=dark_metal,
        name="tilt_yoke",
    )

    # Desk: a shallow punched-metal sheet with raised rims and a central trunnion.
    desk = model.part("desk")
    panel_roll = math.pi / 2 - 0.22

    def panel_origin(x: float, y: float, z: float = 0.0) -> Origin:
        cy = math.cos(panel_roll)
        sy = math.sin(panel_roll)
        return Origin(
            xyz=(x, y * cy - z * sy, y * sy + z * cy),
            rpy=(panel_roll, 0.0, 0.0),
        )

    desk.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (0.520, 0.320),
                0.0032,
                hole_diameter=0.010,
                pitch=(0.024, 0.023),
                frame=0.022,
                corner_radius=0.012,
                stagger=True,
            ),
            "punched_desk",
        ),
        origin=panel_origin(0.0, 0.215, 0.0),
        material=black,
        name="punched_desk",
    )
    desk.visual(
        Box((0.550, 0.026, 0.050)),
        origin=panel_origin(0.0, 0.060, 0.027),
        material=black,
        name="music_lip",
    )
    desk.visual(
        Box((0.546, 0.018, 0.014)),
        origin=panel_origin(0.0, 0.382, 0.004),
        material=worn_edges,
        name="top_rim",
    )
    desk.visual(
        Box((0.018, 0.330, 0.014)),
        origin=panel_origin(-0.270, 0.220, 0.004),
        material=worn_edges,
        name="side_rim_0",
    )
    desk.visual(
        Box((0.018, 0.330, 0.014)),
        origin=panel_origin(0.270, 0.220, 0.004),
        material=worn_edges,
        name="side_rim_1",
    )
    desk.visual(
        Box((0.052, 0.118, 0.020)),
        origin=panel_origin(0.0, 0.034, -0.004),
        material=dark_metal,
        name="pivot_plate",
    )
    desk.visual(
        Cylinder(radius=0.014, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_metal,
        name="trunnion_barrel",
    )

    # The visible clamp knob rides on the same horizontal threaded stub.
    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.0065, length=0.150),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=worn_edges,
        name="threaded_stub",
    )
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.030,
                body_style="lobed",
                base_diameter=0.040,
                top_diameter=0.052,
                crown_radius=0.002,
                bore=KnobBore(style="round", diameter=0.008),
                body_reliefs=(KnobRelief(style="top_recess", width=0.018, depth=0.0016),),
            ),
            "friction_knob",
        ),
        origin=Origin(xyz=(0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=black,
        name="friction_knob",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.666)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.320),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.648)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=-0.45, upper=0.55),
    )
    model.articulation(
        "desk_to_knob",
        ArticulationType.CONTINUOUS,
        parent=desk,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    knob = object_model.get_part("knob")
    slide = object_model.get_articulation("base_to_mast")
    tilt = object_model.get_articulation("mast_to_desk")
    spin = object_model.get_articulation("desk_to_knob")

    ctx.allow_overlap(
        mast,
        desk,
        elem_a="tilt_yoke",
        elem_b="trunnion_barrel",
        reason="The desk trunnion is intentionally captured through the yoke bores at the tilt head.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="lower_tube",
        elem_b="lower_glide",
        reason="The nylon glide is a tight sliding bushing inside the lower tube proxy.",
    )
    ctx.allow_overlap(
        desk,
        knob,
        elem_a="trunnion_barrel",
        elem_b="threaded_stub",
        reason="The threaded clamp stub passes coaxially through the desk trunnion.",
    )
    ctx.allow_overlap(
        desk,
        knob,
        elem_a="pivot_plate",
        elem_b="threaded_stub",
        reason="The clamp screw passes through the central desk pivot tab before tightening the tilt head.",
    )
    ctx.allow_overlap(
        mast,
        knob,
        elem_a="tilt_yoke",
        elem_b="threaded_stub",
        reason="The visible threaded stub is captured through the tilt yoke bore.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="lower_tube",
        margin=0.002,
        name="inner post centered in lower tube",
    )
    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="lower_glide",
        outer_elem="lower_tube",
        margin=0.001,
        name="sliding glide contained by lower tube",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="lower_glide",
        elem_b="lower_tube",
        min_overlap=0.04,
        name="lower glide engaged in sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="lower_tube",
        min_overlap=0.12,
        name="collapsed post retained in lower tube",
    )
    with ctx.pose({slide: 0.320}):
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="lower_tube",
            min_overlap=0.08,
            name="raised post remains inserted",
        )
        raised_pos = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.0}):
        lowered_pos = ctx.part_world_position(mast)
    ctx.check(
        "post translates upward",
        lowered_pos is not None and raised_pos is not None and raised_pos[2] > lowered_pos[2] + 0.30,
        details=f"lowered={lowered_pos}, raised={raised_pos}",
    )

    ctx.expect_contact(
        mast,
        desk,
        elem_a="tilt_yoke",
        elem_b="trunnion_barrel",
        contact_tol=0.020,
        name="desk trunnion seated in horizontal yoke",
    )
    ctx.expect_overlap(
        mast,
        desk,
        axes="x",
        elem_a="tilt_yoke",
        elem_b="trunnion_barrel",
        min_overlap=0.05,
        name="trunnion spans the yoke",
    )
    rest_aabb = ctx.part_world_aabb(desk)
    with ctx.pose({tilt: 0.45}):
        tilted_aabb = ctx.part_world_aabb(desk)
    ctx.check(
        "desk rotates about tilt bracket",
        rest_aabb is not None
        and tilted_aabb is not None
        and max(
            abs(tilted_aabb[0][1] - rest_aabb[0][1]),
            abs(tilted_aabb[1][1] - rest_aabb[1][1]),
        )
        > 0.05,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )

    ctx.expect_contact(
        desk,
        knob,
        elem_a="trunnion_barrel",
        elem_b="threaded_stub",
        contact_tol=0.012,
        name="knob stub mounted through tilt head",
    )
    ctx.expect_overlap(
        desk,
        knob,
        axes="x",
        elem_a="pivot_plate",
        elem_b="threaded_stub",
        min_overlap=0.04,
        name="clamp screw passes through desk pivot tab",
    )
    ctx.check(
        "friction knob spins continuously",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
