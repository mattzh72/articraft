from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_SIZE = (0.390, 0.520, 0.065)
GLASS_CENTER = (-0.025, -0.020, 0.067)
GLASS_SIZE = (0.235, 0.386, 0.004)
HINGE_AXIS = (0.0, 0.238, 0.098)
RAIL_CENTER_X = 0.136
RAIL_CENTER_Y = -0.010


def _rounded_slab(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A broad slab with rounded plan corners, authored in meters."""
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="legal_flatbed_scanner")

    warm_plastic = model.material("warm_plastic", rgba=(0.82, 0.84, 0.82, 1.0))
    white_lid = model.material("white_lid", rgba=(0.93, 0.94, 0.91, 1.0))
    hinge_dark = model.material("dark_hinge", rgba=(0.06, 0.065, 0.07, 1.0))
    glass_blue = model.material("platen_glass", rgba=(0.42, 0.70, 0.86, 0.45))
    satin_gray = model.material("satin_gray", rgba=(0.38, 0.40, 0.41, 1.0))
    foam_gray = model.material("foam_gray", rgba=(0.18, 0.19, 0.19, 1.0))
    mark_black = model.material("mark_black", rgba=(0.02, 0.02, 0.018, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_slab(BODY_SIZE, 0.018), "rounded_body"),
        origin=Origin(xyz=(0.0, 0.0, BODY_SIZE[2] / 2.0)),
        material=warm_plastic,
        name="body_shell",
    )

    platen_bezel = BezelGeometry(
        opening_size=(0.247, 0.398),
        outer_size=(0.292, 0.438),
        depth=0.010,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.006,
        outer_corner_radius=0.018,
        face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.0015),
    )
    body.visual(
        mesh_from_geometry(platen_bezel, "platen_bezel"),
        origin=Origin(xyz=(GLASS_CENTER[0], GLASS_CENTER[1], 0.068)),
        material=warm_plastic,
        name="platen_bezel",
    )
    body.visual(
        Box(GLASS_SIZE),
        origin=Origin(xyz=GLASS_CENTER),
        material=glass_blue,
        name="platen_glass",
    )

    # Raised rail and guide lips along the right side of the platen.  The
    # sliding document alignment bar rides on this channel.
    body.visual(
        Box((0.030, 0.405, 0.004)),
        origin=Origin(xyz=(RAIL_CENTER_X, RAIL_CENTER_Y, 0.0670)),
        material=satin_gray,
        name="rail_bed",
    )
    body.visual(
        Box((0.005, 0.405, 0.011)),
        origin=Origin(xyz=(RAIL_CENTER_X - 0.019, RAIL_CENTER_Y, 0.0705)),
        material=satin_gray,
        name="inner_rail_lip",
    )
    body.visual(
        Box((0.005, 0.405, 0.011)),
        origin=Origin(xyz=(RAIL_CENTER_X + 0.019, RAIL_CENTER_Y, 0.0705)),
        material=satin_gray,
        name="outer_rail_lip",
    )

    # Subtle document registration ticks printed near the right edge of the
    # glass. They are slightly embedded so they read as markings, not loose
    # floating strips.
    for index, y in enumerate((-0.170, -0.085, 0.000, 0.085, 0.170)):
        body.visual(
            Box((0.018, 0.0015, 0.0008)),
            origin=Origin(xyz=(GLASS_CENTER[0] + GLASS_SIZE[0] / 2.0 - 0.018, y, 0.0694)),
            material=mark_black,
            name=f"tick_{index}",
        )

    # Two rear barrel hinge assemblies: each has body-side outer knuckles and
    # a real stanchion rising from the scanner body.
    for hinge_index, hinge_x in enumerate((-0.112, 0.112)):
        body.visual(
            Cylinder(radius=0.003, length=0.066),
            origin=Origin(
                xyz=(hinge_x, HINGE_AXIS[1], HINGE_AXIS[2]),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_dark,
            name=f"hinge_{hinge_index}_pin",
        )
        for side_index, dx in enumerate((-0.0215, 0.0215)):
            body.visual(
                Cylinder(radius=0.007, length=0.017),
                origin=Origin(
                    xyz=(hinge_x + dx, HINGE_AXIS[1], HINGE_AXIS[2]),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_dark,
                name=f"hinge_{hinge_index}_body_knuckle_{side_index}",
            )
            body.visual(
                Box((0.017, 0.010, 0.030)),
                origin=Origin(xyz=(hinge_x + dx, HINGE_AXIS[1] + 0.006, 0.080)),
                material=hinge_dark,
                name=f"hinge_{hinge_index}_stanchion_{side_index}",
            )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_slab((0.370, 0.478, 0.022), 0.015), "lid_shell_mesh"),
        # The lid frame is on the hinge axis; the closed lid extends forward
        # along local -Y and stops short of the hinge barrels.
        origin=Origin(xyz=(0.0, -0.251, 0.004)),
        material=white_lid,
        name="lid_shell",
    )
    lid.visual(
        Box((0.318, 0.415, 0.004)),
        origin=Origin(xyz=(0.0, -0.255, -0.009)),
        material=foam_gray,
        name="pressure_pad",
    )
    for hinge_index, hinge_x in enumerate((-0.112, 0.112)):
        lid.visual(
            Cylinder(radius=0.007, length=0.022),
            origin=Origin(
                xyz=(hinge_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_dark,
            name=f"hinge_{hinge_index}_lid_knuckle",
        )
        lid.visual(
            Box((0.025, 0.020, 0.004)),
            origin=Origin(xyz=(hinge_x, -0.012, -0.008)),
            material=hinge_dark,
            name=f"hinge_{hinge_index}_lid_leaf",
        )

    alignment_bar = model.part("alignment_bar")
    alignment_bar.visual(
        Box((0.018, 0.085, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_gray,
        name="slider_shoe",
    )
    alignment_bar.visual(
        Box((0.012, 0.082, 0.008)),
        origin=Origin(xyz=(-0.003, 0.0, 0.010)),
        material=mark_black,
        name="guide_lip",
    )
    alignment_bar.visual(
        Box((0.024, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.041, 0.008)),
        material=mark_black,
        name="front_thumb_tab",
    )
    alignment_bar.visual(
        Box((0.024, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.041, 0.008)),
        material=mark_black,
        name="rear_thumb_tab",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=HINGE_AXIS),
        # Closed lid geometry extends along local -Y, so negative X makes a
        # positive angle lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "body_to_alignment_bar",
        ArticulationType.PRISMATIC,
        parent=body,
        child=alignment_bar,
        origin=Origin(xyz=(RAIL_CENTER_X, RAIL_CENTER_Y, 0.0690)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.18, lower=-0.130, upper=0.130),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    alignment_bar = object_model.get_part("alignment_bar")
    lid_hinge = object_model.get_articulation("body_to_lid")
    bar_slide = object_model.get_articulation("body_to_alignment_bar")

    for hinge_index in (0, 1):
        ctx.allow_overlap(
            body,
            lid,
            elem_a=f"hinge_{hinge_index}_pin",
            elem_b=f"hinge_{hinge_index}_lid_knuckle",
            reason="The hinge pin is intentionally captured inside the lid-side barrel knuckle.",
        )
        ctx.expect_within(
            body,
            lid,
            axes="yz",
            inner_elem=f"hinge_{hinge_index}_pin",
            outer_elem=f"hinge_{hinge_index}_lid_knuckle",
            margin=0.001,
            name=f"hinge {hinge_index} pin is concentric in lid barrel",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="x",
            elem_a=f"hinge_{hinge_index}_pin",
            elem_b=f"hinge_{hinge_index}_lid_knuckle",
            min_overlap=0.020,
            name=f"hinge {hinge_index} pin spans the lid barrel",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="platen_glass",
        min_overlap=0.20,
        name="closed lid covers the legal-size platen",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="pressure_pad",
        negative_elem="platen_glass",
        min_gap=0.010,
        max_gap=0.025,
        name="closed lid clears the glass surface",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge axis",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.14,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.expect_gap(
        alignment_bar,
        body,
        axis="x",
        positive_elem="slider_shoe",
        negative_elem="platen_glass",
        min_gap=0.025,
        max_gap=0.045,
        name="alignment bar rides along the right edge of glass",
    )
    ctx.expect_within(
        alignment_bar,
        body,
        axes="xy",
        inner_elem="slider_shoe",
        outer_elem="rail_bed",
        margin=0.002,
        name="slider shoe stays captured in the rail at rest",
    )

    rest_bar_position = ctx.part_world_position(alignment_bar)
    with ctx.pose({bar_slide: 0.130}):
        ctx.expect_within(
            alignment_bar,
            body,
            axes="xy",
            inner_elem="slider_shoe",
            outer_elem="rail_bed",
            margin=0.002,
            name="slider shoe remains captured at rear travel",
        )
        rear_bar_position = ctx.part_world_position(alignment_bar)
    ctx.check(
        "alignment bar translates along the platen edge",
        rest_bar_position is not None
        and rear_bar_position is not None
        and rear_bar_position[1] > rest_bar_position[1] + 0.11,
        details=f"rest={rest_bar_position}, rear={rear_bar_position}",
    )

    return ctx.report()


object_model = build_object_model()
