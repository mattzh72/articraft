from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_slab(width: float, length: float, thickness: float, radius: float) -> cq.Workplane:
    """Small helper for the gently rounded glass plates used in the cassette."""
    return (
        cq.Workplane("XY")
        .box(width, length, thickness)
        .edges("|Z")
        .fillet(radius)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vent_only_skylight_sunroof_cassette")

    satin_black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("black_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    dark_anodized = model.material("dark_anodized_rail", rgba=(0.09, 0.095, 0.10, 1.0))
    shoe_plastic = model.material("graphite_slide_shoe", rgba=(0.18, 0.18, 0.17, 1.0))
    fixed_glass_mat = model.material("fixed_tinted_glass", rgba=(0.04, 0.10, 0.14, 0.42))
    vent_glass_mat = model.material("vent_tinted_glass", rgba=(0.035, 0.09, 0.13, 0.48))

    cassette = model.part("cassette_frame")

    frame_mesh = mesh_from_geometry(
        BezelGeometry(
            opening_size=(0.66, 0.40),
            outer_size=(0.84, 0.58),
            depth=0.030,
            opening_shape="rounded_rect",
            outer_shape="rounded_rect",
            opening_corner_radius=0.040,
            outer_corner_radius=0.065,
        ),
        "cassette_outer_frame",
    )
    cassette.visual(
        frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_black,
        name="outer_frame",
    )

    # Structural crossbar separating the small fixed skylight pane from the vent aperture.
    cassette.visual(
        Box((0.70, 0.035, 0.032)),
        origin=Origin(xyz=(0.0, -0.0375, 0.016)),
        material=satin_black,
        name="front_divider",
    )

    # Fixed forward glass pane, bonded into a rubber gasket that touches the outer frame
    # and the divider so the fixed panel reads as part of the cassette assembly.
    fixed_glass = _rounded_slab(0.50, 0.12, 0.007, 0.020)
    cassette.visual(
        mesh_from_cadquery(fixed_glass, "fixed_glass"),
        origin=Origin(xyz=(0.0, -0.130, 0.050)),
        material=fixed_glass_mat,
        name="fixed_glass",
    )
    cassette.visual(
        Box((0.530, 0.015, 0.010)),
        origin=Origin(xyz=(0.0, -0.1975, 0.046)),
        material=rubber,
        name="front_glass_gasket",
    )
    cassette.visual(
        Box((0.530, 0.015, 0.024)),
        origin=Origin(xyz=(0.0, -0.0625, 0.039)),
        material=rubber,
        name="rear_glass_gasket",
    )
    cassette.visual(
        Box((0.015, 0.135, 0.010)),
        origin=Origin(xyz=(-0.2575, -0.130, 0.046)),
        material=rubber,
        name="side_glass_gasket_0",
    )
    cassette.visual(
        Box((0.015, 0.135, 0.010)),
        origin=Origin(xyz=(0.2575, -0.130, 0.046)),
        material=rubber,
        name="side_glass_gasket_1",
    )

    # Parallel guide rails for the sliding vent insert.  Each rail has a low floor
    # with an outboard wall, like the shallow channels used in cassette sunroofs.
    cassette.visual(
        Box((0.085, 0.370, 0.007)),
        origin=Origin(xyz=(-0.292, 0.145, 0.0325)),
        material=dark_anodized,
        name="rail_floor_0",
    )
    cassette.visual(
        Box((0.012, 0.370, 0.026)),
        origin=Origin(xyz=(-0.327, 0.145, 0.046)),
        material=dark_anodized,
        name="outer_rail_wall_0",
    )
    cassette.visual(
        Box((0.070, 0.014, 0.016)),
        origin=Origin(xyz=(-0.292, -0.047, 0.041)),
        material=dark_anodized,
        name="front_stop_0",
    )
    cassette.visual(
        Box((0.070, 0.014, 0.016)),
        origin=Origin(xyz=(-0.292, 0.337, 0.041)),
        material=dark_anodized,
        name="rear_stop_0",
    )
    cassette.visual(
        Box((0.085, 0.370, 0.007)),
        origin=Origin(xyz=(0.292, 0.145, 0.0325)),
        material=dark_anodized,
        name="rail_floor_1",
    )
    cassette.visual(
        Box((0.012, 0.370, 0.026)),
        origin=Origin(xyz=(0.327, 0.145, 0.046)),
        material=dark_anodized,
        name="outer_rail_wall_1",
    )
    cassette.visual(
        Box((0.070, 0.014, 0.016)),
        origin=Origin(xyz=(0.292, -0.047, 0.041)),
        material=dark_anodized,
        name="front_stop_1",
    )
    cassette.visual(
        Box((0.070, 0.014, 0.016)),
        origin=Origin(xyz=(0.292, 0.337, 0.041)),
        material=dark_anodized,
        name="rear_stop_1",
    )

    vent = model.part("vent_insert")

    # The vent insert is a framed dark glass panel with low slide shoes under both sides.
    vent.visual(
        mesh_from_cadquery(_rounded_slab(0.44, 0.17, 0.006, 0.018), "vent_glass"),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=vent_glass_mat,
        name="vent_glass",
    )
    vent.visual(
        Box((0.035, 0.210, 0.014)),
        origin=Origin(xyz=(-0.2375, 0.0, 0.052)),
        material=rubber,
        name="side_frame_0",
    )
    vent.visual(
        Box((0.035, 0.210, 0.014)),
        origin=Origin(xyz=(0.2375, 0.0, 0.052)),
        material=rubber,
        name="side_frame_1",
    )
    vent.visual(
        Box((0.510, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.095, 0.052)),
        material=rubber,
        name="front_frame",
    )
    vent.visual(
        Box((0.510, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.095, 0.052)),
        material=rubber,
        name="rear_frame",
    )
    vent.visual(
        Box((0.025, 0.190, 0.010)),
        origin=Origin(xyz=(-0.292, 0.0, 0.041)),
        material=shoe_plastic,
        name="shoe_0",
    )
    vent.visual(
        Box((0.0245, 0.170, 0.008)),
        origin=Origin(xyz=(-0.26725, 0.0, 0.046)),
        material=shoe_plastic,
        name="shoe_web_0",
    )
    vent.visual(
        Box((0.025, 0.190, 0.010)),
        origin=Origin(xyz=(0.292, 0.0, 0.041)),
        material=shoe_plastic,
        name="shoe_1",
    )
    vent.visual(
        Box((0.0245, 0.170, 0.008)),
        origin=Origin(xyz=(0.26725, 0.0, 0.046)),
        material=shoe_plastic,
        name="shoe_web_1",
    )

    model.articulation(
        "frame_to_vent",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=vent,
        # At q=0 the insert closes the vent opening just behind the fixed pane.
        # Positive travel is rearward along the vehicle/cassette +Y direction.
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    vent = object_model.get_part("vent_insert")
    slide = object_model.get_articulation("frame_to_vent")

    ctx.expect_gap(
        vent,
        frame,
        axis="z",
        positive_elem="shoe_0",
        negative_elem="rail_floor_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="left shoe rides on guide rail",
    )
    ctx.expect_gap(
        vent,
        frame,
        axis="z",
        positive_elem="shoe_1",
        negative_elem="rail_floor_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="right shoe rides on guide rail",
    )
    ctx.expect_within(
        vent,
        frame,
        axes="xy",
        inner_elem="shoe_0",
        outer_elem="rail_floor_0",
        margin=0.002,
        name="left shoe is captured on rail",
    )
    ctx.expect_within(
        vent,
        frame,
        axes="xy",
        inner_elem="shoe_1",
        outer_elem="rail_floor_1",
        margin=0.002,
        name="right shoe is captured on rail",
    )

    rest_pos = ctx.part_world_position(vent)
    with ctx.pose({slide: 0.160}):
        ctx.expect_within(
            vent,
            frame,
            axes="xy",
            inner_elem="shoe_0",
            outer_elem="rail_floor_0",
            margin=0.002,
            name="left shoe remains in rearward travel",
        )
        ctx.expect_within(
            vent,
            frame,
            axes="xy",
            inner_elem="shoe_1",
            outer_elem="rail_floor_1",
            margin=0.002,
            name="right shoe remains in rearward travel",
        )
        extended_pos = ctx.part_world_position(vent)

    ctx.check(
        "vent insert slides rearward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[1] > rest_pos[1] + 0.150,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    fixed_aabb = ctx.part_element_world_aabb(frame, elem="fixed_glass")
    vent_aabb = ctx.part_element_world_aabb(vent, elem="vent_glass")
    ctx.check(
        "fixed pane sits ahead of vent insert",
        fixed_aabb is not None
        and vent_aabb is not None
        and fixed_aabb[1][1] < vent_aabb[0][1],
        details=f"fixed_aabb={fixed_aabb}, vent_aabb={vent_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
