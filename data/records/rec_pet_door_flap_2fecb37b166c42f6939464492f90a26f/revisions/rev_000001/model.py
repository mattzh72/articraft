from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_pet_door_insert")

    off_white = Material("warm_white_plastic", rgba=(0.88, 0.84, 0.76, 1.0))
    cut_edge = Material("dark_recess", rgba=(0.12, 0.11, 0.10, 1.0))
    smoked = Material("smoked_translucent_flap", rgba=(0.35, 0.45, 0.50, 0.58))
    rubber = Material("dark_flexible_edge", rgba=(0.03, 0.035, 0.035, 1.0))

    # Object frame: X is the horizontal pet-door width, Y is through the wall/door,
    # and Z is vertical.  The root part includes a short door-panel coupon so the
    # insert clearly reads as mounted around a rectangular opening.
    panel_width = 0.54
    panel_height = 0.68
    panel_thickness = 0.040
    rough_width = 0.290
    rough_height = 0.430

    outer_width = 0.380
    outer_height = 0.520
    aperture_width = 0.255
    aperture_height = 0.375
    trim_depth = 0.026
    trim_y = -panel_thickness / 2.0 - trim_depth / 2.0
    trim_center_y = trim_y
    trim_front_y = -panel_thickness / 2.0 - trim_depth

    frame = model.part("frame")

    # Four connected slabs leave an actual rectangular door-panel opening.
    side_panel_width = (panel_width - rough_width) / 2.0
    top_panel_height = (panel_height - rough_height) / 2.0
    frame.visual(
        Box((side_panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(-(rough_width / 2.0 + side_panel_width / 2.0), 0.0, 0.0)),
        material=off_white,
        name="door_panel_side_0",
    )
    frame.visual(
        Box((side_panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=((rough_width / 2.0 + side_panel_width / 2.0), 0.0, 0.0)),
        material=off_white,
        name="door_panel_side_1",
    )
    frame.visual(
        Box((rough_width, panel_thickness, top_panel_height)),
        origin=Origin(xyz=(0.0, 0.0, rough_height / 2.0 + top_panel_height / 2.0)),
        material=off_white,
        name="door_panel_top",
    )
    frame.visual(
        Box((rough_width, panel_thickness, top_panel_height)),
        origin=Origin(xyz=(0.0, 0.0, -(rough_height / 2.0 + top_panel_height / 2.0))),
        material=off_white,
        name="door_panel_bottom",
    )

    # Raised insert trim on the front face.
    side_border = (outer_width - aperture_width) / 2.0
    end_border = (outer_height - aperture_height) / 2.0
    frame.visual(
        Box((outer_width, trim_depth, end_border)),
        origin=Origin(xyz=(0.0, trim_center_y, aperture_height / 2.0 + end_border / 2.0)),
        material=off_white,
        name="top_trim",
    )
    frame.visual(
        Box((outer_width, trim_depth, end_border)),
        origin=Origin(xyz=(0.0, trim_center_y, -(aperture_height / 2.0 + end_border / 2.0))),
        material=off_white,
        name="bottom_trim",
    )
    frame.visual(
        Box((side_border, trim_depth, aperture_height)),
        origin=Origin(xyz=(-(aperture_width / 2.0 + side_border / 2.0), trim_center_y, 0.0)),
        material=off_white,
        name="side_trim_0",
    )
    frame.visual(
        Box((side_border, trim_depth, aperture_height)),
        origin=Origin(xyz=((aperture_width / 2.0 + side_border / 2.0), trim_center_y, 0.0)),
        material=off_white,
        name="side_trim_1",
    )

    # Dark tunnel lips inside the opening make the frame read hollow instead of solid.
    tunnel_depth = panel_thickness + trim_depth
    tunnel_y = trim_front_y + tunnel_depth / 2.0
    lip = 0.012
    frame.visual(
        Box((lip, tunnel_depth, aperture_height)),
        origin=Origin(xyz=(-(aperture_width / 2.0 + lip / 2.0), tunnel_y, 0.0)),
        material=cut_edge,
        name="inner_liner_0",
    )
    frame.visual(
        Box((lip, tunnel_depth, aperture_height)),
        origin=Origin(xyz=((aperture_width / 2.0 + lip / 2.0), tunnel_y, 0.0)),
        material=cut_edge,
        name="inner_liner_1",
    )
    frame.visual(
        Box((aperture_width, tunnel_depth, lip)),
        origin=Origin(xyz=(0.0, tunnel_y, aperture_height / 2.0 + lip / 2.0)),
        material=cut_edge,
        name="inner_liner_top",
    )
    frame.visual(
        Box((aperture_width, tunnel_depth, lip)),
        origin=Origin(xyz=(0.0, tunnel_y, -(aperture_height / 2.0 + lip / 2.0))),
        material=cut_edge,
        name="inner_liner_bottom",
    )

    # Small forked hinge supports attached to the top trim.  They sit just beyond
    # the moving flap sleeve so the revolute axis is visually grounded.
    hinge_y = trim_front_y - 0.012
    hinge_z = aperture_height / 2.0 - 0.010
    bracket_x = aperture_width / 2.0 + 0.006
    for index, sign in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.028, 0.018, 0.028)),
            origin=Origin(xyz=(sign * bracket_x, hinge_y + 0.004, hinge_z)),
            material=off_white,
            name=f"hinge_bracket_{index}",
        )
        frame.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(
                xyz=(sign * bracket_x, hinge_y, hinge_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=cut_edge,
            name=f"hinge_pin_{index}",
        )
    frame.visual(
        Cylinder(radius=0.005, length=0.270),
        origin=Origin(
            xyz=(0.0, hinge_y, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cut_edge,
        name="hinge_rod",
    )

    flap = model.part("flap")
    flap_width = 0.225
    flap_height = 0.345
    flap_thickness = 0.008
    hinge_sleeve_length = 0.222
    hinge_radius = 0.012
    panel_drop = 0.006
    flap.visual(
        Box((flap_width, flap_thickness, flap_height)),
        # The part frame is the hinge axis; the flat sheet starts just below
        # the rod and overlaps the sleeve shell, as a molded flap would.
        origin=Origin(xyz=(0.0, 0.0, -(panel_drop + flap_height / 2.0))),
        material=smoked,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=hinge_radius, length=hinge_sleeve_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="hinge_sleeve",
    )
    flap.visual(
        Box((0.175, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, -(panel_drop + flap_height) + 0.012)),
        material=rubber,
        name="bottom_seal",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    ctx.allow_overlap(
        frame,
        flap,
        elem_a="hinge_rod",
        elem_b="hinge_sleeve",
        reason="The hinge rod is intentionally captured inside the simplified solid flap sleeve.",
    )

    limits = hinge.motion_limits
    ctx.check(
        "flap has symmetric 45 degree hinge limits",
        limits is not None
        and abs(limits.lower + math.pi / 4.0) < 1.0e-6
        and abs(limits.upper - math.pi / 4.0) < 1.0e-6,
        details=f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            frame,
            flap,
            axis="y",
            positive_elem="top_trim",
            negative_elem="flap_panel",
            min_gap=0.004,
            max_gap=0.020,
            name="flap rests just proud of the front frame",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="x",
            positive_elem="inner_liner_1",
            negative_elem="flap_panel",
            min_gap=0.010,
            max_gap=0.030,
            name="flap clears the side opening on one edge",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="x",
            positive_elem="flap_panel",
            negative_elem="inner_liner_0",
            min_gap=0.010,
            max_gap=0.030,
            name="flap clears the side opening on the other edge",
        )
        ctx.expect_gap(
            frame,
            flap,
            axis="z",
            positive_elem="inner_liner_top",
            negative_elem="flap_panel",
            min_gap=0.004,
            max_gap=0.020,
            name="flap top sits just below the top opening",
        )
        ctx.expect_gap(
            flap,
            frame,
            axis="z",
            positive_elem="flap_panel",
            negative_elem="inner_liner_bottom",
            min_gap=0.010,
            max_gap=0.035,
            name="flap bottom remains above the lower sill",
        )
        ctx.expect_within(
            frame,
            flap,
            axes="yz",
            inner_elem="hinge_rod",
            outer_elem="hinge_sleeve",
            margin=0.001,
            name="hinge rod is centered inside the sleeve",
        )
        ctx.expect_overlap(
            frame,
            flap,
            axes="x",
            elem_a="hinge_rod",
            elem_b="hinge_sleeve",
            min_overlap=0.200,
            name="hinge rod passes through most of the sleeve",
        )

    def panel_center_y() -> float | None:
        aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    with ctx.pose({hinge: 0.0}):
        rest_y = panel_center_y()
    with ctx.pose({hinge: math.pi / 4.0}):
        inward_y = panel_center_y()
    with ctx.pose({hinge: -math.pi / 4.0}):
        outward_y = panel_center_y()

    ctx.check(
        "flap swings both inward and outward",
        rest_y is not None
        and inward_y is not None
        and outward_y is not None
        and inward_y > rest_y + 0.08
        and outward_y < rest_y - 0.08,
        details=f"rest_y={rest_y}, inward_y={inward_y}, outward_y={outward_y}",
    )

    return ctx.report()


object_model = build_object_model()
