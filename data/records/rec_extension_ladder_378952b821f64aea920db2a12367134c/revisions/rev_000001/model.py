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
    model = ArticulatedObject(name="combination_a_frame_extension_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    darker_aluminum = model.material("darker_aluminum", rgba=(0.47, 0.49, 0.48, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    yellow_plastic = model.material("yellow_safety_latch", rgba=(1.0, 0.72, 0.08, 1.0))
    red_marking = model.material("red_lock_marking", rgba=(0.72, 0.05, 0.04, 1.0))

    # Overall dimensions are in the range of a compact 2 m combination ladder.
    height = 2.00
    front_foot_x = -0.55
    rear_foot_x = 0.64
    front_half_width = 0.29
    rear_half_width = 0.34
    brace_side_width = 0.385
    rail_depth = 0.050
    rail_width = 0.045
    rung_radius = 0.024
    rung_span_front = 0.70
    rung_span_rear = 0.74
    front_theta = math.atan2(-front_foot_x, height)
    rear_theta = math.atan2(rear_foot_x, -height)

    def add_sloped_box(part, name, start, end, size_xy, material):
        sx, sy = size_xy
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        # All long ladder members live in an X-Z plane, so a Y rotation is enough.
        theta = math.atan2(dx, dz)
        part.visual(
            Box((sx, sy, length)),
            origin=Origin(
                xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
                rpy=(0.0, theta, 0.0),
            ),
            material=material,
            name=name,
        )

    def point_on_front(t, y=0.0, offset=0.0):
        """Point along the front rails with a small normal offset in the ladder plane."""
        x = front_foot_x + (-front_foot_x) * t
        z = height * t
        nx = math.cos(front_theta)
        nz = -math.sin(front_theta)
        return (x + nx * offset, y, z + nz * offset)

    # Root: the front section with two rails, rungs, guide channels, feet, and hinge pin.
    front_section = model.part("front_section")
    for side, y in (("side_0", -front_half_width), ("side_1", front_half_width)):
        add_sloped_box(
            front_section,
            f"front_rail_{side}",
            (front_foot_x, y, 0.02),
            (0.0, y, height),
            (rail_depth, rail_width),
            aluminum,
        )
        front_section.visual(
            Box((0.18, 0.09, 0.045)),
            origin=Origin(xyz=(front_foot_x - 0.03, y, 0.025), rpy=(0.0, -0.18, 0.0)),
            material=black_rubber,
            name=f"front_foot_{side}",
        )

    for idx, t in enumerate((0.16, 0.31, 0.46, 0.61, 0.76, 0.90)):
        p = point_on_front(t)
        front_section.visual(
            Cylinder(radius=rung_radius, length=rung_span_front),
            origin=Origin(xyz=p, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_aluminum,
            name=f"front_rung_{idx}",
        )

    # Rear-facing guide channels that the extension fly rides in.  They are tied
    # into the rails by short spacer blocks so the front section reads as one welded assembly.
    guide_offset = 0.068
    guide_half_width = 0.225
    for side, guide_name, y in (
        ("side_0", "fly_guide_side_0", -guide_half_width),
        ("side_1", "fly_guide_side_1", guide_half_width),
    ):
        start = point_on_front(0.09, y, guide_offset)
        end = point_on_front(0.93, y, guide_offset)
        add_sloped_box(
            front_section,
            guide_name,
            start,
            end,
            (0.030, 0.036),
            darker_aluminum,
        )
        rail_y = -front_half_width if y < 0 else front_half_width
        for j, t in enumerate((0.18, 0.48, 0.78)):
            p = point_on_front(t, (y + rail_y) * 0.5, guide_offset * 0.5)
            front_section.visual(
                Box((0.070, abs(rail_y - y) + 0.050, 0.050)),
                origin=Origin(xyz=p, rpy=(0.0, front_theta, 0.0)),
                material=darker_aluminum,
                name=f"guide_spacer_{side}_{j}",
            )

    # Hinge hardware at the apex: a pin on the front section and cheeks that
    # visually capture the rear section's hinge sleeve.
    front_section.visual(
        Cylinder(radius=0.025, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, height), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=darker_aluminum,
        name="apex_pin",
    )
    for side, y in (("side_0", -0.345), ("side_1", 0.345)):
        front_section.visual(
            Box((0.075, 0.085, 0.060)),
            origin=Origin(xyz=(-0.065, y, height - 0.015)),
            material=aluminum,
            name=f"apex_hinge_plate_{side}",
        )

    # Side brackets for the spreader brace hinge.
    brace_hinge_world = point_on_front(0.38, 0.0, 0.02)
    for side, y in (("side_0", -brace_side_width), ("side_1", brace_side_width)):
        rail_y = -front_half_width if y < 0 else front_half_width
        front_section.visual(
            Box((0.050, abs(y - rail_y) + 0.050, 0.055)),
            origin=Origin(xyz=(brace_hinge_world[0], (y + rail_y) * 0.5, brace_hinge_world[2])),
            material=darker_aluminum,
            name=f"brace_hinge_tab_{side}",
        )

    # The rear support frame is hinged at the apex and sits behind the front
    # section in the default, locked A-frame pose.
    rear_section = model.part("rear_section")
    for side, y in (("side_0", -rear_half_width), ("side_1", rear_half_width)):
        add_sloped_box(
            rear_section,
            f"rear_rail_{side}",
            (0.035, y, -0.070),
            (rear_foot_x, y, -height),
            (rail_depth, rail_width),
            aluminum,
        )
        rear_section.visual(
            Box((0.18, 0.09, 0.045)),
            origin=Origin(xyz=(rear_foot_x + 0.04, y, -height + 0.025), rpy=(0.0, 0.18, 0.0)),
            material=black_rubber,
            name=f"rear_foot_{side}",
        )

    for idx, t in enumerate((0.22, 0.42, 0.56, 0.82)):
        x = rear_foot_x * t
        z = -height * t
        rear_section.visual(
            Cylinder(radius=0.022, length=rung_span_rear),
            origin=Origin(xyz=(x, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_aluminum,
            name=f"rear_cross_tie_{idx}",
        )
    rear_section.visual(
        Cylinder(radius=0.036, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=darker_aluminum,
        name="hinge_sleeve",
    )
    for side, y in (("side_0", -0.337), ("side_1", 0.337)):
        rear_section.visual(
            Box((0.045, 0.045, 0.070)),
            origin=Origin(xyz=(0.012, y, -0.045)),
            material=aluminum,
            name=f"top_hinge_yoke_{side}",
        )

    # The sliding fly section is a narrower ladder nested in the guide channels.
    fly_section = model.part("fly_section")
    fly_half_width = 0.185
    fly_length = 2.05
    for side, rail_name, y in (
        ("side_0", "fly_rail_side_0", -fly_half_width),
        ("side_1", "fly_rail_side_1", fly_half_width),
    ):
        fly_section.visual(
            Box((0.040, 0.038, fly_length)),
            origin=Origin(xyz=(0.0, y, 0.88)),
            material=aluminum,
            name=rail_name,
        )
        fly_section.visual(
            Box((0.080, 0.062, 0.055)),
            origin=Origin(xyz=(0.0, y, -0.13)),
            material=black_rubber,
            name=f"fly_end_cap_{side}",
        )

    for idx, z in enumerate((0.10, 0.46, 0.82, 1.18, 1.54, 1.90)):
        fly_section.visual(
            Cylinder(radius=0.021, length=0.34),
            origin=Origin(xyz=(0.0, 0.0, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_aluminum,
            name=f"fly_rung_{idx}",
        )
    for side, y in (("side_0", -fly_half_width), ("side_1", fly_half_width)):
        fly_section.visual(
            Box((0.030, 0.070, 0.080)),
            origin=Origin(xyz=(0.030, y, 0.55)),
            material=yellow_plastic,
            name=f"rung_lock_{side}",
        )

    # A folding spreader brace is hinged to the front frame.  In the rest pose it
    # is level and nearly seated on the rear legs, as on a locked step ladder.
    spreader_brace = model.part("spreader_brace")
    rear_anchor_x = rear_foot_x * ((height - brace_hinge_world[2]) / height)
    brace_length = rear_anchor_x - brace_hinge_world[0] - 0.035
    for side, y in (("side_0", -brace_side_width), ("side_1", brace_side_width)):
        brace_bar_length = brace_length - 0.033
        spreader_brace.visual(
            Box((brace_bar_length, 0.022, 0.026)),
            origin=Origin(xyz=(0.026 + brace_bar_length * 0.5, y, 0.0)),
            material=yellow_plastic,
            name=f"brace_bar_{side}",
        )
        spreader_brace.visual(
            Cylinder(radius=0.026, length=0.080),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_aluminum,
            name=f"brace_hinge_pin_{side}",
        )
        spreader_brace.visual(
            Box((0.055, 0.090, 0.060)),
            origin=Origin(xyz=(brace_length + 0.020, y, 0.0)),
            material=red_marking,
            name=f"brace_hook_{side}",
        )
    spreader_brace.visual(
        Cylinder(radius=0.015, length=2.0 * brace_side_width),
        origin=Origin(xyz=(brace_length * 0.56, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=yellow_plastic,
        name="brace_cross_handle",
    )

    # Articulations: rear A-frame hinge, fly extension slide, and spreader hinge.
    model.articulation(
        "apex_hinge",
        ArticulationType.REVOLUTE,
        parent=front_section,
        child=rear_section,
        origin=Origin(xyz=(0.0, 0.0, height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.25, upper=0.42),
    )

    fly_origin = point_on_front(0.08, 0.0, guide_offset)
    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=front_section,
        child=fly_section,
        origin=Origin(xyz=fly_origin, rpy=(0.0, front_theta, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.62),
    )

    model.articulation(
        "spreader_hinge",
        ArticulationType.REVOLUTE,
        parent=front_section,
        child=spreader_brace,
        origin=Origin(xyz=brace_hinge_world),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-0.95, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_section")
    rear = object_model.get_part("rear_section")
    fly = object_model.get_part("fly_section")
    brace = object_model.get_part("spreader_brace")
    apex = object_model.get_articulation("apex_hinge")
    slide = object_model.get_articulation("fly_slide")
    spreader = object_model.get_articulation("spreader_hinge")

    ctx.allow_overlap(
        front,
        rear,
        elem_a="apex_pin",
        elem_b="hinge_sleeve",
        reason="The rear hinge sleeve is intentionally modeled around the front apex pin.",
    )
    for side in ("side_0", "side_1"):
        ctx.allow_overlap(
            front,
            rear,
            elem_a="apex_pin",
            elem_b=f"top_hinge_yoke_{side}",
            reason="The apex pin passes through the rear yoke plate hole at the hinge.",
        )
        ctx.expect_overlap(
            front,
            rear,
            axes="xz",
            elem_a="apex_pin",
            elem_b=f"top_hinge_yoke_{side}",
            min_overlap=0.010,
            name=f"apex pin crosses rear yoke {side}",
        )
        ctx.allow_overlap(
            front,
            rear,
            elem_a=f"apex_hinge_plate_{side}",
            elem_b="hinge_sleeve",
            reason="The rear hinge sleeve locally passes through the front hinge cheek plate.",
        )
        ctx.expect_overlap(
            front,
            rear,
            axes="yz",
            elem_a=f"apex_hinge_plate_{side}",
            elem_b="hinge_sleeve",
            min_overlap=0.008,
            name=f"front hinge cheek {side} surrounds sleeve",
        )
    for side in ("side_0", "side_1"):
        ctx.allow_overlap(
            front,
            rear,
            elem_a=f"front_rail_{side}",
            elem_b="hinge_sleeve",
            reason="The hinge sleeve passes through the drilled top of each front rail at the apex.",
        )
        ctx.expect_overlap(
            front,
            rear,
            axes="xz",
            elem_a=f"front_rail_{side}",
            elem_b="hinge_sleeve",
            min_overlap=0.020,
            name=f"hinge sleeve crosses front rail {side}",
        )
    for side in ("side_0", "side_1"):
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"brace_hinge_tab_{side}",
            elem_b=f"brace_hinge_pin_{side}",
            reason="The spreader brace hinge pin passes through the front frame tab.",
        )
        ctx.expect_within(
            brace,
            front,
            axes="xz",
            inner_elem=f"brace_hinge_pin_{side}",
            outer_elem=f"brace_hinge_tab_{side}",
            margin=0.004,
            name=f"spreader pin {side} is held in front tab",
        )
        ctx.allow_overlap(
            rear,
            brace,
            elem_a=f"rear_rail_{side}",
            elem_b=f"brace_hook_{side}",
            reason="The hook at the end of the spreader brace is seated around the rear rail to lock the A-frame angle.",
        )
        ctx.expect_overlap(
            rear,
            brace,
            axes="xz",
            elem_a=f"rear_rail_{side}",
            elem_b=f"brace_hook_{side}",
            min_overlap=0.020,
            name=f"spreader hook {side} captures rear rail",
        )
    ctx.expect_within(
        front,
        rear,
        axes="xz",
        inner_elem="apex_pin",
        outer_elem="hinge_sleeve",
        margin=0.003,
        name="apex pin is captured by rear sleeve",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="y",
        elem_a="apex_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.45,
        name="hinge sleeve has broad pin engagement",
    )

    rest_fly = ctx.part_world_position(fly)
    with ctx.pose({slide: 0.62}):
        extended_fly = ctx.part_world_position(fly)
        ctx.expect_overlap(
            fly,
            front,
            axes="xz",
            elem_a="fly_rail_side_0",
            elem_b="fly_guide_side_0",
            min_overlap=0.010,
            name="extended fly remains engaged along guide",
        )
        ctx.expect_gap(
            fly,
            front,
            axis="y",
            positive_elem="fly_rail_side_0",
            negative_elem="fly_guide_side_0",
            min_gap=0.0,
            max_gap=0.010,
            name="extended fly rail has guide clearance",
        )
    ctx.check(
        "fly slide raises extension section",
        rest_fly is not None and extended_fly is not None and extended_fly[2] > rest_fly[2] + 0.50,
        details=f"rest={rest_fly}, extended={extended_fly}",
    )

    rest_rear = ctx.part_element_world_aabb(rear, elem="rear_foot_side_0")
    with ctx.pose({apex: 0.30}):
        moved_rear = ctx.part_element_world_aabb(rear, elem="rear_foot_side_0")
    rest_rear_x = None if rest_rear is None else (rest_rear[0][0] + rest_rear[1][0]) * 0.5
    moved_rear_x = None if moved_rear is None else (moved_rear[0][0] + moved_rear[1][0]) * 0.5
    ctx.check(
        "apex hinge changes rear support angle",
        rest_rear_x is not None and moved_rear_x is not None and abs(moved_rear_x - rest_rear_x) > 0.05,
        details=f"rest={rest_rear}, moved={moved_rear}",
    )

    rest_brace = ctx.part_element_world_aabb(brace, elem="brace_hook_side_0")
    with ctx.pose({spreader: -0.70}):
        folded_brace = ctx.part_element_world_aabb(brace, elem="brace_hook_side_0")
    rest_brace_z = None if rest_brace is None else (rest_brace[0][2] + rest_brace[1][2]) * 0.5
    folded_brace_z = None if folded_brace is None else (folded_brace[0][2] + folded_brace[1][2]) * 0.5
    ctx.check(
        "spreader brace folds on its hinge",
        rest_brace_z is not None and folded_brace_z is not None and abs(folded_brace_z - rest_brace_z) > 0.10,
        details=f"rest={rest_brace}, folded={folded_brace}",
    )

    return ctx.report()


object_model = build_object_model()
