from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_TOP_Z = 0.150
SADDLE_SPINDLE_ORIGIN = (0.0, 0.090, 0.110)


def make_base_shape() -> cq.Workplane:
    left_foot = cq.Workplane("XY").box(0.16, 0.22, 0.03).translate((-0.28, 0.0, 0.015))
    right_foot = cq.Workplane("XY").box(0.16, 0.22, 0.03).translate((0.28, 0.0, 0.015))
    deck = cq.Workplane("XY").box(0.82, 0.18, 0.025).translate((0.0, 0.0, 0.0425))
    rail_bed = cq.Workplane("XY").box(0.70, 0.12, 0.045).translate((0.0, 0.0, 0.0775))
    guide_rail = cq.Workplane("XY").box(0.62, 0.08, 0.05).translate((0.0, 0.0, 0.125))
    left_stop = cq.Workplane("XY").box(0.045, 0.10, 0.035).translate((-0.34, 0.0, 0.1175))
    right_stop = cq.Workplane("XY").box(0.045, 0.10, 0.035).translate((0.34, 0.0, 0.1175))

    base = left_foot.union(right_foot)
    base = base.union(deck)
    base = base.union(rail_bed)
    base = base.union(guide_rail)
    base = base.union(left_stop)
    base = base.union(right_stop)
    return base


def make_saddle_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(0.17, 0.076, 0.025).translate((0.0, 0.0, 0.0125))
    carriage = cq.Workplane("XY").box(0.18, 0.15, 0.045).translate((0.0, 0.0, 0.0475))
    column = cq.Workplane("XY").box(0.12, 0.12, 0.075).translate((0.0, -0.01, 0.1025))
    front_block = cq.Workplane("XY").box(0.09, 0.04, 0.055).translate((0.0, 0.07, 0.1075))
    support_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.045, 0.065),
                (-0.045, 0.120),
                (-0.020, 0.145),
                (0.020, 0.145),
                (0.045, 0.120),
                (0.045, 0.065),
            ]
        )
        .close()
        .extrude(0.04)
        .translate((0.0, 0.04, 0.0))
    )
    bearing_housing = cq.Workplane("XZ").circle(0.038).extrude(0.045).translate((0.0, 0.08, 0.110))

    saddle = shoe.union(carriage)
    saddle = saddle.union(column)
    saddle = saddle.union(front_block)
    saddle = saddle.union(support_rib)
    saddle = saddle.union(bearing_housing)
    return saddle


def make_tool_nose_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(0.031).extrude(0.010)
    spindle = cq.Workplane("XY").circle(0.022).extrude(0.046).translate((0.0, 0.0, 0.0095))
    nose_collar = cq.Workplane("XY").circle(0.018).extrude(0.016).translate((0.0, 0.0, 0.055))
    tool_stub = cq.Workplane("XY").circle(0.006).extrude(0.012).translate((0.0, 0.0, 0.0705))

    tool_nose = flange.union(spindle)
    tool_nose = tool_nose.union(nose_collar)
    tool_nose = tool_nose.union(tool_stub)
    return tool_nose.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_transfer_axis")

    frame_paint = model.material("frame_paint", rgba=(0.63, 0.66, 0.70, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.28, 0.31, 0.35, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.76, 0.78, 0.80, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(make_base_shape(), "base_frame"),
        origin=Origin(),
        material=rail_steel,
        name="base_shell",
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_cadquery(make_saddle_shape(), "saddle"),
        origin=Origin(),
        material=frame_paint,
        name="saddle_shell",
    )

    tool_nose = model.part("tool_nose")
    tool_nose.visual(
        mesh_from_cadquery(make_tool_nose_shape(), "tool_nose"),
        origin=Origin(),
        material=spindle_steel,
        name="tool_nose_shell",
    )

    model.articulation(
        "slide_axis",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.35,
            lower=-0.18,
            upper=0.18,
        ),
    )

    model.articulation(
        "spindle_spin",
        ArticulationType.REVOLUTE,
        parent=saddle,
        child=tool_nose,
        origin=Origin(xyz=SADDLE_SPINDLE_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=12.0,
            lower=-3.1416,
            upper=3.1416,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    saddle = object_model.get_part("saddle")
    tool_nose = object_model.get_part("tool_nose")
    slide_axis = object_model.get_articulation("slide_axis")
    spindle_spin = object_model.get_articulation("spindle_spin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slide joint configured along rail",
        slide_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide_axis.axis) == (1.0, 0.0, 0.0)
        and slide_axis.motion_limits is not None
        and slide_axis.motion_limits.lower == -0.18
        and slide_axis.motion_limits.upper == 0.18,
        details=f"unexpected slide articulation: {slide_axis}",
    )
    ctx.check(
        "tool nose spins about forward axis",
        spindle_spin.articulation_type == ArticulationType.REVOLUTE
        and tuple(spindle_spin.axis) == (0.0, 1.0, 0.0),
        details=f"unexpected spindle articulation: {spindle_spin}",
    )

    ctx.expect_gap(
        saddle,
        base_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="saddle seats on fixed guide rail",
    )
    ctx.expect_overlap(
        saddle,
        base_frame,
        axes="xy",
        min_overlap=0.07,
        name="saddle stays over guide rail footprint",
    )
    ctx.expect_gap(
        tool_nose,
        saddle,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="tool nose seats against saddle support face",
    )
    ctx.expect_overlap(
        tool_nose,
        saddle,
        axes="xz",
        min_overlap=0.035,
        name="tool nose remains centered in local support",
    )

    rest_saddle_pos = ctx.part_world_position(saddle)
    with ctx.pose({slide_axis: 0.12}):
        translated_saddle_pos = ctx.part_world_position(saddle)
        translated_tool_pos = ctx.part_world_position(tool_nose)
        ctx.expect_gap(
            saddle,
            base_frame,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="translated saddle remains supported by rail",
        )

    ctx.check(
        "saddle translates only along rail axis",
        rest_saddle_pos is not None
        and translated_saddle_pos is not None
        and translated_tool_pos is not None
        and translated_saddle_pos[0] - rest_saddle_pos[0] > 0.11
        and abs(translated_saddle_pos[1] - rest_saddle_pos[1]) < 1e-6
        and abs(translated_saddle_pos[2] - rest_saddle_pos[2]) < 1e-6
        and translated_tool_pos[0] > translated_saddle_pos[0] - 0.001,
        details=(
            f"rest={rest_saddle_pos}, translated={translated_saddle_pos}, "
            f"tool={translated_tool_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
