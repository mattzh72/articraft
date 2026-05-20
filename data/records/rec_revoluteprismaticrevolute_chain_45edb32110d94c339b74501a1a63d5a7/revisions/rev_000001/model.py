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


PLATE_T = 0.014
PLATE_W = 0.180
PLATE_H = 0.280

HINGE_AXIS_X = PLATE_T / 2.0 + 0.016
CARRIER_BODY_LEN = 0.160
CARRIER_BODY_W = 0.040
CARRIER_BODY_H = 0.040

SLIDER_BEAM_LEN = 0.140
SLIDER_BEAM_W = 0.010
SLIDER_BEAM_H = 0.010
SLIDER_ORIGIN_X = 0.060

TIP_HINGE_X = 0.168


def _backplate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H)
    pivot_boss = (
        cq.Workplane("XY")
        .box(0.016, 0.030, 0.060)
        .translate((HINGE_AXIS_X - 0.008, 0.0, 0.0))
    )
    upper_rib = (
        cq.Workplane("XY")
        .box(0.012, 0.050, 0.072)
        .translate((0.010, 0.0, 0.050))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.012, 0.050, 0.072)
        .translate((0.010, 0.0, -0.050))
    )
    return plate.union(pivot_boss).union(upper_rib).union(lower_rib)


def _carrier_shape() -> cq.Workplane:
    hinge_tongue = cq.Workplane("XY").box(0.012, 0.018, 0.030).translate((0.006, 0.0, 0.0))
    neck = cq.Workplane("XY").box(0.030, 0.022, 0.026).translate((0.021, 0.0, 0.0))
    shell_outer = (
        cq.Workplane("XY")
        .box(0.150, 0.040, 0.032)
        .translate((0.110, 0.0, 0.0))
    )
    shell_inner = (
        cq.Workplane("XY")
        .box(0.144, 0.028, 0.020)
        .translate((0.116, 0.0, 0.0))
    )
    return hinge_tongue.union(neck).union(shell_outer).cut(shell_inner)


def _slider_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(0.142, SLIDER_BEAM_W, SLIDER_BEAM_H)
        .translate((0.075, 0.0, 0.0))
    )
    rear_guide = (
        cq.Workplane("XY")
        .box(0.024, 0.014, 0.018)
        .translate((-0.004, 0.0, 0.0))
    )
    tip_block = cq.Workplane("XY").box(0.026, 0.018, 0.018).translate((0.155, 0.0, 0.0))
    return beam.union(rear_guide).union(tip_block)


def _tab_shape() -> cq.Workplane:
    tongue = cq.Workplane("XY").box(0.010, 0.012, 0.014).translate((0.005, 0.0, 0.0))
    stem = cq.Workplane("XY").box(0.016, 0.014, 0.012).translate((0.014, 0.0, 0.0))
    paddle = cq.Workplane("XY").box(0.040, 0.028, 0.008).translate((0.036, 0.0, -0.002))
    return tongue.union(stem).union(paddle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_transfer_nose")

    model.material("backplate_paint", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("carrier_alloy", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("slider_steel", rgba=(0.48, 0.51, 0.55, 1.0))
    model.material("tool_tab_finish", rgba=(0.82, 0.46, 0.14, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate"),
        material="backplate_paint",
        name="backplate_shell",
    )

    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_cadquery(_carrier_shape(), "carrier"),
        material="carrier_alloy",
        name="carrier_shell",
    )

    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_slider_shape(), "slider"),
        material="slider_steel",
        name="slider_shell",
    )

    tab = model.part("tool_tab")
    tab.visual(
        mesh_from_cadquery(_tab_shape(), "tool_tab"),
        material="tool_tab_finish",
        name="tool_tab_shell",
    )

    model.articulation(
        "backplate_to_carrier",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=carrier,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=0.0, upper=1.05),
    )
    model.articulation(
        "carrier_to_slider",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=slider,
        origin=Origin(xyz=(SLIDER_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.100),
    )
    model.articulation(
        "slider_to_tool_tab",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=tab,
        origin=Origin(xyz=(TIP_HINGE_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2, lower=-0.35, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    carrier = object_model.get_part("carrier")
    slider = object_model.get_part("slider")
    tab = object_model.get_part("tool_tab")
    carrier_hinge = object_model.get_articulation("backplate_to_carrier")
    slide = object_model.get_articulation("carrier_to_slider")
    tab_hinge = object_model.get_articulation("slider_to_tool_tab")

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
    ctx.allow_overlap(
        carrier,
        slider,
        reason=(
            "The slider is intentionally modeled as a captured internal ram inside the carrier guide "
            "frame; the mesh-backed guide is authored at nominal zero running clearance, so the "
            "compiled solid overlap sensor treats the nested telescoping fit as penetration."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "carrier hinge is revolute pitch",
        carrier_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(carrier_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={carrier_hinge.articulation_type}, axis={carrier_hinge.axis}",
    )
    ctx.check(
        "slider joint is forward prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.check(
        "tool tab hinge is revolute pitch",
        tab_hinge.articulation_type == ArticulationType.REVOLUTE and tuple(tab_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"type={tab_hinge.articulation_type}, axis={tab_hinge.axis}",
    )

    ctx.expect_contact(carrier, backplate, name="carrier is mounted to backplate")
    ctx.expect_contact(slider, carrier, name="slider is supported by carrier")
    ctx.expect_contact(tab, slider, name="tool tab is mounted to slider")
    ctx.expect_within(slider, carrier, axes="yz", margin=0.002, name="slider stays captured by carrier envelope")

    closed_tip_pos = ctx.part_world_position(tab)
    with ctx.pose({carrier_hinge: 0.90}):
        opened_tip_pos = ctx.part_world_position(tab)
    ctx.check(
        "carrier opening raises the nose",
        closed_tip_pos is not None
        and opened_tip_pos is not None
        and opened_tip_pos[2] > closed_tip_pos[2] + 0.12,
        details=f"closed={closed_tip_pos}, opened={opened_tip_pos}",
    )

    retracted_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slide: 0.09}):
        extended_slider_pos = ctx.part_world_position(slider)
        ctx.expect_within(
            slider,
            carrier,
            axes="yz",
            margin=0.002,
            name="slider remains laterally captured when extended",
        )
    ctx.check(
        "slider extends outward",
        retracted_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > retracted_slider_pos[0] + 0.08,
        details=f"retracted={retracted_slider_pos}, extended={extended_slider_pos}",
    )

    rest_tab_aabb = ctx.part_world_aabb(tab)
    with ctx.pose({tab_hinge: 0.95}):
        raised_tab_aabb = ctx.part_world_aabb(tab)
    ctx.check(
        "tool tab rotates upward",
        rest_tab_aabb is not None
        and raised_tab_aabb is not None
        and raised_tab_aabb[1][2] > rest_tab_aabb[1][2] + 0.015,
        details=f"rest={rest_tab_aabb}, raised={raised_tab_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
