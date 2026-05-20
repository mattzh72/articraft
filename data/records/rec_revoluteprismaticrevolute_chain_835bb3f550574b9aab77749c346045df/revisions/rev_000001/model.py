from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_SIZE = (0.30, 0.22, 0.02)
HINGE_Z = 0.19

FRAME_GUIDE_START = 0.04
FRAME_GUIDE_LENGTH = 0.44
FRAME_GUIDE_WIDTH = 0.072
FRAME_GUIDE_HEIGHT = 0.050

SLIDE_JOINT_X = 0.11
SLIDE_TRAVEL = 0.18
WRIST_JOINT_X = 0.17


def _support_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(*BASE_SIZE).edges("|Z").fillet(0.01)
    pedestal = cq.Workplane("XY").box(0.14, 0.12, 0.10).translate((-0.03, 0.0, 0.060))
    tower = cq.Workplane("XY").box(0.060, 0.10, 0.08).translate((-0.050, 0.0, 0.150))
    shoulder_right = cq.Workplane("XY").box(0.050, 0.016, 0.020).translate((-0.018, 0.034, 0.145))
    shoulder_left = cq.Workplane("XY").box(0.050, 0.016, 0.020).translate((-0.018, -0.034, 0.145))
    cheek_right = cq.Workplane("XY").box(0.024, 0.010, 0.082).translate((0.0, 0.042, 0.179))
    cheek_left = cq.Workplane("XY").box(0.024, 0.010, 0.082).translate((0.0, -0.042, 0.179))

    brace_right = (
        cq.Workplane("XZ")
        .polyline([(-0.070, 0.020), (-0.070, 0.110), (-0.026, 0.145), (0.006, 0.145), (0.006, 0.020)])
        .close()
        .extrude(0.014)
        .translate((0.0, 0.026, 0.0))
    )
    brace_left = brace_right.mirror("XZ")

    return base.union(pedestal).union(tower).union(shoulder_right).union(shoulder_left).union(cheek_right).union(
        cheek_left
    ).union(brace_right).union(brace_left)


def _frame_body_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(0.0185).extrude(0.074, both=True)
    root_hub = cq.Workplane("XY").box(0.030, 0.032, 0.032).translate((0.015, 0.0, 0.0))
    neck = cq.Workplane("XY").box(0.070, 0.018, 0.016).translate((0.055, 0.0, -0.004))
    rear_bridge = cq.Workplane("XY").box(0.040, 0.058, 0.010).translate((0.090, 0.0, -0.013))

    rail_length = 0.340
    rail_center_x = 0.280
    rail_right = cq.Workplane("XY").box(rail_length, 0.010, 0.036).translate((rail_center_x, 0.026, 0.0))
    rail_left = cq.Workplane("XY").box(rail_length, 0.010, 0.036).translate((rail_center_x, -0.026, 0.0))
    front_bridge = cq.Workplane("XY").box(0.020, 0.062, 0.016).translate((0.460, 0.0, 0.0))

    return barrel.union(root_hub).union(neck).union(rear_bridge).union(rail_right).union(rail_left).union(
        front_bridge
    )


def _slider_body_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(0.130, 0.024, 0.014).translate((0.065, 0.0, 0.0))
    shoe_right = cq.Workplane("XY").box(0.130, 0.006, 0.022).translate((0.065, 0.018, 0.0))
    shoe_left = cq.Workplane("XY").box(0.130, 0.006, 0.022).translate((0.065, -0.018, 0.0))
    top_cover = cq.Workplane("XY").box(0.044, 0.018, 0.008).translate((0.096, 0.0, 0.011))
    nose = cq.Workplane("XY").box(0.040, 0.018, 0.012).translate((0.142, 0.0, 0.0))
    front_plate = cq.Workplane("XY").box(0.008, 0.030, 0.030).translate((0.166, 0.0, 0.0))

    return beam.union(shoe_right).union(shoe_left).union(top_cover).union(nose).union(front_plate)


def _output_body_shape() -> cq.Workplane:
    rear_hub = cq.Workplane("XY").box(0.018, 0.030, 0.030).translate((0.009, 0.0, 0.0))
    housing = cq.Workplane("XY").box(0.050, 0.036, 0.028).translate((0.043, 0.0, 0.0))
    lower_mount = cq.Workplane("XY").box(0.022, 0.018, 0.010).translate((0.038, 0.0, -0.013))
    front_cap = cq.Workplane("YZ").circle(0.018).extrude(0.014).translate((0.068, 0.0, 0.0))
    return rear_hub.union(housing).union(lower_mount).union(front_cap)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_mechanism")

    model.material("powder_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("anodized_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("dark_polymer", rgba=(0.13, 0.14, 0.15, 1.0))
    model.material("service_amber", rgba=(0.90, 0.63, 0.20, 1.0))

    support = model.part("support")
    support.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] / 2.0)),
        material="powder_steel",
        name="support_base",
    )
    support.visual(
        Box((0.14, 0.12, 0.10)),
        origin=Origin(xyz=(-0.035, 0.0, 0.070)),
        material="powder_steel",
        name="support_pedestal",
    )
    support.visual(
        Box((0.06, 0.10, 0.08)),
        origin=Origin(xyz=(-0.055, 0.0, 0.160)),
        material="powder_steel",
        name="support_tower",
    )
    support.visual(
        Box((0.060, 0.018, 0.022)),
        origin=Origin(xyz=(-0.015, 0.0305, 0.139)),
        material="powder_steel",
        name="support_shoulder_right",
    )
    support.visual(
        Box((0.060, 0.018, 0.022)),
        origin=Origin(xyz=(-0.015, -0.0305, 0.139)),
        material="powder_steel",
        name="support_shoulder_left",
    )
    support.visual(
        Box((0.024, 0.010, 0.082)),
        origin=Origin(xyz=(0.0, 0.0305, 0.179)),
        material="powder_steel",
        name="support_cheek_right",
    )
    support.visual(
        Box((0.024, 0.010, 0.082)),
        origin=Origin(xyz=(0.0, -0.0305, 0.179)),
        material="powder_steel",
        name="support_cheek_left",
    )
    support.inertial = Inertial.from_geometry(
        Box((0.30, 0.22, 0.25)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
    )

    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.018, length=0.051),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="hinge_barrel",
    )
    frame.visual(
        Box((0.030, 0.024, 0.024)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material="anodized_aluminum",
        name="frame_root_block",
    )
    frame.visual(
        Box((0.060, 0.018, 0.014)),
        origin=Origin(xyz=(0.060, 0.0, -0.002)),
        material="anodized_aluminum",
        name="frame_spine",
    )
    frame.visual(
        Box((0.024, 0.052, 0.010)),
        origin=Origin(xyz=(0.102, 0.0, -0.012)),
        material="anodized_aluminum",
        name="frame_rear_bridge",
    )
    frame.visual(
        Box((0.150, 0.008, 0.032)),
        origin=Origin(xyz=(0.185, 0.022, 0.0)),
        material="anodized_aluminum",
        name="frame_rail_right",
    )
    frame.visual(
        Box((0.150, 0.008, 0.032)),
        origin=Origin(xyz=(0.185, -0.022, 0.0)),
        material="anodized_aluminum",
        name="frame_rail_left",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.29, 0.06, 0.05)),
        mass=7.5,
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.130, 0.020, 0.014)),
        origin=Origin(xyz=(0.065, 0.0, 0.0)),
        material="machined_steel",
        name="slider_beam",
    )
    slider.visual(
        Box((0.130, 0.006, 0.022)),
        origin=Origin(xyz=(0.065, 0.015, 0.0)),
        material="dark_polymer",
        name="slider_shoe_right",
    )
    slider.visual(
        Box((0.130, 0.006, 0.022)),
        origin=Origin(xyz=(0.065, -0.015, 0.0)),
        material="dark_polymer",
        name="slider_shoe_left",
    )
    slider.visual(
        Box((0.044, 0.018, 0.008)),
        origin=Origin(xyz=(0.096, 0.0, 0.011)),
        material="machined_steel",
        name="slider_top_cover",
    )
    slider.visual(
        Box((0.034, 0.026, 0.026)),
        origin=Origin(xyz=(0.147, 0.0, 0.0)),
        material="machined_steel",
        name="slider_head",
    )
    slider.visual(
        Box((0.008, 0.030, 0.030)),
        origin=Origin(xyz=(0.166, 0.0, 0.0)),
        material="machined_steel",
        name="slider_front_plate",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.17, 0.04, 0.04)),
        mass=3.0,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    output = model.part("output")
    output.visual(
        Box((0.018, 0.020, 0.020)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material="machined_steel",
        name="wrist_hub",
    )
    output.visual(
        Box((0.050, 0.036, 0.028)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material="anodized_aluminum",
        name="output_body",
    )
    output.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.073, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="anodized_aluminum",
        name="output_cap",
    )
    output.visual(
        Box((0.026, 0.018, 0.010)),
        origin=Origin(xyz=(0.050, 0.0, -0.019)),
        material="service_amber",
        name="service_pad",
    )
    output.inertial = Inertial.from_geometry(
        Box((0.08, 0.04, 0.04)),
        mass=1.8,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_frame",
        ArticulationType.REVOLUTE,
        parent=support,
        child=frame,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.0),
    )
    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider,
        origin=Origin(xyz=(SLIDE_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.20, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "slider_to_output",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=output,
        origin=Origin(xyz=(WRIST_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.85, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    frame = object_model.get_part("frame")
    slider = object_model.get_part("slider")
    output = object_model.get_part("output")

    root = object_model.get_articulation("support_to_frame")
    slide = object_model.get_articulation("frame_to_slider")
    wrist = object_model.get_articulation("slider_to_output")

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
        "required_parts_present",
        {p.name for p in object_model.parts} >= {"support", "frame", "slider", "output"},
        details="Expected support, frame, slider, and output parts.",
    )
    ctx.check(
        "required_joints_present",
        {j.name for j in object_model.articulations}
        >= {"support_to_frame", "frame_to_slider", "slider_to_output"},
        details="Expected root revolute, slide prismatic, and wrist revolute joints.",
    )

    ctx.expect_contact(frame, support, contact_tol=0.001, name="frame_hinge_is_seated")
    ctx.expect_contact(slider, frame, contact_tol=0.001, name="slider_contacts_guide")
    ctx.expect_contact(output, slider, contact_tol=0.001, name="output_is_mounted")

    with ctx.pose({root: 0.0, slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            slider,
            frame,
            axes="yz",
            margin=0.001,
            name="slider_cross_section_stays_within_frame",
        )

    with ctx.pose({root: 0.0, slide: 0.08, wrist: 0.0}):
        slider_rest = ctx.part_world_position(slider)
    with ctx.pose({root: 0.85, slide: 0.08, wrist: 0.0}):
        slider_lifted = ctx.part_world_position(slider)
    ctx.check(
        "positive_root_angle_lifts_frame",
        slider_rest is not None
        and slider_lifted is not None
        and slider_lifted[2] > slider_rest[2] + 0.06,
        details=f"slider z at q=0.85 should rise above q=0.0: rest={slider_rest}, lifted={slider_lifted}",
    )

    with ctx.pose({root: 0.0, slide: 0.0, wrist: 0.0}):
        output_retracted = ctx.part_world_position(output)
    with ctx.pose({root: 0.0, slide: SLIDE_TRAVEL, wrist: 0.0}):
        output_extended = ctx.part_world_position(output)
    ctx.check(
        "positive_slide_extends_forward",
        output_retracted is not None
        and output_extended is not None
        and output_extended[0] > output_retracted[0] + 0.16,
        details=(
            f"output x should increase as the prismatic stage extends: "
            f"retracted={output_retracted}, extended={output_extended}"
        ),
    )

    with ctx.pose({root: 0.35, slide: 0.10, wrist: -0.55}):
        pad_low_aabb = ctx.part_element_world_aabb(output, elem="service_pad")
    with ctx.pose({root: 0.35, slide: 0.10, wrist: 0.75}):
        pad_high_aabb = ctx.part_element_world_aabb(output, elem="service_pad")
    pad_low = _aabb_center(pad_low_aabb) if pad_low_aabb is not None else None
    pad_high = _aabb_center(pad_high_aabb) if pad_high_aabb is not None else None
    ctx.check(
        "positive_wrist_angle_lifts_service_pad",
        pad_low is not None and pad_high is not None and pad_high[2] > pad_low[2] + 0.04,
        details=f"service pad should lift as wrist rotates upward: low={pad_low}, high={pad_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
