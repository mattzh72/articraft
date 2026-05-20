from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="irrigation_sluice_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.60, 0.64, 0.67, 1.0))
    painted = model.material("painted_steel", rgba=(0.23, 0.31, 0.34, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.21, 0.23, 1.0))
    rubber = model.material("rubber_grip", rgba=(0.08, 0.08, 0.09, 1.0))

    opening_width = 0.72
    post_width = 0.085
    frame_depth = 0.12
    sill_height = 0.10
    rail_height = 1.24
    top_beam_height = 0.12
    opening_half = opening_width * 0.5
    post_center_x = opening_half + post_width * 0.5
    rail_center_z = sill_height + rail_height * 0.5
    top_beam_center_z = sill_height + rail_height + top_beam_height * 0.5

    panel_bottom = 0.102
    panel_height = 0.84
    panel_slide_upper = 0.32

    frame = model.part("frame")
    frame.visual(
        Box((opening_width + 2.0 * post_width + 0.03, 0.16, sill_height)),
        origin=Origin(xyz=(0.0, 0.0, sill_height * 0.5)),
        material=galvanized,
        name="sill",
    )
    frame.visual(
        Box((post_width, frame_depth, rail_height)),
        origin=Origin(xyz=(-post_center_x, 0.0, rail_center_z)),
        material=galvanized,
        name="left_post",
    )
    frame.visual(
        Box((post_width, frame_depth, rail_height)),
        origin=Origin(xyz=(post_center_x, 0.0, rail_center_z)),
        material=galvanized,
        name="right_post",
    )
    frame.visual(
        Box((opening_width + 2.0 * post_width + 0.03, 0.18, top_beam_height)),
        origin=Origin(xyz=(0.0, 0.0, top_beam_center_z)),
        material=galvanized,
        name="top_bracket",
    )
    frame.visual(
        Box((opening_width + 2.0 * post_width + 0.07, 0.20, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, top_beam_center_z + 0.072)),
        material=painted,
        name="cap_plate",
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Box((0.026, 0.008, rail_height - 0.06)),
            origin=Origin(
                xyz=(
                    sign * 0.350,
                    -0.016,
                    sill_height + (rail_height - 0.06) * 0.5,
                )
            ),
            material=painted,
            name=f"{side}_rear_keeper",
        )
        frame.visual(
            Box((0.026, 0.008, rail_height - 0.06)),
            origin=Origin(
                xyz=(
                    sign * 0.350,
                    0.016,
                    sill_height + (rail_height - 0.06) * 0.5,
                )
            ),
            material=painted,
            name=f"{side}_front_keeper",
        )

    frame.visual(
        Box((0.18, 0.10, 0.14)),
        origin=Origin(xyz=(0.0, 0.006, top_beam_center_z - 0.008)),
        material=painted,
        name="gearbox",
    )
    frame.visual(
        Box((0.028, 0.040, 0.080)),
        origin=Origin(xyz=(-0.060, 0.084, top_beam_center_z - 0.008)),
        material=painted,
        name="left_wheel_cheek",
    )
    frame.visual(
        Box((0.028, 0.040, 0.080)),
        origin=Origin(xyz=(0.060, 0.084, top_beam_center_z - 0.008)),
        material=painted,
        name="right_wheel_cheek",
    )
    frame.visual(
        Box((0.032, 0.060, 0.110)),
        origin=Origin(xyz=(0.422, 0.072, top_beam_center_z - 0.040)),
        material=painted,
        name="latch_bracket",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.035),
        origin=Origin(
            xyz=(0.0, 0.0755, top_beam_center_z - 0.008),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=dark_steel,
        name="axle_stub",
    )
    frame.visual(
        Box((0.032, 0.024, 0.024)),
        origin=Origin(xyz=(-0.030, 0.0825, top_beam_center_z - 0.008)),
        material=painted,
        name="left_axle_arm",
    )
    frame.visual(
        Box((0.032, 0.024, 0.024)),
        origin=Origin(xyz=(0.030, 0.0825, top_beam_center_z - 0.008)),
        material=painted,
        name="right_axle_arm",
    )

    panel = model.part("panel")
    panel.visual(
        Box((0.66, 0.016, panel_height)),
        origin=Origin(xyz=(0.0, 0.0, panel_height * 0.5)),
        material=painted,
        name="gate_plate",
    )
    panel.visual(
        Box((0.028, 0.024, panel_height)),
        origin=Origin(xyz=(-0.341, 0.0, panel_height * 0.5)),
        material=dark_steel,
        name="left_runner",
    )
    panel.visual(
        Box((0.028, 0.024, panel_height)),
        origin=Origin(xyz=(0.341, 0.0, panel_height * 0.5)),
        material=dark_steel,
        name="right_runner",
    )
    panel.visual(
        Box((0.54, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, 0.086, panel_height - 0.035)),
        material=galvanized,
        name="lift_beam",
    )
    panel.visual(
        Box((0.14, 0.034, 0.060)),
        origin=Origin(xyz=(0.0, 0.078, panel_height - 0.085)),
        material=galvanized,
        name="stem_clevis",
    )
    panel.visual(
        Box((0.18, 0.080, 0.120)),
        origin=Origin(xyz=(0.0, 0.046, panel_height - 0.115)),
        material=galvanized,
        name="mount_plate",
    )
    panel.visual(
        Box((0.024, 0.024, 0.500)),
        origin=Origin(xyz=(0.326, 0.088, 0.580)),
        material=dark_steel,
        name="rack_bar",
    )
    panel.visual(
        Box((0.240, 0.024, 0.040)),
        origin=Origin(xyz=(0.208, 0.088, 0.760)),
        material=dark_steel,
        name="rack_bridge",
    )
    tooth_heights = (0.38, 0.50, 0.62, 0.738)
    for index, tooth_z in enumerate(tooth_heights):
        panel.visual(
            Box((0.060, 0.024, 0.020)),
            origin=Origin(xyz=(0.326, 0.088, tooth_z)),
            material=dark_steel,
            name=f"rack_tooth_{index}",
        )

    wheel = model.part("wheel")
    wheel_rim = mesh_from_geometry(TorusGeometry(0.145, 0.012), "sluice_wheel_rim")
    wheel_hub = mesh_from_cadquery(
        cq.Workplane("XZ").circle(0.036).extrude(0.035, both=True).cut(
            cq.Workplane("XZ").circle(0.016).extrude(0.040, both=True)
        ),
        "sluice_wheel_hub",
    )
    wheel.visual(
        wheel_rim,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=galvanized,
        name="rim",
    )
    wheel.visual(
        wheel_hub,
        origin=Origin(),
        material=dark_steel,
        name="hub",
    )
    wheel.visual(
        Box((0.280, 0.014, 0.018)),
        origin=Origin(),
        material=galvanized,
        name="spoke_horizontal",
    )
    wheel.visual(
        Box((0.018, 0.014, 0.280)),
        origin=Origin(),
        material=galvanized,
        name="spoke_vertical",
    )
    wheel.visual(
        Box((0.018, 0.050, 0.018)),
        origin=Origin(xyz=(0.100, 0.025, 0.100)),
        material=dark_steel,
        name="crank_arm",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(
            xyz=(0.100, 0.065, 0.100),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=rubber,
        name="crank_grip",
    )

    latch = model.part("latch")
    latch.visual(
        Box((0.100, 0.020, 0.020)),
        origin=Origin(xyz=(-0.050, 0.0, -0.010)),
        material=painted,
        name="latch_head",
    )
    latch.visual(
        Box((0.020, 0.020, 0.230)),
        origin=Origin(xyz=(-0.050, 0.0, -0.114)),
        material=painted,
        name="latch_bar",
    )
    latch.visual(
        Box((0.052, 0.020, 0.016)),
        origin=Origin(xyz=(-0.050, 0.0, -0.228)),
        material=dark_steel,
        name="latch_toe",
    )

    panel_slide = model.articulation(
        "panel_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, panel_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.18,
            lower=0.0,
            upper=panel_slide_upper,
        ),
    )
    wheel_spin = model.articulation(
        "wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.128, top_beam_center_z - 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=8.0,
        ),
    )
    latch_pivot = model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=latch,
        origin=Origin(xyz=(0.406, 0.102, top_beam_center_z - 0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.75,
            upper=0.30,
        ),
    )

    panel_slide.meta["selected_lock_height"] = 0.30
    wheel_spin.meta["qc_samples"] = [0.0, math.pi * 0.5]
    latch_pivot.meta["selected_lock_angle"] = 0.0

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    wheel = object_model.get_part("wheel")
    latch = object_model.get_part("latch")

    panel_slide = object_model.get_articulation("panel_slide")
    wheel_spin = object_model.get_articulation("wheel_spin")
    latch_pivot = object_model.get_articulation("latch_pivot")

    lock_height = 0.30

    with ctx.pose({panel_slide: 0.0}):
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="gate_plate",
            negative_elem="sill",
            min_gap=0.001,
            max_gap=0.004,
            name="panel seats close to the sill when closed",
        )
        ctx.expect_contact(
            wheel,
            frame,
            contact_tol=0.0025,
            elem_a="hub",
            elem_b="axle_stub",
            name="wheel hub remains seated on the top bracket axle",
        )
        rest_pos = ctx.part_world_position(panel)

    with ctx.pose({panel_slide: lock_height, latch_pivot: 0.0}):
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="gate_plate",
            negative_elem="sill",
            min_gap=lock_height + 0.001,
            max_gap=lock_height + 0.004,
            name="raised panel clears the sill by the lift travel",
        )
        ctx.expect_contact(
            latch,
            panel,
            elem_a="latch_toe",
            elem_b="rack_tooth_3",
            name="latch toe can catch the selected panel rack tooth",
        )
        raised_pos = ctx.part_world_position(panel)

    ctx.check(
        "panel travels upward along the slide rails",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.25,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({wheel_spin: math.pi * 0.5}):
        ctx.expect_contact(
            wheel,
            frame,
            contact_tol=0.0025,
            elem_a="hub",
            elem_b="axle_stub",
            name="wheel stays mounted while turning",
        )

    return ctx.report()


object_model = build_object_model()
