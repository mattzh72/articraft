from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_water_control_gate")

    concrete = model.material("weathered_concrete", rgba=(0.48, 0.50, 0.48, 1.0))
    dark_concrete = model.material("damp_concrete_stain", rgba=(0.22, 0.24, 0.23, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.63, 0.66, 1.0))
    dark_steel = model.material("dark_oiled_steel", rgba=(0.06, 0.07, 0.075, 1.0))
    painted_red = model.material("red_painted_handwheel", rgba=(0.78, 0.06, 0.035, 1.0))
    gear_blue = model.material("blue_gray_gear_housing", rgba=(0.20, 0.27, 0.32, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.015, 0.014, 0.012, 1.0))
    brass = model.material("worn_brass_label", rgba=(0.72, 0.58, 0.25, 1.0))

    frame = model.part("frame")

    # A civil-scale concrete headwall: side piers, a sill, and a split top bridge
    # leave a wide rectangular water opening through the center.
    frame.visual(
        Box((0.65, 1.10, 4.65)),
        origin=Origin(xyz=(-2.425, 0.0, 2.325)),
        material=concrete,
        name="pier_0",
    )
    frame.visual(
        Box((0.65, 1.10, 4.65)),
        origin=Origin(xyz=(2.425, 0.0, 2.325)),
        material=concrete,
        name="pier_1",
    )
    frame.visual(
        Box((5.50, 1.10, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=concrete,
        name="sill",
    )
    frame.visual(
        Box((5.50, 0.25, 0.60)),
        origin=Origin(xyz=(0.0, -0.45, 4.50)),
        material=concrete,
        name="front_bridge_beam",
    )
    frame.visual(
        Box((5.50, 0.25, 0.60)),
        origin=Origin(xyz=(0.0, 0.45, 4.50)),
        material=concrete,
        name="rear_bridge_beam",
    )
    frame.visual(
        Box((0.28, 0.18, 3.70)),
        origin=Origin(xyz=(-2.00, -0.19, 2.40)),
        material=dark_steel,
        name="front_track_0",
    )
    frame.visual(
        Box((0.28, 0.18, 3.70)),
        origin=Origin(xyz=(2.00, -0.19, 2.40)),
        material=dark_steel,
        name="front_track_1",
    )
    frame.visual(
        Box((0.28, 0.18, 3.70)),
        origin=Origin(xyz=(-2.00, 0.19, 2.40)),
        material=dark_steel,
        name="rear_track_0",
    )
    frame.visual(
        Box((0.28, 0.18, 3.70)),
        origin=Origin(xyz=(2.00, 0.19, 2.40)),
        material=dark_steel,
        name="rear_track_1",
    )
    frame.visual(
        Box((0.16, 0.56, 3.70)),
        origin=Origin(xyz=(-2.10, 0.0, 2.40)),
        material=dark_steel,
        name="track_web_0",
    )
    frame.visual(
        Box((0.16, 0.56, 3.70)),
        origin=Origin(xyz=(2.10, 0.0, 2.40)),
        material=dark_steel,
        name="track_web_1",
    )
    frame.visual(
        Box((4.10, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, -0.56, 0.60)),
        material=dark_concrete,
        name="wet_sill_line",
    )
    frame.visual(
        Box((4.25, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, -0.58, 3.97)),
        material=dark_concrete,
        name="shadow_under_bridge",
    )

    # Gear unit on the top bridge.
    frame.visual(
        Box((1.30, 0.57, 0.85)),
        origin=Origin(xyz=(0.0, -0.59, 5.225)),
        material=gear_blue,
        name="housing_box",
    )
    frame.visual(
        Box((1.55, 0.42, 0.14)),
        origin=Origin(xyz=(0.0, -0.48, 4.87)),
        material=dark_steel,
        name="housing_base_plate",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.27, tube=0.045, radial_segments=24, tubular_segments=64), "gear_cover_ring"),
        origin=Origin(xyz=(0.0, -0.895, 5.27), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gear_blue,
        name="gear_cover_ring",
    )
    frame.visual(
        Cylinder(radius=0.14, length=0.18),
        origin=Origin(xyz=(0.0, -0.965, 5.27), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_boss",
    )
    frame.visual(
        Box((0.44, 0.035, 0.16)),
        origin=Origin(xyz=(0.0, -0.88, 5.58)),
        material=brass,
        name="cast_label_plate",
    )

    # A simple rack guide over the bridge shows how the rising gate is driven.
    frame.visual(
        Box((0.08, 0.10, 0.36)),
        origin=Origin(xyz=(-0.20, -0.20, 4.72)),
        material=dark_steel,
        name="rack_guide_cheek_0",
    )
    frame.visual(
        Box((0.08, 0.10, 0.36)),
        origin=Origin(xyz=(0.20, -0.20, 4.72)),
        material=dark_steel,
        name="rack_guide_cheek_1",
    )
    frame.visual(
        Box((0.08, 0.20, 0.08)),
        origin=Origin(xyz=(-0.20, -0.30, 4.86)),
        material=dark_steel,
        name="rack_guide_arm_0",
    )
    frame.visual(
        Box((0.08, 0.20, 0.08)),
        origin=Origin(xyz=(0.20, -0.30, 4.86)),
        material=dark_steel,
        name="rack_guide_arm_1",
    )

    panel = model.part("panel")
    panel.visual(
        Box((3.64, 0.14, 3.24)),
        origin=Origin(xyz=(0.0, 0.0, 1.62)),
        material=galvanized,
        name="steel_plate",
    )
    for idx, z in enumerate((0.55, 1.35, 2.15, 2.95)):
        panel.visual(
            Box((3.40, 0.055, 0.14)),
            origin=Origin(xyz=(0.0, -0.095, z)),
            material=dark_steel,
            name=f"horizontal_rib_{idx}",
        )
    panel.visual(
        Box((0.09, 0.20, 3.28)),
        origin=Origin(xyz=(-1.865, 0.0, 1.64)),
        material=dark_steel,
        name="side_shoe_0",
    )
    panel.visual(
        Box((0.09, 0.20, 3.28)),
        origin=Origin(xyz=(1.865, 0.0, 1.64)),
        material=dark_steel,
        name="side_shoe_1",
    )
    panel.visual(
        Box((3.50, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=rubber,
        name="bottom_seal",
    )
    panel.visual(
        Box((0.55, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, -0.105, 3.31)),
        material=dark_steel,
        name="top_clevis_block",
    )
    panel.visual(
        Box((0.14, 0.08, 1.25)),
        origin=Origin(xyz=(0.0, -0.20, 3.95)),
        material=dark_steel,
        name="lift_rack_stem",
    )
    for idx, z in enumerate((3.45, 3.62, 3.79, 3.96, 4.13, 4.30)):
        panel.visual(
            Box((0.05, 0.04, 0.055)),
            origin=Origin(xyz=(0.095, -0.245, z)),
            material=galvanized,
            name=f"rack_tooth_{idx}",
        )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_geometry(TorusGeometry(radius=0.42, tube=0.035, radial_segments=24, tubular_segments=80), "handwheel_rim"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_red,
        name="rim",
    )
    handwheel.visual(
        Cylinder(radius=0.12, length=0.16),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_red,
        name="hub",
    )
    handwheel.visual(
        Cylinder(radius=0.055, length=0.284),
        origin=Origin(xyz=(0.0, 0.142, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle",
    )
    for idx, rpy in enumerate(((0.0, 0.0, 0.0), (0.0, math.pi / 2.0, 0.0), (0.0, math.pi / 4.0, 0.0), (0.0, -math.pi / 4.0, 0.0))):
        handwheel.visual(
            Box((0.86, 0.045, 0.045)),
            origin=Origin(rpy=rpy),
            material=painted_red,
            name=f"spoke_{idx}",
        )
    handwheel.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(xyz=(0.0, -0.08, 0.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=painted_red,
        name="grip_post",
    )
    handwheel.visual(
        Cylinder(radius=0.045, length=0.22),
        origin=Origin(xyz=(0.0, -0.25, 0.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spin_grip",
    )

    flap = model.part("inspection_flap")
    flap.visual(
        Box((0.055, 0.45, 0.36)),
        origin=Origin(xyz=(0.0275, 0.225, 0.0)),
        material=gear_blue,
        name="flap_plate",
    )
    flap.visual(
        Cylinder(radius=0.035, length=0.46),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.025, 0.07, 0.08)),
        origin=Origin(xyz=(0.065, 0.34, 0.0)),
        material=dark_steel,
        name="pull_lip",
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=0.12, lower=0.0, upper=1.85),
    )
    model.articulation(
        "frame_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=handwheel,
        origin=Origin(xyz=(0.0, -1.16, 5.27)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.5),
    )
    model.articulation(
        "frame_to_inspection_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.65, -0.82, 5.225)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    handwheel = object_model.get_part("handwheel")
    flap = object_model.get_part("inspection_flap")
    panel_slide = object_model.get_articulation("frame_to_panel")
    wheel_spin = object_model.get_articulation("frame_to_handwheel")
    flap_hinge = object_model.get_articulation("frame_to_inspection_flap")

    ctx.allow_overlap(
        frame,
        handwheel,
        elem_a="bearing_boss",
        elem_b="axle",
        reason="The handwheel axle is intentionally captured inside the gearbox bearing boss.",
    )
    ctx.expect_within(
        handwheel,
        frame,
        axes="xz",
        inner_elem="axle",
        outer_elem="bearing_boss",
        margin=0.001,
        name="axle is centered in bearing boss",
    )
    ctx.expect_overlap(
        handwheel,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="bearing_boss",
        min_overlap=0.12,
        name="axle remains inserted in bearing boss",
    )

    ctx.expect_overlap(
        panel,
        frame,
        axes="z",
        elem_a="side_shoe_0",
        elem_b="front_track_0",
        min_overlap=3.0,
        name="panel side shoe is retained in left track",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="z",
        elem_a="side_shoe_1",
        elem_b="front_track_1",
        min_overlap=3.0,
        name="panel side shoe is retained in right track",
    )
    ctx.expect_overlap(
        panel,
        frame,
        axes="x",
        elem_a="steel_plate",
        elem_b="sill",
        min_overlap=3.4,
        name="lowered panel spans the wide opening",
    )

    rest_panel_pos = ctx.part_world_position(panel)
    rest_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_plate")
    with ctx.pose({panel_slide: 1.25, flap_hinge: 0.95, wheel_spin: math.pi}):
        raised_panel_pos = ctx.part_world_position(panel)
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_plate")
        ctx.expect_overlap(
            panel,
            frame,
            axes="z",
            elem_a="side_shoe_0",
            elem_b="front_track_0",
            min_overlap=1.4,
            name="raised gate remains captured by track",
        )
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem="steel_plate",
            negative_elem="sill",
            min_gap=1.20,
            name="raised panel opens waterway above sill",
        )

    ctx.check(
        "panel moves upward on prismatic slide",
        rest_panel_pos is not None
        and raised_panel_pos is not None
        and raised_panel_pos[2] > rest_panel_pos[2] + 1.0,
        details=f"rest={rest_panel_pos}, raised={raised_panel_pos}",
    )
    ctx.check(
        "inspection flap swings away from housing side",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][0] > rest_flap_aabb[1][0] + 0.08,
        details=f"closed={rest_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
