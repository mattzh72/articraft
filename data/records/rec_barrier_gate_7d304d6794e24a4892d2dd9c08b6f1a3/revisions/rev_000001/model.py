from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_lift_barrier")

    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.61, 0.60, 1.0))
    dark_steel = model.material("dark_painted_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    mesh_steel = model.material("woven_steel_mesh", rgba=(0.35, 0.38, 0.38, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    concrete = model.material("counterweight_gray", rgba=(0.28, 0.29, 0.28, 1.0))
    cable_mat = model.material("dark_wire_rope", rgba=(0.02, 0.02, 0.018, 1.0))

    frame = model.part("frame")

    def box(part, name, size, xyz, material):
        part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # Ground sill and upper header tie the two main guide columns into one rigid frame.
    box(frame, "base_sill", (2.34, 0.18, 0.08), (0.0, 0.0, 0.04), dark_steel)
    box(frame, "top_header", (2.34, 0.18, 0.08), (0.0, 0.0, 2.42), dark_steel)

    # Main C-channel guide columns.  Front and rear lips leave a clear vertical slot
    # for the sliding barrier side rails.
    for side, sx, lip_x in (("left", -1.04, -0.92), ("right", 1.04, 0.92)):
        box(frame, f"{side}_outer_web", (0.08, 0.16, 2.40), (sx, 0.0, 1.20), galvanized)
        box(frame, f"{side}_front_lip", (0.16, 0.025, 2.34), (lip_x, 0.060, 1.20), galvanized)
        box(frame, f"{side}_rear_lip", (0.16, 0.025, 2.34), (lip_x, -0.060, 1.20), galvanized)
        box(frame, f"{side}_wear_strip", (0.028, 0.010, 2.20), (0.90 if side == "right" else -0.90, 0.0425, 1.20), rubber)

    # Rear counterweight guide cage and tie beams back to the main frame.
    for x in (1.18, 1.52):
        for y in (0.39, 0.71):
            box(frame, f"rear_guide_post_{x:.2f}_{y:.2f}", (0.04, 0.04, 2.35), (x, y, 1.225), galvanized)
    box(frame, "rear_guide_bottom_cap", (0.42, 0.36, 0.06), (1.35, 0.55, 0.08), dark_steel)
    box(frame, "rear_guide_rear_top_cap", (0.42, 0.04, 0.06), (1.35, 0.75, 2.43), dark_steel)
    box(frame, "lower_rear_tie_y", (0.08, 0.62, 0.06), (1.04, 0.27, 0.08), dark_steel)
    box(frame, "upper_rear_tie_y", (0.08, 0.62, 0.06), (1.04, 0.27, 2.42), dark_steel)
    box(frame, "lower_rear_tie_x", (0.36, 0.08, 0.06), (1.19, 0.55, 0.08), dark_steel)
    box(frame, "upper_rear_tie_x", (0.36, 0.08, 0.06), (1.19, 0.55, 2.42), dark_steel)

    # Pulley yoke fixed to the rear guide cap.
    box(frame, "pulley_left_cheek", (0.015, 0.33, 0.36), (1.220, 0.55, 2.61), dark_steel)
    box(frame, "pulley_right_cheek", (0.015, 0.33, 0.36), (1.480, 0.55, 2.61), dark_steel)
    box(frame, "pulley_yoke_bridge", (0.29, 0.07, 0.07), (1.35, 0.55, 2.825), dark_steel)

    barrier = model.part("barrier_panel")
    # Rectangular frame captured between the guide-channel lips.
    box(barrier, "left_slide_rail", (0.050, 0.035, 1.25), (-0.89, 0.0, 0.625), safety_yellow)
    box(barrier, "right_slide_rail", (0.050, 0.035, 1.25), (0.89, 0.0, 0.625), safety_yellow)
    box(barrier, "left_guide_shoe", (0.045, 0.020, 0.14), (-0.89, 0.0275, 0.25), rubber)
    box(barrier, "right_guide_shoe", (0.045, 0.020, 0.14), (0.89, 0.0275, 0.25), rubber)
    box(barrier, "top_rail", (1.83, 0.035, 0.050), (0.0, 0.0, 1.225), safety_yellow)
    box(barrier, "bottom_rail", (1.83, 0.035, 0.060), (0.0, 0.0, 0.030), safety_yellow)

    # Mesh grid: individual rods are welded into the surrounding frame and to each other.
    for i, x in enumerate([-0.68, -0.51, -0.34, -0.17, 0.0, 0.17, 0.34, 0.51, 0.68]):
        box(barrier, f"mesh_vertical_{i}", (0.012, 0.012, 1.12), (x, 0.0, 0.625), mesh_steel)
    for i, z in enumerate([0.22, 0.40, 0.58, 0.76, 0.94, 1.12]):
        box(barrier, f"mesh_horizontal_{i}", (1.73, 0.010, 0.010), (0.0, 0.0, z), mesh_steel)

    # Cable anchor welded to the moving panel top rail.
    box(barrier, "cable_tab", (0.08, 0.04, 0.09), (0.75, 0.035, 1.285), safety_yellow)
    barrier.visual(
        Cylinder(radius=0.014, length=0.10),
        origin=Origin(xyz=(0.75, 0.069, 1.320), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="cable_anchor_pin",
    )

    counterweight = model.part("counterweight")
    box(counterweight, "weight_block", (0.24, 0.20, 0.45), (0.0, 0.0, 0.0), concrete)
    box(counterweight, "top_eye", (0.08, 0.025, 0.11), (0.0, 0.105, 0.275), dark_steel)
    counterweight.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.0, 0.125, 0.325), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="weight_cable_pin",
    )

    pulley = model.part("pulley")
    for name, radius, length, x in (
        ("sheave", 0.130, 0.042, 0.0),
        ("hub", 0.046, 0.245, 0.0),
        ("left_flange", 0.138, 0.010, -0.030),
        ("right_flange", 0.138, 0.010, 0.030),
    ):
        pulley.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized if "flange" in name else dark_steel,
            name=name,
        )

    cable = model.part("cable")
    cable_points = [
        (0.75, 0.070, 1.320),
        (0.98, 0.180, 1.92),
        (1.18, 0.300, 2.38),
        (1.35, 0.411, 2.620),
        (1.35, 0.434, 2.690),
        (1.35, 0.500, 2.745),
        (1.35, 0.550, 2.759),
        (1.35, 0.600, 2.745),
        (1.35, 0.666, 2.690),
        (1.35, 0.689, 2.620),
        (1.35, 0.689, 2.075),
    ]
    cable.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                cable_points,
                radius=0.009,
                samples_per_segment=8,
                radial_segments=16,
                cap_ends=True,
            ),
            "wire_rope_over_pulley",
        ),
        material=cable_mat,
        name="wire_rope",
    )

    panel_lift = model.articulation(
        "panel_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=barrier,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.80),
    )
    model.articulation(
        "counterweight_travel",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=counterweight,
        origin=Origin(xyz=(1.35, 0.55, 1.75)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.80),
        mimic=Mimic(joint="panel_lift", multiplier=1.0, offset=0.0),
    )
    model.articulation(
        "pulley_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=pulley,
        origin=Origin(xyz=(1.35, 0.55, 2.62)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=12.0),
    )
    model.articulation(
        "frame_to_cable",
        ArticulationType.FIXED,
        parent=frame,
        child=cable,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    barrier = object_model.get_part("barrier_panel")
    counterweight = object_model.get_part("counterweight")
    pulley = object_model.get_part("pulley")
    cable = object_model.get_part("cable")
    panel_lift = object_model.get_articulation("panel_lift")

    ctx.allow_overlap(
        barrier,
        cable,
        elem_a="cable_anchor_pin",
        elem_b="wire_rope",
        reason="The wire rope is intentionally captured around the panel's anchor pin.",
    )
    ctx.allow_overlap(
        cable,
        counterweight,
        elem_a="wire_rope",
        elem_b="weight_cable_pin",
        reason="The wire rope is intentionally captured around the counterweight cable pin.",
    )
    ctx.allow_overlap(
        cable,
        pulley,
        elem_a="wire_rope",
        elem_b="sheave",
        reason="The cable is seated into the simplified solid sheave groove.",
    )

    with ctx.pose({panel_lift: 0.0}):
        ctx.expect_gap(
            frame,
            barrier,
            axis="y",
            positive_elem="left_front_lip",
            negative_elem="left_slide_rail",
            min_gap=0.015,
            max_gap=0.040,
            name="left front guide lip clears sliding rail",
        )
        ctx.expect_gap(
            barrier,
            frame,
            axis="y",
            positive_elem="left_slide_rail",
            negative_elem="left_rear_lip",
            min_gap=0.015,
            max_gap=0.040,
            name="left rear guide lip clears sliding rail",
        )
        ctx.expect_overlap(
            barrier,
            frame,
            axes="z",
            elem_a="left_slide_rail",
            elem_b="left_front_lip",
            min_overlap=1.0,
            name="panel rail is retained in left guide at rest",
        )
        ctx.expect_overlap(
            counterweight,
            frame,
            axes="z",
            elem_a="weight_block",
            elem_b="rear_guide_post_1.18_0.39",
            min_overlap=0.40,
            name="counterweight remains inside rear guide at rest",
        )
        ctx.expect_overlap(
            pulley,
            cable,
            axes="yz",
            elem_a="sheave",
            elem_b="wire_rope",
            min_overlap=0.05,
            name="wire rope wraps across the pulley sheave",
        )
        ctx.expect_gap(
            cable,
            barrier,
            axis="y",
            positive_elem="wire_rope",
            negative_elem="cable_anchor_pin",
            max_penetration=0.025,
            name="panel cable pin captures wire rope locally",
        )
        ctx.expect_overlap(
            cable,
            counterweight,
            axes="xz",
            elem_a="wire_rope",
            elem_b="weight_cable_pin",
            min_overlap=0.012,
            name="counterweight cable pin captures wire rope locally",
        )

    rest_panel = ctx.part_world_position(barrier)
    rest_weight = ctx.part_world_position(counterweight)
    with ctx.pose({panel_lift: 0.80}):
        ctx.expect_overlap(
            barrier,
            frame,
            axes="z",
            elem_a="left_slide_rail",
            elem_b="left_front_lip",
            min_overlap=0.45,
            name="raised panel stays captured in the guide",
        )
        ctx.expect_overlap(
            counterweight,
            frame,
            axes="z",
            elem_a="weight_block",
            elem_b="rear_guide_post_1.18_0.39",
            min_overlap=0.40,
            name="lowered counterweight stays captured in rear guide",
        )
        raised_panel = ctx.part_world_position(barrier)
        lowered_weight = ctx.part_world_position(counterweight)

    ctx.check(
        "panel rises while counterweight descends",
        rest_panel is not None
        and raised_panel is not None
        and rest_weight is not None
        and lowered_weight is not None
        and raised_panel[2] > rest_panel[2] + 0.75
        and lowered_weight[2] < rest_weight[2] - 0.75,
        details=f"panel rest/raised={rest_panel}/{raised_panel}, counterweight rest/lowered={rest_weight}/{lowered_weight}",
    )

    return ctx.report()


object_model = build_object_model()
