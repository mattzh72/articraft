from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scratch_direct_drive_turntable")

    model.material("black_plinth", rgba=(0.025, 0.026, 0.028, 1.0))
    model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("rubber_black", rgba=(0.004, 0.004, 0.004, 1.0))
    model.material("vinyl_black", rgba=(0.010, 0.010, 0.012, 1.0))
    model.material("dark_groove", rgba=(0.035, 0.035, 0.038, 1.0))
    model.material("paper_label", rgba=(0.90, 0.16, 0.10, 1.0))
    model.material("white_mark", rgba=(0.92, 0.90, 0.82, 1.0))
    model.material("cartridge_black", rgba=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base")
    base_shape = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.50, 0.38, 0.035, corner_segments=10),
        0.070,
        cap=True,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(base_shape, "rounded_plinth"),
        material="black_plinth",
        name="rounded_plinth",
    )
    base.visual(
        Box((0.44, 0.31, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=Material("satin_top", rgba=(0.075, 0.078, 0.082, 1.0)),
        name="satin_top_plate",
    )
    # A fixed visible motor/bearing ring around the direct-drive platter.
    base.visual(
        mesh_from_geometry(TorusGeometry(0.166, 0.004, radial_segments=12, tubular_segments=96), "motor_trim_ring"),
        origin=Origin(xyz=(-0.090, 0.000, 0.0735)),
        material="brushed_aluminum",
        name="motor_trim_ring",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.056),
        origin=Origin(xyz=(0.150, 0.105, 0.097)),
        material="brushed_aluminum",
        name="tonearm_pedestal",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.150, 0.105, 0.120)),
        material="black_plinth",
        name="tonearm_bearing_cap",
    )
    # Printed selector ticks around the speed knob are intentionally part of the
    # stationary deck surface.
    for i, (dx, dy, yaw) in enumerate(
        (
            (-0.030, -0.003, -0.35),
            (0.000, 0.024, 0.00),
            (0.030, -0.003, 0.35),
        )
    ):
        base.visual(
            Box((0.018, 0.004, 0.0012)),
            origin=Origin(xyz=(0.150 + dx, -0.120 + dy, 0.0718), rpy=(0.0, 0.0, yaw)),
            material="white_mark",
            name=f"speed_tick_{i}",
        )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.155, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="brushed_aluminum",
        name="aluminum_platter",
    )
    platter.visual(
        Cylinder(radius=0.142, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0355)),
        material="vinyl_black",
        name="vinyl_disc",
    )
    for r in (0.070, 0.095, 0.120, 0.134):
        platter.visual(
            mesh_from_geometry(TorusGeometry(r, 0.00035, radial_segments=8, tubular_segments=96), f"record_groove_{int(r * 1000)}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0391)),
            material="dark_groove",
            name=f"groove_{int(r * 1000)}",
        )
    platter.visual(
        Cylinder(radius=0.032, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material="paper_label",
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.006, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material="brushed_aluminum",
        name="central_axle",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.021, length=0.021),
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
        material="brushed_aluminum",
        name="pivot_collar",
    )
    arm_tube = tube_from_spline_points(
        [
            (0.000, 0.000, 0.018),
            (-0.036, -0.006, 0.018),
            (-0.074, -0.042, 0.017),
            (-0.112, -0.037, 0.016),
            (-0.146, -0.098, 0.014),
            (-0.175, -0.165, 0.011),
        ],
        radius=0.004,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    tonearm.visual(
        mesh_from_geometry(arm_tube, "s_tonearm_tube"),
        material="brushed_aluminum",
        name="s_tonearm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.058),
        origin=Origin(xyz=(0.030, 0.014, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_aluminum",
        name="counterweight_stem",
    )
    tonearm.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(0.062, 0.014, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber_black",
        name="counterweight",
    )
    tonearm.visual(
        Box((0.044, 0.018, 0.006)),
        origin=Origin(xyz=(-0.181, -0.169, 0.009), rpy=(0.0, 0.0, -0.76)),
        material="brushed_aluminum",
        name="headshell",
    )
    tonearm.visual(
        Box((0.020, 0.012, 0.008)),
        origin=Origin(xyz=(-0.186, -0.174, 0.004), rpy=(0.0, 0.0, -0.76)),
        material="cartridge_black",
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0012, length=0.010),
        origin=Origin(xyz=(-0.186, -0.174, -0.002), rpy=(0.0, 0.0, 0.0)),
        material="white_mark",
        name="stylus_tip",
    )

    speed_knob = model.part("speed_knob")
    selector_knob = KnobGeometry(
        0.042,
        0.022,
        body_style="skirted",
        top_diameter=0.032,
        edge_radius=0.001,
        skirt=KnobSkirt(0.050, 0.005, flare=0.04, chamfer=0.0008),
        grip=KnobGrip(style="ribbed", count=18, depth=0.0008, width=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        center=False,
    )
    speed_knob.visual(
        mesh_from_geometry(selector_knob, "speed_selector_knob"),
        material="rubber_black",
        name="selector_cap",
    )

    model.articulation(
        "base_to_platter",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=platter,
        origin=Origin(xyz=(-0.090, 0.000, 0.0725)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=12.0),
    )
    model.articulation(
        "base_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tonearm,
        origin=Origin(xyz=(0.150, 0.105, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=1.4, lower=-0.42, upper=0.58),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(0.150, -0.120, 0.0725)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=2.0, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    speed_knob = object_model.get_part("speed_knob")
    platter_joint = object_model.get_articulation("base_to_platter")
    arm_joint = object_model.get_articulation("base_to_tonearm")
    knob_joint = object_model.get_articulation("base_to_speed_knob")

    ctx.expect_gap(
        platter,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="aluminum_platter",
        negative_elem="satin_top_plate",
        name="platter rides just above plinth",
    )
    ctx.expect_gap(
        speed_knob,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="selector_cap",
        negative_elem="satin_top_plate",
        name="speed knob sits on deck",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        min_gap=0.006,
        positive_elem="headshell",
        negative_elem="vinyl_disc",
        name="headshell clears record surface",
    )
    rest_pos = ctx.part_world_position(tonearm)
    with ctx.pose({arm_joint: 0.45, platter_joint: 1.2, knob_joint: 0.5}):
        swept_pos = ctx.part_world_position(tonearm)
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            min_gap=0.004,
            positive_elem="headshell",
            negative_elem="vinyl_disc",
            name="swept tonearm still clears record",
        )
    ctx.check(
        "tonearm pivot pose remains anchored",
        rest_pos is not None and swept_pos is not None and abs(rest_pos[2] - swept_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    return ctx.report()


object_model = build_object_model()
