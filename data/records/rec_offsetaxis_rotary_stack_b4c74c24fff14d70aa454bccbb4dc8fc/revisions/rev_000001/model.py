from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 72, *, cx: float = 0.0, cy: float = 0.0):
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annular_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, 96),
            [_circle_profile(inner_radius, 64)],
            height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_bridge_rotary_stack")

    cast_iron = model.material("dark_cast_iron", color=(0.08, 0.09, 0.095, 1.0))
    gunmetal = model.material("gunmetal_housing", color=(0.25, 0.27, 0.28, 1.0))
    machined = model.material("machined_steel", color=(0.67, 0.68, 0.65, 1.0))
    black = model.material("black_oxide_fasteners", color=(0.015, 0.015, 0.014, 1.0))
    bronze = model.material("oiled_bronze_bushing", color=(0.62, 0.42, 0.19, 1.0))
    bridge_blue = model.material("painted_bridge_blue", color=(0.05, 0.16, 0.31, 1.0))
    warning = model.material("orange_alignment_marks", color=(0.94, 0.38, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.285, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_iron,
        name="floor_plinth",
    )
    base.visual(
        _annular_mesh(0.205, 0.112, 0.080, "base_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=gunmetal,
        name="base_bearing_ring",
    )
    base.visual(
        Cylinder(radius=0.228, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=machined,
        name="lower_thrust_race",
    )
    for i, (x, y, sx, sy) in enumerate(
        (
            (0.300, 0.0, 0.160, 0.085),
            (-0.300, 0.0, 0.160, 0.085),
            (0.0, 0.300, 0.085, 0.160),
            (0.0, -0.300, 0.085, 0.160),
        )
    ):
        base.visual(
            Box((sx, sy, 0.028)),
            origin=Origin(xyz=(x, y, 0.018)),
            material=cast_iron,
            name=f"anchor_ear_{i}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(x, y, 0.036)),
            material=black,
            name=f"anchor_screw_{i}",
        )
    for i in range(8):
        a = 2.0 * math.pi * i / 8.0 + math.pi / 8.0
        base.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(0.165 * math.cos(a), 0.165 * math.sin(a), 0.1385)),
            material=black,
            name=f"base_cap_screw_{i}",
        )

    lower_drum = model.part("lower_drum")
    lower_drum.visual(
        Cylinder(radius=0.145, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=gunmetal,
        name="drum_shell",
    )
    lower_drum.visual(
        Cylinder(radius=0.188, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=machined,
        name="bottom_flange",
    )
    lower_drum.visual(
        Cylinder(radius=0.188, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.217)),
        material=machined,
        name="top_flange",
    )
    lower_drum.visual(
        Cylinder(radius=0.086, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=bronze,
        name="lower_journal",
    )
    lower_drum.visual(
        Cylinder(radius=0.075, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.246)),
        material=machined,
        name="top_boss",
    )
    for i in range(10):
        a = 2.0 * math.pi * i / 10.0
        lower_drum.visual(
            Box((0.020, 0.038, 0.132)),
            origin=Origin(
                xyz=(0.149 * math.cos(a), 0.149 * math.sin(a), 0.116),
                rpy=(0.0, 0.0, a),
            ),
            material=cast_iron,
            name=f"drum_side_rib_{i}",
        )
    for i, y in enumerate((-0.168, 0.168)):
        lower_drum.visual(
            Box((0.245, 0.036, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.236)),
            material=gunmetal,
            name=f"split_retainer_{i}",
        )
        for j, x in enumerate((-0.078, 0.078)):
            lower_drum.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(xyz=(x, y, 0.253)),
                material=black,
                name=f"retainer_screw_{i}_{j}",
            )
    for i in range(12):
        a = 2.0 * math.pi * i / 12.0
        lower_drum.visual(
            Cylinder(radius=0.009, length=0.009),
            origin=Origin(xyz=(0.158 * math.cos(a), 0.158 * math.sin(a), 0.2265)),
            material=black,
            name=f"drum_cap_screw_{i}",
        )

    bridge = model.part("bridge")
    bridge.visual(
        Box((0.370, 0.180, 0.060)),
        origin=Origin(xyz=(0.095, 0.0, 0.260)),
        material=bridge_blue,
        name="bridge_saddle",
    )
    bridge.visual(
        Box((0.570, 0.090, 0.080)),
        origin=Origin(xyz=(0.390, 0.0, 0.390)),
        material=bridge_blue,
        name="cantilever_box",
    )
    for i, y in enumerate((-0.058, 0.058)):
        bridge.visual(
            Box((0.440, 0.026, 0.148)),
            origin=Origin(xyz=(0.355, y, 0.324)),
            material=bridge_blue,
            name=f"side_web_{i}",
        )
        bridge.visual(
            Cylinder(radius=0.012, length=0.458),
            origin=Origin(
                xyz=(0.418, y, 0.355),
                rpy=(0.0, math.atan2(0.430, 0.155), 0.0),
            ),
            material=machined,
            name=f"diagonal_tie_{i}",
        )
        bridge.visual(
            Box((0.500, 0.016, 0.030)),
            origin=Origin(xyz=(0.405, y, 0.445)),
            material=machined,
            name=f"top_rail_{i}",
        )
    for i, y in enumerate((-0.060, 0.060)):
        bridge.visual(
            Box((0.165, 0.038, 0.075)),
            origin=Origin(xyz=(0.640, y, 0.4675)),
            material=bridge_blue,
            name=f"support_cheek_{i}",
        )
    bridge.visual(
        _annular_mesh(0.108, 0.042, 0.090, "upper_bearing_ring"),
        origin=Origin(xyz=(0.640, 0.0, 0.475)),
        material=gunmetal,
        name="upper_bearing_ring",
    )
    for i, y in enumerate((-0.055, 0.055)):
        bridge.visual(
            Box((0.120, 0.030, 0.012)),
            origin=Origin(xyz=(0.640, y, 0.526)),
            material=bronze,
            name=f"upper_thrust_pad_{i}",
        )
    for i, y, retainer_name in (
        (0, -0.108, "upper_retainer_0"),
        (1, 0.108, "upper_retainer_1"),
    ):
        bridge.visual(
            Box((0.040, 0.034, 0.034)),
            origin=Origin(xyz=(0.640, y, 0.513)),
            material=gunmetal,
            name=f"retainer_post_{i}",
        )
        bridge.visual(
            Box((0.210, 0.022, 0.018)),
            origin=Origin(xyz=(0.640, y, 0.531)),
            material=gunmetal,
            name=retainer_name,
        )
        for j, x in enumerate((0.580, 0.700)):
            bridge.visual(
                Cylinder(radius=0.009, length=0.009),
                origin=Origin(xyz=(x, y, 0.5365)),
                material=black,
                name=f"upper_retainer_screw_{i}_{j}",
            )
    bridge.visual(
        Box((0.085, 0.210, 0.020)),
        origin=Origin(xyz=(0.640, 0.0, 0.424)),
        material=machined,
        name="bearing_foot_plate",
    )
    for i, y in enumerate((-0.072, 0.072)):
        bridge.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.640, y, 0.437)),
            material=black,
            name=f"foot_screw_{i}",
        )
    bridge.visual(
        Box((0.040, 0.120, 0.016)),
        origin=Origin(xyz=(0.112, 0.0, 0.298)),
        material=warning,
        name="index_mark",
    )

    upper_flange = model.part("upper_flange")
    upper_flange.visual(
        Cylinder(radius=0.030, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=machined,
        name="upper_journal",
    )
    upper_flange.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=machined,
        name="flange_neck",
    )
    upper_flange.visual(
        Cylinder(radius=0.098, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=machined,
        name="flange_plate",
    )
    upper_flange.visual(
        Cylinder(radius=0.057, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=gunmetal,
        name="output_cap",
    )
    upper_flange.visual(
        Box((0.022, 0.106, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=warning,
        name="flange_index_bar",
    )
    for i in range(6):
        a = 2.0 * math.pi * i / 6.0 + math.pi / 6.0
        upper_flange.visual(
            Cylinder(radius=0.0085, length=0.007),
            origin=Origin(xyz=(0.071 * math.cos(a), 0.071 * math.sin(a), 0.0515)),
            material=black,
            name=f"flange_screw_{i}",
        )

    lower_spin = model.articulation(
        "base_to_lower_drum",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_drum,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=240.0, velocity=1.0, lower=-2.35, upper=2.35),
    )
    lower_spin.meta["qc_samples"] = [-2.35, -1.1, 0.0, 1.1, 2.35]

    model.articulation(
        "lower_drum_to_bridge",
        ArticulationType.FIXED,
        parent=lower_drum,
        child=bridge,
        origin=Origin(),
    )

    upper_spin = model.articulation(
        "bridge_to_upper_flange",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=upper_flange,
        origin=Origin(xyz=(0.640, 0.0, 0.540)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=42.0, velocity=2.4, lower=-math.pi, upper=math.pi),
    )
    upper_spin.meta["qc_samples"] = [-math.pi, -1.2, 0.0, 1.2, math.pi]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_drum = object_model.get_part("lower_drum")
    bridge = object_model.get_part("bridge")
    upper_flange = object_model.get_part("upper_flange")
    lower_spin = object_model.get_articulation("base_to_lower_drum")
    upper_spin = object_model.get_articulation("bridge_to_upper_flange")

    ctx.allow_overlap(
        base,
        lower_drum,
        elem_a="base_bearing_ring",
        elem_b="lower_journal",
        reason=(
            "The lower journal is intentionally captured inside the hollow base bearing; "
            "the mesh-backed annular housing is a conservative proxy for the visible sleeve."
        ),
    )
    ctx.allow_overlap(
        bridge,
        lower_drum,
        elem_a="bridge_saddle",
        elem_b="top_boss",
        reason=(
            "The drum centering boss seats in a relieved underside pocket of the bridge saddle "
            "so the cantilever has a positive mechanical register."
        ),
    )
    ctx.allow_overlap(
        bridge,
        upper_flange,
        elem_a="upper_bearing_ring",
        elem_b="upper_journal",
        reason=(
            "The upper journal is intentionally retained through the hollow offset bearing; "
            "the annular housing mesh is a conservative visible sleeve proxy."
        ),
    )

    ctx.check(
        "two parallel rotary axes are offset",
        tuple(lower_spin.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_spin.axis) == (0.0, 0.0, 1.0)
        and upper_spin.origin.xyz[0] > 0.55,
        details=f"lower_axis={lower_spin.axis}, upper_axis={upper_spin.axis}, upper_origin={upper_spin.origin.xyz}",
    )

    ctx.expect_within(
        lower_drum,
        base,
        axes="xy",
        inner_elem="lower_journal",
        outer_elem="base_bearing_ring",
        margin=0.003,
        name="lower journal is captured inside the base bearing footprint",
    )
    ctx.expect_overlap(
        lower_drum,
        base,
        axes="z",
        elem_a="lower_journal",
        elem_b="base_bearing_ring",
        min_overlap=0.055,
        name="lower journal remains inserted in the base bearing",
    )
    ctx.expect_within(
        lower_drum,
        bridge,
        axes="xy",
        inner_elem="top_boss",
        outer_elem="bridge_saddle",
        margin=0.004,
        name="drum centering boss sits inside the bridge saddle pocket",
    )
    ctx.expect_overlap(
        lower_drum,
        bridge,
        axes="z",
        elem_a="top_boss",
        elem_b="bridge_saddle",
        min_overlap=0.020,
        name="drum centering boss is retained by the saddle pocket",
    )
    ctx.expect_contact(
        bridge,
        lower_drum,
        elem_a="bridge_saddle",
        elem_b="top_flange",
        contact_tol=0.001,
        name="cantilever bridge is seated on the lower drum flange",
    )
    ctx.expect_origin_distance(
        upper_flange,
        lower_drum,
        axes="xy",
        min_dist=0.60,
        max_dist=0.68,
        name="upper rotary stage is deliberately side-displaced",
    )
    ctx.expect_within(
        upper_flange,
        bridge,
        axes="xy",
        inner_elem="upper_journal",
        outer_elem="upper_bearing_ring",
        margin=0.003,
        name="upper journal is centered in the offset bearing footprint",
    )
    ctx.expect_overlap(
        upper_flange,
        bridge,
        axes="z",
        elem_a="upper_journal",
        elem_b="upper_bearing_ring",
        min_overlap=0.080,
        name="upper journal remains inserted in the offset bearing",
    )
    ctx.expect_gap(
        upper_flange,
        bridge,
        axis="z",
        min_gap=0.004,
        positive_elem="flange_plate",
        negative_elem="upper_retainer_0",
        name="upper flange clears the split retainer at rest",
    )

    with ctx.pose({lower_spin: 2.20, upper_spin: -2.60}):
        ctx.expect_gap(
            bridge,
            base,
            axis="z",
            min_gap=0.180,
            name="rotated bridge sweeps above the base cover",
        )
        ctx.expect_gap(
            upper_flange,
            bridge,
            axis="z",
            min_gap=0.004,
            positive_elem="flange_plate",
            negative_elem="upper_retainer_1",
            name="rotated upper flange still clears its retainer",
        )
        ctx.expect_origin_distance(
            upper_flange,
            lower_drum,
            axes="xy",
            min_dist=0.60,
            max_dist=0.68,
            name="offset rotary center is retained during lower rotation",
        )

    return ctx.report()


object_model = build_object_model()
