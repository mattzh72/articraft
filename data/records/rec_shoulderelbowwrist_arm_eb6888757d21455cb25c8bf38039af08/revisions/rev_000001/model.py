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
    mesh_from_cadquery,
)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _cyl(
    part,
    name: str,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    axis: str = "z",
    material=None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _box(
    part,
    name: str,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _ring_body(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    bolt_count: int = 0,
    bolt_circle: float = 0.0,
    bolt_radius: float = 0.0,
):
    """A centered machined annulus with optional through bolt holes."""
    body = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(thickness)
    body = body.translate((0.0, 0.0, -0.5 * thickness))
    if bolt_count and bolt_circle > 0.0 and bolt_radius > 0.0:
        points = [
            (
                bolt_circle * math.cos(2.0 * math.pi * i / bolt_count),
                bolt_circle * math.sin(2.0 * math.pi * i / bolt_count),
            )
            for i in range(bolt_count)
        ]
        cutters = (
            cq.Workplane("XY")
            .pushPoints(points)
            .circle(bolt_radius)
            .extrude(thickness * 3.0)
            .translate((0.0, 0.0, -1.5 * thickness))
        )
        body = body.cut(cutters)
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_mechanical_arm_study")

    aluminum = model.material("bead_blasted_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("blackened_steel", rgba=(0.07, 0.075, 0.075, 1.0))
    gunmetal = model.material("machined_gunmetal", rgba=(0.23, 0.25, 0.26, 1.0))
    bronze = model.material("oil_bronze_bearing", rgba=(0.68, 0.48, 0.24, 1.0))
    cover = model.material("dark_cable_cover", rgba=(0.03, 0.035, 0.04, 1.0))
    fastener = model.material("black_socket_heads", rgba=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    _box(base, "base_plate", size=(0.72, 0.46, 0.035), xyz=(0.0, 0.0, 0.0175), material=gunmetal)
    _box(base, "mount_rail_0", size=(0.62, 0.035, 0.020), xyz=(0.0, -0.18, 0.045), material=dark_steel)
    _box(base, "mount_rail_1", size=(0.62, 0.035, 0.020), xyz=(0.0, 0.18, 0.045), material=dark_steel)
    _cyl(base, "lower_bearing_tube", radius=0.155, length=0.175, xyz=(0.0, 0.0, 0.1225), material=aluminum)
    _box(base, "pedestal_rib_x", size=(0.034, 0.31, 0.175), xyz=(0.0, 0.0, 0.1225), material=aluminum)
    _box(base, "pedestal_rib_y", size=(0.31, 0.034, 0.175), xyz=(0.0, 0.0, 0.1225), material=aluminum)
    base.visual(
        mesh_from_cadquery(
            _ring_body(
                outer_radius=0.175,
                inner_radius=0.073,
                thickness=0.030,
                bolt_count=12,
                bolt_circle=0.136,
                bolt_radius=0.006,
            ),
            "shoulder_bearing_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_steel,
        name="shoulder_bearing_ring",
    )
    base.visual(
        mesh_from_cadquery(
            _ring_body(
                outer_radius=0.130,
                inner_radius=0.075,
                thickness=0.012,
                bolt_count=8,
                bolt_circle=0.104,
                bolt_radius=0.0045,
            ),
            "shoulder_bronze_race",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.246)),
        material=bronze,
        name="shoulder_bronze_race",
    )
    for i in range(8):
        angle = 2.0 * math.pi * i / 8.0 + math.pi / 8.0
        x = 0.29 * math.cos(angle)
        y = 0.17 * math.sin(angle)
        _cyl(base, f"base_bolt_{i}", radius=0.010, length=0.008, xyz=(x, y, 0.039), material=fastener)
    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        _cyl(
            base,
            f"shoulder_ring_bolt_{i}",
            radius=0.0065,
            length=0.006,
            xyz=(0.136 * math.cos(angle), 0.136 * math.sin(angle), 0.244),
            material=fastener,
        )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.142, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="shoulder_hub_plate",
    )
    _cyl(upper_arm, "shoulder_output_drum", radius=0.090, length=0.095, xyz=(0.0, 0.0, 0.090), material=aluminum)
    upper_arm.visual(
        mesh_from_cadquery(
            _ring_body(
                outer_radius=0.128,
                inner_radius=0.070,
                thickness=0.014,
                bolt_count=10,
                bolt_circle=0.104,
                bolt_radius=0.004,
            ),
            "shoulder_joint_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=bronze,
        name="shoulder_joint_ring",
    )
    _box(upper_arm, "shoulder_upright_0", size=(0.155, 0.030, 0.145), xyz=(0.095, -0.086, 0.122), material=aluminum)
    _box(upper_arm, "shoulder_upright_1", size=(0.155, 0.030, 0.145), xyz=(0.095, 0.086, 0.122), material=aluminum)
    _box(upper_arm, "shoulder_bridge", size=(0.190, 0.220, 0.032), xyz=(0.110, 0.0, 0.075), material=aluminum)
    _box(upper_arm, "upper_rail_0", size=(0.440, 0.036, 0.052), xyz=(0.350, -0.058, 0.130), material=aluminum)
    _box(upper_arm, "upper_rail_1", size=(0.440, 0.036, 0.052), xyz=(0.350, 0.058, 0.130), material=aluminum)
    _box(upper_arm, "upper_crossmember_0", size=(0.038, 0.158, 0.036), xyz=(0.235, 0.0, 0.130), material=aluminum)
    _box(upper_arm, "upper_crossmember_1", size=(0.038, 0.158, 0.036), xyz=(0.505, 0.0, 0.130), material=aluminum)
    _box(upper_arm, "upper_cable_cover", size=(0.360, 0.090, 0.018), xyz=(0.370, 0.0, 0.164), material=cover)
    _box(upper_arm, "upper_access_cover_0", size=(0.180, 0.007, 0.036), xyz=(0.375, -0.079, 0.130), material=gunmetal)
    _box(upper_arm, "upper_access_cover_1", size=(0.180, 0.007, 0.036), xyz=(0.375, 0.079, 0.130), material=gunmetal)
    _box(upper_arm, "elbow_yoke_bridge", size=(0.180, 0.270, 0.044), xyz=(0.610, 0.0, 0.035), material=aluminum)
    _box(upper_arm, "elbow_cheek_0", size=(0.125, 0.040, 0.205), xyz=(0.650, -0.116, 0.122), material=aluminum)
    _box(upper_arm, "elbow_cheek_1", size=(0.125, 0.040, 0.205), xyz=(0.650, 0.116, 0.122), material=aluminum)
    _box(upper_arm, "elbow_side_link_0", size=(0.045, 0.022, 0.052), xyz=(0.570, -0.086, 0.130), material=aluminum)
    _box(upper_arm, "elbow_side_link_1", size=(0.045, 0.022, 0.052), xyz=(0.570, 0.086, 0.130), material=aluminum)
    _cyl(upper_arm, "elbow_outer_bearing_0", radius=0.059, length=0.014, xyz=(0.650, -0.143, 0.122), axis="y", material=dark_steel)
    _cyl(upper_arm, "elbow_outer_bearing_1", radius=0.059, length=0.014, xyz=(0.650, 0.143, 0.122), axis="y", material=dark_steel)
    upper_arm.visual(
        mesh_from_cadquery(
            _ring_body(outer_radius=0.046, inner_radius=0.043, thickness=0.010),
            "elbow_inner_bushing_0",
        ),
        origin=Origin(xyz=(0.650, -0.091, 0.122), rpy=_axis_rpy("y")),
        material=bronze,
        name="elbow_inner_bushing_0",
    )
    upper_arm.visual(
        mesh_from_cadquery(
            _ring_body(outer_radius=0.046, inner_radius=0.043, thickness=0.010),
            "elbow_inner_bushing_1",
        ),
        origin=Origin(xyz=(0.650, 0.091, 0.122), rpy=_axis_rpy("y")),
        material=bronze,
        name="elbow_inner_bushing_1",
    )
    for side, y in enumerate((-0.153, 0.153)):
        for i in range(6):
            angle = 2.0 * math.pi * i / 6.0
            _cyl(
                upper_arm,
                f"elbow_cap_bolt_{side}_{i}",
                radius=0.0045,
                length=0.006,
                xyz=(0.650 + 0.043 * math.cos(angle), y, 0.122 + 0.043 * math.sin(angle)),
                axis="y",
                material=fastener,
            )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.057, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_axis_rpy("y")),
        material=dark_steel,
        name="elbow_barrel",
    )
    forearm.visual(
        Cylinder(radius=0.043, length=0.014),
        origin=Origin(xyz=(0.0, -0.089, 0.0), rpy=_axis_rpy("y")),
        material=bronze,
        name="elbow_trunnion_0",
    )
    forearm.visual(
        Cylinder(radius=0.043, length=0.014),
        origin=Origin(xyz=(0.0, 0.089, 0.0), rpy=_axis_rpy("y")),
        material=bronze,
        name="elbow_trunnion_1",
    )
    _box(forearm, "forearm_rail_0", size=(0.460, 0.034, 0.046), xyz=(0.265, -0.052, 0.000), material=aluminum)
    _box(forearm, "forearm_rail_1", size=(0.460, 0.034, 0.046), xyz=(0.265, 0.052, 0.000), material=aluminum)
    _box(forearm, "forearm_spacer_0", size=(0.035, 0.145, 0.032), xyz=(0.145, 0.0, 0.000), material=aluminum)
    _box(forearm, "forearm_spacer_1", size=(0.035, 0.145, 0.032), xyz=(0.375, 0.0, 0.000), material=aluminum)
    _box(forearm, "forearm_cable_cover", size=(0.330, 0.080, 0.018), xyz=(0.270, 0.0, 0.030), material=cover)
    _box(forearm, "forearm_access_cover_0", size=(0.155, 0.006, 0.030), xyz=(0.300, -0.074, 0.000), material=gunmetal)
    _box(forearm, "forearm_access_cover_1", size=(0.155, 0.006, 0.030), xyz=(0.300, 0.074, 0.000), material=gunmetal)
    _box(forearm, "wrist_end_block", size=(0.040, 0.170, 0.140), xyz=(0.480, 0.0, 0.000), material=aluminum)
    _box(forearm, "wrist_side_lug_0", size=(0.042, 0.026, 0.100), xyz=(0.520, -0.085, 0.000), material=aluminum)
    _box(forearm, "wrist_side_lug_1", size=(0.042, 0.026, 0.100), xyz=(0.520, 0.085, 0.000), material=aluminum)
    forearm.visual(
        mesh_from_cadquery(
            _ring_body(
                outer_radius=0.102,
                inner_radius=0.034,
                thickness=0.034,
                bolt_count=8,
                bolt_circle=0.079,
                bolt_radius=0.0042,
            ),
            "wrist_bearing_ring",
        ),
        origin=Origin(xyz=(0.548, 0.0, 0.0), rpy=_axis_rpy("x")),
        material=dark_steel,
        name="wrist_bearing_ring",
    )
    forearm.visual(
        mesh_from_cadquery(
            _ring_body(
                outer_radius=0.060,
                inner_radius=0.034,
                thickness=0.040,
            ),
            "wrist_bronze_liner",
        ),
        origin=Origin(xyz=(0.548, 0.0, 0.0), rpy=_axis_rpy("x")),
        material=bronze,
        name="wrist_bronze_liner",
    )
    for i in range(8):
        angle = 2.0 * math.pi * i / 8.0
        _cyl(
            forearm,
            f"wrist_ring_bolt_{i}",
            radius=0.0048,
            length=0.007,
            xyz=(0.528, 0.079 * math.cos(angle), 0.079 * math.sin(angle)),
            axis="x",
            material=fastener,
        )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.034, length=0.105),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=_axis_rpy("x")),
        material=dark_steel,
        name="wrist_shaft",
    )
    _cyl(wrist, "wrist_hub_collar", radius=0.055, length=0.034, xyz=(0.060, 0.0, 0.0), axis="x", material=aluminum)
    wrist.visual(
        mesh_from_cadquery(
            _ring_body(
                outer_radius=0.118,
                inner_radius=0.028,
                thickness=0.026,
                bolt_count=10,
                bolt_circle=0.086,
                bolt_radius=0.0045,
            ),
            "wrist_flange",
        ),
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=_axis_rpy("x")),
        material=gunmetal,
        name="wrist_flange",
    )
    _cyl(wrist, "flange_pilot_ring", radius=0.047, length=0.018, xyz=(0.108, 0.0, 0.0), axis="x", material=bronze)
    _box(wrist, "flange_cable_cover", size=(0.030, 0.040, 0.020), xyz=(0.070, 0.0, 0.072), material=cover)
    for i in range(10):
        angle = 2.0 * math.pi * i / 10.0
        _cyl(
            wrist,
            f"flange_bolt_{i}",
            radius=0.0048,
            length=0.007,
            xyz=(0.104, 0.086 * math.cos(angle), 0.086 * math.sin(angle)),
            axis="x",
            material=fastener,
        )

    model.articulation(
        "shoulder_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2, lower=-2.6, upper=2.6),
    )
    model.articulation(
        "elbow_axis",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.650, 0.0, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=130.0, velocity=1.1, lower=-1.25, upper=1.45),
    )
    model.articulation(
        "wrist_axis",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.548, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")
    shoulder = object_model.get_articulation("shoulder_axis")
    elbow = object_model.get_articulation("elbow_axis")
    wrist_axis = object_model.get_articulation("wrist_axis")

    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a="elbow_trunnion_0",
        elem_b="elbow_inner_bushing_0",
        reason="The elbow trunnion is intentionally captured inside the bronze bushing proxy.",
    )
    ctx.allow_overlap(
        forearm,
        upper_arm,
        elem_a="elbow_trunnion_1",
        elem_b="elbow_inner_bushing_1",
        reason="The elbow trunnion is intentionally captured inside the bronze bushing proxy.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_bronze_liner",
        elem_b="wrist_shaft",
        reason="The wrist shaft is intentionally retained inside the bronze bearing liner proxy.",
    )
    ctx.allow_overlap(
        forearm,
        wrist,
        elem_a="wrist_bearing_ring",
        elem_b="wrist_shaft",
        reason="The wrist shaft passes through the steel bearing ring as a captured rotary axle.",
    )

    ctx.check(
        "three named revolute axes",
        {shoulder.name, elbow.name, wrist_axis.name} == {"shoulder_axis", "elbow_axis", "wrist_axis"}
        and shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and wrist_axis.articulation_type == ArticulationType.REVOLUTE,
        details="The study assembly should expose shoulder, elbow, and wrist revolute axes only.",
    )
    ctx.expect_gap(
        upper_arm,
        base,
        axis="z",
        positive_elem="shoulder_hub_plate",
        negative_elem="shoulder_bronze_race",
        min_gap=0.0,
        max_gap=0.002,
        name="shoulder hub sits on bronze race",
    )
    ctx.expect_overlap(
        upper_arm,
        base,
        axes="xy",
        elem_a="shoulder_hub_plate",
        elem_b="shoulder_bearing_ring",
        min_overlap=0.09,
        name="shoulder hub is centered over the bearing race",
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        positive_elem="elbow_inner_bushing_1",
        negative_elem="elbow_barrel",
        min_gap=0.0,
        max_gap=0.012,
        name="elbow positive trunnion clearance is tight",
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        positive_elem="elbow_barrel",
        negative_elem="elbow_inner_bushing_0",
        min_gap=0.0,
        max_gap=0.012,
        name="elbow negative trunnion clearance is tight",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="xz",
        inner_elem="elbow_trunnion_0",
        outer_elem="elbow_inner_bushing_0",
        margin=0.004,
        name="elbow trunnion 0 is concentric in bushing",
    )
    ctx.expect_within(
        forearm,
        upper_arm,
        axes="xz",
        inner_elem="elbow_trunnion_1",
        outer_elem="elbow_inner_bushing_1",
        margin=0.004,
        name="elbow trunnion 1 is concentric in bushing",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="y",
        elem_a="elbow_trunnion_0",
        elem_b="elbow_inner_bushing_0",
        min_overlap=0.008,
        name="elbow trunnion 0 remains inserted",
    )
    ctx.expect_overlap(
        forearm,
        upper_arm,
        axes="y",
        elem_a="elbow_trunnion_1",
        elem_b="elbow_inner_bushing_1",
        min_overlap=0.008,
        name="elbow trunnion 1 remains inserted",
    )
    ctx.expect_within(
        wrist,
        forearm,
        axes="yz",
        inner_elem="wrist_shaft",
        outer_elem="wrist_bearing_ring",
        margin=0.003,
        name="wrist shaft stays inside bearing envelope",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="x",
        elem_a="wrist_shaft",
        elem_b="wrist_bronze_liner",
        min_overlap=0.035,
        name="wrist shaft remains inserted in liner",
    )
    ctx.expect_overlap(
        wrist,
        forearm,
        axes="x",
        elem_a="wrist_shaft",
        elem_b="wrist_bearing_ring",
        min_overlap=0.025,
        name="wrist shaft passes through bearing ring",
    )

    rest_wrist = ctx.part_world_position(wrist)
    with ctx.pose({shoulder: 0.65, elbow: -0.55, wrist_axis: 0.9}):
        moved_wrist = ctx.part_world_position(wrist)
        ctx.expect_within(
            wrist,
            forearm,
            axes="yz",
            inner_elem="wrist_shaft",
            outer_elem="wrist_bearing_ring",
            margin=0.003,
            name="posed wrist shaft remains guided",
        )
    ctx.check(
        "elbow pose lifts wrist center",
        rest_wrist is not None and moved_wrist is not None and moved_wrist[2] > rest_wrist[2] + 0.12,
        details=f"rest={rest_wrist}, moved={moved_wrist}",
    )
    return ctx.report()


object_model = build_object_model()
