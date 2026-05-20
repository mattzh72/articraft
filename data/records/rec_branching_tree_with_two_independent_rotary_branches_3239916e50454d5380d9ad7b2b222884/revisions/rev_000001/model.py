from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_tube(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 48,
) -> MeshGeometry:
    """Closed hollow tube centered on local Z, used for visible bearing sleeves."""
    geom = MeshGeometry()
    half = length * 0.5
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, -half))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, half))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, -half))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, half))

    for i in range(segments):
        j = (i + 1) % segments

        # Outside wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])

        # Inside wall.
        geom.add_face(inner_bottom[i], inner_top[i], inner_top[j])
        geom.add_face(inner_bottom[i], inner_top[j], inner_bottom[j])

        # End annuli.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        geom.add_face(outer_bottom[i], inner_bottom[j], outer_bottom[j])
        geom.add_face(outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_arm_positioning_tree")

    painted_steel = model.material("painted_steel", rgba=(0.24, 0.29, 0.32, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.66, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.63, 0.47, 0.20, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    safety_blue = model.material("safety_blue", rgba=(0.08, 0.23, 0.70, 1.0))

    forward_bearing_mesh = mesh_from_geometry(
        _annular_tube(0.044, 0.023, 0.150), "forward_bearing"
    )
    side_bearing_mesh = mesh_from_geometry(
        _annular_tube(0.048, 0.024, 0.165), "side_bearing"
    )

    central_mast = model.part("central_mast")
    central_mast.visual(
        Box((0.42, 0.32, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="floor_plate",
    )
    central_mast.visual(
        Cylinder(radius=0.038, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=painted_steel,
        name="mast_tube",
    )
    central_mast.visual(
        Cylinder(radius=0.064, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_steel,
        name="base_collar",
    )
    for x in (-0.047, 0.047):
        central_mast.visual(
            Box((0.026, 0.120, 0.105)),
            origin=Origin(xyz=(x, 0.070, 0.680)),
            material=painted_steel,
            name=f"forward_yoke_{x}",
        )
    central_mast.visual(
        forward_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.105, 0.680), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bearing_bronze,
        name="forward_bearing",
    )
    for y in (-0.050, 0.050):
        central_mast.visual(
            Box((0.125, 0.026, 0.110)),
            origin=Origin(xyz=(0.072, y, 0.450)),
            material=painted_steel,
            name=f"side_yoke_{y}",
        )
    central_mast.visual(
        side_bearing_mesh,
        origin=Origin(xyz=(0.105, 0.0, 0.450), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_bronze,
        name="side_bearing",
    )
    for x in (-0.155, 0.155):
        for y in (-0.105, 0.105):
            central_mast.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(xyz=(x, y, 0.040)),
                material=brushed_steel,
                name=f"anchor_{x}_{y}",
            )

    forward_branch = model.part("forward_branch")
    forward_branch.visual(
        Cylinder(radius=0.016, length=0.500),
        origin=Origin(xyz=(0.0, 0.200, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="forward_shaft",
    )
    forward_branch.visual(
        Cylinder(radius=0.032, length=0.040),
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_hub",
    )
    forward_branch.visual(
        Box((0.036, 0.210, 0.036)),
        origin=Origin(xyz=(0.0, 0.235, 0.0)),
        material=safety_blue,
        name="short_boom",
    )
    forward_branch.visual(
        Box((0.120, 0.032, 0.090)),
        origin=Origin(xyz=(0.0, 0.390, 0.0)),
        material=dark_steel,
        name="tool_pad_body",
    )
    forward_branch.visual(
        Box((0.105, 0.010, 0.075)),
        origin=Origin(xyz=(0.0, 0.411, 0.0)),
        material=pad_rubber,
        name="tool_pad_face",
    )
    for x in (-0.038, 0.038):
        for z in (-0.028, 0.028):
            forward_branch.visual(
                Cylinder(radius=0.006, length=0.014),
                origin=Origin(xyz=(x, 0.418, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=brushed_steel,
                name=f"pad_screw_{x}_{z}",
            )

    side_branch = model.part("side_branch")
    side_branch.visual(
        Cylinder(radius=0.017, length=0.650),
        origin=Origin(xyz=(0.285, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="side_shaft",
    )
    side_branch.visual(
        Cylinder(radius=0.035, length=0.048),
        origin=Origin(xyz=(0.1065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="side_hub",
    )
    side_branch.visual(
        Box((0.360, 0.038, 0.038)),
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        material=safety_blue,
        name="long_boom",
    )
    side_branch.visual(
        Box((0.034, 0.135, 0.095)),
        origin=Origin(xyz=(0.575, 0.0, 0.0)),
        material=dark_steel,
        name="tool_pad_body",
    )
    side_branch.visual(
        Box((0.010, 0.120, 0.080)),
        origin=Origin(xyz=(0.597, 0.0, 0.0)),
        material=pad_rubber,
        name="tool_pad_face",
    )
    for y in (-0.044, 0.044):
        for z in (-0.030, 0.030):
            side_branch.visual(
                Cylinder(radius=0.006, length=0.014),
                origin=Origin(xyz=(0.604, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brushed_steel,
                name=f"pad_screw_{y}_{z}",
            )

    model.articulation(
        "mast_to_forward_branch",
        ArticulationType.REVOLUTE,
        parent=central_mast,
        child=forward_branch,
        origin=Origin(xyz=(0.0, 0.105, 0.680)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "mast_to_side_branch",
        ArticulationType.REVOLUTE,
        parent=central_mast,
        child=side_branch,
        origin=Origin(xyz=(0.105, 0.0, 0.450)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=-0.95, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    central_mast = object_model.get_part("central_mast")
    forward_branch = object_model.get_part("forward_branch")
    side_branch = object_model.get_part("side_branch")
    forward_joint = object_model.get_articulation("mast_to_forward_branch")
    side_joint = object_model.get_articulation("mast_to_side_branch")

    ctx.check(
        "forward branch uses front-back revolute axis",
        tuple(round(v, 6) for v in forward_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={forward_joint.axis}",
    )
    ctx.check(
        "side branch uses left-right revolute axis",
        tuple(round(v, 6) for v in side_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={side_joint.axis}",
    )
    ctx.expect_origin_gap(
        forward_branch,
        side_branch,
        axis="z",
        min_gap=0.18,
        name="forward module is mounted higher than side module",
    )
    ctx.expect_overlap(
        forward_branch,
        central_mast,
        axes="y",
        elem_a="forward_shaft",
        elem_b="forward_bearing",
        min_overlap=0.12,
        name="forward shaft is retained in its bearing",
    )
    ctx.expect_within(
        forward_branch,
        central_mast,
        axes="xz",
        inner_elem="forward_shaft",
        outer_elem="forward_bearing",
        margin=0.002,
        name="forward shaft is centered inside its sleeve",
    )
    ctx.expect_overlap(
        side_branch,
        central_mast,
        axes="x",
        elem_a="side_shaft",
        elem_b="side_bearing",
        min_overlap=0.12,
        name="side shaft is retained in its bearing",
    )
    ctx.expect_within(
        side_branch,
        central_mast,
        axes="yz",
        inner_elem="side_shaft",
        outer_elem="side_bearing",
        margin=0.002,
        name="side shaft is centered inside its sleeve",
    )

    with ctx.pose({forward_joint: 0.75, side_joint: -0.65}):
        ctx.expect_within(
            forward_branch,
            central_mast,
            axes="xz",
            inner_elem="forward_shaft",
            outer_elem="forward_bearing",
            margin=0.002,
            name="forward shaft remains coaxial while rolled",
        )
        ctx.expect_within(
            side_branch,
            central_mast,
            axes="yz",
            inner_elem="side_shaft",
            outer_elem="side_bearing",
            margin=0.002,
            name="side shaft remains coaxial while rolled",
        )

    return ctx.report()


object_model = build_object_model()
