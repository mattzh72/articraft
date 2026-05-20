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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _variable_radius_tube(points, radii, *, segments=32, cap_start=True, cap_end=False) -> MeshGeometry:
    """A single connected swept mesh for the faucet spout, in local meters."""

    geom = MeshGeometry()

    def _sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def _cross(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    def _norm(v):
        length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if length < 1e-9:
            return (0.0, 0.0, 1.0)
        return (v[0] / length, v[1] / length, v[2] / length)

    ring_indices = []
    x_normal = (1.0, 0.0, 0.0)
    for index, (point, radius) in enumerate(zip(points, radii)):
        if index == 0:
            tangent = _sub(points[1], points[0])
        elif index == len(points) - 1:
            tangent = _sub(points[-1], points[-2])
        else:
            tangent = _sub(points[index + 1], points[index - 1])
        tangent = _norm(tangent)
        y_normal = _norm(_cross(tangent, x_normal))
        ring = []
        for seg in range(segments):
            angle = 2.0 * math.pi * seg / segments
            ca = math.cos(angle)
            sa = math.sin(angle)
            ring.append(
                geom.add_vertex(
                    point[0] + radius * (ca * x_normal[0] + sa * y_normal[0]),
                    point[1] + radius * (ca * x_normal[1] + sa * y_normal[1]),
                    point[2] + radius * (ca * x_normal[2] + sa * y_normal[2]),
                )
            )
        ring_indices.append(ring)

    for i in range(len(ring_indices) - 1):
        a = ring_indices[i]
        b = ring_indices[i + 1]
        for seg in range(segments):
            n = (seg + 1) % segments
            geom.add_face(a[seg], b[seg], b[n])
            geom.add_face(a[seg], b[n], a[n])

    if cap_start:
        center = geom.add_vertex(*points[0])
        first = ring_indices[0]
        for seg in range(segments):
            geom.add_face(center, first[(seg + 1) % segments], first[seg])
    if cap_end:
        center = geom.add_vertex(*points[-1])
        last = ring_indices[-1]
        for seg in range(segments):
            geom.add_face(center, last[seg], last[(seg + 1) % segments])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widespread_bathroom_faucet")

    chrome = model.material("polished_chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_seam = model.material("black_shadow", rgba=(0.015, 0.016, 0.018, 1.0))
    ceramic = model.material("white_ceramic_deck", rgba=(0.93, 0.91, 0.86, 1.0))
    brass = model.material("brushed_brass_underbody", rgba=(0.70, 0.49, 0.25, 1.0))
    hot_red = model.material("hot_red", rgba=(0.80, 0.05, 0.035, 1.0))
    cold_blue = model.material("cold_blue", rgba=(0.05, 0.18, 0.78, 1.0))

    spout_body_mesh = mesh_from_geometry(
        _variable_radius_tube(
            [
                (0.0, 0.000, 0.000),
                (0.0, 0.000, 0.040),
                (0.0, -0.006, 0.060),
                (0.0, -0.020, 0.086),
                (0.0, -0.054, 0.132),
                (0.0, -0.096, 0.154),
                (0.0, -0.150, 0.150),
                (0.0, -0.190, 0.120),
                (0.0, -0.205, 0.092),
                (0.0, -0.205, 0.060),
                (0.0, -0.205, 0.052),
            ],
            [0.033, 0.033, 0.028, 0.015, 0.012, 0.012, 0.012, 0.013, 0.016, 0.018, 0.020],
            segments=32,
            cap_start=True,
            cap_end=False,
        ),
        "continuous_spout_body",
    )
    handle_lever = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.000, 0.034),
                (0.0, 0.025, 0.042),
                (0.0, 0.070, 0.038),
                (0.0, 0.094, 0.033),
            ],
            radius=0.007,
            samples_per_segment=16,
            radial_segments=20,
            cap_ends=True,
        ),
        "arched_lever_handle",
    )

    fixed_body = model.part("fixed_body")
    fixed_body.visual(
        Box((0.52, 0.28, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=ceramic,
        name="mounting_deck",
    )
    fixed_body.visual(
        Box((0.40, 0.045, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=brass,
        name="underdeck_manifold",
    )
    for x, stem_name, esc_name, collar_name in (
        (-0.16, "hot_stem", "hot_escutcheon", "hot_collar"),
        (0.16, "cold_stem", "cold_escutcheon", "cold_collar"),
    ):
        fixed_body.visual(
            Cylinder(radius=0.043, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.006)),
            material=chrome,
            name=esc_name,
        )
        fixed_body.visual(
            Cylinder(radius=0.030, length=0.044),
            origin=Origin(xyz=(x, 0.0, 0.030)),
            material=chrome,
            name=collar_name,
        )
        fixed_body.visual(
            Cylinder(radius=0.010, length=0.041),
            origin=Origin(xyz=(x, 0.0, 0.0645)),
            material=chrome,
            name=stem_name,
        )
        fixed_body.visual(
            Cylinder(radius=0.012, length=0.052),
            origin=Origin(xyz=(x, 0.0, -0.032)),
            material=brass,
            name=f"{stem_name}_tail",
        )

    fixed_body.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=chrome,
        name="center_escutcheon",
    )
    fixed_body.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=chrome,
        name="center_post",
    )
    fixed_body.visual(
        Cylinder(radius=0.034, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.0712)),
        material=dark_seam,
        name="spout_bearing_seam",
    )
    fixed_body.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=brass,
        name="spout_shank",
    )

    spout = model.part("spout")
    spout.visual(
        spout_body_mesh,
        material=chrome,
        name="spout_body",
    )
    spout.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_seam,
        name="swivel_sleeve",
    )
    spout.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, -0.205, 0.052)),
        material=dark_seam,
        name="nozzle_body",
    )

    for name, marker_material in (("hot_handle", hot_red), ("cold_handle", cold_blue)):
        handle = model.part(name)
        handle.visual(
            Cylinder(radius=0.024, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
            material=chrome,
            name="handle_hub",
        )
        handle.visual(
            Cylinder(radius=0.031, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=chrome,
            name="hub_skirt",
        )
        handle.visual(
            handle_lever,
            material=chrome,
            name="lever_arm",
        )
        handle.visual(
            Sphere(radius=0.014),
            origin=Origin(xyz=(0.0, 0.096, 0.033)),
            material=chrome,
            name="lever_tip",
        )
        handle.visual(
            Cylinder(radius=0.0065, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.0335)),
            material=marker_material,
            name="temperature_dot",
        )

    model.articulation(
        "spout_bearing",
        ArticulationType.CONTINUOUS,
        parent=fixed_body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=4.5),
    )
    model.articulation(
        "hot_valve",
        ArticulationType.REVOLUTE,
        parent=fixed_body,
        child="hot_handle",
        origin=Origin(xyz=(-0.16, 0.0, 0.085), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-math.pi / 2.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "cold_valve",
        ArticulationType.REVOLUTE,
        parent=fixed_body,
        child="cold_handle",
        origin=Origin(xyz=(0.16, 0.0, 0.085), rpy=(0.0, 0.0, -math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-math.pi / 2.0, upper=math.pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_body = object_model.get_part("fixed_body")
    spout = object_model.get_part("spout")
    hot_handle = object_model.get_part("hot_handle")
    cold_handle = object_model.get_part("cold_handle")
    spout_bearing = object_model.get_articulation("spout_bearing")
    hot_valve = object_model.get_articulation("hot_valve")
    cold_valve = object_model.get_articulation("cold_valve")

    ctx.check(
        "spout uses a continuous vertical bearing",
        spout_bearing.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spout_bearing.axis) == (0.0, 0.0, 1.0),
        details=f"type={spout_bearing.articulation_type}, axis={spout_bearing.axis}",
    )
    ctx.check(
        "hot and cold handles use limited valve-stem revolutes",
        hot_valve.articulation_type == ArticulationType.REVOLUTE
        and cold_valve.articulation_type == ArticulationType.REVOLUTE
        and tuple(hot_valve.axis) == (0.0, 0.0, 1.0)
        and tuple(cold_valve.axis) == (0.0, 0.0, 1.0),
        details=f"hot=({hot_valve.articulation_type}, {hot_valve.axis}), cold=({cold_valve.articulation_type}, {cold_valve.axis})",
    )

    ctx.expect_gap(
        spout,
        fixed_body,
        axis="z",
        positive_elem="swivel_sleeve",
        negative_elem="center_post",
        max_gap=0.001,
        max_penetration=0.0,
        name="spout sleeve seats on center mounting post",
    )
    ctx.expect_overlap(
        spout,
        fixed_body,
        axes="xy",
        elem_a="swivel_sleeve",
        elem_b="center_post",
        min_overlap=0.040,
        name="spout bearing is centered on the post axis",
    )
    for handle, stem_name, check_name in (
        (hot_handle, "hot_stem", "hot handle sits on hot valve stem"),
        (cold_handle, "cold_stem", "cold handle sits on cold valve stem"),
    ):
        ctx.expect_gap(
            handle,
            fixed_body,
            axis="z",
            positive_elem="hub_skirt",
            negative_elem=stem_name,
            max_gap=0.001,
            max_penetration=0.0,
            name=check_name,
        )
        ctx.expect_overlap(
            handle,
            fixed_body,
            axes="xy",
            elem_a="handle_hub",
            elem_b=stem_name,
            min_overlap=0.016,
            name=f"{check_name} axis alignment",
        )

    ctx.expect_gap(
        fixed_body,
        spout,
        axis="y",
        positive_elem="center_post",
        negative_elem="nozzle_body",
        min_gap=0.12,
        name="rest spout projects forward over the basin",
    )
    with ctx.pose({spout_bearing: math.pi / 2.0}):
        ctx.expect_gap(
            spout,
            fixed_body,
            axis="x",
            positive_elem="nozzle_body",
            negative_elem="center_post",
            min_gap=0.12,
            name="rotated spout swings around the vertical post",
        )

    return ctx.report()


object_model = build_object_model()
