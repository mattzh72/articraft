from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _add_loop_faces(mesh: MeshGeometry, lower: list[int], upper: list[int]) -> None:
    """Connect two rectangular loops with quad side faces."""
    for i in range(4):
        j = (i + 1) % 4
        mesh.add_face(lower[i], lower[j], upper[j])
        mesh.add_face(lower[i], upper[j], upper[i])


def _rect_loop(mesh: MeshGeometry, width: float, depth: float, z: float) -> list[int]:
    hw = width / 2.0
    hd = depth / 2.0
    return [
        mesh.add_vertex(-hw, -hd, z),
        mesh.add_vertex(hw, -hd, z),
        mesh.add_vertex(hw, hd, z),
        mesh.add_vertex(-hw, hd, z),
    ]


def _stepped_housing_mesh() -> MeshGeometry:
    """One connected, capped stepped-pyramid shell and plinth mesh."""
    sections = [
        (0.620, 0.360, 0.000),  # broad plinth bottom
        (0.620, 0.360, 0.045),  # flat plinth top
        (0.460, 0.255, 0.045),  # lower tier footprint on the plinth
        (0.370, 0.215, 0.270),
        (0.320, 0.190, 0.270),  # first horizontal step-in ledge
        (0.220, 0.150, 0.490),
        (0.180, 0.130, 0.490),  # second horizontal step-in ledge
        (0.080, 0.080, 0.620),  # small apex cap
    ]
    mesh = MeshGeometry()
    loops = [_rect_loop(mesh, w, d, z) for w, d, z in sections]
    for lower, upper in zip(loops[:-1], loops[1:]):
        _add_loop_faces(mesh, lower, upper)

    bottom = loops[0]
    mesh.add_face(bottom[0], bottom[2], bottom[1])
    mesh.add_face(bottom[0], bottom[3], bottom[2])
    top = loops[-1]
    mesh.add_face(top[0], top[1], top[2])
    mesh.add_face(top[0], top[2], top[3])
    return mesh


def _sliding_weight_cadquery() -> cq.Workplane:
    """Hexagonal weight with a real vertical rod bore."""
    width = 0.096
    height = 0.124
    thickness = 0.052
    hole_radius = 0.0165
    pts = [
        (0.0, -height / 2.0),
        (width / 2.0, -height / 4.0),
        (width / 2.0, height / 4.0),
        (0.0, height / 2.0),
        (-width / 2.0, height / 4.0),
        (-width / 2.0, -height / 4.0),
    ]
    body = cq.Workplane("XZ").polyline(pts).close().extrude(thickness / 2.0, both=True)
    rod_bore = cq.Workplane("XY").circle(hole_radius).extrude(height * 1.8, both=True)
    return body.cut(rod_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stepped_pyramid_metronome")

    wood = model.material("warm_walnut", rgba=(0.34, 0.18, 0.08, 1.0))
    dark = model.material("matte_black_scale", rgba=(0.015, 0.013, 0.011, 1.0))
    brass = model.material("aged_brass", rgba=(0.86, 0.63, 0.25, 1.0))
    ivory = model.material("ivory_tick_marks", rgba=(0.93, 0.86, 0.66, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_stepped_housing_mesh(), "stepped_housing"),
        material=wood,
        name="stepped_shell",
    )
    housing.visual(
        Box((0.078, 0.008, 0.405)),
        origin=Origin(xyz=(0.0, -0.134, 0.330)),
        material=dark,
        name="scale_plate",
    )
    housing.visual(
        Box((0.092, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, -0.116, 0.150)),
        material=dark,
        name="lower_scale_mount",
    )
    housing.visual(
        Box((0.082, 0.088, 0.018)),
        origin=Origin(xyz=(0.0, -0.098, 0.515)),
        material=dark,
        name="upper_scale_mount",
    )
    for i, z in enumerate([0.185, 0.225, 0.265, 0.305, 0.345, 0.385, 0.425, 0.465]):
        width = 0.050 if i % 2 == 0 else 0.034
        housing.visual(
            Box((width, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, -0.140, z)),
            material=ivory,
            name=f"tick_{i}",
        )
    housing.visual(
        Cylinder(radius=0.0175, length=0.130),
        origin=Origin(xyz=(0.0, -0.100, 0.580), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_shaft",
    )
    housing.visual(
        Cylinder(radius=0.034, length=0.020),
        origin=Origin(xyz=(0.205, 0.0, 0.200), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="key_bushing",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0115, length=0.480),
        origin=Origin(xyz=(0.0, -0.012, -0.255)),
        material=brass,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.035, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="hub_disk",
    )
    pendulum.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, -0.012, -0.495)),
        material=brass,
        name="lower_stop",
    )

    weight = model.part("weight")
    weight.visual(
        mesh_from_cadquery(_sliding_weight_cadquery(), "hex_sliding_weight", tolerance=0.0008),
        material=brass,
        name="weight_body",
    )
    weight.visual(
        Box((0.005, 0.030, 0.050)),
        origin=Origin(xyz=(0.0140, 0.0, 0.0)),
        material=brass,
        name="clamp_pad_0",
    )
    weight.visual(
        Box((0.005, 0.030, 0.050)),
        origin=Origin(xyz=(-0.0140, 0.0, 0.0)),
        material=brass,
        name="clamp_pad_1",
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="shaft",
    )
    key.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.064, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="collar",
    )
    key.visual(
        Box((0.018, 0.014, 0.112)),
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=brass,
        name="center_web",
    )
    key.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.080, 0.0, 0.054), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="upper_lobe",
    )
    key.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.080, 0.0, -0.054), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_lobe",
    )

    model.articulation(
        "apex_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.165, 0.580)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, -0.012, -0.365)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=0.0, upper=0.245),
    )
    model.articulation(
        "key_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(xyz=(0.215, 0.0, 0.200)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    key = object_model.get_part("winding_key")
    apex = object_model.get_articulation("apex_pivot")
    slide = object_model.get_articulation("weight_slide")

    ctx.expect_contact(
        pendulum,
        housing,
        elem_a="hub_disk",
        elem_b="pivot_shaft",
        contact_tol=0.002,
        name="pendulum hub rides on the apex shaft",
    )
    ctx.expect_contact(
        key,
        housing,
        elem_a="shaft",
        elem_b="key_bushing",
        contact_tol=0.002,
        name="winding key shaft meets side bushing",
    )
    ctx.expect_within(
        pendulum,
        weight,
        axes="xy",
        inner_elem="rod",
        outer_elem="weight_body",
        margin=0.004,
        name="rod passes through the weight bore",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        elem_a="weight_body",
        elem_b="rod",
        min_overlap=0.10,
        name="weight surrounds a retained length of rod",
    )

    rest_weight = ctx.part_world_position(weight)
    with ctx.pose({slide: 0.245}):
        raised_weight = ctx.part_world_position(weight)
        ctx.expect_within(
            pendulum,
            weight,
            axes="xy",
            inner_elem="rod",
            outer_elem="weight_body",
            margin=0.004,
            name="raised weight stays on the rod",
        )
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            elem_a="weight_body",
            elem_b="rod",
            min_overlap=0.10,
            name="raised weight remains captured on rod",
        )
    ctx.check(
        "weight translates upward along rod",
        rest_weight is not None
        and raised_weight is not None
        and raised_weight[2] > rest_weight[2] + 0.20,
        details=f"rest={rest_weight}, raised={raised_weight}",
    )

    with ctx.pose({apex: 0.0}):
        rest_rod_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    with ctx.pose({apex: 0.30}):
        swung_rod_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    if rest_rod_aabb is not None and swung_rod_aabb is not None:
        rest_center_x = (rest_rod_aabb[0][0] + rest_rod_aabb[1][0]) / 2.0
        swung_center_x = (swung_rod_aabb[0][0] + swung_rod_aabb[1][0]) / 2.0
        swing_ok = abs(swung_center_x - rest_center_x) > 0.05
    else:
        swing_ok = False
    ctx.check(
        "pendulum swings across front plane",
        swing_ok,
        details=f"rest_aabb={rest_rod_aabb}, swung_aabb={swung_rod_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
