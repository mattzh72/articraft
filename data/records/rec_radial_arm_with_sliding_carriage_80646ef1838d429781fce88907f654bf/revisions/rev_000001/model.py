from __future__ import annotations

from math import cos, pi, sin, tau

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


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _tapered_beam_mesh(
    *,
    x0: float,
    x1: float,
    root_width: float,
    tip_width: float,
    root_height: float,
    tip_height: float,
) -> MeshGeometry:
    """Closed rectangular frustum, long in +X, for the tapered boom arm."""

    mesh = MeshGeometry()

    def add_section(x: float, width: float, height: float) -> list[int]:
        hy = width / 2.0
        hz = height / 2.0
        return [
            mesh.add_vertex(x, -hy, -hz),
            mesh.add_vertex(x, hy, -hz),
            mesh.add_vertex(x, hy, hz),
            mesh.add_vertex(x, -hy, hz),
        ]

    root = add_section(x0, root_width, root_height)
    tip = add_section(x1, tip_width, tip_height)

    # End caps.
    _add_quad(mesh, root[0], root[3], root[2], root[1])
    _add_quad(mesh, tip[0], tip[1], tip[2], tip[3])
    # Four tapered side faces.
    _add_quad(mesh, root[0], root[1], tip[1], tip[0])
    _add_quad(mesh, root[1], root[2], tip[2], tip[1])
    _add_quad(mesh, root[2], root[3], tip[3], tip[2])
    _add_quad(mesh, root[3], root[0], tip[0], tip[3])

    return mesh


def _hollow_cylinder_mesh(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    radial_segments: int = 48,
) -> MeshGeometry:
    """Closed annular cylinder centered on local Z, used as a rotating sleeve."""

    mesh = MeshGeometry()
    z0 = -length / 2.0
    z1 = length / 2.0
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for i in range(radial_segments):
        angle = tau * i / radial_segments
        ca = cos(angle)
        sa = sin(angle)
        outer_bottom.append(mesh.add_vertex(outer_radius * ca, outer_radius * sa, z0))
        outer_top.append(mesh.add_vertex(outer_radius * ca, outer_radius * sa, z1))
        inner_bottom.append(mesh.add_vertex(inner_radius * ca, inner_radius * sa, z0))
        inner_top.append(mesh.add_vertex(inner_radius * ca, inner_radius * sa, z1))

    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        # Outer barrel, inner bore, and annular end caps.
        _add_quad(mesh, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(mesh, inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        _add_quad(mesh, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        _add_quad(mesh, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_boom")

    model.material("painted_steel", rgba=(0.17, 0.19, 0.21, 1.0))
    model.material("warm_yellow", rgba=(0.93, 0.64, 0.16, 1.0))
    model.material("dark_carriage", rgba=(0.08, 0.09, 0.10, 1.0))
    model.material("wear_strip", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("fastener", rgba=(0.48, 0.50, 0.52, 1.0))
    model.material("glass", rgba=(0.03, 0.04, 0.045, 1.0))

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.32, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material="painted_steel",
        name="floor_plate",
    )
    post.visual(
        Cylinder(radius=0.055, length=1.430),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material="painted_steel",
        name="post_tube",
    )
    post.visual(
        Cylinder(radius=0.055, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 1.600)),
        material="fastener",
        name="spindle",
    )
    for idx, (x, y) in enumerate(((0.23, 0.0), (0.0, 0.23), (-0.23, 0.0), (0.0, -0.23))):
        post.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=Origin(xyz=(x, y, 0.068)),
            material="fastener",
            name=f"anchor_{idx}",
        )

    arm = model.part("arm")
    arm.visual(
        mesh_from_geometry(
            _hollow_cylinder_mesh(inner_radius=0.052, outer_radius=0.130, length=0.220),
            "arm_hub_sleeve",
        ),
        material="warm_yellow",
        name="hub_sleeve",
    )
    arm.visual(
        mesh_from_geometry(
            _tapered_beam_mesh(
                x0=0.090,
                x1=1.550,
                root_width=0.240,
                tip_width=0.130,
                root_height=0.180,
                tip_height=0.110,
            ),
            "tapered_boom_arm",
        ),
        material="warm_yellow",
        name="tapered_beam",
    )
    arm.visual(
        Box((1.250, 0.070, 0.035)),
        origin=Origin(xyz=(0.850, 0.0, -0.075)),
        material="wear_strip",
        name="slide_rail",
    )
    arm.visual(
        Box((0.080, 0.170, 0.050)),
        origin=Origin(xyz=(0.170, 0.0, -0.080)),
        material="warm_yellow",
        name="root_lug",
    )
    arm.visual(
        Box((0.070, 0.115, 0.038)),
        origin=Origin(xyz=(1.545, 0.0, -0.070)),
        material="warm_yellow",
        name="end_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.140, 0.160)),
        origin=Origin(xyz=(0.0, 0.0, -0.0975)),
        material="dark_carriage",
        name="block",
    )
    carriage.visual(
        Box((0.165, 0.025, 0.100)),
        origin=Origin(xyz=(0.0, 0.070, -0.065)),
        material="dark_carriage",
        name="guide_plate_0",
    )
    carriage.visual(
        Box((0.165, 0.025, 0.100)),
        origin=Origin(xyz=(0.0, -0.070, -0.065)),
        material="dark_carriage",
        name="guide_plate_1",
    )
    carriage.visual(
        Cylinder(radius=0.040, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.1840)),
        material="glass",
        name="inspection_window",
    )

    model.articulation(
        "post_to_arm",
        ArticulationType.REVOLUTE,
        parent=post,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=180.0, velocity=0.8),
    )
    model.articulation(
        "arm_to_carriage",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=carriage,
        origin=Origin(xyz=(0.450, 0.0, -0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.700, effort=120.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    post = object_model.get_part("post")
    arm = object_model.get_part("arm")
    carriage = object_model.get_part("carriage")
    swivel = object_model.get_articulation("post_to_arm")
    slide = object_model.get_articulation("arm_to_carriage")

    ctx.allow_overlap(
        arm,
        post,
        elem_a="hub_sleeve",
        elem_b="spindle",
        reason=(
            "The boom's annular sleeve is modeled with a tiny hidden bushing "
            "interference around the fixed spindle so the revolute bearing reads "
            "as physically captured and supported."
        ),
    )

    ctx.check(
        "radial arm uses vertical revolute post axis",
        swivel.articulation_type == ArticulationType.REVOLUTE and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "carriage uses prismatic arm-axis slide",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type}, axis={slide.axis}",
    )
    ctx.expect_overlap(
        arm,
        post,
        axes="z",
        elem_a="hub_sleeve",
        elem_b="spindle",
        min_overlap=0.18,
        name="arm sleeve is captured around spindle height",
    )
    ctx.expect_gap(
        arm,
        carriage,
        axis="z",
        positive_elem="slide_rail",
        negative_elem="block",
        max_penetration=0.00001,
        max_gap=0.002,
        name="carriage block hangs just below arm rail",
    )
    ctx.expect_overlap(
        carriage,
        arm,
        axes="x",
        elem_a="block",
        elem_b="slide_rail",
        min_overlap=0.18,
        name="carriage sits under the rail at rest",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.700}):
        ctx.expect_gap(
            arm,
            carriage,
            axis="z",
            positive_elem="slide_rail",
            negative_elem="block",
            max_penetration=0.00001,
            max_gap=0.002,
            name="extended carriage remains hanging below rail",
        )
        ctx.expect_overlap(
            carriage,
            arm,
            axes="x",
            elem_a="block",
            elem_b="slide_rail",
            min_overlap=0.18,
            name="extended carriage remains on the arm rail",
        )
        extended_carriage = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates outward along arm",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.65
        and abs(extended_carriage[1] - rest_carriage[1]) < 0.005,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_beam_aabb = ctx.part_element_world_aabb(arm, elem="tapered_beam")
    with ctx.pose({swivel: pi / 2.0}):
        swept_beam_aabb = ctx.part_element_world_aabb(arm, elem="tapered_beam")

    def _aabb_center_xy(aabb) -> tuple[float, float] | None:
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    rest_center = _aabb_center_xy(rest_beam_aabb)
    swept_center = _aabb_center_xy(swept_beam_aabb)
    ctx.check(
        "arm sweeps radially about the post",
        rest_center is not None
        and swept_center is not None
        and rest_center[0] > 0.75
        and swept_center[1] > 0.75
        and abs(swept_center[0]) < 0.20,
        details=f"rest_center={rest_center}, swept_center={swept_center}",
    )

    return ctx.report()


object_model = build_object_model()
