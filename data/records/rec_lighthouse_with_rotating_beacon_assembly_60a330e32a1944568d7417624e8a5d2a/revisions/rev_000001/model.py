from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_lighthouse_lantern")

    iron = model.material("blackened_cast_iron", rgba=(0.015, 0.014, 0.012, 1.0))
    worn_iron = model.material("worn_cast_iron_edges", rgba=(0.09, 0.085, 0.075, 1.0))
    dark = model.material("dark_cut_shadow", rgba=(0.0, 0.0, 0.0, 1.0))
    glass = model.material("greenish_lantern_glass", rgba=(0.58, 0.88, 0.95, 0.33))
    optic_glass = model.material("clear_fresnel_glass", rgba=(0.72, 0.95, 1.0, 0.48))
    brass = model.material("aged_brass", rgba=(0.72, 0.52, 0.20, 1.0))
    glow = model.material("warm_lamp_glow", rgba=(1.0, 0.62, 0.18, 0.78))

    body = model.part("lantern_body")

    base_shell = LatheGeometry.from_shell_profiles(
        [
            (0.52, 0.00),
            (0.72, 0.055),
            (0.75, 0.13),
            (0.70, 0.18),
            (0.69, 0.43),
            (0.75, 0.50),
            (0.58, 0.58),
        ],
        [
            (0.34, 0.035),
            (0.47, 0.11),
            (0.52, 0.19),
            (0.53, 0.43),
            (0.45, 0.53),
            (0.31, 0.56),
        ],
        segments=80,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    body.visual(
        mesh_from_geometry(base_shell, "base_curb_shell"),
        material=iron,
        name="base_curb",
    )

    for z, ring_radius, tube, name in (
        (0.055, 0.665, 0.022, "lower_moulding"),
        (0.50, 0.720, 0.026, "upper_moulding"),
        (0.60, 0.665, 0.020, "glass_sill_ring"),
        (1.64, 0.665, 0.022, "glass_head_ring"),
    ):
        body.visual(
            mesh_from_geometry(TorusGeometry(ring_radius, tube, radial_segments=24, tubular_segments=96), name),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=worn_iron,
            name=name,
        )

    body.visual(
        Cylinder(radius=0.67, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=iron,
        name="lantern_floor_plate",
    )

    # A dark, recessed rectangle and proud trim make the curb read as cut for the
    # maintenance access panel without letting the hinged panel intersect the base.
    body.visual(
        Box((0.39, 0.008, 0.30)),
        origin=Origin(xyz=(0.0, 0.692, 0.31)),
        material=dark,
        name="maintenance_recess",
    )
    body.visual(
        Box((0.43, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, 0.690, 0.465)),
        material=worn_iron,
        name="recess_top_lip",
    )
    body.visual(
        Box((0.43, 0.012, 0.025)),
        origin=Origin(xyz=(0.0, 0.690, 0.155)),
        material=worn_iron,
        name="recess_bottom_lip",
    )
    body.visual(
        Box((0.025, 0.012, 0.30)),
        origin=Origin(xyz=(-0.215, 0.690, 0.31)),
        material=worn_iron,
        name="recess_side_lip_0",
    )
    body.visual(
        Box((0.025, 0.012, 0.30)),
        origin=Origin(xyz=(0.215, 0.690, 0.31)),
        material=worn_iron,
        name="recess_side_lip_1",
    )
    body.visual(
        Box((0.022, 0.104, 0.30)),
        origin=Origin(xyz=(-0.185, 0.692, 0.31)),
        material=worn_iron,
        name="fixed_hinge_leaf",
    )

    pane_radius = 0.635
    pane_width = 2.0 * pane_radius * math.sin(math.pi / 8.0) - 0.045
    for i in range(8):
        theta = i * math.tau / 8.0
        x = pane_radius * math.sin(theta)
        y = pane_radius * math.cos(theta)
        body.visual(
            Box((pane_width, 0.018, 0.98)),
            origin=Origin(xyz=(x, y, 1.13), rpy=(0.0, 0.0, -theta)),
            material=glass,
            name=f"glass_pane_{i}",
        )

    post_radius = 0.672
    for i in range(8):
        theta = (i + 0.5) * math.tau / 8.0
        body.visual(
            Cylinder(radius=0.026, length=1.08),
            origin=Origin(
                xyz=(post_radius * math.sin(theta), post_radius * math.cos(theta), 1.13)
            ),
            material=iron,
            name=f"mullion_{i}",
        )

    roof = LatheGeometry(
        [
            (0.0, 1.625),
            (0.74, 1.625),
            (0.68, 1.72),
            (0.49, 1.91),
            (0.19, 2.07),
            (0.0, 2.11),
        ],
        segments=80,
        closed=True,
    )
    body.visual(
        mesh_from_geometry(roof, "copper_black_roof"),
        material=iron,
        name="domed_roof",
    )
    body.visual(
        Cylinder(radius=0.11, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 2.16)),
        material=worn_iron,
        name="ventilator_stack",
    )
    body.visual(
        Sphere(radius=0.075),
        origin=Origin(xyz=(0.0, 0.0, 2.245)),
        material=worn_iron,
        name="roof_finial",
    )

    # Static pedestal inside the glass room.  The rotating optic shaft sits on
    # the pedestal cap at z=1.00.
    body.visual(
        Cylinder(radius=0.23, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=worn_iron,
        name="pedestal_foot",
    )
    body.visual(
        Cylinder(radius=0.12, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=iron,
        name="pedestal_column",
    )
    body.visual(
        Cylinder(radius=0.18, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=brass,
        name="pedestal_cap",
    )

    optic = model.part("optic")
    optic.visual(
        Cylinder(radius=0.032, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
        material=brass,
        name="center_shaft",
    )
    optic.visual(
        Sphere(radius=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=glow,
        name="lamp_globe",
    )
    for z, name in ((0.17, "lower_optic_ring"), (0.55, "upper_optic_ring")):
        optic.visual(
            mesh_from_geometry(TorusGeometry(0.335, 0.013, radial_segments=16, tubular_segments=72), name),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=brass,
            name=name,
        )
    for i in range(8):
        theta = i * math.tau / 8.0
        optic.visual(
            Cylinder(radius=0.010, length=0.42),
            origin=Origin(xyz=(0.335 * math.sin(theta), 0.335 * math.cos(theta), 0.36)),
            material=brass,
            name=f"optic_cage_rod_{i}",
        )

    lens_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, -0.016),
                (0.040, -0.019),
                (0.055, -0.016),
                (0.080, -0.022),
                (0.095, -0.017),
                (0.125, -0.015),
                (0.155, -0.007),
                (0.155, 0.007),
                (0.125, 0.015),
                (0.095, 0.017),
                (0.080, 0.022),
                (0.055, 0.016),
                (0.040, 0.019),
                (0.0, 0.016),
            ],
            segments=64,
            closed=True,
        ),
        "convex_fresnel_lens",
    )
    for i, theta in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        lens_origin = Origin(
            xyz=(0.285 * math.sin(theta), 0.285 * math.cos(theta), 0.36),
            rpy=(-math.pi / 2.0, 0.0, -theta),
        )
        optic.visual(lens_mesh, origin=lens_origin, material=optic_glass, name=f"fresnel_lens_{i}")
        optic.visual(
            Box((0.022, 0.36, 0.020)),
            origin=Origin(
                xyz=(0.18 * math.sin(theta), 0.18 * math.cos(theta), 0.36),
                rpy=(0.0, 0.0, -theta),
            ),
            material=brass,
            name=f"radial_spoke_{i}",
        )

    panel = model.part("maintenance_panel")
    panel.visual(
        Box((0.32, 0.026, 0.26)),
        origin=Origin(xyz=(0.160, -0.023, 0.0)),
        material=iron,
        name="panel_plate",
    )
    panel.visual(
        Box((0.25, 0.011, 0.018)),
        origin=Origin(xyz=(0.175, -0.007, 0.105)),
        material=worn_iron,
        name="panel_top_rail",
    )
    panel.visual(
        Box((0.25, 0.011, 0.018)),
        origin=Origin(xyz=(0.175, -0.007, -0.105)),
        material=worn_iron,
        name="panel_bottom_rail",
    )
    panel.visual(
        Box((0.018, 0.011, 0.21)),
        origin=Origin(xyz=(0.055, -0.007, 0.0)),
        material=worn_iron,
        name="panel_hinge_stile",
    )
    panel.visual(
        Box((0.018, 0.011, 0.21)),
        origin=Origin(xyz=(0.295, -0.007, 0.0)),
        material=worn_iron,
        name="panel_latch_stile",
    )
    panel.visual(
        Cylinder(radius=0.014, length=0.31),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=worn_iron,
        name="hinge_barrel",
    )
    panel.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.275, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="round_latch_boss",
    )

    model.articulation(
        "optic_rotation",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=optic,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.8),
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(-0.16, 0.742, 0.31)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("lantern_body")
    optic = object_model.get_part("optic")
    panel = object_model.get_part("maintenance_panel")
    optic_joint = object_model.get_articulation("optic_rotation")
    panel_joint = object_model.get_articulation("panel_hinge")

    ctx.check(
        "optic has continuous vertical rotation",
        optic_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(optic_joint.axis) == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "maintenance panel has realistic hinge limits",
        panel_joint.motion_limits is not None
        and panel_joint.motion_limits.lower == 0.0
        and panel_joint.motion_limits.upper is not None
        and 1.3 < panel_joint.motion_limits.upper < 1.7,
    )

    ctx.expect_gap(
        panel,
        body,
        axis="y",
        min_gap=0.003,
        max_gap=0.020,
        positive_elem="panel_plate",
        negative_elem="maintenance_recess",
        name="closed panel sits just proud of cut recess",
    )
    ctx.expect_overlap(
        panel,
        body,
        axes="xz",
        min_overlap=0.20,
        elem_a="panel_plate",
        elem_b="maintenance_recess",
        name="panel covers the rectangular curb opening",
    )
    ctx.expect_contact(
        optic,
        body,
        elem_a="center_shaft",
        elem_b="pedestal_cap",
        contact_tol=0.002,
        name="rotating shaft sits on central pedestal",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_plate")
    with ctx.pose({panel_joint: 1.20}):
        opened_panel_aabb = ctx.part_element_world_aabb(panel, elem="panel_plate")
    ctx.check(
        "panel hinge swings outward from curb",
        closed_panel_aabb is not None
        and opened_panel_aabb is not None
        and opened_panel_aabb[1][1] > closed_panel_aabb[1][1] + 0.10,
        details=f"closed={closed_panel_aabb}, opened={opened_panel_aabb}",
    )

    def _aabb_center_xy(aabb):
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)

    lens_start = ctx.part_element_world_aabb(optic, elem="fresnel_lens_0")
    with ctx.pose({optic_joint: math.pi / 2.0}):
        lens_quarter = ctx.part_element_world_aabb(optic, elem="fresnel_lens_0")
    start_xy = _aabb_center_xy(lens_start) if lens_start else None
    quarter_xy = _aabb_center_xy(lens_quarter) if lens_quarter else None
    ctx.check(
        "optic lens changes azimuth under continuous joint",
        start_xy is not None
        and quarter_xy is not None
        and abs(start_xy[0] - quarter_xy[0]) > 0.18
        and abs(start_xy[1] - quarter_xy[1]) > 0.18,
        details=f"start={start_xy}, quarter={quarter_xy}",
    )

    return ctx.report()


object_model = build_object_model()
