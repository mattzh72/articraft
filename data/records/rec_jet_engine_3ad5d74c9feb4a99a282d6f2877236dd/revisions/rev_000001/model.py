from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _axisymmetric_shell(
    sections: list[tuple[float, float, float]], *, segments: int = 96
) -> MeshGeometry:
    """Build an annular, open-ended duct about the model X axis."""
    geom = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []

    for x, outer_r, inner_r in sections:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for j in range(segments):
            angle = 2.0 * math.pi * j / segments
            ca = math.cos(angle)
            sa = math.sin(angle)
            outer_ring.append(geom.add_vertex(x, outer_r * ca, outer_r * sa))
            inner_ring.append(geom.add_vertex(x, inner_r * ca, inner_r * sa))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for i in range(len(sections) - 1):
        for j in range(segments):
            j2 = (j + 1) % segments
            # Outer cowling skin.
            geom.add_face(outer[i][j], outer[i + 1][j], outer[i + 1][j2])
            geom.add_face(outer[i][j], outer[i + 1][j2], outer[i][j2])
            # Inner acoustic liner skin.
            geom.add_face(inner[i][j], inner[i + 1][j2], inner[i + 1][j])
            geom.add_face(inner[i][j], inner[i][j2], inner[i + 1][j2])

    # Annular lips at the intake and exhaust ends leave the duct visibly hollow.
    for ring_index in (0, len(sections) - 1):
        for j in range(segments):
            j2 = (j + 1) % segments
            if ring_index == 0:
                geom.add_face(outer[ring_index][j], outer[ring_index][j2], inner[ring_index][j2])
                geom.add_face(outer[ring_index][j], inner[ring_index][j2], inner[ring_index][j])
            else:
                geom.add_face(outer[ring_index][j], inner[ring_index][j2], outer[ring_index][j2])
                geom.add_face(outer[ring_index][j], inner[ring_index][j], inner[ring_index][j2])

    return geom


def _axisymmetric_solid(
    profile: list[tuple[float, float]], *, segments: int = 72
) -> MeshGeometry:
    """Build a closed spinner/core body about the model X axis."""
    geom = MeshGeometry()
    rings: list[list[int]] = []

    for x, radius in profile:
        if radius <= 1e-7:
            rings.append([geom.add_vertex(x, 0.0, 0.0)])
            continue
        ring: list[int] = []
        for j in range(segments):
            angle = 2.0 * math.pi * j / segments
            ring.append(geom.add_vertex(x, radius * math.cos(angle), radius * math.sin(angle)))
        rings.append(ring)

    for i in range(len(profile) - 1):
        ring_a = rings[i]
        ring_b = rings[i + 1]
        a_tip = len(ring_a) == 1
        b_tip = len(ring_b) == 1
        for j in range(segments):
            j2 = (j + 1) % segments
            if a_tip and not b_tip:
                geom.add_face(ring_a[0], ring_b[j], ring_b[j2])
            elif b_tip and not a_tip:
                geom.add_face(ring_a[j], ring_b[0], ring_a[j2])
            elif not a_tip and not b_tip:
                geom.add_face(ring_a[j], ring_b[j], ring_b[j2])
                geom.add_face(ring_a[j], ring_b[j2], ring_a[j2])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airliner_turbofan_access_panel")

    gloss_white = model.material("gloss_white", rgba=(0.92, 0.94, 0.93, 1.0))
    panel_white = model.material("panel_white", rgba=(0.86, 0.88, 0.87, 1.0))
    dark_composite = model.material("dark_composite", rgba=(0.035, 0.038, 0.043, 1.0))
    titanium = model.material("brushed_titanium", rgba=(0.46, 0.47, 0.48, 1.0))
    steel = model.material("hinge_steel", rgba=(0.62, 0.62, 0.60, 1.0))

    nacelle = model.part("nacelle")

    # A wide-body under-wing turbofan at real scale: about 1.7 m fan-case diameter
    # and a hollow annular nacelle enclosing both the fan face and aft core.
    nacelle_sections = [
        (-0.95, 0.78, 0.60),
        (-0.84, 0.86, 0.64),
        (-0.55, 0.86, 0.67),
        (-0.12, 0.81, 0.66),
        (0.56, 0.71, 0.54),
        (1.20, 0.58, 0.42),
    ]
    nacelle.visual(
        mesh_from_geometry(_axisymmetric_shell(nacelle_sections), "nacelle_shell"),
        material=gloss_white,
        name="nacelle_shell",
    )

    core_profile = [
        (-0.36, 0.0),
        (-0.29, 0.15),
        (-0.08, 0.25),
        (0.62, 0.30),
        (1.06, 0.18),
        (1.16, 0.0),
    ]
    nacelle.visual(
        mesh_from_geometry(_axisymmetric_solid(core_profile), "core_fairing"),
        material=dark_composite,
        name="core_fairing",
    )

    # Stator/OGV struts visibly tie the central core to the fan case behind the rotor.
    for name, size, xyz in (
        ("stator_top", (0.08, 0.055, 0.62), (-0.23, 0.0, 0.46)),
        ("stator_bottom", (0.08, 0.055, 0.62), (-0.23, 0.0, -0.46)),
        ("stator_side_0", (0.08, 0.62, 0.055), (-0.23, 0.46, 0.0)),
        ("stator_side_1", (0.08, 0.62, 0.055), (-0.23, -0.46, 0.0)),
    ):
        nacelle.visual(Box(size), origin=Origin(xyz=xyz), material=titanium, name=name)

    # A shaft through the rotor bore makes the fan visibly mounted rather than floating.
    nacelle.visual(
        Cylinder(radius=0.073, length=0.235),
        origin=Origin(xyz=(-0.3975, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="fan_shaft",
    )
    nacelle.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(-0.34, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_bearing",
    )

    # A short upper pylon stub helps the assembly read as an airliner engine.
    nacelle.visual(
        Box((0.82, 0.24, 0.34)),
        origin=Origin(xyz=(0.02, 0.0, 1.00)),
        material=gloss_white,
        name="pylon_stub",
    )

    hinge_x = -0.77
    hinge_z = -0.905
    for y in (-0.18, 0.18):
        nacelle.visual(
            Cylinder(radius=0.022, length=0.12),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fixed_hinge_{0 if y < 0 else 1}",
        )
        nacelle.visual(
            Box((0.055, 0.12, 0.090)),
            origin=Origin(xyz=(hinge_x, y, hinge_z + 0.045)),
            material=steel,
            name=f"hinge_bracket_{0 if y < 0 else 1}",
        )
    nacelle.visual(
        Cylinder(radius=0.010, length=0.54),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )

    fan_rotor = model.part("fan_rotor")
    fan_mesh = FanRotorGeometry(
        0.60,
        0.16,
        22,
        thickness=0.11,
        blade_pitch_deg=33.0,
        blade_sweep_deg=36.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.16, tip_clearance=0.018),
        hub=FanRotorHub(
            style="spinner",
            rear_collar_height=0.035,
            rear_collar_radius=0.145,
            bore_diameter=0.144,
        ),
    )
    fan_rotor.visual(
        mesh_from_geometry(fan_mesh, "fan_blades"),
        # Fan helper spins about local Z; rotate that axis onto the engine X axis.
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="fan_blades",
    )

    model.articulation(
        "nacelle_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=fan_rotor,
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=80.0),
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.50, 0.50, 0.026)),
        origin=Origin(xyz=(0.285, 0.0, -0.038)),
        material=panel_white,
        name="panel_skin",
    )
    access_panel.visual(
        Box((0.12, 0.18, 0.014)),
        origin=Origin(xyz=(0.055, 0.0, -0.019)),
        material=steel,
        name="moving_leaf",
    )
    access_panel.visual(
        Cylinder(radius=0.022, length=0.20),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="moving_hinge",
    )
    access_panel.visual(
        Box((0.09, 0.17, 0.014)),
        origin=Origin(xyz=(0.455, 0.0, -0.055)),
        material=dark_composite,
        name="panel_latch",
    )

    model.articulation(
        "nacelle_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=nacelle,
        child=access_panel,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    nacelle = object_model.get_part("nacelle")
    fan_rotor = object_model.get_part("fan_rotor")
    access_panel = object_model.get_part("access_panel")
    fan_joint = object_model.get_articulation("nacelle_to_fan_rotor")
    panel_joint = object_model.get_articulation("nacelle_to_access_panel")

    ctx.check(
        "fan rotor has continuous longitudinal spin",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(fan_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={fan_joint.articulation_type}, axis={fan_joint.axis}",
    )
    limits = panel_joint.motion_limits
    ctx.check(
        "access panel has fold-down hinge limits",
        panel_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(panel_joint.axis) == (0.0, 1.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 1.2,
        details=f"type={panel_joint.articulation_type}, axis={panel_joint.axis}, limits={limits}",
    )

    ctx.allow_overlap(
        nacelle,
        fan_rotor,
        elem_a="fan_shaft",
        elem_b="fan_blades",
        reason="The stationary shaft is intentionally seated a short distance inside the rotor hub/bore as a captured bearing support.",
    )
    ctx.expect_within(
        nacelle,
        fan_rotor,
        axes="yz",
        inner_elem="fan_shaft",
        outer_elem="fan_blades",
        margin=0.0,
        name="fan shaft is centered in rotor hub",
    )
    ctx.expect_overlap(
        nacelle,
        fan_rotor,
        axes="x",
        elem_a="fan_shaft",
        elem_b="fan_blades",
        min_overlap=0.004,
        name="fan shaft is retained in rotor hub",
    )

    ctx.allow_overlap(
        nacelle,
        access_panel,
        elem_a="hinge_pin",
        elem_b="moving_hinge",
        reason="The hinge pin is intentionally captured inside the moving hinge barrel so the access door is mechanically mounted.",
    )
    ctx.expect_within(
        nacelle,
        access_panel,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="moving_hinge",
        margin=0.0,
        name="hinge pin sits inside moving hinge barrel",
    )
    ctx.expect_overlap(
        nacelle,
        access_panel,
        axes="y",
        elem_a="hinge_pin",
        elem_b="moving_hinge",
        min_overlap=0.18,
        name="hinge pin spans moving hinge barrel",
    )

    ctx.expect_within(
        fan_rotor,
        nacelle,
        axes="yz",
        inner_elem="fan_blades",
        outer_elem="nacelle_shell",
        margin=0.0,
        name="fan rotor sits inside nacelle annulus",
    )
    ctx.expect_overlap(
        fan_rotor,
        nacelle,
        axes="x",
        elem_a="fan_blades",
        elem_b="nacelle_shell",
        min_overlap=0.08,
        name="fan rotor is located within intake section",
    )
    ctx.expect_gap(
        nacelle,
        access_panel,
        axis="z",
        positive_elem="nacelle_shell",
        negative_elem="panel_skin",
        min_gap=0.005,
        max_gap=0.08,
        name="closed access panel is just below fan case",
    )

    closed_aabb = ctx.part_element_world_aabb(access_panel, elem="panel_skin")
    with ctx.pose({panel_joint: 1.15}):
        open_aabb = ctx.part_element_world_aabb(access_panel, elem="panel_skin")
    ctx.check(
        "access panel folds downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
