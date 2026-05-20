from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _annular_segment(
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 32,
) -> MeshGeometry:
    """Closed solid annular sector used for hollow drum walls and bearing rings."""
    geom = MeshGeometry()
    span = end_angle - start_angle
    closed_loop = math.isclose(abs(span), 2.0 * math.pi, rel_tol=0.0, abs_tol=1e-6)
    count = segments if closed_loop else segments + 1

    rings: list[tuple[int, int, int, int]] = []
    for i in range(count):
        t = start_angle + span * (i / segments)
        c = math.cos(t)
        s = math.sin(t)
        inner_bottom = geom.add_vertex(inner_radius * c, inner_radius * s, z_min)
        inner_top = geom.add_vertex(inner_radius * c, inner_radius * s, z_max)
        outer_bottom = geom.add_vertex(outer_radius * c, outer_radius * s, z_min)
        outer_top = geom.add_vertex(outer_radius * c, outer_radius * s, z_max)
        rings.append((inner_bottom, inner_top, outer_bottom, outer_top))

    last = count if closed_loop else count - 1
    for i in range(last):
        j = (i + 1) % count
        ib0, it0, ob0, ot0 = rings[i]
        ib1, it1, ob1, ot1 = rings[j]

        # Outer cylindrical face.
        geom.add_face(ob0, ob1, ot1)
        geom.add_face(ob0, ot1, ot0)
        # Inner cylindrical face, wound oppositely.
        geom.add_face(ib0, it1, ib1)
        geom.add_face(ib0, it0, it1)
        # Top and bottom annular faces.
        geom.add_face(it0, ot1, it1)
        geom.add_face(it0, ot0, ot1)
        geom.add_face(ib0, ib1, ob1)
        geom.add_face(ib0, ob1, ob0)

    if not closed_loop:
        for idx in (0, count - 1):
            ib, it, ob, ot = rings[idx]
            geom.add_face(ib, ob, ot)
            geom.add_face(ib, ot, it)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_wing_revolving_door")

    brushed_metal = Material("brushed_metal", rgba=(0.62, 0.64, 0.64, 1.0))
    dark_rubber = Material("dark_rubber", rgba=(0.035, 0.037, 0.04, 1.0))
    floor_mat = Material("dark_stone_floor", rgba=(0.18, 0.18, 0.17, 1.0))
    housing_glass = Material("slightly_blue_glass", rgba=(0.62, 0.84, 0.95, 0.34))
    wing_glass = Material("clear_wing_glass", rgba=(0.72, 0.90, 1.0, 0.42))

    # Fixed cylindrical drum: a circular floor and canopy, two curved glass wall
    # sectors, and four vertical jamb posts that mark the opposing entry gaps.
    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=1.08, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=floor_mat,
        name="floor_disk",
    )
    drum.visual(
        Cylinder(radius=1.08, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.56)),
        material=brushed_metal,
        name="top_canopy",
    )

    for index, (start_deg, end_deg) in enumerate(((35.0, 145.0), (215.0, 325.0))):
        wall_mesh = mesh_from_geometry(
            _annular_segment(
                inner_radius=0.99,
                outer_radius=1.025,
                z_min=0.074,
                z_max=2.506,
                start_angle=math.radians(start_deg),
                end_angle=math.radians(end_deg),
                segments=36,
            ),
            f"curved_glass_{index}",
        )
        drum.visual(
            wall_mesh,
            material=housing_glass,
            name=f"curved_glass_{index}",
        )

    for index, angle in enumerate(math.radians(a) for a in (35.0, 145.0, 215.0, 325.0)):
        drum.visual(
            Cylinder(radius=0.035, length=2.44),
            origin=Origin(xyz=(1.005 * math.cos(angle), 1.005 * math.sin(angle), 1.29)),
            material=brushed_metal,
            name=f"jamb_post_{index}",
        )

    for name, z_min, z_max in (("bottom_bearing", 0.045, 0.105), ("top_bearing", 2.475, 2.535)):
        bearing_mesh = mesh_from_geometry(
            _annular_segment(
                inner_radius=0.060,
                outer_radius=0.125,
                z_min=z_min,
                z_max=z_max,
                start_angle=0.0,
                end_angle=2.0 * math.pi,
                segments=40,
            ),
            name,
        )
        drum.visual(bearing_mesh, material=brushed_metal, name=name)

    # Rotating child frame is coincident with the drum axis.  Each flat wing is
    # a radial glass rectangle with metal rails and a free-edge stile.
    wings = model.part("wing_carrier")
    wings.visual(
        Cylinder(radius=0.060, length=2.40),
        origin=Origin(xyz=(0.0, 0.0, 1.29)),
        material=brushed_metal,
        name="central_shaft",
    )
    wings.visual(
        Cylinder(radius=0.075, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=brushed_metal,
        name="lower_hub",
    )
    wings.visual(
        Cylinder(radius=0.075, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 2.365)),
        material=brushed_metal,
        name="upper_hub",
    )

    wing_radius = 0.955
    panel_height = 2.34
    panel_z = 1.29
    panel_thickness = 0.032
    rail_height = 0.045
    wing_names = (
        ("wing_0_panel", "wing_0_bottom_rail", "wing_0_top_rail", "wing_0_stile"),
        ("wing_1_panel", "wing_1_bottom_rail", "wing_1_top_rail", "wing_1_stile"),
        ("wing_2_panel", "wing_2_bottom_rail", "wing_2_top_rail", "wing_2_stile"),
        ("wing_3_panel", "wing_3_bottom_rail", "wing_3_top_rail", "wing_3_stile"),
    )
    for index, (panel_name, bottom_rail_name, top_rail_name, stile_name) in enumerate(wing_names):
        angle = index * math.pi / 2.0
        c = math.cos(angle)
        s = math.sin(angle)
        yaw = angle
        center = (0.5 * wing_radius * c, 0.5 * wing_radius * s, panel_z)
        wings.visual(
            Box((wing_radius, panel_thickness, panel_height)),
            origin=Origin(xyz=center, rpy=(0.0, 0.0, yaw)),
            material=wing_glass,
            name=panel_name,
        )
        for rail_name, z in ((bottom_rail_name, 0.185), (top_rail_name, 2.395)):
            wings.visual(
                Box((wing_radius, 0.052, rail_height)),
                origin=Origin(xyz=(center[0], center[1], z), rpy=(0.0, 0.0, yaw)),
                material=brushed_metal,
                name=rail_name,
            )
        wings.visual(
            Box((0.030, 0.060, panel_height)),
            origin=Origin(
                xyz=(wing_radius * c, wing_radius * s, panel_z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=brushed_metal,
            name=stile_name,
        )

    model.articulation(
        "drum_to_wings",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=wings,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    wings = object_model.get_part("wing_carrier")
    spin = object_model.get_articulation("drum_to_wings")

    for bearing_name in ("bottom_bearing", "top_bearing"):
        ctx.allow_overlap(
            drum,
            wings,
            elem_a=bearing_name,
            elem_b="central_shaft",
            reason="The rotating vertical shaft is intentionally captured inside the fixed bearing ring.",
        )
        ctx.expect_within(
            wings,
            drum,
            axes="xy",
            inner_elem="central_shaft",
            outer_elem=bearing_name,
            margin=0.0,
            name=f"central shaft is centered inside {bearing_name}",
        )
        ctx.expect_overlap(
            drum,
            wings,
            axes="z",
            elem_a=bearing_name,
            elem_b="central_shaft",
            min_overlap=0.01,
            name=f"central shaft is vertically retained by {bearing_name}",
        )

    joint_type = getattr(spin, "articulation_type", getattr(spin, "type", None))
    ctx.check(
        "wing assembly has continuous vertical rotation",
        joint_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={joint_type}, axis={getattr(spin, 'axis', None)}",
    )

    wing_box = ctx.part_element_world_aabb(wings, elem="wing_0_panel")
    wing_height = None
    if wing_box is not None:
        wing_height = wing_box[1][2] - wing_box[0][2]
    ctx.check(
        "wing panels span nearly floor to ceiling",
        wing_height is not None and wing_height > 2.30,
        details=f"wing_0_panel height={wing_height}",
    )

    ctx.expect_within(
        wings,
        drum,
        axes="xy",
        inner_elem="wing_0_stile",
        outer_elem="floor_disk",
        margin=0.0,
        name="wing tip remains inside circular drum footprint",
    )
    ctx.expect_gap(
        wings,
        drum,
        axis="z",
        positive_elem="wing_0_panel",
        negative_elem="floor_disk",
        min_gap=0.02,
        max_gap=0.06,
        name="wing panel clears the floor slab",
    )
    ctx.expect_gap(
        drum,
        wings,
        axis="z",
        positive_elem="top_canopy",
        negative_elem="wing_0_panel",
        min_gap=0.02,
        max_gap=0.08,
        name="wing panel clears the canopy",
    )

    with ctx.pose({spin: math.pi / 4.0}):
        ctx.expect_within(
            wings,
            drum,
            axes="xy",
            inner_elem="wing_0_stile",
            outer_elem="floor_disk",
            margin=0.0,
            name="rotated wing tip remains within the drum",
        )

    return ctx.report()


object_model = build_object_model()
