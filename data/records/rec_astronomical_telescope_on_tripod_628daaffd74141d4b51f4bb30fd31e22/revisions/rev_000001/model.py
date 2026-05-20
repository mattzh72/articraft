from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_between(part, start, end, radius, material, name, *, segments=32):
    """Add a cylinder whose local Z axis spans start -> end in the part frame."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    horizontal = math.sqrt(vx * vx + vy * vy)
    yaw = math.atan2(vy, vx) if horizontal > 1e-9 else 0.0
    pitch = math.atan2(horizontal, vz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refractor_alt_az_tripod")

    cast_iron = model.material("blackened_cast_iron", color=(0.015, 0.018, 0.017, 1.0))
    worn_edges = model.material("rubbed_iron_edges", color=(0.12, 0.13, 0.12, 1.0))
    brass = model.material("aged_brass", color=(0.72, 0.52, 0.23, 1.0))
    cream_tube = model.material("cream_enamel", color=(0.86, 0.84, 0.76, 1.0))
    black_trim = model.material("matte_black_trim", color=(0.005, 0.005, 0.006, 1.0))
    lens_glass = model.material("pale_coated_glass", color=(0.35, 0.62, 0.78, 0.45))
    dark_wood = model.material("dark_varnished_wood", color=(0.24, 0.12, 0.055, 1.0))

    tripod = model.part("tripod")
    # Ground footprint and three splayed wooden/iron tripod legs, all tied into
    # the same central casting so the root assembly reads as a supported tripod.
    tripod.visual(
        Cylinder(radius=0.070, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="floor_boss",
    )
    tripod.visual(
        Cylinder(radius=0.055, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=cast_iron,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.105, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        material=cast_iron,
        name="leg_crown",
    )
    tripod.visual(
        Cylinder(radius=0.095, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.845)),
        material=worn_edges,
        name="azimuth_seat",
    )
    for i, angle in enumerate((math.radians(90), math.radians(210), math.radians(330))):
        x = 0.62 * math.cos(angle)
        y = 0.62 * math.sin(angle)
        _cylinder_between(
            tripod,
            (0.035 * math.cos(angle), 0.035 * math.sin(angle), 0.72),
            (x, y, 0.045),
            0.025,
            dark_wood,
            f"leg_{i}",
        )
        _cylinder_between(
            tripod,
            (0.12 * math.cos(angle), 0.12 * math.sin(angle), 0.42),
            (0.45 * math.cos(angle), 0.45 * math.sin(angle), 0.25),
            0.012,
            cast_iron,
            f"leg_brace_{i}",
        )
        tripod.visual(
            Box((0.16, 0.055, 0.035)),
            origin=Origin(xyz=(x, y, 0.025), rpy=(0.0, 0.0, angle)),
            material=cast_iron,
            name=f"foot_{i}",
        )

    azimuth_head = model.part("azimuth_head")
    azimuth_head.visual(
        Cylinder(radius=0.115, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=cast_iron,
        name="turntable",
    )
    azimuth_head.visual(
        Cylinder(radius=0.060, length=0.245),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=cast_iron,
        name="neck",
    )
    azimuth_head.visual(
        Box((0.28, 0.42, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=cast_iron,
        name="yoke_bridge",
    )
    azimuth_head.visual(
        Box((0.20, 0.060, 0.38)),
        origin=Origin(xyz=(0.0, 0.18, 0.43)),
        material=cast_iron,
        name="yoke_cheek_0",
    )
    azimuth_head.visual(
        Cylinder(radius=0.067, length=0.018),
        origin=Origin(xyz=(0.0, 0.219, 0.52), rpy=(math.pi / 2, 0.0, 0.0)),
        material=worn_edges,
        name="bearing_cap_0",
    )
    azimuth_head.visual(
        Box((0.20, 0.060, 0.38)),
        origin=Origin(xyz=(0.0, -0.18, 0.43)),
        material=cast_iron,
        name="yoke_cheek_1",
    )
    azimuth_head.visual(
        Cylinder(radius=0.067, length=0.018),
        origin=Origin(xyz=(0.0, -0.219, 0.52), rpy=(math.pi / 2, 0.0, 0.0)),
        material=worn_edges,
        name="bearing_cap_1",
    )
    azimuth_head.visual(
        Box((0.040, 0.34, 0.11)),
        origin=Origin(xyz=(0.095, 0.0, 0.35)),
        material=cast_iron,
        name="front_rib",
    )

    telescope = model.part("telescope")
    main_tube_mesh = mesh_from_geometry(
        CylinderGeometry(0.072, 1.48, radial_segments=72, closed=False),
        "main_open_tube",
    )
    telescope.visual(
        main_tube_mesh,
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=cream_tube,
        name="main_tube",
    )
    telescope.visual(
        Cylinder(radius=0.078, length=0.055),
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=brass,
        name="balance_band",
    )
    telescope.visual(
        Cylinder(radius=0.050, length=0.120),
        origin=Origin(xyz=(0.0, 0.108, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="trunnion_0",
    )
    telescope.visual(
        Cylinder(radius=0.050, length=0.120),
        origin=Origin(xyz=(0.0, -0.108, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="trunnion_1",
    )
    telescope.visual(
        Cylinder(radius=0.094, length=0.22),
        origin=Origin(xyz=(0.89, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=black_trim,
        name="dew_shield",
    )
    telescope.visual(
        Cylinder(radius=0.070, length=0.018),
        origin=Origin(xyz=(0.995, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=lens_glass,
        name="objective_lens",
    )
    telescope.visual(
        Cylinder(radius=0.052, length=0.035),
        origin=Origin(xyz=(-0.665, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=black_trim,
        name="rear_cell",
    )
    telescope.visual(
        Cylinder(radius=0.080, length=0.045),
        origin=Origin(xyz=(-0.655, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=brass,
        name="rear_flange",
    )
    telescope.visual(
        Box((0.22, 0.12, 0.095)),
        origin=Origin(xyz=(-0.775, 0.0, -0.02)),
        material=black_trim,
        name="focuser_block",
    )
    telescope.visual(
        Cylinder(radius=0.032, length=0.18),
        origin=Origin(xyz=(-0.91, 0.0, -0.02), rpy=(0.0, math.pi / 2, 0.0)),
        material=black_trim,
        name="drawtube",
    )
    telescope.visual(
        Cylinder(radius=0.022, length=0.11),
        origin=Origin(xyz=(-1.03, 0.0, 0.015), rpy=(0.0, -math.pi / 2, 0.0)),
        material=black_trim,
        name="eyepiece_barrel",
    )
    telescope.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(-1.092, 0.0, 0.015)),
        material=lens_glass,
        name="eyepiece_glass",
    )
    # Small finder scope above the main tube, held by two bracket feet.
    telescope.visual(
        Cylinder(radius=0.022, length=0.55),
        origin=Origin(xyz=(0.09, 0.0, 0.132), rpy=(0.0, math.pi / 2, 0.0)),
        material=black_trim,
        name="finder_tube",
    )
    telescope.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.375, 0.0, 0.132), rpy=(0.0, math.pi / 2, 0.0)),
        material=lens_glass,
        name="finder_lens",
    )
    for x, suffix in ((-0.10, "0"), (0.26, "1")):
        telescope.visual(
            Box((0.030, 0.018, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.094)),
            material=brass,
            name=f"finder_stem_{suffix}",
        )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="shaft",
    )
    focus_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.056,
                0.026,
                body_style="cylindrical",
                grip=KnobGrip(style="knurled", count=36, depth=0.0014, helix_angle_deg=18.0),
            ),
            "fine_focus_knob",
        ),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=black_trim,
        name="knurled_wheel",
    )

    model.articulation(
        "tripod_to_azimuth",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=azimuth_head,
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.8, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "azimuth_to_telescope",
        ArticulationType.REVOLUTE,
        parent=azimuth_head,
        child=telescope,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.7, lower=-0.25, upper=1.20),
    )
    model.articulation(
        "telescope_to_focus_knob",
        ArticulationType.REVOLUTE,
        parent=telescope,
        child=focus_knob,
        origin=Origin(xyz=(-0.79, -0.060, -0.02)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    azimuth_head = object_model.get_part("azimuth_head")
    telescope = object_model.get_part("telescope")
    focus_knob = object_model.get_part("focus_knob")
    azimuth = object_model.get_articulation("tripod_to_azimuth")
    altitude = object_model.get_articulation("azimuth_to_telescope")
    focus = object_model.get_articulation("telescope_to_focus_knob")

    ctx.check(
        "three user-facing revolute axes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (azimuth, altitude, focus)
        ),
        details="Alt-azimuth head and fine-focus knob should all be revolute joints.",
    )
    ctx.allow_overlap(
        azimuth_head,
        telescope,
        elem_a="yoke_cheek_0",
        elem_b="trunnion_0",
        reason="The altitude trunnion is intentionally captured through the cast-iron yoke cheek.",
    )
    ctx.allow_overlap(
        azimuth_head,
        telescope,
        elem_a="yoke_cheek_1",
        elem_b="trunnion_1",
        reason="The opposite altitude trunnion is intentionally captured through the cast-iron yoke cheek.",
    )
    ctx.expect_contact(
        focus_knob,
        telescope,
        elem_a="shaft",
        elem_b="focuser_block",
        contact_tol=0.001,
        name="focus shaft seats in focuser block",
    )
    ctx.expect_overlap(
        azimuth_head,
        telescope,
        axes="yz",
        elem_a="yoke_cheek_0",
        elem_b="trunnion_0",
        min_overlap=0.015,
        name="altitude trunnion aligns with yoke cheek",
    )
    ctx.expect_overlap(
        azimuth_head,
        telescope,
        axes="yz",
        elem_a="yoke_cheek_1",
        elem_b="trunnion_1",
        min_overlap=0.015,
        name="opposite trunnion aligns with yoke cheek",
    )

    objective_aabb = ctx.part_element_world_aabb(telescope, elem="objective_lens")
    objective_z = (objective_aabb[0][2] + objective_aabb[1][2]) / 2.0 if objective_aabb else None
    with ctx.pose({altitude: 1.0}):
        raised_aabb = ctx.part_element_world_aabb(telescope, elem="objective_lens")
        raised_z = (raised_aabb[0][2] + raised_aabb[1][2]) / 2.0 if raised_aabb else None
    ctx.check(
        "altitude joint raises objective end",
        objective_z is not None and raised_z is not None and raised_z > objective_z + 0.35,
        details=f"objective_z={objective_z}, raised_z={raised_z}",
    )

    with ctx.pose({azimuth: math.pi / 2}):
        swung_aabb = ctx.part_element_world_aabb(telescope, elem="objective_lens")
        swung_y = (swung_aabb[0][1] + swung_aabb[1][1]) / 2.0 if swung_aabb else None
    ctx.check(
        "azimuth joint swings tube around vertical axis",
        swung_y is not None and swung_y > 0.75,
        details=f"swung objective center y={swung_y}",
    )

    return ctx.report()


object_model = build_object_model()
