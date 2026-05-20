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


def _axisymmetric_solid(profile: list[tuple[float, float]], *, segments: int = 96) -> MeshGeometry:
    """Closed stepped body around the local X axis.  Profile entries are (x, radius)."""
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for x, radius in profile:
        ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            ring.append(geom.add_vertex(x, radius * math.cos(theta), radius * math.sin(theta)))
        rings.append(ring)

    for a, b in zip(rings[:-1], rings[1:]):
        for i in range(segments):
            j = (i + 1) % segments
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])

    # Cap the rear and front so the alloy barrel reads as a manufactured shell/skin.
    rear_center = geom.add_vertex(profile[0][0], 0.0, 0.0)
    front_center = geom.add_vertex(profile[-1][0], 0.0, 0.0)
    rear = rings[0]
    front = rings[-1]
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(rear_center, rear[j], rear[i])
        geom.add_face(front_center, front[i], front[j])
    return geom


def _hollow_ring(
    *,
    length: float,
    inner_radius: float,
    outer_radius: float,
    segments: int = 96,
    rib_count: int = 0,
    rib_depth: float = 0.0,
) -> MeshGeometry:
    """Closed annular ring around local X, optionally with axial rubber ribs."""
    geom = MeshGeometry()
    xs = (-length / 2.0, length / 2.0)
    outer: list[list[int]] = []
    inner: list[list[int]] = []

    for x in xs:
        outer_ring: list[int] = []
        inner_ring: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            if rib_count:
                # Rounded high ribs with shallow valleys; the periodic profile remains one mesh.
                rib_phase = 0.5 + 0.5 * math.cos(rib_count * theta)
                radius = outer_radius + rib_depth * (rib_phase**3)
            else:
                radius = outer_radius
            c = math.cos(theta)
            s = math.sin(theta)
            outer_ring.append(geom.add_vertex(x, radius * c, radius * s))
            inner_ring.append(geom.add_vertex(x, inner_radius * c, inner_radius * s))
        outer.append(outer_ring)
        inner.append(inner_ring)

    # Outer and inner cylindrical surfaces.
    for i in range(segments):
        j = (i + 1) % segments
        geom.add_face(outer[0][i], outer[1][i], outer[1][j])
        geom.add_face(outer[0][i], outer[1][j], outer[0][j])
        geom.add_face(inner[0][j], inner[1][j], inner[1][i])
        geom.add_face(inner[0][j], inner[1][i], inner[0][i])

    # Annular end faces.
    for side in range(2):
        for i in range(segments):
            j = (i + 1) % segments
            if side == 0:
                geom.add_face(inner[side][i], outer[side][i], outer[side][j])
                geom.add_face(inner[side][i], outer[side][j], inner[side][j])
            else:
                geom.add_face(inner[side][j], outer[side][j], outer[side][i])
                geom.add_face(inner[side][j], outer[side][i], inner[side][i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="super_telephoto_prime_lens")

    alloy = model.material("warm_white_alloy", rgba=(0.86, 0.84, 0.78, 1.0))
    satin_black = model.material("satin_black", rgba=(0.01, 0.011, 0.012, 1.0))
    rubber = model.material("ribbed_black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    brushed_steel = model.material("brushed_bayonet_steel", rgba=(0.62, 0.63, 0.62, 1.0))
    glass = model.material("coated_front_glass", rgba=(0.18, 0.32, 0.42, 0.55))
    white_paint = model.material("engraved_white_paint", rgba=(1.0, 1.0, 0.92, 1.0))

    barrel = model.part("barrel")
    barrel_profile = [
        (-0.430, 0.045),
        (-0.402, 0.052),
        (-0.360, 0.058),
        (-0.285, 0.064),
        (-0.120, 0.070),
        (-0.060, 0.072),
        (0.070, 0.072),
        (0.100, 0.067),
        (0.155, 0.069),
        (0.180, 0.076),
        (0.310, 0.076),
        (0.335, 0.070),
        (0.385, 0.085),
        (0.455, 0.092),
    ]
    barrel.visual(
        mesh_from_geometry(_axisymmetric_solid(barrel_profile), "barrel_shell"),
        material=alloy,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.087, length=0.022),
        origin=Origin(xyz=(0.461, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_inner_rim",
    )
    barrel.visual(
        Cylinder(radius=0.073, length=0.007),
        origin=Origin(xyz=(0.474, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_element",
    )
    barrel.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(-0.432, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="rear_mount_plate",
    )
    for idx, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial = 0.047
        barrel.visual(
            Box((0.011, 0.031, 0.010)),
            origin=Origin(
                xyz=(-0.452, -radial * math.sin(theta), radial * math.cos(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=brushed_steel,
            name=f"bayonet_lug_{idx}",
        )
    barrel.visual(
        Box((0.006, 0.045, 0.014)),
        origin=Origin(xyz=(-0.407, 0.0, 0.051)),
        material=dark_metal,
        name="rear_weather_gasket",
    )

    collar = model.part("collar")
    collar.visual(
        mesh_from_geometry(
            _hollow_ring(length=0.082, inner_radius=0.075, outer_radius=0.089, segments=96),
            "collar_ring",
        ),
        material=dark_metal,
        name="collar_ring",
    )
    for idx, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        pad_radius = 0.0735
        collar.visual(
            Box((0.016, 0.008, 0.003)),
            origin=Origin(
                xyz=(0.0, -pad_radius * math.sin(theta), pad_radius * math.cos(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"collar_bearing_pad_{idx}",
        )
    collar.visual(
        Box((0.066, 0.044, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.109)),
        material=dark_metal,
        name="foot_stem",
    )
    collar.visual(
        Box((0.165, 0.072, 0.023)),
        origin=Origin(xyz=(0.006, 0.0, -0.139)),
        material=dark_metal,
        name="foot_plate",
    )
    collar.visual(
        Box((0.142, 0.046, 0.010)),
        origin=Origin(xyz=(0.006, 0.0, -0.153)),
        material=satin_black,
        name="dovetail_sole",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(
            _hollow_ring(
                length=0.132,
                inner_radius=0.081,
                outer_radius=0.090,
                segments=144,
                rib_count=36,
                rib_depth=0.006,
            ),
            "focus_shell",
        ),
        material=rubber,
        name="focus_shell",
    )
    for idx, theta in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        pad_radius = 0.0785
        focus_ring.visual(
            Box((0.020, 0.007, 0.005)),
            origin=Origin(
                xyz=(0.0, -pad_radius * math.sin(theta), pad_radius * math.cos(theta)),
                rpy=(theta, 0.0, 0.0),
            ),
            material=rubber,
            name=f"focus_bearing_pad_{idx}",
        )
    focus_ring.visual(
        Box((0.040, 0.005, 0.003)),
        origin=Origin(xyz=(0.000, 0.0, 0.096)),
        material=white_paint,
        name="focus_mark",
    )

    model.articulation(
        "barrel_to_collar",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=collar,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "barrel_to_focus",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.8, lower=-1.75, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    collar = object_model.get_part("collar")
    focus_ring = object_model.get_part("focus_ring")
    collar_joint = object_model.get_articulation("barrel_to_collar")
    focus_joint = object_model.get_articulation("barrel_to_focus")

    ctx.check(
        "long alloy barrel is the single root",
        len(object_model.root_parts()) == 1 and object_model.root_parts()[0].name == "barrel",
    )
    ctx.check(
        "tripod collar uses continuous rotation",
        collar_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(collar_joint.axis) == (1.0, 0.0, 0.0),
    )
    ctx.check(
        "focus ring has a limited revolute throw",
        focus_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(focus_joint.axis) == (1.0, 0.0, 0.0)
        and focus_joint.motion_limits is not None
        and focus_joint.motion_limits.lower < 0.0
        and focus_joint.motion_limits.upper > 0.0,
    )
    ctx.expect_overlap(
        collar,
        barrel,
        axes="x",
        elem_a="collar_ring",
        elem_b="barrel_shell",
        min_overlap=0.075,
        name="collar ring is seated around the barrel mid-section",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="x",
        elem_a="focus_shell",
        elem_b="barrel_shell",
        min_overlap=0.120,
        name="focus ring surrounds the front barrel group",
    )
    for idx in range(3):
        pad_name = f"collar_bearing_pad_{idx}"
        ctx.allow_overlap(
            barrel,
            collar,
            elem_a="barrel_shell",
            elem_b=pad_name,
            reason=(
                "Hidden polymer bearing pads are modeled with a tiny preload into the alloy barrel "
                "so the rotating tripod collar is mechanically supported instead of floating."
            ),
        )
        ctx.expect_contact(
            barrel,
            collar,
            elem_a="barrel_shell",
            elem_b=pad_name,
            contact_tol=0.001,
            name=f"{pad_name} bears on the barrel",
        )
        ctx.expect_overlap(
            collar,
            barrel,
            axes="x",
            elem_a=pad_name,
            elem_b="barrel_shell",
            min_overlap=0.010,
            name=f"{pad_name} has axial bearing engagement",
        )
    for idx in range(4):
        ctx.expect_contact(
            barrel,
            focus_ring,
            elem_a="barrel_shell",
            elem_b=f"focus_bearing_pad_{idx}",
            contact_tol=0.001,
            name=f"focus_bearing_pad_{idx} supports the focus ring",
        )

    rest_foot = ctx.part_element_world_aabb(collar, elem="foot_plate")
    with ctx.pose({collar_joint: math.pi / 2.0}):
        rotated_foot = ctx.part_element_world_aabb(collar, elem="foot_plate")
    if rest_foot is not None and rotated_foot is not None:
        rest_center = tuple((rest_foot[0][i] + rest_foot[1][i]) / 2.0 for i in range(3))
        rotated_center = tuple((rotated_foot[0][i] + rotated_foot[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "collar foot sweeps around the lens axis",
            rotated_center[1] > rest_center[1] + 0.10 and rotated_center[2] > rest_center[2] + 0.10,
            details=f"rest={rest_center}, rotated={rotated_center}",
        )
    else:
        ctx.fail("collar foot sweep can be measured", "foot_plate AABB was unavailable")

    rest_mark = ctx.part_element_world_aabb(focus_ring, elem="focus_mark")
    with ctx.pose({focus_joint: 1.20}):
        turned_mark = ctx.part_element_world_aabb(focus_ring, elem="focus_mark")
    if rest_mark is not None and turned_mark is not None:
        rest_center = tuple((rest_mark[0][i] + rest_mark[1][i]) / 2.0 for i in range(3))
        turned_center = tuple((turned_mark[0][i] + turned_mark[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "focus index mark visibly rotates with the ring",
            abs(turned_center[1] - rest_center[1]) > 0.060 and turned_center[2] < rest_center[2] - 0.030,
            details=f"rest={rest_center}, turned={turned_center}",
        )
    else:
        ctx.fail("focus mark rotation can be measured", "focus_mark AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
