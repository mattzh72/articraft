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


def _quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _profiled_annular_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    radial_segments: int = 96,
) -> MeshGeometry:
    """Build a hollow rotational shell whose axis is the local +X direction."""
    mesh = MeshGeometry()
    outer: list[list[int]] = []
    inner: list[list[int]] = []

    for x, radius in outer_profile:
        ring = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            ring.append(mesh.add_vertex(x, radius * math.cos(theta), radius * math.sin(theta)))
        outer.append(ring)

    for x, radius in inner_profile:
        ring = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            ring.append(mesh.add_vertex(x, radius * math.cos(theta), radius * math.sin(theta)))
        inner.append(ring)

    for rings in (outer, inner):
        for s in range(len(rings) - 1):
            for i in range(radial_segments):
                j = (i + 1) % radial_segments
                if rings is outer:
                    _quad(mesh, rings[s][i], rings[s + 1][i], rings[s + 1][j], rings[s][j])
                else:
                    _quad(mesh, rings[s][j], rings[s + 1][j], rings[s + 1][i], rings[s][i])

    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        _quad(mesh, inner[0][i], outer[0][i], outer[0][j], inner[0][j])
        _quad(mesh, outer[-1][i], inner[-1][i], inner[-1][j], outer[-1][j])

    return mesh


def _ribbed_focus_ring_mesh(
    *,
    length: float,
    inner_radius: float,
    base_radius: float,
    rib_radius: float,
    rib_count: int = 48,
    radial_segments: int = 192,
) -> MeshGeometry:
    """A hollow rubber ring with lengthwise raised grip ribs."""
    mesh = MeshGeometry()
    x_values = (-length / 2.0, -length * 0.38, length * 0.38, length / 2.0)
    outer: list[list[int]] = []
    inner: list[list[int]] = []

    def radius_at(index: int) -> float:
        phase = (index * rib_count / radial_segments) % 1.0
        return rib_radius if phase < 0.38 else base_radius

    for x in x_values:
        outer_ring = []
        inner_ring = []
        for i in range(radial_segments):
            theta = 2.0 * math.pi * i / radial_segments
            c, s = math.cos(theta), math.sin(theta)
            outer_ring.append(mesh.add_vertex(x, radius_at(i) * c, radius_at(i) * s))
            inner_ring.append(mesh.add_vertex(x, inner_radius * c, inner_radius * s))
        outer.append(outer_ring)
        inner.append(inner_ring)

    for s in range(len(x_values) - 1):
        for i in range(radial_segments):
            j = (i + 1) % radial_segments
            _quad(mesh, outer[s][i], outer[s + 1][i], outer[s + 1][j], outer[s][j])
            _quad(mesh, inner[s][j], inner[s + 1][j], inner[s + 1][i], inner[s][i])

    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        _quad(mesh, inner[0][i], outer[0][i], outer[0][j], inner[0][j])
        _quad(mesh, outer[-1][i], inner[-1][i], inner[-1][j], outer[-1][j])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_lens")

    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.017, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.18, 0.17, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.08, 0.18, 0.26, 0.55))
    red_mark = model.material("red_macro_mark", rgba=(0.75, 0.02, 0.015, 1.0))

    body = model.part("body")
    body_shell = _profiled_annular_mesh(
        [
            (-0.060, 0.0370),
            (-0.056, 0.0400),
            (-0.052, 0.0400),
            (-0.048, 0.0365),
            (-0.034, 0.0365),
            (-0.030, 0.0358),
            (0.030, 0.0358),
            (0.034, 0.0365),
            (0.052, 0.0365),
        ],
        [
            (-0.060, 0.0308),
            (-0.056, 0.0308),
            (-0.052, 0.0308),
            (-0.048, 0.0308),
            (-0.034, 0.0308),
            (-0.030, 0.0308),
            (0.030, 0.0308),
            (0.034, 0.0308),
            (0.052, 0.0308),
        ],
    )
    body.visual(mesh_from_geometry(body_shell, "body_shell"), material=satin_black, name="body_shell")

    accent_band = _profiled_annular_mesh(
        [(-0.0470, 0.0366), (-0.0445, 0.0377)],
        [(-0.0470, 0.0360), (-0.0445, 0.0360)],
        radial_segments=96,
    )
    body.visual(mesh_from_geometry(accent_band, "macro_mark"), material=red_mark, name="macro_mark")

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(
            _ribbed_focus_ring_mesh(
                length=0.060,
                inner_radius=0.0363,
                base_radius=0.0415,
                rib_radius=0.0440,
            ),
            "ribbed_focus_ring",
        ),
        material=rubber,
        name="ribbed_grip",
    )
    for pad_name, z in (("ring_bearing_pad_top", 0.0359), ("ring_bearing_pad_bottom", -0.0359)):
        focus_ring.visual(
            Box((0.010, 0.006, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rubber,
            name=pad_name,
        )

    extension_barrel = model.part("extension_barrel")
    extension_tube = _profiled_annular_mesh(
        [
            (-0.055, 0.0285),
            (-0.050, 0.0285),
            (0.024, 0.0288),
            (0.028, 0.0305),
            (0.035, 0.0305),
        ],
        [
            (-0.055, 0.0205),
            (-0.050, 0.0205),
            (0.024, 0.0205),
            (0.028, 0.0205),
            (0.035, 0.0205),
        ],
    )
    extension_barrel.visual(
        mesh_from_geometry(extension_tube, "extension_tube"),
        material=gunmetal,
        name="extension_tube",
    )
    for pad_name, z in (("barrel_guide_pad_top", 0.0296), ("barrel_guide_pad_bottom", -0.0296)):
        extension_barrel.visual(
            Box((0.012, 0.005, 0.0032)),
            origin=Origin(xyz=(-0.047, 0.0, z)),
            material=gunmetal,
            name=pad_name,
        )
    extension_barrel.visual(
        Cylinder(radius=0.0210, length=0.004),
        origin=Origin(xyz=(0.033, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=coated_glass,
        name="front_glass",
    )

    model.articulation(
        "body_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=focus_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=-2.2, upper=2.2),
    )

    model.articulation(
        "body_to_extension_barrel",
        ArticulationType.PRISMATIC,
        parent=body,
        child=extension_barrel,
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.08, lower=0.0, upper=0.034),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    extension_barrel = object_model.get_part("extension_barrel")
    focus_joint = object_model.get_articulation("body_to_focus_ring")
    extension_joint = object_model.get_articulation("body_to_extension_barrel")

    ctx.check(
        "focus ring uses revolute joint",
        getattr(focus_joint, "articulation_type", None) == ArticulationType.REVOLUTE,
        details=f"joint type is {getattr(focus_joint, 'articulation_type', None)}",
    )
    ctx.check(
        "extension barrel uses prismatic joint",
        getattr(extension_joint, "articulation_type", None) == ArticulationType.PRISMATIC,
        details=f"joint type is {getattr(extension_joint, 'articulation_type', None)}",
    )
    ctx.check(
        "both mechanisms follow the optical axis",
        tuple(round(v, 3) for v in getattr(focus_joint, "axis", ())) == (1.0, 0.0, 0.0)
        and tuple(round(v, 3) for v in getattr(extension_joint, "axis", ())) == (1.0, 0.0, 0.0),
        details=f"focus axis={getattr(focus_joint, 'axis', None)}, extension axis={getattr(extension_joint, 'axis', None)}",
    )

    for pad in ("ring_bearing_pad_top", "ring_bearing_pad_bottom"):
        ctx.allow_overlap(
            body,
            focus_ring,
            elem_a="body_shell",
            elem_b=pad,
            reason="Small hidden bearing pads touch into the focus-ring track so the rotating ring is mechanically supported.",
        )
        ctx.expect_contact(
            focus_ring,
            body,
            elem_a=pad,
            elem_b="body_shell",
            contact_tol=0.001,
            name=f"{pad} is seated on the body track",
        )

    for pad in ("barrel_guide_pad_top", "barrel_guide_pad_bottom"):
        ctx.allow_overlap(
            body,
            extension_barrel,
            elem_a="body_shell",
            elem_b=pad,
            reason="Small hidden guide pads represent the sliding bearing contact inside the body bore.",
        )
        ctx.expect_contact(
            extension_barrel,
            body,
            elem_a=pad,
            elem_b="body_shell",
            contact_tol=0.001,
            name=f"{pad} is seated in the body bore",
        )

    ring_aabb = ctx.part_element_world_aabb(focus_ring, elem="ribbed_grip")
    if ring_aabb is not None:
        ring_length = ring_aabb[1][0] - ring_aabb[0][0]
        ring_diameter = max(ring_aabb[1][1] - ring_aabb[0][1], ring_aabb[1][2] - ring_aabb[0][2])
    else:
        ring_length = ring_diameter = 0.0
    ctx.check(
        "focus ring is wide and proud",
        ring_length >= 0.055 and ring_diameter >= 0.084,
        details=f"ring_length={ring_length:.4f}, ring_diameter={ring_diameter:.4f}",
    )

    ctx.expect_overlap(
        focus_ring,
        body,
        axes="x",
        min_overlap=0.050,
        elem_a="ribbed_grip",
        elem_b="body_shell",
        name="focus ring wraps the lens body",
    )

    ctx.expect_within(
        extension_barrel,
        body,
        axes="yz",
        margin=0.002,
        inner_elem="extension_tube",
        outer_elem="body_shell",
        name="extension barrel is centered in the body bore",
    )
    ctx.expect_overlap(
        extension_barrel,
        body,
        axes="x",
        min_overlap=0.050,
        elem_a="extension_tube",
        elem_b="body_shell",
        name="collapsed barrel retains deep insertion",
    )

    rest_pos = ctx.part_world_position(extension_barrel)
    with ctx.pose({extension_joint: 0.034}):
        ctx.expect_overlap(
            extension_barrel,
            body,
            axes="x",
            min_overlap=0.020,
            elem_a="extension_tube",
            elem_b="body_shell",
            name="extended barrel remains inserted",
        )
        extended_pos = ctx.part_world_position(extension_barrel)

    ctx.check(
        "extension barrel slides forward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.030,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
