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


def _revolved_profile(points: list[tuple[float, float]]) -> MeshGeometry:
    """Lathe a closed radius/Z boundary into a watertight mesh."""
    segments = 72
    geom = MeshGeometry()
    rings: list[list[int]] = []
    for segment in range(segments):
        angle = 2.0 * math.pi * segment / segments
        c = math.cos(angle)
        s = math.sin(angle)
        rings.append([geom.add_vertex(radius * c, radius * s, z) for radius, z in points])

    for segment in range(segments):
        next_segment = (segment + 1) % segments
        for index in range(len(points)):
            next_index = (index + 1) % len(points)
            a = rings[segment][index]
            b = rings[next_segment][index]
            c = rings[next_segment][next_index]
            d = rings[segment][next_index]
            geom.add_face(a, b, c)
            geom.add_face(a, c, d)
    return geom


def _bottle_shell() -> MeshGeometry:
    # A hollow, open-neck bottle wall.  The profile starts at the bottom axis,
    # follows the rounded outer shoulder and neck, then returns down the inner
    # wall so the mesh reads as a real plastic shell rather than a solid plug.
    wall_profile = [
        (0.000, 0.000),
        (0.039, 0.000),
        (0.043, 0.006),
        (0.043, 0.122),
        (0.041, 0.136),
        (0.035, 0.149),
        (0.026, 0.161),
        (0.017, 0.174),
        (0.017, 0.211),
        (0.020, 0.213),
        (0.020, 0.216),
        (0.011, 0.216),
        (0.011, 0.178),
        (0.019, 0.166),
        (0.029, 0.151),
        (0.036, 0.128),
        (0.036, 0.010),
        (0.000, 0.010),
    ]
    shell = _revolved_profile(wall_profile)

    # Three shallow raised thread crests around the translucent neck.
    for z in (0.184, 0.195, 0.206):
        thread = _revolved_profile(
            [
                (0.0164, z - 0.0028),
                (0.0205, z),
                (0.0164, z + 0.0028),
            ]
        )
        shell.merge(thread)

    return shell


def _collar_shell() -> MeshGeometry:
    # Screw-on pump collar with a central clearance bore for the moving stem.
    base = _revolved_profile(
        [
            (0.009001, 0.000),
            (0.026, 0.000),
            (0.026, 0.017),
            (0.023, 0.023),
            (0.017, 0.026),
            (0.009001, 0.026),
        ]
    )
    guide_bushing = _revolved_profile(
        [
            (0.009001, 0.026),
            (0.016, 0.026),
            (0.016, 0.036),
            (0.009001, 0.036),
        ]
    )
    base.merge(guide_bushing)
    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soap_dispenser_pump_bottle")

    bottle_mat = model.material("translucent_aqua_plastic", rgba=(0.42, 0.86, 0.92, 0.38))
    soap_mat = model.material("pale_blue_soap", rgba=(0.55, 0.82, 0.95, 0.55))
    label_mat = model.material("cream_label", rgba=(0.96, 0.93, 0.82, 1.0))
    label_ink = model.material("blue_label_ink", rgba=(0.12, 0.36, 0.62, 1.0))
    white_plastic = model.material("white_plastic", rgba=(0.94, 0.94, 0.90, 1.0))
    shadow = model.material("dark_outlet", rgba=(0.03, 0.03, 0.035, 1.0))
    stem_mat = model.material("satin_stem", rgba=(0.78, 0.80, 0.78, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_bottle_shell(), "hollow_threaded_bottle"),
        material=bottle_mat,
        name="bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.034, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=soap_mat,
        name="soap_fill",
    )
    bottle.visual(
        Box((0.0015, 0.041, 0.055)),
        origin=Origin(xyz=(0.0432, 0.0, 0.078)),
        material=label_mat,
        name="front_label",
    )
    bottle.visual(
        Box((0.0018, 0.030, 0.006)),
        origin=Origin(xyz=(0.0441, 0.0, 0.091)),
        material=label_ink,
        name="label_stripe",
    )
    bottle.visual(
        Box((0.0018, 0.023, 0.004)),
        origin=Origin(xyz=(0.0442, 0.0, 0.070)),
        material=label_ink,
        name="label_text_bar",
    )

    collar = model.part("pump_collar")
    collar.visual(
        mesh_from_geometry(_collar_shell(), "pump_collar_ring"),
        material=white_plastic,
        name="collar_shell",
    )
    for i in range(20):
        theta = 2.0 * math.pi * i / 20.0
        collar.visual(
            Box((0.0030, 0.0042, 0.018)),
            origin=Origin(
                xyz=(0.0270 * math.cos(theta), 0.0270 * math.sin(theta), 0.011),
                rpy=(0.0, 0.0, theta),
            ),
            material=white_plastic,
            name=f"collar_rib_{i}",
        )

    plunger = model.part("plunger_stem")
    plunger.visual(
        Cylinder(radius=0.008992, length=0.098),
        # Local span is z=-0.035 to +0.063: the hidden lower length stays
        # retained in the neck/collar bore through the full pump stroke.
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=stem_mat,
        name="stem",
    )

    spout = model.part("spout_head")
    spout.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=white_plastic,
        name="hub",
    )
    spout.visual(
        Cylinder(radius=0.007, length=0.064),
        origin=Origin(xyz=(0.040, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_plastic,
        name="spout_tube",
    )
    spout.visual(
        Cylinder(radius=0.0064, length=0.017),
        origin=Origin(xyz=(0.071, 0.0, 0.0015)),
        material=white_plastic,
        name="downturned_tip",
    )
    spout.visual(
        Cylinder(radius=0.0040, length=0.0012),
        origin=Origin(xyz=(0.071, 0.0, -0.0076)),
        material=shadow,
        name="outlet_shadow",
    )

    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.216)),
    )
    model.articulation(
        "collar_to_plunger",
        ArticulationType.PRISMATIC,
        parent=collar,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.025),
    )
    model.articulation(
        "plunger_to_spout",
        ArticulationType.CONTINUOUS,
        parent=plunger,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("pump_collar")
    plunger = object_model.get_part("plunger_stem")
    spout = object_model.get_part("spout_head")
    slide = object_model.get_articulation("collar_to_plunger")
    swivel = object_model.get_articulation("plunger_to_spout")

    ctx.expect_contact(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        contact_tol=0.0015,
        name="pump collar seats on threaded neck",
    )
    ctx.expect_within(
        plunger,
        collar,
        axes="xy",
        inner_elem="stem",
        outer_elem="collar_shell",
        margin=0.0,
        name="plunger stem is centered through collar bore",
    )
    ctx.expect_gap(
        spout,
        collar,
        axis="z",
        min_gap=0.030,
        name="raised spout head clears the collar",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.025}):
        lowered_pos = ctx.part_world_position(plunger)
        ctx.expect_gap(
            spout,
            collar,
            axis="z",
            min_gap=0.015,
            name="depressed pump still leaves finger clearance",
        )

    ctx.check(
        "plunger moves downward over a short pump stroke",
        rest_pos is not None
        and lowered_pos is not None
        and lowered_pos[2] < rest_pos[2] - 0.020,
        details=f"rest={rest_pos}, lowered={lowered_pos}",
    )

    def _coord(vec, index: int) -> float:
        try:
            return vec[index]
        except TypeError:
            return (vec.x, vec.y, vec.z)[index]

    def _span(aabb, index: int) -> float:
        return _coord(aabb[1], index) - _coord(aabb[0], index)

    front_aabb = ctx.part_element_world_aabb(spout, elem="spout_tube")
    with ctx.pose({swivel: math.pi / 2.0}):
        side_aabb = ctx.part_element_world_aabb(spout, elem="spout_tube")

    ctx.check(
        "spout head rotates continuously about the vertical stem",
        front_aabb is not None
        and side_aabb is not None
        and _span(front_aabb, 0) > 0.050
        and _span(front_aabb, 1) < 0.018
        and _span(side_aabb, 1) > 0.050
        and _span(side_aabb, 0) < 0.018,
        details=f"front_aabb={front_aabb}, side_aabb={side_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
