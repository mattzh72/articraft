from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.126
BODY_HEIGHT = 0.092
BODY_DEPTH = 0.020
BODY_FRONT_Z = 0.024
DIAL_JOINT_Z = 0.0245
DIAL_OUTER_RADIUS = 0.036
DIAL_INNER_RADIUS = 0.009
DIAL_HEIGHT = 0.0165
SHAFT_RADIUS = 0.0065
RETAINER_RADIUS = 0.0125


def _dial_ring_geometry() -> MeshGeometry:
    """Low-cost fluted annular dial with a real center bore."""
    segments = 96
    ribs = 24
    geom = MeshGeometry()

    # A closed radial cross-section loop.  The outside loops are angle-varying
    # to mold shallow finger ribs without adding separate parts.
    section = (
        ("inner_bottom", DIAL_INNER_RADIUS, 0.0010, 0.0),
        ("outer_bottom", DIAL_OUTER_RADIUS, 0.0010, 0.0010),
        ("outer_grip", DIAL_OUTER_RADIUS + 0.0007, 0.0105, 0.0013),
        ("outer_crown", DIAL_OUTER_RADIUS - 0.0022, DIAL_HEIGHT, 0.0005),
        ("inner_crown", DIAL_INNER_RADIUS + 0.0015, DIAL_HEIGHT, 0.0),
    )

    loops: list[list[int]] = []
    for _name, base_radius, z, rib_amp in section:
        loop: list[int] = []
        for i in range(segments):
            theta = 2.0 * math.pi * i / segments
            # Clamp the cosine so each rib has a flat valley and a soft crest,
            # a draft-friendly texture rather than many glued-on islands.
            rib = max(0.0, math.cos(ribs * theta))
            radius = base_radius + rib_amp * rib
            loop.append(geom.add_vertex(radius * math.cos(theta), radius * math.sin(theta), z))
        loops.append(loop)

    for li in range(len(loops)):
        a_loop = loops[li]
        b_loop = loops[(li + 1) % len(loops)]
        for i in range(segments):
            a0 = a_loop[i]
            a1 = a_loop[(i + 1) % segments]
            b0 = b_loop[i]
            b1 = b_loop[(i + 1) % segments]
            geom.add_face(a0, b0, b1)
            geom.add_face(a0, b1, a1)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_wall_thermostat")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    off_white = model.material("slightly_glossy_face", rgba=(0.95, 0.94, 0.88, 1.0))
    dark_lcd = model.material("dark_lcd_window", rgba=(0.05, 0.07, 0.07, 1.0))
    printed_gray = model.material("printed_gray", rgba=(0.18, 0.18, 0.16, 1.0))
    dial_plastic = model.material("charcoal_dial_plastic", rgba=(0.18, 0.18, 0.17, 1.0))
    pointer_white = model.material("white_pointer_print", rgba=(0.97, 0.97, 0.93, 1.0))
    zinc = model.material("zinc_plated_retainer", rgba=(0.72, 0.72, 0.68, 1.0))

    body = model.part("body")

    # One molded backplate and one shallow rounded cover: a two-shot-looking
    # body, but authored as one fixed link to reflect a low part-count assembly.
    body.visual(
        Box((0.142, 0.108, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=warm_white,
        name="wall_plate",
    )
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(BODY_WIDTH, BODY_HEIGHT, 0.014, corner_segments=10),
                BODY_DEPTH,
            ),
            "rounded_front_cover",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=off_white,
        name="front_cover",
    )

    # Molded latch shelves and visible screw pads make the intended assembly
    # order obvious: screw the plate down, then snap the cover/body on.
    for y, name in ((0.048, "top_snap_lip"), (-0.048, "bottom_snap_lip")):
        body.visual(
            Box((0.050, 0.004, 0.005)),
            origin=Origin(xyz=(0.0, y, 0.0065)),
            material=warm_white,
            name=name,
        )
    for x, name in ((-0.058, "mount_screw_pad_0"), (0.058, "mount_screw_pad_1")):
        body.visual(
            Cylinder(radius=0.008, length=0.002),
            origin=Origin(xyz=(x, 0.0, 0.0063)),
            material=zinc,
            name=name,
        )

    body.visual(
        Box((0.044, 0.014, 0.0010)),
        origin=Origin(xyz=(0.0, 0.028, BODY_FRONT_Z + 0.00010)),
        material=dark_lcd,
        name="display_window",
    )

    # Printed temperature ticks are surface ink, not extra assembled pieces.
    for i, angle_deg in enumerate(range(-120, 121, 24)):
        angle = math.radians(angle_deg)
        radius = 0.047
        major = i % 2 == 0
        body.visual(
            Box((0.010 if major else 0.006, 0.0012, 0.0007)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), BODY_FRONT_Z + 0.0002),
                rpy=(0.0, 0.0, angle),
            ),
            material=printed_gray,
            name=f"temp_tick_{i}",
        )

    # The center post and washer are fixed to the body.  They pass through the
    # dial bore and retain the dial from the front without needing a third link.
    body.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, BODY_FRONT_Z + 0.010)),
        material=zinc,
        name="shaft",
    )
    body.visual(
        Cylinder(radius=RETAINER_RADIUS, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, DIAL_JOINT_Z + DIAL_HEIGHT + 0.0010)),
        material=zinc,
        name="retainer_cap",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, DIAL_JOINT_Z + DIAL_HEIGHT + 0.00295)),
        material=printed_gray,
        name="retainer_screw_head",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(_dial_ring_geometry(), "fluted_dial_ring"),
        origin=Origin(),
        material=dial_plastic,
        name="dial_ring",
    )
    dial.visual(
        Box((0.020, 0.0022, 0.0010)),
        origin=Origin(xyz=(0.024, 0.0, DIAL_HEIGHT + 0.00045)),
        material=pointer_white,
        name="pointer",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, DIAL_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-2.1, upper=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_origin_distance(
        body,
        dial,
        axes="xy",
        max_dist=0.0005,
        name="dial axis is concentric with body shaft",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        positive_elem="dial_ring",
        negative_elem="front_cover",
        min_gap=0.0008,
        max_gap=0.0025,
        name="dial clears the front cover",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="z",
        elem_a="dial_ring",
        elem_b="shaft",
        min_overlap=0.012,
        name="shaft passes through the dial bore span",
    )
    ctx.expect_gap(
        body,
        dial,
        axis="z",
        positive_elem="retainer_cap",
        negative_elem="dial_ring",
        min_gap=0.0,
        max_gap=0.0010,
        name="retainer cap captures dial with a small running gap",
    )

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[1] + hi[1])

    rest_y = _aabb_center_y(ctx.part_element_world_aabb(dial, elem="pointer"))
    with ctx.pose({dial_joint: 1.0}):
        turned_y = _aabb_center_y(ctx.part_element_world_aabb(dial, elem="pointer"))

    ctx.check(
        "dial pointer rotates about the center shaft",
        rest_y is not None and turned_y is not None and turned_y > rest_y + 0.010,
        details=f"rest pointer y={rest_y}, turned pointer y={turned_y}",
    )

    return ctx.report()


object_model = build_object_model()
