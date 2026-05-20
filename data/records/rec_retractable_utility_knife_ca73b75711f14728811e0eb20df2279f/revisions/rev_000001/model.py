from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.170
BODY_WIDTH = 0.038
BODY_HEIGHT = 0.026
BLADE_TRAVEL = 0.050
DETENTS = (0.0, 0.0125, 0.0250, 0.0375, 0.0500)


def _housing_shape() -> cq.Workplane:
    """One connected hollow shell: chunky handle with a tapered nose and open blade channel."""

    half_w = BODY_WIDTH / 2.0
    half_h = BODY_HEIGHT / 2.0
    rear_x = -BODY_LENGTH / 2.0
    shoulder_x = 0.045
    nose_x = BODY_LENGTH / 2.0
    nose_half_w = 0.0075

    top_profile = [
        (rear_x, -half_w),
        (shoulder_x, -half_w),
        (nose_x, -nose_half_w),
        (nose_x, nose_half_w),
        (shoulder_x, half_w),
        (rear_x, half_w),
    ]
    body = cq.Workplane("XY").polyline(top_profile).close().extrude(BODY_HEIGHT)
    body = body.translate((0.0, 0.0, -half_h))

    # A straight, clear blade/carrier channel through the nose.  It is visibly
    # open at the tapered tip and leaves continuous bottom and side shell walls.
    channel = cq.Workplane("XY").box(0.190, 0.011, 0.016).translate((0.004, 0.0, 0.0))
    body = body.cut(channel)

    # Thumb-slider slot in the top roof.  It intersects the internal channel
    # but stops before the nose so the front bridge remains structurally legible.
    slot = cq.Workplane("XY").box(0.095, 0.010, 0.032).translate((-0.020, 0.0, 0.012))
    body = body.cut(slot)

    # Small edge softening keeps the shell from reading as a raw block; if a
    # kernel corner is too small to fillet, fall back to the crisp cut shell.
    try:
        body = body.edges("|Z").fillet(0.0015)
    except Exception:
        pass
    return body


def _blade_shape() -> cq.Workplane:
    """Thin snap-off blade side profile, extruded through its steel thickness."""

    # Workplane XZ gives an x-by-z side outline, extruded symmetrically in Y.
    profile = [
        (-0.014, -0.0060),
        (0.074, -0.0060),
        (0.090, 0.0000),
        (0.078, 0.0060),
        (-0.014, 0.0060),
    ]
    blade = cq.Workplane("XZ").polyline(profile).close().extrude(0.0022, both=True)

    # Raised diagonal score ribs on one face.  They are fused into the blade so
    # the blade remains one connected solid while reading as snap-off segments.
    for x in (0.010, 0.022, 0.034, 0.046, 0.058, 0.070):
        rib = cq.Workplane("XY").box(0.017, 0.00045, 0.00075)
        rib = rib.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -33.0)
        rib = rib.translate((x, -0.00112, 0.0))
        blade = blade.union(rib)
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chunky_snap_off_utility_knife")

    model.material("yellow_plastic", rgba=(0.96, 0.64, 0.08, 1.0))
    model.material("black_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    model.material("dark_plastic", rgba=(0.06, 0.06, 0.055, 1.0))
    model.material("brushed_steel", rgba=(0.76, 0.78, 0.76, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell", tolerance=0.0006),
        material="yellow_plastic",
        name="housing_shell",
    )

    # Flush rubber side grips, slightly seated into the plastic side faces.
    for y, name in ((BODY_WIDTH / 2.0 + 0.0002, "side_grip_0"), (-(BODY_WIDTH / 2.0 + 0.0002), "side_grip_1")):
        housing.visual(
            Box((0.074, 0.0012, 0.014)),
            origin=Origin(xyz=(-0.026, y, -0.001)),
            material="black_rubber",
            name=name,
        )

    # Ratchet detents alongside the top slot, matching the carrier's five modeled
    # extension stations.
    for i, q in enumerate(DETENTS):
        housing.visual(
            Box((0.0032, 0.0030, 0.0015)),
            origin=Origin(xyz=(-0.045 + q, 0.0082, BODY_HEIGHT / 2.0 + 0.00025)),
            material="dark_plastic",
            name=f"detent_{i}",
        )

    # Dark throat-plate lips around the exit, leaving a real open slot for the blade.
    housing.visual(
        Box((0.006, 0.017, 0.003)),
        origin=Origin(xyz=(0.079, 0.0, 0.0095)),
        material="dark_plastic",
        name="nose_lip_top",
    )
    housing.visual(
        Box((0.006, 0.017, 0.003)),
        origin=Origin(xyz=(0.079, 0.0, -0.0105)),
        material="dark_plastic",
        name="nose_lip_bottom",
    )
    housing.visual(
        Box((0.006, 0.003, 0.014)),
        origin=Origin(xyz=(0.079, 0.0060, -0.0010)),
        material="dark_plastic",
        name="nose_lip_0",
    )
    housing.visual(
        Box((0.006, 0.003, 0.014)),
        origin=Origin(xyz=(0.079, -0.0060, -0.0010)),
        material="dark_plastic",
        name="nose_lip_1",
    )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Box((0.084, 0.0082, 0.0052)),
        origin=Origin(xyz=(-0.041, 0.0, -0.0012)),
        material="dark_plastic",
        name="carrier_rail",
    )

    # The top thumb piece is part of the moving carrier assembly: a stem passes
    # through the housing slot and ties it to the internal rail.
    carrier.visual(
        Box((0.010, 0.0052, 0.0186)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0062)),
        material="dark_plastic",
        name="slider_stem",
    )
    carrier.visual(
        Box((0.030, 0.018, 0.0075)),
        origin=Origin(xyz=(-0.045, 0.0, 0.019)),
        material="black_rubber",
        name="thumb_slider",
    )
    for i, dx in enumerate((-0.009, -0.0045, 0.0, 0.0045, 0.009)):
        carrier.visual(
            Box((0.0018, 0.016, 0.0014)),
            origin=Origin(xyz=(-0.045 + dx, 0.0, 0.0234)),
            material="dark_plastic",
            name=f"slider_rib_{i}",
        )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "segmented_blade", tolerance=0.00035),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material="brushed_steel",
        name="segmented_blade",
    )

    model.articulation(
        "housing_to_carrier",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=carrier,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=BLADE_TRAVEL),
        meta={"detent_positions": DETENTS},
    )
    model.articulation(
        "carrier_to_blade",
        ArticulationType.FIXED,
        parent=carrier,
        child=blade,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    carrier = object_model.get_part("blade_carrier")
    blade = object_model.get_part("blade")
    slide = object_model.get_articulation("housing_to_carrier")

    ctx.allow_overlap(
        carrier,
        blade,
        elem_a="carrier_rail",
        elem_b="segmented_blade",
        reason="The rear blade tang is intentionally seated under the carrier clamp.",
    )
    ctx.expect_overlap(
        carrier,
        blade,
        axes="x",
        elem_a="carrier_rail",
        elem_b="segmented_blade",
        min_overlap=0.010,
        name="blade tang is captured by carrier clamp",
    )
    ctx.expect_within(
        blade,
        carrier,
        axes="y",
        inner_elem="segmented_blade",
        outer_elem="carrier_rail",
        margin=0.0,
        name="blade tang sits inside carrier width",
    )

    ctx.check(
        "carrier travel is 50 mm",
        slide.motion_limits is not None and abs(slide.motion_limits.upper - BLADE_TRAVEL) < 1e-6,
        details=f"limits={slide.motion_limits}",
    )
    ctx.check(
        "five detent positions are modeled",
        tuple(slide.meta.get("detent_positions", ())) == DETENTS,
        details=f"detents={slide.meta.get('detent_positions')}",
    )

    # The rail remains in the straight housing channel at both retracted and
    # extended stops, while the child origin actually advances along +X.
    ctx.expect_within(
        carrier,
        housing,
        axes="yz",
        inner_elem="carrier_rail",
        outer_elem="housing_shell",
        margin=0.001,
        name="carrier rail stays centered in channel",
    )
    ctx.expect_overlap(
        carrier,
        housing,
        axes="x",
        elem_a="carrier_rail",
        elem_b="housing_shell",
        min_overlap=0.070,
        name="retracted carrier is retained in housing",
    )

    rest_pos = ctx.part_world_position(carrier)
    with ctx.pose({slide: BLADE_TRAVEL}):
        ctx.expect_within(
            carrier,
            housing,
            axes="yz",
            inner_elem="carrier_rail",
            outer_elem="housing_shell",
            margin=0.001,
            name="extended carrier remains in same channel",
        )
        ctx.expect_overlap(
            carrier,
            housing,
            axes="x",
            elem_a="carrier_rail",
            elem_b="housing_shell",
            min_overlap=0.050,
            name="extended carrier remains retained",
        )
        extended_pos = ctx.part_world_position(carrier)

    ctx.check(
        "thumb slider drives blade outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + BLADE_TRAVEL - 0.001,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
