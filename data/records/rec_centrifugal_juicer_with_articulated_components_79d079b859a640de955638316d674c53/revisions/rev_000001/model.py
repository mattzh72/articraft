from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    mesh_from_geometry,
    resample_side_sections,
    superellipse_side_loft,
)


def _revolved_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    y_offset: float = 0.0,
    segments: int = 80,
) -> MeshGeometry:
    """Build a connected hollow surface of revolution about local Z.

    Profiles are (radius, z) points.  The result is a single watertight shell
    with annular end caps, translated in Y so it can sit under the rear hinge.
    """

    geom = MeshGeometry()
    profile = list(outer_profile) + list(reversed(inner_profile))
    rings: list[list[int]] = []
    for radius, z in profile:
        ring = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            ring.append(
                geom.add_vertex(
                    radius * math.cos(angle),
                    y_offset + radius * math.sin(angle),
                    z,
                )
            )
        rings.append(ring)

    for j in range(len(rings)):
        a = rings[j]
        b = rings[(j + 1) % len(rings)]
        for i in range(segments):
            i2 = (i + 1) % segments
            geom.add_face(a[i], b[i], b[i2])
            geom.add_face(a[i], b[i2], a[i2])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_centrifugal_juicer")

    matte_white = model.material("warm_white_plastic", rgba=(0.92, 0.90, 0.84, 1.0))
    dark_trim = model.material("charcoal_trim", rgba=(0.04, 0.045, 0.05, 1.0))
    clear_smoke = model.material("clear_smoky_plastic", rgba=(0.68, 0.90, 1.0, 0.38))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    pulp_orange = model.material("warm_orange_mark", rgba=(1.0, 0.55, 0.12, 1.0))

    # Root: a broad rounded motor base with a hollow raised chamber ring.
    base = model.part("base")
    base_sections = resample_side_sections(
        [
            (-0.165, 0.010, 0.090, 0.245),
            (-0.120, 0.000, 0.120, 0.325),
            (0.000, 0.000, 0.135, 0.350),
            (0.120, 0.000, 0.120, 0.325),
            (0.165, 0.010, 0.090, 0.245),
        ],
        samples_per_span=3,
        smooth_passes=1,
    )
    base.visual(
        mesh_from_geometry(
            superellipse_side_loft(base_sections, exponents=3.6, segments=72),
            "rounded_base_housing",
        ),
        material=matte_white,
        name="rounded_base_housing",
    )
    # A black anti-slip plinth keeps the appliance grounded on the counter.
    base.visual(
        Box((0.285, 0.245, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black_rubber,
        name="counter_plinth",
    )
    base.visual(
        Cylinder(radius=0.140, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=matte_white,
        name="chamber_seat",
    )
    chamber_rim = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.132, 0.135), (0.132, 0.248)],
        inner_profile=[(0.113, 0.135), (0.113, 0.248)],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(chamber_rim, "chamber_rim"),
        material=clear_smoke,
        name="chamber_rim",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.143)),
        material=dark_trim,
        name="drive_coupler",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=dark_trim,
        name="motor_socket_plate",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.020),
        origin=Origin(xyz=(0.0, -0.183, 0.087), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="selector_boss",
    )
    base.visual(
        Box((0.075, 0.034, 0.060)),
        origin=Origin(xyz=(0.0, -0.166, 0.087)),
        material=dark_trim,
        name="selector_mount_bridge",
    )
    # Small juice outlet on one side of the chamber, tied into the housing.
    base.visual(
        Box((0.030, 0.070, 0.026)),
        origin=Origin(xyz=(0.130, -0.020, 0.143)),
        material=matte_white,
        name="juice_spout",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.078),
        origin=Origin(xyz=(0.168, -0.020, 0.143), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_smoke,
        name="spout_tip",
    )

    # Continuous stainless centrifugal basket below the transparent cover.
    filter_basket = model.part("filter_basket")
    basket_wall = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.048, 0.006), (0.068, 0.028), (0.102, 0.083)],
        inner_profile=[(0.040, 0.008), (0.059, 0.029), (0.094, 0.077)],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    filter_basket.visual(
        mesh_from_geometry(basket_wall, "perforated_basket_wall"),
        material=stainless,
        name="perforated_basket_wall",
    )
    filter_basket.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.100, tube=0.004, radial_segments=16, tubular_segments=80),
            "top_rolled_rim",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
        material=stainless,
        name="top_rolled_rim",
    )
    filter_basket.visual(
        Cylinder(radius=0.052, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stainless,
        name="cutting_disk",
    )
    filter_basket.visual(
        Cylinder(radius=0.019, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_trim,
        name="basket_hub",
    )
    # Raised cutting teeth on the disk give the spinner a serrated appliance feel.
    for i in range(12):
        angle = 2.0 * math.pi * i / 12
        filter_basket.visual(
            Box((0.004, 0.025, 0.004)),
            origin=Origin(
                xyz=(0.030 * math.cos(angle), 0.030 * math.sin(angle), 0.014),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_trim,
            name=f"cutting_tooth_{i}",
        )

    model.articulation(
        "base_to_filter_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=filter_basket,
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=120.0),
    )

    # Rear-hinged clear lid: a one-piece hollow cover and feed chute.
    lid = model.part("lid")
    lid_shell = _revolved_shell(
        outer_profile=[
            (0.121, 0.000),
            (0.130, 0.020),
            (0.117, 0.055),
            (0.071, 0.086),
            (0.044, 0.092),
            (0.044, 0.255),
        ],
        inner_profile=[
            (0.108, 0.000),
            (0.111, 0.015),
            (0.101, 0.049),
            (0.054, 0.079),
            (0.035, 0.088),
            (0.035, 0.255),
        ],
        y_offset=-0.140,
        segments=96,
    )
    lid.visual(
        mesh_from_geometry(lid_shell, "clear_lid_and_chute"),
        material=clear_smoke,
        name="clear_lid_and_chute",
    )
    # Hinge leaves bridge the forward clear cover back to the rear hinge axis.
    for i, x in enumerate((-0.060, 0.060)):
        lid.visual(
            Box((0.032, 0.045, 0.012)),
            origin=Origin(xyz=(x, -0.012, 0.006)),
            material=clear_smoke,
            name=f"hinge_leaf_{i}",
        )
    lid.visual(
        Box((0.115, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.002, 0.006)),
        material=dark_trim,
        name="hinge_bar",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.140, 0.248)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    # Cylindrical pusher seated inside the vertical chute.  It is parented to
    # the lid so it follows the lid when the cover is opened.
    pusher = model.part("feed_pusher")
    pusher.visual(
        Cylinder(radius=0.027, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=matte_white,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=matte_white,
        name="thumb_cap",
    )
    pusher.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=pulp_orange,
        name="cap_button",
    )
    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.140, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.120),
    )

    # Front rotary speed selector on a short horizontal shaft.
    speed_selector = model.part("speed_selector")
    selector_mesh = KnobGeometry(
        0.056,
        0.028,
        body_style="skirted",
        top_diameter=0.043,
        grip=KnobGrip(style="fluted", count=18, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    speed_selector.visual(
        mesh_from_geometry(selector_mesh, "speed_selector_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="selector_knob",
    )
    speed_selector.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="selector_shaft",
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.0, -0.193, 0.087)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    basket = object_model.get_part("filter_basket")
    selector = object_model.get_part("speed_selector")

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_feed_pusher")
    basket_spin = object_model.get_articulation("base_to_filter_basket")
    selector_turn = object_model.get_articulation("base_to_speed_selector")

    ctx.check(
        "four user-facing mechanisms are articulated",
        lid_hinge is not None
        and pusher_slide is not None
        and basket_spin is not None
        and selector_turn is not None,
        details="lid hinge, feed pusher slide, filter basket spin, and speed selector are all required",
    )

    # Closed lid sits on the raised chamber rim, with enough footprint overlap to
    # read as a sealed transparent cover rather than a floating dome.
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="clear_lid_and_chute",
        negative_elem="chamber_rim",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid seats on chamber rim",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="clear_lid_and_chute",
        elem_b="chamber_rim",
        min_overlap=0.12,
        name="lid covers the chamber footprint",
    )

    # The pusher is clearanced inside the chute but its cap rests on the chute
    # lip, so the prismatic part is visibly supported at the seated position.
    ctx.expect_gap(
        pusher,
        lid,
        axis="z",
        positive_elem="thumb_cap",
        negative_elem="clear_lid_and_chute",
        max_gap=0.001,
        max_penetration=0.0,
        name="pusher cap rests on feed chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_stem",
        elem_b="clear_lid_and_chute",
        min_overlap=0.18,
        name="seated pusher remains inserted in chute",
    )
    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.120}):
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_stem",
            elem_b="clear_lid_and_chute",
            min_overlap=0.060,
            name="raised pusher retains insertion in chute",
        )
        raised_pusher_pos = ctx.part_world_position(pusher)
    ctx.check(
        "feed pusher slides upward along chute axis",
        rest_pusher_pos is not None
        and raised_pusher_pos is not None
        and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.10,
        details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
    )

    # Basket is centered inside the chamber and below the clear lid, leaving a
    # real visible air gap for spinning clearance.
    ctx.expect_within(
        basket,
        base,
        axes="xy",
        inner_elem="perforated_basket_wall",
        outer_elem="chamber_rim",
        margin=0.0,
        name="filter basket sits inside chamber ring",
    )
    ctx.expect_gap(
        lid,
        basket,
        axis="z",
        positive_elem="clear_lid_and_chute",
        negative_elem="top_rolled_rim",
        min_gap=0.006,
        max_gap=0.025,
        name="lid clears spinning basket rim",
    )

    # Opening the rear hinge should lift the front of the cover upward/outward.
    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_element_world_aabb(lid, elem="clear_lid_and_chute")
    ctx.check(
        "rear hinge opens lid upward",
        opened_lid_aabb is not None and opened_lid_aabb[1][2] > 0.55,
        details=f"opened_lid_aabb={opened_lid_aabb}",
    )

    # Front rotary selector is on the front face, separate from the base but
    # aligned to a short horizontal shaft.
    ctx.expect_origin_gap(
        base,
        selector,
        axis="y",
        min_gap=0.15,
        max_gap=0.24,
        name="speed selector is mounted on front of base",
    )
    basket_rest_pos = ctx.part_world_position(basket)
    with ctx.pose({basket_spin: 1.2, selector_turn: 0.5}):
        basket_rotated_pos = ctx.part_world_position(basket)
    ctx.check(
        "filter basket spins about fixed central axis",
        basket_rest_pos is not None
        and basket_rotated_pos is not None
        and abs(basket_rotated_pos[0] - basket_rest_pos[0]) < 1e-6
        and abs(basket_rotated_pos[1] - basket_rest_pos[1]) < 1e-6,
        details=f"rest={basket_rest_pos}, rotated={basket_rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
