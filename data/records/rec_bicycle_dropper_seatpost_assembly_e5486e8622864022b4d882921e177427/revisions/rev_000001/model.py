from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    tube_from_spline_points,
)


def _ellipse_section(
    x: float,
    *,
    width: float,
    thickness: float,
    z_center: float,
    segments: int = 32,
) -> list[tuple[float, float, float]]:
    """A YZ oval loop at station x, used for the molded saddle shell."""
    pts: list[tuple[float, float, float]] = []
    for i in range(segments):
        t = math.tau * i / segments
        # Slightly flatten the lower half so the part reads like a molded shell,
        # not a round pillow.
        z_scale = 0.72 if math.sin(t) < 0.0 else 1.0
        pts.append(
            (
                x,
                0.5 * width * math.cos(t),
                z_center + 0.5 * thickness * z_scale * math.sin(t),
            )
        )
    return pts


def _build_saddle_shell() -> MeshGeometry:
    sections = [
        _ellipse_section(0.155, width=0.034, thickness=0.016, z_center=0.078),
        _ellipse_section(0.095, width=0.052, thickness=0.018, z_center=0.083),
        _ellipse_section(0.020, width=0.092, thickness=0.020, z_center=0.087),
        _ellipse_section(-0.065, width=0.132, thickness=0.021, z_center=0.087),
        _ellipse_section(-0.125, width=0.145, thickness=0.018, z_center=0.083),
    ]
    return repair_loft(section_loft(sections))


def _build_outer_tube_shell() -> MeshGeometry:
    """Hollow dropper sleeve with a subtly larger upper collar."""
    tube = LatheGeometry.from_shell_profiles(
        [
            (0.0158, 0.000),
            (0.0158, 0.318),
            (0.0184, 0.328),
            (0.0184, 0.414),
            (0.0166, 0.424),
        ],
        [
            (0.0123, 0.006),
            (0.0123, 0.424),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )

    # Side battery pod: a short round electronics/battery compartment protruding
    # from the post, plus an exposed vertical hinge clevis for the cap.
    pod = (
        CylinderGeometry(radius=0.0125, height=0.052, radial_segments=36)
        .rotate_y(math.pi / 2.0)
        .translate(0.040, 0.000, 0.242)
    )
    bridge = BoxGeometry((0.015, 0.018, 0.035)).translate(0.020, 0.000, 0.242)
    hinge_leaf = BoxGeometry((0.005, 0.010, 0.050)).translate(0.062, -0.015, 0.242)
    top_knuckle = CylinderGeometry(radius=0.0042, height=0.012, radial_segments=20).translate(
        0.064,
        -0.019,
        0.263,
    )
    bottom_knuckle = CylinderGeometry(
        radius=0.0042,
        height=0.012,
        radial_segments=20,
    ).translate(0.064, -0.019, 0.221)

    # A low-profile wireless status window molded into the pod face.
    led = BoxGeometry((0.003, 0.009, 0.004)).translate(0.066, 0.006, 0.254)

    tube.merge(pod)
    tube.merge(bridge)
    tube.merge(hinge_leaf)
    tube.merge(top_knuckle)
    tube.merge(bottom_knuckle)
    tube.merge(led)
    return tube


def _build_wiper_seal() -> MeshGeometry:
    seal = MeshGeometry()
    # Four small wiping pads suggest the rubber collar without creating a solid
    # collision plug through the center where the stanchion must slide.
    seal.merge(BoxGeometry((0.006, 0.020, 0.010)).translate(0.018, 0.0, 0.0))
    seal.merge(BoxGeometry((0.006, 0.020, 0.010)).translate(-0.018, 0.0, 0.0))
    seal.merge(BoxGeometry((0.020, 0.006, 0.010)).translate(0.0, 0.018, 0.0))
    seal.merge(BoxGeometry((0.020, 0.006, 0.010)).translate(0.0, -0.018, 0.0))
    return seal


def _build_saddle_rails_and_bosses() -> MeshGeometry:
    rail_geom = MeshGeometry()
    for y in (-0.026, 0.026):
        rail = tube_from_spline_points(
            [
                (0.122, y, 0.066),
                (0.080, y, 0.046),
                (0.020, y, 0.034),
                (-0.040, y, 0.034),
                (-0.096, y, 0.058),
            ],
            radius=0.003,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        )
        # Molded composite blocks tie the steel rail into the shell at the front
        # and rear. They intentionally touch/seat into the shell and rail within
        # the saddle part so the rail assembly is not visually floating.
        front_boss = BoxGeometry((0.020, 0.012, 0.044)).translate(0.118, y, 0.064)
        rear_boss = BoxGeometry((0.022, 0.012, 0.044)).translate(-0.098, y, 0.066)
        rail_geom.merge(rail)
        rail_geom.merge(front_boss)
        rail_geom.merge(rear_boss)
    rail_geom.merge(BoxGeometry((0.018, 0.060, 0.016)).translate(0.118, 0.0, 0.054))
    rail_geom.merge(BoxGeometry((0.018, 0.060, 0.016)).translate(-0.098, 0.0, 0.058))
    return rail_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.015, 0.017, 0.020, 1.0))
    satin_black = model.material("satin_black", rgba=(0.035, 0.037, 0.040, 1.0))
    polished_stanchion = model.material("polished_stanchion", rgba=(0.78, 0.80, 0.82, 1.0))
    composite_black = model.material("composite_black", rgba=(0.025, 0.023, 0.021, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.43, 0.45, 0.46, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.62, 0.63, 0.65, 1.0))
    rubber_seal = model.material("rubber_seal", rgba=(0.005, 0.005, 0.006, 1.0))
    led_green = model.material("led_green", rgba=(0.10, 0.85, 0.36, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        mesh_from_geometry(_build_outer_tube_shell(), "outer_tube_shell"),
        material=anodized_black,
        name="tube_shell",
    )
    # Add the LED as a separate colored insert, slightly proud of the pod surface.
    outer_tube.visual(
        Box((0.0032, 0.008, 0.0035)),
        origin=Origin(xyz=(0.0675, 0.006, 0.254)),
        material=led_green,
        name="status_window",
    )
    outer_tube.visual(
        mesh_from_geometry(_build_wiper_seal(), "wiper_seal_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=rubber_seal,
        name="wiper_seal",
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0118, length=0.540),
        # Child frame is at the outer tube entry; the stanchion extends below it
        # for retained insertion at full extension and above it to carry the crown.
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=polished_stanchion,
        name="stanchion",
    )
    inner_post.visual(
        Box((0.00052, 0.006, 0.060)),
        # Hidden anti-rotation guide pad: it bridges the small running clearance
        # to the outer sleeve so the sliding post has a real support path.
        origin=Origin(xyz=(0.01205, 0.0, -0.080)),
        material=polished_stanchion,
        name="guide_key",
    )
    inner_post.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        material=satin_black,
        name="crown_collar",
    )
    inner_post.visual(
        Box((0.034, 0.013, 0.020)),
        origin=Origin(xyz=(0.0, 0.027, 0.318)),
        material=satin_black,
        name="pivot_cheek_0",
    )
    inner_post.visual(
        Box((0.034, 0.013, 0.020)),
        origin=Origin(xyz=(0.0, -0.027, 0.318)),
        material=satin_black,
        name="pivot_cheek_1",
    )
    inner_post.visual(
        Box((0.038, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=satin_black,
        name="cheek_bridge",
    )

    battery_cap = model.part("battery_cap")
    battery_cap.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.008,
                body_style="cylindrical",
                grip=KnobGrip(style="fluted", count=20, depth=0.0011),
                edge_radius=0.0006,
            ),
            "battery_screw_cap",
        ),
        origin=Origin(xyz=(0.0025, 0.019, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="screw_cap",
    )
    battery_cap.visual(
        Cylinder(radius=0.0034, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bolt_steel,
        name="cap_hinge_barrel",
    )
    battery_cap.visual(
        Box((0.0045, 0.024, 0.006)),
        origin=Origin(xyz=(0.001, 0.009, 0.0)),
        material=satin_black,
        name="cap_hinge_leaf",
    )
    battery_cap.visual(
        Cylinder(radius=0.0125, length=0.0018),
        origin=Origin(xyz=(-0.0022, 0.019, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_seal,
        name="cap_o_ring",
    )

    clamp_plate = model.part("clamp_plate")
    clamp_plate.visual(
        Cylinder(radius=0.010, length=0.040),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="tilt_pivot_barrel",
    )
    clamp_plate.visual(
        Box((0.034, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=satin_black,
        name="pivot_bridge",
    )
    clamp_plate.visual(
        Box((0.098, 0.074, 0.016)),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=satin_black,
        name="lower_plate",
    )
    clamp_plate.visual(
        Box((0.074, 0.010, 0.006)),
        origin=Origin(xyz=(-0.010, -0.026, 0.028)),
        material=satin_black,
        name="rail_groove_0",
    )
    clamp_plate.visual(
        Box((0.074, 0.010, 0.006)),
        origin=Origin(xyz=(-0.010, 0.026, 0.028)),
        material=satin_black,
        name="rail_groove_1",
    )
    clamp_plate.visual(
        Cylinder(radius=0.0043, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=bolt_steel,
        name="single_bolt",
    )
    clamp_plate.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=bolt_steel,
        name="bolt_washer",
    )

    saddle = model.part("saddle")
    saddle.visual(
        mesh_from_geometry(_build_saddle_shell(), "molded_saddle_shell"),
        material=composite_black,
        name="shell",
    )
    saddle.visual(
        mesh_from_geometry(_build_saddle_rails_and_bosses(), "saddle_rails"),
        material=dark_steel,
        name="rails_and_bosses",
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.424)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=420.0, velocity=0.40, lower=0.0, upper=0.150),
    )
    model.articulation(
        "outer_to_battery_cap",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=battery_cap,
        origin=Origin(xyz=(0.064, -0.019, 0.242)),
        # Positive motion swings the tethered cap outward from the tube.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "inner_to_clamp",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=clamp_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.6,
            lower=-math.radians(12.0),
            upper=math.radians(12.0),
        ),
    )
    model.articulation(
        "clamp_to_saddle",
        ArticulationType.FIXED,
        parent=clamp_plate,
        child=saddle,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tube")
    inner = object_model.get_part("inner_post")
    cap = object_model.get_part("battery_cap")
    clamp = object_model.get_part("clamp_plate")
    saddle = object_model.get_part("saddle")

    post_slide = object_model.get_articulation("outer_to_inner")
    cap_hinge = object_model.get_articulation("outer_to_battery_cap")
    tilt = object_model.get_articulation("inner_to_clamp")

    # The stanchion is a retained telescoping member inside the hollow sleeve.
    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="tube_shell",
        margin=0.001,
        name="stanchion centered in outer tube",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="stanchion",
        elem_b="tube_shell",
        min_overlap=0.200,
        name="collapsed post has deep insertion",
    )

    rest_inner_pos = ctx.part_world_position(inner)
    with ctx.pose({post_slide: 0.150}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="stanchion",
            outer_elem="tube_shell",
            margin=0.001,
            name="extended stanchion remains centered",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="stanchion",
            elem_b="tube_shell",
            min_overlap=0.080,
            name="extended post retains insertion",
        )
        extended_inner_pos = ctx.part_world_position(inner)
    ctx.check(
        "post extends upward",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[2] > rest_inner_pos[2] + 0.145,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )

    ctx.expect_contact(
        cap,
        outer,
        elem_a="cap_hinge_barrel",
        elem_b="tube_shell",
        contact_tol=0.003,
        name="battery cap hinge sits in tube-side clevis",
    )
    with ctx.pose({cap_hinge: 1.25}):
        ctx.expect_gap(
            cap,
            outer,
            axis="x",
            min_gap=-0.005,
            positive_elem="screw_cap",
            negative_elem="tube_shell",
            name="opened cap swings outward of the pod",
        )

    ctx.expect_contact(
        saddle,
        clamp,
        elem_a="rails_and_bosses",
        elem_b="rail_groove_0",
        contact_tol=0.004,
        name="one rail is seated in clamp groove",
    )
    ctx.expect_contact(
        saddle,
        clamp,
        elem_a="rails_and_bosses",
        elem_b="rail_groove_1",
        contact_tol=0.004,
        name="second rail is seated in clamp groove",
    )

    rest_saddle_aabb = ctx.part_world_aabb(saddle)
    with ctx.pose({tilt: math.radians(10.0)}):
        tilted_saddle_aabb = ctx.part_world_aabb(saddle)
    ctx.check(
        "saddle tilts on crown pivot",
        rest_saddle_aabb is not None
        and tilted_saddle_aabb is not None
        and abs(tilted_saddle_aabb[1][0] - rest_saddle_aabb[1][0]) > 0.005,
        details=f"rest={rest_saddle_aabb}, tilted={tilted_saddle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
