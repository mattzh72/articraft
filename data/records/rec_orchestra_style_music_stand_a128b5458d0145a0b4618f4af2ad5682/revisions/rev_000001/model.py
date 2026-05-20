from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """A true hollow cylindrical tube, centered on its local Z axis."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an origin/rpy that aligns a Z-axis cylinder between two points."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.sqrt(vx * vx + vy * vy), vz)
    return (
        Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_segment(part, start, end, radius: float, material, name: str) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_tripod_orchestra_stand")

    black = model.material("satin_black_powdercoat", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_steel = model.material("dark_stamped_steel", rgba=(0.07, 0.075, 0.08, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    hinge_metal = model.material("bright_hinge_pins", rgba=(0.86, 0.82, 0.68, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.002, 0.002, 0.002, 1.0))

    # Stage-stand scale: a 1.0 m lower tube, three folding legs spreading
    # roughly half a meter, and a score desk near chest height.
    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        _tube_mesh(0.026, 0.0205, 0.78, "outer_sleeve_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
        material=black,
        name="outer_sleeve",
    )
    tripod_base.visual(
        _tube_mesh(0.039, 0.0215, 0.075, "upper_clamp_ring"),
        origin=Origin(xyz=(0.0, 0.0, 1.065)),
        material=black,
        name="upper_clamp",
    )
    tripod_base.visual(
        _tube_mesh(0.060, 0.0265, 0.080, "lower_leg_collar_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.325)),
        material=black,
        name="leg_collar",
    )
    tripod_base.visual(
        Cylinder(radius=0.009, length=0.095),
        origin=Origin(xyz=(0.080, 0.0, 1.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="height_screw",
    )
    tripod_base.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.137, 0.0, 1.065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="height_knob",
    )
    for i, phi in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        tripod_base.visual(
            Box((0.010, 0.006, 0.740)),
            origin=Origin(xyz=(0.028 * math.cos(phi), 0.028 * math.sin(phi), 0.710), rpy=(0.0, 0.0, phi)),
            material=black,
            name=f"sleeve_spine_{i}",
        )
    for i, phi in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial = (math.cos(phi), math.sin(phi), 0.0)
        # A short lug visibly carries each folding leg hinge outboard of the collar.
        _add_segment(
            tripod_base,
            (radial[0] * 0.050, radial[1] * 0.050, 0.325),
            (radial[0] * 0.080, radial[1] * 0.080, 0.325),
            0.010,
            black,
            f"leg_lug_{i}",
        )

    # Three separate folding legs, each hinged to the lower collar.
    for i, phi in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial = (math.cos(phi), math.sin(phi), 0.0)
        tangent = (-math.sin(phi), math.cos(phi), 0.0)
        leg = model.part(f"leg_{i}")
        foot_end = (radial[0] * 0.50, radial[1] * 0.50, -0.290)
        _add_segment(leg, (0.0, 0.0, 0.0), foot_end, 0.0115, black, "leg_tube")
        leg.visual(
            Box((0.100, 0.040, 0.030)),
            origin=Origin(
                xyz=(foot_end[0], foot_end[1], foot_end[2] - 0.003),
                rpy=(0.0, 0.0, phi),
            ),
            material=rubber,
            name="rubber_foot",
        )
        model.articulation(
            f"leg_{i}_fold",
            ArticulationType.REVOLUTE,
            parent=tripod_base,
            child=leg,
            origin=Origin(xyz=(radial[0] * 0.080, radial[1] * 0.080, 0.325)),
            # Rotate around the local tangent so increasing q folds the foot upward.
            axis=(-tangent[0], -tangent[1], -tangent[2]),
            motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=18.0, velocity=1.6),
        )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.016, length=1.00),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=chrome,
        name="inner_post_tube",
    )
    inner_post.visual(
        Box((0.006, 0.006, 0.880)),
        origin=Origin(xyz=(0.0, 0.016, 0.050)),
        material=chrome,
        name="post_keyway",
    )
    inner_post.visual(
        Cylinder(radius=0.0206, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        material=black,
        name="lower_guide_bushing",
    )
    inner_post.visual(
        Cylinder(radius=0.0206, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=black,
        name="upper_guide_bushing",
    )
    inner_post.visual(
        Box((0.052, 0.035, 0.070)),
        origin=Origin(xyz=(0.0, -0.043, 0.545)),
        material=black,
        name="head_block",
    )
    inner_post.visual(
        Box((0.036, 0.038, 0.060)),
        origin=Origin(xyz=(0.0, -0.018, 0.535)),
        material=black,
        name="post_head_bridge",
    )
    inner_post.visual(
        Cylinder(radius=0.010, length=0.260),
        origin=Origin(xyz=(0.0, -0.045, 0.580), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="tilt_cross_pin",
    )
    for side, x in enumerate((-0.105, 0.105)):
        inner_post.visual(
            Box((0.020, 0.050, 0.070)),
            origin=Origin(xyz=(x, -0.052, 0.580)),
            material=black,
            name=f"yoke_cheek_{side}",
        )
    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=tripod_base,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 1.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.300, effort=90.0, velocity=0.20),
    )

    # Slotted, shallowly lipped stamped steel desk.  The mesh panels lie in local
    # XY, so rotate them into each desk part's XZ sheet plane.
    main_width = 0.520
    main_height = 0.340
    main_desk = model.part("main_desk")
    main_panel = SlotPatternPanelGeometry(
        (main_width, main_height),
        0.006,
        slot_size=(0.060, 0.006),
        pitch=(0.090, 0.034),
        frame=0.024,
        corner_radius=0.010,
        slot_angle_deg=0.0,
        stagger=True,
    )
    main_desk.visual(
        mesh_from_geometry(main_panel, "main_slotted_desk_panel"),
        origin=Origin(xyz=(0.0, 0.0, main_height / 2.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="main_panel",
    )
    main_desk.visual(
        Box((0.570, 0.060, 0.028)),
        origin=Origin(xyz=(0.0, 0.040, 0.014)),
        material=dark_steel,
        name="bottom_score_lip",
    )
    main_desk.visual(
        Box((0.500, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, main_height - 0.018)),
        material=dark_steel,
        name="pressed_top_rib",
    )
    for z in (0.125, 0.215):
        main_desk.visual(
            Box((0.440, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.005, z)),
            material=dark_steel,
            name=f"pressed_rib_{int(z * 1000)}",
        )
    main_desk.visual(
        Cylinder(radius=0.012, length=0.170),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="tilt_hinge_barrel",
    )
    main_desk.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.150, -0.012, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="tilt_knob",
    )
    for idx, sign in enumerate((-1.0, 1.0)):
        x_edge = sign * (main_width / 2.0)
        main_desk.visual(
            Box((0.020, 0.016, 0.285)),
            origin=Origin(xyz=(x_edge + sign * 0.004, 0.009, main_height / 2.0)),
            material=hinge_metal,
            name=f"side_hinge_leaf_{idx}",
        )
    model.articulation(
        "desk_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=main_desk,
        origin=Origin(xyz=(0.0, -0.006, 0.580)),
        # At q=0 the desk is vertical. Positive q tips its upper edge back.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.75, effort=25.0, velocity=1.0),
    )

    wing_width = 0.220
    wing_height = 0.270
    hinge_offset = 0.008
    for idx, sign in enumerate((1.0, -1.0)):
        wing = model.part(f"side_wing_{idx}")
        wing_panel = SlotPatternPanelGeometry(
            (wing_width, wing_height),
            0.005,
            slot_size=(0.046, 0.005),
            pitch=(0.074, 0.032),
            frame=0.018,
            corner_radius=0.008,
            stagger=True,
        )
        wing_center_x = sign * (hinge_offset + wing_width / 2.0)
        wing.visual(
            mesh_from_geometry(wing_panel, f"side_wing_{idx}_slotted_panel"),
            origin=Origin(xyz=(wing_center_x, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="wing_panel",
        )
        wing.visual(
            Box((wing_width, 0.045, 0.020)),
            origin=Origin(xyz=(wing_center_x, 0.021, -wing_height / 2.0 + 0.010)),
            material=dark_steel,
            name="wing_score_lip",
        )
        wing.visual(
            Cylinder(radius=0.0065, length=wing_height * 0.96),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=hinge_metal,
            name="wing_hinge_barrel",
        )
        wing.visual(
            Box((0.020, 0.014, wing_height * 0.86)),
            origin=Origin(xyz=(sign * 0.006, -0.004, 0.0)),
            material=hinge_metal,
            name="wing_hinge_leaf",
        )
        x_hinge = sign * (main_width / 2.0 + hinge_offset)
        limits = MotionLimits(lower=-2.15, upper=0.0, effort=5.0, velocity=1.4)
        axis = (0.0, 0.0, 1.0)
        if sign < 0.0:
            limits = MotionLimits(lower=0.0, upper=2.15, effort=5.0, velocity=1.4)
        model.articulation(
            f"wing_{idx}_hinge",
            ArticulationType.REVOLUTE,
            parent=main_desk,
            child=wing,
            origin=Origin(xyz=(x_hinge, 0.0, main_height / 2.0)),
            axis=axis,
            motion_limits=limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod_base")
    inner = object_model.get_part("inner_post")
    desk = object_model.get_part("main_desk")
    wing_0 = object_model.get_part("side_wing_0")
    wing_1 = object_model.get_part("side_wing_1")
    slide = object_model.get_articulation("post_slide")
    tilt = object_model.get_articulation("desk_tilt")
    wing_0_hinge = object_model.get_articulation("wing_0_hinge")
    wing_1_hinge = object_model.get_articulation("wing_1_hinge")
    leg_0_hinge = object_model.get_articulation("leg_0_fold")

    ctx.allow_overlap(
        inner,
        tripod,
        elem_a="lower_guide_bushing",
        elem_b="outer_sleeve",
        reason="The lower nylon guide bushing is intentionally a light press fit inside the telescoping sleeve.",
    )
    ctx.allow_overlap(
        inner,
        tripod,
        elem_a="upper_guide_bushing",
        elem_b="outer_sleeve",
        reason="The upper nylon guide bushing is intentionally a light press fit inside the telescoping sleeve.",
    )
    for i in range(3):
        ctx.allow_overlap(
            f"leg_{i}",
            tripod,
            elem_a="leg_tube",
            elem_b=f"leg_lug_{i}",
            reason="The folding leg tube end is locally captured in the tripod collar lug at its hinge.",
        )

    ctx.expect_within(
        inner,
        tripod,
        axes="xy",
        inner_elem="inner_post_tube",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="telescoping post is centered in sleeve footprint",
    )
    ctx.expect_overlap(
        inner,
        tripod,
        axes="z",
        elem_a="inner_post_tube",
        elem_b="outer_sleeve",
        min_overlap=0.16,
        name="collapsed post remains inserted in lower sleeve",
    )
    ctx.expect_within(
        inner,
        tripod,
        axes="xy",
        inner_elem="lower_guide_bushing",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="guide bushing is retained in sleeve bore",
    )
    ctx.expect_overlap(
        inner,
        tripod,
        axes="z",
        elem_a="lower_guide_bushing",
        elem_b="outer_sleeve",
        min_overlap=0.030,
        name="guide bushing has sleeve engagement",
    )
    ctx.expect_within(
        inner,
        tripod,
        axes="xy",
        inner_elem="upper_guide_bushing",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="upper guide bushing is retained in sleeve bore",
    )
    ctx.expect_overlap(
        inner,
        tripod,
        axes="z",
        elem_a="upper_guide_bushing",
        elem_b="outer_sleeve",
        min_overlap=0.030,
        name="upper guide bushing has sleeve engagement",
    )
    for i in range(3):
        ctx.expect_contact(
            f"leg_{i}",
            tripod,
            elem_a="leg_tube",
            elem_b=f"leg_lug_{i}",
            contact_tol=0.001,
            name=f"leg {i} hinge tube is seated in collar lug",
        )
    rest_post = ctx.part_world_position(inner)
    with ctx.pose({slide: 0.300}):
        ctx.expect_overlap(
            inner,
            tripod,
            axes="z",
            elem_a="inner_post_tube",
            elem_b="outer_sleeve",
            min_overlap=0.12,
            name="extended post retains insertion in sleeve",
        )
        extended_post = ctx.part_world_position(inner)
    ctx.check(
        "post slide raises the desk head",
        rest_post is not None and extended_post is not None and extended_post[2] > rest_post[2] + 0.25,
        details=f"rest={rest_post}, extended={extended_post}",
    )

    # The wings are not freestanding panels: their inner edges sit within a few
    # centimeters of the main desk side edges and overlap vertically.
    ctx.expect_gap(
        wing_0,
        desk,
        axis="x",
        min_gap=0.004,
        max_gap=0.030,
        positive_elem="wing_panel",
        negative_elem="main_panel",
        name="side wing 0 is carried at the desk edge",
    )
    ctx.expect_gap(
        desk,
        wing_1,
        axis="x",
        min_gap=0.004,
        max_gap=0.030,
        positive_elem="main_panel",
        negative_elem="wing_panel",
        name="side wing 1 is carried at the desk edge",
    )
    ctx.expect_overlap(
        wing_0,
        desk,
        axes="z",
        elem_a="wing_panel",
        elem_b="main_panel",
        min_overlap=0.24,
        name="side wing 0 hinge spans the score height",
    )
    ctx.expect_overlap(
        wing_1,
        desk,
        axes="z",
        elem_a="wing_panel",
        elem_b="main_panel",
        min_overlap=0.24,
        name="side wing 1 hinge spans the score height",
    )

    desk_aabb = ctx.part_element_world_aabb(desk, elem="main_panel")
    rest_desk_y = (desk_aabb[0][1] + desk_aabb[1][1]) / 2.0 if desk_aabb else None
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_element_world_aabb(desk, elem="main_panel")
        tilted_desk_y = (tilted_aabb[0][1] + tilted_aabb[1][1]) / 2.0 if tilted_aabb else None
    ctx.check(
        "desk tilt moves sheet backward",
        rest_desk_y is not None and tilted_desk_y is not None and tilted_desk_y < rest_desk_y - 0.07,
        details=f"rest_y={rest_desk_y}, tilted_y={tilted_desk_y}",
    )

    wing0_aabb = ctx.part_element_world_aabb(wing_0, elem="wing_panel")
    wing1_aabb = ctx.part_element_world_aabb(wing_1, elem="wing_panel")
    wing0_rest_y = (wing0_aabb[0][1] + wing0_aabb[1][1]) / 2.0 if wing0_aabb else None
    wing1_rest_y = (wing1_aabb[0][1] + wing1_aabb[1][1]) / 2.0 if wing1_aabb else None
    with ctx.pose({wing_0_hinge: -1.4, wing_1_hinge: 1.4}):
        wing0_fold = ctx.part_element_world_aabb(wing_0, elem="wing_panel")
        wing1_fold = ctx.part_element_world_aabb(wing_1, elem="wing_panel")
        wing0_fold_y = (wing0_fold[0][1] + wing0_fold[1][1]) / 2.0 if wing0_fold else None
        wing1_fold_y = (wing1_fold[0][1] + wing1_fold[1][1]) / 2.0 if wing1_fold else None
    ctx.check(
        "side wings rotate backward on edge hinges",
        wing0_rest_y is not None
        and wing1_rest_y is not None
        and wing0_fold_y is not None
        and wing1_fold_y is not None
        and wing0_fold_y < wing0_rest_y - 0.10
        and wing1_fold_y < wing1_rest_y - 0.10,
        details=f"wing0 rest/fold={wing0_rest_y}/{wing0_fold_y}, wing1 rest/fold={wing1_rest_y}/{wing1_fold_y}",
    )

    leg0_rest = ctx.part_world_aabb("leg_0")
    rest_leg_top = leg0_rest[1][2] if leg0_rest else None
    with ctx.pose({leg_0_hinge: 1.0}):
        leg0_fold = ctx.part_world_aabb("leg_0")
        folded_leg_top = leg0_fold[1][2] if leg0_fold else None
    ctx.check(
        "tripod legs fold upward",
        rest_leg_top is not None and folded_leg_top is not None and folded_leg_top > rest_leg_top + 0.10,
        details=f"rest_top={rest_leg_top}, folded_top={folded_leg_top}",
    )

    return ctx.report()


object_model = build_object_model()
