from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pit_orchestra_stand")

    satin_black = model.material("satin_black", rgba=(0.01, 0.012, 0.012, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.025, 0.024, 0.022, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.002, 0.002, 0.002, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.11, 0.115, 0.12, 1.0))
    worn_edge = model.material("worn_edge", rgba=(0.28, 0.28, 0.26, 1.0))

    base = model.part("base")

    # Low, weighted, compact base: small enough for a pit row but broad enough
    # to plausibly counter the offset desk.
    base_body = LatheGeometry(
        [
            (0.000, 0.000),
            (0.145, 0.000),
            (0.168, 0.006),
            (0.170, 0.024),
            (0.138, 0.038),
            (0.048, 0.044),
            (0.000, 0.044),
        ],
        segments=72,
        closed=True,
    )
    base.visual(
        mesh_from_geometry(base_body, "weighted_base"),
        material=satin_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=rubber,
        name="rubber_foot",
    )

    outer_sleeve = LatheGeometry.from_shell_profiles(
        [(0.022, 0.000), (0.022, 0.420)],
        [(0.0165, 0.000), (0.0165, 0.420)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(outer_sleeve, "outer_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_steel,
        name="outer_sleeve",
    )

    collar = LatheGeometry.from_shell_profiles(
        [(0.031, 0.000), (0.031, 0.052)],
        [(0.0170, 0.000), (0.0170, 0.052)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(collar, "collar_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.436)),
        material=satin_black,
        name="collar_ring",
    )

    screw_boss = LatheGeometry.from_shell_profiles(
        [(0.0105, -0.010), (0.0105, 0.012)],
        [(0.0058, -0.010), (0.0058, 0.012)],
        segments=32,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(screw_boss, "threaded_boss"),
        # The boss is authored along local Z; rotate it so the screw axis is X.
        origin=Origin(xyz=(0.032, 0.0, 0.478), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_black,
        name="threaded_boss",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.013, length=0.600),
        # The child frame sits at the top lip of the sleeve; the tube extends
        # down inside the sleeve so it remains captured when raised.
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_steel,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.364)),
        material=dark_steel,
        name="head_neck",
    )
    head_yoke = TrunnionYokeGeometry(
        (0.128, 0.056, 0.072),
        span_width=0.084,
        trunnion_diameter=0.034,
        trunnion_center_z=0.046,
        base_thickness=0.012,
        corner_radius=0.004,
        center=False,
    )
    mast.visual(
        mesh_from_geometry(head_yoke, "head_yoke"),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=satin_black,
        name="head_yoke",
    )

    desk = model.part("desk")
    desk_panel = ExtrudeGeometry(
        rounded_rect_profile(0.560, 0.280, 0.018, corner_segments=8),
        0.012,
        cap=True,
        center=True,
    )
    desk.visual(
        mesh_from_geometry(desk_panel, "desk_panel"),
        # The mesh is X-by-Y with thickness along local Z.  Rotate it so the
        # panel is a shallow wide music desk rising above the hinge line.
        origin=Origin(xyz=(0.0, 0.0, 0.172), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="desk_panel",
    )
    desk.visual(
        Cylinder(radius=0.0175, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="tilt_barrel",
    )
    for x in (-0.022, 0.022):
        desk.visual(
            Box((0.018, 0.010, 0.052)),
            origin=Origin(xyz=(x, -0.001, 0.035)),
            material=dark_steel,
            name=f"hinge_tab_{0 if x < 0 else 1}",
        )
    desk.visual(
        Box((0.530, 0.034, 0.020)),
        origin=Origin(xyz=(0.0, 0.022, 0.052)),
        material=satin_black,
        name="pencil_lip",
    )
    desk.visual(
        Box((0.530, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.002, 0.309)),
        material=worn_edge,
        name="top_rolled_edge",
    )
    for x in (-0.275, 0.275):
        desk.visual(
            Box((0.010, 0.010, 0.245)),
            origin=Origin(xyz=(x, -0.002, 0.180)),
            material=worn_edge,
            name=f"side_edge_{0 if x < 0 else 1}",
        )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.0050, length=0.061),
        origin=Origin(xyz=(0.0105, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="clamp_screw",
    )
    knob_geom = KnobGeometry(
        0.046,
        0.026,
        body_style="lobed",
        base_diameter=0.030,
        top_diameter=0.040,
        crown_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=10, depth=0.0012),
        bore=KnobBore(style="round", diameter=0.008),
    )
    clamp_knob.visual(
        mesh_from_geometry(knob_geom, "clamp_knob_cap"),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_plastic,
        name="knob_cap",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.464)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.180),
    )
    model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.406)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.45, upper=0.65),
    )
    model.articulation(
        "base_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=clamp_knob,
        origin=Origin(xyz=(0.032, 0.0, 0.478)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    clamp_knob = object_model.get_part("clamp_knob")

    mast_slide = object_model.get_articulation("base_to_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    knob_spin = object_model.get_articulation("base_to_clamp_knob")

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.003,
        name="inner mast is centered inside the sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.18,
        name="collapsed mast remains deeply inserted",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.180}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="raised mast stays centered in the sleeve",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.035,
            name="raised mast keeps retained insertion",
        )
        raised_mast_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast slides upward for height adjustment",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.15,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(desk, elem="desk_panel")
    with ctx.pose({desk_tilt: 0.55}):
        tilted_panel_aabb = ctx.part_element_world_aabb(desk, elem="desk_panel")

    if rest_panel_aabb is not None and tilted_panel_aabb is not None:
        rest_center_y = (rest_panel_aabb[0][1] + rest_panel_aabb[1][1]) * 0.5
        tilted_center_y = (tilted_panel_aabb[0][1] + tilted_panel_aabb[1][1]) * 0.5
        tilt_ok = tilted_center_y < rest_center_y - 0.05
    else:
        rest_center_y = tilted_center_y = None
        tilt_ok = False
    ctx.check(
        "desk tilts back on horizontal head hinge",
        tilt_ok,
        details=f"rest_center_y={rest_center_y}, tilted_center_y={tilted_center_y}",
    )

    ctx.allow_overlap(
        mast,
        desk,
        elem_a="head_yoke",
        elem_b="tilt_barrel",
        reason="The tilt barrel is represented as a captured hinge pin seated in the yoke bores.",
    )
    ctx.expect_overlap(
        desk,
        mast,
        axes="x",
        elem_a="tilt_barrel",
        elem_b="head_yoke",
        min_overlap=0.090,
        name="tilt barrel spans the yoke cheeks",
    )
    ctx.expect_within(
        desk,
        mast,
        axes="yz",
        inner_elem="tilt_barrel",
        outer_elem="head_yoke",
        margin=0.003,
        name="tilt barrel is centered in the head yoke",
    )

    ctx.allow_overlap(
        base,
        clamp_knob,
        elem_a="collar_ring",
        elem_b="clamp_screw",
        reason="The clamp screw intentionally passes through the split height collar wall on its threaded axis.",
    )
    ctx.expect_overlap(
        base,
        clamp_knob,
        axes="x",
        elem_a="collar_ring",
        elem_b="clamp_screw",
        min_overlap=0.010,
        name="clamp screw is threaded through the collar wall",
    )
    ctx.allow_overlap(
        clamp_knob,
        mast,
        elem_a="clamp_screw",
        elem_b="inner_tube",
        reason="The clamp screw tip is modeled with slight compression against the telescoping mast.",
    )
    ctx.expect_gap(
        clamp_knob,
        mast,
        axis="x",
        positive_elem="clamp_screw",
        negative_elem="inner_tube",
        max_penetration=0.002,
        max_gap=0.0005,
        name="clamp screw tip bears on mast tube",
    )
    ctx.expect_within(
        clamp_knob,
        base,
        axes="yz",
        inner_elem="clamp_screw",
        outer_elem="threaded_boss",
        margin=0.001,
        name="clamp screw is coaxial with collar boss",
    )
    ctx.check(
        "clamp knob uses continuous threaded rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={knob_spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
