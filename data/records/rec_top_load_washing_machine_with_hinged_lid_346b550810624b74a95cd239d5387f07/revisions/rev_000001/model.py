from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_box(width: float, depth: float, height: float, radius: float):
    """CadQuery rounded rectangular solid, centered in XY and resting on z=0."""
    shape = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .translate((0.0, 0.0, height / 2.0))
    )
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_top_load_washer")

    porcelain = model.material("warm_white_porcelain", rgba=(0.92, 0.94, 0.93, 1.0))
    satin = model.material("satin_chrome", rgba=(0.63, 0.66, 0.67, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark = model.material("gloss_black", rgba=(0.015, 0.017, 0.020, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.12, 0.16, 0.42))
    blue = model.material("soft_status_blue", rgba=(0.15, 0.48, 0.85, 1.0))

    # Cabinet with a real cylindrical wash opening cut through the top deck.
    cabinet = (
        cq.Workplane("XY")
        .box(0.74, 0.72, 0.90)
        .translate((0.0, 0.0, 0.45))
        .edges("|Z")
        .fillet(0.025)
    )
    cavity = (
        cq.Workplane("XY")
        .circle(0.286)
        .extrude(1.08)
        .translate((0.0, -0.045, -0.06))
    )
    cabinet = cabinet.cut(cavity)

    console = _rounded_box(0.72, 0.13, 0.20, 0.018).translate((0.0, 0.425, 0.895))
    cabinet = cabinet.union(console)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(cabinet, "cabinet_shell", tolerance=0.0012),
        material=porcelain,
        name="cabinet_shell",
    )
    body.visual(
        Box((0.62, 0.012, 0.120)),
        origin=Origin(xyz=(0.0, 0.356, 1.020)),
        material=dark,
        name="console_fascia",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(0.287, 0.012, radial_segments=24, tubular_segments=72), "opening_gasket"),
        origin=Origin(xyz=(0.0, -0.045, 0.906)),
        material=rubber,
        name="opening_gasket",
    )
    # Stationary trim/recesses in the top deck.
    body.visual(
        Box((0.104, 0.250, 0.004)),
        origin=Origin(xyz=(-0.315, -0.238, 0.904)),
        material=dark,
        name="drawer_recess",
    )
    body.visual(
        Box((0.112, 0.118, 0.004)),
        origin=Origin(xyz=(0.315, 0.257, 0.904)),
        material=dark,
        name="bleach_recess",
    )
    body.visual(
        Box((0.660, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.045, 0.035)),
        material=dark,
        name="lower_support_x",
    )
    body.visual(
        Box((0.045, 0.650, 0.040)),
        origin=Origin(xyz=(0.0, -0.045, 0.035)),
        material=dark,
        name="lower_support_y",
    )
    body.visual(
        Cylinder(radius=0.045, length=0.105),
        origin=Origin(xyz=(0.0, -0.045, 0.1075)),
        material=satin,
        name="drive_bearing",
    )
    body.visual(
        Box((0.070, 0.040, 0.048)),
        origin=Origin(xyz=(-0.225, 0.230, 0.924)),
        material=satin,
        name="hinge_block_0",
    )
    body.visual(
        Box((0.070, 0.040, 0.048)),
        origin=Origin(xyz=(0.225, 0.230, 0.924)),
        material=satin,
        name="hinge_block_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.145),
        origin=Origin(xyz=(-0.225, 0.230, 0.956), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin,
        name="hinge_barrel_0",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.145),
        origin=Origin(xyz=(0.225, 0.230, 0.956), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin,
        name="hinge_barrel_1",
    )

    # Stainless basket and low impeller: one continuous rotating wash chamber.
    tub = model.part("tub")
    tub_shell = LatheGeometry.from_shell_profiles(
        [(0.155, 0.010), (0.225, 0.070), (0.248, 0.575), (0.260, 0.650)],
        [(0.115, 0.045), (0.175, 0.095), (0.216, 0.565), (0.232, 0.650)],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    tub.visual(
        mesh_from_geometry(tub_shell, "stainless_basket"),
        material=stainless,
        name="stainless_basket",
    )
    tub.visual(
        Cylinder(radius=0.170, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark,
        name="dark_basin_floor",
    )
    tub.visual(
        Cylinder(radius=0.070, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=satin,
        name="low_impeller",
    )
    tub.visual(
        Cylinder(radius=0.052, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin,
        name="drive_hub",
    )
    tub.visual(
        Cylinder(radius=0.026, length=0.135),
        origin=Origin(xyz=(0.0, 0.0, -0.0675)),
        material=satin,
        name="drive_spindle",
    )
    model.articulation(
        "body_to_tub",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tub,
        origin=Origin(xyz=(0.0, -0.045, 0.210)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=8.0),
    )

    # Rear-hinged soft-close lid, modeled in its hinge-line child frame.
    lid = model.part("lid")
    lid_width = 0.540
    lid_depth = 0.525
    lid_thickness = 0.030
    lid_frame = (
        cq.Workplane("XY")
        .center(0.0, -lid_depth / 2.0)
        .rect(lid_width, lid_depth)
        .extrude(lid_thickness)
        .cut(
            cq.Workplane("XY")
            .center(0.0, -lid_depth / 2.0)
            .rect(lid_width - 0.112, lid_depth - 0.128)
            .extrude(lid_thickness + 0.012)
            .translate((0.0, 0.0, -0.006))
        )
    )
    lid.visual(
        mesh_from_cadquery(lid_frame, "lid_frame"),
        material=porcelain,
        name="lid_frame",
    )
    lid.visual(
        Box((lid_width - 0.082, lid_depth - 0.098, 0.012)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0, 0.014)),
        material=glass,
        name="glass_pane",
    )
    lid.visual(
        Box((0.110, 0.030, 0.026)),
        origin=Origin(xyz=(-0.225, -0.035, 0.013)),
        material=satin,
        name="hinge_leaf_0",
    )
    lid.visual(
        Box((0.110, 0.030, 0.026)),
        origin=Origin(xyz=(0.225, -0.035, 0.013)),
        material=satin,
        name="hinge_leaf_1",
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.212, 0.956)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.65, lower=0.0, upper=1.22),
    )

    # Detergent drawer, sliding forward from the left side of the top deck.
    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.092, 0.250, 0.028)),
        origin=Origin(xyz=(0.0, 0.125, 0.012)),
        material=porcelain,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.110, 0.020, 0.046)),
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
        material=satin,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.060, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, -0.022, 0.016)),
        material=dark,
        name="pull_shadow",
    )
    model.articulation(
        "body_to_detergent_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(-0.315, -0.360, 0.916)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.18, lower=0.0, upper=0.180),
    )

    # Bleach fill flap near the rear edge, also framed by a dark stationary recess.
    flap = model.part("bleach_flap")
    flap.visual(
        Box((0.100, 0.105, 0.012)),
        origin=Origin(xyz=(0.0, -0.0525, 0.006)),
        material=porcelain,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.008, length=0.118),
        origin=Origin(xyz=(0.0, 0.000, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin,
        name="flap_hinge_pin",
    )
    model.articulation(
        "body_to_bleach_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.315, 0.314, 0.906)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    # Central dial on the rear console.
    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.086,
            0.032,
            body_style="skirted",
            top_diameter=0.066,
            edge_radius=0.002,
            grip=KnobGrip(style="fluted", count=28, depth=0.0014),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
        ),
        "central_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin,
        name="dial_cap",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.347, 1.035)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    button_xs = (-0.245, -0.150, -0.055, 0.150, 0.245)
    button_mats = (porcelain, porcelain, blue, porcelain, porcelain)
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.060, 0.014, 0.026)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=button_mats[i],
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, 0.347, 0.975)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=0.05, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tub = object_model.get_part("tub")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("detergent_drawer")
    flap = object_model.get_part("bleach_flap")
    dial = object_model.get_part("dial")

    lid_joint = object_model.get_articulation("body_to_lid")
    drawer_joint = object_model.get_articulation("body_to_detergent_drawer")
    flap_joint = object_model.get_articulation("body_to_bleach_flap")
    tub_joint = object_model.get_articulation("body_to_tub")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.allow_overlap(
        body,
        tub,
        elem_a="drive_bearing",
        elem_b="drive_spindle",
        reason="The rotating wash basket spindle is intentionally captured inside the fixed drive bearing.",
    )

    ctx.check("tub rotates continuously", tub_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("dial rotates continuously", dial_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("five separate push buttons", len([p for p in object_model.parts if p.name.startswith("button_")]) == 5)

    ctx.expect_within(tub, body, axes="xy", margin=0.020, name="basket is centered inside cabinet footprint")
    ctx.expect_within(
        tub,
        body,
        axes="xy",
        inner_elem="drive_spindle",
        outer_elem="drive_bearing",
        margin=0.0,
        name="spindle is captured in the bearing bore",
    )
    ctx.expect_overlap(
        tub,
        body,
        axes="z",
        elem_a="drive_spindle",
        elem_b="drive_bearing",
        min_overlap=0.070,
        name="spindle remains engaged in drive bearing",
    )
    ctx.expect_overlap(lid, tub, axes="xy", min_overlap=0.32, name="glass lid covers the wash opening")
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="opening_gasket",
        min_gap=0.0,
        max_gap=0.070,
        name="closed lid rests just above the top deck gasket",
    )
    ctx.expect_contact(drawer, body, elem_a="drawer_pull", elem_b="drawer_recess", contact_tol=0.022, name="detergent drawer is mounted in deck recess")
    ctx.expect_contact(flap, body, elem_a="flap_panel", elem_b="bleach_recess", contact_tol=0.018, name="bleach flap sits in rear deck recess")
    ctx.expect_contact(dial, body, elem_a="dial_cap", elem_b="console_fascia", contact_tol=0.006, name="dial is seated on console")

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.0}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.22,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.160}):
        pulled_drawer = ctx.part_world_position(drawer)
    ctx.check(
        "detergent drawer slides forward",
        rest_drawer is not None and pulled_drawer is not None and pulled_drawer[1] < rest_drawer[1] - 0.12,
        details=f"rest={rest_drawer}, pulled={pulled_drawer}",
    )

    closed_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: 1.0}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "bleach flap rotates upward",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.045,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    for i in range(5):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"body_to_button_{i}")
        ctx.expect_contact(
            button,
            body,
            elem_a="button_cap",
            elem_b="console_fascia",
            contact_tol=0.004,
            name=f"button_{i} is a separate console control",
        )
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} pushes inward",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
