from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_WIDTH = 0.285
BODY_DEPTH = 0.245
BODY_HEIGHT = 0.900
BODY_BOTTOM_Z = 0.035
BODY_CENTER_Z = BODY_BOTTOM_Z + BODY_HEIGHT / 2.0
REAR_Y = -BODY_DEPTH / 2.0
FRONT_Y = BODY_DEPTH / 2.0
FILTER_JOINT_Y = REAR_Y + 0.004
FILTER_Z = 0.490
BUTTON_PANEL_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT + 0.0035


def _rounded_box(width: float, depth: float, height: float, radius: float) -> cq.Workplane:
    """A compact rounded rectangular solid centered on the origin."""
    solid = cq.Workplane("XY").box(width, depth, height)
    if radius > 0.0:
        solid = solid.edges("|Z").fillet(radius)
    return solid


def _tower_shell() -> cq.Workplane:
    """Rounded tower case with a real rear filter bay cut into the shell."""
    outer = _rounded_box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, 0.038).translate(
        (0.0, 0.0, BODY_CENTER_Z)
    )

    # A rear service opening and filter cavity.  The cut leaves a front plenum
    # wall so the model reads as a hollow appliance housing rather than a solid
    # block with a cartridge intersecting it.
    cavity = (
        cq.Workplane("XY")
        .box(0.218, 0.300, 0.540)
        .translate((0.0, -0.055, FILTER_Z))
    )
    return outer.cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_purifier")

    off_white = model.material("warm_white_plastic", rgba=(0.86, 0.86, 0.82, 1.0))
    matte_white = model.material("matte_white", rgba=(0.94, 0.94, 0.90, 1.0))
    dark = model.material("charcoal_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    gray = model.material("soft_gray_button", rgba=(0.55, 0.58, 0.58, 1.0))
    filter_paper = model.material("pleated_filter_media", rgba=(0.82, 0.78, 0.65, 1.0))
    filter_frame = model.material("blue_filter_frame", rgba=(0.20, 0.42, 0.58, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    body = model.part("purifier_body")
    body.visual(
        mesh_from_cadquery(_tower_shell(), "rounded_tower_shell", tolerance=0.002),
        material=off_white,
        name="tower_shell",
    )

    base = _rounded_box(0.315, 0.270, 0.040, 0.050).translate((0.0, 0.0, 0.020))
    body.visual(
        mesh_from_cadquery(base, "base_plinth", tolerance=0.0015),
        material=rubber,
        name="base_plinth",
    )

    front_grille = VentGrilleGeometry(
        (0.170, 0.480),
        frame=0.014,
        face_thickness=0.004,
        duct_depth=0.018,
        slat_pitch=0.020,
        slat_width=0.008,
        slat_angle_deg=28.0,
        corner_radius=0.012,
        slats=VentGrilleSlats(profile="airfoil", direction="down", divider_count=1),
        frame_profile=VentGrilleFrame(style="radiused", depth=0.0012),
        sleeve=VentGrilleSleeve(style="short", depth=0.014, wall=0.0025),
    )
    body.visual(
        mesh_from_geometry(front_grille, "front_outlet_grille"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.001, 0.545), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_white,
        name="front_grille",
    )

    rear_bezel = BezelGeometry(
        (0.218, 0.540),
        (0.260, 0.600),
        0.008,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.030,
        face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.001),
    )
    body.visual(
        mesh_from_geometry(rear_bezel, "rear_filter_bezel"),
        origin=Origin(xyz=(0.0, REAR_Y - 0.001, FILTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=off_white,
        name="rear_bezel",
    )

    control_panel = _rounded_box(0.210, 0.075, 0.008, 0.022).translate(
        (0.0, 0.032, BUTTON_PANEL_TOP_Z - 0.004)
    )
    body.visual(
        mesh_from_cadquery(control_panel, "top_control_panel", tolerance=0.001),
        material=dark,
        name="control_panel",
    )

    # A fixed hinge leaf on the body side visually carries the moving access
    # door knuckle.
    body.visual(
        Box((0.014, 0.027, 0.525)),
        origin=Origin(xyz=(-0.1345, REAR_Y + 0.003, FILTER_Z)),
        material=dark,
        name="body_hinge_leaf",
    )
    body.visual(
        Box((0.005, 0.190, 0.432)),
        origin=Origin(xyz=(-0.1065, -0.024, FILTER_Z)),
        material=dark,
        name="filter_track_0",
    )
    body.visual(
        Box((0.005, 0.190, 0.432)),
        origin=Origin(xyz=(0.1065, -0.024, FILTER_Z)),
        material=dark,
        name="filter_track_1",
    )

    access_door = model.part("access_door")
    door_panel = SlotPatternPanelGeometry(
        (0.238, 0.560),
        0.006,
        slot_size=(0.060, 0.008),
        pitch=(0.075, 0.026),
        frame=0.022,
        corner_radius=0.018,
        stagger=True,
    )
    access_door.visual(
        mesh_from_geometry(door_panel, "vented_access_door_panel"),
        origin=Origin(xyz=(0.121, -0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_white,
        name="door_panel",
    )
    access_door.visual(
        Cylinder(radius=0.006, length=0.530),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="hinge_knuckle",
    )
    access_door.visual(
        Box((0.016, 0.014, 0.520)),
        origin=Origin(xyz=(0.008, -0.007, 0.0)),
        material=dark,
        name="hinge_leaf",
    )
    access_door.visual(
        Box((0.045, 0.009, 0.026)),
        origin=Origin(xyz=(0.205, -0.019, 0.090)),
        material=dark,
        name="finger_latch",
    )

    model.articulation(
        "body_to_access_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_door,
        origin=Origin(xyz=(-0.126, REAR_Y - 0.012, FILTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=0.0, upper=1.75),
    )

    cartridge = model.part("filter_cartridge")
    cartridge.visual(
        Box((0.166, 0.185, 0.402)),
        origin=Origin(xyz=(0.0, 0.0925, 0.0)),
        material=filter_paper,
        name="filter_media",
    )
    cartridge.visual(
        Box((0.188, 0.196, 0.030)),
        origin=Origin(xyz=(0.0, 0.098, 0.216)),
        material=filter_frame,
        name="top_filter_rail",
    )
    cartridge.visual(
        Box((0.188, 0.196, 0.030)),
        origin=Origin(xyz=(0.0, 0.098, -0.216)),
        material=filter_frame,
        name="bottom_filter_rail",
    )
    cartridge.visual(
        Box((0.018, 0.196, 0.432)),
        origin=Origin(xyz=(-0.096, 0.098, 0.0)),
        material=filter_frame,
        name="side_filter_rail_0",
    )
    cartridge.visual(
        Box((0.018, 0.196, 0.432)),
        origin=Origin(xyz=(0.096, 0.098, 0.0)),
        material=filter_frame,
        name="side_filter_rail_1",
    )
    for idx, x in enumerate((-0.060, -0.040, -0.020, 0.0, 0.020, 0.040, 0.060)):
        cartridge.visual(
            Box((0.004, 0.188, 0.398)),
            origin=Origin(xyz=(x, 0.094, 0.0)),
            material=Material("pleat_shadow", rgba=(0.62, 0.58, 0.48, 1.0)),
            name=f"pleat_{idx}",
        )
    cartridge.visual(
        Box((0.062, 0.018, 0.072)),
        origin=Origin(xyz=(0.0, -0.009, 0.145)),
        material=filter_frame,
        name="pull_tab",
    )
    cartridge.visual(
        Box((0.036, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.012, 0.145)),
        material=dark,
        name="tab_grip_recess",
    )

    model.articulation(
        "body_to_filter_cartridge",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cartridge,
        origin=Origin(xyz=(0.0, FILTER_JOINT_Y, FILTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=0.0, upper=0.130),
    )

    button_specs = (
        ("power_button", -0.070, 0.032, 0.014, (0.18, 0.70, 0.55, 1.0)),
        ("fan_button", -0.025, 0.032, 0.012, (0.62, 0.65, 0.64, 1.0)),
        ("mode_button", 0.020, 0.032, 0.012, (0.62, 0.65, 0.64, 1.0)),
        ("timer_button", 0.065, 0.032, 0.012, (0.62, 0.65, 0.64, 1.0)),
    )
    for name, x, y, radius, rgba in button_specs:
        button = model.part(name)
        button.visual(
            Cylinder(radius=radius, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=Material(f"{name}_cap", rgba=rgba),
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=radius * 0.58, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
            material=gray,
            name="button_icon",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, y, BUTTON_PANEL_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("purifier_body")
    door = object_model.get_part("access_door")
    cartridge = object_model.get_part("filter_cartridge")
    door_joint = object_model.get_articulation("body_to_access_door")
    filter_joint = object_model.get_articulation("body_to_filter_cartridge")

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="rear_bezel",
        negative_elem="door_panel",
        min_gap=0.004,
        name="closed rear access door sits outside the rear bezel",
    )
    ctx.expect_within(
        cartridge,
        body,
        axes="xz",
        inner_elem="filter_media",
        outer_elem="rear_bezel",
        margin=0.003,
        name="filter cartridge aligns with the rear service opening",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.55}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "side-hinged access door swings rearward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    rest_filter = ctx.part_world_aabb(cartridge)
    with ctx.pose({filter_joint: 0.130}):
        extended_filter = ctx.part_world_aabb(cartridge)
        ctx.expect_overlap(
            cartridge,
            body,
            axes="y",
            elem_a="filter_media",
            elem_b="rear_bezel",
            min_overlap=0.008,
            name="extended filter remains retained through the rear opening",
        )
    ctx.check(
        "filter cartridge slides out from the rear",
        rest_filter is not None
        and extended_filter is not None
        and extended_filter[0][1] < rest_filter[0][1] - 0.10,
        details=f"rest={rest_filter}, extended={extended_filter}",
    )

    for button_name in ("power_button", "fan_button", "mode_button", "timer_button"):
        joint = object_model.get_articulation(f"body_to_{button_name}")
        button = object_model.get_part(button_name)
        up = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            down = ctx.part_world_position(button)
        ctx.check(
            f"{button_name} depresses independently",
            up is not None and down is not None and down[2] < up[2] - 0.004,
            details=f"up={up}, down={down}",
        )

    return ctx.report()


object_model = build_object_model()
