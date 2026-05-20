from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.68
DEPTH = 0.74
CABINET_HEIGHT = 0.88
TOP_THICKNESS = 0.055
TOP_Z = CABINET_HEIGHT + TOP_THICKNESS
OPENING_Y = -0.045
OPENING_RADIUS = 0.255
CONSOLE_FRONT_Y = DEPTH / 2.0 - 0.020


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _union_all(shapes):
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _washer_cabinet_shell():
    wall = 0.035
    console_depth = 0.160
    console_height = 0.240
    console_center_y = CONSOLE_FRONT_Y + console_depth / 2.0
    console_center_z = TOP_Z + console_height / 2.0

    front_panel = _cq_box((WIDTH, wall, CABINET_HEIGHT), (0.0, -DEPTH / 2.0 + wall / 2.0, CABINET_HEIGHT / 2.0))
    rear_panel = _cq_box((WIDTH, wall, CABINET_HEIGHT), (0.0, DEPTH / 2.0 - wall / 2.0, CABINET_HEIGHT / 2.0))
    side_panel_0 = _cq_box((wall, DEPTH, CABINET_HEIGHT), (-WIDTH / 2.0 + wall / 2.0, 0.0, CABINET_HEIGHT / 2.0))
    side_panel_1 = _cq_box((wall, DEPTH, CABINET_HEIGHT), (WIDTH / 2.0 - wall / 2.0, 0.0, CABINET_HEIGHT / 2.0))
    bottom_pan = _cq_box((WIDTH, DEPTH, 0.050), (0.0, 0.0, 0.025))

    deck = _cq_box((WIDTH, DEPTH, TOP_THICKNESS), (0.0, 0.0, CABINET_HEIGHT + TOP_THICKNESS / 2.0))
    opening_cutter = (
        cq.Workplane("XY")
        .circle(OPENING_RADIUS)
        .extrude(TOP_THICKNESS + 0.040)
        .translate((0.0, OPENING_Y, CABINET_HEIGHT - 0.020))
    )
    deck = deck.cut(opening_cutter)

    console = _cq_box((WIDTH, console_depth, console_height), (0.0, console_center_y, console_center_z))
    for x_pos, z_pos, radius in (
        (0.0, TOP_Z + 0.135, 0.054),
        (-0.125, TOP_Z + 0.060, 0.036),
        (0.125, TOP_Z + 0.060, 0.036),
    ):
        control_socket = (
            cq.Workplane("XZ")
            .circle(radius)
            .extrude(console_depth + 0.080, both=True)
            .translate((x_pos, console_center_y, z_pos))
        )
        console = console.cut(control_socket)

    # Low, broad hinge pads read as a real support for the heavy lid without
    # colliding with the rotating lid panel.
    hinge_pad_0 = _cq_box((0.150, 0.070, 0.010), (-0.210, 0.285, TOP_Z + 0.005))
    hinge_pad_1 = _cq_box((0.150, 0.070, 0.010), (0.210, 0.285, TOP_Z + 0.005))

    return _union_all(
        [
            front_panel,
            rear_panel,
            side_panel_0,
            side_panel_1,
            bottom_pan,
            deck,
            console,
            hinge_pad_0,
            hinge_pad_1,
        ]
    )


def _washer_tub_shell():
    outer_radius = OPENING_RADIUS + 0.028
    inner_radius = 0.238
    tub_height = 0.600
    tub_bottom = CABINET_HEIGHT - tub_height

    tub_wall = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(tub_height)
        .translate((0.0, OPENING_Y, tub_bottom))
    )
    lower_bowl = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .circle(0.060)
        .extrude(0.045)
        .translate((0.0, OPENING_Y, tub_bottom - 0.040))
    )
    drive_hub = (
        cq.Workplane("XY")
        .circle(0.070)
        .extrude(0.060)
        .translate((0.0, OPENING_Y, tub_bottom - 0.010))
    )
    return tub_wall.union(lower_bowl).union(drive_hub)


def _wash_basket():
    outer_radius = 0.215
    inner_radius = 0.202
    height = 0.570
    wall = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)

    bottom = cq.Workplane("XY").circle(outer_radius).extrude(0.034)
    top_roll = (
        cq.Workplane("XY")
        .circle(outer_radius + 0.010)
        .circle(inner_radius - 0.004)
        .extrude(0.035)
        .translate((0.0, 0.0, height - 0.018))
    )
    agitator = cq.Workplane("XY").circle(0.060).extrude(0.230).union(
        cq.Workplane("XY").circle(0.040).extrude(0.210).translate((0.0, 0.0, 0.210))
    )
    for idx in range(4):
        angle = 90.0 * idx
        vane = (
            cq.Workplane("XY")
            .box(0.026, 0.126, 0.245)
            .translate((0.0, 0.065, 0.145))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        agitator = agitator.union(vane)

    return wall.union(bottom).union(top_roll).union(agitator)


def _lid_shell():
    lid_depth = 0.565
    lid_width = 0.600
    lid_thickness = 0.048

    panel = cq.Workplane("XY").box(lid_depth, lid_width, lid_thickness).translate(
        (lid_depth / 2.0, 0.0, lid_thickness / 2.0)
    )
    rear_bar = _cq_box((0.055, lid_width, 0.028), (0.0275, 0.0, lid_thickness + 0.014))
    front_lip = _cq_box((0.038, lid_width * 0.86, 0.030), (lid_depth - 0.019, 0.0, 0.030))
    side_rib_0 = _cq_box((lid_depth * 0.78, 0.026, 0.018), (lid_depth * 0.50, -lid_width / 2.0 + 0.033, lid_thickness + 0.009))
    side_rib_1 = _cq_box((lid_depth * 0.78, 0.026, 0.018), (lid_depth * 0.50, lid_width / 2.0 - 0.033, lid_thickness + 0.009))

    # Hinge barrels run along the lid width in local Y and share the lid frame's
    # hinge line at local X=0.
    barrel_0 = (
        cq.Workplane("XZ")
        .circle(0.017)
        .extrude(0.150)
        .translate((0.0, -0.275, 0.024))
    )
    barrel_1 = (
        cq.Workplane("XZ")
        .circle(0.017)
        .extrude(0.150)
        .translate((0.0, 0.125, 0.024))
    )

    return _union_all([panel, rear_bar, front_lip, side_rib_0, side_rib_1, barrel_0, barrel_1])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_top_load_washer")

    warm_white = model.material("warm_white_enamel", rgba=(0.88, 0.90, 0.88, 1.0))
    dark_panel = model.material("dark_control_panel", rgba=(0.045, 0.050, 0.055, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.68, 1.0))
    shadow = model.material("deep_tub_shadow", rgba=(0.025, 0.028, 0.030, 1.0))
    soft_grey = model.material("soft_grey_plastic", rgba=(0.62, 0.65, 0.63, 1.0))
    green = model.material("start_green", rgba=(0.06, 0.52, 0.22, 1.0))
    red = model.material("cancel_red", rgba=(0.68, 0.07, 0.055, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_washer_cabinet_shell(), "cabinet_shell", tolerance=0.002),
        material=warm_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_geometry(CylinderGeometry(OPENING_RADIUS + 0.025, 0.600, radial_segments=64, closed=False), "tub_wall"),
        origin=Origin(xyz=(0.0, OPENING_Y, CABINET_HEIGHT - 0.300)),
        material=stainless,
        name="tub_wall",
    )
    cabinet.visual(
        Cylinder(radius=OPENING_RADIUS + 0.025, length=0.020),
        origin=Origin(xyz=(0.0, OPENING_Y, 0.280)),
        material=shadow,
        name="tub_floor",
    )
    cabinet.visual(
        Box((0.585, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, CONSOLE_FRONT_Y + 0.006, TOP_Z + 0.120)),
        material=dark_panel,
        name="console_face",
    )
    cabinet.visual(
        Box((0.170, 0.108, 0.012)),
        origin=Origin(xyz=(-0.160, OPENING_Y + OPENING_RADIUS + 0.010, TOP_Z + 0.006)),
        material=soft_grey,
        name="bleach_pad",
    )
    cabinet.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, OPENING_Y, 0.305), rpy=(0.0, 0.0, 0.0)),
        material=shadow,
        name="drive_shadow",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell", tolerance=0.002),
        material=warm_white,
        name="lid_shell",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_wash_basket(), "basket_shell", tolerance=0.0015),
        material=stainless,
        name="basket_shell",
    )

    knob_geom = KnobGeometry(
        0.092,
        0.044,
        body_style="skirted",
        top_diameter=0.066,
        grip=KnobGrip(style="fluted", count=24, depth=0.0020),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    cycle_knob = model.part("cycle_knob")
    cycle_knob.visual(
        mesh_from_geometry(knob_geom, "cycle_knob"),
        origin=Origin(xyz=(0.0, 0.0, -0.04532)),
        material=soft_grey,
        name="knob_body",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=green,
        name="button_face",
    )

    cancel_button = model.part("cancel_button")
    cancel_button.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=red,
        name="button_face",
    )

    bleach_cap = model.part("bleach_cap")
    bleach_cap.visual(
        Cylinder(radius=0.041, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=soft_grey,
        name="cap_body",
    )
    bleach_cap.visual(
        Box((0.012, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=soft_grey,
        name="cap_grip",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.285, TOP_Z + 0.010), rpy=(0.0, 0.0, -math.pi / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=0.0, upper=1.85),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, OPENING_Y, 0.330)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=18.0),
    )
    model.articulation(
        "cycle_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=cycle_knob,
        origin=Origin(xyz=(0.0, CONSOLE_FRONT_Y, TOP_Z + 0.135), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "start_button_press",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=start_button,
        origin=Origin(xyz=(-0.125, CONSOLE_FRONT_Y, TOP_Z + 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.35, lower=0.0, upper=0.012),
    )
    model.articulation(
        "cancel_button_press",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=cancel_button,
        origin=Origin(xyz=(0.125, CONSOLE_FRONT_Y, TOP_Z + 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.35, lower=0.0, upper=0.012),
    )
    model.articulation(
        "bleach_cap_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=bleach_cap,
        origin=Origin(xyz=(-0.160, OPENING_Y + OPENING_RADIUS + 0.010, TOP_Z + 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def coord(vec, index: int) -> float:
        try:
            return vec[index]
        except TypeError:
            return (vec.x, vec.y, vec.z)[index]

    def aabb_size(aabb, index: int) -> float:
        return coord(aabb[1], index) - coord(aabb[0], index)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    cycle_knob = object_model.get_part("cycle_knob")
    start_button = object_model.get_part("start_button")
    cancel_button = object_model.get_part("cancel_button")
    bleach_cap = object_model.get_part("bleach_cap")

    lid_hinge = object_model.get_articulation("lid_hinge")
    basket_spin = object_model.get_articulation("basket_spin")
    knob_spin = object_model.get_articulation("cycle_knob_spin")
    start_press = object_model.get_articulation("start_button_press")
    cancel_press = object_model.get_articulation("cancel_button_press")
    cap_spin = object_model.get_articulation("bleach_cap_spin")

    ctx.check(
        "utility laundry scale",
        (cab_aabb := ctx.part_world_aabb(cabinet)) is not None
        and aabb_size(cab_aabb, 0) > 0.64
        and aabb_size(cab_aabb, 1) > 0.78
        and aabb_size(cab_aabb, 2) > 1.12,
        details=f"cabinet_aabb={cab_aabb}",
    )
    ctx.check(
        "primary mechanisms present",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and start_press.articulation_type == ArticulationType.PRISMATIC
        and cancel_press.articulation_type == ArticulationType.PRISMATIC
        and cap_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=str(
            [
                lid_hinge.articulation_type,
                basket_spin.articulation_type,
                knob_spin.articulation_type,
                start_press.articulation_type,
                cancel_press.articulation_type,
                cap_spin.articulation_type,
            ]
        ),
    )
    ctx.check(
        "lid opens upward on rear hinge",
        lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.5,
        details=f"limits={lid_hinge.motion_limits}",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.45}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid pose lifts the front edge",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and coord(opened_lid_aabb[1], 2) > coord(closed_lid_aabb[1], 2) + 0.32,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    ctx.expect_within(
        basket,
        cabinet,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="tub_wall",
        margin=0.002,
        name="basket remains centered in tub opening",
    )
    ctx.expect_overlap(
        basket,
        cabinet,
        axes="z",
        elem_a="basket_shell",
        elem_b="tub_wall",
        min_overlap=0.50,
        name="basket extends deep into hollow tub",
    )
    ctx.expect_gap(
        basket,
        cabinet,
        axis="z",
        positive_elem="basket_shell",
        negative_elem="drive_shadow",
        max_gap=0.003,
        max_penetration=0.0,
        name="basket is supported by the lower drive hub",
    )

    ctx.expect_gap(
        cabinet,
        cycle_knob,
        axis="y",
        positive_elem="console_face",
        negative_elem="knob_body",
        max_gap=0.003,
        max_penetration=0.0,
        name="cycle knob sits proud of console face",
    )
    ctx.expect_gap(
        cabinet,
        start_button,
        axis="y",
        positive_elem="console_face",
        negative_elem="button_face",
        max_gap=0.003,
        max_penetration=0.0,
        name="start button is seated on console face",
    )
    ctx.expect_gap(
        cabinet,
        cancel_button,
        axis="y",
        positive_elem="console_face",
        negative_elem="button_face",
        max_gap=0.003,
        max_penetration=0.0,
        name="cancel button is seated on console face",
    )
    ctx.expect_gap(
        bleach_cap,
        cabinet,
        axis="z",
        positive_elem="cap_body",
        negative_elem="bleach_pad",
        max_gap=0.003,
        max_penetration=0.0,
        name="bleach cap sits on rear dispenser pad",
    )

    for joint, button_name in ((start_press, "start"), (cancel_press, "cancel")):
        button_part = object_model.get_part(joint.child) if isinstance(joint.child, str) else joint.child
        rest = ctx.part_world_position(button_part)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else 0.0
        with ctx.pose({joint: upper}):
            pressed = ctx.part_world_position(button_part)
        ctx.check(
            f"{button_name} button pushes inward",
            rest is not None and pressed is not None and coord(pressed, 1) > coord(rest, 1) + 0.009,
            details=f"rest={rest}, pressed={pressed}, upper={upper}",
        )

    return ctx.report()


object_model = build_object_model()
