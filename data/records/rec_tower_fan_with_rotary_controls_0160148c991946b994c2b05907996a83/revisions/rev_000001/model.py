from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_disc(outer_radius: float, inner_radius: float, height: float):
    """A simple bearing trim ring with a real center clearance."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _top_cap_with_handle(width: float, depth: float, height: float):
    """Rounded top cap with a rear carry slot and a shallow control recess."""
    cap = cq.Workplane("XY").box(width, depth, height)

    # Rear-facing carry slot: a real opening under a molded bridge, with side
    # cheeks left intact so the cap reads as a supported handle rather than a
    # painted groove.
    slot = (
        cq.Workplane("XY")
        .box(width * 0.58, depth * 0.72, height * 0.42)
        .translate((0.0, depth * 0.34, -height * 0.10))
    )
    cap = cap.cut(slot)

    # Shallow oval-ish rectangular pocket for the rotary controls.
    recess = (
        cq.Workplane("XY")
        .box(width * 0.72, depth * 0.40, height * 0.45)
        .translate((0.0, -depth * 0.24, height * 0.42))
    )
    cap = cap.cut(recess)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_tower_fan")

    warm_white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    satin_grey = model.material("satin_grey", rgba=(0.50, 0.52, 0.54, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.060, 0.065, 1.0))
    grille_black = model.material("grille_black", rgba=(0.010, 0.012, 0.014, 1.0))
    rubber = model.material("rubber", rgba=(0.025, 0.023, 0.022, 1.0))
    knob_grey = model.material("knob_grey", rgba=(0.70, 0.72, 0.72, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.16, 0.32, 0.42, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.135, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=warm_white,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.118, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber,
        name="rubber_foot",
    )
    base.visual(
        mesh_from_cadquery(_annular_disc(0.070, 0.052, 0.008), "bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_grey,
        name="bearing_ring",
    )

    tower = model.part("tower")
    body_width = 0.170
    body_depth = 0.110
    body_height = 0.450
    wall = 0.012
    bottom_cap_h = 0.024
    body_bottom = 0.030
    top_cap_h = 0.038

    # Rotating neck and lower body, deliberately visible above the base so the
    # oscillation axis reads as a mechanical turntable.
    tower.visual(
        Cylinder(radius=0.046, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_grey,
        name="turntable_neck",
    )
    tower.visual(
        Box((body_width, body_depth, bottom_cap_h)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom + bottom_cap_h * 0.5)),
        material=warm_white,
        name="bottom_cap",
    )
    tower.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + wall * 0.5, 0.0, body_bottom + bottom_cap_h + body_height * 0.5)),
        material=warm_white,
        name="side_shell_0",
    )
    tower.visual(
        Box((wall, body_depth, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - wall * 0.5, 0.0, body_bottom + bottom_cap_h + body_height * 0.5)),
        material=warm_white,
        name="side_shell_1",
    )
    tower.visual(
        Box((body_width, wall, body_height)),
        origin=Origin(xyz=(0.0, body_depth * 0.5 - wall * 0.5, body_bottom + bottom_cap_h + body_height * 0.5)),
        material=warm_white,
        name="rear_shell",
    )

    grille_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.154, 0.385),
            0.004,
            slot_size=(0.055, 0.010),
            pitch=(0.017, 0.069),
            frame=0.008,
            corner_radius=0.006,
            slot_angle_deg=88.0,
            stagger=True,
        ),
        "front_slot_grille",
    )
    tower.visual(
        grille_mesh,
        origin=Origin(
            xyz=(0.0, -body_depth * 0.5 - 0.0015, body_bottom + bottom_cap_h + 0.225),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=grille_black,
        name="front_grille",
    )

    top_z = body_bottom + bottom_cap_h + body_height + top_cap_h * 0.5
    tower.visual(
        mesh_from_cadquery(_top_cap_with_handle(body_width, body_depth, top_cap_h), "top_cap_handle"),
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=warm_white,
        name="top_cap",
    )
    tower.visual(
        Box((0.126, 0.048, 0.004)),
        origin=Origin(xyz=(0.0, -0.027, top_z + 0.009)),
        material=dark_plastic,
        name="control_recess",
    )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.040,
                0.018,
                0.360,
                30,
                blade_thickness=0.0022,
                blade_sweep_deg=32.0,
                backplate=True,
                shroud=True,
            ),
            "vertical_blower_wheel",
        ),
        material=accent_blue,
        name="blower_cage",
    )
    blower_wheel.visual(
        Cylinder(radius=0.006, length=0.390),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_grey,
        name="center_shaft",
    )
    blower_wheel.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.178)),
        material=satin_grey,
        name="lower_hub",
    )
    blower_wheel.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material=satin_grey,
        name="upper_hub",
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.016,
                body_style="faceted",
                top_diameter=0.027,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        material=knob_grey,
        name="knob_cap",
    )
    speed_knob.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_plastic,
        name="knob_stem",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.028,
                0.014,
                body_style="domed",
                edge_radius=0.0008,
                grip=KnobGrip(style="fluted", count=18, depth=0.0007),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_knob",
        ),
        material=knob_grey,
        name="knob_cap",
    )
    timer_knob.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_plastic,
        name="knob_stem",
    )

    model.articulation(
        "oscillation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "blower_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower_wheel,
        origin=Origin(xyz=(0.0, -0.015, body_bottom + bottom_cap_h + 0.225)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )

    knob_z = top_z + top_cap_h * 0.5 - 0.003
    model.articulation(
        "speed_rotate",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=speed_knob,
        origin=Origin(xyz=(-0.032, -0.028, knob_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "timer_rotate",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=timer_knob,
        origin=Origin(xyz=(0.032, -0.028, knob_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    blower = object_model.get_part("blower_wheel")
    speed = object_model.get_part("speed_knob")
    timer = object_model.get_part("timer_knob")

    oscillation = object_model.get_articulation("oscillation")
    blower_spin = object_model.get_articulation("blower_spin")
    speed_rotate = object_model.get_articulation("speed_rotate")
    timer_rotate = object_model.get_articulation("timer_rotate")

    ctx.expect_contact(
        tower,
        base,
        elem_a="turntable_neck",
        elem_b="base_disc",
        name="turntable neck sits on the round base",
    )
    ctx.expect_within(
        blower,
        tower,
        axes="xz",
        margin=0.002,
        elem_a="blower_cage",
        name="vertical blower cage stays inside tower envelope",
    )
    ctx.expect_within(
        speed,
        tower,
        axes="xy",
        margin=0.003,
        elem_a="knob_stem",
        outer_elem="control_recess",
        name="speed knob stem is centered in recessed controls",
    )
    ctx.expect_within(
        timer,
        tower,
        axes="xy",
        margin=0.003,
        elem_a="knob_stem",
        outer_elem="control_recess",
        name="timer knob stem is centered in recessed controls",
    )

    ctx.check(
        "oscillation is a limited vertical revolute joint",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and tuple(oscillation.axis) == (0.0, 0.0, 1.0)
        and oscillation.motion_limits is not None
        and oscillation.motion_limits.lower is not None
        and oscillation.motion_limits.upper is not None
        and oscillation.motion_limits.lower < 0.0
        and oscillation.motion_limits.upper > 0.0,
        details=f"type={oscillation.articulation_type}, axis={oscillation.axis}, limits={oscillation.motion_limits}",
    )
    ctx.check(
        "blower and both rotary controls spin continuously about vertical axes",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            for joint in (blower_spin, speed_rotate, timer_rotate)
        ),
        details=f"blower={blower_spin}, speed={speed_rotate}, timer={timer_rotate}",
    )

    rest_aabb = ctx.part_element_world_aabb(tower, elem="front_grille")
    with ctx.pose({oscillation: 0.65}):
        turned_aabb = ctx.part_element_world_aabb(tower, elem="front_grille")

    def _aabb_center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    rest_x = _aabb_center_x(rest_aabb)
    turned_x = _aabb_center_x(turned_aabb)
    ctx.check(
        "front grille swings with oscillation pose",
        rest_x is not None and turned_x is not None and abs(turned_x - rest_x) > 0.020,
        details=f"rest_x={rest_x}, turned_x={turned_x}",
    )

    return ctx.report()


object_model = build_object_model()
