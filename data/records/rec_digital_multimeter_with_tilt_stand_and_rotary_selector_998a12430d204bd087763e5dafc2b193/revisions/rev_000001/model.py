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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


CASE_W = 0.092
CASE_D = 0.036
CASE_H = 0.172
PANEL_T = 0.0014
FRONT_Y = -CASE_D / 2.0
PANEL_FRONT_Y = FRONT_Y - PANEL_T


def _y_cylinder(radius: float, length: float, x: float, z: float):
    """A cutter cylinder whose axis is the model Y direction."""
    return cq.Workplane("XZ").center(x, z).circle(radius).extrude(length / 2.0, both=True)


def _y_box(width: float, height: float, length: float, x: float, z: float):
    """A rectangular cutter whose long direction is the model Y direction."""
    return cq.Workplane("XZ").center(x, z).rect(width, height).extrude(length / 2.0, both=True)


def _rounded_slab(width: float, height: float, depth: float, corner_radius: float):
    slab = cq.Workplane("XY").box(width, depth, height).edges("|Y").fillet(corner_radius)
    return slab.translate((0.0, 0.0, height / 2.0))


def _case_shell():
    shell = _rounded_slab(CASE_W, CASE_H, CASE_D, 0.012)

    # Through-clearances for the dial shaft and for the pressable button caps.
    for x, z, r in (
        (0.0, 0.090, 0.0115),
        (-0.014, 0.052, 0.0068),
        (0.014, 0.052, 0.0068),
    ):
        shell = shell.cut(_y_cylinder(r, CASE_D * 3.0, x, z))
    for x, z in ((-0.014, 0.033), (0.014, 0.033)):
        shell = shell.cut(_y_box(0.012, 0.012, CASE_D * 3.0, x, z))

    return shell


def _front_panel():
    panel = _rounded_slab(0.078, 0.150, PANEL_T, 0.008).translate(
        (0.0, FRONT_Y - PANEL_T / 2.0, 0.010)
    )
    for x, z, r in (
        (0.0, 0.090, 0.0115),
        (-0.014, 0.052, 0.0068),
        (0.014, 0.052, 0.0068),
        (-0.018, 0.013, 0.006),
        (0.018, 0.013, 0.006),
    ):
        panel = panel.cut(_y_cylinder(r, CASE_D * 3.0, x, z))
    for x, z in ((-0.014, 0.033), (0.014, 0.033)):
        panel = panel.cut(_y_box(0.012, 0.012, CASE_D * 3.0, x, z))
    panel = panel.cut(_y_box(0.055, 0.026, CASE_D * 3.0, 0.0, 0.136))
    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_electronics_multimeter")

    case_plastic = model.material("case_plastic", rgba=(0.94, 0.60, 0.14, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.035, 0.037, 0.040, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.015, 0.015, 0.016, 1.0))
    display_glass = model.material("display_glass", rgba=(0.28, 0.40, 0.38, 1.0))
    dial_gray = model.material("dial_gray", rgba=(0.18, 0.18, 0.19, 1.0))
    marker_white = model.material("marker_white", rgba=(0.92, 0.92, 0.88, 1.0))
    button_blue = model.material("button_blue", rgba=(0.07, 0.27, 0.70, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.95, 0.75, 0.09, 1.0))
    button_red = model.material("button_red", rgba=(0.72, 0.06, 0.04, 1.0))
    button_gray = model.material("button_gray", rgba=(0.24, 0.25, 0.26, 1.0))
    metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.17, 1.0))

    case = model.part("case")
    case.visual(
        mesh_from_cadquery(_case_shell(), "rounded_case", tolerance=0.0008),
        material=case_plastic,
        name="rounded_case",
    )
    # Rear hinge knuckles and molded bosses for the folding stand.
    for x in (-0.037, 0.037):
        case.visual(
            Box((0.018, 0.007, 0.010)),
            origin=Origin(xyz=(x, CASE_D / 2.0 + 0.0018, 0.032)),
            material=case_plastic,
            name=f"stand_boss_{'n' if x < 0 else 'p'}",
        )
        case.visual(
            Cylinder(radius=0.0042, length=0.014),
            origin=Origin(xyz=(x, CASE_D / 2.0 + 0.0050, 0.032), rpy=(0.0, pi / 2.0, 0.0)),
            material=case_plastic,
            name=f"stand_knuckle_{'n' if x < 0 else 'p'}",
        )

    front_panel = model.part("front_panel")
    front_panel.visual(
        mesh_from_cadquery(_front_panel(), "front_panel", tolerance=0.0006),
        material=dark_plastic,
        name="front_panel",
    )

    display = model.part("display")
    display.visual(
        Box((0.061, 0.0010, 0.030)),
        origin=Origin(xyz=(0.0, PANEL_FRONT_Y - 0.0005, 0.136)),
        material=display_glass,
        name="lcd_window",
    )
    display.visual(
        Box((0.034, 0.0006, 0.003)),
        origin=Origin(xyz=(0.0, PANEL_FRONT_Y - 0.0011, 0.140)),
        material=marker_white,
        name="lcd_digits",
    )

    dial = model.part("dial")
    dial_knob = KnobGeometry(
        0.046,
        0.008,
        body_style="faceted",
        top_diameter=0.040,
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=20, depth=0.0007, width=0.0014),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=90.0),
        center=False,
    )
    dial.visual(
        mesh_from_geometry(dial_knob, "function_dial"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial_gray,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.0085, length=0.007),
        origin=Origin(xyz=(0.0, 0.0035, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="dial_shaft",
    )
    dial.visual(
        Box((0.0040, 0.0008, 0.016)),
        origin=Origin(xyz=(0.0, -0.0082, 0.012)),
        material=marker_white,
        name="dial_pointer",
    )

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.0032, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="stand_barrel",
    )
    stand.visual(
        Box((0.006, 0.006, 0.100)),
        origin=Origin(xyz=(-0.020, 0.0025, 0.049)),
        material=dark_plastic,
        name="stand_leg_0",
    )
    stand.visual(
        Box((0.006, 0.006, 0.100)),
        origin=Origin(xyz=(0.020, 0.0025, 0.049)),
        material=dark_plastic,
        name="stand_leg_1",
    )
    stand.visual(
        Box((0.050, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.0025, 0.094)),
        material=dark_plastic,
        name="stand_crossbar",
    )
    stand.visual(
        Box((0.058, 0.008, 0.005)),
        origin=Origin(xyz=(0.0, 0.004, 0.101)),
        material=dark_plastic,
        name="stand_foot",
    )

    def add_round_button(name: str, x: float, z: float, mat: Material) -> None:
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.0080, length=0.0040),
            origin=Origin(xyz=(0.0, -0.0020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.0040, length=0.0060),
            origin=Origin(xyz=(0.0, 0.0030, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name="button_stem",
        )
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=front_panel,
            child=button,
            origin=Origin(xyz=(x, PANEL_FRONT_Y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.0025),
        )

    def add_square_button(name: str, x: float, z: float, mat: Material) -> None:
        button = model.part(name)
        button.visual(
            Box((0.014, 0.0040, 0.014)),
            origin=Origin(xyz=(0.0, -0.0020, 0.0)),
            material=mat,
            name="button_cap",
        )
        button.visual(
            Box((0.006, 0.0060, 0.006)),
            origin=Origin(xyz=(0.0, 0.0030, 0.0)),
            material=rubber_black,
            name="button_stem",
        )
        model.articulation(
            f"{name}_press",
            ArticulationType.PRISMATIC,
            parent=front_panel,
            child=button,
            origin=Origin(xyz=(x, PANEL_FRONT_Y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.0025),
        )

    add_round_button("round_button_0", -0.014, 0.052, button_blue)
    add_round_button("round_button_1", 0.014, 0.052, button_yellow)
    add_square_button("square_button_0", -0.014, 0.033, button_red)
    add_square_button("square_button_1", 0.014, 0.033, button_gray)

    # Static input-jack rims at the bottom of the panel complete the multimeter
    # face while keeping the prompt-requested buttons as the only press controls.
    for index, x in enumerate((-0.018, 0.018)):
        jack = model.part(f"jack_{index}")
        jack.visual(
            Cylinder(radius=0.0075, length=0.0014),
            origin=Origin(xyz=(x, PANEL_FRONT_Y - 0.0007, 0.013), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="jack_ring",
        )
        model.articulation(
            f"jack_{index}_mount",
            ArticulationType.FIXED,
            parent=front_panel,
            child=jack,
            origin=Origin(),
        )

    model.articulation(
        "front_panel_mount",
        ArticulationType.FIXED,
        parent=case,
        child=front_panel,
        origin=Origin(),
    )
    model.articulation(
        "display_mount",
        ArticulationType.FIXED,
        parent=front_panel,
        child=display,
        origin=Origin(),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=front_panel,
        child=dial,
        origin=Origin(xyz=(0.0, PANEL_FRONT_Y, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.45, velocity=6.0),
    )
    model.articulation(
        "stand_hinge",
        ArticulationType.REVOLUTE,
        parent=case,
        child=stand,
        origin=Origin(xyz=(0.0, CASE_D / 2.0 + 0.0050, 0.032)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=0.0, upper=1.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    dial_joint = object_model.get_articulation("dial_spin")
    stand_joint = object_model.get_articulation("stand_hinge")
    ctx.check(
        "function dial rotates continuously",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.axis == (0.0, 1.0, 0.0)
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}, limits={dial_joint.motion_limits}",
    )
    ctx.check(
        "rear stand has outward hinge travel",
        stand_joint.articulation_type == ArticulationType.REVOLUTE
        and stand_joint.motion_limits is not None
        and stand_joint.motion_limits.lower == 0.0
        and stand_joint.motion_limits.upper is not None
        and stand_joint.motion_limits.upper > 1.4,
        details=f"type={stand_joint.articulation_type}, limits={stand_joint.motion_limits}",
    )

    stand = object_model.get_part("stand")
    with ctx.pose({stand_joint: 0.0}):
        folded_aabb = ctx.part_world_aabb(stand)
    with ctx.pose({stand_joint: 1.45}):
        open_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "stand swings rearward from the lower hinge",
        folded_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > folded_aabb[1][1] + 0.070
        and open_aabb[1][2] < folded_aabb[1][2] - 0.050,
        details=f"folded={folded_aabb}, open={open_aabb}",
    )

    button_names = ("round_button_0", "round_button_1", "square_button_0", "square_button_1")
    for button_name in button_names:
        joint = object_model.get_articulation(f"{button_name}_press")
        button = object_model.get_part(button_name)
        limits = joint.motion_limits
        ctx.check(
            f"{button_name} has independent prismatic press",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.004,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        with ctx.pose({joint: 0.0}):
            rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: limits.upper if limits is not None and limits.upper is not None else 0.0}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{button_name} moves inward when pressed",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    dial_pos = ctx.part_world_position(object_model.get_part("dial"))
    ctx.check(
        "dial is centered above the control cluster",
        dial_pos is not None and abs(dial_pos[0]) < 0.002 and 0.080 < dial_pos[2] < 0.100,
        details=f"dial_pos={dial_pos}",
    )

    return ctx.report()


object_model = build_object_model()
