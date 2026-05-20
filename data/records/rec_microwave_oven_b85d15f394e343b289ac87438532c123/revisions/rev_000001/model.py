from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CREAM = Material("warm_cream_enamel", rgba=(0.92, 0.82, 0.62, 1.0))
TAN = Material("tan_panel", rgba=(0.70, 0.55, 0.36, 1.0))
DARK = Material("dark_cavity", rgba=(0.03, 0.028, 0.025, 1.0))
GLASS = Material("smoked_glass", rgba=(0.04, 0.10, 0.12, 0.46))
TURNTABLE_GLASS = Material("pale_green_glass", rgba=(0.72, 0.94, 0.86, 0.52))
CHROME = Material("brushed_chrome", rgba=(0.68, 0.66, 0.60, 1.0))
BLACK = Material("black_bakelite", rgba=(0.025, 0.022, 0.020, 1.0))
WHITE = Material("white_indicator", rgba=(0.96, 0.93, 0.84, 1.0))


def _rounded_plate_mesh(width: float, height: float, depth: float, radius: float, name: str):
    """Rounded-rectangle plate whose broad face lies in local XZ and depth in Y."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=10),
        depth,
        cap=True,
        center=True,
    )
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="retro_windowed_microwave",
        materials=(CREAM, TAN, DARK, GLASS, TURNTABLE_GLASS, CHROME, BLACK, WHITE),
    )

    housing = model.part("housing")

    # Overall appliance footprint: about 0.58 m wide, 0.40 m deep, 0.34 m tall,
    # standing on small feet.  The cavity side is deliberately open behind the
    # door so the turntable and dark oven cavity are not embedded in a solid box.
    housing.visual(Box((0.58, 0.40, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0475)), material=CREAM, name="bottom_shell")
    housing.visual(Box((0.58, 0.40, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.3525)), material=CREAM, name="top_shell")
    housing.visual(Box((0.035, 0.40, 0.34)), origin=Origin(xyz=(-0.2725, 0.0, 0.20)), material=CREAM, name="hinge_side_wall")
    housing.visual(Box((0.150, 0.40, 0.34)), origin=Origin(xyz=(0.215, 0.0, 0.20)), material=CREAM, name="control_side_wall")
    housing.visual(Box((0.58, 0.035, 0.34)), origin=Origin(xyz=(0.0, 0.1825, 0.20)), material=CREAM, name="rear_wall")

    # Rounded retro front corners as vertical enamel radius caps, slightly
    # tucked into the side walls so they read as molded rounded corners.
    housing.visual(Cylinder(radius=0.018, length=0.335), origin=Origin(xyz=(-0.272, -0.192, 0.20)), material=CREAM, name="front_corner_0")
    housing.visual(Cylinder(radius=0.018, length=0.335), origin=Origin(xyz=(0.272, -0.192, 0.20)), material=CREAM, name="front_corner_1")

    # Fixed dark cavity behind the windowed door: rear wall, floor, ceiling and
    # side reveals are fixed to the housing and leave a real free space for the
    # rotating glass turntable.
    housing.visual(Box((0.370, 0.018, 0.225)), origin=Origin(xyz=(-0.060, 0.045, 0.195)), material=DARK, name="cavity_rear")
    housing.visual(Box((0.370, 0.230, 0.014)), origin=Origin(xyz=(-0.060, -0.070, 0.082)), material=DARK, name="cavity_floor")
    housing.visual(Box((0.370, 0.230, 0.014)), origin=Origin(xyz=(-0.060, -0.070, 0.308)), material=DARK, name="cavity_ceiling")
    housing.visual(Box((0.020, 0.230, 0.225)), origin=Origin(xyz=(-0.247, -0.070, 0.195)), material=DARK, name="cavity_side_0")
    housing.visual(Box((0.030, 0.230, 0.225)), origin=Origin(xyz=(0.130, -0.070, 0.195)), material=DARK, name="cavity_side_1")
    housing.visual(Box((0.030, 0.012, 0.250)), origin=Origin(xyz=(-0.257, -0.204, 0.195)), material=CREAM, name="front_hinge_stile")
    housing.visual(Box((0.385, 0.012, 0.030)), origin=Origin(xyz=(-0.060, -0.204, 0.322)), material=CREAM, name="front_top_lip")
    housing.visual(Box((0.385, 0.012, 0.030)), origin=Origin(xyz=(-0.060, -0.204, 0.068)), material=CREAM, name="front_bottom_lip")

    # Raised tan control panel and the four-sided guide that clips the release
    # button so it slides in a shallow sleeve instead of separating from the
    # housing.
    housing.visual(_rounded_plate_mesh(0.116, 0.275, 0.014, 0.018, "control_panel_mesh"), origin=Origin(xyz=(0.214, -0.193, 0.197)), material=TAN, name="control_panel")
    housing.visual(Box((0.090, 0.054, 0.012)), origin=Origin(xyz=(0.214, -0.226, 0.166)), material=CHROME, name="release_guide_top")
    housing.visual(Box((0.090, 0.054, 0.012)), origin=Origin(xyz=(0.214, -0.226, 0.108)), material=CHROME, name="release_guide_bottom")
    housing.visual(Box((0.012, 0.054, 0.070)), origin=Origin(xyz=(0.163, -0.226, 0.137)), material=CHROME, name="release_guide_0")
    housing.visual(Box((0.012, 0.054, 0.070)), origin=Origin(xyz=(0.265, -0.226, 0.137)), material=CHROME, name="release_guide_1")

    # Small fixed decals around the controls give the panel a retro appliance
    # scale without introducing extra mechanisms.
    housing.visual(Box((0.044, 0.003, 0.006)), origin=Origin(xyz=(0.214, -0.2015, 0.326)), material=WHITE, name="timer_label")
    housing.visual(Box((0.044, 0.003, 0.006)), origin=Origin(xyz=(0.214, -0.2015, 0.244)), material=WHITE, name="power_label")

    # Feet are intentionally fused into the root housing and touch the bottom
    # shell, avoiding isolated little blocks.
    for i, x in enumerate((-0.22, 0.22)):
        for j, y in enumerate((-0.135, 0.135)):
            housing.visual(Box((0.070, 0.050, 0.030)), origin=Origin(xyz=(x, y, 0.015)), material=BLACK, name=f"foot_{i}_{j}")

    door = model.part("door")
    door_frame = BezelGeometry(
        (0.255, 0.155),
        (0.390, 0.268),
        0.032,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.034,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    door.visual(mesh_from_geometry(door_frame, "door_frame_mesh"), origin=Origin(xyz=(0.195, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=CREAM, name="door_frame")
    door.visual(Box((0.280, 0.006, 0.178)), origin=Origin(xyz=(0.195, -0.038, 0.0)), material=GLASS, name="window_glass")
    door.visual(Cylinder(radius=0.012, length=0.255), origin=Origin(xyz=(0.0, -0.018, 0.0)), material=CHROME, name="hinge_barrel")
    door.visual(Box((0.030, 0.014, 0.220)), origin=Origin(xyz=(0.018, -0.018, 0.0)), material=CHROME, name="hinge_leaf")
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.246, -0.208, 0.195)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.8, lower=0.0, upper=1.75),
    )

    turntable = model.part("turntable")
    turntable.visual(Cylinder(radius=0.106, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=TURNTABLE_GLASS, name="glass_plate")
    turntable.visual(Cylinder(radius=0.019, length=0.030), origin=Origin(xyz=(0.0, 0.0, -0.010)), material=CHROME, name="central_spindle")
    turntable.visual(Cylinder(radius=0.081, length=0.004), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=CHROME, name="roller_ring")
    model.articulation(
        "housing_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=turntable,
        origin=Origin(xyz=(-0.060, -0.070, 0.092)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.034,
            body_style="skirted",
            top_diameter=0.042,
            skirt=KnobSkirt(0.062, 0.006, flare=0.08, chamfer=0.0015),
            grip=KnobGrip(style="fluted", count=20, depth=0.0016),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0010, angle_deg=0.0),
        ),
        "retro_knob_mesh",
    )
    timer_knob = model.part("timer_knob")
    timer_knob.visual(knob_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=BLACK, name="timer_dial")
    timer_knob.visual(Cylinder(radius=0.006, length=0.028), origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=CHROME, name="timer_shaft")
    model.articulation(
        "housing_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=timer_knob,
        origin=Origin(xyz=(0.214, -0.228, 0.273)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=math.tau),
    )

    power_knob = model.part("power_knob")
    power_knob.visual(knob_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=BLACK, name="power_dial")
    power_knob.visual(Cylinder(radius=0.006, length=0.028), origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=CHROME, name="power_shaft")
    model.articulation(
        "housing_to_power_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=power_knob,
        origin=Origin(xyz=(0.214, -0.228, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.5, lower=0.0, upper=4.7),
    )

    release_button = model.part("release_button")
    release_button.visual(_rounded_plate_mesh(0.055, 0.046, 0.018, 0.007, "release_button_mesh"), origin=Origin(xyz=(0.0, -0.012, 0.0)), material=CHROME, name="button_cap")
    release_button.visual(Box((0.034, 0.030, 0.020)), origin=Origin(xyz=(0.0, -0.025, 0.0)), material=CHROME, name="guide_tongue")
    model.articulation(
        "housing_to_release_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=release_button,
        origin=Origin(xyz=(0.214, -0.223, 0.137)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.25, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    release_button = object_model.get_part("release_button")
    door_joint = object_model.get_articulation("housing_to_door")
    turntable_joint = object_model.get_articulation("housing_to_turntable")
    button_joint = object_model.get_articulation("housing_to_release_button")

    ctx.allow_overlap(
        housing,
        turntable,
        elem_a="cavity_floor",
        elem_b="central_spindle",
        reason="The rotating central spindle is intentionally seated through the cavity floor drive socket.",
    )

    # The closed door is a large side-hinged windowed panel standing just proud
    # of the fixed cavity opening.
    ctx.expect_overlap(door, housing, axes="xz", elem_a="window_glass", elem_b="cavity_rear", min_overlap=0.12, name="window covers fixed cavity")
    ctx.expect_gap(housing, door, axis="y", positive_elem="front_hinge_stile", negative_elem="door_frame", max_penetration=0.001, max_gap=0.004, name="door stands proud of front trim")

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.20}):
        open_aabb = ctx.part_world_aabb(door)
    closed_front_y = closed_aabb[0][1] if closed_aabb else None
    open_front_y = open_aabb[0][1] if open_aabb else None
    ctx.check(
        "side hinge opens door outward",
        closed_front_y is not None and open_front_y is not None and open_front_y < closed_front_y - 0.05,
        details=f"closed_front_y={closed_front_y}, open_front_y={open_front_y}",
    )

    # The glass turntable is retained inside the fixed cavity and rotates about
    # a central vertical spindle.
    ctx.expect_within(turntable, housing, axes="xy", inner_elem="glass_plate", outer_elem="cavity_floor", margin=0.002, name="turntable stays within cavity floor")
    ctx.expect_gap(turntable, housing, axis="z", positive_elem="glass_plate", negative_elem="cavity_floor", min_gap=0.002, max_gap=0.020, name="turntable rides above cavity floor")
    ctx.expect_overlap(turntable, housing, axes="z", elem_a="central_spindle", elem_b="cavity_floor", min_overlap=0.010, name="spindle remains seated in floor socket")
    ctx.check("turntable uses continuous joint", turntable_joint.articulation_type == ArticulationType.CONTINUOUS, details=str(turntable_joint.articulation_type))

    # The release button remains captured by the front-panel guide throughout
    # its short inward prismatic travel.
    ctx.expect_contact(release_button, housing, elem_a="button_cap", elem_b="release_guide_top", contact_tol=0.001, name="release button clipped under top guide")
    ctx.expect_contact(release_button, housing, elem_a="button_cap", elem_b="release_guide_bottom", contact_tol=0.001, name="release button clipped above bottom guide")
    ctx.expect_overlap(release_button, housing, axes="y", elem_a="guide_tongue", elem_b="release_guide_0", min_overlap=0.018, name="release tongue retained at rest")
    with ctx.pose({button_joint: 0.018}):
        ctx.expect_overlap(release_button, housing, axes="y", elem_a="guide_tongue", elem_b="release_guide_0", min_overlap=0.018, name="release tongue retained when pressed")
        ctx.expect_gap(housing, release_button, axis="y", positive_elem="control_panel", negative_elem="button_cap", min_gap=0.001, max_gap=0.030, name="pressed button stops before panel face")

    return ctx.report()


object_model = build_object_model()
