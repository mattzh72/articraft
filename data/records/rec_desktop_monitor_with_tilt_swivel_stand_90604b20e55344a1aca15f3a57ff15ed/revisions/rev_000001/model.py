from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)
import cadquery as cq


def _rounded_box(width: float, depth: float, height: float, radius: float):
    """Small CadQuery rounded rectangular solid in meters."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(radius)
    )


def _hollow_sleeve(width: float, depth: float, height: float, inner_width: float, inner_depth: float):
    """Rectangular open-ended telescoping sleeve centered on the local origin."""
    outer = cq.Workplane("XY").box(width, depth, height)
    cutter = cq.Workplane("XY").box(inner_width, inner_depth, height + 0.012)
    return outer.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor_telescoping_stand")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.055, 0.058, 0.062, 1.0))
    charcoal = model.material("charcoal_shell", rgba=(0.095, 0.098, 0.105, 1.0))
    satin_grey = model.material("satin_grey", rgba=(0.23, 0.235, 0.24, 1.0))
    rubber = model.material("rubber_feet", rgba=(0.006, 0.006, 0.006, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.012, 0.018, 0.030, 1.0))

    # Root: low rounded pedestal with a raised turntable boss.  The footprint and
    # height are typical of a small office monitor rather than a TV stand.
    base = model.part("base")
    base_profile = rounded_rect_profile(0.300, 0.220, 0.060, corner_segments=12)
    base.visual(
        mesh_from_geometry(ExtrudeGeometry(base_profile, 0.035, center=True), "pedestal_base"),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_plastic,
        name="pedestal",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=charcoal,
        name="swivel_boss",
    )
    base.visual(
        Box((0.235, 0.150, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="rubber_pad",
    )

    # The whole upright, neck, and display rotate as a unit on this stand part.
    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.049, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=satin_grey,
        name="turntable",
    )
    sleeve_mesh = mesh_from_cadquery(
        _hollow_sleeve(0.096, 0.070, 0.230, 0.072, 0.048),
        "outer_sleeve",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    stand.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.035, 0.125)),
        material=satin_grey,
        name="outer_sleeve",
    )
    stand.visual(
        Box((0.078, 0.010, 0.205)),
        origin=Origin(xyz=(0.0, 0.074, 0.130)),
        material=charcoal,
        name="rear_raceway",
    )
    collar_mesh = mesh_from_cadquery(
        _hollow_sleeve(0.115, 0.076, 0.018, 0.073, 0.050),
        "sleeve_collar",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    stand.visual(
        collar_mesh,
        origin=Origin(xyz=(0.0, 0.035, 0.238)),
        material=satin_grey,
        name="sleeve_collar",
    )

    # The sliding neck is intentionally long: it remains captured inside the
    # sleeve even at the top of its height-travel.
    neck = model.part("neck")
    neck.visual(
        Box((0.064, 0.038, 0.320)),
        origin=Origin(xyz=(0.0, 0.035, 0.010)),
        material=charcoal,
        name="inner_mast",
    )
    neck.visual(
        Box((0.006, 0.032, 0.050)),
        origin=Origin(xyz=(-0.0345, 0.035, -0.100)),
        material=matte_black,
        name="guide_shoe_0",
    )
    neck.visual(
        Box((0.006, 0.032, 0.050)),
        origin=Origin(xyz=(0.0345, 0.035, -0.100)),
        material=matte_black,
        name="guide_shoe_1",
    )
    neck.visual(
        Box((0.160, 0.044, 0.060)),
        origin=Origin(xyz=(0.0, 0.004, 0.170)),
        material=satin_grey,
        name="head_block",
    )
    neck.visual(
        Box((0.020, 0.056, 0.072)),
        origin=Origin(xyz=(-0.075, -0.024, 0.180)),
        material=satin_grey,
        name="hinge_ear_0",
    )
    neck.visual(
        Box((0.020, 0.056, 0.072)),
        origin=Origin(xyz=(0.075, -0.024, 0.180)),
        material=satin_grey,
        name="hinge_ear_1",
    )
    neck.visual(
        Cylinder(radius=0.012, length=0.135),
        origin=Origin(xyz=(0.0, -0.055, 0.180), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="tilt_pin",
    )
    neck.visual(
        Box((0.010, 0.006, 0.190)),
        origin=Origin(xyz=(-0.037, 0.056, 0.115)),
        material=satin_grey,
        name="door_hinge_stile",
    )
    neck.visual(
        Cylinder(radius=0.0037, length=0.170),
        origin=Origin(xyz=(-0.035, 0.059, 0.115)),
        material=charcoal,
        name="door_pin",
    )

    # A real rear cable-cover flap, not a painted seam.  Its local frame is the
    # vertical hinge line; the panel spans across +X and sits proud of the spine.
    cable_door = model.part("cable_door")
    cable_door.visual(
        Box((0.060, 0.006, 0.168)),
        origin=Origin(xyz=(0.030, 0.003, 0.0)),
        material=dark_plastic,
        name="door_panel",
    )
    cable_door.visual(
        Box((0.046, 0.003, 0.0025)),
        origin=Origin(xyz=(0.033, 0.0062, 0.040)),
        material=matte_black,
        name="finger_recess",
    )
    cable_door.visual(
        Cylinder(radius=0.0048, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=dark_plastic,
        name="hinge_barrel_0",
    )
    cable_door.visual(
        Cylinder(radius=0.0048, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_plastic,
        name="hinge_barrel_1",
    )

    display = model.part("display")
    shell_mesh = mesh_from_cadquery(
        _rounded_box(0.520, 0.055, 0.320, 0.018),
        "display_shell",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )
    display.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        material=charcoal,
        name="shell",
    )
    display.visual(
        Box((0.462, 0.003, 0.262)),
        origin=Origin(xyz=(0.0, -0.073, 0.010)),
        material=screen_glass,
        name="screen",
    )
    display.visual(
        Box((0.520, 0.006, 0.025)),
        origin=Origin(xyz=(0.0, -0.0735, -0.147)),
        material=matte_black,
        name="lower_bezel",
    )
    display.visual(
        Box((0.120, 0.030, 0.105)),
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
        material=dark_plastic,
        name="rear_boss",
    )
    display.visual(
        Cylinder(radius=0.016, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="tilt_socket",
    )

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "stand_to_neck",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.18, lower=0.0, upper=0.100),
    )
    model.articulation(
        "neck_to_display",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=display,
        origin=Origin(xyz=(0.0, -0.055, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=1.2, lower=-0.30, upper=0.35),
    )
    model.articulation(
        "neck_to_cable_door",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=cable_door,
        origin=Origin(xyz=(-0.035, 0.059, 0.115)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    neck = object_model.get_part("neck")
    display = object_model.get_part("display")
    door = object_model.get_part("cable_door")

    swivel = object_model.get_articulation("base_to_stand")
    height = object_model.get_articulation("stand_to_neck")
    tilt = object_model.get_articulation("neck_to_display")
    door_hinge = object_model.get_articulation("neck_to_cable_door")

    # Intentional modeled-in captures: the mast slides inside a sleeve envelope,
    # and the two small hinge pins/barrels are coaxial rather than separated.
    ctx.allow_overlap(
        stand,
        neck,
        elem_a="outer_sleeve",
        elem_b="inner_mast",
        reason="The telescoping mast is intentionally retained inside the hollow sleeve envelope.",
    )
    ctx.allow_overlap(
        stand,
        neck,
        elem_a="outer_sleeve",
        elem_b="guide_shoe_0",
        reason="A low-friction guide shoe is intentionally seated against the sleeve wall for telescoping support.",
    )
    ctx.allow_overlap(
        stand,
        neck,
        elem_a="outer_sleeve",
        elem_b="guide_shoe_1",
        reason="A low-friction guide shoe is intentionally seated against the sleeve wall for telescoping support.",
    )
    ctx.allow_overlap(
        neck,
        display,
        elem_a="tilt_pin",
        elem_b="tilt_socket",
        reason="The display tilt socket is modeled around the supported hinge pin.",
    )
    ctx.allow_overlap(
        neck,
        display,
        elem_a="tilt_pin",
        elem_b="rear_boss",
        reason="The hinge pin passes through the rear display boss that carries the tilt socket.",
    )
    ctx.allow_overlap(
        neck,
        door,
        elem_a="door_pin",
        elem_b="hinge_barrel_0",
        reason="The cable-cover hinge barrel is captured around the small vertical door pin.",
    )
    ctx.allow_overlap(
        neck,
        door,
        elem_a="door_pin",
        elem_b="hinge_barrel_1",
        reason="The cable-cover hinge barrel is captured around the small vertical door pin.",
    )

    ctx.expect_within(
        neck,
        stand,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="mast centered in sleeve at low height",
    )
    ctx.expect_overlap(
        neck,
        stand,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.060,
        name="mast retained in sleeve at low height",
    )
    ctx.expect_overlap(
        neck,
        stand,
        axes="z",
        elem_a="guide_shoe_0",
        elem_b="outer_sleeve",
        min_overlap=0.030,
        name="left guide shoe rides inside sleeve",
    )
    ctx.expect_overlap(
        neck,
        stand,
        axes="z",
        elem_a="guide_shoe_1",
        elem_b="outer_sleeve",
        min_overlap=0.030,
        name="right guide shoe rides inside sleeve",
    )
    with ctx.pose({height: 0.100}):
        ctx.expect_within(
            neck,
            stand,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.004,
            name="mast centered in sleeve at raised height",
        )
        ctx.expect_overlap(
            neck,
            stand,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.045,
            name="mast retained in sleeve at raised height",
        )

    with ctx.pose({tilt: 0.28}):
        ctx.expect_overlap(
            display,
            neck,
            axes="x",
            elem_a="tilt_socket",
            elem_b="tilt_pin",
            min_overlap=0.050,
            name="tilt hinge remains captured while tilted",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.2}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    closed_center_y = None if closed_aabb is None else (closed_aabb[0][1] + closed_aabb[1][1]) / 2.0
    opened_center_y = None if opened_aabb is None else (opened_aabb[0][1] + opened_aabb[1][1]) / 2.0
    ctx.check(
        "cable door swings outward on vertical edge",
        closed_center_y is not None and opened_center_y is not None and opened_center_y > closed_center_y + 0.010,
        details=f"closed_y={closed_center_y}, opened_y={opened_center_y}",
    )

    rest_pos = ctx.part_world_position(stand)
    with ctx.pose({swivel: pi / 2.0}):
        turned_pos = ctx.part_world_position(stand)
    ctx.check(
        "stand swivel joint is vertical and continuous",
        rest_pos is not None and turned_pos is not None and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
