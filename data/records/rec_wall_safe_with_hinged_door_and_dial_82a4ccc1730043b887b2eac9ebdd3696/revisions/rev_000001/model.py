from __future__ import annotations

import math

import cadquery as cq
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


def _handwheel_geometry() -> cq.Workplane:
    """Flat steel handwheel in the XZ plane, extruded along local +Y."""
    thickness = 0.018
    outer_r = 0.130
    inner_r = 0.108
    hub_outer_r = 0.040
    shaft_clearance_r = 0.014

    rim = cq.Workplane("XZ").circle(outer_r).circle(inner_r).extrude(thickness)
    hub = cq.Workplane("XZ").circle(hub_outer_r).extrude(thickness)

    spokes = cq.Workplane("XZ")
    for angle in (0.0, 60.0, 120.0):
        spoke = (
            cq.Workplane("XZ")
            .rect(2.0 * inner_r, 0.017)
            .extrude(thickness)
            .rotate((0, 0, 0), (0, 1, 0), angle)
        )
        spokes = spokes.union(spoke)

    wheel = rim.union(spokes).union(hub)
    bore = cq.Workplane("XZ").circle(shaft_clearance_r).extrude(thickness * 1.4)
    return wheel.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_safe")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.12, 0.13, 1.0))
    blued_door = model.material("blued_door", rgba=(0.16, 0.19, 0.21, 1.0))
    black_shadow = model.material("black_shadow", rgba=(0.015, 0.015, 0.014, 1.0))
    satin_chrome = model.material("satin_chrome", rgba=(0.62, 0.64, 0.62, 1.0))
    brass = model.material("aged_brass", rgba=(0.68, 0.52, 0.22, 1.0))
    wall_paint = model.material("wall_paint", rgba=(0.78, 0.76, 0.70, 1.0))

    case = model.part("case")
    # A shallow wall surface makes the safe read as recessed rather than free-standing.
    case.visual(
        Box((1.05, 0.018, 1.20)),
        origin=Origin(xyz=(0.0, -0.046, 0.0)),
        material=wall_paint,
        name="wall_surface",
    )
    # Hollow steel vault box behind the trim flange.
    case.visual(
        Box((0.62, 0.020, 0.76)),
        origin=Origin(xyz=(0.0, -0.180, 0.0)),
        material=dark_steel,
        name="back_wall",
    )
    case.visual(
        Box((0.026, 0.165, 0.76)),
        origin=Origin(xyz=(-0.313, -0.105, 0.0)),
        material=dark_steel,
        name="side_wall_0",
    )
    case.visual(
        Box((0.026, 0.165, 0.76)),
        origin=Origin(xyz=(0.313, -0.105, 0.0)),
        material=dark_steel,
        name="side_wall_1",
    )
    case.visual(
        Box((0.652, 0.165, 0.026)),
        origin=Origin(xyz=(0.0, -0.105, 0.393)),
        material=dark_steel,
        name="top_wall",
    )
    case.visual(
        Box((0.652, 0.165, 0.026)),
        origin=Origin(xyz=(0.0, -0.105, -0.393)),
        material=dark_steel,
        name="bottom_wall",
    )
    case.visual(
        Box((0.60, 0.010, 0.74)),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        material=black_shadow,
        name="shadow_recess",
    )
    # Broad trim flange around the opening; the pieces overlap at the mitred-looking
    # corners so the authored part is one connected steel surround.
    case.visual(
        Box((0.82, 0.028, 0.090)),
        origin=Origin(xyz=(0.0, -0.014, 0.455)),
        material=dark_steel,
        name="top_flange",
    )
    case.visual(
        Box((0.82, 0.028, 0.090)),
        origin=Origin(xyz=(0.0, -0.014, -0.455)),
        material=dark_steel,
        name="bottom_flange",
    )
    case.visual(
        Box((0.090, 0.028, 1.00)),
        origin=Origin(xyz=(-0.365, -0.014, 0.0)),
        material=dark_steel,
        name="hinge_flange",
    )
    case.visual(
        Box((0.090, 0.028, 1.00)),
        origin=Origin(xyz=(0.365, -0.014, 0.0)),
        material=dark_steel,
        name="strike_flange",
    )

    # Stationary hinge knuckles on the left trim, alternating with the moving door knuckles.
    for i, zc in enumerate((-0.250, 0.0, 0.250)):
        case.visual(
            Cylinder(radius=0.018, length=0.140),
            origin=Origin(xyz=(-0.285, 0.035, zc)),
            material=satin_chrome,
            name=f"fixed_knuckle_{i}",
        )
        case.visual(
            Box((0.080, 0.036, 0.110)),
            origin=Origin(xyz=(-0.325, 0.018, zc)),
            material=satin_chrome,
            name=f"fixed_leaf_{i}",
        )

    door = model.part("door")
    door.visual(
        Box((0.520, 0.065, 0.680)),
        origin=Origin(xyz=(0.295, 0.0, 0.0)),
        material=blued_door,
        name="door_slab",
    )
    # Raised perimeter ribs make the slab read as a heavy safe door instead of a flat plate.
    door.visual(
        Box((0.445, 0.010, 0.030)),
        origin=Origin(xyz=(0.315, 0.037, 0.300)),
        material=dark_steel,
        name="top_rib",
    )
    door.visual(
        Box((0.445, 0.010, 0.030)),
        origin=Origin(xyz=(0.315, 0.037, -0.300)),
        material=dark_steel,
        name="bottom_rib",
    )
    door.visual(
        Box((0.030, 0.010, 0.580)),
        origin=Origin(xyz=(0.535, 0.037, 0.0)),
        material=dark_steel,
        name="strike_rib",
    )
    door.visual(
        Box((0.040, 0.010, 0.580)),
        origin=Origin(xyz=(0.060, 0.037, 0.0)),
        material=dark_steel,
        name="hinge_rib",
    )
    for i, zc in enumerate((-0.125, 0.125)):
        door.visual(
            Cylinder(radius=0.018, length=0.110),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=satin_chrome,
            name=f"moving_knuckle_{i}",
        )
        door.visual(
            Box((0.055, 0.036, 0.090)),
            origin=Origin(xyz=(0.033, 0.018, zc)),
            material=satin_chrome,
            name=f"moving_leaf_{i}",
        )
    # Bearing washer and shaft for the center handwheel.
    door.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.295, 0.0385, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=satin_chrome,
        name="wheel_bearing",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.0275),
        origin=Origin(xyz=(0.295, 0.04875, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=satin_chrome,
        name="wheel_shaft",
    )
    door.visual(
        Cylinder(radius=0.082, length=0.0095),
        origin=Origin(xyz=(0.295, 0.03725, 0.230), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=satin_chrome,
        name="dial_bearing",
    )

    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.145,
            0.034,
            body_style="faceted",
            top_diameter=0.118,
            base_diameter=0.150,
            edge_radius=0.002,
            grip=KnobGrip(style="fluted", count=36, depth=0.002),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.001, angle_deg=0.0),
            center=False,
        ),
        "combination_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="dial_cap",
    )

    wheel = model.part("wheel")
    wheel.visual(
        mesh_from_cadquery(_handwheel_geometry(), "spoked_handwheel", tolerance=0.0007),
        origin=Origin(),
        material=satin_chrome,
        name="handwheel",
    )
    wheel.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=satin_chrome,
        name="retaining_cap",
    )

    door_hinge = model.articulation(
        "case_to_door",
        ArticulationType.REVOLUTE,
        parent=case,
        child=door,
        origin=Origin(xyz=(-0.285, 0.035, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.75, lower=0.0, upper=1.65),
    )
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.295, 0.042, 0.230)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "door_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=wheel,
        origin=Origin(xyz=(0.295, 0.0625, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=5.0),
    )

    # Keep the local variable used so linters do not mistake the hinge for an unused idea.
    assert door_hinge is not None
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    case = object_model.get_part("case")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    wheel = object_model.get_part("wheel")
    hinge = object_model.get_articulation("case_to_door")
    dial_spin = object_model.get_articulation("door_to_dial")
    wheel_spin = object_model.get_articulation("door_to_wheel")

    ctx.expect_within(
        door,
        case,
        axes="xz",
        inner_elem="door_slab",
        outer_elem="shadow_recess",
        margin=0.0,
        name="door slab sits within the recessed opening",
    )
    ctx.expect_gap(
        wheel,
        door,
        axis="y",
        positive_elem="handwheel",
        negative_elem="wheel_bearing",
        max_gap=0.004,
        max_penetration=0.0001,
        name="handwheel bears on the center washer",
    )
    ctx.expect_gap(
        dial,
        door,
        axis="y",
        positive_elem="dial_cap",
        negative_elem="dial_bearing",
        max_gap=0.004,
        max_penetration=0.0001,
        name="dial mounts proud of the door face",
    )

    closed_slab = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({hinge: 1.2}):
        open_slab = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "door articulation has safe-door swing limits",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper > 1.5,
        details=f"limits={hinge.motion_limits}",
    )
    ctx.check(
        "rotary controls are continuous about the door normal",
        getattr(dial_spin, "articulation_type", None) == ArticulationType.CONTINUOUS
        and getattr(wheel_spin, "articulation_type", None) == ArticulationType.CONTINUOUS
        and tuple(dial_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0),
        details=f"dial={dial_spin}, wheel={wheel_spin}",
    )
    ctx.check(
        "left-hinged door swings outward",
        closed_slab is not None
        and open_slab is not None
        and open_slab[1][1] > closed_slab[1][1] + 0.25,
        details=f"closed={closed_slab}, open={open_slab}",
    )

    return ctx.report()


object_model = build_object_model()
