from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


PLATE_THICKNESS = 0.010
BOSS_HEIGHT = 0.004
PIN_RADIUS = 0.006
PIN_CLEARANCE_RADIUS = 0.0085

LINK0_LENGTH = 0.190
LINK1_LENGTH = 0.165
TAB_LENGTH = 0.115

LOW_LAYER_Z = 0.012
HIGH_LAYER_Z = 0.026


def _rounded_bar(length: float, width: float, thickness: float) -> cq.Workplane:
    """Flat-sided obround plate from a proximal pin center to a distal pin center."""
    radius = width * 0.5
    web = cq.Workplane("XY").center(length * 0.5, 0.0).rect(length, width).extrude(thickness)
    start_lobe = cq.Workplane("XY").center(0.0, 0.0).circle(radius).extrude(thickness)
    end_lobe = cq.Workplane("XY").center(length, 0.0).circle(radius).extrude(thickness)
    return web.union(start_lobe).union(end_lobe)


def _hole_cutter(points: list[tuple[float, float]], radius: float, height: float) -> cq.Workplane:
    cutter = cq.Workplane("XY")
    for x, y in points:
        cutter = cutter.union(
            cq.Workplane("XY")
            .center(x, y)
            .circle(radius)
            .extrude(height)
            .translate((0.0, 0.0, -height * 0.25))
        )
    return cutter


def _link_plate(
    length: float,
    *,
    width: float = 0.046,
    proximal_hole: bool = True,
    distal_hole: bool = False,
    distal_tab: bool = False,
) -> cq.Workplane:
    """Flat chain link with compact raised pin bosses and optional clearance holes."""
    body = _rounded_bar(length, width, PLATE_THICKNESS)
    boss_radius = width * 0.42

    if distal_tab:
        tab = (
            cq.Workplane("XY")
            .center(length + 0.015, 0.0)
            .rect(0.030, width * 0.70)
            .extrude(PLATE_THICKNESS)
        )
        tip = cq.Workplane("XY").center(length + 0.030, 0.0).circle(width * 0.35).extrude(PLATE_THICKNESS)
        body = body.union(tab).union(tip)

    for x in (0.0, length):
        body = body.union(
            cq.Workplane("XY")
            .center(x, 0.0)
            .circle(boss_radius)
            .extrude(BOSS_HEIGHT)
            .translate((0.0, 0.0, PLATE_THICKNESS))
        )

    holes: list[tuple[float, float]] = []
    if proximal_hole:
        holes.append((0.0, 0.0))
    if distal_hole:
        holes.append((length, 0.0))
    if distal_tab:
        holes.append((length + 0.030, 0.0))
    if holes:
        body = body.cut(_hole_cutter(holes, PIN_CLEARANCE_RADIUS, PLATE_THICKNESS + BOSS_HEIGHT + 0.010))

    return body


def _cheek_plate() -> cq.Workplane:
    """Fixed side cheek with a broad mounting flange and a raised first-pivot pad."""
    plate_thickness = 0.008
    flange = cq.Workplane("XY").center(-0.050, 0.0).rect(0.125, 0.082).extrude(plate_thickness)
    pivot_lobe = cq.Workplane("XY").center(0.0, 0.0).circle(0.046).extrude(plate_thickness)
    tail_lobe = cq.Workplane("XY").center(-0.105, 0.0).circle(0.036).extrude(plate_thickness)
    body = flange.union(pivot_lobe).union(tail_lobe)

    raised_pad = (
        cq.Workplane("XY")
        .center(0.0, 0.0)
        .circle(0.024)
        .extrude(0.004)
        .translate((0.0, 0.0, plate_thickness))
    )
    body = body.union(raised_pad)

    # Two countersunk-looking mounting holes through the stationary cheek.
    body = body.cut(_hole_cutter([(-0.085, 0.024), (-0.085, -0.024)], 0.006, plate_thickness + 0.010))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_plate_lever_chain")

    cheek_mat = model.material("blued_fixed_cheek", rgba=(0.20, 0.25, 0.30, 1.0))
    link_mat = model.material("dark_phosphate_links", rgba=(0.05, 0.055, 0.06, 1.0))
    tab_mat = model.material("worn_end_tab", rgba=(0.12, 0.13, 0.13, 1.0))
    pin_mat = model.material("brushed_steel_pins", rgba=(0.72, 0.70, 0.66, 1.0))

    cheek = model.part("cheek")
    cheek.visual(
        mesh_from_cadquery(_cheek_plate(), "fixed_cheek_plate", tolerance=0.0005),
        material=cheek_mat,
        name="cheek_plate",
    )
    cheek.visual(
        Cylinder(radius=PIN_RADIUS, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=pin_mat,
        name="pivot_pin",
    )
    cheek.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=pin_mat,
        name="pivot_head",
    )

    link_0 = model.part("link_0")
    link_0.visual(
        mesh_from_cadquery(_link_plate(LINK0_LENGTH), "first_flat_link", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, LOW_LAYER_Z)),
        material=link_mat,
        name="link_plate",
    )
    # The distal pin is staked to link_0 and supports link_1 on the next layer.
    link_0.visual(
        Cylinder(radius=PIN_RADIUS, length=0.025),
        origin=Origin(xyz=(LINK0_LENGTH, 0.0, 0.0355)),
        material=pin_mat,
        name="distal_pin",
    )
    link_0.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(LINK0_LENGTH, 0.0, 0.044)),
        material=pin_mat,
        name="distal_head",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_link_plate(LINK1_LENGTH), "second_flat_link", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, HIGH_LAYER_Z)),
        material=link_mat,
        name="link_plate",
    )
    # This pin is upset downward through the lower end tab, alternating the side plates.
    link_1.visual(
        Cylinder(radius=PIN_RADIUS, length=0.032),
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.024)),
        material=pin_mat,
        name="distal_pin",
    )
    link_1.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.011)),
        material=pin_mat,
        name="distal_head",
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(
            _link_plate(TAB_LENGTH, width=0.038, proximal_hole=True, distal_hole=False, distal_tab=True),
            "small_end_tab",
            tolerance=0.0005,
        ),
        origin=Origin(xyz=(0.0, 0.0, LOW_LAYER_Z)),
        material=tab_mat,
        name="tab_plate",
    )

    model.articulation(
        "cheek_to_link_0",
        ArticulationType.REVOLUTE,
        parent=cheek,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.70, upper=0.85),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK0_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.8, lower=-1.10, upper=1.10),
    )
    model.articulation(
        "link_1_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=end_tab,
        origin=Origin(xyz=(LINK1_LENGTH, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = object_model.articulations
    ctx.check(
        "three supported revolute axes",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "joint axes are parallel",
        all(tuple(j.axis or ()) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    cheek = object_model.get_part("cheek")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_tab = object_model.get_part("end_tab")

    ctx.expect_contact(
        link_0,
        cheek,
        elem_a="link_plate",
        elem_b="cheek_plate",
        contact_tol=0.0008,
        name="first link rests on fixed cheek boss",
    )
    ctx.expect_contact(
        link_1,
        link_0,
        elem_a="link_plate",
        elem_b="link_plate",
        contact_tol=0.0008,
        name="middle link is supported by first link boss",
    )
    ctx.expect_contact(
        end_tab,
        link_1,
        elem_a="tab_plate",
        elem_b="link_plate",
        contact_tol=0.0008,
        name="end tab is supported by middle link boss",
    )
    ctx.expect_gap(
        link_0,
        cheek,
        axis="z",
        positive_elem="link_plate",
        negative_elem="cheek_plate",
        max_penetration=0.00001,
        max_gap=0.001,
        name="first link seats without penetrating cheek",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="z",
        positive_elem="link_plate",
        negative_elem="link_plate",
        max_penetration=0.00001,
        max_gap=0.001,
        name="middle link seats without penetrating first link",
    )
    ctx.expect_gap(
        link_1,
        end_tab,
        axis="z",
        positive_elem="link_plate",
        negative_elem="tab_plate",
        max_penetration=0.00001,
        max_gap=0.001,
        name="end tab seats without penetrating middle link",
    )

    link_0_to_link_1 = object_model.get_articulation("link_0_to_link_1")
    rest_pos = ctx.part_world_position(end_tab)
    with ctx.pose({link_0_to_link_1: 0.65}):
        moved_pos = ctx.part_world_position(end_tab)
    ctx.check(
        "middle revolute joint swings the end tab",
        rest_pos is not None and moved_pos is not None and moved_pos[1] > rest_pos[1] + 0.06,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
