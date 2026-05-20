from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


LINK_LENGTH = 0.205
LINK_EYE_RADIUS = 0.033
LINK_HOLE_RADIUS = 0.014
LINK_THICKNESS = 0.008
LINK_BOSS_THICKNESS = 0.012
PIN_RADIUS = 0.0095
PIN_LENGTH = 0.038
LAYER_OFFSET = 0.007


def _cyl_xy(x: float, y: float, radius: float, height: float) -> cq.Workplane:
    """A cylinder extruded along Z, centered about z=0."""

    return (
        cq.Workplane("XY")
        .center(x, y)
        .circle(radius)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _rect_xy(x: float, y: float, sx: float, sy: float, height: float) -> cq.Workplane:
    """A rectangular prism extruded along Z, centered about z=0."""

    return (
        cq.Workplane("XY")
        .center(x, y)
        .rect(sx, sy)
        .extrude(height)
        .translate((0.0, 0.0, -height / 2.0))
    )


def _rounded_slot(x: float, y: float, length: float, radius: float, height: float) -> cq.Workplane:
    """Through-cut obround slot in the link plate."""

    slot = _rect_xy(x, y, max(0.001, length - 2.0 * radius), 2.0 * radius, height)
    slot = slot.union(_cyl_xy(x - length / 2.0 + radius, y, radius, height))
    slot = slot.union(_cyl_xy(x + length / 2.0 - radius, y, radius, height))
    return slot


def _ladder_link_shape(length: float) -> cq.Workplane:
    """Flat ladder-frame link with hinge eyes, annular bosses, and lightening slots."""

    cut_height = LINK_BOSS_THICKNESS + 0.010

    plate = _cyl_xy(0.0, 0.0, LINK_EYE_RADIUS, LINK_THICKNESS)
    plate = plate.union(_cyl_xy(length, 0.0, LINK_EYE_RADIUS, LINK_THICKNESS))
    plate = plate.union(_rect_xy(length / 2.0, 0.0, length, 0.041, LINK_THICKNESS))

    # Raised annular hinge blocks around each pin hole.
    plate = plate.union(_cyl_xy(0.0, 0.0, 0.024, LINK_BOSS_THICKNESS))
    plate = plate.union(_cyl_xy(length, 0.0, 0.024, LINK_BOSS_THICKNESS))

    # Two windows leave a center rung so each member reads as a miniature ladder frame.
    slot_len = (length - 0.120) / 2.0
    plate = plate.cut(_rounded_slot(0.060 + slot_len / 2.0, 0.0, slot_len, 0.0105, cut_height))
    plate = plate.cut(
        _rounded_slot(length - 0.060 - slot_len / 2.0, 0.0, slot_len, 0.0105, cut_height)
    )

    # True through holes keep the visible pins clear of the link plates.
    plate = plate.cut(_cyl_xy(0.0, 0.0, LINK_HOLE_RADIUS, cut_height))
    plate = plate.cut(_cyl_xy(length, 0.0, LINK_HOLE_RADIUS, cut_height))

    return plate


def _output_plate_shape(length: float) -> cq.Workplane:
    """Bolted output tab fixed to the distal end of the last ladder member."""

    cut_height = LINK_BOSS_THICKNESS + 0.010
    plate = _rect_xy(length + 0.032, 0.0, 0.064, 0.034, LINK_THICKNESS)
    plate = plate.union(_rect_xy(length + 0.079, 0.0, 0.070, 0.082, LINK_THICKNESS))
    plate = plate.union(_cyl_xy(length + 0.044, 0.041, 0.035, LINK_THICKNESS))
    plate = plate.union(_cyl_xy(length + 0.044, -0.041, 0.035, LINK_THICKNESS))
    plate = plate.cut(_cyl_xy(length + 0.083, 0.024, 0.0065, cut_height))
    plate = plate.cut(_cyl_xy(length + 0.083, -0.024, 0.0065, cut_height))
    return plate


def _add_pin_visuals(part, *, x: float, material_pin: str) -> None:
    """Add a visible hinge pin with retaining washers at a local hinge center."""

    part.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(x, 0.0, 0.0)),
        material=material_pin,
        name="hinge_pin",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(x, 0.0, 0.015)),
        material=material_pin,
        name="upper_washer",
    )
    part.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(x, 0.0, -0.015)),
        material=material_pin,
        name="lower_washer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_four_joint_hinge_chain")

    model.material("blued_steel", rgba=(0.10, 0.18, 0.28, 1.0))
    model.material("dark_oxide", rgba=(0.035, 0.038, 0.042, 1.0))
    model.material("machined_pin", rgba=(0.70, 0.70, 0.66, 1.0))
    model.material("base_painted", rgba=(0.24, 0.25, 0.23, 1.0))
    model.material("output_plate", rgba=(0.86, 0.48, 0.12, 1.0))

    regular_link_mesh = mesh_from_cadquery(_ladder_link_shape(LINK_LENGTH), "ladder_link")
    output_plate_mesh = mesh_from_cadquery(_output_plate_shape(LINK_LENGTH), "output_plate")
    base_link_mesh = mesh_from_cadquery(
        _ladder_link_shape(LINK_LENGTH).translate((-LINK_LENGTH, 0.0, 0.0)),
        "fixed_ladder_link",
    )

    base = model.part("base_lug")
    base.visual(
        base_link_mesh,
        origin=Origin(xyz=(0.0, 0.0, -LAYER_OFFSET)),
        material="blued_steel",
        name="fixed_bar",
    )
    base.visual(
        Box((0.185, 0.130, 0.014)),
        origin=Origin(xyz=(-0.115, 0.0, -0.061)),
        material="base_painted",
        name="mounting_foot",
    )
    base.visual(
        Box((0.135, 0.026, 0.048)),
        origin=Origin(xyz=(-0.090, 0.0, -0.034)),
        material="base_painted",
        name="center_web",
    )
    for side, y in (("near", 0.046), ("far", -0.046)):
        base.visual(
            Box((0.070, 0.012, 0.054)),
            origin=Origin(xyz=(-0.010, y, -0.027)),
            material="base_painted",
            name=f"{side}_lug_cheek",
        )
    _add_pin_visuals(base, x=0.0, material_pin="machined_pin")

    layers = [LAYER_OFFSET, -LAYER_OFFSET, LAYER_OFFSET, -LAYER_OFFSET]
    links = []
    for i, layer in enumerate(layers):
        link = model.part(f"link_{i}")
        link.visual(
            regular_link_mesh,
            origin=Origin(xyz=(0.0, 0.0, layer)),
            material="blued_steel",
            name="bar_frame",
        )
        if i == 3:
            link.visual(
                output_plate_mesh,
                origin=Origin(xyz=(0.0, 0.0, layer)),
                material="output_plate",
                name="output_plate",
            )
        if i < 3:
            _add_pin_visuals(link, x=LINK_LENGTH, material_pin="machined_pin")
        links.append(link)

    hinge_limits = MotionLimits(effort=12.0, velocity=2.0, lower=-1.10, upper=1.10)
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=hinge_limits,
    )
    for i in range(3):
        model.articulation(
            f"link_{i}_to_link_{i + 1}",
            ArticulationType.REVOLUTE,
            parent=links[i],
            child=links[i + 1],
            origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=hinge_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_lug")
    links = [object_model.get_part(f"link_{i}") for i in range(4)]
    joints = [
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
    ]

    ctx.check(
        "four revolute serial joints",
        len(joints) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and [j.parent for j in joints]
        == ["base_lug", "link_0", "link_1", "link_2"]
        and [j.child for j in joints]
        == ["link_0", "link_1", "link_2", "link_3"],
        details=f"joints={[(j.name, j.articulation_type, j.parent, j.child) for j in joints]}",
    )
    ctx.check(
        "hinge axes normal to the linkage plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 0.0, 1.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    hinge_positions = [ctx.part_world_position(link) for link in links]
    ctx.check(
        "hinge blocks are evenly spaced at rest",
        all(p is not None for p in hinge_positions)
        and all(abs(hinge_positions[i][0] - i * LINK_LENGTH) < 1e-5 for i in range(4))
        and all(abs(hinge_positions[i][1]) < 1e-5 for i in range(4))
        and all(abs(hinge_positions[i][2]) < 1e-5 for i in range(4)),
        details=f"positions={hinge_positions}",
    )

    ctx.expect_overlap(
        base,
        links[0],
        axes="xy",
        elem_a="fixed_bar",
        elem_b="bar_frame",
        min_overlap=0.020,
        name="base hinge eyes share the first pin footprint",
    )
    ctx.expect_gap(
        links[0],
        base,
        axis="z",
        positive_elem="bar_frame",
        negative_elem="fixed_bar",
        min_gap=0.001,
        max_gap=0.003,
        name="alternating link plates have realistic stack clearance",
    )
    ctx.expect_contact(
        base,
        links[0],
        elem_a="upper_washer",
        elem_b="bar_frame",
        contact_tol=0.001,
        name="base retaining washer captures first link",
    )

    washer_for_child = ["lower_washer", "upper_washer", "lower_washer"]
    for i, washer in enumerate(washer_for_child):
        ctx.expect_overlap(
            links[i],
            links[i + 1],
            axes="xy",
            elem_a="bar_frame",
            elem_b="bar_frame",
            min_overlap=0.020,
            name=f"link_{i} and link_{i + 1} hinge eyes align",
        )
        ctx.expect_contact(
            links[i],
            links[i + 1],
            elem_a=washer,
            elem_b="bar_frame",
            contact_tol=0.001,
            name=f"link_{i} retaining washer captures link_{i + 1}",
        )

    output_aabb = ctx.part_element_world_aabb(links[3], elem="output_plate")
    ctx.check(
        "final member carries a small output plate beyond the last bar",
        output_aabb is not None and output_aabb[1][0] > 4.0 * LINK_LENGTH + 0.080,
        details=f"output_aabb={output_aabb}",
    )

    rest_pos = ctx.part_world_position(links[3])
    with ctx.pose({joints[0]: 0.45, joints[1]: 0.20, joints[2]: -0.15, joints[3]: 0.10}):
        moved_pos = ctx.part_world_position(links[3])
    ctx.check(
        "serial chain articulates in one horizontal plane",
        rest_pos is not None
        and moved_pos is not None
        and moved_pos[1] > rest_pos[1] + 0.20
        and abs(moved_pos[2] - rest_pos[2]) < 1e-5,
        details=f"rest={rest_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
