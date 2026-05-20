from __future__ import annotations

import math

import cadquery as cq
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


def _translated_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _union_boxes(boxes: list[tuple[tuple[float, float, float], tuple[float, float, float]]]):
    shape = _translated_box(*boxes[0])
    for size, center in boxes[1:]:
        shape = shape.union(_translated_box(size, center))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_parcel_mailbox")

    mailbox_blue = model.material("powder_coated_blue", rgba=(0.06, 0.16, 0.27, 1.0))
    dark_liner = model.material("shadowed_interior", rgba=(0.015, 0.018, 0.020, 1.0))
    brushed_metal = model.material("brushed_stainless", rgba=(0.62, 0.64, 0.62, 1.0))
    black_rubber = model.material("black_gasket", rgba=(0.01, 0.01, 0.009, 1.0))

    # Object frame: X is depth (front at negative X), Y is width, Z is height.
    width = 0.62
    depth = 0.55
    body_bottom = 0.74
    body_height = 0.78
    body_top = body_bottom + body_height
    body_center_z = body_bottom + body_height / 2.0
    wall = 0.035
    front_x = -depth / 2.0
    back_x = depth / 2.0
    eps = 0.004

    # Continuous hollow parcel cabinet, front frame, and pedestal.  The front is
    # mostly open behind the articulated faces so the lower door reads as access
    # to a deep parcel compartment rather than a flat panel.
    front_depth = 0.040
    front_center_x = front_x + front_depth / 2.0
    mid_rail_z = 1.19
    slot_hinge_z = 1.405
    slot_height = 0.135
    slot_width = 0.50
    flap_width = 0.49

    cabinet_boxes = [
        # Deep hollow box: sides, back, top and bottom.
        ((wall, width, body_height), (back_x - wall / 2.0, 0.0, body_center_z)),
        ((depth, wall, body_height), (0.0, -width / 2.0 + wall / 2.0, body_center_z)),
        ((depth, wall, body_height), (0.0, width / 2.0 - wall / 2.0, body_center_z)),
        ((depth, width, wall), (0.0, 0.0, body_bottom + wall / 2.0)),
        ((depth, width, wall), (0.0, 0.0, body_top - wall / 2.0)),
        # Front perimeter and center rail splitting upper and lower sections.
        ((front_depth, wall, body_height), (front_center_x, -width / 2.0 + wall / 2.0, body_center_z)),
        ((front_depth, wall, body_height), (front_center_x, width / 2.0 - wall / 2.0, body_center_z)),
        ((front_depth, width, wall), (front_center_x, 0.0, body_top - wall / 2.0)),
        ((front_depth, width, wall), (front_center_x, 0.0, body_bottom + wall / 2.0)),
        ((front_depth, width, 0.044), (front_center_x, 0.0, mid_rail_z)),
        # Static upper face bars around the narrow deposit slot.
        ((front_depth, 0.050, 0.260), (front_center_x, -slot_width / 2.0 - 0.025, 1.335)),
        ((front_depth, 0.050, 0.260), (front_center_x, slot_width / 2.0 + 0.025, 1.335)),
        ((front_depth, slot_width + 0.100, 0.040), (front_center_x, 0.0, slot_hinge_z + 0.045)),
        ((front_depth, slot_width + 0.100, 0.040), (front_center_x, 0.0, slot_hinge_z - slot_height - 0.025)),
        # Pedestal and ground plate, slightly overlapping so it is one solid.
        ((0.18, 0.16, body_bottom - 0.055 + eps), (0.010, 0.0, (body_bottom + 0.055) / 2.0)),
        ((0.46, 0.40, 0.060), (0.010, 0.0, 0.030)),
    ]
    cabinet_shape = _union_boxes(cabinet_boxes).edges("|Z").fillet(0.006)

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(cabinet_shape, "cabinet_shell", tolerance=0.001),
        material=mailbox_blue,
        name="cabinet_shell",
    )
    # Dark inner surfaces seen through gaps and when the retrieval door opens.
    cabinet.visual(
        Box((0.006, width - 2.0 * wall, 0.46)),
        origin=Origin(xyz=(back_x - wall - 0.003, 0.0, 0.96)),
        material=dark_liner,
        name="parcel_cavity_back",
    )
    cabinet.visual(
        Box((0.010, slot_width + 0.020, slot_height + 0.020)),
        origin=Origin(xyz=(front_x + 0.018, 0.0, slot_hinge_z - slot_height / 2.0)),
        material=dark_liner,
        name="deposit_slot_shadow",
    )
    # Fixed hinge leaves project just proud of the front face.  They touch the
    # moving flap/door in the closed pose, giving each articulated panel a real
    # support path instead of leaving it visually floating.
    cabinet.visual(
        Box((0.025, flap_width, 0.014)),
        origin=Origin(xyz=(front_x + 0.0025, 0.0, slot_hinge_z - 0.003)),
        material=brushed_metal,
        name="flap_hinge_leaf",
    )
    cabinet.visual(
        Box((0.025, 0.052, 0.340)),
        origin=Origin(xyz=(front_x + 0.0025, -0.276, 0.985)),
        material=brushed_metal,
        name="door_hinge_leaf",
    )

    # Upper deposit flap: hinge frame lies on the horizontal slot edge.  The
    # closed panel hangs downward and opens outward toward negative X.
    flap_height = 0.130
    flap_thick = 0.018
    flap = model.part("deposit_flap")
    flap.visual(
        Box((flap_thick, flap_width, flap_height)),
        origin=Origin(xyz=(-flap_thick / 2.0, 0.0, -flap_height / 2.0)),
        material=mailbox_blue,
        name="flap_panel",
    )
    flap.visual(
        Box((0.008, flap_width * 0.82, 0.018)),
        origin=Origin(xyz=(-flap_thick - 0.004, 0.0, -flap_height * 0.72)),
        material=brushed_metal,
        name="flap_pull_lip",
    )
    flap.visual(
        Cylinder(radius=0.012, length=flap_width + 0.050),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="flap_hinge_barrel",
    )
    flap_hinge = model.articulation(
        "cabinet_to_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=flap,
        origin=Origin(xyz=(front_x - 0.010, 0.0, slot_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    # Lower retrieval door: child frame is the vertical side hinge line.  The
    # slab extends across the lower front and swings outward on +Z rotation.
    door_width = 0.50
    door_height = 0.360
    door_thick = 0.024
    door_bottom = 0.805
    door_hinge_y = -door_width / 2.0
    door = model.part("retrieval_door")
    door.visual(
        Box((door_thick, door_width, door_height)),
        origin=Origin(xyz=(-door_thick / 2.0, door_width / 2.0, door_height / 2.0)),
        material=mailbox_blue,
        name="door_slab",
    )
    door.visual(
        Box((0.006, door_width - 0.075, door_height - 0.075)),
        origin=Origin(xyz=(-door_thick - 0.003, door_width / 2.0, door_height / 2.0)),
        material=mailbox_blue,
        name="raised_door_panel",
    )
    door.visual(
        Cylinder(radius=0.014, length=door_height * 0.92),
        origin=Origin(xyz=(-0.026, 0.0, door_height / 2.0), rpy=(0.0, 0.0, 0.0)),
        material=brushed_metal,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.026, 0.012, 0.045)),
        origin=Origin(xyz=(-0.037, door_width * 0.70 - 0.055, door_height * 0.54)),
        material=brushed_metal,
        name="handle_post_0",
    )
    door.visual(
        Box((0.026, 0.012, 0.045)),
        origin=Origin(xyz=(-0.037, door_width * 0.70 + 0.055, door_height * 0.54)),
        material=brushed_metal,
        name="handle_post_1",
    )
    door.visual(
        Box((0.018, 0.135, 0.020)),
        origin=Origin(xyz=(-0.057, door_width * 0.70, door_height * 0.54)),
        material=brushed_metal,
        name="pull_handle",
    )
    door.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(
            xyz=(-door_thick - 0.004, door_width * 0.70, door_height * 0.74),
            rpy=(0.0, -math.pi / 2.0, 0.0),
        ),
        material=black_rubber,
        name="lock_cylinder",
    )
    door_hinge = model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(front_x - 0.010, door_hinge_y, door_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.65),
    )

    model.meta["description"] = (
        "A pedestal parcel mailbox with a deep hollow compartment, a narrow "
        "horizontal deposit flap, and a larger vertical-side-hinged retrieval door."
    )
    # Keep local variables alive for readability and to make the two intended
    # front mechanisms explicit in the authored script.
    _ = (flap_hinge, door_hinge)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    flap = object_model.get_part("deposit_flap")
    door = object_model.get_part("retrieval_door")
    flap_hinge = object_model.get_articulation("cabinet_to_flap")
    door_hinge = object_model.get_articulation("cabinet_to_door")

    ctx.expect_contact(
        cabinet,
        flap,
        elem_a="flap_hinge_leaf",
        elem_b="flap_panel",
        contact_tol=0.0015,
        name="deposit flap is carried by the horizontal hinge leaf",
    )
    ctx.expect_overlap(
        flap,
        cabinet,
        axes="yz",
        min_overlap=0.10,
        name="deposit flap covers the narrow upper slot",
    )
    ctx.expect_contact(
        cabinet,
        door,
        elem_a="door_hinge_leaf",
        elem_b="door_slab",
        contact_tol=0.0015,
        name="retrieval door is carried by the vertical hinge leaf",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="yz",
        min_overlap=0.25,
        name="retrieval door spans the lower parcel opening",
    )

    rest_flap_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: 1.0}):
        open_flap_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "deposit flap opens outward on horizontal hinge",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][0] < rest_flap_aabb[0][0] - 0.055,
        details=f"closed={rest_flap_aabb}, open={open_flap_aabb}",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "retrieval door opens outward on vertical side hinge",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][0] < rest_door_aabb[0][0] - 0.15,
        details=f"closed={rest_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
