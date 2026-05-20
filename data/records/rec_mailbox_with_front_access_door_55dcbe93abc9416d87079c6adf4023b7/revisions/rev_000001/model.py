from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_parcel_mailbox")

    painted_steel = model.material("powder_coated_blue", rgba=(0.08, 0.18, 0.32, 1.0))
    dark_steel = model.material("dark_hinge_steel", rgba=(0.025, 0.027, 0.030, 1.0))
    black_rubber = model.material("black_gasket", rgba=(0.01, 0.01, 0.012, 1.0))
    brushed_metal = model.material("brushed_handle", rgba=(0.72, 0.70, 0.64, 1.0))

    box = model.part("box")

    # Short pedestal and plinth, kept as the fixed root assembly.
    box.visual(Box((0.48, 0.36, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.04)), material=painted_steel, name="base_foot")
    box.visual(Box((0.32, 0.24, 0.34)), origin=Origin(xyz=(0.0, 0.0, 0.245)), material=painted_steel, name="plinth_column")
    box.visual(Box((0.54, 0.42, 0.075)), origin=Origin(xyz=(0.0, 0.0, 0.4175)), material=painted_steel, name="plinth_cap")

    # Deep rectangular parcel box shell.  The front is open behind the retrieval door.
    body_w = 0.62
    body_d = 0.52
    body_h = 0.80
    body_bottom = 0.45
    body_center_z = body_bottom + body_h / 2.0
    wall = 0.035

    box.visual(Box((body_w, body_d, wall)), origin=Origin(xyz=(0.0, 0.0, body_bottom + wall / 2.0)), material=painted_steel, name="bottom_shell")
    box.visual(Box((body_w, body_d, wall)), origin=Origin(xyz=(0.0, 0.0, body_bottom + body_h - wall / 2.0)), material=painted_steel, name="top_shell")
    box.visual(Box((wall, body_d, body_h)), origin=Origin(xyz=(-(body_w - wall) / 2.0, 0.0, body_center_z)), material=painted_steel, name="side_shell_0")
    box.visual(Box((wall, body_d, body_h)), origin=Origin(xyz=((body_w - wall) / 2.0, 0.0, body_center_z)), material=painted_steel, name="side_shell_1")
    box.visual(Box((body_w, wall, body_h)), origin=Origin(xyz=(0.0, body_d / 2.0 - wall / 2.0, body_center_z)), material=painted_steel, name="rear_shell")

    # A front flange/gasket frame gives the closed door a visible seat without capping the opening.
    frame_y = -body_d / 2.0 + 0.012
    box.visual(Box((body_w, 0.024, 0.055)), origin=Origin(xyz=(0.0, frame_y, body_bottom + body_h - 0.0275)), material=painted_steel, name="front_header")
    box.visual(Box((body_w, 0.024, 0.055)), origin=Origin(xyz=(0.0, frame_y, body_bottom + 0.0275)), material=painted_steel, name="front_sill")
    box.visual(Box((0.050, 0.024, body_h)), origin=Origin(xyz=(-(body_w - 0.050) / 2.0, frame_y, body_center_z)), material=painted_steel, name="front_jamb_0")
    box.visual(Box((0.050, 0.024, body_h)), origin=Origin(xyz=((body_w - 0.050) / 2.0, frame_y, body_center_z)), material=painted_steel, name="front_jamb_1")
    box.visual(Box((0.55, 0.006, 0.66)), origin=Origin(xyz=(0.010, -body_d / 2.0 - 0.001, body_bottom + 0.40)), material=black_rubber, name="door_gasket")

    # Stationary hinge pin and alternating fixed knuckles on the hinged side.
    hinge_x = -0.285
    hinge_y = -0.285
    hinge_z = body_bottom + 0.065
    hinge_len = 0.69
    box.visual(Cylinder(radius=0.009, length=hinge_len), origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + hinge_len / 2.0)), material=dark_steel, name="hinge_pin")
    box.visual(Box((0.020, 0.040, hinge_len)), origin=Origin(xyz=(hinge_x - 0.014, -body_d / 2.0 + 0.025, hinge_z + hinge_len / 2.0)), material=dark_steel, name="hinge_mount_strip")
    for i, local_z in enumerate((0.220, 0.452)):
        box.visual(
            Cylinder(radius=0.021, length=0.100),
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + local_z)),
            material=dark_steel,
            name=f"fixed_knuckle_{i}",
        )
        box.visual(
            Box((0.026, 0.055, 0.080)),
            origin=Origin(xyz=(hinge_x - 0.010, hinge_y + 0.020, hinge_z + local_z)),
            material=dark_steel,
            name=f"hinge_lug_{i}",
        )

    door = model.part("door")
    door_w = 0.570
    door_h = 0.670
    door_t = 0.035
    panel_x0 = 0.018

    door.visual(Box((door_w - panel_x0, door_t, door_h)), origin=Origin(xyz=(panel_x0 + (door_w - panel_x0) / 2.0, 0.0, door_h / 2.0)), material=painted_steel, name="panel")
    # Raised face frame and inset panel details on the front of the retrieval door.
    face_y = -door_t / 2.0 - 0.006
    door.visual(Box((door_w - 0.080, 0.012, 0.040)), origin=Origin(xyz=(0.300, face_y, door_h - 0.055)), material=painted_steel, name="top_rail")
    door.visual(Box((door_w - 0.080, 0.012, 0.040)), origin=Origin(xyz=(0.300, face_y, 0.055)), material=painted_steel, name="bottom_rail")
    door.visual(Box((0.040, 0.012, door_h - 0.080)), origin=Origin(xyz=(0.055, face_y, door_h / 2.0)), material=painted_steel, name="hinge_stile")
    door.visual(Box((0.040, 0.012, door_h - 0.080)), origin=Origin(xyz=(door_w - 0.055, face_y, door_h / 2.0)), material=painted_steel, name="latch_stile")
    door.visual(Box((0.220, 0.010, 0.350)), origin=Origin(xyz=(0.245, -0.0215, door_h / 2.0 + 0.010)), material=black_rubber, name="recess_shadow")

    # Door-side knuckles wrap the stationary pin.  Short leaf straps overlap the panel
    # so the hinged edge reads as clipped in rather than floating.
    for i, local_z in enumerate((0.100, 0.335, 0.570)):
        door.visual(
            Cylinder(radius=0.021, length=0.110),
            origin=Origin(xyz=(0.0, 0.0, local_z)),
            material=dark_steel,
            name=f"door_knuckle_{i}",
        )
        door.visual(
            Box((0.100, 0.014, 0.085)),
            origin=Origin(xyz=(0.070, -0.001, local_z)),
            material=dark_steel,
            name=f"hinge_leaf_{i}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=box,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    handle = model.part("handle")
    # The handle frame is centered on its rotary pivot, just proud of the door face.
    handle.visual(Cylinder(radius=0.058, length=0.012), origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=brushed_metal, name="rosette")
    handle.visual(Cylinder(radius=0.014, length=0.080), origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(pi / 2.0, 0.0, 0.0)), material=brushed_metal, name="stem")
    handle.visual(Box((0.155, 0.026, 0.034)), origin=Origin(xyz=(0.0, -0.064, 0.0)), material=brushed_metal, name="turn_bar")
    handle.visual(Box((0.012, 0.030, 0.082)), origin=Origin(xyz=(0.0, -0.065, 0.0)), material=dark_steel, name="center_grip")

    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(door_w - 0.140, -door_t / 2.0, door_h / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    box = object_model.get_part("box")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    door_hinge = object_model.get_articulation("door_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    # The door knuckles are intentionally shown wrapped around a fixed hinge pin:
    # this is the visible clip that keeps the parcel door seated while it swings.
    for i in range(3):
        knuckle = f"door_knuckle_{i}"
        ctx.allow_overlap(
            box,
            door,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The fixed hinge pin is intentionally captured inside the door-side hinge knuckle.",
        )
        ctx.expect_overlap(
            door,
            box,
            axes="xy",
            elem_a=knuckle,
            elem_b="hinge_pin",
            min_overlap=0.012,
            name=f"{knuckle} is clipped around the hinge pin",
        )
        ctx.expect_overlap(
            door,
            box,
            axes="z",
            elem_a=knuckle,
            elem_b="hinge_pin",
            min_overlap=0.080,
            name=f"{knuckle} remains on the vertical hinge pin",
        )

    # A small rotary spindle passes through the face of the door to make the
    # handle read as a functioning latch rather than glued-on trim.
    ctx.allow_overlap(
        door,
        handle,
        elem_a="panel",
        elem_b="stem",
        reason="The handle stem intentionally penetrates the door panel as the rotary latch spindle.",
    )
    ctx.expect_overlap(
        handle,
        door,
        axes="xz",
        elem_a="stem",
        elem_b="panel",
        min_overlap=0.020,
        name="handle spindle passes through the door face",
    )
    ctx.expect_contact(
        handle,
        door,
        elem_a="rosette",
        elem_b="panel",
        contact_tol=0.001,
        name="handle rosette seats on the door face",
    )

    ctx.expect_gap(
        box,
        door,
        axis="y",
        positive_elem="front_jamb_1",
        negative_elem="panel",
        min_gap=0.002,
        max_gap=0.015,
        name="closed door sits just proud of the front frame",
    )

    closed_latch = ctx.part_element_world_aabb(door, elem="latch_stile")
    with ctx.pose({door_hinge: 1.20}):
        open_latch = ctx.part_element_world_aabb(door, elem="latch_stile")
        for i in range(3):
            ctx.expect_overlap(
                door,
                box,
                axes="xy",
                elem_a=f"door_knuckle_{i}",
                elem_b="hinge_pin",
                min_overlap=0.012,
                name=f"open {i} hinge knuckle stays clipped",
            )

    if closed_latch is not None and open_latch is not None:
        closed_y = (closed_latch[0][1] + closed_latch[1][1]) / 2.0
        open_y = (open_latch[0][1] + open_latch[1][1]) / 2.0
    else:
        closed_y = open_y = None
    ctx.check(
        "front door swings outward from the side hinge",
        closed_y is not None and open_y is not None and open_y < closed_y - 0.18,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    rest_bar = ctx.part_element_world_aabb(handle, elem="turn_bar")
    with ctx.pose({handle_pivot: 1.57}):
        turned_bar = ctx.part_element_world_aabb(handle, elem="turn_bar")
    if rest_bar is not None and turned_bar is not None:
        rest_dx = rest_bar[1][0] - rest_bar[0][0]
        turned_dz = turned_bar[1][2] - turned_bar[0][2]
    else:
        rest_dx = turned_dz = None
    ctx.check(
        "handle rotates on its own small pivot",
        rest_dx is not None and turned_dz is not None and rest_dx > 0.13 and turned_dz > 0.13,
        details=f"rest_dx={rest_dx}, turned_dz={turned_dz}",
    )

    return ctx.report()


object_model = build_object_model()
