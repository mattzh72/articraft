from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gullwing_service_hatch")

    body_green = model.material("equipment_body_green", rgba=(0.18, 0.30, 0.22, 1.0))
    darker_green = model.material("pressed_lid_green", rgba=(0.12, 0.23, 0.17, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    dark_steel = model.material("dark_phosphate_steel", rgba=(0.08, 0.085, 0.08, 1.0))
    worn_steel = model.material("worn_zinc_steel", rgba=(0.55, 0.57, 0.52, 1.0))

    # Object frame: X is the vehicle/equipment longitudinal direction,
    # Y is outward from the side skin (negative Y is exterior), and Z is up.
    panel_w = 1.30
    panel_h = 0.86
    skin_t = 0.040
    opening_w = 0.84
    opening_h = 0.42
    opening_z = 0.42
    opening_bottom = opening_z - opening_h / 2.0
    opening_top = opening_z + opening_h / 2.0

    body = model.part("body_frame")

    # Side skin built as four welded/formed panels around a genuine opening.
    body.visual(
        Box((panel_w, skin_t, panel_h - opening_top)),
        origin=Origin(xyz=(0.0, skin_t / 2.0, (panel_h + opening_top) / 2.0)),
        material=body_green,
        name="upper_skin",
    )
    body.visual(
        Box((panel_w, skin_t, opening_bottom)),
        origin=Origin(xyz=(0.0, skin_t / 2.0, opening_bottom / 2.0)),
        material=body_green,
        name="lower_skin",
    )
    side_w = (panel_w - opening_w) / 2.0
    for i, x in enumerate((-panel_w / 2.0 + side_w / 2.0, panel_w / 2.0 - side_w / 2.0)):
        body.visual(
            Box((side_w, skin_t, opening_h)),
            origin=Origin(xyz=(x, skin_t / 2.0, opening_z)),
            material=body_green,
            name=f"side_skin_{i}",
        )

    frame = BezelGeometry(
        (opening_w, opening_h),
        (0.98, 0.56),
        0.030,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.035,
        outer_corner_radius=0.055,
    )
    body.visual(
        mesh_from_geometry(frame, "perimeter_frame_mesh"),
        origin=Origin(xyz=(0.0, -0.015, opening_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_green,
        name="perimeter_frame",
    )

    gasket = BezelGeometry(
        (0.77, 0.35),
        (0.91, 0.49),
        0.004,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.030,
        outer_corner_radius=0.045,
    )
    body.visual(
        mesh_from_geometry(gasket, "gasket_ring_mesh"),
        origin=Origin(xyz=(0.0, -0.032, opening_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="gasket_ring",
    )

    # Visible fasteners and two lower latch keepers on the fixed frame.
    fastener_positions = [
        (-0.40, 0.665),
        (0.0, 0.675),
        (0.40, 0.665),
        (-0.40, 0.175),
        (0.0, 0.165),
        (0.40, 0.175),
        (-0.455, 0.325),
        (-0.455, 0.515),
        (0.455, 0.325),
        (0.455, 0.515),
    ]
    for i, (x, z) in enumerate(fastener_positions):
        body.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, -0.033, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"frame_bolt_{i}",
        )

    for i, x in enumerate((-0.28, 0.28)):
        body.visual(
            Box((0.14, 0.018, 0.025)),
            origin=Origin(xyz=(x, -0.039, 0.145)),
            material=dark_steel,
            name=f"keeper_{i}",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(x - 0.045, -0.050, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"keeper_bolt_{i}_0",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(x + 0.045, -0.050, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"keeper_bolt_{i}_1",
        )

    hinge_z = opening_top + 0.070
    hinge_y = -0.055
    body_hinge_segments = [(-0.39, 0.20), (0.0, 0.20), (0.39, 0.20)]
    for i, (x, length) in enumerate(body_hinge_segments):
        body.visual(
            Box((length, 0.012, 0.046)),
            origin=Origin(xyz=(x, -0.034, hinge_z + 0.005)),
            material=body_green,
            name=f"fixed_hinge_leaf_{i}",
        )
        body.visual(
            Cylinder(radius=0.017, length=length),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"fixed_knuckle_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.94, 0.028, 0.50)),
        # Child frame lies on the longitudinal hinge axis; the closed hatch
        # panel drops downward from it and sits just outside the gasket.
        origin=Origin(xyz=(0.0, 0.007, -0.270)),
        material=body_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.74, 0.008, 0.30)),
        origin=Origin(xyz=(0.0, -0.011, -0.285)),
        material=darker_green,
        name="pressed_panel",
    )
    lid.visual(
        Box((0.86, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, -0.088)),
        material=darker_green,
        name="upper_stiffener",
    )
    lid.visual(
        Box((0.86, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, -0.012, -0.482)),
        material=darker_green,
        name="lower_stiffener",
    )
    for i, x in enumerate((-0.43, 0.43)):
        lid.visual(
            Box((0.030, 0.010, 0.39)),
            origin=Origin(xyz=(x, -0.012, -0.285)),
            material=darker_green,
            name=f"side_stiffener_{i}",
        )

    lid_hinge_segments = [(-0.195, 0.17), (0.195, 0.17)]
    for i, (x, length) in enumerate(lid_hinge_segments):
        lid.visual(
            Box((length, 0.012, 0.055)),
            origin=Origin(xyz=(x, -0.012, -0.025)),
            material=body_green,
            name=f"moving_hinge_leaf_{i}",
        )
        lid.visual(
            Cylinder(radius=0.017, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"moving_knuckle_{i}",
        )

    latch_x_positions = (-0.28, 0.28)
    for i, x in enumerate(latch_x_positions):
        lid.visual(
            Cylinder(radius=0.044, length=0.008),
            origin=Origin(xyz=(x, -0.011, -0.455), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=darker_green,
            name=f"latch_pad_{i}",
        )

    hatch_hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    for i, (x, direction) in enumerate(((-0.28, 1.0), (0.28, -1.0))):
        dog = model.part(f"latch_dog_{i}")
        dog.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name="pivot_boss",
        )
        dog.visual(
            Box((0.17, 0.012, 0.034)),
            origin=Origin(xyz=(direction * 0.075, -0.024, 0.0)),
            material=dark_steel,
            name="dog_bar",
        )
        dog.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(direction * 0.145, -0.024, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name="rounded_tip",
        )
        model.articulation(
            f"lid_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=dog,
            origin=Origin(xyz=(x, -0.015, -0.455)),
            axis=(0.0, direction, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
        )

    # Keep a reference in metadata for tests/readability without adding mechanics.
    hatch_hinge.meta["role"] = "longitudinal roof-edge gullwing hinge"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body_frame")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_overlap(
        lid,
        body,
        axes="xz",
        elem_a="lid_panel",
        elem_b="perimeter_frame",
        min_overlap=0.45,
        name="closed lid covers the framed opening",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="y",
        positive_elem="gasket_ring",
        negative_elem="lid_panel",
        max_penetration=0.0005,
        max_gap=0.002,
        name="lid sits just proud of the gasket",
    )

    closed_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.20}):
        open_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "gullwing lid swings outward and upward",
        closed_panel is not None
        and open_panel is not None
        and open_panel[0][1] < closed_panel[0][1] - 0.25
        and open_panel[0][2] > closed_panel[0][2] + 0.25,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )

    for i in (0, 1):
        dog = object_model.get_part(f"latch_dog_{i}")
        latch = object_model.get_articulation(f"lid_to_latch_{i}")
        ctx.expect_contact(
            dog,
            lid,
            elem_a="pivot_boss",
            elem_b=f"latch_pad_{i}",
            contact_tol=0.001,
            name=f"latch dog {i} is seated on its local pivot pad",
        )
        rest_bar = ctx.part_element_world_aabb(dog, elem="dog_bar")
        with ctx.pose({latch: math.pi / 2.0}):
            turned_bar = ctx.part_element_world_aabb(dog, elem="dog_bar")
        ctx.check(
            f"latch dog {i} quarter-turns on the free edge",
            rest_bar is not None
            and turned_bar is not None
            and turned_bar[0][2] < rest_bar[0][2] - 0.09,
            details=f"rest_bar={rest_bar}, turned_bar={turned_bar}",
        )

    return ctx.report()


object_model = build_object_model()
