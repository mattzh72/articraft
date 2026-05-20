from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    hw = width / 2.0
    hh = height / 2.0
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_service_access_panel")

    painted_steel = Material("painted_steel", color=(0.45, 0.48, 0.48, 1.0))
    frame_paint = Material("deep_frame_paint", color=(0.26, 0.30, 0.31, 1.0))
    door_paint = Material("door_paint", color=(0.72, 0.74, 0.70, 1.0))
    dark_gasket = Material("dark_gasket", color=(0.025, 0.028, 0.026, 1.0))
    hinge_metal = Material("hinge_metal", color=(0.15, 0.16, 0.17, 1.0))
    warning_yellow = Material("latch_yellow", color=(0.92, 0.68, 0.15, 1.0))

    model.materials.extend(
        [painted_steel, frame_paint, door_paint, dark_gasket, hinge_metal, warning_yellow]
    )

    # World axes: X is across the equipment face, Z is vertical, and +Y is out
    # from the face.  The tall/narrow sheet has a real through-opening so the
    # dark cavity is visible when the door swings away.
    face = model.part("equipment_face")
    face_rotation = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))

    body_w = 0.62
    body_h = 1.32
    opening_w = 0.34
    opening_h = 0.78
    outer_w = 0.50
    outer_h = 0.96

    face_panel = ExtrudeWithHolesGeometry(
        _rect_profile(body_w, body_h),
        [_rect_profile(opening_w, opening_h)],
        0.080,
    )
    face.visual(
        mesh_from_geometry(face_panel, "face_panel_cutout"),
        origin=Origin(xyz=(0.0, 0.0, 0.66), rpy=face_rotation.rpy),
        material=painted_steel,
        name="face_panel",
    )

    frame_ring = ExtrudeWithHolesGeometry(
        _rect_profile(outer_w, outer_h),
        [_rect_profile(opening_w, opening_h)],
        0.075,
    )
    face.visual(
        mesh_from_geometry(frame_ring, "deep_frame_ring"),
        # Rear of the raised frame touches the front of the equipment face.
        origin=Origin(xyz=(0.0, 0.0775, 0.66), rpy=face_rotation.rpy),
        material=frame_paint,
        name="frame_ring",
    )

    # The hinge side is deliberately deeper and wider, like a service-door jamb
    # carrying a full-height hinge load.
    face.visual(
        Box((0.070, 0.095, 1.020)),
        origin=Origin(xyz=(-0.276, 0.0875, 0.66)),
        material=frame_paint,
        name="hinge_jamb",
    )
    face.visual(
        Box((0.050, 0.018, 0.900)),
        origin=Origin(xyz=(-0.282, 0.139, 0.66)),
        material=hinge_metal,
        name="fixed_hinge_leaf",
    )
    face.visual(
        Cylinder(radius=0.015, length=0.900),
        origin=Origin(xyz=(-0.245, 0.139, 0.66)),
        material=hinge_metal,
        name="hinge_pin",
    )

    # Latch-side frame detail opposite the hinge line.
    face.visual(
        Box((0.050, 0.018, 0.520)),
        origin=Origin(xyz=(0.250, 0.124, 0.66)),
        material=hinge_metal,
        name="latch_strike",
    )
    face.visual(
        Box((0.030, 0.020, 0.125)),
        origin=Origin(xyz=(0.226, 0.142, 0.66)),
        material=warning_yellow,
        name="latch_keeper",
    )

    door = model.part("door")
    door_w = 0.435
    door_h = 0.870
    door_t = 0.026

    # The door part frame is exactly on the vertical hinge axis.  At q=0 the
    # panel extends along local +X and sits just proud of the raised frame.
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.038, 0.010, 0.020)),
        origin=Origin(xyz=(0.250, 0.018, door_h / 2.0 - 0.030)),
        material=dark_gasket,
        name="top_gasket",
    )
    door.visual(
        Box((door_w - 0.038, 0.010, 0.020)),
        origin=Origin(xyz=(0.250, 0.018, -door_h / 2.0 + 0.030)),
        material=dark_gasket,
        name="bottom_gasket",
    )
    door.visual(
        Box((0.020, 0.010, door_h - 0.080)),
        origin=Origin(xyz=(0.065, 0.018, 0.0)),
        material=dark_gasket,
        name="hinge_gasket",
    )
    door.visual(
        Box((0.022, 0.012, door_h - 0.070)),
        origin=Origin(xyz=(door_w + 0.010, 0.020, 0.0)),
        material=dark_gasket,
        name="latch_edge",
    )
    door.visual(
        Box((0.040, 0.012, door_h - 0.040)),
        origin=Origin(xyz=(0.035, -0.001, 0.0)),
        material=hinge_metal,
        name="moving_hinge_leaf",
    )
    door.visual(
        Box((0.045, 0.014, 0.160)),
        origin=Origin(xyz=(door_w - 0.010, 0.021, 0.0)),
        material=warning_yellow,
        name="latch_pad",
    )
    for i, z in enumerate((-0.345, -0.115, 0.115, 0.345)):
        door.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(0.075, 0.025, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"hinge_screw_{i}",
        )

    hinge = model.articulation(
        "face_to_door",
        ArticulationType.REVOLUTE,
        parent=face,
        child=door,
        origin=Origin(xyz=(-0.245, 0.139, 0.66)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    hinge.meta["description"] = "Vertical side hinge: positive rotation swings the service panel outward."

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("face_to_door")

    ctx.check(
        "single vertical revolute door hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (0.0, 0.0, 1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.expect_gap(
        door,
        face,
        axis="y",
        min_gap=0.004,
        max_gap=0.020,
        positive_elem="door_panel",
        negative_elem="frame_ring",
        name="closed door sits proud of the deep frame",
    )
    ctx.expect_overlap(
        door,
        face,
        axes="xz",
        min_overlap=0.30,
        elem_a="door_panel",
        elem_b="frame_ring",
        name="door covers the framed service opening",
    )

    rest_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "positive hinge angle swings door outward",
        rest_aabb is not None
        and open_aabb is not None
        and (open_aabb[1][1] - rest_aabb[1][1]) > 0.25,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
