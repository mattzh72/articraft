from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    outer_w = 0.88
    outer_h = 1.00
    body_t = 0.05

    opening_w = 0.46
    opening_h = 0.72

    side_stile_w = (outer_w - opening_w) / 2.0
    top_rail_h = (outer_h - opening_h) / 2.0

    frame_trim_w = 0.03
    frame_trim_t = 0.012
    cheek_w = 0.02
    cheek_t = 0.03
    cheek_h = 0.76

    door_leaf_w = 0.424
    door_leaf_h = 0.704
    door_t = 0.018
    hinge_stile_w = 0.022
    latch_stile_w = 0.012

    hinge_axis_x = -0.222
    hinge_axis_y = 0.0445
    trim_y = body_t / 2.0 + frame_trim_t / 2.0
    cheek_y = body_t / 2.0 + cheek_t / 2.0

    housing = model.material("housing_gray", rgba=(0.62, 0.65, 0.68, 1.0))
    frame = model.material("frame_charcoal", rgba=(0.26, 0.28, 0.30, 1.0))
    panel = model.material("panel_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    hardware = model.material("hardware_dark", rgba=(0.17, 0.18, 0.19, 1.0))

    equipment_face = model.part("equipment_face")
    equipment_face.inertial = Inertial.from_geometry(
        Box((outer_w, body_t, outer_h)),
        mass=24.0,
    )

    equipment_face.visual(
        Box((side_stile_w, body_t, outer_h)),
        origin=Origin(xyz=(-(opening_w + side_stile_w) / 2.0, 0.0, 0.0)),
        material=housing,
        name="left_face_stile",
    )
    equipment_face.visual(
        Box((side_stile_w, body_t, outer_h)),
        origin=Origin(xyz=((opening_w + side_stile_w) / 2.0, 0.0, 0.0)),
        material=housing,
        name="right_face_stile",
    )
    equipment_face.visual(
        Box((opening_w, body_t, top_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, (opening_h + top_rail_h) / 2.0)),
        material=housing,
        name="top_face_rail",
    )
    equipment_face.visual(
        Box((opening_w, body_t, top_rail_h)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_h + top_rail_h) / 2.0)),
        material=housing,
        name="bottom_face_rail",
    )

    equipment_face.visual(
        Box((frame_trim_w, frame_trim_t, opening_h + 2.0 * frame_trim_w)),
        origin=Origin(xyz=(-(opening_w / 2.0 + frame_trim_w / 2.0), trim_y, 0.0)),
        material=frame,
        name="left_trim",
    )
    equipment_face.visual(
        Box((frame_trim_w, frame_trim_t, opening_h + 2.0 * frame_trim_w)),
        origin=Origin(xyz=((opening_w / 2.0 + frame_trim_w / 2.0), trim_y, 0.0)),
        material=frame,
        name="right_trim",
    )
    equipment_face.visual(
        Box((opening_w + 2.0 * frame_trim_w, frame_trim_t, frame_trim_w)),
        origin=Origin(xyz=(0.0, trim_y, opening_h / 2.0 + frame_trim_w / 2.0)),
        material=frame,
        name="top_trim",
    )
    equipment_face.visual(
        Box((opening_w + 2.0 * frame_trim_w, frame_trim_t, frame_trim_w)),
        origin=Origin(xyz=(0.0, trim_y, -(opening_h / 2.0 + frame_trim_w / 2.0))),
        material=frame,
        name="bottom_trim",
    )

    equipment_face.visual(
        Box((cheek_w, cheek_t, cheek_h)),
        origin=Origin(xyz=(-0.240, cheek_y, 0.0)),
        material=frame,
        name="left_cheek",
    )
    equipment_face.visual(
        Box((cheek_w, cheek_t, cheek_h)),
        origin=Origin(xyz=(0.240, cheek_y, 0.0)),
        material=frame,
        name="right_cheek",
    )
    equipment_face.visual(
        Cylinder(radius=0.010, length=0.18),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.24)),
        material=hardware,
        name="frame_hinge_upper",
    )
    equipment_face.visual(
        Cylinder(radius=0.010, length=0.18),
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, -0.24)),
        material=hardware,
        name="frame_hinge_lower",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((0.448, door_t, door_leaf_h)),
        mass=5.8,
        origin=Origin(xyz=(0.224, 0.0, 0.0)),
    )

    door.visual(
        Box((door_leaf_w, door_t, door_leaf_h)),
        origin=Origin(xyz=(0.236, 0.0, 0.0)),
        material=panel,
        name="door_skin",
    )
    door.visual(
        Box((hinge_stile_w, door_t + 0.004, door_leaf_h)),
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        material=panel,
        name="hinge_stile",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((latch_stile_w, door_t + 0.006, 0.58)),
        origin=Origin(xyz=(0.442, 0.002, 0.0)),
        material=hardware,
        name="latch_stile",
    )
    door.visual(
        Box((0.05, 0.012, 0.12)),
        origin=Origin(xyz=(0.401, 0.015, 0.0)),
        material=hardware,
        name="pull_handle",
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=equipment_face,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, hinge_axis_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    equipment_face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("frame_to_door")

    ctx.expect_gap(
        equipment_face,
        door,
        axis="x",
        positive_elem="right_cheek",
        negative_elem="door_skin",
        min_gap=0.004,
        max_gap=0.008,
        name="door keeps a narrow latch-side reveal",
    )
    ctx.expect_gap(
        equipment_face,
        door,
        axis="z",
        positive_elem="top_trim",
        negative_elem="door_skin",
        min_gap=0.006,
        max_gap=0.012,
        name="door keeps a narrow top reveal",
    )
    ctx.expect_overlap(
        equipment_face,
        door,
        axes="z",
        elem_a="right_cheek",
        elem_b="door_skin",
        min_overlap=0.68,
        name="door panel spans the framed opening height",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_skin")
    with ctx.pose({hinge: 1.15}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_skin")

    def center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    closed_center_y = center_y(closed_aabb)
    open_center_y = center_y(open_aabb)
    ctx.check(
        "positive hinge motion swings the door outward",
        closed_center_y is not None
        and open_center_y is not None
        and open_center_y > closed_center_y + 0.15,
        details=f"closed_center_y={closed_center_y}, open_center_y={open_center_y}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
