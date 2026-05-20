from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_service_access_panel")

    powder_coat = Material("powder_coat", rgba=(0.48, 0.52, 0.54, 1.0))
    frame_paint = Material("darker_frame_paint", rgba=(0.20, 0.23, 0.24, 1.0))
    door_paint = Material("slightly_lighter_door", rgba=(0.62, 0.65, 0.64, 1.0))
    rubber = Material("black_rubber_gasket", rgba=(0.015, 0.016, 0.014, 1.0))
    hardware = Material("dark_zinc_hardware", rgba=(0.08, 0.085, 0.09, 1.0))
    warning = Material("yellow_latch_marker", rgba=(0.95, 0.72, 0.10, 1.0))

    face_width = 1.10
    face_height = 0.78
    face_thickness = 0.040
    face_z = face_height / 2.0

    opening_center_x = -0.20
    opening_center_z = 0.40
    opening_w = 0.40
    opening_h = 0.50
    opening_left = opening_center_x - opening_w / 2.0
    opening_right = opening_center_x + opening_w / 2.0
    opening_bottom = opening_center_z - opening_h / 2.0
    opening_top = opening_center_z + opening_h / 2.0

    frame_border = 0.040
    frame_depth = 0.018
    frame_y = frame_depth / 2.0
    frame_outer_w = opening_w + 2.0 * frame_border
    frame_outer_h = opening_h + 2.0 * frame_border

    hinge_x = opening_left - 0.006
    hinge_y = 0.035
    hinge_z = opening_center_z

    face = model.part("equipment_face")

    # The fixed cabinet face is built as one connected sheet around an offset
    # opening. The much larger blank area on the latch side makes the fixed body
    # visibly heavier to one side, as on real equipment enclosures.
    face.visual(
        Box((face_width, face_thickness, face_height - opening_top)),
        origin=Origin(xyz=(0.0, -face_thickness / 2.0, (face_height + opening_top) / 2.0)),
        material=powder_coat,
        name="upper_face",
    )
    face.visual(
        Box((face_width, face_thickness, opening_bottom)),
        origin=Origin(xyz=(0.0, -face_thickness / 2.0, opening_bottom / 2.0)),
        material=powder_coat,
        name="lower_face",
    )
    left_margin = opening_left + face_width / 2.0
    right_margin = face_width / 2.0 - opening_right
    face.visual(
        Box((left_margin, face_thickness, opening_h)),
        origin=Origin(xyz=((-face_width / 2.0 + opening_left) / 2.0, -face_thickness / 2.0, opening_center_z)),
        material=powder_coat,
        name="narrow_face",
    )
    face.visual(
        Box((right_margin, face_thickness, opening_h)),
        origin=Origin(xyz=((opening_right + face_width / 2.0) / 2.0, -face_thickness / 2.0, opening_center_z)),
        material=powder_coat,
        name="wide_face",
    )

    # Raised rectangular frame around the access opening.
    face.visual(
        Box((frame_outer_w, frame_depth, frame_border)),
        origin=Origin(xyz=(opening_center_x, frame_y, opening_top + frame_border / 2.0)),
        material=frame_paint,
        name="frame_top",
    )
    face.visual(
        Box((frame_outer_w, frame_depth, frame_border)),
        origin=Origin(xyz=(opening_center_x, frame_y, opening_bottom - frame_border / 2.0)),
        material=frame_paint,
        name="frame_bottom",
    )
    face.visual(
        Box((frame_border, frame_depth, opening_h)),
        origin=Origin(xyz=(opening_left - frame_border / 2.0, frame_y, opening_center_z)),
        material=frame_paint,
        name="hinge_jamb",
    )
    face.visual(
        Box((frame_border, frame_depth, opening_h)),
        origin=Origin(xyz=(opening_right + frame_border / 2.0, frame_y, opening_center_z)),
        material=frame_paint,
        name="latch_jamb",
    )

    # A dark reveal gasket is visible in the clearance between the door and the
    # frame, making the center read as a real service opening rather than a solid
    # plate.
    gasket_depth = 0.006
    gasket_y = 0.003
    door_w = 0.360
    door_h = 0.460
    door_left_clearance = 0.016
    door_left_x = hinge_x + door_left_clearance
    door_right_x = door_left_x + door_w
    face.visual(
        Box((opening_w, gasket_depth, 0.018)),
        origin=Origin(xyz=(opening_center_x, gasket_y, door_left_x * 0.0 + (opening_top - 0.009))),
        material=rubber,
        name="gasket_top",
    )
    face.visual(
        Box((opening_w, gasket_depth, 0.018)),
        origin=Origin(xyz=(opening_center_x, gasket_y, opening_bottom + 0.009)),
        material=rubber,
        name="gasket_bottom",
    )
    face.visual(
        Box((0.018, gasket_depth, door_h)),
        origin=Origin(xyz=(opening_left + 0.009, gasket_y, opening_center_z)),
        material=rubber,
        name="gasket_hinge",
    )
    face.visual(
        Box((0.018, gasket_depth, door_h)),
        origin=Origin(xyz=(opening_right - 0.009, gasket_y, opening_center_z)),
        material=rubber,
        name="gasket_latch",
    )

    # Fixed hinge leaf and two fixed knuckles of an exposed side hinge.
    face.visual(
        Box((0.070, 0.006, 0.110)),
        origin=Origin(xyz=(hinge_x - 0.035, 0.021, hinge_z + 0.190)),
        material=hardware,
        name="fixed_leaf_top",
    )
    face.visual(
        Box((0.070, 0.006, 0.110)),
        origin=Origin(xyz=(hinge_x - 0.035, 0.021, hinge_z - 0.190)),
        material=hardware,
        name="fixed_leaf_bottom",
    )
    face.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + 0.190)),
        material=hardware,
        name="fixed_knuckle_top",
    )
    face.visual(
        Cylinder(radius=0.014, length=0.120),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z - 0.190)),
        material=hardware,
        name="fixed_knuckle_bottom",
    )

    # Strike plate mounted on the latch jamb opposite the hinge.
    face.visual(
        Box((0.032, 0.008, 0.105)),
        origin=Origin(xyz=(opening_right + frame_border / 2.0, 0.022, opening_center_z)),
        material=hardware,
        name="strike_plate",
    )
    face.visual(
        Box((0.020, 0.006, 0.042)),
        origin=Origin(xyz=(opening_right + 0.010, 0.028, opening_center_z)),
        material=hardware,
        name="strike_lip",
    )

    # Subtle screw heads on the frame help scale and make the service-panel
    # construction read as bolted sheet metal.
    screw_points = (
        (opening_left - 0.020, opening_top + 0.020),
        (opening_right + 0.020, opening_top + 0.020),
        (opening_left - 0.020, opening_bottom - 0.020),
        (opening_right + 0.020, opening_bottom - 0.020),
    )
    for index, (sx, sz) in enumerate(screw_points):
        face.visual(
            Cylinder(radius=0.011, length=0.004),
            origin=Origin(xyz=(sx, 0.020, sz), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hardware,
            name=f"frame_screw_{index}",
        )

    door = model.part("door")
    door_center_x = door_left_clearance + door_w / 2.0

    door.visual(
        Box((door_w, 0.024, door_h)),
        origin=Origin(xyz=(door_center_x, 0.0, 0.0)),
        material=door_paint,
        name="door_slab",
    )
    door.visual(
        Box((0.024, 0.006, door_h - 0.040)),
        origin=Origin(xyz=(door_left_clearance + door_w - 0.012, 0.015, 0.0)),
        material=hardware,
        name="latch_edge",
    )
    door.visual(
        Box((0.030, 0.006, 0.045)),
        origin=Origin(xyz=(door_left_clearance + door_w + 0.015, 0.018, 0.0)),
        material=hardware,
        name="latch_tongue",
    )
    door.visual(
        Box((0.115, 0.004, 0.036)),
        origin=Origin(xyz=(door_left_clearance + door_w - 0.078, 0.016, -0.165)),
        material=warning,
        name="service_label",
    )

    # Moving hinge leaf and center knuckle are part of the door and rotate with it.
    door.visual(
        Box((0.085, 0.006, 0.215)),
        origin=Origin(xyz=(0.044, -0.010, 0.0)),
        material=hardware,
        name="moving_leaf",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware,
        name="moving_knuckle",
    )

    # Non-articulated pull/quarter-turn detail on the latch edge. The prompt
    # specifies only the door hinge joint, so this remains fixed to the door.
    door.visual(
        Cylinder(radius=0.008, length=0.155),
        origin=Origin(xyz=(door_left_clearance + door_w - 0.050, 0.037, 0.050)),
        material=hardware,
        name="pull_bar",
    )
    door.visual(
        Box((0.012, 0.030, 0.018)),
        origin=Origin(xyz=(door_left_clearance + door_w - 0.050, 0.020, 0.105)),
        material=hardware,
        name="pull_standoff_0",
    )
    door.visual(
        Box((0.012, 0.030, 0.018)),
        origin=Origin(xyz=(door_left_clearance + door_w - 0.050, 0.020, -0.005)),
        material=hardware,
        name="pull_standoff_1",
    )

    model.articulation(
        "face_to_door",
        ArticulationType.REVOLUTE,
        parent=face,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("face_to_door")

    ctx.expect_gap(
        door,
        face,
        axis="y",
        positive_elem="door_slab",
        negative_elem="frame_top",
        min_gap=0.002,
        max_gap=0.012,
        name="closed door sits just proud of the frame",
    )
    ctx.expect_overlap(
        door,
        face,
        axes="z",
        elem_a="door_slab",
        elem_b="gasket_hinge",
        min_overlap=0.40,
        name="door height spans the framed opening",
    )

    wide = ctx.part_element_world_aabb(face, elem="wide_face")
    narrow = ctx.part_element_world_aabb(face, elem="narrow_face")
    if wide is not None and narrow is not None:
        wide_width = wide[1][0] - wide[0][0]
        narrow_width = narrow[1][0] - narrow[0][0]
        ctx.check(
            "opening is offset with a heavier latch side",
            wide_width > 2.5 * narrow_width,
            details=f"wide side {wide_width:.3f} m, narrow side {narrow_width:.3f} m",
        )
    else:
        ctx.fail("opening is offset with a heavier latch side", "missing face side aabb")

    closed = ctx.part_element_world_aabb(door, elem="latch_edge")
    with ctx.pose({hinge: 1.20}):
        opened = ctx.part_element_world_aabb(door, elem="latch_edge")
    if closed is not None and opened is not None:
        closed_center_y = (closed[0][1] + closed[1][1]) / 2.0
        opened_center_y = (opened[0][1] + opened[1][1]) / 2.0
        ctx.check(
            "latch edge swings outward about the vertical side hinge",
            opened_center_y > closed_center_y + 0.20,
            details=f"closed_y={closed_center_y:.3f}, opened_y={opened_center_y:.3f}",
        )
    else:
        ctx.fail("latch edge swings outward about the vertical side hinge", "missing latch edge aabb")

    return ctx.report()


object_model = build_object_model()
