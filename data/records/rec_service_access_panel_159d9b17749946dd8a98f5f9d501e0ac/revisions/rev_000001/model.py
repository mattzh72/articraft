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
    model = ArticulatedObject(name="hinged_service_access_panel")

    painted_shell = Material("blue_gray_painted_steel", rgba=(0.23, 0.29, 0.34, 1.0))
    raised_frame = Material("slightly_lighter_frame", rgba=(0.33, 0.39, 0.43, 1.0))
    door_paint = Material("matte_service_door", rgba=(0.46, 0.50, 0.52, 1.0))
    black_rubber = Material("black_rubber_gasket", rgba=(0.015, 0.016, 0.016, 1.0))
    hardware = Material("brushed_stainless_hardware", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_fastener = Material("dark_fastener", rgba=(0.05, 0.05, 0.045, 1.0))

    # Coordinate frame: the equipment face lies in XZ; +Y points outward toward
    # the technician.  The door hinge axis is a proud, vertical line on the
    # left side of the framed opening.
    face = model.part("equipment_face")

    # Main sheet-metal face with a real rectangular opening left in the center.
    face.visual(
        Box((0.210, 0.030, 0.620)),
        origin=Origin(xyz=(-0.275, -0.015, 0.0)),
        material=painted_shell,
        name="left_face_bar",
    )
    face.visual(
        Box((0.210, 0.030, 0.620)),
        origin=Origin(xyz=(0.275, -0.015, 0.0)),
        material=painted_shell,
        name="right_face_bar",
    )
    face.visual(
        Box((0.340, 0.030, 0.110)),
        origin=Origin(xyz=(0.0, -0.015, 0.255)),
        material=painted_shell,
        name="top_face_bar",
    )
    face.visual(
        Box((0.340, 0.030, 0.110)),
        origin=Origin(xyz=(0.0, -0.015, -0.255)),
        material=painted_shell,
        name="bottom_face_bar",
    )

    # Raised rectangular service-frame trim around the access opening.
    face.visual(
        Box((0.060, 0.014, 0.560)),
        origin=Origin(xyz=(-0.230, 0.005, 0.0)),
        material=raised_frame,
        name="left_frame_stile",
    )
    face.visual(
        Box((0.060, 0.014, 0.560)),
        origin=Origin(xyz=(0.230, 0.005, 0.0)),
        material=raised_frame,
        name="right_frame_stile",
    )
    face.visual(
        Box((0.400, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.005, 0.255)),
        material=raised_frame,
        name="top_frame_rail",
    )
    face.visual(
        Box((0.400, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.005, -0.255)),
        material=raised_frame,
        name="bottom_frame_rail",
    )

    # Dark gasket/void glimpsed behind the closed door at the framed opening.
    face.visual(
        Box((0.010, 0.006, 0.410)),
        origin=Origin(xyz=(-0.175, 0.002, 0.0)),
        material=black_rubber,
        name="left_gasket_lip",
    )
    face.visual(
        Box((0.010, 0.006, 0.410)),
        origin=Origin(xyz=(0.175, 0.002, 0.0)),
        material=black_rubber,
        name="right_gasket_lip",
    )
    face.visual(
        Box((0.350, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, 0.205)),
        material=black_rubber,
        name="top_gasket_lip",
    )
    face.visual(
        Box((0.350, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, -0.205)),
        material=black_rubber,
        name="bottom_gasket_lip",
    )

    # Fixed half of the hinge: two proud knuckles and broad support leaves that
    # visibly pull the pin axis forward from the equipment face.
    hinge_x = -0.235
    hinge_y = 0.055
    for i, z in enumerate((-0.160, 0.160)):
        face.visual(
            Box((0.052, 0.070, 0.082)),
            origin=Origin(xyz=(hinge_x, 0.027, z)),
            material=hardware,
            name=f"fixed_hinge_leaf_{i}",
        )
        face.visual(
            Cylinder(radius=0.012, length=0.105),
            origin=Origin(xyz=(hinge_x, hinge_y, z)),
            material=hardware,
            name=f"fixed_hinge_knuckle_{i}",
        )
        face.visual(
            Cylinder(radius=0.004, length=0.004),
            origin=Origin(xyz=(hinge_x - 0.012, 0.064, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_fastener,
            name=f"fixed_hinge_screw_{i}",
        )
    face.visual(
        Cylinder(radius=0.004, length=0.450),
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        material=dark_fastener,
        name="hinge_pin",
    )

    # Door part frame is exactly on the hinge pin axis.  At q=0 the access door
    # extends in local +X and sits just proud of the raised frame.
    door = model.part("door")
    door.visual(
        Box((0.360, 0.026, 0.420)),
        origin=Origin(xyz=(0.225, -0.033, 0.0)),
        material=door_paint,
        name="door_slab",
    )
    door.visual(
        Box((0.310, 0.006, 0.018)),
        origin=Origin(xyz=(0.230, -0.017, 0.170)),
        material=black_rubber,
        name="top_door_seam",
    )
    door.visual(
        Box((0.310, 0.006, 0.018)),
        origin=Origin(xyz=(0.230, -0.017, -0.170)),
        material=black_rubber,
        name="bottom_door_seam",
    )
    door.visual(
        Box((0.018, 0.006, 0.320)),
        origin=Origin(xyz=(0.365, -0.017, 0.0)),
        material=black_rubber,
        name="latch_edge_seam",
    )

    # Moving hinge leaf and center knuckle, overlapped locally with the door
    # slab so the child part is one supported assembly.
    door.visual(
        Box((0.076, 0.036, 0.112)),
        origin=Origin(xyz=(0.044, -0.018, 0.0)),
        material=hardware,
        name="moving_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hardware,
        name="moving_hinge_knuckle",
    )

    # Latch edge detail opposite the hinge: fixed on the door because the only
    # requested joint is the door's side hinge.
    door.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.365, -0.021, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="latch_boss",
    )
    door.visual(
        Box((0.075, 0.014, 0.018)),
        origin=Origin(xyz=(0.365, -0.013, 0.010)),
        material=hardware,
        name="latch_handle",
    )
    face.visual(
        Box((0.022, 0.018, 0.090)),
        origin=Origin(xyz=(0.205, 0.021, 0.010)),
        material=hardware,
        name="latch_strike",
    )

    model.articulation(
        "face_to_door",
        ArticulationType.REVOLUTE,
        parent=face,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    face = object_model.get_part("equipment_face")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("face_to_door")

    ctx.allow_overlap(
        face,
        door,
        elem_a="hinge_pin",
        elem_b="moving_hinge_knuckle",
        reason="The visible hinge pin is intentionally captured inside the moving barrel knuckle.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            face,
            axis="y",
            positive_elem="door_slab",
            negative_elem="left_face_bar",
            min_gap=0.006,
            max_gap=0.014,
            name="closed door stands just proud of the face",
        )
        ctx.expect_overlap(
            door,
            face,
            axes="xz",
            elem_a="door_slab",
            elem_b="left_gasket_lip",
            min_overlap=0.010,
            name="closed door covers the left gasket edge",
        )
        ctx.expect_within(
            face,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem="moving_hinge_knuckle",
            margin=0.001,
            name="hinge pin is coaxial inside the moving knuckle",
        )
        ctx.expect_overlap(
            face,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b="moving_hinge_knuckle",
            min_overlap=0.120,
            name="moving knuckle captures a long length of hinge pin",
        )

    fixed_knuckle_box = ctx.part_element_world_aabb(face, elem="fixed_hinge_knuckle_0")
    door_slab_box = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "hinge barrel is pulled outward from the door skin",
        fixed_knuckle_box is not None
        and door_slab_box is not None
        and fixed_knuckle_box[1][1] > door_slab_box[1][1] + 0.020,
        details=f"fixed_knuckle_aabb={fixed_knuckle_box}, door_slab_aabb={door_slab_box}",
    )

    rest_door_box = ctx.part_element_world_aabb(door, elem="door_slab")
    with ctx.pose({hinge: 1.25}):
        open_door_box = ctx.part_element_world_aabb(door, elem="door_slab")
    ctx.check(
        "positive hinge motion swings the latch edge outward",
        rest_door_box is not None
        and open_door_box is not None
        and open_door_box[1][1] > rest_door_box[1][1] + 0.17,
        details=f"rest={rest_door_box}, open={open_door_box}",
    )

    return ctx.report()


object_model = build_object_model()
