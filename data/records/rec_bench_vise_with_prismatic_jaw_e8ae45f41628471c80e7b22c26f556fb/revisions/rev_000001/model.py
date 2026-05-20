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
    model = ArticulatedObject(name="angle_vise")

    cast_iron = Material("blue_gray_cast_iron", rgba=(0.10, 0.16, 0.22, 1.0))
    dark_iron = Material("dark_blued_iron", rgba=(0.03, 0.035, 0.04, 1.0))
    machined_steel = Material("brushed_machined_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    black_oxide = Material("black_oxide_steel", rgba=(0.01, 0.01, 0.012, 1.0))
    brass = Material("engraved_brass_scale", rgba=(0.80, 0.57, 0.20, 1.0))

    # World frame: X runs rear-to-front through the vise, Y is jaw width,
    # Z is up.  The tilt hinge is the rear trunnion axis at x=-0.22 m.
    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.70, 0.34, 0.035)),
        origin=Origin(xyz=(0.08, 0.0, 0.0175)),
        material=cast_iron,
        name="floor_plate",
    )
    # Low pads make the foundation read as a bolted machine-tool base.
    for i, x in enumerate((-0.18, 0.30)):
        for j, y in enumerate((-0.115, 0.115)):
            base_plate.visual(
                Cylinder(radius=0.020, length=0.006),
                origin=Origin(xyz=(x, y, 0.038)),
                material=black_oxide,
                name=f"bolt_head_{i}_{j}",
            )

    # Rear trunnion cheeks that carry the tilting upper frame.
    for side, y in enumerate((-0.128, 0.128)):
        base_plate.visual(
            Box((0.095, 0.040, 0.140)),
            origin=Origin(xyz=(-0.22, y, 0.105)),
            material=cast_iron,
            name=f"trunnion_cheek_{side}",
        )
        base_plate.visual(
            Cylinder(radius=0.047, length=0.044),
            origin=Origin(xyz=(-0.22, y, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_iron,
            name=f"trunnion_boss_{side}",
        )
        # Small external pin caps sit proud of the cheeks without entering the
        # moving centre barrel, avoiding a hidden collision while still reading
        # as a real hinge pin assembly.
        cap_y = -0.1565 if y < 0 else 0.1565
        base_plate.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(-0.22, cap_y, 0.105), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined_steel,
            name=f"pin_cap_{side}",
        )

    # A simple brass angle reference plate on the side of the fixed base.
    base_plate.visual(
        Box((0.010, 0.006, 0.110)),
        origin=Origin(xyz=(-0.045, -0.171, 0.086), rpy=(0.0, -0.45, 0.0)),
        material=brass,
        name="angle_scale_plate",
    )
    for i, z in enumerate((0.060, 0.075, 0.090, 0.105, 0.120)):
        base_plate.visual(
            Box((0.026 if i % 2 == 0 else 0.016, 0.004, 0.0025)),
            origin=Origin(xyz=(-0.045 + 0.028 * (z - 0.09), -0.1745, z), rpy=(0.0, -0.45, 0.0)),
            material=dark_iron,
            name=f"angle_tick_{i}",
        )

    tilt_frame = model.part("tilt_frame")
    # Child frame is the trunnion axis.  At q=0 the upper vise bed is level and
    # extends along local +X from the rear hinge.
    tilt_frame.visual(
        Cylinder(radius=0.034, length=0.212),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="center_hinge_barrel",
    )
    tilt_frame.visual(
        Box((0.500, 0.200, 0.035)),
        origin=Origin(xyz=(0.250, 0.0, 0.030)),
        material=cast_iron,
        name="tilting_bed",
    )
    tilt_frame.visual(
        Box((0.120, 0.180, 0.030)),
        origin=Origin(xyz=(0.060, 0.0, 0.018)),
        material=dark_iron,
        name="rear_hinge_web",
    )
    tilt_frame.visual(
        Box((0.430, 0.024, 0.018)),
        origin=Origin(xyz=(0.265, -0.075, 0.0555)),
        material=machined_steel,
        name="slide_rail_0",
    )
    tilt_frame.visual(
        Box((0.430, 0.024, 0.018)),
        origin=Origin(xyz=(0.265, 0.075, 0.0555)),
        material=machined_steel,
        name="slide_rail_1",
    )
    # A fixed front jaw is cast into the tilting frame, with a replaceable
    # machined face plate and serrations facing the moving jaw.
    tilt_frame.visual(
        Box((0.062, 0.230, 0.160)),
        origin=Origin(xyz=(0.474, 0.0, 0.1275)),
        material=cast_iron,
        name="fixed_jaw_body",
    )
    tilt_frame.visual(
        Box((0.008, 0.205, 0.070)),
        origin=Origin(xyz=(0.439, 0.0, 0.1575)),
        material=machined_steel,
        name="fixed_face_plate",
    )
    for i, z in enumerate((0.1365, 0.1565, 0.1765)):
        tilt_frame.visual(
            Box((0.006, 0.178, 0.004)),
            origin=Origin(xyz=(0.434, 0.0, z)),
            material=dark_iron,
            name=f"fixed_tooth_{i}",
        )
    # Side locking shoe follows the tilting frame along the angle scale.
    tilt_frame.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.190, -0.121, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="angle_lock_stem",
    )
    tilt_frame.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.190, -0.138, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="angle_lock_knob",
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.118, 0.172, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_iron,
        name="slide_foot",
    )
    moving_jaw.visual(
        Box((0.070, 0.205, 0.108)),
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        material=cast_iron,
        name="moving_jaw_body",
    )
    moving_jaw.visual(
        Box((0.008, 0.190, 0.066)),
        origin=Origin(xyz=(0.039, 0.0, 0.093)),
        material=machined_steel,
        name="moving_face_plate",
    )
    for i, z in enumerate((0.073, 0.093, 0.113)):
        moving_jaw.visual(
            Box((0.006, 0.166, 0.004)),
            origin=Origin(xyz=(0.044, 0.0, z)),
            material=dark_iron,
            name=f"moving_tooth_{i}",
        )
    moving_jaw.visual(
        Box((0.052, 0.105, 0.040)),
        origin=Origin(xyz=(-0.050, 0.0, 0.050)),
        material=cast_iron,
        name="screw_boss",
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.009, length=0.160),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="lead_screw",
    )
    screw_handle.visual(
        Cylinder(radius=0.007, length=0.155),
        origin=Origin(xyz=(-0.154, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="tommy_bar",
    )
    for side, y in enumerate((-0.0825, 0.0825)):
        screw_handle.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(xyz=(-0.154, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined_steel,
            name=f"handle_end_{side}",
        )

    model.articulation(
        "base_to_tilt",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=tilt_frame,
        origin=Origin(xyz=(-0.22, 0.0, 0.105)),
        # Positive angle lifts the front of the upper vise bed.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.6, lower=0.0, upper=math.radians(45.0)),
    )
    model.articulation(
        "tilt_to_jaw",
        ArticulationType.PRISMATIC,
        parent=tilt_frame,
        child=moving_jaw,
        # The child frame rides on the top of the guide rails; positive travel
        # moves the moving jaw toward the fixed front jaw.
        origin=Origin(xyz=(0.230, 0.0, 0.0645)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.12, lower=0.0, upper=0.160),
    )
    model.articulation(
        "jaw_to_screw",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=screw_handle,
        origin=Origin(xyz=(-0.035, 0.0, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tilt = object_model.get_part("tilt_frame")
    jaw = object_model.get_part("moving_jaw")
    screw = object_model.get_part("screw_handle")
    tilt_joint = object_model.get_articulation("base_to_tilt")
    jaw_slide = object_model.get_articulation("tilt_to_jaw")
    screw_spin = object_model.get_articulation("jaw_to_screw")

    ctx.allow_overlap(
        jaw,
        screw,
        elem_a="screw_boss",
        elem_b="lead_screw",
        reason="The lead screw is intentionally shown entering the threaded boss of the moving jaw.",
    )

    with ctx.pose({tilt_joint: 0.0, jaw_slide: 0.0, screw_spin: 0.0}):
        ctx.expect_gap(
            jaw,
            tilt,
            axis="z",
            positive_elem="slide_foot",
            negative_elem="slide_rail_0",
            max_gap=0.001,
            max_penetration=0.0001,
            name="sliding foot sits on the left guide rail",
        )
        ctx.expect_gap(
            jaw,
            tilt,
            axis="z",
            positive_elem="slide_foot",
            negative_elem="slide_rail_1",
            max_gap=0.001,
            max_penetration=0.0001,
            name="sliding foot sits on the right guide rail",
        )
        ctx.expect_gap(
            tilt,
            jaw,
            axis="x",
            positive_elem="fixed_face_plate",
            negative_elem="moving_face_plate",
            min_gap=0.150,
            max_gap=0.190,
            name="jaw is open at retracted slide limit",
        )
        ctx.expect_overlap(
            jaw,
            tilt,
            axes="yz",
            elem_a="moving_face_plate",
            elem_b="fixed_face_plate",
            min_overlap=0.060,
            name="jaw plates face one another across their width and height",
        )
        ctx.expect_contact(
            screw,
            jaw,
            elem_a="lead_screw",
            elem_b="screw_boss",
            contact_tol=0.002,
            name="lead screw seats against the moving jaw boss",
        )

    bed_closed = ctx.part_element_world_aabb(tilt, elem="tilting_bed")
    with ctx.pose({tilt_joint: math.radians(35.0), jaw_slide: 0.0, screw_spin: 0.0}):
        bed_tilted = ctx.part_element_world_aabb(tilt, elem="tilting_bed")
    ctx.check(
        "tilt hinge raises the front of the vise bed",
        bed_closed is not None
        and bed_tilted is not None
        and bed_tilted[1][2] > bed_closed[1][2] + 0.12,
        details=f"closed_aabb={bed_closed}, tilted_aabb={bed_tilted}",
    )

    with ctx.pose({tilt_joint: 0.0, jaw_slide: 0.160, screw_spin: math.pi / 2.0}):
        ctx.expect_gap(
            tilt,
            jaw,
            axis="x",
            positive_elem="fixed_face_plate",
            negative_elem="moving_face_plate",
            min_gap=0.001,
            max_gap=0.015,
            name="jaw approaches fixed jaw at full slide travel without collision",
        )
        ctx.expect_overlap(
            jaw,
            tilt,
            axes="xy",
            elem_a="slide_foot",
            elem_b="slide_rail_0",
            min_overlap=0.015,
            name="sliding jaw remains retained on the rails at full travel",
        )

    return ctx.report()


object_model = build_object_model()
