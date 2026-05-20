from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _rail(
    part,
    *,
    name: str,
    x: float,
    foot_y: float,
    foot_z: float,
    top_y: float,
    top_z: float,
    width: float,
    depth: float,
    material: Material,
) -> None:
    """Add one rectangular ladder rail whose long local axis follows the Y-Z line."""
    dy = top_y - foot_y
    dz = top_z - foot_z
    length = math.sqrt(dy * dy + dz * dz)
    # A box is long along local +Z.  R_x(theta) maps local +Z to
    # (0, -sin(theta), cos(theta)).
    theta = -math.asin(dy / length)
    part.visual(
        Box((width, depth, length)),
        origin=Origin(
            xyz=(x, (foot_y + top_y) * 0.5, (foot_z + top_z) * 0.5),
            rpy=(theta, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def _cyl_x(part, *, name: str, center, radius: float, length: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cyl_y(part, *, name: str, center, radius: float, length: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_aframe_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("black_hardcoat", rgba=(0.03, 0.035, 0.04, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    brass = model.material("etched_brass_marks", rgba=(0.95, 0.72, 0.20, 1.0))
    blue = model.material("blue_adjusters", rgba=(0.05, 0.24, 0.78, 1.0))
    white = model.material("white_gap_labels", rgba=(0.92, 0.94, 0.90, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")

    # Root/front frame: two paired precision extrusions, four flat datum treads,
    # a broad top datum plate, rubber feet, etched scales, and hinge bosses.
    front_top_z = 1.50
    front_foot_z = 0.055
    front_foot_y = -0.45
    side_x = 0.265
    for sx, suffix in ((-side_x, "0"), (side_x, "1")):
        _rail(
            front,
            name=f"front_leg_{suffix}",
            x=sx,
            foot_y=front_foot_y,
            foot_z=front_foot_z,
            top_y=0.0,
            top_z=front_top_z,
            width=0.052,
            depth=0.044,
            material=aluminum,
        )
        front.visual(
            Box((0.18, 0.105, 0.045)),
            origin=Origin(xyz=(sx, front_foot_y - 0.020, 0.033)),
            material=rubber,
            name=f"front_foot_{suffix}",
        )
        # Outboard brace-pivot bracket, tied back into each front leg.
        front.visual(
            Box((0.116, 0.038, 0.064)),
            origin=Origin(xyz=(sx + math.copysign(0.055, sx), -0.258, 0.690)),
            material=aluminum,
            name=f"brace_bracket_{suffix}",
        )
        _cyl_x(
            front,
            name=f"brace_pivot_boss_{suffix}",
            center=(sx + math.copysign(0.102, sx), -0.258, 0.690),
            radius=0.020,
            length=0.030,
            material=dark,
        )
        # Vertical reference scale on the outside face of each front leg.
        for idx, z in enumerate((0.46, 0.58, 0.70, 0.82, 0.94, 1.06, 1.18)):
            rail_y = front_foot_y + (0.0 - front_foot_y) * ((z - front_foot_z) / (front_top_z - front_foot_z))
            front.visual(
                Box((0.004, 0.010 if idx % 2 else 0.016, 0.026)),
                origin=Origin(xyz=(sx + math.copysign(0.028, sx), rail_y - 0.018, z)),
                material=brass,
                name=f"leg_index_{suffix}_{idx}",
            )

    tread_levels = (0.36, 0.68, 1.00, 1.30)
    for i, z in enumerate(tread_levels):
        rail_y = front_foot_y + (0.0 - front_foot_y) * ((z - front_foot_z) / (front_top_z - front_foot_z))
        center_y = rail_y - 0.067
        front.visual(
            Box((0.650, 0.190, 0.036)),
            origin=Origin(xyz=(0.0, center_y, z)),
            material=aluminum,
            name=f"datum_tread_{i}",
        )
        front.visual(
            Box((0.650, 0.018, 0.060)),
            origin=Origin(xyz=(0.0, center_y - 0.095, z - 0.004)),
            material=dark,
            name=f"tread_lip_{i}",
        )
        for m, x in enumerate((-0.22, -0.11, 0.0, 0.11, 0.22)):
            front.visual(
                Box((0.011 if m != 2 else 0.018, 0.052, 0.004)),
                origin=Origin(xyz=(x, center_y - 0.030, z + 0.019)),
                material=brass,
                name=f"tread_index_{i}_{m}",
            )

    front.visual(
        Box((0.705, 0.250, 0.046)),
        origin=Origin(xyz=(0.0, -0.155, 1.475)),
        material=aluminum,
        name="top_platform",
    )
    front.visual(
        Box((0.440, 0.126, 0.008)),
        origin=Origin(xyz=(0.0, -0.155, 1.501)),
        material=dark,
        name="top_datum_plate",
    )
    front.visual(
        Box((0.180, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.040, 1.503)),
        material=white,
        name="zero_gap_label",
    )
    _cyl_x(
        front,
        name="front_hinge_barrel",
        center=(0.0, 0.0, front_top_z),
        radius=0.027,
        length=0.488,
        material=dark,
    )
    for sx, suffix in ((-0.340, "0"), (0.340, "1")):
        front.visual(
            Box((0.048, 0.075, 0.098)),
            origin=Origin(xyz=(sx - math.copysign(0.072, sx), -0.016, 1.482)),
            material=aluminum,
            name=f"hinge_cheek_{suffix}",
        )

    # Rear frame is the hinged back pair of legs.  It is intentionally a single
    # rigid part so the top hinge alone defines the A-frame spread state.
    rear_side_x = 0.340
    rear_foot_y = 0.670
    rear_foot_z = -1.455
    for sx, suffix in ((-rear_side_x, "0"), (rear_side_x, "1")):
        _rail(
            rear,
            name=f"rear_leg_{suffix}",
            x=sx,
            foot_y=rear_foot_y,
            foot_z=rear_foot_z,
            top_y=0.095,
            top_z=-0.095,
            width=0.052,
            depth=0.044,
            material=aluminum,
        )
        _rail(
            rear,
            name=f"rear_hinge_strut_{suffix}",
            x=sx,
            foot_y=0.0,
            foot_z=0.0,
            top_y=0.095,
            top_z=-0.095,
            width=0.050,
            depth=0.040,
            material=aluminum,
        )
        rear.visual(
            Box((0.076, 0.082, 0.082)),
            origin=Origin(xyz=(sx, 0.095, -0.095)),
            material=aluminum,
            name=f"rear_top_socket_{suffix}",
        )
        rear.visual(
            Box((0.180, 0.105, 0.045)),
            origin=Origin(xyz=(sx, rear_foot_y + 0.018, rear_foot_z - 0.026)),
            material=rubber,
            name=f"rear_foot_{suffix}",
        )
        _cyl_x(
            rear,
            name=f"rear_hinge_knuckle_{suffix}",
            center=(sx, 0.0, 0.0),
            radius=0.029,
            length=0.096,
            material=dark,
        )
        rear.visual(
            Box((0.046, 0.040, 0.060)),
            origin=Origin(xyz=(sx, 0.045, -0.030)),
            material=aluminum,
            name=f"rear_knuckle_web_{suffix}",
        )
        rear.visual(
            Box((0.098, 0.104, 0.038)),
            origin=Origin(xyz=(sx + math.copysign(0.048, sx), 0.397, -0.887)),
            material=aluminum,
            name=f"receiver_bridge_{suffix}",
        )
        rear.visual(
            Box((0.072, 0.052, 0.088)),
            origin=Origin(xyz=(sx + math.copysign(0.085, sx), 0.430, -0.850)),
            material=dark,
            name=f"brace_receiver_{suffix}",
        )
        rear.visual(
            Box((0.080, 0.010, 0.064)),
            origin=Origin(xyz=(sx + math.copysign(0.085, sx), 0.344, -0.850)),
            material=brass,
            name=f"receiver_datum_{suffix}",
        )
    rear.visual(
        Box((0.735, 0.050, 0.050)),
        origin=Origin(xyz=(0.0, 0.510, -1.105)),
        material=aluminum,
        name="rear_crossbar",
    )
    rear.visual(
        Box((0.705, 0.036, 0.040)),
        origin=Origin(xyz=(0.0, 0.220, -0.455)),
        material=aluminum,
        name="rear_tie_bar",
    )

    rear_joint = model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, front_top_z)),
        # Positive motion folds the rear feet forward toward the front frame.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.55, lower=0.0, upper=0.36),
        motion_properties=MotionProperties(damping=0.8, friction=0.25),
        meta={"lower_state": "open datum stop", "upper_state": "closed transport stop"},
    )

    # Two side spread-limit braces.  They are hinged to the front frame, linearly
    # mimicked from the main rear-leg pivot for a repeatable open/closed stack,
    # and each carries a prismatic fine stop plus a rotating blue lock knob.
    brace_pitch = -0.048
    brace_pivots = ((-0.367, "0", -1.0), (0.367, "1", 1.0))
    for x, suffix, sign in brace_pivots:
        brace = model.part(f"brace_{suffix}")
        stop = model.part(f"stop_{suffix}")
        knob = model.part(f"lock_knob_{suffix}")

        brace.visual(
            Box((0.026, 0.560, 0.020)),
            origin=Origin(xyz=(0.0, 0.296, 0.0)),
            material=dark,
            name="brace_bar",
        )
        _cyl_x(
            brace,
            name="brace_pivot_eye",
            center=(sign * 0.024, 0.0, 0.0),
            radius=0.026,
            length=0.018,
            material=dark,
        )
        brace.visual(
            Box((0.030, 0.050, 0.020)),
            origin=Origin(xyz=(0.0, 0.045, 0.0)),
            material=dark,
            name="pivot_neck",
        )
        for idx, y in enumerate((0.145, 0.225, 0.305, 0.385, 0.465)):
            brace.visual(
                Box((0.028, 0.006 if idx % 2 else 0.010, 0.005)),
                origin=Origin(xyz=(0.0, y, 0.0125)),
                material=brass,
                name=f"brace_index_{idx}",
            )

        stop.visual(
            Box((0.048, 0.084, 0.034)),
            origin=Origin(xyz=(0.0, -0.005, 0.027)),
            material=aluminum,
            name="slide_block",
        )
        stop.visual(
            Box((0.068, 0.012, 0.052)),
            origin=Origin(xyz=(sign * 0.055, 0.058, 0.028)),
            material=brass,
            name="gap_stop_face",
        )
        stop.visual(
            Box((0.012, 0.005, 0.058)),
            origin=Origin(xyz=(sign * 0.055, 0.0665, 0.060)),
            material=white,
            name="gap_pointer",
        )
        stop.visual(
            Box((0.014, 0.034, 0.030)),
            origin=Origin(xyz=(sign * 0.015, 0.039, 0.028)),
            material=aluminum,
            name="stop_neck",
        )
        stop.visual(
            Box((0.018, 0.022, 0.026)),
            origin=Origin(xyz=(sign * 0.025, 0.036, 0.052)),
            material=aluminum,
            name="screw_post",
        )
        _cyl_y(
            stop,
            name="fine_leadscrew",
            center=(sign * 0.025, 0.036, 0.065),
            radius=0.007,
            length=0.110,
            material=blue,
        )

        _cyl_x(
            knob,
            name="knob_disk",
            center=(sign * 0.011, 0.0, 0.0),
            radius=0.027,
            length=0.022,
            material=blue,
        )
        knob.visual(
            Box((0.010, 0.058, 0.010)),
            origin=Origin(xyz=(sign * 0.024, 0.0, 0.0)),
            material=blue,
            name="knob_grip",
        )

        model.articulation(
            f"front_to_brace_{suffix}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=brace,
            origin=Origin(xyz=(x, -0.258, 0.690), rpy=(brace_pitch, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=0.0, upper=1.08),
            motion_properties=MotionProperties(damping=0.35, friction=0.15),
            mimic=Mimic(joint=rear_joint.name, multiplier=3.0, offset=0.0),
        )
        model.articulation(
            f"brace_{suffix}_to_stop",
            ArticulationType.PRISMATIC,
            parent=brace,
            child=stop,
            origin=Origin(xyz=(0.0, 0.560, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=0.08, lower=-0.012, upper=0.038),
            motion_properties=MotionProperties(damping=0.6, friction=0.5),
            meta={"nominal_gap": "0.004 m visual controlled gap to receiver datum"},
        )
        model.articulation(
            f"stop_{suffix}_to_knob",
            ArticulationType.CONTINUOUS,
            parent=stop,
            child=knob,
            origin=Origin(xyz=(sign * 0.024, -0.005, 0.034)),
            axis=(sign, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    rear_joint = object_model.get_articulation("front_to_rear")

    # The visible hinge barrels are coaxial and very close to the top-frame
    # geometry, but intentionally arranged with axial clearances rather than
    # volumetric interpenetration.
    ctx.expect_origin_distance(front, rear, axes="xy", min_dist=0.0, max_dist=0.001, name="rear hinge shares top datum axis")
    ctx.expect_overlap(
        rear,
        front,
        axes="z",
        min_overlap=0.040,
        elem_a="rear_hinge_knuckle_0",
        elem_b="front_hinge_barrel",
        name="hinge knuckle sits on common barrel height",
    )

    for suffix in ("0", "1"):
        brace = object_model.get_part(f"brace_{suffix}")
        stop = object_model.get_part(f"stop_{suffix}")
        knob = object_model.get_part(f"lock_knob_{suffix}")
        stop_joint = object_model.get_articulation(f"brace_{suffix}_to_stop")

        ctx.allow_overlap(
            brace,
            stop,
            elem_a="brace_bar",
            elem_b="slide_block",
            reason="The fine-adjustment stop is modeled as a captured saddle sliding around the rectangular spread brace rail.",
        )
        ctx.allow_overlap(
            stop,
            knob,
            elem_a="fine_leadscrew",
            elem_b="knob_disk",
            reason="The blue lock knob is represented as a solid hub captured on the fine-adjust leadscrew.",
        )
        ctx.expect_contact(
            brace,
            stop,
            elem_a="brace_bar",
            elem_b="slide_block",
            contact_tol=0.002,
            name=f"stop_{suffix} rides on brace datum bar",
        )
        ctx.expect_contact(
            stop,
            knob,
            elem_a="slide_block",
            elem_b="knob_disk",
            contact_tol=0.004,
            name=f"lock_knob_{suffix} seats against adjustment block",
        )
        ctx.expect_overlap(
            stop,
            knob,
            axes="yz",
            elem_a="fine_leadscrew",
            elem_b="knob_disk",
            min_overlap=0.006,
            name=f"lock_knob_{suffix} is retained on leadscrew",
        )
        ctx.expect_overlap(
            brace,
            stop,
            axes="y",
            elem_a="brace_bar",
            elem_b="slide_block",
            min_overlap=0.025,
            name=f"stop_{suffix} retained on brace rail",
        )
        rest_pos = ctx.part_world_position(stop)
        with ctx.pose({stop_joint: 0.038}):
            ext_pos = ctx.part_world_position(stop)
        ctx.check(
            f"stop_{suffix} fine adjuster advances",
            rest_pos is not None and ext_pos is not None and ext_pos[1] > rest_pos[1] + 0.025,
            details=f"rest={rest_pos}, extended={ext_pos}",
        )

    brace_0 = object_model.get_part("brace_0")
    rest_rear = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
    rest_brace = ctx.part_element_world_aabb(brace_0, elem="brace_bar")
    with ctx.pose({rear_joint: 0.36}):
        folded_rear = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
        folded_brace = ctx.part_element_world_aabb(brace_0, elem="brace_bar")
    ctx.check(
        "rear frame folds toward front frame",
        rest_rear is not None and folded_rear is not None and folded_rear[1][1] < rest_rear[1][1] - 0.08,
        details=f"open={rest_rear}, folded={folded_rear}",
    )
    ctx.check(
        "spread brace mimic folds with rear pivot",
        rest_brace is not None and folded_brace is not None and folded_brace[1][1] < rest_brace[1][1] - 0.08,
        details=f"open={rest_brace}, folded={folded_brace}",
    )

    return ctx.report()


object_model = build_object_model()
