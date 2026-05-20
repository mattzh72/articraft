from __future__ import annotations

from math import atan2, pi, sqrt

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
    model = ArticulatedObject(name="classic_pickup_tailgate_with_rigid_stays")

    body_blue = Material("aged_blue_paint", color=(0.08, 0.22, 0.38, 1.0))
    inner_blue = Material("stamped_inner_blue", color=(0.05, 0.16, 0.28, 1.0))
    black = Material("black_liner", color=(0.015, 0.015, 0.013, 1.0))
    steel = Material("zinc_plated_steel", color=(0.62, 0.64, 0.62, 1.0))
    dark_steel = Material("blackened_stay_steel", color=(0.06, 0.065, 0.06, 1.0))
    rubber = Material("dark_rubber", color=(0.02, 0.02, 0.018, 1.0))

    hinge_z = 0.76
    gate_width = 1.56
    gate_len = 0.62
    gate_thickness = 0.055

    bed = model.part("bed_stubs")
    bed.visual(
        Box((0.50, 1.92, 0.06)),
        origin=Origin(xyz=(0.34, 0.0, hinge_z - 0.03)),
        material=black,
        name="bed_floor",
    )
    bed.visual(
        Box((0.62, 0.08, 0.58)),
        origin=Origin(xyz=(0.25, 0.92, hinge_z + 0.29)),
        material=body_blue,
        name="left_side_stub",
    )
    bed.visual(
        Box((0.62, 0.08, 0.58)),
        origin=Origin(xyz=(0.25, -0.92, hinge_z + 0.29)),
        material=body_blue,
        name="right_side_stub",
    )
    bed.visual(
        Box((0.52, 1.70, 0.035)),
        origin=Origin(xyz=(0.31, 0.0, hinge_z + 0.045)),
        material=black,
        name="bed_liner_lip",
    )

    for side, y in (("left", 0.66), ("right", -0.66)):
        bed.visual(
            Box((0.10, 0.13, 0.07)),
            origin=Origin(xyz=(0.075, y, hinge_z + 0.018)),
            material=steel,
            name=f"{side}_lower_hinge_plate",
        )
        bed.visual(
            Cylinder(radius=0.018, length=0.16),
            origin=Origin(xyz=(0.010, y, hinge_z), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"{side}_lower_hinge_pin",
        )

    for side, sign in (("left", 1.0), ("right", -1.0)):
        pivot_y = sign * 0.835
        bed.visual(
            Box((0.13, 0.045, 0.13)),
            origin=Origin(xyz=(0.32, sign * 0.890, 1.05)),
            material=steel,
            name=f"{side}_stay_bracket",
        )
        bed.visual(
            Cylinder(radius=0.019, length=0.085),
            origin=Origin(xyz=(0.32, pivot_y, 1.05), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name=f"{side}_stay_pivot_pin",
        )
        bed.visual(
            Box((0.045, 0.04, 0.16)),
            origin=Origin(xyz=(0.47, sign * 0.890, 1.10)),
            material=steel,
            name=f"{side}_latch_striker",
        )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((gate_len, gate_width - 0.04, gate_thickness)),
        origin=Origin(xyz=(-gate_len / 2 - 0.01, 0.0, 0.030)),
        material=body_blue,
        name="gate_panel",
    )
    tailgate.visual(
        Box((0.46, 1.18, 0.018)),
        origin=Origin(xyz=(-0.34, 0.0, 0.066)),
        material=inner_blue,
        name="stamped_recess",
    )
    tailgate.visual(
        Box((0.075, gate_width, 0.080)),
        origin=Origin(xyz=(-gate_len - 0.01, 0.0, 0.040)),
        material=body_blue,
        name="top_rail",
    )
    tailgate.visual(
        Box((gate_len, 0.045, 0.075)),
        origin=Origin(xyz=(-gate_len / 2 - 0.01, gate_width / 2, 0.040)),
        material=body_blue,
        name="left_edge_rail",
    )
    tailgate.visual(
        Box((gate_len, 0.045, 0.075)),
        origin=Origin(xyz=(-gate_len / 2 - 0.01, -gate_width / 2, 0.040)),
        material=body_blue,
        name="right_edge_rail",
    )
    tailgate.visual(
        Cylinder(radius=0.025, length=0.24),
        origin=Origin(xyz=(0.0, 0.58, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="left_hinge_barrel",
    )
    tailgate.visual(
        Cylinder(radius=0.025, length=0.24),
        origin=Origin(xyz=(0.0, -0.58, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="right_hinge_barrel",
    )
    tailgate.visual(
        Box((0.18, 0.38, 0.014)),
        origin=Origin(xyz=(-0.39, 0.0, 0.079)),
        material=inner_blue,
        name="handle_recess",
    )
    tailgate.visual(
        Cylinder(radius=0.010, length=1.20),
        origin=Origin(xyz=(-0.23, 0.0, 0.067), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="latch_rod",
    )

    gate_pin_x = -0.52
    gate_pin_z = 0.080
    for side, sign in (("left", 1.0), ("right", -1.0)):
        tailgate.visual(
            Cylinder(radius=0.018, length=0.070),
            origin=Origin(
                xyz=(gate_pin_x, sign * 0.835, gate_pin_z),
                rpy=(-pi / 2, 0.0, 0.0),
            ),
            material=steel,
            name=f"{side}_stay_pin",
        )

    lower_hinge = model.articulation(
        "lower_hinge",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.9, lower=0.0, upper=pi / 2),
    )
    lower_hinge.meta["description"] = "Horizontal lower hinge axis; q=0 is the supported lowered tailgate."

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.024, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.032), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="handle_pivot",
    )
    latch_handle.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="handle_spindle",
    )
    latch_handle.visual(
        Box((0.16, 0.30, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, 0.034)),
        material=rubber,
        name="pull_paddle",
    )
    latch_handle.visual(
        Box((0.06, 0.22, 0.038)),
        origin=Origin(xyz=(-0.035, 0.0, 0.055)),
        material=rubber,
        name="raised_grip",
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(xyz=(-0.39, 0.0, 0.086)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=0.75),
    )

    def add_stay(side: str, sign: float) -> None:
        pivot = (0.32, sign * 0.835, 1.05)
        gate_pin_world = (gate_pin_x, sign * 0.835, hinge_z + gate_pin_z)
        dx = gate_pin_world[0] - pivot[0]
        dz = gate_pin_world[2] - pivot[2]
        length = sqrt(dx * dx + dz * dz)
        theta = atan2(-dz / length, dx / length)

        stay = model.part(f"{side}_side_stay")
        stay.visual(
            Cylinder(radius=0.044, length=0.055),
            origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name="bed_eye",
        )
        stay.visual(
            Box((length - 0.18, 0.030, 0.034)),
            origin=Origin(xyz=(dx / 2.0, 0.0, dz / 2.0), rpy=(0.0, theta, 0.0)),
            material=dark_steel,
            name="flat_bar",
        )
        stay.visual(
            Box((0.070, 0.026, 0.052)),
            origin=Origin(
                xyz=(dx * 0.065 / length, 0.0, dz * 0.065 / length),
                rpy=(0.0, theta, 0.0),
            ),
            material=dark_steel,
            name="bed_eye_neck",
        )

        model.articulation(
            f"{side}_stay_bed_pivot",
            ArticulationType.REVOLUTE,
            parent=bed,
            child=stay,
            origin=Origin(xyz=pivot),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.25, upper=1.15),
        )

        eye = model.part(f"{side}_stay_eye")
        eye.visual(
            Cylinder(radius=0.048, length=0.058),
            origin=Origin(rpy=(-pi / 2, 0.0, 0.0)),
            material=dark_steel,
            name="eye_socket",
        )
        eye.visual(
            Box((0.044, 0.028, 0.040)),
            origin=Origin(
                xyz=(-dx * 0.068 / length, 0.0, -dz * 0.068 / length),
                rpy=(0.0, theta, 0.0),
            ),
            material=dark_steel,
            name="eye_tang",
        )
        eye.visual(
            Cylinder(radius=0.030, length=0.008),
            origin=Origin(xyz=(0.0, sign * 0.033, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
            material=steel,
            name="retainer_clip",
        )
        model.articulation(
            f"{side}_stay_gate_pivot",
            ArticulationType.REVOLUTE,
            parent=stay,
            child=eye,
            origin=Origin(xyz=(dx, 0.0, dz)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.90, upper=0.90),
        )

    add_stay("left", 1.0)
    add_stay("right", -1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tailgate = object_model.get_part("tailgate")
    bed = object_model.get_part("bed_stubs")
    lower_hinge = object_model.get_articulation("lower_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    for side in ("left", "right"):
        ctx.allow_overlap(
            bed,
            tailgate,
            elem_a=f"{side}_lower_hinge_pin",
            elem_b=f"{side}_hinge_barrel",
            reason="The lower corner hinge pin is intentionally seated inside the tailgate hinge barrel.",
        )
        ctx.expect_overlap(
            bed,
            tailgate,
            axes="y",
            elem_a=f"{side}_lower_hinge_pin",
            elem_b=f"{side}_hinge_barrel",
            min_overlap=0.10,
            name=f"{side} lower hinge pin runs through barrel",
        )

    for side in ("left", "right"):
        stay = object_model.get_part(f"{side}_side_stay")
        eye = object_model.get_part(f"{side}_stay_eye")
        bed_pin = f"{side}_stay_pivot_pin"
        gate_pin = f"{side}_stay_pin"

        ctx.allow_overlap(
            bed,
            stay,
            elem_a=bed_pin,
            elem_b="bed_eye",
            reason="The rigid stay eye is intentionally captured around the fixed bed-side pivot pin.",
        )
        ctx.expect_within(
            bed,
            stay,
            axes="xz",
            inner_elem=bed_pin,
            outer_elem="bed_eye",
            margin=0.002,
            name=f"{side} stay eye is centered on bed pivot",
        )
        ctx.expect_overlap(
            stay,
            bed,
            axes="y",
            elem_a="bed_eye",
            elem_b=bed_pin,
            min_overlap=0.040,
            name=f"{side} stay eye remains on bed pivot pin",
        )

        ctx.allow_overlap(
            tailgate,
            eye,
            elem_a=gate_pin,
            elem_b="eye_socket",
            reason="The stay's gate-end eye is clipped over the tailgate pivot pin in the supported lowered pose.",
        )
        ctx.expect_within(
            tailgate,
            eye,
            axes="xz",
            inner_elem=gate_pin,
            outer_elem="eye_socket",
            margin=0.016,
            name=f"{side} stay eye is centered on gate pin",
        )
        ctx.expect_overlap(
            tailgate,
            eye,
            axes="y",
            elem_a=gate_pin,
            elem_b="eye_socket",
            min_overlap=0.045,
            name=f"{side} gate pin is retained through stay eye",
        )
        ctx.allow_overlap(
            stay,
            eye,
            elem_a="flat_bar",
            elem_b="eye_tang",
            reason="The gate-end eye tang is intentionally seated against the stay bar at the local pivot joint.",
        )
        ctx.expect_overlap(
            stay,
            eye,
            axes="xz",
            elem_a="flat_bar",
            elem_b="eye_tang",
            min_overlap=0.006,
            name=f"{side} gate pivot tang is seated at stay end",
        )
        ctx.allow_overlap(
            tailgate,
            eye,
            elem_a=gate_pin,
            elem_b="retainer_clip",
            reason="The visible retainer clip snaps over the tailgate stay pin to keep the rigid stay captive.",
        )
        ctx.expect_overlap(
            tailgate,
            eye,
            axes="xyz",
            elem_a=gate_pin,
            elem_b="retainer_clip",
            min_overlap=0.005,
            name=f"{side} stay retainer clip captures gate pin",
        )

    lowered = ctx.part_element_world_aabb(tailgate, elem="gate_panel")
    with ctx.pose({lower_hinge: pi / 2}):
        raised = ctx.part_element_world_aabb(tailgate, elem="gate_panel")
    ctx.check(
        "tailgate rotates upward to closed height",
        lowered is not None
        and raised is not None
        and raised[1][2] > lowered[1][2] + 0.45,
        details=f"lowered={lowered}, raised={raised}",
    )

    handle_rest = ctx.part_world_aabb("latch_handle")
    with ctx.pose({handle_pivot: 0.65}):
        handle_lifted = ctx.part_world_aabb("latch_handle")
    ctx.check(
        "latch handle lifts away on its own pivot",
        handle_rest is not None
        and handle_lifted is not None
        and handle_lifted[1][2] > handle_rest[1][2] + 0.015,
        details=f"rest={handle_rest}, lifted={handle_lifted}",
    )

    return ctx.report()


object_model = build_object_model()
