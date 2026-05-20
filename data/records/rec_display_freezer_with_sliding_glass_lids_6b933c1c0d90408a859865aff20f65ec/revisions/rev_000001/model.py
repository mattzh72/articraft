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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_island_freezer")

    white = model.material("insulated_white", rgba=(0.90, 0.94, 0.96, 1.0))
    liner = model.material("cold_liner", rgba=(0.72, 0.82, 0.88, 1.0))
    rail = model.material("brushed_aluminum", rgba=(0.58, 0.62, 0.64, 1.0))
    dark = model.material("black_gasket", rgba=(0.025, 0.030, 0.035, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.48, 0.76, 0.92, 0.38))
    control_dark = model.material("recessed_control_black", rgba=(0.04, 0.045, 0.05, 1.0))
    red = model.material("red_indicator", rgba=(0.9, 0.05, 0.03, 1.0))
    green = model.material("green_indicator", rgba=(0.05, 0.75, 0.18, 1.0))
    brass = model.material("brass_lock", rgba=(0.80, 0.62, 0.25, 1.0))

    body_length = 2.40
    body_width = 0.95
    body_height = 0.78
    rail_top_z = 0.83

    body_shape = (
        cq.Workplane("XY")
        .box(body_length, body_width, body_height)
        .edges("|Z")
        .fillet(0.045)
        .translate((0.0, 0.0, body_height / 2.0))
    )
    cavity_cut = cq.Workplane("XY").box(2.04, 0.58, 0.86).translate((0.0, 0.0, 0.59))
    body_shape = body_shape.cut(cavity_cut)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_shape, "insulated_open_body", tolerance=0.002),
        material=white,
        name="insulated_open_body",
    )
    body.visual(
        Box((2.22, 0.82, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=dark,
        name="black_plinth",
    )
    body.visual(
        Box((1.98, 0.52, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.169)),
        material=liner,
        name="inner_floor_liner",
    )

    # Four lengthwise bay rails divide the top into three separate sliding-lid lanes.
    for idx, x in enumerate((-1.05, -0.35, 0.35, 1.05)):
        body.visual(
            Box((0.030, 0.82, 0.050)),
            origin=Origin(xyz=(x, -0.02, 0.805)),
            material=rail,
            name=f"top_slide_rail_{idx}",
        )
    for idx, x in enumerate((-1.008, -0.392, -0.308, 0.308, 0.392, 1.008)):
        body.visual(
            Box((0.022, 0.82, 0.050)),
            origin=Origin(xyz=(x, -0.02, 0.805)),
            material=rail,
            name=f"lid_support_rail_{idx}",
        )
    for idx, y in enumerate((-0.43, 0.41)):
        body.visual(
            Box((2.15, 0.030, 0.042)),
            origin=Origin(xyz=(0.0, y, 0.801)),
            material=rail,
            name=f"end_track_stop_{idx}",
        )

    # Recessed control pocket on the front right top corner, outside the lid lane.
    body.visual(
        Box((0.14, 0.15, 0.010)),
        origin=Origin(xyz=(1.125, -0.380, 0.812)),
        material=control_dark,
        name="control_recess_floor",
    )
    body.visual(
        Box((0.16, 0.018, 0.018)),
        origin=Origin(xyz=(1.125, -0.464, 0.821)),
        material=rail,
        name="control_front_lip",
    )
    body.visual(
        Box((0.018, 0.15, 0.018)),
        origin=Origin(xyz=(1.045, -0.380, 0.821)),
        material=rail,
        name="control_side_lip_0",
    )
    body.visual(
        Box((0.018, 0.15, 0.018)),
        origin=Origin(xyz=(1.205, -0.380, 0.821)),
        material=rail,
        name="control_side_lip_1",
    )
    for name, x, mat in (("control_lamp_red", 1.155, red), ("control_lamp_green", 1.185, green)):
        body.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, -0.385, 0.818)),
            material=mat,
            name=name,
        )

    # Side-wall lock hardware that the rotating flap protects.
    hinge_x = -1.095
    lock_z = 0.440
    body.visual(
        Box((0.024, 0.028, 0.120)),
        origin=Origin(xyz=(hinge_x, -0.489, lock_z)),
        material=rail,
        name="lock_hinge_standoff",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(hinge_x + 0.062, -0.484, lock_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_cylinder",
    )
    body.visual(
        Box((0.006, 0.004, 0.032)),
        origin=Origin(xyz=(hinge_x + 0.062, -0.495, lock_z)),
        material=dark,
        name="key_slot",
    )

    lid_centers = (-0.70, 0.0, 0.70)
    for index, x in enumerate(lid_centers):
        lid = model.part(f"lid_{index}")
        # The child frame is the lower center of the framed panel at the closed position.
        lid.visual(
            Box((0.66, 0.042, 0.035)),
            origin=Origin(xyz=(0.0, -0.329, 0.0175)),
            material=rail,
            name="front_frame",
        )
        lid.visual(
            Box((0.66, 0.042, 0.035)),
            origin=Origin(xyz=(0.0, 0.329, 0.0175)),
            material=rail,
            name="rear_frame",
        )
        lid.visual(
            Box((0.044, 0.70, 0.035)),
            origin=Origin(xyz=(-0.308, 0.0, 0.0175)),
            material=rail,
            name="side_frame_0",
        )
        lid.visual(
            Box((0.044, 0.70, 0.035)),
            origin=Origin(xyz=(0.308, 0.0, 0.0175)),
            material=rail,
            name="side_frame_1",
        )
        lid.visual(
            Box((0.585, 0.625, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
            material=glass,
            name="glass_pane",
        )
        lid.visual(
            Box((0.11, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -0.315, 0.043)),
            material=dark,
            name="finger_pull",
        )
        model.articulation(
            f"body_to_lid_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=lid,
            origin=Origin(xyz=(x, -0.045, rail_top_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.20),
        )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(
        Box((0.125, 0.012, 0.095)),
        origin=Origin(xyz=(0.063, -0.006, 0.0)),
        material=rail,
        name="flap_plate",
    )
    lock_flap.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=rail,
        name="flap_hinge_barrel",
    )
    lock_flap.visual(
        Box((0.050, 0.004, 0.018)),
        origin=Origin(xyz=(0.078, -0.014, -0.026)),
        material=dark,
        name="flap_grip_lip",
    )
    model.articulation(
        "body_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lock_flap,
        origin=Origin(xyz=(hinge_x, -0.505, lock_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    thermostat = model.part("thermostat_knob")
    thermostat.visual(
        Cylinder(radius=0.027, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark,
        name="knob_cap",
    )
    thermostat.visual(
        Box((0.038, 0.005, 0.004)),
        origin=Origin(xyz=(0.006, 0.0, 0.026)),
        material=white,
        name="knob_pointer",
    )
    model.articulation(
        "body_to_thermostat_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=thermostat,
        origin=Origin(xyz=(1.105, -0.385, 0.817)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lids = [object_model.get_part(f"lid_{i}") for i in range(3)]
    lid_joints = [object_model.get_articulation(f"body_to_lid_{i}") for i in range(3)]
    lock_flap = object_model.get_part("lock_flap")
    lock_joint = object_model.get_articulation("body_to_lock_flap")
    thermostat = object_model.get_part("thermostat_knob")

    ctx.check(
        "three separate sliding lid assemblies",
        len(lids) == 3 and len(lid_joints) == 3,
        details="Expected three individually articulated glass-lid parts.",
    )

    for index, (lid, joint) in enumerate(zip(lids, lid_joints)):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.002,
            max_penetration=0.001,
            name=f"lid_{index} rides on top rails",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.45,
            name=f"lid_{index} covers its freezer bay",
        )
        rest_position = ctx.part_world_position(lid)
        with ctx.pose({joint: 0.20}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                max_gap=0.002,
                max_penetration=0.001,
                name=f"lid_{index} remains supported when slid",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="x",
                min_overlap=0.55,
                name=f"lid_{index} stays in its lane while sliding",
            )
            extended_position = ctx.part_world_position(lid)
        ctx.check(
            f"lid_{index} translates along the top rails",
            rest_position is not None
            and extended_position is not None
            and extended_position[1] > rest_position[1] + 0.18,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    ctx.expect_gap(
        thermostat,
        body,
        axis="z",
        positive_elem="knob_cap",
        negative_elem="control_recess_floor",
        max_gap=0.002,
        max_penetration=0.001,
        name="thermostat knob seated in recessed control area",
    )
    control_floor = ctx.part_element_world_aabb(body, elem="control_recess_floor")
    control_lip = ctx.part_element_world_aabb(body, elem="control_front_lip")
    ctx.check(
        "control area floor is recessed below its lip",
        control_floor is not None
        and control_lip is not None
        and control_floor[1][2] < control_lip[1][2] - 0.004,
        details=f"floor={control_floor}, lip={control_lip}",
    )

    with ctx.pose({lock_joint: 0.0}):
        ctx.expect_overlap(
            lock_flap,
            body,
            axes="xz",
            elem_a="flap_plate",
            elem_b="key_cylinder",
            min_overlap=0.040,
            name="closed lock flap covers key cylinder",
        )
        ctx.expect_gap(
            body,
            lock_flap,
            axis="y",
            positive_elem="key_cylinder",
            negative_elem="flap_plate",
            min_gap=0.004,
            max_gap=0.020,
            name="lock flap stands proud of side wall hardware",
        )
        closed_flap_aabb = ctx.part_element_world_aabb(lock_flap, elem="flap_plate")

    with ctx.pose({lock_joint: 1.20}):
        open_flap_aabb = ctx.part_element_world_aabb(lock_flap, elem="flap_plate")

    def aabb_center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    closed_y = aabb_center_y(closed_flap_aabb)
    open_y = aabb_center_y(open_flap_aabb)
    ctx.check(
        "lock flap rotates outward on side hinge",
        closed_y is not None and open_y is not None and open_y < closed_y - 0.04,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )

    return ctx.report()


object_model = build_object_model()
