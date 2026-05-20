from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_service_access_panel")

    powder = model.material("warm_gray_powdercoat", rgba=(0.37, 0.40, 0.39, 1.0))
    darker = model.material("dark_recess", rgba=(0.08, 0.09, 0.09, 1.0))
    gasket = model.material("black_epdm_gasket", rgba=(0.005, 0.006, 0.005, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.78, 0.78, 0.72, 1.0))
    label = model.material("service_label", rgba=(0.10, 0.18, 0.22, 1.0))

    frame = model.part("frame")

    # Back-mounted corrosion-resistant frame surrounding a clear service opening.
    frame.visual(Box((0.080, 0.045, 0.840)), origin=Origin(xyz=(-0.320, 0.000, 0.000)), material=powder, name="hinge_stile")
    frame.visual(Box((0.080, 0.045, 0.840)), origin=Origin(xyz=(0.320, 0.000, 0.000)), material=powder, name="latch_stile")
    frame.visual(Box((0.720, 0.045, 0.080)), origin=Origin(xyz=(0.000, 0.000, 0.380)), material=powder, name="top_rail")
    frame.visual(Box((0.720, 0.045, 0.080)), origin=Origin(xyz=(0.000, 0.000, -0.380)), material=powder, name="bottom_rail")

    # Raised EPDM compression gasket just proud of the frame face.
    frame.visual(Box((0.030, 0.012, 0.670)), origin=Origin(xyz=(-0.285, 0.0285, 0.000)), material=gasket, name="gasket_hinge")
    frame.visual(Box((0.030, 0.012, 0.240)), origin=Origin(xyz=(0.285, 0.0285, 0.215)), material=gasket, name="gasket_latch_upper")
    frame.visual(Box((0.030, 0.012, 0.240)), origin=Origin(xyz=(0.285, 0.0285, -0.215)), material=gasket, name="gasket_latch_lower")
    frame.visual(Box((0.540, 0.012, 0.030)), origin=Origin(xyz=(0.000, 0.0285, 0.340)), material=gasket, name="gasket_top")
    frame.visual(Box((0.540, 0.012, 0.030)), origin=Origin(xyz=(0.000, 0.0285, -0.340)), material=gasket, name="gasket_bottom")

    # Drip-aware formed overhangs and gutters protect the opening and hinge.
    frame.visual(Box((0.790, 0.095, 0.032)), origin=Origin(xyz=(0.000, 0.044, 0.435)), material=powder, name="drip_cap")
    frame.visual(Box((0.790, 0.016, 0.038)), origin=Origin(xyz=(0.000, 0.084, 0.407)), material=powder, name="drip_lip")
    frame.visual(Box((0.030, 0.070, 0.760)), origin=Origin(xyz=(-0.382, 0.032, 0.000)), material=powder, name="hinge_gutter")
    frame.visual(Box((0.030, 0.060, 0.300)), origin=Origin(xyz=(0.370, 0.027, 0.230)), material=powder, name="latch_gutter_upper")
    frame.visual(Box((0.030, 0.060, 0.300)), origin=Origin(xyz=(0.370, 0.027, -0.230)), material=powder, name="latch_gutter_lower")
    frame.visual(Box((0.700, 0.060, 0.026)), origin=Origin(xyz=(0.000, 0.020, -0.430)), material=powder, name="sloped_sill")

    # Fixed half of the protected piano-style hinge.
    frame.visual(Box((0.044, 0.006, 0.730)), origin=Origin(xyz=(-0.352, 0.029, 0.000)), material=stainless, name="fixed_hinge_leaf")
    for idx, zc in enumerate((-0.245, 0.245)):
        frame.visual(Cylinder(radius=0.012, length=0.215), origin=Origin(xyz=(-0.325, 0.047, zc)), material=stainless, name=f"fixed_knuckle_{idx}")
        frame.visual(Box((0.020, 0.026, 0.205)), origin=Origin(xyz=(-0.342, 0.039, zc)), material=stainless, name=f"fixed_hinge_wrap_{idx}")

    # Latch keeper bracket on the frame side, tucked behind the closed door.
    frame.visual(Box((0.030, 0.024, 0.180)), origin=Origin(xyz=(0.348, 0.018, 0.000)), material=stainless, name="latch_keeper_plate")
    frame.visual(Box((0.060, 0.008, 0.030)), origin=Origin(xyz=(0.320, 0.024, 0.000)), material=stainless, name="latch_keeper_lip")

    screw_positions = [
        (-0.320, 0.0, 0.305),
        (-0.320, 0.0, -0.305),
        (0.320, 0.0, 0.305),
        (0.320, 0.0, -0.305),
        (-0.115, 0.0, 0.380),
        (0.115, 0.0, 0.380),
        (-0.115, 0.0, -0.380),
        (0.115, 0.0, -0.380),
    ]
    for idx, (x, _, z) in enumerate(screw_positions):
        frame.visual(
            Cylinder(radius=0.012, length=0.007),
            origin=Origin(xyz=(x, 0.026, z), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"frame_screw_{idx}",
        )
        frame.visual(
            Box((0.016, 0.0015, 0.003)),
            origin=Origin(xyz=(x, 0.030, z), rpy=(0.0, 0.0, pi / 8.0)),
            material=darker,
            name=f"screw_slot_{idx}",
        )

    door = model.part("door")

    # The door frame is placed on the hinge pin; the service panel extends in +X.
    door.visual(Box((0.620, 0.024, 0.740)), origin=Origin(xyz=(0.325, 0.000, 0.000)), material=powder, name="door_slab")
    door.visual(Box((0.650, 0.010, 0.030)), origin=Origin(xyz=(0.335, 0.017, 0.385)), material=powder, name="door_top_return")
    door.visual(Box((0.650, 0.010, 0.026)), origin=Origin(xyz=(0.335, 0.017, -0.380)), material=powder, name="door_bottom_return")
    door.visual(Box((0.026, 0.010, 0.740)), origin=Origin(xyz=(0.640, 0.017, 0.000)), material=powder, name="door_latch_return")
    door.visual(Box((0.024, 0.010, 0.740)), origin=Origin(xyz=(0.015, 0.017, 0.000)), material=powder, name="door_hinge_return")

    # Inner compression strip on the door back: it visibly closes against the frame gasket.
    door.visual(Box((0.540, 0.006, 0.020)), origin=Origin(xyz=(0.330, -0.009, 0.320)), material=gasket, name="door_seal_top")
    door.visual(Box((0.540, 0.006, 0.020)), origin=Origin(xyz=(0.330, -0.009, -0.320)), material=gasket, name="door_seal_bottom")
    door.visual(Box((0.020, 0.006, 0.650)), origin=Origin(xyz=(0.075, -0.009, 0.000)), material=gasket, name="door_seal_hinge")
    door.visual(Box((0.020, 0.006, 0.650)), origin=Origin(xyz=(0.585, -0.009, 0.000)), material=gasket, name="door_seal_latch")

    # Moving hinge leaf and center knuckle, sharing the same vertical pin line.
    door.visual(Box((0.045, 0.006, 0.710)), origin=Origin(xyz=(0.036, 0.014, 0.000)), material=stainless, name="moving_hinge_leaf")
    door.visual(Cylinder(radius=0.012, length=0.205), origin=Origin(xyz=(0.000, 0.000, 0.000)), material=stainless, name="moving_knuckle")
    door.visual(Box((0.026, 0.026, 0.195)), origin=Origin(xyz=(0.018, 0.006, 0.000)), material=stainless, name="moving_hinge_wrap")

    # Proud label plate and latch escutcheon make the service purpose and closure side clear.
    door.visual(Box((0.205, 0.006, 0.060)), origin=Origin(xyz=(0.330, 0.0135, 0.135)), material=label, name="service_badge")
    door.visual(Box((0.070, 0.006, 0.120)), origin=Origin(xyz=(0.540, 0.0135, 0.000)), material=stainless, name="latch_escutcheon")

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.000, 0.006, 0.000), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="latch_hub",
    )
    latch.visual(
        Box((0.034, 0.014, 0.140)),
        origin=Origin(xyz=(0.000, 0.014, 0.000)),
        material=stainless,
        name="turn_handle",
    )
    latch.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.000, -0.018, 0.000), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="latch_shaft",
    )
    latch.visual(
        Box((0.180, 0.006, 0.026)),
        origin=Origin(xyz=(0.065, -0.030, 0.000)),
        material=stainless,
        name="rear_cam",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-0.325, 0.047, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.75),
        motion_properties=MotionProperties(damping=0.15, friction=0.08),
    )

    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.540, 0.012, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=pi / 2.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.allow_overlap(
        latch,
        door,
        elem_a="latch_shaft",
        elem_b="door_slab",
        reason="The sealed quarter-turn latch shaft intentionally passes through the door skin.",
    )
    ctx.allow_overlap(
        latch,
        door,
        elem_a="latch_shaft",
        elem_b="latch_escutcheon",
        reason="The latch shaft is captured inside the sealed exterior escutcheon bushing.",
    )

    ctx.expect_gap(
        door,
        frame,
        axis="y",
        positive_elem="door_slab",
        negative_elem="gasket_latch_upper",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed door seats on the upper latch gasket",
    )
    ctx.expect_gap(
        door,
        frame,
        axis="y",
        positive_elem="door_slab",
        negative_elem="gasket_latch_lower",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed door seats on the lower latch gasket",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="xz",
        elem_a="door_slab",
        elem_b="gasket_latch_upper",
        min_overlap=0.020,
        name="door face covers the upper weather gasket",
    )
    ctx.expect_overlap(
        door,
        frame,
        axes="xz",
        elem_a="door_slab",
        elem_b="gasket_latch_lower",
        min_overlap=0.020,
        name="door face covers the lower weather gasket",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="xz",
        elem_a="latch_shaft",
        elem_b="door_slab",
        min_overlap=0.010,
        name="latch shaft is aligned through the door skin",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="xz",
        elem_a="latch_shaft",
        elem_b="latch_escutcheon",
        min_overlap=0.010,
        name="latch shaft is captured by the escutcheon bushing",
    )
    ctx.expect_contact(
        latch,
        frame,
        elem_a="rear_cam",
        elem_b="latch_keeper_lip",
        contact_tol=0.006,
        name="locked latch cam bears on the keeper lip",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from hinge side",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    locked_handle_aabb = ctx.part_element_world_aabb(latch, elem="turn_handle")
    with ctx.pose({latch_turn: pi / 2.0}):
        unlatched_handle_aabb = ctx.part_element_world_aabb(latch, elem="turn_handle")
    locked_handle_tall = (
        locked_handle_aabb is not None
        and (locked_handle_aabb[1][2] - locked_handle_aabb[0][2])
        > (locked_handle_aabb[1][0] - locked_handle_aabb[0][0]) + 0.050
    )
    unlatched_handle_wide = (
        unlatched_handle_aabb is not None
        and (unlatched_handle_aabb[1][0] - unlatched_handle_aabb[0][0])
        > (unlatched_handle_aabb[1][2] - unlatched_handle_aabb[0][2]) + 0.050
    )
    ctx.check(
        "quarter-turn handle rotates from locked to service position",
        locked_handle_tall and unlatched_handle_wide,
        details=f"locked={locked_handle_aabb}, unlatched={unlatched_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
