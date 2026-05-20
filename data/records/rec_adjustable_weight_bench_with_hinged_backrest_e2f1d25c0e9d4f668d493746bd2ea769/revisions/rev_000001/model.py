from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _y_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_adjustable_gym_bench")

    frame_metal = model.material("frame_metal", rgba=(0.17, 0.18, 0.19, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.58, 0.61, 0.65, 1.0))
    upholstery = model.material("upholstery", rgba=(0.09, 0.09, 0.10, 1.0))
    board = model.material("board", rgba=(0.14, 0.14, 0.15, 1.0))
    roller_foam = model.material("roller_foam", rgba=(0.11, 0.11, 0.12, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.46, 0.48, 0.50, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.12, 0.42, 0.05)),
        origin=Origin(xyz=(-0.24, 0.0, 0.025)),
        material=frame_metal,
        name="front_base",
    )
    frame.visual(
        Box((0.16, 0.42, 0.05)),
        origin=Origin(xyz=(0.64, 0.0, 0.025)),
        material=frame_metal,
        name="rear_base",
    )
    frame.visual(
        Box((0.04, 0.03, 0.35)),
        origin=Origin(xyz=(-0.18, 0.085, 0.20)),
        material=frame_metal,
        name="front_leg_0",
    )
    frame.visual(
        Box((0.04, 0.03, 0.35)),
        origin=Origin(xyz=(-0.18, -0.085, 0.20)),
        material=frame_metal,
        name="front_leg_1",
    )
    frame.visual(
        Box((0.04, 0.03, 0.34)),
        origin=Origin(xyz=(0.56, 0.085, 0.195)),
        material=frame_metal,
        name="rear_leg_0",
    )
    frame.visual(
        Box((0.04, 0.03, 0.34)),
        origin=Origin(xyz=(0.56, -0.085, 0.195)),
        material=frame_metal,
        name="rear_leg_1",
    )
    frame.visual(
        Box((0.74, 0.03, 0.04)),
        origin=Origin(xyz=(0.20, 0.085, 0.37)),
        material=frame_metal,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.74, 0.03, 0.04)),
        origin=Origin(xyz=(0.20, -0.085, 0.37)),
        material=frame_metal,
        name="side_rail_1",
    )
    frame.visual(
        Box((0.58, 0.05, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, 0.345)),
        material=frame_metal,
        name="center_spine",
    )
    frame.visual(
        Box((0.06, 0.17, 0.03)),
        origin=Origin(xyz=(-0.06, 0.0, 0.35)),
        material=frame_metal,
        name="crossmember_0",
    )
    frame.visual(
        Box((0.08, 0.17, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, 0.35)),
        material=frame_metal,
        name="crossmember_1",
    )
    frame.visual(
        Box((0.08, 0.17, 0.03)),
        origin=Origin(xyz=(0.42, 0.0, 0.35)),
        material=frame_metal,
        name="crossmember_2",
    )
    frame.visual(
        Box((0.03, 0.02, 0.07)),
        origin=Origin(xyz=(-0.15, 0.079, 0.40)),
        material=frame_metal,
        name="seat_hinge_tab_0",
    )
    frame.visual(
        Box((0.03, 0.02, 0.07)),
        origin=Origin(xyz=(-0.15, -0.079, 0.40)),
        material=frame_metal,
        name="seat_hinge_tab_1",
    )
    frame.visual(
        Box((0.05, 0.10, 0.03)),
        origin=Origin(xyz=(0.18, 0.125, 0.39)),
        material=frame_metal,
        name="back_hinge_outrigger_0",
    )
    frame.visual(
        Box((0.05, 0.10, 0.03)),
        origin=Origin(xyz=(0.18, -0.125, 0.39)),
        material=frame_metal,
        name="back_hinge_outrigger_1",
    )
    frame.visual(
        Box((0.03, 0.02, 0.07)),
        origin=Origin(xyz=(0.18, 0.14, 0.405)),
        material=frame_metal,
        name="back_hinge_tab_0",
    )
    frame.visual(
        Box((0.03, 0.02, 0.07)),
        origin=Origin(xyz=(0.18, -0.14, 0.405)),
        material=frame_metal,
        name="back_hinge_tab_1",
    )
    frame.visual(
        Box((0.08, 0.02, 0.10)),
        origin=Origin(xyz=(0.63, 0.08, 0.09)),
        material=frame_metal,
        name="ladder_pivot_tab_0",
    )
    frame.visual(
        Box((0.08, 0.02, 0.10)),
        origin=Origin(xyz=(0.63, -0.08, 0.09)),
        material=frame_metal,
        name="ladder_pivot_tab_1",
    )
    frame.visual(
        Box((0.04, 0.02, 0.18)),
        origin=Origin(xyz=(-0.30, 0.10, 0.14)),
        material=frame_metal,
        name="yoke_plate_0",
    )
    frame.visual(
        Box((0.04, 0.02, 0.18)),
        origin=Origin(xyz=(-0.30, -0.10, 0.14)),
        material=frame_metal,
        name="yoke_plate_1",
    )
    frame.visual(
        Box((0.03, 0.20, 0.02)),
        origin=Origin(xyz=(-0.295, 0.0, 0.225)),
        material=frame_metal,
        name="yoke_bridge",
    )
    frame.visual(
        Box((0.04, 0.02, 0.02)),
        origin=Origin(xyz=(-0.34, 0.10, 0.145)),
        material=frame_metal,
        name="yoke_cap_0",
    )
    frame.visual(
        Box((0.04, 0.02, 0.02)),
        origin=Origin(xyz=(-0.34, -0.10, 0.145)),
        material=frame_metal,
        name="yoke_cap_1",
    )
    frame.visual(
        Box((0.04, 0.02, 0.12)),
        origin=Origin(xyz=(0.72, 0.16, 0.09)),
        material=frame_metal,
        name="wheel_bracket_0",
    )
    frame.visual(
        Box((0.04, 0.02, 0.12)),
        origin=Origin(xyz=(0.72, -0.16, 0.09)),
        material=frame_metal,
        name="wheel_bracket_1",
    )
    frame.visual(
        Box((0.08, 0.02, 0.02)),
        origin=Origin(xyz=(0.76, 0.173, 0.08)),
        material=frame_metal,
        name="wheel_arm_0",
    )
    frame.visual(
        Box((0.08, 0.02, 0.02)),
        origin=Origin(xyz=(0.76, -0.173, 0.08)),
        material=frame_metal,
        name="wheel_arm_1",
    )

    seat_pad = model.part("seat_pad")
    seat_pad.visual(
        Box((0.31, 0.25, 0.05)),
        origin=Origin(xyz=(0.17, 0.0, 0.03)),
        material=upholstery,
        name="cushion",
    )
    seat_pad.visual(
        Box((0.30, 0.23, 0.008)),
        origin=Origin(xyz=(0.17, 0.0, 0.001)),
        material=board,
        name="backer",
    )
    seat_barrel_geom, seat_barrel_origin = _y_cylinder(0.010, 0.128)
    seat_pad.visual(
        seat_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, -0.010), rpy=seat_barrel_origin.rpy),
        material=satin_steel,
        name="hinge_barrel",
    )
    seat_pad.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.015, 0.045, -0.004)),
        material=satin_steel,
        name="hinge_cheek_0",
    )
    seat_pad.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.015, -0.045, -0.004)),
        material=satin_steel,
        name="hinge_cheek_1",
    )

    back_pad = model.part("back_pad")
    back_pad.visual(
        Box((0.72, 0.25, 0.05)),
        origin=Origin(xyz=(0.38, 0.0, 0.03)),
        material=upholstery,
        name="cushion",
    )
    back_pad.visual(
        Box((0.70, 0.23, 0.008)),
        origin=Origin(xyz=(0.38, 0.0, 0.001)),
        material=board,
        name="backer",
    )
    back_barrel_geom, back_barrel_origin = _y_cylinder(0.010, 0.128)
    back_pad.visual(
        back_barrel_geom,
        origin=Origin(xyz=(0.0, 0.0, -0.010), rpy=back_barrel_origin.rpy),
        material=satin_steel,
        name="hinge_barrel",
    )
    back_pad.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.015, 0.045, -0.004)),
        material=satin_steel,
        name="hinge_cheek_0",
    )
    back_pad.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.015, -0.045, -0.004)),
        material=satin_steel,
        name="hinge_cheek_1",
    )
    back_pad.visual(
        Box((0.08, 0.14, 0.016)),
        origin=Origin(xyz=(0.39, 0.0, -0.012)),
        material=satin_steel,
        name="support_bar",
    )
    back_pad.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.375, 0.045, -0.004)),
        material=satin_steel,
        name="support_web_0",
    )
    back_pad.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.375, -0.045, -0.004)),
        material=satin_steel,
        name="support_web_1",
    )

    ladder_support = model.part("ladder_support")
    ladder_pivot_geom, ladder_pivot_origin = _y_cylinder(0.014, 0.14)
    ladder_support.visual(
        ladder_pivot_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=ladder_pivot_origin.rpy),
        material=satin_steel,
        name="pivot_barrel",
    )
    ladder_support.visual(
        Box((0.03, 0.016, 0.34)),
        origin=Origin(xyz=(-0.061, 0.06, 0.153), rpy=(0.0, -0.38, 0.0)),
        material=frame_metal,
        name="side_bar_0",
    )
    ladder_support.visual(
        Box((0.03, 0.016, 0.34)),
        origin=Origin(xyz=(-0.061, -0.06, 0.153), rpy=(0.0, -0.38, 0.0)),
        material=frame_metal,
        name="side_bar_1",
    )
    ladder_support.visual(
        Box((0.03, 0.12, 0.02)),
        origin=Origin(xyz=(-0.034, 0.0, 0.085)),
        material=frame_metal,
        name="rung_0",
    )
    ladder_support.visual(
        Box((0.03, 0.12, 0.02)),
        origin=Origin(xyz=(-0.072, 0.0, 0.165)),
        material=frame_metal,
        name="rung_1",
    )
    ladder_support.visual(
        Box((0.03, 0.12, 0.02)),
        origin=Origin(xyz=(-0.103, 0.0, 0.245)),
        material=frame_metal,
        name="rung_2",
    )
    ladder_support.visual(
        Box((0.09, 0.12, 0.018)),
        origin=Origin(xyz=(-0.118, 0.0, 0.303)),
        material=satin_steel,
        name="top_plate",
    )

    transport_wheels = model.part("transport_wheels")
    wheel_axle_geom, wheel_axle_origin = _y_cylinder(0.009, 0.30)
    transport_wheels.visual(
        wheel_axle_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=wheel_axle_origin.rpy),
        material=satin_steel,
        name="axle",
    )
    left_wheel_geom, left_wheel_origin = _y_cylinder(0.055, 0.030)
    transport_wheels.visual(
        left_wheel_geom,
        origin=Origin(xyz=(0.0, 0.145, 0.0), rpy=left_wheel_origin.rpy),
        material=wheel_rubber,
        name="wheel_0",
    )
    transport_wheels.visual(
        left_wheel_geom,
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=left_wheel_origin.rpy),
        material=wheel_rubber,
        name="wheel_1",
    )
    hub_geom, hub_origin = _y_cylinder(0.032, 0.036)
    transport_wheels.visual(
        hub_geom,
        origin=Origin(xyz=(0.0, 0.145, 0.0), rpy=hub_origin.rpy),
        material=hub_gray,
        name="hub_0",
    )
    transport_wheels.visual(
        hub_geom,
        origin=Origin(xyz=(0.0, -0.145, 0.0), rpy=hub_origin.rpy),
        material=hub_gray,
        name="hub_1",
    )

    front_rollers = model.part("front_rollers")
    roller_axle_geom, roller_axle_origin = _y_cylinder(0.008, 0.18)
    front_rollers.visual(
        roller_axle_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=roller_axle_origin.rpy),
        material=satin_steel,
        name="axle",
    )
    roller_geom, roller_origin = _y_cylinder(0.042, 0.075)
    front_rollers.visual(
        roller_geom,
        origin=Origin(xyz=(0.0, 0.047, 0.0), rpy=roller_origin.rpy),
        material=roller_foam,
        name="roller_0",
    )
    front_rollers.visual(
        roller_geom,
        origin=Origin(xyz=(0.0, -0.047, 0.0), rpy=roller_origin.rpy),
        material=roller_foam,
        name="roller_1",
    )
    spacer_geom, spacer_origin = _y_cylinder(0.016, 0.022)
    front_rollers.visual(
        spacer_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=spacer_origin.rpy),
        material=hub_gray,
        name="spacer",
    )

    model.articulation(
        "seat_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=seat_pad,
        origin=Origin(xyz=(-0.15, 0.0, 0.405)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.5, lower=0.0, upper=0.34),
    )
    model.articulation(
        "back_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=back_pad,
        origin=Origin(xyz=(0.18, 0.0, 0.41)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.3, lower=0.0, upper=1.22),
    )
    model.articulation(
        "ladder_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=ladder_support,
        origin=Origin(xyz=(0.66, 0.0, 0.07)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-0.85, upper=0.30),
    )
    model.articulation(
        "transport_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=transport_wheels,
        origin=Origin(xyz=(0.80, 0.0, 0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "front_roller_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_rollers,
        origin=Origin(xyz=(-0.34, 0.0, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    seat_pad = object_model.get_part("seat_pad")
    back_pad = object_model.get_part("back_pad")
    ladder_support = object_model.get_part("ladder_support")
    transport_wheels = object_model.get_part("transport_wheels")
    front_rollers = object_model.get_part("front_rollers")

    seat_hinge = object_model.get_articulation("seat_hinge")
    back_hinge = object_model.get_articulation("back_hinge")
    ladder_pivot = object_model.get_articulation("ladder_pivot")
    transport_spin = object_model.get_articulation("transport_spin")
    front_roller_spin = object_model.get_articulation("front_roller_spin")

    def elem_max_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        return None if aabb is None else aabb[1][2]

    limits = back_hinge.motion_limits
    seat_limits = seat_hinge.motion_limits
    ladder_limits = ladder_pivot.motion_limits

    ctx.check(
        "primary joints use expected articulation types",
        back_hinge.articulation_type == ArticulationType.REVOLUTE
        and seat_hinge.articulation_type == ArticulationType.REVOLUTE
        and ladder_pivot.articulation_type == ArticulationType.REVOLUTE
        and transport_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_roller_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"back={back_hinge.articulation_type}, seat={seat_hinge.articulation_type}, "
            f"ladder={ladder_pivot.articulation_type}, wheels={transport_spin.articulation_type}, "
            f"rollers={front_roller_spin.articulation_type}"
        ),
    )
    ctx.check(
        "spin axes run across the bench width",
        tuple(transport_spin.axis) == (0.0, 1.0, 0.0) and tuple(front_roller_spin.axis) == (0.0, 1.0, 0.0),
        details=f"wheel_axis={transport_spin.axis}, roller_axis={front_roller_spin.axis}",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({back_hinge: limits.lower}):
            ctx.expect_gap(
                back_pad,
                seat_pad,
                axis="x",
                positive_elem="cushion",
                negative_elem="cushion",
                min_gap=0.008,
                max_gap=0.030,
                name="flat pads keep a small upholstery gap",
            )
            ctx.expect_overlap(
                back_pad,
                seat_pad,
                axes="y",
                elem_a="cushion",
                elem_b="cushion",
                min_overlap=0.20,
                name="flat pads stay aligned across bench width",
            )
            flat_back_z = elem_max_z(back_pad, "cushion")
        with ctx.pose({back_hinge: limits.upper}):
            raised_back_z = elem_max_z(back_pad, "cushion")
        ctx.check(
            "back pad raises clearly at upper limit",
            flat_back_z is not None and raised_back_z is not None and raised_back_z > flat_back_z + 0.22,
            details=f"flat_back_z={flat_back_z}, raised_back_z={raised_back_z}",
        )

    if seat_limits is not None and seat_limits.lower is not None and seat_limits.upper is not None:
        with ctx.pose({seat_hinge: seat_limits.lower}):
            flat_seat_z = elem_max_z(seat_pad, "cushion")
        with ctx.pose({seat_hinge: seat_limits.upper}):
            raised_seat_z = elem_max_z(seat_pad, "cushion")
        ctx.check(
            "seat pad tilts upward at its upper limit",
            flat_seat_z is not None and raised_seat_z is not None and raised_seat_z > flat_seat_z + 0.045,
            details=f"flat_seat_z={flat_seat_z}, raised_seat_z={raised_seat_z}",
        )

    ctx.expect_overlap(
        ladder_support,
        back_pad,
        axes="xy",
        elem_a="top_plate",
        elem_b="support_bar",
        min_overlap=0.05,
        name="ladder top plate stays under the back support bar",
    )
    ctx.expect_gap(
        back_pad,
        ladder_support,
        axis="z",
        positive_elem="support_bar",
        negative_elem="top_plate",
        min_gap=0.0,
        max_gap=0.02,
        name="support bar sits just above the ladder top plate",
    )

    if ladder_limits is not None and ladder_limits.lower is not None:
        current_ladder_z = elem_max_z(ladder_support, "top_plate")
        with ctx.pose({ladder_pivot: ladder_limits.lower}):
            folded_ladder_z = elem_max_z(ladder_support, "top_plate")
        ctx.check(
            "ladder support folds down from its visible support position",
            current_ladder_z is not None and folded_ladder_z is not None and folded_ladder_z < current_ladder_z - 0.12,
            details=f"current_ladder_z={current_ladder_z}, folded_ladder_z={folded_ladder_z}",
        )

    ctx.expect_overlap(
        front_rollers,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="yoke_bridge",
        min_overlap=0.16,
        name="front roller axle spans the yoke width",
    )
    ctx.expect_overlap(
        transport_wheels,
        frame,
        axes="y",
        elem_a="axle",
        elem_b="rear_base",
        min_overlap=0.24,
        name="transport wheel axle spans the rear frame width",
    )

    return ctx.report()


object_model = build_object_model()
