from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_workstation")

    powder = model.material("powder_coated_dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    rail_mat = model.material("satin_aluminum_rail", rgba=(0.62, 0.64, 0.62, 1.0))
    carriage_mat = model.material("cast_graphite_carriage", rgba=(0.16, 0.17, 0.18, 1.0))
    board_mat = model.material("warm_laminated_board", rgba=(0.78, 0.70, 0.56, 1.0))
    edge_mat = model.material("pale_hardwood_edge", rgba=(0.86, 0.78, 0.62, 1.0))
    ledge_mat = model.material("brushed_aluminum_ledge", rgba=(0.70, 0.72, 0.70, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))

    # X is desk depth (front is +X), Y is board width, Z is height.
    floor_frame = model.part("floor_frame")
    floor_frame.visual(Box((0.05, 1.15, 0.05)), origin=Origin(xyz=(0.38, 0.0, 0.025)), material=powder, name="front_tube")
    floor_frame.visual(Box((0.05, 1.15, 0.05)), origin=Origin(xyz=(-0.38, 0.0, 0.025)), material=powder, name="rear_tube")
    floor_frame.visual(Box((0.80, 0.05, 0.05)), origin=Origin(xyz=(0.0, 0.55, 0.025)), material=powder, name="side_tube_0")
    floor_frame.visual(Box((0.80, 0.05, 0.05)), origin=Origin(xyz=(0.0, -0.55, 0.025)), material=powder, name="side_tube_1")
    floor_frame.visual(Box((0.08, 1.10, 0.05)), origin=Origin(xyz=(-0.22, 0.0, 0.025)), material=powder, name="rail_crossmember")
    for index, (x, y) in enumerate(((0.38, 0.55), (0.38, -0.55), (-0.38, 0.55), (-0.38, -0.55))):
        floor_frame.visual(Cylinder(radius=0.035, length=0.02), origin=Origin(xyz=(x, y, 0.010)), material=rubber, name=f"leveling_foot_{index}")

    support_rail = model.part("support_rail")
    support_rail.visual(Box((0.20, 0.22, 0.025)), origin=Origin(xyz=(-0.22, 0.0, 0.0625)), material=powder, name="base_plate")
    support_rail.visual(Box((0.060, 0.045, 1.275)), origin=Origin(xyz=(-0.22, 0.0, 0.7125)), material=rail_mat, name="upright_rail")
    support_rail.visual(Box((0.006, 0.012, 1.08)), origin=Origin(xyz=(-0.187, 0.0, 0.74)), material=carriage_mat, name="front_index_slot")

    carriage = model.part("rail_carriage")
    carriage.visual(Box((0.095, 0.018, 0.26)), origin=Origin(xyz=(0.0, 0.055, 0.0)), material=carriage_mat, name="side_plate_0")
    carriage.visual(Box((0.095, 0.018, 0.26)), origin=Origin(xyz=(0.0, -0.055, 0.0)), material=carriage_mat, name="side_plate_1")
    carriage.visual(Box((0.020, 0.130, 0.26)), origin=Origin(xyz=(0.052, 0.0, 0.0)), material=carriage_mat, name="front_bridge")
    carriage.visual(Box((0.020, 0.130, 0.26)), origin=Origin(xyz=(-0.052, 0.0, 0.0)), material=carriage_mat, name="rear_bridge")
    carriage.visual(Box((0.050, 0.120, 0.080)), origin=Origin(xyz=(0.085, 0.0, 0.080)), material=carriage_mat, name="head_mount_pad")
    carriage.visual(Box((0.007, 0.030, 0.105)), origin=Origin(xyz=(0.0395, 0.0, 0.050)), material=rubber, name="front_guide_pad")
    carriage.visual(Box((0.013, 0.030, 0.105)), origin=Origin(xyz=(-0.0365, 0.0, 0.050)), material=rubber, name="rear_guide_pad")
    carriage.visual(Box((0.030, 0.025, 0.105)), origin=Origin(xyz=(0.0, 0.0350, -0.055)), material=rubber, name="side_guide_pad_0")
    carriage.visual(Box((0.030, 0.025, 0.105)), origin=Origin(xyz=(0.0, -0.0350, -0.055)), material=rubber, name="side_guide_pad_1")

    tilt_head = model.part("tilt_head")
    tilt_head.visual(Box((0.105, 0.120, 0.060)), origin=Origin(xyz=(-0.0775, 0.0, -0.060)), material=carriage_mat, name="forward_arm")
    tilt_head.visual(Box((0.055, 0.270, 0.080)), origin=Origin(xyz=(-0.010, 0.0, -0.045)), material=carriage_mat, name="head_block")
    tilt_head.visual(Box((0.080, 0.018, 0.058)), origin=Origin(xyz=(0.0, 0.125, -0.010)), material=carriage_mat, name="fork_cheek_0")
    tilt_head.visual(Box((0.080, 0.018, 0.058)), origin=Origin(xyz=(0.0, -0.125, -0.010)), material=carriage_mat, name="fork_cheek_1")
    tilt_head.visual(
        Cylinder(radius=0.018, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=powder,
        name="hinge_barrel",
    )

    board = model.part("board")
    board.visual(Box((0.90, 1.35, 0.035)), origin=Origin(xyz=(0.39, 0.0, 0.0525)), material=board_mat, name="drafting_surface")
    board.visual(Box((0.925, 0.030, 0.045)), origin=Origin(xyz=(0.39, 0.690, 0.058)), material=edge_mat, name="edge_band_0")
    board.visual(Box((0.925, 0.030, 0.045)), origin=Origin(xyz=(0.39, -0.690, 0.058)), material=edge_mat, name="edge_band_1")
    board.visual(Box((0.035, 1.35, 0.045)), origin=Origin(xyz=(-0.075, 0.0, 0.058)), material=edge_mat, name="hinge_edge_band")
    board.visual(Box((0.080, 1.20, 0.025)), origin=Origin(xyz=(0.805, 0.0, 0.0825)), material=ledge_mat, name="storage_ledge_shelf")
    board.visual(Box((0.025, 1.20, 0.090)), origin=Origin(xyz=(0.855, 0.0, 0.115)), material=ledge_mat, name="storage_ledge_lip")
    board.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(0.0, 0.194, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=powder,
        name="hinge_lug_0",
    )
    board.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(0.0, -0.194, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=powder,
        name="hinge_lug_1",
    )
    board.visual(Box((0.135, 0.052, 0.035)), origin=Origin(xyz=(0.055, 0.194, 0.0225)), material=powder, name="hinge_strap_0")
    board.visual(Box((0.135, 0.052, 0.035)), origin=Origin(xyz=(0.055, -0.194, 0.0225)), material=powder, name="hinge_strap_1")

    clamp_knob = model.part("clamp_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.090,
            0.045,
            body_style="lobed",
            base_diameter=0.070,
            top_diameter=0.082,
            crown_radius=0.003,
            bore=KnobBore(style="round", diameter=0.014),
        ),
        "lobed_clamp_knob",
    )
    clamp_knob.visual(knob_mesh, origin=Origin(xyz=(0.0, 0.056, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=rubber, name="lobed_grip")
    clamp_knob.visual(Cylinder(radius=0.008, length=0.090), origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=powder, name="threaded_stem")

    model.articulation("frame_to_rail", ArticulationType.FIXED, parent=floor_frame, child=support_rail, origin=Origin())
    model.articulation(
        "rail_height_slide",
        ArticulationType.PRISMATIC,
        parent=support_rail,
        child=carriage,
        origin=Origin(xyz=(-0.22, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.18, lower=-0.12, upper=0.24),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.FIXED,
        parent=carriage,
        child=tilt_head,
        origin=Origin(xyz=(0.24, 0.0, 0.14)),
    )
    model.articulation(
        "head_to_board",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=board,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.7, lower=0.0, upper=1.25),
    )
    model.articulation(
        "head_to_knob",
        ArticulationType.CONTINUOUS,
        parent=tilt_head,
        child=clamp_knob,
        origin=Origin(xyz=(0.0, 0.150, -0.077)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=3.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    rail = object_model.get_part("support_rail")
    carriage = object_model.get_part("rail_carriage")
    head = object_model.get_part("tilt_head")
    board = object_model.get_part("board")
    knob = object_model.get_part("clamp_knob")
    slide = object_model.get_articulation("rail_height_slide")
    board_hinge = object_model.get_articulation("head_to_board")
    knob_spin = object_model.get_articulation("head_to_knob")

    ctx.allow_overlap(
        knob,
        head,
        elem_a="threaded_stem",
        elem_b="head_block",
        reason="The clamp knob's threaded stem is intentionally seated through the tilt-head boss while the hand knob remains outside the bracket.",
    )

    ctx.expect_contact(
        carriage,
        rail,
        elem_a="side_guide_pad_0",
        elem_b="upright_rail",
        contact_tol=0.001,
        name="carriage guide pad bears on rail",
    )
    ctx.expect_contact(
        head,
        carriage,
        elem_a="forward_arm",
        elem_b="head_mount_pad",
        contact_tol=0.001,
        name="tilt head arm mounts to carriage pad",
    )
    ctx.expect_within(
        knob,
        head,
        axes="xz",
        inner_elem="threaded_stem",
        outer_elem="head_block",
        margin=0.001,
        name="threaded stem is centered in the head boss",
    )
    ctx.expect_overlap(
        knob,
        head,
        axes="y",
        elem_a="threaded_stem",
        elem_b="head_block",
        min_overlap=0.020,
        name="threaded stem remains engaged in the head",
    )

    rest_head = ctx.part_world_position(head)
    with ctx.pose({slide: slide.motion_limits.upper}):
        raised_head = ctx.part_world_position(head)
        ctx.expect_overlap(
            carriage,
            rail,
            axes="z",
            elem_a="front_bridge",
            elem_b="upright_rail",
            min_overlap=0.20,
            name="raised carriage still wraps the vertical rail",
        )

    ctx.check(
        "height slide raises the tilt head",
        rest_head is not None and raised_head is not None and raised_head[2] > rest_head[2] + 0.20,
        details=f"rest_head={rest_head}, raised_head={raised_head}",
    )

    rest_lip = ctx.part_element_world_aabb(board, elem="storage_ledge_lip")
    with ctx.pose({board_hinge: 0.85}):
        tilted_lip = ctx.part_element_world_aabb(board, elem="storage_ledge_lip")

    ctx.check(
        "board hinge lifts the lower storage ledge",
        rest_lip is not None and tilted_lip is not None and tilted_lip[1][2] > rest_lip[1][2] + 0.45,
        details=f"rest_lip={rest_lip}, tilted_lip={tilted_lip}",
    )

    ctx.check(
        "clamp knob spins continuously on its threaded axis",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(knob_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
