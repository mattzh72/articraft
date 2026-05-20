from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="insulated_pet_flap")

    frame_white = model.material("frame_white", rgba=(0.94, 0.95, 0.96, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.66, 0.69, 0.72, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.56, 0.59, 0.62, 1.0))
    clear_poly = model.material("clear_poly", rgba=(0.68, 0.86, 0.95, 0.35))

    outer_width = 0.44
    outer_height = 0.60
    frame_depth = 0.075
    opening_width = 0.31
    jamb_width = (outer_width - opening_width) * 0.5
    head_height = 0.095
    sill_height = 0.075

    hinge_x = 0.126
    hinge_y = 0.015
    hinge_z = 0.456

    frame = model.part("frame")
    frame.visual(
        Box((jamb_width, frame_depth, outer_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + jamb_width * 0.5), 0.0, outer_height * 0.5)),
        material=frame_white,
        name="left_jamb",
    )
    frame.visual(
        Box((jamb_width, frame_depth, outer_height)),
        origin=Origin(xyz=((opening_width * 0.5 + jamb_width * 0.5), 0.0, outer_height * 0.5)),
        material=frame_white,
        name="right_jamb",
    )
    frame.visual(
        Box((outer_width, frame_depth, sill_height)),
        origin=Origin(xyz=(0.0, 0.0, sill_height * 0.5)),
        material=frame_white,
        name="sill",
    )
    frame.visual(
        Box((outer_width, frame_depth, head_height)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - head_height * 0.5)),
        material=frame_white,
        name="head",
    )
    frame.visual(
        Box((0.33, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.018, 0.491)),
        material=frame_white,
        name="hood_cover",
    )
    frame.visual(
        Box((0.024, 0.030, 0.042)),
        origin=Origin(xyz=(-0.161, 0.018, 0.470)),
        material=frame_white,
        name="left_hood_side",
    )
    frame.visual(
        Box((0.024, 0.030, 0.042)),
        origin=Origin(xyz=(0.161, 0.018, 0.470)),
        material=frame_white,
        name="right_hood_side",
    )
    frame.visual(
        Box((0.025, 0.016, 0.014)),
        origin=Origin(xyz=(-0.1495, 0.015, hinge_z)),
        material=frame_white,
        name="left_pin_arm",
    )
    frame.visual(
        Box((0.025, 0.016, 0.014)),
        origin=Origin(xyz=(0.1495, 0.015, hinge_z)),
        material=frame_white,
        name="right_pin_arm",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.022),
        origin=Origin(xyz=(-hinge_x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="left_pivot_pin",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.022),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="right_pivot_pin",
    )
    frame.visual(
        Box((0.006, 0.018, 0.026)),
        origin=Origin(xyz=(0.023, 0.030, 0.078)),
        material=frame_white,
        name="strike_support",
    )
    frame.visual(
        Box((0.040, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.044, 0.085)),
        material=seal_gray,
        name="lower_strike",
    )
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )

    flap = model.part("flap")
    flap.visual(
        Box((0.032, 0.020, 0.304)),
        origin=Origin(xyz=(-0.125, 0.010, -0.184)),
        material=seal_gray,
        name="left_stile",
    )
    flap.visual(
        Box((0.032, 0.020, 0.304)),
        origin=Origin(xyz=(0.125, 0.010, -0.184)),
        material=seal_gray,
        name="right_stile",
    )
    flap.visual(
        Box((0.282, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, 0.010, -0.016)),
        material=seal_gray,
        name="top_rail",
    )
    flap.visual(
        Box((0.100, 0.020, 0.030)),
        origin=Origin(xyz=(-0.070, 0.010, -0.337)),
        material=seal_gray,
        name="bottom_left_rail",
    )
    flap.visual(
        Box((0.100, 0.020, 0.030)),
        origin=Origin(xyz=(0.070, 0.010, -0.337)),
        material=seal_gray,
        name="bottom_right_rail",
    )
    flap.visual(
        Box((0.212, 0.008, 0.286)),
        origin=Origin(xyz=(0.0, 0.010, -0.184)),
        material=clear_poly,
        name="clear_panel",
    )
    flap.visual(
        Cylinder(radius=0.0060, length=0.022),
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="left_hinge_barrel",
    )
    flap.visual(
        Cylinder(radius=0.0060, length=0.022),
        origin=Origin(xyz=(hinge_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="right_hinge_barrel",
    )
    flap.visual(
        Box((0.014, 0.018, 0.028)),
        origin=Origin(xyz=(-0.021, 0.013, -0.337)),
        material=seal_gray,
        name="left_latch_cheek",
    )
    flap.visual(
        Box((0.014, 0.018, 0.028)),
        origin=Origin(xyz=(0.021, 0.013, -0.337)),
        material=seal_gray,
        name="right_latch_cheek",
    )
    flap.visual(
        Cylinder(radius=0.0045, length=0.048),
        origin=Origin(xyz=(0.0, 0.013, -0.337), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="latch_axle",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.282, 0.022, 0.352)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.010, -0.176)),
    )

    latch = model.part("latch_tab")
    latch.visual(
        Cylinder(radius=0.0055, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_gray,
        name="latch_barrel",
    )
    latch.visual(
        Box((0.038, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.000, -0.020)),
        material=seal_gray,
        name="latch_body",
    )
    latch.visual(
        Box((0.026, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.002, -0.036)),
        material=seal_gray,
        name="latch_tip",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.038, 0.012, 0.040)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.000, -0.020)),
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "flap_to_latch",
        ArticulationType.REVOLUTE,
        parent=flap,
        child=latch,
        origin=Origin(xyz=(0.0, 0.013, -0.337)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.28,
            upper=0.08,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    latch = object_model.get_part("latch_tab")

    flap_hinge = object_model.get_articulation("frame_to_flap")
    latch_pivot = object_model.get_articulation("flap_to_latch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        frame,
        flap,
        elem_a="left_pivot_pin",
        elem_b="left_hinge_barrel",
        reason="Left pivot pin is captured inside the left upper flap barrel.",
    )
    ctx.allow_overlap(
        frame,
        flap,
        elem_a="right_pivot_pin",
        elem_b="right_hinge_barrel",
        reason="Right pivot pin is captured inside the right upper flap barrel.",
    )
    ctx.allow_overlap(
        flap,
        latch,
        elem_a="latch_axle",
        elem_b="latch_barrel",
        reason="Latch tab rotates around a short axle at the lower flap edge.",
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(frame, flap, elem_a="left_pivot_pin", elem_b="left_hinge_barrel", name="left_hinge_supported")
    ctx.expect_contact(frame, flap, elem_a="right_pivot_pin", elem_b="right_hinge_barrel", name="right_hinge_supported")
    ctx.expect_contact(flap, latch, elem_a="latch_axle", elem_b="latch_barrel", name="latch_pivot_supported")

    ctx.expect_overlap(flap, frame, axes="xz", min_overlap=0.25, name="flap_reads_inside_frame")
    ctx.expect_within(flap, frame, axes="x", margin=0.0, name="flap_between_jambs_rest")
    ctx.expect_within(
        flap,
        frame,
        axes="xz",
        inner_elem="clear_panel",
        margin=0.02,
        name="clear_panel_within_frame_opening",
    )
    ctx.expect_overlap(
        latch,
        frame,
        axes="xz",
        elem_a="latch_tip",
        elem_b="lower_strike",
        min_overlap=0.009,
        name="latch_tip_aligned_with_strike",
    )
    ctx.expect_gap(
        frame,
        latch,
        axis="y",
        positive_elem="lower_strike",
        negative_elem="latch_tip",
        min_gap=0.002,
        max_gap=0.020,
        name="latch_clearance_to_strike",
    )

    flap_rest_aabb = ctx.part_world_aabb(flap)
    latch_rest_aabb = ctx.part_world_aabb(latch)
    assert flap_rest_aabb is not None
    assert latch_rest_aabb is not None

    flap_limits = flap_hinge.motion_limits
    assert flap_limits is not None
    assert flap_limits.lower is not None
    assert flap_limits.upper is not None

    with ctx.pose({flap_hinge: flap_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="frame_to_flap_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="frame_to_flap_lower_no_floating")

    with ctx.pose({flap_hinge: flap_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="frame_to_flap_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="frame_to_flap_upper_no_floating")
        ctx.expect_within(flap, frame, axes="x", margin=0.0, name="flap_between_jambs_open")
        flap_open_aabb = ctx.part_world_aabb(flap)
        assert flap_open_aabb is not None
        assert flap_open_aabb[0][1] < flap_rest_aabb[0][1] - 0.10

    latch_limits = latch_pivot.motion_limits
    assert latch_limits is not None
    assert latch_limits.lower is not None
    assert latch_limits.upper is not None

    with ctx.pose({latch_pivot: latch_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="flap_to_latch_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="flap_to_latch_lower_no_floating")
        latch_lower_aabb = ctx.part_world_aabb(latch)
        assert latch_lower_aabb is not None
        assert latch_lower_aabb[1][1] < latch_rest_aabb[1][1] - 0.001

    with ctx.pose({latch_pivot: latch_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="flap_to_latch_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="flap_to_latch_upper_no_floating")
        latch_open_aabb = ctx.part_world_aabb(latch)
        assert latch_open_aabb is not None
        assert latch_open_aabb[1][1] <= latch_rest_aabb[1][1] + 0.004

    with ctx.pose({flap_hinge: flap_limits.upper, latch_pivot: latch_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_open_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
