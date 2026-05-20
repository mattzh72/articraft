from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.34
BASE_W = 0.24
BASE_T = 0.03

COLUMN_R = 0.055
COLUMN_TOP_Z = 0.55

HOUSING_L = 0.18
HOUSING_W = 0.14
HOUSING_Z0 = 0.55
HOUSING_Z1 = 0.605

ROOT_FLANGE_R = 0.082
ROOT_FLANGE_Z0 = HOUSING_Z1
ROOT_FLANGE_Z1 = 0.623
ROOT_SPINDLE_R = 0.05
ROOT_SPINDLE_Z1 = 0.673
ROOT_JOINT_Z = ROOT_FLANGE_Z1

BEAM_TUBE_X0 = 0.12
BEAM_TUBE_X1 = 0.78
BEAM_Y_HALF = 0.07
BEAM_Z0 = 0.055
BEAM_Z1 = 0.165
BEAM_INNER_Y_HALF = 0.047
BEAM_INNER_Z0 = 0.078
BEAM_INNER_Z1 = 0.142

SLIDE_JOINT_X = 0.46
SLIDE_JOINT_Z = 0.11
SLIDE_HALF_LEN = 0.33
SLIDE_Y_HALF = 0.034
SLIDE_Z_HALF = 0.02
SLIDE_UPPER = 0.14

GUIDE_PAD_Y_HALF = 0.039
GUIDE_PAD_X0 = -0.30
GUIDE_PAD_X1 = 0.22
GUIDE_PAD_Z_HALF = 0.016

GUIDE_RAIL_X0 = 0.18
GUIDE_RAIL_X1 = 0.74
GUIDE_RAIL_Y_INNER = GUIDE_PAD_Y_HALF
GUIDE_RAIL_Y_OUTER = BEAM_INNER_Y_HALF
GUIDE_RAIL_Z0 = 0.088
GUIDE_RAIL_Z1 = 0.132


def _box(
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
    zmin: float,
    zmax: float,
) -> cq.Workplane:
    return cq.Workplane("XY").box(
        xmax - xmin,
        ymax - ymin,
        zmax - zmin,
    ).translate(
        (
            (xmin + xmax) * 0.5,
            (ymin + ymax) * 0.5,
            (zmin + zmax) * 0.5,
        )
    )


def _cylinder(radius: float, zmin: float, zmax: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(zmax - zmin).translate((0.0, 0.0, zmin))


def _annulus(outer_r: float, inner_r: float, zmin: float, zmax: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(zmax - zmin)
        .translate((0.0, 0.0, zmin))
    )


def make_pedestal_shape() -> cq.Workplane:
    base = _box(-BASE_L / 2, BASE_L / 2, -BASE_W / 2, BASE_W / 2, 0.0, BASE_T).edges("|Z").fillet(0.012)
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.115, -0.075),
                (-0.115, 0.075),
                (0.115, -0.075),
                (0.115, 0.075),
            ]
        )
        .hole(0.018)
    )

    column = _cylinder(COLUMN_R, BASE_T, COLUMN_TOP_Z)
    transition = _cylinder(0.068, COLUMN_TOP_Z - 0.02, HOUSING_Z0)
    housing = _box(
        -HOUSING_L / 2,
        HOUSING_L / 2,
        -HOUSING_W / 2,
        HOUSING_W / 2,
        HOUSING_Z0,
        HOUSING_Z1,
    ).edges("|Z").fillet(0.01)
    flange = _cylinder(ROOT_FLANGE_R, ROOT_FLANGE_Z0, ROOT_FLANGE_Z1)
    spindle = _cylinder(ROOT_SPINDLE_R, ROOT_FLANGE_Z1, ROOT_SPINDLE_Z1)

    return base.union(column).union(transition).union(housing).union(flange).union(spindle)


def make_beam_root_mount_shape() -> cq.Workplane:
    collar = _annulus(0.078, 0.06, 0.0, 0.042)
    collar_cap = _annulus(0.088, 0.062, 0.042, 0.05)
    bridge = _box(0.09, 0.18, -0.07, 0.07, 0.018, 0.07).edges("|X").fillet(0.008)
    lower_web = _box(0.11, 0.17, -0.05, 0.05, 0.0, BEAM_Z0 + 0.005)
    spindle_clear = _cylinder(0.06, 0.0, 0.072)
    return collar.union(collar_cap).union(bridge).union(lower_web).cut(spindle_clear)


def make_beam_carrier_body_shape() -> cq.Workplane:
    carrier = _box(BEAM_TUBE_X0, BEAM_TUBE_X1, -BEAM_Y_HALF, BEAM_Y_HALF, BEAM_Z0, BEAM_Z1).edges("|X").fillet(0.008)
    carrier = carrier.cut(
        _box(
            BEAM_TUBE_X0 - 0.002,
            BEAM_TUBE_X1 + 0.002,
            -BEAM_INNER_Y_HALF,
            BEAM_INNER_Y_HALF,
            BEAM_INNER_Z0,
            BEAM_INNER_Z1,
        )
    )
    carrier = carrier.cut(_box(0.30, 0.64, -0.038, 0.038, 0.13, 0.19))

    right_rail = _box(
        GUIDE_RAIL_X0,
        GUIDE_RAIL_X1,
        GUIDE_RAIL_Y_INNER,
        GUIDE_RAIL_Y_OUTER,
        GUIDE_RAIL_Z0,
        GUIDE_RAIL_Z1,
    )
    left_rail = _box(
        GUIDE_RAIL_X0,
        GUIDE_RAIL_X1,
        -GUIDE_RAIL_Y_OUTER,
        -GUIDE_RAIL_Y_INNER,
        GUIDE_RAIL_Z0,
        GUIDE_RAIL_Z1,
    )

    rear_sleeve = _box(0.21, 0.27, -BEAM_INNER_Y_HALF, BEAM_INNER_Y_HALF, BEAM_INNER_Z0, BEAM_INNER_Z1)
    rear_sleeve = rear_sleeve.cut(
        _box(
            0.208,
            0.272,
            -0.04,
            0.04,
            0.083,
            0.137,
        )
    )

    front_sleeve = _box(0.69, 0.78, -BEAM_INNER_Y_HALF, BEAM_INNER_Y_HALF, BEAM_INNER_Z0, BEAM_INNER_Z1)
    front_sleeve = front_sleeve.cut(
        _box(
            0.688,
            0.782,
            -0.04,
            0.04,
            0.083,
            0.137,
        )
    )

    return carrier.union(right_rail).union(left_rail).union(rear_sleeve).union(front_sleeve)


def make_stage_bar_shape() -> cq.Workplane:
    bar = _box(
        -SLIDE_HALF_LEN,
        SLIDE_HALF_LEN,
        -SLIDE_Y_HALF,
        SLIDE_Y_HALF,
        -SLIDE_Z_HALF,
        SLIDE_Z_HALF,
    ).edges("|X").fillet(0.003)
    right_pad = _box(
        GUIDE_PAD_X0,
        GUIDE_PAD_X1,
        SLIDE_Y_HALF,
        GUIDE_PAD_Y_HALF,
        -GUIDE_PAD_Z_HALF,
        GUIDE_PAD_Z_HALF,
    )
    left_pad = _box(
        GUIDE_PAD_X0,
        GUIDE_PAD_X1,
        -GUIDE_PAD_Y_HALF,
        -SLIDE_Y_HALF,
        -GUIDE_PAD_Z_HALF,
        GUIDE_PAD_Z_HALF,
    )
    nose_head = _box(0.33, 0.355, -0.041, 0.041, -0.028, 0.028)
    return bar.union(right_pad).union(left_pad).union(nose_head)


def make_tool_pad_shape() -> cq.Workplane:
    return _box(0.355, 0.367, -0.045, 0.045, -0.037, 0.037).edges("|X").fillet(0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_slide_arm")

    pedestal_mat = model.material("pedestal_paint", rgba=(0.22, 0.23, 0.24, 1.0))
    beam_mat = model.material("machined_aluminum", rgba=(0.72, 0.73, 0.75, 1.0))
    slide_mat = model.material("slide_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    pad_mat = model.material("tool_pad_black", rgba=(0.08, 0.09, 0.1, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(make_pedestal_shape(), "pedestal_body"),
        material=pedestal_mat,
        name="pedestal_body",
    )

    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(make_beam_root_mount_shape(), "beam_root_mount"),
        material=beam_mat,
        name="root_mount",
    )
    beam.visual(
        mesh_from_cadquery(make_beam_carrier_body_shape(), "beam_carrier_body"),
        material=beam_mat,
        name="carrier_body",
    )

    nose_slide = model.part("nose_slide")
    nose_slide.visual(
        mesh_from_cadquery(make_stage_bar_shape(), "nose_stage_bar"),
        material=slide_mat,
        name="stage_bar",
    )
    nose_slide.visual(
        mesh_from_cadquery(make_tool_pad_shape(), "nose_tool_pad"),
        material=pad_mat,
        name="tool_pad",
    )

    model.articulation(
        "pedestal_to_beam",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, ROOT_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.6,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "beam_to_nose_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=nose_slide,
        origin=Origin(xyz=(SLIDE_JOINT_X, 0.0, SLIDE_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    nose_slide = object_model.get_part("nose_slide")
    root_joint = object_model.get_articulation("pedestal_to_beam")
    slide_joint = object_model.get_articulation("beam_to_nose_slide")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred baseline QC stack, with the generic overlap gate replaced here by
    # exact checks because the real mechanism intentionally keeps the beam root
    # seated on the pedestal and the slide stage in guided contact with the beam.
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.check(
        "root_joint_axis_vertical",
        tuple(root_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical rotary root axis, got {root_joint.axis}",
    )
    ctx.check(
        "slide_joint_axis_along_beam",
        tuple(slide_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected prismatic slide along beam x-axis, got {slide_joint.axis}",
    )
    ctx.expect_contact(
        beam,
        pedestal,
        elem_a="root_mount",
        elem_b="pedestal_body",
        name="beam_root_mount_contacts_pedestal_housing",
    )
    ctx.expect_gap(
        beam,
        pedestal,
        axis="z",
        positive_elem="carrier_body",
        negative_elem="pedestal_body",
        min_gap=0.004,
        max_gap=0.02,
        name="beam_carrier_clears_pedestal_above_root_housing",
    )
    ctx.expect_within(
        nose_slide,
        beam,
        axes="yz",
        inner_elem="stage_bar",
        outer_elem="carrier_body",
        margin=0.012,
        name="slide_bar_stays_guided_inside_beam_window",
    )
    ctx.expect_overlap(
        nose_slide,
        beam,
        axes="x",
        elem_a="stage_bar",
        elem_b="carrier_body",
        min_overlap=0.5,
        name="slide_bar_has_substantial_telescoping_overlap",
    )
    ctx.expect_gap(
        nose_slide,
        beam,
        axis="x",
        positive_elem="tool_pad",
        negative_elem="carrier_body",
        min_gap=0.015,
        max_gap=0.08,
        name="tool_pad_protrudes_past_beam_face",
    )
    ctx.expect_gap(
        nose_slide,
        pedestal,
        axis="z",
        min_gap=0.02,
        name="nose_slide_clears_pedestal_at_rest",
    )

    with ctx.pose({slide_joint: SLIDE_UPPER}):
        ctx.expect_within(
            nose_slide,
            beam,
            axes="yz",
            inner_elem="stage_bar",
            outer_elem="carrier_body",
            margin=0.012,
            name="slide_bar_remains_guided_at_full_extension",
        )
        ctx.expect_overlap(
            nose_slide,
            beam,
            axes="x",
            elem_a="stage_bar",
            elem_b="carrier_body",
            min_overlap=0.48,
            name="slide_retains_rear_engagement_at_full_extension",
        )
        ctx.expect_gap(
            nose_slide,
            pedestal,
            axis="z",
            min_gap=0.02,
            name="nose_slide_clears_pedestal_at_full_extension",
        )

    with ctx.pose({root_joint: 1.4, slide_joint: SLIDE_UPPER}):
        ctx.expect_contact(
            beam,
            pedestal,
            elem_a="root_mount",
            elem_b="pedestal_body",
            name="beam_root_remains_seated_when_rotated",
        )
        ctx.expect_gap(
            beam,
            pedestal,
            axis="z",
            positive_elem="carrier_body",
            negative_elem="pedestal_body",
            min_gap=0.004,
            max_gap=0.02,
            name="beam_carrier_still_clears_pedestal_when_rotated",
        )
        ctx.expect_gap(
            nose_slide,
            pedestal,
            axis="z",
            min_gap=0.02,
            name="nose_slide_clears_pedestal_in_rotated_extended_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
