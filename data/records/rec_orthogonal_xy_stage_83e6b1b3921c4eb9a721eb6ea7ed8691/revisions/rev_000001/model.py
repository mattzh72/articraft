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


BASE_LENGTH = 0.42
BASE_WIDTH = 0.28
BASE_THICKNESS = 0.018
BASE_CORNER_RADIUS = 0.004

X_RAIL_LENGTH = 0.34
X_RAIL_WIDTH = 0.026
X_RAIL_HEIGHT = 0.014
X_RAIL_Y_OFFSET = 0.055
X_CENTER_COVER_WIDTH = 0.038
X_CENTER_COVER_HEIGHT = 0.008

X_CARRIAGE_LENGTH = 0.14
X_CARRIAGE_WIDTH = 0.18
X_CARRIAGE_HEIGHT = 0.042
X_TRAVEL = 0.10

Y_SLIDE_WIDTH = 0.12
Y_SLIDE_LENGTH = 0.20
Y_BASE_THICKNESS = 0.012
Y_RAIL_WIDTH = 0.018
Y_RAIL_HEIGHT = 0.010
Y_RAIL_X_OFFSET = 0.034
Y_CENTER_COVER_WIDTH = 0.024
Y_CENTER_COVER_HEIGHT = 0.007

TOP_TABLE_WIDTH = 0.096
TOP_TABLE_LENGTH = 0.10
TOP_TABLE_THICKNESS = 0.022
Y_TRAVEL = 0.045


def _rounded_plate(length: float, width: float, thickness: float, edge_radius: float) -> cq.Workplane:
    body = cq.Workplane("XY").box(length, width, thickness, centered=(True, True, False))
    return body.edges("|Z").fillet(edge_radius)


def _make_base_shape() -> cq.Workplane:
    hole_x = BASE_LENGTH * 0.34
    hole_y = BASE_WIDTH * 0.32
    plate = _rounded_plate(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, BASE_CORNER_RADIUS)
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-hole_x, -hole_y),
                (-hole_x, hole_y),
                (hole_x, -hole_y),
                (hole_x, hole_y),
            ]
        )
        .cboreHole(0.009, 0.016, 0.004)
    )

    rail = (
        cq.Workplane("XY")
        .box(X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0015)
    )
    rail_neg = rail.translate((0.0, -X_RAIL_Y_OFFSET, BASE_THICKNESS))
    rail_pos = rail.translate((0.0, X_RAIL_Y_OFFSET, BASE_THICKNESS))

    center_cover = (
        cq.Workplane("XY")
        .box(
            X_RAIL_LENGTH * 0.92,
            X_CENTER_COVER_WIDTH,
            X_CENTER_COVER_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.0015)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )

    return plate.union(rail_neg).union(rail_pos).union(center_cover)


def _make_x_carriage_shape() -> cq.Workplane:
    body = _rounded_plate(X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, X_CARRIAGE_HEIGHT, 0.003)
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(X_CARRIAGE_LENGTH - 0.040, X_CARRIAGE_WIDTH - 0.050)
        .cutBlind(-0.006)
    )
    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(0.080, 0.120, forConstruction=True)
        .vertices()
        .hole(0.006, 0.012)
    )
    return body


def _make_y_slide_shape() -> cq.Workplane:
    base = _rounded_plate(Y_SLIDE_WIDTH, Y_SLIDE_LENGTH, Y_BASE_THICKNESS, 0.0025)

    rail = (
        cq.Workplane("XY")
        .box(Y_RAIL_WIDTH, Y_SLIDE_LENGTH * 0.88, Y_RAIL_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0012)
    )
    rail_neg = rail.translate((-Y_RAIL_X_OFFSET, 0.0, Y_BASE_THICKNESS))
    rail_pos = rail.translate((Y_RAIL_X_OFFSET, 0.0, Y_BASE_THICKNESS))

    center_cover = (
        cq.Workplane("XY")
        .box(
            Y_CENTER_COVER_WIDTH,
            Y_SLIDE_LENGTH * 0.86,
            Y_CENTER_COVER_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.0012)
        .translate((0.0, 0.0, Y_BASE_THICKNESS))
    )

    return base.union(rail_neg).union(rail_pos).union(center_cover)


def _make_top_table_shape() -> cq.Workplane:
    table = _rounded_plate(TOP_TABLE_WIDTH, TOP_TABLE_LENGTH, TOP_TABLE_THICKNESS, 0.0025)
    table = (
        table.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(TOP_TABLE_WIDTH - 0.028, TOP_TABLE_LENGTH - 0.028)
        .cutBlind(-0.004)
    )
    table = (
        table.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rarray(0.042, 0.042, 2, 2)
        .hole(0.005, 0.010)
    )
    return table


def _axis_tuple(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(round(value, 6) for value in axis)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_axis_positioning_stage")

    model.material("base_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("carriage_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("table_gray", rgba=(0.82, 0.84, 0.87, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "stage_base"),
        origin=Origin(),
        material="base_gray",
        name="base_visual",
    )

    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_make_x_carriage_shape(), "x_carriage"),
        origin=Origin(),
        material="carriage_gray",
        name="x_carriage_visual",
    )

    y_slide = model.part("y_slide")
    y_slide.visual(
        mesh_from_cadquery(_make_y_slide_shape(), "y_slide"),
        origin=Origin(),
        material="carriage_gray",
        name="y_slide_visual",
    )

    top_table = model.part("top_table")
    top_table.visual(
        mesh_from_cadquery(_make_top_table_shape(), "top_table"),
        origin=Origin(),
        material="table_gray",
        name="top_table_visual",
    )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + X_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )

    model.articulation(
        "x_carriage_to_y_slide",
        ArticulationType.FIXED,
        parent=x_carriage,
        child=y_slide,
        origin=Origin(xyz=(0.0, 0.0, X_CARRIAGE_HEIGHT)),
    )

    model.articulation(
        "y_slide_to_top_table",
        ArticulationType.PRISMATIC,
        parent=y_slide,
        child=top_table,
        origin=Origin(xyz=(0.0, 0.0, Y_BASE_THICKNESS + Y_RAIL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.14,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_slide = object_model.get_part("y_slide")
    top_table = object_model.get_part("top_table")

    x_joint = object_model.get_articulation("base_to_x_carriage")
    y_joint = object_model.get_articulation("y_slide_to_top_table")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.check(
        "x_stage_is_prismatic_along_x",
        x_joint.joint_type == ArticulationType.PRISMATIC
        and _axis_tuple(x_joint.axis) == (1.0, 0.0, 0.0),
        details=f"expected PRISMATIC along +X, got {x_joint.joint_type} axis={x_joint.axis}",
    )
    ctx.check(
        "y_stage_is_prismatic_along_y",
        y_joint.joint_type == ArticulationType.PRISMATIC
        and _axis_tuple(y_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected PRISMATIC along +Y, got {y_joint.joint_type} axis={y_joint.axis}",
    )
    ctx.check(
        "stage_axes_are_orthogonal",
        abs(sum(a * b for a, b in zip(x_joint.axis, y_joint.axis))) < 1e-6,
        details=f"joint axes must be orthogonal, got {x_joint.axis} and {y_joint.axis}",
    )

    with ctx.pose({x_joint: 0.0, y_joint: 0.0}):
        ctx.expect_gap(
            x_carriage,
            base,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="x_carriage_seated_on_base",
        )
        ctx.expect_overlap(
            x_carriage,
            base,
            axes="xy",
            min_overlap=0.12,
            name="x_carriage_overlaps_base_footprint",
        )
        ctx.expect_within(
            x_carriage,
            base,
            axes="xy",
            margin=0.0,
            name="x_carriage_within_base_at_center",
        )
        ctx.expect_gap(
            y_slide,
            x_carriage,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="y_slide_mounted_on_x_carriage",
        )
        ctx.expect_within(
            y_slide,
            x_carriage,
            axes="x",
            margin=0.0,
            name="y_slide_centered_within_x_carriage_width",
        )
        ctx.expect_overlap(
            y_slide,
            x_carriage,
            axes="y",
            min_overlap=0.16,
            name="y_slide_has_broad_support_overlap_on_x_carriage",
        )
        ctx.expect_gap(
            top_table,
            y_slide,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name="top_table_seated_on_y_slide",
        )
        ctx.expect_within(
            top_table,
            y_slide,
            axes="xy",
            margin=0.0,
            name="top_table_within_y_slide_at_center",
        )

    x_limits = x_joint.motion_limits
    y_limits = y_joint.motion_limits

    if (
        x_limits is not None
        and x_limits.lower is not None
        and x_limits.upper is not None
        and y_limits is not None
        and y_limits.lower is not None
        and y_limits.upper is not None
    ):
        x_positions = (("lower", x_limits.lower), ("upper", x_limits.upper))
        y_positions = (("lower", y_limits.lower), ("upper", y_limits.upper))

        for label, value in x_positions:
            with ctx.pose({x_joint: value, y_joint: 0.0}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"base_to_x_carriage_{label}_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"base_to_x_carriage_{label}_no_floating")
                ctx.expect_gap(
                    x_carriage,
                    base,
                    axis="z",
                    max_gap=0.0005,
                    max_penetration=0.0,
                    name=f"x_carriage_{label}_seated_on_base",
                )
                ctx.expect_within(
                    x_carriage,
                    base,
                    axes="x",
                    margin=0.0,
                    name=f"x_carriage_{label}_within_base",
                )

        for label, value in y_positions:
            with ctx.pose({x_joint: 0.0, y_joint: value}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"y_slide_to_top_table_{label}_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"y_slide_to_top_table_{label}_no_floating")
                ctx.expect_gap(
                    top_table,
                    y_slide,
                    axis="z",
                    max_gap=0.0005,
                    max_penetration=0.0,
                    name=f"top_table_{label}_seated_on_y_slide",
                )
                ctx.expect_within(
                    top_table,
                    y_slide,
                    axes="y",
                    margin=0.0,
                    name=f"top_table_{label}_within_y_slide",
                )

        for x_label, x_value in x_positions:
            for y_label, y_value in y_positions:
                with ctx.pose({x_joint: x_value, y_joint: y_value}):
                    ctx.fail_if_parts_overlap_in_current_pose(
                        name=f"corner_pose_{x_label}_{y_label}_no_overlap"
                    )
                    ctx.fail_if_isolated_parts(
                        name=f"corner_pose_{x_label}_{y_label}_no_floating"
                    )
                    ctx.expect_within(
                        top_table,
                        base,
                        axes="xy",
                        margin=0.03,
                        name=f"top_table_{x_label}_{y_label}_stays_over_base",
                    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
