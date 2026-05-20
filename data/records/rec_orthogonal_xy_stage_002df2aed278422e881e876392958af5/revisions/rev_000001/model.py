from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.220
BASE_W = 0.140
BASE_TH = 0.014
BASE_POCKET_L = 0.172
BASE_POCKET_W = 0.050
BASE_POCKET_D = 0.005

X_RAIL_L = 0.180
X_RAIL_BOTTOM_W = 0.020
X_RAIL_TOP_W = 0.013
X_RAIL_H = 0.010
X_RAIL_Y = 0.046

X_SLIDE_L = 0.190
X_SLIDE_W = 0.110
X_SLIDE_TH = 0.012
X_SLIDE_TROUGH_L = 0.102
X_SLIDE_TROUGH_W = 0.026
X_SLIDE_TROUGH_D = 0.004

Y_GUIDE_W = 0.014
Y_GUIDE_L = 0.092
Y_GUIDE_H = 0.008
Y_GUIDE_X = 0.028

Y_SADDLE_W = 0.074
Y_SADDLE_L = 0.106
Y_SADDLE_TH = 0.010
Y_SADDLE_WINDOW_W = 0.038
Y_SADDLE_WINDOW_L = 0.060

TOP_PLATE_L = 0.118
TOP_PLATE_W = 0.088
TOP_PLATE_TH = 0.006
TOP_APERTURE_L = 0.054
TOP_APERTURE_W = 0.038
TOP_HOLE_D = 0.0045

X_TRAVEL = 0.035
Y_TRAVEL = 0.028


def _trapezoid_rail(length: float, bottom_width: float, top_width: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .polyline(
            [
                (-bottom_width / 2.0, 0.0),
                (bottom_width / 2.0, 0.0),
                (top_width / 2.0, height),
                (-top_width / 2.0, height),
            ]
        )
        .close()
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _base_body() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BASE_L, BASE_W, BASE_TH, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    return (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BASE_POCKET_L, BASE_POCKET_W)
        .cutBlind(-BASE_POCKET_D)
    )


def _slide_body() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(X_SLIDE_L, X_SLIDE_W, X_SLIDE_TH, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    return (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(X_SLIDE_TROUGH_L, X_SLIDE_TROUGH_W)
        .cutBlind(-X_SLIDE_TROUGH_D)
    )


def _guide_bar(x_pos: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(Y_GUIDE_W, Y_GUIDE_L, Y_GUIDE_H, centered=(True, True, False))
        .edges("|Y")
        .fillet(0.0012)
        .translate((x_pos, 0.0, X_SLIDE_TH))
    )


def _saddle_frame() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .box(Y_SADDLE_W, Y_SADDLE_L, Y_SADDLE_TH, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0025)
    )
    return (
        frame.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(Y_SADDLE_WINDOW_W, Y_SADDLE_WINDOW_L)
        .cutThruAll()
    )


def _top_plate() -> cq.Workplane:
    hole_points = [
        (-0.038, -0.030),
        (-0.038, -0.010),
        (-0.038, 0.010),
        (-0.038, 0.030),
        (0.038, -0.030),
        (0.038, -0.010),
        (0.038, 0.010),
        (0.038, 0.030),
        (-0.020, -0.033),
        (0.020, -0.033),
        (-0.020, 0.033),
        (0.020, 0.033),
    ]
    plate = (
        cq.Workplane("XY")
        .box(TOP_PLATE_L, TOP_PLATE_W, TOP_PLATE_TH, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0018)
    )
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(TOP_APERTURE_L, TOP_APERTURE_W)
        .cutThruAll()
    )
    return (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_points)
        .hole(TOP_HOLE_D)
    )


def _extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float]:
    if aabb is None:
        return (0.0, 0.0, 0.0)
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microscope_translation_table")

    black_oxide = model.material("black_oxide", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    carriage_gray = model.material("carriage_gray", rgba=(0.57, 0.59, 0.61, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    bronze_way = model.material("bronze_way", rgba=(0.67, 0.56, 0.29, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "base_body"),
        material=black_oxide,
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(
            _trapezoid_rail(X_RAIL_L, X_RAIL_BOTTOM_W, X_RAIL_TOP_W, X_RAIL_H).translate((0.0, -X_RAIL_Y, BASE_TH)),
            "x_guide_front",
        ),
        material=dark_steel,
        name="x_guide_front",
    )
    base.visual(
        mesh_from_cadquery(
            _trapezoid_rail(X_RAIL_L, X_RAIL_BOTTOM_W, X_RAIL_TOP_W, X_RAIL_H).translate((0.0, X_RAIL_Y, BASE_TH)),
            "x_guide_rear",
        ),
        material=dark_steel,
        name="x_guide_rear",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_TH + X_RAIL_H)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_TH + X_RAIL_H) / 2.0)),
    )

    x_slide = model.part("x_slide")
    x_slide.visual(
        mesh_from_cadquery(_slide_body(), "x_slide_body"),
        material=carriage_gray,
        name="x_slide_body",
    )
    x_slide.visual(
        mesh_from_cadquery(_guide_bar(-Y_GUIDE_X), "y_guide_left"),
        material=bronze_way,
        name="y_guide_left",
    )
    x_slide.visual(
        mesh_from_cadquery(_guide_bar(Y_GUIDE_X), "y_guide_right"),
        material=bronze_way,
        name="y_guide_right",
    )
    x_slide.inertial = Inertial.from_geometry(
        Box((X_SLIDE_L, X_SLIDE_W, X_SLIDE_TH + Y_GUIDE_H)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, (X_SLIDE_TH + Y_GUIDE_H) / 2.0)),
    )

    y_saddle = model.part("y_saddle")
    y_saddle.visual(
        mesh_from_cadquery(_saddle_frame(), "y_saddle_frame"),
        material=brushed_aluminum,
        name="y_saddle_frame",
    )
    y_saddle.inertial = Inertial.from_geometry(
        Box((Y_SADDLE_W, Y_SADDLE_L, Y_SADDLE_TH)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, Y_SADDLE_TH / 2.0)),
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        mesh_from_cadquery(_top_plate(), "top_plate"),
        material=black_oxide,
        name="top_plate",
    )
    top_plate.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_L, TOP_PLATE_W, TOP_PLATE_TH)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_TH / 2.0)),
    )

    model.articulation(
        "base_to_x_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_slide,
        origin=Origin(xyz=(0.0, 0.0, BASE_TH + X_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.08,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )

    model.articulation(
        "x_slide_to_y_saddle",
        ArticulationType.PRISMATIC,
        parent=x_slide,
        child=y_saddle,
        origin=Origin(xyz=(0.0, 0.0, X_SLIDE_TH + Y_GUIDE_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.08,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

    model.articulation(
        "y_saddle_to_top_plate",
        ArticulationType.FIXED,
        parent=y_saddle,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, Y_SADDLE_TH)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_slide = object_model.get_part("x_slide")
    y_saddle = object_model.get_part("y_saddle")
    top_plate = object_model.get_part("top_plate")

    x_axis = object_model.get_articulation("base_to_x_slide")
    y_axis = object_model.get_articulation("x_slide_to_y_saddle")
    plate_mount = object_model.get_articulation("y_saddle_to_top_plate")

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

    def joint_kind(joint) -> str:
        return getattr(joint.articulation_type, "name", str(joint.articulation_type))

    def near(a: float, b: float, tol: float = 1e-6) -> bool:
        return abs(a - b) <= tol

    ctx.check(
        "x joint is prismatic",
        joint_kind(x_axis) == "PRISMATIC",
        f"found {joint_kind(x_axis)}",
    )
    ctx.check(
        "y joint is prismatic",
        joint_kind(y_axis) == "PRISMATIC",
        f"found {joint_kind(y_axis)}",
    )
    ctx.check(
        "top plate is fixed to saddle",
        joint_kind(plate_mount) == "FIXED",
        f"found {joint_kind(plate_mount)}",
    )
    ctx.check(
        "x axis aligned with world x",
        tuple(x_axis.axis) == (1.0, 0.0, 0.0),
        f"axis={x_axis.axis}",
    )
    ctx.check(
        "y axis aligned with world y",
        tuple(y_axis.axis) == (0.0, 1.0, 0.0),
        f"axis={y_axis.axis}",
    )

    ctx.expect_gap(
        x_slide,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="x slide seats on lower ways",
    )
    ctx.expect_overlap(
        x_slide,
        base,
        axes="xy",
        min_overlap=0.10,
        name="x slide remains broadly supported",
    )
    ctx.expect_gap(
        y_saddle,
        x_slide,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="y saddle seats on upper ways",
    )
    ctx.expect_overlap(
        y_saddle,
        x_slide,
        axes="xy",
        min_overlap=0.05,
        name="y saddle overlaps the x carriage footprint",
    )
    ctx.expect_gap(
        top_plate,
        y_saddle,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="top plate seats on saddle frame",
    )
    ctx.expect_overlap(
        top_plate,
        y_saddle,
        axes="xy",
        min_overlap=0.07,
        name="top plate stays centered over the saddle",
    )

    x_dims = _extent(ctx.part_world_aabb(x_slide))
    y_dims = _extent(ctx.part_world_aabb(y_saddle))
    ctx.check(
        "lower slide reads broad in x",
        x_dims[0] > x_dims[1] * 1.5,
        f"x_slide dims={x_dims}",
    )
    ctx.check(
        "upper saddle reads turned ninety degrees",
        y_dims[1] > y_dims[0] * 1.25,
        f"y_saddle dims={y_dims}",
    )

    rest_x = ctx.part_world_position(x_slide)
    rest_y = ctx.part_world_position(y_saddle)

    with ctx.pose({x_axis: X_TRAVEL}):
        moved_x = ctx.part_world_position(x_slide)
        if rest_x is not None and moved_x is not None:
            dx = moved_x[0] - rest_x[0]
            dy = moved_x[1] - rest_x[1]
            dz = moved_x[2] - rest_x[2]
            ctx.check(
                "x stage moves only along x",
                near(dx, X_TRAVEL, 1e-6) and near(dy, 0.0, 1e-6) and near(dz, 0.0, 1e-6),
                f"delta={(dx, dy, dz)}",
            )
        ctx.expect_overlap(
            x_slide,
            base,
            axes="xy",
            min_overlap=0.10,
            name="x slide stays supported at positive x travel",
        )

    with ctx.pose({y_axis: Y_TRAVEL}):
        moved_y = ctx.part_world_position(y_saddle)
        if rest_y is not None and moved_y is not None:
            dx = moved_y[0] - rest_y[0]
            dy = moved_y[1] - rest_y[1]
            dz = moved_y[2] - rest_y[2]
            ctx.check(
                "y stage moves only along y",
                near(dx, 0.0, 1e-6) and near(dy, Y_TRAVEL, 1e-6) and near(dz, 0.0, 1e-6),
                f"delta={(dx, dy, dz)}",
            )
        ctx.expect_overlap(
            y_saddle,
            x_slide,
            axes="xy",
            min_overlap=0.05,
            name="y saddle stays supported at positive y travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
