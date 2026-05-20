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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_drafting_table")

    cast_iron = model.material("cast_iron", rgba=(0.26, 0.27, 0.28, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.44, 0.46, 0.48, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.22, 1.0))
    birch = model.material("birch", rgba=(0.74, 0.66, 0.50, 1.0))
    drawing_surface = model.material("drawing_surface", rgba=(0.93, 0.92, 0.87, 1.0))
    rail_aluminum = model.material("rail_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    def _foot_mesh(name: str, length: float, width: float, thickness: float):
        return mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(length, width, radius=min(width * 0.28, 0.030)),
                thickness,
            ),
            name,
        )

    base = model.part("cast_base")
    x_foot_mesh = _foot_mesh("drafting_table_x_foot", length=0.78, width=0.16, thickness=0.036)
    y_foot_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(0.62, 0.15, radius=0.028),
            0.034,
        ).rotate_z(math.pi / 2.0),
        "drafting_table_y_foot",
    )
    base.visual(
        x_foot_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=cast_iron,
        name="x_foot",
    )
    base.visual(
        y_foot_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=cast_iron,
        name="y_foot",
    )
    base.visual(
        Cylinder(radius=0.120, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=cast_iron,
        name="hub_drum",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=cast_iron,
        name="column_socket",
    )
    for x_sign in (-1.0, 1.0):
        base.visual(
            Box((0.090, 0.070, 0.100)),
            origin=Origin(xyz=(0.145 * x_sign, 0.0, 0.050)),
            material=cast_iron,
            name=f"x_web_{'l' if x_sign < 0 else 'r'}",
        )
    for y_sign in (-1.0, 1.0):
        base.visual(
            Box((0.070, 0.090, 0.095)),
            origin=Origin(xyz=(0.0, 0.120 * y_sign, 0.0475)),
            material=cast_iron,
            name=f"y_web_{'b' if y_sign < 0 else 'f'}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.78, 0.62, 0.14)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    column = model.part("column")
    column.visual(
        Box((0.190, 0.150, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=painted_steel,
        name="base_shoe",
    )
    column.visual(
        Box((0.150, 0.110, 1.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=painted_steel,
        name="column_body",
    )
    column.visual(
        Box((0.180, 0.140, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.130)),
        material=painted_steel,
        name="top_cap",
    )
    column.visual(
        Box((0.100, 0.010, 0.820)),
        origin=Origin(xyz=(0.0, 0.050, 0.650)),
        material=dark_steel,
        name="front_slide_face",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.19, 0.15, 1.16)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.580)),
    )
    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )

    bracket = model.part("upper_bracket")
    bracket.visual(
        Box((0.165, 0.018, 0.250)),
        origin=Origin(xyz=(0.0, 0.009, 0.005)),
        material=dark_steel,
        name="back_pad",
    )
    bracket.visual(
        Box((0.205, 0.028, 0.210)),
        origin=Origin(xyz=(0.0, 0.030, 0.000)),
        material=dark_steel,
        name="front_plate",
    )
    bracket.visual(
        Box((0.028, 0.065, 0.300)),
        origin=Origin(xyz=(-0.097, 0.0325, 0.000)),
        material=dark_steel,
        name="left_guide",
    )
    bracket.visual(
        Box((0.028, 0.065, 0.300)),
        origin=Origin(xyz=(0.097, 0.0325, 0.000)),
        material=dark_steel,
        name="right_guide",
    )
    bracket.visual(
        Box((0.120, 0.050, 0.110)),
        origin=Origin(xyz=(0.0, 0.035, -0.060)),
        material=dark_steel,
        name="lower_body",
    )
    bracket.visual(
        Box((0.210, 0.070, 0.052)),
        origin=Origin(xyz=(0.0, 0.042, 0.155)),
        material=dark_steel,
        name="top_bridge",
    )
    bracket.visual(
        Box((0.030, 0.085, 0.120)),
        origin=Origin(xyz=(-0.095, 0.050, 0.240)),
        material=dark_steel,
        name="left_yoke_ear",
    )
    bracket.visual(
        Box((0.030, 0.085, 0.120)),
        origin=Origin(xyz=(0.095, 0.050, 0.240)),
        material=dark_steel,
        name="right_yoke_ear",
    )
    bracket.visual(
        Box((0.070, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.042, 0.201)),
        material=painted_steel,
        name="pivot_saddle",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((0.22, 0.09, 0.38)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.035, 0.050)),
    )
    model.articulation(
        "column_to_upper_bracket",
        ArticulationType.PRISMATIC,
        parent=column,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.055, 0.690)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.18,
            lower=0.0,
            upper=0.320,
        ),
    )

    board = model.part("drawing_board")
    board.visual(
        Cylinder(radius=0.017, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="trunnion_barrel",
    )
    board.visual(
        Box((0.034, 0.028, 0.078)),
        origin=Origin(xyz=(-0.050, 0.020, -0.027)),
        material=birch,
        name="left_hanger",
    )
    board.visual(
        Box((0.034, 0.028, 0.078)),
        origin=Origin(xyz=(0.050, 0.020, -0.027)),
        material=birch,
        name="right_hanger",
    )
    board.visual(
        Box((0.920, 0.040, 0.038)),
        origin=Origin(xyz=(0.0, 0.040, -0.078)),
        material=birch,
        name="frame_top",
    )
    board.visual(
        Box((0.038, 0.040, 0.620)),
        origin=Origin(xyz=(-0.441, 0.040, -0.390)),
        material=birch,
        name="frame_left",
    )
    board.visual(
        Box((0.038, 0.040, 0.620)),
        origin=Origin(xyz=(0.441, 0.040, -0.390)),
        material=birch,
        name="frame_right",
    )
    board.visual(
        Box((0.920, 0.048, 0.048)),
        origin=Origin(xyz=(0.0, 0.046, -0.678)),
        material=birch,
        name="frame_bottom",
    )
    board.visual(
        Box((0.836, 0.016, 0.566)),
        origin=Origin(xyz=(0.0, 0.028, -0.384)),
        material=drawing_surface,
        name="board_panel",
    )
    board.visual(
        Box((0.862, 0.034, 0.030)),
        origin=Origin(xyz=(0.0, 0.076, -0.674)),
        material=rail_aluminum,
        name="rail_body",
    )
    board.visual(
        Cylinder(radius=0.010, length=0.830),
        origin=Origin(xyz=(0.0, 0.093, -0.689), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_aluminum,
        name="rail_rod",
    )
    board.inertial = Inertial.from_geometry(
        Box((0.92, 0.10, 0.72)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.045, -0.360)),
    )
    model.articulation(
        "upper_bracket_to_board",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=board,
        origin=Origin(xyz=(0.0, 0.072, 0.240)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.80,
            lower=-0.20,
            upper=1.18,
        ),
    )

    knob = model.part("locking_knob")
    knob.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="knob_hub",
    )
    knob.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.018, 0.012, 0.044)),
        origin=Origin(xyz=(0.031, 0.0, 0.022)),
        material=knob_black,
        name="knob_handle",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.070, 0.070, 0.070)),
        mass=0.25,
        origin=Origin(xyz=(0.028, 0.0, 0.012)),
    )
    model.articulation(
        "upper_bracket_to_locking_knob",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=knob,
        origin=Origin(xyz=(0.111, 0.032, -0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("cast_base")
    column = object_model.get_part("column")
    bracket = object_model.get_part("upper_bracket")
    board = object_model.get_part("drawing_board")
    knob = object_model.get_part("locking_knob")

    slide = object_model.get_articulation("column_to_upper_bracket")
    tilt = object_model.get_articulation("upper_bracket_to_board")
    knob_joint = object_model.get_articulation("upper_bracket_to_locking_knob")

    ctx.check("cast base exists", base is not None)
    ctx.check("column exists", column is not None)
    ctx.check("upper bracket exists", bracket is not None)
    ctx.check("drawing board exists", board is not None)
    ctx.check("locking knob exists", knob is not None)

    slide_limits = slide.motion_limits
    tilt_limits = tilt.motion_limits
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "slider joint is vertical",
        tuple(round(v, 6) for v in slide.axis) == (0.0, 0.0, 1.0)
        and slide_limits is not None
        and slide_limits.lower == 0.0
        and slide_limits.upper is not None
        and slide_limits.upper >= 0.30,
        details=f"axis={slide.axis}, limits={slide_limits}",
    )
    ctx.check(
        "board tilt joint is horizontal",
        tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0)
        and tilt_limits is not None
        and tilt_limits.upper is not None
        and tilt_limits.upper > 1.0,
        details=f"axis={tilt.axis}, limits={tilt_limits}",
    )
    ctx.check(
        "locking knob rotates on side stub axis",
        tuple(round(v, 6) for v in knob_joint.axis) == (1.0, 0.0, 0.0)
        and knob_limits is not None
        and knob_limits.upper is not None
        and knob_limits.upper >= 2.0,
        details=f"axis={knob_joint.axis}, limits={knob_limits}",
    )

    with ctx.pose({slide: 0.0, tilt: 0.0}):
        ctx.expect_gap(
            bracket,
            column,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="back_pad",
            negative_elem="column_body",
            name="slider bracket bears on column front face",
        )
        ctx.expect_gap(
            board,
            column,
            axis="y",
            min_gap=0.030,
            positive_elem="board_panel",
            negative_elem="column_body",
            name="board panel stays in front of the column",
        )
        ctx.expect_overlap(
            bracket,
            column,
            axes="x",
            min_overlap=0.120,
            elem_a="front_plate",
            elem_b="column_body",
            name="slider carriage remains centered on the column",
        )

    rest_bracket_pos = ctx.part_world_position(bracket)
    upper_slide = slide.motion_limits.upper if slide.motion_limits and slide.motion_limits.upper is not None else 0.0
    with ctx.pose({slide: upper_slide}):
        raised_bracket_pos = ctx.part_world_position(bracket)
        ctx.expect_gap(
            bracket,
            column,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="back_pad",
            negative_elem="column_body",
            name="slider bracket stays on the column face at max height",
        )
    ctx.check(
        "upper bracket slides upward",
        rest_bracket_pos is not None
        and raised_bracket_pos is not None
        and raised_bracket_pos[2] > rest_bracket_pos[2] + 0.20,
        details=f"rest={rest_bracket_pos}, raised={raised_bracket_pos}",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rail_rest = None
    rail_tilted = None
    with ctx.pose({slide: 0.12, tilt: 0.0}):
        rail_rest = _center_from_aabb(ctx.part_element_world_aabb(board, elem="rail_body"))
    with ctx.pose({slide: 0.12, tilt: 1.0}):
        rail_tilted = _center_from_aabb(ctx.part_element_world_aabb(board, elem="rail_body"))
    ctx.check(
        "board tilt raises and advances the lower straightedge rail",
        rail_rest is not None
        and rail_tilted is not None
        and rail_tilted[1] > rail_rest[1] + 0.20
        and rail_tilted[2] > rail_rest[2] + 0.20,
        details=f"rest={rail_rest}, tilted={rail_tilted}",
    )

    knob_rest = None
    knob_rotated = None
    with ctx.pose({knob_joint: 0.0}):
        knob_rest = _center_from_aabb(ctx.part_element_world_aabb(knob, elem="knob_handle"))
    with ctx.pose({knob_joint: 1.2}):
        knob_rotated = _center_from_aabb(ctx.part_element_world_aabb(knob, elem="knob_handle"))
    ctx.check(
        "locking knob handle visibly rotates",
        knob_rest is not None
        and knob_rotated is not None
        and (
            abs(knob_rotated[1] - knob_rest[1]) > 0.010
            or abs(knob_rotated[2] - knob_rest[2]) > 0.010
        ),
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
