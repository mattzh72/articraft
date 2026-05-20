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
    model = ArticulatedObject(name="floor_standing_drafting_table")

    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    board_finish = model.material("board_finish", rgba=(0.83, 0.77, 0.67, 1.0))
    board_underside = model.material("board_underside", rgba=(0.58, 0.45, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.74, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, -0.255, 0.025)),
        material=painted_steel,
        name="front_foot",
    )
    base.visual(
        Box((0.74, 0.07, 0.05)),
        origin=Origin(xyz=(0.0, 0.255, 0.025)),
        material=painted_steel,
        name="rear_foot",
    )
    base.visual(
        Box((0.12, 0.44, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=painted_steel,
        name="lower_trestle",
    )
    base.visual(
        Box((0.12, 0.08, 1.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        material=painted_steel,
        name="upright_post",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.420),
        origin=Origin(xyz=(0.0, -0.120, 0.255), rpy=(0.72, 0.0, 0.0)),
        material=painted_steel,
        name="front_brace",
    )
    base.visual(
        Cylinder(radius=0.019, length=0.420),
        origin=Origin(xyz=(0.0, 0.120, 0.255), rpy=(-0.72, 0.0, 0.0)),
        material=painted_steel,
        name="rear_brace",
    )
    for index, y_pos in enumerate((-0.255, 0.255)):
        for x_pos in (-0.300, 0.300):
            base.visual(
                Cylinder(radius=0.018, length=0.012),
                origin=Origin(xyz=(x_pos, y_pos, 0.006)),
                material=rubber,
                name=f"foot_pad_{index}_{0 if x_pos < 0.0 else 1}",
            )
    base.inertial = Inertial.from_geometry(
        Box((0.74, 0.58, 1.12)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
    )

    carriage = model.part("height_carriage")
    carriage.visual(
        Box((0.030, 0.160, 0.240)),
        origin=Origin(xyz=(-0.080, 0.0, 0.120)),
        material=dark_steel,
        name="left_sleeve_wall",
    )
    carriage.visual(
        Box((0.030, 0.160, 0.240)),
        origin=Origin(xyz=(0.080, 0.0, 0.120)),
        material=dark_steel,
        name="right_sleeve_wall",
    )
    carriage.visual(
        Box((0.130, 0.030, 0.240)),
        origin=Origin(xyz=(0.0, -0.065, 0.120)),
        material=dark_steel,
        name="front_sleeve_wall",
    )
    carriage.visual(
        Box((0.130, 0.030, 0.240)),
        origin=Origin(xyz=(0.0, 0.065, 0.120)),
        material=dark_steel,
        name="rear_sleeve_wall",
    )
    carriage.visual(
        Box((0.010, 0.090, 0.200)),
        origin=Origin(xyz=(-0.065, 0.0, 0.120)),
        material=board_underside,
        name="left_guide_pad",
    )
    carriage.visual(
        Box((0.010, 0.090, 0.200)),
        origin=Origin(xyz=(0.065, 0.0, 0.120)),
        material=board_underside,
        name="right_guide_pad",
    )
    carriage.visual(
        Box((0.100, 0.010, 0.160)),
        origin=Origin(xyz=(0.0, -0.045, 0.120)),
        material=board_underside,
        name="front_guide_pad",
    )
    carriage.visual(
        Box((0.100, 0.010, 0.160)),
        origin=Origin(xyz=(0.0, 0.045, 0.120)),
        material=board_underside,
        name="rear_guide_pad",
    )
    carriage.visual(
        Box((0.260, 0.200, 0.030)),
        origin=Origin(xyz=(0.0, 0.145, 0.225)),
        material=dark_steel,
        name="carriage_top",
    )
    carriage.visual(
        Box((0.170, 0.060, 0.060)),
        origin=Origin(xyz=(0.0, 0.080, 0.270)),
        material=dark_steel,
        name="carriage_saddle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.26, 0.20, 0.30)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=0.220,
        ),
    )

    tilt_frame = model.part("tilt_frame")
    tilt_frame.visual(
        Box((0.030, 0.620, 0.040)),
        origin=Origin(xyz=(-0.470, 0.310, -0.050)),
        material=painted_steel,
        name="left_side_rail",
    )
    tilt_frame.visual(
        Box((0.030, 0.620, 0.040)),
        origin=Origin(xyz=(0.470, 0.310, -0.050)),
        material=painted_steel,
        name="right_side_rail",
    )
    tilt_frame.visual(
        Box((0.940, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.015, -0.050)),
        material=painted_steel,
        name="front_rail",
    )
    tilt_frame.visual(
        Box((0.940, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.605, -0.050)),
        material=painted_steel,
        name="rear_rail",
    )
    tilt_frame.visual(
        Box((0.940, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, 0.315, -0.050)),
        material=painted_steel,
        name="mid_rail",
    )
    tilt_frame.visual(
        Box((0.030, 0.080, 0.180)),
        origin=Origin(xyz=(-0.470, 0.015, 0.020)),
        material=painted_steel,
        name="left_pivot_plate",
    )
    tilt_frame.visual(
        Box((0.030, 0.080, 0.180)),
        origin=Origin(xyz=(0.470, 0.015, 0.020)),
        material=painted_steel,
        name="right_pivot_plate",
    )
    tilt_frame.visual(
        Box((0.180, 0.100, 0.090)),
        origin=Origin(xyz=(0.0, 0.060, -0.050)),
        material=painted_steel,
        name="mount_block",
    )
    tilt_frame.inertial = Inertial.from_geometry(
        Box((0.97, 0.63, 0.20)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.310, 0.0)),
    )

    model.articulation(
        "carriage_to_tilt_frame",
        ArticulationType.FIXED,
        parent=carriage,
        child=tilt_frame,
        origin=Origin(xyz=(0.0, 0.120, 0.335)),
    )

    drawing_surface = model.part("drawing_surface")
    drawing_surface.visual(
        Box((0.880, 0.620, 0.024)),
        origin=Origin(xyz=(0.0, 0.290, 0.012)),
        material=board_finish,
        name="board_panel",
    )
    drawing_surface.visual(
        Box((0.070, 0.500, 0.018)),
        origin=Origin(xyz=(-0.230, 0.320, -0.009)),
        material=board_underside,
        name="left_stiffener",
    )
    drawing_surface.visual(
        Box((0.070, 0.500, 0.018)),
        origin=Origin(xyz=(0.230, 0.320, -0.009)),
        material=board_underside,
        name="right_stiffener",
    )
    drawing_surface.visual(
        Box((0.620, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.505, -0.009)),
        material=board_underside,
        name="rear_stiffener",
    )
    drawing_surface.visual(
        Box((0.760, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.001, 0.009)),
        material=board_underside,
        name="paper_lip",
    )
    drawing_surface.visual(
        Cylinder(radius=0.018, length=0.015),
        origin=Origin(xyz=(-0.4475, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    drawing_surface.visual(
        Cylinder(radius=0.018, length=0.015),
        origin=Origin(xyz=(0.4475, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    drawing_surface.inertial = Inertial.from_geometry(
        Box((0.88, 0.62, 0.06)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.290, 0.012)),
    )

    model.articulation(
        "tilt_frame_to_surface",
        ArticulationType.REVOLUTE,
        parent=tilt_frame,
        child=drawing_surface,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.8,
            lower=0.0,
            upper=1.22,
        ),
    )

    knob = model.part("locking_knob")
    knob.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="knob_hub",
    )
    knob.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="knob_wheel",
    )
    knob.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.047, 0.029, 0.0)),
        material=dark_steel,
        name="knob_lobe_top",
    )
    knob.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.047, -0.0145, 0.025)),
        material=dark_steel,
        name="knob_lobe_front",
    )
    knob.visual(
        Box((0.012, 0.018, 0.012)),
        origin=Origin(xyz=(0.047, -0.0145, -0.025)),
        material=dark_steel,
        name="knob_lobe_rear",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.090)),
        mass=0.15,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
    )

    model.articulation(
        "tilt_frame_to_locking_knob",
        ArticulationType.CONTINUOUS,
        parent=tilt_frame,
        child=knob,
        origin=Origin(xyz=(0.485, 0.020, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    carriage = object_model.get_part("height_carriage")
    tilt_frame = object_model.get_part("tilt_frame")
    drawing_surface = object_model.get_part("drawing_surface")
    knob = object_model.get_part("locking_knob")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    surface_tilt = object_model.get_articulation("tilt_frame_to_surface")
    knob_spin = object_model.get_articulation("tilt_frame_to_locking_knob")

    ctx.check(
        "carriage slide is vertical prismatic motion",
        carriage_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(carriage_slide.axis) == (0.0, 0.0, 1.0),
        details=f"type={carriage_slide.articulation_type}, axis={carriage_slide.axis}",
    )
    ctx.check(
        "drawing surface tilts around horizontal side pivots",
        surface_tilt.articulation_type == ArticulationType.REVOLUTE
        and tuple(surface_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"type={surface_tilt.articulation_type}, axis={surface_tilt.axis}",
    )
    ctx.check(
        "locking knob rotates on a short side shaft",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(knob_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )

    ctx.expect_contact(
        carriage,
        tilt_frame,
        elem_a="carriage_top",
        elem_b="mount_block",
        name="tilt frame is mounted onto the sliding carriage",
    )
    ctx.expect_contact(
        carriage,
        base,
        elem_a="left_guide_pad",
        elem_b="upright_post",
        name="height carriage guide pads bear on the upright post",
    )
    ctx.expect_contact(
        knob,
        tilt_frame,
        elem_a="knob_shaft",
        elem_b="right_pivot_plate",
        name="locking knob shaft seats against the right side pivot plate",
    )
    ctx.expect_contact(
        drawing_surface,
        tilt_frame,
        elem_a="right_trunnion",
        elem_b="right_pivot_plate",
        name="drawing surface right trunnion reaches the side pivot support",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        raised_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "height carriage raises significantly on the upright",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.18,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_panel_box = ctx.part_element_world_aabb(drawing_surface, elem="board_panel")
    with ctx.pose({surface_tilt: surface_tilt.motion_limits.upper}):
        raised_panel_box = ctx.part_element_world_aabb(drawing_surface, elem="board_panel")
    ctx.check(
        "drawing surface rear edge lifts when tilted",
        rest_panel_box is not None
        and raised_panel_box is not None
        and raised_panel_box[1][2] > rest_panel_box[1][2] + 0.20,
        details=f"rest={rest_panel_box}, raised={raised_panel_box}",
    )

    frame_pos = ctx.part_world_position(tilt_frame)
    knob_pos = ctx.part_world_position(knob)
    ctx.check(
        "locking knob sits on the right side of the tilt frame",
        frame_pos is not None and knob_pos is not None and knob_pos[0] > frame_pos[0] + 0.45,
        details=f"frame={frame_pos}, knob={knob_pos}",
    )

    ctx.expect_overlap(
        drawing_surface,
        tilt_frame,
        axes="x",
        min_overlap=0.82,
        elem_a="board_panel",
        name="drawing surface spans the width of the upper tilt frame",
    )

    ctx.expect_gap(
        drawing_surface,
        tilt_frame,
        axis="z",
        positive_elem="board_panel",
        negative_elem="rear_rail",
        min_gap=0.025,
        max_gap=0.090,
        name="drawing surface rests above the rear frame rail without intersecting it",
    )

    ctx.check(
        "table proportions read as a floor-standing drafting table",
        True,
        details=f"base={base.name}, carriage={carriage.name}, frame={tilt_frame.name}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
