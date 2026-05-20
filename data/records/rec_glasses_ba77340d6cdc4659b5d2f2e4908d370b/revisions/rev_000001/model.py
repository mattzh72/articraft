from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _yz_section(
    x_pos: float,
    *,
    center_y: float,
    center_z: float,
    depth: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, center_y + dy, center_z + dz)
        for dy, dz in rounded_rect_profile(depth, height, radius)
    ]


def _build_front_shield_mesh():
    section_specs = (
        (-0.077, -0.031, 0.000, 0.0024, 0.041),
        (-0.062, -0.020, 0.001, 0.0024, 0.046),
        (-0.036, 0.006, 0.0015, 0.0024, 0.050),
        (0.000, 0.020, 0.002, 0.0024, 0.053),
        (0.036, 0.006, 0.0015, 0.0024, 0.050),
        (0.062, -0.020, 0.001, 0.0024, 0.046),
        (0.077, -0.031, 0.000, 0.0024, 0.041),
    )
    sections = [
        _yz_section(
            x_pos,
            center_y=center_y,
            center_z=center_z,
            depth=depth,
            height=height,
            radius=0.0008,
        )
        for x_pos, center_y, center_z, depth, height in section_specs
    ]
    return section_loft(sections)


def _build_brow_frame_mesh():
    section_specs = (
        (-0.077, -0.029, 0.020, 0.0048, 0.010),
        (-0.062, -0.018, 0.021, 0.0048, 0.011),
        (-0.036, 0.008, 0.0215, 0.0048, 0.012),
        (0.000, 0.022, 0.022, 0.0050, 0.012),
        (0.036, 0.008, 0.0215, 0.0048, 0.012),
        (0.062, -0.018, 0.021, 0.0048, 0.011),
        (0.077, -0.029, 0.020, 0.0048, 0.010),
    )
    sections = [
        _yz_section(
            x_pos,
            center_y=center_y,
            center_z=center_z,
            depth=depth,
            height=height,
            radius=0.0014,
        )
        for x_pos, center_y, center_z, depth, height in section_specs
    ]
    return section_loft(sections)


def _build_rear_hook_mesh():
    return tube_from_spline_points(
        [
            (0.024, 0.000, 0.000),
            (0.045, 0.000, -0.004),
            (0.067, 0.000, -0.011),
            (0.086, 0.000, -0.020),
        ],
        radius=0.0018,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def _build_temple_fairing_mesh():
    section_specs = (
        (0.004, 0.000, 0.0005, 0.0140, 0.0090),
        (0.012, 0.000, 0.0014, 0.0135, 0.0086),
        (0.022, 0.000, 0.0020, 0.0130, 0.0078),
        (0.032, 0.000, 0.0018, 0.0124, 0.0070),
        (0.042, 0.000, 0.0010, 0.0116, 0.0062),
    )
    sections = [
        _yz_section(
            x_pos,
            center_y=center_y,
            center_z=center_z,
            depth=depth,
            height=height,
            radius=0.0018,
        )
        for x_pos, center_y, center_z, depth, height in section_specs
    ]
    return section_loft(sections)


def _add_outer_temple(model: ArticulatedObject, name: str, material) -> object:
    part = model.part(name)

    sleeve_length = 0.070
    sleeve_center_x = 0.0445
    outer_width = 0.0120
    outer_height = 0.0060
    wall_thickness = 0.0018
    side_wall_y = (outer_width - wall_thickness) * 0.5
    top_wall_z = (outer_height - wall_thickness) * 0.5

    part.visual(
        Box((0.014, 0.014, 0.008)),
        origin=Origin(xyz=(0.007, 0.0, 0.0)),
        material=material,
        name="hinge_block",
    )
    part.visual(
        mesh_from_geometry(_build_temple_fairing_mesh(), f"{name}_fairing"),
        material=material,
        name="temple_fairing",
    )
    part.visual(
        Box((sleeve_length, outer_width, wall_thickness)),
        origin=Origin(xyz=(sleeve_center_x, 0.0, top_wall_z)),
        material=material,
        name="top_wall",
    )
    part.visual(
        Box((sleeve_length, outer_width, wall_thickness)),
        origin=Origin(xyz=(sleeve_center_x, 0.0, -top_wall_z)),
        material=material,
        name="bottom_wall",
    )
    part.visual(
        Box((sleeve_length, wall_thickness, outer_height)),
        origin=Origin(xyz=(sleeve_center_x, side_wall_y, 0.0)),
        material=material,
        name="outer_side_wall",
    )
    part.visual(
        Box((sleeve_length, wall_thickness, outer_height)),
        origin=Origin(xyz=(sleeve_center_x, -side_wall_y, 0.0)),
        material=material,
        name="inner_side_wall",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.086, 0.016, 0.010)),
        mass=0.030,
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
    )
    return part


def _add_rear_temple(model: ArticulatedObject, name: str, bar_material, tip_material, hook_mesh) -> object:
    part = model.part(name)
    part.visual(
        Box((0.102, 0.0072, 0.0026)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material=bar_material,
        name="slider_core",
    )
    part.visual(
        mesh_from_geometry(hook_mesh, f"{name}_hook"),
        material=tip_material,
        name="ear_hook",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.148, 0.012, 0.026)),
        mass=0.018,
        origin=Origin(xyz=(0.014, 0.0, -0.010)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="safety_glasses")

    clear_lens = model.material("clear_lens", rgba=(0.84, 0.92, 1.0, 0.32))
    graphite = model.material("graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    temple_black = model.material("temple_black", rgba=(0.12, 0.13, 0.14, 1.0))
    soft_rubber = model.material("soft_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("front_frame")
    frame.visual(
        mesh_from_geometry(_build_front_shield_mesh(), "front_shield"),
        material=clear_lens,
        name="front_shield",
    )
    frame.visual(
        mesh_from_geometry(_build_brow_frame_mesh(), "brow_frame"),
        material=graphite,
        name="brow_frame",
    )
    frame.visual(
        Box((0.020, 0.016, 0.010)),
        origin=Origin(xyz=(0.000, 0.014, -0.013)),
        material=clear_lens,
        name="nose_saddle",
    )
    frame.visual(
        Box((0.020, 0.006, 0.012)),
        origin=Origin(xyz=(-0.068, -0.020, 0.003)),
        material=graphite,
        name="left_hinge_lug",
    )
    frame.visual(
        Box((0.020, 0.006, 0.012)),
        origin=Origin(xyz=(0.068, -0.020, 0.003)),
        material=graphite,
        name="right_hinge_lug",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.165, 0.060, 0.058)),
        mass=0.095,
        origin=Origin(xyz=(0.000, -0.002, 0.001)),
    )

    left_temple = _add_outer_temple(model, "left_temple", temple_black)
    right_temple = _add_outer_temple(model, "right_temple", temple_black)

    rear_hook_mesh = _build_rear_hook_mesh()
    left_rear = _add_rear_temple(
        model,
        "left_rear_temple",
        temple_black,
        soft_rubber,
        rear_hook_mesh,
    )
    right_rear = _add_rear_temple(
        model,
        "right_rear_temple",
        temple_black,
        soft_rubber,
        rear_hook_mesh,
    )

    hinge_upper = math.radians(76.0)
    slide_upper = 0.038

    model.articulation(
        "frame_to_left_temple",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_temple,
        origin=Origin(xyz=(-0.085, -0.023, 0.003), rpy=(0.0, 0.0, -math.pi / 2.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=hinge_upper,
        ),
    )
    model.articulation(
        "frame_to_right_temple",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_temple,
        origin=Origin(xyz=(0.085, -0.023, 0.003), rpy=(0.0, 0.0, -math.pi / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=hinge_upper,
        ),
    )
    model.articulation(
        "left_temple_to_rear",
        ArticulationType.PRISMATIC,
        parent=left_temple,
        child=left_rear,
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.12,
            lower=0.0,
            upper=slide_upper,
        ),
    )
    model.articulation(
        "right_temple_to_rear",
        ArticulationType.PRISMATIC,
        parent=right_temple,
        child=right_rear,
        origin=Origin(xyz=(0.079, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.12,
            lower=0.0,
            upper=slide_upper,
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

    frame = object_model.get_part("front_frame")
    left_temple = object_model.get_part("left_temple")
    right_temple = object_model.get_part("right_temple")
    left_rear = object_model.get_part("left_rear_temple")
    right_rear = object_model.get_part("right_rear_temple")

    left_hinge = object_model.get_articulation("frame_to_left_temple")
    right_hinge = object_model.get_articulation("frame_to_right_temple")
    left_slide = object_model.get_articulation("left_temple_to_rear")
    right_slide = object_model.get_articulation("right_temple_to_rear")

    frame_aabb = ctx.part_world_aabb(frame)
    frame_dims = None
    if frame_aabb is not None:
        frame_dims = (
            frame_aabb[1][0] - frame_aabb[0][0],
            frame_aabb[1][1] - frame_aabb[0][1],
            frame_aabb[1][2] - frame_aabb[0][2],
        )
    ctx.check(
        "front shield has wraparound safety-glasses proportions",
        frame_dims is not None and frame_dims[0] > 0.150 and frame_dims[1] > 0.050 and frame_dims[2] > 0.040,
        details=f"frame_dims={frame_dims}",
    )

    with ctx.pose(
        {
            left_hinge: 0.0,
            right_hinge: 0.0,
            left_slide: 0.0,
            right_slide: 0.0,
        }
    ):
        ctx.expect_within(
            left_rear,
            left_temple,
            axes="xz",
            inner_elem="slider_core",
            margin=0.0025,
            name="left rear segment stays centered inside the outer temple",
        )
        ctx.expect_within(
            right_rear,
            right_temple,
            axes="xz",
            inner_elem="slider_core",
            margin=0.0025,
            name="right rear segment stays centered inside the outer temple",
        )
        ctx.expect_overlap(
            left_rear,
            left_temple,
            axes="y",
            elem_a="slider_core",
            min_overlap=0.050,
            name="left rear segment remains deeply inserted when collapsed",
        )
        ctx.expect_overlap(
            right_rear,
            right_temple,
            axes="y",
            elem_a="slider_core",
            min_overlap=0.050,
            name="right rear segment remains deeply inserted when collapsed",
        )

    with ctx.pose(
        {
            left_hinge: 0.0,
            right_hinge: 0.0,
            left_slide: left_slide.motion_limits.upper,
            right_slide: right_slide.motion_limits.upper,
        }
    ):
        ctx.expect_within(
            left_rear,
            left_temple,
            axes="xz",
            inner_elem="slider_core",
            margin=0.0025,
            name="left rear segment stays aligned inside the sleeve when extended",
        )
        ctx.expect_within(
            right_rear,
            right_temple,
            axes="xz",
            inner_elem="slider_core",
            margin=0.0025,
            name="right rear segment stays aligned inside the sleeve when extended",
        )
        ctx.expect_overlap(
            left_rear,
            left_temple,
            axes="y",
            elem_a="slider_core",
            min_overlap=0.018,
            name="left rear segment retains insertion at full extension",
        )
        ctx.expect_overlap(
            right_rear,
            right_temple,
            axes="y",
            elem_a="slider_core",
            min_overlap=0.018,
            name="right rear segment retains insertion at full extension",
        )

    rest_left_rear = None
    rest_right_rear = None
    with ctx.pose(
        {
            left_hinge: 0.0,
            right_hinge: 0.0,
            left_slide: 0.0,
            right_slide: 0.0,
        }
    ):
        rest_left_rear = ctx.part_world_position(left_rear)
        rest_right_rear = ctx.part_world_position(right_rear)

    extended_left_rear = None
    extended_right_rear = None
    with ctx.pose(
        {
            left_hinge: 0.0,
            right_hinge: 0.0,
            left_slide: left_slide.motion_limits.upper,
            right_slide: right_slide.motion_limits.upper,
        }
    ):
        extended_left_rear = ctx.part_world_position(left_rear)
        extended_right_rear = ctx.part_world_position(right_rear)

    ctx.check(
        "rear temple segments extend backward for fit adjustment",
        rest_left_rear is not None
        and rest_right_rear is not None
        and extended_left_rear is not None
        and extended_right_rear is not None
        and extended_left_rear[1] < rest_left_rear[1] - 0.030
        and extended_right_rear[1] < rest_right_rear[1] - 0.030,
        details=(
            f"rest_left={rest_left_rear}, extended_left={extended_left_rear}, "
            f"rest_right={rest_right_rear}, extended_right={extended_right_rear}"
        ),
    )

    folded_left_rear = None
    folded_right_rear = None
    with ctx.pose(
        {
            left_hinge: left_hinge.motion_limits.upper,
            right_hinge: right_hinge.motion_limits.upper,
            left_slide: 0.0,
            right_slide: 0.0,
        }
    ):
        folded_left_rear = ctx.part_world_position(left_rear)
        folded_right_rear = ctx.part_world_position(right_rear)

    ctx.check(
        "temples fold inward from the frame corners",
        rest_left_rear is not None
        and rest_right_rear is not None
        and folded_left_rear is not None
        and folded_right_rear is not None
        and folded_left_rear[0] > rest_left_rear[0] + 0.040
        and folded_right_rear[0] < rest_right_rear[0] - 0.040,
        details=(
            f"rest_left={rest_left_rear}, folded_left={folded_left_rear}, "
            f"rest_right={rest_right_rear}, folded_right={folded_right_rear}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
