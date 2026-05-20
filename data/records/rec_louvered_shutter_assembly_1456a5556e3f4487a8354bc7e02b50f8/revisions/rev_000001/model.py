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
    superellipse_profile,
)


def _build_louver_mesh(name: str, *, length: float, height: float, thickness: float):
    profile = superellipse_profile(height, thickness, exponent=2.0, segments=32)
    geometry = (
        ExtrudeGeometry.centered(profile, length, cap=True, closed=True)
        .rotate_y(math.pi * 0.5)
        .rotate_x(math.pi)
    )
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter_panel")

    painted_white = model.material("painted_white", rgba=(0.95, 0.95, 0.93, 1.0))
    warm_white = model.material("warm_white", rgba=(0.97, 0.96, 0.92, 1.0))
    off_white = model.material("off_white", rgba=(0.92, 0.92, 0.89, 1.0))
    hinge_brass = model.material("hinge_brass", rgba=(0.73, 0.63, 0.39, 1.0))

    frame_width = 0.66
    frame_height = 1.20
    frame_depth = 0.045
    frame_bar = 0.05
    frame_inner_width = frame_width - 2.0 * frame_bar
    frame_inner_height = frame_height - 2.0 * frame_bar

    panel_gap = 0.006
    panel_width = frame_inner_width - 2.0 * panel_gap
    panel_height = frame_inner_height - 2.0 * panel_gap
    panel_depth = 0.03
    panel_stile = 0.05
    panel_rail = 0.09

    louver_count = 9
    louver_length = panel_width - 2.0 * panel_stile - 0.010
    louver_height = 0.089
    louver_thickness = 0.011
    louver_field_height = panel_height - 2.0 * panel_rail
    louver_margin = 0.055
    louver_pitch = (louver_field_height - 2.0 * louver_margin) / (louver_count - 1)
    louver_center_x = panel_stile + (panel_width - 2.0 * panel_stile) * 0.5

    tilt_rod_x = panel_width - panel_stile - 0.030
    tilt_rod_y = 0.019
    tilt_rod_width = 0.008
    tilt_rod_depth = 0.006

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Box((frame_bar, frame_depth, frame_height)),
        origin=Origin(xyz=(-(frame_width * 0.5) + frame_bar * 0.5, 0.0, frame_height * 0.5)),
        material=painted_white,
        name="frame_left_jamb",
    )
    outer_frame.visual(
        Box((frame_bar, frame_depth, frame_height)),
        origin=Origin(xyz=((frame_width * 0.5) - frame_bar * 0.5, 0.0, frame_height * 0.5)),
        material=painted_white,
        name="frame_right_jamb",
    )
    outer_frame.visual(
        Box((frame_width, frame_depth, frame_bar)),
        origin=Origin(xyz=(0.0, 0.0, frame_bar * 0.5)),
        material=painted_white,
        name="frame_bottom_rail",
    )
    outer_frame.visual(
        Box((frame_width, frame_depth, frame_bar)),
        origin=Origin(xyz=(0.0, 0.0, frame_height - frame_bar * 0.5)),
        material=painted_white,
        name="frame_top_rail",
    )
    for hinge_index, hinge_z in enumerate((0.28, 0.92)):
        outer_frame.visual(
            Box((0.018, 0.003, 0.120)),
            origin=Origin(xyz=(-0.289, frame_depth * 0.5 - 0.0015, hinge_z)),
            material=hinge_brass,
            name=f"frame_hinge_leaf_{hinge_index}",
        )
        outer_frame.visual(
            Box((0.006, 0.010, 0.120)),
            origin=Origin(xyz=(-0.277, 0.016, hinge_z)),
            material=hinge_brass,
            name=f"frame_hinge_knuckle_{hinge_index}",
        )
    outer_frame.inertial = Inertial.from_geometry(
        Box((frame_width, frame_depth, frame_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, frame_height * 0.5)),
    )

    shutter_panel = model.part("shutter_panel")
    shutter_panel.visual(
        Box((panel_stile, panel_depth, panel_height)),
        origin=Origin(xyz=(panel_stile * 0.5, 0.0, panel_height * 0.5)),
        material=warm_white,
        name="panel_left_stile",
    )
    shutter_panel.visual(
        Box((panel_stile, panel_depth, panel_height)),
        origin=Origin(xyz=(panel_width - panel_stile * 0.5, 0.0, panel_height * 0.5)),
        material=warm_white,
        name="panel_right_stile",
    )
    shutter_panel.visual(
        Box((panel_width, panel_depth, panel_rail)),
        origin=Origin(xyz=(panel_width * 0.5, 0.0, panel_rail * 0.5)),
        material=warm_white,
        name="panel_bottom_rail",
    )
    shutter_panel.visual(
        Box((panel_width, panel_depth, panel_rail)),
        origin=Origin(xyz=(panel_width * 0.5, 0.0, panel_height - panel_rail * 0.5)),
        material=warm_white,
        name="panel_top_rail",
    )
    for hinge_index, hinge_z in enumerate((0.28, 0.92)):
        shutter_panel.visual(
            Box((0.018, 0.003, 0.120)),
            origin=Origin(xyz=(0.009, panel_depth * 0.5 - 0.0015, hinge_z)),
            material=hinge_brass,
            name=f"panel_hinge_leaf_{hinge_index}",
        )
        shutter_panel.visual(
            Box((0.006, 0.010, 0.120)),
            origin=Origin(xyz=(0.003, 0.016, hinge_z)),
            material=hinge_brass,
            name=f"panel_hinge_knuckle_{hinge_index}",
        )
    shutter_panel.inertial = Inertial.from_geometry(
        Box((panel_width, panel_depth, panel_height)),
        mass=4.0,
        origin=Origin(xyz=(panel_width * 0.5, 0.0, panel_height * 0.5)),
    )

    model.articulation(
        "frame_to_panel",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=shutter_panel,
        origin=Origin(
            xyz=(
                -(frame_width * 0.5) + frame_bar + panel_gap,
                0.0,
                frame_bar + panel_gap,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=0.0,
            upper=1.45,
        ),
    )

    first_louver_center = panel_rail + louver_margin
    louver_centers = [
        first_louver_center + louver_pitch * index for index in range(louver_count)
    ]

    tilt_rod = model.part("tilt_rod")
    tilt_rod_length = (louver_centers[-1] - louver_centers[0]) + 0.060
    tilt_rod_center_z = (louver_centers[0] + louver_centers[-1]) * 0.5
    tilt_rod.visual(
        Box((tilt_rod_width, tilt_rod_depth, tilt_rod_length)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=off_white,
        name="rod_body",
    )
    tilt_rod.inertial = Inertial.from_geometry(
        Box((tilt_rod_width, tilt_rod_depth, tilt_rod_length)),
        mass=0.22,
        origin=Origin(),
    )
    model.articulation(
        "panel_to_tilt_rod",
        ArticulationType.FIXED,
        parent=shutter_panel,
        child=tilt_rod,
        origin=Origin(xyz=(tilt_rod_x, tilt_rod_y, tilt_rod_center_z)),
    )

    louver_mesh = _build_louver_mesh(
        "plantation_louver_blade",
        length=louver_length,
        height=louver_height,
        thickness=louver_thickness,
    )
    for index, center_z in enumerate(louver_centers):
        louver = model.part(f"louver_{index}")
        louver.visual(
            louver_mesh,
            material=off_white,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=0.004, length=0.005),
            origin=Origin(
                xyz=(-(louver_length * 0.5 + 0.0025), 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=off_white,
            name="left_pivot",
        )
        louver.visual(
            Cylinder(radius=0.004, length=0.005),
            origin=Origin(
                xyz=((louver_length * 0.5 + 0.0025), 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=off_white,
            name="right_pivot",
        )
        louver.visual(
            Box((0.018, 0.011, 0.010)),
            origin=Origin(xyz=(tilt_rod_x - louver_center_x, 0.0105, 0.0)),
            material=off_white,
            name="tilt_tab",
        )
        louver.inertial = Inertial.from_geometry(
            Box((louver_length, louver_thickness, louver_height)),
            mass=0.14,
            origin=Origin(),
        )
        model.articulation(
            f"panel_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=shutter_panel,
            child=louver,
            origin=Origin(xyz=(louver_center_x, 0.0, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.0,
                lower=-0.80,
                upper=0.80,
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

    outer_frame = object_model.get_part("outer_frame")
    shutter_panel = object_model.get_part("shutter_panel")
    tilt_rod = object_model.get_part("tilt_rod")
    panel_hinge = object_model.get_articulation("frame_to_panel")
    sample_louver = object_model.get_part("louver_4")
    sample_louver_joint = object_model.get_articulation("panel_to_louver_4")

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    def _aabb_span(aabb, axis_index: int):
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.expect_overlap(
        shutter_panel,
        outer_frame,
        axes="z",
        min_overlap=1.00,
        name="panel matches the tall frame opening",
    )
    ctx.expect_contact(
        tilt_rod,
        sample_louver,
        elem_a="rod_body",
        elem_b="tilt_tab",
        contact_tol=0.001,
        name="tilt rod sits against a louver tab",
    )

    closed_right_stile = ctx.part_element_world_aabb(shutter_panel, elem="panel_right_stile")
    with ctx.pose({panel_hinge: panel_hinge.motion_limits.upper}):
        open_right_stile = ctx.part_element_world_aabb(shutter_panel, elem="panel_right_stile")

    closed_right_stile_y = _aabb_center_y(closed_right_stile)
    open_right_stile_y = _aabb_center_y(open_right_stile)
    ctx.check(
        "panel swings outward from the frame",
        closed_right_stile_y is not None
        and open_right_stile_y is not None
        and open_right_stile_y > closed_right_stile_y + 0.12,
        details=f"closed_y={closed_right_stile_y}, open_y={open_right_stile_y}",
    )

    closed_blade = ctx.part_element_world_aabb(sample_louver, elem="blade")
    with ctx.pose({sample_louver_joint: 0.65}):
        opened_blade = ctx.part_element_world_aabb(sample_louver, elem="blade")

    closed_depth = _aabb_span(closed_blade, 1)
    opened_depth = _aabb_span(opened_blade, 1)
    ctx.check(
        "sample louver rotates about its long axis",
        closed_depth is not None
        and opened_depth is not None
        and opened_depth > closed_depth + 0.03,
        details=f"closed_depth={closed_depth}, opened_depth={opened_depth}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
