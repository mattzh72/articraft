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
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


CABINET_SIZE = 0.96
CABINET_PANEL = 0.018
CABINET_HEIGHT = 0.72
INNER_HEIGHT = CABINET_HEIGHT - (2.0 * CABINET_PANEL)
POLE_X = -0.08
POLE_Y = -0.08
POLE_RADIUS = 0.016
SLEEVE_INNER_RADIUS = 0.019
SLEEVE_OUTER_RADIUS = 0.034
CLIP_OUTER_RADIUS = 0.042
STOP_COLLAR_RADIUS = 0.045
STOP_COLLAR_LENGTH = 0.012
SHELF_THICKNESS = 0.022
LOWER_SHELF_Z = 0.19
UPPER_SHELF_Z = 0.443


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _kidney_outline() -> list[tuple[float, float]]:
    guide_points = [
        (0.040, -0.040),
        (0.110, -0.160),
        (0.220, -0.230),
        (0.340, -0.170),
        (0.380, 0.000),
        (0.340, 0.190),
        (0.220, 0.270),
        (0.040, 0.290),
        (-0.140, 0.250),
        (-0.250, 0.140),
        (-0.280, 0.000),
        (-0.250, -0.150),
        (-0.120, -0.250),
        (0.000, -0.200),
    ]
    return sample_catmull_rom_spline_2d(
        guide_points,
        samples_per_segment=10,
        closed=True,
    )


def _tube_mesh(name: str, *, inner_radius: float, outer_radius: float, length: float):
    half_length = length * 0.5
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half_length), (outer_radius, half_length)],
            [(inner_radius, -half_length), (inner_radius, half_length)],
            segments=56,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet_lazy_susan")

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.95, 0.93, 1.0))
    face_frame = model.material("face_frame", rgba=(0.90, 0.89, 0.86, 1.0))
    shelf_wood = model.material("shelf_wood", rgba=(0.72, 0.56, 0.38, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.24, 0.25, 0.27, 1.0))

    cabinet = model.part("cabinet_carcass")
    cabinet.visual(
        Box((CABINET_SIZE, CABINET_SIZE, CABINET_PANEL)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_PANEL * 0.5)),
        material=cabinet_white,
        name="floor_panel",
    )
    cabinet.visual(
        Box((CABINET_SIZE, CABINET_SIZE, CABINET_PANEL)),
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT - (CABINET_PANEL * 0.5))),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((CABINET_PANEL, CABINET_SIZE, INNER_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_SIZE * 0.5) + (CABINET_PANEL * 0.5),
                0.0,
                CABINET_PANEL + (INNER_HEIGHT * 0.5),
            )
        ),
        material=cabinet_white,
        name="back_wall",
    )
    cabinet.visual(
        Box((CABINET_SIZE, CABINET_PANEL, INNER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_SIZE * 0.5) + (CABINET_PANEL * 0.5),
                CABINET_PANEL + (INNER_HEIGHT * 0.5),
            )
        ),
        material=cabinet_white,
        name="side_wall",
    )
    cabinet.visual(
        Box((CABINET_PANEL, 0.36, INNER_HEIGHT)),
        origin=Origin(
            xyz=(
                (CABINET_SIZE * 0.5) - (CABINET_PANEL * 0.5),
                0.29,
                CABINET_PANEL + (INNER_HEIGHT * 0.5),
            )
        ),
        material=face_frame,
        name="front_return_right",
    )
    cabinet.visual(
        Box((0.36, CABINET_PANEL, INNER_HEIGHT)),
        origin=Origin(
            xyz=(
                0.29,
                (CABINET_SIZE * 0.5) - (CABINET_PANEL * 0.5),
                CABINET_PANEL + (INNER_HEIGHT * 0.5),
            )
        ),
        material=face_frame,
        name="front_return_left",
    )
    cabinet.visual(
        Cylinder(radius=POLE_RADIUS, length=INNER_HEIGHT),
        origin=Origin(
            xyz=(
                POLE_X,
                POLE_Y,
                CABINET_PANEL + (INNER_HEIGHT * 0.5),
            )
        ),
        material=chrome,
        name="center_pole",
    )
    cabinet.visual(
        Cylinder(radius=STOP_COLLAR_RADIUS, length=STOP_COLLAR_LENGTH),
        origin=Origin(
            xyz=(
                POLE_X,
                POLE_Y,
                LOWER_SHELF_Z - 0.006,
            )
        ),
        material=dark_hardware,
        name="lower_stop_collar",
    )
    cabinet.visual(
        Cylinder(radius=STOP_COLLAR_RADIUS, length=STOP_COLLAR_LENGTH),
        origin=Origin(
            xyz=(
                POLE_X,
                POLE_Y,
                UPPER_SHELF_Z + 0.042,
            )
        ),
        material=dark_hardware,
        name="upper_stop_collar",
    )

    shelf_outline = _kidney_outline()
    shelf_hole = _circle_profile(SLEEVE_OUTER_RADIUS, segments=32)
    shelf_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            shelf_outline,
            [shelf_hole],
            height=SHELF_THICKNESS,
            center=True,
        ),
        "kidney_shelf_board",
    )
    sleeve_mesh = _tube_mesh(
        "lazy_susan_center_sleeve",
        inner_radius=SLEEVE_INNER_RADIUS,
        outer_radius=SLEEVE_OUTER_RADIUS,
        length=UPPER_SHELF_Z - LOWER_SHELF_Z + SHELF_THICKNESS,
    )
    clip_mesh = _tube_mesh(
        "lazy_susan_clip_ring",
        inner_radius=SLEEVE_INNER_RADIUS,
        outer_radius=CLIP_OUTER_RADIUS,
        length=0.012,
    )

    rotating_shelves = model.part("rotating_shelves")
    rotating_shelves.visual(
        sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, (LOWER_SHELF_Z + UPPER_SHELF_Z) * 0.5)),
        material=chrome,
        name="center_sleeve",
    )
    rotating_shelves.visual(
        shelf_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, LOWER_SHELF_Z),
            rpy=(0.0, 0.0, math.pi / 4.0),
        ),
        material=shelf_wood,
        name="lower_shelf",
    )
    rotating_shelves.visual(
        shelf_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, UPPER_SHELF_Z),
            rpy=(0.0, 0.0, math.pi / 4.0),
        ),
        material=shelf_wood,
        name="upper_shelf",
    )
    for visual_name, clip_z in (
        ("lower_clip_below", LOWER_SHELF_Z - 0.012),
        ("lower_clip_above", LOWER_SHELF_Z + 0.012),
        ("upper_clip_below", UPPER_SHELF_Z - 0.012),
        ("upper_clip_above", UPPER_SHELF_Z + 0.012),
    ):
        rotating_shelves.visual(
            clip_mesh,
            origin=Origin(xyz=(0.0, 0.0, clip_z)),
            material=dark_hardware,
            name=visual_name,
        )

    model.articulation(
        "shelf_rotation",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=rotating_shelves,
        origin=Origin(xyz=(POLE_X, POLE_Y, CABINET_PANEL)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet_carcass")
    rotating_shelves = object_model.get_part("rotating_shelves")
    shelf_rotation = object_model.get_articulation("shelf_rotation")

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

    ctx.check(
        "shelf_rotation_is_continuous",
        shelf_rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous rotation, got {shelf_rotation.articulation_type}",
    )
    ctx.check(
        "shelf_rotation_axis_is_vertical",
        tuple(float(value) for value in shelf_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical z axis, got {shelf_rotation.axis}",
    )

    ctx.expect_gap(
        rotating_shelves,
        cabinet,
        axis="z",
        positive_elem="lower_shelf",
        negative_elem="floor_panel",
        min_gap=0.16,
        max_gap=0.19,
        name="lower_shelf_above_floor",
    )
    ctx.expect_gap(
        cabinet,
        rotating_shelves,
        axis="z",
        positive_elem="top_panel",
        negative_elem="upper_shelf",
        min_gap=0.229,
        max_gap=0.28,
        name="upper_shelf_below_top_panel",
    )
    ctx.expect_overlap(
        cabinet,
        rotating_shelves,
        axes="xy",
        elem_a="center_pole",
        elem_b="center_sleeve",
        min_overlap=0.03,
        name="center_sleeve_tracks_pole_axis",
    )
    ctx.expect_within(
        cabinet,
        rotating_shelves,
        axes="xy",
        inner_elem="center_pole",
        outer_elem="center_sleeve",
        margin=0.0,
        name="pole_within_sleeve_footprint",
    )
    ctx.expect_contact(
        cabinet,
        rotating_shelves,
        elem_a="lower_stop_collar",
        elem_b="lower_clip_below",
        name="lower_clip_seated_on_stop_collar",
    )
    ctx.expect_contact(
        cabinet,
        rotating_shelves,
        elem_a="upper_stop_collar",
        elem_b="upper_clip_above",
        name="upper_clip_retained_by_stop_collar",
    )

    with ctx.pose({shelf_rotation: math.pi / 2.0}):
        ctx.expect_gap(
            rotating_shelves,
            cabinet,
            axis="z",
            positive_elem="lower_shelf",
            negative_elem="floor_panel",
            min_gap=0.16,
            max_gap=0.19,
            name="lower_shelf_above_floor_when_turned",
        )
        ctx.expect_gap(
            cabinet,
            rotating_shelves,
            axis="z",
            positive_elem="top_panel",
            negative_elem="upper_shelf",
            min_gap=0.229,
            max_gap=0.28,
            name="upper_shelf_below_top_panel_when_turned",
        )
        ctx.expect_overlap(
            cabinet,
            rotating_shelves,
            axes="xy",
            elem_a="center_pole",
            elem_b="center_sleeve",
            min_overlap=0.03,
            name="center_sleeve_tracks_pole_axis_when_turned",
        )
        ctx.expect_contact(
            cabinet,
            rotating_shelves,
            elem_a="lower_stop_collar",
            elem_b="lower_clip_below",
            name="lower_clip_seated_on_stop_collar_when_turned",
        )
        ctx.expect_contact(
            cabinet,
            rotating_shelves,
            elem_a="upper_stop_collar",
            elem_b="upper_clip_above",
            name="upper_clip_retained_by_stop_collar_when_turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
