from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)

FRAME_WIDTH = 0.68
FRAME_HEIGHT = 1.04
FRAME_DEPTH = 0.045
STILE_WIDTH = 0.06
RAIL_HEIGHT = 0.08
INNER_WIDTH = FRAME_WIDTH - 2.0 * STILE_WIDTH
INNER_HEIGHT = FRAME_HEIGHT - 2.0 * RAIL_HEIGHT

LOUVER_COUNT = 8
BLADE_HEIGHT = 0.086
BLADE_THICKNESS = 0.011
BLADE_BODY_LENGTH = 0.548
PIVOT_RADIUS = 0.005
PIVOT_TOTAL_LENGTH = 0.008
PIVOT_CENTER_X = INNER_WIDTH * 0.5 - PIVOT_TOTAL_LENGTH * 0.5
LOUVER_CLEAR_GAP = 0.018
LOUVER_SWING = math.radians(40.0)


def _louver_centers() -> list[float]:
    pitch = BLADE_HEIGHT + LOUVER_CLEAR_GAP
    used_height = LOUVER_COUNT * BLADE_HEIGHT + (LOUVER_COUNT - 1) * LOUVER_CLEAR_GAP
    margin = (INNER_HEIGHT - used_height) * 0.5
    start = -INNER_HEIGHT * 0.5 + margin + BLADE_HEIGHT * 0.5
    return [start + index * pitch for index in range(LOUVER_COUNT)]


def _write_louver_mesh():
    profile = superellipse_profile(
        BLADE_HEIGHT,
        BLADE_THICKNESS,
        exponent=1.8,
        segments=48,
    )
    geom = ExtrudeGeometry(
        profile,
        BLADE_BODY_LENGTH,
        cap=True,
        center=True,
        closed=True,
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path("plantation_shutter_louver.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plantation_shutter", assets=ASSETS)

    painted_wood = model.material("painted_wood", rgba=(0.94, 0.93, 0.90, 1.0))
    pivot_metal = model.material("pivot_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    shadow_trim = model.material("shadow_trim", rgba=(0.82, 0.82, 0.80, 1.0))

    louver_mesh = _write_louver_mesh()

    frame = model.part("frame")
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-FRAME_WIDTH * 0.5 + STILE_WIDTH * 0.5, 0.0, 0.0)),
        material=painted_wood,
        name="left_stile",
    )
    frame.visual(
        Box((STILE_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(FRAME_WIDTH * 0.5 - STILE_WIDTH * 0.5, 0.0, 0.0)),
        material=painted_wood,
        name="right_stile",
    )
    frame.visual(
        Box((INNER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT * 0.5 - RAIL_HEIGHT * 0.5)),
        material=painted_wood,
        name="top_rail",
    )
    frame.visual(
        Box((INNER_WIDTH, FRAME_DEPTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -FRAME_HEIGHT * 0.5 + RAIL_HEIGHT * 0.5)),
        material=painted_wood,
        name="bottom_rail",
    )
    frame.visual(
        Box((STILE_WIDTH * 0.55, FRAME_DEPTH * 0.18, FRAME_HEIGHT - 0.05)),
        origin=Origin(
            xyz=(-FRAME_WIDTH * 0.5 + STILE_WIDTH * 0.5, -FRAME_DEPTH * 0.41, 0.0)
        ),
        material=shadow_trim,
        name="left_back_bead",
    )
    frame.visual(
        Box((STILE_WIDTH * 0.55, FRAME_DEPTH * 0.18, FRAME_HEIGHT - 0.05)),
        origin=Origin(
            xyz=(FRAME_WIDTH * 0.5 - STILE_WIDTH * 0.5, -FRAME_DEPTH * 0.41, 0.0)
        ),
        material=shadow_trim,
        name="right_back_bead",
    )
    frame.visual(
        Box((INNER_WIDTH - 0.04, FRAME_DEPTH * 0.18, RAIL_HEIGHT * 0.55)),
        origin=Origin(
            xyz=(0.0, -FRAME_DEPTH * 0.41, FRAME_HEIGHT * 0.5 - RAIL_HEIGHT * 0.5)
        ),
        material=shadow_trim,
        name="top_back_bead",
    )
    frame.visual(
        Box((INNER_WIDTH - 0.04, FRAME_DEPTH * 0.18, RAIL_HEIGHT * 0.55)),
        origin=Origin(
            xyz=(0.0, -FRAME_DEPTH * 0.41, -FRAME_HEIGHT * 0.5 + RAIL_HEIGHT * 0.5)
        ),
        material=shadow_trim,
        name="bottom_back_bead",
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, FRAME_HEIGHT)),
        mass=8.5,
        origin=Origin(),
    )

    for index, center_z in enumerate(_louver_centers(), start=1):
        louver = model.part(f"louver_{index}")
        louver.visual(louver_mesh, material=painted_wood, name="blade")
        louver.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIVOT_TOTAL_LENGTH),
            origin=Origin(
                xyz=(-PIVOT_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pivot_metal,
            name="left_pivot",
        )
        louver.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIVOT_TOTAL_LENGTH),
            origin=Origin(
                xyz=(PIVOT_CENTER_X, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=pivot_metal,
            name="right_pivot",
        )
        louver.inertial = Inertial.from_geometry(
            Box((INNER_WIDTH, BLADE_THICKNESS, BLADE_HEIGHT)),
            mass=0.24,
            origin=Origin(),
        )
        model.articulation(
            f"frame_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=louver,
            origin=Origin(xyz=(0.0, 0.0, center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=1.2,
                lower=-LOUVER_SWING,
                upper=LOUVER_SWING,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_stile = frame.get_visual("left_stile")
    right_stile = frame.get_visual("right_stile")
    top_rail = frame.get_visual("top_rail")
    bottom_rail = frame.get_visual("bottom_rail")
    louvers = [object_model.get_part(f"louver_{index}") for index in range(1, LOUVER_COUNT + 1)]
    louver_joints = [
        object_model.get_articulation(f"frame_to_louver_{index}")
        for index in range(1, LOUVER_COUNT + 1)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "frame_and_louver_count",
        len(object_model.parts) == LOUVER_COUNT + 1 and len(louvers) == LOUVER_COUNT,
        details=f"expected 1 frame and {LOUVER_COUNT} louvers, found {len(object_model.parts)} parts",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    if frame_aabb is None:
        ctx.fail("frame_has_measurable_bounds", "frame AABB was unavailable")
    else:
        (frame_min, frame_max) = frame_aabb
        frame_dims = (
            frame_max[0] - frame_min[0],
            frame_max[1] - frame_min[1],
            frame_max[2] - frame_min[2],
        )
        ctx.check(
            "frame_proportions_match_shutter_panel",
            abs(frame_dims[0] - FRAME_WIDTH) < 0.01
            and abs(frame_dims[2] - FRAME_HEIGHT) < 0.01
            and frame_dims[2] > frame_dims[0] * 1.45,
            details=f"frame dims were {frame_dims}",
        )

    for index, (louver, joint) in enumerate(zip(louvers, louver_joints), start=1):
        blade = louver.get_visual("blade")
        left_pivot = louver.get_visual("left_pivot")
        right_pivot = louver.get_visual("right_pivot")

        ctx.expect_origin_distance(
            louver,
            frame,
            axes="xy",
            max_dist=1e-6,
            name=f"louver_{index}_axis_centered_in_opening",
        )
        ctx.expect_gap(
            louver,
            frame,
            axis="x",
            max_gap=5e-4,
            max_penetration=5e-4,
            positive_elem=left_pivot,
            negative_elem=left_stile,
            name=f"louver_{index}_left_pivot_seated",
        )
        ctx.expect_gap(
            frame,
            louver,
            axis="x",
            max_gap=5e-4,
            max_penetration=5e-4,
            positive_elem=right_stile,
            negative_elem=right_pivot,
            name=f"louver_{index}_right_pivot_seated",
        )
        with ctx.pose({joint: LOUVER_SWING}):
            ctx.expect_gap(
                louver,
                frame,
                axis="x",
                max_gap=5e-4,
                max_penetration=5e-4,
                positive_elem=left_pivot,
                negative_elem=left_stile,
                name=f"louver_{index}_left_pivot_seated_when_open",
            )
            ctx.expect_gap(
                frame,
                louver,
                axis="x",
                max_gap=5e-4,
                max_penetration=5e-4,
                positive_elem=right_stile,
                negative_elem=right_pivot,
                name=f"louver_{index}_right_pivot_seated_when_open",
            )
            ctx.expect_within(
                louver,
                frame,
                axes="x",
                margin=0.0,
                inner_elem=blade,
                name=f"louver_{index}_open_pose_blade_stays_between_stiles",
            )
            ctx.expect_overlap(
                louver,
                frame,
                axes="x",
                min_overlap=BLADE_BODY_LENGTH - 0.02,
                elem_a=blade,
                name=f"louver_{index}_open_pose_keeps_length_inside_frame_width_reference",
            )

    for lower, upper in zip(louvers[:-1], louvers[1:]):
        lower_blade = lower.get_visual("blade")
        upper_blade = upper.get_visual("blade")
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.014,
            max_gap=0.022,
            positive_elem=upper_blade,
            negative_elem=lower_blade,
            name=f"{lower.name}_to_{upper.name}_rest_gap",
        )

    bottom_blade = louvers[0].get_visual("blade")
    top_blade = louvers[-1].get_visual("blade")
    ctx.expect_gap(
        louvers[0],
        frame,
        axis="z",
        min_gap=0.025,
        max_gap=0.040,
        positive_elem=bottom_blade,
        negative_elem=bottom_rail,
        name="bottom_louver_clears_bottom_rail",
    )
    ctx.expect_gap(
        frame,
        louvers[-1],
        axis="z",
        min_gap=0.025,
        max_gap=0.040,
        positive_elem=top_rail,
        negative_elem=top_blade,
        name="top_louver_clears_top_rail",
    )

    middle_index = LOUVER_COUNT // 2
    middle_louver = louvers[middle_index]
    middle_joint = louver_joints[middle_index]
    rest_aabb = ctx.part_world_aabb(middle_louver)
    open_dims = None
    rest_dims = None
    if rest_aabb is not None:
        rest_dims = (
            rest_aabb[1][0] - rest_aabb[0][0],
            rest_aabb[1][1] - rest_aabb[0][1],
            rest_aabb[1][2] - rest_aabb[0][2],
        )
    with ctx.pose({middle_joint: LOUVER_SWING}):
        open_aabb = ctx.part_world_aabb(middle_louver)
        if open_aabb is not None:
            open_dims = (
                open_aabb[1][0] - open_aabb[0][0],
                open_aabb[1][1] - open_aabb[0][1],
                open_aabb[1][2] - open_aabb[0][2],
            )
    ctx.check(
        "middle_louver_rotates_about_long_axis",
        rest_dims is not None
        and open_dims is not None
        and open_dims[1] > rest_dims[1] + 0.03
        and open_dims[2] < rest_dims[2] - 0.008
        and abs(open_dims[0] - rest_dims[0]) < 0.002,
        details=f"rest dims={rest_dims}, open dims={open_dims}",
    )

    alternating_pose = {
        joint: (LOUVER_SWING if index % 2 else -LOUVER_SWING)
        for index, joint in enumerate(louver_joints, start=1)
    }
    with ctx.pose(alternating_pose):
        for lower, upper in zip(louvers[:-1], louvers[1:]):
            ctx.expect_gap(
                upper,
                lower,
                axis="z",
                min_gap=0.026,
                positive_elem=upper.get_visual("blade"),
                negative_elem=lower.get_visual("blade"),
                name=f"{lower.name}_to_{upper.name}_alternating_open_gap",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
