from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

CABINET_WIDTH = 0.84
CABINET_DEPTH = 0.48
CABINET_HEIGHT = 0.92
SIDE_THICKNESS = 0.018
TOP_THICKNESS = 0.024
BOTTOM_THICKNESS = 0.022
BACK_THICKNESS = 0.012
PLINTH_HEIGHT = 0.055
PLINTH_SETBACK = 0.050

INNER_WIDTH = CABINET_WIDTH - 2.0 * SIDE_THICKNESS
LEFT_INNER_FACE_X = -CABINET_WIDTH / 2.0 + SIDE_THICKNESS
RIGHT_INNER_FACE_X = CABINET_WIDTH / 2.0 - SIDE_THICKNESS

RUNNER_THICKNESS = 0.010
RUNNER_HEIGHT = 0.018
RUNNER_LENGTH = 0.290
DRAWER_SIDE_CLEARANCE = 0.004

DRAWER_BOX_DEPTH = 0.360
DRAWER_FRONT_THICKNESS = 0.022
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BACK_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.008
DRAWER_FRONT_REVEAL_X = 0.005
DRAWER_BOX_OUTER_WIDTH = INNER_WIDTH - 2.0 * (RUNNER_THICKNESS + DRAWER_SIDE_CLEARANCE)
DRAWER_FRONT_WIDTH = INNER_WIDTH - 2.0 * DRAWER_FRONT_REVEAL_X
DRAWER_ORIGIN_X = LEFT_INNER_FACE_X + RUNNER_THICKNESS + DRAWER_SIDE_CLEARANCE
DRAWER_BOX_CENTER_Y = CABINET_DEPTH / 2.0 - 0.004 - DRAWER_FRONT_THICKNESS - DRAWER_BOX_DEPTH / 2.0

PULL_RADIUS = 0.005
PULL_STANDOFF = 0.018
PULL_CENTER_OFFSET_Y = DRAWER_BOX_DEPTH / 2.0 + DRAWER_FRONT_THICKNESS - 0.001 + PULL_STANDOFF

DRAWER_SPECS = [
    {
        "name": "drawer_top",
        "front_height": 0.125,
        "center_z": 0.8175,
        "travel": 0.260,
        "pull_span": 0.105,
        "double_pull": False,
    },
    {
        "name": "drawer_upper",
        "front_height": 0.155,
        "center_z": 0.6695,
        "travel": 0.270,
        "pull_span": 0.125,
        "double_pull": False,
    },
    {
        "name": "drawer_lower",
        "front_height": 0.215,
        "center_z": 0.4765,
        "travel": 0.290,
        "pull_span": 0.155,
        "double_pull": False,
    },
    {
        "name": "drawer_bottom",
        "front_height": 0.248,
        "center_z": 0.2370,
        "travel": 0.310,
        "pull_span": 0.115,
        "double_pull": True,
    },
]


def _drawer_side_height(front_height: float) -> float:
    return front_height - 0.030


def _drawer_joint_name(drawer_name: str) -> str:
    return f"carcass_to_{drawer_name}"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drawer_cabinet", assets=ASSETS)

    oak = Material(name="oak_veneer", rgba=(0.55, 0.43, 0.31, 1.0))
    birch = Material(name="birch_plywood", rgba=(0.76, 0.69, 0.56, 1.0))
    back_panel = Material(name="back_panel", rgba=(0.46, 0.37, 0.28, 1.0))
    graphite = Material(name="graphite_runner", rgba=(0.14, 0.14, 0.15, 1.0))
    steel = Material(name="brushed_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    model.materials.extend([oak, birch, back_panel, graphite, steel])

    carcass = model.part("carcass")
    side_center_z = CABINET_HEIGHT / 2.0
    panel_depth = CABINET_DEPTH - BACK_THICKNESS
    panel_center_y = BACK_THICKNESS / 2.0

    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(-CABINET_WIDTH / 2.0 + SIDE_THICKNESS / 2.0, 0.0, side_center_z)),
        material=oak,
        name="left_side",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, CABINET_HEIGHT)),
        origin=Origin(xyz=(CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0, 0.0, side_center_z)),
        material=oak,
        name="right_side",
    )
    carcass.visual(
        Box((INNER_WIDTH, panel_depth, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, panel_center_y, PLINTH_HEIGHT + BOTTOM_THICKNESS / 2.0)),
        material=oak,
        name="bottom_panel",
    )
    carcass.visual(
        Box((INNER_WIDTH, panel_depth, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, panel_center_y, CABINET_HEIGHT - TOP_THICKNESS / 2.0)),
        material=oak,
        name="top_panel",
    )
    carcass.visual(
        Box((INNER_WIDTH, BACK_THICKNESS, CABINET_HEIGHT - PLINTH_HEIGHT - TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH / 2.0 + BACK_THICKNESS / 2.0,
                PLINTH_HEIGHT + (CABINET_HEIGHT - PLINTH_HEIGHT - TOP_THICKNESS) / 2.0,
            )
        ),
        material=back_panel,
        name="rear_panel",
    )
    carcass.visual(
        Box((INNER_WIDTH, SIDE_THICKNESS, PLINTH_HEIGHT - 0.006)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_DEPTH / 2.0 - PLINTH_SETBACK - SIDE_THICKNESS / 2.0,
                (PLINTH_HEIGHT - 0.006) / 2.0,
            )
        ),
        material=back_panel,
        name="plinth_front",
    )
    for side_name, x_sign in (("plinth_left", -1.0), ("plinth_right", 1.0)):
        carcass.visual(
            Box((SIDE_THICKNESS, PLINTH_SETBACK, PLINTH_HEIGHT - 0.006)),
            origin=Origin(
                xyz=(
                    x_sign * (CABINET_WIDTH / 2.0 - SIDE_THICKNESS / 2.0),
                    CABINET_DEPTH / 2.0 - PLINTH_SETBACK / 2.0,
                    (PLINTH_HEIGHT - 0.006) / 2.0,
                )
            ),
            material=back_panel,
            name=side_name,
        )

    for spec in DRAWER_SPECS:
        z = spec["center_z"]
        carcass.visual(
            Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
            origin=Origin(
                xyz=(
                    LEFT_INNER_FACE_X + RUNNER_THICKNESS / 2.0,
                    DRAWER_BOX_CENTER_Y,
                    z,
                )
            ),
            material=graphite,
            name=f"{spec['name']}_runner_left",
        )
        carcass.visual(
            Box((RUNNER_THICKNESS, RUNNER_LENGTH, RUNNER_HEIGHT)),
            origin=Origin(
                xyz=(
                    RIGHT_INNER_FACE_X - RUNNER_THICKNESS / 2.0,
                    DRAWER_BOX_CENTER_Y,
                    z,
                )
            ),
            material=graphite,
            name=f"{spec['name']}_runner_right",
        )

    gap_pairs = list(zip(DRAWER_SPECS[:-1], DRAWER_SPECS[1:]))
    for upper, lower in gap_pairs:
        upper_bottom = upper["center_z"] - _drawer_side_height(upper["front_height"]) / 2.0
        lower_top = lower["center_z"] + _drawer_side_height(lower["front_height"]) / 2.0
        gap_height = upper_bottom - lower_top
        rail_height = min(0.028, gap_height - 0.006)
        rail_center_z = (upper_bottom + lower_top) / 2.0
        carcass.visual(
            Box((INNER_WIDTH, 0.050, rail_height)),
            origin=Origin(xyz=(0.0, 0.095, rail_center_z)),
            material=back_panel,
            name=f"{upper['name']}_web_front",
        )
        carcass.visual(
            Box((INNER_WIDTH, 0.034, rail_height)),
            origin=Origin(xyz=(0.0, -0.160, rail_center_z)),
            material=back_panel,
            name=f"{upper['name']}_web_back",
        )

    carcass.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT / 2.0)),
    )

    def add_bar_pull(
        part,
        *,
        x_center: float,
        z_center: float,
        span: float,
    ) -> None:
        post_y = PULL_CENTER_OFFSET_Y - PULL_STANDOFF / 2.0
        bar_y = PULL_CENTER_OFFSET_Y
        for offset in (-span / 2.0, span / 2.0):
            part.visual(
                Cylinder(radius=PULL_RADIUS * 0.72, length=PULL_STANDOFF),
                origin=Origin(
                    xyz=(x_center + offset, post_y, z_center),
                    rpy=(-pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
            )
        part.visual(
            Cylinder(radius=PULL_RADIUS, length=span),
            origin=Origin(
                xyz=(x_center, bar_y, z_center),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
        )

    for spec in DRAWER_SPECS:
        front_height = spec["front_height"]
        side_height = _drawer_side_height(front_height)
        drawer = model.part(spec["name"])
        front_panel_center_y = DRAWER_BOX_DEPTH / 2.0 + DRAWER_FRONT_THICKNESS / 2.0 - 0.001
        bottom_center_z = -side_height / 2.0 + DRAWER_BOTTOM_THICKNESS / 2.0 + 0.008
        front_center_x = DRAWER_BOX_OUTER_WIDTH / 2.0

        drawer.visual(
            Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, front_height)),
            origin=Origin(xyz=(front_center_x, front_panel_center_y, 0.0)),
            material=oak,
            name="front",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, side_height)),
            origin=Origin(xyz=(DRAWER_SIDE_THICKNESS / 2.0, 0.0, 0.0)),
            material=birch,
            name="side_left",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, side_height)),
            origin=Origin(xyz=(DRAWER_BOX_OUTER_WIDTH - DRAWER_SIDE_THICKNESS / 2.0, 0.0, 0.0)),
            material=birch,
            name="side_right",
        )
        drawer.visual(
            Box(
                (
                    DRAWER_BOX_OUTER_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                    DRAWER_BACK_THICKNESS,
                    side_height,
                )
            ),
            origin=Origin(
                xyz=(
                    front_center_x,
                    -DRAWER_BOX_DEPTH / 2.0 + DRAWER_BACK_THICKNESS / 2.0,
                    0.0,
                )
            ),
            material=birch,
            name="back",
        )
        drawer.visual(
            Box(
                (
                    DRAWER_BOX_OUTER_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                    DRAWER_BOX_DEPTH - 0.020,
                    DRAWER_BOTTOM_THICKNESS,
                )
            ),
            origin=Origin(xyz=(front_center_x, -0.004, bottom_center_z)),
            material=birch,
            name="bottom",
        )
        drawer.visual(
            Box((0.008, DRAWER_BOX_DEPTH * 0.72, 0.010)),
            origin=Origin(xyz=(DRAWER_SIDE_THICKNESS + 0.004, -0.010, -side_height / 2.0 + 0.018)),
            material=graphite,
            name="slide_strip_left",
        )
        drawer.visual(
            Box((0.008, DRAWER_BOX_DEPTH * 0.72, 0.010)),
            origin=Origin(
                xyz=(
                    DRAWER_BOX_OUTER_WIDTH - DRAWER_SIDE_THICKNESS - 0.004,
                    -0.010,
                    -side_height / 2.0 + 0.018,
                )
            ),
            material=graphite,
            name="slide_strip_right",
        )

        if spec["double_pull"]:
            handle_offset = DRAWER_FRONT_WIDTH * 0.18
            add_bar_pull(
                drawer,
                x_center=front_center_x - handle_offset,
                z_center=0.0,
                span=spec["pull_span"],
            )
            add_bar_pull(
                drawer,
                x_center=front_center_x + handle_offset,
                z_center=0.0,
                span=spec["pull_span"],
            )
        else:
            add_bar_pull(
                drawer,
                x_center=front_center_x,
                z_center=0.0,
                span=spec["pull_span"],
            )

        drawer.inertial = Inertial.from_geometry(
            Box((DRAWER_FRONT_WIDTH, DRAWER_BOX_DEPTH + DRAWER_FRONT_THICKNESS, front_height)),
            mass=2.8 + front_height * 10.0,
            origin=Origin(xyz=(front_center_x, DRAWER_FRONT_THICKNESS / 2.0, 0.0)),
        )

        model.articulation(
            _drawer_joint_name(spec["name"]),
            ArticulationType.PRISMATIC,
            parent="carcass",
            child=spec["name"],
            origin=Origin(xyz=(DRAWER_ORIGIN_X, DRAWER_BOX_CENTER_Y, spec["center_z"])),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.40,
                lower=0.0,
                upper=spec["travel"],
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    drawer_names = [spec["name"] for spec in DRAWER_SPECS]
    closed_y = {name: ctx.part_world_position(name)[1] for name in drawer_names}

    for spec in DRAWER_SPECS:
        name = spec["name"]
        joint = _drawer_joint_name(name)
        ctx.expect_aabb_overlap(name, "carcass", axes="xy", min_overlap=0.120)
        ctx.expect_joint_motion_axis(
            joint,
            name,
            world_axis="y",
            direction="positive",
            min_delta=0.040,
        )

    ctx.expect_aabb_overlap("drawer_top", "drawer_upper", axes="xy", min_overlap=0.120)
    ctx.expect_aabb_overlap("drawer_upper", "drawer_lower", axes="xy", min_overlap=0.120)
    ctx.expect_aabb_overlap("drawer_lower", "drawer_bottom", axes="xy", min_overlap=0.120)

    ctx.expect_aabb_gap("drawer_top", "drawer_upper", axis="z", max_gap=0.012, max_penetration=0.0)
    ctx.expect_aabb_gap("drawer_upper", "drawer_lower", axis="z", max_gap=0.012, max_penetration=0.0)
    ctx.expect_aabb_gap("drawer_lower", "drawer_bottom", axis="z", max_gap=0.012, max_penetration=0.0)

    with ctx.pose(carcass_to_drawer_top=0.260):
        ctx.expect_aabb_overlap("drawer_top", "carcass", axes="xy", min_overlap=0.120)
        ctx.expect_aabb_gap("drawer_top", "drawer_upper", axis="z", max_gap=0.012, max_penetration=0.0)
        assert ctx.part_world_position("drawer_top")[1] > closed_y["drawer_top"] + 0.240
        assert abs(ctx.part_world_position("drawer_bottom")[1] - closed_y["drawer_bottom"]) < 1e-4

    with ctx.pose(carcass_to_drawer_upper=0.270):
        ctx.expect_aabb_overlap("drawer_upper", "carcass", axes="xy", min_overlap=0.110)
        ctx.expect_aabb_gap("drawer_upper", "drawer_lower", axis="z", max_gap=0.012, max_penetration=0.0)
        assert ctx.part_world_position("drawer_upper")[1] > closed_y["drawer_upper"] + 0.250

    with ctx.pose(carcass_to_drawer_lower=0.290):
        ctx.expect_aabb_overlap("drawer_lower", "carcass", axes="xy", min_overlap=0.090)
        ctx.expect_aabb_gap("drawer_lower", "drawer_bottom", axis="z", max_gap=0.012, max_penetration=0.0)
        assert ctx.part_world_position("drawer_lower")[1] > closed_y["drawer_lower"] + 0.270

    with ctx.pose(carcass_to_drawer_bottom=0.310):
        ctx.expect_aabb_overlap("drawer_bottom", "carcass", axes="xy", min_overlap=0.070)
        ctx.expect_aabb_gap("drawer_lower", "drawer_bottom", axis="z", max_gap=0.012, max_penetration=0.0)
        assert ctx.part_world_position("drawer_bottom")[1] > closed_y["drawer_bottom"] + 0.290
        assert abs(ctx.part_world_position("drawer_top")[1] - closed_y["drawer_top"]) < 1e-4

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
