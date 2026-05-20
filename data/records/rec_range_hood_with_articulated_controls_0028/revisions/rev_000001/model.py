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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

HOOD_WIDTH = 0.90
HOOD_DEPTH = 0.52
FRONT_Y = HOOD_DEPTH / 2.0
BACK_Y = -HOOD_DEPTH / 2.0

VALANCE_HEIGHT = 0.12
CANOPY_TOP_HEIGHT = 0.23
PANEL_THICKNESS = 0.015
SIDE_PANEL_THICKNESS = 0.015
FRONT_PANEL_DEPTH = 0.022
REAR_PANEL_DEPTH = 0.018
ROOF_START_Y = FRONT_Y - FRONT_PANEL_DEPTH
TOP_FRONT_Y = -0.040

CHIMNEY_OUTER_WIDTH = 0.30
CHIMNEY_OUTER_DEPTH = 0.22
CHIMNEY_WALL = 0.012
CHIMNEY_HEIGHT = 0.70
CHIMNEY_CENTER_Y = -0.150

KNOB_RADIUS = 0.026
KNOB_DEPTH = 0.032
INDICATOR_SIZE = (0.004, 0.003, 0.010)
KNOB_X = 0.285
KNOB_Z_UPPER = 0.092
KNOB_Z_LOWER = 0.028


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _side_panel_section(x_pos: float) -> list[tuple[float, float, float]]:
    return [
        (x_pos, FRONT_Y, 0.0),
        (x_pos, FRONT_Y, VALANCE_HEIGHT),
        (x_pos, ROOF_START_Y, VALANCE_HEIGHT),
        (x_pos, TOP_FRONT_Y, CANOPY_TOP_HEIGHT),
        (x_pos, BACK_Y, CANOPY_TOP_HEIGHT),
        (x_pos, BACK_Y, 0.0),
    ]


def _indicator_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.20, 1.0))
    knob_finish = model.material("knob_finish", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_indicator = model.material("knob_indicator", rgba=(0.88, 0.88, 0.90, 1.0))

    hood_body = model.part("hood_body")
    inner_width = HOOD_WIDTH - (2.0 * SIDE_PANEL_THICKNESS)

    left_side_mesh = _save_mesh(
        "range_hood_left_side.obj",
        section_loft(
            [
                _side_panel_section((-HOOD_WIDTH * 0.5)),
                _side_panel_section((-HOOD_WIDTH * 0.5) + SIDE_PANEL_THICKNESS),
            ]
        ),
    )
    right_side_mesh = _save_mesh(
        "range_hood_right_side.obj",
        section_loft(
            [
                _side_panel_section((HOOD_WIDTH * 0.5) - SIDE_PANEL_THICKNESS),
                _side_panel_section(HOOD_WIDTH * 0.5),
            ]
        ),
    )

    hood_body.visual(left_side_mesh, material=stainless, name="left_side_shell")
    hood_body.visual(right_side_mesh, material=stainless, name="right_side_shell")
    hood_body.visual(
        Box((inner_width, FRONT_PANEL_DEPTH, VALANCE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_Y - (FRONT_PANEL_DEPTH * 0.5),
                VALANCE_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="front_valance",
    )

    roof_run = ROOF_START_Y - TOP_FRONT_Y
    roof_rise = CANOPY_TOP_HEIGHT - VALANCE_HEIGHT
    roof_length = math.hypot(roof_run, roof_rise)
    roof_angle = -math.atan2(roof_rise, roof_run)
    hood_body.visual(
        Box((inner_width, roof_length, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (ROOF_START_Y + TOP_FRONT_Y) * 0.5,
                (VALANCE_HEIGHT + CANOPY_TOP_HEIGHT) * 0.5,
            ),
            rpy=(roof_angle, 0.0, 0.0),
        ),
        material=stainless,
        name="sloped_roof",
    )
    hood_body.visual(
        Box((inner_width, abs(BACK_Y - TOP_FRONT_Y), PANEL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                (BACK_Y + TOP_FRONT_Y) * 0.5,
                CANOPY_TOP_HEIGHT - (PANEL_THICKNESS * 0.5),
            )
        ),
        material=stainless,
        name="top_deck",
    )
    hood_body.visual(
        Box((inner_width, REAR_PANEL_DEPTH, CANOPY_TOP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_Y + (REAR_PANEL_DEPTH * 0.5),
                CANOPY_TOP_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="rear_panel",
    )

    chimney_outer_half_w = CHIMNEY_OUTER_WIDTH * 0.5
    chimney_outer_half_d = CHIMNEY_OUTER_DEPTH * 0.5
    chimney_inner_half_w = chimney_outer_half_w - CHIMNEY_WALL
    chimney_inner_half_d = chimney_outer_half_d - CHIMNEY_WALL
    chimney_geom = ExtrudeWithHolesGeometry(
        [
            (-chimney_outer_half_w, -chimney_outer_half_d),
            (chimney_outer_half_w, -chimney_outer_half_d),
            (chimney_outer_half_w, chimney_outer_half_d),
            (-chimney_outer_half_w, chimney_outer_half_d),
        ],
        [
            [
                (-chimney_inner_half_w, -chimney_inner_half_d),
                (-chimney_inner_half_w, chimney_inner_half_d),
                (chimney_inner_half_w, chimney_inner_half_d),
                (chimney_inner_half_w, -chimney_inner_half_d),
            ]
        ],
        CHIMNEY_HEIGHT,
        cap=True,
        center=False,
        closed=True,
    )
    hood_body.visual(
        _save_mesh("range_hood_chimney.obj", chimney_geom),
        origin=Origin(xyz=(0.0, CHIMNEY_CENTER_Y, CANOPY_TOP_HEIGHT)),
        material=stainless,
        name="chimney_chase",
    )
    hood_body.inertial = Inertial.from_geometry(
        Box((HOOD_WIDTH, HOOD_DEPTH, CANOPY_TOP_HEIGHT + CHIMNEY_HEIGHT)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, (CANOPY_TOP_HEIGHT + CHIMNEY_HEIGHT) * 0.5)),
    )

    def add_knob(
        part_name: str,
        joint_name: str,
        *,
        x_pos: float,
        z_pos: float,
    ) -> None:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
            origin=Origin(xyz=(0.0, KNOB_DEPTH * 0.5, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=knob_finish,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(0.0, KNOB_DEPTH - 0.003, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=dark_trim,
            name="knob_face",
        )
        knob.visual(
            Box(INDICATOR_SIZE),
            origin=Origin(
                xyz=(
                    0.0,
                    KNOB_DEPTH + (INDICATOR_SIZE[1] * 0.5),
                    KNOB_RADIUS - 0.007,
                )
            ),
            material=knob_indicator,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.060, KNOB_DEPTH + 0.010, 0.060)),
            mass=0.12,
            origin=Origin(xyz=(0.0, (KNOB_DEPTH + 0.010) * 0.5, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x_pos, FRONT_Y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=10.0),
        )

    add_knob(
        "knob_upper_left",
        "hood_to_knob_upper_left",
        x_pos=-KNOB_X,
        z_pos=KNOB_Z_UPPER,
    )
    add_knob(
        "knob_upper_right",
        "hood_to_knob_upper_right",
        x_pos=KNOB_X,
        z_pos=KNOB_Z_UPPER,
    )
    add_knob(
        "knob_lower_left",
        "hood_to_knob_lower_left",
        x_pos=-KNOB_X,
        z_pos=KNOB_Z_LOWER,
    )
    add_knob(
        "knob_lower_right",
        "hood_to_knob_lower_right",
        x_pos=KNOB_X,
        z_pos=KNOB_Z_LOWER,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    hood_body = object_model.get_part("hood_body")
    front_valance = hood_body.get_visual("front_valance")
    chimney_chase = hood_body.get_visual("chimney_chase")

    knob_names = (
        "knob_upper_left",
        "knob_upper_right",
        "knob_lower_left",
        "knob_lower_right",
    )
    joint_names = (
        "hood_to_knob_upper_left",
        "hood_to_knob_upper_right",
        "hood_to_knob_lower_left",
        "hood_to_knob_lower_right",
    )
    knobs = [object_model.get_part(name) for name in knob_names]
    knob_joints = [object_model.get_articulation(name) for name in joint_names]

    ctx.check(
        "only_four_knob_articulations",
        len(object_model.articulations) == 4,
        f"Expected exactly four articulations, found {len(object_model.articulations)}.",
    )

    hood_aabb = ctx.part_world_aabb(hood_body)
    assert hood_aabb is not None
    hood_width = hood_aabb[1][0] - hood_aabb[0][0]
    hood_depth = hood_aabb[1][1] - hood_aabb[0][1]
    hood_height = hood_aabb[1][2] - hood_aabb[0][2]
    ctx.check(
        "realistic_range_hood_size",
        0.80 <= hood_width <= 1.05 and 0.45 <= hood_depth <= 0.65 and 0.80 <= hood_height <= 1.05,
        f"Unexpected hood size {(hood_width, hood_depth, hood_height)}.",
    )

    front_aabb = ctx.part_element_world_aabb(hood_body, elem=front_valance)
    chimney_aabb = ctx.part_element_world_aabb(hood_body, elem=chimney_chase)
    assert front_aabb is not None
    assert chimney_aabb is not None
    front_width = front_aabb[1][0] - front_aabb[0][0]
    chimney_width = chimney_aabb[1][0] - chimney_aabb[0][0]
    chimney_center_x = (chimney_aabb[0][0] + chimney_aabb[1][0]) * 0.5
    ctx.check(
        "broad_front_valance",
        front_width >= 0.82 and front_width >= (2.5 * chimney_width),
        f"Front valance width {front_width:.3f} is not broad relative to chimney width {chimney_width:.3f}.",
    )
    ctx.check(
        "centered_chimney_chase",
        abs(chimney_center_x) <= 0.01,
        f"Chimney center x-offset was {chimney_center_x:.4f} m.",
    )

    knob_positions = []
    for knob, joint in zip(knobs, knob_joints):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            f"{joint.name} was not authored as an unbounded continuous rotary control.",
        )
        ctx.check(
            f"{joint.name}_front_to_back_axis",
            abs(joint.axis[0]) < 1e-9 and abs(joint.axis[1] - 1.0) < 1e-9 and abs(joint.axis[2]) < 1e-9,
            f"{joint.name} axis {joint.axis} was not aligned to +Y front-to-back rotation.",
        )
        ctx.expect_gap(
            knob,
            hood_body,
            axis="y",
            max_gap=0.0005,
            max_penetration=0.0,
            negative_elem=front_valance,
            name=f"{knob.name}_flush_on_front_panel",
        )
        ctx.expect_overlap(
            knob,
            hood_body,
            axes="xz",
            min_overlap=0.02,
            elem_b=front_valance,
            name=f"{knob.name}_footprint_on_front_panel",
        )
        knob_pos = ctx.part_world_position(knob)
        assert knob_pos is not None
        knob_positions.append(knob_pos)

        indicator = knob.get_visual("indicator")
        rest_indicator_aabb = ctx.part_element_world_aabb(knob, elem=indicator)
        assert rest_indicator_aabb is not None
        rest_center = _indicator_center(rest_indicator_aabb)
        with ctx.pose({joint: math.pi * 0.5}):
            turned_indicator_aabb = ctx.part_element_world_aabb(knob, elem=indicator)
            assert turned_indicator_aabb is not None
            turned_center = _indicator_center(turned_indicator_aabb)
            ctx.expect_gap(
                knob,
                hood_body,
                axis="y",
                max_gap=0.0005,
                max_penetration=0.0,
                negative_elem=front_valance,
                name=f"{joint.name}_quarter_turn_stays_mounted",
            )
            ctx.check(
                f"{joint.name}_quarter_turn_moves_indicator",
                abs(turned_center[0] - rest_center[0]) >= 0.012 and abs(turned_center[2] - rest_center[2]) >= 0.012,
                f"{joint.name} indicator did not move enough between rest {rest_center} and quarter-turn {turned_center}.",
            )

    xs = sorted(position[0] for position in knob_positions)
    zs = sorted(position[2] for position in knob_positions)
    ctx.check(
        "knobs_leave_middle_open",
        xs[1] <= -0.20 and xs[2] >= 0.20 and zs[1] <= 0.05 and zs[2] >= 0.07,
        f"Knob positions did not frame an open center: xs={xs}, zs={zs}.",
    )

    with ctx.pose({joint: math.pi * 0.5 for joint in knob_joints}):
        ctx.fail_if_parts_overlap_in_current_pose(name="knobs_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="knobs_quarter_turn_no_floating")

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
