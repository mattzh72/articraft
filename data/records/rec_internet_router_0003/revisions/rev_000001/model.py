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
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.232
BODY_DEPTH = 0.156
BODY_HEIGHT = 0.026
BODY_CORNER_RADIUS = 0.018
BODY_WALL = 0.0028
BODY_BOTTOM = 0.0028
BODY_TOP = 0.0032

VENT_LENGTH = 0.116
VENT_SLOT_WIDTH = 0.0055
VENT_OFFSETS = (-0.026, -0.016, -0.006, 0.004, 0.014, 0.024)

ANTENNA_WIDTH = 0.022
ANTENNA_LENGTH = 0.094
ANTENNA_THICKNESS = 0.003
ANTENNA_CORNER_RADIUS = 0.0045
HINGE_RADIUS = 0.004
HINGE_LENGTH = 0.020
HINGE_Y = BODY_DEPTH * 0.5 - 0.005
HINGE_Z = BODY_HEIGHT + HINGE_RADIUS
HINGE_X_OFFSET = 0.072
ANTENNA_RANGE = math.radians(100.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_router_shell_mesh():
    outer = ExtrudeGeometry.from_z0(
        rounded_rect_profile(
            BODY_WIDTH,
            BODY_DEPTH,
            BODY_CORNER_RADIUS,
            corner_segments=8,
        ),
        BODY_HEIGHT,
    )
    inner = ExtrudeGeometry.from_z0(
        rounded_rect_profile(
            BODY_WIDTH - (2.0 * BODY_WALL),
            BODY_DEPTH - (2.0 * BODY_WALL),
            BODY_CORNER_RADIUS - BODY_WALL,
            corner_segments=8,
        ),
        BODY_HEIGHT - BODY_BOTTOM - BODY_TOP,
    ).translate(0.0, 0.0, BODY_BOTTOM)

    shell = boolean_difference(outer, inner)
    for offset_y in VENT_OFFSETS:
        slot = BoxGeometry((VENT_LENGTH, VENT_SLOT_WIDTH, BODY_TOP + 0.006)).translate(
            0.0,
            offset_y,
            BODY_HEIGHT - (BODY_TOP * 0.5) + 0.0015,
        )
        shell = boolean_difference(shell, slot)
    return shell


def _build_antenna_panel_mesh():
    return ExtrudeGeometry(
        rounded_rect_profile(
            ANTENNA_WIDTH,
            ANTENNA_LENGTH,
            ANTENNA_CORNER_RADIUS,
            corner_segments=8,
        ),
        ANTENNA_THICKNESS,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_router", assets=ASSETS)

    shell_black = model.material("shell_black", rgba=(0.13, 0.14, 0.15, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.11, 0.11, 0.12, 1.0))

    body = model.part("router_body")
    body.visual(
        _save_mesh("router_shell.obj", _build_router_shell_mesh()),
        material=shell_black,
        name="shell",
    )
    body.visual(
        Box((0.090, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, -0.070, 0.003)),
        material=trim_dark,
        name="front_status_window",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    antenna_mesh = _save_mesh("router_antenna_panel.obj", _build_antenna_panel_mesh())

    left_antenna = model.part("left_antenna")
    left_antenna.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_black,
        name="hinge_barrel",
    )
    left_antenna.visual(
        Box((0.016, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.013, -0.002)),
        material=antenna_black,
        name="pivot_base",
    )
    left_antenna.visual(
        antenna_mesh,
        origin=Origin(xyz=(0.0, -0.070, -0.002)),
        material=antenna_black,
        name="paddle_panel",
    )
    left_antenna.inertial = Inertial.from_geometry(
        Box((ANTENNA_WIDTH, ANTENNA_LENGTH + 0.024, 0.008)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.048, -0.001)),
    )

    right_antenna = model.part("right_antenna")
    right_antenna.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_black,
        name="hinge_barrel",
    )
    right_antenna.visual(
        Box((0.016, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.013, -0.002)),
        material=antenna_black,
        name="pivot_base",
    )
    right_antenna.visual(
        antenna_mesh,
        origin=Origin(xyz=(0.0, -0.070, -0.002)),
        material=antenna_black,
        name="paddle_panel",
    )
    right_antenna.inertial = Inertial.from_geometry(
        Box((ANTENNA_WIDTH, ANTENNA_LENGTH + 0.024, 0.008)),
        mass=0.06,
        origin=Origin(xyz=(0.0, -0.048, -0.001)),
    )

    model.articulation(
        "body_to_left_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_antenna,
        origin=Origin(xyz=(-HINGE_X_OFFSET, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.4,
            lower=0.0,
            upper=ANTENNA_RANGE,
        ),
    )
    model.articulation(
        "body_to_right_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_antenna,
        origin=Origin(xyz=(HINGE_X_OFFSET, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.4,
            lower=0.0,
            upper=ANTENNA_RANGE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("router_body")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    left_hinge = object_model.get_articulation("body_to_left_antenna")
    right_hinge = object_model.get_articulation("body_to_right_antenna")
    shell = body.get_visual("shell")
    left_base = left_antenna.get_visual("pivot_base")
    right_base = right_antenna.get_visual("pivot_base")
    left_panel = left_antenna.get_visual("paddle_panel")
    right_panel = right_antenna.get_visual("paddle_panel")

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

    ctx.expect_origin_distance(
        left_antenna,
        right_antenna,
        axes="x",
        min_dist=0.12,
        name="antennas are spaced across the router width",
    )
    ctx.expect_origin_distance(
        left_antenna,
        right_antenna,
        axes="y",
        max_dist=0.002,
        name="antenna hinges share the same back-edge line",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_antenna,
            body,
            elem_a=left_base,
            elem_b=shell,
            contact_tol=0.0005,
            name="left antenna base seats on the router top when folded",
        )
        ctx.expect_contact(
            right_antenna,
            body,
            elem_a=right_base,
            elem_b=shell,
            contact_tol=0.0005,
            name="right antenna base seats on the router top when folded",
        )
        ctx.expect_overlap(
            left_antenna,
            body,
            axes="xy",
            min_overlap=0.018,
            elem_a=left_panel,
            elem_b=shell,
            name="left antenna panel lies over the router top when folded",
        )
        ctx.expect_overlap(
            right_antenna,
            body,
            axes="xy",
            min_overlap=0.018,
            elem_a=right_panel,
            elem_b=shell,
            name="right antenna panel lies over the router top when folded",
        )
        ctx.expect_gap(
            left_antenna,
            body,
            axis="z",
            positive_elem=left_panel,
            negative_elem=shell,
            min_gap=0.0003,
            max_gap=0.004,
            name="left antenna panel stays close to the top when folded",
        )
        ctx.expect_gap(
            right_antenna,
            body,
            axis="z",
            positive_elem=right_panel,
            negative_elem=shell,
            min_gap=0.0003,
            max_gap=0.004,
            name="right antenna panel stays close to the top when folded",
        )

    with ctx.pose({left_hinge: math.radians(95.0), right_hinge: math.radians(95.0)}):
        ctx.expect_origin_gap(
            left_antenna,
            body,
            axis="z",
            min_gap=0.03,
            name="left antenna origin rises above the body when upright",
        )
        ctx.expect_origin_gap(
            right_antenna,
            body,
            axis="z",
            min_gap=0.03,
            name="right antenna origin rises above the body when upright",
        )
        ctx.expect_overlap(
            left_antenna,
            body,
            axes="x",
            min_overlap=0.018,
            elem_a=left_panel,
            elem_b=shell,
            name="left antenna remains aligned over its hinge line in x when upright",
        )
        ctx.expect_overlap(
            right_antenna,
            body,
            axes="x",
            min_overlap=0.018,
            elem_a=right_panel,
            elem_b=shell,
            name="right antenna remains aligned over its hinge line in x when upright",
        )

    with ctx.pose({left_hinge: 0.0}):
        left_folded_panel = ctx.part_element_world_aabb(left_antenna, elem="paddle_panel")
    with ctx.pose({left_hinge: math.radians(95.0)}):
        left_upright_panel = ctx.part_element_world_aabb(left_antenna, elem="paddle_panel")

    folded_height = (
        left_folded_panel[1][2] - left_folded_panel[0][2]
        if left_folded_panel is not None
        else None
    )
    upright_height = (
        left_upright_panel[1][2] - left_upright_panel[0][2]
        if left_upright_panel is not None
        else None
    )
    ctx.check(
        "left antenna rotates from flat to upright",
        folded_height is not None
        and upright_height is not None
        and folded_height < 0.008
        and upright_height > 0.080,
        details=(
            f"expected folded panel z-height < 0.008 and upright z-height > 0.080, "
            f"got folded={folded_height}, upright={upright_height}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
