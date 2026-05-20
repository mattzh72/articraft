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


OUTER_LENGTH = 0.72
MIDDLE_LENGTH = 0.62
INNER_LENGTH = 0.52

OUTER_WIDTH = 0.070
MIDDLE_WIDTH = 0.050
INNER_WIDTH = 0.032

OUTER_HEIGHT = 0.032
MIDDLE_HEIGHT = 0.024
INNER_HEIGHT = 0.017

OUTER_WALL = 0.0022
MIDDLE_WALL = 0.0020
INNER_WALL = 0.0018

OUTER_LIP = 0.0060
MIDDLE_LIP = 0.0050
INNER_LIP = 0.0042

WEAR_PAD_HEIGHT = 0.0008
WEAR_PAD_WIDTH = 0.0060
RUNNING_CLEARANCE = 0.0006
INNER_RUNNER_WIDTH = 0.0060

MIDDLE_HOME_X = 0.050
INNER_HOME_X = 0.060
OUTER_TO_MIDDLE_TRAVEL = 0.300
MIDDLE_TO_INNER_TRAVEL = 0.270

OUTER_STAGE_Z = OUTER_WALL + WEAR_PAD_HEIGHT + RUNNING_CLEARANCE
MIDDLE_STAGE_Z = MIDDLE_WALL + WEAR_PAD_HEIGHT + RUNNING_CLEARANCE

CARRIAGE_LENGTH = 0.088
CARRIAGE_WIDTH = 0.032
CARRIAGE_PLATE_THICKNESS = 0.008
CARRIAGE_PEDESTAL_LENGTH = 0.052
CARRIAGE_PEDESTAL_WIDTH = 0.022
CARRIAGE_PEDESTAL_HEIGHT = 0.008
CARRIAGE_HOME_X = INNER_LENGTH - 0.078


def _prism(
    size: tuple[float, float, float],
    *,
    x0: float,
    y_center: float,
    z0: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(size[0], size[1], size[2], centered=(False, True, False))
        .translate((x0, y_center, z0))
    )


def _u_channel_rail(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    lip: float,
    bridge_x0: float,
    bridge_length: float,
) -> cq.Workplane:
    body = _prism((length, width, wall), x0=0.0, y_center=0.0, z0=0.0)
    side_y = width / 2.0 - wall / 2.0
    body = body.union(
        _prism((length, wall, height), x0=0.0, y_center=side_y, z0=0.0)
    )
    body = body.union(
        _prism((length, wall, height), x0=0.0, y_center=-side_y, z0=0.0)
    )

    lip_y = width / 2.0 - wall - lip / 2.0
    body = body.union(
        _prism((length, lip, wall), x0=0.0, y_center=lip_y, z0=height - wall)
    )
    body = body.union(
        _prism((length, lip, wall), x0=0.0, y_center=-lip_y, z0=height - wall)
    )

    bridge_width = width - 2.0 * (wall + lip + 0.0006)
    body = body.union(
        _prism(
            (bridge_length, bridge_width, wall),
            x0=bridge_x0,
            y_center=0.0,
            z0=height - wall,
        )
    )
    return body


def _wear_strip_shape(
    *,
    length: float,
    x_margin: float,
    runner_y: float,
    runner_width: float,
    thickness: float,
    bridge_length: float,
    bridge_inset: float,
) -> cq.Workplane:
    runner_length = length - 2.0 * x_margin
    body = _prism(
        (runner_length, runner_width, thickness),
        x0=x_margin,
        y_center=runner_y,
        z0=0.0,
    )
    body = body.union(
        _prism(
            (runner_length, runner_width, thickness),
            x0=x_margin,
            y_center=-runner_y,
            z0=0.0,
        )
    )
    body = body.union(
        _prism(
            (bridge_length, 2.0 * runner_y + runner_width, thickness),
            x0=bridge_inset,
            y_center=0.0,
            z0=0.0,
        )
    )
    body = body.union(
        _prism(
            (bridge_length, 2.0 * runner_y + runner_width, thickness),
            x0=length - bridge_inset - bridge_length,
            y_center=0.0,
            z0=0.0,
        )
    )
    return body


def _add_external_stop_tabs(
    body: cq.Workplane,
    *,
    width: float,
    z0: float,
    x_positions: tuple[float, ...],
    tab_length: float,
    tab_depth: float,
    tab_height: float,
) -> cq.Workplane:
    for x0 in x_positions:
        body = body.union(
            _prism(
                (tab_length, tab_depth, tab_height),
                x0=x0,
                y_center=width / 2.0 + tab_depth / 2.0,
                z0=z0,
            )
        )
        body = body.union(
            _prism(
                (tab_length, tab_depth, tab_height),
                x0=x0,
                y_center=-(width / 2.0 + tab_depth / 2.0),
                z0=z0,
            )
        )
    return body


def _outer_rail_shape() -> cq.Workplane:
    body = _u_channel_rail(
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        lip=OUTER_LIP,
        bridge_x0=0.012,
        bridge_length=0.022,
    )
    return _add_external_stop_tabs(
        body,
        width=OUTER_WIDTH,
        z0=0.017,
        x_positions=(OUTER_LENGTH - 0.058, OUTER_LENGTH - 0.032),
        tab_length=0.010,
        tab_depth=0.0024,
        tab_height=0.006,
    )


def _middle_rail_shape() -> cq.Workplane:
    body = _u_channel_rail(
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        lip=MIDDLE_LIP,
        bridge_x0=0.014,
        bridge_length=0.020,
    )
    return _add_external_stop_tabs(
        body,
        width=MIDDLE_WIDTH,
        z0=0.013,
        x_positions=(MIDDLE_LENGTH - 0.060, MIDDLE_LENGTH - 0.036),
        tab_length=0.009,
        tab_depth=0.0022,
        tab_height=0.005,
    )


def _inner_rail_shape() -> cq.Workplane:
    top_width = INNER_WIDTH - 2.0 * 0.004
    runner_width = INNER_RUNNER_WIDTH
    side_y = top_width / 2.0 - INNER_WALL / 2.0
    runner_y = INNER_WIDTH / 2.0 - runner_width / 2.0

    body = _prism(
        (INNER_LENGTH, top_width, INNER_WALL),
        x0=0.0,
        y_center=0.0,
        z0=INNER_HEIGHT - INNER_WALL,
    )
    body = body.union(
        _prism(
            (INNER_LENGTH, INNER_WALL, INNER_HEIGHT - INNER_WALL),
            x0=0.0,
            y_center=side_y,
            z0=0.0,
        )
    )
    body = body.union(
        _prism(
            (INNER_LENGTH, INNER_WALL, INNER_HEIGHT - INNER_WALL),
            x0=0.0,
            y_center=-side_y,
            z0=0.0,
        )
    )
    body = body.union(
        _prism((INNER_LENGTH, runner_width, INNER_WALL), x0=0.0, y_center=runner_y, z0=0.0)
    )
    body = body.union(
        _prism((INNER_LENGTH, runner_width, INNER_WALL), x0=0.0, y_center=-runner_y, z0=0.0)
    )
    body = body.union(
        _prism((0.022, INNER_WIDTH - 0.004, INNER_WALL), x0=INNER_LENGTH - 0.032, y_center=0.0, z0=INNER_HEIGHT - INNER_WALL)
    )
    return _add_external_stop_tabs(
        body,
        width=INNER_WIDTH,
        z0=0.008,
        x_positions=(0.022, 0.044),
        tab_length=0.008,
        tab_depth=0.0020,
        tab_height=0.004,
    )


def _carriage_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_LENGTH,
            CARRIAGE_WIDTH,
            CARRIAGE_PLATE_THICKNESS,
            centered=(False, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_PEDESTAL_HEIGHT))
    )
    body = body.union(
        _prism(
            (
                CARRIAGE_PEDESTAL_LENGTH,
                CARRIAGE_PEDESTAL_WIDTH,
                CARRIAGE_PEDESTAL_HEIGHT,
            ),
            x0=0.020,
            y_center=0.0,
            z0=0.0,
        )
    )
    body = body.union(
        _prism((0.016, 0.018, 0.006), x0=CARRIAGE_LENGTH - 0.022, y_center=0.0, z0=0.006)
    )
    return (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(-0.020, 0.0), (0.020, 0.0)])
        .slot2D(0.018, 0.006)
        .cutBlind(-CARRIAGE_PLATE_THICKNESS)
    )


def _outer_strip_shape() -> cq.Workplane:
    runner_y = OUTER_WIDTH / 2.0 - OUTER_WALL - OUTER_LIP - WEAR_PAD_WIDTH / 2.0 - 0.0010
    return _wear_strip_shape(
        length=OUTER_LENGTH,
        x_margin=0.018,
        runner_y=runner_y,
        runner_width=WEAR_PAD_WIDTH,
        thickness=OUTER_STAGE_Z - OUTER_WALL,
        bridge_length=0.016,
        bridge_inset=0.014,
    )


def _middle_strip_shape() -> cq.Workplane:
    runner_y = INNER_WIDTH / 2.0 - INNER_RUNNER_WIDTH / 2.0
    return _wear_strip_shape(
        length=MIDDLE_LENGTH,
        x_margin=0.016,
        runner_y=runner_y,
        runner_width=INNER_RUNNER_WIDTH,
        thickness=MIDDLE_STAGE_Z - MIDDLE_WALL,
        bridge_length=0.014,
        bridge_inset=0.012,
    )


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material: str,
    box_size: tuple[float, float, float],
    box_center: tuple[float, float, float],
    mass: float,
    visual_name: str,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        material=material,
        name=visual_name,
    )
    part.inertial = Inertial.from_geometry(
        Box(box_size),
        mass=mass,
        origin=Origin(xyz=box_center),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_extension_rail")

    model.material("outer_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("middle_steel", rgba=(0.48, 0.50, 0.53, 1.0))
    model.material("inner_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_finish", rgba=(0.82, 0.83, 0.85, 1.0))
    model.material("wear_polymer", rgba=(0.90, 0.91, 0.88, 1.0))

    outer = _add_mesh_part(
        model,
        name="outer_member",
        shape=_outer_rail_shape(),
        mesh_name="outer_member",
        material="outer_steel",
        box_size=(OUTER_LENGTH, OUTER_WIDTH + 0.005, OUTER_HEIGHT),
        box_center=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0),
        mass=1.10,
        visual_name="outer_body",
    )
    middle = _add_mesh_part(
        model,
        name="middle_member",
        shape=_middle_rail_shape(),
        mesh_name="middle_member",
        material="middle_steel",
        box_size=(MIDDLE_LENGTH, MIDDLE_WIDTH + 0.005, MIDDLE_HEIGHT),
        box_center=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0),
        mass=0.78,
        visual_name="middle_body",
    )
    outer_guide = _add_mesh_part(
        model,
        name="outer_guide_strip",
        shape=_outer_strip_shape(),
        mesh_name="outer_guide_strip",
        material="wear_polymer",
        box_size=(OUTER_LENGTH, 0.052, OUTER_STAGE_Z - OUTER_WALL),
        box_center=(OUTER_LENGTH / 2.0, 0.0, (OUTER_STAGE_Z - OUTER_WALL) / 2.0),
        mass=0.05,
        visual_name="outer_guide_body",
    )
    inner = _add_mesh_part(
        model,
        name="inner_member",
        shape=_inner_rail_shape(),
        mesh_name="inner_member",
        material="inner_steel",
        box_size=(INNER_LENGTH, INNER_WIDTH + 0.004, INNER_HEIGHT),
        box_center=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0),
        mass=0.52,
        visual_name="inner_body",
    )
    middle_guide = _add_mesh_part(
        model,
        name="middle_guide_strip",
        shape=_middle_strip_shape(),
        mesh_name="middle_guide_strip",
        material="wear_polymer",
        box_size=(MIDDLE_LENGTH, INNER_WIDTH, MIDDLE_STAGE_Z - MIDDLE_WALL),
        box_center=(MIDDLE_LENGTH / 2.0, 0.0, (MIDDLE_STAGE_Z - MIDDLE_WALL) / 2.0),
        mass=0.04,
        visual_name="middle_guide_body",
    )
    carriage = _add_mesh_part(
        model,
        name="top_carriage",
        shape=_carriage_shape(),
        mesh_name="top_carriage",
        material="carriage_finish",
        box_size=(
            CARRIAGE_LENGTH,
            CARRIAGE_WIDTH,
            CARRIAGE_PEDESTAL_HEIGHT + CARRIAGE_PLATE_THICKNESS,
        ),
        box_center=(
            CARRIAGE_LENGTH / 2.0,
            0.0,
            (CARRIAGE_PEDESTAL_HEIGHT + CARRIAGE_PLATE_THICKNESS) / 2.0,
        ),
        mass=0.24,
        visual_name="carriage_body",
    )

    model.articulation(
        "outer_to_outer_guide",
        ArticulationType.FIXED,
        parent=outer,
        child=outer_guide,
        origin=Origin(xyz=(0.0, 0.0, OUTER_WALL)),
    )
    outer_to_middle = model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(MIDDLE_HOME_X, 0.0, OUTER_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=0.55,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_middle_guide",
        ArticulationType.FIXED,
        parent=middle,
        child=middle_guide,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_WALL)),
    )
    middle_to_inner = model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(INNER_HOME_X, 0.0, MIDDLE_STAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.60,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_carriage",
        ArticulationType.FIXED,
        parent=inner,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, 0.0, INNER_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_member")
    middle = object_model.get_part("middle_member")
    outer_guide = object_model.get_part("outer_guide_strip")
    inner = object_model.get_part("inner_member")
    middle_guide = object_model.get_part("middle_guide_strip")
    carriage = object_model.get_part("top_carriage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "serial slide joints are prismatic on x",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC
        and tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0)
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        details="Both telescoping stages should translate along the shared x-axis.",
    )

    ctx.expect_contact(outer, outer_guide, name="outer guide strip is seated on outer rail")
    ctx.expect_contact(
        outer_guide,
        middle,
        name="middle rail bears on outer guide strip when retracted",
    )
    ctx.expect_contact(
        middle,
        middle_guide,
        name="middle guide strip is seated on middle rail",
    )
    ctx.expect_contact(
        middle_guide,
        inner,
        name="inner rail bears on middle guide strip when retracted",
    )
    ctx.expect_contact(inner, carriage, name="carriage is mounted to inner rail")
    ctx.expect_origin_gap(
        middle,
        outer,
        axis="z",
        min_gap=0.003,
        max_gap=0.005,
        name="middle rides slightly above outer wear tracks",
    )
    ctx.expect_origin_gap(
        inner,
        middle,
        axis="z",
        min_gap=0.003,
        max_gap=0.005,
        name="inner rides slightly above middle wear tracks",
    )
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0,
        name="middle stays nested within outer channel section",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.0,
        name="inner stays nested within middle channel section",
    )
    ctx.expect_overlap(
        outer,
        middle,
        axes="x",
        min_overlap=0.60,
        name="outer and middle keep deep retracted engagement",
    )
    ctx.expect_overlap(
        middle,
        inner,
        axes="x",
        min_overlap=0.50,
        name="middle and inner keep deep retracted engagement",
    )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no collisions at full extension")
        ctx.expect_contact(
            outer_guide,
            middle,
            name="middle rail stays supported by outer guide at full extension",
        )
        ctx.expect_contact(
            middle_guide,
            inner,
            name="inner rail stays supported by middle guide at full extension",
        )
        ctx.expect_overlap(
            outer,
            middle,
            axes="x",
            min_overlap=0.35,
            name="outer-middle keep engagement at full extension",
        )
        ctx.expect_overlap(
            middle,
            inner,
            axes="x",
            min_overlap=0.28,
            name="middle-inner keep engagement at full extension",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="middle remains captured in outer section at full extension",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="inner remains captured in middle section at full extension",
        )
        ctx.expect_contact(inner, carriage, name="carriage stays seated at full extension")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
