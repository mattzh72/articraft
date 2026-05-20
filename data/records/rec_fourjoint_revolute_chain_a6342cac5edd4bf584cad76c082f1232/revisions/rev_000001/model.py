from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LINK_PITCH = 0.088
LINK_BAR_WIDTH = 0.010
LINK_BAR_THICKNESS = 0.006
HINGE_BLOCK_HEIGHT = 0.020
HINGE_OUTER_WIDTH = 0.022
HINGE_SLOT_WIDTH = 0.009
HINGE_BARREL_WIDTH = HINGE_SLOT_WIDTH
HINGE_HOLE_RADIUS = 0.0035
HINGE_BARREL_LENGTH = 0.014
HINGE_CLEVIS_AFT = 0.016
HINGE_CLEVIS_FWD = 0.004
TIP_REACH = 0.096
TIP_TAB_LENGTH = 0.030
TIP_TAB_WIDTH = 0.012
TIP_TAB_THICKNESS = 0.005

OPEN_POSE = 0.18
FOLDED_MAGNITUDE = 2.05

LINK_LAYER_Z = (0.008, 0.004, 0.0, -0.004, -0.008)


def _hinge_hole(x_pos: float, span: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_pos, 0.0)
        .circle(HINGE_HOLE_RADIUS)
        .extrude(span, both=True)
    )


def _clevis_at(x_axis: float) -> cq.Workplane:
    length = HINGE_CLEVIS_AFT + HINGE_CLEVIS_FWD
    center_x = x_axis + (HINGE_CLEVIS_FWD - HINGE_CLEVIS_AFT) * 0.5
    outer = cq.Workplane("XY").box(length, HINGE_OUTER_WIDTH, HINGE_BLOCK_HEIGHT).translate(
        (center_x, 0.0, 0.0)
    )
    slot = cq.Workplane("XY").box(
        length + 0.002,
        HINGE_SLOT_WIDTH,
        HINGE_BLOCK_HEIGHT + 0.002,
    ).translate((center_x, 0.0, 0.0))
    return outer.cut(slot)


def _barrel_at(x_axis: float) -> cq.Workplane:
    return cq.Workplane("XY").box(
        HINGE_BARREL_LENGTH,
        HINGE_BARREL_WIDTH,
        HINGE_BLOCK_HEIGHT,
    ).translate((x_axis, 0.0, 0.0))


def _spine_to(x_end: float, layer_z: float) -> cq.Workplane:
    x_start = HINGE_BARREL_LENGTH * 0.5
    x_mid = 0.026
    path = (
        cq.Workplane("XZ")
        .moveTo(x_start, 0.0)
        .lineTo(x_mid, layer_z)
        .lineTo(x_end, layer_z)
        .wire()
    )
    return (
        cq.Workplane("YZ")
        .rect(LINK_BAR_WIDTH, LINK_BAR_THICKNESS)
        .sweep(path.val(), transition="round")
    )


def _make_link(layer_z: float) -> cq.Workplane:
    clevis_back_x = LINK_PITCH - HINGE_CLEVIS_AFT
    body = _barrel_at(0.0)
    body = body.union(_spine_to(clevis_back_x, layer_z))
    body = body.union(_clevis_at(LINK_PITCH))
    body = body.cut(_hinge_hole(0.0, HINGE_OUTER_WIDTH + 0.006))
    body = body.cut(_hinge_hole(LINK_PITCH, HINGE_OUTER_WIDTH + 0.006))
    return body


def _make_terminal_link(layer_z: float) -> cq.Workplane:
    tab_center_x = TIP_REACH - TIP_TAB_LENGTH * 0.5
    spine_end = TIP_REACH - TIP_TAB_LENGTH - 0.006
    body = _barrel_at(0.0)
    body = body.union(_spine_to(spine_end, layer_z))
    neck = (
        cq.Workplane("XY")
        .box(0.020, 0.010, 0.006)
        .translate((TIP_REACH - 0.026, 0.0, layer_z * 0.7))
    )
    tab = (
        cq.Workplane("XY")
        .box(TIP_TAB_LENGTH, TIP_TAB_WIDTH, TIP_TAB_THICKNESS)
        .translate((tab_center_x, 0.0, layer_z))
    )
    slot = (
        cq.Workplane("XY")
        .box(0.018, 0.0045, TIP_TAB_THICKNESS + 0.002)
        .translate((TIP_REACH - 0.009, 0.0, layer_z))
    )
    body = body.union(neck).union(tab).cut(slot)
    body = body.cut(_hinge_hole(0.0, HINGE_OUTER_WIDTH + 0.006))
    return body


def _make_root_bracket() -> cq.Workplane:
    upright = (
        cq.Workplane("XY")
        .box(0.006, 0.050, 0.056)
        .translate((-0.030, 0.0, 0.0))
    )
    foot = (
        cq.Workplane("XY")
        .box(0.042, 0.054, 0.006)
        .translate((-0.013, 0.0, -0.025))
    )
    clevis = _clevis_at(0.0)
    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.028, -0.022),
                (-0.008, -0.022),
                (-0.001, -0.002),
                (-0.028, -0.002),
            ]
        )
        .close()
        .extrude(0.012, both=True)
    )
    hole_pair = (
        cq.Workplane("XY")
        .pushPoints([(-0.026, -0.016), (-0.026, 0.016)])
        .circle(0.0045)
        .extrude(0.010, both=True)
        .translate((0.0, 0.0, -0.025))
    )

    bracket = upright.union(foot).union(clevis).union(gusset)
    bracket = bracket.cut(hole_pair)
    bracket = bracket.cut(_hinge_hole(0.0, HINGE_OUTER_WIDTH + 0.006))
    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="accordion_arm")

    dark_bracket = model.material("bracket_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    link_metal = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.78, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_make_root_bracket(), "root_bracket"),
        material=dark_bracket,
        name="root_bracket_shell",
    )

    links = []
    for idx, layer_z in enumerate(LINK_LAYER_Z, start=1):
        link = model.part(f"link_{idx}")
        if idx < 5:
            geom = _make_link(layer_z)
        else:
            geom = _make_terminal_link(layer_z)
        link.visual(
            mesh_from_cadquery(geom, f"link_{idx}"),
            material=link_metal,
            name=f"link_{idx}_shell",
        )
        links.append(link)

    model.articulation(
        "bracket_to_link_1",
        ArticulationType.FIXED,
        parent=root_bracket,
        child=links[0],
        origin=Origin(),
    )

    joint_limits = MotionLimits(
        effort=12.0,
        velocity=1.8,
        lower=-2.25,
        upper=2.25,
    )
    for idx in range(4):
        model.articulation(
            f"hinge_{idx + 1}",
            ArticulationType.REVOLUTE,
            parent=links[idx],
            child=links[idx + 1],
            origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=joint_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    links = [object_model.get_part(f"link_{idx}") for idx in range(1, 6)]
    hinges = [object_model.get_articulation(f"hinge_{idx}") for idx in range(1, 5)]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.allow_overlap(
        root_bracket,
        links[0],
        reason="first hinge block is modeled as a captured knuckle nested into the root bracket clevis",
    )
    for idx in range(3):
        ctx.allow_overlap(
            links[idx],
            links[idx + 1],
            reason="adjacent hinge blocks intentionally use interleaved knuckle geometry around the common pin axis",
        )
    ctx.allow_overlap(
        links[3],
        links[4],
        reason="the terminal fork uses the same captured knuckle simplification as the preceding hinge blocks",
    )

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

    ctx.check("part_present_root_bracket", root_bracket is not None, "missing root bracket")
    for idx, link in enumerate(links, start=1):
        ctx.check(f"part_present_link_{idx}", link is not None, f"missing link_{idx}")

    for idx, hinge in enumerate(hinges, start=1):
        axis_ok = tuple(round(val, 6) for val in hinge.axis) == (0.0, 1.0, 0.0)
        limits = hinge.motion_limits
        limit_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
            and (limits.upper - limits.lower) >= 4.0
        )
        ctx.check(
            f"hinge_axis_family_{idx}",
            axis_ok and limit_ok,
            f"hinge_{idx} should be a y-axis revolute with a wide folding range",
        )

    ctx.expect_contact(root_bracket, links[0], name="root_bracket_mount_contact")
    for idx in range(4):
        ctx.expect_contact(
            links[idx],
            links[idx + 1],
            name=f"hinge_block_contact_{idx + 1}",
        )

    open_pose = {hinge: OPEN_POSE for hinge in hinges}
    with ctx.pose(open_pose):
        ctx.expect_gap(
            links[4],
            root_bracket,
            axis="x",
            min_gap=0.24,
            name="open_pose_reach",
        )

    folded_pose = {
        hinges[0]: FOLDED_MAGNITUDE,
        hinges[1]: -FOLDED_MAGNITUDE,
        hinges[2]: FOLDED_MAGNITUDE,
        hinges[3]: -FOLDED_MAGNITUDE,
    }
    with ctx.pose(folded_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clear")
        bracket_aabb = ctx.part_world_aabb(root_bracket)
        end_aabb = ctx.part_world_aabb(links[4])
        compact = False
        if bracket_aabb is not None and end_aabb is not None:
            compact = abs(end_aabb[0][0] - bracket_aabb[1][0]) <= 0.12
        ctx.check(
            "folded_pose_compact",
            compact,
            "terminal fork should fold back near the root bracket",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
