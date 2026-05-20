from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_RADIUS = 0.0065
LOWER_SUPPORT_Z = 0.024
MIDDLE_SUPPORT_Z = 0.058
TOP_SUPPORT_Z = 0.086
SHAFT_TOP_Z = 0.125


def _make_base_support() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.13).extrude(0.010)
    pedestal = cq.Workplane("XY").circle(0.072).extrude(0.014).translate((0.0, 0.0, 0.010))
    shaft_core = cq.Workplane("XY").circle(SHAFT_RADIUS).extrude(SHAFT_TOP_Z)
    middle_collar = cq.Workplane("XY").circle(0.032).extrude(0.012).translate((0.0, 0.0, 0.046))
    top_collar = cq.Workplane("XY").circle(0.027).extrude(0.012).translate((0.0, 0.0, 0.074))
    upper_spindle = cq.Workplane("XY").circle(0.016).extrude(0.018).translate((0.0, 0.0, 0.100))
    retainer = cq.Workplane("XY").circle(0.020).extrude(0.007).translate((0.0, 0.0, 0.118))
    return (
        base.union(pedestal)
        .union(shaft_core)
        .union(middle_collar)
        .union(top_collar)
        .union(upper_spindle)
        .union(retainer)
    )


def _make_stage(
    *,
    radius: float,
    thickness: float,
    bore_radius: float,
    hub_radius: float,
    hub_height: float,
    pocket_depth: float,
    rim_band: float,
) -> cq.Workplane:
    stage = cq.Workplane("XY").circle(radius).extrude(thickness)
    stage = stage.union(cq.Workplane("XY").circle(hub_radius).extrude(hub_height))
    if bore_radius > 0.0:
        stage = stage.cut(cq.Workplane("XY").circle(bore_radius).extrude(hub_height))

    pocket_outer = radius - rim_band
    pocket_inner = hub_radius + 0.010
    if pocket_outer > pocket_inner + 0.004 and pocket_depth > 0.0:
        pocket = (
            cq.Workplane("XY")
            .circle(pocket_outer)
            .circle(pocket_inner)
            .extrude(pocket_depth)
            .translate((0.0, 0.0, thickness - pocket_depth))
        )
        stage = stage.cut(pocket)

    return stage


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))


def _planar_distance(a, b) -> float | None:
    if a is None or b is None:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wafer_style_coaxial_stage_stack")

    model.material("base_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("lower_stage_finish", rgba=(0.28, 0.29, 0.31, 1.0))
    model.material("middle_stage_finish", rgba=(0.52, 0.54, 0.57, 1.0))
    model.material("top_stage_finish", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("marker_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_support = model.part("base_support")
    base_support.visual(
        Cylinder(radius=0.13, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="base_graphite",
        name="base_foot",
    )
    base_support.visual(
        Cylinder(radius=0.072, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material="base_graphite",
        name="lower_pedestal",
    )
    base_support.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_TOP_Z),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_TOP_Z * 0.5)),
        material="base_graphite",
        name="shaft_core",
    )
    base_support.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material="base_graphite",
        name="middle_spacer",
    )
    base_support.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material="base_graphite",
        name="top_spacer",
    )
    base_support.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material="base_graphite",
        name="upper_spindle",
    )
    base_support.visual(
        Cylinder(radius=0.020, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.1215)),
        material="base_graphite",
        name="retainer_cap",
    )

    lower_platter = model.part("lower_platter")
    lower_platter.visual(
        mesh_from_cadquery(
            _make_stage(
                radius=0.170,
                thickness=0.016,
                bore_radius=0.019,
                hub_radius=0.048,
                hub_height=0.022,
                pocket_depth=0.004,
                rim_band=0.018,
            ),
            "lower_platter",
        ),
        material="lower_stage_finish",
        name="lower_platter_shell",
    )
    lower_platter.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.122, 0.0, 0.0125)),
        material="marker_black",
        name="lower_marker",
    )

    middle_deck = model.part("middle_deck")
    middle_deck.visual(
        mesh_from_cadquery(
            _make_stage(
                radius=0.122,
                thickness=0.010,
                bore_radius=0.0175,
                hub_radius=0.040,
                hub_height=0.016,
                pocket_depth=0.003,
                rim_band=0.013,
            ),
            "middle_deck",
        ),
        material="middle_stage_finish",
        name="middle_deck_shell",
    )
    middle_deck.visual(
        Cylinder(radius=0.005, length=0.003),
        origin=Origin(xyz=(0.084, 0.0, 0.0075)),
        material="marker_black",
        name="middle_marker",
    )

    top_deck = model.part("top_deck")
    top_deck.visual(
        mesh_from_cadquery(
            _make_stage(
                radius=0.085,
                thickness=0.009,
                bore_radius=0.0165,
                hub_radius=0.033,
                hub_height=0.014,
                pocket_depth=0.0025,
                rim_band=0.010,
            ),
            "top_deck",
        ),
        material="top_stage_finish",
        name="top_deck_shell",
    )
    top_deck.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.058, 0.0, 0.0070)),
        material="marker_black",
        name="top_marker",
    )

    model.articulation(
        "base_to_lower_platter",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=lower_platter,
        origin=Origin(xyz=(0.0, 0.0, LOWER_SUPPORT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "base_to_middle_deck",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=middle_deck,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_SUPPORT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "base_to_top_deck",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=top_deck,
        origin=Origin(xyz=(0.0, 0.0, TOP_SUPPORT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.2,
            lower=-math.pi,
            upper=math.pi,
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

    base_support = object_model.get_part("base_support")
    lower_platter = object_model.get_part("lower_platter")
    middle_deck = object_model.get_part("middle_deck")
    top_deck = object_model.get_part("top_deck")

    lower_joint = object_model.get_articulation("base_to_lower_platter")
    middle_joint = object_model.get_articulation("base_to_middle_deck")
    top_joint = object_model.get_articulation("base_to_top_deck")

    joints = (lower_joint, middle_joint, top_joint)
    joint_summary = [
        {
            "name": joint.name,
            "type": str(joint.articulation_type),
            "axis": tuple(joint.axis),
            "origin": tuple(joint.origin.xyz),
        }
        for joint in joints
    ]
    ctx.check(
        "all decks use vertical revolute joints on one common axis",
        all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints)
        and all(tuple(joint.axis) == (0.0, 0.0, 1.0) for joint in joints)
        and all(abs(joint.origin.xyz[0]) < 1e-9 and abs(joint.origin.xyz[1]) < 1e-9 for joint in joints)
        and lower_joint.origin.xyz[2] < middle_joint.origin.xyz[2] < top_joint.origin.xyz[2],
        details=str(joint_summary),
    )

    ctx.expect_contact(
        lower_platter,
        base_support,
        contact_tol=5e-4,
        name="lower platter is seated on the grounded pedestal",
    )
    ctx.expect_contact(
        middle_deck,
        base_support,
        contact_tol=5e-4,
        name="middle deck is seated on the common shaft collar",
    )
    ctx.expect_contact(
        top_deck,
        base_support,
        contact_tol=5e-4,
        name="top deck is seated on the upper shaft collar",
    )

    ctx.expect_gap(
        middle_deck,
        lower_platter,
        axis="z",
        min_gap=0.008,
        max_gap=0.020,
        name="middle deck remains clearly separated above the lower platter",
    )
    ctx.expect_gap(
        top_deck,
        middle_deck,
        axis="z",
        min_gap=0.008,
        max_gap=0.020,
        name="top deck remains clearly separated above the middle deck",
    )

    rest_lower = _aabb_center(ctx.part_element_world_aabb(lower_platter, elem="lower_marker"))
    rest_middle = _aabb_center(ctx.part_element_world_aabb(middle_deck, elem="middle_marker"))
    rest_top = _aabb_center(ctx.part_element_world_aabb(top_deck, elem="top_marker"))

    with ctx.pose({lower_joint: 1.0}):
        moved_lower = _aabb_center(ctx.part_element_world_aabb(lower_platter, elem="lower_marker"))
        still_middle = _aabb_center(ctx.part_element_world_aabb(middle_deck, elem="middle_marker"))
        still_top = _aabb_center(ctx.part_element_world_aabb(top_deck, elem="top_marker"))
    ctx.check(
        "lower platter rotates independently around the shared shaft",
        moved_lower is not None
        and rest_lower is not None
        and moved_lower[1] > rest_lower[1] + 0.05
        and moved_lower[0] < rest_lower[0] - 0.03
        and _planar_distance(rest_middle, still_middle) is not None
        and _planar_distance(rest_middle, still_middle) < 0.002
        and _planar_distance(rest_top, still_top) is not None
        and _planar_distance(rest_top, still_top) < 0.002,
        details=(
            f"lower_rest={rest_lower}, lower_moved={moved_lower}, "
            f"middle_delta={_planar_distance(rest_middle, still_middle)}, "
            f"top_delta={_planar_distance(rest_top, still_top)}"
        ),
    )

    with ctx.pose({middle_joint: 1.0}):
        still_lower = _aabb_center(ctx.part_element_world_aabb(lower_platter, elem="lower_marker"))
        moved_middle = _aabb_center(ctx.part_element_world_aabb(middle_deck, elem="middle_marker"))
        still_top = _aabb_center(ctx.part_element_world_aabb(top_deck, elem="top_marker"))
    ctx.check(
        "middle deck rotates independently around the shared shaft",
        moved_middle is not None
        and rest_middle is not None
        and moved_middle[1] > rest_middle[1] + 0.04
        and moved_middle[0] < rest_middle[0] - 0.02
        and _planar_distance(rest_lower, still_lower) is not None
        and _planar_distance(rest_lower, still_lower) < 0.002
        and _planar_distance(rest_top, still_top) is not None
        and _planar_distance(rest_top, still_top) < 0.002,
        details=(
            f"middle_rest={rest_middle}, middle_moved={moved_middle}, "
            f"lower_delta={_planar_distance(rest_lower, still_lower)}, "
            f"top_delta={_planar_distance(rest_top, still_top)}"
        ),
    )

    with ctx.pose({top_joint: 1.0}):
        still_lower = _aabb_center(ctx.part_element_world_aabb(lower_platter, elem="lower_marker"))
        still_middle = _aabb_center(ctx.part_element_world_aabb(middle_deck, elem="middle_marker"))
        moved_top = _aabb_center(ctx.part_element_world_aabb(top_deck, elem="top_marker"))
    ctx.check(
        "top deck rotates independently around the shared shaft",
        moved_top is not None
        and rest_top is not None
        and moved_top[1] > rest_top[1] + 0.03
        and moved_top[0] < rest_top[0] - 0.015
        and _planar_distance(rest_lower, still_lower) is not None
        and _planar_distance(rest_lower, still_lower) < 0.002
        and _planar_distance(rest_middle, still_middle) is not None
        and _planar_distance(rest_middle, still_middle) < 0.002,
        details=(
            f"top_rest={rest_top}, top_moved={moved_top}, "
            f"lower_delta={_planar_distance(rest_lower, still_lower)}, "
            f"middle_delta={_planar_distance(rest_middle, still_middle)}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
