from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_turntable")

    model.material("warm_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("soft_black", rgba=(0.005, 0.005, 0.006, 1.0))
    model.material("brushed_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("rubber_dark", rgba=(0.025, 0.025, 0.026, 1.0))
    model.material("ivory_label", rgba=(0.86, 0.82, 0.68, 1.0))
    model.material("safety_orange", rgba=(0.95, 0.36, 0.12, 1.0))

    # A small apartment/desktop footprint: 34 cm x 24 cm, with a rounded
    # low plinth that carries both pivots and the stowed tonearm cradle.
    plinth = model.part("plinth")
    plinth_shell = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.340, 0.240, 0.026, corner_segments=8),
        0.035,
    )
    plinth.visual(
        mesh_from_geometry(plinth_shell, "plinth_shell"),
        material="warm_plastic",
        name="plinth_shell",
    )

    platter_x, platter_y = -0.055, 0.005
    platter_joint_z = 0.045
    tonearm_x, tonearm_y = 0.105, 0.080
    tonearm_joint_z = 0.077

    # Fixed platter bearing race and thrust pad on the plinth: the rotating
    # platter hub touches this pad at q=0 while the outer disc clears the deck.
    plinth.visual(
        Cylinder(radius=0.040, length=0.010),
        origin=Origin(xyz=(platter_x, platter_y, 0.040)),
        material="brushed_metal",
        name="platter_bearing",
    )
    plinth.visual(
        mesh_from_geometry(TorusGeometry(radius=0.041, tube=0.0035, radial_segments=18, tubular_segments=40), "bearing_ring"),
        origin=Origin(xyz=(platter_x, platter_y, 0.046)),
        material="brushed_metal",
        name="bearing_ring",
    )

    # Tonearm fixed pivot: low, compact bearing post with a broad foot so the
    # arm stage is visibly supported rather than floating from the deck.
    plinth.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(tonearm_x, tonearm_y, 0.038)),
        material="brushed_metal",
        name="tonearm_bearing_foot",
    )
    plinth.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(tonearm_x, tonearm_y, 0.059)),
        material="brushed_metal",
        name="tonearm_post",
    )

    # Fold-flat stow cradle along the right edge.  It sits below the arm tube,
    # leaving a visible clearance while indicating a safe parked position.
    plinth.visual(
        Cylinder(radius=0.006, length=0.043),
        origin=Origin(xyz=(tonearm_x, -0.074, 0.0565)),
        material="brushed_metal",
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.034, 0.013, 0.004)),
        origin=Origin(xyz=(tonearm_x, -0.074, 0.080)),
        material="brushed_metal",
        name="arm_rest_saddle",
    )
    plinth.visual(
        Box((0.004, 0.014, 0.004)),
        origin=Origin(xyz=(tonearm_x - 0.012, -0.074, 0.082)),
        material="brushed_metal",
        name="arm_rest_fork_0",
    )
    plinth.visual(
        Box((0.004, 0.014, 0.004)),
        origin=Origin(xyz=(tonearm_x + 0.012, -0.074, 0.082)),
        material="brushed_metal",
        name="arm_rest_fork_1",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.033, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material="brushed_metal",
        name="platter_hub",
    )
    platter.visual(
        Cylinder(radius=0.096, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material="brushed_metal",
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.088, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material="rubber_dark",
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.026, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.0230)),
        material="ivory_label",
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.0055, length=0.021),
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material="brushed_metal",
        name="spindle",
    )
    platter.visual(
        Box((0.018, 0.004, 0.002)),
        origin=Origin(xyz=(0.072, 0.0, 0.0240)),
        material="safety_orange",
        name="index_mark",
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_x, platter_y, platter_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="brushed_metal",
        name="pivot_cap",
    )
    tonearm.visual(
        Cylinder(radius=0.0035, length=0.142),
        origin=Origin(xyz=(0.0, -0.074, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="brushed_metal",
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(0.0, 0.020, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="soft_black",
        name="counterweight",
    )
    tonearm.visual(
        Box((0.025, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.154, 0.008)),
        material="soft_black",
        name="headshell",
    )
    tonearm.visual(
        Box((0.010, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.165, 0.002)),
        material="warm_plastic",
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0012, length=0.008),
        origin=Origin(xyz=(0.0, -0.168, -0.005)),
        material="brushed_metal",
        name="stylus",
    )

    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(tonearm_x, tonearm_y, tonearm_joint_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=1.0, lower=0.0, upper=1.12),
    )

    # A small articulated speed selector is kept low so it does not enlarge the
    # footprint; it rotates independently instead of being fused into the deck.
    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material="soft_black",
        name="selector_dial",
    )
    selector.visual(
        Box((0.020, 0.003, 0.002)),
        origin=Origin(xyz=(0.005, 0.0, 0.011)),
        material="ivory_label",
        name="selector_pointer",
    )
    model.articulation(
        "plinth_to_selector",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=selector,
        origin=Origin(xyz=(0.118, -0.092, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.5, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    selector = object_model.get_part("selector")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")
    selector_joint = object_model.get_articulation("plinth_to_selector")

    def _center(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="platter_hub",
        elem_b="platter_bearing",
        name="platter hub is supported by bearing pad",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disc",
        negative_elem="plinth_shell",
        min_gap=0.015,
        name="platter disc clears low plinth deck",
    )
    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="pivot_cap",
        elem_b="tonearm_post",
        name="tonearm cap is supported by pivot post",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="arm_tube",
        negative_elem="arm_rest_fork_0",
        min_gap=0.001,
        name="stowed arm clears cradle fork",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="x",
        positive_elem="arm_tube",
        negative_elem="platter_disc",
        min_gap=0.040,
        name="stowed tonearm stays beside platter",
    )
    ctx.expect_contact(
        selector,
        plinth,
        elem_a="selector_dial",
        elem_b="plinth_shell",
        name="speed selector sits on plinth",
    )

    marker_rest = ctx.part_element_world_aabb(platter, elem="index_mark")
    with ctx.pose({platter_joint: math.pi / 2.0}):
        marker_rotated = ctx.part_element_world_aabb(platter, elem="index_mark")
    ctx.check(
        "platter joint visibly rotates platter marker",
        _center(marker_rest, 1) is not None
        and _center(marker_rotated, 1) is not None
        and _center(marker_rotated, 1) > _center(marker_rest, 1) + 0.050,
        details=f"rest={marker_rest}, rotated={marker_rotated}",
    )

    stylus_stowed = ctx.part_element_world_aabb(tonearm, elem="stylus")
    with ctx.pose({tonearm_joint: 1.05}):
        ctx.expect_within(
            tonearm,
            platter,
            axes="xy",
            inner_elem="stylus",
            outer_elem="record_mat",
            margin=0.0,
            name="playing stylus lies within record mat",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="arm_tube",
            negative_elem="record_mat",
            min_gap=0.015,
            name="tonearm tube clears spinning platter",
        )
        ctx.expect_contact(
            tonearm,
            platter,
            contact_tol=0.001,
            elem_a="stylus",
            elem_b="record_mat",
            name="stylus reaches record surface",
        )
        stylus_playing = ctx.part_element_world_aabb(tonearm, elem="stylus")
    ctx.check(
        "tonearm swings inward from stow to play",
        _center(stylus_stowed, 0) is not None
        and _center(stylus_playing, 0) is not None
        and _center(stylus_playing, 0) < _center(stylus_stowed, 0) - 0.100,
        details=f"stowed={stylus_stowed}, playing={stylus_playing}",
    )

    selector_rest = ctx.part_element_world_aabb(selector, elem="selector_pointer")
    with ctx.pose({selector_joint: 0.60}):
        selector_rotated = ctx.part_element_world_aabb(selector, elem="selector_pointer")
    ctx.check(
        "selector pointer rotates as a separate control",
        _center(selector_rest, 1) is not None
        and _center(selector_rotated, 1) is not None
        and _center(selector_rotated, 1) > _center(selector_rest, 1) + 0.002,
        details=f"rest={selector_rest}, rotated={selector_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
