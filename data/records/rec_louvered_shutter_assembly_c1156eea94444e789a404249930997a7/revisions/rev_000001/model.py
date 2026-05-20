from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_panel_louvered_shutter")

    painted = Material("warm_white_painted_wood", rgba=(0.86, 0.84, 0.76, 1.0))
    edge_shadow = Material("shadowed_louver_edges", rgba=(0.52, 0.50, 0.44, 1.0))
    dark_metal = Material("aged_dark_hinge_metal", rgba=(0.12, 0.12, 0.11, 1.0))
    model.materials.extend([painted, edge_shadow, dark_metal])

    opening = model.part("opening_frame")
    # A broad built-in frame around the wide opening.  The inner jamb faces sit
    # exactly on the two outer hinge axes, giving the closed panels a real seat.
    opening.visual(
        Box((0.14, 0.11, 1.62)),
        origin=Origin(xyz=(-1.07, 0.0, 0.0)),
        material=painted,
        name="jamb_0",
    )
    opening.visual(
        Box((0.14, 0.11, 1.62)),
        origin=Origin(xyz=(1.07, 0.0, 0.0)),
        material=painted,
        name="jamb_1",
    )
    opening.visual(
        Box((2.28, 0.11, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=painted,
        name="head_rail",
    )
    opening.visual(
        Box((2.28, 0.11, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.81)),
        material=painted,
        name="sill_rail",
    )

    panel_width = 1.00
    panel_height = 1.40
    frame_depth = 0.060
    stile_w = 0.070
    rail_h = 0.100
    louver_len = 0.800
    pivot_len = 0.860
    louver_chord = 0.110
    louver_thick = 0.018
    louver_tilt = math.radians(24.0)
    louver_zs = [-0.49, -0.35, -0.21, -0.07, 0.07, 0.21, 0.35, 0.49]

    def add_panel(panel_name: str, sign: float, hinge_x: float) -> None:
        panel = model.part(panel_name)
        panel.visual(
            Box((stile_w, frame_depth, panel_height)),
            origin=Origin(xyz=(sign * stile_w / 2.0, 0.0, 0.0)),
            material=painted,
            name="hinge_stile",
        )
        panel.visual(
            Box((stile_w, frame_depth * 0.96, panel_height)),
            origin=Origin(xyz=(sign * (panel_width - stile_w / 2.0), 0.0, 0.0)),
            material=painted,
            name="meeting_stile",
        )
        panel.visual(
            Box((panel_width, frame_depth * 0.96, rail_h)),
            origin=Origin(xyz=(sign * panel_width / 2.0, 0.0, panel_height / 2.0 - rail_h / 2.0)),
            material=painted,
            name="top_rail",
        )
        panel.visual(
            Box((panel_width, frame_depth * 0.96, rail_h)),
            origin=Origin(xyz=(sign * panel_width / 2.0, 0.0, -panel_height / 2.0 + rail_h / 2.0)),
            material=painted,
            name="bottom_rail",
        )
        # Three proud hinge knuckles on the front face make the outer jamb hinge
        # line visible without hiding the panel/frame seating contact.
        for hinge_i, z in enumerate((-0.46, 0.0, 0.46)):
            panel.visual(
                Box((0.044, 0.026, 0.170)),
                origin=Origin(xyz=(sign * 0.022, 0.043, z)),
                material=dark_metal,
                name=f"hinge_leaf_{hinge_i}",
            )
            panel.visual(
                Cylinder(radius=0.012, length=0.175),
                origin=Origin(xyz=(sign * 0.018, 0.057, z)),
                material=dark_metal,
                name=f"hinge_knuckle_{hinge_i}",
            )

        model.articulation(
            f"opening_to_{panel_name}",
            ArticulationType.REVOLUTE,
            parent=opening,
            child=panel,
            origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=math.radians(105.0)),
        )

        louver_origin_x = sign * panel_width / 2.0
        for row, z in enumerate(louver_zs):
            louver = model.part(f"louver_{0 if sign > 0 else 1}_{row}")
            louver.visual(
                Box((louver_len, louver_thick, louver_chord)),
                origin=Origin(rpy=(louver_tilt, 0.0, 0.0)),
                material=painted,
                name="slat",
            )
            louver.visual(
                Cylinder(radius=0.010, length=pivot_len),
                origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_metal,
                name="pivot_pin",
            )
            model.articulation(
                f"{panel_name}_to_louver_{row}",
                ArticulationType.REVOLUTE,
                parent=panel,
                child=louver,
                origin=Origin(xyz=(louver_origin_x, 0.0, z)),
                axis=(1.0, 0.0, 0.0),
                motion_limits=MotionLimits(effort=0.8, velocity=2.5, lower=math.radians(-35.0), upper=math.radians(55.0)),
            )

    add_panel("panel_0", 1.0, -1.0)
    add_panel("panel_1", -1.0, 1.0)

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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    panel_0 = object_model.get_part("panel_0")
    panel_1 = object_model.get_part("panel_1")
    opening = object_model.get_part("opening_frame")
    panel_hinges = [
        object_model.get_articulation("opening_to_panel_0"),
        object_model.get_articulation("opening_to_panel_1"),
    ]
    louver_joints = [
        joint
        for joint in object_model.articulations
        if "_to_louver_" in joint.name
    ]

    ctx.check("two whole-panel hinge joints", len(panel_hinges) == 2)
    ctx.check("every louver has its own pivot", len(louver_joints) == 16)
    ctx.check(
        "panel hinge axes are vertical and mirrored",
        tuple(panel_hinges[0].axis) == (0.0, 0.0, 1.0)
        and tuple(panel_hinges[1].axis) == (0.0, 0.0, -1.0),
    )

    ctx.expect_contact(
        panel_0,
        panel_1,
        elem_a="meeting_stile",
        elem_b="meeting_stile",
        contact_tol=1e-5,
        name="closed panels meet at center",
    )
    ctx.expect_contact(
        panel_0,
        opening,
        elem_a="hinge_stile",
        elem_b="jamb_0",
        contact_tol=1e-5,
        name="panel 0 seats on its outer jamb",
    )
    ctx.expect_contact(
        panel_1,
        opening,
        elem_a="hinge_stile",
        elem_b="jamb_1",
        contact_tol=1e-5,
        name="panel 1 seats on its outer jamb",
    )
    ctx.expect_contact(
        "louver_0_0",
        panel_0,
        elem_a="pivot_pin",
        elem_b="hinge_stile",
        contact_tol=1e-5,
        name="sample louver pin reaches hinge stile",
    )
    ctx.expect_contact(
        "louver_0_0",
        panel_0,
        elem_a="pivot_pin",
        elem_b="meeting_stile",
        contact_tol=1e-5,
        name="sample louver pin reaches meeting stile",
    )

    def aabb_center_y(part_name: str, elem: str) -> float | None:
        bounds = ctx.part_element_world_aabb(object_model.get_part(part_name), elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return (lo[1] + hi[1]) / 2.0

    closed_y_0 = aabb_center_y("panel_0", "meeting_stile")
    closed_y_1 = aabb_center_y("panel_1", "meeting_stile")
    with ctx.pose({"opening_to_panel_0": math.radians(75.0), "opening_to_panel_1": math.radians(75.0)}):
        open_y_0 = aabb_center_y("panel_0", "meeting_stile")
        open_y_1 = aabb_center_y("panel_1", "meeting_stile")
    ctx.check(
        "both panels swing outward from the wide opening",
        closed_y_0 is not None
        and closed_y_1 is not None
        and open_y_0 is not None
        and open_y_1 is not None
        and open_y_0 > closed_y_0 + 0.40
        and open_y_1 > closed_y_1 + 0.40,
        details=f"closed_y=({closed_y_0}, {closed_y_1}), open_y=({open_y_0}, {open_y_1})",
    )

    return ctx.report()


object_model = build_object_model()
