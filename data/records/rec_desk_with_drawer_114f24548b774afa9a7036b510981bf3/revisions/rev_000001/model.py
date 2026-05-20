from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secretary_desk")

    walnut = model.material("aged_walnut", color=(0.42, 0.22, 0.10, 1.0))
    dark_walnut = model.material("dark_endgrain", color=(0.25, 0.12, 0.055, 1.0))
    brass = model.material("warm_brass", color=(0.86, 0.62, 0.24, 1.0))
    shadow = model.material("shadowed_interior", color=(0.12, 0.075, 0.045, 1.0))
    rail_steel = model.material("dark_steel", color=(0.18, 0.18, 0.17, 1.0))

    width = 1.00
    depth = 0.42
    top_z = 1.35
    front_surface_y = -0.24
    face_frame_y = -0.225
    board_t = 0.035

    case = model.part("case")

    # Tall secretary desk carcass: connected boards with a proud face frame.
    case.visual(
        Box((board_t, depth, 1.30)),
        origin=Origin(xyz=(-width / 2 + board_t / 2, 0.0, 0.70)),
        material=walnut,
        name="side_panel_0",
    )
    case.visual(
        Box((board_t, depth, 1.30)),
        origin=Origin(xyz=(width / 2 - board_t / 2, 0.0, 0.70)),
        material=walnut,
        name="side_panel_1",
    )
    case.visual(
        Box((width, 0.025, 1.30)),
        origin=Origin(xyz=(0.0, depth / 2 - 0.0125, 0.70)),
        material=dark_walnut,
        name="back_panel",
    )
    case.visual(
        Box((width, depth, board_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=walnut,
        name="bottom_deck",
    )
    case.visual(
        Box((width, depth, board_t)),
        origin=Origin(xyz=(0.0, 0.0, top_z - board_t / 2)),
        material=walnut,
        name="top_cap",
    )
    case.visual(
        Box((width, depth, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=walnut,
        name="writing_floor",
    )
    case.visual(
        Box((width, depth, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.895)),
        material=walnut,
        name="upper_shelf",
    )

    # Face frame rails and stiles define the drawer, drop-front, and upper-door
    # openings without filling the visibly hollow compartments.
    for x, visual_name in [(-0.4825, "front_stile_0"), (0.4825, "front_stile_1")]:
        case.visual(
            Box((0.035, 0.030, 1.30)),
            origin=Origin(xyz=(x, face_frame_y, 0.70)),
            material=dark_walnut,
            name=visual_name,
        )
    case.visual(
        Box((0.035, 0.030, 0.40)),
        origin=Origin(xyz=(0.0, face_frame_y, 1.105)),
        material=dark_walnut,
        name="upper_center_stile",
    )
    case.visual(
        Box((width, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, face_frame_y, 0.075)),
        material=dark_walnut,
        name="toe_rail",
    )
    case.visual(
        Box((width, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, face_frame_y, 0.355)),
        material=dark_walnut,
        name="drawer_top_rail",
    )
    case.visual(
        Box((width, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, face_frame_y, 0.895)),
        material=dark_walnut,
        name="writing_top_rail",
    )
    case.visual(
        Box((width, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, face_frame_y, 1.300)),
        material=dark_walnut,
        name="upper_top_rail",
    )

    # Dark backs inside the two visible compartments make the cabinet read as
    # hollow rather than as a single solid block.
    case.visual(
        Box((0.86, 0.006, 0.47)),
        origin=Origin(xyz=(0.0, 0.182, 0.620)),
        material=shadow,
        name="writing_shadow",
    )
    case.visual(
        Box((0.86, 0.006, 0.36)),
        origin=Origin(xyz=(0.0, 0.182, 1.105)),
        material=shadow,
        name="upper_shadow",
    )

    # Stationary drawer guide rails below the cabinet.
    case.visual(
        Box((0.040, 0.360, 0.018)),
        origin=Origin(xyz=(-0.455, -0.040, 0.205)),
        material=rail_steel,
        name="guide_rail_0",
    )
    case.visual(
        Box((0.040, 0.360, 0.018)),
        origin=Origin(xyz=(0.455, -0.040, 0.205)),
        material=rail_steel,
        name="guide_rail_1",
    )

    # Long base drawer on prismatic rails.
    drawer = model.part("drawer")
    drawer.visual(
        Box((0.900, 0.040, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=walnut,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.835, 0.380, 0.014)),
        origin=Origin(xyz=(0.0, 0.200, -0.088)),
        material=dark_walnut,
        name="drawer_bottom",
    )
    for x, visual_name in [(-0.425, "drawer_side_0"), (0.425, "drawer_side_1")]:
        drawer.visual(
            Box((0.020, 0.380, 0.140)),
            origin=Origin(xyz=(x, 0.200, -0.020)),
            material=walnut,
            name=visual_name,
        )
    drawer.visual(
        Box((0.840, 0.018, 0.135)),
        origin=Origin(xyz=(0.0, 0.386, -0.020)),
        material=walnut,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.016, 0.410, 0.016)),
        origin=Origin(xyz=(-0.407, 0.205, -0.030)),
        material=rail_steel,
        name="moving_rail_0",
    )
    drawer.visual(
        Box((0.016, 0.410, 0.016)),
        origin=Origin(xyz=(0.407, 0.205, -0.030)),
        material=rail_steel,
        name="moving_rail_1",
    )
    drawer.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(-0.130, -0.036, 0.018), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="pull_post_0",
    )
    drawer.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.130, -0.036, 0.018), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="pull_post_1",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.280),
        origin=Origin(xyz=(0.0, -0.070, 0.018), rpy=(0.0, math.pi / 2, 0.0)),
        material=brass,
        name="drawer_pull",
    )
    model.articulation(
        "case_to_drawer",
        ArticulationType.PRISMATIC,
        parent=case,
        child=drawer,
        origin=Origin(xyz=(0.0, front_surface_y - 0.020, 0.225)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.28),
    )

    # Drop-front writing panel.  The hinge frame lies along the lower front edge;
    # positive rotation folds the closed vertical panel down to a horizontal desk.
    writing_panel = model.part("writing_panel")
    writing_panel.visual(
        Box((0.860, 0.030, 0.470)),
        origin=Origin(xyz=(0.0, -0.015, 0.235)),
        material=walnut,
        name="writing_slab",
    )
    for z, h, visual_name in [
        (0.045, 0.035, "writing_lower_trim"),
        (0.425, 0.035, "writing_upper_trim"),
    ]:
        writing_panel.visual(
            Box((0.800, 0.018, h)),
            origin=Origin(xyz=(0.0, -0.038, z)),
            material=dark_walnut,
            name=visual_name,
        )
    for x, visual_name in [(-0.380, "writing_side_trim_0"), (0.380, "writing_side_trim_1")]:
        writing_panel.visual(
            Box((0.035, 0.018, 0.390)),
            origin=Origin(xyz=(x, -0.038, 0.235)),
            material=dark_walnut,
            name=visual_name,
        )
    writing_panel.visual(
        Cylinder(radius=0.015, length=0.820),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=brass,
        name="piano_hinge",
    )
    writing_panel.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.0, -0.043, 0.370), rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="writing_pull_stem",
    )
    writing_panel.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, -0.074, 0.370)),
        material=brass,
        name="writing_pull_knob",
    )
    model.articulation(
        "case_to_writing_panel",
        ArticulationType.REVOLUTE,
        parent=case,
        child=writing_panel,
        origin=Origin(xyz=(0.0, front_surface_y, 0.385)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=math.pi / 2),
    )

    # Pair of small upper cabinet doors.  Numeric names avoid inventing a
    # left/right semantic distinction for an otherwise symmetric cabinet pair.
    for index, hinge_x, panel_center_x, axis_z, knob_x in [
        (0, -0.450, 0.2025, -1.0, 0.355),
        (1, 0.450, -0.2025, 1.0, -0.355),
    ]:
        door = model.part(f"door_{index}")
        door.visual(
            Box((0.405, 0.025, 0.340)),
            origin=Origin(xyz=(panel_center_x, -0.0125, 0.170)),
            material=walnut,
            name="door_slab",
        )
        door.visual(
            Cylinder(radius=0.009, length=0.330),
            origin=Origin(xyz=(0.0, -0.014, 0.170)),
            material=brass,
            name="hinge_barrel",
        )
        for z, h, visual_name in [
            (0.040, 0.030, "door_lower_trim"),
            (0.300, 0.030, "door_upper_trim"),
        ]:
            door.visual(
                Box((0.330, 0.016, h)),
                origin=Origin(xyz=(panel_center_x, -0.032, z)),
                material=dark_walnut,
                name=visual_name,
            )
        inner_sign = 1.0 if index == 0 else -1.0
        for sx, visual_name in [
            (hinge_x * 0.0 + panel_center_x - inner_sign * 0.165, "door_outer_trim"),
            (panel_center_x + inner_sign * 0.165, "door_inner_trim"),
        ]:
            door.visual(
                Box((0.028, 0.016, 0.270)),
                origin=Origin(xyz=(sx, -0.032, 0.170)),
                material=dark_walnut,
                name=visual_name,
            )
        door.visual(
            Cylinder(radius=0.007, length=0.042),
            origin=Origin(xyz=(knob_x, -0.038, 0.170), rpy=(math.pi / 2, 0.0, 0.0)),
            material=brass,
            name="knob_stem",
        )
        door.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(knob_x, -0.064, 0.170)),
            material=brass,
            name="door_knob",
        )
        model.articulation(
            f"case_to_door_{index}",
            ArticulationType.REVOLUTE,
            parent=case,
            child=door,
            origin=Origin(xyz=(hinge_x, front_surface_y, 0.930)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.75),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    case = object_model.get_part("case")
    drawer = object_model.get_part("drawer")
    writing_panel = object_model.get_part("writing_panel")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")

    drawer_slide = object_model.get_articulation("case_to_drawer")
    writing_hinge = object_model.get_articulation("case_to_writing_panel")
    door_hinge_0 = object_model.get_articulation("case_to_door_0")
    door_hinge_1 = object_model.get_articulation("case_to_door_1")

    ctx.expect_gap(
        case,
        drawer,
        axis="y",
        positive_elem="drawer_top_rail",
        negative_elem="drawer_front",
        max_gap=0.002,
        max_penetration=0.0,
        name="drawer front closes flush with face frame",
    )
    ctx.expect_gap(
        case,
        writing_panel,
        axis="y",
        positive_elem="writing_top_rail",
        negative_elem="writing_slab",
        max_gap=0.002,
        max_penetration=0.0,
        name="drop front closes flush with face frame",
    )
    ctx.expect_gap(
        case,
        door_0,
        axis="y",
        positive_elem="upper_top_rail",
        negative_elem="door_slab",
        max_gap=0.002,
        max_penetration=0.0,
        name="upper door 0 closes flush",
    )
    ctx.expect_gap(
        case,
        door_1,
        axis="y",
        positive_elem="upper_top_rail",
        negative_elem="door_slab",
        max_gap=0.002,
        max_penetration=0.0,
        name="upper door 1 closes flush",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.28}):
        drawer_extended = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            case,
            axes="y",
            elem_a="moving_rail_0",
            elem_b="guide_rail_0",
            min_overlap=0.045,
            name="extended drawer rail remains engaged",
        )
    ctx.check(
        "drawer slides outward on prismatic rails",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.25,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    with ctx.pose({writing_hinge: math.pi / 2}):
        slab_aabb = ctx.part_element_world_aabb(writing_panel, elem="writing_slab")
        ctx.check(
            "drop front folds down into a writing surface",
            slab_aabb is not None
            and slab_aabb[0][1] < -0.65
            and slab_aabb[1][2] < 0.42,
            details=f"writing_slab_aabb={slab_aabb}",
        )

    with ctx.pose({door_hinge_0: 1.40, door_hinge_1: 1.40}):
        door_0_aabb = ctx.part_element_world_aabb(door_0, elem="door_slab")
        door_1_aabb = ctx.part_element_world_aabb(door_1, elem="door_slab")
        ctx.check(
            "upper doors swing outward from their side hinges",
            door_0_aabb is not None
            and door_1_aabb is not None
            and door_0_aabb[0][1] < -0.55
            and door_1_aabb[0][1] < -0.55,
            details=f"door_0_aabb={door_0_aabb}, door_1_aabb={door_1_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
