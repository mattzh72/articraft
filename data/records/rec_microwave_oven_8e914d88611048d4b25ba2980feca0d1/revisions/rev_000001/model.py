from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_microwave")

    brushed_steel = model.material("brushed_steel", color=(0.62, 0.64, 0.62, 1.0))
    dark_cavity = model.material("dark_cavity", color=(0.035, 0.038, 0.04, 1.0))
    black_glass = model.material("black_glass", color=(0.02, 0.025, 0.03, 0.72))
    smoked_window = model.material("smoked_window", color=(0.04, 0.07, 0.09, 0.56))
    control_plastic = model.material("control_plastic", color=(0.10, 0.105, 0.11, 1.0))
    button_gray = model.material("button_gray", color=(0.21, 0.22, 0.23, 1.0))
    button_accent = model.material("button_accent", color=(0.12, 0.34, 0.18, 1.0))
    rubber = model.material("rubber", color=(0.015, 0.015, 0.014, 1.0))
    glass = model.material("faint_glass", color=(0.78, 0.88, 0.92, 0.38))

    body = model.part("body")

    # Fixed commercial microwave carcass: a hollow cooking bay on the left and a
    # solid control/electronics column on the right.  The dimensions are close to
    # a countertop commercial unit rather than a toy scale.
    body.visual(
        Box((0.125, 0.55, 0.40)),
        origin=Origin(xyz=(0.2675, 0.0, 0.20)),
        material=control_plastic,
        name="control_panel",
    )
    body.visual(
        Box((0.535, 0.55, 0.030)),
        origin=Origin(xyz=(-0.0625, 0.0, 0.385)),
        material=brushed_steel,
        name="top_wall",
    )
    body.visual(
        Box((0.535, 0.55, 0.030)),
        origin=Origin(xyz=(-0.0625, 0.0, 0.015)),
        material=brushed_steel,
        name="bottom_wall",
    )
    body.visual(
        Box((0.030, 0.55, 0.40)),
        origin=Origin(xyz=(-0.315, 0.0, 0.20)),
        material=brushed_steel,
        name="hinge_jamb",
    )
    body.visual(
        Box((0.030, 0.55, 0.40)),
        origin=Origin(xyz=(0.190, 0.0, 0.20)),
        material=brushed_steel,
        name="center_divider",
    )
    body.visual(
        Box((0.535, 0.030, 0.34)),
        origin=Origin(xyz=(-0.0625, 0.260, 0.20)),
        material=dark_cavity,
        name="rear_cavity_wall",
    )
    body.visual(
        Box((0.505, 0.022, 0.030)),
        origin=Origin(xyz=(-0.0625, -0.264, 0.385)),
        material=brushed_steel,
        name="front_top_rail",
    )
    body.visual(
        Box((0.505, 0.022, 0.030)),
        origin=Origin(xyz=(-0.0625, -0.264, 0.015)),
        material=brushed_steel,
        name="front_bottom_rail",
    )
    body.visual(
        Box((0.026, 0.022, 0.340)),
        origin=Origin(xyz=(-0.317, -0.264, 0.20)),
        material=brushed_steel,
        name="front_hinge_rail",
    )
    body.visual(
        Box((0.026, 0.022, 0.340)),
        origin=Origin(xyz=(0.190, -0.264, 0.20)),
        material=brushed_steel,
        name="front_latch_rail",
    )
    body.visual(
        Cylinder(radius=0.165, length=0.008),
        origin=Origin(xyz=(-0.070, -0.030, 0.034)),
        material=glass,
        name="glass_turntable",
    )
    body.visual(
        Box((0.092, 0.006, 0.045)),
        origin=Origin(xyz=(0.265, -0.278, 0.326)),
        material=black_glass,
        name="display_window",
    )

    # Rear-side vent slots and rubber feet are fused to the fixed body so they
    # read as surface treatment rather than floating details.
    for idx, z in enumerate((0.135, 0.165, 0.195, 0.225, 0.255)):
        body.visual(
            Box((0.070, 0.006, 0.010)),
            origin=Origin(xyz=(0.266, 0.278, z)),
            material=dark_cavity,
            name=f"panel_slot_{idx}",
        )
    for idx, (x, y) in enumerate(((-0.245, -0.180), (0.245, -0.180), (-0.245, 0.190), (0.245, 0.190))):
        body.visual(
            Box((0.060, 0.055, 0.020)),
            origin=Origin(xyz=(x, y, -0.010)),
            material=rubber,
            name=f"foot_{idx}",
        )

    door = model.part("door")
    door_w = 0.490
    door.visual(
        Box((door_w, 0.026, 0.340)),
        origin=Origin(xyz=(door_w / 2.0, -0.013, 0.0)),
        material=brushed_steel,
        name="door_panel",
    )
    door.visual(
        Box((0.315, 0.006, 0.205)),
        origin=Origin(xyz=(0.235, -0.029, 0.005)),
        material=smoked_window,
        name="viewing_window",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(xyz=(0.0, -0.035, 0.0)),
        material=brushed_steel,
        name="hinge_barrel",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.245),
        origin=Origin(xyz=(door_w - 0.045, -0.065, 0.0)),
        material=control_plastic,
        name="pull_grip",
    )
    for idx, z in enumerate((-0.105, 0.105)):
        door.visual(
            Box((0.032, 0.040, 0.026)),
            origin=Origin(xyz=(door_w - 0.045, -0.045, z)),
            material=control_plastic,
            name=f"handle_standoff_{idx}",
        )

    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.310, -0.275, 0.200)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    door_hinge.meta["description"] = "Vertical side hinge: positive rotation swings the broad door outward."

    key_w = 0.021
    key_h = 0.024
    key_t = 0.012
    key_xs = (0.226, 0.252, 0.278, 0.304)
    key_zs = (0.265, 0.230, 0.195, 0.160, 0.125, 0.090)
    for row, z in enumerate(key_zs):
        for col, x in enumerate(key_xs):
            key = model.part(f"key_{row}_{col}")
            material = button_accent if (row, col) in ((5, 2), (5, 3)) else button_gray
            key.visual(
                Box((key_w, key_t, key_h)),
                origin=Origin(xyz=(0.0, -key_t / 2.0, 0.0)),
                material=material,
                name="key_cap",
            )
            model.articulation(
                f"body_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=key,
                origin=Origin(xyz=(x, -0.275, z)),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
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

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.expect_contact(
        door,
        body,
        elem_a="door_panel",
        elem_b="front_top_rail",
        contact_tol=0.002,
        name="closed door seats on front rail",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward on vertical hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    key_parts = [p for p in object_model.parts if p.name.startswith("key_")]
    key_joints = [j for j in object_model.articulations if j.name.startswith("body_to_key_")]
    ctx.check("dense keypad has 24 moving keys", len(key_parts) == 24 and len(key_joints) == 24)

    sample_key = object_model.get_part("key_2_1")
    sample_joint = object_model.get_articulation("body_to_key_2_1")
    ctx.expect_contact(
        sample_key,
        body,
        elem_a="key_cap",
        elem_b="control_panel",
        contact_tol=0.001,
        name="key cap rests on panel face",
    )
    rest_pos = ctx.part_world_position(sample_key)
    with ctx.pose({sample_joint: 0.006}):
        pressed_pos = ctx.part_world_position(sample_key)
    ctx.check(
        "key plunger travels into control panel",
        rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.005,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
