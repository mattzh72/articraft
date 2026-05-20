from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="compact_dorm_microwave")

    enamel = Material("warm_white_enamel", color=(0.86, 0.84, 0.78, 1.0))
    shadow = Material("oven_shadow_black", color=(0.015, 0.015, 0.014, 1.0))
    glass = Material("smoky_window_glass", color=(0.08, 0.12, 0.15, 0.72))
    charcoal = Material("charcoal_plastic", color=(0.05, 0.055, 0.06, 1.0))
    silver = Material("brushed_silver", color=(0.62, 0.61, 0.57, 1.0))
    ink = Material("printed_black", color=(0.0, 0.0, 0.0, 1.0))

    body_w = 0.45
    body_d = 0.34
    body_h = 0.26

    cabinet = model.part("cabinet")
    # A connected, boxy sheet-metal oven shell with a dark front cavity and a
    # fixed control strip to the right of the door.
    cabinet.visual(
        Box((body_w, body_d, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, body_h - 0.0175)),
        material=enamel,
        name="top_shell",
    )
    cabinet.visual(
        Box((body_w, body_d, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=enamel,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((0.035, body_d, body_h)),
        origin=Origin(xyz=(-0.2075, 0.0, body_h / 2.0)),
        material=enamel,
        name="side_shell_0",
    )
    cabinet.visual(
        Box((0.035, body_d, body_h)),
        origin=Origin(xyz=(0.2075, 0.0, body_h / 2.0)),
        material=enamel,
        name="side_shell_1",
    )
    cabinet.visual(
        Box((body_w, 0.025, body_h)),
        origin=Origin(xyz=(0.0, 0.1575, body_h / 2.0)),
        material=enamel,
        name="rear_shell",
    )
    cabinet.visual(
        Box((0.335, 0.020, 0.032)),
        origin=Origin(xyz=(-0.045, -0.160, 0.244)),
        material=enamel,
        name="front_lintel",
    )
    cabinet.visual(
        Box((0.335, 0.020, 0.032)),
        origin=Origin(xyz=(-0.045, -0.160, 0.016)),
        material=enamel,
        name="front_sill",
    )
    cabinet.visual(
        Box((0.032, 0.020, 0.210)),
        origin=Origin(xyz=(-0.209, -0.160, 0.130)),
        material=enamel,
        name="hinge_jamb",
    )
    cabinet.visual(
        Box((0.016, 0.010, 0.070)),
        origin=Origin(xyz=(-0.205, -0.170, 0.197)),
        material=silver,
        name="fixed_hinge_leaf_0",
    )
    cabinet.visual(
        Box((0.016, 0.010, 0.070)),
        origin=Origin(xyz=(-0.205, -0.170, 0.063)),
        material=silver,
        name="fixed_hinge_leaf_1",
    )
    cabinet.visual(
        Box((0.105, 0.026, 0.220)),
        origin=Origin(xyz=(0.1625, -0.157, 0.130)),
        material=enamel,
        name="control_strip",
    )
    cabinet.visual(
        Box((0.296, 0.010, 0.168)),
        origin=Origin(xyz=(-0.045, 0.125, 0.135)),
        material=shadow,
        name="dark_cavity_back",
    )
    cabinet.visual(
        Box((0.296, 0.230, 0.012)),
        origin=Origin(xyz=(-0.045, -0.035, 0.041)),
        material=shadow,
        name="dark_cavity_floor",
    )
    cabinet.visual(
        Cylinder(radius=0.105, length=0.007),
        origin=Origin(xyz=(-0.045, -0.045, 0.0505)),
        material=glass,
        name="glass_turntable",
    )
    for i, z in enumerate((0.085, 0.108, 0.131, 0.154, 0.177)):
        cabinet.visual(
            Box((0.003, 0.105, 0.006)),
            origin=Origin(xyz=(0.226, 0.025, z)),
            material=shadow,
            name=f"side_vent_{i}",
        )

    door = model.part("door")
    door.visual(
        Box((0.315, 0.020, 0.215)),
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        material=charcoal,
        name="door_panel",
    )
    door.visual(
        Box((0.208, 0.004, 0.128)),
        origin=Origin(xyz=(0.142, -0.012, 0.006)),
        material=glass,
        name="window_pane",
    )
    door.visual(
        Box((0.018, 0.014, 0.160)),
        origin=Origin(xyz=(0.292, -0.017, 0.0)),
        material=silver,
        name="pull_lip",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=silver,
        name="hinge_barrel_0",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=silver,
        name="hinge_barrel_1",
    )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=0.015, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=charcoal,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.039, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=silver,
        name="dial_cap",
    )
    knob.visual(
        Box((0.006, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, 0.019, 0.038)),
        material=ink,
        name="pointer_mark",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.205, -0.185, 0.130)),
        # The closed door extends along local +X from the hinge line.  A
        # negative vertical axis makes positive q swing the free edge outward
        # toward the viewer/front (-Y).
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.92),
    )
    model.articulation(
        "knob_shaft",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        # Rotate the joint frame so the knob's local +Z shaft points outward
        # from the front panel along world -Y.
        origin=Origin(xyz=(0.1625, -0.170, 0.145), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0, lower=0.0, upper=5.2),
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

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    knob = object_model.get_part("knob")
    door_hinge = object_model.get_articulation("door_hinge")
    knob_shaft = object_model.get_articulation("knob_shaft")

    with ctx.pose({door_hinge: 0.0, knob_shaft: 0.0}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0005,
            name="closed door seats against the oven face",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.14,
            elem_a="door_panel",
            elem_b="dark_cavity_back",
            name="door covers the oven opening",
        )
        ctx.expect_contact(
            knob,
            cabinet,
            elem_a="shaft",
            elem_b="control_strip",
            contact_tol=0.001,
            name="timer shaft is seated on the control strip",
        )

    rest_door_box = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.45}):
        open_door_box = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door swings outward on vertical hinge",
        rest_door_box is not None
        and open_door_box is not None
        and open_door_box[0][1] < rest_door_box[0][1] - 0.12,
        details=f"rest={rest_door_box}, open={open_door_box}",
    )

    rest_mark_box = ctx.part_element_world_aabb(knob, elem="pointer_mark")
    with ctx.pose({knob_shaft: pi / 2.0}):
        turned_mark_box = ctx.part_element_world_aabb(knob, elem="pointer_mark")
    ctx.check(
        "timer pointer rotates around its own shaft",
        rest_mark_box is not None
        and turned_mark_box is not None
        and abs(
            ((rest_mark_box[0][0] + rest_mark_box[1][0]) / 2.0)
            - ((turned_mark_box[0][0] + turned_mark_box[1][0]) / 2.0)
        )
        > 0.010,
        details=f"rest={rest_mark_box}, turned={turned_mark_box}",
    )

    return ctx.report()


object_model = build_object_model()
