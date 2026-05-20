from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _bowl_shell_mesh():
    """Thin transparent locking bowl with open ingredient volume."""
    outer_profile = [
        (0.063, 0.000),  # underside socket ring clears the base coupling
        (0.068, 0.030),
        (0.120, 0.040),
        (0.142, 0.075),
        (0.148, 0.230),
        (0.156, 0.255),  # rolled upper lip
    ]
    inner_profile = [
        (0.052, 0.004),
        (0.054, 0.030),
        (0.094, 0.048),
        (0.126, 0.082),
        (0.133, 0.225),
        (0.140, 0.247),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ),
        "bowl_shell",
    )


def _feed_tube_mesh():
    """Rectangular hollow feed tube with a flanged base."""
    tube = (
        cq.Workplane("XY")
        .rect(0.096, 0.076)
        .rect(0.066, 0.046)
        .extrude(0.225)
    )
    flange = (
        cq.Workplane("XY")
        .rect(0.142, 0.118)
        .rect(0.072, 0.052)
        .extrude(0.016)
    )
    rim = (
        cq.Workplane("XY")
        .workplane(offset=0.225)
        .rect(0.106, 0.086)
        .rect(0.066, 0.046)
        .extrude(0.012)
    )
    return mesh_from_cadquery(tube.union(flange).union(rim), "feed_tube")


def _slicing_disc_mesh():
    """Annular slicing disc and raised hub, both clear the center shaft."""
    disc = cq.Workplane("XY").circle(0.107).circle(0.018).extrude(0.006)
    hub = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .circle(0.034)
        .circle(0.014)
        .extrude(0.025)
    )
    return mesh_from_cadquery(disc.union(hub), "slicing_disc")


def _locking_socket_mesh():
    """Open annular bayonet socket that clears the driven base coupling."""
    socket = cq.Workplane("XY").circle(0.071).circle(0.049).extrude(0.028)
    return mesh_from_cadquery(socket, "locking_socket")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxy_food_processor")

    white_plastic = model.material("white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.045, 0.050, 0.055, 1.0))
    rubber = model.material("rubber", rgba=(0.020, 0.020, 0.018, 1.0))
    translucent = model.material("clear_bowl", rgba=(0.70, 0.90, 1.00, 0.38))
    grey_plastic = model.material("grey_plastic", rgba=(0.55, 0.57, 0.58, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.78, 0.79, 0.76, 1.0))
    button_blue = model.material("button_blue", rgba=(0.12, 0.23, 0.34, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.420, 0.340, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=white_plastic,
        name="base_body",
    )
    base.visual(
        Box((0.380, 0.300, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=rubber,
        name="bottom_foot",
    )
    base.visual(
        Box((0.245, 0.006, 0.150)),
        origin=Origin(xyz=(0.0, -0.1730, 0.112)),
        material=dark_plastic,
        name="keypad_plate",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.035),
        origin=Origin(xyz=(0.0, 0.035, 0.2375)),
        material=grey_plastic,
        name="bowl_coupling",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.300),
        origin=Origin(xyz=(0.0, 0.035, 0.370)),
        material=brushed_metal,
        name="center_shaft",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.0, 0.035, 0.225)),
        material=grey_plastic,
        name="coupling_flange",
    )

    bowl = model.part("bowl")
    bowl.visual(_bowl_shell_mesh(), material=translucent, name="bowl_shell")
    bowl.visual(
        _locking_socket_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=translucent,
        name="locking_socket",
    )
    for idx, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radius = 0.084
        bowl.visual(
            Box((0.046, 0.017, 0.012)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.006),
                rpy=(0.0, 0.0, angle),
            ),
            material=translucent,
            name=f"lock_lug_{idx}",
        )
    bowl.visual(
        Box((0.028, 0.030, 0.045)),
        origin=Origin(xyz=(0.158, 0.000, 0.125)),
        material=translucent,
        name="handle_upper_mount",
    )
    bowl.visual(
        Box((0.028, 0.030, 0.045)),
        origin=Origin(xyz=(0.153, 0.000, 0.205)),
        material=translucent,
        name="handle_lower_mount",
    )
    bowl.visual(
        Box((0.028, 0.040, 0.125)),
        origin=Origin(xyz=(0.186, 0.000, 0.165)),
        material=translucent,
        name="handle_grip",
    )

    feed_tube = model.part("feed_tube")
    feed_tube.visual(_feed_tube_mesh(), material=translucent, name="tube_shell")

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.054, 0.034, 0.280)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=grey_plastic,
        name="pusher_shaft",
    )
    pusher.visual(
        Box((0.086, 0.064, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.254)),
        material=grey_plastic,
        name="pusher_cap",
    )
    for idx, (x, y, size) in enumerate(
        (
            (0.029, 0.0, (0.006, 0.012, 0.220)),
            (-0.029, 0.0, (0.006, 0.012, 0.220)),
            (0.0, 0.019, (0.012, 0.006, 0.220)),
            (0.0, -0.019, (0.012, 0.006, 0.220)),
        )
    ):
        pusher.visual(
            Box(size),
            origin=Origin(xyz=(x, y, 0.125)),
            material=grey_plastic,
            name=f"guide_rail_{idx}",
        )

    disc = model.part("slicing_disc")
    disc.visual(_slicing_disc_mesh(), material=brushed_metal, name="disc_plate")
    for idx, angle in enumerate((math.radians(17), math.radians(197))):
        disc.visual(
            Box((0.078, 0.012, 0.004)),
            origin=Origin(
                xyz=(0.043 * math.cos(angle), 0.043 * math.sin(angle), 0.009),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_metal,
            name=f"slice_blade_{idx}",
        )

    dial = model.part("selector_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.074,
            0.028,
            body_style="skirted",
            top_diameter=0.058,
            grip=KnobGrip(style="fluted", count=22, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "selector_dial",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey_plastic,
        name="dial_cap",
    )

    buttons = []
    button_x = -0.070
    for idx in range(5):
        button = model.part(f"button_{idx}")
        z = 0.057 + idx * 0.027
        button.visual(
            Box((0.050, 0.010, 0.020)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_blue,
            name="button_cap",
        )
        buttons.append((button, button_x, z))

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.035, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    model.articulation(
        "bowl_to_feed_tube",
        ArticulationType.FIXED,
        parent=bowl,
        child=feed_tube,
        origin=Origin(xyz=(0.092, 0.032, 0.255)),
    )
    model.articulation(
        "feed_tube_to_pusher",
        ArticulationType.PRISMATIC,
        parent=feed_tube,
        child=pusher,
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.140),
    )
    model.articulation(
        "base_to_slicing_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(0.0, 0.035, 0.315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=40.0),
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.085, -0.176, 0.112)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )
    for idx, (button, x, z) in enumerate(buttons):
        model.articulation(
            f"base_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x, -0.176, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=0.08, lower=0.0, upper=0.006),
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

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    feed_tube = object_model.get_part("feed_tube")
    pusher = object_model.get_part("pusher")
    disc = object_model.get_part("slicing_disc")
    dial = object_model.get_part("selector_dial")

    ctx.allow_overlap(
        bowl,
        feed_tube,
        elem_a="bowl_shell",
        elem_b="tube_shell",
        reason="The feed-chute flange is intentionally seated a few millimeters into the transparent bowl rim.",
    )

    ctx.check(
        "five separate button controls",
        all(object_model.get_part(f"button_{idx}") is not None for idx in range(5)),
        details="Expected five authored button parts.",
    )
    ctx.check(
        "continuous bowl coupling",
        object_model.get_articulation("base_to_bowl").articulation_type == ArticulationType.CONTINUOUS,
        details="The locking bowl should twist continuously on the base coupling.",
    )
    ctx.check(
        "continuous slicing disc",
        object_model.get_articulation("base_to_slicing_disc").articulation_type == ArticulationType.CONTINUOUS,
        details="The slicing disc should rotate on the vertical shaft.",
    )
    ctx.check(
        "continuous selector dial",
        object_model.get_articulation("base_to_selector_dial").articulation_type == ArticulationType.CONTINUOUS,
        details="The selector dial should rotate continuously.",
    )
    ctx.expect_within(
        pusher,
        feed_tube,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="tube_shell",
        margin=0.002,
        name="pusher shaft fits inside feed tube clearance",
    )
    ctx.expect_overlap(
        pusher,
        feed_tube,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="tube_shell",
        min_overlap=0.120,
        name="pusher remains inserted when seated",
    )
    ctx.expect_gap(
        feed_tube,
        bowl,
        axis="z",
        positive_elem="tube_shell",
        negative_elem="bowl_shell",
        max_gap=0.001,
        max_penetration=0.006,
        name="feed tube flange is shallowly seated in bowl rim",
    )
    slide = object_model.get_articulation("feed_tube_to_pusher")
    rest_pos = ctx.part_world_position(pusher)
    with ctx.pose({slide: 0.140}):
        ctx.expect_within(
            pusher,
            feed_tube,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="tube_shell",
            margin=0.002,
            name="raised pusher stays guided by tube",
        )
        ctx.expect_overlap(
            pusher,
            feed_tube,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="tube_shell",
            min_overlap=0.075,
            name="raised pusher still has retained insertion",
        )
        raised_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.120,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.expect_within(
        disc,
        bowl,
        axes="xy",
        inner_elem="disc_plate",
        outer_elem="bowl_shell",
        margin=0.002,
        name="slicing disc sits within bowl footprint",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="lock_lug_0",
        negative_elem="base_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="bowl locking lugs seat on base top",
    )
    ctx.expect_contact(
        dial,
        base,
        elem_a="dial_cap",
        elem_b="keypad_plate",
        contact_tol=0.003,
        name="selector dial mounts on keypad",
    )

    for idx in range(5):
        button = object_model.get_part(f"button_{idx}")
        joint = object_model.get_articulation(f"base_to_button_{idx}")
        at_rest = ctx.part_world_position(button)
        ctx.expect_contact(
            button,
            base,
            elem_a="button_cap",
            elem_b="keypad_plate",
            contact_tol=0.003,
            name=f"button_{idx} sits on keypad",
        )
        with ctx.pose({joint: 0.006}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} presses inward",
            at_rest is not None and pressed is not None and pressed[1] > at_rest[1] + 0.004,
            details=f"rest={at_rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
