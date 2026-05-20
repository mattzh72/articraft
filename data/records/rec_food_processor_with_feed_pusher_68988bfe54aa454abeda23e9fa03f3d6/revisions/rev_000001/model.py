from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_FRONT_X = 0.24
BOWL_ORIGIN = (-0.03, 0.0, 0.145)
BOWL_TOP_Z = 0.174
HINGE_X = -0.138
LID_CENTER_X = 0.138
FEED_TUBE_X = 0.200
FEED_TUBE_TOP_Z = 0.120


def make_body_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XY")
        .box(0.42, 0.30, 0.115)
        .translate((0.0, 0.0, 0.0575))
        .edges("|Z")
        .fillet(0.024)
    )
    deck = (
        cq.Workplane("XY")
        .box(0.24, 0.22, 0.030)
        .translate((-0.03, 0.0, 0.130))
        .edges("|Z")
        .fillet(0.010)
    )
    pod = (
        cq.Workplane("XY")
        .box(0.12, 0.24, 0.145)
        .translate((0.18, 0.0, 0.0725))
        .edges("|Z")
        .fillet(0.016)
    )
    body = housing.union(deck).union(pod)

    pocket_specs = (
        (0.0, 0.129, 0.011, 0.044, 0.020),
        (0.0, 0.035, 0.011, 0.044, 0.020),
        (-0.070, 0.082, 0.011, 0.020, 0.044),
        (0.070, 0.082, 0.011, 0.020, 0.044),
    )
    for y, z, dx, dy, dz in pocket_specs:
        pocket = cq.Workplane("XY").box(dx, dy, dz).translate((BODY_FRONT_X - dx / 2.0, y, z))
        body = body.cut(pocket)

    return body


def make_bowl_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .ellipse(0.092, 0.072)
        .workplane(offset=0.060)
        .ellipse(0.122, 0.095)
        .workplane(offset=0.115)
        .ellipse(0.134, 0.104)
        .loft(combine=True)
        .val()
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=0.006)
        .ellipse(0.078, 0.058)
        .workplane(offset=0.054)
        .ellipse(0.118, 0.091)
        .workplane(offset=0.118)
        .ellipse(0.130, 0.100)
        .loft(combine=True)
        .val()
    )
    shell = cq.Workplane("XY").add(outer.cut(inner))

    hinge_mount = cq.Workplane("XY").box(0.020, 0.120, 0.018).translate((-0.132, 0.0, 0.165))
    return shell.union(hinge_mount)


def make_lid_shell_shape() -> cq.Workplane:
    cap = (
        cq.Workplane("XY")
        .ellipse(0.139, 0.109)
        .workplane(offset=0.018)
        .ellipse(0.130, 0.100)
        .workplane(offset=0.016)
        .ellipse(0.116, 0.088)
        .loft(combine=True)
        .faces("<Z")
        .shell(-0.0035)
        .translate((LID_CENTER_X, 0.0, 0.0))
    )
    feed_clear = (
        cq.Workplane("XY")
        .ellipse(0.0335, 0.0235)
        .extrude(0.065)
        .translate((FEED_TUBE_X, 0.0, -0.002))
    )
    return cap.cut(feed_clear)


def make_feed_tube_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").ellipse(0.038, 0.028).extrude(0.110).translate((FEED_TUBE_X, 0.0, 0.010))
    inner = cq.Workplane("XY").ellipse(0.0330, 0.0230).extrude(0.116).translate((FEED_TUBE_X, 0.0, 0.007))
    collar = cq.Workplane("XY").ellipse(0.041, 0.031).extrude(0.012).translate((FEED_TUBE_X, 0.0, 0.108))
    collar_clear = cq.Workplane("XY").ellipse(0.0330, 0.0230).extrude(0.014).translate((FEED_TUBE_X, 0.0, 0.107))
    return outer.cut(inner).union(collar.cut(collar_clear))


def make_pusher_shape() -> cq.Workplane:
    body = cq.Workplane("XY").ellipse(0.031, 0.021).extrude(0.180).translate((0.0, 0.0, -0.025))
    grip = cq.Workplane("XY").ellipse(0.038, 0.026).extrude(0.024).translate((0.0, 0.0, 0.131))
    return body.union(grip)


def make_disc_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(0.009).extrude(0.050).translate((0.0, 0.0, -0.012))
    plate = cq.Workplane("XY").circle(0.086).extrude(0.004)
    hub = cq.Workplane("XY").circle(0.024).extrude(0.018)
    blade_seed = cq.Workplane("XY").box(0.105, 0.016, 0.006).translate((0.012, 0.0, 0.006))
    blade_a = blade_seed.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 24.0)
    blade_b = blade_seed.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 204.0)
    return shaft.union(plate).union(hub).union(blade_a).union(blade_b)


def articulation_name(kind: object) -> str:
    return getattr(kind, "name", str(kind))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="food_processor")

    body_mat = model.material("body_mat", rgba=(0.16, 0.16, 0.18, 1.0))
    pod_accent = model.material("pod_accent", rgba=(0.23, 0.23, 0.25, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.72, 0.77, 0.82, 0.38))
    control_mat = model.material("control_mat", rgba=(0.12, 0.12, 0.13, 1.0))
    button_mat = model.material("button_mat", rgba=(0.88, 0.88, 0.90, 1.0))
    steel_mat = model.material("steel_mat", rgba=(0.78, 0.80, 0.82, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(make_body_shape(), "body_housing"), material=body_mat, name="housing")
    body.visual(
        Box((0.110, 0.210, 0.012)),
        origin=Origin(xyz=(0.185, 0.0, 0.146)),
        material=pod_accent,
        name="pod_top",
    )

    bowl = model.part("bowl")
    bowl.visual(mesh_from_cadquery(make_bowl_shape(), "bowl_shell"), material=clear_smoke, name="bowl_shell")

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(make_lid_shell_shape(), "lid_shell"), material=clear_smoke, name="lid_shell")
    lid.visual(mesh_from_cadquery(make_feed_tube_shape(), "feed_tube"), material=clear_smoke, name="feed_tube")

    pusher = model.part("pusher")
    pusher.visual(mesh_from_cadquery(make_pusher_shape(), "pusher_body"), material=control_mat, name="pusher_body")

    disc = model.part("disc")
    disc.visual(mesh_from_cadquery(make_disc_shape(), "disc_plate"), material=steel_mat, name="disc_plate")

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.042, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_mat,
        name="dial_knob",
    )
    dial.visual(
        Box((0.006, 0.010, 0.026)),
        origin=Origin(xyz=(0.018, 0.0, 0.028)),
        material=button_mat,
        name="dial_pointer",
    )

    button_specs = (
        ("button_0", (0.0, 0.129, 0.014, 0.040, 0.018)),
        ("button_1", (0.0, 0.035, 0.014, 0.040, 0.018)),
        ("button_2", (-0.070, 0.082, 0.014, 0.018, 0.040)),
        ("button_3", (0.070, 0.082, 0.014, 0.018, 0.040)),
    )
    buttons: list[tuple[str, tuple[float, float, float, float, float]]] = list(button_specs)
    for part_name, (y, z, dx, dy, dz) in buttons:
        button = model.part(part_name)
        button.visual(
            Box((dx, dy, dz)),
            origin=Origin(xyz=(0.007, 0.0, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((0.010, dy * 0.70, dz * 0.70)),
            origin=Origin(xyz=(-0.001, 0.0, 0.0)),
            material=control_mat,
            name="button_stem",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BODY_FRONT_X, y, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    model.articulation(
        "body_to_bowl",
        ArticulationType.FIXED,
        parent=body,
        child=bowl,
        origin=Origin(xyz=BOWL_ORIGIN),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, BOWL_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(FEED_TUBE_X, 0.0, FEED_TUBE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=0.065),
    )
    model.articulation(
        "bowl_to_disc",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=disc,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_FRONT_X, 0.0, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    disc = object_model.get_part("disc")

    lid_hinge = object_model.get_articulation("bowl_to_lid")
    lid_limits = lid_hinge.motion_limits
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    pusher_limits = pusher_slide.motion_limits
    disc_spin = object_model.get_articulation("bowl_to_disc")
    dial_spin = object_model.get_articulation("body_to_dial")

    for i in range(4):
        ctx.allow_isolated_part(
            object_model.get_part(f"button_{i}"),
            reason="Each preset button is intentionally modeled with operating clearance inside its control-pod pocket.",
        )
    ctx.allow_isolated_part(
        pusher,
        reason="The feed pusher is intentionally modeled with sliding clearance inside the lid feed tube.",
    )
    ctx.allow_overlap(
        bowl,
        disc,
        elem_a="bowl_shell",
        elem_b="disc_plate",
        reason="The cutting disc is intentionally housed inside the bowl's open prep chamber; the thin shell bowl mesh is treated as occupying the enclosed cavity by overlap QC.",
    )

    def joint_kind_name(joint_name: str) -> str:
        joint = object_model.get_articulation(joint_name)
        kind = joint.articulation_type
        return getattr(kind, "name", str(kind))

    ctx.check(
        "prompt motion types are preserved",
        joint_kind_name("bowl_to_lid") == "REVOLUTE"
        and joint_kind_name("lid_to_pusher") == "PRISMATIC"
        and joint_kind_name("bowl_to_disc") == "CONTINUOUS"
        and joint_kind_name("body_to_dial") == "CONTINUOUS"
        and all(joint_kind_name(f"body_to_button_{i}") == "PRISMATIC" for i in range(4)),
        details=", ".join(
            f"{name}={joint_kind_name(name)}"
            for name in (
                "bowl_to_lid",
                "lid_to_pusher",
                "bowl_to_disc",
                "body_to_dial",
                "body_to_button_0",
                "body_to_button_1",
                "body_to_button_2",
                "body_to_button_3",
            )
        ),
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            bowl,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="bowl_shell",
            max_gap=0.010,
            max_penetration=0.002,
            name="closed lid seats on the bowl rim",
        )
        ctx.expect_overlap(
            lid,
            bowl,
            axes="xy",
            elem_a="lid_shell",
            elem_b="bowl_shell",
            min_overlap=0.180,
            name="closed lid covers the prep bowl opening",
        )
        ctx.expect_within(
            disc,
            bowl,
            axes="xy",
            inner_elem="disc_plate",
            outer_elem="bowl_shell",
            margin=0.010,
            name="cutting disc stays within the bowl footprint",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        closed_aabb = None
        opened_aabb = None
        with ctx.pose({lid_hinge: 0.0}):
            closed_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper}):
            opened_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "rear hinge opens the lid upward",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[1][2] > closed_aabb[1][2] + 0.090,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    if pusher_limits is not None and pusher_limits.upper is not None:
        with ctx.pose({lid_hinge: 0.0, pusher_slide: 0.0}):
            ctx.expect_within(
                pusher,
                lid,
                axes="xy",
                inner_elem="pusher_body",
                outer_elem="feed_tube",
                margin=0.004,
                name="pusher stays centered in the feed tube at rest",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_body",
                elem_b="feed_tube",
                min_overlap=0.020,
                name="resting pusher remains captured in the feed tube",
            )
            rest_pos = ctx.part_world_position(pusher)

        with ctx.pose({lid_hinge: 0.0, pusher_slide: pusher_limits.upper}):
            ctx.expect_within(
                pusher,
                lid,
                axes="xy",
                inner_elem="pusher_body",
                outer_elem="feed_tube",
                margin=0.004,
                name="pressed pusher remains centered in the feed tube",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_body",
                elem_b="feed_tube",
                min_overlap=0.080,
                name="pressed pusher still retains insertion in the feed tube",
            )
            pressed_pos = ctx.part_world_position(pusher)

        ctx.check(
            "pusher travels downward when pressed",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.050,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    for i in range(4):
        joint = object_model.get_articulation(f"body_to_button_{i}")
        limits = joint.motion_limits
        button = object_model.get_part(f"button_{i}")
        if limits is None or limits.upper is None:
            continue
        rest_pos = None
        pressed_pos = None
        with ctx.pose({joint: 0.0}):
            rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{i} presses inward independently",
            rest_pos is not None and pressed_pos is not None and pressed_pos[0] < rest_pos[0] - 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.check(
        "dial and disc remain continuous rotaries",
        articulation_name(disc_spin.articulation_type) == "CONTINUOUS"
        and articulation_name(dial_spin.articulation_type) == "CONTINUOUS",
        details=f"disc={disc_spin.articulation_type}, dial={dial_spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
