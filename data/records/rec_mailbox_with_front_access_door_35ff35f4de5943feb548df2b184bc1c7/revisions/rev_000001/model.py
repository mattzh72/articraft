from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 0.38
BODY_HEIGHT = 0.42
BODY_DEPTH = 0.14
BACK_THICKNESS = 0.004
SHELL_THICKNESS = 0.022
DOOR_HEIGHT = BODY_HEIGHT - 2.0 * SHELL_THICKNESS - 0.004
DOOR_WIDTH = BODY_WIDTH - 2.0 * SHELL_THICKNESS - 0.005
HINGE_X = BODY_DEPTH - 0.004
HINGE_Y = -(BODY_WIDTH * 0.5) + SHELL_THICKNESS
HINGE_Z = SHELL_THICKNESS + 0.002


def _door_profile(
    width: float,
    *,
    edge_proud: float = 0.007,
    center_proud: float = 0.029,
    thickness: float = 0.016,
    samples: int = 24,
) -> list[tuple[float, float]]:
    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        y = width * t
        s = math.sin(math.pi * t) ** 1.1
        x_outer = edge_proud + (center_proud - edge_proud) * s
        outer.append((x_outer, y))
    for index in range(samples, -1, -1):
        t = index / samples
        y = width * t
        s = math.sin(math.pi * t) ** 1.1
        x_outer = edge_proud + (center_proud - edge_proud) * s
        inner.append((x_outer - thickness, y))
    return outer + inner


def _button_cap_origin() -> Origin:
    return Origin(
        xyz=(0.0035, 0.0, 0.0),
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="designer_wall_mailbox")

    body_finish = model.material("body_finish", rgba=(0.20, 0.21, 0.22, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    button_finish = model.material("button_finish", rgba=(0.78, 0.72, 0.60, 1.0))
    interior_finish = model.material("interior_finish", rgba=(0.32, 0.33, 0.35, 1.0))

    body = model.part("body")
    inner_width = BODY_WIDTH - 2.0 * SHELL_THICKNESS
    shell_depth = BODY_DEPTH - BACK_THICKNESS

    body.visual(
        Box((BACK_THICKNESS, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BACK_THICKNESS * 0.5, 0.0, BODY_HEIGHT * 0.5)),
        material=body_finish,
        name="back_panel",
    )
    body.visual(
        Box((shell_depth, SHELL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS + shell_depth * 0.5,
                -(BODY_WIDTH * 0.5) + (SHELL_THICKNESS * 0.5),
                BODY_HEIGHT * 0.5,
            )
        ),
        material=body_finish,
        name="left_wall",
    )
    body.visual(
        Box((shell_depth, SHELL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS + shell_depth * 0.5,
                (BODY_WIDTH * 0.5) - (SHELL_THICKNESS * 0.5),
                BODY_HEIGHT * 0.5,
            )
        ),
        material=body_finish,
        name="right_wall",
    )
    body.visual(
        Box((shell_depth, inner_width, SHELL_THICKNESS)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS + shell_depth * 0.5,
                0.0,
                SHELL_THICKNESS * 0.5,
            )
        ),
        material=body_finish,
        name="bottom_tray",
    )
    body.visual(
        Box((shell_depth, inner_width, SHELL_THICKNESS)),
        origin=Origin(
            xyz=(
                BACK_THICKNESS + shell_depth * 0.5,
                0.0,
                BODY_HEIGHT - (SHELL_THICKNESS * 0.5),
            )
        ),
        material=body_finish,
        name="top_shell",
    )
    body.visual(
        Box((BODY_DEPTH + 0.012, BODY_WIDTH + 0.016, 0.008)),
        origin=Origin(xyz=((BODY_DEPTH + 0.012) * 0.5, 0.0, BODY_HEIGHT + 0.004)),
        material=trim_finish,
        name="rain_cap",
    )
    body.visual(
        Box((0.006, 0.014, 0.082)),
        origin=Origin(
            xyz=(
                BODY_DEPTH - 0.018,
                (BODY_WIDTH * 0.5) - SHELL_THICKNESS - 0.007,
                BODY_HEIGHT * 0.5,
            )
        ),
        material=interior_finish,
        name="strike_plate",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_DEPTH + 0.012, BODY_WIDTH + 0.016, BODY_HEIGHT + 0.008)),
        mass=4.5,
        origin=Origin(
            xyz=(
                (BODY_DEPTH + 0.012) * 0.5,
                0.0,
                (BODY_HEIGHT + 0.008) * 0.5,
            )
        ),
    )

    door = model.part("door")
    door_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_door_profile(DOOR_WIDTH), DOOR_HEIGHT),
        "mailbox_curved_door",
    )
    door.visual(
        door_mesh,
        material=body_finish,
        name="door_shell",
    )
    door.visual(
        Box((0.010, 0.014, DOOR_HEIGHT)),
        origin=Origin(xyz=(-0.002, 0.007, DOOR_HEIGHT * 0.5)),
        material=trim_finish,
        name="hinge_stile",
    )
    door.visual(
        Box((0.010, 0.014, DOOR_HEIGHT)),
        origin=Origin(xyz=(-0.002, DOOR_WIDTH - 0.007, DOOR_HEIGHT * 0.5)),
        material=trim_finish,
        name="meeting_edge_strip",
    )
    door.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(
            xyz=(0.015, DOOR_WIDTH - 0.026, DOOR_HEIGHT * 0.52),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_finish,
        name="button_bezel",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.045, DOOR_WIDTH, DOOR_HEIGHT)),
        mass=1.2,
        origin=Origin(xyz=(0.012, DOOR_WIDTH * 0.5, DOOR_HEIGHT * 0.5)),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Cylinder(radius=0.0085, length=0.004),
        origin=_button_cap_origin(),
        material=button_finish,
        name="button_cap",
    )
    latch_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0085, length=0.004),
        mass=0.03,
        origin=_button_cap_origin(),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        # Closed door geometry extends across local +Y from the hinge.
        # -Z makes positive q swing the free edge outward toward +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    model.articulation(
        "door_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=door,
        child=latch_button,
        origin=Origin(xyz=(0.0155, DOOR_WIDTH - 0.026, DOOR_HEIGHT * 0.52)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.005,
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

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    latch_button = object_model.get_part("latch_button")
    door_hinge = object_model.get_articulation("body_to_door")
    button_slide = object_model.get_articulation("door_to_latch_button")

    ctx.check("body part exists", body is not None, details="Missing fixed mailbox body part.")
    ctx.check("door part exists", door is not None, details="Missing hinged door part.")
    ctx.check(
        "latch button part exists",
        latch_button is not None,
        details="Missing latch button part.",
    )

    hinge_axis = tuple(float(v) for v in door_hinge.axis)
    ctx.check(
        "door hinge axis is vertical",
        abs(hinge_axis[0]) < 1e-9 and abs(hinge_axis[1]) < 1e-9 and abs(abs(hinge_axis[2]) - 1.0) < 1e-9,
        details=f"axis={hinge_axis}",
    )

    button_axis = tuple(float(v) for v in button_slide.axis)
    ctx.check(
        "latch button slides inward through the door face",
        button_axis[0] < -0.99 and abs(button_axis[1]) < 1e-9 and abs(button_axis[2]) < 1e-9,
        details=f"axis={button_axis}",
    )

    with ctx.pose({door_hinge: 0.0, button_slide: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.32,
            name="closed door covers the body opening footprint",
        )
        ctx.expect_overlap(
            latch_button,
            door,
            axes="yz",
            min_overlap=0.012,
            elem_a="button_cap",
            elem_b="button_bezel",
            name="latch button stays registered to the door bezel",
        )
        closed_edge = ctx.part_element_world_aabb(door, elem="meeting_edge_strip")
        rest_button = ctx.part_element_world_aabb(latch_button, elem="button_cap")

    with ctx.pose({door_hinge: math.radians(85.0), button_slide: 0.0}):
        open_edge = ctx.part_element_world_aabb(door, elem="meeting_edge_strip")

    ctx.check(
        "door swings outward from the left side hinge",
        closed_edge is not None
        and open_edge is not None
        and open_edge[1][0] > closed_edge[1][0] + 0.10,
        details=f"closed_edge={closed_edge}, open_edge={open_edge}",
    )

    with ctx.pose({door_hinge: 0.0, button_slide: 0.005}):
        pressed_button = ctx.part_element_world_aabb(latch_button, elem="button_cap")

    ctx.check(
        "button depresses into the curved door",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1][0] < rest_button[1][0] - 0.003,
        details=f"rest_button={rest_button}, pressed_button={pressed_button}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
