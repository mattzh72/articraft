from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="dorm_microwave")

    body_color = model.material("body_color", rgba=(0.18, 0.19, 0.21, 1.0))
    bezel_color = model.material("bezel_color", rgba=(0.11, 0.12, 0.13, 1.0))
    cavity_color = model.material("cavity_color", rgba=(0.72, 0.74, 0.76, 1.0))
    glass_color = model.material("glass_color", rgba=(0.18, 0.22, 0.25, 0.45))
    trim_color = model.material("trim_color", rgba=(0.50, 0.53, 0.56, 1.0))
    knob_color = model.material("knob_color", rgba=(0.83, 0.84, 0.86, 1.0))
    pointer_color = model.material("pointer_color", rgba=(0.94, 0.18, 0.14, 1.0))

    width = 0.49
    depth = 0.36
    height = 0.29
    wall = 0.018
    control_strip_width = 0.105
    seam = 0.002

    door_width = width - (2.0 * wall) - control_strip_width
    door_height = height - (2.0 * wall)
    door_thickness = 0.020

    body = model.part("body")
    # Back shell and cavity walls.
    body.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, height / 2.0)),
        material=body_color,
        name="back_wall",
    )
    body.visual(
        Box((width, depth - wall, wall)),
        origin=Origin(xyz=(0.0, -wall / 2.0, wall / 2.0)),
        material=body_color,
        name="floor_panel",
    )
    body.visual(
        Box((width, depth - wall, wall)),
        origin=Origin(xyz=(0.0, -wall / 2.0, height - wall / 2.0)),
        material=body_color,
        name="ceiling_panel",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=body_color,
        name="right_wall",
    )
    # Front bezel around the door opening.
    body.visual(
        Box((wall + seam, wall, door_height + seam)),
        origin=Origin(
            xyz=(-width / 2.0 + wall / 2.0, depth / 2.0 - wall / 2.0, wall + door_height / 2.0)
        ),
        material=bezel_color,
        name="left_bezel",
    )
    body.visual(
        Box((door_width + seam, wall, wall + seam)),
        origin=Origin(
            xyz=(-width / 2.0 + wall + door_width / 2.0, depth / 2.0 - wall / 2.0, height - wall / 2.0)
        ),
        material=bezel_color,
        name="top_bezel",
    )
    body.visual(
        Box((door_width + seam, wall, wall + seam)),
        origin=Origin(
            xyz=(-width / 2.0 + wall + door_width / 2.0, depth / 2.0 - wall / 2.0, wall / 2.0)
        ),
        material=bezel_color,
        name="bottom_bezel",
    )
    body.visual(
        Box((control_strip_width + seam, wall, door_height + seam)),
        origin=Origin(
            xyz=(width / 2.0 - wall - control_strip_width / 2.0, depth / 2.0 - wall / 2.0, wall + door_height / 2.0)
        ),
        material=trim_color,
        name="control_strip",
    )
    # Interior back panel tint so the cavity reads as an oven chamber.
    body.visual(
        Box((width - (2.0 * wall), 0.003, height - (2.0 * wall))),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall + 0.0015, height / 2.0)),
        material=cavity_color,
        name="cavity_back_panel",
    )

    door = model.part("door")
    frame_bar = 0.032
    latch_bar = 0.040
    glass_overlap = 0.006
    handle_width = 0.016
    handle_depth = 0.018
    handle_height = 0.140

    door.visual(
        Box((frame_bar, door_thickness, door_height)),
        origin=Origin(xyz=(frame_bar / 2.0, door_thickness / 2.0, door_height / 2.0)),
        material=trim_color,
        name="hinge_stile",
    )
    door.visual(
        Box((latch_bar, door_thickness, door_height)),
        origin=Origin(xyz=(door_width - latch_bar / 2.0, door_thickness / 2.0, door_height / 2.0)),
        material=trim_color,
        name="latch_stile",
    )
    door.visual(
        Box((door_width, door_thickness, frame_bar)),
        origin=Origin(xyz=(door_width / 2.0, door_thickness / 2.0, frame_bar / 2.0)),
        material=trim_color,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width, door_thickness, frame_bar)),
        origin=Origin(xyz=(door_width / 2.0, door_thickness / 2.0, door_height - frame_bar / 2.0)),
        material=trim_color,
        name="top_rail",
    )
    door.visual(
        Box(
            (
                door_width - frame_bar - latch_bar + glass_overlap,
                0.006,
                door_height - (2.0 * frame_bar) + glass_overlap,
            )
        ),
        origin=Origin(
            xyz=(
                frame_bar + (door_width - frame_bar - latch_bar + glass_overlap) / 2.0 - glass_overlap / 2.0,
                0.008,
                frame_bar + (door_height - (2.0 * frame_bar) + glass_overlap) / 2.0 - glass_overlap / 2.0,
            )
        ),
        material=glass_color,
        name="door_glass",
    )
    door.visual(
        Box((handle_width, handle_depth, handle_height)),
        origin=Origin(
            xyz=(door_width - latch_bar / 2.0, door_thickness + handle_depth / 2.0, door_height / 2.0)
        ),
        material=knob_color,
        name="door_handle",
    )

    knob = model.part("timer_knob")
    knob_radius = 0.028
    knob_depth = 0.018
    shaft_radius = 0.010
    shaft_depth = 0.010

    knob.visual(
        Cylinder(radius=shaft_radius, length=shaft_depth),
        origin=Origin(xyz=(0.0, shaft_depth / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=trim_color,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=knob_radius, length=knob_depth),
        origin=Origin(xyz=(0.0, shaft_depth + knob_depth / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=knob_color,
        name="knob_body",
    )
    knob.visual(
        Box((0.006, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, shaft_depth + knob_depth + 0.002, knob_radius * 0.72)),
        material=pointer_color,
        name="knob_pointer",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-width / 2.0 + wall, depth / 2.0, wall)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "knob_turn",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(width / 2.0 - wall - control_strip_width / 2.0, depth / 2.0, height * 0.57)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=5.2,
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
    knob = object_model.get_part("timer_knob")
    door_hinge = object_model.get_articulation("door_hinge")
    knob_turn = object_model.get_articulation("knob_turn")

    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="door closes flush with microwave face",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.20,
        name="door covers the front opening",
    )
    ctx.expect_gap(
        knob,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="timer knob mounts on the control strip",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(lower, upper))

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.2}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward on its side hinge",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > rest_door_aabb[1][1] + 0.12,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_pointer_center = _aabb_center(ctx.part_element_world_aabb(knob, elem="knob_pointer"))
    with ctx.pose({knob_turn: pi / 2.0}):
        turned_pointer_center = _aabb_center(ctx.part_element_world_aabb(knob, elem="knob_pointer"))
    ctx.check(
        "timer knob rotates about its short shaft",
        rest_pointer_center is not None
        and turned_pointer_center is not None
        and turned_pointer_center[0] > rest_pointer_center[0] + 0.012
        and turned_pointer_center[2] < rest_pointer_center[2] - 0.012,
        details=f"rest={rest_pointer_center}, turned={turned_pointer_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
