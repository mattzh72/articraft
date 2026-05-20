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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_microwave")

    cabinet_color = model.material("cabinet_cream", rgba=(0.93, 0.89, 0.79, 1.0))
    trim_color = model.material("retro_teal", rgba=(0.60, 0.71, 0.69, 1.0))
    knob_color = model.material("knob_ivory", rgba=(0.95, 0.93, 0.86, 1.0))
    glass_color = model.material("smoked_glass", rgba=(0.18, 0.23, 0.28, 0.40))
    metal_color = model.material("brushed_metal", rgba=(0.76, 0.77, 0.80, 1.0))

    cabinet_width = 0.52
    cabinet_depth = 0.40
    cabinet_height = 0.32
    shell_t = 0.018
    back_t = 0.014
    control_face_t = 0.012

    door_width = 0.375
    door_height = 0.272
    door_depth = 0.034
    door_frame_t = 0.030
    door_gap = 0.0

    left_hinge_x = -0.25
    door_right_x = left_hinge_x + door_width
    seam_gap = 0.015
    control_left_x = door_right_x + seam_gap
    control_right_x = cabinet_width / 2.0 - shell_t
    control_width = control_right_x - control_left_x
    control_center_x = (control_left_x + control_right_x) / 2.0

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height / 2.0 - shell_t / 2.0)),
        material=cabinet_color,
        name="top_shell",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, -cabinet_height / 2.0 + shell_t / 2.0)),
        material=cabinet_color,
        name="bottom_shell",
    )
    cabinet.visual(
        Box((shell_t, cabinet_depth, cabinet_height - 2.0 * shell_t)),
        origin=Origin(xyz=(-cabinet_width / 2.0 + shell_t / 2.0, 0.0, 0.0)),
        material=cabinet_color,
        name="left_shell",
    )
    cabinet.visual(
        Box((shell_t, cabinet_depth, cabinet_height - 2.0 * shell_t)),
        origin=Origin(xyz=(cabinet_width / 2.0 - shell_t / 2.0, 0.0, 0.0)),
        material=cabinet_color,
        name="right_shell",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * shell_t, back_t, cabinet_height - 2.0 * shell_t)),
        origin=Origin(xyz=(0.0, -cabinet_depth / 2.0 + back_t / 2.0, 0.0)),
        material=cabinet_color,
        name="rear_shell",
    )
    cabinet.visual(
        Box((control_width, control_face_t, cabinet_height - 2.0 * shell_t)),
        origin=Origin(
            xyz=(
                control_center_x,
                cabinet_depth / 2.0 - control_face_t / 2.0,
                0.0,
            )
        ),
        material=trim_color,
        name="control_panel",
    )
    cabinet.visual(
        Box(
            (
                shell_t,
                cabinet_depth - back_t - control_face_t,
                cabinet_height - 2.0 * shell_t,
            )
        ),
        origin=Origin(
            xyz=(
                control_left_x - shell_t / 2.0,
                0.001,
                0.0,
            )
        ),
        material=cabinet_color,
        name="control_partition",
    )

    door = model.part("door")
    door.visual(
        Box((door_frame_t, door_depth, door_height)),
        origin=Origin(xyz=(door_frame_t / 2.0, 0.0, 0.0)),
        material=trim_color,
        name="hinge_stile",
    )
    door.visual(
        Box((door_frame_t, door_depth, door_height)),
        origin=Origin(xyz=(door_width - door_frame_t / 2.0, 0.0, 0.0)),
        material=trim_color,
        name="latch_stile",
    )
    door.visual(
        Box((door_width - 2.0 * door_frame_t, door_depth, door_frame_t)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, door_height / 2.0 - door_frame_t / 2.0)),
        material=trim_color,
        name="top_rail",
    )
    door.visual(
        Box((door_width - 2.0 * door_frame_t, door_depth, door_frame_t)),
        origin=Origin(xyz=(door_width / 2.0, 0.0, -door_height / 2.0 + door_frame_t / 2.0)),
        material=trim_color,
        name="bottom_rail",
    )

    glass = model.part("door_glass")
    glass.visual(
        Box((door_width - 0.020, 0.004, door_height - 0.020)),
        origin=Origin(xyz=(door_width / 2.0, -door_depth / 2.0 - 0.002, 0.0)),
        material=glass_color,
        name="window_pane",
    )

    handle = model.part("door_handle")
    handle_x = door_width - door_frame_t / 2.0
    post_len = 0.016
    post_radius = 0.0045
    grip_radius = 0.007
    handle.visual(
        Cylinder(radius=grip_radius, length=0.165),
        origin=Origin(
            xyz=(handle_x, door_depth / 2.0 + post_len + grip_radius, 0.0)
        ),
        material=metal_color,
        name="grip_bar",
    )
    handle.visual(
        Cylinder(radius=post_radius, length=post_len),
        origin=Origin(
            xyz=(handle_x, door_depth / 2.0 + post_len / 2.0, 0.060),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_color,
        name="upper_post",
    )
    handle.visual(
        Cylinder(radius=post_radius, length=post_len),
        origin=Origin(
            xyz=(handle_x, door_depth / 2.0 + post_len / 2.0, -0.060),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal_color,
        name="lower_post",
    )

    def add_knob(part_name: str, pointer_name: str) -> None:
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal_color,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.032, length=0.022),
            origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=knob_color,
            name="body",
        )
        knob.visual(
            Cylinder(radius=0.021, length=0.008),
            origin=Origin(xyz=(0.0, 0.036, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=trim_color,
            name="cap",
        )
        knob.visual(
            Box((0.010, 0.004, 0.014)),
            origin=Origin(xyz=(0.0, 0.042, 0.020)),
            material=metal_color,
            name=pointer_name,
        )

    add_knob("upper_knob", "pointer")
    add_knob("lower_knob", "pointer")

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(
            xyz=(
                left_hinge_x,
                cabinet_depth / 2.0 + door_gap + door_depth / 2.0,
                0.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.9,
        ),
    )
    model.articulation(
        "door_to_glass",
        ArticulationType.FIXED,
        parent=door,
        child=glass,
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.FIXED,
        parent=door,
        child=handle,
    )
    model.articulation(
        "cabinet_to_upper_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=model.get_part("upper_knob"),
        origin=Origin(xyz=(control_center_x, cabinet_depth / 2.0, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    model.articulation(
        "cabinet_to_lower_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=model.get_part("lower_knob"),
        origin=Origin(xyz=(control_center_x, cabinet_depth / 2.0, -0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
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

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    glass = object_model.get_part("door_glass")
    handle = object_model.get_part("door_handle")
    upper_knob = object_model.get_part("upper_knob")
    lower_knob = object_model.get_part("lower_knob")

    door_hinge = object_model.get_articulation("cabinet_to_door")
    upper_joint = object_model.get_articulation("cabinet_to_upper_knob")
    lower_joint = object_model.get_articulation("cabinet_to_lower_knob")

    def center_x(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) / 2.0

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            name="door closes flush to the cabinet front",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            min_overlap=0.24,
            name="door covers the front cooking opening",
        )

    ctx.expect_contact(
        glass,
        door,
        name="glass pane is seated against the door frame",
    )
    ctx.expect_contact(
        handle,
        door,
        name="handle mounts directly to the door face",
    )
    ctx.expect_contact(
        upper_knob,
        cabinet,
        name="upper knob mounts to the control panel",
    )
    ctx.expect_contact(
        lower_knob,
        cabinet,
        name="lower knob mounts to the control panel",
    )
    ctx.expect_origin_gap(
        upper_knob,
        lower_knob,
        axis="z",
        min_gap=0.11,
        max_gap=0.15,
        name="control knobs are vertically stacked",
    )
    ctx.expect_origin_distance(
        upper_knob,
        lower_knob,
        axes="x",
        max_dist=0.005,
        name="stacked knobs share the same control column",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        open_door_aabb = ctx.part_world_aabb(door)
    closed_max_y = None if closed_door_aabb is None else closed_door_aabb[1][1]
    open_max_y = None if open_door_aabb is None else open_door_aabb[1][1]
    ctx.check(
        "door swings outward on its side hinge",
        closed_max_y is not None and open_max_y is not None and open_max_y > closed_max_y + 0.12,
        details=f"closed_max_y={closed_max_y}, open_max_y={open_max_y}",
    )

    upper_pointer_rest = ctx.part_element_world_aabb(upper_knob, elem="pointer")
    with ctx.pose({upper_joint: 1.15}):
        upper_pointer_turned = ctx.part_element_world_aabb(upper_knob, elem="pointer")
    ctx.check(
        "upper knob pointer rotates around the shaft",
        center_x(upper_pointer_rest) is not None
        and center_x(upper_pointer_turned) is not None
        and center_x(upper_pointer_turned) > center_x(upper_pointer_rest) + 0.012,
        details=(
            f"rest_center_x={center_x(upper_pointer_rest)}, "
            f"turned_center_x={center_x(upper_pointer_turned)}"
        ),
    )

    lower_pointer_rest = ctx.part_element_world_aabb(lower_knob, elem="pointer")
    with ctx.pose({lower_joint: 0.95}):
        lower_pointer_turned = ctx.part_element_world_aabb(lower_knob, elem="pointer")
    ctx.check(
        "lower knob pointer rotates around the shaft",
        center_x(lower_pointer_rest) is not None
        and center_x(lower_pointer_turned) is not None
        and center_x(lower_pointer_turned) > center_x(lower_pointer_rest) + 0.010,
        details=(
            f"rest_center_x={center_x(lower_pointer_rest)}, "
            f"turned_center_x={center_x(lower_pointer_turned)}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
