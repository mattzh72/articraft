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


def _dims() -> dict[str, float]:
    return {
        "body_w": 0.19,
        "body_d": 0.16,
        "body_h": 0.30,
        "wall_t": 0.0045,
        "bottom_t": 0.006,
        "lid_t": 0.008,
        "lid_w": 0.194,
        "lid_d": 0.165,
        "hinge_r": 0.006,
        "hinge_len": 0.050,
        "lid_knuckle_len": 0.054,
        "pedal_x": 0.046,
        "pedal_pivot_y": -0.084,
        "pedal_pivot_z": 0.045,
        "pedal_w": 0.043,
        "pedal_d": 0.042,
        "pedal_t": 0.008,
        "pedal_axle_len": 0.049,
        "pedal_axle_r": 0.004,
        "pedal_bracket_w": 0.006,
        "pedal_bracket_d": 0.014,
        "pedal_bracket_h": 0.018,
        "lid_open_angle": 1.18,
        "pedal_press_angle": 0.50,
    }


def build_object_model() -> ArticulatedObject:
    d = _dims()
    model = ArticulatedObject(name="slim_bathroom_step_bin")

    body_mat = model.material("body_painted_steel", rgba=(0.94, 0.95, 0.96, 1.0))
    hinge_mat = model.material("hinge_trim", rgba=(0.74, 0.76, 0.78, 1.0))
    pedal_mat = model.material("pedal_black", rgba=(0.16, 0.17, 0.18, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((d["body_w"] - 2.0 * d["wall_t"], d["body_d"] - 2.0 * d["wall_t"], d["bottom_t"])),
        origin=Origin(xyz=(0.0, 0.0, d["bottom_t"] / 2.0)),
        material=body_mat,
        name="bottom_pan",
    )
    shell.visual(
        Box((d["wall_t"], d["body_d"], d["body_h"])),
        origin=Origin(xyz=(-(d["body_w"] / 2.0) + d["wall_t"] / 2.0, 0.0, d["body_h"] / 2.0)),
        material=body_mat,
        name="left_wall",
    )
    shell.visual(
        Box((d["wall_t"], d["body_d"], d["body_h"])),
        origin=Origin(xyz=((d["body_w"] / 2.0) - d["wall_t"] / 2.0, 0.0, d["body_h"] / 2.0)),
        material=body_mat,
        name="right_wall",
    )
    shell.visual(
        Box((d["body_w"] - 2.0 * d["wall_t"], d["wall_t"], d["body_h"])),
        origin=Origin(xyz=(0.0, -(d["body_d"] / 2.0) + d["wall_t"] / 2.0, d["body_h"] / 2.0)),
        material=body_mat,
        name="front_wall",
    )
    shell.visual(
        Box((d["body_w"] - 2.0 * d["wall_t"], d["wall_t"], d["body_h"])),
        origin=Origin(xyz=(0.0, (d["body_d"] / 2.0) - d["wall_t"] / 2.0, d["body_h"] / 2.0)),
        material=body_mat,
        name="rear_wall",
    )

    hinge_y = d["body_d"] / 2.0 + d["hinge_r"]
    hinge_z = d["body_h"] + d["lid_t"] / 2.0
    left_hinge_x = -0.058
    right_hinge_x = 0.058
    shell.visual(
        Box((0.034, 0.012, 0.014)),
        origin=Origin(xyz=(left_hinge_x, 0.086, d["body_h"] + 0.003)),
        material=body_mat,
        name="left_hinge_mount",
    )
    shell.visual(
        Box((0.034, 0.012, 0.014)),
        origin=Origin(xyz=(right_hinge_x, 0.086, d["body_h"] + 0.003)),
        material=body_mat,
        name="right_hinge_mount",
    )
    shell.visual(
        Cylinder(radius=d["hinge_r"], length=d["hinge_len"]),
        origin=Origin(xyz=(left_hinge_x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_mat,
        name="left_hinge_knuckle",
    )
    shell.visual(
        Cylinder(radius=d["hinge_r"], length=d["hinge_len"]),
        origin=Origin(xyz=(right_hinge_x, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_mat,
        name="right_hinge_knuckle",
    )

    bracket_gap = d["pedal_w"] + 0.006
    left_bracket_x = d["pedal_x"] - (bracket_gap / 2.0 + d["pedal_bracket_w"] / 2.0)
    right_bracket_x = d["pedal_x"] + (bracket_gap / 2.0 + d["pedal_bracket_w"] / 2.0)
    bracket_y = -(d["body_d"] / 2.0) - d["pedal_bracket_d"] / 2.0
    bracket_z = d["pedal_pivot_z"]
    shell.visual(
        Box((d["pedal_bracket_w"], d["pedal_bracket_d"], d["pedal_bracket_h"])),
        origin=Origin(xyz=(left_bracket_x, bracket_y, bracket_z)),
        material=body_mat,
        name="left_pedal_bracket",
    )
    shell.visual(
        Box((d["pedal_bracket_w"], d["pedal_bracket_d"], d["pedal_bracket_h"])),
        origin=Origin(xyz=(right_bracket_x, bracket_y, bracket_z)),
        material=body_mat,
        name="right_pedal_bracket",
    )

    lid = model.part("lid")
    lid.visual(
        Box((d["lid_w"], d["lid_d"], d["lid_t"])),
        origin=Origin(xyz=(0.0, -0.086, 0.0)),
        material=body_mat,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=d["hinge_r"], length=d["lid_knuckle_len"]),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_mat,
        name="lid_hinge_tube",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Box((d["pedal_w"], d["pedal_d"], d["pedal_t"])),
        origin=Origin(xyz=(0.0, -d["pedal_d"] / 2.0, -d["pedal_t"] / 2.0)),
        material=pedal_mat,
        name="pedal_tread",
    )
    pedal.visual(
        Cylinder(radius=d["pedal_axle_r"], length=d["pedal_axle_len"]),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_mat,
        name="pedal_axle",
    )
    pedal.visual(
        Box((0.008, 0.018, 0.012)),
        origin=Origin(xyz=(-(d["pedal_w"] / 2.0) + 0.004, -0.009, -0.006)),
        material=pedal_mat,
        name="pedal_left_web",
    )
    pedal.visual(
        Box((0.008, 0.018, 0.012)),
        origin=Origin(xyz=((d["pedal_w"] / 2.0) - 0.004, -0.009, -0.006)),
        material=pedal_mat,
        name="pedal_right_web",
    )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=d["lid_open_angle"],
        ),
    )
    model.articulation(
        "shell_to_pedal",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(d["pedal_x"], d["pedal_pivot_y"], d["pedal_pivot_z"])),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=4.0,
            lower=0.0,
            upper=d["pedal_press_angle"],
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

    d = _dims()
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    pedal_hinge = object_model.get_articulation("shell_to_pedal")

    ctx.expect_overlap(
        lid,
        shell,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.15,
        name="lid panel covers the shell opening",
    )
    ctx.expect_gap(
        lid,
        shell,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="left_wall",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid sits on the shell rim",
    )
    ctx.expect_gap(
        shell,
        pedal,
        axis="y",
        positive_elem="front_wall",
        negative_elem="pedal_tread",
        min_gap=0.002,
        max_gap=0.008,
        name="pedal sits just forward of the lower front wall",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: d["lid_open_angle"]}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.09,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    rest_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_tread")
    with ctx.pose({pedal_hinge: d["pedal_press_angle"]}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem="pedal_tread")
    ctx.check(
        "pedal rotates downward when pressed",
        rest_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < rest_pedal_aabb[0][2] - 0.012,
        details=f"rest={rest_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    pedal_pos = ctx.part_world_position(pedal)
    ctx.check(
        "pedal is offset toward one front corner",
        pedal_pos is not None and pedal_pos[0] > 0.02,
        details=f"pedal_pos={pedal_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
